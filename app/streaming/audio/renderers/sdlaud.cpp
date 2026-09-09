#include "sdl.h"
#include "timecompress.h"

#include <Limelight.h>

namespace AudioStats {
    std::atomic<int> pendingMs(0);
    std::atomic<int> sdlQueuedMs(0);
    std::atomic<int> pendingPeakMs(0);
    std::atomic<int> totalPeakMs(0);
    std::atomic<int> hardDrops(0);
    std::atomic<int> drainedMs(0);
    std::atomic<int> drainActive(0);
    std::atomic<uint32_t> lastSampleTicks(0);
    std::atomic<int> dropThresholdMs(0);
    std::atomic<int> drainThresholdMs(0);

    void reset()
    {
        pendingMs.store(0, std::memory_order_relaxed);
        sdlQueuedMs.store(0, std::memory_order_relaxed);
        pendingPeakMs.store(0, std::memory_order_relaxed);
        totalPeakMs.store(0, std::memory_order_relaxed);
        hardDrops.store(0, std::memory_order_relaxed);
        drainedMs.store(0, std::memory_order_relaxed);
        drainActive.store(0, std::memory_order_relaxed);
        lastSampleTicks.store(0, std::memory_order_relaxed);
    }
}

namespace {
    // The backlog must sit above the drain target this long before we touch
    // it. This is what distinguishes a burst being absorbed (leave it alone,
    // that is the whole point of the drop threshold) from a backlog we are
    // still carrying long after the burst that created it.
    constexpr Uint32 kDrainDwellMs = 2000;

    // While the drain is active every packet is shrunk by this fraction of
    // its length (1/120 = 0.83%, about 14 cents of pitch), which removes about
    // 8 ms of backlog per second. A 90 ms backlog is gone in roughly 10 s.
    // This replaced cutting whole packets, which left a step in the waveform
    // that was audible as a pop in anything but silence.
    constexpr int kDrainShrinkDivisor = 120;

    // The backlog sampled in submitAudio() wobbles by up to one device chunk
    // (the device pulls that much at a time) plus a packet or so of thread
    // scheduling and whole-packet rounding. Arm the drain only once the
    // backlog is a chunk plus this many packets above the target, and disarm
    // at the target. Without the deadband that wobble would trigger a
    // pointless drop every few seconds forever. The same margin is the floor
    // for the target itself: any lower and a single device pull could empty
    // SDL's queue and leave a gap.
    constexpr int kDrainArmPackets = 2;
}

SdlAudioRenderer::SdlAudioRenderer(int audioPlaybackThresholdMs, int audioDropThresholdMs,
                                   int audioDrainThresholdMs)
    : m_AudioDevice(0),
      m_AudioBuffer(nullptr),
      m_DrainScratch(nullptr),
      m_BytesPerMs(0),
      m_AudioPlaybackThresholdMs(SDL_max(0, audioPlaybackThresholdMs)),
      m_AudioDropThresholdMs(SDL_max(1, audioDropThresholdMs)),
      m_AudioDrainThresholdMs(SDL_max(0, audioDrainThresholdMs)),
      m_DrainTargetMs(0),
      m_DrainArmMarginMs(0),
      m_DeviceChunkMs(0),
      m_MaxQueuedAudioMs(0),
      m_BacklogHighSinceMs(0),
      m_Channels(0),
      m_SampleRate(0),
      m_DrainedSamples(0),
      m_WaitingForPlaybackThreshold(audioPlaybackThresholdMs > 0)
{
    AudioStats::dropThresholdMs.store(m_AudioDropThresholdMs, std::memory_order_relaxed);

    // The effective drain target is settled in configureDrain() once the
    // packet and device chunk sizes are known. Until then report it as off.
    AudioStats::drainThresholdMs.store(0, std::memory_order_relaxed);

    SDL_assert(!SDL_WasInit(SDL_INIT_AUDIO));

    if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "SDL_InitSubSystem(SDL_INIT_AUDIO) failed: %s",
                     SDL_GetError());
        SDL_assert(SDL_WasInit(SDL_INIT_AUDIO));
    }
}

bool SdlAudioRenderer::prepareForPlayback(const OPUS_MULTISTREAM_CONFIGURATION* opusConfig)
{
    SDL_AudioSpec want, have;

    SDL_zero(want);
    want.freq = opusConfig->sampleRate;
    want.format = AUDIO_F32SYS;
    want.channels = opusConfig->channelCount;

    // On PulseAudio systems, setting a value too small can cause underruns for other
    // applications sharing this output device. We impose a floor of 480 samples (10 ms)
    // to mitigate this issue. Otherwise, we will buffer up to 3 frames of audio which
    // is 15 ms at regular 5 ms frames and 30 ms at 10 ms frames for slow connections.
    // The buffering helps avoid audio underruns due to network jitter.
    want.samples = SDL_max(480, opusConfig->samplesPerFrame * 3);

    m_FrameDurationMs = opusConfig->samplesPerFrame / (opusConfig->sampleRate / 1000);
    m_FrameSize = opusConfig->samplesPerFrame *
                  opusConfig->channelCount *
                  getAudioBufferSampleSize();

    m_AudioDevice = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (m_AudioDevice == 0) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "Failed to open audio device: %s",
                     SDL_GetError());
        return false;
    }

    m_BytesPerMs = SDL_max(1, (have.freq * have.channels * getAudioBufferSampleSize()) / 1000);

    configureDrain(have);

    m_AudioBuffer = SDL_malloc(m_FrameSize);
    if (m_AudioBuffer == nullptr) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "Failed to allocate audio buffer");
        return false;
    }

    m_DrainScratch = SDL_malloc(m_FrameSize);
    if (m_DrainScratch == nullptr) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "Failed to allocate audio drain buffer");
        return false;
    }

    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                "Desired audio buffer: %u samples (%u bytes)",
                want.samples,
                want.samples * want.channels * getAudioBufferSampleSize());

    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                "Obtained audio buffer: %u samples (%u bytes)",
                have.samples,
                have.size);

    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                "SDL audio driver: %s",
                SDL_GetCurrentAudioDriver());

    if (!m_WaitingForPlaybackThreshold) {
        SDL_PauseAudioDevice(m_AudioDevice, 0);
    }

    return true;
}

// Settle the drain parameters now that the packet size and the device's pull
// size are known. Everything here is derived from the obtained SDL_AudioSpec,
// so it is right for whatever device actually opened rather than assumed.
void SdlAudioRenderer::configureDrain(const SDL_AudioSpec& have)
{
    m_Channels = have.channels;
    m_SampleRate = have.freq;

    // The device pulls this much per callback, so SDL's queue (and the total
    // sampled in submitAudio()) naturally wobbles by this amount.
    m_DeviceChunkMs = (Uint32)SDL_max(1, (int)have.samples / SDL_max(1, have.freq / 1000));

    // The backpressure loop in submitAudio() caps SDL's queue here. The
    // common-c queue only starts filling once SDL is at this cap.
    m_MaxQueuedAudioMs = (Uint32)SDL_max(50, m_AudioPlaybackThresholdMs);

    m_DrainArmMarginMs = (int)m_DeviceChunkMs + kDrainArmPackets * (int)m_FrameDurationMs;
    m_DrainTargetMs = 0;

    if (m_AudioDrainThresholdMs > 0) {
        int target = m_AudioDrainThresholdMs;

        // Floor: keep at least one device pull plus a couple of packets in
        // hand at the target, or every callback risks an underrun there. With
        // a playback threshold in use, stay above it as well, or a hiccup
        // becomes a pause-and-refill instead of a short glitch.
        int floorMs = m_DrainArmMarginMs;
        if (m_AudioPlaybackThresholdMs > 0) {
            floorMs = SDL_max(floorMs, m_AudioPlaybackThresholdMs + (int)m_DeviceChunkMs);
        }
        if (target < floorMs) {
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                        "Audio drain target %d ms is too low for this device. Raising it to %d ms.",
                        target,
                        floorMs);
            target = floorMs;
        }

        // Ceiling: the total can never exceed SDL's cap plus the drop
        // threshold, because anything beyond that is hard-dropped before the
        // drain runs. A target that cannot be armed within that range is not
        // a gentler setting, it is an inert one, so refuse it loudly rather
        // than leave the user wondering why nothing happens.
        int ceilingMs = (int)m_MaxQueuedAudioMs + m_AudioDropThresholdMs;
        if (target + m_DrainArmMarginMs >= ceilingMs) {
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                        "Audio drain target %d ms can never trigger: the backlog is capped at "
                        "%d ms and the drain arms %d ms above its target. Disabling drain.",
                        target,
                        ceilingMs,
                        m_DrainArmMarginMs);
            target = 0;
        }

        m_DrainTargetMs = target;
    }

    if (m_DrainTargetMs > 0) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "Audio drain: target %d ms total, arms above %d ms "
                    "(device chunk %u ms, packet %u ms, SDL cap %u ms)",
                    m_DrainTargetMs,
                    m_DrainTargetMs + m_DrainArmMarginMs,
                    m_DeviceChunkMs,
                    m_FrameDurationMs,
                    m_MaxQueuedAudioMs);
    }

    AudioStats::drainThresholdMs.store(m_DrainTargetMs, std::memory_order_relaxed);
}

SdlAudioRenderer::~SdlAudioRenderer()
{
    if (m_AudioDevice != 0) {
        // Stop playback
        SDL_PauseAudioDevice(m_AudioDevice, 1);
        SDL_CloseAudioDevice(m_AudioDevice);
    }

    if (m_AudioBuffer != nullptr) {
        SDL_free(m_AudioBuffer);
    }

    if (m_DrainScratch != nullptr) {
        SDL_free(m_DrainScratch);
    }

    SDL_QuitSubSystem(SDL_INIT_AUDIO);
    SDL_assert(!SDL_WasInit(SDL_INIT_AUDIO));
}

void* SdlAudioRenderer::getAudioBuffer(int*)
{
    return m_AudioBuffer;
}

bool SdlAudioRenderer::submitAudio(int bytesWritten)
{
    // Sample both queues together, before any early return below, so the
    // overlay sees the backlog even while frames are being dropped. The
    // common-c queue only fills once SDL's queue is at its cap, so the sum is
    // the real backlog; either one alone hides the other's share.
    int pendingMs = LiGetPendingAudioDuration();
    int sdlMs = (int)(SDL_GetQueuedAudioSize(m_AudioDevice) / m_FrameSize * m_FrameDurationMs);
    int totalMs = pendingMs + sdlMs;
    Uint32 now = SDL_GetTicks();

    AudioStats::pendingMs.store(pendingMs, std::memory_order_relaxed);
    AudioStats::sdlQueuedMs.store(sdlMs, std::memory_order_relaxed);
    AudioStats::lastSampleTicks.store((now == 0) ? 1 : now, std::memory_order_relaxed);

    // Track the high-water marks here rather than sampling once per second
    // from the render thread, which would miss the bursts entirely. This
    // thread is the only writer, so a plain compare and store is enough.
    if (pendingMs > AudioStats::pendingPeakMs.load(std::memory_order_relaxed)) {
        AudioStats::pendingPeakMs.store(pendingMs, std::memory_order_relaxed);
    }
    if (totalMs > AudioStats::totalPeakMs.load(std::memory_order_relaxed)) {
        AudioStats::totalPeakMs.store(totalMs, std::memory_order_relaxed);
    }

    if (bytesWritten == 0) {
        // Nothing to do
        return true;
    }

    // Don't queue if there's already more than the configured amount of audio
    // in Moonlight's audio queue.
    if (pendingMs > m_AudioDropThresholdMs) {
        AudioStats::hardDrops.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    // Walk a persistent backlog back down.
    //
    // Audio arrives and plays at real time, so a backlog is conserved once
    // created: only removing audio drains it. That makes the total backlog
    // ratchet upward with each burst and stay there, which is pure latency
    // with no benefit once the burst that caused it is long gone. When the
    // total has been above the drain target for a while, play every packet
    // very slightly faster (see kDrainShrinkDivisor) so latency drifts back
    // down without the burst tolerance of the drop threshold being given up,
    // and without the pop that cutting whole packets produced.
    //
    // The total is what matters, not the common-c queue alone: a backlog
    // smaller than SDL's cap lives entirely in SDL's queue, and queueing a
    // little less audio than arrived shrinks that just the same, because the
    // device keeps playing at its own pace.
    bool drainNow = false;
    if (m_DrainTargetMs > 0) {
        // Arm above the target by a margin but only disarm at the target
        // itself. The deadband matters: the sampled total wobbles by up to a
        // device chunk plus a packet just from timing, so arming and
        // disarming on the same value would shed a frame every few seconds
        // indefinitely without ever lowering latency.
        if (totalMs > m_DrainTargetMs + m_DrainArmMarginMs) {
            if (m_BacklogHighSinceMs == 0) {
                // Start the dwell timer. Bias off zero, the "not high" sentinel.
                m_BacklogHighSinceMs = (now == 0) ? 1 : now;
            }
        }
        else if (totalMs <= m_DrainTargetMs) {
            // Back at the target, so a later rise starts a fresh dwell.
            m_BacklogHighSinceMs = 0;
        }

        // The dwell distinguishes a burst still being absorbed (leave it
        // alone) from a backlog we are carrying long after the burst.
        drainNow = m_BacklogHighSinceMs != 0 &&
                   now - m_BacklogHighSinceMs >= kDrainDwellMs;
    }
    AudioStats::drainActive.store(drainNow ? 1 : 0, std::memory_order_relaxed);

    // Provide backpressure on the queue to ensure too many frames don't build up
    // in SDL's audio queue, but don't wait forever to avoid a deadlock if the
    // audio device fails.
    //
    // The queue must be allowed to reach the playback threshold, otherwise we
    // could never buffer enough audio to start or resume playback.
    for (int i = 0; i < 100; i++) {
        // Our device may enter a permanent error status upon removal, so we need
        // to recreate the audio device to pick up the new default audio device.
        if (SDL_GetAudioDeviceStatus(m_AudioDevice) == SDL_AUDIO_STOPPED) {
            return false;
        }

        // Only queue more samples when SDL's queue is at or below the maximum
        if (SDL_GetQueuedAudioSize(m_AudioDevice) / m_FrameSize * m_FrameDurationMs <= m_MaxQueuedAudioMs) {
            break;
        }

        SDL_Delay(1);
    }

    // If SDL's queue has run dry, pause playback until we've buffered enough
    // audio again. This trades a brief silence for fewer underrun glitches.
    if (m_AudioPlaybackThresholdMs > 0 && !m_WaitingForPlaybackThreshold &&
            SDL_GetQueuedAudioSize(m_AudioDevice) == 0) {
        SDL_PauseAudioDevice(m_AudioDevice, 1);
        m_WaitingForPlaybackThreshold = true;
    }

    // While draining, queue a slightly time-compressed copy of the frame
    // instead of the frame itself. The device plays it in less time than the
    // original covered, so the backlog shrinks by the difference.
    const void* queueData = m_AudioBuffer;
    int queueBytes = bytesWritten;
    if (drainNow && m_Channels > 0 && m_SampleRate > 0) {
        int inFrames = bytesWritten / (m_Channels * (int)sizeof(float));
        int shrinkBy = inFrames / kDrainShrinkDivisor;
        int outFrames = inFrames - shrinkBy;
        if (shrinkBy > 0 && outFrames >= 4) {
            TimeCompress::shrink((const float*)m_AudioBuffer, inFrames,
                                 (float*)m_DrainScratch, outFrames, m_Channels);
            queueData = m_DrainScratch;
            queueBytes = outFrames * m_Channels * (int)sizeof(float);

            m_DrainedSamples += (Uint64)shrinkBy;
            AudioStats::drainedMs.store((int)(m_DrainedSamples * 1000 / (Uint64)m_SampleRate),
                                        std::memory_order_relaxed);
        }
    }

    if (SDL_QueueAudio(m_AudioDevice, queueData, queueBytes) < 0) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "Failed to queue audio sample: %s",
                     SDL_GetError());
        return true;
    }

    if (m_WaitingForPlaybackThreshold &&
            SDL_GetQueuedAudioSize(m_AudioDevice) >= (Uint32)(m_AudioPlaybackThresholdMs * m_BytesPerMs)) {
        SDL_PauseAudioDevice(m_AudioDevice, 0);
        m_WaitingForPlaybackThreshold = false;
    }

    return true;
}

IAudioRenderer::AudioFormat SdlAudioRenderer::getAudioBufferFormat()
{
    return AudioFormat::Float32NE;
}
