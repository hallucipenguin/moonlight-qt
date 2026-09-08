#include "sdl.h"

#include <Limelight.h>

namespace AudioStats {
    std::atomic<int> pendingPeakMs(0);
    std::atomic<int> hardDrops(0);
    std::atomic<int> drainDrops(0);

    void reset()
    {
        pendingPeakMs.store(0, std::memory_order_relaxed);
        hardDrops.store(0, std::memory_order_relaxed);
        drainDrops.store(0, std::memory_order_relaxed);
    }
}

namespace {
    // The backlog must sit above the drain threshold this long before we touch
    // it. This is what distinguishes a burst being absorbed (leave it alone,
    // that is the whole point of the drop threshold) from a backlog we are
    // still carrying long after the burst that created it.
    constexpr Uint32 kDrainDwellMs = 2000;

    // At most one frame is shed per interval, so latency walks down gently
    // rather than jumping. At 5 ms frames this drains ~10 ms per second.
    constexpr Uint32 kDrainIntervalMs = 500;

    // If no quiet frame turns up within this long, shed a loud one anyway. A
    // 5 ms splice is well masked by audio loud enough to have kept us waiting.
    constexpr Uint32 kDrainForceMs = 2000;

    // Peak sample magnitude (of full scale) below which removing a frame is
    // inaudible: both sides of the splice are near zero, so there is no step.
    constexpr float kQuietLevel = 0.01f;
}

SdlAudioRenderer::SdlAudioRenderer(int audioPlaybackThresholdMs, int audioDropThresholdMs,
                                   int audioDrainThresholdMs)
    : m_AudioDevice(0),
      m_AudioBuffer(nullptr),
      m_BytesPerMs(0),
      m_AudioPlaybackThresholdMs(SDL_max(0, audioPlaybackThresholdMs)),
      m_AudioDropThresholdMs(SDL_max(1, audioDropThresholdMs)),
      m_AudioDrainThresholdMs(SDL_max(0, audioDrainThresholdMs)),
      m_BacklogHighSinceMs(0),
      m_LastDrainMs(0),
      m_WaitingForPlaybackThreshold(audioPlaybackThresholdMs > 0)
{
    AudioStats::reset();

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

    m_AudioBuffer = SDL_malloc(m_FrameSize);
    if (m_AudioBuffer == nullptr) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "Failed to allocate audio buffer");
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

    m_LastDrainMs = SDL_GetTicks();

    return true;
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

    SDL_QuitSubSystem(SDL_INIT_AUDIO);
    SDL_assert(!SDL_WasInit(SDL_INIT_AUDIO));
}

void* SdlAudioRenderer::getAudioBuffer(int*)
{
    return m_AudioBuffer;
}

bool SdlAudioRenderer::submitAudio(int bytesWritten)
{
    if (bytesWritten == 0) {
        // Nothing to do
        return true;
    }

    int pendingMs = LiGetPendingAudioDuration();

    // Track the high-water mark for the stats overlay. Sampling this once per
    // second from the render thread would miss the bursts entirely.
    int oldPeak = AudioStats::pendingPeakMs.load(std::memory_order_relaxed);
    while (pendingMs > oldPeak &&
           !AudioStats::pendingPeakMs.compare_exchange_weak(oldPeak, pendingMs,
                                                            std::memory_order_relaxed)) {
        // compare_exchange_weak refreshes oldPeak on failure
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
    // created: only discarding drains it. That makes the queue depth ratchet
    // upward with each burst and stay there, which is pure latency with no
    // benefit once the burst that caused it is long gone. When the backlog has
    // been above the drain threshold for a while, shed a single frame at a
    // controlled rate so latency drifts back down without the burst tolerance
    // of the drop threshold being given up.
    //
    // Dropping here (before the backpressure loop below) is what actually
    // drains the queue: the decoder thread does not wait on this frame, so it
    // picks up the next packet immediately.
    if (m_AudioDrainThresholdMs > 0 && pendingMs > m_AudioDrainThresholdMs) {
        Uint32 now = SDL_GetTicks();

        if (m_BacklogHighSinceMs == 0) {
            // Start the dwell timer. Bias off zero, which is the "not high" sentinel.
            m_BacklogHighSinceMs = (now == 0) ? 1 : now;
        }
        else if (now - m_BacklogHighSinceMs >= kDrainDwellMs &&
                 now - m_LastDrainMs >= kDrainIntervalMs &&
                 (isQuietFrame(bytesWritten) || now - m_LastDrainMs >= kDrainForceMs)) {
            m_LastDrainMs = now;
            AudioStats::drainDrops.fetch_add(1, std::memory_order_relaxed);
            return true;
        }
    }
    else {
        // Backlog is comfortable again, so a later rise starts a fresh dwell.
        m_BacklogHighSinceMs = 0;
    }

    // Provide backpressure on the queue to ensure too many frames don't build up
    // in SDL's audio queue, but don't wait forever to avoid a deadlock if the
    // audio device fails.
    //
    // The queue must be allowed to reach the playback threshold, otherwise we
    // could never buffer enough audio to start or resume playback.
    Uint32 maxQueuedAudioMs = (Uint32)SDL_max(50, m_AudioPlaybackThresholdMs);
    for (int i = 0; i < 100; i++) {
        // Our device may enter a permanent error status upon removal, so we need
        // to recreate the audio device to pick up the new default audio device.
        if (SDL_GetAudioDeviceStatus(m_AudioDevice) == SDL_AUDIO_STOPPED) {
            return false;
        }

        // Only queue more samples when SDL's queue is at or below the maximum
        if (SDL_GetQueuedAudioSize(m_AudioDevice) / m_FrameSize * m_FrameDurationMs <= maxQueuedAudioMs) {
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

    if (SDL_QueueAudio(m_AudioDevice, m_AudioBuffer, bytesWritten) < 0) {
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

// True if this frame is quiet enough that removing it leaves no audible step.
// getAudioBufferFormat() is Float32NE, so the buffer is normalized floats.
bool SdlAudioRenderer::isQuietFrame(int bytesWritten)
{
    const float* samples = (const float*)m_AudioBuffer;
    int count = bytesWritten / (int)sizeof(float);

    for (int i = 0; i < count; i++) {
        if (SDL_fabsf(samples[i]) > kQuietLevel) {
            return false;
        }
    }

    return true;
}

IAudioRenderer::AudioFormat SdlAudioRenderer::getAudioBufferFormat()
{
    return AudioFormat::Float32NE;
}
