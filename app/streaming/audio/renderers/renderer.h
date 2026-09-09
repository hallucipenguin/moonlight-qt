#pragma once

#include <Limelight.h>
#include <QtGlobal>

#include <atomic>
#include <cstdint>

class IAudioRenderer
{
public:
    virtual ~IAudioRenderer() {}

    virtual bool prepareForPlayback(const OPUS_MULTISTREAM_CONFIGURATION* opusConfig) = 0;

    virtual void* getAudioBuffer(int* size) = 0;

    // Return false if an unrecoverable error has occurred and the renderer must be reinitialized
    virtual bool submitAudio(int bytesWritten) = 0;

    virtual void remapChannels(POPUS_MULTISTREAM_CONFIGURATION) {
        // Use default channel mapping:
        // 0 - Front Left
        // 1 - Front Right
        // 2 - Center
        // 3 - LFE
        // 4 - Surround Left
        // 5 - Surround Right
    }

    enum class AudioFormat {
        Sint16NE,  // 16-bit signed integer (native endian)
        Float32NE, // 32-bit floating point (native endian)
    };
    virtual AudioFormat getAudioBufferFormat() = 0;

    int getAudioBufferSampleSize() {
        switch (getAudioBufferFormat()) {
        case IAudioRenderer::AudioFormat::Sint16NE:
            return sizeof(short);
        case IAudioRenderer::AudioFormat::Float32NE:
            return sizeof(float);
        default:
            Q_UNREACHABLE();
        }
    }
};

// Diagnostic counters for the audio backlog, surfaced by the on-screen stats
// overlay (Ctrl+Alt+Shift+S). Written from the audio decode thread and read
// from the video depacketizer and main threads, so they are atomic; relaxed
// ordering is fine because nothing is synchronized through them and each is
// read independently.
//
// Audio waits in two places before it is heard: moonlight-common-c's packet
// queue (LiGetPendingAudioDuration) and SDL's own queue. The decoder thread
// only lets packets pile up in the first once the second is at its cap, so a
// backlog smaller than that cap lives entirely in SDL's queue and is invisible
// to the common-c number alone. Both are sampled together on every submit,
// and the sum is the number that matters.
namespace AudioStats {
    extern std::atomic<int> pendingMs;      // common-c queue at the last sample
    extern std::atomic<int> sdlQueuedMs;    // SDL's queue at the last sample
    extern std::atomic<int> pendingPeakMs;  // high-water mark of pendingMs
    extern std::atomic<int> totalPeakMs;    // high-water mark of pendingMs + sdlQueuedMs
    extern std::atomic<int> hardDrops;      // frames discarded at the drop threshold
    extern std::atomic<int> drainedMs;      // audio removed by the drain so far
    extern std::atomic<int> drainActive;    // 1 while the drain is shrinking packets

    // SDL_GetTicks() when the sample above was taken, 0 if never. Lets the
    // overlay say so when audio has stopped flowing (mute, device loss, host
    // silence) instead of presenting a frozen number as if it were live.
    extern std::atomic<uint32_t> lastSampleTicks;

    // The thresholds actually in force, so the overlay can show whether the
    // configured settings reached this stream. They only take effect when a
    // stream starts, which is easy to forget. drainThresholdMs is the
    // effective target after any floor was applied, 0 when off.
    extern std::atomic<int> dropThresholdMs;
    extern std::atomic<int> drainThresholdMs;

    void reset();
}
