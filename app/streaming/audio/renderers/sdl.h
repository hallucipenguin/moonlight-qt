#pragma once

#include "renderer.h"
#include "SDL_compat.h"

class SdlAudioRenderer : public IAudioRenderer
{
public:
    explicit SdlAudioRenderer(int audioPlaybackThresholdMs = 0, int audioDropThresholdMs = 30,
                              int audioDrainThresholdMs = 0);

    virtual ~SdlAudioRenderer();

    virtual bool prepareForPlayback(const OPUS_MULTISTREAM_CONFIGURATION* opusConfig);

    virtual void* getAudioBuffer(int* size);

    virtual bool submitAudio(int bytesWritten);

    virtual AudioFormat getAudioBufferFormat();

private:
    void configureDrain(const SDL_AudioSpec& have);
    bool isQuietFrame(int bytesWritten);

    SDL_AudioDeviceID m_AudioDevice;
    void* m_AudioBuffer;
    Uint32 m_FrameSize;
    Uint32 m_FrameDurationMs;
    int m_BytesPerMs;
    int m_AudioPlaybackThresholdMs;
    int m_AudioDropThresholdMs;
    int m_AudioDrainThresholdMs;   // as requested in settings, 0 = off
    int m_DrainTargetMs;           // effective target after floors, 0 = off
    int m_DrainArmMarginMs;        // arm only this far above the target
    Uint32 m_DeviceChunkMs;        // audio the device pulls per callback
    Uint32 m_MaxQueuedAudioMs;     // backpressure cap on SDL's queue
    Uint32 m_BacklogHighSinceMs;
    Uint32 m_LastDrainMs;
    Uint32 m_DrainSearchSinceMs;
    bool m_WaitingForPlaybackThreshold;
};
