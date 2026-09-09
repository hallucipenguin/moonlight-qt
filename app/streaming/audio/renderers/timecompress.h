#pragma once

// Time-compress a block of interleaved float audio by resampling it to fewer
// sample frames. The SDL audio renderer's drain uses this to remove a little
// audio from every packet with no splice: the packet simply plays slightly
// faster, which raises pitch by the same small fraction (1/120 = 0.83%, about
// 14 cents, at the default shrink) instead of leaving a step in the waveform.
// A step is what a pop is; a 14-cent rise for a few seconds is at the edge of
// hearing on sustained music and imperceptible on game sound.
//
// Output sample j is taken at input position j * inFrames / outFrames, so the
// spacing is uniform within the packet and, because the last output lands one
// output step short of inFrames, across the join into the next packet as well.
// Values are interpolated with a 4-point cubic Hermite, which keeps the top
// octave far flatter than linear interpolation would.
namespace TimeCompress {

inline float hermite(float y0, float y1, float y2, float y3, float t)
{
    float c1 = 0.5f * (y2 - y0);
    float c2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
    float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
    return ((c3 * t + c2) * t + c1) * t + y1;
}

// in: inFrames * channels interleaved floats. out: outFrames * channels.
// Requires 0 < outFrames <= inFrames. in and out must not overlap.
inline void shrink(const float* in, int inFrames, float* out, int outFrames, int channels)
{
    for (int j = 0; j < outFrames; j++) {
        double pos = (double)j * inFrames / outFrames;
        int i = (int)pos;
        if (i > inFrames - 1) {
            i = inFrames - 1;
        }
        float t = (float)(pos - i);

        // Clamp the neighbours at the ends of the block. At j == 0, t is
        // exactly 0, so the first output equals the first input regardless of
        // y0, which keeps the join with the previous packet exact.
        int i0 = (i > 0) ? i - 1 : 0;
        int i2 = (i + 1 < inFrames) ? i + 1 : inFrames - 1;
        int i3 = (i + 2 < inFrames) ? i + 2 : inFrames - 1;

        for (int c = 0; c < channels; c++) {
            out[j * channels + c] = hermite(in[i0 * channels + c],
                                            in[i * channels + c],
                                            in[i2 * channels + c],
                                            in[i3 * channels + c],
                                            t);
        }
    }
}

}
