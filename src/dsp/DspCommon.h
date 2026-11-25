// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/DspCommon.h
//  Layer:  Signal processing (internal header, not used by the application)
//
//  What this file does
//  -------------------
//  Small building blocks shared by every part of the DSP library:
//    - kPi / kTwoPi constants,
//    - a sine table addressed by a 32-bit phase accumulator (2^32 = one
//      turn), used by the oscillators that run once per sample,
//    - a fast, accurate atan2 for the FM discriminators,
//    - the Kaiser-window FIR design used by all the low-pass filters.
//
//  How it fits in
//  --------------
//  Included by Filters.cpp, Receiver.cpp, DemoSource.cpp, Rds.cpp.  All in
//  namespace dsp::detail so nothing here leaks into the public API.
//
//  Contributing
//  ------------
//  Everything here is plain float/double C++11.  If you change the table
//  size, keep it a power of two: the lookup is a shift, not a division.
// ============================================================================
#ifndef OFFGRID_DSP_DSPCOMMON_H
#define OFFGRID_DSP_DSPCOMMON_H

#include <cmath>
#include <cstdint>
#include <vector>

namespace dsp {
namespace detail {

const double kPi = 3.14159265358979323846;
const double kTwoPi = 2.0 * kPi;

// ---- Sine table ------------------------------------------------------------
// A phase accumulator is a uint32_t where 2^32 means one full turn: adding a
// fixed increment every sample makes an oscillator whose frequency is
// increment * sampleRate / 2^32, and the wrap-around at 2^32 is free (unsigned
// overflow is well defined).  The top kSinBits bits index the table.  With
// 13 bits the phase error is at most 2*pi/8192, so spurs stay ~78 dB down.
const int kSinBits = 13;
const int kSinSize = 1 << kSinBits;

// Returns the table (kSinSize + kSinSize/4 entries, so cos can index past the
// end without wrapping).  Built once, thread-safe (C++11 magic statics).
const float* sinTable();

inline float tableSin(const float* t, std::uint32_t phase) {
    return t[phase >> (32 - kSinBits)];
}
inline float tableCos(const float* t, std::uint32_t phase) {
    return t[(phase >> (32 - kSinBits)) + kSinSize / 4];
}
// Hz -> phase increment for a given sample rate.
inline std::uint32_t phaseIncrement(double hz, double rate) {
    double turns = hz / rate;
    turns -= std::floor(turns);  // negative frequencies wrap to 2^32 - x
    return static_cast<std::uint32_t>(static_cast<std::uint64_t>(turns * 4294967296.0) & 0xFFFFFFFFu);
}

// ---- atan2 ------------------------------------------------------------------
// Polynomial arctangent (max error about 1e-6 rad), several times faster than
// std::atan2.  The FM discriminator calls it once per sample at ~340 kHz.
inline float fastAtan2(float y, float x) {
    const float ax = std::fabs(x), ay = std::fabs(y);
    if (ax == 0.0f && ay == 0.0f) return 0.0f;
    const bool swap = ay > ax;
    const float z = swap ? ax / ay : ay / ax;  // 0..1
    const float z2 = z * z;
    float r = z * (0.99997726f + z2 * (-0.33262347f + z2 * (0.19354346f + z2 * (-0.11643287f + z2 * (0.05265332f + z2 * -0.01172120f)))));
    if (swap) r = 1.57079632679f - r;
    if (x < 0.0f) r = 3.14159265359f - r;
    return y < 0.0f ? -r : r;
}

// ---- FIR design ----------------------------------------------------------
// Kaiser window: given the stop-band attenuation A (dB) and the transition
// width (as a fraction of the sample rate) the Kaiser formulas give the tap
// count and the window shape parameter beta directly:
//    N ~= (A - 8) / (2.285 * 2*pi*transition)       beta from A (see .cpp)
double besselI0(double x);
double kaiserBeta(double attenDb);
int kaiserLength(double transitionHz, double rate, double attenDb);

// Low-pass windowed sinc.  The pass band is flat to passHz, the stop band
// starts at stopHz (attenuated by attenDb); the -6 dB point is in the middle.
// Odd length (symmetric, linear phase), DC gain = `gain`.
std::vector<float> designLowPass(double rate, double passHz, double stopHz, double attenDb,
                                 int maxTaps = 2047, double gain = 1.0);

inline float dbToLin(double db) { return static_cast<float>(std::pow(10.0, db / 20.0)); }
inline double powerToDb(double p) { return 10.0 * std::log10(p > 1e-30 ? p : 1e-30); }

// One-pole smoothing coefficient for a time constant (seconds) at a rate:
// y += a * (x - y) reaches 63% of a step after timeConstant.
inline double decayCoef(double rate, double timeConstant) {
    return 1.0 - std::exp(-1.0 / (rate * timeConstant));
}

}  // namespace detail
}  // namespace dsp

#endif
