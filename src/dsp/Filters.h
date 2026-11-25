// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/Filters.h
//  Layer:  Signal processing (internal header)
//
//  What this file does
//  -------------------
//  The filters and oscillators the receiver chain is built from:
//    Mixer             frequency shifter (NCO): moves the tuned signal to 0 Hz
//    FirDecimator      FIR low-pass (real or complex taps) on two planar
//                      channels, keeping one output in D (polyphase-style:
//                      only the kept outputs are computed)
//    Resampler         arbitrary-ratio resampler (polyphase windowed sinc with
//                      interpolation between phases), exact on average
//    Agc               peak-following automatic gain control with hang
//    BiquadCascade     the SSB high-pass (port of OFFgrid-SDR's SsbHighPass)
//    OnePole           one-pole low-pass (FM de-emphasis, DC trackers)
//
//  How it fits in
//  --------------
//  Receiver.cpp strings these together per mode.  The approach (shift,
//  decimate, filter, demodulate, resample) follows jtarrio/signals
//  (Apache-2.0), the library the browser OFFgrid-SDR uses, re-designed here
//  for C++: multi-stage integer decimation first, so the expensive filters
//  run at the lowest possible rate.
//
//  Contributing
//  ------------
//  Hot loops must not allocate: buffers are std::vectors that only grow
//  (the first blocks size them, later calls reuse them).  Complex data is
//  kept "planar" (separate re[] and im[] arrays) because that is what lets
//  the compiler keep the multiply-accumulate loops tight.
// ============================================================================
#ifndef OFFGRID_DSP_FILTERS_H
#define OFFGRID_DSP_FILTERS_H

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace dsp {
namespace detail {

// Grows a vector to at least n elements (never shrinks; no work if big enough).
inline void ensureSize(std::vector<float>& v, std::size_t n) {
    if (v.size() < n) v.resize(n);
}

// ---- Mixer -----------------------------------------------------------------
// Multiplies the input by exp(j*2*pi*f*t), turning interleaved complex input
// into planar output.  The rotating phasor is recomputed exactly (from a
// double phase) every 1024 samples and advanced by complex multiplication in
// between: the recursion is cheap and the re-anchoring stops its rounding
// error from growing.
class Mixer {
public:
    Mixer();
    void setFrequency(double hz, double rate);
    void process(const std::complex<float>* in, std::size_t n, float* outRe, float* outIm);
    void processPlanar(float* re, float* im, std::size_t n);  // in place
    void reset() { phase_ = 0.0; }

private:
    double inc_;    // radians per sample
    double phase_;  // radians, kept in [0, 2*pi)
    bool zero_;
};

// ---- FIR decimator -----------------------------------------------------------
// y[m] = sum_k h[k] * x[m*D - k], computed only for the outputs that are kept.
// Works on two planar channels at once: either the I and Q of a complex signal
// (real taps filter each one; complex taps do a full complex convolution), or
// two unrelated real signals (e.g. L+R and L-R of FM stereo) with real taps.
class FirDecimator {
public:
    FirDecimator();
    // Real taps (symmetric low-pass from designLowPass).
    void setup(const std::vector<float>& taps, int decimation);
    // Complex taps (band-pass made by shifting a low-pass): out = h (*) x.
    void setupComplex(const std::vector<float>& tapsRe, const std::vector<float>& tapsIm, int decimation);
    void reset();

    // Filters n input samples; writes up to maxOutput(n) samples; returns
    // the count.  `im`/`outIm` may be null for a single real channel (real
    // taps only).
    std::size_t process(const float* re, const float* im, std::size_t n, float* outRe, float* outIm);
    std::size_t maxOutput(std::size_t n) const { return n / decim_ + 2; }
    int decimation() const { return decim_; }
    int length() const { return len_; }

private:
    std::vector<float> hr_, hi_;  // taps, stored time-reversed
    bool complexTaps_;
    int len_, decim_;
    std::vector<float> bufRe_, bufIm_;  // [len-1 history][new block]
    std::size_t next_;                  // where the next output's window starts
};

// ---- Resampler ---------------------------------------------------------------
// Converts one or two real channels from inRate to outRate (any ratio, up or
// down).  For each output instant t (in input samples) it evaluates a
// windowed-sinc low-pass centred on t: the filter is tabulated at P = 128
// sub-sample phases and the two nearest phases are blended linearly.  The
// position advances by inRate/outRate in double precision, so the average
// output rate is exact (no drift over hours).
class Resampler {
public:
    Resampler();
    // passHz: flat up to here; stopHz: attenuated from here (both <= the
    // lower of the two Nyquist rates is the caller's job).
    void setup(double inRate, double outRate, double passHz, double stopHz, double attenDb = 60.0);
    void reset();
    std::size_t process(const float* in0, const float* in1, std::size_t n, float* out0, float* out1);
    std::size_t maxOutput(std::size_t n) const;

private:
    static const int kPhases = 128;
    int taps_;                 // per phase (even)
    std::vector<float> table_; // (kPhases + 1) * taps_
    double step_;              // input samples per output sample
    double pos_;               // position of the next output in buf (input samples)
    std::vector<float> buf0_, buf1_;
    std::size_t fill_;         // valid samples in buf
};

// ---- AGC -------------------------------------------------------------------
// Peak envelope follower: rises quickly (attack), holds for `hang` seconds,
// then falls slowly (decay).  Gain = target / envelope, limited to maxGain.
class Agc {
public:
    Agc();
    void setup(double rate, double attackS, double hangS, double decayS, float target, float maxGain);
    void reset();
    void process(float* x, std::size_t n);

private:
    float atk_, dec_, target_, minEnv_, env_;
    int hangN_, hang_;
};

// ---- Biquad cascade (SSB high-pass) ----------------------------------------
// Port of OFFgrid-SDR's SsbHighPass: an 8th-order Butterworth high-pass
// (four biquads, 48 dB/octave) that sets the bottom edge of the SSB pass band.
class BiquadCascade {
public:
    BiquadCascade();
    void configureHighPass(double fc, double fs);  // fc < 20 Hz: bypass
    void reset();
    void process(float* x, std::size_t n);

private:
    struct Section { float b0, b1, b2, a1, a2, z1, z2; };
    std::vector<Section> sec_;
};

// ---- One-pole low-pass -----------------------------------------------------
struct OnePole {
    OnePole() : a(1.0f), y(0.0f) {}
    void setup(double rate, double timeConstant) { a = static_cast<float>(1.0 - std::exp(-1.0 / (rate * timeConstant))); }
    float a, y;
    float step(float x) { y += a * (x - y); return y; }
};

}  // namespace detail
}  // namespace dsp

#endif
