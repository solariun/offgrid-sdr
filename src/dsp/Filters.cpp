// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/Filters.cpp
//  Layer:  Signal processing
//
//  What this file does
//  -------------------
//  Implements DspCommon.h (sine table, Kaiser FIR design) and Filters.h
//  (mixer, FIR decimator, resampler, AGC, SSB high-pass).
//
//  How it fits in
//  --------------
//  Pure building blocks: no knowledge of modes or of the application.  The
//  filter-design approach (windowed sinc low-pass kernels, shifting them for
//  band-pass) follows jtarrio/signals (Apache-2.0); the Kaiser window replaces
//  its fixed Hamming window so the tap count can be derived from the wanted
//  transition width and attenuation instead of being a fixed 151.
//
//  Contributing
//  ------------
//  The inner loops (dot products) are written with several independent
//  accumulators: that breaks the add-latency chain and lets the compiler
//  pair or vectorise the multiply-adds even without -ffast-math.
// ============================================================================
#include "DspCommon.h"
#include "Filters.h"

#include <algorithm>
#include <cstring>

namespace dsp {
namespace detail {

// ---- Sine table --------------------------------------------------------------
namespace {
struct SinTableHolder {
    std::vector<float> t;
    SinTableHolder() : t(kSinSize + kSinSize / 4 + 1) {
        for (std::size_t i = 0; i < t.size(); ++i) t[i] = static_cast<float>(std::sin(kTwoPi * static_cast<double>(i) / kSinSize));
    }
};
}  // namespace

const float* sinTable() {
    static const SinTableHolder holder;  // built on first use (thread-safe in C++11)
    return &holder.t[0];
}

// ---- Kaiser design -----------------------------------------------------------
double besselI0(double x) {
    // Power series: I0(x) = sum ((x/2)^k / k!)^2; converges fast for the
    // beta values (< 12) that filter design uses.
    double sum = 1.0, term = 1.0;
    const double h = x / 2.0;
    for (int k = 1; k < 60; ++k) {
        term *= h / k;
        const double t2 = term * term;
        sum += t2;
        if (t2 < sum * 1e-12) break;
    }
    return sum;
}

double kaiserBeta(double a) {
    if (a > 50.0) return 0.1102 * (a - 8.7);
    if (a > 21.0) return 0.5842 * std::pow(a - 21.0, 0.4) + 0.07886 * (a - 21.0);
    return 0.0;
}

int kaiserLength(double transitionHz, double rate, double attenDb) {
    const double dw = kTwoPi * transitionHz / rate;  // transition, radians/sample
    if (dw <= 0.0) return 4095;
    return static_cast<int>(std::ceil((attenDb - 8.0) / (2.285 * dw))) + 1;
}

std::vector<float> designLowPass(double rate, double passHz, double stopHz, double attenDb, int maxTaps, double gain) {
    int n = kaiserLength(stopHz - passHz, rate, attenDb);
    if (n > maxTaps) n = maxTaps;
    if (n < 5) n = 5;
    if (n % 2 == 0) ++n;  // odd: a centre tap, whole-sample delay
    const double fc = 0.5 * (passHz + stopHz) / rate;  // -6 dB point, cycles/sample
    const double beta = kaiserBeta(attenDb), i0b = besselI0(beta);
    const int m = (n - 1) / 2;
    std::vector<double> h(n);
    double sum = 0.0;
    for (int i = 0; i < n; ++i) {
        const double x = i - m;
        const double sinc = x == 0 ? 2.0 * fc : std::sin(kTwoPi * fc * x) / (kPi * x);
        const double r = x / m;
        const double w = besselI0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / i0b;
        h[i] = sinc * w;
        sum += h[i];
    }
    std::vector<float> out(n);
    for (int i = 0; i < n; ++i) out[i] = static_cast<float>(h[i] * gain / sum);
    return out;
}

// ---- Dot products --------------------------------------------------------------
namespace {
// sum h[k] * x[k], four accumulators.
inline float dot(const float* h, const float* x, int n) {
    float a0 = 0, a1 = 0, a2 = 0, a3 = 0;
    int k = 0;
    for (; k + 3 < n; k += 4) {
        a0 += h[k] * x[k];
        a1 += h[k + 1] * x[k + 1];
        a2 += h[k + 2] * x[k + 2];
        a3 += h[k + 3] * x[k + 3];
    }
    for (; k < n; ++k) a0 += h[k] * x[k];
    return (a0 + a1) + (a2 + a3);
}

// The same real taps against two channels at once (one load of h per tap).
inline void dot2(const float* h, const float* x, const float* y, int n, float& outX, float& outY) {
    float a0 = 0, a1 = 0, a2 = 0, a3 = 0, b0 = 0, b1 = 0, b2 = 0, b3 = 0;
    int k = 0;
    for (; k + 3 < n; k += 4) {
        a0 += h[k] * x[k];
        b0 += h[k] * y[k];
        a1 += h[k + 1] * x[k + 1];
        b1 += h[k + 1] * y[k + 1];
        a2 += h[k + 2] * x[k + 2];
        b2 += h[k + 2] * y[k + 2];
        a3 += h[k + 3] * x[k + 3];
        b3 += h[k + 3] * y[k + 3];
    }
    for (; k < n; ++k) {
        a0 += h[k] * x[k];
        b0 += h[k] * y[k];
    }
    outX = (a0 + a1) + (a2 + a3);
    outY = (b0 + b1) + (b2 + b3);
}
}  // namespace

// ---- Mixer --------------------------------------------------------------------
Mixer::Mixer() : inc_(0.0), phase_(0.0), zero_(true) {}

void Mixer::setFrequency(double hz, double rate) {
    inc_ = kTwoPi * hz / rate;
    zero_ = hz == 0.0;
}

void Mixer::process(const std::complex<float>* in, std::size_t n, float* outRe, float* outIm) {
    if (zero_) {  // nothing to shift: just split into planar arrays
        for (std::size_t i = 0; i < n; ++i) {
            outRe[i] = in[i].real();
            outIm[i] = in[i].imag();
        }
        return;
    }
    const float wr = static_cast<float>(std::cos(inc_)), wi = static_cast<float>(std::sin(inc_));
    std::size_t i = 0;
    while (i < n) {
        const std::size_t m = std::min<std::size_t>(n - i, 1024);
        float pr = static_cast<float>(std::cos(phase_)), pi = static_cast<float>(std::sin(phase_));
        for (std::size_t k = 0; k < m; ++k) {
            const float xr = in[i + k].real(), xi = in[i + k].imag();
            outRe[i + k] = xr * pr - xi * pi;
            outIm[i + k] = xr * pi + xi * pr;
            const float t = pr * wr - pi * wi;  // phasor *= exp(j*inc)
            pi = pr * wi + pi * wr;
            pr = t;
        }
        phase_ = std::fmod(phase_ + inc_ * static_cast<double>(m), kTwoPi);
        if (phase_ < 0) phase_ += kTwoPi;
        i += m;
    }
}

void Mixer::processPlanar(float* re, float* im, std::size_t n) {
    if (zero_) return;
    const float wr = static_cast<float>(std::cos(inc_)), wi = static_cast<float>(std::sin(inc_));
    std::size_t i = 0;
    while (i < n) {
        const std::size_t m = std::min<std::size_t>(n - i, 1024);
        float pr = static_cast<float>(std::cos(phase_)), pi = static_cast<float>(std::sin(phase_));
        for (std::size_t k = 0; k < m; ++k) {
            const float xr = re[i + k], xi = im[i + k];
            re[i + k] = xr * pr - xi * pi;
            im[i + k] = xr * pi + xi * pr;
            const float t = pr * wr - pi * wi;
            pi = pr * wi + pi * wr;
            pr = t;
        }
        phase_ = std::fmod(phase_ + inc_ * static_cast<double>(m), kTwoPi);
        if (phase_ < 0) phase_ += kTwoPi;
        i += m;
    }
}

// ---- FIR decimator ----------------------------------------------------------------
FirDecimator::FirDecimator() : complexTaps_(false), len_(1), decim_(1), next_(0) {
    hr_.assign(1, 1.0f);
}

void FirDecimator::setup(const std::vector<float>& taps, int decimation) {
    len_ = static_cast<int>(taps.size());
    decim_ = decimation < 1 ? 1 : decimation;
    hr_.assign(taps.rbegin(), taps.rend());  // reversed: buffer order is oldest first
    hi_.clear();
    complexTaps_ = false;
    reset();
}

void FirDecimator::setupComplex(const std::vector<float>& tapsRe, const std::vector<float>& tapsIm, int decimation) {
    len_ = static_cast<int>(tapsRe.size());
    decim_ = decimation < 1 ? 1 : decimation;
    hr_.assign(tapsRe.rbegin(), tapsRe.rend());
    hi_.assign(tapsIm.rbegin(), tapsIm.rend());
    complexTaps_ = true;
    reset();
}

void FirDecimator::reset() {
    bufRe_.assign(len_ - 1 + 4096, 0.0f);
    bufIm_.assign(len_ - 1 + 4096, 0.0f);
    next_ = 0;
}

std::size_t FirDecimator::process(const float* re, const float* im, std::size_t n, float* outRe, float* outIm) {
    const std::size_t hist = static_cast<std::size_t>(len_ - 1), total = hist + n;
    const bool two = im != 0;
    ensureSize(bufRe_, total);
    std::memcpy(&bufRe_[hist], re, n * sizeof(float));
    if (two) {
        ensureSize(bufIm_, total);
        std::memcpy(&bufIm_[hist], im, n * sizeof(float));
    }
    const float* h = &hr_[0];
    const int L = len_;
    std::size_t pos = next_, count = 0;
    while (pos + L <= total) {
        const float* xr = &bufRe_[pos];
        if (!complexTaps_) {
            if (two) {
                dot2(h, xr, &bufIm_[pos], L, outRe[count], outIm[count]);
            } else {
                outRe[count] = dot(h, xr, L);
            }
        } else {
            // (hr + j hi) * (xr + j xi) = (hr xr - hi xi) + j (hr xi + hi xr)
            const float* xi = &bufIm_[pos];
            float rr, ri, ir, ii;
            dot2(h, xr, xi, L, rr, ri);
            dot2(&hi_[0], xr, xi, L, ir, ii);
            outRe[count] = rr - ii;
            if (outIm) outIm[count] = ri + ir;
        }
        ++count;
        pos += decim_;
    }
    // Keep the last len-1 samples as history for the next block.
    if (hist) {
        std::memmove(&bufRe_[0], &bufRe_[total - hist], hist * sizeof(float));
        if (two) std::memmove(&bufIm_[0], &bufIm_[total - hist], hist * sizeof(float));
    }
    next_ = pos - (total - hist);
    return count;
}

// ---- Resampler -----------------------------------------------------------------
Resampler::Resampler() : taps_(8), step_(1.0), pos_(0.0), fill_(0) {}

void Resampler::setup(double inRate, double outRate, double passHz, double stopHz, double attenDb) {
    step_ = inRate / outRate;
    int n = kaiserLength(stopHz - passHz, inRate, attenDb);
    n = (n + 1) & ~1;  // even: the window straddles the output instant
    taps_ = std::max(8, std::min(n, 512));
    const double fc = 0.5 * (passHz + stopHz) / inRate;  // cycles per input sample
    const double beta = kaiserBeta(attenDb), i0b = besselI0(beta), half = taps_ / 2.0;
    table_.assign(static_cast<std::size_t>((kPhases + 1) * taps_), 0.0f);
    for (int p = 0; p <= kPhases; ++p) {
        const double frac = static_cast<double>(p) / kPhases;
        std::vector<double> h(taps_);
        double sum = 0;
        for (int k = 0; k < taps_; ++k) {
            // Distance (in input samples) from tap k to the output instant,
            // which sits `frac` past the centre of the window.
            const double t = k - (half - 1.0) - frac;
            const double sinc = std::fabs(t) < 1e-12 ? 2.0 * fc : std::sin(kTwoPi * fc * t) / (kPi * t);
            const double r = t / half;
            const double w = std::fabs(r) >= 1.0 ? 0.0 : besselI0(beta * std::sqrt(1.0 - r * r)) / i0b;
            h[k] = sinc * w;
            sum += h[k];
        }
        // Unity DC gain for every phase: no amplitude ripple at the output rate.
        for (int k = 0; k < taps_; ++k) table_[p * taps_ + k] = static_cast<float>(h[k] / sum);
    }
    reset();
}

void Resampler::reset() {
    // Start with taps-1 zeros of history so output begins immediately and
    // the output count tracks input * outRate / inRate from the first sample.
    fill_ = static_cast<std::size_t>(taps_ - 1);
    buf0_.assign(fill_ + 4096, 0.0f);
    buf1_.assign(fill_ + 4096, 0.0f);
    pos_ = 0.0;
}

std::size_t Resampler::maxOutput(std::size_t n) const {
    return static_cast<std::size_t>(static_cast<double>(fill_ + n) / step_) + 2;
}

std::size_t Resampler::process(const float* in0, const float* in1, std::size_t n, float* out0, float* out1) {
    const bool two = in1 != 0 && out1 != 0;
    ensureSize(buf0_, fill_ + n);
    std::memcpy(&buf0_[fill_], in0, n * sizeof(float));
    if (two) {
        ensureSize(buf1_, fill_ + n);
        std::memcpy(&buf1_[fill_], in1, n * sizeof(float));
    }
    fill_ += n;
    const int T = taps_;
    std::size_t count = 0;
    for (;;) {
        const std::size_t i0 = static_cast<std::size_t>(pos_);
        if (i0 + T > fill_) break;
        const double pf = (pos_ - static_cast<double>(i0)) * kPhases;
        const int p = static_cast<int>(pf);
        const float w = static_cast<float>(pf - p);
        const float* h0 = &table_[p * T];
        const float* h1 = h0 + T;
        if (two) {
            float a0, b0, a1, b1;
            dot2(h0, &buf0_[i0], &buf1_[i0], T, a0, b0);
            dot2(h1, &buf0_[i0], &buf1_[i0], T, a1, b1);
            out0[count] = a0 + w * (a1 - a0);
            out1[count] = b0 + w * (b1 - b0);
        } else {
            const float a0 = dot(h0, &buf0_[i0], T), a1 = dot(h1, &buf0_[i0], T);
            out0[count] = a0 + w * (a1 - a0);
        }
        ++count;
        pos_ += step_;
    }
    // Drop the samples no future output needs.
    std::size_t drop = static_cast<std::size_t>(pos_);
    if (drop > fill_) drop = fill_;
    if (drop) {
        std::memmove(&buf0_[0], &buf0_[drop], (fill_ - drop) * sizeof(float));
        if (two) std::memmove(&buf1_[0], &buf1_[drop], (fill_ - drop) * sizeof(float));
        fill_ -= drop;
        pos_ -= static_cast<double>(drop);
    }
    return count;
}

// ---- AGC ----------------------------------------------------------------------
Agc::Agc() : atk_(0.1f), dec_(0.001f), target_(0.4f), minEnv_(1e-4f), env_(0.0f), hangN_(0), hang_(0) {}

void Agc::setup(double rate, double attackS, double hangS, double decayS, float target, float maxGain) {
    atk_ = static_cast<float>(decayCoef(rate, attackS));
    dec_ = static_cast<float>(decayCoef(rate, decayS));
    hangN_ = static_cast<int>(hangS * rate);
    target_ = target;
    minEnv_ = target / maxGain;
    reset();
}

void Agc::reset() {
    env_ = 0.0f;
    hang_ = 0;
}

void Agc::process(float* x, std::size_t n) {
    float env = env_;
    int hang = hang_;
    for (std::size_t i = 0; i < n; ++i) {
        const float a = std::fabs(x[i]);
        if (a > env) {
            env += atk_ * (a - env);  // attack: follow peaks up quickly
            hang = hangN_;
        } else if (hang > 0) {
            --hang;                   // hold the gain through short pauses
        } else {
            env += dec_ * (a - env);  // decay: recover gain slowly
        }
        x[i] *= target_ / std::max(env, minEnv_);
    }
    env_ = env;
    hang_ = hang;
}

// ---- SSB high-pass (port of OFFgrid-SDR's SsbHighPass) --------------------------------
BiquadCascade::BiquadCascade() {}

void BiquadCascade::configureHighPass(double fc, double fs) {
    sec_.clear();
    if (fc < 20.0) return;
    // An 8th-order Butterworth split into four biquads; these are the Q values
    // of its four pole pairs (RBJ cookbook high-pass form).
    static const double kQ[4] = {0.5098, 0.6013, 0.9000, 2.5629};
    const double w = kTwoPi * fc / fs, c = std::cos(w), sn = std::sin(w);
    for (int i = 0; i < 4; ++i) {
        const double al = sn / (2.0 * kQ[i]), a0 = 1.0 + al;
        Section s;
        s.b0 = static_cast<float>((1.0 + c) / 2.0 / a0);
        s.b1 = static_cast<float>(-(1.0 + c) / a0);
        s.b2 = s.b0;
        s.a1 = static_cast<float>(-2.0 * c / a0);
        s.a2 = static_cast<float>((1.0 - al) / a0);
        s.z1 = s.z2 = 0.0f;
        sec_.push_back(s);
    }
}

void BiquadCascade::reset() {
    for (std::size_t i = 0; i < sec_.size(); ++i) sec_[i].z1 = sec_[i].z2 = 0.0f;
}

void BiquadCascade::process(float* buf, std::size_t n) {
    for (std::size_t j = 0; j < sec_.size(); ++j) {
        Section& s = sec_[j];
        float z1 = s.z1, z2 = s.z2;
        for (std::size_t i = 0; i < n; ++i) {  // transposed direct form II
            const float x = buf[i], y = s.b0 * x + z1;
            z1 = s.b1 * x - s.a1 * y + z2;
            z2 = s.b2 * x - s.a2 * y;
            buf[i] = y;
        }
        s.z1 = z1;
        s.z2 = z2;
    }
}

}  // namespace detail
}  // namespace dsp
