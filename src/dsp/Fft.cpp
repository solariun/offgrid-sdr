// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/Fft.cpp
//  Layer:  Signal processing
//
//  What this file does
//  -------------------
//  Radix-2 decimation-in-time FFT.  Step 1 reorders the input into
//  bit-reversed order; step 2 runs log2(N) passes of "butterflies", each
//  combining pairs of half-size transforms with a twiddle factor:
//      a' = a + w*b,   b' = a - w*b
//
//  How it fits in
//  --------------
//  See Fft.h.  Twiddles are computed in double and stored as float, so the
//  transform's own error stays far below the 8-bit input's noise.
// ============================================================================
#include "Fft.h"
#include "DspCommon.h"

#include <utility>

namespace dsp {
namespace detail {

Fft::Fft(int n) : n_(0) {
    if (n > 0) resize(n);
}

void Fft::resize(int n) {
    n_ = n;
    tw_.resize(n / 2);
    for (int k = 0; k < n / 2; ++k) {
        const double a = -kTwoPi * k / n;
        tw_[k] = std::complex<float>(static_cast<float>(std::cos(a)), static_cast<float>(std::sin(a)));
    }
    rev_.resize(n);
    int bits = 0;
    while ((1 << bits) < n) ++bits;
    for (int i = 0; i < n; ++i) {
        int r = 0;
        for (int b = 0; b < bits; ++b)
            if (i & (1 << b)) r |= 1 << (bits - 1 - b);
        rev_[i] = r;
    }
}

void Fft::forward(std::complex<float>* x) const {
    const int n = n_;
    for (int i = 0; i < n; ++i)
        if (rev_[i] > i) std::swap(x[i], x[rev_[i]]);
    for (int len = 2; len <= n; len <<= 1) {
        const int half = len / 2, stride = n / len;
        for (int start = 0; start < n; start += len) {
            for (int k = 0; k < half; ++k) {
                const std::complex<float> w = tw_[k * stride];
                std::complex<float>& a = x[start + k];
                std::complex<float>& b = x[start + k + half];
                // Written out by hand: std::complex operator* may take a slow
                // NaN-checking path without -ffast-math.
                const float br = b.real() * w.real() - b.imag() * w.imag();
                const float bi = b.real() * w.imag() + b.imag() * w.real();
                const float ar = a.real(), ai = a.imag();
                a = std::complex<float>(ar + br, ai + bi);
                b = std::complex<float>(ar - br, ai - bi);
            }
        }
    }
}

}  // namespace detail
}  // namespace dsp
