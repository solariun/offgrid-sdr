// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/Fft.h
//  Layer:  Signal processing (internal header)
//
//  What this file does
//  -------------------
//  A radix-2 complex FFT (iterative, decimation in time) with precomputed
//  twiddle factors and bit-reversal table.  No external library (no FFTW).
//
//  How it fits in
//  --------------
//  Used by SpectrumAnalyzer (spectrum/waterfall) and by the receiver's noise
//  floor estimate; the tests use it to find tone frequencies in audio.
//  Same algorithm as jtarrio/signals' dsp/fft.ts.
// ============================================================================
#ifndef OFFGRID_DSP_FFT_H
#define OFFGRID_DSP_FFT_H

#include <complex>
#include <vector>

namespace dsp {
namespace detail {

class Fft {
public:
    explicit Fft(int n = 0);
    void resize(int n);  // n must be a power of two
    int size() const { return n_; }
    // In-place forward transform: X[k] = sum x[n] exp(-j 2 pi k n / N).
    void forward(std::complex<float>* data) const;

private:
    int n_;
    std::vector<std::complex<float> > tw_;  // exp(-j 2 pi k / N), k < N/2
    std::vector<int> rev_;                  // bit-reversed index
};

}  // namespace detail
}  // namespace dsp

#endif
