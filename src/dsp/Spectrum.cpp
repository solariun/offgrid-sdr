// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/Spectrum.cpp
//  Layer:  Signal processing
//
//  What this file does
//  -------------------
//  dsp::SpectrumAnalyzer: IQ -> power spectrum in dBFS for the spectrum
//  display and the waterfall.
//    - keeps the latest `size` samples in a ring buffer;
//    - at most framesPerSecond times per second of input, multiplies them by
//      a 4-term Blackman-Harris window (side lobes -92 dB, so strong stations
//      don't leak over weak ones) and runs the FFT;
//    - power per bin = |X|^2 / (sum of window)^2: a full-scale complex tone
//      (|x| = 1) centred on a bin reads 0 dBFS;
//    - exponential averaging in linear power (averaging = weight of the
//      previous frame), then dB, fftshifted (index 0 = -rate/2, DC in the
//      middle).
//
//  How it fits in
//  --------------
//  The application pushes every IQ block here and to the Receiver, and draws
//  powerDb() when push() returns true.
// ============================================================================
#include "Receiver.h"

#include "DspCommon.h"
#include "Fft.h"

#include <algorithm>

namespace dsp {

using namespace detail;

struct SpectrumAnalyzer::Impl {
    int n;
    double averaging;
    Fft fft;
    std::vector<float> window;
    double windowSum;
    std::vector<Complex> ring, work;
    std::size_t ringPos, filled;
    double sinceFrame;
    std::vector<float> power, db;
    bool havePower;

    Impl(int size, double avg) : n(0), averaging(avg), windowSum(1), ringPos(0), filled(0), sinceFrame(0), havePower(false) {
        resize(size);
    }

    void resize(int size) {
        int s = 16;
        while (s < size && s < (1 << 20)) s <<= 1;  // round up to a power of two
        n = s;
        fft.resize(n);
        window.resize(n);
        windowSum = 0;
        // 4-term Blackman-Harris (periodic form, for spectral analysis).
        const double a0 = 0.35875, a1 = 0.48829, a2 = 0.14128, a3 = 0.01168;
        for (int i = 0; i < n; ++i) {
            const double x = kTwoPi * i / n;
            window[i] = static_cast<float>(a0 - a1 * std::cos(x) + a2 * std::cos(2 * x) - a3 * std::cos(3 * x));
            windowSum += window[i];
        }
        ring.assign(n, Complex());
        work.assign(n, Complex());
        ringPos = filled = 0;
        sinceFrame = 0;
        power.assign(n, 0.0f);
        db.assign(n, -200.0f);
        havePower = false;
    }

    void compute() {
        // Oldest sample first: the ring's write position is the oldest.
        for (int i = 0; i < n; ++i) {
            const Complex& v = ring[(ringPos + i) % n];
            work[i] = Complex(v.real() * window[i], v.imag() * window[i]);
        }
        fft.forward(&work[0]);
        const float norm = static_cast<float>(1.0 / (windowSum * windowSum));
        const float keep = static_cast<float>(havePower ? std::max(0.0, std::min(0.999, averaging)) : 0.0);
        const int half = n / 2;
        for (int k = 0; k < n; ++k) {
            const float p = (work[k].real() * work[k].real() + work[k].imag() * work[k].imag()) * norm;
            const int j = (k + half) % n;  // fftshift: bin 0 (DC) goes to the middle
            power[j] = keep * power[j] + (1.0f - keep) * p;
        }
        havePower = true;
        for (int j = 0; j < n; ++j) db[j] = static_cast<float>(powerToDb(power[j]));
    }
};

SpectrumAnalyzer::SpectrumAnalyzer(int size, double averaging) : impl_(new Impl(size, averaging)) {}
SpectrumAnalyzer::~SpectrumAnalyzer() {}

void SpectrumAnalyzer::setSize(int size) {
    if (size != impl_->n) impl_->resize(size);
}
void SpectrumAnalyzer::setAveraging(double averaging) { impl_->averaging = averaging; }
int SpectrumAnalyzer::size() const { return impl_->n; }

bool SpectrumAnalyzer::push(const Complex* iq, std::size_t count, double sampleRate, double framesPerSecond) {
    Impl& s = *impl_;
    const std::size_t n = static_cast<std::size_t>(s.n);
    // Only the last n samples can matter.
    const std::size_t skip = count > n ? count - n : 0;
    for (std::size_t i = skip; i < count; ++i) {
        s.ring[s.ringPos] = iq[i];
        s.ringPos = s.ringPos + 1 == n ? 0 : s.ringPos + 1;
    }
    s.filled = std::min(n, s.filled + (count - skip));
    s.sinceFrame += static_cast<double>(count);
    const double interval = framesPerSecond > 0 ? sampleRate / framesPerSecond : 0;
    if (s.filled < n || s.sinceFrame < interval) return false;
    s.sinceFrame -= interval;
    if (s.sinceFrame > interval) s.sinceFrame = 0;  // a huge block: don't build a backlog
    s.compute();
    return true;
}

const std::vector<float>& SpectrumAnalyzer::powerDb() const { return impl_->db; }

}  // namespace dsp
