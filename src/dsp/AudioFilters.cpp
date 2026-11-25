// ============================================================================
//  offgrid-dsp - liboffgrid-dsp, the signal processing library of OFFgrid-SDR TUI.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr) by diddy-boy.
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/AudioFilters.cpp
//  Layer:  Library liboffgrid-dsp
//
//  What this file does
//  -------------------
//  The implementations of AudioFilters.h, each a port of the JS class of the
//  same name in OFFgrid-SDR's index.html (sections "DC spike removal",
//  "noise blanker (impulse noise)", "spectral noise reduction (hiss)",
//  "hum filter", "Auto notch begin" (SmartNotch, AutoNotch) and "retune mute,
//  fade-in and headphone limiter").  The JS comments that explain WHY a
//  number is what it is are carried over (quoted or paraphrased).
//
//  How it fits in
//  --------------
//  Used by Receiver.cpp; tests/test_filters.cpp checks each filter on
//  synthetic signals (impulses, hiss, hum, whistles, peaks).
//
//  Contributing
//  ------------
//  JS numbers are doubles and its Float32Arrays store floats; the port keeps
//  float storage where the JS has Float32Arrays and double where it has
//  plain numbers, so the behaviour matches (not bit for bit: the JS's FFTs
//  compute in double).  Math.round(x) is floor(x + 0.5) (jsRound below).
//  Where the JS reads past an array (undefined -> NaN), the port reproduces
//  the outcome explicitly and says so.
// ============================================================================
#include "AudioFilters.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace dsp {

namespace {

const double kPi = 3.14159265358979323846;
const double kLn2 = 0.69314718055994530942;

inline double jsRound(double x) { return std::floor(x + 0.5); }  // JavaScript's Math.round

// A radix-2 complex FFT on planar arrays, as the JS SpectralNR/AutoNotch/
// fftC ones: bit reversal, then butterflies with tabulated twiddles.
// inverse = true uses exp(+j...) (no 1/N scaling, as the JS).
template <typename T>
class RadixFft {
public:
    explicit RadixFft(int n = 0) { resize(n); }
    void resize(int n) {
        n_ = n;
        rev_.assign(n, 0);
        cos_.assign(n / 2 > 0 ? n / 2 : 1, T(1));
        sin_.assign(n / 2 > 0 ? n / 2 : 1, T(0));
        if (n < 2) return;
        int bits = 0;
        while ((1 << bits) < n) ++bits;
        for (int i = 0; i < n; ++i) {
            int r = 0;
            for (int b = 0; b < bits; ++b) r |= ((i >> b) & 1) << (bits - 1 - b);
            rev_[i] = r;
        }
        for (int i = 0; i < n / 2; ++i) {
            cos_[i] = static_cast<T>(std::cos(2 * kPi * i / n));
            sin_[i] = static_cast<T>(std::sin(2 * kPi * i / n));
        }
    }
    void run(T* re, T* im, bool inverse) const {
        const int N = n_;
        for (int i = 0; i < N; ++i) {
            const int j = rev_[i];
            if (j > i) {
                std::swap(re[i], re[j]);
                std::swap(im[i], im[j]);
            }
        }
        for (int size = 2; size <= N; size <<= 1) {
            const int half = size >> 1, step = N / size;
            for (int i = 0; i < N; i += size)
                for (int k = 0; k < half; ++k) {
                    const T c = cos_[k * step], sn = inverse ? sin_[k * step] : -sin_[k * step];
                    const int a = i + k, b = a + half;
                    const T tr = re[b] * c - im[b] * sn, ti = re[b] * sn + im[b] * c;
                    re[b] = re[a] - tr;
                    im[b] = im[a] - ti;
                    re[a] += tr;
                    im[a] += ti;
                }
        }
    }

private:
    int n_;
    std::vector<int> rev_;
    std::vector<T> cos_, sin_;
};

// The (m >> 1)-th smallest of v[0..m) (what the JS gets by sorting and
// indexing [m >> 1]); reorders v.
inline float medianOf(std::vector<float>& v, std::size_t m) {
    std::nth_element(v.begin(), v.begin() + (m >> 1), v.begin() + m);
    return v[m >> 1];
}

}  // namespace

// ============================================================================
//  DcBlocker
// ============================================================================
DcBlocker::DcBlocker(double sampleRate) : a_(2e-5), dcI_(0), dcQ_(0), freq_(0), haveFreq_(false) {
    setSampleRate(sampleRate);
}
void DcBlocker::setSampleRate(double rate) { a_ = std::min(1e-3, 40 / rate); }  // ~25 ms time constant
void DcBlocker::reset() {
    dcI_ = dcQ_ = 0;
    haveFreq_ = false;
}
void DcBlocker::process(std::complex<float>* iq, std::size_t n, double centerHz) {
    if (n == 0) return;
    if (!haveFreq_ || centerHz != freq_) {  // retuned: start from this block's mean
        double si = 0, sq = 0;
        for (std::size_t i = 0; i < n; ++i) {
            si += iq[i].real();
            sq += iq[i].imag();
        }
        dcI_ = si / n;
        dcQ_ = sq / n;
        freq_ = centerHz;
        haveFreq_ = true;
    }
    double di = dcI_, dq = dcQ_;
    const double a = a_;
    for (std::size_t i = 0; i < n; ++i) {
        const float I = iq[i].real(), Q = iq[i].imag();
        di += a * (I - di);
        dq += a * (Q - dq);
        iq[i] = std::complex<float>(static_cast<float>(I - di), static_cast<float>(Q - dq));
    }
    dcI_ = di;
    dcQ_ = dq;
}

// ============================================================================
//  WideBlanker
// ============================================================================
// The reference level is the MEDIAN sample power of each block, not a
// running average that freezes while blanking: the old running average
// could lock up after a jump to a louder band (every sample looked like an
// impulse and all audio was blanked).  A median ignores short spikes but
// follows real level changes within one block.  Safety net: if more than 5%
// of a block would be blanked, that isn't impulse noise, so the block passes
// untouched.
WideBlanker::WideBlanker() : holdN_(4), back_(2), hold_(0), thrScale_(1) { setSampleRate(2048000); }

void WideBlanker::setSampleRate(double rate) {
    // Blank only while the spike lasts plus a short tail: each blank is
    // itself a small hole in the carrier, so keeping holes short matters as
    // much as catching the spike.
    holdN_ = std::max(2, static_cast<int>(jsRound(rate * 2e-6)));
    back_ = std::max(1, static_cast<int>(jsRound(rate * 1e-6)));
}
void WideBlanker::reset() { hold_ = 0; }
void WideBlanker::setThresholdDb(double db) { thrScale_ = std::pow(10.0, db / 10.0); }

void WideBlanker::process(std::complex<float>* iq, std::size_t n, BlankerStats* stats) {
    if (n == 0) return;
    // Median power from ~2048 evenly spread samples (cheap, robust to impulses).
    const std::size_t m = std::min<std::size_t>(2048, n);
    const double stride = static_cast<double>(n) / m;
    if (pw_.size() < m) pw_.resize(m);
    for (std::size_t k = 0; k < m; ++k) pw_[k] = std::norm(iq[static_cast<std::size_t>(std::floor(k * stride))]);
    // Median of exponentially distributed noise power = mean x ln 2; the
    // threshold is ~13 dB above the mean.
    const double thr = 20 * medianOf(pw_, m) / kLn2 * thrScale_;
    if (!(thr > 0)) return;
    // (JS "from 2.3": ONE pass notes where the impulses are (counting them
    //  all: more than 5% is a level change, skipped); the blanking then
    //  visits only those places.)
    const double lim = n * 0.05;
    const std::size_t cap = static_cast<std::size_t>(std::floor(lim)) + 1;
    if (pos_.size() < cap) pos_.resize(cap);
    std::size_t over = 0;
    for (std::size_t i = 0; i < n; ++i)
        if (std::norm(iq[i]) > thr) {
            if (over < cap) pos_[over] = static_cast<int>(i);
            over++;
        }
    if (over > lim) {
        if (stats) stats->skipped++;
        return;
    }
    // Each impulse starts or extends a stretch of holdN samples; a new
    // stretch after a gap also blanks the `back` samples before it; a
    // stretch running past the block's end carries into the next.
    long rs = 0, re = hold_;
    unsigned long long blanked = 0;
    const long N = static_cast<long>(n), hN = holdN_, bk = back_;
    const std::complex<float> zero(0, 0);
    for (std::size_t q = 0; q < over; ++q) {
        const long p = pos_[q];
        if (p < re) {  // a new impulse while blanking: the stretch goes on
            re = p + hN;
            continue;
        }
        if (re > rs) {
            const long e = std::min(re, N);
            for (long j = rs; j < e; ++j) iq[j] = zero;
            blanked += e - rs;
        }
        for (long j = std::max(0L, p - bk); j < p; ++j) iq[j] = zero;
        rs = p;
        re = p + hN;
    }
    if (re > rs) {
        const long e = std::min(re, N);
        for (long j = rs; j < e; ++j) iq[j] = zero;
        blanked += e - rs;
    }
    hold_ = static_cast<int>(std::max(0L, re - N));
    if (stats) stats->blanked += blanked;
}

// ============================================================================
//  ChannelBlanker
// ============================================================================
// For crowded bands (MW especially): the wideband blanker's threshold
// follows the whole band, so strong locals set it and a crackle that is loud
// next to a weak station never crosses it; and zeroing the whole band
// punches holes in those locals that splatter back.  This one extracts only
// the +-128 kHz around the tuned station (mix to 0 Hz + 3-stage moving
// average) and detects spikes there, ~10 dB above that slice's level; during
// a spike it subtracts just that slice from the wideband signal.  (JS tests:
// crowded MW crackle -18.6 -> -26.7 dB, quiet band -18.6 -> -31.3 dB, a clean
// noise-only channel touched 0.006% of the time.)
ChannelBlanker::ChannelBlanker() : rate_(2048000), holdN_(4), hold_(0), k_(0), ph_(0), lastOff_(0), haveOff_(false), thrScale_(1) {
    reset();
    setSampleRate(2048000);
}

void ChannelBlanker::reset() {
    std::memset(v_, 0, sizeof v_);
    std::memset(s_, 0, sizeof s_);
    k_ = 0;
    ph_ = 0;
    haveOff_ = false;
    hold_ = 0;
}
void ChannelBlanker::setSampleRate(double rate) {
    rate_ = rate;
    holdN_ = std::max(2, static_cast<int>(jsRound(rate * 2e-6)));
}
void ChannelBlanker::setThresholdDb(double db) { thrScale_ = std::pow(10.0, db / 10.0); }

void ChannelBlanker::process(std::complex<float>* iq, std::size_t n, double off, BlankerStats* stats) {
    if (n == 0) return;
    const int L = kL;
    const double iL = 1.0 / L;
    if (cre_.size() < n) {
        cre_.resize(n);
        cim_.resize(n);
        det_.resize(n);
    }
    if (!haveOff_ || off != lastOff_) {
        reset();
        lastOff_ = off;
        haveOff_ = true;
    }
    const double w = -2 * kPi * off / rate_, dr = std::cos(w), di = std::sin(w), ph0 = ph_;
    double cr = std::cos(ph0), ci = std::sin(ph0);
    int k = k_;
    double s0r = s_[0], s0i = s_[1], s1r = s_[2], s1i = s_[3], s2r = s_[4], s2i = s_[5];
    double *B0r = v_[0], *B0i = v_[1], *B1r = v_[2], *B1i = v_[3], *B2r = v_[4], *B2i = v_[5];
    float *cre = &cre_[0], *cim = &cim_[0], *det = &det_[0];
    for (std::size_t i = 0; i < n; ++i) {
        const double I = iq[i].real(), Q = iq[i].imag();
        const double xr = I * cr - Q * ci, xi = I * ci + Q * cr;
        const double t = cr * dr - ci * di;
        ci = cr * di + ci * dr;
        cr = t;
        s0r += xr - B0r[k];
        s0i += xi - B0i[k];
        B0r[k] = xr;
        B0i[k] = xi;
        const double y1r = s0r * iL, y1i = s0i * iL;
        s1r += y1r - B1r[k];
        s1i += y1i - B1i[k];
        B1r[k] = y1r;
        B1i[k] = y1i;
        const double y2r = s1r * iL, y2i = s1i * iL;
        s2r += y2r - B2r[k];
        s2i += y2i - B2i[k];
        B2r[k] = y2r;
        B2i[k] = y2i;
        const double zr = s2r * iL, zi = s2i * iL;
        cre[i] = static_cast<float>(zr);
        cim[i] = static_cast<float>(zi);
        det[i] = static_cast<float>(zr * zr + zi * zi);
        k = k + 1 == L ? 0 : k + 1;
    }
    s_[0] = s0r;
    s_[1] = s0i;
    s_[2] = s1r;
    s_[3] = s1i;
    s_[4] = s2r;
    s_[5] = s2i;
    k_ = k;
    ph_ = std::fmod(ph0 + w * static_cast<double>(n), 2 * kPi);  // exact phase carried to the next block
    const std::size_t m = std::min<std::size_t>(2048, n);
    const double stride = static_cast<double>(n) / m;
    if (pw_.size() < m) pw_.resize(m);
    for (std::size_t q = 0; q < m; ++q) pw_[q] = det[static_cast<std::size_t>(std::floor(q * stride))];
    const double thr = 10 * medianOf(pw_, m) / kLn2 * thrScale_;
    if (!(thr > 0)) return;
    // One pass notes the impulses, as in the wideband stage.
    const double lim = n * 0.05;
    const std::size_t cap = static_cast<std::size_t>(std::floor(lim)) + 1;
    if (pos_.size() < cap) pos_.resize(cap);
    std::size_t over = 0;
    for (std::size_t i = 0; i < n; ++i)
        if (det[i] > thr) {
            if (over < cap) pos_[over] = static_cast<int>(i);
            over++;
        }
    if (over > lim) {
        if (stats) stats->skipped++;
        return;
    }
    if (!over && !hold_) return;
    const long D = static_cast<long>(jsRound(3.0 * (L - 1) / 2));  // the slice lags the wideband signal by D samples
    long rs = 0, re = hold_;
    unsigned long long touched = 0;
    const long N = static_cast<long>(n), hN = holdN_;
    // Re-mix the slice up to the station and subtract it.
    struct Sub {
        static void run(std::complex<float>* iq, const float* cre, const float* cim, long a, long b, long D, double ph0,
                        double w, unsigned long long& touched) {
            for (long i = std::max(a, D); i < b; ++i) {
                const long j = i - D;
                const double ang = ph0 + w * j, ur = std::cos(ang), ui = -std::sin(ang), r2 = cre[i], i2 = cim[i];
                iq[j] = std::complex<float>(static_cast<float>(iq[j].real() - (r2 * ur - i2 * ui)),
                                            static_cast<float>(iq[j].imag() - (r2 * ui + i2 * ur)));
                touched++;
            }
        }
    };
    for (std::size_t q = 0; q < over; ++q) {
        const long p = pos_[q];
        if (p < re) {
            re = p + hN;
            continue;
        }
        if (re > rs) Sub::run(iq, cre, cim, rs, std::min(re, N), D, ph0, w, touched);
        rs = p;
        re = p + hN;
    }
    if (re > rs) Sub::run(iq, cre, cim, rs, std::min(re, N), D, ph0, w, touched);
    hold_ = static_cast<int>(std::max(0L, re - N));
    if (stats) stats->blanked += touched;
}

// ============================================================================
//  NoiseBlanker
// ============================================================================
NoiseBlanker::NoiseBlanker() {}
void NoiseBlanker::setSampleRate(double rate) {
    w_.setSampleRate(rate);
    c_.setSampleRate(rate);
}
void NoiseBlanker::reset() {
    w_.reset();
    c_.reset();
}
void NoiseBlanker::setThresholdDb(double db) {
    w_.setThresholdDb(db);
    c_.setThresholdDb(db);
}
void NoiseBlanker::process(std::complex<float>* iq, std::size_t n, double offsetHz) {
    w_.process(iq, n, &stats_);
    c_.process(iq, n, offsetHz, &stats_);
    stats_.total += n;
}

// ============================================================================
//  SpectralNR
// ============================================================================
// Short-time FFT (512 points, 50% overlap, sqrt-Hann windows for perfect
// reconstruction).  Per bin it tracks the noise floor with minimum
// statistics (follows dips quickly, rises slowly, so speech and music don't
// count as noise) and applies a Wiener-style gain 1 - alpha * noise/signal,
// with a floor, smoothed over time to limit "musical noise".
namespace {
struct NrLevel {
    double alpha, floor;
};
const NrLevel kNrLevels[4] = {{0, 1}, {1.4, 0.32}, {2.2, 0.18}, {3.5, 0.09}};  // (JS NR_LEVELS; [0] unused)
const unsigned kNrQueue = 16384;                                                // (JS this.q, a ring)
}  // namespace

SpectralNR::SpectralNR(int N) : N_(N), H_(N / 2), fill_(0), rate_(48000), qr_(0), qw_(0) {
    win_.resize(N);
    for (int i = 0; i < N; ++i) win_[i] = static_cast<float>(std::sqrt(0.5 - 0.5 * std::cos(2 * kPi * i / N)));
    // FFT tables (the JS's rev/cos/sin)
    rev_.resize(N);
    int bits = 0;
    while ((1 << bits) < N) ++bits;
    for (int i = 0; i < N; ++i) {
        int r = 0;
        for (int b = 0; b < bits; ++b) r |= ((i >> b) & 1) << (bits - 1 - b);
        rev_[i] = r;
    }
    cos_.resize(N / 2);
    sin_.resize(N / 2);
    for (int i = 0; i < N / 2; ++i) {
        cos_[i] = static_cast<float>(std::cos(2 * kPi * i / N));
        sin_[i] = static_cast<float>(std::sin(2 * kPi * i / N));
    }
    reset();
    setSampleRate(48000);
}

void SpectralNR::reset() {
    const int N = N_, B = N / 2 + 1;
    frame_.assign(N, 0.0f);
    re_.assign(N, 0.0f);
    im_.assign(N, 0.0f);
    ola_.assign(H_, 0.0f);
    ps_.assign(B, 0.0f);
    noise_.assign(B, 1e30f);
    gain_.assign(B, 1.0f);
    q_.assign(kNrQueue, 0.0f);
    qr_ = qw_ = 0;
    fill_ = 0;
}

void SpectralNR::setSampleRate(double rate) {
    rate_ = rate > 0 ? rate : 48000;
    // Subtract only 30% as hard in 1.5-4.5 kHz, where consonants (speech
    // clarity) sit.  Full strength there removed ~22% of that range at High
    // and made male voices muddy.
    const int B = N_ / 2 + 1;
    aprof_.resize(B);
    for (int k = 0; k < B; ++k) {
        const double f = k * rate_ / N_;
        aprof_[k] = f >= 1500 && f <= 4500 ? 0.3f : 1.0f;
    }
}

void SpectralNR::processFrame(double alpha, double floorG) {
    const int N = N_, H = H_, B = N / 2 + 1;
    float *re = &re_[0], *im = &im_[0], *ps = &ps_[0], *noise = &noise_[0], *gain = &gain_[0];
    for (int i = 0; i < N; ++i) {
        re[i] = frame_[i] * win_[i];
        im[i] = 0;
    }
    // forward FFT (the JS's fft(re, im, false))
    struct F {
        static void run(const SpectralNR& s, float* re, float* im, bool inv) {
            const int N = s.N_;
            for (int i = 0; i < N; ++i) {
                const int j = s.rev_[i];
                if (j > i) {
                    std::swap(re[i], re[j]);
                    std::swap(im[i], im[j]);
                }
            }
            for (int size = 2; size <= N; size <<= 1) {
                const int half = size >> 1, step = N / size;
                for (int i = 0; i < N; i += size)
                    for (int k = 0; k < half; ++k) {
                        const float c = s.cos_[k * step], sn = inv ? s.sin_[k * step] : -s.sin_[k * step];
                        const int a = i + k, b = a + half;
                        const float tr = re[b] * c - im[b] * sn, ti = re[b] * sn + im[b] * c;
                        re[b] = re[a] - tr;
                        im[b] = im[a] - ti;
                        re[a] += tr;
                        im[a] += ti;
                    }
            }
        }
    };
    F::run(*this, re, im, false);
    // (JS: a gap in the audio (a busy computer, a retune) is exact zeros,
    //  which real noise and speech never are: the floor isn't learned from a
    //  frame with more than a few.  Taken as noise, a single 50 ms gap dropped
    //  the floor almost to zero, and noise reduction then did little for many
    //  seconds.  Exact zeros, not "much quieter": speech pauses must still
    //  teach it.)
    int zeros = 0;
    for (int i = 0; i < N; ++i)
        if (frame_[i] == 0) zeros++;
    const bool gap = zeros > N * 0.05;
    for (int k = 0; k < B; ++k) {
        const double p = static_cast<double>(re[k]) * re[k] + static_cast<double>(im[k]) * im[k];
        ps[k] = static_cast<float>(ps[k] != 0 ? 0.6 * ps[k] + 0.4 * p : p);  // smoothed power
        if (!gap) noise[k] = ps[k] < noise[k] ? ps[k] : static_cast<float>(noise[k] * 1.0025 + 1e-12);  // min-statistics floor, slow rise
        const double g = std::max(floorG, 1 - alpha * aprof_[k] * noise[k] * 1.5 / (ps[k] + 1e-20));
        gain[k] = static_cast<float>(g > gain[k] ? 0.5 * gain[k] + 0.5 * g : 0.8 * gain[k] + 0.2 * g);  // fast attack, gentle release
    }
    for (int k = 0; k < B; ++k) {  // light smoothing across bins
        const double g = k > 0 && k < B - 1 ? 0.25 * gain[k - 1] + 0.5 * gain[k] + 0.25 * gain[k + 1] : gain[k];
        re[k] = static_cast<float>(re[k] * g);
        im[k] = static_cast<float>(im[k] * g);
        if (k > 0 && k < N / 2) {
            re[N - k] = re[k];
            im[N - k] = -im[k];
        }
    }
    F::run(*this, re, im, true);
    for (int i = 0; i < N; ++i) re[i] = static_cast<float>(static_cast<double>(re[i]) / N * win_[i]);
    // Overlap-add: emit H samples.
    for (int i = 0; i < H; ++i) {
        q_[qw_] = ola_[i] + re[i];
        qw_ = (qw_ + 1) & (kNrQueue - 1);
        ola_[i] = re[H + i];
    }
    std::memmove(&frame_[0], &frame_[H], (N - H) * sizeof(float));  // (JS frame.copyWithin(0, H))
}

void SpectralNR::process(float* buf, std::size_t n, int level) {
    if (level < 1) return;
    if (level > 3) level = 3;
    const NrLevel& lv = kNrLevels[level];
    const int H = H_;
    for (std::size_t i = 0; i < n; ++i) {
        frame_[H + fill_++] = buf[i];
        if (fill_ == H) {
            processFrame(lv.alpha, lv.floor);
            fill_ = 0;
        }
        if (qr_ != qw_) {
            buf[i] = q_[qr_];
            qr_ = (qr_ + 1) & (kNrQueue - 1);
        } else {
            buf[i] = 0;
        }
    }
}

// ============================================================================
//  HumFilter
// ============================================================================
// Hum is a buzz of harmonics (50, 100, 150 ... Hz), not one tone.  The
// notches are narrow, so the body of a voice (a male fundamental is
// 85-180 Hz) passes, unlike a plain low-cut.
HumFilter::HumFilter() : f0_(0), fs_(0) {}

void HumFilter::configure(double f0, double fs) {
    if (f0 == f0_ && fs == fs_) return;
    f0_ = f0;
    fs_ = fs;
    sec_.clear();
    if (!(f0 > 0) || !(fs > 0)) return;
    for (int k = 1; k <= 8; ++k) {
        if (k * f0 >= 0.5 * fs) break;  // Addition to the JS: no notch at or past Nyquist (low rates)
        // Notch width grows with the harmonic (6 Hz ... 20 Hz) so mains
        // drift, which multiplies up the harmonics, still lands inside.
        const double bw = 4 + 2 * k, w = 2 * kPi * k * f0 / fs, Q = k * f0 / bw, al = std::sin(w) / (2 * Q),
                     c = std::cos(w), a0 = 1 + al;
        Section s = {1 / a0, -2 * c / a0, 1 / a0, -2 * c / a0, (1 - al) / a0, 0, 0};
        sec_.push_back(s);
    }
}

void HumFilter::reset() {
    for (std::size_t i = 0; i < sec_.size(); ++i) sec_[i].z1 = sec_[i].z2 = 0;
}

void HumFilter::process(float* buf, std::size_t n) {
    for (std::size_t s = 0; s < sec_.size(); ++s) {
        Section& S = sec_[s];
        double z1 = S.z1, z2 = S.z2;
        const double b0 = S.b0, b1 = S.b1, b2 = S.b2, a1 = S.a1, a2 = S.a2;
        for (std::size_t i = 0; i < n; ++i) {
            const double x = buf[i], y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;
            buf[i] = static_cast<float>(y);
        }
        S.z1 = z1;
        S.z2 = z2;
    }
}

// ============================================================================
//  AutoNotch
// ============================================================================
// Automatic notch filter: removes steady whistles (heterodynes from a nearby
// carrier, tuning whistles) from AM and SSB audio.  Every 0.1 s a fine
// spectrum of the last 85 ms of audio (11.7 Hz resolution) is checked for
// narrow peaks standing well above their surroundings.  A peak that stays on
// the same frequency for about a second is a whistle - the harmonics of a
// voice keep moving as its pitch changes, so they don't qualify - and gets a
// very narrow notch (20 Hz wide), which follows it if it drifts.  Up to 3
// whistles at once; a notch is removed when its whistle goes.
namespace {
struct CandGreater {
    template <typename C>
    bool operator()(const C& a, const C& b) const { return a.p > b.p; }
};
}  // namespace

AutoNotch::AutoNotch() : fs_(0), pos_(0), sinceA_(0), count_(0) {
    const int N = kN, B = N / 2 + 1;
    win_.resize(N);
    for (int i = 0; i < N; ++i) win_[i] = static_cast<float>(0.5 - 0.5 * std::cos(2 * kPi * i / N));
    rev_.resize(N);
    for (int i = 0, b = 12; i < N; ++i) {  // log2(4096) = 12
        int r = 0;
        for (int j = 0; j < b; ++j) r |= ((i >> j) & 1) << (b - 1 - j);
        rev_[i] = r;
    }
    re_.resize(N);
    im_.resize(N);
    L_.resize(B);
    cs_.resize(B + 1);
    cand_.resize(B);
    reset();
}

void AutoNotch::reset() {
    ring_.assign(kN, 0.0f);
    pos_ = 0;
    sinceA_ = 0;
    prom_.assign(kN / 2 + 1, 0.0f);
    count_ = 0;
}

std::vector<double> AutoNotch::notches() const {
    std::vector<double> v;
    for (int i = 0; i < count_; ++i) v.push_back(notches_[i].f);
    return v;
}

void AutoNotch::design(Notch& n, double f) const {  // a 20 Hz-wide notch (as the hum filter)
    const double w = 2 * kPi * f / fs_, Q = f / 20, al = std::sin(w) / (2 * Q), c = std::cos(w), a0 = 1 + al;
    n.f = f;
    n.b0 = 1 / a0;
    n.b1 = -2 * c / a0;
    n.b2 = 1 / a0;
    n.a1 = -2 * c / a0;
    n.a2 = (1 - al) / a0;
}

void AutoNotch::analyse() {
    const int N = kN, B = N / 2 + 1;
    float *re = &re_[0], *im = &im_[0], *L = &L_[0], *prom = &prom_[0];
    const double fs = fs_, bin = fs / N;
    for (int i = 0; i < N; ++i) {
        re[i] = ring_[(pos_ + i) % N] * win_[i];
        im[i] = 0;
    }
    // The JS's own FFT (twiddles by recursion, in place).
    for (int i = 0; i < N; ++i) {
        const int j = rev_[i];
        if (j > i) {
            std::swap(re[i], re[j]);
            std::swap(im[i], im[j]);
        }
    }
    for (int len = 2; len <= N; len <<= 1) {
        const double ang = -2 * kPi / len, wr = std::cos(ang), wi = std::sin(ang);
        for (int i = 0; i < N; i += len) {
            double cr = 1, ci = 0;
            for (int k = 0; k < len / 2; ++k) {
                const int p = i + k, q = p + len / 2;
                const double tr = re[q] * cr - im[q] * ci, ti = re[q] * ci + im[q] * cr;
                re[q] = static_cast<float>(re[p] - tr);
                im[q] = static_cast<float>(im[p] - ti);
                re[p] = static_cast<float>(re[p] + tr);
                im[p] = static_cast<float>(im[p] + ti);
                const double t = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = t;
            }
        }
    }
    for (int k = 0; k < B; ++k)
        L[k] = static_cast<float>(10 * std::log10(static_cast<double>(re[k]) * re[k] + static_cast<double>(im[k]) * im[k] + 1e-12));
    double* cs = &cs_[0];
    cs[0] = 0;
    for (int k = 0; k < B; ++k) cs[k + 1] = cs[k] + L[k];
    const int lo = static_cast<int>(std::ceil(200 / bin)), hi = static_cast<int>(std::floor(3500 / bin)), W = 24, G = 4;  // 200 Hz - 3.5 kHz
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (int k = 0; k < B; ++k) {
        float p = 0;
        if (k >= lo && k <= hi && k >= 2 && k + 2 < B && L[k] >= L[k - 1] && L[k] >= L[k + 1] && L[k] >= L[k - 2] &&
            L[k] >= L[k + 2]) {
            // Background: mean level from 4 to 24 bins away on each side (~50-280 Hz).
            // (As the JS: when that window runs off the array (the first 24
            //  bins, under ~280 Hz at 48 kHz) the JS reads undefined, the
            //  result is NaN and the bin can never qualify; reproduced here.)
            if (k - W < 0 || k + W + 1 > B) p = nan;
            else p = static_cast<float>(L[k] - (cs[k - G + 1] - cs[k - W] + cs[k + W + 1] - cs[k + G]) / (2 * (W - G + 1)));
        }
        const float pp = p != p ? nan : std::max(0.0f, p);
        prom[k] = prom[k] * 0.9f + 0.1f * pp;  // a peak must persist (~1 s)
    }
    // Whistles: the strongest persistent peaks (merging neighbouring bins).
    Cand* cand = &cand_[0];
    int nc = 0;
    for (int k = std::max(lo, 1); k <= hi && k + 1 < B; ++k)
        if (prom[k] > 12 && prom[k] >= prom[k - 1] && prom[k] >= prom[k + 1]) {
            const double a = L[k - 1], b = L[k], c = L[k + 1], den = 2 * (a - 2 * b + c);
            const double d = (a - c) / (den != 0 && den == den ? den : 1);  // sub-bin frequency (JS: || 1)
            cand[nc].f = (k + std::max(-0.5, std::min(0.5, d))) * bin;
            cand[nc].p = prom[k];
            cand[nc].k = k;
            nc++;
        }
    std::stable_sort(cand, cand + nc, CandGreater());
    Cand keep[3];
    int nk = 0;
    for (int i = 0; i < nc && nk < 3; ++i) {
        bool apart = true;
        for (int j = 0; j < nk; ++j)
            if (!(std::fabs(keep[j].f - cand[i].f) > 40)) apart = false;
        if (apart) keep[nk++] = cand[i];
    }
    // Update the notches: follow ones that moved a little, add new ones,
    // drop ones whose whistle went.
    Notch next[3];
    bool used[3] = {false, false, false};
    int nn = 0;
    for (int c = 0; c < nk; ++c) {
        int old = -1;
        for (int i = 0; i < count_; ++i)
            if (std::fabs(notches_[i].f - keep[c].f) < 20) {
                old = i;
                break;
            }
        if (old >= 0) {
            Notch o = notches_[old];
            if (std::fabs(o.f - keep[c].f) > 2) design(o, keep[c].f);
            used[old] = true;
            next[nn++] = o;
        } else {
            Notch n;
            n.z1 = n.z2 = 0;
            design(n, keep[c].f);
            next[nn++] = n;
        }
    }
    for (int i = 0; i < count_; ++i)  // hysteresis: keep a notch while its peak is still fairly there
        if (!used[i]) {
            const int k = static_cast<int>(jsRound(notches_[i].f / bin));
            if (k >= 0 && k < B && prom[k] > 6 && nn < 3) next[nn++] = notches_[i];
        }
    for (int i = 0; i < nn; ++i) notches_[i] = next[i];
    count_ = nn;
}

void AutoNotch::process(float* x, std::size_t n, double fs) {
    if (fs != fs_) {
        fs_ = fs;
        reset();
    }
    const int N = kN, hop = static_cast<int>(jsRound(fs / 10));
    for (std::size_t i = 0; i < n; ++i) {
        ring_[pos_] = x[i];
        pos_ = (pos_ + 1) % N;
        if (++sinceA_ >= hop) {
            sinceA_ = 0;
            analyse();
        }
    }
    for (int s = 0; s < count_; ++s) {
        Notch& S = notches_[s];
        double z1 = S.z1, z2 = S.z2;
        const double b0 = S.b0, b1 = S.b1, b2 = S.b2, a1 = S.a1, a2 = S.a2;
        for (std::size_t i = 0; i < n; ++i) {
            const double v = x[i], y = b0 * v + z1;
            z1 = b1 * v - a1 * y + z2;
            z2 = b2 * v - a2 * y;
            x[i] = static_cast<float>(y);
        }
        S.z1 = z1;
        S.z2 = z2;
    }
}

// ============================================================================
//  SmartNotch
// ============================================================================
// Finds steady whines in the audio, shows them as lines you can drag, and
// notches them.  A whine: a narrow peak, 10 dB over its surroundings, at
// the same pitch through 85% of the last 3 s.  Speech glides about; a held
// musical note rarely lasts that long; a heterodyne whine goes on for
// minutes.
SmartNotch::SmartNotch(double fs) : fs_(fs), ring_(160000, 0.0f), post_(24000, 0.0f), pos_(0), ppos_(0), count_(0) {}

void SmartNotch::coeffs(Notch& n) const {  // RBJ notch, Q 30 (about 40 Hz wide at 1 kHz)
    const double w = 2 * kPi * n.f / fs_, al = std::sin(w) / (2 * 30), a0 = 1 + al, c = std::cos(w);
    n.b0 = 1 / a0;
    n.b1 = -2 * c / a0;
    n.b2 = 1 / a0;
    n.a1 = -2 * c / a0;
    n.a2 = (1 - al) / a0;
}

std::vector<double> SmartNotch::notches() const {
    std::vector<double> v;
    for (int i = 0; i < count_; ++i) v.push_back(notches_[i].f);
    return v;
}

int SmartNotch::add(double f) {
    if (count_ >= kMaxNotches) {  // the oldest goes (JS notches.shift())
        for (int i = 1; i < count_; ++i) notches_[i - 1] = notches_[i];
        count_--;
    }
    Notch n;
    n.f = f;
    n.x1 = n.x2 = n.y1 = n.y2 = 0;
    coeffs(n);
    notches_[count_++] = n;
    return count_ - 1;
}

void SmartNotch::move(int index, double f) {
    if (index < 0 || index >= count_) return;
    notches_[index].f = std::max(80.0, std::min(4500.0, f));
    coeffs(notches_[index]);
}

void SmartNotch::remove(int index) {
    if (index < 0 || index >= count_) return;
    for (int i = index + 1; i < count_; ++i) notches_[i - 1] = notches_[i];
    count_--;
}

void SmartNotch::clear() { count_ = 0; }

void SmartNotch::process(float* x, std::size_t n, double fs) {
    if (fs != fs_) {
        fs_ = fs;
        for (int i = 0; i < count_; ++i) coeffs(notches_[i]);
    }
    const std::size_t L = ring_.size();
    std::size_t p = pos_;
    for (std::size_t i = 0; i < n; ++i) {  // (the audio before notching: to find and show whines)
        ring_[p] = x[i];
        if (++p == L) p = 0;
    }
    pos_ = p;
    for (int s = 0; s < count_; ++s) {
        Notch& N = notches_[s];
        double x1 = N.x1, x2 = N.x2, y1 = N.y1, y2 = N.y2;
        const double b0 = N.b0, b1 = N.b1, b2 = N.b2, a1 = N.a1, a2 = N.a2;
        for (std::size_t i = 0; i < n; ++i) {
            const double v = x[i], y = b0 * v + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            x2 = x1;
            x1 = v;
            y2 = y1;
            y1 = y;
            x[i] = static_cast<float>(y);
        }
        N.x1 = x1;
        N.x2 = x2;
        N.y1 = y1;
        N.y2 = y2;
    }
    const std::size_t PL = post_.size();
    std::size_t q = ppos_;
    for (std::size_t i = 0; i < n; ++i) {
        post_[q] = x[i];
        if (++q == PL) q = 0;
    }
    ppos_ = q;
}

void SmartNotch::recent(std::size_t n, std::vector<float>& out, bool post) const {
    const std::vector<float>& R = post ? post_ : ring_;
    const std::size_t L = R.size(), p = post ? ppos_ : pos_;
    if (n > L) n = L;  // Addition to the JS: never more than was kept
    out.resize(n);
    for (std::size_t i = 0; i < n; ++i) out[i] = R[(p + L - n + i) % L];
}

namespace {
// The JS's spectrum(): averaged power spectrum (4096-point Hann slices, half
// overlapping) in dB, and each slice's dB spectrum.
void smartSpectrum(const std::vector<float>& x, std::vector<float>& avg, std::vector<std::vector<float> >& frames) {
    const int N = 4096, B = N / 2;
    RadixFft<double> fft(N);
    std::vector<double> acc(B, 0.0), re(N), im(N), win(N);
    for (int i = 0; i < N; ++i) win[i] = 0.5 - 0.5 * std::cos(2 * kPi * i / N);
    frames.clear();
    int k = 0;
    for (std::size_t st = 0; st + N <= x.size(); st += N / 2, ++k) {
        for (int i = 0; i < N; ++i) {
            re[i] = x[st + i] * win[i];
            im[i] = 0;
        }
        fft.run(&re[0], &im[0], false);
        frames.push_back(std::vector<float>(B));
        std::vector<float>& d = frames.back();
        for (int j = 0; j < B; ++j) {
            const double v = re[j] * re[j] + im[j] * im[j];
            acc[j] += v;
            d[j] = static_cast<float>(10 * std::log10(v + 1e-20));
        }
    }
    avg.assign(B, 0.0f);
    for (int j = 0; j < B; ++j) avg[j] = static_cast<float>(10 * std::log10(acc[j] / std::max(1, k) + 1e-20));
}

double prominence(const std::vector<float>& d, int j) {
    float side[6] = {d[j - 4], d[j - 5], d[j - 6], d[j + 4], d[j + 5], d[j + 6]};
    std::sort(side, side + 6);
    return d[j] - side[2];
}

struct ByProm {
    bool operator()(const SmartNotch::Whine& a, const SmartNotch::Whine& b) const { return b.prom < a.prom; }
};
struct ByNear {
    double near;
    bool operator()(const SmartNotch::Whine& a, const SmartNotch::Whine& b) const {
        return std::fabs(a.f - near) < std::fabs(b.f - near);
    }
};
}  // namespace

std::vector<SmartNotch::Whine> SmartNotch::find(const std::vector<float>& x, double fs, double near, int* mainsHumOut) {
    std::vector<float> avg;
    std::vector<std::vector<float> > frames;
    smartSpectrum(x, avg, frames);
    const double bin = fs / 4096;
    const int B = static_cast<int>(avg.size());
    // (Addition to the JS: hi also stays 7 bins inside the array, for low rates)
    const int lo = std::max(6, std::max(3, static_cast<int>(jsRound(120 / bin))));
    const int hi = std::min(B - 7, std::min(B - 4, static_cast<int>(jsRound(4300 / bin))));
    std::vector<Whine> found;
    for (int j = lo; j <= hi; ++j) {
        if (!(avg[j] >= avg[j - 1] && avg[j] >= avg[j + 1])) continue;
        if (prominence(avg, j) < 12) continue;
        std::size_t hits = 0;
        for (std::size_t f = 0; f < frames.size(); ++f) {
            const std::vector<float>& d = frames[f];
            const float m = std::max(d[j - 1], std::max(d[j], d[j + 1]));
            const float mn = std::min(std::min(d[j - 5], d[j + 5]), std::min(d[j - 6], d[j + 6]));
            if (m - mn >= 10) hits++;
        }
        if (hits < frames.size() * 0.85) continue;
        const double a = avg[j - 1], b = avg[j], c = avg[j + 1], den = 2 * (a - 2 * b + c);
        const double off = (a - c) / (den != 0 && den == den ? den : 1);  // (parabolic: between bins)
        Whine w;
        w.f = (j + std::max(-0.5, std::min(0.5, off))) * bin;
        w.prom = prominence(avg, j);
        found.push_back(w);
    }
    if (near > 0) {
        ByNear cmp;
        cmp.near = near;
        std::stable_sort(found.begin(), found.end(), cmp);
    } else {
        std::stable_sort(found.begin(), found.end(), ByProm());
    }
    // Mains hum is a comb of 50 (or 60) Hz and its harmonics: the hum
    // filter's job, not a whine.
    const int hum = found.empty() ? 0 : mainsHum(x, fs);
    if (mainsHumOut) *mainsHumOut = 0;
    std::vector<Whine> kept;
    for (std::size_t i = 0; i < found.size(); ++i) {
        const Whine& w = found[i];
        if (hum) {
            const double k = jsRound(w.f / hum);
            if (w.f < 450 && k >= 1 && std::fabs(w.f - k * hum) < 12) continue;
        }
        if (near > 0 && !(std::fabs(w.f - near) < 250)) continue;
        kept.push_back(w);
    }
    if (hum && mainsHumOut) {
        // (JS: this.hum is set when mains hum was all there was)
        bool anyKept = false;
        for (std::size_t i = 0; i < found.size(); ++i) {
            const double k = jsRound(found[i].f / hum);
            if (!(found[i].f < 450 && k >= 1 && std::fabs(found[i].f - k * hum) < 12)) anyKept = true;
        }
        if (!anyKept) *mainsHumOut = hum;
    }
    return kept;
}

// 50 or 60 if the last second of audio has mains hum (at least 2 of the
// first 4 harmonics well above the rest), else 0.  The 11.7 Hz spectrum is
// too coarse to tell a 50 Hz comb from a 60 Hz one, so the lines are
// measured exactly (Goertzel, about 1 Hz wide).
int SmartNotch::mainsHum(const std::vector<float>& x, double fs) {
    const int N = static_cast<int>(jsRound(fs));
    if (N <= 0) return 0;
    std::vector<double> w(N, 0.0);
    const std::size_t have = std::min<std::size_t>(x.size(), static_cast<std::size_t>(N));
    for (std::size_t i = 0; i < have; ++i) {  // the last second (zeros before, if shorter)
        const std::size_t at = N - have + i;
        w[at] = x[x.size() - have + i] * (0.5 - 0.5 * std::cos(2 * kPi * at / N));
    }
    struct G {
        static double pw(const std::vector<double>& w, double f, double fs) {
            const double c = 2 * std::cos(2 * kPi * f / fs);
            double s1 = 0, s2 = 0;
            for (std::size_t i = 0; i < w.size(); ++i) {
                const double s0 = w[i] + c * s1 - s2;
                s2 = s1;
                s1 = s0;
            }
            return s1 * s1 + s2 * s2 - c * s1 * s2 + 1e-20;
        }
        static double line(const std::vector<double>& w, double f, double fs) {  // (the mains drifts a little, more so at the harmonics)
            const double d[5] = {-1, -0.5, 0, 0.5, 1};
            double m = -1e300;
            for (int i = 0; i < 5; ++i) m = std::max(m, pw(w, f + d[i] * std::max(1.0, f / 100), fs));
            return m;
        }
    };
    const double probeHz[7] = {75, 87, 135, 165, 215, 265, 330};  // (frequencies on neither comb)
    double probes[7];
    for (int i = 0; i < 7; ++i) probes[i] = G::pw(w, probeHz[i], fs);
    std::sort(probes, probes + 7);
    const double ref = probes[3];
    int best = 0;
    double bestScore = 0;
    const int combs[2] = {50, 60};
    for (int c = 0; c < 2; ++c) {
        const int f0 = combs[c];
        int n = 0;
        double db = 0;
        for (int k = 1; k <= 4; ++k) {
            const double d = 10 * std::log10(G::line(w, k * f0, fs) / ref);
            if (d >= 12) {
                n++;
                db += d;
            }
        }
        if (n >= 2 && n * 1000 + db > bestScore) {
            best = f0;
            bestScore = n * 1000 + db;
        }
    }
    return best;
}

double SmartNotch::quickSpectrum(const std::vector<float>& x, double fs, int top, std::vector<float>& avgDb) {
    const int N = 4096, B = N / 2;
    top = std::max(0, std::min(B, top));
    RadixFft<double> fft(N);
    std::vector<double> re(N), im(N), win(N), acc(top, 0.0);
    for (int i = 0; i < N; ++i) win[i] = 0.5 - 0.5 * std::cos(2 * kPi * i / N);
    int k = 0;
    for (std::size_t st = 0; st + N <= x.size(); st += N / 2, ++k) {
        for (int i = 0; i < N; ++i) {
            re[i] = x[st + i] * win[i];
            im[i] = 0;
        }
        fft.run(&re[0], &im[0], false);
        for (int j = 0; j < top; ++j) acc[j] += re[j] * re[j] + im[j] * im[j];
    }
    avgDb.assign(top, 0.0f);
    for (int j = 0; j < top; ++j) avgDb[j] = static_cast<float>(10 * std::log10(acc[j] / std::max(1, k) + 1e-20));
    return fs / N;
}

double SmartNotch::snap(double f, const std::vector<float>& avg, double bin, int top) {
    if (avg.empty() || !(bin > 0)) return f;
    const int size = static_cast<int>(avg.size());
    const int lo = std::max(7, static_cast<int>(jsRound((f - 40) / bin)));
    const int hi = std::min(std::min(top, size) - 7, static_cast<int>(jsRound((f + 40) / bin)));
    int bj = -1;
    double bv = -1e9;
    for (int j = lo; j <= hi; ++j)
        if (avg[j] > bv) {
            bv = avg[j];
            bj = j;
        }
    if (bj < 0 || bj + 6 >= size || avg[bj] - std::max(avg[bj - 6], avg[bj + 6]) < 8 ||
        !(avg[bj] >= avg[bj - 1] && avg[bj] >= avg[bj + 1]))
        return f;
    const double a = avg[bj - 1], b = avg[bj], d = avg[bj + 1], den = 2 * (a - 2 * b + d);
    const double off = (a - d) / (den != 0 && den == den ? den : 1);
    return (bj + std::max(-0.5, std::min(0.5, off))) * bin;
}

// ============================================================================
//  Limiter (retune mute, fade-in and headphone limiter)
// ============================================================================
// After a retune the demodulators' level control is still set for the
// previous signal, so the first moments of a stronger station came out up
// to ~58 dB too loud: the audio is muted for a moment, then faded back in
// over 100 ms.  A limiter catches anything left (SSB gain overshoot,
// lightning crashes): look-ahead (each block is in hand before it plays, so
// the gain eases down ~2 ms ahead of a peak and recovers over ~0.3 s: no
// steps in gain, so no clicks or pumping ticks) and volume-aware (it only
// acts if what leaves the speakers would reach full scale).
Limiter::Limiter(double sampleRate)
    : rate_(48000), look_(96), atk_(0), rel_(0), g_(1), active_(false), muteLeft_(0), fade_(4800), fadeN_(4800) {
    setSampleRate(sampleRate);
}

void Limiter::setSampleRate(double rate) {
    rate_ = rate > 0 ? rate : 48000;
    look_ = std::max(1, static_cast<int>(jsRound(96 * rate_ / 48000)));  // (JS LIM_LOOK = 96 at 48 kHz)
    atk_ = static_cast<float>(1 - std::exp(-5.0 / look_));
    rel_ = static_cast<float>(1 - std::exp(-1 / (0.3 * rate_)));
    fadeN_ = static_cast<long>(rate_ / 10);
    reset();
}

void Limiter::reset() {
    g_ = 1;
    active_ = false;
    muteLeft_ = 0;
    fade_ = fadeN_;
}

void Limiter::process(float* L, float* R, std::size_t n, float ceil) {
    const bool same = R == 0 || R == L;
    float pk = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const float a = std::fabs(L[i]);
        if (a > pk) pk = a;
        if (!same) {
            const float b = std::fabs(R[i]);
            if (b > pk) pk = b;
        }
    }
    if (pk <= ceil && g_ >= 0.9999f) {
        g_ = 1;
        active_ = false;
        return;
    }
    if (need_.size() < n) {
        need_.resize(n);
        m_.resize(n);
    }
    float *need = &need_[0], *m = &m_[0];
    for (std::size_t i = 0; i < n; ++i) {
        float a = std::fabs(L[i]);
        if (!same) a = std::max(a, std::fabs(R[i]));
        need[i] = a > ceil ? ceil / a : 1;
    }
    for (std::size_t i = n; i-- > 0;) {
        float v = need[i];
        const std::size_t e = std::min(n, i + look_);
        for (std::size_t j = i + 1; j < e; ++j)
            if (need[j] < v) v = need[j];
        m[i] = v;
    }
    float g = g_;
    for (std::size_t i = 0; i < n; ++i) {
        const float t = m[i];
        g = t < g ? g + (t - g) * atk_ : g + (1 - g) * rel_;
        if (g > need[i]) g = need[i];  // safety: never past the ceiling
        L[i] *= g;
        if (!same) R[i] *= g;
    }
    g_ = g;
    active_ = g < 0.999f;
}

void Limiter::armRetune(double secs) {
    if (!(secs > 0)) secs = 0;
    muteLeft_ = static_cast<long>(jsRound(secs * rate_));
    fade_ = secs > 0 ? 0 : fadeN_;
}

void Limiter::gate(float* L, float* R, std::size_t n) {
    if (muteLeft_ <= 0 && fade_ >= fadeN_) return;
    const bool same = R == 0 || R == L;
    for (std::size_t i = 0; i < n; ++i) {
        float g;
        if (muteLeft_ > 0) {
            muteLeft_--;
            g = 0;
        } else if (fade_ < fadeN_) {
            g = static_cast<float>(fade_) / fadeN_;
            fade_++;
        } else {
            g = 1;
        }
        L[i] *= g;
        if (!same) R[i] *= g;
    }
}

}  // namespace dsp
