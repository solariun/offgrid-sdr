// ============================================================================
//  offgrid-dsp - liboffgrid-dsp, the signal processing library of OFFgrid-SDR TUI.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr) by diddy-boy.
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_filters.cpp
//  Layer:  Tests (library liboffgrid-dsp)
//
//  What this file does
//  -------------------
//  Self-checking test of the audio clean-up filters (AudioFilters.h) and the
//  AI voice cleaner (VoiceCleaner.h), alone and wired into dsp::Receiver.
//  Prints PASS/FAIL per check (INFO/PERF lines are measurements) and
//  returns nonzero if anything failed.  Checks:
//    - noise blanker: impulses injected into the IQ of an AM station, the
//      audio error with and without the blanker (improvement in dB), also
//      with a strong station elsewhere in the band (the channel stage);
//    - noise reduction: SNR of a tone in hiss, and noise removed in the
//      pauses of a voice-like signal while its talk spurts keep their level;
//    - hum filter: 50 Hz (and 60 Hz) and harmonics removed, a voice tone kept;
//    - auto notch and smart notch: a steady whistle over voice-like audio is
//      found (within a few Hz) and removed, the speech band is kept; mains
//      hum is recognised as hum; the smart notch list via the Receiver;
//    - limiter: peaks bounded by the ceiling, gain recovers; retune mute and
//      fade-in;
//    - voice cleaner: noise-only audio strongly reduced, voice kept much
//      better; any audio rate works; and, the strongest evidence, the C++
//      RNNoise port against the reference C code of RNNoise on the same
//      input: per-frame gains, voice probability and output samples;
//    - Receiver: defaults are all off; filters do not run in modes the JS
//      excludes (output identical); the decoders' tap is before the filters;
//    - CPU per filter.
//
//  How it fits in
//  --------------
//  Build (from the repository root):
//    c++ -std=c++11 -O2 -Isrc src/dsp/*.cpp tests/test_filters.cpp -o test_filters
//  The RNNoise reference comparison needs the reference C build of RNNoise
//  v0.1.1 (https://github.com/xiph/rnnoise, tag v0.1.1) with a small harness
//  that writes, per 480-sample frame: int32 silent, float vad, float gains[22]
//  (the network's output), float lastg[22], float out[480].  Then:
//    ./test_filters --write-rnn-input in.f32     (the test's input, int16 scale)
//    ./rnnoise_ref in.f32 ref.bin                (the reference C code)
//    ./test_filters --rnn-ref ref.bin            (all checks + the comparison)
//  Without --rnn-ref the comparison is reported as SKIP (not a failure).
//  Build the reference with the same compiler and floating-point settings
//  (e.g. both cc/c++ -O2 with the default -ffp-contract): then the two agree
//  bit for bit.  Fused multiply-add on one side only changes the last bits,
//  and the recurrent network carries them on (measured with -ffp-contract=off
//  on the C++ side only: gains differ by up to 0.005, samples by ~6 LSB), so
//  the comparison would fail.
//
//  Contributing
//  ------------
//  Keep every signal synthetic and deterministic (fixed seeds), so a failure
//  reproduces.  Thresholds are set well inside what the filters achieve, so
//  they catch breakage, not noise.
// ============================================================================
#include "../src/dsp/AudioFilters.h"
#include "../src/dsp/Receiver.h"
#include "../src/dsp/VoiceCleaner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

using namespace dsp;

namespace {

int g_failures = 0, g_checks = 0;

void check(bool ok, const std::string& name, const std::string& detail = "") {
    ++g_checks;
    if (!ok) ++g_failures;
    std::printf("%s  %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : "  -- ", detail.c_str());
}

std::string fmt(const char* f, double a, double b = 0, double c = 0, double d = 0) {
    char buf[320];
    std::snprintf(buf, sizeof buf, f, a, b, c, d);
    return buf;
}

const double kPi = 3.14159265358979323846;
const double kFs = 48000;

// Deterministic noise: a 64-bit LCG, uniform and Gaussian.
struct Rng {
    unsigned long long s;
    explicit Rng(unsigned long long seed) : s(seed) {}
    double uni() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return ((s >> 11) + 0.5) / 9007199254740992.0;
    }
    double gauss() { return std::sqrt(-2 * std::log(uni())) * std::cos(2 * kPi * uni()); }
};

double energy(const std::vector<float>& x, std::size_t from, std::size_t to) {
    double s = 0;
    for (std::size_t i = from; i < to && i < x.size(); ++i) s += static_cast<double>(x[i]) * x[i];
    return s;
}
double db(double r) { return 10 * std::log10(r > 1e-300 ? r : 1e-300); }

// Power (dB) of the f Hz component of x[from, to) (Goertzel, Hann window).
double toneDb(const std::vector<float>& x, std::size_t from, std::size_t to, double f, double fs = kFs) {
    const std::size_t N = to - from;
    const double c = 2 * std::cos(2 * kPi * f / fs);
    double s1 = 0, s2 = 0;
    for (std::size_t i = 0; i < N; ++i) {
        const double w = 0.5 - 0.5 * std::cos(2 * kPi * i / N);
        const double s0 = x[from + i] * w + c * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return db((s1 * s1 + s2 * s2 - c * s1 * s2) / (N * N) + 1e-30);
}

// Energy (dB) of x[from, to) in [lo, hi] Hz, leaving out +-skipW Hz around
// `skip` (FFT of 16384-sample Hann slices).
double bandDb(const std::vector<float>& x, std::size_t from, std::size_t to, double lo, double hi, double skip = -1,
              double skipW = 0) {
    const int N = 16384;
    std::vector<double> re(N), im(N);
    double acc = 0;
    for (std::size_t st = from; st + N <= to; st += N / 2) {
        for (int i = 0; i < N; ++i) {
            re[i] = x[st + i] * (0.5 - 0.5 * std::cos(2 * kPi * i / N));
            im[i] = 0;
        }
        // plain radix-2 FFT
        for (int i = 1, j = 0; i < N; ++i) {
            int bit = N >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) {
                std::swap(re[i], re[j]);
                std::swap(im[i], im[j]);
            }
        }
        for (int len = 2; len <= N; len <<= 1) {
            const double a = -2 * kPi / len;
            for (int i = 0; i < N; i += len)
                for (int k = 0; k < len / 2; ++k) {
                    const double wr = std::cos(a * k), wi = std::sin(a * k);
                    const int p = i + k, q = p + len / 2;
                    const double tr = re[q] * wr - im[q] * wi, ti = re[q] * wi + im[q] * wr;
                    re[q] = re[p] - tr;
                    im[q] = im[p] - ti;
                    re[p] += tr;
                    im[p] += ti;
                }
        }
        for (int k = 0; k < N / 2; ++k) {
            const double f = k * kFs / N;
            if (f < lo || f > hi) continue;
            if (skip > 0 && std::fabs(f - skip) <= skipW) continue;
            acc += re[k] * re[k] + im[k] * im[k];
        }
    }
    return db(acc + 1e-30);
}

// A voice-like signal at 48 kHz: talk spurts of syllables (a gliding pitch,
// glottal pulses through three formant resonators) with pauses.  `talk`
// marks the samples inside a syllable.
std::vector<float> voiceLike(std::size_t n, unsigned long long seed, std::vector<char>* talk = 0) {
    Rng r(seed);
    std::vector<float> x(n, 0.0f);
    if (talk) talk->assign(n, 0);
    const double vowels[5][3] = {{730, 1090, 2440}, {530, 1840, 2480}, {270, 2290, 3010}, {570, 840, 2410}, {440, 1020, 2240}};
    double ph = 0, y[3][2] = {{0, 0}, {0, 0}, {0, 0}};
    std::size_t i = 0;
    int syl = 0;
    while (i < n) {
        const std::size_t len = static_cast<std::size_t>((0.16 + 0.12 * r.uni()) * kFs);
        const bool pause = (syl % 5) == 4;
        const std::size_t gap = static_cast<std::size_t>((pause ? 0.35 : 0.05) * kFs);
        const double* F = vowels[static_cast<int>(r.uni() * 5) % 5];
        const double f0a = 110 + 50 * r.uni(), f0b = f0a * (0.8 + 0.1 * r.uni());
        double coef[3][2], gain[3];
        for (int k = 0; k < 3; ++k) {
            const double bw = 80 + 40 * k, rr = std::exp(-kPi * bw / kFs);
            coef[k][0] = 2 * rr * std::cos(2 * kPi * F[k] / kFs);
            coef[k][1] = -rr * rr;
            gain[k] = (1 - rr) * (k == 0 ? 1.0 : (k == 1 ? 0.6 : 0.3));
        }
        for (std::size_t j = 0; j < len && i < n; ++j, ++i) {
            const double t = static_cast<double>(j) / len, f0 = f0a + (f0b - f0a) * t;
            const double env = std::sin(kPi * t);
            const double prev = ph;
            ph += f0 / kFs;
            const double pulse = std::floor(ph) != std::floor(prev) ? 1.0 : 0.0;
            const double e = pulse - 0.002;  // (glottal pulse, no DC)
            double out = 0;
            for (int k = 0; k < 3; ++k) {
                const double v = e + coef[k][0] * y[k][0] + coef[k][1] * y[k][1];
                y[k][1] = y[k][0];
                y[k][0] = v;
                out += gain[k] * v;
            }
            x[i] = static_cast<float>(0.25 * env * out);
            if (talk && env > 0.3) (*talk)[i] = 1;
        }
        for (std::size_t j = 0; j < gap && i < n; ++j, ++i) x[i] = 0;
        ++syl;
    }
    // normalise: talk-spurt rms 0.1
    double s = 0, c = 0;
    for (std::size_t k = 0; k < n; ++k)
        if (x[k] != 0) {
            s += x[k] * x[k];
            c++;
        }
    const float g = static_cast<float>(0.1 / std::sqrt(s / std::max(1.0, c)));
    for (std::size_t k = 0; k < n; ++k) x[k] *= g;
    return x;
}

void addNoise(std::vector<float>& x, double sigma, unsigned long long seed) {
    Rng r(seed);
    for (std::size_t i = 0; i < x.size(); ++i) x[i] += static_cast<float>(sigma * r.gauss());
}

// The lag (0..maxLag) at which b best matches a (b is a delayed).
int bestLag(const std::vector<float>& a, const std::vector<float>& b, int maxLag, std::size_t from, std::size_t len) {
    int best = 0;
    double bv = -1e300;
    for (int L = 0; L <= maxLag; ++L) {
        double s = 0;
        for (std::size_t i = from; i < from + len; ++i) s += a[i] * b[i + L];
        if (s > bv) {
            bv = s;
            best = L;
        }
    }
    return best;
}

double cpuSeconds() { return static_cast<double>(std::clock()) / CLOCKS_PER_SEC; }

// Audio (mono, the left channel) of a Receiver over IQ blocks.
std::vector<float> run(Receiver& rx, const std::vector<Complex>& iq, std::size_t block) {
    std::vector<float> a, out;
    for (std::size_t p = 0; p < iq.size(); p += block) {
        a.clear();
        rx.process(&iq[p], std::min(block, iq.size() - p), a);
        for (std::size_t i = 0; i < a.size(); i += 2) out.push_back(a[i]);
    }
    return out;
}

// ============================================================================
//  Noise blanker
// ============================================================================
// An AM station (1 kHz tone, 50%) at +150 kHz in light noise, optionally with
// a strong carrier elsewhere, at 2.048 MS/s; impulses: 3-sample spikes.
std::vector<Complex> amIq(std::size_t n, bool impulses, bool strongNeighbour, unsigned long long seed) {
    Rng r(seed), ri(seed + 99);
    std::vector<Complex> iq(n);
    const double fs = 2048000, off = 150000;
    for (std::size_t i = 0; i < n; ++i) {
        const double t = i / fs;
        const double env = 0.05 * (1 + 0.5 * std::sin(2 * kPi * 1000 * t));
        double re = env * std::cos(2 * kPi * off * t), im = env * std::sin(2 * kPi * off * t);
        if (strongNeighbour) {
            re += 0.5 * std::cos(2 * kPi * -420000 * t);
            im += 0.5 * std::sin(2 * kPi * -420000 * t);
        }
        re += 0.002 * r.gauss();
        im += 0.002 * r.gauss();
        iq[i] = Complex(static_cast<float>(re), static_cast<float>(im));
    }
    if (impulses) {
        // ~150 impulses a second, at random times, random phase, amplitude
        // 0.5-1 (impulse noise: much louder than anything else, very short).
        std::size_t at = 0;
        while (true) {
            at += static_cast<std::size_t>(2000 + 22000 * ri.uni());
            if (at + 3 >= n) break;
            const double a = 0.5 + 0.5 * ri.uni(), p = 2 * kPi * ri.uni();
            for (int k = 0; k < 3; ++k) iq[at + k] += Complex(static_cast<float>(a * std::cos(p + k)), static_cast<float>(a * std::sin(p + k)));
        }
    }
    return iq;
}

void testNoiseBlanker() {
    std::printf("\n== Noise blanker (impulses in the IQ of an AM station)\n");
    const std::size_t n = 2048000 * 2, block = 102400;  // 2 s in 50 ms blocks (as the JS)
    for (int strong = 0; strong < 2; ++strong) {
        const std::vector<Complex> clean = amIq(n, false, strong != 0, 7), dirty = amIq(n, true, strong != 0, 7);
        ReceiverConfig c;
        c.mode = Mode::AM;
        c.offsetHz = 150000;
        c.agc = false;
        Receiver r0, r1, r2;
        r0.configure(c);
        r1.configure(c);
        c.noiseBlanker = true;
        r2.configure(c);
        const std::vector<float> a0 = run(r0, clean, block), a1 = run(r1, dirty, block), a2 = run(r2, dirty, block);
        const std::size_t from = 24000, to = std::min(a0.size(), std::min(a1.size(), a2.size()));
        double e1 = 0, e2 = 0, s = 0;
        for (std::size_t i = from; i < to; ++i) {
            e1 += (a1[i] - a0[i]) * (a1[i] - a0[i]);
            e2 += (a2[i] - a0[i]) * (a2[i] - a0[i]);
            s += a0[i] * a0[i];
        }
        unsigned long long blanked = 0, total = 0;
        r2.noiseBlankerStats(blanked, total);
        // (The JS's own measurements on real crackle: quiet band 12.7 dB
        //  better, crowded MW 8.1 dB better.)
        const std::string what = strong ? " (with a station 20 dB stronger elsewhere in the band)" : "";
        check(db(e1 / e2) > (strong ? 2 : 10), "impulse crackle reduced by the blanker" + what,
              fmt("audio error %.1f dB -> %.1f dB below the programme: %.1f dB better", db(e1 / s), db(e2 / s), db(e1 / e2)));
        check(total == n && blanked < n / 100, "blanker touched under 1% of the samples" + what,
              fmt("%.3f%% of %.0f", 100.0 * blanked / total, static_cast<double>(total)));
        // Clean band, blanker on: (almost) no change.
        Receiver r3;
        r3.configure(c);
        const std::vector<float> a3 = run(r3, clean, block);
        double e3 = 0;
        for (std::size_t i = from; i < to; ++i) e3 += (a3[i] - a0[i]) * (a3[i] - a0[i]);
        check(db(e3 / s) < -40, "no impulses: the blanker leaves the audio alone" + what, fmt("difference %.1f dB", db(e3 / s)));
    }
    // The stages on their own.
    {
        std::vector<Complex> iq = amIq(204800, true, false, 3);
        std::vector<Complex> ref = amIq(204800, false, false, 3);
        double before = 0, after = 0;
        for (std::size_t i = 0; i < iq.size(); ++i) before += std::norm(iq[i] - ref[i]);
        WideBlanker w;
        w.setSampleRate(2048000);
        BlankerStats st;
        w.process(&iq[0], iq.size(), &st);
        for (std::size_t i = 0; i < iq.size(); ++i) after += std::norm(iq[i] - ref[i]);
        check(db(before / after) > 15, "wideband stage: impulse energy in the IQ removed", fmt("%.1f dB less", db(before / after)));
    }
}

// ============================================================================
//  Noise reduction
// ============================================================================
void testNoiseReduction() {
    std::printf("\n== Spectral noise reduction (hiss)\n");
    const std::size_t n = static_cast<std::size_t>(kFs * 8);
    // SNR (from the error against the clean signal) of a keyed tone and of a
    // voice-like signal in white hiss.  (A tone that never stops is not a
    // fair case: the noise floor is a minimum statistic, so to this filter a
    // steady tone IS noise, as the JS intends: steady whistles are noise.)
    for (int sig = 0; sig < 2; ++sig) {
        std::vector<float> s;
        if (sig == 0) {
            s.assign(n, 0.0f);
            for (std::size_t i = 0; i < n; ++i) {  // 800 Hz, 200 ms on / 150 ms off, 5 ms ramps
                const double t = std::fmod(static_cast<double>(i) / kFs, 0.35);
                const double env = t < 0.2 ? std::min(1.0, std::min(t, 0.2 - t) / 0.005) : 0.0;
                s[i] = static_cast<float>(0.1 * env * std::sin(2 * kPi * 800 * i / kFs));
            }
        } else {
            s = voiceLike(n, 9);
        }
        std::vector<float> x = s;
        addNoise(x, sig == 0 ? 0.05 : 0.03, 11);
        for (int level = 1; level <= 3; ++level) {
            std::vector<float> y = x;
            SpectralNR nr;
            for (std::size_t p = 0; p < n; p += 2400) nr.process(&y[p], std::min<std::size_t>(2400, n - p), level);
            const std::size_t from = static_cast<std::size_t>(kFs * 3), len = static_cast<std::size_t>(kFs * 4);
            const int lag = bestLag(x, y, 1024, from, 48000);  // (on the noisy input: broadband, one clear peak)
            double es = 0, eIn = 0, eOut = 0;
            for (std::size_t i = from; i < from + len; ++i) {
                es += s[i] * s[i];
                eIn += (x[i] - s[i]) * (x[i] - s[i]);
                eOut += (y[i + lag] - s[i]) * (y[i + lag] - s[i]);
            }
            const double gainDb = db(es / eOut) - db(es / eIn);
            check(gainDb > 2 + level, fmt(sig == 0 ? "level %.0f: keyed tone in hiss, SNR improved" : "level %.0f: voice-like signal in hiss, SNR improved", level),
                  fmt("SNR %.1f dB -> %.1f dB: %+.1f dB (delay %.0f samples)", db(es / eIn), db(es / eOut), gainDb, lag));
        }
    }
    // A voice-like signal: the hiss in the pauses goes, the talk keeps its level.
    {
        std::vector<char> talk;
        const std::vector<float> s = voiceLike(n, 5, &talk);
        std::vector<float> x = s;
        addNoise(x, 0.02, 12);
        std::vector<float> y = x;
        SpectralNR nr;
        for (std::size_t p = 0; p < n; p += 2400) nr.process(&y[p], std::min<std::size_t>(2400, n - p), 2);
        const int lag = bestLag(x, y, 1024, static_cast<std::size_t>(kFs * 3), 96000);
        double pin = 0, pout = 0, tin = 0, tout = 0;
        std::size_t np = 0;
        for (std::size_t i = static_cast<std::size_t>(kFs * 3); i + lag < n; ++i) {
            if (s[i] == 0) {  // a pause (noise only)
                pin += x[i] * x[i];
                pout += y[i + lag] * y[i + lag];
                np++;
            } else if (talk[i]) {
                tin += s[i] * s[i];
                tout += y[i + lag] * y[i + lag];
            }
        }
        check(np > 10000 && db(pin / pout) > 6, "voice-like: hiss in the pauses reduced", fmt("%.1f dB less", db(pin / pout)));
        check(std::fabs(db(tout / tin)) < 3, "voice-like: talk spurts keep their level", fmt("%+.1f dB", db(tout / tin)));
    }
    // A gap (exact zeros) must not wreck the noise floor (the JS's 2.3 fix).
    {
        std::vector<float> x(n, 0.0f);
        addNoise(x, 0.02, 13);
        for (std::size_t i = static_cast<std::size_t>(kFs * 3); i < static_cast<std::size_t>(kFs * 3.05); ++i) x[i] = 0;
        std::vector<float> y = x;
        SpectralNR nr;
        for (std::size_t p = 0; p < n; p += 2400) nr.process(&y[p], std::min<std::size_t>(2400, n - p), 2);
        const std::size_t a = static_cast<std::size_t>(kFs * 2), b = static_cast<std::size_t>(kFs * 4);
        const double before = db(energy(x, a - 48000, a) / energy(y, a - 48000, a)), after = db(energy(x, b, b + 48000) / energy(y, b, b + 48000));
        check(after > before - 2, "a 50 ms gap of zeros does not undo the reduction",
              fmt("noise reduction %.1f dB before the gap, %.1f dB 1 s after", before, after));
    }
}

// ============================================================================
//  Hum filter
// ============================================================================
void testHum() {
    std::printf("\n== Hum filter (mains hum and harmonics)\n");
    const std::size_t n = static_cast<std::size_t>(kFs * 4);
    const int mains[2] = {50, 60};
    for (int m = 0; m < 2; ++m) {
        const double f0 = mains[m];
        std::vector<float> x(n);
        for (std::size_t i = 0; i < n; ++i) {
            double v = 0.3 * std::sin(2 * kPi * 1000 * i / kFs);  // a voice tone
            for (int k = 1; k <= 8; ++k) v += 0.05 / k * std::sin(2 * kPi * k * f0 * i / kFs + k);
            x[i] = static_cast<float>(v);
        }
        std::vector<float> y = x;
        HumFilter h;
        h.configure(f0, kFs);
        for (std::size_t p = 0; p < n; p += 2400) h.process(&y[p], std::min<std::size_t>(2400, n - p));
        const std::size_t a = static_cast<std::size_t>(kFs * 2), b = n;
        double worst = 1e9;
        for (int k = 1; k <= 8; ++k) worst = std::min(worst, toneDb(x, a, b, k * f0) - toneDb(y, a, b, k * f0));
        const double keep = toneDb(y, a, b, 1000) - toneDb(x, a, b, 1000);
        check(worst > 30, fmt("%.0f Hz hum: every harmonic 1-8 removed", f0), fmt("weakest reduction %.1f dB", worst));
        check(std::fabs(keep) < 0.2, fmt("%.0f Hz hum filter keeps a 1 kHz voice tone", f0), fmt("%+.2f dB", keep));
        // A male voice's fundamental between the notches passes (unlike a low-cut).
        std::vector<float> v(n), w;
        for (std::size_t i = 0; i < n; ++i) v[i] = static_cast<float>(0.3 * std::sin(2 * kPi * 1.5 * f0 * i / kFs));
        w = v;
        HumFilter h2;
        h2.configure(f0, kFs);
        h2.process(&w[0], n);
        const double low = toneDb(w, a, b, 1.5 * f0) - toneDb(v, a, b, 1.5 * f0);
        check(std::fabs(low) < 1, fmt("%.0f Hz hum filter passes a %.0f Hz voice fundamental", f0, 1.5 * f0), fmt("%+.2f dB", low));
    }
}

// ============================================================================
//  Auto notch and smart notch
// ============================================================================
std::vector<float> whistleOverVoice(std::size_t n, double fw, unsigned long long seed) {
    std::vector<float> x = voiceLike(n, seed);
    addNoise(x, 0.003, seed + 1);
    for (std::size_t i = 0; i < n; ++i) x[i] += static_cast<float>(0.05 * std::sin(2 * kPi * fw * i / kFs));
    return x;
}

void testNotches() {
    std::printf("\n== Auto notch and smart notch (a steady whistle over voice-like audio)\n");
    const std::size_t n = static_cast<std::size_t>(kFs * 6);
    const double fw = 1234.5;
    const std::vector<float> x = whistleOverVoice(n, fw, 21);
    const std::size_t a = static_cast<std::size_t>(kFs * 4), b = n;
    // Auto notch.
    {
        std::vector<float> y = x;
        AutoNotch an;
        for (std::size_t p = 0; p < n; p += 2400) an.process(&y[p], std::min<std::size_t>(2400, n - p), kFs);
        const std::vector<double> f = an.notches();
        double best = 1e9;
        for (std::size_t i = 0; i < f.size(); ++i) best = std::min(best, std::fabs(f[i] - fw));
        check(f.size() >= 1 && best < 3, "auto notch: whistle detected", fmt("%.0f notch(es), nearest %.2f Hz from %.1f Hz", f.size(), best, fw));
        const double cut = toneDb(x, a, b, fw) - toneDb(y, a, b, fw);
        const double speech = bandDb(y, a, b, 300, 3000, fw, 60) - bandDb(x, a, b, 300, 3000, fw, 60);
        check(cut > 20, "auto notch: whistle removed", fmt("%.1f dB down", cut));
        check(std::fabs(speech) < 1, "auto notch: speech band (300-3000 Hz, whistle excluded) kept", fmt("%+.2f dB", speech));
    }
    // Voice alone: the auto notch should leave it alone (no persistent peaks).
    {
        std::vector<float> v = voiceLike(n, 23);
        addNoise(v, 0.003, 24);
        AutoNotch an;
        int most = 0;
        for (std::size_t p = 0; p < n; p += 2400) {
            an.process(&v[p], std::min<std::size_t>(2400, n - p), kFs);
            most = std::max(most, static_cast<int>(an.notches().size()));
        }
        check(most <= 1, "auto notch: voice alone gets (almost) no notches", fmt("at most %.0f at once", most));
    }
    // Smart notch on its own.
    {
        SmartNotch sn(kFs);
        std::vector<float> y = x;
        const std::size_t half = static_cast<std::size_t>(kFs * 3.5);
        for (std::size_t p = 0; p < half; p += 2400) sn.process(&y[p], std::min<std::size_t>(2400, half - p), kFs);
        std::vector<float> rec;
        sn.recent(static_cast<std::size_t>(kFs * 3), rec);
        int hum = -1;
        const std::vector<SmartNotch::Whine> got = SmartNotch::find(rec, kFs, 0, &hum);
        check(!got.empty() && std::fabs(got[0].f - fw) < 3, "smart notch: find() reports the whine",
              got.empty() ? std::string("nothing found") : fmt("%.2f Hz (%.1f dB proud), %.0f found", got[0].f, got[0].prom, got.size()));
        if (!got.empty()) sn.add(got[0].f);
        for (std::size_t p = half; p < n; p += 2400) sn.process(&y[p], std::min<std::size_t>(2400, n - p), kFs);
        const double cut = toneDb(x, a, b, fw) - toneDb(y, a, b, fw);
        const double speech = bandDb(y, a, b, 300, 3000, fw, 60) - bandDb(x, a, b, 300, 3000, fw, 60);
        check(cut > 20, "smart notch: whine removed", fmt("%.1f dB down", cut));
        check(std::fabs(speech) < 1, "smart notch: speech band (whine excluded) kept", fmt("%+.2f dB", speech));
        // The list: move (clamped), add up to 3, remove.
        sn.move(0, 20000);
        bool ok = sn.notches().size() == 1 && sn.notches()[0] == 4500;
        sn.add(700);
        sn.add(900);
        sn.add(1100);  // the oldest (4500) goes
        ok = ok && sn.notches().size() == 3 && sn.notches()[0] == 700 && sn.notches()[2] == 1100;
        sn.remove(1);
        ok = ok && sn.notches().size() == 2 && sn.notches()[1] == 1100;
        sn.clear();
        ok = ok && sn.notches().empty();
        check(ok, "smart notch list: add (3 at most, oldest out), move (clamped 80-4500 Hz), remove, clear");
    }
    // Mains hum is not a whine.
    {
        std::vector<float> h(static_cast<std::size_t>(kFs * 3));
        for (std::size_t i = 0; i < h.size(); ++i) {
            double v = 0;
            for (int k = 1; k <= 6; ++k) v += 0.05 / k * std::sin(2 * kPi * k * 50 * i / kFs + k);
            h[i] = static_cast<float>(v);
        }
        addNoise(h, 0.001, 31);
        int hum = 0;
        const std::vector<SmartNotch::Whine> got = SmartNotch::find(h, kFs, 0, &hum);
        check(got.empty() && hum == 50, "smart notch: 50 Hz mains hum reported as hum, not as a whine",
              fmt("%.0f whines, hum %.0f Hz", got.size(), hum));
    }
}

// The smart and auto notches inside the Receiver: a USB voice with a whistle.
void testNotchReceiver() {
    std::printf("\n== Smart notch and auto notch in the Receiver (USB voice + whistle, simulated band)\n");
    const double fw = 1500;
    std::vector<float> audio = whistleOverVoice(static_cast<std::size_t>(kFs * 4), fw, 41);
    DemoSource src(2048000, 7000000);
    src.addAudioStation(7000000 + 150000, audio, kFs, -20, Mode::USB, "whistle");
    ReceiverConfig c;
    c.mode = Mode::USB;
    c.offsetHz = 150000;
    c.smartNotch = true;
    Receiver rx;
    rx.configure(c);
    std::vector<std::uint8_t> bytes;
    std::vector<Complex> iq;
    std::vector<float> out;
    for (int k = 0; k < 80; ++k) {  // 4 s
        src.generate(102400, bytes);
        convertU8(&bytes[0], bytes.size(), iq);
        out.clear();
        rx.process(&iq[0], iq.size(), out);
    }
    int hum = 0;
    const double f = rx.smartNotchFind(&hum);
    check(std::fabs(f - fw) < 4, "Receiver::smartNotchFind() notches the whistle", fmt("%.2f Hz (whistle %.0f Hz)", f, fw));
    const double again = rx.smartNotchFind();
    check(rx.smartNotches().size() == 1 && std::fabs(again - f) < 1e-9, "finding it again reuses the existing notch",
          fmt("%.0f notch(es)", rx.smartNotches().size()));
    const int i = rx.smartNotchAdd(2500, true);
    rx.smartNotchMove(i, 2600);
    check(rx.smartNotches().size() == 2 && rx.smartNotches()[1] == 2600, "smartNotchAdd / smartNotchMove");
    rx.smartNotchRemove(1);
    std::vector<float> strip;
    const double bin = rx.smartNotchSpectrum(strip, 3000);
    check(rx.smartNotches().size() == 1 && bin > 11 && bin < 12 && strip.size() > 250, "smartNotchRemove, smartNotchSpectrum (the strip)",
          fmt("bin %.2f Hz, %.0f bins", bin, strip.size()));
    c.offsetHz += 2500;  // another station: cleared (as the JS)
    rx.configure(c);
    src.generate(102400, bytes);
    convertU8(&bytes[0], bytes.size(), iq);
    rx.process(&iq[0], iq.size(), out);
    check(rx.smartNotches().empty(), "smart notches cleared when the station changes (> 2 kHz)");
    // Auto notch in the Receiver.
    c.offsetHz = 150000;
    c.smartNotch = false;
    c.autoNotch = true;
    rx.configure(c);
    for (int k = 0; k < 60; ++k) {
        src.generate(102400, bytes);
        convertU8(&bytes[0], bytes.size(), iq);
        out.clear();
        rx.process(&iq[0], iq.size(), out);
    }
    const std::vector<double> an = rx.autoNotches();
    double best = 1e9;
    for (std::size_t k = 0; k < an.size(); ++k) best = std::min(best, std::fabs(an[k] - fw));
    check(best < 4, "Receiver::autoNotches() follows the whistle", fmt("nearest %.2f Hz off", best));
}

// ============================================================================
//  Limiter, retune mute
// ============================================================================
void testLimiter() {
    std::printf("\n== Limiter and retune mute\n");
    const std::size_t n = static_cast<std::size_t>(kFs * 3);
    std::vector<float> x(n);
    for (std::size_t i = 0; i < n; ++i) {
        double v = 0.3 * std::sin(2 * kPi * 440 * i / kFs);
        if (i >= 48000 && i < 52800) v *= 8;  // a 100 ms burst 8x too loud (lightning crash)
        x[i] = static_cast<float>(v);
    }
    std::vector<float> y = x;
    Limiter lim(kFs);
    for (std::size_t p = 0; p < n; p += 2400) lim.process(&y[p], 0, std::min<std::size_t>(2400, n - p), 0.95f);
    float pk = 0;
    for (std::size_t i = 0; i < n; ++i) pk = std::max(pk, std::fabs(y[i]));
    check(pk <= 0.95f + 1e-6f, "limiter: peaks never pass the ceiling (0.95)", fmt("peak %.4f (input peak %.2f)", pk, 2.4));
    const double before = db(energy(y, 0, 48000) / energy(x, 0, 48000)), late = db(energy(y, 120000, 144000) / energy(x, 120000, 144000));
    check(std::fabs(before) < 0.01 && std::fabs(late) < 0.5, "limiter: untouched below the ceiling, recovers after the burst",
          fmt("%+.3f dB before, %+.2f dB 1.4 s after", before, late));
    // Retune: 0.25 s of silence, then a 100 ms fade-in.
    std::vector<float> g(24000, 1.0f);
    Limiter gate(kFs);
    gate.armRetune(0.25);
    gate.gate(&g[0], 0, g.size());
    const bool muted = std::fabs(g[0]) == 0 && g[11999] == 0, fading = g[12000 + 2400] > 0.49f && g[12000 + 2400] < 0.51f,
               done = g[12000 + 4800] == 1.0f;
    check(muted && fading && done, "retune mute 0.25 s then 100 ms fade-in", fmt("half-way gain %.3f", g[14400]));
}

// ============================================================================
//  AI voice cleaner (RNNoise)
// ============================================================================
// Audio from the simulated band: the demodulated "USB voice" station, or
// (voice = false) an empty USB channel: band noise through the receiver
// (what the cleaner meets on the air), at `rate`.
std::vector<float> demoVoice(std::size_t frames48, bool voice = true, int rate = 48000) {
    DemoSource src(2048000, 7000000);
    double off = 150000;  // (empty)
    for (std::size_t i = 0; voice && i < src.stations().size(); ++i)
        if (src.stations()[i].mode == Mode::USB) {
            off = src.stations()[i].offsetHz;
            break;
        }
    ReceiverConfig c;
    c.mode = Mode::USB;
    c.offsetHz = off;
    Receiver rx(rate);
    rx.configure(c);
    std::vector<std::uint8_t> bytes;
    std::vector<Complex> iq;
    std::vector<float> a, out;
    while (out.size() < frames48) {
        src.generate(102400, bytes);
        convertU8(&bytes[0], bytes.size(), iq);
        a.clear();
        rx.process(&iq[0], iq.size(), a);
        for (std::size_t i = 0; i < a.size(); i += 2) out.push_back(a[i]);
    }
    out.resize(frames48);
    return out;
}

// The input of the RNNoise comparison (int16 scale): voice-like audio in
// noise, an exact-zero gap (the silent-frame path), noise alone, the
// simulated band's USB voice, a loud tone.
std::vector<float> rnnInput() {
    const std::size_t s = 48000;
    std::vector<float> x;
    std::vector<float> v = voiceLike(3 * s, 51);
    addNoise(v, 0.01, 52);
    x.insert(x.end(), v.begin(), v.end());
    x.insert(x.end(), s / 2, 0.0f);
    std::vector<float> nz(2 * s, 0.0f);
    addNoise(nz, 0.02, 53);
    x.insert(x.end(), nz.begin(), nz.end());
    std::vector<float> d = demoVoice(3 * s);
    x.insert(x.end(), d.begin(), d.end());
    for (std::size_t i = 0; i < s; ++i) x.push_back(static_cast<float>(0.5 * std::sin(2 * kPi * 440 * i / kFs)));
    x.resize(x.size() / 480 * 480);
    for (std::size_t i = 0; i < x.size(); ++i) x[i] *= 32768;
    return x;
}

void testVoiceCleaner(const char* refPath) {
    std::printf("\n== AI voice cleaner (RNNoise port)\n");
    const std::size_t n = static_cast<std::size_t>(kFs * 6);
    // Noise only vs voice: at Full strength.  (The noise is band noise from
    // an empty USB channel.  RNNoise removes that almost completely, but not
    // synthetic full-band white noise, which it barely touches (about 3 dB,
    // the reference C code alike): it was trained on real-world noise.)
    {
        const std::vector<float> nz = demoVoice(n, false);
        std::vector<float> y = nz;
        VoiceCleaner vc;
        vc.setMix(VoiceCleaner::kFull);
        for (std::size_t p = 0; p < n; p += 2400) vc.process(&y[p], std::min<std::size_t>(2400, n - p));
        const double noiseCut = db(energy(nz, 96000, n) / energy(y, 96000, n));
        const std::vector<float> v = demoVoice(n);
        std::vector<float> w = v;
        VoiceCleaner vc2;
        vc2.setMix(VoiceCleaner::kFull);
        double vadSum = 0;
        int vadN = 0;
        for (std::size_t p = 0; p < n; p += 2400) {
            vc2.process(&w[p], std::min<std::size_t>(2400, n - p));
            vadSum += vc2.voiceProbability();
            vadN++;
        }
        const double voiceLoss = db(energy(v, 96000, n - 480) / energy(w, 96000 + 480, n));
        check(noiseCut > 15, "noise alone: strongly reduced", fmt("%.1f dB less", noiseCut));
        check(voiceLoss < noiseCut - 10, "voice (simulated band, USB) kept far better than noise",
              fmt("voice %.1f dB less, noise %.1f dB less; mean voice probability %.2f", voiceLoss, noiseCut, vadSum / vadN));
        // Medium: 25% of the original is kept, so noise is cut by at most ~12 dB.
        std::vector<float> m = nz;
        VoiceCleaner vc3;
        for (std::size_t p = 0; p < n; p += 2400) vc3.process(&m[p], std::min<std::size_t>(2400, n - p));
        const double medCut = db(energy(nz, 96000, n) / energy(m, 96000, n));
        check(medCut > 8 && medCut < 13, "Medium (default) strength keeps a quarter of the original",
              fmt("noise %.1f dB less", medCut));
    }
    // Other audio rates (Addition to the JS: resampled to 48 kHz and back).
    {
        const double rate = 24000;
        const std::size_t m = static_cast<std::size_t>(rate * 6);
        const std::vector<float> nz = demoVoice(m, false, static_cast<int>(rate));
        std::vector<float> y = nz;
        VoiceCleaner vc(rate);
        vc.setMix(VoiceCleaner::kFull);
        for (std::size_t p = 0; p < m; p += 1000) vc.process(&y[p], std::min<std::size_t>(1000, m - p));
        bool finite = true;
        for (std::size_t i = 0; i < m; ++i) finite = finite && std::fabs(y[i]) < 10;
        const double cut = db(energy(nz, m / 3, m) / energy(y, m / 3, m));
        check(finite && cut > 10, "24 kHz audio (resampled internally): noise reduced", fmt("%.1f dB less", cut));
    }
    // The port against the reference C code of RNNoise.
    if (!refPath) {
        std::printf("SKIP  RNNoise reference comparison (no --rnn-ref file given)\n");
        return;
    }
    const std::vector<float> in = rnnInput();
    FILE* f = std::fopen(refPath, "rb");
    if (!f) {
        check(false, "RNNoise reference file readable", refPath);
        return;
    }
    rnnoise::Denoiser d;
    const int frames = static_cast<int>(in.size() / 480);
    int got = 0, silentMismatch = 0, exactFrames = 0, silentFrames = 0;
    double maxG = 0, maxV = 0, maxO = 0, peak = 0;
    float out[480];
    for (int k = 0; k < frames; ++k) {
        int rs;
        float rv, rg[22], rl[22], ro[480];
        if (std::fread(&rs, 4, 1, f) != 1 || std::fread(&rv, 4, 1, f) != 1 || std::fread(rg, 4, 22, f) != 22 ||
            std::fread(rl, 4, 22, f) != 22 || std::fread(ro, 4, 480, f) != 480)
            break;
        const float v = d.processFrame(out, &in[k * 480]);
        const float* g = d.lastGains();
        if ((rs != 0) != d.lastSilent()) silentMismatch++;
        if (rs) silentFrames++;
        bool exact = v == rv;
        maxV = std::max(maxV, static_cast<double>(std::fabs(v - rv)));
        for (int b = 0; b < 22; ++b) {
            maxG = std::max(maxG, static_cast<double>(std::fabs(g[b] - rg[b])));
            exact = exact && g[b] == rg[b];
        }
        for (int i = 0; i < 480; ++i) {
            maxO = std::max(maxO, static_cast<double>(std::fabs(out[i] - ro[i])));
            peak = std::max(peak, static_cast<double>(std::fabs(ro[i])));
            exact = exact && out[i] == ro[i];
        }
        exactFrames += exact;
        got++;
    }
    std::fclose(f);
    check(got == frames, "reference file has every frame", fmt("%.0f of %.0f", got, frames));
    check(silentMismatch == 0, "same silent frames as the reference", fmt("%.0f silent frames", silentFrames));
    check(maxG < 1e-5 && maxV < 1e-5, "per-frame band gains and voice probability equal to the reference C code",
          fmt("max gain difference %.2g, max VAD difference %.2g over %.0f frames", maxG, maxV, got));
    check(maxO < 1e-3 * std::max(1.0, peak / 32768), "output samples equal to the reference",
          fmt("max difference %.3g (int16 scale, peak %.0f); %.0f frames bit-identical", maxO, peak, exactFrames));
}

// ============================================================================
//  Receiver integration
// ============================================================================
void testReceiver() {
    std::printf("\n== Receiver integration\n");
    const ReceiverConfig def;
    check(!def.noiseBlanker && !def.noiseReduction && def.humFilterHz == 0 && !def.autoNotch && !def.smartNotch &&
              !def.voiceCleaner && !def.limiter && def.retuneMuteSeconds == 0,
          "ReceiverConfig: every filter off by default");
    // In WFM and NFM (where the JS runs none of them) switching every filter
    // on changes nothing (except the limiter/retune mute, left off here).
    const Mode modes[2] = {Mode::WFM, Mode::NFM};
    for (int m = 0; m < 2; ++m) {
        DemoSource s1(2048000, m == 0 ? 100000000 : 7000000), s2(2048000, m == 0 ? 100000000 : 7000000);
        double off = 0;
        for (std::size_t i = 0; i < s1.stations().size(); ++i)
            if (s1.stations()[i].mode == modes[m]) {
                off = s1.stations()[i].offsetHz;
                break;
            }
        ReceiverConfig a;
        a.mode = modes[m];
        a.offsetHz = off;
        ReceiverConfig b = a;
        b.noiseBlanker = b.noiseReduction = b.autoNotch = b.smartNotch = b.voiceCleaner = true;
        b.humFilterHz = 50;
        Receiver ra, rb;
        ra.configure(a);
        rb.configure(b);
        std::vector<std::uint8_t> by1, by2;
        std::vector<Complex> i1, i2;
        std::vector<float> o1, o2;
        bool same = true;
        for (int k = 0; k < 20; ++k) {
            s1.generate(51200, by1);
            s2.generate(51200, by2);
            convertU8(&by1[0], by1.size(), i1);
            convertU8(&by2[0], by2.size(), i2);
            o1.clear();
            o2.clear();
            ra.process(&i1[0], i1.size(), o1);
            rb.process(&i2[0], i2.size(), o2);
            same = same && o1 == o2;
        }
        check(same, std::string(modeName(modes[m])) + ": the filters do not run (output bit-identical to filters off)");
    }
    // AM with every filter on: runs, stays finite, the decoders' tap is unchanged.
    {
        DemoSource s1(2048000, 7000000), s2(2048000, 7000000);
        double off = 0;
        for (std::size_t i = 0; i < s1.stations().size(); ++i)
            if (s1.stations()[i].mode == Mode::AM) {
                off = s1.stations()[i].offsetHz;
                break;
            }
        ReceiverConfig a;
        a.mode = Mode::AM;
        a.offsetHz = off;
        ReceiverConfig b = a;
        b.noiseReduction = b.autoNotch = b.smartNotch = b.voiceCleaner = b.limiter = true;
        b.humFilterHz = 60;
        Receiver ra, rb;
        ra.configure(a);
        rb.configure(b);
        std::vector<std::uint8_t> by1, by2;
        std::vector<Complex> i1, i2;
        std::vector<float> o1, o2;
        bool tapSame = true, finite = true, differs = false;
        for (int k = 0; k < 40; ++k) {
            s1.generate(102400, by1);
            s2.generate(102400, by2);
            convertU8(&by1[0], by1.size(), i1);
            convertU8(&by2[0], by2.size(), i2);
            o1.clear();
            o2.clear();
            ra.process(&i1[0], i1.size(), o1);
            rb.process(&i2[0], i2.size(), o2);
            tapSame = tapSame && ra.decoderAudio() == rb.decoderAudio();
            for (std::size_t i = 0; i < o2.size(); ++i) finite = finite && std::fabs(o2[i]) <= 1.0f;
            differs = differs || o1 != o2;
        }
        check(finite && differs, "AM with every audio filter on: runs, output bounded and changed");
        check(tapSame, "decoderAudio() is taken before the audio filters (as the JS)");
    }
}

// ============================================================================
//  CPU per filter
// ============================================================================
void testCpu() {
    std::printf("\n== CPU (seconds of signal per CPU second; %% of one core in real time)\n");
    const std::size_t n = static_cast<std::size_t>(kFs * 10);
    std::vector<float> base = voiceLike(n, 71);
    addNoise(base, 0.01, 72);
    for (std::size_t i = 0; i < n; ++i) base[i] += static_cast<float>(0.02 * std::sin(2 * kPi * 1234 * i / kFs));
    struct Timer {
        static void report(const char* name, double cpu, double seconds) {
            std::printf("PERF  %-28s %7.0fx real time  (%.2f%% of a core)\n", name, seconds / cpu, 100 * cpu / seconds);
        }
    };
    {
        std::vector<float> x = base;
        HumFilter h;
        h.configure(50, kFs);
        const double t0 = cpuSeconds();
        for (std::size_t p = 0; p < n; p += 2400) h.process(&x[p], 2400);
        Timer::report("hum filter (8 notches)", cpuSeconds() - t0, 10);
    }
    {
        std::vector<float> x = base;
        SpectralNR nr;
        const double t0 = cpuSeconds();
        for (std::size_t p = 0; p < n; p += 2400) nr.process(&x[p], 2400, 2);
        Timer::report("noise reduction", cpuSeconds() - t0, 10);
    }
    {
        std::vector<float> x = base;
        AutoNotch an;
        const double t0 = cpuSeconds();
        for (std::size_t p = 0; p < n; p += 2400) an.process(&x[p], 2400, kFs);
        Timer::report("auto notch", cpuSeconds() - t0, 10);
    }
    {
        std::vector<float> x = base;
        SmartNotch sn;
        sn.add(1234);
        sn.add(2000);
        sn.add(700);
        const double t0 = cpuSeconds();
        for (std::size_t p = 0; p < n; p += 2400) sn.process(&x[p], 2400, kFs);
        Timer::report("smart notch (3 notches)", cpuSeconds() - t0, 10);
        std::vector<float> rec;
        sn.recent(144000, rec);
        const double t1 = cpuSeconds();
        int hum;
        SmartNotch::find(rec, kFs, 0, &hum);
        std::printf("PERF  %-28s %7.1f ms per search (3 s of audio)\n", "smart notch find()", 1000 * (cpuSeconds() - t1));
    }
    {
        std::vector<float> x = base;
        VoiceCleaner vc;
        const double t0 = cpuSeconds();
        for (std::size_t p = 0; p < n; p += 2400) vc.process(&x[p], 2400);
        Timer::report("AI voice cleaner (RNNoise)", cpuSeconds() - t0, 10);
    }
    {
        std::vector<float> x = base;
        for (std::size_t i = 0; i < n; ++i) x[i] *= 30;  // always limiting
        Limiter lim;
        const double t0 = cpuSeconds();
        for (std::size_t p = 0; p < n; p += 2400) lim.process(&x[p], 0, 2400, 0.95f);
        Timer::report("limiter (limiting all along)", cpuSeconds() - t0, 10);
    }
    {
        std::vector<Complex> iq = amIq(2048000, true, false, 81);
        NoiseBlanker nb;
        const double t0 = cpuSeconds();
        for (int rep = 0; rep < 3; ++rep)
            for (std::size_t p = 0; p < iq.size(); p += 102400) nb.process(&iq[p], 102400, 150000);
        Timer::report("noise blanker (2.048 MS/s)", cpuSeconds() - t0, 3);
    }
}

}  // namespace

int main(int argc, char** argv) {
    const char* refPath = 0;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--write-rnn-input") == 0 && i + 1 < argc) {
            const std::vector<float> in = rnnInput();
            FILE* f = std::fopen(argv[i + 1], "wb");
            if (!f || std::fwrite(&in[0], sizeof(float), in.size(), f) != in.size()) {
                std::printf("cannot write %s\n", argv[i + 1]);
                return 1;
            }
            std::fclose(f);
            std::printf("wrote %u frames of 480 samples (float32, int16 scale) to %s\n", static_cast<unsigned>(in.size() / 480), argv[i + 1]);
            return 0;
        }
        if (std::strcmp(argv[i], "--rnn-ref") == 0 && i + 1 < argc) refPath = argv[++i];
    }
    std::printf("Audio clean-up filters and AI voice cleaner (liboffgrid-dsp)\n");
    testNoiseBlanker();
    testNoiseReduction();
    testHum();
    testNotches();
    testNotchReceiver();
    testLimiter();
    testVoiceCleaner(refPath);
    testReceiver();
    testCpu();
    std::printf("\n%d checks, %d failed: %s\n", g_checks, g_failures, g_failures ? "FAILURES" : "ALL PASS");
    return g_failures ? 1 : 0;
}
