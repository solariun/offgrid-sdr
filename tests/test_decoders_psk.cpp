// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_decoders_psk.cpp
//  Layer:  Tests (Library liboffgrid-decoders)
//
//  What this file does
//  -------------------
//  Self-checking test of the PSK and Hell decoders against the encoders in
//  PskEncoders.h.  Prints PASS/FAIL per check, returns nonzero on failure.
//    - PSK31/63/125 (PskDecoder): exact text at 48000 and 12000 Hz with
//      white noise (SNR 10 dB in 3 kHz), on 1000 Hz and 30 Hz off (AFC);
//    - PskAutoDecoder names the right speed and prints the text, and follows
//      a change of speed; tuned in mid-over, the rest of the text reads;
//    - Hell: ~17.5 columns a second, values 0..1, and the sent letters read
//      back (the strip correlated against the font, letter by letter);
//    - Hell status().signal: up in every over of a looped signal with
//      AGC-loud noise gaps, never up on noise only;
//    - a NaN/Inf sample does not poison either decoder;
//    - no text from plain noise;
//    - CPU: seconds of audio per CPU second.
//
//  How it fits in
//  --------------
//  Build (from the repository root):
//    c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors -Isrc src/dsp/Fft.cpp \
//        src/decoders/Psk*.cpp src/decoders/Hell.cpp \
//        tests/test_decoders_psk.cpp -o test_decoders_psk
//
//  Contributing
//  ------------
//  Keep it deterministic (own noise generator, fixed seeds).  Algorithms
//  under test are ports from OFFgrid-SDR (diddy-boy).
// ============================================================================
#include "decoders/Hell.h"
#include "decoders/Psk.h"
#include "decoders/PskEncoders.h"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <limits>
#include <string>
#include <vector>

namespace {

int g_failures = 0, g_checks = 0;

void check(bool ok, const std::string& name, const std::string& detail = "") {
    ++g_checks;
    if (!ok) ++g_failures;
    std::printf("%s  %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : "  -- ", detail.c_str());
}

std::string printable(const std::string& s) {
    std::string r;
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '\n') r += "\\n";
        else r += s[i];
    }
    return r;
}

// Deterministic Gaussian noise (xorshift64 + Box-Muller): same on every platform.
class Noise {
public:
    explicit Noise(unsigned long long seed) : s_(seed ? seed : 1), have_(false), spare_(0) {}
    double uniform() {
        s_ ^= s_ << 13; s_ ^= s_ >> 7; s_ ^= s_ << 17;
        return ((s_ >> 11) + 0.5) / 9007199254740992.0;
    }
    double gauss() {
        if (have_) { have_ = false; return spare_; }
        const double u = uniform(), v = uniform(), r = std::sqrt(-2 * std::log(u));
        spare_ = r * std::sin(6.283185307179586 * v);
        have_ = true;
        return r * std::cos(6.283185307179586 * v);
    }
private:
    unsigned long long s_;
    bool have_;
    double spare_;
};

// Noise standard deviation for a tone of amplitude A at snrDb in 3 kHz.
double noiseSigma(double A, double snrDb, double rate) {
    const double sig = A * A / 2, n3k = sig / std::pow(10.0, snrDb / 10);
    return std::sqrt(n3k * (rate / 2) / 3000);
}

// lead s of silence, the signal, tail s of silence; plus noise.
std::vector<float> withNoise(const std::vector<float>& sig, double rate, double lead, double tail, double sigma,
                             unsigned long long seed) {
    const std::size_t nl = static_cast<std::size_t>(lead * rate), nt = static_cast<std::size_t>(tail * rate);
    std::vector<float> a(nl + sig.size() + nt, 0.0f);
    for (std::size_t i = 0; i < sig.size(); ++i) a[nl + i] = sig[i];
    Noise g(seed);
    for (std::size_t i = 0; i < a.size(); ++i) a[i] += static_cast<float>(sigma * g.gauss());
    return a;
}

// Feeds in blocks of uneven size (as a sound card would).
std::string feed(dec::Decoder& d, const std::vector<float>& a, double rate) {
    std::string text;
    static const std::size_t sizes[] = {1024, 333, 4096, 7, 2048, 960};
    std::size_t i = 0, k = 0;
    while (i < a.size()) {
        std::size_t n = sizes[k++ % 6];
        if (n > a.size() - i) n = a.size() - i;
        d.process(&a[i], n, rate);
        d.takeText(text);
        i += n;
    }
    return text;
}

const double kSpeeds[3] = {31.25, 62.5, 125.0};
const char* const kNames[3] = {"PSK31", "PSK63", "PSK125"};
const char* const kTexts[3] = {
    "CQ CQ de OFFGRID-SDR pse k\n",
    "PSK63 is twice as fast as PSK31, 73!\n",
    "The PSK mode finds the speed by itself. 73 de OFFGRID-SDR\n",
};

void testPsk() {
    const double rates[2] = {48000, 12000};
    const double offs[2] = {0, 30};
    for (int s = 0; s < 3; ++s)
        for (int r = 0; r < 2; ++r)
            for (int o = 0; o < 2; ++o) {
                std::vector<float> sig;
                dec::synthPsk(kTexts[s], kSpeeds[s], rates[r], sig, 1000 + offs[o], 0.3);
                const std::vector<float> a =
                    withNoise(sig, rates[r], 1.0, 1.5, noiseSigma(0.3, 10, rates[r]), 1000 + s * 10 + r * 3 + o);
                char name[96];
                std::snprintf(name, sizeof name, "%s @%.0f Hz, tone %+.0f Hz, SNR 10 dB", kNames[s], rates[r], offs[o]);
                dec::PskDecoder d(kSpeeds[s]);
                const std::string got = feed(d, a, rates[r]);
                check(got == kTexts[s], name, "got \"" + printable(got) + "\"");
                if (o == 1) {
                    // AFC: where the decoder sits at the end of the over (fed without the tail)
                    dec::PskDecoder d2(kSpeeds[s]);
                    feed(d2, std::vector<float>(a.begin(), a.end() - static_cast<long>(1.5 * rates[r])), rates[r]);
                    char det[64];
                    std::snprintf(det, sizeof det, "AFC tone %.1f Hz", d2.toneHz());
                    check(std::fabs(d2.toneHz() - 1030) < 3, std::string(name) + " AFC", det);
                }

                dec::PskAutoDecoder au;
                const std::string ga = feed(au, a, rates[r]);
                const dec::Status st = au.status();
                char an[128];
                std::snprintf(an, sizeof an, "auto %s", name);
                check(st.mode == kNames[s] && ga.find(kTexts[s]) != std::string::npos, an,
                      "mode \"" + st.mode + "\" text \"" + printable(ga) + "\"");
            }
}

// AFC at the edge the JS allows (+-50 Hz) with PSK31.
void testPskAfcEdge() {
    const double offs[2] = {-50, 50};
    for (int o = 0; o < 2; ++o) {
        std::vector<float> sig;
        dec::synthPsk(kTexts[0], 31.25, 48000, sig, 1000 + offs[o], 0.3);
        const std::vector<float> a = withNoise(sig, 48000, 1.0, 1.0, noiseSigma(0.3, 20, 48000), 77 + o);
        dec::PskDecoder d(31.25);
        const std::string got = feed(d, a, 48000);
        char name[64];
        std::snprintf(name, sizeof name, "PSK31 tone %+.0f Hz (AFC edge), SNR 20 dB", offs[o]);
        check(got == kTexts[0], name, "got \"" + printable(got) + "\"");
    }
}

// Tuned in part-way through an over (no idle heard): the squelch opens on
// the clean phase, the first (partial) character is dropped, the rest reads.
void testPskMidOver() {
    const double rate = 48000;
    const std::string body = "the quick brown fox jumps over the lazy dog 0123456789\n";
    std::string bits = dec::pskBits("xxxx" + body, 0);
    bits = bits.substr(13) + std::string(48, '0');   // start inside the "x"s
    std::vector<float> sig;
    dec::synthPskBits(bits, 31.25, rate, sig, 1000, 0.3);
    const std::vector<float> a = withNoise(sig, rate, 0.5, 1.0, noiseSigma(0.3, 10, rate), 321);
    dec::PskDecoder d(31.25);
    const std::string got = feed(d, a, rate);
    check(got.size() >= 40 && body.compare(body.size() - 40, 40, got, got.size() - 40, 40) == 0,
          "PSK31 tuned in mid-over: the rest of the text", "got \"" + printable(got) + "\"");
}

// PskAutoDecoder follows a change of speed: a PSK125 over, then a PSK31 one.
void testPskAutoSwitch() {
    const double rate = 12000;
    std::vector<float> sig;
    dec::synthPsk("PSK125 over first, then PSK31\n", 125, rate, sig, 1000, 0.3);
    sig.resize(sig.size() + static_cast<std::size_t>(4 * rate), 0.0f);   // 4 s gap
    dec::synthPsk("now PSK31 at the slow speed de OFFGRID\n", 31.25, rate, sig, 1010, 0.3);
    const std::vector<float> a = withNoise(sig, rate, 0.5, 1.0, noiseSigma(0.3, 10, rate), 654);
    dec::PskAutoDecoder au;
    const std::string got = feed(au, a, rate);
    check(au.status().mode == "PSK31" && got.find("PSK125 over first, then PSK31\n") != std::string::npos &&
              got.find("now PSK31 at the slow speed de OFFGRID\n") != std::string::npos,
          "PSK auto: PSK125 then PSK31, both overs, ends on PSK31",
          "mode \"" + au.status().mode + "\" text \"" + printable(got) + "\"");
}

void testPskNaN() {
    const double rate = 48000;
    std::vector<float> sig;
    dec::synthPsk(kTexts[0], 31.25, rate, sig, 1000, 0.3);
    std::vector<float> a = withNoise(sig, rate, 1.0, 1.0, noiseSigma(0.3, 15, rate), 4242);
    a[20000] = std::numeric_limits<float>::quiet_NaN();
    a[30000] = std::numeric_limits<float>::infinity();
    dec::PskDecoder d(31.25);
    std::string got = feed(d, a, rate);
    const dec::Status st = d.status();
    check(got == kTexts[0] && st.toneHz == st.toneHz, "PSK31 NaN/Inf in the lead-in: still decodes",
          "got \"" + printable(got) + "\"");

    // NaN in the middle of the text: the text before it is kept, the decoder
    // recovers (the next over decodes).
    std::vector<float> two;
    dec::synthPsk(kTexts[0], 31.25, rate, two, 1000, 0.3);
    const std::size_t mid = two.size() / 2;
    dec::synthPsk(kTexts[0], 31.25, rate, two, 1000, 0.3);
    std::vector<float> b = withNoise(two, rate, 0.5, 1.0, noiseSigma(0.3, 15, rate), 99);
    b[static_cast<std::size_t>(0.5 * rate) + mid / 2] = std::numeric_limits<float>::quiet_NaN();
    dec::PskAutoDecoder au;
    got = feed(au, b, rate);
    check(got.find(kTexts[0]) != std::string::npos && au.status().toneHz == au.status().toneHz,
          "PSK auto: NaN mid-over, the next over decodes", "got \"" + printable(got) + "\"");
}

void testPskNoise() {
    const double rates[2] = {48000, 12000};
    for (int r = 0; r < 2; ++r) {
        std::vector<float> a(static_cast<std::size_t>(60 * rates[r]));
        Noise g(555 + r);
        for (std::size_t i = 0; i < a.size(); ++i) a[i] = static_cast<float>(0.1 * g.gauss());
        dec::PskAutoDecoder au;
        const std::string got = feed(au, a, rates[r]);
        char name[64];
        std::snprintf(name, sizeof name, "PSK auto: no text from 60 s of noise @%.0f Hz", rates[r]);
        check(got.empty(), name, "got \"" + printable(got) + "\"");
        for (int s = 0; s < 3; ++s) {
            dec::PskDecoder d(kSpeeds[s]);
            const std::string g1 = feed(d, a, rates[r]);
            std::snprintf(name, sizeof name, "%s alone: <= 2 characters from 60 s of noise @%.0f", kNames[s], rates[r]);
            check(g1.size() <= 2, name, "got \"" + printable(g1) + "\"");
        }
    }
}

// --- Hell ------------------------------------------------------------------

// Decoded columns (top first) -> dot stream in time order (bottom first).
std::vector<float> dotStream(const std::vector<std::vector<float> >& cols) {
    std::vector<float> d;
    for (std::size_t c = 0; c < cols.size(); ++c)
        for (int r = 13; r >= 0; --r) d.push_back(cols[c][static_cast<std::size_t>(r)]);
    return d;
}

// The expected dot stream (0/1) of a text, as sent.
std::vector<float> expectedDots(const std::string& text) {
    std::vector<float> e;
    for (std::size_t i = 0; i < text.size(); ++i) {
        unsigned short g[7];
        dec::hellGlyph(text[i], g);
        for (int x = 0; x < 7; ++x)
            for (int d = 0; d < 14; ++d) e.push_back((g[x] >> d & 1) ? 1.0f : 0.0f);
    }
    return e;
}

double corr(const std::vector<float>& a, std::size_t aoff, const std::vector<float>& b, std::size_t boff, std::size_t n) {
    double ma = 0, mb = 0;
    for (std::size_t i = 0; i < n; ++i) { ma += a[aoff + i]; mb += b[boff + i]; }
    ma /= n; mb /= n;
    double sab = 0, saa = 0, sbb = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const double x = a[aoff + i] - ma, y = b[boff + i] - mb;
        sab += x * y; saa += x * x; sbb += y * y;
    }
    return saa > 0 && sbb > 0 ? sab / std::sqrt(saa * sbb) : 0;
}

void testHell() {
    const std::string text = "OFFGRID-SDR HELL 73, CQ? 0123456789 ABCDEFGHIJKLMNOPQRSTUVWXYZ./";
    const double rates[2] = {48000, 12000};
    for (int r = 0; r < 2; ++r) {
        const double rate = rates[r];
        std::vector<float> sig;
        dec::synthHell(text, rate, sig, 1000, 0.3);
        const double lead = 0.5, tail = 0.5;
        const std::vector<float> a = withNoise(sig, rate, lead, tail, noiseSigma(0.3, 10, rate), 31 + r);
        dec::HellDecoder h;
        std::vector<std::vector<float> > cols;
        static const std::size_t sizes[] = {1024, 333, 4096, 7, 2048, 960};
        std::size_t i = 0, k = 0;
        while (i < a.size()) {
            std::size_t n = sizes[k++ % 6];
            if (n > a.size() - i) n = a.size() - i;
            h.process(&a[i], n, rate);
            h.takeColumns(cols);
            i += n;
        }
        char name[128];
        const double secs = a.size() / rate, want = secs * 245 / 14;
        std::snprintf(name, sizeof name, "Hell @%.0f Hz: %zu columns for %.2f s (%.1f expected, 17.5/s)", rate,
                      cols.size(), secs, want);
        check(std::fabs(cols.size() - want) <= 1.5, name);
        bool range = true;
        for (std::size_t c = 0; c < cols.size(); ++c) {
            if (cols[c].size() != 14) range = false;
            for (std::size_t j = 0; j < cols[c].size(); ++j)
                if (!(cols[c][j] >= 0 && cols[c][j] <= 1)) range = false;
        }
        std::snprintf(name, sizeof name, "Hell @%.0f Hz: columns of 14 values in 0..1", rate);
        check(range, name);

        // Find where the lettering starts in the dot stream, then read each letter.
        const std::vector<float> d = dotStream(cols), e = expectedDots(text);
        const std::size_t startGuess = static_cast<std::size_t>(lead * 245);
        double bestC = -2;
        std::size_t best = 0;
        for (std::size_t lag = startGuess > 40 ? startGuess - 40 : 0; lag < startGuess + 40 && lag + e.size() <= d.size(); ++lag) {
            const double c = corr(d, lag, e, 0, e.size());
            if (c > bestC) { bestC = c; best = lag; }
        }
        std::snprintf(name, sizeof name, "Hell @%.0f Hz: strip matches the sent lettering (corr %.2f at dot %zu)", rate,
                      bestC, best);
        check(bestC > 0.75, name);
        // OCR: each letter's 98 dots against every glyph of the font.
        const char* alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,/?-";
        int right = 0, total = 0;
        std::string read;
        for (std::size_t ci = 0; ci < text.size(); ++ci) {
            if (text[ci] == ' ') { read += ' '; continue; }
            ++total;
            const std::size_t off = best + ci * 98;
            if (off + 98 > d.size()) break;
            double bc = -2;
            char bch = '#';
            for (const char* p = alphabet; *p; ++p) {
                const std::vector<float> g = expectedDots(std::string(1, *p));
                const double c = corr(d, off, g, 0, 98);
                if (c > bc) { bc = c; bch = *p; }
            }
            read += bch;
            if (bch == text[ci]) ++right;
        }
        std::snprintf(name, sizeof name, "Hell @%.0f Hz: letters read back from the strip (%d/%d)", rate, right, total);
        check(right == total, name, "read \"" + read + "\"");
    }
}

// Hell status().signal through a receiver with AGC: the encoder's lettering
// looped with 2 s gaps in which the AGC turns the noise up to the signal's
// level.  The flag must be up during every over and never up on noise only
// (white, or band-limited 300-3000 Hz like a receiver's audio).
void testHellSignalFlag() {
    const double rate = 48000;
    std::vector<float> over;
    dec::synthHell("OFFGRID HELL 73", rate, over, 1000, 0.3);
    double ps = 0;
    for (std::size_t i = 0; i < over.size(); ++i) ps += over[i] * over[i];
    const double rmsSig = std::sqrt(ps / over.size());
    const std::size_t gap = static_cast<std::size_t>(2 * rate);
    std::vector<float> a;
    std::vector<std::size_t> mids;
    Noise g(2024);
    const double sigmaUnder = noiseSigma(0.3, 10, rate) * 0.3;   // quiet noise under the over (the AGC turned down)
    for (int loop = 0; loop < 4; ++loop) {
        mids.push_back(a.size() + over.size() / 2);
        for (std::size_t i = 0; i < over.size(); ++i) a.push_back(static_cast<float>(over[i] + sigmaUnder * g.gauss()));
        for (std::size_t i = 0; i < gap; ++i) a.push_back(static_cast<float>(rmsSig * g.gauss()));   // loud noise
    }
    dec::HellDecoder h;
    std::vector<std::vector<float> > cols;
    int up = 0;
    std::string seen;
    for (std::size_t i = 0, m = 0; i < a.size(); i += 480) {
        h.process(&a[i], i + 480 <= a.size() ? 480 : a.size() - i, rate);
        h.takeColumns(cols);
        if (m < mids.size() && i >= mids[m]) {
            const bool sig = h.status().signal && h.status().locked;
            if (sig) ++up;
            seen += sig ? '1' : '0';
            ++m;
        }
    }
    check(up == static_cast<int>(mids.size()), "Hell: signal/locked up in every over (looped, AGC-loud noise gaps)",
          "mid-over flags " + seen + ", info \"" + h.status().info + "\"");

    for (int kind = 0; kind < 2; ++kind) {
        std::vector<float> n(static_cast<std::size_t>(20 * rate));
        Noise gn(77 + kind);
        double lp1 = 0, lp2 = 0, hp = 0, prev = 0;
        const double al = 1 - std::exp(-6.283185307179586 * 3000 / rate), ah = std::exp(-6.283185307179586 * 300 / rate);
        for (std::size_t i = 0; i < n.size(); ++i) {
            double v = 0.3 * gn.gauss();
            if (kind == 1) {   // 300-3000 Hz
                lp1 += al * (v - lp1); lp2 += al * (lp1 - lp2);
                hp = ah * (hp + lp2 - prev); prev = lp2;
                v = hp * 3;
            }
            n[i] = static_cast<float>(v);
        }
        dec::HellDecoder hn;
        int falseUp = 0, blocks = 0;
        for (std::size_t i = 0; i < n.size(); i += 480, ++blocks) {
            hn.process(&n[i], 480, rate);
            hn.takeColumns(cols);
            if (hn.status().signal || hn.status().locked) ++falseUp;
        }
        char name[96];
        std::snprintf(name, sizeof name, "Hell: signal flag never up on 20 s of %s noise (%d/%d blocks)",
                      kind ? "300-3000 Hz" : "white", falseUp, blocks);
        check(falseUp == 0, name, "info \"" + hn.status().info + "\"");
    }
}

void testHellNaN() {
    const double rate = 48000;
    std::vector<float> sig;
    dec::synthHell("HELL", rate, sig, 1000, 0.3);
    std::vector<float> a = withNoise(sig, rate, 0.3, 0.2, noiseSigma(0.3, 20, rate), 8);
    a[1000] = std::numeric_limits<float>::quiet_NaN();
    dec::HellDecoder h;
    std::vector<std::vector<float> > cols;
    for (std::size_t i = 0; i < a.size(); i += 1024) {
        h.process(&a[i], i + 1024 <= a.size() ? 1024 : a.size() - i, rate);
        h.takeColumns(cols);
    }
    bool ok = !cols.empty();
    double maxLate = 0;
    for (std::size_t c = 0; c < cols.size(); ++c)
        for (std::size_t j = 0; j < 14; ++j) {
            if (!(cols[c][j] >= 0 && cols[c][j] <= 1)) ok = false;
            if (c > 20 && cols[c][j] > maxLate) maxLate = cols[c][j];
        }
    check(ok && maxLate > 0.9, "Hell: a NaN sample does not poison the strip");
}

void testCpu() {
    const double rate = 48000, secs = 30;
    std::vector<float> sig;
    while (sig.size() < secs * rate) dec::synthPsk(kTexts[2], 125, rate, sig, 1010, 0.3);
    sig.resize(static_cast<std::size_t>(secs * rate));
    Noise g(1);
    for (std::size_t i = 0; i < sig.size(); ++i) sig[i] += static_cast<float>(0.05 * g.gauss());
    {
        dec::PskAutoDecoder au;
        const std::clock_t t0 = std::clock();
        for (std::size_t i = 0; i < sig.size(); i += 1024)
            au.process(&sig[i], i + 1024 <= sig.size() ? 1024 : sig.size() - i, rate);
        const double cpu = double(std::clock() - t0) / CLOCKS_PER_SEC;
        char name[128];
        std::snprintf(name, sizeof name, "CPU: PSK auto (3 speeds) @48 kHz: %.0fx real time (%.1f%% of a core)",
                      secs / (cpu > 1e-9 ? cpu : 1e-9), 100 * cpu / secs);
        check(true, name);
    }
    {
        dec::HellDecoder h;
        std::vector<std::vector<float> > cols;
        const std::clock_t t0 = std::clock();
        for (std::size_t i = 0; i < sig.size(); i += 1024) {
            h.process(&sig[i], i + 1024 <= sig.size() ? 1024 : sig.size() - i, rate);
            h.takeColumns(cols);
        }
        const double cpu = double(std::clock() - t0) / CLOCKS_PER_SEC;
        char name[128];
        std::snprintf(name, sizeof name, "CPU: Hell @48 kHz: %.0fx real time (%.1f%% of a core)",
                      secs / (cpu > 1e-9 ? cpu : 1e-9), 100 * cpu / secs);
        check(true, name);
    }
}

void testEncoders() {
    // Varicode: no code contains "00"; every code round-trips.
    bool ok = true;
    for (int c = 0; c < 128; ++c) {
        const std::string v = dec::pskVaricode(c);
        if (v.find("00") != std::string::npos || v[0] != '1' || v[v.size() - 1] != '1') ok = false;
        if (dec::pskVaricodeLookup(v.c_str(), v.size()) != c) ok = false;
    }
    check(ok, "Varicode table: 128 codes, none with \"00\", all round-trip");
    check(dec::pskBits("e", 2) == std::string("00") + "11" + "00" + "00", "pskBits(\"e\") framing");
    std::vector<float> a, b;
    dec::synthPsk("TEST", 31.25, 8000, a);
    dec::synthPsk("TEST", 31.25, 8000, b);
    check(a == b && !a.empty(), "PSK synthesis is deterministic");
    unsigned short g[7];
    check(dec::hellGlyph('H', g) && g[0] == 0 && g[1] == 0x3FFF && g[3] == (3u << 6) && g[6] == 0, "Hell glyph 'H'");
}

}  // namespace

int main() {
    testEncoders();
    testPsk();
    testPskAfcEdge();
    testPskMidOver();
    testPskAutoSwitch();
    testPskNaN();
    testPskNoise();
    testHell();
    testHellSignalFlag();
    testHellNaN();
    testCpu();
    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
