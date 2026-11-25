// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_decoders_fsk.cpp
//  Layer:  Tests (library liboffgrid-decoders)
//
//  What this file does
//  -------------------
//  Self-checking test of the CW, RTTY, NAVTEX and DSC decoders against the
//  generators in FskEncoders.h.  Prints PASS/FAIL per check and returns
//  nonzero if anything failed.  At 48000 and 12000 Hz, with white noise
//  for 15 dB SNR in 3 kHz:
//    - CW at 20 and 35 WPM: the second sending decodes exactly (the first
//      is used to learn the speed);
//    - RTTY 45.45 Bd / 170 Hz and 50 Bd / 450 Hz, normal and reversed
//      (found automatically): the text line decodes exactly;
//    - NAVTEX: the message (header, subject, body) exactly;
//    - DSC: an all-ships safety call and a distress call (format,
//      category, nature, MMSI);
//    - RTTY overs separated by 3 s of band noise at the signal's level (as
//      a receiver's AGC delivers a looped station): every over after the
//      first decodes exactly, nothing else is printed, the speed holds;
//    - reset(): decoding again after reset() gives the same text;
//    - noise only (20 s): no text from any decoder;
//    - CPU: seconds of audio decoded per CPU second, per decoder at 48 kHz.
//
//  How it fits in
//  --------------
//  Build from the repository root (scratch output anywhere):
//    c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors -Isrc src/dsp/Fft.cpp \
//        src/decoders/Codes.cpp src/decoders/Cw.cpp src/decoders/Rtty.cpp \
//        src/decoders/Navtex.cpp src/decoders/FskEncoders.cpp \
//        tests/test_decoders_fsk.cpp -o test_decoders_fsk
//
//  Contributing
//  ------------
//  Add a check per new decoder feature; keep the run under ~30 s.
// ============================================================================
#include "decoders/Codes.h"
#include "decoders/Cw.h"
#include "decoders/FskEncoders.h"
#include "decoders/Navtex.h"
#include "decoders/Rtty.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

using namespace dec;

namespace {

int g_failures = 0, g_checks = 0;

void check(bool ok, const std::string& name, const std::string& detail = "") {
    ++g_checks;
    if (!ok) ++g_failures;
    std::printf("%s  %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : "  -- ", detail.c_str());
}

std::string show(const std::string& s) {
    std::string r;
    for (std::size_t i = 0; i < s.size(); ++i) r += s[i] == '\n' ? std::string("\\n") : std::string(1, s[i]);
    return r.size() > 400 ? r.substr(r.size() - 400) : r;
}

// Feeds the audio in uneven blocks; returns the text and adds the CPU time.
std::string run(Decoder& d, const std::vector<float>& a, double rate, double* cpu = nullptr) {
    std::string text;
    const std::clock_t c0 = std::clock();
    std::size_t i = 0, k = 0;
    const std::size_t sizes[] = {1000, 4096, 333, 2048, 7};
    while (i < a.size()) {
        const std::size_t n = std::min(sizes[k++ % 5], a.size() - i);
        d.process(&a[i], n, rate);
        i += n;
        d.takeText(text);
    }
    if (cpu) *cpu += static_cast<double>(std::clock() - c0) / CLOCKS_PER_SEC;
    return text;
}

void noisy(std::vector<float>& a, double amp, double rate, unsigned seed) {
    enc::addWhiteNoise(a, enc::noiseRmsForSnr(amp * amp / 2, 15, rate), seed);
}

struct Cpu {
    Cpu() : audio(0), cpu(0) {}
    double audio, cpu;
};
Cpu g_cpu[4];   // CW, RTTY, NAVTEX, DSC (48 kHz only)

const double kAmp = 0.3;

// ---- CW ----
void testCw(double rate, double wpm) {
    const std::string msg = "CQ CQ DE OFFGRID-SDR DESIGNED BY DIDDY 73";
    // (sent twice in one transmission: the first letters of a transmission go to learning the
    //  speed, and after a pause of over a second the first element can be cut short while the
    //  levels re-adapt, as in the browser: "missing the first letter or so is the better trade")
    std::vector<float> a = enc::cwAudio(msg + " " + msg, wpm, 700, rate, kAmp, 1.0, 2.5);
    noisy(a, kAmp, rate, 11);
    CwDecoder d;
    double cpu = 0;
    const std::string t = run(d, a, rate, &cpu);
    if (rate == 48000) { g_cpu[0].audio += a.size() / rate; g_cpu[0].cpu += cpu; }
    char name[96];
    std::snprintf(name, sizeof name, "CW %g WPM at %g Hz: second sending exact", wpm, rate);
    check(t.find(" " + msg + "\n") != std::string::npos, name, "got [" + show(t) + "]");
    std::snprintf(name, sizeof name, "CW %g WPM at %g Hz: speed and tone", wpm, rate);
    const Status s = d.status();
    char det[96];
    std::snprintf(det, sizeof det, "wpm %.1f tone %.1f", d.wpm(), s.toneHz);
    check(std::fabs(d.wpm() - wpm) < wpm * 0.15 && std::fabs(s.toneHz - 700) < 10, name, det);
    // reset: the same again
    d.reset();
    const std::string t2 = run(d, a, rate);
    std::snprintf(name, sizeof name, "CW %g WPM at %g Hz: reset() then the same text", wpm, rate);
    check(t2 == t, name, "got [" + show(t2) + "]");
}

// ---- RTTY ----
void testRtty(double rate, double baud, double shift, bool reverse) {
    const std::string line = "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG 0123456789";
    std::string text = "RYRYRY CQ CQ CQ DE OFFGRID-SDR K\n";
    for (int i = 0; i < 4; ++i) text += line + "\n";
    std::vector<float> a = enc::rttyAudio(text, baud, shift, 1500, rate, reverse, kAmp);
    noisy(a, kAmp, rate, 22);
    RttyDecoder d;
    double cpu = 0;
    const std::string t = run(d, a, rate, &cpu);
    if (rate == 48000) { g_cpu[1].audio += a.size() / rate; g_cpu[1].cpu += cpu; }
    char name[128];
    std::snprintf(name, sizeof name, "RTTY %g/%g %s at %g Hz: text exact", baud, shift, reverse ? "reversed" : "normal", rate);
    check(t.find("\n" + line + "\n") != std::string::npos, name, "got [" + show(t) + "]");
    const Status s = d.status();
    char det[128];
    std::snprintf(det, sizeof det, "mode %s mark %.1f space %.1f reverse %d", s.mode.c_str(), d.markHz(), d.spaceHz(), d.reverse());
    std::snprintf(name, sizeof name, "RTTY %g/%g %s at %g Hz: tones, speed, polarity found", baud, shift, reverse ? "reversed" : "normal", rate);
    check(std::fabs(s.baud - baud) < 0.01 && std::fabs(d.markHz() - (1500 + shift / 2)) < 8 &&
              std::fabs(d.spaceHz() - (1500 - shift / 2)) < 8 && d.reverse() == reverse,
          name, det);
    if (!reverse && baud < 46) {
        d.reset();
        const std::string t2 = run(d, a, rate);
        std::snprintf(name, sizeof name, "RTTY %g/%g at %g Hz: reset() then the same text", baud, shift, rate);
        check(t2 == t, name, "got [" + show(t2) + "]");
    }
}

// ---- RTTY overs with loud noise between them ----
// A looped station as a receiver with AGC delivers it: each over, then 3 s in which the AGC lifts
// the band noise to about the signal's level. (Once, the noise's flicker was read as 75 baud and
// the decoder stayed there: every over after the first gap came out as rubbish.)
std::vector<float> bandNoise(std::size_t n, double rms, double rate, unsigned seed) {
    std::vector<float> x(n, 0.f);
    enc::addWhiteNoise(x, 1.0, seed);
    const double a = 1 - std::exp(-2 * 3.14159265358979 * 2500 / rate);   // ~3 kHz passband
    double l1 = 0, l2 = 0, l3 = 0, hp = 0, sum = 0;
    for (std::size_t i = 0; i < n; ++i) {
        l1 += a * (x[i] - l1); l2 += a * (l1 - l2); l3 += a * (l2 - l3);
        hp += 0.01 * (l3 - hp);                                             // (and no DC)
        x[i] = static_cast<float>(l3 - hp);
        sum += x[i] * x[i];
    }
    const double g = rms / std::sqrt(sum / std::max<std::size_t>(1, n));
    for (std::size_t i = 0; i < n; ++i) x[i] = static_cast<float>(x[i] * g);
    return x;
}

void testRttyOvers(double rate, double baud, double shift) {
    const std::string over = "CQ CQ CQ DE OFFGRID OFFGRID PSE K";
    std::vector<float> a;
    const int kOvers = 5;
    for (int k = 0; k < kOvers; ++k) {
        std::vector<float> o = enc::rttyAudio("\n" + over + "\n", baud, shift, 1500, rate, false, kAmp);
        noisy(o, kAmp, rate, 60 + k);
        enc::append(a, o);
        enc::append(a, bandNoise(static_cast<std::size_t>(3 * rate), kAmp / std::sqrt(2.0), rate, 70 + k));
    }
    RttyDecoder d;
    const std::string t = run(d, a, rate);
    // every over after the first (which goes partly to finding tones and speed) exactly, and
    // nothing else: no rubbish from the noise between them
    int good = 0, bad = 0, line = 0;
    std::size_t p = 0;
    while (p < t.size()) {
        std::size_t e = t.find('\n', p);
        if (e == std::string::npos) e = t.size();
        const std::string l = t.substr(p, e - p);
        if (line++ > 0 && !l.empty()) { if (l == over) ++good; else ++bad; }
        p = e + 1;
    }
    char name[128], det[64];
    std::snprintf(name, sizeof name, "RTTY %g/%g at %g Hz: overs with loud noise between: each decodes", baud, shift, rate);
    std::snprintf(det, sizeof det, "%d good, %d bad; ", good, bad);
    check(good == kOvers - 1 && bad == 0, name, det + std::string("got [") + show(t) + "]");
    std::snprintf(name, sizeof name, "RTTY %g/%g at %g Hz: overs with loud noise between: speed kept", baud, shift, rate);
    check(std::fabs(d.status().baud - baud) < 0.01, name, d.status().mode);
}

// ---- NAVTEX ----
void testNavtex(double rate) {
    const std::string body = "051200 UTC OCT 26\nOFFGRID-SDR NAVTEX TEST\nGALE WARNING: DOVER, WIGHT: SW 7 TO SEVERE GALE 9, VEERING NW LATER.";
    const std::string msg = "ZCZC SA23\n" + body + "\nNNNN\n";
    std::vector<float> a = enc::silence(0.5, rate);
    enc::append(a, enc::navtexAudio(msg, rate, 1500, kAmp, 144));
    noisy(a, kAmp, rate, 33);
    NavtexDecoder d;
    double cpu = 0;
    const std::string t = run(d, a, rate, &cpu);
    if (rate == 48000) { g_cpu[2].audio += a.size() / rate; g_cpu[2].cpu += cpu; }
    const std::string want = "Station S \xC2\xB7 Navigational warning \xC2\xB7 No. 23\n" + body + "\n\n";
    char name[96];
    std::snprintf(name, sizeof name, "NAVTEX at %g Hz: message exact (header, subject, body)", rate);
    check(t == want, name, "got [" + show(t) + "]");
    std::snprintf(name, sizeof name, "NAVTEX at %g Hz: messages() holds it", rate);
    check(d.messages().size() == 1 && d.messages()[0].done && d.messages()[0].id == "SA23" && d.messages()[0].text == body, name);
    d.reset();
    const std::string t2 = run(d, a, rate);
    std::snprintf(name, sizeof name, "NAVTEX at %g Hz: reset() then the same text", rate);
    check(t2 == t, name, "got [" + show(t2) + "]");
}

// ---- DSC ----
void testDsc(double rate) {
    std::vector<int> bits = enc::dscBits(enc::dscAllShipsFields(108, "235009876"));
    std::vector<int> dots;
    for (int i = 0; i < 300; ++i) dots.push_back(i & 1);
    bits.insert(bits.end(), dots.begin(), dots.end());
    const std::vector<int> b2 = enc::dscBits(enc::dscDistressFields("232123456", 105));
    bits.insert(bits.end(), b2.begin(), b2.end());
    bits.insert(bits.end(), dots.begin(), dots.end());
    std::vector<int> twice(bits);
    twice.insert(twice.end(), bits.begin(), bits.end());   // the demo loops the pair
    std::vector<float> a = enc::silence(0.5, rate);
    enc::append(a, enc::dscAudio(twice, rate, 1500, kAmp));
    noisy(a, kAmp, rate, 44);
    DscDecoder d;
    double cpu = 0;
    const std::string t = run(d, a, rate, &cpu);
    if (rate == 48000) { g_cpu[3].audio += a.size() / rate; g_cpu[3].cpu += cpu; }
    const std::string safety = "Safety \xC2\xB7 all ships \xC2\xB7 from MMSI 235009876\n";
    const std::string distress = "Distress (sinking) \xC2\xB7 all ships \xC2\xB7 from MMSI 232123456\n";
    char name[96];
    std::snprintf(name, sizeof name, "DSC at %g Hz: safety call to all ships", rate);
    check(t.find(safety) != std::string::npos, name, "got [" + show(t) + "]");
    std::snprintf(name, sizeof name, "DSC at %g Hz: distress call (sinking)", rate);
    check(t.find(distress) != std::string::npos, name, "got [" + show(t) + "]");
    bool fields = false;
    for (std::size_t i = 0; i < d.calls().size(); ++i) {
        const DscCall& c = d.calls()[i];
        if (c.format == "Distress" && c.category == "Distress" && c.nature == "sinking" && c.from == "232123456") fields = true;
    }
    std::snprintf(name, sizeof name, "DSC at %g Hz: call fields (format, category, nature, MMSI)", rate);
    check(fields, name);
    std::snprintf(name, sizeof name, "DSC at %g Hz: no garbled calls", rate);
    {
        bool clean = true;
        std::size_t p = 0;
        while (p < t.size()) {
            const std::size_t e = t.find('\n', p);
            const std::string l = t.substr(p, e - p + 1);
            if (l != safety && l != distress) clean = false;
            p = e == std::string::npos ? t.size() : e + 1;
        }
        check(clean && !t.empty(), name, "got [" + show(t) + "]");
    }
    d.reset();
    const std::string t2 = run(d, a, rate);
    std::snprintf(name, sizeof name, "DSC at %g Hz: reset() then the same text", rate);
    check(t2 == t, name, "got [" + show(t2) + "]");
}

void testNoise(double rate) {
    std::vector<float> a = enc::silence(20, rate);
    noisy(a, kAmp, rate, 55);
    CwDecoder cw;
    RttyDecoder rtty;
    NavtexDecoder nav;
    DscDecoder dsc;
    const std::string t1 = run(cw, a, rate), t2 = run(rtty, a, rate), t3 = run(nav, a, rate), t4 = run(dsc, a, rate);
    char name[64];
    std::snprintf(name, sizeof name, "noise only 20 s at %g Hz: CW silent", rate);
    check(t1.empty(), name, "got [" + show(t1) + "]");
    std::snprintf(name, sizeof name, "noise only 20 s at %g Hz: RTTY silent", rate);
    check(t2.empty(), name, "got [" + show(t2) + "]");
    std::snprintf(name, sizeof name, "noise only 20 s at %g Hz: NAVTEX silent", rate);
    check(t3.empty(), name, "got [" + show(t3) + "]");
    std::snprintf(name, sizeof name, "noise only 20 s at %g Hz: DSC silent", rate);
    check(t4.empty(), name, "got [" + show(t4) + "]");
}

}  // namespace

int main() {
    const double rates[] = {48000, 12000};
    for (int r = 0; r < 2; ++r) {
        const double rate = rates[r];
        testCw(rate, 20);
        testCw(rate, 35);
        testRtty(rate, 45.45, 170, false);
        testRtty(rate, 45.45, 170, true);
        testRtty(rate, 50, 450, false);
        testRtty(rate, 50, 450, true);
        testRttyOvers(rate, 45.45, 170);
        testRttyOvers(rate, 50, 450);
        testNavtex(rate);
        testDsc(rate);
        testNoise(rate);
    }
    const char* names[] = {"CW", "RTTY (with twin)", "NAVTEX", "DSC"};
    for (int i = 0; i < 4; ++i)
        if (g_cpu[i].cpu > 0)
            std::printf("CPU  %-17s %.0f s of 48 kHz audio per CPU second\n", names[i], g_cpu[i].audio / g_cpu[i].cpu);
    std::printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
