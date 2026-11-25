// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_decoders_wspr.cpp
//  Layer:  Tests (library liboffgrid-decoders)
//
//  What this file does
//  -------------------
//  Self-checking test of the WSPR decoder (Wspr.h) against the generators
//  in WsprEncoders.h.  Prints PASS/FAIL per check and returns nonzero if
//  anything failed.  Time is simulated: the audio is generated for a
//  chosen UTC span and the timestamps passed with it (no real waiting).
//    - pack/unpack round trips, invalid messages refused, power rounding,
//      the K1ABC FN42 37 reference symbols;
//    - convolutional encode + Fano decode, with hard and soft errors;
//    - four stations in one slot at 1400-1600 Hz, -20 to -26 dB (2500 Hz
//      reference), at 12000 and 48000 Hz: decoded exactly (call, locator,
//      power), frequency within 0.5 Hz, dt within 0.2 s, SNR within 3 dB,
//      RF = dial + audio;
//    - the computer's clock 40 s fast and 40 s slow: still decoded (the
//      start-time search) and the clock offset reported within 0.2 s;
//      with that correction applied, the next slot's dt is ~0;
//    - a stick 150 Hz off on 20 m: the stations found at 1550-1750 Hz, the
//      calibration offer at +12.9 ppm (median 1681.5 Hz); after the correction the check
//      confirms it;
//    - drifting stations (+2, -2 Hz over the transmission): decoded, drift
//      reported (the JS's sync search assumes none: larger drifts are lost);
//    - noise alone (two slots, two rates): nothing decoded;
//    - NaN / inf samples, bad rates, NaN timestamps: no crash, still decodes;
//    - CPU time of each slot's decode.
//
//  How it fits in
//  --------------
//  Build from the repository root (scratch output anywhere):
//    c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors -Isrc src/dsp/Fft.cpp \
//        src/decoders/Wspr.cpp src/decoders/WsprEncoders.cpp \
//        tests/test_decoders_wspr.cpp -o wspr_t
//
//  Contributing
//  ------------
//  Add a check per new feature; keep the run well under a minute.
// ============================================================================
#include "decoders/Wspr.h"
#include "decoders/WsprEncoders.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace dec;

namespace {

int g_failures = 0, g_checks = 0;
double g_maxCpu = 0, g_sumCpu = 0;
int g_cpuN = 0;

void check(bool ok, const std::string& name, const std::string& detail = "") {
    ++g_checks;
    if (!ok) ++g_failures;
    std::printf("%s  %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : "  ", detail.c_str());
}

std::string fmt(const char* f, double a, double b = 0, double c = 0, double d = 0) {
    char buf[256];
    std::snprintf(buf, sizeof buf, f, a, b, c, d);
    return buf;
}

const double kT0 = 1795000000.0 - std::fmod(1795000000.0, 120.0);   // an even minute (true time)

// Feeds the decoder from a WsprAir: true time [t, t + secs), the PC clock `pcErr` s ahead of true time.
struct Run {
    Run(double rate, const std::vector<enc::WsprStation>& st, double noise, unsigned seed = 7)
        : air(rate, st, noise, seed), t(0), pcErr(0), rng(seed * 2654435761u + 1), nanAt(-1) {}
    enc::WsprAir air;
    WsprDecoder dec;
    double t, pcErr;
    unsigned rng;
    double nanAt;   // true time of a 0.3 s burst of NaN/inf (-1: none)
    std::vector<WsprDecode> decodes;
    std::vector<float> buf;

    unsigned rnd() { rng = rng * 1664525u + 1013904223u; return rng >> 8; }
    void step(double secs) {
        const double rate = air.rate();
        const double end = t + secs;
        while (t < end - 1e-9) {
            // uneven blocks of 40-200 ms
            std::size_t n = static_cast<std::size_t>(rate * (0.04 + 0.16 * (rnd() % 1000) / 1000.0));
            n = std::min(n, static_cast<std::size_t>(std::ceil((end - t) * rate)));
            if (n == 0) n = 1;
            buf.resize(n);
            air.render(t, &buf[0], n);
            if (nanAt >= 0)
                for (std::size_t i = 0; i < n; ++i) {
                    const double ti = t + i / rate;
                    if (ti >= nanAt && ti < nanAt + 0.3) buf[i] = (i & 1) ? std::nanf("") : HUGE_VALF;
                }
            // the timestamp: the PC clock at the first sample, 0-20 ms late
            dec.setTimestamp(t + pcErr + 0.02 * (rnd() % 1000) / 1000.0);
            dec.process(&buf[0], n, rate);
            t += n / rate;
            if (dec.newSlotDone()) {
                g_maxCpu = std::max(g_maxCpu, dec.lastDecodeCpuSeconds());
                g_sumCpu += dec.lastDecodeCpuSeconds();
                ++g_cpuN;
            }
            dec.takeDecodes(decodes);
        }
    }
};

const WsprDecode* find(const std::vector<WsprDecode>& v, const std::string& call) {
    for (std::size_t i = 0; i < v.size(); ++i)
        if (v[i].call == call) return &v[i];
    return 0;
}

// ---- codec ----
void codecTests() {
    unsigned char b[50];
    wspr::Message m;
    const char* calls[] = {"K1ABC", "G4OGR", "VK2OGS", "AA1A", "W1AW", "9A1AA", "KH6ABC"};
    const char* grids[] = {"FN42", "IO91", "QF56", "AA00", "RR99", "JN75", "BL01"};
    const int dbms[] = {37, 30, 23, 0, 60, 10, 33};
    bool all = true;
    for (int i = 0; i < 7; ++i) {
        const bool ok = wspr::pack(calls[i], grids[i], dbms[i], b) && wspr::unpack(b, m) && m.call == calls[i] &&
                        m.grid == grids[i] && m.dbm == dbms[i];
        if (!ok) std::printf("   round trip failed: %s %s %d -> %s %s %d\n", calls[i], grids[i], dbms[i], m.call.c_str(),
                             m.grid.c_str(), m.dbm);
        all = all && ok;
    }
    check(all, "pack/unpack round trip (7 messages)");
    check(wspr::pack("k1abc", "fn42", 36, b) && wspr::unpack(b, m) && m.call == "K1ABC" && m.dbm == 37,
          "lower case accepted, 36 dBm rounded to 37");
    check(!wspr::pack("ABCDEF", "FN42", 37, b) && !wspr::pack("K1ABC", "SS42", 37, b) &&
              !wspr::pack("K1ABC/P", "FN42", 37, b) && !wspr::pack("K1", "FN42", 37, b),
          "invalid messages refused");
    // a type 2-like power value is refused by unpack
    wspr::pack("K1ABC", "FN42", 37, b);
    unsigned char b2[50];
    std::copy(b, b + 50, b2);
    b2[49] ^= 1;   // 37 -> 36 dBm
    check(!wspr::unpack(b2, m), "unpack refuses a non-standard power (types 2/3)");
    check(std::fabs(wspr::dbmToWatts(30) - 1) < 1e-9 && wspr::wattsText(30) == "1 W" && wspr::wattsText(23) == "200 mW" &&
              wspr::wattsText(37) == "5 W",
          "watts: 30 dBm = 1 W, 23 dBm = 200 mW, 37 dBm = 5 W", wspr::wattsText(23) + ", " + wspr::wattsText(37));
    // reference: WSJT's K1ABC FN42 37 channel symbols (as published with wsprsim / G4JNT's coding notes)
    static const unsigned char ref[162] = {
        3,3,0,0,2,0,0,0,1,0,2,0,1,3,1,2,2,2,1,0,0,3,2,3,1,3,3,2,2,0,2,0,0,0,3,2,0,1,2,3,2,2,0,0,2,2,3,2,1,1,0,2,3,3,2,1,0,2,2,1,
        3,2,1,2,2,2,0,3,3,0,3,0,3,0,1,2,1,0,2,1,2,0,3,2,1,3,2,0,0,3,3,2,3,0,3,2,2,0,3,0,2,0,2,0,1,0,2,3,0,2,1,1,1,2,3,3,0,2,3,1,
        2,1,2,2,2,1,3,3,2,0,0,0,0,1,0,3,2,0,1,3,2,2,2,2,2,0,2,3,3,2,3,2,3,3,2,0,0,3,1,2,2,2};
    unsigned char sym[162];
    int diff = 0;
    if (wspr::symbols("K1ABC", "FN42", 37, sym))
        for (int i = 0; i < 162; ++i) diff += sym[i] != ref[i];
    else diff = 162;
    check(diff == 0, "K1ABC FN42 37 symbols match the reference", fmt("%.0f differ", diff));
}

void fanoTests() {
    unsigned rng = 99;
    struct R { static double u(unsigned& r) { r = r * 1664525u + 1013904223u; return (r >> 8) / 16777216.0; } };
    int okClean = 0, okHard = 0, okSoft = 0, wrong = 0;
    const int trials = 40;
    for (int tr = 0; tr < trials; ++tr) {
        unsigned char b[50], c[162], o[50];
        for (int i = 0; i < 50; ++i) b[i] = R::u(rng) < 0.5;
        wspr::encode(b, c);
        double soft[162];
        for (int i = 0; i < 162; ++i) soft[i] = c[i] ? 4 : -4;
        if (wspr::fano(soft, o) && std::equal(b, b + 50, o)) ++okClean;
        // hard errors: 8 % of the bits flipped (13 of 162), the rest confident
        for (int i = 0; i < 162; ++i) soft[i] = (c[i] ? 3 : -3) * (R::u(rng) < 0.08 ? -1 : 1);
        if (wspr::fano(soft, o)) { if (std::equal(b, b + 50, o)) ++okHard; else ++wrong; }
        // soft: Gaussian noise on +-1 at Eb/N0 ~ 3 dB (LLR = 2 y / sigma^2)
        const double sigma = 0.75;
        for (int i = 0; i < 162; ++i) {
            const double u1 = std::max(1e-12, R::u(rng)), u2 = R::u(rng);
            const double y = (c[i] ? 1 : -1) + sigma * std::sqrt(-2 * std::log(u1)) * std::cos(6.283185307 * u2);
            soft[i] = 2 * y / (sigma * sigma);
        }
        if (wspr::fano(soft, o)) { if (std::equal(b, b + 50, o)) ++okSoft; else ++wrong; }
    }
    check(okClean == trials, "Fano: clean codewords decode", fmt("%.0f/%.0f", okClean, trials));
    check(okHard >= trials * 0.95, "Fano: 8% hard bit errors", fmt("%.0f/%.0f", okHard, trials));
    check(okSoft >= trials * 0.9, "Fano: soft Gaussian noise, sigma 0.75", fmt("%.0f/%.0f", okSoft, trials));
    check(wrong == 0, "Fano: no wrong decodes", fmt("%.0f wrong", wrong));
    double noise[162];
    for (int i = 0; i < 162; ++i) noise[i] = std::nan("");
    unsigned char o[50];
    (void)wspr::fano(noise, o, 2.4, 50);
    check(true, "Fano: NaN LLRs do not crash");
}

std::vector<enc::WsprStation> fourStations(double amp, double shift = 0) {
    std::vector<enc::WsprStation> v;
    v.push_back(enc::WsprStation("G4OGR", "IO91", 30, 1420 + shift, amp));                      // -20 dB
    v.push_back(enc::WsprStation("K1OGS", "FN42", 37, 1487.3 + shift, amp * std::pow(10, -2 / 20.0)));   // -22
    v.push_back(enc::WsprStation("VK2OGS", "QF56", 23, 1531.5 + shift, amp * std::pow(10, -4 / 20.0)));  // -24
    v.push_back(enc::WsprStation("JA1XYZ", "PM95", 10, 1589.1 + shift, amp * std::pow(10, -6 / 20.0)));  // -26
    return v;
}

// ---- four stations in one slot ----
void slotCase(double rate) {
    const double amp = 0.01, noise = enc::wsprNoiseFor(amp, -20, rate);
    std::vector<enc::WsprStation> st = fourStations(amp);
    Run r(rate, st, noise, static_cast<unsigned>(rate));
    r.dec.setDialFrequency(14.0956e6);
    r.t = kT0 - 110;   // (110 s before an even minute: the first slot end has too little audio)
    r.step(110 + 125);
    const std::string tag = fmt("%.0f Hz: ", rate);
    check(r.dec.slotsDecoded() == 1, tag + "one slot decoded", fmt("%.0f", r.dec.slotsDecoded()));
    const double snrs[4] = {-20, -22, -24, -26};
    for (int i = 0; i < 4; ++i) {
        const WsprDecode* d = find(r.decodes, st[i].call);
        if (!d) { check(false, tag + st[i].call + " decoded"); continue; }
        const bool exact = d->grid == st[i].grid && d->dbm == st[i].dbm;
        check(exact && std::fabs(d->audioHz - st[i].audioHz) < 0.5 && std::fabs(d->dt) < 0.2 &&
                  std::fabs(d->snrDb - snrs[i]) <= 3 && std::fabs(d->rfHz - (14.0956e6 + d->audioHz)) < 1e-3 &&
                  std::fabs(d->slotStart - kT0) < 1e-6,
              tag + st[i].call + " " + st[i].grid + fmt(" %.0f dBm at %.0f dB", st[i].dbm, snrs[i]),
              fmt("got %.2f Hz, dt %+.2f s, SNR %.0f dB, drift %+.0f", d->audioHz, d->dt, d->snrDb, d->drift));
    }
    check(r.decodes.size() == 4, tag + "nothing else decoded", fmt("%.0f decodes", static_cast<double>(r.decodes.size())));
    std::string text;
    r.dec.takeText(text);
    std::printf("   text: %s", text.c_str());
    check(text.find("G4OGR IO91 30 dBm (1 W)") != std::string::npos && text.find("14.09") != std::string::npos,
          tag + "readable lines");
    Status s = r.dec.status();
    check(s.timeValid && std::fabs(s.clockOffset) < 0.2 && s.locked, tag + "status: clock checked, locked",
          s.info);
}

// ---- the clock out by +-40 s ----
void clockCase(double err) {
    const double rate = 12000, amp = 0.01, noise = enc::wsprNoiseFor(amp, -20, rate);
    std::vector<enc::WsprStation> st = fourStations(amp);
    st.pop_back();
    Run r(rate, st, noise, 31);
    r.pcErr = err;
    r.t = kT0 - 110;
    // until a slot with decodes, at most 7 minutes (+40 s: the first window long enough to hold a whole
    // transmission at its start time comes 5 minutes in)
    double off = 0;
    int cnt = 0;
    double spread = 0;
    bool got = false;
    for (int k = 0; k < 14 && !got; ++k) {
        r.step(30);
        got = r.dec.measuredClockOffset(off, &cnt, &spread);
    }
    const std::string tag = fmt("clock %+.0f s: ", err);
    check(got && r.decodes.size() == 3 && std::fabs(off - err) < 0.2, tag + "decoded with the start-time search, offset measured",
          fmt("PC - true %.3f s from %.0f decodes (spread %.2f)", off, cnt, spread));
    // apply it (ClockCorrection::applyFtxTimings would take all of it: 2+ stations within 0.5 s) and go on a slot
    r.dec.setClockOffset(off);
    r.decodes.clear();
    r.step(130);
    double maxDt = 0;
    for (std::size_t i = 0; i < r.decodes.size(); ++i) maxDt = std::max(maxDt, std::fabs(r.decodes[i].dt));
    check(r.decodes.size() == 3 && maxDt < 0.2, tag + "corrected: next slot's dt ~0",
          fmt("%.0f decodes, max |dt| %.2f s", static_cast<double>(r.decodes.size()), maxDt));
}

// ---- a stick off frequency ----
void calibrationCase() {
    const double rate = 12000, amp = 0.01, noise = enc::wsprNoiseFor(amp, -18, rate), dial = 14.0956e6;
    // four stations around 1500 Hz (sorted: 1420, 1487.3, 1531.5, 1589.1; the JS's median is the upper middle one)
    Run r(rate, fourStations(amp, 150), noise, 51);
    r.dec.setDialFrequency(dial);
    r.t = kT0 - 110;
    r.step(110 + 125);
    WsprCalibration c = r.dec.calibration();
    const double expErr = 1531.5 + 150 - 1500, expPpm = expErr / dial * 1e6;
    check(r.decodes.size() == 4 && c.ready && c.offer && std::fabs(c.errorHz - expErr) < 0.5 && std::fabs(c.ppm - expPpm) < 0.05,
          "stick 150 Hz off: stations found, calibration offered",
          fmt("error %.2f Hz (expected %.2f), %.2f ppm (expected %.2f)", c.errorHz, expErr, c.ppm, expPpm) + c.text);
    // the program applies it (its correction moves by c.ppm): the stations come back to their places
    r.dec.calibrationApplied(0);
    std::vector<enc::WsprStation> back = fourStations(amp, 0);
    r.air = enc::WsprAir(rate, back, noise, 53);   // (the same decoder goes on, with the corrected audio)
    r.step(120);
    bool flipped = true;
    double sugg = 0;
    std::string text;
    const bool got = r.dec.takeCalibrationCheck(flipped, sugg, text);
    check(got && !flipped, "after the correction: confirmed", text);
}

// ---- drift ----
void driftCase() {
    const double rate = 12000, amp = 0.01, noise = enc::wsprNoiseFor(amp, -20, rate);
    std::vector<enc::WsprStation> st;
    st.push_back(enc::WsprStation("G4OGR", "IO91", 30, 1450, amp));
    st.back().driftHz = 2;
    st.push_back(enc::WsprStation("K1OGS", "FN42", 37, 1550, amp));
    st.back().driftHz = -2;
    Run r(rate, st, noise, 61);
    r.t = kT0 - 110;
    r.step(110 + 125);
    const WsprDecode* a = find(r.decodes, "G4OGR");
    const WsprDecode* b = find(r.decodes, "K1OGS");
    check(a && b && std::fabs(a->drift - 2) <= 1 && std::fabs(b->drift + 2) <= 1, "drifting stations (+2, -2 Hz): decoded, drift reported",
          a && b ? fmt("drift %+.0f, %+.0f Hz", a->drift, b->drift) : "not decoded");
}

// ---- noise ----
void noiseCase(double rate) {
    std::vector<enc::WsprStation> none;
    Run r(rate, none, 0.05, 71 + static_cast<unsigned>(rate));
    r.t = kT0 - 110;
    r.step(110 + 2 * 120 + 5);
    check(r.decodes.empty() && r.dec.slotsDecoded() == 2, fmt("noise alone at %.0f Hz: nothing in two slots", rate),
          fmt("%.0f decodes", static_cast<double>(r.decodes.size())));
}

// ---- NaN ----
void nanCase() {
    const double rate = 48000, amp = 0.01, noise = enc::wsprNoiseFor(amp, -20, rate);
    std::vector<enc::WsprStation> st = fourStations(amp);
    st.resize(2);
    Run r(rate, st, noise, 81);
    r.nanAt = kT0 + 40;
    r.t = kT0 - 110;
    // bad calls in between
    float junk[4] = {std::nanf(""), HUGE_VALF, -HUGE_VALF, 1e30f};
    r.dec.process(junk, 4, std::nan(""));
    r.dec.process(junk, 4, -1);
    r.dec.process(0, 100, 48000);
    r.dec.setTimestamp(std::nan(""));
    r.dec.setClockOffset(HUGE_VAL);
    r.step(110 + 125);
    bool finiteAll = true;
    for (std::size_t i = 0; i < r.decodes.size(); ++i)
        finiteAll = finiteAll && r.decodes[i].dt == r.decodes[i].dt && r.decodes[i].audioHz == r.decodes[i].audioHz;
    check(r.decodes.size() == 2 && finiteAll, "NaN/inf burst, bad rate, NaN timestamp: still decodes",
          fmt("%.0f decodes", static_cast<double>(r.decodes.size())));
    Status s = r.dec.status();
    check(s.info.find("nan") == std::string::npos, "status has no NaN", s.info);
}

}  // namespace

int main() {
    codecTests();
    fanoTests();
    slotCase(12000);
    slotCase(48000);
    clockCase(40);
    clockCase(-40);
    calibrationCase();
    driftCase();
    noiseCase(12000);
    noiseCase(48000);
    nanCase();
    check(g_maxCpu < 2.0, "CPU per slot decode under 2 s", fmt("max %.3f s, mean %.3f s over %.0f slots", g_maxCpu,
                                                                g_cpuN ? g_sumCpu / g_cpuN : 0, g_cpuN));
    std::printf("CPU  WSPR slot decode: max %.3f s, mean %.3f s (%d slots)\n", g_maxCpu, g_cpuN ? g_sumCpu / g_cpuN : 0, g_cpuN);
    std::printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
