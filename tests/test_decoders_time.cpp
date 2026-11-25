// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_decoders_time.cpp
//  Layer:  Tests (library liboffgrid-decoders)
//
//  What this file does
//  -------------------
//  Self-checking test of the time-signal decoder (Time.h) against the
//  generators in TimeEncoders.h, and of ClockCorrection (Clock.h).  Prints
//  PASS/FAIL per check and returns nonzero if anything failed.
//    - CHU, WWV, WWVH at 48000 and 12000 Hz, white noise (carrier 20 dB
//      above the noise in 3 kHz), the computer's clock -0.4 s and +2.3 s
//      out, timestamps given late by 0-30 ms at random, uneven blocks: the
//      time is read, Status::clockOffset within 10 ms of the truth, the
//      station named right (WWV vs WWVH), the date in the text line;
//    - every alignment of the decoder's 5 ms frames to the second (CHU,
//      RWM at 12000 Hz): the second's start within 3 ms;
//    - RWM (no time code): the minute marker gives the offset (clock
//      within 30 s) within 10 ms;
//    - CHU with the carrier 12 Hz off (a stick off frequency);
//    - WWV and WWVH together (the weaker 6 dB down, 15 ms later): the
//      stronger one named, its time read;
//    - lost samples: 0.5 s of CHU audio dropped (the timestamps go on):
//      the decoder restarts (timeJump) and reads the right time again, and
//      no wrong time is ever printed;
//    - noise alone (90 s, each station setting): no time;
//    - ClockCorrection: offset and now(), sourceText, drift from two
//      measurements a day apart (not from hand settings under 3 days),
//      FT8 timings, save/load round trip;
//    - CPU: seconds of 48 kHz audio decoded per CPU second, per station.
//
//  How it fits in
//  --------------
//  Build from the repository root (scratch output anywhere):
//    c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors -Isrc src/dsp/Fft.cpp \
//        src/decoders/Time.cpp src/decoders/Clock.cpp \
//        src/decoders/TimeEncoders.cpp tests/test_decoders_time.cpp -o time_t
//
//  Contributing
//  ------------
//  Add a check per new feature; keep the run under ~30 s.
// ============================================================================
#include "decoders/Clock.h"
#include "decoders/Time.h"
#include "decoders/TimeEncoders.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>

using namespace dec;

namespace {

int g_failures = 0, g_checks = 0;

void check(bool ok, const std::string& name, const std::string& detail = "") {
    ++g_checks;
    if (!ok) ++g_failures;
    std::printf("%s  %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : "  ", detail.c_str());
}

std::string fmt(const char* f, double a, double b = 0, double c = 0) {
    char buf[256];
    std::snprintf(buf, sizeof buf, f, a, b, c);
    return buf;
}

// noise rms for a carrier of amplitude A to be `snrDb` above the noise in 3 kHz
double noiseFor(double A, double snrDb, double rate) {
    return std::sqrt(A * A / 2 / std::pow(10, snrDb / 10) * (rate / 2) / 3000);
}

struct Cpu { double audio, cpu; Cpu() : audio(0), cpu(0) {} };

struct Run {
    std::string text;
    Status st;
    TimeReading full, bestNow;
    unsigned jumps;
    double endTrue;   // true UTC at the end of the audio
};

// Feeds `audio` (station time starting at T0) in uneven blocks, each with the computer's clock (true + pcErr,
// given 0-30 ms late at random). `dropAt` / `dropLen` (seconds of audio): samples lost, the timestamps go on.
Run feed(TimeDecoder& d, const std::vector<float>& audio, double rate, double T0, double pcErr, Cpu* cpu = 0,
         double dropAt = -1, double dropLen = 0) {
    static const std::size_t sizes[] = {1024, 4096, 333, 2048, 512, 3000, 777};
    unsigned rnd = 12345;
    std::size_t pos = 0, k = 0;
    const std::size_t dropA = dropAt >= 0 ? static_cast<std::size_t>(dropAt * rate) : audio.size() + 1;
    const std::size_t dropB = dropA + static_cast<std::size_t>(dropLen * rate);
    Run r;
    const std::clock_t c0 = std::clock();
    while (pos < audio.size()) {
        if (pos >= dropA && pos < dropB) { pos = dropB; continue; }
        std::size_t n = sizes[k++ % (sizeof sizes / sizeof sizes[0])] * static_cast<std::size_t>(rate) / 48000;
        if (n == 0) n = 1;
        if (pos < dropA && pos + n > dropA) n = dropA - pos;
        if (pos + n > audio.size()) n = audio.size() - pos;
        rnd = rnd * 1103515245u + 12345u;
        const double late = ((rnd >> 8) % 1000) / 1000.0 * 0.03;
        d.setTimestamp(T0 + pos / rate + pcErr + late);
        d.process(&audio[pos], n, rate);
        pos += n;
        std::string t;
        d.takeText(t);
        r.text += t;
    }
    if (cpu) { cpu->cpu += static_cast<double>(std::clock() - c0) / CLOCKS_PER_SEC; cpu->audio += audio.size() / rate; }
    r.st = d.status();
    r.full = d.lastFull();
    r.bestNow = d.best(0);
    r.jumps = d.timeJumps();
    r.endTrue = T0 + audio.size() / rate;
    return r;
}

// every "PC clock" value on a line with a decoded time ("UTC") in the text
std::vector<double> codeOffsets(const std::string& text) {
    std::vector<double> v;
    std::size_t p = 0;
    while (p < text.size()) {
        std::size_t e = text.find('\n', p);
        if (e == std::string::npos) e = text.size();
        const std::string line = text.substr(p, e - p);
        const std::size_t u = line.find(" UTC"), c = line.find("PC clock ");
        if (u != std::string::npos && c != std::string::npos) v.push_back(std::strtod(line.c_str() + c + 9, 0));
        p = e + 1;
    }
    return v;
}

void indent(const std::string& text) {
    std::size_t p = 0;
    while (p < text.size()) {
        std::size_t e = text.find('\n', p);
        if (e == std::string::npos) e = text.size();
        std::printf("      | %s\n", text.substr(p, e - p).c_str());
        p = e + 1;
    }
}


void stationCase(TimeStation enc, double rate, double pcErr, double startSec, double seconds, Cpu* cpu,
                 double carrierHz = 1000) {
    const double T0 = utcFromDayOfYear(2026, 329, 14, 2, 0) + startSec;
    enc::TimeSignalOptions o;
    o.level = 0.25; o.noiseRms = noiseFor(o.level, 20, rate); o.seed = 7 + static_cast<unsigned>(rate); o.carrierHz = carrierHz;
    const std::vector<float> audio = enc::timeSignalAudio(enc, T0, seconds, rate, o);
    TimeDecoder d;
    d.setStation(enc);
    const Run r = feed(d, audio, rate, T0, pcErr, cpu);
    const std::string name = std::string(timeStationName(enc)) + fmt(" %.0f Hz, PC clock %+.1f s", rate, pcErr) +
                             (carrierHz != 1000 ? fmt(", carrier %.0f Hz", carrierHz) : std::string());
    std::printf("  %s:\n", name.c_str());
    indent(r.text);
    const double err = r.st.clockOffset + pcErr;   // (Status: station - computer = -pcErr)
    if (enc == TimeStation::Rwm) {
        check(r.bestNow.level == TimeLevel::Minute, name + ": minute marker found", r.st.mode);
        check(std::fabs(err) <= 0.010, name + ": clockOffset within 10 ms", fmt("got %+.4f s, want %+.4f (error %+.1f ms)", r.st.clockOffset, -pcErr, err * 1000));
        check(r.st.station == "RWM", name + ": station", r.st.station);
        return;
    }
    check(r.st.timeValid, name + ": time code read", r.st.info);
    check(std::fabs(err) <= 0.010, name + ": clockOffset within 10 ms", fmt("got %+.4f s, want %+.4f (error %+.1f ms)", r.st.clockOffset, -pcErr, err * 1000));
    check(std::fabs(r.st.stationTime - r.endTrue) <= 0.015, name + ": stationTime", fmt("error %+.1f ms", (r.st.stationTime - r.endTrue) * 1000));
    check(r.st.station == timeStationName(enc), name + ": station identified", r.st.station);
    check(r.text.find("2026-11-25 14:0") != std::string::npos && r.text.find("(day 329)") != std::string::npos,
          name + ": date and time in the text");
    if (enc == TimeStation::Chu) check(r.text.find("year from the PC clock") == std::string::npos, name + ": year from the code (format B)");
    const std::vector<double> offs = codeOffsets(r.text);
    bool allOk = !offs.empty();
    for (std::size_t i = 0; i < offs.size(); ++i) allOk = allOk && std::fabs(offs[i] - pcErr) <= 0.011;
    check(allOk, name + ": every time line right", fmt("%.0f lines", static_cast<double>(offs.size())));
}

// WWV and WWVH on the same frequency, WWVH 6 dB weaker and 15 ms later, then the other way round: the stronger named
void mixedCase(bool wwvStronger) {
    const double rate = 48000, pcErr = -0.4, T0 = utcFromDayOfYear(2026, 329, 14, 2, 40);
    enc::TimeSignalOptions o, w;
    o.level = 0.2; o.noiseRms = noiseFor(0.2, 20, rate); w.level = 0.1;
    std::vector<float> a = enc::timeSignalAudio(wwvStronger ? TimeStation::Wwv : TimeStation::Wwvh, T0, 88, rate, o);
    const std::vector<float> b = enc::timeSignalAudio(wwvStronger ? TimeStation::Wwvh : TimeStation::Wwv, T0 - 0.015, 88, rate, w);
    for (std::size_t i = 0; i < a.size(); ++i) a[i] += b[i];
    TimeDecoder d;
    d.setStation(TimeStation::Wwv);
    const Run r = feed(d, a, rate, T0, pcErr);
    const std::string want = wwvStronger ? "WWV" : "WWVH", name = "WWV and WWVH together, " + want + " stronger";
    std::printf("  %s:\n", name.c_str());
    indent(r.text);
    check(r.st.timeValid && r.st.station == want, name + ": time read, station named", r.st.station);
    check(std::fabs(r.st.clockOffset + pcErr) <= 0.010, name + ": clockOffset within 10 ms", fmt("got %+.4f", r.st.clockOffset));
}

// the 5 ms frames at every alignment to the second (the clock out by -0.4 s plus 0..4.75 ms): the second's start found
// to within 3 ms each time (+-2.5 ms of frame plus noise; the browser's figure was 0-5 ms early)
void alignmentSweep() {
    const double rate = 12000;
    const TimeStation sts[] = {TimeStation::Chu, TimeStation::Rwm};
    for (int s = 0; s < 2; ++s) {
        double worst = 0, sum = 0;
        int n = 0, read = 0;
        for (int j = 0; j < 20; ++j, ++n) {
            const double pcErr = -0.4 - j * 0.00025, T0 = utcFromDayOfYear(2026, 329, 14, 2, sts[s] == TimeStation::Chu ? 50 : 20);
            enc::TimeSignalOptions o;
            o.noiseRms = noiseFor(o.level, 20, rate); o.seed = 50 + j;
            const std::vector<float> a = enc::timeSignalAudio(sts[s], T0, sts[s] == TimeStation::Chu ? 50 : 70, rate, o);
            TimeDecoder d;
            d.setStation(sts[s]);
            const Run r = feed(d, a, rate, T0, pcErr);
            if (r.bestNow.level < TimeLevel::Minute) continue;
            ++read;
            const double e = r.st.clockOffset + pcErr;
            sum += e; worst = std::max(worst, std::fabs(e));
        }
        check(read == n && worst <= 0.003, std::string("alignment sweep, ") + timeStationName(sts[s]) + ": within 3 ms at every alignment",
              fmt("%.0f of 20 read, mean %+.2f ms, worst %.2f ms", read, read ? sum / read * 1000 : 0, worst * 1000));
    }
}

void lostSamples() {
    const double rate = 48000, pcErr = -0.4;
    const double T0 = utcFromDayOfYear(2026, 329, 14, 2, 50);
    enc::TimeSignalOptions o;
    o.noiseRms = noiseFor(o.level, 20, rate);
    const std::vector<float> audio = enc::timeSignalAudio(TimeStation::Chu, T0, 110, rate, o);
    TimeDecoder d;
    d.setStation(TimeStation::Chu);
    // (0.5 s lost at 14:03:42, after the first minute's code; the next minute's code comes 50 s later)
    const Run r = feed(d, audio, rate, T0, pcErr, 0, 52, 0.5);
    std::printf("  CHU, 0.5 s of samples lost at 14:03:42:\n");
    indent(r.text);
    check(r.jumps >= 1, "lost samples: the jump is noticed (timeJump)", fmt("%.0f jumps", r.jumps));
    check(r.full.level == TimeLevel::Full && r.full.at > T0 + 60 + pcErr, "lost samples: the time read again after it",
          r.full.text);
    check(std::fabs(r.st.clockOffset + pcErr) <= 0.010, "lost samples: clockOffset within 10 ms after it",
          fmt("got %+.4f", r.st.clockOffset));
    const std::vector<double> offs = codeOffsets(r.text);
    bool allOk = offs.size() >= 2;
    for (std::size_t i = 0; i < offs.size(); ++i) allOk = allOk && std::fabs(offs[i] - pcErr) <= 0.011;
    check(allOk, "lost samples: no wrong time printed", fmt("%.0f time lines", static_cast<double>(offs.size())));
}

void noiseOnly() {
    const double rate = 48000, T0 = utcFromDayOfYear(2026, 329, 14, 2, 50);
    std::vector<float> audio(static_cast<std::size_t>(90 * rate), 0.f);
    enc::addTimeNoise(audio, 0.1, 99);
    const TimeStation sts[] = {TimeStation::Chu, TimeStation::Wwv, TimeStation::Rwm};
    for (int i = 0; i < 3; ++i) {
        TimeDecoder d;
        d.setStation(sts[i]);
        const Run r = feed(d, audio, rate, T0, 0.3);
        const std::string name = std::string("noise only, ") + timeStationName(sts[i]);
        check(!r.st.timeValid && r.text.find("UTC") == std::string::npos, name + ": no time", r.st.info);
        check(r.bestNow.level != TimeLevel::Minute && r.bestNow.level != TimeLevel::Full, name + ": no minute marker", r.st.mode);
        if (r.bestNow.level == TimeLevel::Ticks) std::printf("      (note: noise read as ticks: %s)\n", r.st.info.c_str());
    }
}

void clockTests() {
    const double t0 = utcFromDayOfYear(2026, 329, 14, 0, 0), day = 86400;
    ClockCorrection c;
    check(c.sourceText(t0) == "the computer's own, unchecked", "clock: unchecked source text", c.sourceText(t0));
    check(c.now(t0) == t0, "clock: no correction at first");
    // the computer 0.412 s fast, read from CHU
    c.set(0.412, "CHU 7850 kHz", ClockSetKind::Code, t0);
    check(std::fabs(c.now(t0 + 10) - (t0 + 10 - 0.412)) < 1e-6, "clock: now() takes the offset off", fmt("%+.6f", c.now(t0 + 10) - t0 - 10));
    check(c.sourceText(t0 + 300) == "set from CHU 7850 kHz, 5 min ago", "clock: sourceText", c.sourceText(t0 + 300));
    check(c.sourceText(t0 + 20) == "set from CHU 7850 kHz, just now", "clock: sourceText just now", c.sourceText(t0 + 20));
    check(c.sourceText(t0 + 5 * 3600) == "set from CHU 7850 kHz, 5 h ago", "clock: sourceText hours", c.sourceText(t0 + 5 * 3600));
    check(!c.driftKnown(), "clock: no drift from one measurement");
    // a day later it is 1.412 s fast: 1 s a day
    c.set(1.412, "WWV 10 MHz", ClockSetKind::Code, t0 + day);
    check(c.driftKnown() && std::fabs(c.driftPerDay() - 1.0) < 1e-6, "clock: drift from two measurements a day apart",
          fmt("%.4f s/day", c.driftPerDay()));
    check(std::fabs(c.offsetAt(t0 + 1.5 * day) - 1.912) < 1e-6, "clock: offsetAt follows the drift", fmt("%.4f", c.offsetAt(t0 + 1.5 * day)));
    const ClockAge ag = c.age(t0 + 4 * day);
    check(ag.known && std::fabs(ag.days - 3) < 1e-9 && std::fabs(ag.estimate - 1.1) < 1e-9 && !ag.stale, "clock: age",
          fmt("days %.2f est %.2f", ag.days, ag.estimate));
    // save / load
    const std::string saved = c.save();
    ClockCorrection c2;
    check(c2.load(saved) && c2.save() == saved && std::fabs(c2.offsetAt(t0 + 3 * day) - c.offsetAt(t0 + 3 * day)) < 1e-6 &&
              c2.sourceText(t0 + day + 600) == c.sourceText(t0 + day + 600) && c2.history().size() == 2,
          "clock: save/load round trip");
    check(!c2.load("not a clock\nx=1\n") && c2.save() == saved, "clock: load refuses other text");
    // by hand: two settings a day apart give no drift (3 days needed)
    ClockCorrection h;
    h.set(5, "your setting by hand", ClockSetKind::Hand, t0);
    h.set(7, "your setting by hand", ClockSetKind::Hand, t0 + day);
    check(!h.driftKnown(), "clock: no drift from hand settings a day apart");
    h.set(11, "your setting by hand", ClockSetKind::Hand, t0 + 4 * day);
    check(h.driftKnown() && std::fabs(h.driftPerDay() - 4.0 / 3) < 1e-6, "clock: drift from hand settings 3 days apart", fmt("%.3f", h.driftPerDay()));
    // FT8: three stations 0.5 s late on average: 80% of it
    ClockCorrection f;
    std::vector<double> dts;
    dts.push_back(0.4); dts.push_back(0.5); dts.push_back(0.7);
    check(f.applyFtxTimings(dts, "FT8", t0) && std::fabs(f.offsetAt(t0) - 0.4) < 1e-9, "clock: FT8 timings move it 80%",
          fmt("%.3f", f.offsetAt(t0)));
    check(f.sourceText(t0 + 60) == "set from FT8 stations, 1 min ago", "clock: FT8 source", f.sourceText(t0 + 60));
    std::vector<double> small(3, 0.1);
    check(!f.applyFtxTimings(small, "FT8", t0), "clock: FT8 timings under 0.15 s leave it");
}

}  // namespace

int main() {
    Cpu cpu[4];
    const TimeStation sts[] = {TimeStation::Chu, TimeStation::Wwv, TimeStation::Wwvh, TimeStation::Rwm};
    const double rates[] = {48000, 12000}, errs[] = {-0.4, 2.3};
    for (int s = 0; s < 4; ++s)
        for (int ri = 0; ri < 2; ++ri)
            for (int e = 0; e < 2; ++e) {
                const TimeStation st = sts[s];
                // CHU from :50 (its code at :31-:39 of the next minute); WWV / WWVH from :40 (a whole minute after
                // the next minute tone); RWM 70 s
                const double start = st == TimeStation::Chu ? 50 : st == TimeStation::Rwm ? 20 : 40;
                const double secs = st == TimeStation::Chu ? 50 : st == TimeStation::Rwm ? 70 : 88;
                stationCase(st, rates[ri], errs[e], start, secs, rates[ri] == 48000 ? &cpu[s] : 0);
            }
    stationCase(TimeStation::Chu, 48000, 0.7, 50, 50, 0, 1012);
    alignmentSweep();
    mixedCase(true);
    mixedCase(false);
    lostSamples();
    noiseOnly();
    clockTests();
    const char* names[] = {"CHU", "WWV", "WWVH", "RWM"};
    for (int s = 0; s < 4; ++s)
        std::printf("CPU  %-5s %.0f s of 48 kHz audio per CPU second\n", names[s], cpu[s].audio / cpu[s].cpu);
    std::printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
