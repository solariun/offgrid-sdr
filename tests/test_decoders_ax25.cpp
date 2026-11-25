// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_decoders_ax25.cpp
//  Layer:  Tests (Library liboffgrid-decoders)
//
//  What this file does
//  -------------------
//  Self-checking test of the AX.25 / APRS decoder (Ax25.h) against the
//  encoders in Ax25Encoders.h.  Prints PASS/FAIL per check, returns nonzero
//  on failure:
//    - the 7 demo APRS packets decode byte-exact at 48000, 44100, 22050 and
//      12000 Hz in white noise (SNR 15 dB in 3 kHz), AFSK 1200;
//    - with the tones 2% high / low and the bit clock 2% fast / slow;
//    - through a de-emphasis-like low-pass (2200 Hz ~5 dB under 1200 Hz);
//    - frames with a wrong FCS (or a byte changed after the FCS) are not
//      shown;
//    - APRS fields for each kind: Mic-E, weather in a position, message,
//      status, object, uncompressed position, altitude, compressed position,
//      item, ack, bulletin, positionless weather, telemetry, third party;
//      digipeater H bits; distance and bearing from setHome(); the text;
//    - 300 Bd HF (1600/1800 Hz) at 48000 and 12000 Hz in noise, clock 2%
//      off and tuned 20 Hz off; G3RUH 9600 at 48000 Hz;
//    - no frames from 60 s of noise (1200, 300 and 9600);
//    - NaN / Inf samples do not poison the decoder; a rate change works;
//    - CPU: seconds of audio per CPU second.
//
//  How it fits in
//  --------------
//  Build (from the repository root):
//    c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors -Isrc \
//        src/decoders/Ax25*.cpp src/decoders/ax25/*.cpp \
//        tests/test_decoders_ax25.cpp -o test_decoders_ax25
//
//  Contributing
//  ------------
//  Keep it deterministic (own noise generator, fixed seeds).  The decoder
//  under test uses AX25Toolkit (Gustavo Campos) and Dire Wolf (WB2OSZ) DSP;
//  the APRS parser is a port of OFFgrid-SDR's JS (diddy-boy).
// ============================================================================
#include "decoders/Ax25.h"
#include "decoders/Ax25Encoders.h"

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

void addNoise(std::vector<float>& a, double sigma, unsigned long long seed) {
    Noise g(seed);
    for (std::size_t i = 0; i < a.size(); ++i) a[i] += static_cast<float>(sigma * g.gauss());
}

void silence(std::vector<float>& a, double seconds, double rate) {
    a.insert(a.end(), static_cast<std::size_t>(seconds * rate), 0.0f);
}

// Feeds in blocks of uneven size (as a sound card would).
void feed(dec::Ax25Decoder& d, const std::vector<float>& a, double rate) {
    const std::size_t blocks[] = {512, 1000, 37, 4096, 333};
    std::size_t i = 0, k = 0;
    while (i < a.size()) {
        std::size_t n = blocks[k++ % 5];
        if (i + n > a.size()) n = a.size() - i;
        d.process(&a[i], n, rate);
        i += n;
    }
}

std::vector<unsigned char> frameOf(const dec::AprsSample& s) {
    return dec::ax25UiFrame(s.source, s.destination, s.path, s.info);
}

bool sameInfo(const dec::Ax25Frame& f, const dec::AprsSample& s) {
    return f.source.str() == s.source && f.destination.str() == s.destination &&
           std::string(f.info.begin(), f.info.end()) == s.info && f.path.size() == s.path.size();
}

// The demo packets, 0.25 s apart, with noise; returns how many came back
// exactly (in order).
int demoRun(double rate, const dec::Ax25Tx& tx, double snrDb, unsigned long long seed, std::string* detail,
            int baud = 1200, double lowpassHz = 0) {
    const std::vector<dec::AprsSample> s = dec::aprsDemoPackets();
    std::vector<float> a;
    silence(a, 0.3, rate);
    for (std::size_t i = 0; i < s.size(); ++i) {
        dec::synthAx25(frameOf(s[i]), rate, a, tx);
        silence(a, 0.25, rate);
    }
    if (lowpassHz > 0) {   // one-pole low-pass, like an FM radio's de-emphasis
        const double k = 1 - std::exp(-6.283185307179586 * lowpassHz / rate);
        double y = 0;
        for (std::size_t i = 0; i < a.size(); ++i) { y += k * (a[i] - y); a[i] = static_cast<float>(y * 2.2); }
    }
    if (snrDb < 99) addNoise(a, noiseSigma(tx.amplitude, snrDb, rate), seed);
    dec::Ax25Decoder d;
    d.setBaud(baud);
    feed(d, a, rate);
    const std::vector<dec::Ax25Frame> f = d.frames();
    int good = 0;
    std::size_t j = 0;
    for (std::size_t i = 0; i < s.size() && j < f.size(); ++i)
        if (sameInfo(f[j], s[i])) { ++good; ++j; }
    if (detail) {
        char b[64];
        std::snprintf(b, sizeof b, "%d/%d packets, %u frames in all", good, static_cast<int>(s.size()),
                      d.frameCount());
        *detail = b;
    }
    return good + (f.size() != static_cast<std::size_t>(good) ? -100 : 0);   // extra frames: fail
}

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// Decodes one information field through the whole chain (frame bytes,
// no audio) and returns the report.
dec::Ax25Frame parse(const std::string& src, const std::string& dst, const std::vector<std::string>& path,
                     const std::string& info) {
    const std::vector<unsigned char> b = dec::ax25UiFrame(src, dst, path, info);
    dec::Ax25Frame f;
    dec::ax25ParseFrame(&b[0], b.size(), f);
    return f;
}

std::string b91(long v, int n) {
    std::string s(n, '!');
    for (int k = n - 1; k >= 0; --k) { s[k] = static_cast<char>(33 + v % 91); v /= 91; }
    return s;
}

}  // namespace

int main() {
    const dec::Ax25Tx vhf = dec::Ax25Tx::vhf1200();

    // ---- 1200 Bd at four rates, 15 dB SNR ----
    {
        const double rates[] = {48000, 44100, 22050, 12000};
        for (int r = 0; r < 4; ++r) {
            std::string det;
            const int g = demoRun(rates[r], vhf, 15, 100 + r, &det);
            char name[128];
            std::snprintf(name, sizeof name, "AFSK 1200 @%.0f Hz, SNR 15 dB: all 7 demo packets", rates[r]);
            check(g == 7, name, det);
        }
    }
    // ---- tone and clock errors ----
    {
        const double rates[] = {48000, 22050};
        const double scales[] = {1.02, 0.98};
        for (int r = 0; r < 2; ++r)
            for (int s = 0; s < 2; ++s) {
                dec::Ax25Tx t = vhf;
                t.toneScale = scales[s];
                std::string det;
                char name[128];
                std::snprintf(name, sizeof name, "AFSK 1200 @%.0f Hz, tones %+.0f%%, SNR 15 dB", rates[r],
                              (scales[s] - 1) * 100);
                check(demoRun(rates[r], t, 15, 200 + r * 2 + s, &det) == 7, name, det);
                t = vhf;
                t.clockScale = scales[s];
                std::snprintf(name, sizeof name, "AFSK 1200 @%.0f Hz, bit clock %+.0f%%, SNR 15 dB", rates[r],
                              (scales[s] - 1) * 100);
                check(demoRun(rates[r], t, 15, 300 + r * 2 + s, &det) == 7, name, det);
            }
    }
    // ---- de-emphasised audio ----
    {
        std::string det;
        check(demoRun(48000, vhf, 20, 400, &det, 1200, 1000) == 7,
              "AFSK 1200 @48000 Hz through a 1 kHz one-pole low-pass (de-emphasis), SNR 20 dB", det);
    }

    // ---- bad FCS ----
    {
        const std::vector<dec::AprsSample> s = dec::aprsDemoPackets();
        const double rate = 48000;
        std::vector<float> a;
        silence(a, 0.2, rate);
        // 1: the right bytes with a wrong FCS
        std::vector<unsigned char> f = frameOf(s[5]);
        const unsigned short fcs = dec::ax25Fcs(&f[0], f.size());
        std::vector<unsigned char> bad = f;
        bad.push_back(static_cast<unsigned char>((fcs & 0xff) ^ 0x01));
        bad.push_back(static_cast<unsigned char>(fcs >> 8));
        dec::synthAx25Bits(dec::ax25HdlcBits(bad, 40, 3, false), rate, a, vhf);
        silence(a, 0.2, rate);
        // 2: one information byte changed after the FCS was computed
        bad = f;
        bad.push_back(static_cast<unsigned char>(fcs & 0xff));
        bad.push_back(static_cast<unsigned char>(fcs >> 8));
        bad[20] ^= 0x04;
        dec::synthAx25Bits(dec::ax25HdlcBits(bad, 40, 3, false), rate, a, vhf);
        silence(a, 0.2, rate);
        // 3: the same with the right FCS supplied by hand: must decode
        bad = f;
        bad.push_back(static_cast<unsigned char>(fcs & 0xff));
        bad.push_back(static_cast<unsigned char>(fcs >> 8));
        dec::synthAx25Bits(dec::ax25HdlcBits(bad, 40, 3, false), rate, a, vhf);
        silence(a, 0.2, rate);
        addNoise(a, noiseSigma(0.5, 25, rate), 501);
        dec::Ax25Decoder d;
        feed(d, a, rate);
        const std::vector<dec::Ax25Frame> fr = d.frames();
        check(fr.size() == 1 && sameInfo(fr[0], s[5]), "wrong FCS / changed byte rejected, right FCS accepted",
              fr.size() == 1 ? "" : "frames: " + std::to_string(fr.size()));
    }

    // ---- APRS fields of each demo packet, via audio, with text ----
    {
        const std::vector<dec::AprsSample> s = dec::aprsDemoPackets();
        const double rate = 44100;
        std::vector<float> a;
        silence(a, 0.2, rate);
        for (std::size_t i = 0; i < s.size(); ++i) {
            dec::synthAx25(frameOf(s[i]), rate, a, vhf);
            silence(a, 0.2, rate);
        }
        addNoise(a, noiseSigma(0.5, 20, rate), 600);
        dec::Ax25Decoder d;
        d.setHome(53.4808, -2.2426);   // Manchester
        feed(d, a, rate);
        std::vector<dec::Ax25Frame> f;
        d.takeFrames(f);
        std::string text;
        d.takeText(text);
        std::printf("---- decoded text ----\n%s----------------------\n", text.c_str());
        check(f.size() == 7, "demo packets via audio (44100 Hz, SNR 20 dB)", std::to_string(f.size()) + " frames");
        if (f.size() == 7) {
            const dec::AprsReport& car = f[0].aprs;
            check(car.kind == dec::AprsKind::Position && near(car.lat, 53.4808, 2e-4) && near(car.lon, -2.2426, 2e-4) &&
                      car.speedKmh == 61 && car.courseDeg == 135 && car.micE == "En route" && car.symCode == '>' &&
                      car.symTable == '/' && car.symbolName == "car" && car.comment == "OFFgrid-SDR demo car" &&
                      near(car.distanceKm, 0, 0.05),
                  "Mic-E car: position, 61 km/h, 135 deg, En route, car, comment, distance 0",
                  dec::aprsDescribe(car));
            const dec::AprsReport& wx = f[1].aprs;
            check(wx.kind == dec::AprsKind::Weather && wx.timestamp == "221815z" && near(wx.lat, 53.475, 1e-6) &&
                      near(wx.lon, -2.23, 1e-6) && wx.wx.windDirDeg == 270 && wx.wx.windKmh == 19 &&
                      wx.wx.gustKmh == 29 && near(wx.wx.tempC, 12.2, 1e-9) && wx.wx.rain1hMm == 0 &&
                      wx.wx.humidityPct == 78 && near(wx.wx.pressureHpa, 1014.2, 1e-9) &&
                      wx.comment == "OFFgrid-SDR demo weather" && wx.symbolName == "weather station",
                  "weather: wind 270/19 g29 km/h, 12.2 C, 78%, 1014.2 hPa", dec::aprsDescribe(wx));
            const dec::AprsReport& msg = f[2].aprs;
            check(msg.kind == dec::AprsKind::Message && msg.to == "M0OGR-9" &&
                      msg.text == "See you at the radio club tonight" && msg.msgId == "17",
                  "message: to, text, id", dec::aprsDescribe(msg));
            const dec::AprsReport& st = f[3].aprs;
            check(st.kind == dec::AprsKind::Status && st.text == "OFFgrid-SDR demo digipeater, 144.800 MHz" &&
                      f[3].path.empty(),
                  "status: text, no path", dec::aprsDescribe(st));
            const dec::AprsReport& ob = f[4].aprs;
            check(ob.kind == dec::AprsKind::Object && ob.name == "GB3OG" && !ob.killed && near(ob.lat, 53.5, 1e-9) &&
                      near(ob.lon, -2.25, 1e-9) && ob.symCode == 'r' &&
                      ob.comment == "145.6000MHz T082 R40k demo repeater",
                  "object: name, position, repeater, comment", dec::aprsDescribe(ob));
            const dec::AprsReport& wk = f[5].aprs;
            check(wk.kind == dec::AprsKind::Position && near(wk.lat, 51.44, 1e-9) && near(wk.lon, -0.1216667, 1e-6) &&
                      wk.symbolName == "walker" && wk.comment == "Out walking on the hills" &&
                      near(wk.distanceKm, 269, 1) && near(wk.bearingDeg, 147, 1),
                  "walker: position, symbol, comment, 269 km bearing 147 deg from Manchester", dec::aprsDescribe(wk));
            const dec::AprsReport& bl = f[6].aprs;
            check(bl.kind == dec::AprsKind::Position && bl.altitudeM == 9510 && bl.courseDeg == 0 && bl.speedKmh == 83 &&
                      bl.symbolName == "balloon" && bl.timestamp == "221815h" && bl.comment == "OFFgrid-SDR demo balloon",
                  "balloon: altitude 9510 m, 83 km/h, course 0", dec::aprsDescribe(bl));
            const std::string first = text.substr(0, text.find('\n'));
            check(first == f[0].monitorLine() && first.compare(0, 8, "M0OGR-9>") == 0 &&
                      first.find(",WIDE1-1,WIDE2-1: `") != std::string::npos,
                  "text: the monitor line first", first);
            check(text.find("\n  APRS weather \xC2\xB7 53.4750\xC2\xB0N 2.2300\xC2\xB0W \xC2\xB7 12.2\xC2\xB0" "C") !=
                      std::string::npos,
                  "text: the APRS line under each packet");
        }
        const dec::Status stt = d.status();
        check(stt.characters == 7 && stt.info.find("AFSK 1200  7 frames") == 0 && stt.baud == 1200, "status",
              stt.info);
    }

    // ---- APRS parser, kinds not in the demo ----
    {
        std::vector<std::string> none, digi;
        digi.push_back("WIDE1-1*");
        digi.push_back("WIDE2-1");
        dec::Ax25Frame f = parse("N0CALL-9", "APRS", digi, "!4903.50N/07201.75W-Test /A=001234");
        check(f.path.size() == 2 && f.path[0].repeated && !f.path[1].repeated &&
                  f.monitorLine() == "N0CALL-9>APRS,WIDE1-1*,WIDE2-1: !4903.50N/07201.75W-Test /A=001234",
              "digipeater H bits and monitor line", f.monitorLine());
        check(f.aprs.kind == dec::AprsKind::Position && near(f.aprs.lat, 49.0583333, 1e-6) &&
                  near(f.aprs.lon, -72.0291667, 1e-6) && f.aprs.altitudeM == 376 && f.aprs.comment == "Test" &&
                  f.aprs.symbolName == "house",
              "uncompressed position with altitude", dec::aprsDescribe(f.aprs));
        // compressed: 49.5 N 72.75 W, course 88, speed 30 -> (1.08^30 - 1) kn
        const std::string cmp = "=/" + b91(static_cast<long>(380926 * (90 - 49.5)), 4) +
                                b91(static_cast<long>(190463 * (180 - 72.75)), 4) + ">7?!Compressed";
        f = parse("N0CALL", "APRS", none, cmp);
        check(f.aprs.kind == dec::AprsKind::Position && near(f.aprs.lat, 49.5, 1e-4) && near(f.aprs.lon, -72.75, 1e-4) &&
                  f.aprs.courseDeg == 88 && f.aprs.speedKmh == 17 && f.aprs.symCode == '>' &&
                  f.aprs.comment == "Compressed",
              "compressed position, course 88, 17 km/h", dec::aprsDescribe(f.aprs));
        f = parse("N0CALL", "APRS", none, ")AID #2!4903.50N/07201.75WA");
        check(f.aprs.kind == dec::AprsKind::Item && f.aprs.name == "AID #2" && near(f.aprs.lat, 49.0583333, 1e-6),
              "item", dec::aprsDescribe(f.aprs));
        f = parse("N0CALL", "APRS", none, ";LEADER   _092345z4903.50N/07201.75W>");
        check(f.aprs.kind == dec::AprsKind::Object && f.aprs.killed && f.aprs.name == "LEADER" &&
                  dec::aprsWhat(f.aprs) == "object (gone)",
              "killed object", dec::aprsWhat(f.aprs));
        f = parse("N0CALL", "APRS", none, ":KB2ICI-14:ack003");
        check(f.aprs.kind == dec::AprsKind::Ack && f.aprs.to == "KB2ICI-14" && f.aprs.text == "ack003", "ack");
        f = parse("N0CALL", "APRS", none, ":BLN1     :Net tonight at 8");
        check(f.aprs.kind == dec::AprsKind::Bulletin && f.aprs.to == "BLN1" && f.aprs.text == "Net tonight at 8",
              "bulletin");
        f = parse("N0CALL", "APRS", none, "_10090556c220s004g005t077r001p000P000h50b09900wRSW");
        check(f.aprs.kind == dec::AprsKind::Weather && f.aprs.wx.windDirDeg == 220 && f.aprs.wx.windKmh == 6 &&
                  f.aprs.wx.gustKmh == 8 && f.aprs.wx.tempC == 25 && f.aprs.wx.humidityPct == 50 &&
                  near(f.aprs.wx.pressureHpa, 990.0, 1e-9) && near(f.aprs.wx.rain1hMm, 0.3, 1e-9) &&
                  f.aprs.comment == "wRSW",
              "positionless weather", dec::aprsDescribe(f.aprs));
        f = parse("N0CALL", "APRS", none, "T#005,199,000,255,073,123,01101001");
        check(f.aprs.kind == dec::AprsKind::Telemetry, "telemetry (named)");
        f = parse("IGATE", "APRS", none, "}W1AW-5>APRS,TCPIP,IGATE*:>Hello from the internet");
        check(f.aprs.kind == dec::AprsKind::Status && f.aprs.text == "Hello from the internet" &&
                  f.aprs.thirdParty == "IGATE" && f.aprs.innerSource == "W1AW-5",
              "third party", dec::aprsDescribe(f.aprs));
        std::string md, mi;
        dec::aprsMicEEncode(-33.8688, 151.2093, 0, 0, '[', '/', 7, md, mi);
        f = parse("VK2OG", md, none, mi + "]Walking");
        check(f.aprs.kind == dec::AprsKind::Position && near(f.aprs.lat, -33.8688, 2e-4) &&
                  near(f.aprs.lon, 151.2093, 2e-4) && f.aprs.micE == "Off duty" && f.aprs.comment == "Walking" &&
                  dec::aprsDescribe(f.aprs).find("stopped") != std::string::npos,
              "Mic-E south/east, >100 deg, stopped, Off duty", dec::aprsDescribe(f.aprs));
        f = parse("N0CALL", "APRS", none, "#not something the parser knows");
        check(f.aprs.kind == dec::AprsKind::Other && dec::aprsDescribe(f.aprs) == "#not something the parser knows",
              "other: raw field shown");
        const unsigned char junk[] = {0x82, 0xA0, 0xA4, 0xA6, 0x40, 0x40, 0xE0, 0x9C, 0x60, 0x86, 0x82, 0x98, 0x98, 0x61,
                                      0x3F, 0xF0, 'h', 'i'};
        f = dec::Ax25Frame();
        check(dec::ax25ParseFrame(junk, sizeof junk, f) && f.frameType == "SABM" && f.aprs.kind == dec::AprsKind::None &&
                  f.monitorLine().find("<SABM>") != std::string::npos,
              "non-UI frame kept, not APRS", f.monitorLine());
        const unsigned char lower[] = {0xC2, 0xA0, 0xA4, 0xA6, 0x40, 0x40, 0xE0, 0x9C, 0x60, 0x86, 0x82, 0x98, 0x98, 0x61,
                                       0x03, 0xF0, 'h', 'i'};
        check(!dec::ax25ParseFrame(lower, sizeof lower, f), "a callsign with a lower-case letter is refused");
    }

    // ---- 300 Bd HF ----
    {
        const dec::Ax25Tx hf = dec::Ax25Tx::hf300();
        const double rates[] = {48000, 12000};
        for (int r = 0; r < 2; ++r) {
            std::string det;
            char name[128];
            std::snprintf(name, sizeof name, "AFSK 300 (HF) @%.0f Hz, SNR 15 dB", rates[r]);
            check(demoRun(rates[r], hf, 15, 700 + r, &det, 300) == 7, name, det);
        }
        dec::Ax25Tx t = hf;
        t.clockScale = 1.02;
        std::string det;
        check(demoRun(22050, t, 15, 710, &det, 300) == 7, "AFSK 300 @22050 Hz, bit clock +2%, SNR 15 dB", det);
        t = hf;
        t.toneScale = 1 + 20.0 / 1700;
        check(demoRun(44100, t, 15, 711, &det, 300) == 7, "AFSK 300 @44100 Hz, tuned 20 Hz off, SNR 15 dB", det);
        t = hf;
        t.toneScale = 1 - 20.0 / 1700;
        check(demoRun(44100, t, 15, 712, &det, 300) == 7, "AFSK 300 @44100 Hz, tuned -20 Hz off, SNR 15 dB", det);
        check(demoRun(48000, vhf, 30, 713, &det, 300) <= 0, "AFSK 300 decoder ignores AFSK 1200", det);
    }

    // ---- 9600 G3RUH ----
    {
        const dec::Ax25Tx g = dec::Ax25Tx::g3ruh9600();
        std::string det;
        const std::vector<dec::AprsSample> s = dec::aprsDemoPackets();
        std::vector<float> a;
        const double rate = 48000;
        silence(a, 0.1, rate);
        for (std::size_t i = 0; i < s.size(); ++i) {
            dec::synthAx25(frameOf(s[i]), rate, a, g);
            silence(a, 0.05, rate);
        }
        addNoise(a, 0.5 * 0.15, 800);   // ~16 dB in the full 24 kHz
        dec::Ax25Decoder d;
        d.setBaud(9600);
        feed(d, a, rate);
        check(d.frameCount() == 7 && d.status().mode == "G3RUH 9600", "G3RUH 9600 @48000 Hz in noise",
              std::to_string(d.frameCount()) + " frames");
        dec::Ax25Decoder lo;
        lo.setBaud(9600);
        std::vector<float> b(22050, 0.0f);
        lo.process(&b[0], b.size(), 22050);
        check(lo.status().info.find("needs") != std::string::npos, "9600 at 22050 Hz says it needs more",
              lo.status().info);
    }

    // ---- no frames from noise ----
    {
        const int bauds[] = {1200, 300, 9600};
        for (int k = 0; k < 3; ++k) {
            const double rate = 48000;
            std::vector<float> a(static_cast<std::size_t>(60 * rate));
            Noise g(900 + k);
            for (std::size_t i = 0; i < a.size(); ++i) a[i] = static_cast<float>(0.2 * g.gauss());
            dec::Ax25Decoder d;
            d.setBaud(bauds[k]);
            feed(d, a, rate);
            std::string t;
            d.takeText(t);
            char name[96];
            std::snprintf(name, sizeof name, "no frames from 60 s of noise (%d Bd)", bauds[k]);
            check(d.frameCount() == 0 && t.empty(), name, std::to_string(d.frameCount()) + " frames");
        }
    }

    // ---- NaN / Inf, rate change ----
    {
        const std::vector<dec::AprsSample> s = dec::aprsDemoPackets();
        const double rate = 48000;
        std::vector<float> a;
        silence(a, 0.2, rate);
        dec::synthAx25(frameOf(s[3]), rate, a, vhf);
        silence(a, 0.2, rate);
        const std::size_t mid = a.size() / 2;
        a[mid] = std::numeric_limits<float>::quiet_NaN();
        a[mid + 1] = std::numeric_limits<float>::infinity();
        a[mid + 2] = -std::numeric_limits<float>::infinity();
        a[mid + 3] = 1e30f;
        std::vector<float> b(a.begin() + mid + 4, a.end());   // the rest, then a clean packet
        dec::synthAx25(frameOf(s[2]), rate, b, vhf);
        silence(b, 0.2, rate);
        dec::Ax25Decoder d;
        d.process(&a[0], mid + 4, rate);
        d.process(&b[0], b.size(), rate);
        std::vector<dec::Ax25Frame> f = d.frames();
        check(!f.empty() && sameInfo(f.back(), s[2]), "NaN / Inf / 1e30 samples: the next packet decodes",
              std::to_string(f.size()) + " frames");
        std::vector<float> c;
        silence(c, 0.1, 22050);
        dec::synthAx25(frameOf(s[5]), 22050, c, vhf);
        silence(c, 0.2, 22050);
        d.process(&c[0], c.size(), 22050);
        f = d.frames();
        check(!f.empty() && sameInfo(f.back(), s[5]), "rate change 48000 -> 22050 Hz");
        d.process(&c[0], c.size(), std::numeric_limits<double>::quiet_NaN());
        d.process(&c[0], c.size(), -1);
        check(d.frameCount() == f.size(), "NaN / negative rate ignored");
        d.reset();
        check(d.frameCount() == 0 && d.frames().empty(), "reset");
    }

    // ---- CPU ----
    {
        const int bauds[] = {1200, 300, 9600};
        const double rates[] = {48000, 44100};
        for (int k = 0; k < 3; ++k)
            for (int r = 0; r < 2; ++r) {
                const double rate = rates[r], secs = 60;
                std::vector<float> a(static_cast<std::size_t>(secs * rate));
                Noise g(1000 + k);
                for (std::size_t i = 0; i < a.size(); ++i) a[i] = static_cast<float>(0.2 * g.gauss());
                dec::Ax25Decoder d;
                d.setBaud(bauds[k]);
                const std::clock_t t0 = std::clock();
                feed(d, a, rate);
                const double cpu = double(std::clock() - t0) / CLOCKS_PER_SEC;
                char name[128];
                std::snprintf(name, sizeof name, "CPU: AX.25 %d Bd @%.0f Hz%s: %.0fx real time (%.2f%% of a core)",
                              bauds[k], rate, bauds[k] == 9600 ? "" : " (5 slicers)", secs / (cpu > 1e-9 ? cpu : 1e-9), 100 * cpu / secs);
                check(cpu < secs / 10, name);
            }
    }

    std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures ? 1 : 0;
}
