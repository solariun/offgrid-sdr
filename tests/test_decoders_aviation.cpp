// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_decoders_aviation.cpp
//  Layer:  Tests (Library liboffgrid-decoders)
//
//  What this file does
//  -------------------
//  Self-checking test of the ACARS and VDL2 decoders against the encoders in
//  AviationEncoders.h.  Prints PASS/FAIL per check, returns nonzero on
//  failure.
//    - ACARS: the JS demo's four messages decode exactly (every field) at
//      48000 and 12000 Hz with white noise (Eb/N0 ~21 dB) and a DC offset,
//      fed in odd-sized blocks; the text lines carry registration, flight, label meaning and
//      text;
//    - ACARS: a bad CRC is rejected, two bit errors in one character (parity
//      still right) are rejected, a single bit error is repaired;
//    - VDL2: the JS demo's five bursts, three of them at the same moment on
//      136.975, 136.675 and 136.775 MHz, decode from IQ at 2.048 MS/s (and
//      2.4 MS/s) with noise (SNR ~23 dB in the 25 kHz channel; ~19 dB in a
//      second run) and a 250 Hz frequency offset; ACARS over AVLC
//      parsed (registration, flight, label, block, message number, text),
//      the acknowledgement and the ATN frame summarised, addresses and
//      countries; channel counts; setLocation(); channels out of view;
//    - nothing from noise alone (either decoder);
//    - NaN/Inf samples do not stop either decoder;
//    - CPU: VDL2 on 10 channels at 2.048 MS/s and ACARS at 48 kHz, as % of
//      a core (VDL2 must be real time with margin: < 50 %).
//
//  How it fits in
//  --------------
//  Build (from the repository root):
//    c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors -Isrc src/dsp/Fft.cpp \
//        src/decoders/Acars.cpp src/decoders/Vdl2.cpp \
//        src/decoders/AviationEncoders.cpp tests/test_decoders_aviation.cpp \
//        -o test_decoders_aviation
//
//  Contributing
//  ------------
//  Keep it deterministic (own noise generator, fixed seeds).  Algorithms
//  under test are ports from OFFgrid-SDR (diddy-boy).
// ============================================================================
#include "decoders/Acars.h"
#include "decoders/AviationEncoders.h"
#include "decoders/Vdl2.h"

#include <cmath>
#include <complex>
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
        s_ ^= s_ << 13;
        s_ ^= s_ >> 7;
        s_ ^= s_ << 17;
        return ((s_ >> 11) + 0.5) / 9007199254740992.0;
    }
    double gauss() {
        if (have_) {
            have_ = false;
            return spare_;
        }
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

typedef std::vector<std::complex<float> > Iq;

// ---------------------------------------------------------------------------
// ACARS
// ---------------------------------------------------------------------------
struct Expect {
    const char *reg, *ack, *label, *bid, *msgNo, *flight, *text;
    bool down;
};
const Expect kAcarsExpect[4] = {
    {"G-OFFG", "NAK", "H1", "3", "M01A", "OG0123", "OFFGRID-SDR DEMO. CRUISE FL350, FUEL 12.4T, ETA 1435Z", true},
    {"G-OFFG", "3", "_d", "A", "", "", "", false},
    {"G-OFFG", "NAK", "H1", "B", "", "", "WX REQ RESPONSE\nDEST 141420Z 24012KT 9999 SCT030 15/09 Q1018", false},
    {"", "NAK", "SQ", " ", "", "", "02XAOFFGRID DEMO GROUND STATION", false},
};

bool sameMsg(const dec::AcarsMessage& m, const Expect& e, std::string& why) {
    why.clear();
    if (m.mode != "2") why += " mode=" + m.mode;
    if (m.reg != e.reg) why += " reg=" + m.reg;
    if (m.ack != e.ack) why += " ack=" + m.ack;
    if (m.label != e.label) why += " label=" + m.label;
    if (m.blockId != e.bid) why += " bid=" + m.blockId;
    if (m.msgNo != e.msgNo) why += " msgNo=" + m.msgNo;
    if (m.flight != e.flight) why += " flight=" + m.flight;
    if (m.text != e.text) why += " text=" + printable(m.text);
    if (m.down != e.down) why += " down";
    return why.empty();
}

// The four demo messages, 0.3 s apart, with noise (sigma) and DC added.
std::vector<float> acarsSignal(double rate, double sigma, double dc, unsigned long long seed) {
    std::vector<float> a(static_cast<std::size_t>(0.3 * rate), 0.0f);
    const std::vector<dec::AcarsDemoItem> msgs = dec::acarsDemoMessages();
    for (std::size_t i = 0; i < msgs.size(); ++i) {
        dec::synthAcars(dec::acarsBits(msgs[i].msg), rate, a);
        a.resize(a.size() + static_cast<std::size_t>(0.3 * rate), 0.0f);
    }
    Noise nz(seed);
    for (std::size_t i = 0; i < a.size(); ++i) a[i] = static_cast<float>(0.5 * a[i] + sigma * nz.gauss() + dc);
    return a;
}

void feed(dec::AcarsDecoder& d, const std::vector<float>& a, double rate, std::size_t block) {
    for (std::size_t i = 0; i < a.size(); i += block) {
        const std::size_t n = a.size() - i < block ? a.size() - i : block;
        d.process(&a[i], n, rate);
    }
}

// sigma: the noise's standard deviation in the audio (white over 0..rate/2);
// the MSK peaks at 0.5.  0.1 at 48 kHz and 0.05 at 12 kHz are the same noise
// density: Eb/N0 about 21 dB.
void testAcarsRate(double rate, double sigma) {
    char name[160];
    const std::vector<float> a = acarsSignal(rate, sigma, 0.3, 7);
    dec::AcarsDecoder d;
    feed(d, a, rate, 997);
    std::vector<dec::AcarsMessage> got;
    d.takeMessages(got);
    std::snprintf(name, sizeof name, "ACARS @%.0f Hz, noise %.2f, DC 0.3: 4 demo messages decoded (ok %u, rejected %u)",
                  rate, sigma, d.okCount(), d.badCount());
    check(got.size() == 4, name);
    for (std::size_t i = 0; i < 4 && i < got.size(); ++i) {
        std::string why;
        const bool ok = sameMsg(got[i], kAcarsExpect[i], why);
        std::snprintf(name, sizeof name, "ACARS @%.0f Hz: message %u fields exact (%s, label %s, level %.1f dB)", rate,
                      static_cast<unsigned>(i + 1), got[i].reg.empty() ? "(no reg)" : got[i].reg.c_str(),
                      got[i].label.c_str(), got[i].levelDb);
        check(ok, name, why);
    }
    std::string text;
    d.takeText(text);
    if (rate == 48000) std::printf("---- ACARS text lines ----\n%s--------------------------\n", text.c_str());
    std::snprintf(name, sizeof name, "ACARS @%.0f Hz: text lines (registration, flight, label meaning, text)", rate);
    check(text.find("ACARS  G-OFFG  OG0123  H1 from aircraft  [M01A]  (message to or from the crew") != std::string::npos &&
              text.find("  OFFGRID-SDR DEMO. CRUISE FL350") != std::string::npos &&
              text.find("(no information (acknowledgement))") != std::string::npos &&
              text.find("ACARS  (no reg)  SQ to aircraft") != std::string::npos &&
              text.find("  DEST 141420Z") != std::string::npos,
          name);
}

void testAcarsErrors() {
    const double rate = 48000;
    const dec::AcarsTx m = dec::acarsDemoMessages()[0].msg;
    const std::vector<unsigned char> good = dec::acarsBits(m);
    const std::size_t nbytes = (good.size() - 128) / 8;
    // byte b, bit k (LSB first) -> index
#define BIT(b, k) (128 + 8 * (b) + (k))
    // (a) a CRC bit flipped
    {
        std::vector<unsigned char> bits = good;
        bits[BIT(nbytes - 3, 2)] ^= 1;
        std::vector<float> a(4800, 0.0f);
        dec::synthAcars(bits, rate, a);
        a.resize(a.size() + 4800, 0.0f);
        dec::AcarsDecoder d;
        d.process(&a[0], a.size(), rate);
        std::vector<dec::AcarsMessage> got;
        d.takeMessages(got);
        check(got.empty() && d.badCount() >= 1, "ACARS: bad CRC rejected");
    }
    // (b) two bits of one text character flipped: its parity still looks
    //     right, so it cannot be repaired; the CRC rejects it
    {
        std::vector<unsigned char> bits = good;
        bits[BIT(30, 0)] ^= 1;
        bits[BIT(30, 1)] ^= 1;
        std::vector<float> a(4800, 0.0f);
        dec::synthAcars(bits, rate, a);
        a.resize(a.size() + 4800, 0.0f);
        dec::AcarsDecoder d;
        d.process(&a[0], a.size(), rate);
        std::vector<dec::AcarsMessage> got;
        d.takeMessages(got);
        check(got.empty() && d.badCount() >= 1, "ACARS: two errors in one character (parity right) rejected");
    }
    // (c) one bit of a text character flipped: bad parity, repaired by the CRC
    {
        std::vector<unsigned char> bits = good;
        bits[BIT(30, 3)] ^= 1;
        std::vector<float> a(4800, 0.0f);
        dec::synthAcars(bits, rate, a);
        a.resize(a.size() + 4800, 0.0f);
        dec::AcarsDecoder d;
        d.process(&a[0], a.size(), rate);
        std::vector<dec::AcarsMessage> got;
        d.takeMessages(got);
        std::string why;
        check(got.size() == 1 && sameMsg(got[0], kAcarsExpect[0], why), "ACARS: a single bit error repaired (parity + CRC)",
              why);
    }
#undef BIT
}

void testAcarsNoiseAndNan() {
    const double rate = 48000;
    {
        Noise nz(99);
        std::vector<float> a(static_cast<std::size_t>(20 * rate));
        for (std::size_t i = 0; i < a.size(); ++i) a[i] = static_cast<float>(0.3 * nz.gauss());
        dec::AcarsDecoder d;
        feed(d, a, rate, 4096);
        std::string t;
        d.takeText(t);
        char name[96];
        std::snprintf(name, sizeof name, "ACARS: nothing from 20 s of noise (%u blocks rejected)", d.badCount());
        check(d.messages().empty() && t.empty(), name);
    }
    {
        std::vector<float> a = acarsSignal(rate, 0.05, 0, 3);
        for (std::size_t i = 1000; i < 1100; ++i)
            a[i] = i % 2 ? std::numeric_limits<float>::quiet_NaN() : std::numeric_limits<float>::infinity();
        dec::AcarsDecoder d;
        feed(d, a, rate, 1024);
        const float nan = std::numeric_limits<float>::quiet_NaN();
        d.process(&nan, 1, std::numeric_limits<double>::quiet_NaN());   // a bad rate: ignored
        check(d.messages().size() == 4, "ACARS: NaN/Inf samples and a NaN rate do not stop it (4 messages after them)");
    }
}

// ---------------------------------------------------------------------------
// VDL2
// ---------------------------------------------------------------------------
// The five demo bursts: three at the same moment on three channels, then the
// acknowledgement and the last on 136.975.  Returns the IQ (noise added).
Iq vdlSignal(double rate, double centerHz, double offsetHz, double amp, double sigma, double seconds,
             unsigned long long seed) {
    Iq iq(static_cast<std::size_t>(seconds * rate));
    const std::vector<dec::Vdl2DemoBurst> plan = dec::vdl2DemoBursts();
    const double at[5] = {0.25, 0.05, 0.05, 0.05, 0.40};   // plan order: 0 (975), 1 (975 ack), 2 (675), 3 (775), 4 (975)
    const double atAck = 0.15;
    for (std::size_t i = 0; i < plan.size(); ++i) {
        const double t = i == 1 ? atAck : at[i];
        dec::vdl2AddBurst(plan[i].frames, plan[i].freqHz + offsetHz, centerHz, rate, amp,
                          static_cast<std::size_t>(t * rate), iq);
    }
    Noise nz(seed);
    for (std::size_t i = 0; i < iq.size(); ++i)
        iq[i] += std::complex<float>(static_cast<float>(sigma * nz.gauss()), static_cast<float>(sigma * nz.gauss()));
    return iq;
}

void feedIq(dec::Vdl2Decoder& d, const Iq& iq, double rate, double center, std::size_t block) {
    for (std::size_t i = 0; i < iq.size(); i += block) {
        const std::size_t n = iq.size() - i < block ? iq.size() - i : block;
        d.processIq(&iq[i], n, rate, center, 136.975e6);
    }
}

const dec::Vdl2Frame* findSrc(const std::vector<dec::Vdl2Frame>& f, unsigned src) {
    for (std::size_t i = 0; i < f.size(); ++i)
        if (f[i].src == src) return &f[i];
    return 0;
}

void testVdl2Rate(double rate, bool verbose) {
    char name[200];
    const double center = 136.975e6 - rate / 4;   // the TUI: a quarter span below the dial
    const Iq iq = vdlSignal(rate, center, 250, 0.05, 0.02, 0.6, 11);
    dec::Vdl2Decoder d;
    feedIq(d, iq, rate, center, 65536);
    std::vector<dec::Vdl2Frame> f;
    d.takeFrames(f);
    std::snprintf(name, sizeof name,
                  "VDL2 @%.3f MS/s: 5 frames from 5 bursts on 3 channels, 3 at once (+250 Hz; %u bursts, %u RS/header, %u CRC failures)",
                  rate / 1e6, d.bursts(), d.rsFailures(), d.crcFailures());
    check(f.size() == 5, name);
    std::snprintf(name, sizeof name, "VDL2 @%.3f MS/s: per channel 136.975: %u, 136.675: %u, 136.775: %u (want 3, 1, 1)",
                  rate / 1e6, d.countOn(136.975e6), d.countOn(136.675e6), d.countOn(136.775e6));
    check(d.countOn(136.975e6) == 3 && d.countOn(136.675e6) == 1 && d.countOn(136.775e6) == 1, name);

    const dec::Vdl2Frame* a = findSrc(f, 0x400ABC);
    std::string why;
    if (!a) why = "missing";
    else {
        if (!a->isAcars || a->kind != 'I') why += " not ACARS";
        if (a->dst != 0x10A0FA || a->dstType != 4 || a->srcType != 1) why += " addresses";
        if (a->acars.reg != "G-OFFG") why += " reg=" + a->acars.reg;
        if (a->acars.flight != "OG0123") why += " flight=" + a->acars.flight;
        if (a->acars.label != "H1") why += " label=" + a->acars.label;
        if (a->acars.blockId != "1" || !a->acars.down) why += " block=" + a->acars.blockId;
        if (a->acars.msgNo != "M01A") why += " msgNo=" + a->acars.msgNo;
        if (a->acars.ack != "NAK") why += " ack=" + a->acars.ack;
        if (a->acars.text != "OFFGRID-SDR DEMO. VDL2 POSITION 5212N 00008E FL350") why += " text=" + a->acars.text;
        if (a->freqHz != 136.975e6) why += " channel";
        if (a->headLine().find("400ABC (UK)") == std::string::npos) why += " head=" + a->headLine();
    }
    std::snprintf(name, sizeof name, "VDL2 @%.3f MS/s: ACARS over AVLC parsed exactly (G-OFFG on 136.975)", rate / 1e6);
    check(why.empty(), name, why);

    const dec::Vdl2Frame* b = findSrc(f, 0x406F21);
    why.clear();
    if (!b || !b->isAcars || b->acars.reg != "G-EZXY" || b->acars.flight != "EZY12B" || b->acars.label != "16" ||
        b->acars.text != "121530,33000,1702, 154,N 51.512 E 0.123" || b->freqHz != 136.675e6)
        why = b ? b->headLine() + " | " + b->body() : "missing";
    std::snprintf(name, sizeof name, "VDL2 @%.3f MS/s: ACARS on 136.675 (G-EZXY EZY12B label 16) exact", rate / 1e6);
    check(why.empty(), name, why);

    const dec::Vdl2Frame* c = findSrc(f, 0x10A0FA);
    check(c && c->kind == 'S' && c->what == "Receive Ready" && c->dst == 0x400ABC &&
              c->body() == "Acknowledgement (Receive Ready)",
          std::string("VDL2: the ground's acknowledgement (S frame, Receive Ready)"), c ? c->headLine() : "missing");
    const dec::Vdl2Frame* e = findSrc(f, 0x408064);
    check(e && e->kind == 'I' && e->atn && !e->isAcars && e->dst == 0x2651D7 && e->dstType == 4 &&
              e->freqHz == 136.775e6,
          "VDL2: ATN data frame on 136.775 summarised", e ? e->headLine() + " | " + e->body() : "missing");
    const dec::Vdl2Frame* g = findSrc(f, 0x3C6544);
    check(g && g->isAcars && g->acars.reg == "D-AIBA" && g->acars.flight == "LH0920" && g->acars.text == "WX REQ EGSS EGLL" &&
              g->headLine().find("3C6544 (Germany)") != std::string::npos,
          "VDL2: D-AIBA LH0920 (country Germany from the address)", g ? g->headLine() : "missing");
    if (verbose) {
        std::string t;
        d.takeText(t);
        std::printf("---- VDL2 text lines ----\n%s-------------------------\n", t.c_str());
    }
}

void testVdl2Misc() {
    const double rate = 2048000, center = 136.975e6 - rate / 4;
    // nothing from noise alone (3 s)
    {
        Noise nz(5);
        Iq iq(static_cast<std::size_t>(3 * rate));
        for (std::size_t i = 0; i < iq.size(); ++i)
            iq[i] = std::complex<float>(static_cast<float>(0.02 * nz.gauss()), static_cast<float>(0.02 * nz.gauss()));
        dec::Vdl2Decoder d;
        feedIq(d, iq, rate, center, 16384);
        char name[128];
        std::snprintf(name, sizeof name, "VDL2: nothing from 3 s of noise (%u bursts looked at)", d.bursts());
        check(d.frameCount() == 0, name);
    }
    // setLocation(): Europe has 136.675/.775/.975; North America only .975 of them
    {
        const Iq iq = vdlSignal(rate, center, -150, 0.05, 0.02, 0.6, 12);
        dec::Vdl2Decoder eu, na;
        eu.setLocation(51.5, -0.1);
        na.setLocation(40.7, -74.0);
        feedIq(eu, iq, rate, center, 16384);
        feedIq(na, iq, rate, center, 16384);
        char name[160];
        std::snprintf(name, sizeof name, "VDL2: setLocation Europe (%u channels) 5 frames, North America (%u) 3 frames: %u, %u",
                      static_cast<unsigned>(eu.channels().size()), static_cast<unsigned>(na.channels().size()),
                      eu.frameCount(), na.frameCount());
        check(eu.channels().size() == 6 && na.channels().size() == 4 && eu.frameCount() == 5 && na.frameCount() == 3, name);
    }
    // channels out of view: the stick centred 2 MHz away
    {
        const Iq iq = vdlSignal(rate, center, 0, 0.05, 0.02, 0.6, 13);
        dec::Vdl2Decoder d;
        feedIq(d, iq, rate, 139e6, 16384);
        check(d.frameCount() == 0 && d.channelsInView() == 0, "VDL2: channels out of the IQ span are not decoded");
    }
    // NaN / Inf in the IQ, and bad arguments
    {
        Iq iq = vdlSignal(rate, center, 0, 0.05, 0.02, 0.6, 14);
        for (std::size_t i = 1000; i < 1200; ++i)
            iq[i] = std::complex<float>(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity());
        dec::Vdl2Decoder d;
        d.processIq(&iq[0], 10, std::numeric_limits<double>::quiet_NaN(), center, 0);
        d.processIq(&iq[0], 10, rate, std::numeric_limits<double>::infinity(), 0);
        feedIq(d, iq, rate, center, 16384);
        char name[128];
        std::snprintf(name, sizeof name, "VDL2: NaN/Inf samples and bad arguments do not stop it (%u frames)", d.frameCount());
        check(d.frameCount() == 5, name);
    }
    // weaker: amp 0.03, noise 0.02: about 19 dB signal to noise in the 25 kHz
    // channel (the decoder starts to lose bursts near 16 dB, as D8PSK does)
    {
        const Iq iq = vdlSignal(rate, center, 400, 0.03, 0.02, 0.6, 15);
        dec::Vdl2Decoder d;
        feedIq(d, iq, rate, center, 262144);
        char name[128];
        std::snprintf(name, sizeof name, "VDL2: weaker (SNR ~19 dB in 25 kHz, +400 Hz, 262144-sample blocks) 5 frames: %u",
                      d.frameCount());
        check(d.frameCount() == 5, name);
    }
}

void testCpu() {
    // VDL2: all 10 channels at 2.048 MS/s, 0.6 s of bursts + noise, fed 5 times
    {
        const double rate = 2048000, center = 136.975e6 - rate / 4;
        const Iq iq = vdlSignal(rate, center, 0, 0.05, 0.02, 0.6, 21);
        dec::Vdl2Decoder d;
        const std::clock_t t0 = std::clock();
        for (int r = 0; r < 5; ++r) feedIq(d, iq, rate, center, 262144);
        const double cpu = double(std::clock() - t0) / CLOCKS_PER_SEC, secs = 5 * 0.6;
        char name[200];
        std::snprintf(name, sizeof name,
                      "CPU: VDL2 %u channels @2.048 MS/s: %.1fx real time (%.1f%% of a core), %u frames",
                      static_cast<unsigned>(d.channelsInView()), secs / (cpu > 1e-9 ? cpu : 1e-9), 100 * cpu / secs,
                      d.frameCount());
        check(cpu / secs < 0.5 && d.frameCount() == 25, name);
    }
    // ACARS at 48 kHz
    {
        const std::vector<float> a = acarsSignal(48000, 0.1, 0, 8);
        dec::AcarsDecoder d;
        const std::clock_t t0 = std::clock();
        for (int r = 0; r < 5; ++r) feed(d, a, 48000, 4096);
        const double cpu = double(std::clock() - t0) / CLOCKS_PER_SEC, secs = 5.0 * a.size() / 48000;
        char name[160];
        std::snprintf(name, sizeof name, "CPU: ACARS @48 kHz: %.0fx real time (%.2f%% of a core)",
                      secs / (cpu > 1e-9 ? cpu : 1e-9), 100 * cpu / secs);
        check(cpu / secs < 0.2 && d.okCount() == 20, name);
    }
}

}  // namespace

int main() {
    testAcarsRate(48000, 0.1);
    testAcarsRate(12000, 0.05);
    testAcarsErrors();
    testAcarsNoiseAndNan();
    testVdl2Rate(2048000, true);
    testVdl2Rate(2400000, false);
    testVdl2Misc();
    testCpu();
    std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures ? 1 : 0;
}
