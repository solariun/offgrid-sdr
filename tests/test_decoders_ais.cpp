// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_decoders_ais.cpp
//  Layer:  Tests (Library liboffgrid-decoders)
//
//  What this file does
//  -------------------
//  Self-checking test of the AIS decoder (Ais.h) against the encoders in
//  AisEncoders.h.  Prints PASS/FAIL per check, returns nonzero on failure:
//    - every message builder parses back field by field (types 1, 4, 5, 18,
//      19 via a hand-built frame, 21, 24 A/B, 27, an "other" type);
//      invalid MMSI 0 / type 0 rejected; the names for numbers;
//    - !AIVDM sentences: 6-bit armoring decodes back to the bits, checksum,
//      fill bits, two-sentence type 5; a published sentence parses;
//    - IQ at 2.048 and 2.4 MS/s, the stick centred on 162.000 MHz, bursts on
//      both channels at once (overlapping in time), in noise (SNR 18 dB in
//      20 kHz), with the stick 0 and ±1 kHz off frequency: every message,
//      every field; the share decoded at 12, 14 and 16 dB;
//    - a burst with a bad CRC is not shown;
//    - the vessel table merges a type 1 and a type 5 from one MMSI (and a
//      class B's 18 + 24A + 24B); distance and bearing from setHome();
//    - the demo plan (30 s cycle) decodes completely at 240 kS/s;
//    - nothing from 30 s of noise; NaN / Inf samples do no harm; a rate
//      change works; a channel out of view is reported;
//    - CPU: both channels at 2.048 MS/s, % of a core.
//
//  How it fits in
//  --------------
//  Build (from the repository root):
//    c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors -Isrc \
//        src/dsp/Fft.cpp src/decoders/Ais.cpp src/decoders/AisEncoders.cpp \
//        tests/test_decoders_ais.cpp -o test_decoders_ais
//
//  Contributing
//  ------------
//  Keep it deterministic (own noise generator, fixed seeds).  The decoder
//  and encoders are ports of OFFgrid-SDR's JS (diddy-boy).
// ============================================================================
#include "decoders/Ais.h"
#include "decoders/AisEncoders.h"

#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
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

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

std::string str(const char* f, double a) {
    char b[128];
    std::snprintf(b, sizeof b, f, a);
    return b;
}

// Deterministic Gaussian noise (xorshift + Box-Muller).
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
        const double u = uniform(), v = uniform(), r = std::sqrt(-2 * std::log(u)), a = 6.283185307179586 * v;
        spare_ = r * std::sin(a);
        have_ = true;
        return r * std::cos(a);
    }

private:
    unsigned long long s_;
    bool have_;
    double spare_;
};

typedef std::complex<float> cf;

// Feeds the decoder in chunks (as a program would).
void feed(dec::AisDecoder& d, const std::vector<cf>& iq, double rate, double center, std::size_t chunk = 16384) {
    for (std::size_t i = 0; i < iq.size(); i += chunk) {
        const std::size_t n = std::min(chunk, iq.size() - i);
        d.processIq(&iq[i], n, rate, center, center);
    }
}

// A de-armoring of an !AIVDM payload back to bits, independent of Ais.cpp.
bool dearmor(const std::string& payload, int fill, std::vector<unsigned char>& bits) {
    for (std::size_t i = 0; i < payload.size(); ++i) {
        int c = static_cast<unsigned char>(payload[i]);
        if (c < 48 || c > 119 || (c > 87 && c < 96)) return false;
        c -= 48;
        if (c > 40) c -= 8;
        for (int k = 5; k >= 0; --k) bits.push_back(static_cast<unsigned char>((c >> k) & 1));
    }
    for (int k = 0; k < fill && !bits.empty(); ++k) bits.pop_back();
    return true;
}

// Splits "!AIVDM,a,b,c,d,payload,fill*HH" and checks the checksum.
bool splitNmea(const std::string& s, std::vector<std::string>& f, bool& csOk) {
    f.clear();
    csOk = false;
    if (s.size() < 10 || s[0] != '!') return false;
    const std::size_t star = s.find('*');
    if (star == std::string::npos || star + 3 != s.size()) return false;
    unsigned c = 0;
    for (std::size_t i = 1; i < star; ++i) c ^= static_cast<unsigned char>(s[i]);
    unsigned got = 0;
    std::sscanf(s.c_str() + star + 1, "%2X", &got);
    csOk = got == c;
    std::string cur;
    for (std::size_t i = 1; i < star; ++i) {
        if (s[i] == ',') {
            f.push_back(cur);
            cur.clear();
        } else {
            cur += s[i];
        }
    }
    f.push_back(cur);
    return f.size() == 7;
}

std::vector<unsigned char> bitsOf(const std::vector<unsigned char>& bytes) {
    std::vector<unsigned char> b;
    for (std::size_t i = 0; i < bytes.size(); ++i)
        for (int k = 7; k >= 0; --k) b.push_back(static_cast<unsigned char>((bytes[i] >> k) & 1));
    return b;
}

// ---------------------------------------------------------------------------
// The test messages
// ---------------------------------------------------------------------------
struct Sent {
    double t;          // seconds into the buffer
    char ch;
    std::vector<unsigned char> msg;
};

const unsigned kShip = 235012345, kPilot = 232004567, kRover = 250001234, kYacht = 235098765, kBase = 2320001,
               kAton = 992351234, kFar = 366123456;

std::vector<Sent> testPlan() {
    std::vector<Sent> p;
    // Both channels at once: pairs overlap in time.
    p.push_back(Sent{0.050, 'A', dec::aisPositionMsg(kShip, 53.5210, -3.3520, 12.3, 285.4, 284, 0)});
    p.push_back(Sent{0.060, 'B', dec::aisStaticMsg(kShip, 9876543, "MOGR7", "OFFGRID EXPLORER", 70, 128, 20, 7.4,
                                                   "DUBLIN", 11, 2, 14, 30)});
    p.push_back(Sent{0.150, 'B', dec::aisPositionMsg(kRover, -33.8568, 151.2153, 0.1, 0, 211, 1, 3)});
    p.push_back(Sent{0.155, 'A', dec::aisBaseMsg(kBase, 53.3100, -4.6300, 2026, 11, 25, 9, 8, 7)});
    p.push_back(Sent{0.250, 'A', dec::aisClassBMsg(kYacht, 53.4020, -3.0470, 5.6, 210.0, 211)});
    p.push_back(Sent{0.255, 'B', dec::aisStaticBNameMsg(kYacht, "WINDFALL")});
    p.push_back(Sent{0.350, 'B', dec::aisStaticBTypeMsg(kYacht, 36, "MWND9")});
    p.push_back(Sent{0.352, 'A', dec::aisAtonMsg(kAton, 21, "BAR LIGHT FLOAT", 53.5333, -3.3500)});
    p.push_back(Sent{0.450, 'A', dec::aisLongRangeMsg(kFar, 40.7, -74.0, 17, 93, 0)});
    p.push_back(Sent{0.452, 'B', dec::aisPositionMsg(kPilot, 53.4640, -3.1050, 9.8, 121.0, 119, 0, 2)});
    return p;
}

// Checks one decoded message against what was sent.
bool sameAsSent(const dec::AisMessage& m, const Sent& s, std::string& why) {
    char b[200];
    if (m.channel != s.ch) {
        std::snprintf(b, sizeof b, "channel %c, sent %c", m.channel, s.ch);
        why = b;
        return false;
    }
    if (m.bytes != s.msg) {
        why = "bytes differ";
        return false;
    }
    return true;
}

// Checks all fields of the messages of testPlan() (as parsed).
void checkFields(const std::vector<dec::AisMessage>& got, const std::string& tag) {
    int okCount = 0, tested = 0;
    for (std::size_t i = 0; i < got.size(); ++i) {
        const dec::AisMessage& m = got[i];
        bool ok = true;
        ++tested;
        if (m.mmsi == kShip && m.type == 1) {
            ok = m.kind == dec::AisKind::Position && near(m.lat, 53.5210, 1e-6) && near(m.lon, -3.3520, 1e-6) &&
                 near(m.sogKn, 12.3, 1e-9) && near(m.cogDeg, 285.4, 1e-9) && near(m.headingDeg, 284, 0) &&
                 m.status == 0 && m.rot == -128 && !m.classB && m.repeat == 0;
        } else if (m.mmsi == kShip && m.type == 5) {
            ok = m.kind == dec::AisKind::Static && m.imo == 9876543 && m.callsign == "MOGR7" &&
                 m.name == "OFFGRID EXPLORER" && m.shipType == 70 && near(m.lengthM, 128, 0) && near(m.beamM, 20, 0) &&
                 near(m.draughtM, 7.4, 1e-9) && m.destination == "DUBLIN" && m.eta == "02/11 14:30 UTC" &&
                 m.etaMonth == 11 && m.etaDay == 2 && m.etaHour == 14 && m.etaMinute == 30;
        } else if (m.mmsi == kRover) {
            ok = m.type == 3 && near(m.lat, -33.8568, 1e-6) && near(m.lon, 151.2153, 1e-6) && near(m.sogKn, 0.1, 1e-9) &&
                 near(m.cogDeg, 0, 0) && near(m.headingDeg, 211, 0) && m.status == 1;
        } else if (m.mmsi == kBase) {
            ok = m.type == 4 && m.kind == dec::AisKind::Base && near(m.lat, 53.31, 1e-6) && near(m.lon, -4.63, 1e-6) &&
                 m.utc == "2026-11-25 09:08:07 UTC" && m.year == 2026 && m.second == 7;
        } else if (m.mmsi == kYacht && m.type == 18) {
            ok = m.classB && near(m.lat, 53.4020, 1e-6) && near(m.lon, -3.0470, 1e-6) && near(m.sogKn, 5.6, 1e-9) &&
                 near(m.cogDeg, 210, 1e-9) && near(m.headingDeg, 211, 0);
        } else if (m.mmsi == kYacht && m.type == 24 && m.callsign.empty()) {
            ok = m.name == "WINDFALL" && m.kind == dec::AisKind::Static;
        } else if (m.mmsi == kYacht && m.type == 24) {
            ok = m.callsign == "MWND9" && m.shipType == 36 && m.name.empty();
        } else if (m.mmsi == kAton) {
            ok = m.type == 21 && m.kind == dec::AisKind::Aton && m.atonType == 21 && m.name == "BAR LIGHT FLOAT" &&
                 near(m.lat, 53.5333, 1e-6) && near(m.lon, -3.35, 1e-6);
        } else if (m.mmsi == kFar) {
            ok = m.type == 27 && near(m.lat, 40.7, 1e-9) && near(m.lon, -74.0, 1e-9) && near(m.sogKn, 17, 0) &&
                 near(m.cogDeg, 93, 0) && m.status == 0;
        } else if (m.mmsi == kPilot) {
            ok = m.type == 2 && near(m.lat, 53.464, 1e-6) && near(m.lon, -3.105, 1e-6) && near(m.sogKn, 9.8, 1e-9) &&
                 near(m.cogDeg, 121, 1e-9) && near(m.headingDeg, 119, 0);
        } else {
            ok = false;
        }
        if (ok) ++okCount;
        else std::printf("      (%s: field mismatch in type %d from %09u)\n", tag.c_str(), m.type, m.mmsi);
    }
    char b[96];
    std::snprintf(b, sizeof b, "%d of %d", okCount, tested);
    check(okCount == tested && tested > 0, tag + ": every field of every message", b);
}

// Builds a test buffer: the plan's bursts at `rate`, the stick centred on
// `center` but `errHz` off (so every signal appears errHz higher), in
// complex noise giving `snrDb` in 20 kHz.
std::vector<cf> makeBuffer(const std::vector<Sent>& plan, double rate, double center, double errHz, double snrDb,
                           double seconds, unsigned long long seed, std::vector<std::size_t>* badCrc = 0) {
    std::vector<cf> iq(static_cast<std::size_t>(seconds * rate));
    const double amp = 0.1;
    if (snrDb < 200) {
        const double sigma = std::sqrt(amp * amp / std::pow(10.0, snrDb / 10) / (2 * 20e3 / rate));
        Noise nz(seed);
        for (std::size_t i = 0; i < iq.size(); ++i)
            iq[i] = cf(static_cast<float>(sigma * nz.gauss()), static_cast<float>(sigma * nz.gauss()));
    }
    for (std::size_t i = 0; i < plan.size(); ++i) {
        const double off = (plan[i].ch == 'B' ? dec::kAisChannelB : dec::kAisChannelA) - center + errHz;
        bool bad = false;
        if (badCrc)
            for (std::size_t k = 0; k < badCrc->size(); ++k) bad = bad || (*badCrc)[k] == i;
        dec::aisAddBurst(&iq[0], iq.size(), rate, plan[i].t, off, amp, dec::aisMakeBurst(plan[i].msg, bad ? 0x0101 : 0),
                         0.3 * static_cast<double>(i));
    }
    return iq;
}

void testIq(double rate, double errHz, double snrDb, unsigned long long seed) {
    const std::vector<Sent> plan = testPlan();
    const std::vector<cf> iq = makeBuffer(plan, rate, dec::kAisCenter, errHz, snrDb, 0.6, seed);
    dec::AisDecoder d;
    feed(d, iq, rate, dec::kAisCenter);
    std::vector<dec::AisMessage> got;
    d.takeMessages(got);
    char tag[128];
    std::snprintf(tag, sizeof tag, "IQ %.3f MS/s, error %+.0f Hz, SNR %.0f dB", rate / 1e6, errHz, snrDb);
    int found = 0;
    std::string why;
    for (std::size_t i = 0; i < plan.size(); ++i) {
        bool f = false;
        for (std::size_t j = 0; j < got.size() && !f; ++j)
            if (got[j].bytes == plan[i].msg) f = sameAsSent(got[j], plan[i], why);
        if (f) ++found;
        else std::printf("      (%s: missing type %d on %c%s%s)\n", tag, plan[i].msg[0] >> 2, plan[i].ch,
                         why.empty() ? "" : ": ", why.c_str());
    }
    char b[96];
    std::snprintf(b, sizeof b, "%d of %zu messages, %zu decoded (A %u, B %u)", found, plan.size(), got.size(),
                  d.messageCount('A'), d.messageCount('B'));
    check(found == static_cast<int>(plan.size()) && got.size() == plan.size(), std::string(tag) + ": all messages, both channels", b);
    checkFields(got, tag);
}

}  // namespace

int main() {
    using namespace dec;

    // ---- the parser, the builders, the names ----
    {
        AisMessage m;
        std::vector<unsigned char> b = aisPositionMsg(kShip, 53.5210, -3.3520, 12.3, 285.4, 284, 5);
        check(b.size() == 21 && aisParse(&b[0], b.size(), m) && m.type == 1 && m.mmsi == kShip && m.status == 5 &&
                  near(m.lat, 53.521, 1e-6) && near(m.lon, -3.352, 1e-6) && near(m.sogKn, 12.3, 1e-9) &&
                  near(m.cogDeg, 285.4, 1e-9) && near(m.headingDeg, 284, 0) && m.channel == '?',
              "parse: type 1 position");
        // not available: 181 / 91 degrees, sog 1023, cog 3600, heading 511
        b = aisPositionMsg(kShip, 91, 181, 102.3, 360, 511, 15);
        check(aisParse(&b[0], b.size(), m) && !aisHas(m.lat) && !aisHas(m.lon) && !aisHas(m.sogKn) && !aisHas(m.cogDeg) &&
                  !aisHas(m.headingDeg) && m.hasMotion && m.status == 15,
              "parse: 'not available' values are NaN");
        b = aisPositionMsg(0, 53, -3, 1, 1, 1, 0);
        check(!aisParse(&b[0], b.size(), m), "parse: MMSI 0 rejected");
        b = aisPositionMsg(kShip, 53, -3, 1, 1, 1, 0, 0);
        check(!aisParse(&b[0], b.size(), m), "parse: type 0 rejected (Addition to the JS)");
        b = aisStaticMsg(kPilot, 0, "MPLT4", "MERSEY PILOT", 50, 21, 6, 1.8, "LIVERPOOL", 0, 0, 24, 60);
        check(b.size() == 53 && aisParse(&b[0], b.size(), m) && m.kind == AisKind::Static && m.eta.empty() &&
                  m.etaMonth == -1 && m.imo == 0 && m.name == "MERSEY PILOT" && m.destination == "LIVERPOOL" &&
                  near(m.lengthM, 21, 0) && near(m.beamM, 6, 0) && near(m.draughtM, 1.8, 1e-9) && m.shipType == 50,
              "parse: type 5, no ETA");
        // type 19 (extended class B), hand-built as in M.1371
        AisPacker p;
        p.field(19, 6); p.field(0, 2); p.field(kYacht, 30); p.field(0, 8); p.field(56, 10); p.field(0, 1);
        p.field(std::floor(-3.047 * 600000 + 0.5), 28); p.field(std::floor(53.402 * 600000 + 0.5), 27);
        p.field(2100, 12); p.field(211, 9); p.field(30, 6); p.field(0, 4); p.text("WINDFALL", 20); p.field(36, 8);
        p.field(8, 9); p.field(4, 9); p.field(2, 6); p.field(2, 6); p.field(1, 4); p.field(0, 1); p.field(0, 1);
        p.field(1, 1); p.field(0, 4);
        b = p.bytes(312);
        check(p.bits() == 312 && aisParse(&b[0], b.size(), m) && m.type == 19 && m.classB && m.name == "WINDFALL" &&
                  m.shipType == 36 && near(m.sogKn, 5.6, 1e-9) && near(m.cogDeg, 210, 1e-9) && near(m.lat, 53.402, 1e-6),
              "parse: type 19 (name and ship type in a position)");
        // an "other" type (8, binary broadcast)
        AisPacker q;
        q.field(8, 6); q.field(0, 2); q.field(kShip, 30); q.field(0, 2); q.field(235, 10); q.field(10, 6);
        b = q.bytes(96);
        check(aisParse(&b[0], b.size(), m) && m.kind == AisKind::Other && aisDescribe(m).find("binary broadcast") != std::string::npos,
              "parse: other types kept (type 8)", aisDescribe(m));
        // names for numbers
        check(std::string(aisStatusName(1)) == "at anchor" && std::string(aisStatusName(8)) == "under way, sailing" &&
                  std::string(aisStatusName(9)).empty(),
              "names: navigation status");
        check(aisShipTypeName(70) == "cargo" && aisShipTypeName(84) == "tanker" && aisShipTypeName(50) == "pilot" &&
                  aisShipTypeName(36) == "sailing" && aisShipTypeName(0).empty() && aisShipTypeName(99) == "vessel",
              "names: ship types");
        check(std::string(aisAtonName(21)) == "light vessel" && std::string(aisAtonName(24)) == "buoy" &&
                  std::string(aisFlag(235012345)) == "UK" && std::string(aisFlag(2320001)) == "UK" &&
                  std::string(aisFlag(992351234)) == "UK" && std::string(aisFlag(366123456)) == "USA" &&
                  std::string(aisFlag(250001234)) == "Ireland",
              "names: aids to navigation, flags (MID)");
    }

    // ---- !AIVDM ----
    {
        const std::vector<unsigned char> b = aisPositionMsg(kShip, 53.5210, -3.3520, 12.3, 285.4, 284, 0);
        const std::vector<std::string> s = aisNmea(&b[0], b.size(), 'A');
        std::vector<std::string> f;
        bool cs = false;
        std::vector<unsigned char> bits;
        const bool ok = s.size() == 1 && splitNmea(s[0], f, cs) && f[0] == "AIVDM" && f[1] == "1" && f[2] == "1" &&
                        f[3].empty() && f[4] == "A" && f[5].size() == 28 && f[6] == "0" &&
                        dearmor(f[5], 0, bits) && bits == bitsOf(b);
        check(ok && cs, "AIVDM: type 1 = one sentence, 28 characters, fill 0, checksum, armoring", s.empty() ? "" : s[0]);
        const std::vector<unsigned char> st =
            aisStaticMsg(kShip, 9876543, "MOGR7", "OFFGRID EXPLORER", 70, 128, 20, 7.4, "DUBLIN", 11, 2, 14, 30);
        const std::vector<std::string> s5 = aisNmea(&st[0], st.size(), 'B', 3);
        std::vector<std::string> f1, f2;
        bool c1 = false, c2 = false;
        std::vector<unsigned char> b5;
        const bool ok5 = s5.size() == 2 && splitNmea(s5[0], f1, c1) && splitNmea(s5[1], f2, c2) && f1[1] == "2" &&
                         f1[2] == "1" && f2[2] == "2" && f1[3] == "3" && f2[3] == "3" && f1[5].size() == 60 &&
                         f2[5].size() == 11 && f1[6] == "0" && f2[6] == "2" && dearmor(f1[5] + f2[5], 2, b5) &&
                         b5 == bitsOf(st);
        check(ok5 && c1 && c2, "AIVDM: type 5 = two sentences (60 + 11 characters, fill 2, sequence id)",
              s5.size() == 2 ? s5[0] + " " + s5[1] : "");
        // A published sentence (the gpsd AIVDM/AIVDO protocol decoding
        // document's example): re-armoured identically, and its fields.
        const std::string pub = "!AIVDM,1,1,,A,13u?etPv2;0n:dDPwUM1U1Cb069D,0*24";
        std::vector<std::string> pf;
        bool pcs = false;
        std::vector<unsigned char> pbits;
        if (splitNmea(pub, pf, pcs) && dearmor(pf[5], 0, pbits)) {
            std::vector<unsigned char> pb((pbits.size() + 7) / 8, 0);
            for (std::size_t i = 0; i < pbits.size(); ++i) pb[i / 8] |= static_cast<unsigned char>(pbits[i] << (7 - i % 8));
            AisMessage m;
            const bool parsed = aisParse(&pb[0], pb.size(), m);
            const std::vector<std::string> back = aisNmea(&pb[0], pb.size(), 'A');
            check(pcs && parsed && back.size() == 1 && back[0] == pub, "AIVDM: a published sentence re-armoured byte-exact",
                  back.empty() ? "" : back[0]);
            check(parsed && m.type == 1 && m.mmsi == 265547250 && near(m.sogKn, 13.9, 1e-9) && near(m.cogDeg, 40.4, 1e-9) &&
                      near(m.headingDeg, 41, 0) && near(m.lat, 57.6603533, 1e-6) && near(m.lon, 11.8329767, 1e-6),
                  "AIVDM: the published sentence's fields (MMSI 265547250, 13.9 kn, 40.4°, 57.66°N 11.83°E)",
                  str("lat %.7f", m.lat) + str(" lon %.7f", m.lon));
        } else {
            check(false, "AIVDM: a published sentence parses");
        }
    }

    // ---- from IQ, both channels at once ----
    testIq(2048000, 0, 18, 11);
    testIq(2048000, 1000, 18, 12);
    testIq(2048000, -1000, 18, 13);
    testIq(2400000, 0, 18, 14);
    testIq(2400000, 1000, 18, 15);
    testIq(2400000, -1000, 18, 16);

    // ---- sensitivity: share of messages decoded, 10 buffers of the plan ----
    {
        const double snrs[] = {12, 14, 16};
        const double need[] = {0, 0.6, 0.9};   // (12 dB: informational)
        for (int k = 0; k < 3; ++k) {
            int got = 0, sent = 0;
            for (int rep = 0; rep < 10; ++rep) {
                const std::vector<Sent> plan = testPlan();
                AisDecoder d;
                feed(d, makeBuffer(plan, 2048000, kAisCenter, rep % 3 == 0 ? 0 : rep % 3 == 1 ? 1000 : -1000, snrs[k], 0.6,
                                   1000 + 10 * k + rep),
                     2048000, kAisCenter);
                got += static_cast<int>(d.messageCount());
                sent += static_cast<int>(plan.size());
            }
            char name[160];
            std::snprintf(name, sizeof name,
                          "sensitivity: SNR %.0f dB (in 20 kHz), both channels, 0/±1 kHz off: %d of %d messages (%.0f%%)",
                          snrs[k], got, sent, 100.0 * got / sent);
            check(got >= need[k] * sent, name);
        }
    }

    // ---- bad CRC ----
    {
        const std::vector<Sent> plan = testPlan();
        std::vector<std::size_t> bad;
        bad.push_back(0);
        bad.push_back(3);
        bad.push_back(7);
        const std::vector<cf> iq = makeBuffer(plan, 2048000, kAisCenter, 0, 20, 0.6, 21, &bad);
        AisDecoder d;
        feed(d, iq, 2048000, kAisCenter);
        std::vector<AisMessage> got;
        d.takeMessages(got);
        bool none = true;
        for (std::size_t i = 0; i < got.size(); ++i)
            for (std::size_t k = 0; k < bad.size(); ++k) none = none && got[i].bytes != plan[bad[k]].msg;
        char b[64];
        std::snprintf(b, sizeof b, "%zu of %zu good ones decoded", got.size(), plan.size() - bad.size());
        check(none && got.size() == plan.size() - bad.size(), "a wrong CRC is not shown (the others are)", b);
    }

    // ---- the vessel table, text, NMEA, home ----
    {
        const std::vector<Sent> plan = testPlan();
        const std::vector<cf> iq = makeBuffer(plan, 2048000, kAisCenter, 0, 20, 0.6, 31);
        AisDecoder d;
        d.setHome(53.4808, -2.2426);
        d.setNmeaInText(true);
        feed(d, iq, 2048000, kAisCenter);
        AisVessel v;
        const bool have = d.vessel(kShip, v);
        check(have && v.messages == 2 && v.name == "OFFGRID EXPLORER" && v.callsign == "MOGR7" && v.imo == 9876543 &&
                  v.destination == "DUBLIN" && v.eta == "02/11 14:30 UTC" && v.shipType == 70 && near(v.lat, 53.521, 1e-6) &&
                  near(v.sogKn, 12.3, 1e-9) && near(v.lengthM, 128, 0) && v.heardA && v.heardB && aisWhat(v) == "cargo",
              "vessel table: type 1 + type 5 merged for one MMSI (both channels)");
        double km = 0, deg = 0;
        aisDistance(53.4808, -2.2426, 53.521, -3.352, km, deg);
        check(have && near(v.distanceKm, km, 1e-9) && km > 70 && km < 80 && deg > 270 && deg < 290,
              "vessel table: distance and bearing from setHome()", str("%.1f km", v.distanceKm) + str(" %.1f deg", v.bearingDeg));
        AisVessel y;
        check(d.vessel(kYacht, y) && y.messages == 3 && y.name == "WINDFALL" && y.callsign == "MWND9" && y.shipType == 36 &&
                  y.classB && aisWhat(y) == "sailing" && near(y.sogKn, 5.6, 1e-9),
              "vessel table: class B 18 + 24A + 24B merged");
        AisVessel bs, at, rv;
        check(d.vessel(kBase, bs) && bs.base && aisWhat(bs) == "coast station" && aisDetail(bs) == "AIS base station, UK" &&
                  d.vessel(kAton, at) && at.aton && aisWhat(at) == "light vessel" && d.vessel(kRover, rv) &&
                  aisDetail(rv) == "stopped \xC2\xB7 at anchor",
              "vessel table: coast station, aid to navigation, a tanker at anchor", aisDetail(rv));
        check(aisDetail(v) == "12.3 kn, course 285\xC2\xB0 \xC2\xB7 to DUBLIN", "vessel table: the list's detail (aisDetail)",
              aisDetail(v));
        const std::vector<AisVessel> list = d.vessels();
        bool sorted = true;
        for (std::size_t i = 1; i < list.size(); ++i) sorted = sorted && list[i - 1].lastHeard >= list[i].lastHeard;
        check(list.size() == 7 && sorted && list[0].mmsi == kPilot, "vessel table: 7 rows, the most recently heard first");
        std::string text;
        d.takeText(text);
        int lines = 0, nmeaLines = 0;
        for (std::size_t i = 0; i < text.size(); ++i)
            if (text[i] == '\n') ++lines;
        for (std::size_t at2 = text.find("!AIVDM"); at2 != std::string::npos; at2 = text.find("!AIVDM", at2 + 1)) ++nmeaLines;
        const bool hasLine =
            text.find("AIS B 235012345 OFFGRID EXPLORER (cargo, UK): static: OFFGRID EXPLORER \xC2\xB7 call sign MOGR7 \xC2\xB7 IMO 9876543") !=
            std::string::npos;
        check(hasLine && lines == 10 + 11 && nmeaLines == 11, "text: one line per message (+ !AIVDM lines when asked)",
              text.substr(0, text.find('\n')));
        std::printf("%s", text.c_str());
        std::vector<std::string> nm;
        d.takeNmea(nm);
        bool allOk = nm.size() == 11;
        for (std::size_t i = 0; i < nm.size(); ++i) {
            std::vector<std::string> f;
            bool cs = false;
            allOk = allOk && splitNmea(nm[i], f, cs) && cs && (f[4] == "A" || f[4] == "B");
        }
        check(allOk, "takeNmea: 11 sentences, checksums right, channel set");
        Status s = d.status();
        check(s.locked && s.info.find("7 vessels") != std::string::npos, "status", s.info);
        d.clearVessels();
        check(d.vessels().empty(), "clearVessels()");
    }

    // ---- the demo plan (one 30 s cycle at 240 kS/s, both channels) ----
    {
        const std::vector<AisDemoBurst> plan = aisDemoPlan(2026, 1, 1, 12, 0, 0);
        const double rate = 240000;
        AisDecoder d;
        Noise nz(41);
        std::vector<cf> blk(24000);
        for (int k = 0; k < 310; ++k) {   // 31 s
            const double sig = std::sqrt(0.015 * 0.015 / std::pow(10.0, 1.8) / (2 * 20e3 / rate));
            for (std::size_t i = 0; i < blk.size(); ++i)
                blk[i] = cf(static_cast<float>(sig * nz.gauss()), static_cast<float>(sig * nz.gauss()));
            aisDemoAdd(plan, &blk[0], blk.size(), rate, kAisCenter, k * 0.1, 0.015);
            d.processIq(&blk[0], blk.size(), rate, kAisCenter, kAisCenter);
        }
        std::vector<AisMessage> got;
        d.takeMessages(got);
        int match = 0;
        for (std::size_t i = 0; i < plan.size(); ++i)
            for (std::size_t j = 0; j < got.size(); ++j)
                if (got[j].bytes == plan[i].msg && got[j].channel == plan[i].channel) {
                    ++match;
                    break;
                }
        // bursts starting before 1 s are heard again in the 31st second
        char b[96];
        std::snprintf(b, sizeof b, "%d of %zu plan entries, %zu messages, %zu vessels", match, plan.size(), got.size(),
                      d.vessels().size());
        check(match == static_cast<int>(plan.size()) && d.vessels().size() == 6, "demo plan: the 30 s cycle decodes (240 kS/s)", b);
        AisVessel w;
        check(d.vessel(235098765, w) && w.name == "WINDFALL" && w.callsign == "MWND9" && aisWhat(w) == "sailing",
              "demo plan: the yacht's name from 24A and call sign from 24B");
    }

    // ---- nothing from noise ----
    {
        AisDecoder d;
        Noise nz(51);
        const double rate = 2048000;
        std::vector<cf> blk(204800);
        for (int k = 0; k < 300; ++k) {   // 30 s
            for (std::size_t i = 0; i < blk.size(); ++i)
                blk[i] = cf(static_cast<float>(0.1 * nz.gauss()), static_cast<float>(0.1 * nz.gauss()));
            d.processIq(&blk[0], blk.size(), rate, kAisCenter, kAisCenter);
        }
        std::string t;
        d.takeText(t);
        check(d.messageCount() == 0 && t.empty(), "nothing from 30 s of noise (2.048 MS/s, both channels)", t);
    }

    // ---- NaN / Inf, a rate change, a channel out of view ----
    {
        const std::vector<Sent> plan = testPlan();
        std::vector<cf> iq = makeBuffer(plan, 2048000, kAisCenter, 0, 20, 0.6, 61);
        const float qnan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
        // poison the gaps between bursts (0.0-0.04 s and 0.54-0.6 s)
        for (std::size_t i = 1000; i < 60000; i += 7) iq[i] = cf(i % 2 ? qnan : inf, -inf);
        for (std::size_t i = 1110000; i < iq.size(); i += 5) iq[i] = cf(qnan, qnan);
        AisDecoder d;
        feed(d, iq, 2048000, kAisCenter);
        feed(d, makeBuffer(plan, 2048000, kAisCenter, 0, 20, 0.6, 62), 2048000, kAisCenter);
        check(d.messageCount() == 2 * plan.size(), "NaN / Inf samples do no harm", str("%.0f messages", d.messageCount()));
        feed(d, makeBuffer(plan, 2400000, kAisCenter, 0, 20, 0.6, 63), 2400000, kAisCenter);
        check(d.messageCount() == 3 * plan.size(), "a rate change (2.048 -> 2.4 MS/s) works");
        AisDecoder n;
        std::vector<cf> narrow = makeBuffer(plan, 240000, 161.900e6, 0, 20, 0.6, 64);
        feed(n, narrow, 240000, 161.900e6);
        check(n.channelInView('A') && !n.channelInView('B') && n.messageCount('A') == 5 && n.messageCount('B') == 0 &&
                  n.status().info.find("B not in view") != std::string::npos,
              "240 kS/s on 161.900: channel A only, B reported out of view", n.status().info);
        d.reset();
        check(d.messageCount() == 0 && d.vessels().empty(), "reset() forgets");
        float a[4] = {0, 0, 0, 0};
        d.process(a, 4, 48000);   // audio is ignored
        d.processIq(0, 100, 2048000, kAisCenter, kAisCenter);
        check(d.wantsIq() && d.kind() == Kind::Ais, "wantsIq(), Kind::Ais");
    }

    // ---- setTimestamp ----
    {
        const std::vector<Sent> plan = testPlan();
        AisDecoder d;
        d.setTimestamp(1.8e9);
        feed(d, makeBuffer(plan, 2048000, kAisCenter, 0, 20, 0.6, 71), 2048000, kAisCenter);
        std::vector<AisMessage> got;
        d.takeMessages(got);
        check(!got.empty() && got[0].time > 1.8e9 + 0.05 && got[0].time < 1.8e9 + 0.2,
              "setTimestamp(): times in Unix seconds", got.empty() ? "" : str("%.4f", got[0].time - 1.8e9));
    }

    // ---- CPU ----
    {
        const double rate = 2048000, seconds = 10;
        Noise nz(81);
        std::vector<cf> blk(262144);
        for (std::size_t i = 0; i < blk.size(); ++i)
            blk[i] = cf(static_cast<float>(0.1 * nz.gauss()), static_cast<float>(0.1 * nz.gauss()));
        AisDecoder d;
        const std::size_t total = static_cast<std::size_t>(seconds * rate);
        const std::clock_t t0 = std::clock();
        for (std::size_t done = 0; done < total; done += blk.size())
            d.processIq(&blk[0], blk.size(), rate, kAisCenter, kAisCenter);
        const double cpu = double(std::clock() - t0) / CLOCKS_PER_SEC;
        char name[160];
        std::snprintf(name, sizeof name, "CPU: AIS both channels @2.048 MS/s: %.1fx real time (%.1f%% of a core)",
                      seconds / cpu, 100 * cpu / seconds);
        check(cpu < seconds / 3, name);
    }

    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
