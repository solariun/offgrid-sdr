// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_decoders_ftx.cpp
//  Layer:  Tests (library liboffgrid-decoders)
//
//  What this file does
//  -------------------
//  Self-checking test of FT8 / FT4 (Ftx.h, Ft8.h), the offline geography
//  (Callsign.h) and the "Heard" summary (Heard.h).  Prints PASS/FAIL per
//  check and returns nonzero if anything failed.
//    - pack / unpack round trips for every message type (standard, /R, /P,
//      CQ with modifiers, hashed calls, free text, telemetry, DXpedition,
//      Field Day, RTTY Roundup, nonstandard calls, EU VHF); unknown hashes
//      shown as <...>; messages that cannot be sent refused;
//    - payloads, FT8 and FT4 tones identical to ft8_lib's encoder (Kārlis
//      Goba, MIT) for reference messages (vectors made with ft8_lib);
//    - CRC-14: a codeword checks, any single bit flipped does not;
//    - LDPC: belief propagation corrects hard errors and noisy soft bits;
//    - FT8 and FT4 live decoders, at 48000 and 12000 Hz: several stations
//      per slot at different audio frequencies and DTs, white noise, SNRs
//      (in 2500 Hz) down to -18 dB (FT8) / -12 dB (FT4), the computer's
//      clock 2.3 s fast or 1.6 s slow with setClockOffset() compensating:
//      exactly the messages sent, in the right slots, DT and frequency
//      right; a weak station 12 Hz from a strong one found by the deeper
//      decoding; the clock error (1 s) measured when not compensated; NaNs;
//      blocks of uneven size, timestamps a little late at random;
//    - noise alone: no decodes;
//    - countries, locators, distance / bearing, cities: spot checks;
//    - HeardList: stations, countries, the farthest, directions, trend;
//    - the time taken to decode a slot (well under the slot).
//
//  How it fits in
//  --------------
//  Build from the repository root (scratch output anywhere):
//    c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors \
//        -Werror=c++14-extensions -Werror=c++17-extensions -Isrc \
//        src/dsp/Fft.cpp src/decoders/Ftx.cpp src/decoders/Ft8.cpp \
//        src/decoders/Callsign.cpp src/decoders/Heard.cpp \
//        tests/test_decoders_ftx.cpp -o ftx_t
//
//  Contributing
//  ------------
//  Add a check per new feature; keep the run under ~60 s.
// ============================================================================
#include "decoders/Callsign.h"
#include "decoders/Ft8.h"
#include "decoders/Ftx.h"
#include "decoders/Heard.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <set>
#include <string>
#include <vector>

using namespace dec;

namespace {

int failures = 0, passes = 0;

void check(bool ok, const std::string& what) {
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
    if (ok) ++passes;
    else ++failures;
}

std::string fmt(const char* f, double a, double b = 0, double c = 0) {
    char buf[256];
    std::snprintf(buf, sizeof buf, f, a, b, c);
    return buf;
}

// a small deterministic generator (the same numbers on every platform)
struct Rng {
    unsigned long long s;
    explicit Rng(unsigned long long seed) : s(seed * 2862933555777941757ULL + 3037000493ULL) {}
    double uniform() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return ((s >> 11) + 0.5) / 9007199254740992.0;
    }
    double gauss() {
        const double u = uniform(), v = uniform();
        return std::sqrt(-2 * std::log(u)) * std::cos(2 * 3.14159265358979323846 * v);
    }
};

// ---------------------------------------------------------------- messages
void testMessages() {
    std::printf("-- messages: pack / unpack\n");
    struct Case { const char* in; const char* out; ftx::MessageType type; };
    const Case cases[] = {
        {"CQ G4OGR IO81", "CQ G4OGR IO81", ftx::MessageType::Standard},
        {"cq  g4ogr   io81", "CQ G4OGR IO81", ftx::MessageType::Standard},
        {"CQ DX K1OGS FN42", "CQ DX K1OGS FN42", ftx::MessageType::Standard},
        {"CQ POTA K1ABC FN42", "CQ POTA K1ABC FN42", ftx::MessageType::Standard},
        {"CQ 123 K1ABC FN42", "CQ 123 K1ABC FN42", ftx::MessageType::Standard},
        {"QRZ K1ABC FN42", "QRZ K1ABC FN42", ftx::MessageType::Standard},
        {"DE K1ABC", "DE K1ABC", ftx::MessageType::Standard},
        {"EA4XYZ G4OGR -12", "EA4XYZ G4OGR -12", ftx::MessageType::Standard},
        {"G4OGR EA4XYZ R-09", "G4OGR EA4XYZ R-09", ftx::MessageType::Standard},
        {"G4OGR EA4XYZ R+05", "G4OGR EA4XYZ R+05", ftx::MessageType::Standard},
        {"G4OGR EA4XYZ R IN80", "G4OGR EA4XYZ R IN80", ftx::MessageType::Standard},
        {"G4OGR EA4XYZ IN80", "G4OGR EA4XYZ IN80", ftx::MessageType::Standard},
        {"EA4XYZ G4OGR RRR", "EA4XYZ G4OGR RRR", ftx::MessageType::Standard},
        {"EA4XYZ G4OGR RR73", "EA4XYZ G4OGR RR73", ftx::MessageType::Standard},
        {"G4OGR EA4XYZ 73", "G4OGR EA4XYZ 73", ftx::MessageType::Standard},
        {"K1ABC W9XYZ", "K1ABC W9XYZ", ftx::MessageType::Standard},
        {"K1ABC/R W9XYZ FN42", "K1ABC/R W9XYZ FN42", ftx::MessageType::Standard},
        {"PA3XYZ/P GM4ABC/P R JO22", "PA3XYZ/P GM4ABC/P R JO22", ftx::MessageType::Standard},
        {"3DA0XYZ K1ABC", "3DA0XYZ K1ABC", ftx::MessageType::Standard},
        {"3XY1AB K1ABC -05", "3XY1AB K1ABC -05", ftx::MessageType::Standard},
        {"<PJ4/K1ABC> W9XYZ -11", "<PJ4/K1ABC> W9XYZ -11", ftx::MessageType::Standard},
        {"TNX BOB 73 GL", "TNX BOB 73 GL", ftx::MessageType::FreeText},
        {"OFFGRID SDR", "OFFGRID SDR", ftx::MessageType::FreeText},
        {"73 GL ?/+-.", "73 GL ?/+-.", ftx::MessageType::FreeText},
        {"telemetry 0123456789ABCDEF01", "telemetry 0123456789ABCDEF01", ftx::MessageType::Telemetry},
        {"TELEMETRY 7FF", "telemetry 0000000000000007FF", ftx::MessageType::Telemetry},
        {"K1ABC RR73; W9XYZ <KH1/KH7Z> -08", "K1ABC RR73; W9XYZ <KH1/KH7Z> -08", ftx::MessageType::Dxpedition},
        {"K1ABC RR73; W9XYZ <KH1/KH7Z> +12", "K1ABC RR73; W9XYZ <KH1/KH7Z> +12", ftx::MessageType::Dxpedition},
        {"WA9XYZ KA1ABC R 16A EMA", "WA9XYZ KA1ABC R 16A EMA", ftx::MessageType::FieldDay},
        {"WA9XYZ KA1ABC 32H DX", "WA9XYZ KA1ABC 32H DX", ftx::MessageType::FieldDay},
        {"WA9XYZ KA1ABC 1B WWA", "WA9XYZ KA1ABC 1B WWA", ftx::MessageType::FieldDay},
        {"TU; W9XYZ K1ABC R 579 MA", "TU; W9XYZ K1ABC R 579 MA", ftx::MessageType::RttyRoundup},
        {"W9XYZ K1ABC 559 0013", "W9XYZ K1ABC 559 0013", ftx::MessageType::RttyRoundup},
        {"W9XYZ K1ABC 599 PEI", "W9XYZ K1ABC 599 PEI", ftx::MessageType::RttyRoundup},
        {"<W9XYZ> PJ4/K1ABC RR73", "<W9XYZ> PJ4/K1ABC RR73", ftx::MessageType::NonStandard},
        {"PJ4/K1ABC <W9XYZ> 73", "PJ4/K1ABC <W9XYZ> 73", ftx::MessageType::NonStandard},
        {"W9XYZ PJ4/K1ABC", "<W9XYZ> PJ4/K1ABC", ftx::MessageType::NonStandard},
        {"YW18FIFA K1ABC RRR", "YW18FIFA <K1ABC> RRR", ftx::MessageType::NonStandard},
        {"CQ PJ4/K1ABC", "CQ PJ4/K1ABC", ftx::MessageType::NonStandard},
        {"CQ YW18FIFA", "CQ YW18FIFA", ftx::MessageType::NonStandard},
        {"<PA3ABC> <G4XYZ/P> R 590003 IO91NP", "<PA3ABC> <G4XYZ/P> R 590003 IO91NP", ftx::MessageType::EuVhf},
        {"<DL1ABC> <G4XYZ/P> 572047 JO22AB", "<DL1ABC> <G4XYZ/P> 572047 JO22AB", ftx::MessageType::EuVhf},
    };
    for (std::size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        ftx::CallHashTable ht;
        ftx::u8 a[ftx::kPayloadBytes];
        ftx::MessageType t = ftx::MessageType::Unknown, t2 = ftx::MessageType::Unknown;
        std::string out;
        const bool ok = ftx::pack77(cases[i].in, a, &ht, &t) && ftx::unpack77(a, out, &ht, &t2);
        check(ok && out == cases[i].out && t == cases[i].type && t2 == cases[i].type,
              std::string("round trip \"") + cases[i].in + "\" -> \"" + out + "\" (" + ftx::messageTypeName(t2) + ")");
    }
    // a hash not heard in full: "<...>"
    {
        ftx::CallHashTable sender, receiver;
        ftx::u8 a[ftx::kPayloadBytes];
        std::string out;
        ftx::pack77("<W9XYZ> PJ4/K1ABC RR73", a, &sender);
        ftx::unpack77(a, out, &receiver);
        check(out == "<...> PJ4/K1ABC RR73", "an unknown hash shows as <...>: " + out);
        // ... and once the call has been heard in full, it is filled in
        ftx::pack77("CQ W9XYZ EN37", a, &sender);
        std::string cq;
        ftx::unpack77(a, cq, &receiver);
        ftx::pack77("<W9XYZ> PJ4/K1ABC RR73", a, &sender);
        ftx::unpack77(a, out, &receiver);
        check(out == "<W9XYZ> PJ4/K1ABC RR73", "the hash filled in after hearing the call: " + out);
    }
    // what cannot be sent
    {
        const char* bad[] = {"THIS IS FAR TOO LONG FOR FT8", "K1ABC W9XYZ -45", "CQ G4OGR IO81 EXTRA", "WA9XYZ KA1ABC 33A EMA",
                             "a@b", ""};
        for (std::size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
            ftx::u8 a[ftx::kPayloadBytes];
            check(!ftx::pack77(bad[i], a), std::string("refused: \"") + bad[i] + "\"");
        }
    }
    // the message types of i3/n3 nobody sends are named
    {
        ftx::u8 a[ftx::kPayloadBytes];
        std::memset(a, 0, sizeof a);
        a[9] = 0x30;   // i3 = 6
        std::string out;
        check(ftx::unpack77(a, out) && out == "(message type 6)", "unknown type named: " + out);
    }
}

// payloads and tones made with ft8_lib (message.c, encode.c) for the same messages
void testAgainstFt8Lib() {
    std::printf("-- against ft8_lib's encoder\n");
    struct Ref { const char* text; const char* payload; const char* ft8; const char* ft4; };
    const Ref refs[] = {
        {"CQ G4OGR IO81", "000000204874ED0F8248",
         "3140652000000001005514572307403327543140652417475305575416053465176740443140652",
         "001321033112330313110223022233110110231330220130123102012013100230123100100020332330033123031011223032010"},
        {"EA4XYZ G4OGR -12", "6D4D47D04874ED1FA9C8",
         "3140652223546074505514572317461425323140652441766560241145535556751726663140652",
         "001320312010220231310223022233110110230330112230000232231020123110323103032203203010330003201023330232010"},
        {"G4OGR EA4XYZ R-09", "090E9DA36A6A3EBFAA88",
         "3140652033073266044616317627463526073140652204277156507557332164350237243140652",
         "001321002110001100112213323001012210233330110330033123023323032131123101313313023032233230203020010032010"},
        {"CQ DX K1OGS FN42", "000046F04E03550A1988",
         "3140652000001047505700236306021525743140652733646113151272147757737501433140652",
         "001321033112320221010222330310320110231223312330011103133122132213223103332101031003130302121231322032010"},
        {"TNX BOB 73 GL", "63EDCEE2A4AE07F50000",
         "3140652207447147063336401773500017703140652646427306546072440503670130533140652",
         "001320331320210121113011003101223310233003223033121300221003030231123100002301012020301213003321232032010"},
        {"K1ABC/R W9XYZ FN42", "09BDE3586149DC0A1988",
         "3140652032247523404061147006021524163140652756446133416764152216524245423140652",
         "001321002230213332320210120023311110231223312330102033321012132323023101203110131332103011023010222132010"},
        {"CQ PJ4/K1ABC", "000001A3A311CAA00460",
         "3140652000000016073153143630005206073140652040337166016431570726475464323140652",
         "001321033112330300112010231323202210232113233102033121230302032101223101203032302120113000032103112132010"},
    };
    for (std::size_t i = 0; i < sizeof(refs) / sizeof(refs[0]); ++i) {
        ftx::u8 a[ftx::kPayloadBytes], t8[ftx::kFt8Symbols], t4[ftx::kFt4Symbols];
        const bool ok = ftx::pack77(refs[i].text, a);
        std::string p, s8, s4;
        char b[4];
        for (int k = 0; k < ftx::kPayloadBytes; ++k) { std::snprintf(b, sizeof b, "%02X", a[k]); p += b; }
        ftx::ft8TonesFromPayload(a, t8);
        ftx::ft4TonesFromPayload(a, t4);
        for (int k = 0; k < ftx::kFt8Symbols; ++k) s8 += static_cast<char>('0' + t8[k]);
        for (int k = 0; k < ftx::kFt4Symbols; ++k) s4 += static_cast<char>('0' + t4[k]);
        check(ok && p == refs[i].payload && s8 == refs[i].ft8 && s4 == refs[i].ft4,
              std::string("payload, FT8 and FT4 tones as ft8_lib: ") + refs[i].text);
    }
}

// ---------------------------------------------------------------- CRC and LDPC
void testCodes() {
    std::printf("-- CRC-14 and LDPC(174,91)\n");
    ftx::u8 a77[ftx::kPayloadBytes], a91[12], cw[ftx::kN];
    ftx::pack77("CQ G4OGR IO81", a77);
    ftx::addCrc(a77, a91);
    check(ftx::checkCrc(a91), "CRC of a message checks");
    bool allCaught = true;
    for (int i = 0; i < ftx::kK; ++i) {
        ftx::u8 b[12];
        std::memcpy(b, a91, sizeof b);
        b[i >> 3] = static_cast<ftx::u8>(b[i >> 3] ^ (0x80 >> (i & 7)));
        if (ftx::checkCrc(b)) allCaught = false;
    }
    check(allCaught, "every single-bit error caught by the CRC");
    ftx::ldpcEncode(a91, cw);
    check(ftx::ldpcCheck(cw) == 0, "the LDPC codeword passes all 83 checks");
    // hard errors: 12 bits wrong (weakly), the rest confident
    {
        float llr[ftx::kN];
        for (int i = 0; i < ftx::kN; ++i) llr[i] = cw[i] ? 3.f : -3.f;
        Rng r(7);
        std::set<int> flipped;
        while (flipped.size() < 12) flipped.insert(static_cast<int>(r.uniform() * ftx::kN));
        for (std::set<int>::const_iterator it = flipped.begin(); it != flipped.end(); ++it) llr[*it] = cw[*it] ? -0.8f : 0.8f;
        ftx::u8 out[ftx::kN];
        const bool ok = ftx::bpDecode(llr, 30, out) && std::memcmp(out, cw, ftx::kN) == 0;
        check(ok, "belief propagation corrects 12 wrong bits");
    }
    // soft bits in Gaussian noise (sigma 0.7): nearly all decode, never wrongly
    {
        Rng r(11);
        int okN = 0, wrong = 0;
        const int trials = 200;
        for (int t = 0; t < trials; ++t) {
            float llr[ftx::kN];
            const double sigma = 0.7;
            for (int i = 0; i < ftx::kN; ++i) {
                const double y = (cw[i] ? 1.0 : -1.0) + sigma * r.gauss();
                llr[i] = static_cast<float>(2 * y / (sigma * sigma));
            }
            ftx::u8 out[ftx::kN];
            if (ftx::bpDecode(llr, 30, out)) {
                if (std::memcmp(out, cw, ftx::kN) == 0) ++okN;
                else ++wrong;
            }
        }
        check(okN >= trials * 9 / 10 && wrong == 0, fmt("noisy soft bits (sigma 0.7): %.0f of 200 decoded, %.0f wrong", okN, wrong));
    }
}

// ---------------------------------------------------------------- live decoding
struct Sig {
    const char* text;
    double f;     // Hz (tone 0)
    double dt;    // s late against 0.5 s into the slot
    double snr;   // dB in 2500 Hz
    int slot;     // 0, 1, 2 ...
};

struct RunResult {
    std::vector<FtxDecode> decodes;
    std::vector<std::vector<double> > timings;
    double maxMs;
    unsigned slots;
};

const double kT0 = 1795000005.0 - std::fmod(1795000005.0, 15.0);   // a slot start (Nov 2026), true UTC

// Builds `slots` slots of audio with the signals in white noise, feeds the decoder in uneven blocks with
// timestamps on a computer clock `pcFast` s fast (a little late at random), with setClockOffset(`correction`).
RunResult run(FtxDecoder& d, ftx::Protocol p, double rate, const std::vector<Sig>& sigs, int slots, double pcFast,
              double correction, unsigned seed, bool nans = false) {
    const ftx::ProtocolInfo& P = ftx::protocolInfo(p);
    const double L = P.slotSeconds;
    const double tStart = kT0 - 3.7;   // the audio's first sample, true time
    const double dur = 3.7 + slots * L + 1.0;
    const std::size_t n = static_cast<std::size_t>(dur * rate);
    std::vector<float> x(n, 0.f);
    const double sigma = 0.05;
    const double n2500 = sigma * sigma * 2500 / (rate / 2);   // white noise's power in 2500 Hz
    for (std::size_t i = 0; i < sigs.size(); ++i) {
        std::vector<float> s;
        const double amp = std::sqrt(2 * n2500 * std::pow(10.0, sigs[i].snr / 10));
        if (p == ftx::Protocol::Ft8) ftx::synthesizeFt8(sigs[i].text, sigs[i].f, rate, amp, s);
        else ftx::synthesizeFt4(sigs[i].text, sigs[i].f, rate, amp, s);
        const double t = kT0 + sigs[i].slot * L + P.txStart + sigs[i].dt;
        const long off = static_cast<long>(std::floor((t - tStart) * rate + 0.5));
        for (std::size_t k = 0; k < s.size(); ++k) {
            const long j = off + static_cast<long>(k);
            if (j >= 0 && j < static_cast<long>(n)) x[static_cast<std::size_t>(j)] += s[k];
        }
    }
    Rng r(seed);
    for (std::size_t i = 0; i < n; ++i) x[i] += static_cast<float>(sigma * r.gauss());
    if (nans) {
        x[n / 3] = std::numeric_limits<float>::quiet_NaN();
        x[n / 3 + 7] = std::numeric_limits<float>::infinity();
    }
    RunResult res;
    std::size_t pos = 0;
    while (pos < n) {
        const std::size_t m = std::min(n - pos, static_cast<std::size_t>(rate * (0.02 + 0.15 * r.uniform())));
        const double pcTime = tStart + pos / rate + pcFast + 0.03 * r.uniform();   // (blocks arrive late)
        d.setClockOffset(correction);
        d.setTimestamp(pcTime);
        d.process(&x[pos], m, rate);
        pos += m;
        std::vector<double> dts;
        if (d.takeSlotTimings(dts)) res.timings.push_back(dts);
    }
    d.takeDecodes(res.decodes);
    res.maxMs = d.maxDecodeMs();
    res.slots = d.slotsDecoded();
    return res;
}

void checkRun(const std::string& name, const RunResult& res, const std::vector<Sig>& sigs, double L, double dtOffset,
              double fTol) {
    // exactly the messages sent, each in its slot
    std::multiset<std::string> want, got;
    for (std::size_t i = 0; i < sigs.size(); ++i) {
        char b[32];
        std::snprintf(b, sizeof b, "%d|", sigs[i].slot);
        want.insert(b + std::string(sigs[i].text));
    }
    int worstDt = 0;
    double maxDtErr = 0, maxFErr = 0, maxSnrErr = 0;
    for (std::size_t i = 0; i < res.decodes.size(); ++i) {
        const FtxDecode& d = res.decodes[i];
        const int slot = static_cast<int>(std::floor((d.slotTime - kT0) / L + 0.5));
        char b[32];
        std::snprintf(b, sizeof b, "%d|", slot);
        got.insert(b + d.text);
        for (std::size_t k = 0; k < sigs.size(); ++k)
            if (sigs[k].slot == slot && d.text == sigs[k].text) {
                const double tol = sigs[k].snr >= -15 ? 0.03 : 0.05;   // (weak signals: the timing scatters more)
                const double e = std::fabs(d.dt - (sigs[k].dt + dtOffset));
                if (e > tol) ++worstDt;
                maxSnrErr = std::max(maxSnrErr, std::fabs(d.snr - sigs[k].snr));
                maxDtErr = std::max(maxDtErr, e);
                maxFErr = std::max(maxFErr, std::fabs(d.freq - sigs[k].f));
            }
    }
    std::string missing, extra;
    for (std::multiset<std::string>::const_iterator it = want.begin(); it != want.end(); ++it)
        if (!got.count(*it)) missing += " [" + *it + "]";
    for (std::multiset<std::string>::const_iterator it = got.begin(); it != got.end(); ++it)
        if (!want.count(*it)) extra += " [" + *it + "]";
    check(missing.empty() && extra.empty() && got.size() == want.size(),
          name + fmt(": %.0f of %.0f messages decoded exactly", static_cast<double>(want.size() - (missing.empty() ? 0 : 1)),
                     static_cast<double>(want.size())) + (missing.empty() ? "" : ", missing" + missing) +
              (extra.empty() ? "" : ", extra" + extra));
    check(worstDt == 0, name + fmt(": DT right (worst error %.3f s)", maxDtErr));
    check(maxFErr < fTol, name + fmt(": frequency error %.2f Hz (under %.1f)", maxFErr, fTol));
    check(maxSnrErr <= 3, name + fmt(": SNR reported within %.0f dB", maxSnrErr));
}

void printDecodes(const RunResult& res, std::size_t maxN) {
    for (std::size_t i = 0; i < res.decodes.size() && i < maxN; ++i) {
        const FtxDecode& d = res.decodes[i];
        std::printf("      %s %s %3d %+5.2f %7.1f  %-34s pass %d  %s%s%s\n", d.proto.c_str(), d.utc.c_str(), d.snr, d.dt,
                    d.freq, d.text.c_str(), d.pass, d.country.c_str(), d.hasDistance ? " " : "",
                    d.hasDistance ? fmt("%.0f km %.0f deg", d.km, d.bearing).c_str() : "");
    }
}

std::vector<Sig> ft8Signals() {
    std::vector<Sig> s;
    const Sig k[] = {
        // slot 0: a QSO starting, a CQ, a free text, a contest station; down to -18 dB
        {"CQ G4OGR IO81", 640, 0.0, -8, 0},
        {"EA4XYZ G4OGR -12", 1240, 0.4, -14, 0},
        {"CQ DX K1OGS FN42", 1890, -0.2, -18, 0},
        {"OFFGRID SDR", 2650, 0.8, -16, 0},
        {"CQ W9XYZ EN37", 2950, -0.6, -12, 0},
        // slot 1: replies; a strong station with a weak one 12 Hz above it (the deeper decoding's job)
        {"G4OGR EA4XYZ R-09", 1240, 0.1, -12, 1},
        {"CQ VK2OGS QF56", 2310, -0.5, -17, 1},
        {"CQ SP9OGS JO90", 410, 0.2, -10, 1},
        {"CQ JA1OGS PM95", 1500, 0.0, -2, 1},
        {"CQ EI2OGS IO63", 1512, 0.05, -14, 1},
        {"CQ KH1/KH7Z", 2800, -0.3, -9, 1},
        // slot 2: other message types
        {"EA4XYZ G4OGR RR73", 1240, 0.3, -11, 2},
        {"<W9XYZ> PJ4/K1ABC RR73", 900, -0.1, -13, 2},
        {"K1ABC RR73; W9XYZ <KH1/KH7Z> -08", 2100, 0.0, -15, 2},
        {"TU; W9XYZ K1ABC R 579 MA", 2700, 0.6, -10, 2},
    };
    for (std::size_t i = 0; i < sizeof(k) / sizeof(k[0]); ++i) s.push_back(k[i]);
    return s;
}

std::vector<Sig> ft4Signals() {
    std::vector<Sig> s;
    const Sig k[] = {
        {"CQ K1OGS FN42", 900, 0.0, -6, 0},
        {"CQ SP9OGS JO90", 1700, 0.3, -12, 0},
        {"K1OGS G4OGR IO81", 2400, -0.2, -10, 0},
        {"G4OGR K1OGS -05", 900, 0.1, -8, 1},
        {"CQ TEST EA4XYZ IN80", 500, -0.3, -12, 1},
        {"K1OGS G4OGR R-11", 2400, 0.4, -11, 2},
        {"CQ DX VK2OGS QF56", 1300, 0.0, -12, 2},
        {"W9XYZ K1ABC 559 0013", 2000, -0.1, -9, 2},
    };
    for (std::size_t i = 0; i < sizeof(k) / sizeof(k[0]); ++i) s.push_back(k[i]);
    return s;
}

void testFt8() {
    std::printf("-- FT8 live decoding\n");
    const std::vector<Sig> sigs = ft8Signals();
    double worstMs = 0;
    {
        Ft8Decoder d;
        d.setHome("IO91");
        d.setDial(14.074e6);
        const RunResult r = run(d, ftx::Protocol::Ft8, 48000, sigs, 3, 2.3, 2.3, 1);
        printDecodes(r, 40);
        checkRun("FT8 48000 Hz, clock 2.3 s fast, corrected", r, sigs, 15, 0, 1.0);
        bool deeper = false, place = false, prev = false;
        for (std::size_t i = 0; i < r.decodes.size(); ++i) {
            const FtxDecode& x = r.decodes[i];
            if (x.text == "CQ EI2OGS IO63" && x.pass > 1) deeper = true;
            if (x.text == "CQ DX K1OGS FN42")
                place = x.country == "USA" && x.hasDistance && std::abs(x.km - 5194) <= 1 && x.bearing == 288 && x.cq &&
                        x.cqTo == "DX" && x.caller == "K1OGS" && x.grid == "FN42";
            if (x.text == "EA4XYZ G4OGR RR73") prev = x.grid == "IO81" && x.gridFromEarlier && x.callee == "EA4XYZ";
        }
        check(deeper, "the weak station under the strong one found by the deeper decoding (pass 2+)");
        check(place, "a CQ's fields: caller, CQ DX, grid, country USA, 5194 km, bearing 288");
        check(prev, "a station's locator from an earlier message");
        std::string t;
        d.takeText(t);
        check(t.find("CQ DX K1OGS FN42") != std::string::npos && t.find("USA") != std::string::npos &&
                  t.find("5194 km 288") != std::string::npos,
              "text lines like the FT8 box");
        std::printf("%s", t.substr(0, t.find('\n') + 1).c_str());
        const Status s = d.status();
        check(s.mode == "FT8" && s.characters == r.decodes.size() && s.info.find("clock corrected +2.3 s") != std::string::npos,
              "status: " + s.info);
        check(r.timings.size() == 3, fmt("timings handed over after each of the %.0f slots", r.timings.size()));
        worstMs = std::max(worstMs, r.maxMs);
        std::printf("      decode time per slot: max %.0f ms (%u slots decoded)\n", r.maxMs, r.slots);
    }
    {
        Ft8Decoder d;
        std::vector<Sig> s2;
        for (std::size_t i = 0; i < sigs.size(); ++i)
            if (sigs[i].slot < 2) s2.push_back(sigs[i]);
        const RunResult r = run(d, ftx::Protocol::Ft8, 12000, s2, 2, -1.6, -1.6, 2, true);
        checkRun("FT8 12000 Hz, clock 1.6 s slow, corrected, NaNs in the audio", r, s2, 15, 0, 1.0);
        worstMs = std::max(worstMs, r.maxMs);
    }
    {
        // not corrected: the decodes are 1 s late, and say so (the slot's stations' DTs centre on 0).  (A clock
        // much further out has the decoding start before the slot's transmissions end: they are then decoded from
        // the next slot's window, as in the browser, but one starting right at that window's edge can be missed.)
        Ft8Decoder d;
        std::vector<Sig> s2;
        for (std::size_t i = 0; i < sigs.size(); ++i)
            if (sigs[i].slot == 0) s2.push_back(sigs[i]);
        const RunResult r = run(d, ftx::Protocol::Ft8, 48000, s2, 2, 1.0, 0, 3);
        checkRun("FT8 48000 Hz, clock 1 s fast, not corrected (DT +1)", r, s2, 15, 1.0, 1.0);
        std::vector<double> v;
        for (std::size_t i = 0; i < r.timings.size(); ++i) v.insert(v.end(), r.timings[i].begin(), r.timings[i].end());
        std::sort(v.begin(), v.end());
        const double med = v.empty() ? 0 : v[v.size() >> 1];
        check(std::fabs(med - 1.0) < 0.05, fmt("the median DT %.3f s measures the clock (1 s fast)", med));
    }
    {
        Ft8Decoder d;
        const RunResult r = run(d, ftx::Protocol::Ft8, 48000, std::vector<Sig>(), 4, 0.4, 0.4, 9);
        check(r.decodes.empty() && r.slots == 4, fmt("noise alone, 4 slots: %.0f decodes", r.decodes.size()));
        worstMs = std::max(worstMs, r.maxMs);
    }
    check(worstMs < 0.4 * 15000, fmt("FT8 decode time per slot %.0f ms, well under the 15 s slot", worstMs));
}

void testFt4() {
    std::printf("-- FT4 live decoding\n");
    const std::vector<Sig> sigs = ft4Signals();
    double worstMs = 0;
    {
        Ft4Decoder d;
        d.setHome("IO91");
        const RunResult r = run(d, ftx::Protocol::Ft4, 48000, sigs, 3, 2.3, 2.3, 4);
        printDecodes(r, 20);
        checkRun("FT4 48000 Hz, clock 2.3 s fast, corrected", r, sigs, 7.5, 0, 2.5);
        worstMs = std::max(worstMs, r.maxMs);
        std::printf("      decode time per slot: max %.0f ms (%u slots decoded)\n", r.maxMs, r.slots);
        bool utc = false;
        for (std::size_t i = 0; i < r.decodes.size(); ++i)
            if (r.decodes[i].text == "G4OGR K1OGS -05") utc = std::fabs(r.decodes[i].slotTime - (kT0 + 7.5)) < 1e-6;
        check(utc, "FT4 slots of 7.5 s");
    }
    {
        Ft4Decoder d;
        const RunResult r = run(d, ftx::Protocol::Ft4, 12000, sigs, 3, -1.6, -1.6, 5);
        checkRun("FT4 12000 Hz, clock 1.6 s slow, corrected", r, sigs, 7.5, 0, 2.5);
        worstMs = std::max(worstMs, r.maxMs);
    }
    {
        Ft4Decoder d;
        const RunResult r = run(d, ftx::Protocol::Ft4, 48000, std::vector<Sig>(), 6, 0, 0, 10);
        check(r.decodes.empty() && r.slots == 6, fmt("noise alone, 6 slots: %.0f decodes", r.decodes.size()));
        worstMs = std::max(worstMs, r.maxMs);
    }
    check(worstMs < 0.4 * 7500, fmt("FT4 decode time per slot %.0f ms, well under the 7.5 s slot", worstMs));
}

void testFtxInfo() {
    std::printf("-- message details (ftxInfo)\n");
    FtxInfo a = ftxInfo("CQ DX K1OGS FN42");
    check(a.type == "CQ" && a.from == "K1OGS" && a.cqTo == "DX" && a.grid == "FN42", "CQ DX K1OGS FN42");
    a = ftxInfo("G4OGR EA4XYZ R-09");
    check(a.type == "R report" && a.from == "EA4XYZ" && a.to == "G4OGR" && a.hasReport && a.report == -9, "R report");
    a = ftxInfo("G4OGR EA4XYZ R IN80");
    check(a.type == "reply" && a.grid == "IN80", "R and a grid");
    a = ftxInfo("EA4XYZ G4OGR RR73");
    check(a.type == "RR73" && a.from == "G4OGR", "RR73");
    a = ftxInfo("TNX BOB 73 GL");
    check(a.type == "text" && a.from.empty(), "free text");
    a = ftxInfo("K1ABC RR73; W9XYZ <KH1/KH7Z> -08");
    check(a.type == "report" && a.from == "<KH1/KH7Z>" && a.to == "W9XYZ" && a.report == -8, "DXpedition");
}

// ---------------------------------------------------------------- geography
void testCallsign() {
    std::printf("-- countries, locators, cities\n");
    struct C { const char* call; const char* name; const char* cont; };
    const C cs[] = {{"G4OGR", "England", "EU"},          {"M0ABC", "England", "EU"},        {"GM4ABC", "Scotland", "EU"},
                    {"MW0ABC", "Wales", "EU"},           {"KH6ABC", "Hawaii", "OC"},        {"EA8ABC", "Canary Islands", "AF"},
                    {"EA4XYZ", "Spain", "EU"},           {"K1OGS", "USA", "NA"},            {"W9XYZ", "USA", "NA"},
                    {"VK2OGS", "Australia", "OC"},       {"JA1OGS", "Japan", "AS"},         {"UA9ABC", "Asiatic Russia", "AS"},
                    {"RA2ABC", "Kaliningrad", "EU"},     {"SP9OGS", "Poland", "EU"},        {"EI2OGS", "Ireland", "EU"},
                    {"EA8/G4ABC", "Canary Islands", "AF"}, {"G4ABC/P", "England", "EU"},    {"W1AW/4", "USA", "NA"},
                    {"PY2ABC", "Brazil", "SA"},          {"DL1ABC", "Germany", "EU"},       {"KL7ABC", "Alaska", "NA"}};
    for (std::size_t i = 0; i < sizeof(cs) / sizeof(cs[0]); ++i) {
        const Country c = countryOf(cs[i].call);
        check(c.found && c.name == cs[i].name && c.continent == cs[i].cont,
              std::string(cs[i].call) + " -> " + (c.found ? c.name + " (" + c.continent + ")" : "none"));
    }
    check(!countryOf("G4ABC/MM").found && !countryOf("<...>").found && !countryOf("").found, "/MM and hashes: no country");
    check(std::string(continentName("OC")) == "Oceania", "continent names");
    double lat, lon;
    check(Locator::toLatLon("IO91", lat, lon) && lat == 51.5 && lon == -1, "IO91 -> 51.5, -1");
    check(Locator::toLatLon("io91wm", lat, lon) && std::fabs(lat - 51.52083) < 1e-4 && std::fabs(lon - (-0.125)) < 1e-4,
          fmt("IO91wm -> %.5f, %.5f", lat, lon));
    check(Locator::fromLatLon(53.48, -2.24) == "IO83vl" && Locator::fromLatLon(53.48, -2.24, 4) == "IO83",
          "53.48, -2.24 -> " + Locator::fromLatLon(53.48, -2.24));
    check(Locator::fromLatLon(-33.87, 151.21) == "QF56od", "Sydney -> " + Locator::fromLatLon(-33.87, 151.21));
    check(!Locator::isValid("SS99") && !Locator::isValid("IO9") && Locator::isValid("RR73"), "locator forms");
    double km, deg;
    check(Locator::distanceBearing("IO91", "FN42", km, deg) && std::fabs(km - 5193.86) < 0.1 && std::fabs(deg - 287.86) < 0.05,
          fmt("IO91 -> FN42: %.1f km, %.1f deg", km, deg));
    check(Locator::distanceBearing("IO91", "QF56", km, deg) && std::fabs(km - 17002.84) < 0.1 && std::fabs(deg - 59.28) < 0.05,
          fmt("IO91 -> QF56: %.1f km, %.1f deg", km, deg));
    check(Locator::sector(287.86) == 6 && Locator::sector(359) == 0 && Locator::sector(22.4) == 0 && Locator::sector(22.6) == 1,
          "compass sectors");
    std::size_t n = 0;
    cities(n);
    check(n == 620, fmt("%.0f built-in cities", n));
    const City* c = findCity("manchester, uk");
    check(c && std::string(c->name) == "Manchester" && Locator::fromLatLon(c->lat, c->lon) == "IO83vl", "find Manchester, UK");
    c = findCity("London");
    check(c && std::string(c->country) == "UK", "find London");
    c = nearestCity(48.80, 2.30, &km);
    check(c && std::string(c->name) == "Paris" && km < 15, std::string("nearest city to 48.80, 2.30: ") + (c ? c->name : "?"));
    check(!findCity("Atlantis"), "an unknown city");
}

void testHeard() {
    std::printf("-- Heard\n");
    HeardList h;
    h.setHome("IO91");
    const double t0 = 1795000000;
    // 20 m: stations to the west and the east, more in the last half hour than before
    h.note(t0 + 100, 14.074e6, "FT8", "K1OGS", "FN42", -15);
    h.note(t0 + 200, 14.074e6, "FT8", "W9XYZ", "EN37", -18);
    h.note(t0 + 2000, 14.074e6, "FT8", "K1OGS", "", -9);          // better report, no grid (remembered)
    h.note(t0 + 2100, 14.074e6, "FT8", "VE3ABC", "FN03", -12);
    h.note(t0 + 2200, 14.074e6, "FT8", "SP9OGS", "JO90", -5);
    h.note(t0 + 2300, 14.074e6, "FT8", "UA9ABC", "MO05", -14);
    h.note(t0 + 2400, 14.074e6, "FT8", "VK2OGS", "QF56", -19);
    h.note(t0 + 2500, 14.074e6, "FT8", "N1ABC", "FN31", -10);
    h.note(t0 + 2550, 14.074e6, "FT8", "K2ABC", "FN20", -16);
    h.note(t0 + 2600, 14.080e6, "FT4", "<...>", "", -10);          // (skipped)
    // 40 m: two stations, both earlier
    h.note(t0 + 500, 7.074e6, "FT8", "EA4XYZ", "IN80", -7);
    h.note(t0 + 600, 7.074e6, "FT8", "EI2OGS", "IO63", -3);
    h.note(t0 + 3500, 7.0386e6, "WSPR", "DL1ABC", "JO62", -20);
    const double now = t0 + 3600;
    const std::vector<HeardBand> s = h.summary(now, 3600);
    check(s.size() == 2 && s[0].band == "40 m" && s[1].band == "20 m", "bands in band order");
    if (s.size() == 2) {
        const HeardBand& b20 = s[1];
        check(b20.stations == 8 && b20.countries == 5, fmt("20 m: %.0f stations, %.0f countries", b20.stations, b20.countries));
        check(b20.hasFarthest && b20.farthest.call == "VK2OGS" && b20.farthest.km == 17003,
              "20 m farthest: " + b20.farthest.call + fmt(", %.0f km", b20.farthest.km));
        check(b20.sectors[6] == 3 && b20.openSectors.size() == 3 && b20.openSectors[0] == 6 && b20.openText == "W 3 \xC2\xB7 NE 2 \xC2\xB7 NW 2",
              "20 m open towards: " + b20.openText);
        check(b20.trend == HeardTrend::Rising, std::string("20 m trend: ") + heardTrendText(b20.trend));
        bool best = false;
        for (std::size_t i = 0; i < b20.stationList.size(); ++i)
            if (b20.stationList[i].call == "K1OGS") best = b20.stationList[i].snr == -9 && b20.stationList[i].grid == "FN42";
        check(best, "a station's best report, its grid from an earlier decode");
        check(!b20.byCountry.empty() && b20.byCountry[0].name == "USA" && b20.byCountry[0].stations == 4,
              "20 m countries, most first: " + (b20.byCountry.empty() ? std::string("?") : b20.byCountry[0].name));
        const HeardBand& b40 = s[0];
        check(b40.stations == 3 && b40.trend == HeardTrend::Fading, std::string("40 m: 3 stations, ") + heardTrendText(b40.trend));
    }
    const std::vector<HeardBand> s15 = h.summary(now, 900);
    check(s15.size() == 1 && s15[0].band == "40 m" && s15[0].stations == 1 && s15[0].trend == HeardTrend::New,
          "last 15 min: only the WSPR spot on 40 m, new");
    const std::vector<HeardBand> s3 = h.summary(now + 7200, 3 * 3600);
    check(s3.size() == 2 && s3[1].stations == 8 && s3[1].trend == HeardTrend::Fading, "last 3 h, 2 h later: all in the earlier half, fading");
    const std::string txt = h.text(now, 3600, true);
    std::printf("%s", txt.c_str());
    check(txt.find("VK2OGS, Australia, 17,003 km") != std::string::npos && txt.find("W 3") != std::string::npos,
          "the Heard table");
    check(h.stationCount(now, 3600) == 11, fmt("%.0f stations in all", h.stationCount(now, 3600)));
    h.note(t0 + 8 * 3600, 14.074e6, "FT8", "K1OGS", "FN42", -1);
    check(h.log().size() == 1, "entries over 6 hours old forgotten");
    check(h.text(t0 + 20 * 3600, 900) == "Nothing heard yet in this time.\n", "nothing heard");
    check(hamBandName(10.1387e6) == "30 m" && hamBandName(13.0e6) == "13.000 MHz", "band names");
}

void testSynthesis() {
    std::printf("-- the synthesiser\n");
    std::vector<float> a;
    check(ftx::synthesizeFt8("CQ G4OGR IO81", 1000, 48000, 0.5, a) && a.size() == static_cast<std::size_t>(79 * 0.16 * 48000),
          fmt("FT8 at 48000 Hz: %.0f samples (12.64 s)", a.size()));
    double p = 0;
    for (std::size_t i = 0; i < a.size(); ++i) p += a[i] * a[i];
    p /= a.size();
    check(std::fabs(p - 0.125) < 0.002, fmt("constant envelope: power %.4f (amplitude^2 / 2)", p));
    a.clear();
    check(ftx::synthesizeFt4("CQ G4OGR IO81", 1000, 44100, 0.5, a) &&
              a.size() == static_cast<std::size_t>(std::floor(105 * 0.048 * 44100 + 0.5)),
          fmt("FT4 at 44100 Hz (fractional samples a symbol): %.0f samples", a.size()));
    a.clear();
    check(!ftx::synthesizeFt8("THIS CANNOT BE SENT AT ALL", 1000, 48000, 0.5, a) && a.empty(), "a message that does not pack");
}

}  // namespace

int main() {
    testMessages();
    testAgainstFt8Lib();
    testCodes();
    testSynthesis();
    testFtxInfo();
    testCallsign();
    testHeard();
    testFt8();
    testFt4();
    std::printf("\n%d passed, %d failed\n", passes, failures);
    return failures ? 1 : 0;
}
