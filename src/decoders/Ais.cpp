// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Ais.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  dec::AisDecoder (see Ais.h), a port of OFFgrid-SDR's JS AIS receiver
//  (diddy-boy, https://github.com/solariun/offgrid-sdr, index.html:
//  class AisChannel, aisBits, parseAis, the "names for numbers", aisGot and
//  the AIS list's aisWhat / aisDetail):
//
//    IQ (the stick's rate, float)
//      -> sanitised (NaN/Inf -> 0)                     (Addition to the JS)
//      -> per channel (A = 161.975, B = 162.025 MHz, each when within
//         ±45% of the rate of the centre):
//           mixed down (an NCO, renormalised every block), summed in blocks
//           of D1 = round(rate / 240000) samples (a boxcar) -> ~240 kHz
//           -> 41-tap Hamming-windowed sinc low-pass (7 kHz; JS 10 kHz),
//              every 5th output computed -> ~48 kHz
//           -> FM discriminator (atan2 of the phase step), averaged over two
//              samples (Addition to the JS)
//           -> the carrier offset: measured at each burst's start (found by
//              its power), then held (Addition; the JS averages always)
//           -> the bit clock: a phase accumulator (9600 / fs2 per sample)
//              pulled towards mid-bit at each level change (crossing and
//              bit value interpolated: Addition to the JS)
//           -> NRZI (no change = 1), HDLC: six 1s then 0 = flag, five 1s
//              then 0 = a stuffed 0 (dropped), seven 1s = abort
//           -> at a flag: >= 9 bytes, the flag's 7 bits pending, CRC-16 (as
//              AX.25) right -> the message (bytes bit-reversed)
//      -> aisParse() (fields read most significant bit first), the vessel
//         table merged (aisGot), the text line, the !AIVDM sentences.
//
//  How it fits in
//  --------------
//  Implements Ais.h.  Standard library only.
//
//  Contributing
//  ------------
//  No allocation per sample (only per decoded frame and on a rate change).
//  Keep the parser, the names and the list's wording in step with the JS;
//  mark changes "Addition to the JS".  Run tests/test_decoders_ais.cpp.
// ============================================================================
#include "Ais.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <limits>
#include <map>

namespace dec {

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kPi = 3.141592653589793;
const int kTaps = 41;          // stage 2 low-pass (JS: N = 41)
const int kD2 = 5;             // stage 2 decimation (JS: D2 = 5)
// Addition to the JS: the JS gives up on a frame after 80 bytes; long
// binary messages (types 6, 8, 25, 26: up to 5 slots, 1008 bits) need more.
const int kMaxBytes = 160;
// Addition to the JS: the carrier-offset average's rates while a burst
// starts (its first kAcquire samples at ~48 kHz) and after (the JS: 0.02
// always), and the bit clock's gain.
const double kDcAcquire = 0.03, kDcTrack = 0.0005;
const int kMeanN = 20, kAcquire = 100;
const double kClockGain = 0.2;
// Addition to the JS: the stage 2 low-pass's cut-off (the JS: 10 kHz).
const double kCutoffHz = 7000;

std::string fmt(const char* f, double a) {
    char b[64];
    std::snprintf(b, sizeof b, f, a);
    return b;
}
std::string fmtI(const char* f, long a) {
    char b[64];
    std::snprintf(b, sizeof b, f, a);
    return b;
}
long roundJs(double v) { return static_cast<long>(std::floor(v + 0.5)); }   // Math.round

// "53.5210°N 3.3520°W" (as the JS list's tooltip).
std::string latLonText(double lat, double lon) {
    char b[80];
    std::snprintf(b, sizeof b, "%.4f\xC2\xB0%c %.4f\xC2\xB0%c", std::fabs(lat), lat >= 0 ? 'N' : 'S', std::fabs(lon),
                  lon >= 0 ? 'E' : 'W');
    return b;
}

// ---------------------------------------------------------------------------
// The JS aisBits: fields read most significant bit first; reads past the end
// give 0 bits.
// ---------------------------------------------------------------------------
struct Bits {
    const unsigned char* b;
    std::size_t n;
    int len;   // bits
    Bits(const unsigned char* bytes, std::size_t count) : b(bytes), n(count), len(static_cast<int>(count * 8)) {}
    unsigned long u(int s, int w) const {
        unsigned long v = 0;
        for (int k = 0; k < w; ++k) {
            const int i = s + k;
            const unsigned bit = static_cast<std::size_t>(i >> 3) < n ? (b[i >> 3] >> (7 - (i & 7))) & 1u : 0u;
            v = v * 2 + bit;
        }
        return v;
    }
    long sg(int s, int w) const {
        const unsigned long v = u(s, w);
        return v >= (1ul << (w - 1)) ? static_cast<long>(v) - static_cast<long>(1ul << w) : static_cast<long>(v);
    }
    // 6-bit text: 0..31 -> '@'..'_', 32..63 as is; cut at the first '@',
    // trimmed (the JS .replace(/@.*$/, "").trim()).
    std::string tx(int s, int chars) const {
        std::string t;
        for (int k = 0; k < chars; ++k) {
            const unsigned c = static_cast<unsigned>(u(s + 6 * k, 6));
            t += static_cast<char>(c < 32 ? c + 64 : c);
        }
        const std::size_t at = t.find('@');
        if (at != std::string::npos) t.erase(at);
        std::size_t a = 0, e = t.size();
        while (a < e && (t[a] == ' ' || t[a] == '\t')) ++a;
        while (e > a && (t[e - 1] == ' ' || t[e - 1] == '\t')) --e;
        return t.substr(a, e - a);
    }
};

// The JS aisPos: 1/10000 minute; 181 / 91 degrees mean not available.
void readPos(const Bits& B, int lonAt, int latAt, AisMessage& m) {
    const double lon = B.sg(lonAt, 28) / 600000.0, lat = B.sg(latAt, 27) / 600000.0;
    if (std::fabs(lon) <= 180 && std::fabs(lat) <= 90) {
        m.lat = lat;
        m.lon = lon;
    }
}

void readMotion(const Bits& B, int sogAt, int cogAt, int hdgAt, AisMessage& m) {
    const unsigned long sog = B.u(sogAt, 10), cog = B.u(cogAt, 12), hdg = B.u(hdgAt, 9);
    m.hasMotion = m.hasHeading = true;
    m.sogKn = sog < 1023 ? sog / 10.0 : kNaN;
    m.cogDeg = cog < 3600 ? cog / 10.0 : kNaN;
    m.headingDeg = hdg < 360 ? static_cast<double>(hdg) : kNaN;
}

// Addition to the JS: names for the message types (for "other" messages).
const char* typeName(int t) {
    static const char* const names[28] = {"",
                                          "position report",
                                          "position report",
                                          "position report",
                                          "base station report",
                                          "static and voyage data",
                                          "addressed binary message",
                                          "binary acknowledge",
                                          "binary broadcast",
                                          "SAR aircraft position",
                                          "UTC/date inquiry",
                                          "UTC/date response",
                                          "addressed safety message",
                                          "safety acknowledge",
                                          "safety broadcast",
                                          "interrogation",
                                          "assignment mode command",
                                          "DGNSS broadcast",
                                          "class B position",
                                          "extended class B position",
                                          "data link management",
                                          "aid to navigation report",
                                          "channel management",
                                          "group assignment",
                                          "class B static data",
                                          "single slot binary",
                                          "multiple slot binary",
                                          "long range position"};
    return t >= 0 && t <= 27 ? names[t] : "";
}

}  // namespace

// ===========================================================================
// Free functions: CRC, parsing, NMEA, names
// ===========================================================================
const char* aisKindName(AisKind k) {
    switch (k) {
    case AisKind::Position: return "position";
    case AisKind::Base: return "base";
    case AisKind::Static: return "static";
    case AisKind::Aton: return "aton";
    default: return "other";
    }
}

AisMessage::AisMessage()
    : type(0), repeat(0), mmsi(0), kind(AisKind::Other), channel('?'), time(0), classB(false), lat(kNaN), lon(kNaN),
      hasMotion(false), hasHeading(false), sogKn(kNaN), cogDeg(kNaN), headingDeg(kNaN), status(-1), rot(-128), imo(0),
      shipType(-1), lengthM(kNaN), beamM(kNaN), draughtM(kNaN), etaMonth(-1), etaDay(-1), etaHour(-1), etaMinute(-1),
      atonType(-1), year(-1), month(-1), day(-1), hour(-1), minute(-1), second(-1), distanceKm(kNaN), bearingDeg(kNaN) {}

AisVessel::AisVessel()
    : mmsi(0), lat(kNaN), lon(kNaN), sogKn(kNaN), cogDeg(kNaN), headingDeg(kNaN), status(-1), imo(0), shipType(-1),
      lengthM(kNaN), beamM(kNaN), draughtM(kNaN), atonType(-1), classB(false), base(false), aton(false), firstHeard(0),
      lastHeard(0), messages(0), heardA(false), heardB(false), lastType(0), distanceKm(kNaN), bearingDeg(kNaN) {}

// The JS AisCrc: CRC-16-CCITT reflected (0x8408), initial 0xFFFF, inverted.
unsigned aisCrc(const unsigned char* bytes, std::size_t n) {
    unsigned c = 0xffff;
    for (std::size_t i = 0; i < n; ++i) {
        c ^= bytes[i];
        for (int k = 0; k < 8; ++k) c = (c & 1) ? (c >> 1) ^ 0x8408 : c >> 1;
    }
    return c ^ 0xffff;
}

unsigned char aisReverse(unsigned char b) {
    unsigned r = 0;
    for (int k = 0; k < 8; ++k) r |= ((b >> k) & 1u) << (7 - k);
    return static_cast<unsigned char>(r);
}

bool aisParse(const unsigned char* bytes, std::size_t n, AisMessage& m) {
    const char ch = m.channel;
    const double t = m.time;
    m = AisMessage();
    m.channel = (ch == 'A' || ch == 'B') ? ch : '?';
    m.time = t;
    const Bits B(bytes, n);
    m.type = static_cast<int>(B.u(0, 6));
    m.repeat = static_cast<int>(B.u(6, 2));
    m.mmsi = static_cast<unsigned>(B.u(8, 30));
    if (!m.mmsi) return false;
    // Addition to the JS: type 0 and types above 27 do not exist (fewer
    // false messages out of noise that happens to pass the CRC).
    if (m.type < 1 || m.type > 27) return false;
    const int type = m.type;
    if (type >= 1 && type <= 3 && B.len >= 168) {
        m.kind = AisKind::Position;
        m.status = static_cast<int>(B.u(38, 4));
        readMotion(B, 50, 116, 128, m);
        readPos(B, 61, 89, m);
        const long rot = B.sg(42, 8);
        if (rot != -128) m.rot = static_cast<int>(rot);
    } else if (type == 18 && B.len >= 168) {
        m.kind = AisKind::Position;
        m.classB = true;
        readMotion(B, 46, 112, 124, m);
        readPos(B, 57, 85, m);
    } else if (type == 19 && B.len >= 312) {
        m.kind = AisKind::Position;
        m.classB = true;
        readMotion(B, 46, 112, 124, m);
        readPos(B, 57, 85, m);
        m.name = B.tx(143, 20);
        m.shipType = static_cast<int>(B.u(263, 8));
    } else if (type == 4 && B.len >= 168) {
        m.kind = AisKind::Base;
        readPos(B, 79, 107, m);
        m.year = static_cast<int>(B.u(38, 14));
        m.month = static_cast<int>(B.u(52, 4));
        m.day = static_cast<int>(B.u(56, 5));
        m.hour = static_cast<int>(B.u(61, 5));
        m.minute = static_cast<int>(B.u(66, 6));
        m.second = static_cast<int>(B.u(72, 6));
        char b[64];
        std::snprintf(b, sizeof b, "%d-%02d-%02d %02d:%02d:%02d UTC", m.year, m.month, m.day, m.hour, m.minute, m.second);
        m.utc = b;
    } else if (type == 5 && B.len >= 424) {
        m.kind = AisKind::Static;
        m.imo = static_cast<unsigned>(B.u(40, 30));
        m.callsign = B.tx(70, 7);
        m.name = B.tx(112, 20);
        m.shipType = static_cast<int>(B.u(232, 8));
        m.lengthM = static_cast<double>(B.u(240, 9) + B.u(249, 9));
        m.beamM = static_cast<double>(B.u(258, 6) + B.u(264, 6));
        m.draughtM = B.u(294, 8) / 10.0;
        m.destination = B.tx(302, 20);
        const int mo = static_cast<int>(B.u(274, 4)), dy = static_cast<int>(B.u(278, 5)),
                  hr = static_cast<int>(B.u(283, 5)), mi = static_cast<int>(B.u(288, 6));
        if (mo && dy && hr < 24 && mi < 60) {
            m.etaMonth = mo;
            m.etaDay = dy;
            m.etaHour = hr;
            m.etaMinute = mi;
            char b[48];
            std::snprintf(b, sizeof b, "%02d/%02d %02d:%02d UTC", dy, mo, hr, mi);
            m.eta = b;
        }
    } else if (type == 24 && B.len >= 160) {
        const unsigned part = static_cast<unsigned>(B.u(38, 2));
        if (part == 0) {
            m.kind = AisKind::Static;
            m.name = B.tx(40, 20);
        } else if (part == 1) {
            m.kind = AisKind::Static;
            m.shipType = static_cast<int>(B.u(40, 8));
            m.callsign = B.tx(90, 7);
        } else {
            return false;
        }
    } else if (type == 21 && B.len >= 272) {
        m.kind = AisKind::Aton;
        m.atonType = static_cast<int>(B.u(38, 5));
        m.name = B.tx(43, 20);
        readPos(B, 164, 192, m);
    } else if (type == 27 && B.len >= 96) {
        // Addition to the JS: type 27, long-range position (satellite AIS
        // and far-off ships): 1/10 minute, speed in knots, course in degrees.
        m.kind = AisKind::Position;
        m.status = static_cast<int>(B.u(40, 4));
        const double lon = B.sg(44, 18) / 600.0, lat = B.sg(62, 17) / 600.0;
        if (std::fabs(lon) <= 180 && std::fabs(lat) <= 90) {
            m.lat = lat;
            m.lon = lon;
        }
        const unsigned long sog = B.u(79, 6), cog = B.u(85, 9);
        m.hasMotion = true;
        m.sogKn = sog < 63 ? static_cast<double>(sog) : kNaN;
        m.cogDeg = cog < 360 ? static_cast<double>(cog) : kNaN;
    } else {
        m.kind = AisKind::Other;
    }
    m.bytes.assign(bytes, bytes + n);
    m.nmea = aisNmea(bytes, n, m.channel == '?' ? 0 : m.channel, 0);
    return true;
}

unsigned aisNmeaChecksum(const std::string& body) {
    unsigned c = 0;
    for (std::size_t i = 0; i < body.size(); ++i) c ^= static_cast<unsigned char>(body[i]);
    return c & 0xff;
}

// Addition to the JS: !AIVDM sentences (IEC 61162-1 / NMEA 0183 as AIS
// receivers send them): the bits in groups of 6, each armoured as an ASCII
// character (0..39 -> '0'..'W', 40..63 -> '`'..'w').
std::vector<std::string> aisNmea(const unsigned char* bytes, std::size_t n, char channel, int seqId) {
    std::vector<std::string> out;
    const int bits = static_cast<int>(n * 8), chars = (bits + 5) / 6, fill = chars * 6 - bits;
    std::string payload;
    payload.reserve(chars);
    const Bits B(bytes, n);
    for (int k = 0; k < chars; ++k) {
        const unsigned v = static_cast<unsigned>(B.u(6 * k, 6));
        payload += static_cast<char>(v < 40 ? v + 48 : v + 56);
    }
    const int per = 60, parts = chars == 0 ? 1 : (chars + per - 1) / per;
    for (int p = 0; p < parts; ++p) {
        std::string body = "AIVDM,";
        body += fmtI("%ld", parts) + "," + fmtI("%ld", p + 1) + ",";
        if (parts > 1) body += fmtI("%ld", ((seqId % 10) + 10) % 10);
        body += ",";
        if (channel) body += channel;
        body += ",";
        body += payload.substr(static_cast<std::size_t>(p * per), per);
        body += ",";
        body += fmtI("%ld", p == parts - 1 ? fill : 0);
        char cs[8];
        std::snprintf(cs, sizeof cs, "*%02X", aisNmeaChecksum(body));
        out.push_back("!" + body + cs);
    }
    return out;
}

// ---- names for numbers (the JS AIS_STATUS, aisTypeName, AIS_ATON, AIS_MID)
const char* aisStatusName(int s) {
    static const char* const names[9] = {"under way",          "at anchor", "not under command",
                                         "restricted manoeuvrability", "constrained by draught",
                                         "moored",             "aground",   "fishing",
                                         "under way, sailing"};
    return s >= 0 && s <= 8 ? names[s] : "";
}

std::string aisShipTypeName(int t) {
    if (t <= 0) return "";
    if (t == 30) return "fishing";
    if (t == 31 || t == 32) return "towing";
    if (t == 33) return "dredger";
    if (t == 34) return "diving";
    if (t == 35) return "military";
    if (t == 36) return "sailing";
    if (t == 37) return "pleasure craft";
    if (t >= 40 && t <= 49) return "high-speed craft";
    if (t == 50) return "pilot";
    if (t == 51) return "search and rescue";
    if (t == 52) return "tug";
    if (t == 53) return "port tender";
    if (t == 55) return "law enforcement";
    if (t == 58) return "medical";
    if (t >= 60 && t <= 69) return "passenger";
    if (t >= 70 && t <= 79) return "cargo";
    if (t >= 80 && t <= 89) return "tanker";
    return "vessel";
}

const char* aisAtonName(int t) {
    switch (t) {
    case 1: return "reference point";
    case 5: case 6: return "light";
    case 9: return "beacon";
    case 20: case 21: case 22: case 23: case 31: return "light vessel";
    case 24: case 25: case 26: case 27: case 28: case 29: case 30: return "buoy";
    default: return "";
    }
}

const char* aisFlag(unsigned mmsi) {
    struct Mid { int mid; const char* name; };
    static const Mid table[] = {
        {201, "Albania"}, {205, "Belgium"}, {209, "Cyprus"}, {210, "Cyprus"}, {211, "Germany"}, {212, "Cyprus"},
        {218, "Germany"}, {219, "Denmark"}, {220, "Denmark"}, {224, "Spain"}, {225, "Spain"}, {226, "France"},
        {227, "France"}, {228, "France"}, {229, "Malta"}, {230, "Finland"}, {231, "Faroe Islands"}, {232, "UK"},
        {233, "UK"}, {234, "UK"}, {235, "UK"}, {236, "Gibraltar"}, {237, "Greece"}, {239, "Greece"}, {240, "Greece"},
        {241, "Greece"}, {244, "Netherlands"}, {245, "Netherlands"}, {246, "Netherlands"}, {247, "Italy"},
        {248, "Malta"}, {249, "Malta"}, {250, "Ireland"}, {251, "Iceland"}, {255, "Madeira"}, {256, "Malta"},
        {257, "Norway"}, {258, "Norway"}, {259, "Norway"}, {261, "Poland"}, {263, "Portugal"}, {265, "Sweden"},
        {266, "Sweden"}, {271, "Turkey"}, {273, "Russia"}, {275, "Latvia"}, {276, "Estonia"}, {277, "Lithuania"},
        {304, "Antigua"}, {305, "Antigua"}, {308, "Bahamas"}, {309, "Bahamas"}, {311, "Bahamas"}, {316, "Canada"},
        {338, "USA"}, {351, "Panama"}, {352, "Panama"}, {353, "Panama"}, {354, "Panama"}, {355, "Panama"},
        {356, "Panama"}, {357, "Panama"}, {366, "USA"}, {367, "USA"}, {368, "USA"}, {369, "USA"}, {370, "Panama"},
        {371, "Panama"}, {372, "Panama"}, {373, "Panama"}, {412, "China"}, {413, "China"}, {431, "Japan"},
        {432, "Japan"}, {440, "South Korea"}, {441, "South Korea"}, {477, "Hong Kong"}, {503, "Australia"},
        {512, "New Zealand"}, {525, "Indonesia"}, {533, "Malaysia"}, {538, "Marshall Islands"}, {563, "Singapore"},
        {564, "Singapore"}, {565, "Singapore"}, {566, "Singapore"}, {636, "Liberia"}, {637, "Liberia"},
        {710, "Brazil"}, {725, "Chile"}};
    char s[16];
    std::snprintf(s, sizeof s, "%09u", mmsi);
    int mid;
    // 00MIDxxxx coast stations, 0MIDxxxxx groups, 97/98/99 + MID: SARTs, craft
    // associated with a parent ship, aids to navigation (as the JS).
    if (s[0] == '0' && s[1] == '0') mid = (s[2] - '0') * 100 + (s[3] - '0') * 10 + (s[4] - '0');
    else if (s[0] == '0') mid = (s[1] - '0') * 100 + (s[2] - '0') * 10 + (s[3] - '0');
    else if (s[0] == '9' && (s[1] == '7' || s[1] == '8' || s[1] == '9'))
        mid = (s[2] - '0') * 100 + (s[3] - '0') * 10 + (s[4] - '0');
    else mid = (s[0] - '0') * 100 + (s[1] - '0') * 10 + (s[2] - '0');
    for (std::size_t i = 0; i < sizeof table / sizeof table[0]; ++i)
        if (table[i].mid == mid) return table[i].name;
    return "";
}

std::string aisWhat(const AisVessel& v) {
    if (v.base) return "coast station";
    if (v.aton) {
        const char* a = aisAtonName(v.atonType);
        return *a ? a : "navigation aid";
    }
    const std::string t = aisShipTypeName(v.shipType);
    if (!t.empty()) return t;
    return v.classB ? "small craft" : "vessel";
}

std::string aisDetail(const AisVessel& v) {
    if (v.base) {
        const char* f = aisFlag(v.mmsi);
        return std::string("AIS base station") + (*f ? std::string(", ") + f : std::string());
    }
    if (v.aton) return "aid to navigation";
    std::vector<std::string> p;
    if (aisHas(v.sogKn)) {
        if (roundJs(v.sogKn * 10) >= 5)
            p.push_back(fmt("%.1f kn, course ", v.sogKn) +
                        (aisHas(v.cogDeg) ? fmtI("%ld\xC2\xB0", roundJs(v.cogDeg)) : std::string("?")));
        else
            p.push_back("stopped");
    }
    if (v.status > 0 && v.status != 15 && *aisStatusName(v.status)) p.push_back(aisStatusName(v.status));
    if (!v.destination.empty()) p.push_back("to " + v.destination);
    std::string s;
    for (std::size_t i = 0; i < p.size(); ++i) s += (i ? " \xC2\xB7 " : "") + p[i];
    return s;
}

std::string aisDescribe(const AisMessage& m) {
    std::vector<std::string> p;
    const std::string dot = " \xC2\xB7 ";
    std::string head;
    switch (m.kind) {
    case AisKind::Position:
        head = m.type == 27 ? "long-range position" : m.classB ? "class B position" : "position";
        if (!m.name.empty()) p.push_back(m.name);
        if (m.shipType > 0) p.push_back(aisShipTypeName(m.shipType));
        if (aisHas(m.lat)) p.push_back(latLonText(m.lat, m.lon));
        else p.push_back("position not available");
        if (m.hasMotion) {
            if (!aisHas(m.sogKn)) p.push_back("speed not available");
            else if (roundJs(m.sogKn * 10) >= 5)
                p.push_back(fmt("%.1f kn, course ", m.sogKn) +
                            (aisHas(m.cogDeg) ? fmtI("%ld\xC2\xB0", roundJs(m.cogDeg)) : std::string("?")));
            else p.push_back("stopped");
        }
        if (aisHas(m.headingDeg)) p.push_back(fmtI("heading %ld\xC2\xB0", roundJs(m.headingDeg)));
        if (m.status >= 0 && m.status != 15 && *aisStatusName(m.status)) p.push_back(aisStatusName(m.status));
        break;
    case AisKind::Base:
        head = "coast station";
        if (aisHas(m.lat)) p.push_back(latLonText(m.lat, m.lon));
        p.push_back("time " + m.utc);
        break;
    case AisKind::Static:
        head = m.type == 24 ? "class B static" : "static";
        if (!m.name.empty()) p.push_back(m.name);
        if (!m.callsign.empty()) p.push_back("call sign " + m.callsign);
        if (m.imo) p.push_back(fmtI("IMO %ld", static_cast<long>(m.imo)));
        if (m.shipType >= 0) {
            const std::string t = aisShipTypeName(m.shipType);
            p.push_back((t.empty() ? std::string("type") : t) + fmtI(" (%ld)", m.shipType));
        }
        if (aisHas(m.lengthM) && m.lengthM > 0)
            p.push_back(fmt("%.0f", m.lengthM) + fmt(" x %.0f m", m.beamM));
        if (aisHas(m.draughtM) && m.draughtM > 0) p.push_back(fmt("draught %.1f m", m.draughtM));
        if (!m.destination.empty()) p.push_back("to " + m.destination);
        if (!m.eta.empty()) p.push_back("ETA " + m.eta);
        break;
    case AisKind::Aton: {
        head = "aid to navigation";
        if (!m.name.empty()) p.push_back(m.name);
        const char* a = aisAtonName(m.atonType);
        p.push_back(*a ? std::string(a) : fmtI("type %ld", m.atonType));
        if (aisHas(m.lat)) p.push_back(latLonText(m.lat, m.lon));
        break;
    }
    default:
        head = fmtI("type %ld", m.type);
        if (*typeName(m.type)) head += std::string(" (") + typeName(m.type) + ")";
        p.push_back(fmtI("%ld bits", static_cast<long>(m.bytes.size() * 8)));
        break;
    }
    if (aisHas(m.distanceKm))
        p.push_back(fmt("%.0f km", m.distanceKm) + fmtI(", bearing %ld\xC2\xB0", roundJs(m.bearingDeg) % 360));
    std::string s = head;
    for (std::size_t i = 0; i < p.size(); ++i) s += (i ? dot : std::string(": ")) + p[i];
    return s;
}

// The JS distBrg (haversine, 6371 km), not rounded.
void aisDistance(double lat1, double lon1, double lat2, double lon2, double& km, double& deg) {
    const double r = kPi / 180, la1 = lat1 * r, lo1 = lon1 * r, la2 = lat2 * r, lo2 = lon2 * r;
    const double s1 = std::sin((la2 - la1) / 2), s2 = std::sin((lo2 - lo1) / 2);
    const double h = s1 * s1 + std::cos(la1) * std::cos(la2) * s2 * s2;
    km = 2 * 6371 * std::asin(std::min(1.0, std::sqrt(h)));
    const double y = std::sin(lo2 - lo1) * std::cos(la2),
                 x = std::cos(la1) * std::sin(la2) - std::sin(la1) * std::cos(la2) * std::cos(lo2 - lo1);
    deg = std::fmod(std::atan2(y, x) / r + 360, 360);
}

// ===========================================================================
// One channel: the JS class AisChannel
// ===========================================================================
namespace {

struct RawFrame {
    int ch;
    double t;   // stream seconds at the closing flag
    std::vector<unsigned char> bytes;   // as received (HDLC order), CRC removed
};

class AisChannel {
public:
    AisChannel() : rate_(0), out_(0) { std::memset(h_, 0, sizeof h_); }

    void attach(std::vector<RawFrame>* out, int ch) {
        out_ = out;
        ch_ = ch;
    }

    void reset() { rate_ = 0; }

    // The JS setup(rate).
    void setup(double rate) {
        rate_ = rate;
        D1_ = std::max(1, static_cast<int>(std::floor(rate / 240000 + 0.5)));
        fs1_ = rate / D1_;
        fs2_ = fs1_ / kD2;
        // Stage 2 low-pass: 41 taps, Hamming-windowed sinc.
        // Addition to the JS: the cut-off is 7 kHz, not the JS's 10 kHz: the
        // GMSK signal (±2.4 kHz deviation, 9600 bd, BT 0.4) is within about
        // ±6 kHz, and the narrower filter is worth some 3 dB (messages
        // decode about 3 dB weaker), still with room for a stick a few kHz
        // off frequency (4 kHz tested in the sweep that chose it).
        const double fc = kCutoffHz / fs1_;
        double sum = 0;
        for (int i = 0; i < kTaps; ++i) {
            const double x = i - (kTaps - 1) / 2.0, w = 0.54 - 0.46 * std::cos(2 * kPi * i / (kTaps - 1));
            h_[i] = (x == 0 ? 2 * fc : std::sin(2 * kPi * fc * x) / (kPi * x)) * w;
            sum += h_[i];
        }
        for (int i = 0; i < kTaps; ++i) h_[i] /= sum;
        std::memset(zr_, 0, sizeof zr_);
        std::memset(zi_, 0, sizeof zi_);
        zk_ = 0;
        k2_ = 0;
        a1r_ = a1i_ = 0;
        k1_ = 0;
        mr_ = 1;
        mi_ = 0;
        off_ = kNaN;
        pr_ = pi_ = 0;
        dc_ = 0;
        prevV_ = 0;
        pf_ = 0;
        pl_ = 0;
        dPrev_ = 0;
        sigN_ = kAcquire;
        last_ = 0;
        ph_ = 0;
        step_ = 9600 / fs2_;
        prevBit_ = 0;
        ones_ = 0;
        byte_ = 0;
        nb_ = 0;
        nBytes_ = 0;
        inFrame_ = false;
        lvl_ = 0;
    }

    // I/Q at `rate`, this channel `off` Hz from the centre.  t0: stream
    // seconds at iq[0].
    void process(const std::complex<float>* iq, std::size_t n, double rate, double off, double t0) {
        if (rate != rate_) setup(rate);
        if (off != off_) {   // (the NCO step, recomputed only on a retune)
            off_ = off;
            const double w = -2 * kPi * off / rate;
            cw_ = std::cos(w);
            sw_ = std::sin(w);
        }
        t0_ = t0;
        double mr = mr_, mi = mi_, ar = a1r_, ai = a1i_;
        int k1 = k1_;
        const int D1 = D1_;
        const double cw = cw_, sw = sw_;
        for (std::size_t j = 0; j < n; ++j) {
            double x = iq[j].real(), y = iq[j].imag();
            // Addition to the JS: NaN / Inf samples count as 0.
            if (!(std::fabs(x) <= 1e30)) x = 0;
            if (!(std::fabs(y) <= 1e30)) y = 0;
            ar += x * mr - y * mi;   // mixed down (this channel to 0 Hz)
            ai += x * mi + y * mr;
            const double t = mr * cw - mi * sw;
            mi = mr * sw + mi * cw;
            mr = t;
            if (++k1 == D1) {
                k1 = 0;
                idx_ = j;
                stage2(ar, ai);
                ar = ai = 0;
                const double g = 1.5 - 0.5 * (mr * mr + mi * mi);   // (keeps |NCO| = 1)
                mr *= g;
                mi *= g;
            }
        }
        mr_ = mr;
        mi_ = mi;
        a1r_ = ar;
        a1i_ = ai;
        k1_ = k1;
    }

    double level() const { return lvl_; }

private:
    // At about 240 kHz: the low-pass, kept every 5th output.
    void stage2(double r, double i) {
        zr_[zk_] = r;
        zi_[zk_] = i;
        if (++zk_ == kTaps) zk_ = 0;
        if (++k2_ < kD2) return;
        k2_ = 0;
        double sr = 0, si = 0;
        for (int j = 0, k = zk_; j < kTaps; ++j) {
            const double v = h_[j];
            sr += v * zr_[k];
            si += v * zi_[k];
            if (++k == kTaps) k = 0;
        }
        // The FM discriminator: the phase step from the last sample.
        // Addition to the JS: averaged over two samples (the
        // discriminator's noise rises with frequency; ~1 dB better).
        const double d0 = std::atan2(si * pr_ - sr * pi_, sr * pr_ + si * pi_);
        const double d = 0.5 * (d0 + dPrev_);
        dPrev_ = d0;
        pr_ = sr;
        pi_ = si;
        const double p = sr * sr + si * si;
        lvl_ = lvl_ * 0.995 + p * 0.005;
        if (!(lvl_ <= 1e300)) lvl_ = 0;   // (Addition to the JS: never Inf)
        // The carrier's offset, followed.  Addition to the JS: the JS
        // averages the discriminator all the time (rate 0.02).  Between
        // bursts that average wanders with the noise (whose phase steps are
        // anywhere in ±pi) and a burst's 24 training bits are not enough to
        // forget it; within a burst, runs of equal bits and messages with
        // more 1 than 0 levels pull it.  Off centre, the slicer moves the
        // rising and falling crossings opposite ways, the bit clock can sit
        // on the transitions, and the flag is missed.  Here a burst's start
        // is found by its power (the last ~5 samples' average 8 dB over the
        // last ~200's), and the offset is then the plain mean of its first
        // 20 samples (a period of the training sequence's 1100 levels),
        // followed quickly to the 100th sample (the training sequence), and
        // only very slowly after (the offset of one burst is constant).
        pf_ += (p - pf_) * 0.2;
        pl_ += (p - pl_) * 0.005;
        if (sigN_ >= kAcquire && pf_ > 6 * pl_) sigN_ = 0;   // (a burst starts)
        {
            const double a = sigN_ < kMeanN ? 1.0 / (sigN_ + 1) : sigN_ < kAcquire ? kDcAcquire : kDcTrack;
            dc_ += (d - dc_) * a;
            if (sigN_ < kAcquire) ++sigN_;
        }
        const double v = d - dc_;
        const int lv = v > 0 ? 1 : 0;
        // The bit clock: phase 0 (1) = mid-bit, 0.5 = a transition.
        // Addition to the JS: the crossing's time is interpolated between
        // the two samples, the clock pulled by a fraction of its error (the
        // JS pulls 30% towards mid-bit from the sample after the crossing,
        // which jitters by up to a fifth of a bit at 4.7 samples a bit),
        // and the bit's value is interpolated at mid-bit.
        if (lv != last_) {
            const double a = std::fabs(v) / (std::fabs(v) + std::fabs(prevV_) + 1e-30);   // samples since it
            double e = ph_ + step_ * (1 - a) - 0.5;
            e -= std::floor(e + 0.5);
            ph_ -= kClockGain * e;
            last_ = lv;
        }
        ph_ += step_;
        if (ph_ >= 1) {
            ph_ -= 1;
            const double o = ph_ / step_;   // how far past mid-bit, in samples
            const double mid = v - o * (v - prevV_);
            bit(mid > 0 ? 1 : 0);
        }
        prevV_ = v;
    }

    void bit(int lv) {
        const int b = lv == prevBit_ ? 1 : 0;   // NRZI: no change is a 1
        prevBit_ = lv;
        if (b) {
            if (++ones_ > 6) {   // (an abort / idle: seven 1s)
                inFrame_ = false;
                nBytes_ = 0;
            }
        } else {
            if (ones_ == 6) {
                flag();
                ones_ = 0;
                return;
            }
            if (ones_ == 5) {   // (a stuffed 0)
                ones_ = 0;
                return;
            }
            ones_ = 0;
        }
        if (!inFrame_) return;
        byte_ |= b << nb_;
        if (++nb_ == 8) {
            bytes_[nBytes_++] = static_cast<unsigned char>(byte_);
            byte_ = 0;
            nb_ = 0;
            if (nBytes_ >= kMaxBytes) {
                inFrame_ = false;
                nBytes_ = 0;
            }
        }
    }

    void flag() {
        const int n = nBytes_;
        if (inFrame_ && n >= 9 && nb_ == 7 && out_) {
            const unsigned fcs = bytes_[n - 2] | (bytes_[n - 1] << 8);
            if (aisCrc(bytes_, n - 2) == fcs) {
                RawFrame f;
                f.ch = ch_;
                f.t = t0_ + (idx_ + 1) / rate_;
                f.bytes.assign(bytes_, bytes_ + n - 2);
                out_->push_back(f);
            }
        }
        inFrame_ = true;
        nBytes_ = 0;
        byte_ = 0;
        nb_ = 0;
    }

    double rate_;
    std::vector<RawFrame>* out_;
    int ch_;
    int D1_;
    double fs1_, fs2_;
    double h_[kTaps], zr_[kTaps], zi_[kTaps];
    int zk_, k2_;
    double a1r_, a1i_;
    int k1_;
    double mr_, mi_, off_, cw_, sw_;
    double pr_, pi_, dc_, prevV_, pf_, pl_, dPrev_;
    int sigN_;
    int last_;
    double ph_, step_;
    int prevBit_, ones_, byte_, nb_;
    unsigned char bytes_[kMaxBytes];
    int nBytes_;
    bool inFrame_;
    double lvl_;
    double t0_;
    std::size_t idx_;
};

}  // namespace

// ===========================================================================
// The decoder
// ===========================================================================
struct AisDecoder::Impl {
    AisChannel ch[2];
    bool inView[2];
    std::vector<RawFrame> frames;
    double streamT;           // seconds of IQ since reset()
    bool haveTs;
    double tsUnix, tsStream;  // setTimestamp(): Unix time at stream time
    double lastMsgT;          // stream seconds of the last message (-1 none)
    std::map<unsigned, AisVessel> ships;
    std::deque<AisMessage> recent;
    std::vector<AisMessage> fresh;
    Impl() : streamT(0), haveTs(false), tsUnix(0), tsStream(0), lastMsgT(-1) {
        inView[0] = inView[1] = false;
        frames.reserve(16);
    }
};

AisDecoder::AisDecoder()
    : impl_(new Impl), haveHome_(false), homeLat_(0), homeLon_(0), nmeaInText_(false), maxMessages_(200),
      maxVessels_(1000), count_(0), countA_(0), countB_(0), seqId_(0) {
    impl_->ch[0].attach(&impl_->frames, 0);
    impl_->ch[1].attach(&impl_->frames, 1);
}

AisDecoder::~AisDecoder() {}

void AisDecoder::reset() {
    impl_->ch[0].reset();
    impl_->ch[1].reset();
    impl_->inView[0] = impl_->inView[1] = false;
    impl_->frames.clear();
    impl_->streamT = 0;
    impl_->haveTs = false;
    impl_->lastMsgT = -1;
    impl_->ships.clear();
    impl_->recent.clear();
    impl_->fresh.clear();
    count_ = countA_ = countB_ = 0;
    seqId_ = 0;
    text_.clear();
    nmea_.clear();
}

void AisDecoder::process(const float* audio, std::size_t n, double rate) {
    (void)audio;
    (void)n;
    (void)rate;
}

void AisDecoder::setTimestamp(double unixSeconds) {
    if (!(unixSeconds == unixSeconds)) return;
    impl_->haveTs = true;
    impl_->tsUnix = unixSeconds;
    impl_->tsStream = impl_->streamT;
}

void AisDecoder::processIq(const std::complex<float>* iq, std::size_t n, double rate, double centerHz, double dialHz) {
    (void)dialHz;
    if (!iq || !n || !(rate >= 1000 && rate <= 1e9) || !(centerHz == centerHz)) return;
    static const double chHz[2] = {kAisChannelA, kAisChannelB};
    const double t0 = impl_->streamT;
    for (int k = 0; k < 2; ++k) {
        const double off = chHz[k] - centerHz;
        impl_->inView[k] = std::fabs(off) < rate * 0.45;   // (as the JS aisFeed)
        if (impl_->inView[k]) impl_->ch[k].process(iq, n, rate, off, t0);
    }
    impl_->streamT += n / rate;
    if (impl_->frames.empty()) return;
    // The two channels' frames in time order.
    std::vector<RawFrame>& f = impl_->frames;
    std::stable_sort(f.begin(), f.end(), [](const RawFrame& a, const RawFrame& b) { return a.t < b.t; });
    for (std::size_t i = 0; i < f.size(); ++i) {
        for (std::size_t j = 0; j < f[i].bytes.size(); ++j) f[i].bytes[j] = aisReverse(f[i].bytes[j]);
        gotFrame(f[i].ch, f[i].bytes.empty() ? 0 : &f[i].bytes[0], f[i].bytes.size(), f[i].t);
    }
    f.clear();
}

// The JS aisGot, plus the text and NMEA lines.
void AisDecoder::gotFrame(int ch, const unsigned char* bytes, std::size_t n, double t) {
    AisMessage m;
    m.channel = ch ? 'B' : 'A';
    m.time = impl_->haveTs ? impl_->tsUnix + (t - impl_->tsStream) : t;
    if (!aisParse(bytes, n, m)) return;
    if (m.nmea.size() > 1) {   // (multi-sentence: a sequential id 0..9)
        m.nmea = aisNmea(bytes, n, m.channel, seqId_);
        seqId_ = (seqId_ + 1) % 10;
    }
    if (haveHome_ && aisHas(m.lat)) aisDistance(homeLat_, homeLon_, m.lat, m.lon, m.distanceKm, m.bearingDeg);
    ++count_;
    if (ch) ++countB_;
    else ++countA_;
    impl_->lastMsgT = t;

    // ---- the vessel table (aisGot): one entry per MMSI ----
    std::map<unsigned, AisVessel>::iterator it = impl_->ships.find(m.mmsi);
    if (it == impl_->ships.end()) {
        if (impl_->ships.size() >= maxVessels_ && maxVessels_ > 0) {   // (Addition: a bounded list)
            std::map<unsigned, AisVessel>::iterator old = impl_->ships.begin();
            for (std::map<unsigned, AisVessel>::iterator q = impl_->ships.begin(); q != impl_->ships.end(); ++q)
                if (q->second.lastHeard < old->second.lastHeard) old = q;
            impl_->ships.erase(old);
        }
        AisVessel nv;
        nv.mmsi = m.mmsi;
        nv.firstHeard = m.time;
        it = impl_->ships.insert(std::make_pair(m.mmsi, nv)).first;
    }
    AisVessel& v = it->second;
    ++v.messages;
    v.lastHeard = m.time;
    v.lastType = m.type;
    if (ch) v.heardB = true;
    else v.heardA = true;
    if (aisHas(m.lat)) {
        v.lat = m.lat;
        v.lon = m.lon;
    }
    if (m.hasMotion) {
        v.sogKn = m.sogKn;
        v.cogDeg = m.cogDeg;
    }
    if (m.hasHeading) v.headingDeg = m.headingDeg;
    if (m.status >= 0) v.status = m.status;
    if (!m.name.empty()) v.name = m.name;
    if (!m.callsign.empty()) v.callsign = m.callsign;
    if (m.type == 5) v.imo = m.imo;
    if (m.shipType >= 0) v.shipType = m.shipType;
    if (aisHas(m.lengthM)) v.lengthM = m.lengthM;
    if (aisHas(m.beamM)) v.beamM = m.beamM;
    if (aisHas(m.draughtM)) v.draughtM = m.draughtM;
    if (!m.destination.empty()) v.destination = m.destination;
    if (!m.eta.empty()) v.eta = m.eta;
    if (m.atonType >= 0) v.atonType = m.atonType;
    if (m.classB) v.classB = true;
    if (m.kind == AisKind::Base) v.base = true;
    if (m.kind == AisKind::Aton) v.aton = true;

    // ---- the text line ----
    std::string line = "AIS ";
    line += m.channel;
    line += fmtI(" %09ld", static_cast<long>(m.mmsi));
    const std::string what = aisWhat(v);
    const char* flag = aisFlag(m.mmsi);
    if (!v.name.empty()) line += " " + v.name;
    line += " (" + what + (*flag ? std::string(", ") + flag : std::string()) + "): ";
    line += aisDescribe(m);
    text_ += line + "\n";
    for (std::size_t i = 0; i < m.nmea.size(); ++i) {
        nmea_.push_back(m.nmea[i]);
        if (nmeaInText_) text_ += m.nmea[i] + "\n";
    }

    impl_->recent.push_back(m);
    while (impl_->recent.size() > maxMessages_) impl_->recent.pop_front();
    impl_->fresh.push_back(m);
    if (impl_->fresh.size() > 10000) impl_->fresh.erase(impl_->fresh.begin());   // (nobody taking them)
    if (nmea_.size() > 20000) nmea_.erase(nmea_.begin(), nmea_.begin() + 10000);
    if (text_.size() > (1u << 20)) text_.erase(0, text_.size() - (1u << 19));
}

void AisDecoder::takeText(std::string& out) {
    out += text_;
    text_.clear();
}

Status AisDecoder::status() const {
    Status s;
    const double since = impl_->lastMsgT < 0 ? 1e9 : impl_->streamT - impl_->lastMsgT;
    s.locked = since < 60;
    s.signal = since < 60;
    s.baud = 9600;
    s.characters = count_;
    s.mode = "AIS";
    s.quality = s.locked ? 1 : 0;
    const std::size_t nv = impl_->ships.size();
    char b[200];
    if (!impl_->inView[0] && !impl_->inView[1] && impl_->streamT > 0)
        std::snprintf(b, sizeof b, "AIS channels not in view: centre the stick on 162.000 MHz");
    else if (nv)
        std::snprintf(b, sizeof b, "%u vessel%s and stations \xC2\xB7 %u message%s (A %u, B %u)%s", static_cast<unsigned>(nv),
                      nv == 1 ? "" : "s", count_, count_ == 1 ? "" : "s", countA_, countB_,
                      impl_->inView[0] && impl_->inView[1] ? "" : impl_->inView[0] ? " \xC2\xB7 B not in view" : " \xC2\xB7 A not in view");
    else
        std::snprintf(b, sizeof b, "listening on both channels (161.975 and 162.025 MHz)\xE2\x80\xA6");
    s.info = b;
    return s;
}

void AisDecoder::setHome(double lat, double lon) {
    if (!(std::fabs(lat) <= 90) || !(std::fabs(lon) <= 180)) return;
    haveHome_ = true;
    homeLat_ = lat;
    homeLon_ = lon;
}

void AisDecoder::clearHome() { haveHome_ = false; }

void AisDecoder::setMaxMessages(std::size_t n) {
    maxMessages_ = n;
    while (impl_->recent.size() > maxMessages_) impl_->recent.pop_front();
}

void AisDecoder::setMaxVessels(std::size_t n) { maxVessels_ = n ? n : 1; }

std::vector<AisVessel> AisDecoder::vessels() const {
    std::vector<AisVessel> out;
    out.reserve(impl_->ships.size());
    for (std::map<unsigned, AisVessel>::const_iterator it = impl_->ships.begin(); it != impl_->ships.end(); ++it) {
        out.push_back(it->second);
        AisVessel& v = out.back();
        if (haveHome_ && aisHas(v.lat)) aisDistance(homeLat_, homeLon_, v.lat, v.lon, v.distanceKm, v.bearingDeg);
    }
    // The most recently heard first (the JS list's order).
    std::stable_sort(out.begin(), out.end(),
                     [](const AisVessel& a, const AisVessel& b) { return a.lastHeard > b.lastHeard; });
    return out;
}

bool AisDecoder::vessel(unsigned mmsi, AisVessel& out) const {
    std::map<unsigned, AisVessel>::const_iterator it = impl_->ships.find(mmsi);
    if (it == impl_->ships.end()) return false;
    out = it->second;
    if (haveHome_ && aisHas(out.lat)) aisDistance(homeLat_, homeLon_, out.lat, out.lon, out.distanceKm, out.bearingDeg);
    return true;
}

void AisDecoder::clearVessels() { impl_->ships.clear(); }

std::vector<AisMessage> AisDecoder::messages() const {
    return std::vector<AisMessage>(impl_->recent.begin(), impl_->recent.end());
}

void AisDecoder::takeMessages(std::vector<AisMessage>& out) {
    out.insert(out.end(), impl_->fresh.begin(), impl_->fresh.end());
    impl_->fresh.clear();
}

void AisDecoder::takeNmea(std::vector<std::string>& out) {
    out.insert(out.end(), nmea_.begin(), nmea_.end());
    nmea_.clear();
}

bool AisDecoder::channelInView(char channel) const { return impl_->inView[channel == 'B' ? 1 : 0]; }

}  // namespace dec
