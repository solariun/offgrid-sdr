// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Ax25Aprs.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  AX.25 frame parsing into dec::Ax25Frame and the APRS interpretation of
//  the information field (see Ax25.h).  The APRS part is a line-by-line port
//  of OFFgrid-SDR's JS (diddy-boy, https://github.com/solariun/offgrid-sdr,
//  index.html, sections "AX.25: destination, source, the digipeater path"
//  and "APRS: what the information field says", plus aprsWhat, aprsWxText,
//  aprsDetail and distBrg); each JS regular expression is written out as a
//  small matcher, with the expression quoted above it.  Written from the
//  published AX.25 and APRS (APRS101, Mic-E) specifications, as the JS was.
//  The frame bytes are decoded by AX25Toolkit's ax25::Frame (ax25/ax25frame.h).
//
//  Differences from the JS:
//    - non-APRS frames (connected-mode I, RR, SABM ..., or UI with another
//      PID) are kept (as AX.25 frames, AprsKind::None) instead of dropped;
//      all callsigns (not only the source) must be A-Z 0-9;
//    - ")" items are AprsKind::Item (the JS calls them objects);
//    - the readable line also has the position, distance and bearing (the
//      JS shows those in separate columns and a tooltip).
//
//  How it fits in
//  --------------
//  Part of the AX.25 decoder (Ax25.h); used by Ax25.cpp.  Standard library
//  and ax25/ax25frame.h only.
//
//  Contributing
//  ------------
//  Keep in step with the JS parseAprs; tests/test_decoders_ax25.cpp checks
//  each kind of packet.
// ============================================================================
#include "Ax25.h"

#include "ax25/ax25frame.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace dec {

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kPi = 3.141592653589793;

// JS Math.round: halves go up.
double jsRound(double v) { return std::floor(v + 0.5); }

bool isDigit(char c) { return c >= '0' && c <= '9'; }
bool isUpper(char c) { return c >= 'A' && c <= 'Z'; }
bool isWord(char c) { return isDigit(c) || isUpper(c) || (c >= 'a' && c <= 'z') || c == '_'; }
bool inSet(char c, const char* set) {
    for (; *set; ++set)
        if (*set == c) return true;
    return false;
}

// String.prototype.trim (ASCII whitespace).
std::string trim(const std::string& s) {
    std::size_t a = 0, b = s.size();
    while (a < b && inSet(s[a], " \t\r\n\v\f")) ++a;
    while (b > a && inSet(s[b - 1], " \t\r\n\v\f")) --b;
    return s.substr(a, b - a);
}

// Number formatting as JS prints a value rounded to 0.1: "12.2", "12", "-3.5".
std::string num(double v) {
    char b[32];
    const double r = jsRound(v * 10) / 10;
    if (r == std::floor(r)) std::snprintf(b, sizeof b, "%.0f", r);
    else std::snprintf(b, sizeof b, "%.1f", r);
    return b;
}
// An integer with thousands separators (toLocaleString in English).
std::string grouped(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.0f", std::fabs(jsRound(v)));
    std::string d(b), out;
    for (std::size_t i = 0; i < d.size(); ++i) {
        if (i && (d.size() - i) % 3 == 0) out += ',';
        out += d[i];
    }
    return (v < 0 && jsRound(v) != 0 ? "-" : "") + out;
}

// A field of `n` digits at s[at] (or all '.' / all ' ' when `blank` lists
// them): returns true and the value (NaN when blank).
bool digitsOrBlank(const std::string& s, std::size_t at, std::size_t n, const char* blank, double& v) {
    if (at + n > s.size()) return false;
    bool dig = true;
    for (std::size_t k = 0; k < n; ++k) dig = dig && isDigit(s[at + k]);
    if (dig) { v = std::atof(s.substr(at, n).c_str()); return true; }
    for (const char* b = blank; *b; ++b) {
        bool all = true;
        for (std::size_t k = 0; k < n; ++k) all = all && s[at + k] == *b;
        if (all) { v = kNaN; return true; }
    }
    return false;
}

// APRS_SYM (primary table).
const char* symbolName(char c) {
    switch (c) {
        case '>': return "car";            case '<': return "motorcycle";  case '-': return "house";
        case '_': return "weather station"; case 'k': return "truck";      case 'u': return "truck";
        case 'v': return "van";            case 'j': return "jeep";        case 'b': return "bicycle";
        case '[': return "walker";         case 'Y': return "yacht";       case 's': return "boat";
        case 'O': return "balloon";        case '\'': return "aircraft";   case '^': return "aircraft";
        case 'X': return "helicopter";     case '#': return "digipeater";  case '&': return "gateway";
        case 'r': return "repeater";       case 'y': return "station";     case 'a': return "ambulance";
        case 'f': return "fire engine";    case 'U': return "bus";         case 'R': return "motorhome";
        case '=': return "train";          case '!': return "police";      case '/': return "dot";
        case '`': return "antenna";        case 'l': return "laptop";      case '$': return "phone";
        case 'n': return "node";
        default: return 0;
    }
}

// A position parsed from the start of a string (aprsLatLon / aprsCompressed).
struct Pos {
    Pos() : lat(kNaN), lon(kNaN), table(0), sym(0), course(kNaN), speed(kNaN), alt(kNaN) {}
    double lat, lon;
    char table, sym;
    double course, speed, alt;
    std::string rest;
};

// aprsLatLon: "5328.80N/00214.40W>"
// /^(\d{2})([\d ]{2}\.[\d ]{2})([NS])(.)(\d{3})([\d ]{2}\.[\d ]{2})([EW])(.)/
bool latLon(const std::string& s, Pos& p) {
    if (s.size() < 19) return false;
    const char* d = s.c_str();
    const int dsp[] = {2, 3, 5, 6}, dspLon[] = {12, 13, 15, 16};
    if (!isDigit(d[0]) || !isDigit(d[1]) || d[4] != '.' || (d[7] != 'N' && d[7] != 'S')) return false;
    for (int k = 0; k < 4; ++k)
        if (!isDigit(d[dsp[k]]) && d[dsp[k]] != ' ') return false;
    if (d[8] == '\n') return false;
    if (!isDigit(d[9]) || !isDigit(d[10]) || !isDigit(d[11]) || d[14] != '.' || (d[17] != 'E' && d[17] != 'W'))
        return false;
    for (int k = 0; k < 4; ++k)
        if (!isDigit(d[dspLon[k]]) && d[dspLon[k]] != ' ') return false;
    if (d[18] == '\n') return false;
    // (spaces: position ambiguity; read as 0, as the JS does)
    std::string latMin = s.substr(2, 5), lonMin = s.substr(12, 5);
    for (std::size_t k = 0; k < 5; ++k) {
        if (latMin[k] == ' ') latMin[k] = '0';
        if (lonMin[k] == ' ') lonMin[k] = '0';
    }
    const double lat = ((d[0] - '0') * 10 + (d[1] - '0') + std::atof(latMin.c_str()) / 60) * (d[7] == 'S' ? -1 : 1);
    const double lon = ((d[9] - '0') * 100 + (d[10] - '0') * 10 + (d[11] - '0') + std::atof(lonMin.c_str()) / 60) *
                       (d[17] == 'W' ? -1 : 1);
    if (!(std::fabs(lat) <= 90 && std::fabs(lon) <= 180)) return false;
    p.lat = lat;
    p.lon = lon;
    p.table = d[8];
    p.sym = d[18];
    p.rest = s.substr(19);
    return true;
}

double base91(const std::string& s, std::size_t at, std::size_t n) {
    double v = 0;
    for (std::size_t k = 0; k < n; ++k) v = v * 91 + (static_cast<unsigned char>(s[at + k]) - 33);
    return v;
}

// aprsCompressed: "/YYYYXXXX$csT", base 91.
bool compressed(const std::string& s, Pos& p) {
    if (s.size() < 13) return false;
    const char t = s[0];
    if (!(t == '/' || t == '\\' || isUpper(t) || (t >= 'a' && t <= 'j'))) return false;
    const double lat = 90 - base91(s, 1, 4) / 380926, lon = -180 + base91(s, 5, 4) / 190463;
    if (!(std::fabs(lat) <= 90 && std::fabs(lon) <= 180)) return false;
    p.lat = lat;
    p.lon = lon;
    p.table = t;
    p.sym = s[9];
    p.rest = s.substr(13);
    const int c = static_cast<unsigned char>(s[10]) - 33, sp = static_cast<unsigned char>(s[11]) - 33,
              T = static_cast<unsigned char>(s[12]) - 33;
    // (the type byte says what the two "cs" bytes hold: an altitude, or a
    // course and speed)
    if (s[10] != ' ' && (T & 0x18) == 0x10) p.alt = jsRound(std::pow(1.002, c * 91 + sp) * 0.3048);
    else if (s[10] != ' ' && c >= 0 && c <= 89) {
        p.course = c * 4;
        p.speed = jsRound((std::pow(1.08, sp) - 1) * 1.852);
    }
    return true;
}

// aprsWx: "ddd/sssgGGGtTTTrRRRpPPPPPPhHHbBBBBB" (US units).  True when any
// value was read; `t` is left with what follows.
bool weather(std::string& t, AprsWeather& wx) {
    double a = kNaN, b = kNaN;
    // /^(\d{3}|\.{3}| {3})\/(\d{3}|\.{3}| {3})/
    if (digitsOrBlank(t, 0, 3, ". ", a) && t.size() > 3 && t[3] == '/' && digitsOrBlank(t, 4, 3, ". ", b)) {
        if (aprsHas(a)) wx.windDirDeg = a;
        if (aprsHas(b)) wx.windKmh = jsRound(b * 1.609);
        t = t.substr(7);
    } else if (!t.empty() && t[0] == 'c' && digitsOrBlank(t, 1, 3, ".", a) && t.size() > 4 && t[4] == 's' &&
               digitsOrBlank(t, 5, 3, ".", b)) {   // /^c(\d{3}|\.{3})s(\d{3}|\.{3})/
        if (aprsHas(a)) wx.windDirDeg = a;
        if (aprsHas(b)) wx.windKmh = jsRound(b * 1.609);
        t = t.substr(8);
    }
    for (int guard = 0; guard < 12 && !t.empty(); ++guard) {
        const char k = t[0];
        double v = kNaN;
        std::size_t len = 0;
        if ((k == 'g' || k == 'r' || k == 'p' || k == 'P') && digitsOrBlank(t, 1, 3, ".", v)) len = 4;
        else if (k == 't') {
            // (-?\d{2,3}|\.{3})
            std::size_t at = 1;
            if (at < t.size() && t[at] == '-') ++at;
            std::size_t nd = 0;
            while (nd < 3 && at + nd < t.size() && isDigit(t[at + nd])) ++nd;
            if (nd >= 2) { v = std::atof(t.substr(1, at - 1 + nd).c_str()); len = at + nd; }
            else if (digitsOrBlank(t, 1, 3, ".", v) && !aprsHas(v)) len = 4;
        } else if (k == 'h' && digitsOrBlank(t, 1, 2, ".", v)) len = 3;
        else if (k == 'b' && digitsOrBlank(t, 1, 5, ".", v)) len = 6;
        else if ((k == 'L' || k == 'l') && digitsOrBlank(t, 1, 3, "", v)) len = 4;   // luminosity: skipped
        if (!len) break;
        t = t.substr(len);
        if (!aprsHas(v)) continue;
        switch (k) {
            case 'g': wx.gustKmh = jsRound(v * 1.609); break;
            case 't': wx.tempC = jsRound((v - 32) / 1.8 * 10) / 10; break;
            case 'r': wx.rain1hMm = jsRound(v * 0.254 * 10) / 10; break;
            case 'p': wx.rain24hMm = jsRound(v * 0.254 * 10) / 10; break;
            case 'P': wx.rainDayMm = jsRound(v * 0.254 * 10) / 10; break;
            case 'h': wx.humidityPct = v == 0 ? 100 : v; break;
            case 'b': wx.pressureHpa = v / 10; break;
            default: break;
        }
    }
    return wx.any();
}

// aprsExtras: course/speed, altitude, weather in the comment.
void extras(AprsReport& r, std::string t) {
    // /^(\d{3})\/(\d{3})/
    if (t.size() >= 7 && isDigit(t[0]) && isDigit(t[1]) && isDigit(t[2]) && t[3] == '/' && isDigit(t[4]) &&
        isDigit(t[5]) && isDigit(t[6]) && r.symCode != '_') {
        r.courseDeg = static_cast<int>(std::atof(t.substr(0, 3).c_str())) % 360;
        r.speedKmh = jsRound(std::atof(t.substr(4, 3).c_str()) * 1.852);
        t = t.substr(7);
    }
    // /\/A=(-?\d{6})/
    for (std::size_t i = 0; i + 9 <= t.size(); ++i) {
        if (t[i] != '/' || t[i + 1] != 'A' || t[i + 2] != '=') continue;
        std::size_t at = i + 3;
        if (t[at] == '-') ++at;
        if (at + 6 > t.size()) continue;
        bool ok = true;
        for (std::size_t k = 0; k < 6; ++k) ok = ok && isDigit(t[at + k]);
        if (!ok) continue;
        r.altitudeM = jsRound(std::atof(t.substr(i + 3, at + 6 - (i + 3)).c_str()) * 0.3048);
        t.erase(i, at + 6 - i);
        break;
    }
    if (r.symCode == '_') {
        AprsWeather w;
        std::string u = t;
        if (weather(u, w)) { r.wx = w; t = u; }
    }
    r.comment = trim(t);
}

void applyPos(AprsReport& r, const Pos& p) {
    r.lat = p.lat;
    r.lon = p.lon;
    r.symTable = p.table;
    r.symCode = p.sym;
    if (aprsHas(p.course)) r.courseDeg = p.course;
    if (aprsHas(p.speed)) r.speedKmh = p.speed;
    if (aprsHas(p.alt)) r.altitudeM = p.alt;
}

// A position at the start of s: uncompressed if it starts with a digit.
bool position(const std::string& s, Pos& p) {
    return !s.empty() && isDigit(s[0]) ? latLon(s, p) : compressed(s, p);
}

// aprsMicE: the latitude is in the destination.
bool micE(const std::string& dst, const std::string& info, AprsReport& r) {
    std::string d = dst.substr(0, dst.find('-'));
    while (d.size() < 6) d += ' ';
    if (d.size() < 6 || info.size() < 9) return false;
    int ds[6];
    for (int k = 0; k < 6; ++k) {
        const char c = d[k];
        ds[k] = isDigit(c) ? c - '0' : (c >= 'A' && c <= 'J') ? c - 'A' : (c >= 'P' && c <= 'Y') ? c - 'P'
              : (c == 'K' || c == 'L' || c == 'Z' || c == ' ') ? 0 : -1;
        if (ds[k] < 0) return false;
    }
    double lat = ds[0] * 10 + ds[1] + (ds[2] * 10 + ds[3] + (ds[4] * 10 + ds[5]) / 100.0) / 60;
    const bool north = d[3] >= 'P' && d[3] <= 'Z', west = d[5] >= 'P' && d[5] <= 'Z';
    const int off = d[4] >= 'P' && d[4] <= 'Z' ? 100 : 0;
    if (!north) lat = -lat;
    struct C {
        const std::string& s;
        int operator()(std::size_t k) const { return static_cast<unsigned char>(s[k]) - 28; }
    } c = {info};
    int ldeg = c(1) + off;
    if (ldeg >= 180 && ldeg <= 189) ldeg -= 80;
    else if (ldeg >= 190 && ldeg <= 199) ldeg -= 190;
    int lmin = c(2);
    if (lmin >= 60) lmin -= 60;
    double lon = ldeg + (lmin + c(3) / 100.0) / 60;
    if (west) lon = -lon;
    int sp = c(4) * 10 + static_cast<int>(std::floor(c(5) / 10.0)), cse = (c(5) % 10) * 100 + c(6);
    if (sp >= 800) sp -= 800;
    if (cse >= 400) cse -= 400;
    if (!(std::fabs(lat) <= 90 && std::fabs(lon) <= 180)) return false;
    int bits[3];
    for (int k = 0; k < 3; ++k) {
        const char ch = d[k];
        bits[k] = (ch >= 'P' && ch <= 'Z') ? 1 : (ch >= 'A' && ch <= 'K') ? 2 : 0;
    }
    // (the three message bits, A B C: 111 Off duty, 110 En route, 101 In
    // service, 100 Returning, 011 Committed, 010 Special, 001 Priority,
    // 000 Emergency)
    static const char* const kMsg[8] = {"Emergency", "Priority", "Special", "Committed",
                                        "Returning", "In service", "En route", "Off duty"};
    const bool standard = bits[0] != 2 && bits[1] != 2 && bits[2] != 2;
    r.micE = standard ? kMsg[(bits[0] ? 4 : 0) + (bits[1] ? 2 : 0) + (bits[2] ? 1 : 0)] : "custom";
    r.lat = lat;
    r.lon = lon;
    r.symCode = info[7];
    r.symTable = info[8];
    r.speedKmh = jsRound(sp * 1.852);
    r.courseDeg = cse % 360;
    std::string t = info.substr(9);
    // /^(.{3})\}/: altitude, base 91, metres above -10 km
    if (t.size() >= 4 && t[3] == '}' && t[0] != '\n' && t[1] != '\n' && t[2] != '\n') {
        r.altitudeM = base91(t, 0, 3) - 10000;
        t = t.substr(4);
    }
    // /^[\]`'>=]|[`'"\]=>_]$/g: one type character at each end
    if (!t.empty() && inSet(t[0], "]`'>=")) t.erase(0, 1);
    if (!t.empty() && inSet(t[t.size() - 1], "`'\"]=>_")) t.erase(t.size() - 1);
    r.comment = trim(t);
    return true;
}

bool callOk(const std::string& c) {
    if (c.empty() || c.size() > 6) return false;
    for (std::size_t k = 0; k < c.size(); ++k)
        if (!isDigit(c[k]) && !isUpper(c[k])) return false;
    return true;
}

std::string addrStr(const std::string& call, int ssid) {
    if (!ssid) return call;
    char b[8];
    std::snprintf(b, sizeof b, "-%d", ssid);
    return call + b;
}

void parseInto(const std::string& src, const std::string& dst, const std::string& info, AprsReport& r, int depth);

}  // namespace

// ---------------------------------------------------------------------------

AprsWeather::AprsWeather()
    : windDirDeg(kNaN), windKmh(kNaN), gustKmh(kNaN), tempC(kNaN), rain1hMm(kNaN), rain24hMm(kNaN), rainDayMm(kNaN),
      humidityPct(kNaN), pressureHpa(kNaN) {}

bool AprsWeather::any() const {
    return aprsHas(windDirDeg) || aprsHas(windKmh) || aprsHas(gustKmh) || aprsHas(tempC) || aprsHas(rain1hMm) ||
           aprsHas(rain24hMm) || aprsHas(rainDayMm) || aprsHas(humidityPct) || aprsHas(pressureHpa);
}

AprsReport::AprsReport()
    : kind(AprsKind::None), lat(kNaN), lon(kNaN), symTable(0), symCode(0), courseDeg(kNaN), speedKmh(kNaN),
      altitudeM(kNaN), killed(false), distanceKm(kNaN), bearingDeg(kNaN) {}

const char* aprsKindName(AprsKind k) {
    switch (k) {
        case AprsKind::Position: return "position";
        case AprsKind::Weather: return "weather";
        case AprsKind::Message: return "message";
        case AprsKind::Bulletin: return "bulletin";
        case AprsKind::Ack: return "ack";
        case AprsKind::Status: return "status";
        case AprsKind::Object: return "object";
        case AprsKind::Item: return "item";
        case AprsKind::Telemetry: return "telemetry";
        case AprsKind::Other: return "other";
        case AprsKind::None: break;
    }
    return "";
}

std::string Ax25Address::str() const { return addrStr(call, ssid); }

std::string Ax25Frame::infoText() const {
    std::string s;
    std::size_t n = info.size();
    while (n && (info[n - 1] == '\r' || info[n - 1] == '\n')) --n;
    for (std::size_t i = 0; i < n; ++i) {
        const unsigned char c = info[i];
        if (c >= 0x20 && c < 0x7f) s += static_cast<char>(c);
        else {
            char b[8];
            std::snprintf(b, sizeof b, "<0x%02x>", c);
            s += b;
        }
    }
    return s;
}

std::string Ax25Frame::monitorLine() const {
    std::string s = source.str() + ">" + destination.str();
    for (std::size_t i = 0; i < path.size(); ++i) s += "," + path[i].str() + (path[i].repeated ? "*" : "");
    const bool aprsLike = frameType == "UI" && hasPid && pid == 0xF0;
    if (!aprsLike) {
        s += " <" + frameType;
        if (hasPid && pid != 0xF0) {
            char b[16];
            std::snprintf(b, sizeof b, " pid=%02X", pid);
            s += b;
        }
        s += ">";
    }
    if (aprsLike || !info.empty()) s += ": " + infoText();
    return s;
}

bool ax25ParseFrame(const unsigned char* bytes, std::size_t n, Ax25Frame& out) {
    if (!bytes || n < 15) return false;
    std::vector<uint8_t> raw(bytes, bytes + n);
    ax25::Frame f;
    if (!ax25::Frame::decode(raw, f)) return false;
    // (callsigns as the JS checks the source: /^[A-Z0-9]{1,6}/; also the
    // others here, as noise can pass a 16-bit check now and then)
    if (!callOk(f.src.call) || !callOk(f.dest.call)) return false;
    for (std::size_t i = 0; i < f.digis.size(); ++i)
        if (!callOk(f.digis[i].call)) return false;
    out = Ax25Frame();
    out.destination.call = f.dest.call;
    out.destination.ssid = f.dest.ssid;
    out.source.call = f.src.call;
    out.source.ssid = f.src.ssid;
    // (bit 7 of the source and destination SSID bytes is the command /
    // response bit, not "has repeated": only digipeaters carry H)
    for (std::size_t i = 0; i < f.digis.size(); ++i) {
        Ax25Address a;
        a.call = f.digis[i].call;
        a.ssid = f.digis[i].ssid;
        a.repeated = f.digis[i].repeated;
        out.path.push_back(a);
    }
    out.control = f.ctrl;
    out.hasPid = f.has_pid;
    out.pid = f.has_pid ? f.pid : 0;
    out.info.assign(f.info.begin(), f.info.end());
    switch (f.type()) {
        case ax25::Frame::Type::UI: out.frameType = "UI"; break;
        case ax25::Frame::Type::IFrame: out.frameType = "I"; break;
        case ax25::Frame::Type::RR: out.frameType = "RR"; break;
        case ax25::Frame::Type::RNR: out.frameType = "RNR"; break;
        case ax25::Frame::Type::REJ: out.frameType = "REJ"; break;
        case ax25::Frame::Type::SFrame: out.frameType = "S"; break;
        case ax25::Frame::Type::SABM: out.frameType = "SABM"; break;
        case ax25::Frame::Type::DISC: out.frameType = "DISC"; break;
        case ax25::Frame::Type::UA: out.frameType = "UA"; break;
        case ax25::Frame::Type::DM: out.frameType = "DM"; break;
        case ax25::Frame::Type::FRMR: out.frameType = "FRMR"; break;
        default: out.frameType = "?"; break;
    }
    // (UI frames, no layer 3: APRS)
    if (f.type() == ax25::Frame::Type::UI && f.has_pid && f.pid == 0xF0) {
        const std::string info(out.info.begin(), out.info.end());
        aprsParse(out.source.str(), out.destination.str(), info, out.aprs);
    }
    return true;
}

namespace {

void parseInto(const std::string& src, const std::string& dst, const std::string& info, AprsReport& r, int depth) {
    r = AprsReport();
    r.kind = AprsKind::Other;
    r.raw = info;
    if (info.empty()) return;
    const char t = info[0];
    if (t == '!' || t == '=' || t == '/' || t == '@') {
        std::string s = info.substr(1);
        if (t == '/' || t == '@') {
            r.timestamp = s.substr(0, 7);
            s = s.size() > 7 ? s.substr(7) : std::string();
        }
        Pos p;
        if (!position(s, p)) return;
        applyPos(r, p);
        extras(r, p.rest);
        r.kind = r.wx.any() ? AprsKind::Weather : AprsKind::Position;
    } else if (t == '`' || t == '\'') {
        AprsReport m;
        m.raw = info;
        if (micE(dst, info, m)) { r = m; r.kind = AprsKind::Position; }
    } else if (t == '_') {
        std::string u = info.size() > 9 ? info.substr(9) : std::string();
        AprsWeather w;
        if (weather(u, w)) { r.wx = w; r.kind = AprsKind::Weather; r.comment = trim(u); }
    } else if (t == ':') {
        // /^:(.{9}):(.*?)(?:\{(\w{1,5}))?$/
        if (info.size() >= 11 && info[10] == ':' && info.find('\n') == std::string::npos) {
            std::string text = info.substr(11), id;
            const std::size_t b = text.rfind('{');
            if (b != std::string::npos && text.size() - b - 1 >= 1 && text.size() - b - 1 <= 5) {
                bool w = true;
                for (std::size_t k = b + 1; k < text.size(); ++k) w = w && isWord(text[k]);
                if (w) { id = text.substr(b + 1); text.erase(b); }
            }
            // /^(ack|rej)\w+/
            const bool ack = text.size() >= 4 && (text.compare(0, 3, "ack") == 0 || text.compare(0, 3, "rej") == 0) &&
                             isWord(text[3]);
            r.kind = ack ? AprsKind::Ack : AprsKind::Message;
            r.to = trim(info.substr(1, 9));
            r.text = text;
            r.msgId = id;
            if (r.to.compare(0, 3, "BLN") == 0) r.kind = AprsKind::Bulletin;
        }
    } else if (t == '>') {
        r.kind = AprsKind::Status;
        r.text = trim(info.substr(1));
    } else if (t == ';' || t == ')') {
        std::string name, s;
        bool ok = false, killed = false;
        if (t == ';') {
            // /^;(.{9})([*_])(.{7})(.*)$/
            if (info.size() >= 18 && (info[10] == '*' || info[10] == '_') && info.find('\n') == std::string::npos) {
                name = info.substr(1, 9);
                killed = info[10] == '_';
                s = info.substr(18);
                ok = true;
            }
        } else {
            // /^\)([^!_]{3,9})([!_])(.*)$/
            const std::size_t e = info.find_first_of("!_", 1);
            if (e != std::string::npos && e - 1 >= 3 && e - 1 <= 9 && info.find('\n') == std::string::npos) {
                name = info.substr(1, e - 1);
                killed = info[e] == '_';
                s = info.substr(e + 1);
                ok = true;
            }
        }
        if (ok) {
            r.kind = t == ';' ? AprsKind::Object : AprsKind::Item;
            r.name = trim(name);
            r.killed = killed;
            Pos p;
            if (position(s, p)) { applyPos(r, p); extras(r, p.rest); }
        }
    } else if (t == 'T') {
        r.kind = AprsKind::Telemetry;
    } else if (t == '}' && depth < 2) {
        // /^([A-Z0-9-]+)>([^:,]+)(?:,[^:]*)?:(.*)$/
        const std::string q = info.substr(1);
        const std::size_t gt = q.find('>'), colon = q.find(':');
        if (gt != std::string::npos && gt > 0 && colon != std::string::npos && colon > gt + 1 &&
            q.find('\n', colon) == std::string::npos) {
            bool okSrc = true;
            for (std::size_t k = 0; k < gt; ++k) okSrc = okSrc && (isDigit(q[k]) || isUpper(q[k]) || q[k] == '-');
            const std::size_t comma = q.find(',', gt);
            const std::size_t dEnd = comma != std::string::npos && comma < colon ? comma : colon;
            if (okSrc && dEnd > gt + 1) {
                const std::string isrc = q.substr(0, gt), idst = q.substr(gt + 1, dEnd - gt - 1);
                AprsReport inner;
                parseInto(isrc, idst, q.substr(colon + 1), inner, depth + 1);
                inner.thirdParty = src;
                inner.innerSource = isrc;
                inner.innerPath.assign(1, "(via the internet)");
                r = inner;
            }
        }
    }
    if (r.symTable == '/' && r.symCode) {
        const char* n = symbolName(r.symCode);
        if (n) r.symbolName = n;
    }
}

}  // namespace

void aprsParse(const std::string& source, const std::string& destination, const std::string& info, AprsReport& out) {
    parseInto(source, destination, info, out, 0);
}

std::string aprsWhat(const AprsReport& r) {
    switch (r.kind) {
        case AprsKind::Weather: return "weather";
        case AprsKind::Message: return "message";
        case AprsKind::Bulletin: return "bulletin";
        case AprsKind::Ack: return "ack";
        case AprsKind::Status: return "status";
        case AprsKind::Object: return r.killed ? "object (gone)" : "object";
        case AprsKind::Item: return r.killed ? "item (gone)" : "item";
        case AprsKind::Telemetry: return "telemetry";
        case AprsKind::Position: return r.symbolName.empty() ? "position" : r.symbolName;
        case AprsKind::Other: return "other";
        case AprsKind::None: break;
    }
    return "";
}

namespace {

// aprsWxText
std::string wxText(const AprsWeather& w) {
    std::vector<std::string> p;
    if (aprsHas(w.tempC)) p.push_back(num(w.tempC) + "\xC2\xB0" "C");
    if (aprsHas(w.windKmh)) {
        std::string s = "wind ";
        if (aprsHas(w.windDirDeg)) s += num(w.windDirDeg) + "\xC2\xB0 ";
        s += num(w.windKmh) + " km/h";
        if (aprsHas(w.gustKmh)) s += " (gusts " + num(w.gustKmh) + ")";
        p.push_back(s);
    }
    if (aprsHas(w.pressureHpa)) p.push_back(num(w.pressureHpa) + " hPa");
    if (aprsHas(w.humidityPct)) p.push_back(num(w.humidityPct) + "% humidity");
    if (aprsHas(w.rain1hMm) && w.rain1hMm != 0) p.push_back("rain " + num(w.rain1hMm) + " mm in the last hour");
    std::string s;
    for (std::size_t i = 0; i < p.size(); ++i) s += (i ? ", " : "") + p[i];
    return s;
}

std::string lower(std::string s) {
    for (std::size_t i = 0; i < s.size(); ++i)
        if (isUpper(s[i])) s[i] = static_cast<char>(s[i] - 'A' + 'a');
    return s;
}

}  // namespace

std::string aprsDescribe(const AprsReport& r) {
    std::vector<std::string> p;
    if (r.kind == AprsKind::Message || r.kind == AprsKind::Bulletin || r.kind == AprsKind::Ack) {
        p.push_back("to " + r.to + ": " + r.text);
    } else if (r.kind == AprsKind::Status) {
        p.push_back(r.text);
    } else {
        if (r.kind == AprsKind::Object || r.kind == AprsKind::Item) p.push_back(r.name);
        if (aprsHas(r.lat)) {
            char b[64];
            std::snprintf(b, sizeof b, "%.4f\xC2\xB0%c %.4f\xC2\xB0%c", std::fabs(r.lat), r.lat >= 0 ? 'N' : 'S',
                          std::fabs(r.lon), r.lon >= 0 ? 'E' : 'W');
            p.push_back(b);
        }
        if (r.wx.any()) p.push_back(wxText(r.wx));
        if (aprsHas(r.speedKmh) && r.speedKmh != 0)
            p.push_back(num(r.speedKmh) + " km/h heading " + num(aprsHas(r.courseDeg) ? r.courseDeg : 0) + "\xC2\xB0");
        else if (r.courseDeg == 0 && r.speedKmh == 0 && !r.micE.empty())
            p.push_back("stopped");
        if (aprsHas(r.altitudeM) && r.altitudeM > 50) p.push_back(grouped(r.altitudeM) + " m up");
        if (!r.micE.empty() && r.micE != "Off duty" && r.micE != "custom") p.push_back(lower(r.micE));
        if (!r.comment.empty()) p.push_back(r.comment);
        if (r.kind == AprsKind::Other || r.kind == AprsKind::Telemetry) p.push_back(r.raw);
    }
    if (aprsHas(r.distanceKm)) {
        char b[64];
        std::snprintf(b, sizeof b, "%s km, bearing %.0f\xC2\xB0", grouped(r.distanceKm).c_str(), jsRound(r.bearingDeg));
        p.push_back(b);
    }
    if (!r.thirdParty.empty()) p.push_back("(" + r.innerSource + ", relayed by " + r.thirdParty + ")");
    std::string s;
    for (std::size_t i = 0; i < p.size(); ++i) {
        if (p[i].empty()) continue;
        if (!s.empty()) s += " \xC2\xB7 ";
        s += p[i];
    }
    return s;
}

// distBrg
void aprsDistance(double lat1, double lon1, double lat2, double lon2, double& km, double& deg) {
    const double r = kPi / 180, la1 = lat1 * r, lo1 = lon1 * r, la2 = lat2 * r, lo2 = lon2 * r;
    const double s1 = std::sin((la2 - la1) / 2), s2 = std::sin((lo2 - lo1) / 2);
    const double h = s1 * s1 + std::cos(la1) * std::cos(la2) * s2 * s2;
    km = 2 * 6371 * std::asin(std::min(1.0, std::sqrt(h)));
    const double y = std::sin(lo2 - lo1) * std::cos(la2),
                 x = std::cos(la1) * std::sin(la2) - std::sin(la1) * std::cos(la2) * std::cos(lo2 - lo1);
    deg = std::fmod(std::atan2(y, x) / r + 360, 360);
}

}  // namespace dec
