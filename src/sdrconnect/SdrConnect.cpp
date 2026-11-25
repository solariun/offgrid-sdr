// ============================================================================
//  offgrid-sdrconnect - an SDRplay RSP through SDRplay's SDRconnect WebSocket
//  API, in C++11 (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/sdrconnect/SdrConnect.cpp
//  Layer:  Library liboffgrid-sdrconnect
//
//  What this file does
//  -------------------
//  Implementation of sdrc::Client (SdrConnect.h), ported from OFFgrid-SDR's
//  browser version (index.html, by diddy-boy):
//    sdrcUrl            -> Client::parseAddress
//    sdrcConnect        -> Client::connect, Impl::handleText / handleBinary
//    sdrcGen            -> Impl::setFrequencyLocked, setSampleRate, the IQ
//                          path (drop while retuning, resample, LO shift)
//    sdrcStart          -> Impl::startDeviceImpl
//    sdrcStop           -> Client::stop (+ disconnect)
//    sdrcLost           -> the worker's reconnection
//    sdrcGain/LnaFor    -> Impl::applyGain, Client::lnaForFraction
//    sdrcRangeWait/Done -> Impl::rangeWait / rangeDone
//    sdrcAntList/Rules/For/Probe -> the antenna functions
//    sdrcLiteSet/Sync/In -> startLite / setLite / liteSyncLocked / handleBinary
//    sdrcSpeedTest/Say  -> Client::speedTest / judgeSpeed
//  Left to the application, as they belong to its radio rather than the
//  link: the JS's jitter reserve (priming 0.1 s before handing samples
//  over), Lite mode's loudness levelling and its spectrum calibration
//  against the full stream, the diagnostics log, and the bias-tee/notch
//  property probe (only ever asked, for the log).
//
//  Threads
//  -------
//  One worker thread per Client reads the socket, keeps the properties,
//  runs the timers (retune confirmation, LNA range, VFO checks, the
//  once-a-second figures), reconnects, and calls every callback.  Public
//  calls take the one mutex briefly; they never send while holding it
//  (sends are queued in an Outbox and written after unlocking), so a slow
//  network can't stall the reader.  Blocking calls wait on a condition
//  variable the worker signals at every property.  A reconnection's restart
//  (which waits for SDRconnect) runs on a short-lived helper thread.
//
//  How it fits in
//  --------------
//  WebSocket.cpp carries the bytes; src/app's Source adapter turns the
//  callbacks into its sample queue.
//
//  Contributing
//  ------------
//  When changing behaviour, check the matching JS function first: most odd
//  details here (5 kHz tolerance, 0.8 s, 250 ms, the antenna letters) are
//  fixes from the field, explained in the JS's comments and repeated here.
// ============================================================================
#include "SdrConnect.h"

#include "WebSocket.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>

namespace sdrc {

// ------------------------------------------------------------------ JSON ----
namespace json {
namespace {
void skipWs(const std::string& s, std::size_t& i) {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) ++i;
}
void putUtf8(std::string& o, unsigned cp) {
    if (cp < 0x80) o += static_cast<char>(cp);
    else if (cp < 0x800) { o += static_cast<char>(0xC0 | (cp >> 6)); o += static_cast<char>(0x80 | (cp & 0x3F)); }
    else if (cp < 0x10000) { o += static_cast<char>(0xE0 | (cp >> 12)); o += static_cast<char>(0x80 | ((cp >> 6) & 0x3F)); o += static_cast<char>(0x80 | (cp & 0x3F)); }
    else { o += static_cast<char>(0xF0 | (cp >> 18)); o += static_cast<char>(0x80 | ((cp >> 12) & 0x3F)); o += static_cast<char>(0x80 | ((cp >> 6) & 0x3F)); o += static_cast<char>(0x80 | (cp & 0x3F)); }
}
bool hex4(const std::string& s, std::size_t i, unsigned& v) {
    if (i + 4 > s.size()) return false;
    v = 0;
    for (std::size_t k = i; k < i + 4; ++k) {
        char c = s[k];
        v <<= 4;
        if (c >= '0' && c <= '9') v |= unsigned(c - '0');
        else if (c >= 'a' && c <= 'f') v |= unsigned(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') v |= unsigned(c - 'A' + 10);
        else return false;
    }
    return true;
}
bool parseString(const std::string& s, std::size_t& i, std::string& out) {
    if (i >= s.size() || s[i] != '"') return false;
    ++i;
    out.clear();
    while (i < s.size()) {
        char c = s[i++];
        if (c == '"') return true;
        if (c != '\\') { out += c; continue; }
        if (i >= s.size()) return false;
        char e = s[i++];
        switch (e) {
            case '"': out += '"'; break;
            case '\\': out += '\\'; break;
            case '/': out += '/'; break;
            case 'b': out += '\b'; break;
            case 'f': out += '\f'; break;
            case 'n': out += '\n'; break;
            case 'r': out += '\r'; break;
            case 't': out += '\t'; break;
            case 'u': {
                unsigned cp;
                if (!hex4(s, i, cp)) return false;
                i += 4;
                if (cp >= 0xD800 && cp < 0xDC00 && i + 6 <= s.size() && s[i] == '\\' && s[i + 1] == 'u') {
                    unsigned lo;
                    if (hex4(s, i + 2, lo) && lo >= 0xDC00 && lo < 0xE000) {
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                        i += 6;
                    }
                }
                putUtf8(out, cp);
                break;
            }
            default: return false;
        }
    }
    return false;
}
// a nested object or array, skipped (strings inside respected)
bool skipNested(const std::string& s, std::size_t& i) {
    int depth = 0;
    std::string dummy;
    while (i < s.size()) {
        char c = s[i];
        if (c == '"') { if (!parseString(s, i, dummy)) return false; continue; }
        if (c == '{' || c == '[') ++depth;
        if (c == '}' || c == ']') { --depth; if (depth == 0) { ++i; return true; } }
        ++i;
    }
    return false;
}
}  // namespace

bool parseFlat(const std::string& s, std::map<std::string, std::string>& out) {
    std::size_t i = 0;
    skipWs(s, i);
    if (i >= s.size() || s[i] != '{') return false;
    ++i;
    skipWs(s, i);
    if (i < s.size() && s[i] == '}') return true;
    while (i < s.size()) {
        std::string key, val;
        skipWs(s, i);
        if (!parseString(s, i, key)) return false;
        skipWs(s, i);
        if (i >= s.size() || s[i] != ':') return false;
        ++i;
        skipWs(s, i);
        if (i >= s.size()) return false;
        if (s[i] == '"') {
            if (!parseString(s, i, val)) return false;
        } else if (s[i] == '{' || s[i] == '[') {
            if (!skipNested(s, i)) return false;
        } else {  // number, true, false, null: as written
            std::size_t a = i;
            while (i < s.size() && s[i] != ',' && s[i] != '}' && s[i] != ' ' && s[i] != '\t' && s[i] != '\r' && s[i] != '\n') ++i;
            val = s.substr(a, i - a);
            if (val == "null") val.clear();
        }
        out[key] = val;
        skipWs(s, i);
        if (i < s.size() && s[i] == ',') { ++i; continue; }
        if (i < s.size() && s[i] == '}') return true;
        return false;
    }
    return false;
}

std::string quote(const std::string& s) {
    std::string o = "\"";
    for (std::size_t i = 0; i < s.size(); ++i) {
        unsigned char c = static_cast<unsigned char>(s[i]);
        if (c == '"') o += "\\\"";
        else if (c == '\\') o += "\\\\";
        else if (c == '\n') o += "\\n";
        else if (c == '\r') o += "\\r";
        else if (c == '\t') o += "\\t";
        else if (c < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", c); o += b; }
        else o += static_cast<char>(c);
    }
    return o + "\"";
}

// sdrcSend: always the three fields, the value as a string
std::string event(const std::string& t, const std::string& p, const std::string& v) {
    return "{\"event_type\":" + quote(t) + ",\"property\":" + quote(p) + ",\"value\":" + quote(v) + "}";
}
}  // namespace json

// --------------------------------------------------------------- helpers ----
namespace {
typedef std::chrono::steady_clock Clock;
typedef Clock::time_point TimePoint;
typedef std::vector<std::string> Outbox;  // JSON messages to send once unlocked

const int kNoLna = INT_MIN;
const double kPi = 3.14159265358979323846;

std::string lowerStr(std::string s) {
    for (std::size_t i = 0; i < s.size(); ++i)
        if (s[i] >= 'A' && s[i] <= 'Z') s[i] = static_cast<char>(s[i] - 'A' + 'a');
    return s;
}
std::string trimStr(const std::string& s) {
    std::size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}
double toNum(const std::string& s) {
    std::string t = trimStr(s);
    if (t.empty()) return NAN;
    char* end = 0;
    double v = std::strtod(t.c_str(), &end);
    return end && *end == '\0' ? v : NAN;
}
bool isTrue(const std::string& s) { return lowerStr(trimStr(s)) == "true"; }
std::string intStr(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%lld", static_cast<long long>(std::floor(v + 0.5)));  // Math.round
    return b;
}
std::vector<std::string> splitList(const std::string& s) {
    std::vector<std::string> out;
    std::size_t a = 0;
    while (a <= s.size()) {
        std::size_t c = s.find(',', a);
        if (c == std::string::npos) c = s.size();
        std::string item = trimStr(s.substr(a, c - a));
        if (!item.empty()) out.push_back(item);
        a = c + 1;
    }
    return out;
}
bool isWordChar(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
}
// /RSPdx|nRSP-?ST/i: the RSPdx family (the RSPdx-R2 and the nRSP-ST included)
bool isRspDx(const std::string& dev) {
    std::string d = lowerStr(dev);
    return d.find("rspdx") != std::string::npos || d.find("nrsp-st") != std::string::npos ||
           d.find("nrspst") != std::string::npos;
}
// /antenna\s*X$/i
bool isAntennaLetter(const std::string& a, char letter) {
    std::string s = lowerStr(a);
    if (s.empty() || s[s.size() - 1] != letter) return false;
    std::size_t i = s.size() - 1;
    while (i > 0 && (s[i - 1] == ' ' || s[i - 1] == '\t')) --i;
    return i >= 7 && s.compare(i - 7, 7, "antenna") == 0;
}
// /\b(antenna\s*)?C\s*$/i
bool endsWithC(const std::string& a) {
    std::string s = lowerStr(a);
    std::size_t e = s.find_last_not_of(" \t\r\n");
    if (e == std::string::npos || s[e] != 'c') return false;
    if (e == 0 || !isWordChar(s[e - 1])) return true;
    return e >= 7 && s.compare(e - 7, 7, "antenna") == 0 && (e == 7 || !isWordChar(s[e - 8]));
}
// the antenna's letter (sdrcAntRules's letter()): "Antenna B", "Ant B", "B" -> 'B'; 0 if none
char antennaLetter(const std::string& a) {
    std::string s = trimStr(a);
    if (!s.empty()) {
        char c = static_cast<char>(s[s.size() - 1] & ~0x20);
        if (c >= 'A' && c <= 'C') {
            std::string pre = s.substr(0, s.size() - 1);
            std::size_t e = pre.find_last_not_of(" \t");
            if (e == std::string::npos) return c;  // just the letter
            bool spaced = e + 1 < pre.size();
            if (spaced || !isWordChar(pre[e])) return c;
            std::string lp = lowerStr(pre.substr(0, e + 1));
            const char* words[] = {"antenna", "ant"};
            for (int k = 0; k < 2; ++k) {
                std::size_t n = std::strlen(words[k]);
                if (lp.size() >= n && lp.compare(lp.size() - n, n, words[k]) == 0 &&
                    (lp.size() == n || !isWordChar(lp[lp.size() - n - 1])))
                    return c;
            }
        }
    }
    // the second form: /\bantenna\s*([A-C])\b/i anywhere
    std::string l = lowerStr(a);
    for (std::size_t p = l.find("antenna"); p != std::string::npos; p = l.find("antenna", p + 1)) {
        if (p > 0 && isWordChar(l[p - 1])) continue;
        std::size_t q = p + 7;
        while (q < l.size() && (l[q] == ' ' || l[q] == '\t')) ++q;
        if (q < l.size() && l[q] >= 'a' && l[q] <= 'c' && (q + 1 == l.size() || !isWordChar(l[q + 1])))
            return static_cast<char>(l[q] - 'a' + 'A');
    }
    return 0;
}
std::string lnaStr(int v) { char b[16]; std::snprintf(b, sizeof b, "%d", v); return b; }
}  // namespace

// ------------------------------------------------------------------ Impl ----
struct Client::Impl {
    enum GainMode { GainLeave, GainAuto, GainFraction, GainState };
    enum Reconnect { RcNone, RcWaiting, RcAwaitApi };

    Impl()
        : port(Client::kDefaultPort), workerId(std::thread::id()), quit(false), closing(false), link(LinkState::Off), apiSeen(false),
          iqOn(false), liteOn(false), liteWanted(false), wantStream(false), speedTesting(false),
          wantCenter(0), wantRate(0), loErr(0), waiting(false), retuneArmed(false), rangePending(false),
          rateAskPending(false), gainMode(GainLeave), gainFraction(0.5), gainState(0), lnaSent(kNoLna),
          autoAnt(true), antSentValid(false), ctlWarned(false),
          liteDmSent(false), liteVfoSent(NAN), liteBwSent(NAN), liteWantVfo(NAN), liteWantBw(NAN),
          liteFrames(0), liteSq(0), liteN(0), liteStereo(false), specFrames(0), specBins(0), liteF0(0), liteB0(0),
          liteSaid(false), bytes(0), bytesAtTick(0), MBs(0), iqSamples(0), dropped(0), reconnects(0),
          autoReconnect(true), retryMs(3000), firstMs(1000), rc(RcNone),
          dspReset(true), rsPos(1.0), phase(1.0, 0.0) {}

    // ---- connection
    std::string host, path, url;
    int port;
    std::shared_ptr<WebSocket> ws;
    std::thread worker, restarter;
    std::atomic<std::thread::id> workerId;
    mutable std::mutex m;
    std::condition_variable cv;
    bool quit, closing;
    LinkState link;
    bool apiSeen;
    std::map<std::string, std::string> props;

    // ---- callbacks
    IqCallback iqCb;
    AudioCallback audioCb;
    SpectrumCallback specCb;
    StateCallback stateCb;
    PropertyCallback propCb;
    std::vector<std::function<void()> > posted;  // run on the worker

    // ---- streaming
    bool iqOn, liteOn, liteWanted, wantStream, speedTesting;

    // ---- tuning (sdrc.want, waiting, loErr)
    double wantCenter, wantRate, loErr;
    bool waiting, retuneArmed, rangePending, rateAskPending;
    TimePoint retuneDeadline, rangeDeadline, settleUntil, rateAskAt;

    // ---- gain
    GainMode gainMode;
    double gainFraction;
    int gainState, lnaSent;

    // ---- antennas
    std::map<std::string, std::vector<std::string> > seenAnts;  // per device
    std::map<std::string, AntennaRules> rules;                  // per device
    bool autoAnt, antSentValid;
    std::string antSent;
    bool ctlWarned;

    // ---- Lite mode
    std::string liteWantDm, liteDmSentVal;
    bool liteDmSent;
    double liteVfoSent, liteBwSent, liteWantVfo, liteWantBw;
    TimePoint liteVfoAt, liteAskAt, liteT0, liteStatT;
    std::uint64_t liteFrames;
    double liteSq, liteN;
    bool liteStereo;
    std::uint64_t specFrames;
    std::size_t specBins;
    std::uint64_t liteF0, liteB0;
    bool liteSaid;
    LiteStats liteStat;

    // ---- profile
    std::string startProfile;

    // ---- figures
    std::uint64_t bytes, bytesAtTick;
    double MBs;
    TimePoint tickAt;
    std::uint64_t iqSamples, dropped;
    unsigned reconnects;

    // ---- reconnection (sdrcLost)
    bool autoReconnect;
    int retryMs, firstMs;
    Reconnect rc;
    TimePoint rcNext, rcApiDeadline;

    // ---- the IQ path's state (worker only, but dspReset under m)
    bool dspReset;
    std::vector<std::complex<float> > in, out, hist;
    double rsPos;
    std::complex<double> phase;

    // ================================================================ sends
    static void q(Outbox& ob, const std::string& t, const std::string& p, const std::string& v) {
        ob.push_back(json::event(t, p, v));
    }
    // Writes queued messages; never called with m held.
    void flush(Outbox& ob) {
        if (ob.empty()) return;
        std::shared_ptr<WebSocket> w;
        {
            std::lock_guard<std::mutex> lk(m);
            w = ws;
        }
        if (w)
            for (std::size_t i = 0; i < ob.size(); ++i) w->sendText(ob[i]);
        ob.clear();
    }
    void post(const std::function<void()>& fn) { posted.push_back(fn); }  // m held
    void postState(LinkState s, const std::string& msg) {                  // m held
        StateCallback cb = stateCb;
        if (cb) post([cb, s, msg]() { cb(s, msg); });
    }

    bool onWorker() const { return std::this_thread::get_id() == workerId; }
    bool connectedLocked() const { return ws && link == LinkState::Connected && !closing && !quit; }
    bool startedLocked() const {
        std::map<std::string, std::string>::const_iterator it = props.find("started");
        return it != props.end() && isTrue(it->second);
    }
    std::string prop(const std::string& k) const {
        std::map<std::string, std::string>::const_iterator it = props.find(k);
        return it == props.end() ? std::string() : it->second;
    }
    bool aborted() const { return quit || closing || link != LinkState::Connected; }

    // Waits (m held through lk) until pred(), ms at most; askProp asked every
    // askMs meanwhile (the JS's wait() helpers).  Stops early if the link goes.
    template <class Pred>
    bool waitUntil(std::unique_lock<std::mutex>& lk, int ms, Pred pred, const char* askProp = 0, int askMs = 1000) {
        TimePoint end = Clock::now() + std::chrono::milliseconds(ms);
        TimePoint nextAsk = Clock::now() + std::chrono::milliseconds(askMs);
        for (;;) {
            if (pred()) return true;
            if (aborted()) return false;
            TimePoint now = Clock::now();
            if (now >= end) return pred();
            if (askProp && now >= nextAsk) {
                nextAsk = now + std::chrono::milliseconds(askMs);
                Outbox ob;
                q(ob, "get_property", askProp, "");
                lk.unlock();
                flush(ob);
                lk.lock();
                continue;
            }
            TimePoint until = std::min(end, now + std::chrono::milliseconds(100));
            cv.wait_until(lk, until);
        }
    }
    struct Never { bool operator()() const { return false; } };
    void sleepLocked(std::unique_lock<std::mutex>& lk, int ms) { waitUntil(lk, ms, Never()); }

    // ================================================================ gain
    // sdrcGain: the LNA state for the band's range; only a real change is sent;
    // the RSP's samples for 250 ms after it are dropped (settling)
    void applyGain(Outbox& ob) {
        if (gainMode == GainLeave || !connectedLocked() || !startedLocked()) return;
        double lo = toNum(prop("lna_state_min")), hi = toNum(prop("lna_state_max"));
        if (std::isnan(lo)) lo = 0;
        if (std::isnan(hi)) hi = 0;
        if (!(hi > lo)) return;
        int ilo = static_cast<int>(lo), ihi = static_cast<int>(hi), lna;
        if (gainMode == GainAuto) lna = static_cast<int>(std::floor(lo + (hi - lo) / 2 + 0.5));
        else if (gainMode == GainFraction) lna = Client::lnaForFraction(gainFraction, ilo, ihi);
        else lna = std::max(ilo, std::min(ihi, gainState));
        double cur = toNum(prop("lna_state"));
        if ((!std::isnan(cur) && static_cast<int>(cur) == lna) || lna == lnaSent) return;
        lnaSent = lna;
        settleUntil = Clock::now() + std::chrono::milliseconds(250);
        q(ob, "set_property", "lna_state", lnaStr(lna));
    }

    // ================================================================ antennas
    std::vector<std::string> antListLocked() {
        std::string dev = prop("active_device");
        if (dev.empty()) dev = "?";
        std::vector<std::string> now = splitList(prop("valid_antennas"));
        std::vector<std::string> had = seenAnts[dev], all = had;
        for (std::size_t i = 0; i < now.size(); ++i)
            if (std::find(all.begin(), all.end(), now[i]) == all.end()) all.push_back(now[i]);
        bool hasA = false, hasC = false;
        for (std::size_t i = 0; i < all.size(); ++i) {
            if (isAntennaLetter(all[i], 'a')) hasA = true;
            if (isAntennaLetter(all[i], 'c')) hasC = true;
        }
        // (an RSPdx's Antenna C works only up to 200 MHz, and SDRconnect lists it only there: shown always)
        if (isRspDx(dev) && hasA && !hasC) all.push_back("Antenna C");
        std::sort(all.begin(), all.end());
        if (!now.empty() && all != had) seenAnts[dev] = all;
        return all;
    }
    AntennaRules* rulesLocked() {
        std::string dev = prop("active_device");
        if (dev.empty()) dev = "?";
        std::vector<std::string> ants = antListLocked();
        if (ants.size() < 2) return 0;
        bool lettered = true;
        for (std::size_t i = 0; i < ants.size(); ++i)
            if (!antennaLetter(ants[i])) lettered = false;
        std::map<std::string, AntennaRule> defs, byOrder;
        for (std::size_t k = 0; k < ants.size(); ++k) {
            AntennaRule::Kind ko = k == 0 ? AntennaRule::Above : k == 1 ? AntennaRule::Below : AntennaRule::None;
            byOrder[ants[k]] = AntennaRule(ko);
            if (lettered) {
                char l = antennaLetter(ants[k]);
                defs[ants[k]] = AntennaRule(l == 'A' ? AntennaRule::Above : l == 'B' ? AntennaRule::Below : AntennaRule::None);
            } else {
                defs[ants[k]] = AntennaRule(ko);
            }
        }
        std::map<std::string, AntennaRules>::iterator it = rules.find(dev);
        if (it == rules.end()) {
            AntennaRules r;
            r.map = defs;
            it = rules.insert(std::make_pair(dev, r)).first;
        } else if (!it->second.user && sameKinds(it->second.map, byOrder) && !sameKinds(byOrder, defs)) {
            // (rules still as the old by-position defaults made them: redone by letter)
            it->second.map = defs;
        }
        for (std::size_t k = 0; k < ants.size(); ++k)
            if (!it->second.map.count(ants[k])) it->second.map[ants[k]] = AntennaRule(AntennaRule::None);
        return &it->second;
    }
    static bool sameKinds(const std::map<std::string, AntennaRule>& a, const std::map<std::string, AntennaRule>& b) {
        if (a.size() != b.size()) return false;
        for (std::map<std::string, AntennaRule>::const_iterator i = a.begin(), j = b.begin(); i != a.end(); ++i, ++j)
            if (i->first != j->first || i->second.kind != j->second.kind || i->second.loMHz != j->second.loMHz ||
                i->second.hiMHz != j->second.hiMHz)
                return false;
        return true;
    }
    std::string antennaForLocked(double hz) {
        AntennaRules* r = rulesLocked();
        if (!r) return std::string();
        std::string dev = prop("active_device");
        double mhz = hz / 1e6;
        std::vector<std::string> all = antListLocked(), ants;
        for (std::size_t i = 0; i < all.size(); ++i)
            if (hz <= Client::antennaLimitHz(dev, all[i])) ants.push_back(all[i]);
        for (std::size_t i = 0; i < ants.size(); ++i) {
            const AntennaRule& v = r->map[ants[i]];
            if (v.kind == AntennaRule::Range && mhz >= v.loMHz && mhz < v.hiMHz) return ants[i];
        }
        for (std::size_t i = 0; i < ants.size(); ++i) {
            const AntennaRule& v = r->map[ants[i]];
            if ((v.kind == AntennaRule::Below && mhz < r->crossMHz) || (v.kind == AntennaRule::Above && mhz >= r->crossMHz))
                return ants[i];
        }
        // (the antenna chosen for here can't work here (C above 200 MHz): the one for the other side, then any)
        for (std::size_t i = 0; i < ants.size(); ++i)
            if (r->map[ants[i]].kind != AntennaRule::None) return ants[i];
        return ants.empty() ? std::string() : ants[0];
    }
    void autoAntenna(Outbox& ob, double hz) {
        if (!autoAnt) return;
        std::string a = antennaForLocked(hz);
        // (compared with the antenna last ASKED for, not the last confirmed: confirmations can come late)
        std::string cur = antSentValid ? antSent : prop("active_antenna");
        if (!a.empty() && a != cur) {
            q(ob, "set_property", "active_antenna", a);
            antSent = a;
            antSentValid = true;
        }
    }

    // ================================================================ tuning
    // sdrcGen's retune: sent at once; samples dropped until confirmed (unless
    // SDRconnect is already there, after a reconnection, say)
    void setFrequencyLocked(Outbox& ob, double hz, bool force) {
        if (!force && std::floor(hz + 0.5) == std::floor(wantCenter + 0.5)) return;
        wantCenter = hz;
        if (!connectedLocked() || !startedLocked()) return;  // sent once the RSP runs (startDevice)
        double rep = toNum(prop("device_center_frequency"));
        bool already = !std::isnan(rep) && std::fabs(rep - hz) <= 5000;
        loErr = already ? rep - hz : 0;
        if (!already) {
            waiting = true;
            retuneArmed = true;
            rangePending = false;
            retuneDeadline = Clock::now() + std::chrono::milliseconds(1500);
            dspReset = true;
        }
        q(ob, "set_property", "device_center_frequency", intStr(hz));
        applyGain(ob);  // (the gain with the frequency, at once)
        q(ob, "get_property", "valid_antennas", "");  // (SDRconnect's list changes with the frequency)
        autoAntenna(ob, hz);
    }
    void setRateLocked(Outbox& ob, double hz, bool force) {
        if (!force && std::fabs(hz - wantRate) < 1) return;
        wantRate = hz;
        dspReset = true;
        if (!connectedLocked() || !startedLocked()) return;
        q(ob, "set_property", "device_sample_rate", intStr(hz));
        rateAskPending = true;  // (read back: SDRconnect may run at another rate)
        rateAskAt = Clock::now() + std::chrono::milliseconds(600);
    }
    // sdrcRangeWait / sdrcRangeDone: after a retune, the new band's LNA range
    // before carrying on (0.8 s at most)
    void rangeWait(Outbox& ob) {
        rangePending = true;
        retuneArmed = false;
        rangeDeadline = Clock::now() + std::chrono::milliseconds(800);
        q(ob, "get_property", "lna_state_min", "");
        q(ob, "get_property", "lna_state_max", "");
        q(ob, "get_property", "lna_state", "");
    }
    void rangeDone(Outbox& ob) {
        rangePending = false;
        if (wantStream) applyGain(ob);
        waiting = false;
        dspReset = true;
    }
    // Everything sent again (after the device starts, a profile, a reconnection)
    void resendTuning(Outbox& ob, bool forgetSent) {
        if (forgetSent) {
            lnaSent = kNoLna;
            antSentValid = false;
            liteDmSent = false;
            liteVfoSent = NAN;
            liteBwSent = NAN;
        }
        if (wantRate > 0) setRateLocked(ob, wantRate, true);
        if (wantCenter > 0) setFrequencyLocked(ob, wantCenter, true);
        applyGain(ob);
        if (liteOn) liteSyncLocked(ob);
    }

    // ================================================================ Lite
    // sdrcLiteSync: demodulator, VFO and filter, only when changed; the VFO
    // sent again if SDRconnect's is more than 50 Hz off (a second apart at most)
    void liteSyncLocked(Outbox& ob) {
        if (!liteOn || !connectedLocked()) return;
        TimePoint now = Clock::now();
        if (!liteWantDm.empty() && (!liteDmSent || liteDmSentVal != liteWantDm)) {
            q(ob, "set_property", "demodulator", liteWantDm);
            liteDmSent = true;
            liteDmSentVal = liteWantDm;
        }
        if (!std::isnan(liteWantVfo)) {
            double rep = toNum(prop("device_vfo_frequency"));
            bool wrong = !std::isnan(rep) && std::fabs(rep - liteWantVfo) > 50 &&
                         now - liteVfoAt > std::chrono::milliseconds(1000);
            if (liteVfoSent != std::floor(liteWantVfo + 0.5) || wrong) {
                q(ob, "set_property", "device_vfo_frequency", intStr(liteWantVfo));
                liteVfoSent = std::floor(liteWantVfo + 0.5);
                liteVfoAt = now;
            }
        }
        if (!std::isnan(liteWantBw)) {
            double bw = std::max(100.0, std::floor(liteWantBw + 0.5));
            if (liteBwSent != bw) {
                q(ob, "set_property", "filter_bandwidth", intStr(bw));
                liteBwSent = bw;
            }
        }
    }
    void liteOnLocked(Outbox& ob) {  // sdrcLiteSet(true): the streams swapped over
        q(ob, "iq_stream_enable", "", "false");
        iqOn = false;
        liteOn = true;
        liteDmSent = false;
        liteVfoSent = NAN;
        liteBwSent = NAN;
        liteFrames = specFrames = liteF0 = liteB0 = 0;
        liteSq = liteN = 0;
        liteStereo = false;
        liteSaid = false;
        liteT0 = liteStatT = Clock::now();
        liteStat = LiteStats();
        q(ob, "get_property", "spectrum_ref_level", "");
        q(ob, "get_property", "spectrum_base", "");
        q(ob, "get_property", "audio_mute", "");
        q(ob, "get_property", "audio_volume_percent", "");
        liteSyncLocked(ob);
        q(ob, "audio_stream_enable", "", "true");
        q(ob, "spectrum_enable", "", "true");
    }
    void liteOffLocked(Outbox& ob) {  // sdrcLiteSet(false)
        q(ob, "audio_stream_enable", "", "false");
        q(ob, "spectrum_enable", "", "false");
        liteOn = false;
        iqOn = true;
        dspReset = true;
        q(ob, "iq_stream_enable", "", "true");
    }

    // ================================================================ start
    // sdrcStart, up to (not including) the streams
    bool startDeviceImpl(std::string& error) {
        std::unique_lock<std::mutex> lk(m);
        if (!connectedLocked()) { error = "Not connected to SDRconnect"; return false; }
        waitUntil(lk, 1500, [this]() { return props.count("started") > 0; }, "started", 1000);
        if (!startedLocked()) {
            std::string dev = prop("active_device");
            if (dev.empty()) {
                std::vector<std::string> v = splitList(prop("valid_devices"));
                if (!v.empty()) dev = v[0];
            }
            if (dev.empty()) { error = "SDRconnect has no device to use: is the RSP plugged in to that computer?"; return false; }
            Outbox ob;
            // (selected only if it isn't the active device already: selecting it again may make SDRconnect reopen it)
            if (prop("active_device") != dev) {
                q(ob, "selected_device_name", "", dev);
                lk.unlock(); flush(ob); lk.lock();
                sleepLocked(lk, 500);
            }
            q(ob, "device_stream_enable", "", "true");
            lk.unlock(); flush(ob); lk.lock();
            if (!waitUntil(lk, 15000, [this]() { return startedLocked(); }, "started", 1000)) {
                error = aborted() ? "The connection to SDRconnect was lost while starting the RSP"
                                  : "SDRconnect didn't start " + dev + ": check it on that computer (is the RSP in use by another program?)";
                return false;
            }
        }
        Outbox ob;
        const char* again[] = {"can_control", "device_sample_rate", "active_device", "valid_antennas", "lna_state_min", "lna_state_max"};
        for (int i = 0; i < 6; ++i) q(ob, "get_property", again[i], "");
        bool forget = false;
        if (!startProfile.empty()) {
            // (a profile saved in SDRconnect, e.g. with the bias-tee on; then our own settings again)
            q(ob, "apply_device_profile", "", startProfile);
            lk.unlock(); flush(ob); lk.lock();
            sleepLocked(lk, 600);
            q(ob, "get_property", "lna_state", "");
            q(ob, "get_property", "active_antenna", "");
            q(ob, "get_property", "device_sample_rate", "");
            forget = true;
        }
        resendTuning(ob, forget);
        lk.unlock();
        flush(ob);
        return true;
    }

    // after a reconnection: as sdrcLost's "again": start, then the streams and settings
    void restartAfterReconnect() {
        std::string err;
        bool ok = startDeviceImpl(err);
        std::unique_lock<std::mutex> lk(m);
        if (!ok || aborted()) {
            if (!aborted()) postState(LinkState::Connected, "Reconnected, but " + err);
            return;
        }
        Outbox ob;
        resendTuning(ob, true);
        if (liteWanted) liteOnLocked(ob);
        else { iqOn = true; liteOn = false; dspReset = true; q(ob, "iq_stream_enable", "", "true"); }
        postState(LinkState::Connected, "Reconnected to SDRconnect at " + url);
        lk.unlock();
        flush(ob);
    }

    // ================================================================ worker
    void run() {
        workerId = std::this_thread::get_id();
        for (;;) {
            std::shared_ptr<WebSocket> w;
            {
                std::lock_guard<std::mutex> lk(m);
                if (quit) break;
                w = ws;
            }
            if (w) {
                WebSocket::Message msg;
                WebSocket::Result r = w->receive(msg, 20);
                if (r == WebSocket::GotMessage) {
                    if (msg.binary) handleBinary(msg.data);
                    else handleText(msg.data);
                } else if (r == WebSocket::Closed) {
                    onClosed(w);
                }
            } else {
                reconnectStep();
            }
            timers();
            runPosted();
        }
        runPosted();
    }

    void runPosted() {
        std::vector<std::function<void()> > p;
        {
            std::lock_guard<std::mutex> lk(m);
            p.swap(posted);
        }
        for (std::size_t i = 0; i < p.size(); ++i) p[i]();
    }

    void onClosed(const std::shared_ptr<WebSocket>& w) {
        std::lock_guard<std::mutex> lk(m);
        if (ws != w) return;
        ws.reset();
        bool wasConnected = link == LinkState::Connected;
        cv.notify_all();
        if (closing || quit) { link = LinkState::Off; return; }
        if (rc == RcAwaitApi) {  // a reconnection attempt that didn't get as far as api_version
            rc = RcWaiting;
            link = LinkState::Lost;
            rcNext = Clock::now() + std::chrono::milliseconds(retryMs);
            return;
        }
        if (wasConnected && wantStream && autoReconnect) {
            // sdrcLost: the badge shows it, and a new attempt every few seconds
            link = LinkState::Lost;
            rc = RcWaiting;
            rcNext = Clock::now() + std::chrono::milliseconds(firstMs);
            postState(LinkState::Lost, "Lost the connection to SDRconnect at " + url + ": trying again every few seconds...");
            return;
        }
        link = LinkState::Off;
        if (wasConnected) postState(LinkState::Off, "The connection to " + url + " closed");
    }

    void reconnectStep() {
        std::string h, p;
        int pt;
        {
            std::unique_lock<std::mutex> lk(m);
            if (rc != RcWaiting || closing || quit || Clock::now() < rcNext) {
                cv.wait_for(lk, std::chrono::milliseconds(50));
                return;
            }
            h = host; p = path; pt = port;
            link = LinkState::Connecting;
        }
        std::shared_ptr<WebSocket> w(new WebSocket());
        std::string err;
        bool ok = w->connect(h, pt, p, 4000, err);
        Outbox ob;
        {
            std::lock_guard<std::mutex> lk(m);
            if (closing || quit) return;
            if (!ok) {
                link = LinkState::Lost;
                rcNext = Clock::now() + std::chrono::milliseconds(retryMs);
                return;
            }
            ws = w;
            props.clear();
            apiSeen = false;
            rc = RcAwaitApi;
            rcApiDeadline = Clock::now() + std::chrono::milliseconds(4000);
            queueHello(ob);
        }
        flush(ob);
    }

    // the first questions, on opening (sdrcConnect's onopen)
    static void queueHello(Outbox& ob) {
        const char* first[] = {"api_version", "valid_devices", "active_device", "valid_antennas", "active_antenna",
                               "lna_state_min", "lna_state_max", "lna_state", "device_sample_rate",
                               "device_center_frequency", "overload", "can_control", "started"};
        for (std::size_t i = 0; i < sizeof first / sizeof first[0]; ++i) q(ob, "get_property", first[i], "");
    }

    void timers() {
        Outbox ob;
        std::shared_ptr<WebSocket> abortWs;
        {
            std::lock_guard<std::mutex> lk(m);
            TimePoint now = Clock::now();
            // a retune not confirmed within 1.5 s: carry on anyway (asking what it's tuned to)
            if (waiting && retuneArmed && !rangePending && now >= retuneDeadline) {
                rangeWait(ob);
                q(ob, "get_property", "device_center_frequency", "");
            }
            if (rangePending && now >= rangeDeadline) rangeDone(ob);
            if (rateAskPending && now >= rateAskAt) {
                rateAskPending = false;
                q(ob, "get_property", "device_sample_rate", "");
            }
            if (rc == RcAwaitApi && now >= rcApiDeadline && ws) {
                abortWs = ws;  // no answer: dropped, and tried again later
            }
            if (now - tickAt >= std::chrono::milliseconds(1000)) secondTick(ob, now);
        }
        if (abortWs) abortWs->abort();
        flush(ob);
    }

    // once a second: the link's speed, Lite mode's figures and checks
    void secondTick(Outbox& ob, TimePoint now) {
        double dt = std::chrono::duration<double>(now - tickAt).count();
        if (dt > 5 || dt <= 0) dt = 1;
        MBs = (bytes - bytesAtTick) / 1e6 / dt;
        bytesAtTick = bytes;
        tickAt = now;
        if (!liteOn) return;
        double ldt = std::chrono::duration<double>(now - liteStatT).count();
        if (ldt <= 0) ldt = 1;
        liteStatT = now;
        liteStat.soundFramesPerSec = (liteFrames - liteF0) / ldt;
        liteStat.spectrumFramesPerSec = (specFrames - liteB0) / ldt;
        liteStat.soundDb = liteN > 0 ? 10 * std::log10(liteSq / liteN + 1e-20) : -200;
        liteStat.bins = specBins;
        liteStat.stereoSeen = liteStereo;
        liteF0 = liteFrames;
        liteB0 = specFrames;
        liteSq = liteN = 0;
        // SDRconnect's VFO asked for each second (and sent again by liteSyncLocked if wrong)
        q(ob, "get_property", "device_vfo_frequency", "");
        liteSyncLocked(ob);
        // (said plainly, once, if no sound comes, or only silence: SDRconnect's own mute or volume, likely)
        if (!liteSaid && now - liteT0 > std::chrono::seconds(3)) {
            double fps = liteStat.soundFramesPerSec, sps = liteStat.spectrumFramesPerSec;
            bool silent = fps > 0 && liteStat.soundDb < -70;
            if (fps == 0 || silent) {
                liteSaid = true;
                std::string msg;
                if (fps == 0) {
                    msg = std::string("Lite mode: no sound is arriving from SDRconnect (its spectrum is") + (sps > 0 ? "" : "n't either") +
                          "). SDRconnect " + prop("api_version") + " may not send sound this way";
                } else {
                    std::string mute = prop("audio_mute"), vol = prop("audio_volume_percent");
                    msg = "Lite mode: SDRconnect is sending silence. Its own Mute" + (mute.empty() ? "" : " (now " + mute + ")") +
                          " or volume" + (vol.empty() ? "" : " (now " + vol + "%)") +
                          " is probably what's heard here: unmute it, or turn it up, in SDRconnect";
                }
                postState(link, msg);
            }
        }
    }

    void handleText(const std::string& text) {
        std::map<std::string, std::string> msg;
        if (!json::parseFlat(text, msg)) return;
        const std::string& type = msg["event_type"];
        const std::string& name = msg["property"];
        std::map<std::string, std::string>::iterator dv = msg.find("device");
        if ((type != "get_property_response" && type != "property_changed") || name.empty()) return;
        if (dv != msg.end() && !dv->second.empty() && dv->second != "primary") return;
        const std::string value = msg["value"];
        Outbox ob;
        PropertyCallback pcb;
        bool launchRestart = false;
        {
            std::lock_guard<std::mutex> lk(m);
            props[name] = value;
            pcb = propCb;
            if (name == "device_center_frequency" && wantCenter > 0) {
                double v = toNum(value);
                // (a retune confirmed within 5 kHz, as SDRconnect rounds to its tuning step; corrected in the samples)
                if (!std::isnan(v) && std::fabs(v - wantCenter) <= 5000) {
                    loErr = v - wantCenter;
                    if (waiting && !rangePending) rangeWait(ob);
                }
            }
            if ((name == "lna_state_max" || name == "lna_state_min") && wantStream) applyGain(ob);
            if (name == "lna_state_max" && rangePending) rangeDone(ob);
            if (name == "valid_antennas") antListLocked();
            // (another program controlling the RSP: said plainly, once per connection)
            if (name == "can_control" && value == "false" && !ctlWarned && startedLocked()) {
                ctlWarned = true;
                postState(link, "Another program is controlling this RSP through SDRconnect: OFFgrid-SDR can listen, but may not be able to tune it");
            }
            if (name == "can_control" && value == "true") ctlWarned = false;
            if (name == "device_sample_rate") dspReset = true;
            if (name == "api_version") {
                apiSeen = true;
                if (link != LinkState::Connected) {
                    link = LinkState::Connected;
                    if (rc == RcAwaitApi) {
                        rc = RcNone;
                        ++reconnects;
                        launchRestart = true;
                    }
                }
            }
            cv.notify_all();
        }
        flush(ob);
        if (pcb) pcb(name, value);
        if (launchRestart) {
            if (restarter.joinable()) restarter.join();  // the last one has finished: its waits stop when the link goes
            restarter = std::thread(&Impl::restartAfterReconnect, this);
        }
    }

    void handleBinary(const std::string& d) {
        if (d.size() < 6) return;
        const unsigned char* p = reinterpret_cast<const unsigned char*>(d.data());
        unsigned type = p[0] | (unsigned(p[1]) << 8);
        bool deliver = false, lite = false;
        double devRate = 0, outRate = 0, lo = 0;
        bool reset = false;
        IqCallback icb;
        AudioCallback acb;
        SpectrumCallback scb;
        float ref = 0, base = -120;
        {
            std::lock_guard<std::mutex> lk(m);
            bytes += d.size();
            if (type == 1 || type == 3) {
                lite = liteOn;
                acb = audioCb;
                scb = specCb;
                double r = toNum(prop("spectrum_ref_level")), b = toNum(prop("spectrum_base"));
                if (!std::isnan(r)) ref = static_cast<float>(r);
                if (!std::isnan(b)) base = static_cast<float>(b);
            } else if (type == 2) {
                deliver = iqOn && !liteOn && !waiting && !speedTesting && Clock::now() >= settleUntil && iqCb;
                if (!deliver) ++dropped;
                icb = iqCb;
                double dr = toNum(prop("device_sample_rate"));
                devRate = std::isnan(dr) ? 0 : dr;
                outRate = wantRate > 0 ? wantRate : devRate;
                lo = loErr;
                reset = dspReset;
                dspReset = false;
            } else {
                return;
            }
        }
        if (type == 1) {
            if (!lite) return;
            // 16-bit stereo at 48 kHz, L R L R
            std::size_t frames = (d.size() - 2) / 4;
            std::vector<float> lr(frames * 2);
            double sq = 0;
            bool stereo = false;
            for (std::size_t i = 0; i < frames; ++i) {
                const unsigned char* s = p + 2 + 4 * i;
                float l = static_cast<std::int16_t>(s[0] | (s[1] << 8)) / 32768.0f;
                float r = static_cast<std::int16_t>(s[2] | (s[3] << 8)) / 32768.0f;
                lr[2 * i] = l;
                lr[2 * i + 1] = r;
                if (l != r) stereo = true;
                sq += double(l) * l;
            }
            {
                std::lock_guard<std::mutex> lk(m);
                ++liteFrames;
                liteSq += sq;
                liteN += frames;
                if (stereo) liteStereo = true;
            }
            if (acb && frames) acb(lr.data(), frames);
            return;
        }
        if (type == 3) {
            if (!lite) return;
            // a byte a bin, low frequencies first: base .. ref dB
            std::size_t n = d.size() - 2;
            std::vector<float> db(n);
            for (std::size_t i = 0; i < n; ++i) db[i] = base + p[2 + i] / 255.0f * (ref - base);
            {
                std::lock_guard<std::mutex> lk(m);
                ++specFrames;
                specBins = n;
            }
            if (scb && n) scb(db.data(), n);
            return;
        }
        // type 2: IQ
        if (reset) {
            hist.clear();
            rsPos = 1.0;
        }
        if (!deliver) return;
        in.resize((d.size() - 2) / 4);
        std::size_t n = Client::s16ToComplex(p + 2, d.size() - 2, in.data());
        const std::complex<float>* res = in.data();
        std::size_t nOut = n;
        if (devRate > 0 && outRate > 0 && std::fabs(devRate - outRate) > 1) {
            // (SDRconnect at another rate than asked: resampled, linear interpolation; a [1,2,1]/4 low-pass
            //  first when there are fewer samples to make, so stations outside the narrower view don't fold in)
            double step = devRate / outRate;
            bool lp = step > 1.3;
            hist.insert(hist.end(), in.begin(), in.begin() + n);
            out.clear();
            for (;;) {
                std::size_t k = static_cast<std::size_t>(rsPos);
                if (k < 1 || k + 2 >= hist.size()) break;
                double f = rsPos - k;
                std::complex<float> a0, a1;
                if (lp) {
                    a0 = 0.25f * hist[k - 1] + 0.5f * hist[k] + 0.25f * hist[k + 1];
                    a1 = 0.25f * hist[k] + 0.5f * hist[k + 1] + 0.25f * hist[k + 2];
                } else {
                    a0 = hist[k];
                    a1 = hist[k + 1];
                }
                out.push_back(a0 + (a1 - a0) * static_cast<float>(f));
                rsPos += step;
            }
            std::size_t drop = static_cast<std::size_t>(rsPos);
            drop = drop > 1 ? std::min(drop - 1, hist.size()) : 0;
            if (drop) {
                hist.erase(hist.begin(), hist.begin() + drop);
                rsPos -= drop;
            }
            res = out.data();
            nOut = out.size();
        }
        // sdrcShift: SDRconnect tuned NEAR the centre asked for: shifted by loErr, continuously in phase
        if (lo != 0 && outRate > 0 && nOut) {
            std::complex<float>* x = const_cast<std::complex<float>*>(res);
            double w = 2 * kPi * lo / outRate;
            std::complex<double> rot(std::cos(w), std::sin(w)), ph = phase;
            for (std::size_t i = 0; i < nOut; ++i) {
                x[i] = std::complex<float>(static_cast<float>(x[i].real() * ph.real() - x[i].imag() * ph.imag()),
                                           static_cast<float>(x[i].real() * ph.imag() + x[i].imag() * ph.real()));
                ph *= rot;
            }
            double mag = std::abs(ph);
            phase = mag > 0 ? ph / mag : std::complex<double>(1, 0);
        }
        {
            std::lock_guard<std::mutex> lk(m);
            iqSamples += nOut;
        }
        if (icb && nOut) icb(res, nOut);
    }
};

// ---------------------------------------------------------------- Client ----
Client::Client() : d_(new Impl()) {}

Client::~Client() { disconnect(); }

bool Client::parseAddress(const std::string& address, std::string& host, int& port, std::string& path,
                          std::string& error) {
    std::string a = trimStr(address);
    if (a.empty()) { error = "No address: enter the PC name or IP address of the computer running SDRconnect"; return false; }
    std::string la = lowerStr(a);
    if (la.compare(0, 6, "wss://") == 0) {
        error = "wss:// isn't supported: SDRconnect's WebSocket API is plain ws://, on port 5454";
        return false;
    }
    if (la.compare(0, 5, "ws://") == 0) a = a.substr(5);
    std::size_t slash = a.find('/');
    std::string hp = a.substr(0, slash);
    path = slash == std::string::npos ? "/" : a.substr(slash);
    port = kDefaultPort;
    std::string portStr;
    if (!hp.empty() && hp[0] == '[') {
        std::size_t e = hp.find(']');
        if (e == std::string::npos) { error = "That address isn't usable (" + address + ")"; return false; }
        host = hp.substr(1, e - 1);
        if (e + 1 < hp.size()) {
            if (hp[e + 1] != ':') { error = "That address isn't usable (" + address + ")"; return false; }
            portStr = hp.substr(e + 2);
        }
    } else {
        std::size_t c = hp.find(':');
        if (c != std::string::npos && hp.find(':', c + 1) == std::string::npos) {
            host = hp.substr(0, c);
            portStr = hp.substr(c + 1);
        } else {
            host = hp;  // a name, an IPv4 address, or a bare IPv6 address (no port)
        }
    }
    if (host.empty()) { error = "That address isn't usable (" + address + ")"; return false; }
    if (!portStr.empty() || (hp.size() && hp[hp.size() - 1] == ':')) {
        char* end = 0;
        long v = std::strtol(portStr.c_str(), &end, 10);
        if (portStr.empty() || *end != '\0' || v < 1 || v > 65535) {
            error = "That address isn't usable (" + address + ")";
            return false;
        }
        port = static_cast<int>(v);
    }
    return true;
}

bool Client::connect(const std::string& address, std::string& error, int timeoutMs) {
    if (d_->onWorker()) { error = "connect() can't be called from a callback"; return false; }
    disconnect();
    std::string host, path;
    int port;
    if (!parseAddress(address, host, port, path, error)) return false;
    bool v6 = host.find(':') != std::string::npos;
    char ps[16];
    std::snprintf(ps, sizeof ps, "%d", port);
    std::string url = "ws://" + (v6 ? "[" + host + "]" : host) + ":" + ps + (path == "/" ? "" : path);
    // (port 50000 is SDRconnect's OWN server, in SDRplay's private protocol: said plainly)
    bool p50k = port == 50000;
    const std::string msg50k =
        "Port 50000 is SDRconnect's own server (for SDRplay's desktop and web clients), which OFFgrid-SDR can't use. "
        "It needs SDRconnect's WebSocket API, on port 5454: switch WebSocket on in SDRconnect's Settings, or on a computer "
        "without a screen run SDRconnect's headless program (from version 1.0.7) instead of \"--server\", then use the address without :50000";
    const std::string noAnswer = "No answer from SDRconnect at " + url +
                                 ": is it running, with its WebSocket API switched on, and port " + ps + " allowed through the firewall?";
    TimePoint end = Clock::now() + std::chrono::milliseconds(timeoutMs);
    Impl& d = *d_;
    {
        std::lock_guard<std::mutex> lk(d.m);
        d.host = host; d.port = port; d.path = path; d.url = url;
        d.quit = false; d.closing = false; d.apiSeen = false;
        d.props.clear();
        d.link = LinkState::Connecting;
        d.rc = Impl::RcNone;
        d.iqOn = d.liteOn = d.wantStream = d.speedTesting = false;
        d.waiting = d.rangePending = d.retuneArmed = false;
        d.lnaSent = kNoLna;
        d.antSentValid = false;
        d.ctlWarned = false;
        d.loErr = 0;
        d.dspReset = true;
        d.tickAt = Clock::now();
        d.bytes = d.bytesAtTick = 0;
    }
    std::shared_ptr<WebSocket> w(new WebSocket());
    std::string e;
    if (!w->connect(host, port, path, timeoutMs, e)) {
        std::lock_guard<std::mutex> lk(d.m);
        d.link = LinkState::Off;
        error = p50k ? msg50k
                     : e.find("timed out") != std::string::npos
                           ? noAnswer
                           : "Couldn't connect to " + url + ": check the address, and that SDRconnect is running there with its WebSocket API switched on (port 5454) [" + e + "]";
        return false;
    }
    {
        std::lock_guard<std::mutex> lk(d.m);
        d.ws = w;
    }
    d.worker = std::thread(&Impl::run, d_.get());
    Outbox ob;
    Impl::queueHello(ob);
    d.flush(ob);
    bool ok;
    {
        std::unique_lock<std::mutex> lk(d.m);
        while (!d.apiSeen && d.ws && Clock::now() < end) d.cv.wait_until(lk, std::min(end, Clock::now() + std::chrono::milliseconds(50)));
        ok = d.apiSeen;
        if (ok) {
            // (a moment for the other replies, the device's name among them)
            TimePoint t = Clock::now() + std::chrono::milliseconds(250);
            while (d.ws && Clock::now() < t) d.cv.wait_until(lk, t);
            ok = d.ws != 0;
        }
        if (!ok) error = p50k ? msg50k : d.ws ? noAnswer : "The connection to " + url + " closed";
        else d.postState(LinkState::Connected, "Connected to SDRconnect at " + url + " (API " + d.prop("api_version") + ")");
    }
    if (!ok) disconnect();
    return ok;
}

void Client::disconnect() {
    Impl& d = *d_;
    if (d.onWorker()) {  // from a callback: closed, and the thread ends by itself (joined later)
        std::shared_ptr<WebSocket> w;
        {
            std::lock_guard<std::mutex> lk(d.m);
            d.closing = true;
            d.wantStream = false;
            w = d.ws;
        }
        if (w) w->close(1000, "OFFgrid-SDR powered off");
        return;
    }
    if (!d.worker.joinable()) return;
    bool doStop;
    {
        std::lock_guard<std::mutex> lk(d.m);
        doStop = d.connectedLocked() && (d.wantStream || d.speedTesting);
    }
    if (doStop) stop();
    std::shared_ptr<WebSocket> w;
    {
        std::lock_guard<std::mutex> lk(d.m);
        d.closing = true;
        d.rc = Impl::RcNone;
        w = d.ws;
    }
    if (w) {
        w->close(1000, "OFFgrid-SDR powered off");
        std::unique_lock<std::mutex> lk(d.m);
        TimePoint t = Clock::now() + std::chrono::milliseconds(1000);
        while (d.ws && Clock::now() < t) d.cv.wait_until(lk, t);  // the server's close (or the end)
    }
    {
        std::lock_guard<std::mutex> lk(d.m);
        d.quit = true;
        w = d.ws;
    }
    d.cv.notify_all();
    if (w) w->abort();
    d.worker.join();
    if (d.restarter.joinable()) d.restarter.join();
    std::lock_guard<std::mutex> lk(d.m);
    d.ws.reset();
    d.link = LinkState::Off;
    d.iqOn = d.liteOn = d.wantStream = false;
    d.workerId = std::thread::id();
    d.posted.clear();
}

bool Client::isConnected() const {
    std::lock_guard<std::mutex> lk(d_->m);
    return d_->connectedLocked();
}
LinkState Client::state() const {
    std::lock_guard<std::mutex> lk(d_->m);
    return d_->link;
}
std::string Client::url() const {
    std::lock_guard<std::mutex> lk(d_->m);
    return d_->url;
}
void Client::setAutoReconnect(bool on, int retryMs, int firstMs) {
    std::lock_guard<std::mutex> lk(d_->m);
    d_->autoReconnect = on;
    d_->retryMs = std::max(50, retryMs);
    d_->firstMs = std::max(0, firstMs);
}
void Client::setStateCallback(StateCallback cb) {
    std::lock_guard<std::mutex> lk(d_->m);
    d_->stateCb = cb;
}
void Client::setPropertyCallback(PropertyCallback cb) {
    std::lock_guard<std::mutex> lk(d_->m);
    d_->propCb = cb;
}

std::string Client::apiVersion() const { return property("api_version"); }
std::string Client::device() const { return property("active_device"); }
std::vector<std::string> Client::devices() const { return splitList(property("valid_devices")); }
bool Client::canControl() const { return property("can_control") != "false"; }
bool Client::overload() const { return isTrue(property("overload")); }
bool Client::deviceStarted() const { return isTrue(property("started")); }
std::string Client::property(const std::string& name) const {
    std::lock_guard<std::mutex> lk(d_->m);
    return d_->prop(name);
}
bool Client::hasProperty(const std::string& name) const {
    std::lock_guard<std::mutex> lk(d_->m);
    return d_->props.count(name) > 0;
}

bool Client::startDevice(std::string& error) {
    if (d_->onWorker()) { error = "startDevice() can't be called from a callback"; return false; }
    return d_->startDeviceImpl(error);
}

bool Client::startIq(IqCallback cb, std::string& error) {
    Impl& d = *d_;
    if (d.onWorker()) { error = "startIq() can't be called from a callback"; return false; }
    {
        std::lock_guard<std::mutex> lk(d.m);
        if (!d.connectedLocked()) { error = "Not connected to SDRconnect"; return false; }
        if (d.speedTesting) { error = "The speed test is running"; return false; }
        d.iqCb = cb;
    }
    if (!d.startDeviceImpl(error)) return false;
    Outbox ob;
    {
        std::lock_guard<std::mutex> lk(d.m);
        d.wantStream = true;
        d.liteWanted = false;
        if (d.liteOn) d.liteOffLocked(ob);
        else if (!d.iqOn) {
            d.iqOn = true;
            d.dspReset = true;
            Impl::q(ob, "iq_stream_enable", "", "true");
        }
        d.applyGain(ob);
    }
    d.flush(ob);
    return true;
}

bool Client::startLite(AudioCallback audio, SpectrumCallback spectrum, std::string& error) {
    Impl& d = *d_;
    if (d.onWorker()) { error = "startLite() can't be called from a callback"; return false; }
    {
        std::lock_guard<std::mutex> lk(d.m);
        if (!d.connectedLocked()) { error = "Not connected to SDRconnect"; return false; }
        if (d.speedTesting) { error = "The speed test is running"; return false; }
        d.audioCb = audio;
        d.specCb = spectrum;
    }
    if (!d.startDeviceImpl(error)) return false;
    Outbox ob;
    {
        std::lock_guard<std::mutex> lk(d.m);
        d.wantStream = true;
        d.liteWanted = true;
        if (!d.liteOn) d.liteOnLocked(ob);
        d.applyGain(ob);
    }
    d.flush(ob);
    return true;
}

bool Client::setLite(bool on) {
    Impl& d = *d_;
    Outbox ob;
    {
        std::lock_guard<std::mutex> lk(d.m);
        d.liteWanted = on;
        if (!d.connectedLocked() || !d.wantStream) return true;  // used by the next start (after a reconnection too)
        if (on && !d.liteOn) d.liteOnLocked(ob);
        else if (!on && d.liteOn) d.liteOffLocked(ob);
    }
    d.flush(ob);
    return true;
}
bool Client::lite() const {
    std::lock_guard<std::mutex> lk(d_->m);
    return d_->liteOn || (!d_->wantStream && d_->liteWanted);
}
bool Client::streaming() const {
    std::lock_guard<std::mutex> lk(d_->m);
    return d_->wantStream;
}

void Client::stop() {
    Impl& d = *d_;
    if (d.onWorker()) return;
    std::unique_lock<std::mutex> lk(d.m);
    if (!d.connectedLocked()) { d.wantStream = d.iqOn = d.liteOn = false; return; }
    // sdrcStop: the streams off, then the device, waiting until SDRconnect says it has stopped (3 s at most)
    Outbox ob;
    Impl::q(ob, "iq_stream_enable", "", "false");
    if (d.liteOn) {
        Impl::q(ob, "audio_stream_enable", "", "false");
        Impl::q(ob, "spectrum_enable", "", "false");
    }
    d.wantStream = d.iqOn = d.liteOn = false;
    lk.unlock(); d.flush(ob); lk.lock();
    d.sleepLocked(lk, 150);
    Impl::q(ob, "device_stream_enable", "", "false");
    lk.unlock(); d.flush(ob); lk.lock();
    d.waitUntil(lk, 3000, [&d]() { return !d.startedLocked(); }, "started", 300);
}

bool Client::setFrequency(double hz) {
    if (!(hz > 0)) return false;
    Outbox ob;
    {
        std::lock_guard<std::mutex> lk(d_->m);
        d_->setFrequencyLocked(ob, hz, false);
    }
    d_->flush(ob);
    return true;
}
double Client::frequency() const {
    std::lock_guard<std::mutex> lk(d_->m);
    return d_->wantCenter;
}
double Client::reportedFrequency() const {
    double v = toNum(property("device_center_frequency"));
    return std::isnan(v) ? 0 : v;
}
bool Client::setSampleRate(double hz) {
    if (!(hz > 0)) return false;
    Outbox ob;
    {
        std::lock_guard<std::mutex> lk(d_->m);
        d_->setRateLocked(ob, hz, false);
    }
    d_->flush(ob);
    return true;
}
double Client::sampleRate() const {
    std::lock_guard<std::mutex> lk(d_->m);
    if (d_->wantRate > 0) return d_->wantRate;
    double v = toNum(d_->prop("device_sample_rate"));
    return std::isnan(v) ? 0 : v;
}
double Client::deviceSampleRate() const {
    double v = toNum(property("device_sample_rate"));
    return std::isnan(v) ? 0 : v;
}

bool Client::setGainAuto() {
    Outbox ob;
    {
        std::lock_guard<std::mutex> lk(d_->m);
        d_->gainMode = Impl::GainAuto;
        d_->applyGain(ob);
    }
    d_->flush(ob);
    return true;
}
bool Client::setGainFraction(double fraction) {
    Outbox ob;
    {
        std::lock_guard<std::mutex> lk(d_->m);
        d_->gainMode = Impl::GainFraction;
        d_->gainFraction = std::max(0.0, std::min(1.0, fraction));
        d_->applyGain(ob);
    }
    d_->flush(ob);
    return true;
}
bool Client::setLnaState(int state) {
    Outbox ob;
    {
        std::lock_guard<std::mutex> lk(d_->m);
        d_->gainMode = Impl::GainState;
        d_->gainState = state;
        d_->applyGain(ob);
    }
    d_->flush(ob);
    return true;
}
bool Client::lnaRange(int& minState, int& maxState) const {
    std::lock_guard<std::mutex> lk(d_->m);
    double lo = toNum(d_->prop("lna_state_min")), hi = toNum(d_->prop("lna_state_max"));
    if (std::isnan(lo) || std::isnan(hi) || !(hi > lo)) return false;
    minState = static_cast<int>(lo);
    maxState = static_cast<int>(hi);
    return true;
}
int Client::lnaState() const {
    double v = toNum(property("lna_state"));
    return std::isnan(v) ? -1 : static_cast<int>(v);
}
// sdrcLnaFor: the slider's first 90% over the safe 70% of the range (from the
// least gain), the last 10% over the rest, to full gain (state lo)
int Client::lnaForFraction(double frac, int lo, int hi) {
    const double SAFE = 0.7, KNEE = 0.9;
    frac = std::max(0.0, std::min(1.0, frac));
    double safe = hi - SAFE * (hi - lo);
    double v = frac <= KNEE ? hi - (frac / KNEE) * (hi - safe) : safe - ((frac - KNEE) / (1 - KNEE)) * (safe - lo);
    return static_cast<int>(std::floor(v + 0.5));
}

std::vector<std::string> Client::antennas() {
    std::lock_guard<std::mutex> lk(d_->m);
    return d_->antListLocked();
}
bool Client::probeAntennas() {
    Impl& d = *d_;
    if (d.onWorker()) return false;
    std::unique_lock<std::mutex> lk(d.m);
    if (!d.connectedLocked()) return false;
    Outbox ob;
    double was = toNum(d.prop("device_center_frequency"));
    if (!std::isnan(was) && was > 0 && was <= 30e6) {
        Impl::q(ob, "get_property", "valid_antennas", "");
        lk.unlock(); d.flush(ob); lk.lock();
        d.sleepLocked(lk, 400);
        d.antListLocked();
        return true;
    }
    // (above 30 MHz: medium wave (1 MHz) for a moment, so SDRconnect lists them all, then back)
    bool streamingNow = d.wantStream;
    if (streamingNow) d.waiting = true;  // (Addition to the JS: nothing from 1 MHz reaches the samples)
    d.props.erase("valid_antennas");
    Impl::q(ob, "set_property", "device_center_frequency", "1000000");
    lk.unlock(); d.flush(ob); lk.lock();
    d.waitUntil(lk, 1500, [&d]() { return std::fabs(toNum(d.prop("device_center_frequency")) - 1e6) <= 5000; });
    d.props.erase("valid_antennas");
    Impl::q(ob, "get_property", "valid_antennas", "");
    lk.unlock(); d.flush(ob); lk.lock();
    bool got = d.waitUntil(lk, 1500, [&d]() { return d.props.count("valid_antennas") > 0; });
    d.antListLocked();
    if (!std::isnan(was) && was > 0) {
        Impl::q(ob, "set_property", "device_center_frequency", intStr(was));
        lk.unlock(); d.flush(ob); lk.lock();
        d.waitUntil(lk, 1500, [&d, was]() { return std::fabs(toNum(d.prop("device_center_frequency")) - was) <= 5000; });
        Impl::q(ob, "get_property", "valid_antennas", "");
    }
    if (streamingNow && d.wantCenter > 0) d.setFrequencyLocked(ob, d.wantCenter, true);
    lk.unlock();
    d.flush(ob);
    return got;
}
void Client::rememberAntennas(const std::vector<std::string>& seen) {
    std::lock_guard<std::mutex> lk(d_->m);
    std::string dev = d_->prop("active_device");
    if (dev.empty()) dev = "?";
    std::vector<std::string>& v = d_->seenAnts[dev];
    for (std::size_t i = 0; i < seen.size(); ++i)
        if (std::find(v.begin(), v.end(), seen[i]) == v.end()) v.push_back(seen[i]);
    std::sort(v.begin(), v.end());
}
std::string Client::activeAntenna() const { return property("active_antenna"); }
bool Client::setAntenna(const std::string& name) {
    Outbox ob;
    {
        std::lock_guard<std::mutex> lk(d_->m);
        d_->autoAnt = false;
        d_->antSent = name;
        d_->antSentValid = true;
        Impl::q(ob, "set_property", "active_antenna", name);
    }
    d_->flush(ob);
    return true;
}
void Client::setAutoAntenna(bool on) {
    Outbox ob;
    {
        std::lock_guard<std::mutex> lk(d_->m);
        d_->autoAnt = on;
        if (on && d_->wantCenter > 0 && d_->connectedLocked() && d_->startedLocked()) d_->autoAntenna(ob, d_->wantCenter);
    }
    d_->flush(ob);
}
AntennaRules Client::antennaRules() {
    std::lock_guard<std::mutex> lk(d_->m);
    AntennaRules* r = d_->rulesLocked();
    return r ? *r : AntennaRules();
}
void Client::setAntennaRules(const AntennaRules& rules) {
    Outbox ob;
    {
        std::lock_guard<std::mutex> lk(d_->m);
        std::string dev = d_->prop("active_device");
        if (dev.empty()) dev = "?";
        d_->rules[dev] = rules;
        d_->rules[dev].user = true;
        if (d_->wantCenter > 0 && d_->connectedLocked() && d_->startedLocked()) d_->autoAntenna(ob, d_->wantCenter);
    }
    d_->flush(ob);
}
std::string Client::antennaFor(double hz) {
    std::lock_guard<std::mutex> lk(d_->m);
    return d_->antennaForLocked(hz);
}
double Client::antennaLimitHz(const std::string& device, const std::string& antenna) {
    return isRspDx(device) && endsWithC(antenna) ? 200e6 : HUGE_VAL;
}

bool Client::setLiteTuning(const std::string& demodulator, double vfoHz, double bandwidthHz) {
    Outbox ob;
    {
        std::lock_guard<std::mutex> lk(d_->m);
        d_->liteWantDm = demodulator;
        d_->liteWantVfo = vfoHz;
        d_->liteWantBw = bandwidthHz;
        d_->liteSyncLocked(ob);
    }
    d_->flush(ob);
    return true;
}
std::string Client::liteDemodulator(const std::string& mode) {
    // SDRC_DEMOD; data modes as upper sideband
    static const char* const tbl[][2] = {{"AM", "AM"}, {"SAM", "SAM"}, {"USB", "USB"}, {"LSB", "LSB"},  {"CW", "CW"},
                                          {"NBFM", "NFM"}, {"APRS", "NFM"}, {"AIS", "NFM"}, {"WBFM", "WFM"}};
    for (std::size_t i = 0; i < sizeof tbl / sizeof tbl[0]; ++i)
        if (mode == tbl[i][0]) return tbl[i][1];
    return "USB";
}
bool Client::liteSupports(const std::string& mode) {
    return !(mode == "DAB" || mode == "DRM" || mode == "AIS" || mode == "VDL2");
}
LiteStats Client::liteStats() const {
    std::lock_guard<std::mutex> lk(d_->m);
    return d_->liteStat;
}
float Client::spectrumDb(std::uint8_t value) const {
    std::lock_guard<std::mutex> lk(d_->m);
    double r = toNum(d_->prop("spectrum_ref_level")), b = toNum(d_->prop("spectrum_base"));
    if (std::isnan(r)) r = 0;
    if (std::isnan(b)) b = -120;
    return static_cast<float>(b + value / 255.0 * (r - b));
}

void Client::setStartProfile(const std::string& name) {
    std::lock_guard<std::mutex> lk(d_->m);
    d_->startProfile = trimStr(name);
}
bool Client::applyProfile(const std::string& name) {
    Impl& d = *d_;
    if (d.onWorker()) return false;
    std::unique_lock<std::mutex> lk(d.m);
    if (!d.connectedLocked() || trimStr(name).empty()) return false;
    Outbox ob;
    Impl::q(ob, "apply_device_profile", "", trimStr(name));
    lk.unlock(); d.flush(ob); lk.lock();
    d.sleepLocked(lk, 600);
    Impl::q(ob, "get_property", "lna_state", "");
    Impl::q(ob, "get_property", "active_antenna", "");
    Impl::q(ob, "get_property", "device_sample_rate", "");
    d.resendTuning(ob, true);
    lk.unlock();
    d.flush(ob);
    return true;
}

SpeedResult Client::judgeSpeed(double got, double rate) {
    SpeedResult r;
    r.ok = true;
    r.gotMBs = got;
    r.rate = rate;
    r.needMBs = rate * 4 / 1e6;
    r.percent = r.needMBs > 0 ? static_cast<int>(std::floor(100 * got / r.needMBs + 0.5)) : 0;
    // (a link can only deliver what's sent: a fast one measures about 100%, never more)
    bool ok = got >= r.needMBs * 0.97, tight = !ok && got >= r.needMBs * 0.9;
    r.verdict = ok ? SpeedVerdict::KeepsUp : tight ? SpeedVerdict::OnlyJust : SpeedVerdict::TooSlow;
    char b[512];
    double mhz = rate / 1e6;
    if (ok)
        std::snprintf(b, sizeof b, "Network: %.1f MB/s of the %.1f the full stream needs at %.3f MHz: it keeps up.", got, r.needMBs, mhz);
    else if (tight)
        std::snprintf(b, sizeof b, "Network: %.1f MB/s, the full stream needs %.1f at %.3f MHz (%d%%): only just, so a busy moment will make gaps. Lite mode would be safe.", got, r.needMBs, mhz, r.percent);
    else
        std::snprintf(b, sizeof b, "Network: %.1f MB/s, but the full stream needs %.1f at %.3f MHz (%d%%): too slow, the sound and display would break up. Lite mode needs about 0.2 MB/s.", got, r.needMBs, mhz, r.percent);
    r.text = b;
    return r;
}

SpeedResult Client::speedTest(double rate, int measureMs) {
    Impl& d = *d_;
    SpeedResult res;
    if (d.onWorker()) { res.error = "speedTest() can't be called from a callback"; return res; }
    std::unique_lock<std::mutex> lk(d.m);
    if (!d.connectedLocked()) { res.error = "Not connected to SDRconnect"; return res; }
    if (d.wantStream || d.speedTesting) { res.error = "Stop the stream first: the test takes SDRconnect's full stream itself"; return res; }
    if (!(rate > 0)) rate = d.wantRate > 0 ? d.wantRate : 2048000;
    measureMs = std::max(200, measureMs);
    bool was = d.startedLocked();
    Outbox ob;
    Impl::q(ob, "set_property", "device_sample_rate", intStr(rate));
    if (!was) Impl::q(ob, "device_stream_enable", "", "true");
    lk.unlock(); d.flush(ob); lk.lock();
    if (!was && !d.waitUntil(lk, 6000, [&d]() { return d.startedLocked(); }, "started", 1000)) {
        res.error = "SDRconnect didn't start the RSP for the test";
        return res;
    }
    d.speedTesting = true;
    Impl::q(ob, "iq_stream_enable", "", "true");
    lk.unlock(); d.flush(ob); lk.lock();
    // (the first half second is a burst from SDRconnect's own buffers, which would flatter the link)
    d.sleepLocked(lk, 500);
    std::uint64_t b0 = d.bytes;
    TimePoint t0 = Clock::now();
    d.sleepLocked(lk, measureMs);
    double secs = std::chrono::duration<double>(Clock::now() - t0).count();
    double got = (d.bytes - b0) / 1e6 / std::max(0.001, secs);
    bool lost = d.aborted();
    Impl::q(ob, "iq_stream_enable", "", "false");
    lk.unlock(); d.flush(ob); lk.lock();
    d.sleepLocked(lk, 150);
    // (stopped again if the test started it, waiting until SDRconnect says so)
    if (!was) {
        Impl::q(ob, "device_stream_enable", "", "false");
        lk.unlock(); d.flush(ob); lk.lock();
        d.waitUntil(lk, 3000, [&d]() { return !d.startedLocked(); }, "started", 300);
    }
    d.speedTesting = false;
    // (Addition to the JS: the rate asked for before the test put back)
    if (d.wantRate > 0 && std::fabs(d.wantRate - rate) > 1) Impl::q(ob, "set_property", "device_sample_rate", intStr(d.wantRate));
    lk.unlock();
    d.flush(ob);
    if (lost) { res.error = "The connection to SDRconnect was lost during the test"; return res; }
    return judgeSpeed(got, rate);
}

bool Client::sendEvent(const std::string& eventType, const std::string& property, const std::string& value) {
    std::shared_ptr<WebSocket> w;
    {
        std::lock_guard<std::mutex> lk(d_->m);
        w = d_->ws;
    }
    return w && w->sendText(json::event(eventType, property, value));
}
bool Client::requestProperty(const std::string& name) { return sendEvent("get_property", name, ""); }
bool Client::setProperty(const std::string& name, const std::string& value) { return sendEvent("set_property", name, value); }

LinkStats Client::stats() const {
    std::lock_guard<std::mutex> lk(d_->m);
    LinkStats s;
    s.bytes = d_->bytes;
    s.MBs = d_->MBs;
    s.iqSamples = d_->iqSamples;
    s.droppedMessages = d_->dropped;
    s.reconnects = d_->reconnects;
    return s;
}

void Client::toU8(const std::complex<float>* iq, std::size_t n, std::uint8_t* out) {
    for (std::size_t i = 0; i < n; ++i) {
        float a = 127.5f + iq[i].real() * 127.5f, b = 127.5f + iq[i].imag() * 127.5f;
        out[2 * i] = static_cast<std::uint8_t>(std::max(0.0f, std::min(255.0f, std::floor(a + 0.5f))));
        out[2 * i + 1] = static_cast<std::uint8_t>(std::max(0.0f, std::min(255.0f, std::floor(b + 0.5f))));
    }
}
std::size_t Client::s16ToComplex(const unsigned char* p, std::size_t len, std::complex<float>* out) {
    std::size_t n = len / 4;
    for (std::size_t i = 0; i < n; ++i) {
        const unsigned char* s = p + 4 * i;
        out[i] = std::complex<float>(static_cast<std::int16_t>(s[0] | (s[1] << 8)) / 32768.0f,
                                     static_cast<std::int16_t>(s[2] | (s[3] << 8)) / 32768.0f);
    }
    return n;
}

}  // namespace sdrc
