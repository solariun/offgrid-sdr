// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Ft8.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The live FT8 / FT4 decoders of Ft8.h: a port of OFFgrid-SDR's ft8Feed()
//  (resampling, the stream's timing, decoding at each slot's end),
//  ft8Done() (slot, DT, duplicates, the grids heard, the timings for the
//  clock, the adaptive passes), ftxInfo() and the FT8 box rows (diddy-boy,
//  index.html).  The decoding is Ftx.h's (after ft8_lib, Kārlis Goba, MIT).
//
//  Additions to the JS (each marked where it is):
//    - the audio is taken in chunks of at most 0.1 s, the slot's end checked
//      after each, so the result does not depend on the block size (the
//      browser: once per audio block);
//    - an anti-alias low-pass (6th-order Butterworth at 0.375 x the
//      decoder's rate) before the linear interpolation when the input rate
//      is higher (the browser counts on the USB filter);
//    - the stream's start creeps up by 0.2 ms per second of audio (the
//      browser: 20 us per block, whatever its length);
//    - DT from the fine-tuned start (Ftx.cpp's refine) rather than the
//      spectrogram's 80 ms (FT8) / 24 ms (FT4) step;
//    - ftxInfo: "CALL1 CALL2 R GRID" (the R and the grid are two words) is a
//      reply with that grid (the JS saw only "call"); DXpedition messages
//      ("K1ABC RR73; W9XYZ <KH1/KH7Z> -08") give the DX station as the
//      caller; "TU;" before an RTTY Roundup message is skipped; a word with
//      ';' is never a call;
//    - decodeNow(), text lines, the decodes' structured fields.
//
//  How it fits in
//  --------------
//  See Ft8.h.
//
//  Contributing
//  ------------
//  Keep the names and thresholds parallel to the JavaScript; mark
//  deviations.  No allocation per sample.
// ============================================================================
#include "Ft8.h"

#include "Callsign.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace dec {

namespace {

const double kPi = 3.14159265358979323846;
const std::size_t kMaxList = 400;     // decodes kept (the browser's ft8.list)
const std::size_t kMaxGrids = 3000;   // grids remembered (ft8.grids)
const double kChunkSeconds = 0.1;     // (Addition to the JS) audio worked at a time

bool finite(double v) { return v == v && v - v == 0; }

std::vector<std::string> splitWords(const std::string& s) {
    std::vector<std::string> w;
    std::size_t a = 0;
    while (a < s.size()) {
        while (a < s.size() && (s[a] == ' ' || s[a] == '\t')) ++a;
        std::size_t b = a;
        while (b < s.size() && s[b] != ' ' && s[b] != '\t') ++b;
        if (b > a) w.push_back(s.substr(a, b - a));
        a = b;
    }
    return w;
}

// the JS isGrid4: /^[A-R]{2}[0-9]{2}$/ and not RR73
bool isGrid4(const std::string& t) {
    return t.size() == 4 && t[0] >= 'A' && t[0] <= 'R' && t[1] >= 'A' && t[1] <= 'R' && t[2] >= '0' && t[2] <= '9' &&
           t[3] >= '0' && t[3] <= '9' && t != "RR73";
}

// /^[RS]?[+-][0-9]{2}$/
bool isReportWord(const std::string& t) {
    std::size_t i = 0;
    if (!t.empty() && (t[0] == 'R' || t[0] == 'S')) i = 1;
    return t.size() == i + 3 && (t[i] == '+' || t[i] == '-') && t[i + 1] >= '0' && t[i + 1] <= '9' && t[i + 2] >= '0' &&
           t[i + 2] <= '9';
}

// the JS isCall: "<...>", or letters and a digit, 3+ characters, not a report or a locator.  (Addition to the JS:
// never a word with ';', as "RR73;" in DXpedition messages.)
bool isCall(const std::string& t) {
    if (t.size() >= 2 && t[0] == '<' && t[t.size() - 1] == '>') return true;
    if (t.find(';') != std::string::npos) return false;
    bool d = false, l = false;
    for (std::size_t i = 0; i < t.size(); ++i) {
        if (t[i] >= '0' && t[i] <= '9') d = true;
        if (t[i] >= 'A' && t[i] <= 'Z') l = true;
    }
    return d && l && t.size() >= 3 && !isReportWord(t) && !isGrid4(t);
}

// "HHMMSS" of a Unix time (UTC; arithmetic only, no gmtime: thread-safe)
std::string utcText(double t) {
    long long s = static_cast<long long>(std::floor(t)) % 86400;
    if (s < 0) s += 86400;
    char b[16];
    std::snprintf(b, sizeof b, "%02d%02d%02d", static_cast<int>(s / 3600), static_cast<int>(s / 60 % 60), static_cast<int>(s % 60));
    return b;
}

}  // namespace

FtxInfo ftxInfo(const std::string& text) {
    FtxInfo r;
    std::vector<std::string> w = splitWords(text);
    if (!w.empty() && w[0] == "TU;") w.erase(w.begin());   // (Addition to the JS: RTTY Roundup)
    r.type = "text";
    if (w.empty()) return r;
    // (Addition to the JS) DXpedition: "K1ABC RR73; W9XYZ <KH1/KH7Z> -08": the DX station reports to W9XYZ
    if (w.size() == 5 && w[1] == "RR73;" && isCall(w[2]) && isCall(w[3]) && isReportWord(w[4])) {
        r.type = "report";
        r.from = w[3];
        r.to = w[2];
        r.hasReport = true;
        r.report = std::atoi(w[4].c_str());
        return r;
    }
    if (w[0] == "CQ" || w[0] == "QRZ") {
        std::size_t i = 1;
        std::string cqTo;
        if (w.size() > 2 && !isCall(w[1]) && isCall(w[2])) cqTo = w[i++];
        if (i < w.size() && isCall(w[i])) {
            r.type = "CQ";
            r.from = w[i];
            if (i + 1 < w.size() && isGrid4(w[i + 1])) r.grid = w[i + 1];
            r.cqTo = cqTo;
        }
        return r;
    }
    if (w.size() >= 2 && isCall(w[0]) && isCall(w[1])) {
        const std::string x = w.size() > 2 ? w[2] : "";
        r.from = w[1];
        r.to = w[0];
        if (isGrid4(x)) { r.type = "reply"; r.grid = x; return r; }
        if (x == "R" && w.size() > 3 && isGrid4(w[3])) { r.type = "reply"; r.grid = w[3]; return r; }   // (Addition)
        if (x.size() == 5 && x[0] == 'R' && isGrid4(x.substr(1))) { r.type = "reply"; r.grid = x.substr(1); return r; }
        if (isReportWord(x) && x[0] != 'S') {
            r.type = x[0] == 'R' ? "R report" : "report";
            r.hasReport = true;
            r.report = std::atoi(x.c_str() + (x[0] == 'R' ? 1 : 0));
            return r;
        }
        if (x == "RRR") { r.type = "RRR"; return r; }
        if (x == "RR73") { r.type = "RR73"; return r; }
        if (x == "73") { r.type = "73"; return r; }
        r.type = "call";
        return r;
    }
    return r;
}

const std::vector<FtxDial>& ft8Dials() {
    static const FtxDial k[] = {{"160 m", 1.840e6}, {"80 m", 3.573e6},  {"60 m", 5.357e6},   {"40 m", 7.074e6},
                                {"30 m", 10.136e6}, {"20 m", 14.074e6}, {"17 m", 18.100e6},  {"15 m", 21.074e6},
                                {"12 m", 24.915e6}, {"10 m", 28.074e6}, {"6 m", 50.313e6},   {"2 m", 144.174e6}};
    static const std::vector<FtxDial> v(k, k + sizeof(k) / sizeof(k[0]));
    return v;
}

const std::vector<FtxDial>& ft4Dials() {
    static const FtxDial k[] = {{"80 m", 3.575e6},  {"40 m", 7.0475e6}, {"30 m", 10.140e6}, {"20 m", 14.080e6},
                                {"17 m", 18.104e6}, {"15 m", 21.140e6}, {"12 m", 24.919e6}, {"10 m", 28.180e6},
                                {"6 m", 50.318e6},  {"2 m", 144.170e6}};
    static const std::vector<FtxDial> v(k, k + sizeof(k) / sizeof(k[0]));
    return v;
}

FtxDecoder::FtxDecoder(ftx::Protocol p)
    : proto_(p), info_(ftx::protocolInfo(p)), homeSet_(false), homeLat_(0), homeLon_(0), dial_(0), fLo_(200),
      fHi_(3000), deep_(true), maxPasses_(3), passes_(3), tsPending_(false), ts_(0), nextTs_(0), offset_(0),
      anchorSet_(false), anchor_(0), lastSlotSet_(false), lastSlot_(0), rate_(0), step_(1), frac_(1), prev_(0),
      lowpass_(false), n_(0), dtsPending_(false), lastMs_(0), maxMs_(0), slots_(0), total_(0), lastNew_(0),
      lastDeeper_(0), bestSnr_(0) {
    std::memset(bq_, 0, sizeof bq_);
    std::memset(z_, 0, sizeof z_);
    // the ring holds a decode window and a little more (the browser: 30 s at 12800 Hz for both)
    ring_.assign(static_cast<std::size_t>(std::ceil((info_.window + 1) * info_.sampleRate)), 0.f);
    work_.reserve(ring_.size());
}

Kind FtxDecoder::kind() const { return proto_ == ftx::Protocol::Ft8 ? Kind::Ft8 : Kind::Ft4; }

void FtxDecoder::reset() {
    // (ft8Reset(), and what a retune loses: the list, the keys)
    n_ = 0;
    anchorSet_ = false;
    lastSlotSet_ = false;
    frac_ = 1;
    prev_ = 0;
    std::memset(z_, 0, sizeof z_);
    list_.clear();
    keys_.clear();
    fresh_.clear();
    grids_.clear();
    gridOrder_.clear();
    lastDts_.clear();
    dtsPending_ = false;
    hashes_.clear();
    out_.clear();
    slots_ = total_ = lastNew_ = lastDeeper_ = 0;
    lastMs_ = maxMs_ = 0;
    bestSnr_ = 0;
    tsPending_ = false;
    nextTs_ = 0;
}

void FtxDecoder::setTimestamp(double t) {
    if (!finite(t)) return;
    ts_ = t;
    tsPending_ = true;
}

void FtxDecoder::setClockOffset(double off) {
    if (finite(off)) offset_ = off;
}

bool FtxDecoder::setHome(const std::string& grid) {
    double la, lo;
    if (!Locator::toLatLon(grid, la, lo)) { homeSet_ = false; return false; }
    homeSet_ = true;
    homeLat_ = la;
    homeLon_ = lo;
    return true;
}

void FtxDecoder::setHomeLatLon(double lat, double lon) {
    homeSet_ = finite(lat) && finite(lon);
    homeLat_ = lat;
    homeLon_ = lon;
}

void FtxDecoder::clearHome() { homeSet_ = false; }

void FtxDecoder::setMyCall(const std::string& call) {
    myCall_.clear();
    for (std::size_t i = 0; i < call.size(); ++i)
        myCall_ += static_cast<char>(std::toupper(static_cast<unsigned char>(call[i])));
}

void FtxDecoder::setBand(double lo, double hi) {
    if (!finite(lo) || !finite(hi) || hi <= lo) return;
    fLo_ = std::max(0.0, lo);
    fHi_ = hi;
}

void FtxDecoder::setMaxPasses(int n) {
    maxPasses_ = std::max(1, std::min(3, n));
    passes_ = std::min(passes_, maxPasses_);
}

void FtxDecoder::setRate(double rate) {
    rate_ = rate;
    step_ = info_.sampleRate / rate;   // output samples per input sample
    frac_ = 1;
    prev_ = 0;
    n_ = 0;
    anchorSet_ = false;
    std::memset(z_, 0, sizeof z_);
    // (Addition to the JS) above the decoder's rate: a 6th-order Butterworth low-pass (three biquads,
    // bilinear transform) at 0.375 x its rate, so nothing folds into 200-3000 Hz
    lowpass_ = rate > info_.sampleRate * 1.01;
    if (lowpass_) {
        const double fc = std::min(0.375 * info_.sampleRate, 0.45 * rate);
        const double K = std::tan(kPi * fc / rate);
        const double Q[3] = {0.51763809, 0.70710678, 1.93185165};
        for (int i = 0; i < 3; ++i) {
            const double norm = 1 / (1 + K / Q[i] + K * K);
            bq_[i][0] = K * K * norm;               // b0
            bq_[i][1] = 2 * bq_[i][0];              // b1
            bq_[i][2] = bq_[i][0];                  // b2
            bq_[i][3] = 2 * (K * K - 1) * norm;     // a1
            bq_[i][4] = (1 - K / Q[i] + K * K) * norm;  // a2
        }
    }
}

void FtxDecoder::process(const float* audio, std::size_t n, double rate) {
    if (!audio || n == 0 || !(rate >= 4000) || !finite(rate)) return;
    if (rate != rate_) setRate(rate);
    // when the block's first sample was heard, on the computer's clock (without timestamps: carrying on from the
    // last block)
    const double tBlock = tsPending_ ? ts_ : nextTs_;
    tsPending_ = false;
    nextTs_ = tBlock + n / rate;
    const std::size_t chunkN = std::max<std::size_t>(1, static_cast<std::size_t>(rate * kChunkSeconds));
    for (std::size_t o = 0; o < n; o += chunkN) {
        const std::size_t m = std::min(chunkN, n - o);
        chunk(audio + o, m, tBlock + o / rate);
    }
}

void FtxDecoder::chunk(const float* x, std::size_t n, double t0) {
    // to the decoder's rate: (low-pass, then) straight interpolation, as the browser.  Output k lies at input
    // position k * rate / fs; frac_ is the next output's position after the previous input sample.
    const std::size_t L = ring_.size();
    const double inStep = 1.0 / step_;
    for (std::size_t i = 0; i < n; ++i) {
        double v = finite(x[i]) ? x[i] : 0.0;
        if (lowpass_) {
            for (int s = 0; s < 3; ++s) {   // (transposed direct form II)
                const double y = bq_[s][0] * v + z_[s][0];
                z_[s][0] = bq_[s][1] * v - bq_[s][3] * y + z_[s][1];
                z_[s][1] = bq_[s][2] * v - bq_[s][4] * y;
                v = y;
            }
            if (!finite(v) || !finite(z_[0][0]) || !finite(z_[1][0]) || !finite(z_[2][0])) {
                std::memset(z_, 0, sizeof z_);
                v = 0;
            }
        }
        const float fv = static_cast<float>(v);
        while (frac_ <= 1.0) {
            ring_[static_cast<std::size_t>(n_ % static_cast<long long>(L))] = prev_ + (fv - prev_) * static_cast<float>(frac_);
            ++n_;
            frac_ += inStep;
        }
        frac_ -= 1.0;
        prev_ = fv;
    }
    // when was the newest sample?  Blocks can only arrive late, never early: keep the earliest estimate, creeping
    // up slowly to follow the sound card's drift; a jump of over a second (a gap) starts again
    const double fs = info_.sampleRate;
    const double est = t0 + n / rate_ - n_ / fs;
    if (!anchorSet_ || std::fabs(est - anchor_) > 1) { anchor_ = est; anchorSet_ = true; }
    else anchor_ = std::min(anchor_ + 2e-4 * (n / rate_), est);   // (Addition to the JS: per second of audio)
    // just after each slot ends (by the corrected clock): decode the last stretch (27.6 s FT8, 13 s FT4), which
    // always holds one whole transmission
    const double now = anchor_ + n_ / fs - offset_;
    const long long slot = static_cast<long long>(std::floor((now - 0.4) / info_.slotSeconds));
    if (!lastSlotSet_) { lastSlotSet_ = true; lastSlot_ = slot; return; }
    if (slot == lastSlot_) return;
    lastSlot_ = slot;
    decodeWindow();
}

void FtxDecoder::decodeWindow() {
    const double fs = info_.sampleRate;
    const long long L = static_cast<long long>(ring_.size());
    const long long len = std::min(std::min(n_, static_cast<long long>(std::floor(info_.window * fs + 0.5))), L);
    if (len < info_.minLength * fs) return;
    const long long s0 = n_ - len;
    work_.resize(static_cast<std::size_t>(len));
    for (long long i = 0; i < len; ++i) work_[static_cast<std::size_t>(i)] = ring_[static_cast<std::size_t>((s0 + i) % L)];
    // deeper decoding: decode, take the decoded stations out, decode again (up to 3 passes).  Each pass costs
    // time, and the decode must be done well inside its slot, so the number adapts to the computer: one fewer after
    // a decode that took over 60% of the slot, one more after one under 30%
    passes_ = std::max(1, std::min(maxPasses_, passes_));
    ftx::DecodeOptions opt;
    opt.fLo = fLo_;
    opt.fHi = fHi_;
    opt.passes = deep_ ? passes_ : 1;
    if (!deep_) opt.precise = 0;
    const double t0 = anchor_ + s0 / fs - offset_;   // true time of the window's first sample
    raw_.clear();
    const std::chrono::steady_clock::time_point a = std::chrono::steady_clock::now();
    engine_.decode(proto_, &work_[0], work_.size(), opt, &hashes_, raw_);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count();
    lastMs_ = ms;
    maxMs_ = std::max(maxMs_, ms);
    ++slots_;
    const double slotMs = info_.slotSeconds * 1000;
    if (deep_) {
        if (ms > 0.6 * slotMs && passes_ > 1) --passes_;
        else if (ms < 0.3 * slotMs) passes_ = std::min(maxPasses_, passes_ + 1);
    }
    lastNew_ = 0;
    lastDeeper_ = 0;
    bestSnr_ = -99;
    std::vector<double> dts;
    for (std::size_t i = 0; i < raw_.size(); ++i) addDecode(raw_[i], t0, dts);
    // the oldest decodes go (the browser keeps 400)
    while (list_.size() > kMaxList) {
        const FtxDecode& d = list_.front();
        char k[64];
        std::snprintf(k, sizeof k, "%.1f|", d.slotTime);
        keys_.erase(k + d.text);
        list_.pop_front();
    }
    // the timings, for the clock (ClockCorrection::applyFtxTimings)
    if (!dts.empty()) {
        lastDts_ = dts;
        dtsPending_ = true;
    }
}

void FtxDecoder::addDecode(const ftx::RawDecode& r, double t0, std::vector<double>& dts) {
    const double fs = info_.sampleRate, L = info_.slotSeconds, tx = info_.txStart;
    const double t = t0 + r.start / fs;
    const double slot = std::floor((t - tx) / L + 0.5);
    const double dt = t - tx - slot * L;
    char kb[64];
    std::snprintf(kb, sizeof kb, "%.1f|", slot * L);
    const std::string key = kb + r.text;
    if (keys_.count(key)) return;   // (heard in the previous window already)
    keys_.insert(key);
    ++lastNew_;
    ++total_;
    if (r.pass > 1) ++lastDeeper_;
    FtxDecode d;
    d.proto = info_.name;
    d.slotTime = slot * L;
    d.slotSeconds = L;
    d.utc = utcText(d.slotTime);
    d.snr = r.snr;
    d.dt = dt;
    d.freq = r.freq;
    d.text = r.text;
    d.pass = r.pass;
    const FtxInfo inf = ftxInfo(r.text);
    d.type = inf.type;
    d.caller = inf.from;
    d.callee = inf.to;
    d.cqTo = inf.cqTo;
    d.cq = inf.type == "CQ";
    d.hasReport = inf.hasReport;
    d.report = inf.report;
    if (!inf.from.empty() && !inf.grid.empty()) {   // (ft8.grids: the newest grid of each call, 3000 at most)
        if (grids_.find(inf.from) == grids_.end()) gridOrder_.push_back(inf.from);
        grids_[inf.from] = inf.grid;
        while (grids_.size() > kMaxGrids && !gridOrder_.empty()) {
            grids_.erase(gridOrder_.front());
            gridOrder_.pop_front();
        }
    }
    d.grid = inf.grid;
    if (d.grid.empty() && !inf.from.empty()) {
        d.grid = gridOf(inf.from);
        d.gridFromEarlier = !d.grid.empty();
    }
    fillPlace(d);
    if (!myCall_.empty() && !d.callee.empty()) {
        std::string to = d.callee;
        if (to.size() >= 2 && to[0] == '<' && to[to.size() - 1] == '>') to = to.substr(1, to.size() - 2);
        d.toMe = to == myCall_;
    }
    bestSnr_ = std::max(bestSnr_, d.snr);
    list_.push_back(d);
    fresh_.push_back(d);
    if (fresh_.size() > kMaxList) fresh_.erase(fresh_.begin());
    out_ += line(d);
    dts.push_back(dt);
}

void FtxDecoder::fillPlace(FtxDecode& d) const {
    const Country c = countryOf(d.caller);
    if (c.found) {
        d.country = c.name;
        d.continent = c.continent;
    }
    double la, lo;
    if (homeSet_ && Locator::toLatLon(d.grid, la, lo)) {
        double km, deg;
        Locator::distanceBearing(homeLat_, homeLon_, la, lo, km, deg);
        d.hasDistance = true;
        d.km = static_cast<int>(std::floor(km + 0.5));
        d.bearing = static_cast<int>(std::floor(deg + 0.5)) % 360;
    }
}

// a row of the FT8 box: time, SNR, DT, frequency, message, country, distance and bearing
std::string FtxDecoder::line(const FtxDecode& d) const {
    char b[160];
    std::snprintf(b, sizeof b, "%s %3d %+4.1f %4.0f  ", d.utc.c_str(), d.snr, d.dt, d.freq);
    std::string s = b;
    std::string msg = d.text;
    std::size_t cols = msg.size();
    if (d.pass > 1) { msg += " \xE2\x97\x86"; cols += 2; }   // ◆ found by the deeper decoding
    if (d.toMe) { msg += " \xE2\x98\x85"; cols += 2; }       // ★ calling you
    s += msg;
    std::string place = d.country;
    if (d.hasDistance) {
        std::snprintf(b, sizeof b, "%d km %d\xC2\xB0", d.km, d.bearing);
        place += (place.empty() ? "" : " \xC2\xB7 ") + std::string(b);
    }
    if (!place.empty()) {
        if (cols < 24) s += std::string(24 - cols, ' ');
        s += "  " + place;
    }
    return s + "\n";
}

int FtxDecoder::decodeNow() {
    const unsigned before = total_;
    decodeWindow();
    return static_cast<int>(total_ - before);
}

void FtxDecoder::takeText(std::string& out) {
    out += out_;
    out_.clear();
}

void FtxDecoder::takeDecodes(std::vector<FtxDecode>& out) {
    out.insert(out.end(), fresh_.begin(), fresh_.end());
    fresh_.clear();
}

bool FtxDecoder::takeSlotTimings(std::vector<double>& dts) {
    if (!dtsPending_) return false;
    dts = lastDts_;
    dtsPending_ = false;
    return true;
}

std::string FtxDecoder::gridOf(const std::string& call) const {
    std::map<std::string, std::string>::const_iterator it = grids_.find(call);
    return it == grids_.end() ? std::string() : it->second;
}

Status FtxDecoder::status() const {
    Status s;
    s.mode = info_.name;
    s.locked = slots_ > 0 && lastNew_ > 0;   // (the last slot decoded something)
    s.signal = s.locked;
    s.snrDb = s.locked ? bestSnr_ : 0;
    s.quality = s.locked ? 1 : 0;
    s.characters = total_;
    // the browser's info line: "next decode in 7 s · clock corrected +1.2 s · last: 5 new (1 ◆ deeper)"
    std::string info;
    char b[96];
    if (anchorSet_) {
        const double SL = info_.slotSeconds, now = anchor_ + n_ / info_.sampleRate - offset_;
        const double left = std::ceil(SL - std::fmod(std::fmod(now - 0.4, SL) + SL, SL));
        std::snprintf(b, sizeof b, "next decode in %.0f s", left);
        info = b;
    } else {
        info = "listening";
    }
    if (std::fabs(offset_) < 0.05) info += " \xC2\xB7 clock OK";
    else {
        std::snprintf(b, sizeof b, " \xC2\xB7 clock corrected %c%.1f s", offset_ > 0 ? '+' : '-', std::fabs(offset_));
        info += b;
    }
    if (slots_) {
        std::snprintf(b, sizeof b, " \xC2\xB7 last: %u new", lastNew_);
        info += b;
        if (deep_) { std::snprintf(b, sizeof b, " (%u \xE2\x97\x86 deeper)", lastDeeper_); info += b; }
        std::snprintf(b, sizeof b, " \xC2\xB7 %.0f ms", lastMs_);   // (Addition to the JS: the decode's time)
        info += b;
    }
    if (!deep_) info += " \xC2\xB7 deeper decoding off";
    s.info = info;
    return s;
}

}  // namespace dec
