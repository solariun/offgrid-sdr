// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Clock.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  ClockCorrection (Clock.h): a port of OFFgrid-SDR's clock section
//  (clockSet, ft8Off, ft8SetOff, clockMeasured, clockAge, clockSrcText,
//  fmtAgo, and ft8Done's correction) by diddy-boy, index.html.  Times are
//  seconds here (the browser: milliseconds).
//  Additions to the JS: save() / load() (the browser keeps these fields
//  in its saved state); applyFtxTimings() returns whether it moved; the
//  source kind is kept with the source.
//
//  How it fits in
//  --------------
//  See Clock.h.
//
//  Contributing
//  ------------
//  Keep the function names parallel to the JavaScript.
// ============================================================================
#include "Clock.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace dec {

namespace {

const double kDay = 86400.0;

bool finite(double v) { return v == v && v - v == 0; }
double jsRound(double v) { return std::floor(v + 0.5); }

bool parseKind(const std::string& s, ClockSetKind& k) {
    static const ClockSetKind all[] = {ClockSetKind::Code, ClockSetKind::Hand, ClockSetKind::Minute,
                                       ClockSetKind::Ticks, ClockSetKind::Ftx, ClockSetKind::Other};
    for (std::size_t i = 0; i < sizeof all / sizeof all[0]; ++i)
        if (s == clockSetKindName(all[i])) { k = all[i]; return true; }
    return false;
}

// one line of text: no line breaks
std::string oneLine(const std::string& s) {
    std::string o(s);
    for (std::size_t i = 0; i < o.size(); ++i) if (o[i] == '\n' || o[i] == '\r') o[i] = ' ';
    return o;
}

bool toNumber(const std::string& s, double& v) {
    if (s.empty()) return false;
    char* end = 0;
    v = std::strtod(s.c_str(), &end);
    return end && *end == 0 && finite(v);
}

}  // namespace

const char* clockSetKindName(ClockSetKind k) {
    switch (k) {
        case ClockSetKind::Code: return "code";
        case ClockSetKind::Hand: return "hand";
        case ClockSetKind::Minute: return "minute";
        case ClockSetKind::Ticks: return "ticks";
        case ClockSetKind::Ftx: return "ftx";
        case ClockSetKind::Other: return "other";
    }
    return "other";
}

ClockCorrection::ClockCorrection() { clear(); }

void ClockCorrection::clear() {
    off_ = 0; offAt_ = 0; src_.clear(); srcAt_ = 0; srcKind_ = ClockSetKind::Other;
    hist_.clear(); driftKnown_ = false; drift_ = 0;
}

// The correction in use: what was last set, plus the PC clock's drift since then (once two time-signal readings a day
// or more apart have measured it)
double ClockCorrection::offsetAt(double t) const {
    return driftKnown_ && offAt_ ? off_ + drift_ * (t - offAt_) / kDay : off_;
}

void ClockCorrection::setOffset(double v, double pcNow) {
    if (!finite(v) || !finite(pcNow)) return;
    off_ = v; offAt_ = pcNow;
}

void ClockCorrection::set(double off, const std::string& src, ClockSetKind kind, double pcNow) {
    if (!finite(off) || !finite(pcNow)) return;
    setOffset(off, pcNow);
    if (kind == ClockSetKind::Code || kind == ClockSetKind::Hand) measured(off, src, kind, pcNow);
    src_ = oneLine(src); srcAt_ = pcNow; srcKind_ = kind;
}

// A full measurement of the PC clock (a station's time code, or set by hand), kept: two of them a day or more apart give
// its drift (a time code's are good to a few ms, so half a day is enough; by hand only to a second, so 3 days)
void ClockCorrection::measured(double off, const std::string& src, ClockSetKind kind, double at) {
    if (!finite(off) || !finite(at)) return;
    ClockMeasurement m;
    m.at = at; m.offset = jsRound(off * 1000) / 1000; m.source = oneLine(src); m.kind = kind;
    hist_.push_back(m);
    while (hist_.size() > 12) hist_.erase(hist_.begin());
    const ClockMeasurement* prev = 0;
    for (std::size_t i = hist_.size() - 1; i-- > 0;) {
        const ClockMeasurement& h = hist_[i];
        const double need = (kind == ClockSetKind::Hand || h.kind == ClockSetKind::Hand ? 3 : 0.5) * kDay;
        if (at - h.at >= need) { prev = &h; break; }
    }
    if (prev) {
        const double days = (at - prev->at) / kDay, d = (off - prev->offset) / days;
        if (std::fabs(d) < 60) { drift_ = driftKnown_ ? 0.5 * drift_ + 0.5 * d : d; driftKnown_ = true; }
    }
}

bool ClockCorrection::applyFtxTimings(const std::vector<double>& dtsIn, const std::string& proto, double pcNow) {
    std::vector<double> dts;
    for (std::size_t i = 0; i < dtsIn.size(); ++i) if (finite(dtsIn[i])) dts.push_back(dtsIn[i]);
    if (dts.empty() || !finite(pcNow)) return false;
    std::sort(dts.begin(), dts.end());
    const double med = dts[dts.size() >> 1];
    // (80% of the way, or half with only one or two, as FT8's timings scatter by tenths of a second; WSPR stations that
    //  agree to within half a second are taken whole: theirs is the clock to a tenth)
    const double spread = dts.back() - dts.front();
    const bool whole = proto == "WSPR" && dts.size() >= 2 && spread < 0.5;
    if (std::fabs(med) <= 0.15) return false;
    setOffset(offsetAt(pcNow) + med * (whole ? 1 : dts.size() >= 3 ? 0.8 : 0.5), pcNow);
    src_ = proto + " stations"; srcAt_ = pcNow; srcKind_ = ClockSetKind::Ftx;
    return true;
}

std::string ClockCorrection::agoText(double seconds) {
    const double m = seconds / 60;
    char buf[64];
    if (m < 1) return "just now";
    if (m < 120) std::snprintf(buf, sizeof buf, "%.0f min ago", jsRound(m));
    else if (m < 2880) std::snprintf(buf, sizeof buf, "%.0f h ago", jsRound(m / 60));
    else std::snprintf(buf, sizeof buf, "%.0f days ago", jsRound(m / 1440));
    return buf;
}

std::string ClockCorrection::sourceText(double pcNow) const {
    if (!hasSource()) return std::fabs(offsetAt(pcNow)) > 0.05 ? "corrected from FT8 stations" : "the computer's own, unchecked";
    // (clockSrcText rounds to whole minutes before choosing the wording)
    return "set from " + src_ + ", " + agoText(jsRound((pcNow - srcAt_) / 60) * 60);
}

// how old the last full measurement is, and how far out the clock might be by now
ClockAge ClockCorrection::age(double pcNow) const {
    ClockAge a;
    if (hist_.empty()) return a;
    a.known = true; a.last = hist_.back();
    a.days = (pcNow - a.last.at) / kDay;
    a.driftKnown = driftKnown_; a.drift = drift_;
    // (drift known: only what it gets wrong; else a typical PC's 1-2 s a day)
    a.estimate = driftKnown_ ? 0.2 + 0.3 * std::fabs(drift_) * a.days : 1.5 * a.days;
    a.stale = a.days >= 3 && a.estimate >= 1.5;
    return a;
}

std::string ClockCorrection::save() const {
    std::string s = "offgrid-clock 1\n";
    char buf[128];
    std::snprintf(buf, sizeof buf, "offset=%.6f\noffsetAt=%.3f\n", off_, offAt_); s += buf;
    if (driftKnown_) { std::snprintf(buf, sizeof buf, "drift=%.6f\n", drift_); s += buf; }
    if (hasSource()) {
        s += "source=" + src_ + "\n";
        std::snprintf(buf, sizeof buf, "sourceAt=%.3f\nsourceKind=%s\n", srcAt_, clockSetKindName(srcKind_)); s += buf;
    }
    for (std::size_t i = 0; i < hist_.size(); ++i) {
        std::snprintf(buf, sizeof buf, "measurement=%.3f %.3f %s ", hist_[i].at, hist_[i].offset, clockSetKindName(hist_[i].kind));
        s += buf; s += hist_[i].source; s += "\n";
    }
    return s;
}

bool ClockCorrection::load(const std::string& text) {
    std::istringstream in(text);
    std::string line;
    if (!std::getline(in, line)) return false;
    if (!line.empty() && line[line.size() - 1] == '\r') line.erase(line.size() - 1);
    if (line != "offgrid-clock 1") return false;
    ClockCorrection c;
    while (std::getline(in, line)) {
        if (!line.empty() && line[line.size() - 1] == '\r') line.erase(line.size() - 1);
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string k = line.substr(0, eq), v = line.substr(eq + 1);
        double d;
        if (k == "offset" && toNumber(v, d)) c.off_ = d;
        else if (k == "offsetAt" && toNumber(v, d)) c.offAt_ = d;
        else if (k == "drift" && toNumber(v, d)) { c.drift_ = d; c.driftKnown_ = true; }
        else if (k == "source") c.src_ = v;
        else if (k == "sourceAt" && toNumber(v, d)) c.srcAt_ = d;
        else if (k == "sourceKind") parseKind(v, c.srcKind_);
        else if (k == "measurement") {
            std::istringstream ls(v);
            std::string at, off, kind;
            ls >> at >> off >> kind;
            ClockMeasurement m;
            if (!toNumber(at, m.at) || !toNumber(off, m.offset) || !parseKind(kind, m.kind)) continue;
            std::string rest;
            std::getline(ls, rest);
            if (!rest.empty() && rest[0] == ' ') rest.erase(0, 1);
            m.source = rest;
            c.hist_.push_back(m);
            if (c.hist_.size() > 12) c.hist_.erase(c.hist_.begin());
        }
    }
    *this = c;
    return true;
}

}  // namespace dec
