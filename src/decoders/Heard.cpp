// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Heard.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The "Heard" summary of Heard.h: a port of OFFgrid-SDR's HAM_BANDS,
//  bandName(), bandOrder(), heardNote() and renderHeard() (diddy-boy,
//  index.html).
//
//  Additions to the JS (each marked where it is):
//    - times are given, not read from the clock (pure, testable);
//    - the list keeps its own grids heard (the browser looks in the FT8
//      box's), filled by note() and learnGrid();
//    - the summary is returned as data as well as text.
//
//  How it fits in
//  --------------
//  See Heard.h.
//
//  Contributing
//  ------------
//  Keep the thresholds the browser's (2+ stations for a direction, 30% for
//  rising / fading, 6 hours, 20000 entries).
// ============================================================================
#include "Heard.h"

#include "Callsign.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>

namespace dec {

namespace {

struct HamBand { double lo, hi; const char* name; };
const HamBand kBands[] = {
    {0.1357e6, 0.1378e6, "2200 m"}, {0.472e6, 0.479e6, "630 m"}, {1.8e6, 2.0e6, "160 m"},  {3.5e6, 4.0e6, "80 m"},
    {5.25e6, 5.45e6, "60 m"},       {7.0e6, 7.3e6, "40 m"},      {10.1e6, 10.15e6, "30 m"}, {14.0e6, 14.35e6, "20 m"},
    {18.068e6, 18.168e6, "17 m"},   {21.0e6, 21.45e6, "15 m"},   {24.89e6, 24.99e6, "12 m"}, {28.0e6, 29.7e6, "10 m"},
    {50e6, 54e6, "6 m"},            {69.9e6, 70.5e6, "4 m"},     {144e6, 148e6, "2 m"}};
const int kNBands = static_cast<int>(sizeof(kBands) / sizeof(kBands[0]));
const double kKeepSeconds = 6 * 3600;   // (6 hours at most)
const std::size_t kMaxLog = 20000;
const std::size_t kMaxGrids = 3000;

bool finite(double v) { return v == v && v - v == 0; }

// 12345 -> "12,345" (as toLocaleString in English)
std::string thousands(int v) {
    char b[32];
    std::snprintf(b, sizeof b, "%d", v < 0 ? -v : v);
    std::string s = b, r;
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (i && (s.size() - i) % 3 == 0) r += ',';
        r += s[i];
    }
    return (v < 0 ? "-" : "") + r;
}

// display width of UTF-8 text (code points)
std::size_t width(const std::string& s) {
    std::size_t n = 0;
    for (std::size_t i = 0; i < s.size(); ++i)
        if ((static_cast<unsigned char>(s[i]) & 0xC0) != 0x80) ++n;
    return n;
}

std::string padRight(const std::string& s, std::size_t w) {
    const std::size_t n = width(s);
    return n >= w ? s : s + std::string(w - n, ' ');
}

std::string padLeft(const std::string& s, std::size_t w) {
    const std::size_t n = width(s);
    return n >= w ? s : std::string(w - n, ' ') + s;
}

}  // namespace

std::string hamBandName(double hz) {
    for (int i = 0; i < kNBands; ++i)
        if (hz >= kBands[i].lo && hz <= kBands[i].hi) return kBands[i].name;
    char b[32];
    std::snprintf(b, sizeof b, "%.3f MHz", hz / 1e6);
    return b;
}

int hamBandOrder(const std::string& name) {
    for (int i = 0; i < kNBands; ++i)
        if (name == kBands[i].name) return i;
    return 99;
}

const char* heardTrendText(HeardTrend t) {
    switch (t) {
        case HeardTrend::New: return "new";
        case HeardTrend::Rising: return "\xE2\x96\xB2 rising";
        case HeardTrend::Fading: return "\xE2\x96\xBC fading";
        case HeardTrend::Steady: return "steady";
        default: return "";
    }
}

HeardList::HeardList() : homeSet_(false), homeLat_(0), homeLon_(0) {}

bool HeardList::setHome(const std::string& grid) {
    homeSet_ = Locator::toLatLon(grid, homeLat_, homeLon_);
    return homeSet_;
}

void HeardList::setHomeLatLon(double lat, double lon) {
    homeSet_ = finite(lat) && finite(lon);
    homeLat_ = lat;
    homeLon_ = lon;
}

void HeardList::clear() {
    log_.clear();
    grids_.clear();
    gridOrder_.clear();
}

void HeardList::learnGrid(const std::string& call, const std::string& grid) {
    if (call.empty() || !Locator::isValid(grid)) return;
    if (grids_.find(call) == grids_.end()) gridOrder_.push_back(call);
    grids_[call] = grid;
    while (grids_.size() > kMaxGrids && !gridOrder_.empty()) {
        grids_.erase(gridOrder_.front());
        gridOrder_.pop_front();
    }
}

std::string HeardList::gridFor(const std::string& call) const {
    std::map<std::string, std::string>::const_iterator it = grids_.find(call);
    return it == grids_.end() ? std::string() : it->second;
}

void HeardList::note(double t, double dialHz, const std::string& proto, const std::string& call,
                     const std::string& grid, int snr) {
    if (call.empty() || call[0] == '<' || !finite(t)) return;
    HeardEntry e;
    e.t = t;
    e.band = hamBandName(dialHz);
    e.proto = proto;
    e.call = call;
    e.grid = Locator::isValid(grid) ? grid : std::string();
    e.snr = snr;
    log_.push_back(e);
    if (!e.grid.empty()) learnGrid(call, e.grid);   // (Addition to the JS: the list's own grids)
    const double cut = t - kKeepSeconds;
    while (!log_.empty() && log_.front().t < cut) log_.pop_front();
    while (log_.size() > kMaxLog) log_.pop_front();
}

int HeardList::stationCount(double now, double win) const {
    std::set<std::string> calls;
    for (std::size_t i = 0; i < log_.size(); ++i)
        if (now - log_[i].t <= win) calls.insert(log_[i].call);
    return static_cast<int>(calls.size());
}

std::vector<HeardBand> HeardList::summary(double now, double win) const {
    std::vector<HeardBand> out;
    // the decodes in the window, by band
    std::map<std::string, std::vector<const HeardEntry*> > byBand;
    for (std::size_t i = 0; i < log_.size(); ++i)
        if (now - log_[i].t <= win) byBand[log_[i].band].push_back(&log_[i]);
    std::vector<std::string> bands;
    for (std::map<std::string, std::vector<const HeardEntry*> >::const_iterator it = byBand.begin(); it != byBand.end(); ++it)
        bands.push_back(it->first);
    struct ByOrder {
        bool operator()(const std::string& a, const std::string& b) const {
            const int oa = hamBandOrder(a), ob = hamBandOrder(b);
            return oa != ob ? oa < ob : a < b;
        }
    };
    std::sort(bands.begin(), bands.end(), ByOrder());
    for (std::size_t bi = 0; bi < bands.size(); ++bi) {
        const std::vector<const HeardEntry*>& hs = byBand[bands[bi]];
        HeardBand B;
        B.band = bands[bi];
        // one entry per station: its best report, and where it is (its grid from this or an earlier message)
        std::vector<HeardStation> st;
        std::map<std::string, std::size_t> idx;
        for (std::size_t i = 0; i < hs.size(); ++i) {
            const HeardEntry& h = *hs[i];
            const std::string g = !h.grid.empty() ? h.grid : gridFor(h.call);
            std::map<std::string, std::size_t>::iterator it = idx.find(h.call);
            if (it == idx.end() || h.snr > st[it->second].snr) {
                HeardStation x;
                x.call = h.call;
                x.proto = h.proto;
                x.snr = h.snr;
                x.grid = !g.empty() ? g : (it != idx.end() ? st[it->second].grid : std::string());
                x.t = std::max(h.t, it != idx.end() ? st[it->second].t : 0.0);
                if (it == idx.end()) { idx[h.call] = st.size(); st.push_back(x); }
                else st[it->second] = x;
            } else if (!g.empty() && st[it->second].grid.empty()) {
                st[it->second].grid = g;
            }
        }
        std::set<std::string> countries;
        for (std::size_t i = 0; i < st.size(); ++i) {
            HeardStation& x = st[i];
            const Country c = countryOf(x.call);
            x.country = c.found ? c.name : "?";
            x.continent = c.continent;
            double la, lo;
            if (homeSet_ && Locator::toLatLon(x.grid, la, lo)) {
                double km, deg;
                Locator::distanceBearing(homeLat_, homeLon_, la, lo, km, deg);
                x.hasDistance = true;
                x.km = static_cast<int>(std::floor(km + 0.5));
                x.bearing = static_cast<int>(std::floor(deg + 0.5)) % 360;
            }
            if (x.country != "?") countries.insert(x.country);
        }
        B.stations = static_cast<int>(st.size());
        B.countries = static_cast<int>(countries.size());
        // the farthest (the first of the farthest, as a stable sort)
        for (std::size_t i = 0; i < st.size(); ++i)
            if (st[i].hasDistance && (!B.hasFarthest || st[i].km > B.farthest.km)) {
                B.hasFarthest = true;
                B.farthest = st[i];
            }
        // directions: stations per compass sector; the busiest (2+) first, 3 at most
        for (std::size_t i = 0; i < st.size(); ++i)
            if (st[i].hasDistance && st[i].km > 0) ++B.sectors[Locator::sector(st[i].bearing)];
        std::vector<int> order;
        for (int i = 0; i < 8; ++i) if (B.sectors[i] >= 2) order.push_back(i);
        struct BySect {
            const int* s;
            bool operator()(int a, int b) const { return s[a] > s[b]; }
        };
        BySect bs;
        bs.s = B.sectors;
        std::stable_sort(order.begin(), order.end(), bs);
        if (order.size() > 3) order.resize(3);
        B.openSectors = order;
        for (std::size_t i = 0; i < order.size(); ++i) {
            char b[32];
            std::snprintf(b, sizeof b, "%s%s %d", i ? " \xC2\xB7 " : "", Locator::sectorName(order[i]), B.sectors[order[i]]);
            B.openText += b;
        }
        // trend: stations heard in the latest half of the window against the earlier half
        std::set<std::string> late, early;
        for (std::size_t i = 0; i < hs.size(); ++i) {
            if (now - hs[i]->t <= win / 2) late.insert(hs[i]->call);
            else early.insert(hs[i]->call);
        }
        const double a = static_cast<double>(late.size()), b = static_cast<double>(early.size());
        B.trend = b == 0 ? (a > 0 ? HeardTrend::New : HeardTrend::None)
                         : a > b * 1.3 ? HeardTrend::Rising : a < b * 0.7 ? HeardTrend::Fading : HeardTrend::Steady;
        // per country: stations, the farthest, the best report; most stations first, then the farthest
        std::map<std::string, HeardCountry> cs;
        std::vector<std::string> csOrder;
        for (std::size_t i = 0; i < st.size(); ++i) {
            std::map<std::string, HeardCountry>::iterator it = cs.find(st[i].country);
            if (it == cs.end()) {
                csOrder.push_back(st[i].country);
                it = cs.insert(std::make_pair(st[i].country, HeardCountry())).first;
                it->second.name = st[i].country;
            }
            ++it->second.stations;
            it->second.farthestKm = std::max(it->second.farthestKm, st[i].hasDistance ? st[i].km : 0);
            it->second.bestSnr = std::max(it->second.bestSnr, st[i].snr);
        }
        for (std::size_t i = 0; i < csOrder.size(); ++i) B.byCountry.push_back(cs[csOrder[i]]);
        struct ByCount {
            bool operator()(const HeardCountry& p, const HeardCountry& q) const {
                return p.stations != q.stations ? p.stations > q.stations : p.farthestKm > q.farthestKm;
            }
        };
        std::stable_sort(B.byCountry.begin(), B.byCountry.end(), ByCount());
        // the compass rose (with home set)
        if (homeSet_) {
            static const char* const kBars[8] = {"\xE2\x96\x81", "\xE2\x96\x82", "\xE2\x96\x83", "\xE2\x96\x84",
                                                 "\xE2\x96\x85", "\xE2\x96\x86", "\xE2\x96\x87", "\xE2\x96\x88"};
            int mx = 1;
            for (int i = 0; i < 8; ++i) mx = std::max(mx, B.sectors[i]);
            for (int i = 0; i < 8; ++i) {
                char b[48];
                if (B.sectors[i]) {
                    const int k = std::min(7, static_cast<int>(std::floor(7.0 * B.sectors[i] / mx + 0.5)));
                    std::snprintf(b, sizeof b, "%s%s %s%d", i ? "  " : "", Locator::sectorName(i), kBars[k], B.sectors[i]);
                } else {
                    std::snprintf(b, sizeof b, "%s%s \xC2\xB7", i ? "  " : "", Locator::sectorName(i));
                }
                B.rose += b;
            }
        }
        B.stationList = st;
        out.push_back(B);
    }
    return out;
}

std::string HeardList::text(double now, double win, bool details) const {
    const std::vector<HeardBand> s = summary(now, win);
    if (s.empty()) return "Nothing heard yet in this time.\n";
    std::string r = padRight("Band", 8) + padLeft("Stations", 9) + padLeft("Countries", 10) + "  " +
                    padRight("Farthest", 36) + padRight("Open towards", 22) + "Trend\n";
    for (std::size_t i = 0; i < s.size(); ++i) {
        const HeardBand& B = s[i];
        char b[32];
        std::string far = "\xE2\x80\x93";   // –
        if (B.hasFarthest) far = B.farthest.call + ", " + B.farthest.country + ", " + thousands(B.farthest.km) + " km";
        std::snprintf(b, sizeof b, "%d", B.stations);
        std::string row = padRight(B.band, 8) + padLeft(b, 9);
        std::snprintf(b, sizeof b, "%d", B.countries);
        row += padLeft(b, 10) + "  " + padRight(far, 36) + padRight(B.openText.empty() ? "\xE2\x80\x93" : B.openText, 22) +
               heardTrendText(B.trend);
        r += row + "\n";
        if (details) {
            std::string line;
            for (std::size_t k = 0; k < B.byCountry.size(); ++k) {
                const HeardCountry& c = B.byCountry[k];
                std::snprintf(b, sizeof b, " %d", c.stations);
                line += (k ? " \xC2\xB7 " : "") + c.name + b;
                if (c.farthestKm) line += " (to " + thousands(c.farthestKm) + " km)";
            }
            r += "    " + line + "\n";
            if (!B.rose.empty()) r += "    " + B.rose + "\n";
        }
    }
    return r;
}

}  // namespace dec
