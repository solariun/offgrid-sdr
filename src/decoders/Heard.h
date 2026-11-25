// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Heard.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  dec::HeardList: "who can this station hear, band by band?"  A port of
//  OFFgrid-SDR's "Heard" (diddy-boy, index.html: heard, HAM_BANDS,
//  bandName, heardNote, renderHeard).  Every FT8, FT4 and WSPR decode is
//  noted (time, band, mode, call, locator, SNR), apart from the decode
//  lists (so clearing those loses nothing), for 6 hours, and summed up for
//  the last 15 minutes, hour or 3 hours, per band:
//
//    - stations (each counted once, its best report), countries (from the
//      callsign, Callsign.h);
//    - the farthest station (with your locator set);
//    - the directions the band is open towards: the stations in each of
//      the 8 compass sectors, the busiest (2+ stations) first;
//    - the trend: stations in the latest half of the window against the
//      earlier half: new, rising (30% more), fading (30% fewer), steady;
//    - per country: stations and the farthest; a compass rose.
//
//      dec::HeardList heard;
//      heard.setHome("IO91");
//      heard.note(t, 14.074e6, "FT8", "K1OGS", "FN42", -12);   // per decode
//      std::vector<dec::HeardBand> s = heard.summary(now, 3600);
//      std::string table = heard.text(now, 3600);
//
//  Pure: times are given (Unix seconds, any clock, the same for note() and
//  summary()); nothing is read from a clock.
//
//  How it fits in
//  --------------
//  The program calls note() for each FT8 / FT4 decode (Ft8.h: FtxDecode's
//  caller, grid, snr) and each WSPR spot.  Depends on Callsign.h and the
//  C++11 standard library.
//
//  Contributing
//  ------------
//  Keep it in step with the browser's renderHeard(); mark deviations
//  "Addition to the JS".  Tests: tests/test_decoders_ftx.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_HEARD_H
#define OFFGRID_DECODERS_HEARD_H

#include <deque>
#include <map>
#include <string>
#include <vector>

namespace dec {

// The amateur band of a frequency ("20 m"), or "14.074 MHz" outside the
// bands (port of HAM_BANDS / bandName()).
std::string hamBandName(double hz);
// The bands' order, 2200 m first (99 for a name not in the list).
int hamBandOrder(const std::string& name);

// One decode noted.
struct HeardEntry {
    HeardEntry() : t(0), snr(0) {}
    double t;            // when (Unix seconds)
    std::string band;    // "20 m"
    std::string proto;   // "FT8", "FT4", "WSPR"
    std::string call;
    std::string grid;    // "" if the message had none
    int snr;
};

enum class HeardTrend { None, New, Rising, Fading, Steady };
// "", "new", "▲ rising", "▼ fading", "steady" (UTF-8)
const char* heardTrendText(HeardTrend t);

// One station in a band's summary: its best report.
struct HeardStation {
    HeardStation() : snr(0), t(0), hasDistance(false), km(0), bearing(0) {}
    std::string call, grid, proto;
    std::string country;     // "?" if unknown (as the browser)
    std::string continent;
    int snr;                 // the best heard
    double t;                // when (the latest of its best and earlier reports)
    bool hasDistance;        // home set and its locator known
    int km, bearing;
};

struct HeardCountry {
    HeardCountry() : stations(0), farthestKm(0), bestSnr(-99) {}
    std::string name;
    int stations;
    int farthestKm;   // 0 = no distance known
    int bestSnr;
};

// One band's summary (a row of the browser's table and its details).
struct HeardBand {
    HeardBand() : stations(0), countries(0), hasFarthest(false), trend(HeardTrend::None) {
        for (int i = 0; i < 8; ++i) sectors[i] = 0;
    }
    std::string band;
    int stations;
    int countries;
    bool hasFarthest;
    HeardStation farthest;
    int sectors[8];                    // stations per compass sector (N, NE, ... NW)
    std::vector<int> openSectors;      // the busiest sectors (2+ stations), at most 3
    std::string openText;              // "NE 4 · E 3" ("" if none)
    HeardTrend trend;
    std::vector<HeardCountry> byCountry;   // most stations first (then the farthest)
    std::vector<HeardStation> stationList; // in the order first heard
    std::string rose;                  // "N ▂2  NE ·  ..." (with home set)
};

class HeardList {
public:
    HeardList();

    // Your locator: distances, the farthest station, directions.  False
    // (and cleared) if not a locator.
    bool setHome(const std::string& grid);
    void setHomeLatLon(double lat, double lon);
    void clearHome() { homeSet_ = false; }
    bool hasHome() const { return homeSet_; }

    // A decode (port of heardNote()): hashed ("<...>") and empty calls are
    // skipped.  Entries over 6 hours older than t go, 20000 at most.  A
    // locator given is also remembered for the call (the browser's
    // ft8.grids), for its later messages without one.
    void note(double t, double dialHz, const std::string& proto, const std::string& call,
              const std::string& grid, int snr);
    // Remember a call's locator without noting a decode.
    void learnGrid(const std::string& call, const std::string& grid);

    // The summary of the last `windowSeconds` before `now`, band by band
    // (port of renderHeard()), in band order.
    std::vector<HeardBand> summary(double now, double windowSeconds) const;
    // The same as a text table (the browser's), with each band's countries
    // and compass rose when `details`.  "Nothing heard yet in this time." if
    // empty.
    std::string text(double now, double windowSeconds, bool details = false) const;
    // Different calls heard in the window, all bands.
    int stationCount(double now, double windowSeconds) const;

    const std::deque<HeardEntry>& log() const { return log_; }
    void clear();

private:
    std::string gridFor(const std::string& call) const;

    std::deque<HeardEntry> log_;
    std::map<std::string, std::string> grids_;
    std::deque<std::string> gridOrder_;
    bool homeSet_;
    double homeLat_, homeLon_;
};

}  // namespace dec

#endif
