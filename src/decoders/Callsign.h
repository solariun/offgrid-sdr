// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Callsign.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  Offline geography for amateur radio decodes (FT8, FT4, WSPR, APRS ...),
//  all built in, nothing looked up online.  A port of OFFgrid-SDR's
//  "FT8/FT4 message details" and "your location, offline: 620 cities"
//  sections (diddy-boy, index.html: CTY_NAMES, CTY_GRID, CTY_LONG,
//  callCountry, CITIES, latLonToGrid, gridLL, distBrg):
//
//    - dec::countryOf("EA8ABC") -> "Canary Islands" (AF): the country of
//      licence from the callsign's prefix (the ITU's allocation of prefixes
//      to countries, plus the main amateur radio areas within them:
//      Scotland, Wales, Hawaii, Alaska, the Canary Islands, Asiatic
//      Russia ...).  Portable calls (EA8/G4ABC, G4ABC/P, W1AW/4): the part
//      that is a prefix decides; /MM and /AM (at sea, in the air) have no
//      country.  Right for nearly every call; a few special-event prefixes
//      and some portable operations abroad may show the home country, or
//      nothing.
//    - dec::Locator: Maidenhead locators (4 or 6 characters, "IO91" or
//      "IO91wm") to latitude / longitude (the square's centre) and back;
//      great-circle distance (km, Earth radius 6371 km) and the bearing to
//      set off on (degrees from true north).
//    - the 620 built-in cities (every capital and the main cities of each
//      country, more in large countries): find one by name ("London" or
//      "London, UK"), or the nearest one to a position.  Picking the
//      nearest city gives a station its locator offline.
//
//      dec::Country c = dec::countryOf("G4OGR");          // "England", "EU"
//      double lat, lon;
//      dec::Locator::toLatLon("IO91", lat, lon);           // 51.5, -1.0
//      double km, deg;
//      dec::Locator::distanceBearing("IO91", "FN42", km, deg);
//      const dec::City* city = dec::findCity("Manchester");
//      std::string grid = dec::Locator::fromLatLon(city->lat, city->lon); // "IO83vl"
//
//  How it fits in
//  --------------
//  Pure functions and constant tables; standard library only.  Used by the
//  FT8 / FT4 decoders (Ft8.h) and the "Heard" summary (Heard.h); usable by
//  any decoder that reports callsigns or locators.
//
//  Contributing
//  ------------
//  The tables are generated from the browser's (CTY_NAMES, CTY_GRID,
//  CTY_LONG, CITIES); keep them in step.  Deviations are marked "Addition
//  to the JS".  Tests: tests/test_decoders_ftx.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_CALLSIGN_H
#define OFFGRID_DECODERS_CALLSIGN_H

#include <cstddef>
#include <string>

namespace dec {

// A callsign's country (port of callCountry()).
struct Country {
    Country() : found(false) {}
    bool found;             // false: no country (unknown prefix, /MM, a hashed "<...>" call)
    std::string name;       // e.g. "England", "Canary Islands", "USA", "Asiatic Russia"
    std::string continent;  // "EU", "NA", "SA", "AS", "AF", "OC", "AN"
};
Country countryOf(const std::string& callsign);
// "Europe", "North America", ... for a continent code ("" if unknown).
const char* continentName(const std::string& code);

// Maidenhead locators (static functions only).
class Locator {
public:
    // "AA00".."RR99", optionally followed by two letters "AA".."XX"
    // (either case).  "RR73" is a locator by form; callers that read FT8
    // messages exclude it themselves (see ftxInfo in Ft8.h).
    static bool isValid(const std::string& grid);
    // The square's centre (port of gridLL()); false if not a locator.
    static bool toLatLon(const std::string& grid, double& lat, double& lon);
    // The locator of a position (port of latLonToGrid()): 6 characters,
    // the last two lower case ("IO91wm"); `chars` 4 gives "IO91", 6 the
    // default.  (Addition to the JS: the 4-character choice.)
    static std::string fromLatLon(double lat, double lon, int chars = 6);
    // Great-circle distance (km) and initial bearing (0..360 degrees from
    // north) from (lat1, lon1) to (lat2, lon2) (port of distBrg(), which
    // rounds both; round them for display as it does).
    static void distanceBearing(double lat1, double lon1, double lat2, double lon2, double& km, double& deg);
    // The same between two locators' centres; false if either is not one.
    static bool distanceBearing(const std::string& from, const std::string& to, double& km, double& deg);
    // 0 = N, 1 = NE, ... 7 = NW: the compass sector of a bearing (as the
    // "Heard" summary groups them: round(deg / 45) % 8).
    static int sector(double deg);
    static const char* sectorName(int sector);  // "N", "NE", "E", "SE", "S", "SW", "W", "NW"
};

// One of the built-in cities (the browser's CITIES).
struct City {
    const char* name;     // UTF-8, e.g. "Düsseldorf"
    const char* country;  // e.g. "UK", "Germany", "USA"
    double lat, lon;      // degrees
};
// All 620, in the browser's order.
const City* cities(std::size_t& count);
// By name, case-insensitive (ASCII letters): "London, UK" (name and
// country, as the browser's list shows them) or just "London" (the first of
// that name).  Null if not found.
const City* findCity(const std::string& name);
// The nearest city to a position (null only if the table is empty);
// optionally its distance, km.
const City* nearestCity(double lat, double lon, double* km = 0);

}  // namespace dec

#endif
