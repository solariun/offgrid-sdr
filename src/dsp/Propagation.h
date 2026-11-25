// ============================================================================
//  offgrid-dsp - reusable C++11 radio signal processing (part of OFFgrid-SDR
//  TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/Propagation.h
//  Layer:  Library liboffgrid-dsp (public header)
//
//  What this file does
//  -------------------
//  "Likely bands open" (OFFgrid-SDR 2.5, diddy-boy, index.html: propSun,
//  propSunspots, propLimits, propRate, propNext, propRender): a guide to
//  which HF bands are probably open now, worked out offline from the time
//  and the place, with no internet data.  The sun's height here, now and
//  over the last two hours (the ionosphere is slow to fade after dark), the
//  season, and the sunspot cycle as predicted for the date give a rough
//  highest usable frequency (MUF) for long-distance paths, and a lowest
//  (LUF: the daytime D layer swallows the low bands).  A band well under
//  the MUF and above the LUF is likely open (green), one near either limit
//  possible (amber).  One line sums it up, with the next change (dusk or
//  dawn, UTC).  A guide, not a forecast.
//
//  How it fits in
//  --------------
//  Pure functions of (Unix time, latitude, longitude): the application
//  calls propagationGuide() about once a minute while the shortwave chips
//  are shown, and colours the chips by each band's rating.  Locator
//  (Maidenhead) to latitude/longitude is here too.  No clock is read here.
//
//  Contributing
//  ------------
//  The model's constants are the JS's (cycle 25 began December 2019, an
//  11-year cycle rising for 4.8 years ...): keep them in step.  Strict C++11.
//  Ported from OFFgrid-SDR by diddy-boy.
// ============================================================================
#ifndef OFFGRID_DSP_PROPAGATION_H
#define OFFGRID_DSP_PROPAGATION_H

#include <string>
#include <vector>

namespace dsp {

// The sun's height in degrees at Unix time t (seconds), seen from lat, lon (degrees).
double sunElevation(double unixSeconds, double latDeg, double lonDeg);
// The smoothed sunspot number predicted for the date: a guide's worth (8 to 153).
double predictedSunspots(double unixSeconds);

struct PropLimits {
    double muf;  // MHz: highest usable frequency, long paths
    double luf;  // MHz: lowest
    double sun;  // the sun's height now, degrees
};
PropLimits propagationLimits(double unixSeconds, double latDeg, double lonDeg);

enum class PropRating { Unlikely, Possible, Open };
// a band whose middle is at fMHz
PropRating propagationRating(double fMHz, const PropLimits& lim);
const char* propRatingName(PropRating r);  // "unlikely", "possible", "open"

// The next dusk or dawn (the sun crossing 0.833 degrees below the horizon)
// within a day, in 5-minute steps: true and *at (Unix s), *dusk filled.
bool nextDuskOrDawn(double unixSeconds, double latDeg, double lonDeg, double* at, bool* dusk);

// One HF band of the guide.
struct PropBand {
    std::string name;  // "20 m", "31 m"
    double loKHz, hiKHz;
    bool ham;          // amateur band (else broadcast)
    PropRating rating;
};

struct PropGuide {
    PropLimits limits;
    std::vector<PropBand> bands;  // the group's bands, rated
    std::vector<std::string> open, possible;
    bool hasNext;
    double nextAt;   // Unix s of the next dusk or dawn
    bool nextDusk;
    bool greyline;   // the sun within 5 degrees of the horizon
    std::string line;  // "Likely bands open: 20 m, 17 m; possible: 15 m. After dusk (about 18:40 UTC) ..."
};

// The guide for the amateur (ham = true) or broadcast shortwave bands, HF
// only (not CB, not 6 m and up).  fromLocator: false adds the JS's hint to
// set a locator (the place was guessed from the time zone).
PropGuide propagationGuide(double unixSeconds, double latDeg, double lonDeg, bool ham, bool fromLocator = true);

// Maidenhead locator ("IO91" or "IO91wm", as the JS's gridLL) -> the centre of the
// square, degrees.  False for an invalid locator.
bool locatorToLatLon(const std::string& locator, double* latDeg, double* lonDeg);
// Without a locator: the time zone's longitude (UTC offset in minutes east,
// /4) at 50 degrees north, as the JS's propWhere.
void placeFromUtcOffset(int minutesEastOfUtc, double* latDeg, double* lonDeg);

}  // namespace dsp

#endif
