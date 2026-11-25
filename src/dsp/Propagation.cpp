// ============================================================================
//  offgrid-dsp - reusable C++11 radio signal processing (part of OFFgrid-SDR
//  TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/Propagation.cpp
//  Layer:  Library liboffgrid-dsp
//
//  What this file does
//  -------------------
//  The "Likely bands open" guide (see Propagation.h): a port of the JS's
//  propSun (a low-precision solar position: mean anomaly, ecliptic
//  longitude, right ascension and declination, local hour angle from
//  sidereal time), propSunspots, propLimits, propRate, propNext and the
//  summary line of propRender0; gridLL for the locator (diddy-boy).
//
//  How it fits in
//  --------------
//  Standard library only.  Times are Unix seconds (the JS used ms).
//
//  Contributing
//  ------------
//  Keep the band lists in step with SW_BANDS in index.html (HF rows only).
// ============================================================================
#include "Propagation.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <ctime>

namespace dsp {

namespace {
const double kPi = 3.14159265358979323846;

struct BandDef {
    const char* name;
    double lo, hi;
    bool ham;
};
// SW_BANDS: the international broadcast metre bands and the HF amateur bands (kHz)
const BandDef kBands[] = {{"120 m", 2300, 2495, false}, {"90 m", 3200, 3400, false},   {"75 m", 3900, 4000, false},
                          {"60 m", 4750, 5060, false},  {"49 m", 5900, 6200, false},   {"41 m", 7200, 7450, false},
                          {"31 m", 9400, 9900, false},  {"25 m", 11600, 12100, false}, {"22 m", 13570, 13870, false},
                          {"19 m", 15100, 15800, false}, {"16 m", 17480, 17900, false}, {"15 m", 18900, 19020, false},
                          {"13 m", 21450, 21850, false}, {"11 m", 25670, 26100, false}, {"160 m", 1800, 2000, true},
                          {"80 m", 3500, 4000, true},    {"60 m", 5351.5, 5366.5, true}, {"40 m", 7000, 7300, true},
                          {"30 m", 10100, 10150, true},  {"20 m", 14000, 14350, true},  {"17 m", 18068, 18168, true},
                          {"15 m", 21000, 21450, true},  {"12 m", 24890, 24990, true},  {"10 m", 28000, 29700, true}};

std::string hhmm(double unix) {
    std::time_t tt = static_cast<std::time_t>(std::floor(unix));
    std::tm g;
#if defined(_WIN32)
    gmtime_s(&g, &tt);
#else
    gmtime_r(&tt, &g);
#endif
    char b[16];
    std::snprintf(b, sizeof b, "%02d:%02d", g.tm_hour, g.tm_min);
    return b;
}

std::string join(const std::vector<std::string>& v) {
    std::string s;
    for (std::size_t i = 0; i < v.size(); ++i) s += (i ? ", " : "") + v[i];
    return s;
}
}  // namespace

double sunElevation(double unix, double lat, double lon) {
    const double r = kPi / 180, d = unix / 86400.0 - 10957.5, g = (357.529 + 0.98560028 * d) * r,
                 q = 280.459 + 0.98564736 * d;
    const double L = (q + 1.915 * std::sin(g) + 0.020 * std::sin(2 * g)) * r, e = (23.439 - 0.00000036 * d) * r;
    const double ra = std::atan2(std::cos(e) * std::sin(L), std::cos(L)), dec = std::asin(std::sin(e) * std::sin(L));
    const double lha = ((18.697374558 + 24.06570982441908 * d) * 15 + lon) * r - ra;
    return std::asin(std::sin(lat * r) * std::sin(dec) + std::cos(lat * r) * std::cos(dec) * std::cos(lha)) / r;
}

double predictedSunspots(double unix) {
    // (cycle 25 began December 2019 and peaked late 2024; later cycles taken as alike)
    const double start = 2019.96, years = 11, rise = 4.8;
    const double y = 1970 + unix / 31557600.0;
    const double ph = std::fmod(std::fmod(y - start, years) + years, years);
    double k;
    if (ph < rise) {
        const double s = std::sin(kPi / 2 * ph / rise);
        k = s * s;
    } else {
        const double c = std::cos(kPi / 2 * (ph - rise) / (years - rise));
        k = c * c;
    }
    return 8 + 145 * k;
}

PropLimits propagationLimits(double t, double lat, double lon) {
    const double ssn = predictedSunspots(t), doy = std::fmod(t / 86400.0 + 0.5, 365.25);
    const double summer = std::cos(2 * kPi * (doy - 172) / 365.25) * (lat >= 0 ? 1 : -1);
    struct Lit {
        double operator()(double el) const { return std::min(1.0, std::max(0.0, (el + 4) / 20)); }  // (how sunlit the ionosphere is)
    } lit;
    const double H = 3600;
    const double D = std::max(lit(sunElevation(t, lat, lon)),
                              std::max(0.85 * lit(sunElevation(t - H, lat, lon)), 0.6 * lit(sunElevation(t - 2 * H, lat, lon))));
    const double polar = 1 - 0.3 * std::min(1.0, std::max(0.0, (std::fabs(lat) - 55) / 30));
    // (winter days and summer nights carry higher)
    const double day = (13 + 0.13 * ssn) * (1 - 0.12 * summer), night = (6.5 + 0.04 * ssn) * (1 + 0.15 * summer);
    const double sun = sunElevation(t, lat, lon);
    PropLimits l;
    l.muf = (night + (day - night) * D) * polar;
    l.luf = 1.5 + 7.5 * std::max(0.0, std::sin(sun * kPi / 180));
    l.sun = sun;
    return l;
}

PropRating propagationRating(double f, const PropLimits& lim) {
    if (f > lim.muf * 1.1) return PropRating::Unlikely;
    if (f > lim.muf * 0.85) return PropRating::Possible;
    if (f >= lim.luf) return PropRating::Open;
    return f * 1.8 >= lim.luf ? PropRating::Possible : PropRating::Unlikely;  // (a little under the LUF: weaker, shorter paths)
}

const char* propRatingName(PropRating r) {
    return r == PropRating::Open ? "open" : r == PropRating::Possible ? "possible" : "unlikely";
}

bool nextDuskOrDawn(double t, double lat, double lon, double* at, bool* dusk) {
    double prev = sunElevation(t, lat, lon);
    for (int k = 1; k <= 288; ++k) {
        const double tk = t + k * 300.0, e = sunElevation(tk, lat, lon);
        if ((prev > -0.833) != (e > -0.833)) {
            if (at) *at = tk;
            if (dusk) *dusk = e <= -0.833;
            return true;
        }
        prev = e;
    }
    return false;
}

PropGuide propagationGuide(double t, double lat, double lon, bool ham, bool fromLocator) {
    PropGuide g;
    g.limits = propagationLimits(t, lat, lon);
    for (std::size_t i = 0; i < sizeof kBands / sizeof kBands[0]; ++i) {
        const BandDef& b = kBands[i];
        if (b.ham != ham) continue;
        PropBand pb;
        pb.name = b.name;
        pb.loKHz = b.lo;
        pb.hiKHz = b.hi;
        pb.ham = b.ham;
        pb.rating = propagationRating((b.lo + b.hi) / 2e3, g.limits);
        g.bands.push_back(pb);
        if (pb.rating == PropRating::Open) g.open.push_back(pb.name);
        if (pb.rating == PropRating::Possible) g.possible.push_back(pb.name);
    }
    g.hasNext = nextDuskOrDawn(t, lat, lon, &g.nextAt, &g.nextDusk);
    if (!g.hasNext) {
        g.nextAt = 0;
        g.nextDusk = false;
    }
    g.greyline = std::fabs(g.limits.sun) < 5;
    std::string later;
    if (g.hasNext)
        later = g.nextDusk ? " After dusk (about " + hhmm(g.nextAt) + " UTC) the lower bands take over" +
                                 (ham ? ": 40 m, then 80 m and 160 m." : ": 49 m and 41 m.")
                           : " After dawn (about " + hhmm(g.nextAt) + " UTC) the higher bands open again" +
                                 (ham ? ": 20 m and up." : ": 19 m and up.");
    g.line = "Likely bands open: " + (g.open.empty() ? std::string("none clearly") : join(g.open)) +
             (g.possible.empty() ? std::string() : "; possible: " + join(g.possible)) + "." +
             (g.greyline ? " Greyline now: the lower bands can reach far." : "") + later +
             (fromLocator ? "" : " (Set your locator in Settings for a better guide.)");
    return g;
}

bool locatorToLatLon(const std::string& loc, double* lat, double* lon) {
    std::string g;
    for (std::size_t i = 0; i < loc.size(); ++i) g += static_cast<char>(std::toupper(static_cast<unsigned char>(loc[i])));
    if (g.size() != 4 && g.size() != 6) return false;
    if (g[0] < 'A' || g[0] > 'R' || g[1] < 'A' || g[1] > 'R' || !std::isdigit(static_cast<unsigned char>(g[2])) ||
        !std::isdigit(static_cast<unsigned char>(g[3])))
        return false;
    if (g.size() == 6 && (g[4] < 'A' || g[4] > 'X' || g[5] < 'A' || g[5] > 'X')) return false;
    double lo = (g[0] - 'A') * 20 - 180 + (g[2] - '0') * 2, la = (g[1] - 'A') * 10 - 90 + (g[3] - '0');
    if (g.size() == 6) {
        lo += (g[4] - 'A') * (2.0 / 24) + 1.0 / 24;
        la += (g[5] - 'A') * (1.0 / 24) + 0.5 / 24;
    } else {
        lo += 1;
        la += 0.5;
    }
    if (lat) *lat = la;
    if (lon) *lon = lo;
    return true;
}

void placeFromUtcOffset(int minutesEast, double* lat, double* lon) {
    if (lat) *lat = 50;
    if (lon) *lon = std::max(-180.0, std::min(180.0, minutesEast / 4.0));
}

}  // namespace dsp
