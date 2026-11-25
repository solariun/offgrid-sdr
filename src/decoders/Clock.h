// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Clock.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  dec::ClockCorrection: the corrected time for everything that needs the
//  right time (FT8, FT4, WSPR slots, a clock on screen), and where the
//  correction came from.  A port of OFFgrid-SDR's clock section ("the
//  clock: corrected time for everything": clockNow, clockSet, ft8Off,
//  ft8SetOff, clockMeasured, clockAge, clockSrcText, fmtAgo, and the clock
//  correction at the end of ft8Done) by diddy-boy, index.html.
//
//    - the correction: offset = computer clock - true UTC (seconds; + = the
//      computer is fast), as the browser's ft8Off().  now(pc) = pc -
//      offsetAt(pc).  (TimeDecoder's Status::clockOffset is the opposite
//      sign: pass -status.clockOffset, or TimeReading::offset as it is.)
//    - its source and when it was set ("set from CHU 7850 kHz, 5 min ago",
//      or "the computer's own, unchecked");
//    - a history of full measurements (a station's time code, or set by
//      hand), the last 12;
//    - the computer clock's drift (seconds a day), learned from two
//      measurements at least half a day apart (3 days when one was set by
//      hand: good only to a second), each new estimate averaged half and
//      half with the old one; the correction follows it between checks;
//    - save() / load(): a small text form (key=value lines) the program
//      stores where it likes; no file I/O here.
//
//  Every function takes the computer's clock as an argument (Unix seconds,
//  e.g. std::time or std::chrono::system_clock), so it is pure and testable
//  with any times.  The browser's demo-mode copy (never saved) is just a
//  second ClockCorrection object.
//
//  How it fits in
//  --------------
//  A program sets it from TimeDecoder (Time.h) results, by hand, or from
//  FT8 / FT4 / WSPR timings, and uses now() for slots and the clock.
//  Depends only on the C++11 standard library.
//
//  Contributing
//  ------------
//  Keep it in step with the browser's clock section; mark deviations
//  "Addition to the JS"; run tests/test_decoders_time.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_CLOCK_H
#define OFFGRID_DECODERS_CLOCK_H

#include <string>
#include <vector>

namespace dec {

// How a correction was made (the browser's "kind").
enum class ClockSetKind {
    Code,    // a station's time code: a full measurement, to a few ms
    Hand,    // set by hand (from a phone): a full measurement, to a second
    Minute,  // a station's minute tone: the seconds only
    Ticks,   // a station's second ticks: the fraction of a second only
    Ftx,     // FT8 / FT4 / WSPR stations' timings
    Other
};
const char* clockSetKindName(ClockSetKind k);   // "code", "hand", "minute", "ticks", "ftx", "other"

// One full measurement of the computer's clock.
struct ClockMeasurement {
    ClockMeasurement() : at(0), offset(0), kind(ClockSetKind::Code) {}
    double at;           // computer clock when measured (Unix s)
    double offset;       // computer - true, seconds (to the ms)
    std::string source;  // e.g. "CHU 7850 kHz"
    ClockSetKind kind;   // Code or Hand
};

// How old the last full measurement is and how far out the clock may be by
// now (port of clockAge()).
struct ClockAge {
    ClockAge() : known(false), days(0), driftKnown(false), drift(0), estimate(0), stale(false) {}
    bool known;              // false: never measured
    double days;             // since the last measurement
    ClockMeasurement last;
    bool driftKnown;
    double drift;            // seconds a day
    double estimate;         // how far out it may be now, seconds
    bool stale;              // >= 3 days old and maybe >= 1.5 s out
};

class ClockCorrection {
public:
    ClockCorrection();
    void clear();   // back to "the computer's own, unchecked"

    // ---- the correction in use ----
    // computer - true at computer time t: the last offset set, plus the
    // drift since then once it is known (port of ft8Off()).
    double offsetAt(double pcTime) const;
    // Corrected UTC at computer time pcNow (port of clockNow()).
    double now(double pcNow) const { return pcNow - offsetAt(pcNow); }
    // The correction as last set, without drift, and when (0 = never).
    double baseOffset() const { return off_; }
    double baseAt() const { return offAt_; }

    // ---- setting it ----
    // clockSet(): the offset (computer - true), its source text (e.g. "CHU
    // 7850 kHz") and kind; Code and Hand are also full measurements
    // (measured()).
    void set(double offset, const std::string& source, ClockSetKind kind, double pcNow);
    // ft8SetOff(): the offset alone (the source is kept).
    void setOffset(double offset, double pcNow);
    // clockMeasured(): records a full measurement and learns the drift.
    void measured(double offset, const std::string& source, ClockSetKind kind, double at);
    // The clock correction from FT8 / FT4 / WSPR decodes (the end of
    // ft8Done()): `dts` are the decodes' time offsets in their slots
    // (seconds, + = late), `proto` "FT8", "FT4" or "WSPR".  When their median
    // is over 0.15 s the correction moves by 80% of it (50% with fewer than
    // 3 decodes; all of it for 2+ WSPR stations within 0.5 s of each other)
    // and the source becomes "<proto> stations".  True if it moved.
    bool applyFtxTimings(const std::vector<double>& dts, const std::string& proto, double pcNow);

    // ---- where it came from ----
    bool hasSource() const { return srcAt_ > 0; }
    const std::string& source() const { return src_; }
    double sourceAt() const { return srcAt_; }
    // "set from CHU 7850 kHz, 5 min ago"; without a source "corrected from
    // FT8 stations" (if corrected by more than 0.05 s) or "the computer's
    // own, unchecked" (port of clockSrcText()).
    std::string sourceText(double pcNow) const;
    // "just now", "12 min ago", "5 h ago", "3 days ago" (port of fmtAgo()).
    static std::string agoText(double seconds);

    // ---- measurements and drift ----
    const std::vector<ClockMeasurement>& history() const { return hist_; }
    bool driftKnown() const { return driftKnown_; }
    double driftPerDay() const { return drift_; }   // seconds a day (+ = gains)
    ClockAge age(double pcNow) const;

    // ---- storing ----
    // A text form: "offgrid-clock 1" then key=value lines (offset, offsetAt,
    // drift, source, sourceAt, and one "measurement=at offset kind source"
    // line each).  Numbers are written with '.' decimals (C locale).
    std::string save() const;
    // Reads save()'s text; unknown keys are ignored.  False (and nothing
    // changed) if it is not that form.
    bool load(const std::string& text);

private:
    double off_, offAt_;
    std::string src_;
    double srcAt_;
    ClockSetKind srcKind_;
    std::vector<ClockMeasurement> hist_;
    bool driftKnown_;
    double drift_;
};

}  // namespace dec

#endif
