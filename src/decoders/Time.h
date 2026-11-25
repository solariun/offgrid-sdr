// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Time.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  dec::TimeDecoder: reads the shortwave time stations and says how far the
//  computer's clock is from theirs.  Ported from OFFgrid-SDR's TimeDecoder
//  class and timeFeed()/timeBest() (diddy-boy, index.html):
//
//    CHU   Canada, 3330 / 7850 / 14670 kHz.  1000 Hz second ticks (300 ms;
//          10 ms in seconds 31-39 and 51-59; none in second 29), a 0.5 s
//          minute tone (1 s at the hour), and in seconds 31-39 a Bell 103
//          FSK time code (300 baud, mark 2225 / space 2025 Hz above the
//          carrier): ten bytes, format B in second 31 (the year etc., the
//          second half inverted), format A in 32-39 ("6 ddd hh mm ss" twice).
//          The full time in about 15-35 s.
//    WWV   USA (Fort Collins), 2.5 / 5 / 10 / 15 / 20 / 25 MHz, and
//    WWVH  USA (Hawaii), on 2.5-15 MHz: 5 ms ticks (WWV 1000 Hz, WWVH
//          1200 Hz; none in seconds 29 and 59), a 0.8 s minute tone, and a
//          100 Hz subcarrier BCD code, one bit a second (pulse 170 ms = 0,
//          470 ms = 1, 770 ms = position marker, from 30 ms in).  The full
//          time at the end of a whole minute (about 1-2 minutes).  Which of
//          the two is heard is found from the ticks (setStation(Wwv) covers
//          both).
//    RWM   Russia, 4996 / 9996 / 14996 kHz: the carrier keyed, 100 ms each
//          second, 500 ms at the minute.  No time code: the second (and,
//          with the computer's clock within 30 s, the second of the minute).
//
//  Three levels of result (level()), as in the browser:
//    Ticks   the start of each second, to a few ms (any audible station);
//            only the fraction of a second (the clock assumed within 0.5 s);
//    Minute  the minute tone as well: the second within the minute (the
//            clock assumed within 30 s);
//    Full    a station's time code (CHU, WWV, WWVH): date and time.
//
//  Use:
//
//      #include "decoders/Time.h"
//      dec::TimeDecoder td;
//      dec::TimeStation st;
//      if (dec::TimeDecoder::stationForFrequency(7850e3, st)) td.setStation(st);
//      for (;;) {
//          td.setTimestamp(pcClockAtFirstSample);   // Unix seconds, UTC
//          td.process(audio, n, 48000);             // USB audio, dial 1 kHz below
//          std::string t; td.takeText(t);           // e.g. "CHU 2026-11-25 14:03:07 UTC
//                                                   //  (day 329), PC clock -0.412 s ±0.005"
//          dec::Status s = td.status();             // s.timeValid, s.clockOffset ...
//      }
//
//  Audio and dial convention: TIME listens in USB with the dial 1 kHz below
//  the station (OFFgrid-SDR's TIME_SHIFT = 1000; the displayed frequency is
//  the station's), so the carrier is a 1000 Hz tone.  It is found exactly
//  (anywhere 700-1300 Hz, by a long FFT), so a stick a few ppm off does not
//  matter; everything else is measured from it (ticks at carrier + 1000 /
//  1200 Hz, the WWV subcarrier at carrier + 100 Hz, CHU's FSK at carrier +
//  2025 / 2225 Hz: up to ~3.3 kHz of audio is needed for CHU's code).
//
//  Timing: call setTimestamp() before every process() with the computer's
//  clock (Unix seconds) at the block's first sample.  As in timeFeed(), the
//  stream's start on that clock is the earliest estimate over the last 3 s
//  (blocks only ever arrive late), smoothed; a jump of over 50 ms that
//  lasts 1 s (samples lost) restarts what was built up (timeJump()).
//
//  Signs: Status::clockOffset is station time - computer time (as
//  Decoder.h defines it: + = the computer is slow).  The browser (and
//  TimeReading::offset, and ClockCorrection in Clock.h) keep the opposite,
//  computer - station (+ = the computer is fast): the correction taken off
//  the computer's clock.  The text lines show the computer's error
//  ("PC clock -0.412 s" = 0.412 s slow).
//
//  How it fits in
//  --------------
//  Implements dec::Decoder (Kind::Time).  Depends on Decoder.h and
//  src/dsp/Fft.h.  No allocation per sample (buffers are sized when the
//  rate changes).  Clock.h keeps the correction a program applies.
//
//  Contributing
//  ------------
//  The thresholds are the browser's.  Deviations are marked "Addition to
//  the JS" in Time.cpp.  Keep both versions in step; run
//  tests/test_decoders_time.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_TIME_H
#define OFFGRID_DECODERS_TIME_H

#include "Decoder.h"
#include "../dsp/Fft.h"

#include <complex>
#include <string>
#include <vector>

namespace dec {

// The stations.  Wwv stands for both WWV and WWVH in settings (the decoder
// tells them apart by their ticks); Wwvh is what it reports for Hawaii.
enum class TimeStation { Chu, Wwv, Wwvh, Rwm };
const char* timeStationName(TimeStation st);                    // "CHU", "WWV", "WWVH", "RWM"
bool parseTimeStation(const std::string& name, TimeStation& st);  // case-insensitive; "WWVH" -> Wwv

// One listed frequency (the browser's TIME_STATIONS).
struct TimeStationFreq {
    TimeStation station;  // Chu, Wwv (WWV and/or WWVH) or Rwm
    double kHz;           // listed (carrier) frequency
    const char* label;    // e.g. "CHU, Canada", "WWV / WWVH, USA"
};
// All listed frequencies, in the browser's order.
const std::vector<TimeStationFreq>& timeStationList();

// How much of the time a reading gives (see the header comment).
enum class TimeLevel { None, Ticks, Minute, Full };

// The best reading now (port of timeBest()).
struct TimeReading {
    TimeReading() : level(TimeLevel::None), offset(0), error(0), stationTime(0), at(0) {}
    TimeLevel level;
    double offset;        // computer - station, seconds (+ = computer fast; the JS sign)
    double error;         // its uncertainty, seconds
    double stationTime;   // Full: the station's time at the second read (Unix s)
    double at;            // computer clock when it was read
    std::string station;  // "CHU", "WWV", "WWVH", "RWM"
    std::string text;     // Full: "2026-11-25 14:03:07 UTC (day 329)"
};

// UTC helpers (pure, proleptic Gregorian; no time zone, no leap seconds).
// Breaks Unix seconds into fields; doy is 1..366.
void utcFields(double unixSeconds, int& year, int& month, int& day, int& doy, int& hour, int& minute, int& second);
// Unix seconds of a year's day-of-year (1-based) and time.
double utcFromDayOfYear(int year, int doy, int hour, int minute, int second);

class TimeDecoder : public Decoder {
public:
    TimeDecoder();

    Kind kind() const override { return Kind::Time; }
    // Forgets everything, including the stream's timing.  The station is kept.
    void reset() override;
    void process(const float* audio, std::size_t n, double rate) override;
    // One line per event: ticks found, minute found (RWM: every minute),
    // each time read from a code.
    void takeText(std::string& out) override;
    Status status() const override;
    double audioCenterHz() const override { return fc_; }
    void setTimestamp(double unixSeconds) override;

    // ---- settings ----
    // The station (Chu, Wwv (finds WWVH too), Rwm; Wwvh is taken as Wwv).
    // Resets, as the browser does on retuning.
    void setStation(TimeStation st);
    bool setStation(const std::string& name);   // false if unknown
    TimeStation station() const { return kind_; }
    // The station listed within 500 Hz of a frequency (Hz): false if none.
    static bool stationForFrequency(double hz, TimeStation& st, double* listedKHz = 0);

    // ---- readings ----
    // The best reading now; `currentCorrection` (computer - true, seconds,
    // e.g. ClockCorrection::offset()) supplies the whole minutes (Minute)
    // or whole seconds (Ticks) that the reading cannot know.  A Full
    // reading counts for 10 minutes after it was made.
    TimeReading best(double currentCorrection = 0) const;
    TimeLevel level() const { return best().level; }
    // The last time read from a code (level Full or None), however old.
    TimeReading lastFull() const;
    // Where the second starts: computer - station fraction (-0.5..0.5 s);
    // false until the ticks are clear enough (port of tickOffset()).
    bool tickOffset(double& off) const;
    // The second within the minute (assuming the clock within 30 s); false
    // until a minute tone was seen (port of minuteOffset()).
    bool minuteOffset(double& off) const;
    double carrierHz() const { return fc_; }
    double carrierSnrDb() const { return fcN_ ? carrierDb_ - noiseDb_ : 0; }
    double tickQuality() const { return tickQ_; }       // > 6: ticks found, > 8 clear
    unsigned minutesSeen() const { return minuteSeen_; }
    unsigned bitsRead() const;                           // WWV: bits of the current minute
    unsigned timeJumps() const { return jumps_; }        // lost-sample restarts since reset()
    // Forgets what was built up from before a gap in the audio (called by
    // itself on a timestamp jump; public for programs that know of a gap).
    void timeJump();

private:
    struct Frame { double t, tick, tick2, extra, car; };
    struct SubFrame { double t, p; };
    struct ACand { long long k; int doy, hh, mm, ss; double minStart, secStart; };
    struct Hist { double now, est; };
    struct Result {
        Result() : valid(false), T(0), offset(0), at(0), yearKnown(false) {}
        bool valid; double T, offset, at; bool yearKnown; std::string station, text;
    };

    void setRate(double rate);
    void chunk(const float* x, std::size_t n, double t0);
    void findCarrier(const float* x, std::size_t n);
    void frame(const Frame& f);
    bool secondPhase(const double* P, double& phase, double& q, double& peak) const;
    void secondsWork();
    void minuteWork();
    void chuBursts();
    bool chuSecondBytes(double s0, int* bytes);
    void wwvBits();
    void wwvFrame(long long m0);
    void newResult(const std::string& station, double T, double secStart, bool yearKnown);
    void clearProfile();
    double frameWindow(double a, double b, bool takeMax, int& count) const;
    double subWindow(double a, double b) const;
    double sig(const Frame& f) const;
    std::string stationNow() const;

    TimeStation kind_;
    // stream timing (timeFeed)
    bool tsPending_; double ts_;
    long long streamN_; bool anchorSet_; double anchor_;
    Hist hist_[64]; int histN_;
    bool jumping_; double jumpSince_; unsigned jumps_;
    double tEnd_;                 // computer time just after the latest sample
    // rate-dependent
    double rate_; int frN_, subN_; std::size_t chunkN_;
    // carrier search
    double fc_; unsigned fcN_;
    std::vector<float> spec_; std::size_t specN_;
    std::vector<double> specAcc_, sorted_; int specCount_;
    std::vector<float> hann_;
    std::vector<std::complex<float> > fbuf_;
    dsp::detail::Fft fft_;
    double carrierDb_, noiseDb_;
    // 5 ms frames
    int grN_; double grT_; double g_[8]; bool grStarted_;
    std::vector<Frame> frames_; std::size_t frHead_, frCount_;
    double prof_[200], prof2_[200]; unsigned profN_;
    // 20 ms subcarrier frames
    int sbN_; double sbT_; double sg_[4]; bool sbStarted_;
    std::vector<SubFrame> sub_; std::size_t subHead_, subCount_;
    // CHU raw audio ring and its byte reader
    std::vector<float> ring_; long long ringN_; double anchorT_;
    bool chuSecSet_; long long chuSec_;
    std::vector<double> mc_, ms_, sc_, ss_;
    ACand aCands_[64]; int aN_;
    int year_;
    // ticks, minute
    bool havePhase_; double tickPhase_, tickQ_; bool wwvh_;
    bool minSecSet_; long long minSec_;
    double zHist_[40]; int zN_; double zTmp_[40];
    bool minuteKnown_; double minuteStart_; unsigned minuteSeen_;
    // WWV bits
    bool lastSecSet_; long long lastSec_;
    char bits_[60];
    double margin_[60];           // (Addition to the JS) how clearly each bit was read
    bool minuteAtSet_; long long minuteAt_;
    bool wwvPrevSet_; double wwvPrevT_, wwvPrevSecStart_;
    // results, text
    Result result_;
    unsigned results_;
    bool tickAnnounced_; unsigned minuteAnnounced_;
    std::string out_;
};

}  // namespace dec

#endif
