// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Rtty.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  dec::RttyDecoder: radioteletype (RTTY) to text.  Two tones, mark and
//  space, a fixed shift apart; 5-bit ITA2 ("Baudot") characters framed by a
//  start bit (space) and 1.5 stop bits (mark).  Ported from OFFgrid-SDR's
//  RttyDecoder and rttyFeed (diddy-boy, index.html).
//
//  Use alone:
//
//      #include "decoders/Rtty.h"
//      dec::RttyDecoder rtty;
//      rtty.process(audio, n, 48000);   // USB audio, the pair in 250-3000 Hz
//      std::string t; rtty.takeText(t);
//      rtty.status().mode;              // e.g. "RTTY 45.45/170"
//
//  Everything is found automatically:
//    - the tone pair: every ~0.3 s an FFT (4096 points at ~8 kHz) of the
//      audio; the two strongest peaks a standard shift apart (170, 200,
//      425, 450, 850 Hz, within 4% or 12 Hz) are space (lower) and mark
//      (higher);
//    - the speed (45.45, 50, 75 baud): the lengths of single-bit pulses
//      between level changes, the middle of the shortest common group;
//    - normal or reversed: a twin decoder reads the same audio the other
//      way round; when it is clearly ahead (its stop bits are where they
//      belong much more often, or its characters look much more like text:
//      common letters, spaces, figures rather than codes for nothing and
//      needless shifts) the two swap.  Not within 5 s of reset() nor 3 s of
//      the previous swap.  On a swap the twin's text for the same stretch
//      (read the right way) is printed on a new line: the wrong-way text
//      already taken cannot be withdrawn.
//
//  Settings (kept by reset()): setFixedShift(), setFixedBaud() (0 = auto),
//  setReverse() and setAutoReverse(false) for a fixed polarity.
//
//  How it works (per sample)
//  -------------------------
//  Two tone detectors (mix down by mark and by space, two one-pole
//  low-passes at 0.75 x baud each), discriminator d = (m - s) / (m + s)
//  with hysteresis (+-0.15) -> level 1 (mark) or 0 (space).  A 1 -> 0
//  change starts a character: half a bit later the start bit is checked,
//  then 5 data bits (least significant first) one bit apart, then the stop
//  bit; a missing stop bit discards the character (a framing error).
//
//  Changes from the browser version (marked in Rtty.cpp), for overs with
//  loud noise between them (a receiver's AGC lifts the band noise to full
//  level in the gaps): a presence test (the two tone filters must hold at
//  least ~25% of the audio's power) gates the pulse-length measurement and
//  character starts, so noise is never measured as a speed (it used to
//  read as 75 baud and stay there); the run history is cleared when the
//  pair is lost; a new speed needs two agreeing measurements (the first on
//  a signal is taken at once); characters are read on the last tones
//  whenever the pair is on the air (an over starts with mark alone, which
//  the tone search cannot see); a start bit must follow at least 0.8 bit
//  of mark; and tone updates within 30 Hz are smoothed.
//
//  How it fits in
//  --------------
//  Implements dec::Decoder.  Depends on Codes.h (ITA2) and src/dsp/Fft.h.
//  No allocation per sample.
//
//  Contributing
//  ------------
//  The thresholds are the browser's (tested there on English, a weather
//  report and a ham exchange, either polarity, strong and weak).  Keep
//  both versions in step; run tests/test_decoders_fsk.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_RTTY_H
#define OFFGRID_DECODERS_RTTY_H

#include "Decoder.h"
#include "../dsp/Fft.h"

#include <complex>
#include <string>
#include <vector>

namespace dec {

namespace detail {

// One RTTY demodulator (one polarity).  RttyDecoder runs two of these.
// Public fields: it is an implementation detail shared with RttyDecoder.
class RttyCore {
public:
    RttyCore();
    void reset();
    void process(const float* x, std::size_t n, double rate);

    // settings
    bool reverse;
    double fixedShift, fixedBaud;

    // state readable by RttyDecoder
    double mark, space, shift, baud;
    bool lock;
    double q, qs;           // text-likeness of the last few dozen / hundred characters
    double fOk, fBad;       // stop bits found / missing (decaying counts)
    bool figs;
    int st;                 // 0 waiting for a start bit, 1 start, 2 data, 3 stop
    double snrDb;           // tone pair over the median spectrum (rough)
    unsigned chars, errs;
    std::string text;       // decoded, not yet taken (capped at 4000 -> 3000)
    char lastChar;          // last character of the text so far (0 = none)

private:
    void setRate(double rate);
    void tuneUp();
    void guessBaud();
    void emit(int code);

    double rate_;
    int D_;
    double fs2_;
    int lostAt_;
    double cnt_;
    int bits_, nb_, level_;
    // oscillators (rotating phasors) and tone filters
    double pmRe_, pmIm_, psRe_, psIm_, rmRe_, rmIm_, rsRe_, rsIm_, rotMark_, rotSpace_;
    int renorm_;
    double mI_, mQ_, sI_, sQ_, mI2_, mQ2_, sI2_, sQ2_;
    // run lengths between level changes (ring of 300)
    std::vector<int> runs_;
    int runPos_, runCount_, runLen_;
    bool found_;               // a pair has been found since reset()
    bool baudKnown_;           // a speed has been measured on this signal
    double pendingBaud_;       // a speed measured once, waiting for confirmation
    double totP_, toneP_;      // audio power and the two tone filters' power (~30 ms)
    bool present_;             // the pair is on the air (toneP_ a good share of totP_)
    double levelRun_, lastRun_; // samples at the current level / the previous level's length
    // analysis buffer for tuneUp
    double decim_;
    int accN_;
    std::vector<float> buf_, win_;
    int bp_, since_;
    dsp::detail::Fft fft_;
    std::vector<std::complex<float> > fbuf_;
    std::vector<double> P_, sorted_;
    std::vector<int> peaks_;
};

}  // namespace detail

class RttyDecoder : public Decoder {
public:
    RttyDecoder();

    Kind kind() const override { return Kind::Rtty; }
    void reset() override;
    void process(const float* audio, std::size_t n, double rate) override;
    void takeText(std::string& out) override;
    Status status() const override;
    double audioCenterHz() const override;

    // ---- settings (kept by reset()) ----
    // Shift in Hz (e.g. 170, 450, 850), 0 = find it (default).
    void setFixedShift(double hz);
    double fixedShift() const { return fixedShift_; }
    // Speed in baud (45.45, 50, 75 or any), 0 = find it (default).
    void setFixedBaud(double baud);
    double fixedBaud() const { return fixedBaud_; }
    // Mark and space the other way round.  With auto-reverse on (default)
    // this is only the starting point; with it off, it is fixed.
    void setReverse(bool on);
    bool reverse() const { return main_.reverse; }
    void setAutoReverse(bool on) { autoReverse_ = on; }
    bool autoReverse() const { return autoReverse_; }

    // ---- readings ----
    double markHz() const { return main_.mark; }
    double spaceHz() const { return main_.space; }
    double baud() const { return main_.baud; }
    bool locked() const { return main_.lock; }

private:
    void chunk(const float* x, std::size_t n, double rate);

    detail::RttyCore main_, twin_;
    double fixedShift_, fixedBaud_;
    bool reverseSetting_, autoReverse_;
    double t_, swapAt_;
    std::string out_;
};

}  // namespace dec

#endif
