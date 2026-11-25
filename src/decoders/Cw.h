// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Cw.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  dec::CwDecoder: Morse code (CW) to text, from the receiver's audio (the
//  carrier heard as a beat tone).  Ported from OFFgrid-SDR's CwDecoder
//  (diddy-boy, index.html), which is tuned on real and simulated stations.
//
//  Use alone:
//
//      #include "decoders/Cw.h"
//      dec::CwDecoder cw;
//      cw.process(audio, n, 48000);     // CW audio, the tone 250-1500 Hz
//      std::string t; cw.takeText(t);   // new characters
//      cw.status().wpm;                 // learned speed
//
//  How it works
//  ------------
//   0. Tone filter (added in this port, on by default): the strongest
//      steady tone in 250-1500 Hz is found with an FFT every ~0.25 s
//      (spectra averaged); the audio is mixed down by that tone and
//      low-passed (2 one-pole stages, filterWidth()/2 each side), so only
//      a ~250 Hz slice reaches the level detector.  The browser version
//      relies on the receiver's narrow CW filter instead; setToneFilter(
//      false) gives exactly its behaviour (power of the whole audio).
//   1. Level: a running average of power (3 ms time constant), read every
//      2 ms.
//   2. Two levels: the spread of the last 2-6 s; 20th percentile = key up
//      (noise), 90th = key down (or the loudest 3% while no signal is
//      established).  No clear gap between them = no Morse = stay quiet.
//      Thresholds sit between them in dB (on at 50%, off at 40%), with a
//      debounce of a quarter of a dot (4-10 ms).
//   3. Learning: the first 10 key-downs and gaps are kept, not printed.
//      Speed = 2-means on log(mark length) (dots vs dashes); the gap unit
//      is learned from the gaps the same way.  Only when the timing fits
//      Morse (key-downs ~1 or ~3 dots, gaps ~1, 3 or 7+, steady level, both
//      dots and dashes) is everything decoded at once with that speed; else
//      the window slides on by one mark.  This rejects speech, music and
//      noise bursts, and fixes the first letters.
//   4. Decoding: marks become "." or "-" (threshold between the dot and
//      dash groups), gaps of > 2 units end a letter, > 5 units a word;
//      1.5 s (or 20 dots) of silence ends the line.  The last 12 elements
//      are re-checked continuously; if they stop fitting Morse, printing
//      stops and learning starts again.
//
//  How it fits in
//  --------------
//  Implements dec::Decoder (Decoder.h).  Depends on Codes.h (Morse table)
//  and src/dsp/Fft.h.  No allocation per sample; a sort of <= 3000 levels
//  every 20-100 ms (nth_element) and small vectors per Morse element.
//
//  Contributing
//  ------------
//  The timing constants are the browser's, tuned there; change them in both
//  places, and run tests/test_decoders_fsk.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_CW_H
#define OFFGRID_DECODERS_CW_H

#include "Decoder.h"
#include "../dsp/Fft.h"

#include <complex>
#include <string>
#include <vector>

namespace dec {

class CwDecoder : public Decoder {
public:
    CwDecoder();

    Kind kind() const override { return Kind::Cw; }
    void reset() override;
    void process(const float* audio, std::size_t n, double rate) override;
    void takeText(std::string& out) override;
    Status status() const override;
    double audioCenterHz() const override;

    // ---- settings (kept by reset()) ----
    // Tone filter in front of the level detector (default on): see the
    // header comment, step 0.
    void setToneFilter(bool on) { toneFilter_ = on; }
    bool toneFilter() const { return toneFilter_; }
    // Width of the tone filter, Hz (default 250; 100-1000).
    void setFilterWidth(double hz);
    double filterWidth() const { return filterWidth_; }
    // Fixed tone (Hz) instead of searching for it; 0 = search (default).
    void setFixedTone(double hz) { fixedTone_ = hz; }

    // ---- readings ----
    double wpm() const { return wpm_; }
    double dotMs() const { return dot_; }
    bool learning() const { return learning_; }   // timing not yet confirmed as Morse
    double toneHz() const { return toneHz_; }      // 0 until found

private:
    struct Event {
        bool mark;
        double dur;  // ms
        double cv;   // level variation inside a key-down
    };
    struct Fit {
        double marks, spaces, cv, dots;
    };

    void setRate(double rate);
    void step(double e, double dt);
    void keyStep(double e);
    double steadiness();
    void record(bool isMark, double d, double cv);
    void mark(double d, bool replay);
    bool check(bool isMark, double d, double cv);
    void space(double d);
    double learnGap();
    void learnSpeed(double d, bool snap);
    void emit();
    void newLine();
    void put(const char* s);
    Fit fit(const std::vector<Event>& ev, double dot, double sdot);
    void trackTone();

    // settings
    bool toneFilter_;
    double filterWidth_, fixedTone_;

    // rate-dependent
    double rate_;
    int stepN_;          // samples per 2 ms step
    double powA_;        // 3 ms power smoothing coefficient
    double lpA_;         // tone filter low-pass coefficient

    // tone filter / tracking
    double toneHz_, oscRe_, oscIm_, rotRe_, rotIm_, rotFor_;
    std::complex<double> lp1_, lp2_;
    int oscCount_;
    int decimD_, decimN_;
    double decimAcc_;
    std::vector<float> tbuf_;
    int tbufPos_, tbufSince_;
    std::vector<std::complex<float> > fftBuf_;
    std::vector<double> spec_;
    bool specInit_;
    dsp::detail::Fft fft_;

    // level detector
    double pw_;
    int accN_;
    double env_, hi_, lo_, t_;
    std::vector<float> hist_;
    int hn_, hcount_;
    std::vector<float> sortBuf_;
    bool present_;
    double quietFor_;
    bool signal_;
    double snrDb_;

    // keying
    bool key_, pend_;
    double pendAt_, runStart_;
    std::vector<double> mbuf_;

    // timing / decoding
    std::string sym_;
    double dot_, sdot_, markThr_, wpm_;
    std::vector<double> marks_;
    std::vector<Event> events_, recent_;
    std::vector<double> scratch_;
    bool learning_, validated_, wordSpaced_, lineBroken_;
    double lastMarkAt_;
    unsigned chars_;

    // output: the visible text so far is "everything in out_ + taken"; spaces
    // are held back until something follows them, so a line end can drop them.
    std::string out_;
    int heldSpaces_;
    char lastChar_;      // last character of the text (0 = none yet)
};

}  // namespace dec

#endif
