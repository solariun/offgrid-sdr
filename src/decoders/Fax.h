// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Fax.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  HF weather fax (radiofax, IOC 576) receiver.  The audio is one tone whose
//  pitch is the brightness, 1500 Hz black .. 2300 Hz white.  A transmission:
//
//    start tone   ~5 s: black/white alternating 300 times a second
//    phasing      ~30 s of lines: black with a short white pulse (5 % of
//                 the line) that marks where each line starts
//    image        the chart, one line every 60/LPM seconds (120 LPM: 0.5 s)
//    stop tone    ~5 s: black/white alternating 450 times a second
//
//  Lines are cut into 1809 pixels (faxWidth: IOC 576 x pi).  Use alone:
//
//      #include "decoders/Fax.h"
//      dec::FaxDecoder fax;                   // 120 LPM
//      fax.process(audio, n, 48000);          // USB audio, any rate >= 8 kHz
//      std::vector<dec::PictureLine> lines;
//      fax.takeLines(lines);                  // grey, as r = g = b
//      fax.status().info;                     // "receiving  line 312 ..."
//
//  One picture (PictureLine::picture = 1, 2, ... since reset()) per
//  transmission: it begins with the phasing after a start tone (or with
//  startNow()), and ends at the stop tone (or the next start tone, or
//  stop()).  status().pictureWidth is 1809, pictureHeight 0 (open-ended).
//
//  How it works (port of the JS FaxDecoder)
//  ----------------------------------------
//   1. Instantaneous frequency: mix 1900 Hz to 0, low-pass (3 one-pole
//      stages, 1000 Hz), the phase change per sample.  0..1 = 1500..2300 Hz.
//   2. Pixels: the samples of each pixel averaged; free-running rows of
//      1809 pixels, one per line period (rate * 60 / LPM samples).
//   3. Start/stop tones: the frequency swings at 300/450 Hz; the swing is
//      measured (correlation with 300 and 450 Hz) every 0.25 s.  Over 250 Hz
//      for 4 windows (~1 s) = the tone.  After a start tone ends, phasing.
//      A start tone in the middle of a chart ends it (its stop tone was lost
//      in a fade); the lines received during a stop/start tone are dropped.
//   4. Phasing: rows minus their median are summed; the 5 %-wide window
//      with the largest |sum| is the pulse: its start is the line start
//      (the offset every row is rotated by).  Once 2 rows no longer show
//      the pulse (or after 70 rows) the image begins.
//   5. Image: rows rotated by the offset, until the stop tone.
//
//  Additions to the JS (see Fax.cpp):
//    - slant: the pulse's position in each phasing line is measured and a
//      straight line fitted; its slope is the sample-rate difference between
//      sender and receiver, and the line length is corrected by it (the JS
//      had none; setAutoSlant(false) turns it off).  setSlantPpm() is a
//      fixed correction for a known sound-card error.
//    - the JS rotated each free-running row on its own, so the part of each
//      line right of the wrap point came from the line before; here each
//      output line is put together from two rows (one row more latency),
//      line starts kept as an absolute pixel count, so a start that slant
//      moves across a row's edge loses or doubles no line; and when the
//      phasing pulse straddles the row's edge, the image starts one row
//      earlier (the JS's test then looked at the line before);
//    - startNow() allows for the discriminator's filter delay (~1.6 px).
//    - lines are handed out ~1.25 s late, so that the lines spoiled by a
//      stop/start tone can still be dropped as the JS does.
//
//  Ported from OFFgrid-SDR (diddy-boy, https://github.com/solariun/offgrid-sdr)
//  index.html: FAX_W, class FaxDecoder (and its use by the fax box).
//
//  Audio: USB with the dial 1.9 kHz below the listed (assigned) frequency,
//  so the tones are centred on 1900 Hz (OFFgrid-SDR's FAX_SHIFT).
//
//  How it fits in
//  --------------
//  Implements dec::Decoder (Decoder.h); depends only on the C++11 standard
//  library.  PictureEncoders.h synthesises fax transmissions for it.
//
//  Contributing
//  ------------
//  Keep it a faithful port of the JS (mark deviations "Addition to the
//  JS"); no allocation per sample (one std::vector per picture line);
//  strict C++11.  Run tests/test_decoders_picture.cpp after any change.
// ============================================================================
#ifndef OFFGRID_DECODERS_FAX_H
#define OFFGRID_DECODERS_FAX_H

#include "Decoder.h"

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

namespace dec {

// Pixels per fax line (IOC 576: 576 x pi).
const int faxWidth = 1809;
// Where the fax tones are centred in the audio, Hz (black 1500, white 2300).
const double faxAudioHz = 1900.0;

class FaxDecoder : public Decoder {
public:
    enum State { Waiting, StartTone, Phasing, Receiving, Done };

    FaxDecoder();

    Kind kind() const { return Kind::Fax; }
    void reset();
    void process(const float* audio, std::size_t n, double rate);
    void takeText(std::string& out) { (void)out; }
    Status status() const;
    double audioCenterHz() const { return faxAudioHz; }

    bool hasPicture() const { return true; }
    void takeLines(std::vector<PictureLine>& lines);

    // --- settings (kept by reset()) ---
    // Lines per minute: 60, 90, 120 (default) or 240; returns false (and
    // changes nothing) for another value.  Restarts the decoder (reset()),
    // as the JS fax box does.
    bool setLpm(int lpm);
    int lpm() const { return lpm_; }
    // A fixed correction for the receiver's sample rate, parts per million
    // (positive: the sound card runs fast, i.e. more samples per line than
    // its nominal rate says).  Addition to the JS.
    void setSlantPpm(double ppm);
    double slantPpm() const { return slantPpm_; }
    // Measure the slant from the phasing lines (default on).  Addition to
    // the JS.
    void setAutoSlant(bool on) { autoSlant_ = on; }
    bool autoSlant() const { return autoSlant_; }

    // --- actions ---
    // Start receiving now, without start tone or phasing (the JS "Start"
    // button): a new picture, offset 0; line it up with shift().
    void startNow();
    // End the picture now, as a stop tone would (Addition to the JS).
    void stop();
    // Move the line start by dx pixels (the JS: click where the chart's
    // left edge should be: shift(x)).  Applies to the lines that follow.
    void shift(int dx);

    // --- state ---
    State state() const { return state_; }
    const char* stateName() const;          // "waiting", "start tone", "phasing", "receiving", "done"
    int offset() const { return offset_; }  // the line start in the free-running rows, pixels
    double measuredSlantPpm() const { return autoPpm_; }   // from the last phasing (0: none)
    unsigned pictures() const { return picture_; }

private:
    enum { kMaxPhaseRows = 72 };

    void setup(double rate);
    void applyLineLen();
    void toneStep(double d);
    void sample(double v);
    void rowDone();
    void beginPhasing();
    void endPhasing();
    void newPicture();
    void finish();
    void endPicture();
    void trimTo(int n);
    void pushRow(const float* row, long long idx);
    void emitLine(const float* a, const float* b, int off);
    double filterDelayPx() const;
    double segMean(const float* row, int start, int len) const;
    void clearFilters();

    // settings
    int lpm_;
    double slantPpm_;
    bool autoSlant_;

    // discriminator
    double rate_;
    double ncoR_, ncoI_, incR_, incI_;
    double a_;
    double zr_[3], zi_[3];
    double pr_, pi_;
    unsigned long renorm_;

    // pixels and rows
    double lineLen_, pxLen_, slantFactor_;
    double pxAcc_;
    int pxN_;
    std::vector<float> row_;
    int rowN_;
    double sampleInLine_;

    // start/stop tone detectors
    double c300R_, c300I_, w300R_, w300I_, c450R_, c450I_, w450R_, w450I_;
    double s300R_, s300I_, s450R_, s450I_;
    long win_, tw_;
    int startRun_, stopRun_;
    int startMark_, stopMark_;    // picture line counts when the tones were first heard (-1: none)

    // phasing
    State state_;
    std::vector<float> phaseAcc_;
    std::vector<float> phRows_;    // the phasing rows, kMaxPhaseRows x faxWidth (for the slant)
    float phMed_[kMaxPhaseRows];
    int phaseLines_, noPhase_;
    int pend_[2], pendN_;          // phasing-row indices that were already the image
    int offset_;
    double phaseStrength_, autoPpm_;
    std::vector<float> scratch_;

    // image.  Rows are numbered since setup() (rowIdx_: the next one); a
    // line starting at pixel p of row r starts at r * faxWidth + p.
    long long rowIdx_, phaseBase_;  // the next row's number; the first phasing row's
    double nextStart_;              // where the next line starts (Addition to the JS)
    bool manualPending_;            // startNow() before the first process(): place the start in setup()
    std::vector<float> prevRow_;
    bool havePrev_;
    long long prevIdx_;
    unsigned picture_;
    int lines_;                     // lines of the current picture (handed out + held)
    int emitted_;                   // ... of which handed out
    std::deque<PictureLine> held_;
    std::vector<PictureLine> out_;
};

}  // namespace dec

#endif
