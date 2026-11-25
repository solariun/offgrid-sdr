// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Hell.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  Hellschreiber (Feld Hell) receiver.  Feld Hell is a fax-like text mode:
//  the transmitter paints each letter as a 7-column by 14-dot grid, sending
//  the dots of a column from the bottom up, 245 dots a second (122.5 baud),
//  by keying one tone on (ink) and off (paper).  There is nothing to decode
//  into characters: the receiver draws the tone's strength as a scrolling
//  strip and the operator reads the letters off it.  So this decoder makes
//  columns, not text:
//
//      #include "decoders/Hell.h"
//      dec::HellDecoder hell;
//      hell.process(audio, n, 48000);          // USB audio, tone at 1000 Hz
//      std::vector<std::vector<float> > cols;
//      hell.takeColumns(cols);                 // ~17.5 new columns a second
//      // cols[i][0] is the TOP dot, cols[i][13] the bottom one; 0..1
//
//  Draw each column as a strip of 14 dots (receivers usually draw the strip
//  twice, one copy above the other, so lettering slanted by a small timing
//  error still reads).  takeText() gives nothing.
//
//  How it works
//  ------------
//  Mix the audio down by 1000 Hz, low-pass it (2 one-pole stages, 150 Hz),
//  average the strength over each dot (rate/245 samples, fractional), divide
//  by a slowly decaying peak (so the strip is 0..1 whatever the level), and
//  group 14 dots into a column.  The decoder does not lock onto column
//  boundaries (neither does a real Hell receiver: the operator's eye does).
//
//  Audio: the tone is expected at 1000 Hz (the USB dial 1 kHz below the
//  shown frequency, as for PSK); it has to be within ~+-100 Hz.
//
//  Ported from OFFgrid-SDR (diddy-boy, https://github.com/solariun/offgrid-sdr)
//  index.html: class HellDecoder.  Additions: NaN recovery (as the PSK
//  decoder does), columns returned top-to-bottom (the JS keeps them in time
//  order, bottom first, and flips them when drawing), and a signal detector
//  for status() (the tone's power against the noise 500 Hz either side).
//
//  How it fits in
//  --------------
//  Implements dec::Decoder (Decoder.h), Kind::Hell.  Standard library only.
//  PskEncoders.h synthesises Hell test/demo signals (with the font).
//
//  Contributing
//  ------------
//  No allocation per sample (one small vector per column, 17.5 a second).
//  Strict C++11.  Run tests/test_decoders_psk.cpp after any change.
// ============================================================================
#ifndef OFFGRID_DECODERS_HELL_H
#define OFFGRID_DECODERS_HELL_H

#include "Decoder.h"

#include <cstddef>
#include <string>
#include <vector>

namespace dec {

const double hellDotsPerSecond = 245.0;   // Feld Hell
const int hellDotsPerColumn = 14;
const int hellColumnsPerLetter = 7;
const double hellAudioHz = 1000.0;

class HellDecoder : public Decoder {
public:
    HellDecoder();

    Kind kind() const { return Kind::Hell; }
    void reset();
    void process(const float* audio, std::size_t n, double rate);
    void takeText(std::string& out) { (void)out; }
    Status status() const;
    double audioCenterHz() const { return hellAudioHz; }

    bool hasImage() const { return true; }
    // Appends the columns made since the last call (each 14 values 0..1,
    // top dot first).  At most maxPending() are kept between calls; older
    // ones are dropped.
    void takeColumns(std::vector<std::vector<float> >& columns);

    // --- extras ---
    unsigned long columnsMade() const { return n_; }   // since reset()
    static std::size_t maxPending() { return 600; }

private:
    void setRate(double rate);
    void rebuild();               // running state from scratch (columns kept)

    double rate_, per_, a_;
    double ph_;
    double lp_[4];
    double acc_, cnt_, peak_;
    float col_[14];               // the column being filled, in time order (bottom first)
    int ncol_;
    double contrast_;             // smoothed max-min of recent columns (info only)
    // Signal detector: power at the tone vs. at 500 and 1500 Hz (same filter).
    double refPh_[2], refLp_[2][4];
    double pTone_, pRef_[2];      // smoothed powers (~0.3 s)
    double ratio_;                // pTone / mean(pRef), at the last dot
    double holdLeft_;             // samples the "signal" flag still holds
    unsigned long n_;
    std::vector<std::vector<float> > pending_;
};

}  // namespace dec

#endif
