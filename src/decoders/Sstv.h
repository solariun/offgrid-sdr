// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Sstv.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  Slow-scan television (SSTV) receiver: a picture sent as an audio tone
//  whose pitch is the brightness (1500 Hz black .. 2300 Hz white), with
//  1200 Hz sync pulses.  Every picture starts with a VIS code that says
//  which format follows.  Formats (VIS code, size, line layout):
//
//    Martin M1    44  320 x 256  sync 4.862, then G B R (146.432 ms each)
//    Martin M2    40  320 x 256  the same, 73.216 ms per colour
//    Scottie S1   60  320 x 256  G B, sync 9 ms, R (138.240 ms each)
//    Scottie S2   56  320 x 256  the same, 88.064 ms per colour
//    PD90         99  320 x 256  sync 20 ms, Y0 Cr Cb Y1 (170.240 ms each),
//                                one line carries two picture rows
//    PD120        95  640 x 496  the same, 121.600 ms each (the ISS uses PD)
//
//  (Exactly the formats of the JS decoder; Robot, Wraase, ... are not there
//  either.)  Use alone:
//
//      #include "decoders/Sstv.h"
//      dec::SstvDecoder sstv;
//      sstv.process(audio, n, 48000);        // USB/LSB audio, any rate >= 8 kHz
//      std::vector<dec::PictureLine> lines;
//      sstv.takeLines(lines);                 // rows painted since last time
//      sstv.status().mode;                    // "Martin M1", ... or ""
//
//  Each picture (each VIS code received) gets a new PictureLine::picture
//  number (1, 2, ... since reset()); rows are 0..height-1 of that mode
//  (status().pictureWidth / pictureHeight).  A picture that fades out
//  (6 sync pulses missed in a row) simply stops; the next VIS starts a new
//  number.
//
//  How it works (port of the JS SstvDecoder)
//  -----------------------------------------
//   1. Pitch: mix 1750 Hz to 0 Hz, low-pass 4 one-pole stages at 900 Hz
//      (two left the mirror image strong enough to drag the pitch towards
//      the middle), decimate to ~12 kHz, the phase step between decimated
//      samples is the frequency (clamped to 900..2600 Hz).  The pitch
//      history is kept (a ring here, a growing array in the JS).
//   2. Hunt: the pitch smoothed over 5 ms; a >200 ms run near 1900 Hz (the
//      leader) then 18 ms near 1200 Hz (the VIS start bit).  The exact
//      1900 -> 1200 Hz edge is found in the raw pitch (the steepest drop
//      within -15..+10 ms): the whole picture's timing hangs on it.
//   3. VIS: 8 bits of 30 ms after the start bit, 1100 Hz = 1, 1300 Hz = 0,
//      least significant first, the 8th even parity.  Unknown code or a
//      parity error: back to hunting.
//   4. Lines: each line is decoded once its samples have arrived.  Its sync
//      pulse is searched within +-1.5 ms of where it should be (the lowest
//      mean pitch over the sync's length); when found (mean < 1400 Hz) half
//      the error is added to a running drift: this follows a sample-rate
//      error between sender and receiver (slant).  Each colour component
//      is the mean pitch over each pixel's time span; PD's YCrCb is
//      converted to RGB (ITU-R 601 studio range, as the JS).
//
//  Ported from OFFgrid-SDR (diddy-boy, https://github.com/solariun/offgrid-sdr)
//  index.html: SSTV_MODES, sstvLayout, sstvF/sstvV, class SstvDecoder.
//
//  Audio: USB on most shortwave bands with the dial on the listed SSTV
//  frequency (14.230 MHz...), LSB on the 40/80/160 m bands (7.171, 3.845
//  MHz), narrowband FM on VHF/UHF (the ISS on 145.800 MHz): in each case
//  the tones land at 1100..2300 Hz in the audio.
//
//  How it fits in
//  --------------
//  Implements dec::Decoder (Decoder.h); depends only on the C++11 standard
//  library.  PictureEncoders.h synthesises SSTV transmissions for it.
//
//  Contributing
//  ------------
//  Keep it a faithful port of the JS (mark deviations "Addition to the
//  JS"); no allocation per sample (one std::vector per picture row);
//  strict C++11.  Run tests/test_decoders_picture.cpp after any change.
// ============================================================================
#ifndef OFFGRID_DECODERS_SSTV_H
#define OFFGRID_DECODERS_SSTV_H

#include "Decoder.h"

#include <cstddef>
#include <string>
#include <vector>

namespace dec {

// ---------------------------------------------------------------------------
// Formats
// ---------------------------------------------------------------------------
enum class SstvFamily { Martin, Scottie, Pd };

// What a line part carries.
enum SstvComponent { kSstvSync, kSstvR, kSstvG, kSstvB, kSstvY0, kSstvCr, kSstvCb, kSstvY1 };

struct SstvMode {
    int vis;              // VIS code (7 bits)
    const char* name;     // "Martin M1", ...
    SstvFamily family;
    int width, height;    // picture size, pixels
    double compMs;        // length of one colour component scan, ms
};

// One part of a line: from offMs, lenMs long (ms from the line's start).
struct SstvPart {
    double offMs, lenMs;
    SstvComponent what;
};

// A line's layout (as the JS sstvLayout()).
struct SstvLayout {
    SstvPart parts[5];
    int nParts;
    double lenMs;         // line length, ms
    double syncAtMs;      // where the line's sync pulse starts, ms
    double syncLenMs;     // its length, ms
};

int sstvModeCount();                        // 6
const SstvMode& sstvModeAt(int i);          // 0 .. sstvModeCount()-1
const SstvMode* sstvModeByVis(int vis);     // nullptr if not supported
SstvLayout sstvLayout(const SstvMode& m);
// Lines sent per picture: height, or height/2 for PD (two rows a line).
inline int sstvLinesPerPicture(const SstvMode& m) { return m.family == SstvFamily::Pd ? m.height / 2 : m.height; }

// Pixel value 0..255 <-> tone, Hz (1500 black .. 2300 white).
inline double sstvFreq(double v) { return 1500.0 + v * 800.0 / 255.0; }
inline double sstvValue(double f) {
    const double v = (f - 1500.0) * 255.0 / 800.0;
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

// ---------------------------------------------------------------------------
// The decoder (Kind::Sstv)
// ---------------------------------------------------------------------------
class SstvDecoder : public Decoder {
public:
    SstvDecoder();

    Kind kind() const { return Kind::Sstv; }
    void reset();
    void process(const float* audio, std::size_t n, double rate);
    void takeText(std::string& out) { (void)out; }
    Status status() const;
    double audioCenterHz() const { return 1900.0; }

    bool hasPicture() const { return true; }
    void takeLines(std::vector<PictureLine>& lines);

    // --- extras ---
    const SstvMode* mode() const { return mode_; }   // current/last picture's format (nullptr: none yet)
    unsigned pictures() const { return picture_; }   // pictures started since reset()
    bool receiving() const { return st_ == Image; }
    double driftSamples() const { return drift_; }   // slant followed so far (decimated samples)
    // Stop the picture being received (as when its signal fades) and hunt
    // for the next VIS code.
    void abort() { end(); }

private:
    enum State { Hunt, Vis, Image };
    enum { kRingBits = 16, kRing = 1 << kRingBits };

    void setRate(double rate);
    void clearPitch();
    void step(double f);
    void end();
    long ms(double x) const;
    float at(long long i) const;                 // pitch history, Hz
    double mean(long long s, long long e) const; // mean pitch over [s, e)
    void decodeLine();

    // pitch tracker
    double rate_, fs_, a_;
    int D_, k_;
    double lp_[8];
    double phR_, phI_, wR_, wI_;                 // 1750 Hz oscillator (Addition: a phasor, not cos/sin)
    double pzR_, pzI_;
    bool havePz_;
    double lastF_;

    // pitch history ring (the JS array F); n_ = index of the newest value
    std::vector<float> F_;
    long long n_, oldest_;

    // hunt / VIS
    State st_;
    long lead_, startRun_;
    double sm_;
    bool smValid_;
    long long visAt_;

    // picture
    const SstvMode* mode_;
    SstvLayout L_;
    double t0_, drift_;
    int miss_, line_, rows_;
    unsigned picture_;
    std::vector<float> comp_[8];                 // one line's components, 0..255
    std::vector<PictureLine> out_;
};

}  // namespace dec

#endif
