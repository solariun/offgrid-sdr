// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/PictureEncoders.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  Deterministic audio synthesis of weather fax and SSTV transmissions, and
//  text-free test pictures, for the demo band and the tests (the
//  counterpart of Fax.h and Sstv.h):
//
//      std::vector<unsigned char> chart, card;
//      dec::testChartGrey(dec::faxWidth, 200, chart);       // grey, w*h
//      dec::testCardRgb(320, 256, card);                    // rgb, w*h*3
//      std::vector<float> a;
//      dec::synthFax(chart, dec::faxWidth, 200, 48000, a);  // appends
//      dec::synthSstv(44, card, 48000, a);                  // Martin M1 (VIS 44)
//
//  The output is the audio a USB receiver would produce: one constant-
//  amplitude tone, phase-continuous, its pitch following the picture; no
//  noise.  Same input, same samples.
//
//  Fax (port of the JS demo station "fax"): start tone (black/white
//  alternating 300 times a second), phasing lines (white for the first 5 %
//  of each line, black after), the image (pixel x of a line at x/width of
//  the line period), stop tone (450 a second), black.  1500 Hz black ..
//  2300 Hz white.
//
//  SSTV (port of the JS sstvTones()): leader 1900 Hz 300 ms, break 1200 Hz
//  10 ms, leader 300 ms, VIS (start bit 1200 Hz, 7 bits LSB first + even
//  parity at 1100 Hz = 1 / 1300 Hz = 0, 30 ms each, stop bit 1200 Hz), then
//  the lines in the format's layout (Sstv.h), porches at 1500 Hz.  PD sends
//  Y of two rows and their averaged Cr/Cb per line.
//
//  The test pictures: colour bars, a grey ramp and simple shapes (SSTV);
//  a chart-like grid with curves, rings, a ramp and blocks (fax).
//
//  How it fits in
//  --------------
//  Uses Fax.h (faxWidth) and Sstv.h (the format table); standard library
//  only.  Not needed to decode.  Ported from OFFgrid-SDR (diddy-boy,
//  https://github.com/solariun/offgrid-sdr) index.html: the "Demo stations"
//  fax and SSTV stations, sstvTones().
//
//  Contributing
//  ------------
//  Keep the output deterministic (no random numbers, no clocks): the tests
//  compare decoded pictures against the input.  Strict C++11.
// ============================================================================
#ifndef OFFGRID_DECODERS_PICTUREENCODERS_H
#define OFFGRID_DECODERS_PICTUREENCODERS_H

#include "Fax.h"
#include "Sstv.h"

#include <cstddef>
#include <vector>

namespace dec {

// ---------------------------------------------------------------------------
// Fax
// ---------------------------------------------------------------------------
struct FaxTransmission {
    FaxTransmission()
        : lpm(120), startSec(5), phasingLines(20), stopSec(5), blackSec(2), amplitude(0.5) {}
    int lpm;              // lines per minute (60, 90, 120, 240)
    double startSec;      // start tone length (the JS demo: 5 s; real stations 5 s)
    int phasingLines;     // phasing lines (the JS demo: 20; real stations ~60 at 120 LPM)
    double stopSec;       // stop tone length (5 s)
    double blackSec;      // black after the stop tone (2 s)
    double amplitude;     // tone amplitude
};

// Appends a fax transmission of the grey picture (width x height bytes,
// 0 black .. 255 white, rows top first) at `rate` Hz.  Returns the number
// of samples appended.  The line period is 60/lpm s at exactly `rate`;
// to simulate a sender whose clock is off by p ppm, pass rate * (1 + p/1e6)
// here and the nominal rate to the decoder.
std::size_t synthFax(const std::vector<unsigned char>& grey, int width, int height, double rate,
                     std::vector<float>& out, const FaxTransmission& tx = FaxTransmission());

// ---------------------------------------------------------------------------
// SSTV
// ---------------------------------------------------------------------------
struct SstvTone {
    double hz, ms;
};

// The tone sequence for a picture (rgb: width*height*3 of the mode's size),
// VIS first: the JS sstvTones().  Appends to seq.
void sstvTones(const SstvMode& m, const std::vector<unsigned char>& rgb, std::vector<SstvTone>& seq);

// Appends the audio for a tone sequence (phase-continuous).  Returns the
// number of samples appended.
std::size_t synthTones(const std::vector<SstvTone>& seq, double rate, std::vector<float>& out,
                       double amplitude = 0.5);

// Appends an SSTV transmission of rgb in the format with VIS code `vis`
// (rgb must be width*height*3 of that format).  Returns the samples
// appended, 0 for an unknown VIS code or a wrong-size picture.
std::size_t synthSstv(int vis, const std::vector<unsigned char>& rgb, double rate, std::vector<float>& out,
                      double amplitude = 0.5);

// ---------------------------------------------------------------------------
// Test pictures (text-free, any size)
// ---------------------------------------------------------------------------
// Colour test card: 8 colour bars (white yellow cyan green magenta red
// blue black) in the top 45 %, a grey ramp, then a dark blue field with an
// orange disc and a white ring.  rgb gets w*h*3 bytes.
void testCardRgb(int w, int h, std::vector<unsigned char>& rgb);
// Weather-chart-like grey picture: white, a black frame, a grid, wavy
// "isobars", rings, a grey ramp band and solid blocks.  grey gets w*h bytes.
void testChartGrey(int w, int h, std::vector<unsigned char>& grey);

}  // namespace dec

#endif
