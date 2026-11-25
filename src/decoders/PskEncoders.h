// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/PskEncoders.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  Deterministic audio synthesis of PSK31/63/125 and Feld Hell signals, for
//  the demo band and the tests (the counterpart of Psk.h and Hell.h):
//
//      std::vector<float> a;
//      dec::synthPsk("CQ CQ de OFFGRID\n", 31.25, 48000, a);   // appends
//      dec::synthHell("OFFGRID HELL 73", 48000, a);
//
//  The output is the audio a USB receiver would produce: a tone (1000 Hz by
//  default) at the given amplitude, no noise.  Same input, same samples.
//
//  PSK: idle (48 reversals: 0 bits), the text in Varicode, each character
//  followed by "00", idle again; a 0 bit turns the phase over, the strength
//  easing through zero as a cosine over the bit (as the JS demo station and
//  real PSK31 transmitters do); half-bit raised-cosine ramps at the very
//  start and end.  Port of the JS pskBits() and the demo's "psk" station
//  (OFFgrid-SDR, diddy-boy, https://github.com/solariun/offgrid-sdr).
//
//  Hell: each character is 7 columns of 14 dots, sent bottom dot first,
//  245 dots a second, the tone keyed on for ink (0.5 ms ramps).  The JS demo
//  rasterises a canvas font; here a built-in 5x7 font is used, each row
//  doubled (14 dots), one blank column either side.  It covers A-Z (a-z are
//  sent as capitals), 0-9, space and . , / ? - ; other characters are sent
//  blank.
//
//  How it fits in
//  --------------
//  Uses Psk.h for the Varicode table and Hell.h for the Hell constants;
//  standard library only.  Not needed to decode.
//
//  Contributing
//  ------------
//  Keep the output deterministic (no random numbers, no clocks): the tests
//  compare decoded text exactly.  Strict C++11.
// ============================================================================
#ifndef OFFGRID_DECODERS_PSKENCODERS_H
#define OFFGRID_DECODERS_PSKENCODERS_H

#include <cstddef>
#include <string>
#include <vector>

namespace dec {

// The PSK bit string for a text ('0'/'1'): idleBits 0s, each character's
// Varicode + "00" ('\n' is sent as LF), idleBits 0s.  As the JS pskBits().
std::string pskBits(const std::string& text, int idleBits = 48);

// Appends the PSK audio for `text` (pskBits) at `baud` (31.25, 62.5, 125)
// and `rate` Hz.  Returns the number of samples appended.
std::size_t synthPsk(const std::string& text, double baud, double rate, std::vector<float>& out,
                     double toneHz = 1000.0, double amplitude = 0.5);
// The same, from a bit string (for custom framing).
std::size_t synthPskBits(const std::string& bits, double baud, double rate, std::vector<float>& out,
                         double toneHz = 1000.0, double amplitude = 0.5);

// The Hell glyph for a character: 7 columns, left to right; bit d of each
// is dot d counted from the BOTTOM (the order it is sent).  Returns false
// (and a blank glyph) for a character not in the font.
bool hellGlyph(char c, unsigned short columns[7]);

// Appends the Feld Hell audio for `text` at `rate` Hz.  Returns the number
// of samples appended.
std::size_t synthHell(const std::string& text, double rate, std::vector<float>& out,
                      double toneHz = 1000.0, double amplitude = 0.5);

}  // namespace dec

#endif
