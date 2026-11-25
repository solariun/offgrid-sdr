// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/FskEncoders.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  Signal generators: mono audio (float, about -amp..amp) for CW, RTTY,
//  NAVTEX and DSC, exactly as the matching decoders expect it (USB audio).
//  For tests and for demo stations.  Deterministic: the same arguments give
//  the same samples on every platform (the noise uses its own generator).
//
//      std::vector<float> a = dec::enc::rttyAudio("CQ CQ DE TEST\n", 45.45,
//                                                  170, 1500, 48000);
//      dec::enc::addWhiteNoise(a, dec::enc::noiseRmsForSnr(0.25 * 0.25 / 2,
//                                                  15, 48000), 1);
//
//  Conventions (those of OFFgrid-SDR's demo stations, index.html):
//    CW      a keyed tone; dot = 1.2/WPM s; 1 dot between elements, 3
//            between letters, 7 between words; soft edges (12% of a dot).
//    RTTY    start bit (space), 5 ITA2 bits LSB first, 1.5 stop bits
//            (mark); starts with 30 bits of mark and a LETTERS shift; "\n"
//            is sent as CR LF.  Mark = the higher tone (reverse = swapped).
//    NAVTEX  100 baud, 170 Hz shift, 7-bit CCIR 476 MSB first, 1 = the
//            higher tone; phasing (alpha/beta alternating), then each
//            character as DX and again as RX five slots (two DX/RX pairs
//            plus one) later; the text should run "ZCZC SA23\n...NNNN\n".
//    DSC     100 baud, 170 Hz shift, 10-bit symbols (7 data bits LSB first,
//            3 check bits = number of zeros, MSB first); a 1 bit is the
//            LOWER tone (B/Y as on air); 120 bits of dot pattern, 6
//            phasing pairs (125 / 111..106), the call DX/RX interleaved,
//            40 dot bits.  The call: format twice, fields, ECC, EOS
//            (simplified as in the browser demo).
//  Default centre 1500 Hz: OFFgrid-SDR listens to RTTY, NAVTEX and DSC in
//  USB with the dial 1.5 kHz below the listed frequency.
//
//  How it fits in
//  --------------
//  Ports of OFFgrid-SDR's ita2Encode, navtexSlots, dscBits and the demo
//  generator's cwKey / FSK keying (diddy-boy, index.html).  Uses Codes.h.
//  Used by tests/test_decoders_fsk.cpp and by the TUI's demo band.
//
//  Contributing
//  ------------
//  Keep these the exact inverse of the decoders: a test decodes each one.
// ============================================================================
#ifndef OFFGRID_DECODERS_FSKENCODERS_H
#define OFFGRID_DECODERS_FSKENCODERS_H

#include <string>
#include <vector>

namespace dec {
namespace enc {

// ---- generic ------------------------------------------------------------------
// Two-tone FSK, phase continuous: levels[i] lasts 1/levelsPerSecond s; level
// 1 = centre + shift/2, 0 = centre - shift/2.
std::vector<float> fskAudio(const std::vector<int>& levels, double levelsPerSecond, double centerHz,
                            double shiftHz, double rate, double amplitude = 0.5);
// Adds Gaussian white noise of the given RMS (deterministic for a seed).
void addWhiteNoise(std::vector<float>& audio, double rms, unsigned seed);
// The white-noise RMS for a signal of mean power `signalPower` (amplitude^2/2
// for a tone) to have `snrDb` in `bandwidthHz` (default 3 kHz) at `rate`.
double noiseRmsForSnr(double signalPower, double snrDb, double rate, double bandwidthHz = 3000);
// Silence (zeros) of the given length.
std::vector<float> silence(double seconds, double rate);
// a += b (appends).
void append(std::vector<float>& a, const std::vector<float>& b);

// ---- CW ---------------------------------------------------------------------------
// Keying units (1 = key down) for a text, one per dot length, as the demo's
// cwKey (without its trailing pause).  Unknown characters are skipped.
std::vector<int> cwKeying(const std::string& text);
std::vector<float> cwAudio(const std::string& text, double wpm, double toneHz, double rate,
                           double amplitude = 0.5, double leadSeconds = 0.5, double tailSeconds = 2.0);

// ---- RTTY -------------------------------------------------------------------------
// ITA2 codes for a text (upper-cased), starting with LETTERS; " " unshifts.
std::vector<int> ita2Encode(const std::string& text);
// Levels in half bits (1 = mark): idle mark, then per code start, 5 bits, 1.5 stop.
std::vector<int> rttyHalfBits(const std::string& text, int idleBits = 30);
std::vector<float> rttyAudio(const std::string& text, double baud, double shiftHz, double centerHz, double rate,
                             bool reverse = false, double amplitude = 0.5, int idleBits = 30);

// ---- NAVTEX -------------------------------------------------------------------------
// The SITOR-B slot sequence (7-bit codes): phasingSlots of alpha/beta, the
// text DX/RX interleaved, 10 slots of phasing at the end.  The default,
// 144 slots, is the ~10 s real stations send; the browser's demo, which
// loops, uses 24.  The decoder needs about 2 s of phasing (or of an earlier
// message) to find the character boundary before "ZCZC".
std::vector<int> navtexSlots(const std::string& text, int phasingSlots = 144);
// Bits (MSB first) for slots.
std::vector<int> navtexBits(const std::vector<int>& slots);
std::vector<float> navtexAudio(const std::string& text, double rate, double centerHz = 1500,
                               double amplitude = 0.5, int phasingSlots = 144);

// ---- DSC ------------------------------------------------------------------------------
// The bits of a call (see the header comment); fields start with the format
// specifier and end with the end-of-sequence symbol (117, 122 or 127).
std::vector<int> dscBits(const std::vector<int>& fields);
// Fields of an "all ships" call: 116, category (100 routine, 108 safety,
// 110 urgency), the caller's MMSI (9 digits), 100, 126 x 4, 127.
std::vector<int> dscAllShipsFields(int category, const std::string& mmsi);
// Fields of a distress call: 112, MMSI, nature (100-112, e.g. 105 sinking),
// position 99 x 5 (unknown), time 88 88 (unknown), 109, 127.
std::vector<int> dscDistressFields(const std::string& mmsi, int nature);
// Audio for DSC bits (a 1 bit on the lower tone).
std::vector<float> dscAudio(const std::vector<int>& bits, double rate, double centerHz = 1500, double amplitude = 0.5);

}  // namespace enc
}  // namespace dec

#endif
