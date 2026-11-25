// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Codes.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  The character and field tables shared by the text decoders and by the
//  signal generators (FskEncoders.h):
//
//    Morse      dot/dash string <-> character (CW).
//    ITA2       the 5-bit "Baudot" teleprinter code (RTTY): a letters and a
//               figures table; code 31 shifts to letters, 27 to figures.
//    CCIR 476   the 7-bit code of SITOR-B (NAVTEX): every valid character
//               has exactly four 1 bits, so one flipped bit shows; plus the
//               control codes (phasing alpha/beta, letters, figures, space,
//               line feed, carriage return, ...).
//    NAVTEX     the subject indicator letters (B2 of the "ZCZC B1B2B3B4"
//               header): A = navigational warning, B = meteorological ...
//    DSC        ITU-R M.493 format specifiers, categories and the nature
//               of distress, by symbol value (0-127).
//
//  All strings are UTF-8.  Tables return nullptr (or "") for codes that
//  stand for nothing.
//
//  How it fits in
//  --------------
//  Used by Cw.cpp, Rtty.cpp, Navtex.cpp and FskEncoders.cpp.  Values and
//  texts are those of OFFgrid-SDR (diddy-boy, index.html), from which the
//  decoders are ported.
//
//  Contributing
//  ------------
//  Keep the tables in step with the browser version.  Adding a Morse
//  prosign: add it to the table in Codes.cpp; the encoder only uses single
//  character entries.
// ============================================================================
#ifndef OFFGRID_DECODERS_CODES_H
#define OFFGRID_DECODERS_CODES_H

#include <string>

namespace dec {
namespace codes {

// ---- Morse ------------------------------------------------------------------
// The character for a dot/dash string such as ".-" ("A"), or nullptr.
// Prosigns come back as "<SK>", "<KA>", "<HH>".
const char* morseLookup(const std::string& dotsDashes);
// The dot/dash string for one character (upper case), or nullptr.
const char* morseFor(char c);

// ---- ITA2 (RTTY) ------------------------------------------------------------
// The character for a 5-bit code in letters or figures mode, "" for none.
// "\n" for line feed (2); carriage return (8) is "" (no character).
const char* ita2Char(int code, bool figures);
const int kIta2Letters = 31;   // shift to letters
const int kIta2Figures = 27;   // shift to figures
const int kIta2Space = 4;
const int kIta2LineFeed = 2;
const int kIta2CarriageReturn = 8;

// ---- CCIR 476 (SITOR-B, NAVTEX) --------------------------------------------
// The character for a 7-bit code; nullptr if the code is not a character
// (controls and invalid codes).  Figures: "" where the figure has no
// printable meaning (the "who are you" and bell codes).
const char* ccirChar(int code, bool figures);
// The code for a letter (figures=false) or figure (true), or -1.
int ccirCode(const std::string& utf8Char, bool figures);
// Number of 1 bits in the low 7 bits: 4 for every valid code.
int pop7(int code);
const int kCcirAlpha = 0x0F;    // phasing signal alpha (SIA)
const int kCcirBeta = 0x33;     // phasing signal beta (SIB)
const int kCcirFigures = 0x36;
const int kCcirLetters = 0x5A;
const int kCcirSpace = 0x5C;
const int kCcirRepeat = 0x66;   // RQ
const int kCcirBlank = 0x6A;
const int kCcirLineFeed = 0x6C;
const int kCcirCarriageReturn = 0x78;

// ---- NAVTEX -----------------------------------------------------------------
// "Navigational warning" etc. for a subject letter, nullptr if not defined.
const char* navtexSubject(char letter);

// ---- DSC --------------------------------------------------------------------
const char* dscFormat(int symbol);    // 102 "Area call", 112 "Distress", ...
const char* dscCategory(int symbol);  // 100 "Routine", 108 "Safety", ...
const char* dscNature(int symbol);    // 105 "sinking", ...
// A received 10-bit word, rearranged as (check << 7) | info (see Navtex.cpp):
// valid when the 3-bit check equals the number of 0 bits in the 7 info bits.
bool dscCheckOk(int word);

}  // namespace codes
}  // namespace dec

#endif
