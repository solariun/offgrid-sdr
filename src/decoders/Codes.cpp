// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Codes.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The tables behind Codes.h: Morse, ITA2, CCIR 476, NAVTEX subjects and
//  the DSC format / category / nature texts.  Plain constant arrays and
//  small linear searches (the tables are tiny; nothing here is per sample).
//
//  How it fits in
//  --------------
//  Shared by every FSK/CW decoder and by FskEncoders.cpp.  Copied from
//  OFFgrid-SDR (diddy-boy) index.html: CW_CODE, ITA2_LTRS/ITA2_FIGS,
//  CCIR_LTRS/CCIR_FIGS/CC, NAVTEX_SUBJECT, DSC_FORMAT/DSC_CAT/DSC_NATURE.
//
//  Contributing
//  ------------
//  If a table changes in the browser version, change it here too, and run
//  tests/test_decoders_fsk.cpp (the encoders use the same tables, so a
//  one-sided change shows up as a decode failure).
// ============================================================================
#include "Codes.h"

#include <cstring>

namespace dec {
namespace codes {

namespace {

struct MorseEntry {
    const char* code;
    const char* text;
};

const MorseEntry kMorse[] = {
    {".-", "A"}, {"-...", "B"}, {"-.-.", "C"}, {"-..", "D"}, {".", "E"}, {"..-.", "F"}, {"--.", "G"},
    {"....", "H"}, {"..", "I"}, {".---", "J"}, {"-.-", "K"}, {".-..", "L"}, {"--", "M"}, {"-.", "N"},
    {"---", "O"}, {".--.", "P"}, {"--.-", "Q"}, {".-.", "R"}, {"...", "S"}, {"-", "T"}, {"..-", "U"},
    {"...-", "V"}, {".--", "W"}, {"-..-", "X"}, {"-.--", "Y"}, {"--..", "Z"},
    {"-----", "0"}, {".----", "1"}, {"..---", "2"}, {"...--", "3"}, {"....-", "4"}, {".....", "5"},
    {"-....", "6"}, {"--...", "7"}, {"---..", "8"}, {"----.", "9"},
    {".-.-.-", "."}, {"--..--", ","}, {"..--..", "?"}, {".----.", "'"}, {"-.-.--", "!"}, {"-..-.", "/"},
    {"-.--.", "("}, {"-.--.-", ")"}, {".-...", "&"}, {"---...", ":"}, {"-.-.-.", ";"}, {"-...-", "="},
    {".-.-.", "+"}, {"-....-", "-"}, {"..--.-", "_"}, {".-..-.", "\""}, {"...-..-", "$"}, {".--.-.", "@"},
    {"...-.-", "<SK>"}, {"-.-.-", "<KA>"}, {"........", "<HH>"},
};
const int kMorseN = sizeof(kMorse) / sizeof(kMorse[0]);

// ITA2, indexed by the 5-bit code ("" = no character).
const char* const kItaLetters[32] = {"", "E", "\n", "A", " ", "S", "I", "U", "", "D", "R", "J", "N", "F", "C", "K",
                                     "T", "Z", "L", "W", "H", "Y", "P", "Q", "O", "B", "G", "", "M", "X", "V", ""};
const char* const kItaFigures[32] = {"", "3", "\n", "-", " ", "'", "8", "7", "", "", "4", "", ",", "", ":", "(",
                                     "5", "+", ")", "2", "", "6", "0", "1", "9", "?", "", "", ".", "/", "=", ""};

// CCIR 476 characters: code, letter, figure.
struct CcirEntry {
    int code;
    const char* letter;
    const char* figure;
};
const CcirEntry kCcir[] = {
    {0x17, "J", "\x07"}, {0x1B, "F", "!"}, {0x1D, "C", ":"}, {0x1E, "K", "("}, {0x27, "W", "2"},
    {0x2B, "Y", "6"},    {0x2D, "P", "0"}, {0x2E, "Q", "1"}, {0x35, "G", "&"}, {0x39, "M", "."},
    {0x3A, "X", "/"},    {0x3C, "V", "="}, {0x47, "A", "-"}, {0x4B, "S", "'"}, {0x4D, "I", "8"},
    {0x4E, "U", "7"},    {0x53, "D", ""},  {0x55, "R", "4"}, {0x56, "E", "3"}, {0x59, "N", ","},
    {0x63, "Z", "+"},    {0x65, "L", ")"}, {0x69, "H", "\xC2\xA3"}, {0x71, "O", "9"}, {0x72, "B", "?"},
    {0x74, "T", "5"},
};
const int kCcirN = sizeof(kCcir) / sizeof(kCcir[0]);

}  // namespace

const char* morseLookup(const std::string& s) {
    for (int i = 0; i < kMorseN; ++i)
        if (s == kMorse[i].code) return kMorse[i].text;
    return nullptr;
}

const char* morseFor(char c) {
    for (int i = 0; i < kMorseN; ++i)
        if (kMorse[i].text[0] == c && kMorse[i].text[1] == 0) return kMorse[i].code;
    return nullptr;
}

const char* ita2Char(int code, bool figures) {
    if (code < 0 || code > 31) return "";
    return figures ? kItaFigures[code] : kItaLetters[code];
}

const char* ccirChar(int code, bool figures) {
    for (int i = 0; i < kCcirN; ++i)
        if (kCcir[i].code == code) return figures ? kCcir[i].figure : kCcir[i].letter;
    return nullptr;
}

int ccirCode(const std::string& ch, bool figures) {
    for (int i = 0; i < kCcirN; ++i) {
        const char* t = figures ? kCcir[i].figure : kCcir[i].letter;
        if (*t && ch == t) return kCcir[i].code;
    }
    return -1;
}

int pop7(int code) {
    int n = 0;
    for (int i = 0; i < 7; ++i) n += (code >> i) & 1;
    return n;
}

const char* navtexSubject(char c) {
    switch (c) {
    case 'A': return "Navigational warning";
    case 'B': return "Meteorological warning";
    case 'C': return "Ice report";
    case 'D': return "Search and rescue / piracy";
    case 'E': return "Weather forecast";
    case 'F': return "Pilot service";
    case 'G': return "AIS";
    case 'H': return "LORAN";
    case 'J': return "Satellite navigation";
    case 'K': return "Other navigation aids";
    case 'L': return "Navigational warning";
    case 'T': return "Test";
    case 'V': case 'W': case 'X': case 'Y': return "Special service";
    case 'Z': return "No messages on hand";
    default: return nullptr;
    }
}

const char* dscFormat(int v) {
    switch (v) {
    case 102: return "Area call";
    case 112: return "Distress";
    case 114: return "Group call";
    case 116: return "All ships";
    case 120: case 123: return "Individual call";
    default: return nullptr;
    }
}

const char* dscCategory(int v) {
    switch (v) {
    case 100: return "Routine";
    case 108: return "Safety";
    case 110: return "Urgency";
    case 112: return "Distress";
    default: return nullptr;
    }
}

const char* dscNature(int v) {
    switch (v) {
    case 100: return "fire, explosion";
    case 101: return "flooding";
    case 102: return "collision";
    case 103: return "grounding";
    case 104: return "listing, in danger of capsizing";
    case 105: return "sinking";
    case 106: return "disabled and adrift";
    case 107: return "undesignated distress";
    case 108: return "abandoning ship";
    case 109: return "piracy / armed attack";
    case 110: return "man overboard";
    case 112: return "EPIRB emission";
    default: return nullptr;
    }
}

bool dscCheckOk(int w) {
    const int info = w & 0x7F;
    int zeros = 0;
    for (int i = 0; i < 7; ++i) zeros += ((info >> i) & 1) ? 0 : 1;
    return (w >> 7) == zeros;
}

}  // namespace codes
}  // namespace dec
