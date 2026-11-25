// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/PskEncoders.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  PSK and Feld Hell signal synthesis (see PskEncoders.h).  The PSK framing
//  and shaping follow OFFgrid-SDR's JS pskBits() and demo "psk" station
//  (diddy-boy, https://github.com/solariun/offgrid-sdr); the Hell font is a
//  classic 5x7 matrix font, doubled vertically.
//
//  How it fits in
//  --------------
//  Implements PskEncoders.h; uses pskVaricode() from Psk.h.
//
//  Contributing
//  ------------
//  Deterministic output only.  To add a Hell character, add a row to kFont
//  (7 rows top to bottom, 5 bits each, bit 4 = leftmost dot).
// ============================================================================
#include "PskEncoders.h"

#include "Hell.h"
#include "Psk.h"

#include <cmath>

namespace dec {

namespace {

const double kPi = 3.141592653589793;

// 5x7 font: 7 rows (top first), bit 4 = left column.
struct Glyph {
    char c;
    unsigned char rows[7];
};
const Glyph kFont[] = {
    {'A', {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}}, {'B', {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}},
    {'C', {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}}, {'D', {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E}},
    {'E', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}}, {'F', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}},
    {'G', {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F}}, {'H', {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
    {'I', {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}}, {'J', {0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C}},
    {'K', {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}}, {'L', {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}},
    {'M', {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}}, {'N', {0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11}},
    {'O', {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}}, {'P', {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}},
    {'Q', {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}}, {'R', {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}},
    {'S', {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}}, {'T', {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
    {'U', {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}}, {'V', {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04}},
    {'W', {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A}}, {'X', {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}},
    {'Y', {0x11, 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04}}, {'Z', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}},
    {'0', {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}}, {'1', {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}},
    {'2', {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}}, {'3', {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E}},
    {'4', {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}}, {'5', {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}},
    {'6', {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}}, {'7', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}},
    {'8', {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}}, {'9', {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}},
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, {'.', {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}},
    {',', {0x00, 0x00, 0x00, 0x00, 0x0C, 0x04, 0x08}}, {'/', {0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x00}},
    {'?', {0x0E, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04}}, {'-', {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}},
};

}  // namespace

std::string pskBits(const std::string& text, int idleBits) {
    std::string b(idleBits > 0 ? static_cast<std::size_t>(idleBits) : 0, '0');
    for (std::size_t i = 0; i < text.size(); ++i) {
        const int c = static_cast<unsigned char>(text[i]);
        const char* v = pskVaricode(c);   // ('\n' is 10: LF)
        if (v) b += v;
        b += "00";
    }
    b.append(idleBits > 0 ? static_cast<std::size_t>(idleBits) : 0, '0');
    return b;
}

std::size_t synthPskBits(const std::string& bits, double baud, double rate, std::vector<float>& out,
                         double toneHz, double amplitude) {
    if (bits.empty() || !(baud > 0) || !(rate > 0)) return 0;
    // sign[k]: the carrier's phase (+1/-1) at the start of bit k; a 0 turns it over.
    std::vector<signed char> sign(bits.size());
    signed char sg = 1;
    for (std::size_t k = 0; k < bits.size(); ++k) {
        sign[k] = sg;
        if (bits[k] == '0') sg = static_cast<signed char>(-sg);
    }
    const double total = bits.size() / baud;
    const std::size_t n = static_cast<std::size_t>(std::floor(total * rate));
    const double ramp = 0.5 / baud;   // half a bit up and down
    const std::size_t start = out.size();
    out.resize(start + n);
    for (std::size_t i = 0; i < n; ++i) {
        const double t = i / rate;
        std::size_t k = static_cast<std::size_t>(std::floor(t * baud));
        if (k >= bits.size()) k = bits.size() - 1;
        const double u = t * baud - k;
        double a = sign[k] * (bits[k] == '0' ? std::cos(kPi * u) : 1.0);
        if (t < ramp) a *= 0.5 - 0.5 * std::cos(kPi * t / ramp);
        else if (total - t < ramp) a *= 0.5 - 0.5 * std::cos(kPi * (total - t) / ramp);
        const double ph = 2 * kPi * toneHz * t;
        out[start + i] = static_cast<float>(amplitude * a * std::cos(ph));
    }
    return n;
}

std::size_t synthPsk(const std::string& text, double baud, double rate, std::vector<float>& out,
                     double toneHz, double amplitude) {
    return synthPskBits(pskBits(text), baud, rate, out, toneHz, amplitude);
}

bool hellGlyph(char c, unsigned short columns[7]) {
    for (int x = 0; x < 7; ++x) columns[x] = 0;
    if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
    for (std::size_t g = 0; g < sizeof kFont / sizeof kFont[0]; ++g) {
        if (kFont[g].c != c) continue;
        // columns 1..5 hold the font's 5 columns; 0 and 6 are spacing
        for (int fx = 0; fx < 5; ++fx) {
            unsigned short col = 0;
            for (int row = 0; row < 7; ++row) {
                if (!(kFont[g].rows[row] >> (4 - fx) & 1)) continue;
                const int d = (6 - row) * 2;   // dots from the bottom, each row twice
                col = static_cast<unsigned short>(col | (3u << d));
            }
            columns[fx + 1] = col;
        }
        return true;
    }
    return false;
}

std::size_t synthHell(const std::string& text, double rate, std::vector<float>& out, double toneHz,
                      double amplitude) {
    if (text.empty() || !(rate > 0)) return 0;
    const std::size_t dots = text.size() * hellColumnsPerLetter * hellDotsPerColumn;
    const std::size_t n = static_cast<std::size_t>(std::floor(dots / hellDotsPerSecond * rate));
    const double slew = 1.0 / (0.0005 * rate);   // 0.5 ms on/off ramps
    const std::size_t start = out.size();
    out.resize(start + n);
    double env = 0;
    std::size_t lastChar = static_cast<std::size_t>(-1);
    unsigned short cols[7] = {0, 0, 0, 0, 0, 0, 0};
    for (std::size_t i = 0; i < n; ++i) {
        const double t = i / rate;
        std::size_t dot = static_cast<std::size_t>(std::floor(t * hellDotsPerSecond));
        if (dot >= dots) dot = dots - 1;
        const std::size_t ch = dot / (hellColumnsPerLetter * hellDotsPerColumn);
        if (ch != lastChar) { hellGlyph(text[ch], cols); lastChar = ch; }
        const std::size_t inChar = dot % (hellColumnsPerLetter * hellDotsPerColumn);
        const int x = static_cast<int>(inChar / hellDotsPerColumn), d = static_cast<int>(inChar % hellDotsPerColumn);
        const double target = (cols[x] >> d & 1) ? 1.0 : 0.0;
        if (env < target) env = env + slew < target ? env + slew : target;
        else if (env > target) env = env - slew > target ? env - slew : target;
        out[start + i] = static_cast<float>(amplitude * env * std::cos(2 * kPi * toneHz * t));
    }
    return n;
}

}  // namespace dec
