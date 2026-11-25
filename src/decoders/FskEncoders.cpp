// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/FskEncoders.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The generators declared in FskEncoders.h.  Bit sequences are built as in
//  OFFgrid-SDR (diddy-boy, index.html: cwKey, ita2Encode, navtexSlots,
//  dscBits and the demo's FSK keying "lv" arrays); audio is a phase-
//  continuous oscillator.  The noise is a xorshift generator with
//  Box-Muller, so results are identical on every platform and library
//  (std::normal_distribution is not).
//
//  How it fits in
//  --------------
//  See FskEncoders.h.
//
//  Contributing
//  ------------
//  Any change to a format must be matched in the decoder; the tests decode
//  every generator's output.
// ============================================================================
#include "FskEncoders.h"
#include "Codes.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>

namespace dec {
namespace enc {

namespace {

const double kPi = 3.14159265358979323846;

std::string upper(const std::string& s) {
    std::string r(s);
    for (std::size_t i = 0; i < r.size(); ++i) r[i] = static_cast<char>(std::toupper(static_cast<unsigned char>(r[i])));
    return r;
}

// "235009876" -> 23 50 09 87 60 (ten digits, a trailing 0 added to nine)
void mmsiSymbols(const std::string& mmsi, std::vector<int>& out) {
    std::string d;
    for (std::size_t i = 0; i < mmsi.size(); ++i)
        if (mmsi[i] >= '0' && mmsi[i] <= '9') d += mmsi[i];
    while (d.size() < 10) d += '0';
    for (int k = 0; k < 5; ++k) out.push_back((d[2 * k] - '0') * 10 + (d[2 * k + 1] - '0'));
}

}  // namespace

// ---- generic ------------------------------------------------------------------------
std::vector<float> fskAudio(const std::vector<int>& levels, double levelsPerSecond, double centerHz,
                            double shiftHz, double rate, double amplitude) {
    const double dur = levels.size() / levelsPerSecond;
    const std::size_t n = static_cast<std::size_t>(dur * rate + 0.5);
    std::vector<float> out(n);
    double ph = 0;
    const double wHi = 2 * kPi * (centerHz + shiftHz / 2) / rate, wLo = 2 * kPi * (centerHz - shiftHz / 2) / rate;
    for (std::size_t i = 0; i < n; ++i) {
        std::size_t k = static_cast<std::size_t>(i / rate * levelsPerSecond);
        if (k >= levels.size()) k = levels.size() - 1;
        out[i] = static_cast<float>(amplitude * std::sin(ph));
        ph += levels[k] ? wHi : wLo;
        if (ph > 2 * kPi) ph -= 2 * kPi;
    }
    return out;
}

void addWhiteNoise(std::vector<float>& audio, double rms, unsigned seed) {
    unsigned long long s = 0x9E3779B97F4A7C15ULL ^ (static_cast<unsigned long long>(seed) * 0xD1B54A32D192ED03ULL);
    if (!s) s = 1;
    struct Rng {
        static double uni(unsigned long long& x) {   // xorshift64*, (0, 1]
            x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
            return ((x * 0x2545F4914F6CDD1DULL) >> 11) * (1.0 / 9007199254740992.0) + 1.0 / 18014398509481984.0;
        }
    };
    for (std::size_t i = 0; i < audio.size(); i += 2) {
        const double u1 = Rng::uni(s), u2 = Rng::uni(s), r = std::sqrt(-2 * std::log(u1)) * rms;
        audio[i] += static_cast<float>(r * std::cos(2 * kPi * u2));
        if (i + 1 < audio.size()) audio[i + 1] += static_cast<float>(r * std::sin(2 * kPi * u2));
    }
}

double noiseRmsForSnr(double signalPower, double snrDb, double rate, double bandwidthHz) {
    const double inBand = signalPower / std::pow(10.0, snrDb / 10);
    return std::sqrt(inBand * (rate / 2) / bandwidthHz);
}

std::vector<float> silence(double seconds, double rate) {
    return std::vector<float>(static_cast<std::size_t>(seconds * rate + 0.5), 0.f);
}

void append(std::vector<float>& a, const std::vector<float>& b) { a.insert(a.end(), b.begin(), b.end()); }

// ---- CW ---------------------------------------------------------------------------------
// Timing: dot = 1 unit on, 1 off between elements, 3 between letters, 7 between words.
std::vector<int> cwKeying(const std::string& text) {
    std::vector<int> k;
    const std::string t = upper(text);
    std::size_t start = 0;
    for (;;) {   // words, split on single spaces as the browser's msg.split(" ")
        const std::size_t sp = t.find(' ', start);
        const std::string w = t.substr(start, sp == std::string::npos ? std::string::npos : sp - start);
        for (std::size_t i = 0; i < w.size(); ++i) {
            const char* code = codes::morseFor(w[i]);
            if (!code) continue;
            for (const char* p = code; *p; ++p) {
                const int len = *p == '.' ? 1 : 3;
                for (int j = 0; j < len; ++j) k.push_back(1);
                k.push_back(0);
            }
            k.push_back(0); k.push_back(0);
        }
        k.push_back(0); k.push_back(0); k.push_back(0); k.push_back(0);
        if (sp == std::string::npos) break;
        start = sp + 1;
    }
    return k;
}

std::vector<float> cwAudio(const std::string& text, double wpm, double toneHz, double rate, double amplitude,
                           double leadSeconds, double tailSeconds) {
    const std::vector<int> key = cwKeying(text);
    const double dot = 1.2 / wpm;
    const std::size_t lead = static_cast<std::size_t>(leadSeconds * rate + 0.5);
    const std::size_t body = static_cast<std::size_t>(key.size() * dot * rate + 0.5);
    const std::size_t n = lead + body + static_cast<std::size_t>(tailSeconds * rate + 0.5);
    std::vector<float> out(n, 0.f);
    const double w = 2 * kPi * toneHz / rate;
    for (std::size_t i = 0; i < body; ++i) {
        const double pos = i / rate / dot;
        std::size_t k0 = static_cast<std::size_t>(pos);
        if (k0 >= key.size()) k0 = key.size() - 1;
        const double frac = pos - std::floor(pos);
        const int on = key[k0], prev = k0 > 0 ? key[k0 - 1] : 0, next = k0 + 1 < key.size() ? key[k0 + 1] : 0;
        double e = on;
        if (on && !prev && frac < 0.12) e = frac / 0.12;          // soft keying
        if (on && !next && frac > 0.88) e = (1 - frac) / 0.12;
        const std::size_t j = lead + i;
        out[j] = static_cast<float>(amplitude * e * std::sin(w * static_cast<double>(j)));
    }
    return out;
}

// ---- RTTY -----------------------------------------------------------------------------
std::vector<int> ita2Encode(const std::string& text) {
    std::vector<int> out;
    out.push_back(codes::kIta2Letters);   // start with LETTERS, as stations do
    bool figs = false;
    const std::string t = upper(text);
    for (std::size_t i = 0; i < t.size(); ++i) {
        const char c = t[i];
        if (c == ' ') { out.push_back(codes::kIta2Space); figs = false; continue; }
        if (c == '\n') { out.push_back(codes::kIta2CarriageReturn); out.push_back(codes::kIta2LineFeed); continue; }
        int k = -1;
        for (int j = 1; j < 32; ++j) { const char* s = codes::ita2Char(j, false); if (s[0] == c && s[1] == 0) { k = j; break; } }
        if (k > 0) {
            if (figs) { out.push_back(codes::kIta2Letters); figs = false; }
            out.push_back(k);
            continue;
        }
        for (int j = 1; j < 32; ++j) { const char* s = codes::ita2Char(j, true); if (s[0] == c && s[1] == 0) { k = j; break; } }
        if (k > 0) {
            if (!figs) { out.push_back(codes::kIta2Figures); figs = true; }
            out.push_back(k);
        }
    }
    return out;
}

std::vector<int> rttyHalfBits(const std::string& text, int idleBits) {
    std::vector<int> lv;
    for (int i = 0; i < idleBits; ++i) { lv.push_back(1); lv.push_back(1); }
    const std::vector<int> codes = ita2Encode(text);
    for (std::size_t i = 0; i < codes.size(); ++i) {
        lv.push_back(0); lv.push_back(0);   // start
        for (int b = 0; b < 5; ++b) { const int x = (codes[i] >> b) & 1; lv.push_back(x); lv.push_back(x); }
        lv.push_back(1); lv.push_back(1); lv.push_back(1);   // 1.5 stop bits
    }
    return lv;
}

std::vector<float> rttyAudio(const std::string& text, double baud, double shiftHz, double centerHz, double rate,
                             bool reverse, double amplitude, int idleBits) {
    std::vector<int> lv = rttyHalfBits(text, idleBits);
    if (reverse)
        for (std::size_t i = 0; i < lv.size(); ++i) lv[i] = 1 - lv[i];
    return fskAudio(lv, 2 * baud, centerHz, shiftHz, rate, amplitude);
}

// ---- NAVTEX -----------------------------------------------------------------------------
std::vector<int> navtexSlots(const std::string& text, int phasingSlots) {
    using namespace codes;
    std::vector<int> cs;
    cs.push_back(kCcirLetters);
    bool figs = false;
    const std::string t = upper(text);
    for (std::size_t i = 0; i < t.size(); ++i) {
        const std::string ch(1, t[i]);
        if (t[i] == ' ') { cs.push_back(kCcirSpace); figs = false; continue; }
        if (t[i] == '\n') { cs.push_back(kCcirCarriageReturn); cs.push_back(kCcirLineFeed); continue; }
        int k = ccirCode(ch, false);
        if (k >= 0) { if (figs) { cs.push_back(kCcirLetters); figs = false; } cs.push_back(k); continue; }
        k = ccirCode(ch, true);
        if (k >= 0) { if (!figs) { cs.push_back(kCcirFigures); figs = true; } cs.push_back(k); }
    }
    std::vector<int> out;
    for (int i = 0; i < phasingSlots; ++i) out.push_back(i & 1 ? kCcirAlpha : kCcirBeta);   // phasing
    const int L = static_cast<int>(cs.size());
    for (int m = 0; m < L + 3; ++m) {
        out.push_back(m < L ? cs[m] : kCcirAlpha);                        // DX slot
        out.push_back(m - 2 >= 0 && m - 2 < L ? cs[m - 2] : kCcirBeta);   // RX: the character from 5 slots earlier
    }
    for (int i = 0; i < 10; ++i) out.push_back(i & 1 ? kCcirAlpha : kCcirBeta);
    return out;
}

std::vector<int> navtexBits(const std::vector<int>& slots) {
    std::vector<int> bits;
    bits.reserve(slots.size() * 7);
    for (std::size_t i = 0; i < slots.size(); ++i)
        for (int b = 6; b >= 0; --b) bits.push_back((slots[i] >> b) & 1);
    return bits;
}

std::vector<float> navtexAudio(const std::string& text, double rate, double centerHz, double amplitude, int phasingSlots) {
    return fskAudio(navtexBits(navtexSlots(text, phasingSlots)), 100, centerHz, 170, rate, amplitude);
}

// ---- DSC ----------------------------------------------------------------------------------
std::vector<int> dscBits(const std::vector<int>& fields) {
    std::vector<int> msg;
    if (fields.empty()) return msg;
    int ecc = 0;
    for (std::size_t i = 1; i < fields.size(); ++i) ecc ^= fields[i];
    msg.push_back(fields[0]);
    msg.insert(msg.end(), fields.begin(), fields.end());   // format twice, ...
    msg.push_back(ecc);
    msg.push_back(fields.back());                          // EOS, ECC, EOS (simplified)
    std::vector<int> out;
    const int L = static_cast<int>(msg.size());
    for (int m = 0; m < L + 3; ++m) {
        out.push_back(m < L ? msg[m] : 125);
        out.push_back(m - 2 >= 0 && m - 2 < L ? msg[m - 2] : 111 - std::max(0, std::min(7, m)));
    }
    std::vector<int> syms;
    for (int i = 0; i < 6; ++i) { syms.push_back(125); syms.push_back(111 - i); }
    syms.insert(syms.end(), out.begin(), out.end());
    std::vector<int> bits;
    for (int i = 0; i < 120; ++i) bits.push_back(i & 1);   // dot pattern
    for (std::size_t s = 0; s < syms.size(); ++s) {
        const int v = syms[s];
        int z = 0;
        for (int i = 0; i < 7; ++i) z += ((v >> i) & 1) ? 0 : 1;
        for (int i = 0; i < 7; ++i) bits.push_back((v >> i) & 1);   // data, least significant first
        for (int i = 2; i >= 0; --i) bits.push_back((z >> i) & 1);  // check bits, most significant first
    }
    for (int i = 0; i < 40; ++i) bits.push_back(i & 1);
    return bits;
}

std::vector<int> dscAllShipsFields(int category, const std::string& mmsi) {
    std::vector<int> f;
    f.push_back(116);
    f.push_back(category);
    mmsiSymbols(mmsi, f);
    f.push_back(100);
    for (int i = 0; i < 4; ++i) f.push_back(126);
    f.push_back(127);
    return f;
}

std::vector<int> dscDistressFields(const std::string& mmsi, int nature) {
    std::vector<int> f;
    f.push_back(112);
    mmsiSymbols(mmsi, f);
    f.push_back(nature);
    for (int i = 0; i < 5; ++i) f.push_back(99);
    f.push_back(88); f.push_back(88);
    f.push_back(109);
    f.push_back(127);
    return f;
}

std::vector<float> dscAudio(const std::vector<int>& bits, double rate, double centerHz, double amplitude) {
    std::vector<int> lv(bits.size());
    for (std::size_t i = 0; i < bits.size(); ++i) lv[i] = 1 - bits[i];   // a 1 bit on the lower tone
    return fskAudio(lv, 100, centerHz, 170, rate, amplitude);
}

}  // namespace enc
}  // namespace dec
