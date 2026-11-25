// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DabFec.cpp
//  Layer:  Library liboffgrid-decoders (internal)
//
//  What this file does
//  -------------------
//  DAB's tables and error correction (declared in DabInternal.h):
//    - the mode I tables: carriers, phase reference symbol (EN 300 401
//      table 23/24: the h table and the per-carrier offsets), the frequency
//      interleaver (the 13 i + 511 mod 2048 permutation), the 24 puncturing
//      vectors and the tail's, the convolutional code (0133, 0171, 0145,
//      0133), the FIC's energy dispersal;
//    - Viterbi in the "butterfly" form (each new state has two predecessors,
//      2s and 2s+1 mod 64, with the same input bit; the 16 branch scores of
//      a step worked out once), depuncturing, PRBS, CRC-16;
//    - Reed-Solomon RS(120,110) for DAB+ (GF(256) with 0x11D, first root
//      alpha^0, 10 roots: Berlekamp-Massey, Chien search, Forney), the
//      super frame's fire code;
//    - sub-channel protection: EEP (options A and B, levels 1-4) and UEP
//      (table 8 with the welle.io profiles) to puncturing sections;
//    - DAB's character sets to UTF-8.
//
//  How it fits in
//  --------------
//  A port of OFFgrid-SDR's JS (diddy-boy, https://github.com/solariun/
//  offgrid-sdr, index.html, "dablib": DAB_PT, DAB_H, DAB_PCODES, DAB_OUT,
//  dabViterbi, dabPrbs, dabCrc16, dabDepunct, DAB_UEP, DAB_UEP_PROF,
//  dabProtection, dabRsDecode, dabFire, dabSfOk, DAB_EBU_LATIN, dabText).
//  The tables come from ETSI EN 300 401 as welle.io (GPL-2.0-or-later)
//  uses them.  Used by Dab.cpp (the receiver) and DabEncoders.cpp.
//
//  Contributing
//  ------------
//  Changing a table here changes both the receiver and the demo multiplex:
//  run tests/test_decoders_dab.cpp.  Mark changes "Addition to the JS".
// ============================================================================
#include "DabInternal.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace dec {
namespace dab {

namespace {

// the phase reference symbol: [first carrier, h row, phase offset] (JS DAB_PT)
const int kPt[48][3] = {
    {-768, 0, 1}, {-736, 1, 2}, {-704, 2, 0}, {-672, 3, 1}, {-640, 0, 3}, {-608, 1, 2}, {-576, 2, 2}, {-544, 3, 3},
    {-512, 0, 2}, {-480, 1, 1}, {-448, 2, 2}, {-416, 3, 3}, {-384, 0, 1}, {-352, 1, 2}, {-320, 2, 3}, {-288, 3, 3},
    {-256, 0, 2}, {-224, 1, 2}, {-192, 2, 2}, {-160, 3, 1}, {-128, 0, 1}, {-96, 1, 3},  {-64, 2, 1},  {-32, 3, 2},
    {1, 0, 3},    {33, 3, 1},   {65, 2, 1},   {97, 1, 1},   {129, 0, 2},  {161, 3, 2},  {193, 2, 1},  {225, 1, 0},
    {257, 0, 2},  {289, 3, 2},  {321, 2, 3},  {353, 1, 3},  {385, 0, 0},  {417, 3, 2},  {449, 2, 1},  {481, 1, 3},
    {513, 0, 3},  {545, 3, 3},  {577, 2, 3},  {609, 1, 0},  {641, 0, 3},  {673, 3, 0},  {705, 2, 1},  {737, 1, 1}};
const int kH[4][16] = {{0, 2, 0, 0, 0, 0, 1, 1, 2, 0, 0, 0, 2, 2, 1, 1},
                       {0, 3, 2, 3, 0, 1, 3, 0, 2, 1, 2, 3, 2, 3, 3, 0},
                       {0, 0, 0, 2, 0, 2, 1, 3, 2, 2, 0, 2, 2, 0, 1, 3},
                       {0, 1, 2, 1, 0, 3, 3, 2, 2, 3, 2, 1, 2, 1, 3, 2}};
// the puncturing vectors PI 1..24 (JS DAB_PCODES)
const char* const kPcodes[24] = {
    "11001000100010001000100010001000", "11001000100010001100100010001000", "11001000110010001100100010001000",
    "11001000110010001100100011001000", "11001100110010001100100011001000", "11001100110010001100110011001000",
    "11001100110011001100110011001000", "11001100110011001100110011001100", "11101100110011001100110011001100",
    "11101100110011001110110011001100", "11101100111011001110110011001100", "11101100111011001110110011101100",
    "11101110111011001110110011101100", "11101110111011001110111011101100", "11101110111011101110111011101100",
    "11101110111011101110111011101110", "11111110111011101110111011101110", "11111110111011101111111011101110",
    "11111110111111101111111011101110", "11111110111111101111111011111110", "11111111111111101111111011111110",
    "11111111111111101111111111111110", "11111111111111111111111111111110", "11111111111111111111111111111111"};
const unsigned char kPix[24] = {1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0};

// DAB's EBU Latin character set (ETSI TS 101 756 annex C, as welle.io and the JS: not ASCII even below 0x80)
const unsigned short kEbuLatin[256] = {
    0,   280, 302, 370, 258, 278, 270, 536, 538, 266, 10,  11,  288, 313, 379, 323, 261, 281, 303, 371, 259, 279,
    271, 537, 539, 267, 327, 282, 289, 314, 380, 130, 32,  33,  34,  35,  322, 37,  38,  39,  40,  41,  42,  43,
    44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,
    66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,
    88,  89,  90,  91,  366, 93,  321, 95,  260, 97,  98,  99,  100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
    110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 171, 367, 187, 317, 294, 225, 224, 233, 232,
    237, 236, 243, 242, 250, 249, 209, 199, 350, 223, 161, 376, 226, 228, 234, 235, 238, 239, 244, 246, 251, 252,
    241, 231, 351, 287, 305, 255, 310, 325, 169, 290, 286, 283, 328, 337, 336, 8364, 163, 36, 256, 274, 298, 362,
    311, 326, 315, 291, 316, 304, 324, 369, 368, 191, 318, 176, 257, 275, 299, 363, 193, 192, 201, 200, 205, 204,
    211, 210, 218, 217, 344, 268, 352, 381, 208, 319, 194, 196, 202, 203, 206, 207, 212, 214, 219, 220, 345, 269,
    353, 382, 273, 320, 195, 197, 198, 338, 375, 221, 213, 216, 222, 330, 340, 262, 346, 377, 356, 240, 227, 229,
    230, 339, 373, 253, 245, 248, 254, 331, 341, 263, 347, 378, 357, 295};

Tables* buildTables() {
    Tables* t = new Tables;
    const double kPi = 3.14159265358979323846;
    int q = 0;
    for (int k = -768; k <= 768; ++k)
        if (k) t->carriers[q++] = k;
    for (int i = 0; i < K; ++i) {
        const int k = t->carriers[i];
        int r = 0;
        for (int j = 47; j >= 0; --j)
            if (k >= kPt[j][0]) {
                r = j;
                break;
            }
        const double ph = kPi / 2 * (kH[kPt[r][1]][(k - kPt[r][0]) % 16] + kPt[r][2]);
        t->prs[i] = Cf(static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
        t->bin[i] = ((k % Tu) + Tu) % Tu;
        const double a = 2 * kPi * k * 32 / Tu;
        t->win[i] = Cf(static_cast<float>(std::cos(a)), static_cast<float>(std::sin(a)));
    }
    // the frequency interleaver: 13 i + 511 mod 2048, the values 256..1792 except 1024 in order, each a carrier
    {
        int v = 0, n = 0;
        for (int i = 0; i < Tu; ++i) {
            if (i) v = (13 * v + 511) % Tu;
            if (v != 1024 && v >= 256 && v <= 256 + K) {
                const int k = v - 1024;                  // the carrier, -768..768
                t->perm[n++] = k < 0 ? k + 768 : k + 767;  // its index in `carriers`
            }
        }
    }
    for (int p = 0; p < 24; ++p)
        for (int i = 0; i < 32; ++i) t->pcodes[p][i] = static_cast<unsigned char>(kPcodes[p][i] == '1');
    std::memcpy(t->pix, kPix, sizeof kPix);
    const int P[4] = {0133, 0171, 0145, 0133};
    for (int w = 0; w < 128; ++w) {
        int c4 = 0;
        for (int j = 0; j < 4; ++j) {
            int x = w & P[j], c = 0;
            while (x) {
                c ^= x & 1;
                x >>= 1;
            }
            t->out[w * 4 + j] = static_cast<signed char>(c ? 1 : -1);
            if (c) c4 |= 8 >> j;
        }
        t->vcode[w] = static_cast<unsigned char>(c4);
    }
    {
        std::vector<unsigned char> p;
        prbs(768, p);
        std::memcpy(t->prbs768, &p[0], 768);
    }
    int x = 1;
    for (int i = 0; i < 255; ++i) {
        t->gfExp[i] = static_cast<unsigned char>(x);
        t->gfLog[x] = static_cast<unsigned char>(i);
        x <<= 1;
        if (x & 256) x ^= 0x11d;
    }
    t->gfLog[0] = 0;
    for (int i = 255; i < 512; ++i) t->gfExp[i] = t->gfExp[i - 255];
    return t;
}

// GF(256) arithmetic (JS dabGfMul, dabGfDiv, dabGfPow)
inline int gfMul(const Tables& t, int a, int b) { return a && b ? t.gfExp[t.gfLog[a] + t.gfLog[b]] : 0; }
inline int gfDiv(const Tables& t, int a, int b) { return a ? t.gfExp[(t.gfLog[a] + 255 - t.gfLog[b]) % 255] : 0; }
inline int gfPow(const Tables& t, int e) { return t.gfExp[((e % 255) + 255) % 255]; }

// the UEP profiles (JS DAB_UEP_PROF; the 320 and 384 kbit/s rows as welle.io):
// kbit/s, level, then L1 L2 L3 L4 PI1 PI2 PI3 PI4
const int kUepProf[][10] = {
    {320, 5, 11, 26, 200, 3, 8, 5, 2, 6},     {320, 4, 11, 25, 201, 3, 13, 9, 5, 10},
    {320, 2, 11, 26, 200, 3, 24, 17, 9, 17},  {384, 5, 11, 27, 247, 3, 8, 6, 2, 7},
    {384, 3, 11, 24, 250, 3, 16, 9, 7, 10},   {384, 1, 12, 28, 245, 3, 24, 20, 14, 23},
    {32, 5, 3, 4, 17, 0, 5, 3, 2, -1},        {32, 4, 3, 3, 18, 0, 11, 6, 5, -1},
    {32, 3, 3, 4, 14, 3, 15, 9, 6, 8},        {32, 2, 3, 4, 14, 3, 22, 13, 8, 13},
    {32, 1, 3, 5, 13, 3, 24, 17, 12, 17},     {48, 5, 4, 3, 26, 3, 5, 4, 2, 3},
    {48, 4, 3, 4, 26, 3, 9, 6, 4, 6},         {48, 3, 3, 4, 26, 3, 15, 10, 6, 9},
    {48, 2, 3, 4, 26, 3, 24, 14, 8, 15},      {48, 1, 3, 5, 25, 3, 24, 18, 13, 18},
    {56, 5, 6, 10, 23, 3, 5, 4, 2, 3},        {56, 4, 6, 10, 23, 3, 9, 6, 4, 5},
    {56, 3, 6, 12, 21, 3, 16, 7, 6, 9},       {56, 2, 6, 10, 23, 3, 23, 13, 8, 13},
    {64, 5, 6, 9, 31, 2, 5, 3, 2, 3},         {64, 4, 6, 9, 33, 0, 11, 6, 5, -1},
    {64, 3, 6, 12, 27, 3, 16, 8, 6, 9},       {64, 2, 6, 10, 29, 3, 23, 13, 8, 13},
    {64, 1, 6, 11, 28, 3, 24, 18, 12, 18},    {80, 5, 6, 10, 41, 3, 6, 3, 2, 3},
    {80, 4, 6, 10, 41, 3, 11, 6, 5, 6},       {80, 3, 6, 11, 40, 3, 16, 8, 6, 7},
    {80, 2, 6, 10, 41, 3, 23, 13, 8, 13},     {80, 1, 6, 10, 41, 3, 24, 7, 12, 18},
    {96, 5, 7, 9, 53, 3, 5, 4, 2, 4},         {96, 4, 7, 10, 52, 3, 9, 6, 4, 6},
    {96, 3, 6, 12, 51, 3, 16, 9, 6, 10},      {96, 2, 6, 10, 53, 3, 22, 12, 9, 12},
    {96, 1, 6, 13, 50, 3, 24, 18, 13, 19},    {112, 5, 14, 17, 50, 3, 5, 4, 2, 5},
    {112, 4, 11, 21, 49, 3, 9, 6, 4, 8},      {112, 3, 11, 23, 47, 3, 16, 8, 6, 9},
    {112, 2, 11, 21, 49, 3, 23, 12, 9, 14},   {128, 5, 12, 19, 62, 3, 5, 3, 2, 4},
    {128, 4, 11, 21, 61, 3, 11, 6, 5, 7},     {128, 3, 11, 22, 60, 3, 16, 9, 6, 10},
    {128, 2, 11, 21, 61, 3, 22, 12, 9, 14},   {128, 1, 11, 20, 62, 3, 24, 17, 13, 19},
    {160, 5, 11, 19, 87, 3, 5, 4, 2, 4},      {160, 4, 11, 23, 83, 3, 11, 6, 5, 9},
    {160, 3, 11, 24, 82, 3, 16, 8, 6, 11},    {160, 2, 11, 21, 85, 3, 22, 11, 9, 13},
    {160, 1, 11, 22, 84, 3, 24, 18, 12, 19},  {192, 5, 11, 20, 110, 3, 6, 4, 2, 5},
    {192, 4, 11, 22, 108, 3, 10, 6, 4, 9},    {192, 3, 11, 24, 106, 3, 16, 10, 6, 11},
    {192, 2, 11, 20, 110, 3, 22, 13, 9, 13},  {192, 1, 11, 21, 109, 3, 24, 20, 13, 24},
    {224, 5, 12, 22, 131, 3, 8, 6, 2, 6},     {224, 4, 12, 26, 127, 3, 12, 8, 4, 11},
    {224, 3, 11, 20, 134, 3, 16, 10, 7, 9},   {224, 2, 11, 22, 132, 3, 24, 16, 10, 15},
    {224, 1, 11, 24, 130, 3, 24, 20, 12, 20}, {256, 5, 11, 24, 154, 3, 6, 5, 2, 5},
    {256, 4, 11, 24, 154, 3, 12, 9, 5, 10},   {256, 3, 11, 27, 151, 3, 16, 10, 7, 10},
    {256, 2, 11, 22, 156, 3, 24, 14, 10, 13}, {256, 1, 11, 26, 152, 3, 24, 19, 14, 18}};

}  // namespace

const Tables& tables() {
    static const Tables* t = buildTables();
    return *t;
}

const int kImap[16] = {0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15};

// the UEP table (EN 300 401 table 8): index -> [CUs, protection level, kbit/s] (JS DAB_UEP)
const UepRow kUep[64] = {
    {16, 5, 32},   {21, 4, 32},   {24, 3, 32},   {29, 2, 32},   {35, 1, 32},   {24, 5, 48},   {29, 4, 48},
    {35, 3, 48},   {42, 2, 48},   {52, 1, 48},   {29, 5, 56},   {35, 4, 56},   {42, 3, 56},   {52, 2, 56},
    {32, 5, 64},   {42, 4, 64},   {48, 3, 64},   {58, 2, 64},   {70, 1, 64},   {40, 5, 80},   {52, 4, 80},
    {58, 3, 80},   {70, 2, 80},   {84, 1, 80},   {48, 5, 96},   {58, 4, 96},   {70, 3, 96},   {84, 2, 96},
    {104, 1, 96},  {58, 5, 112},  {70, 4, 112},  {84, 3, 112},  {104, 2, 112}, {64, 5, 128},  {84, 4, 128},
    {96, 3, 128},  {116, 2, 128}, {140, 1, 128}, {80, 5, 160},  {104, 4, 160}, {116, 3, 160}, {140, 2, 160},
    {168, 1, 160}, {96, 5, 192},  {116, 4, 192}, {128, 3, 192}, {168, 2, 192}, {208, 1, 192}, {116, 5, 224},
    {140, 4, 224}, {168, 3, 224}, {208, 2, 224}, {232, 1, 224}, {128, 5, 256}, {168, 4, 256}, {192, 3, 256},
    {232, 2, 256}, {280, 1, 256}, {160, 5, 320}, {208, 4, 320}, {280, 2, 320}, {192, 5, 384}, {280, 3, 384},
    {416, 1, 384}};

bool uepProfile(int kbps, int lev, int prof[8]) {
    for (std::size_t i = 0; i < sizeof kUepProf / sizeof kUepProf[0]; ++i)
        if (kUepProf[i][0] == kbps && kUepProf[i][1] == lev) {
            for (int j = 0; j < 8; ++j) prof[j] = kUepProf[i][2 + j];
            return true;
        }
    return false;
}

// ---- Viterbi (JS dabViterbi): the fast "butterfly" form; unused start
// states begin at a large score, so no "impossible" checks are needed.
void Viterbi::decode(const float* soft, int nbits, unsigned char* out) {
    const Tables& T = tables();
    const int n = nbits + 6;
    const double BIG = 1e15;
    double pmA[64], pmB[64], bm[16];
    double* pm = pmA;
    double* nm = pmB;
    for (int i = 0; i < 64; ++i) pm[i] = BIG;
    pm[0] = 0;
    if (dec_.size() < static_cast<std::size_t>(n) * 64) dec_.resize(static_cast<std::size_t>(n) * 64);
    unsigned char* dec = &dec_[0];
    const unsigned char* C = T.vcode;
    for (int t = 0; t < n; ++t) {
        const double s0 = soft[4 * t], s1 = soft[4 * t + 1], s2 = soft[4 * t + 2], s3 = soft[4 * t + 3];
        for (int c = 0; c < 16; ++c)
            bm[c] = ((c & 8) ? s0 : -s0) + ((c & 4) ? s1 : -s1) + ((c & 2) ? s2 : -s2) + ((c & 1) ? s3 : -s3);
        unsigned char* d = dec + static_cast<std::size_t>(t) * 64;
        for (int ns = 0; ns < 64; ++ns) {
            const int b = ns >> 5, p0 = (ns << 1) & 63, p1 = p0 | 1, hi = b << 6;
            const double m0 = pm[p0] - bm[C[hi | p0]], m1 = pm[p1] - bm[C[hi | p1]];
            if (m1 < m0) {
                nm[ns] = m1;
                d[ns] = 1;
            } else {
                nm[ns] = m0;
                d[ns] = 0;
            }
        }
        double* tmp = pm;
        pm = nm;
        nm = tmp;
        // (Addition to the JS: the scores kept small, so they stay exact over long sub-channels)
        if ((t & 255) == 255) {
            double lo = pm[0];
            for (int i = 1; i < 64; ++i)
                if (pm[i] < lo) lo = pm[i];
            for (int i = 0; i < 64; ++i) pm[i] -= lo;
        }
    }
    int s = 0;
    double best = pm[0];
    for (int i = 1; i < 64; ++i)
        if (pm[i] < best) {
            best = pm[i];
            s = i;
        }
    for (int t = n - 1; t >= 0; --t) {
        if (t < nbits) out[t] = static_cast<unsigned char>(s >> 5);
        s = ((s << 1) & 63) | dec[static_cast<std::size_t>(t) * 64 + s];
    }
}

void depuncture(const float* soft, const Section* sections, int nSections, int nout, std::vector<float>& vb) {
    const Tables& T = tables();
    vb.assign(static_cast<std::size_t>(nout) * 4 + 24, 0.f);
    std::size_t c = 0, loc = 0;
    const std::size_t cap = vb.size();
    for (int s = 0; s < nSections; ++s) {
        const unsigned char* pat = T.pcodes[sections[s].pattern - 1];
        for (int i = 0; i < sections[s].blocks; ++i)
            for (int k = 0; k < 128; ++k, ++loc)
                if (pat[k & 31] && loc < cap) vb[loc] = soft[c++];
    }
    for (int k = 0; k < 24; ++k, ++loc)
        if (T.pix[k] && loc < cap) vb[loc] = soft[c++];
}

int puncturedLength(const Section* sections, int nSections) {
    const Tables& T = tables();
    int n = 0;
    for (int s = 0; s < nSections; ++s) {
        int ones = 0;
        for (int k = 0; k < 32; ++k) ones += T.pcodes[sections[s].pattern - 1][k];
        n += sections[s].blocks * 4 * ones;
    }
    return n + 12;
}

void prbs(int n, std::vector<unsigned char>& o) {
    unsigned char r[9];
    for (int i = 0; i < 9; ++i) r[i] = 1;
    o.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const unsigned char b = static_cast<unsigned char>(r[8] ^ r[4]);
        o[static_cast<std::size_t>(i)] = b;
        for (int j = 8; j > 0; --j) r[j] = r[j - 1];
        r[0] = b;
    }
}

unsigned crc16(const unsigned char* by, std::size_t n) {
    unsigned c = 0xffff;
    for (std::size_t i = 0; i < n; ++i) {
        c ^= static_cast<unsigned>(by[i]) << 8;
        for (int j = 0; j < 8; ++j) c = (c & 0x8000) ? ((c << 1) ^ 0x1021) & 0xffff : (c << 1) & 0xffff;
    }
    return c ^ 0xffff;
}

bool crcOk(const unsigned char* by, std::size_t n) {
    if (n < 3) return false;
    return crc16(by, n - 2) == ((static_cast<unsigned>(by[n - 2]) << 8) | by[n - 1]);
}

// ---- Reed-Solomon (JS dabRsDecode): corrects up to 5 bytes a codeword
int rsDecode(unsigned char* cw, int n) {
    const Tables& T = tables();
    int S[10];
    int any = 0;
    for (int j = 0; j < 10; ++j) {
        int s = 0;
        const int a = gfPow(T, j);
        for (int i = 0; i < n; ++i) s = gfMul(T, s, a) ^ cw[i];
        S[j] = s;
        any |= s;
    }
    if (!any) return 0;
    // Berlekamp-Massey: the error locator
    int C[32] = {1}, B[32] = {1}, Tm[32];
    int clen = 1, blen = 1, Lg = 0, m = 1, b = 1;
    for (int k = 0; k < 10; ++k) {
        int d = S[k];
        for (int i = 1; i <= Lg; ++i) d ^= gfMul(T, i < clen ? C[i] : 0, S[k - i]);
        if (!d) {
            ++m;
            continue;
        }
        for (int i = 0; i < clen; ++i) Tm[i] = C[i];
        const int tlen = clen, coef = gfDiv(T, d, b);
        for (int i = 0; i < blen; ++i) {
            if (i + m >= 32) return -1;
            while (clen <= i + m) C[clen++] = 0;
            C[i + m] ^= gfMul(T, coef, B[i]);
        }
        if (2 * Lg <= k) {
            Lg = k + 1 - Lg;
            for (int i = 0; i < tlen; ++i) B[i] = Tm[i];
            blen = tlen;
            b = d;
            m = 1;
        } else
            ++m;
    }
    if (Lg > 5) return -1;
    // Chien search: a byte at index i (power n-1-i) is in error where C(alpha^-(n-1-i)) = 0
    int pos[16], npos = 0;
    for (int i = 0; i < n; ++i) {
        const int xinv = gfPow(T, -(n - 1 - i));
        int v = 0;
        for (int k = clen - 1; k >= 0; --k) v = gfMul(T, v, xinv) ^ C[k];
        if (!v) {
            if (npos >= 16) return -1;
            pos[npos++] = i;
        }
    }
    if (npos != Lg) return -1;
    // Forney: Omega = S(x) C(x) mod x^10; value = X * Omega(1/X) / C'(1/X) (first root alpha^0)
    int Om[10];
    for (int i = 0; i < 10; ++i) {
        Om[i] = 0;
        for (int k = 0; k <= i && k < clen; ++k) Om[i] ^= gfMul(T, C[k], S[i - k]);
    }
    for (int e = 0; e < npos; ++e) {
        const int i = pos[e];
        const int X = gfPow(T, n - 1 - i), xi = gfPow(T, -(n - 1 - i));
        int om = 0, dc = 0;
        for (int k = 9; k >= 0; --k) om = gfMul(T, om, xi) ^ Om[k];
        for (int k = 1; k < clen; k += 2) dc ^= gfMul(T, C[k], gfPow(T, -(n - 1 - i) * (k - 1)));  // (odd terms: the derivative in GF(2^m))
        if (!dc) return -1;
        cw[i] = static_cast<unsigned char>(cw[i] ^ gfMul(T, X, gfDiv(T, om, dc)));
    }
    return npos;
}

// (Addition to the JS: systematic encoding, g(x) = (x - a^0)...(x - a^9), for tests)
void rsEncode(const unsigned char* data, unsigned char* parity) {
    const Tables& T = tables();
    int g[11] = {1};
    int glen = 1;
    for (int j = 0; j < 10; ++j) {   // g *= (x + a^j)
        const int r = gfPow(T, j);
        int ng[11] = {0};
        for (int i = 0; i < glen; ++i) {
            ng[i] ^= g[i];                    // x * g: coefficients by falling power, so shift handled below
            ng[i + 1] ^= gfMul(T, g[i], r);
        }
        ++glen;
        for (int i = 0; i < glen; ++i) g[i] = ng[i];
    }
    // g[0] = 1 is the leading (x^10) coefficient
    int reg[10] = {0};
    for (int i = 0; i < 110; ++i) {
        const int fb = data[i] ^ reg[0];
        for (int k = 0; k < 9; ++k) reg[k] = reg[k + 1] ^ gfMul(T, fb, g[k + 1]);
        reg[9] = gfMul(T, fb, g[10]);
    }
    for (int k = 0; k < 10; ++k) parity[k] = static_cast<unsigned char>(reg[k]);
}

bool fireOk(const unsigned char* by) {
    unsigned c = 0;
    for (int i = 2; i < 11; ++i)
        for (int b = 7; b >= 0; --b) {
            const unsigned fb = ((c >> 15) & 1) ^ ((by[i] >> b) & 1);
            c = ((c << 1) & 0xffff) ^ (fb ? 0x782f : 0);
        }
    return c == ((static_cast<unsigned>(by[0]) << 8) | by[1]);
}

// a super frame's header is real only if its fire code passes AND its audio units' start positions make sense: at
// least two units, so they must increase, inside the frame (zeros pass the fire code; JS dabSfOk)
bool superframeOk(const unsigned char* sf, int s) {
    if (!fireOk(sf)) return false;
    const int dac = (sf[2] >> 6) & 1, sbr = (sf[2] >> 5) & 1, n = dac ? (sbr ? 3 : 6) : (sbr ? 2 : 4);
    const int st[6] = {dac ? (sbr ? 6 : 11) : (sbr ? 5 : 8), (sf[3] << 4) | (sf[4] >> 4), ((sf[4] & 15) << 8) | sf[5],
                       (sf[6] << 4) | (sf[7] >> 4),          ((sf[7] & 15) << 8) | sf[8], (sf[9] << 4) | (sf[10] >> 4)};
    for (int j = 1; j < n; ++j)
        if (st[j] <= st[j - 1] || st[j] >= 110 * s) return false;
    return true;
}

// ---- protection (JS dabProtection)
std::string Protection::name() const { return level; }

bool protectionOf(const SubChannel& sc, Protection& p) {
    p = Protection();
    char buf[32];
    if (sc.uep) {
        if (sc.uepIndex < 0 || sc.uepIndex >= 64) return false;
        const UepRow& r = kUep[sc.uepIndex];
        int pr[8];
        if (!uepProfile(r.kbps, r.lev, pr)) return false;
        p.kbps = r.kbps;
        p.nSections = 0;
        for (int i = 0; i < 4; ++i) {
            if (i == 3 && !pr[3]) break;
            p.sections[p.nSections].pattern = pr[4 + i];
            p.sections[p.nSections].blocks = pr[i];
            ++p.nSections;
        }
        std::snprintf(buf, sizeof buf, "UEP %d", r.lev);
        p.level = buf;
        return true;
    }
    const int lev = sc.lev;
    if (lev < 0 || lev > 3) return false;
    if (sc.opt == 0) {
        const int div[4] = {12, 8, 6, 4};
        if (sc.size % div[lev]) return false;   // (Addition to the JS: a size that isn't a whole multiple is refused)
        const int n = sc.size / div[lev];
        if (n < 1) return false;
        p.kbps = 8 * n;
        std::snprintf(buf, sizeof buf, "EEP %d-A", lev + 1);
        p.level = buf;
        if (lev == 1 && n == 1) {   // (EEP 2-A at 8 kbit/s: the standard's special case, table 15)
            p.sections[0].pattern = 13;
            p.sections[0].blocks = 5;
            p.sections[1].pattern = 12;
            p.sections[1].blocks = 1;
            p.nSections = 2;
            return true;
        }
        const int pi1[4] = {24, 14, 8, 3}, pi2[4] = {23, 13, 7, 2};
        const int l1[4] = {6 * n - 3, 2 * n - 3, 6 * n - 3, 4 * n - 3}, l2[4] = {3, 4 * n + 3, 3, 2 * n + 3};
        p.sections[0].pattern = pi1[lev];
        p.sections[0].blocks = l1[lev];
        p.sections[1].pattern = pi2[lev];
        p.sections[1].blocks = l2[lev];
        p.nSections = 2;
        return p.sections[0].blocks >= 0;
    }
    if (sc.opt == 1) {
        const int div[4] = {27, 21, 18, 15};
        if (sc.size % div[lev]) return false;
        const int n = sc.size / div[lev];
        if (n < 1) return false;
        p.kbps = 32 * n;
        std::snprintf(buf, sizeof buf, "EEP %d-B", lev + 1);
        p.level = buf;
        const int pi1[4] = {10, 6, 4, 2}, pi2[4] = {9, 5, 3, 1};
        p.sections[0].pattern = pi1[lev];
        p.sections[0].blocks = 24 * n - 3;
        p.sections[1].pattern = pi2[lev];
        p.sections[1].blocks = 3;
        p.nSections = 2;
        return true;
    }
    return false;
}

// ---- characters (JS dabText)
void appendUtf8(std::string& s, unsigned cp) {
    if (cp < 0x80)
        s += static_cast<char>(cp);
    else if (cp < 0x800) {
        s += static_cast<char>(0xc0 | (cp >> 6));
        s += static_cast<char>(0x80 | (cp & 63));
    } else if (cp < 0x10000) {
        s += static_cast<char>(0xe0 | (cp >> 12));
        s += static_cast<char>(0x80 | ((cp >> 6) & 63));
        s += static_cast<char>(0x80 | (cp & 63));
    } else {
        s += static_cast<char>(0xf0 | (cp >> 18));
        s += static_cast<char>(0x80 | ((cp >> 12) & 63));
        s += static_cast<char>(0x80 | ((cp >> 6) & 63));
        s += static_cast<char>(0x80 | (cp & 63));
    }
}

namespace {
std::string trim(const std::string& s) {
    std::size_t a = 0, b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\n' || s[a] == '\r' || s[a] == '\v' || s[a] == '\f')) ++a;
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\n' || s[b - 1] == '\r' || s[b - 1] == '\v' ||
                     s[b - 1] == '\f'))
        --b;
    return s.substr(a, b - a);
}
}  // namespace

std::string text(const unsigned char* b, std::size_t n, int cs) {
    std::string s;
    if (cs == 15) {
        // UTF-8: passed through, invalid bytes replaced by U+FFFD (as TextDecoder does)
        std::size_t i = 0;
        while (i < n) {
            const unsigned c = b[i];
            int len = c < 0x80 ? 1 : (c >> 5) == 6 ? 2 : (c >> 4) == 14 ? 3 : (c >> 3) == 30 ? 4 : 0;
            bool ok = len > 0 && i + static_cast<std::size_t>(len) <= n;
            for (int k = 1; ok && k < len; ++k) ok = (b[i + static_cast<std::size_t>(k)] >> 6) == 2;
            if (!ok) {
                appendUtf8(s, 0xfffd);
                ++i;
                continue;
            }
            s.append(reinterpret_cast<const char*>(b + i), static_cast<std::size_t>(len));
            i += static_cast<std::size_t>(len);
        }
        return trim(s);
    }
    if (cs == 6) {
        for (std::size_t i = 0; i + 1 < n; i += 2) appendUtf8(s, (static_cast<unsigned>(b[i]) << 8) | b[i + 1]);
        return trim(s);
    }
    for (std::size_t i = 0; i < n; ++i) {
        const unsigned cp = kEbuLatin[b[i]];
        if (cp) appendUtf8(s, cp);   // (Addition to the JS: a 0 byte leaves nothing, not a NUL)
    }
    return trim(s);
}

std::vector<unsigned char> plusAsc(const AacFormat& f) {
    static const int SR[12] = {96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000};
    const int core = f.sbr ? f.rate / 2 : f.rate;
    int ci = 0, ri = 0;
    for (int i = 0; i < 12; ++i) {
        if (SR[i] == core) ci = i;
        if (SR[i] == f.rate) ri = i;
    }
    std::vector<unsigned char> out;
    unsigned acc = 0;
    int nb = 0;
    struct Put {
        std::vector<unsigned char>& o;
        unsigned& a;
        int& n;
        void operator()(unsigned v, int bits) {
            for (int i = bits - 1; i >= 0; --i) {
                a = (a << 1) | ((v >> i) & 1);
                if (++n == 8) {
                    o.push_back(static_cast<unsigned char>(a));
                    a = 0;
                    n = 0;
                }
            }
        }
    } put = {out, acc, nb};
    put(2, 5);
    put(static_cast<unsigned>(ci), 4);
    put(f.stereo && !f.ps ? 2 : 1, 4);
    put(4, 3);   // frameLengthFlag 1 (960), dependsOnCoreCoder 0, extensionFlag 0
    if (f.sbr) {
        put(0x2b7, 11);
        put(5, 5);
        put(1, 1);
        put(static_cast<unsigned>(ri), 4);
        if (f.ps) {
            put(0x548, 11);
            put(1, 1);
        }
    }
    if (nb) put(0, 8 - nb);
    return out;
}

}  // namespace dab
}  // namespace dec
