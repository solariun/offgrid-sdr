// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DrmCoding.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  DRM's coding pieces (ETSI ES 201 980 7.2-7.3, as Dream's MLC): the bit
//  and cell interleaver tables, the puncturing pattern of a code rate, the
//  mother code (rate 1/4, constraint length 7, generators 133/171/145/133
//  octal as DRM_G), its Viterbi decoder, the energy-dispersal PRBS, Dream's
//  CRC-8/CRC-16, the CCITT CRC-16 of text and packets, the gain references'
//  phases (ETSI 8.4.4) and a DFT of any length.
//
//  Port of OFFgrid-SDR's JS (diddy-boy, https://github.com/solariun/
//  offgrid-sdr, index.html "drmlib": drmIlTable, drmPattern, drmEncode,
//  drmViterbi, drmPrbs, drmCrc, drmTextCrc, drmGainRefs, DrmDft), itself
//  following Dream (TU Darmstadt, Volker Fischer and others; GPL-2.0-or-
//  later, licenses/dream-COPYING.txt).
//
//  Differences from the JS:
//    - the Viterbi decoder keeps its survivor tables between calls (the JS
//      allocated two arrays a step) and normalises its metrics each step;
//    - the DFT uses liboffgrid-dsp's float radix-2 FFT inside Bluestein's
//      method (the JS: its own double FFT), and the FFT directly when the
//      length is a power of two (mode B): the same transform.
//
//  How it fits in
//  --------------
//  Declared in DrmInternal.h; used by DrmRx.cpp and DrmEncoders.cpp.
//
//  Contributing
//  ------------
//  The encoder and the decoder must agree bit for bit: the tests run the
//  generator's output through the receiver.  No allocation in the loops.
// ============================================================================
#include "decoders/DrmInternal.h"

#include <cmath>
#include <limits>

namespace dec {
namespace drm {

namespace {

const double kPi = 3.14159265358979323846;
// DRM_G: the four generator polynomials (octal 155, 117, 123, 155: ETSI's
// 133, 171, 145, 133 in the order the shift register is kept here, newest
// bit in bit 0).
const int kGen[4] = {0155, 0117, 0123, 0155};
// DRM_GENS: which generators a step sends, by code 1..5.
const int kGensN[6] = {0, 4, 3, 2, 1, 2};
const int kGens[6][4] = {{0, 0, 0, 0}, {0, 1, 2, 3}, {0, 1, 2, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}, {0, 2, 0, 0}};

// DRM_PAR: the parity of a 7-bit value.
struct ParityTable {
    unsigned char p[128];
    ParityTable() {
        for (int i = 0; i < 128; ++i) {
            int v = i, q = 0;
            for (; v; v >>= 1) q ^= v & 1;
            p[i] = static_cast<unsigned char>(q);
        }
    }
};
const ParityTable kPar;

}  // namespace

void ilTable(int N, int t0, std::vector<int>& t) {
    t.assign(N > 0 ? N : 0, 0);
    if (N <= 0) return;
    int s = 1;
    while (s < N) s <<= 1;
    const int q = s / 4 - 1;
    t[0] = 0;
    for (int i = 1; i < N; ++i) {
        long long v = (static_cast<long long>(t0) * t[i - 1] + q) % s;
        while (v >= N) v = (static_cast<long long>(t0) * v + q) % s;
        t[i] = static_cast<int>(v);
    }
}

void makePattern(int rate, int M, int N2, bool fac, Pattern& out) {
    const int* P = punct(rate);
    const int grp = P[0], ones = P[1];
    out.clear();
    out.reserve(static_cast<std::size_t>(M) + 6);
    for (int i = 0; i < M; ++i) out.push_back(static_cast<unsigned char>(P[2 + i % grp]));
    if (fac) {
        for (int i = 0; i < 6; ++i) out.push_back(static_cast<unsigned char>(P[2 + (M + i) % grp]));
    } else {
        // the tail: which of the 12 tail patterns, from what the rate leaves over
        const int tp = 2 * N2 - 12 - ones * ((2 * N2 - 12) / ones);
        const int* t = tailPattern(tp);
        for (int i = 0; i < 6; ++i) out.push_back(static_cast<unsigned char>(t[i]));
    }
}

int patternBits(const Pattern& pat) {
    int n = 0;
    for (std::size_t i = 0; i < pat.size(); ++i) n += kGensN[pat[i]];
    return n;
}

void convEncode(const unsigned char* bits, std::size_t nbits, const Pattern& pat, Bytes& out) {
    out.clear();
    out.reserve(static_cast<std::size_t>(patternBits(pat)));
    int reg = 0;
    for (std::size_t st = 0; st < pat.size(); ++st) {
        reg = ((reg << 1) | (st < nbits ? (bits[st] & 1) : 0)) & 0x7f;
        const int code = pat[st];
        for (int j = 0; j < kGensN[code]; ++j) out.push_back(kPar.p[reg & kGen[kGens[code][j]]]);
    }
}

// soft: positive means 0.  64 states, the path ending in state 0 (the tail
// steps feed zeros).
void Viterbi::decode(const double* soft, const Pattern& pat, Bytes& bits) {
    const std::size_t n = pat.size();
    prv_.resize(n * 64);
    bit_.resize(n * 64);
    const double inf = std::numeric_limits<double>::infinity();
    double m[64], nm[64];
    for (int s = 0; s < 64; ++s) m[s] = inf;
    m[0] = 0;
    std::size_t pos = 0;
    for (std::size_t st = 0; st < n; ++st) {
        const int code = pat[st], ng = kGensN[code];
        const bool tail = st + 6 >= n;
        unsigned char* pv = &prv_[st * 64];
        unsigned char* bv = &bit_[st * 64];
        for (int s = 0; s < 64; ++s) nm[s] = inf;
        for (int s = 0; s < 64; ++s) {
            if (m[s] == inf) continue;
            for (int b = 0; b <= (tail ? 0 : 1); ++b) {
                const int reg = ((s << 1) | b) & 0x7f;
                double c = m[s];
                for (int j = 0; j < ng; ++j) c -= soft[pos + j] * (1 - 2 * kPar.p[reg & kGen[kGens[code][j]]]);
                const int ns = reg & 0x3f;
                if (c < nm[ns]) {
                    nm[ns] = c;
                    pv[ns] = static_cast<unsigned char>(s);
                    bv[ns] = static_cast<unsigned char>(b);
                }
            }
        }
        pos += static_cast<std::size_t>(ng);
        // (normalised: the best metric back to 0, so long frames keep precision)
        double lo = inf;
        for (int s = 0; s < 64; ++s)
            if (nm[s] < lo) lo = nm[s];
        for (int s = 0; s < 64; ++s) m[s] = (nm[s] == inf || lo == inf) ? nm[s] : nm[s] - lo;
    }
    const std::size_t nb = n >= 6 ? n - 6 : 0;
    bits.assign(nb, 0);
    int s = 0;
    for (std::size_t st = n; st-- > 0;) {
        if (st < nb) bits[st] = bit_[st * 64 + s];
        s = prv_[st * 64 + s];
    }
}

void prbs(std::size_t n, Bytes& out) {
    out.resize(n);
    int r = 0x1ff;
    for (std::size_t i = 0; i < n; ++i) {
        const int b = ((r >> 4) ^ (r >> 8)) & 1;
        r = ((r << 1) | b) & 0x1ff;
        out[i] = static_cast<unsigned char>(b);
    }
}

unsigned crcDream(const unsigned char* bytes, std::size_t n, int deg) {
    const unsigned mask = deg == 8 ? ((1u << 2) | (1u << 3) | (1u << 4)) : ((1u << 5) | (1u << 12));
    unsigned reg = 0xffffffffu;
    for (std::size_t q = 0; q < n; ++q)
        for (int i = 0; i < 8; ++i) {
            reg <<= 1;
            if (reg & (1u << deg)) reg |= 1;
            if (bytes[q] & (1 << (7 - i))) reg ^= 1;
            if (reg & 1) reg ^= mask;
        }
    return (~reg) & ((1u << deg) - 1);
}

unsigned crc16(const unsigned char* b, std::size_t n) {
    unsigned c = 0xffff;
    for (std::size_t i = 0; i < n; ++i) {
        c ^= static_cast<unsigned>(b[i]) << 8;
        for (int j = 0; j < 8; ++j) c = (c & 0x8000) ? (((c << 1) ^ 0x1021) & 0xffff) : ((c << 1) & 0xffff);
    }
    return c ^ 0xffff;
}

unsigned bitsNum(const unsigned char* b, std::size_t at, int n) {
    unsigned v = 0;
    for (int i = 0; i < n; ++i) v = v * 2 + (b[at + static_cast<std::size_t>(i)] & 1);
    return v;
}

void gainRefs(int mi, int s, int kmin, int kmax, std::vector<IntPair>& out) {
    out.clear();
    const ModeTable& t = modeTable(mi);
    const int x = t.scat[0], y = t.scat[1], k0 = t.scat[2], cols = t.nW / y, xy = x * y;
    const int n = s % y, m = s / y;
    for (int k = kmin; k <= kmax; ++k) {
        if ((((k - (x + 1) / 2 - x * (s % y)) % xy) + xy) % xy != 0) continue;
        const int p = static_cast<int>(std::floor((k - k0 - n * x) / static_cast<double>(xy) + 0.5));
        const long long v = 4LL * t.Z[n * cols + m] + static_cast<long long>(p) * t.W[n * cols + m] +
                            static_cast<long long>(p) * p * (1 + s) * t.Q;
        out.push_back(IntPair(k, static_cast<int>(((v % 1024) + 1024) % 1024)));
    }
}

// ---- the DFT of any length --------------------------------------------------
void Dft::init(int n) {
    n_ = n;
    pow2_ = n > 0 && (n & (n - 1)) == 0;
    if (pow2_) {
        m_ = n;
        fft_.resize(n);
        a_.assign(static_cast<std::size_t>(n), std::complex<float>());
        return;
    }
    int M = 1;
    while (M < 2 * n - 1) M <<= 1;
    m_ = M;
    fft_.resize(M);
    w_.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const double a = kPi * static_cast<double>((static_cast<long long>(i) * i) % (2LL * n)) / n;
        w_[static_cast<std::size_t>(i)] = std::complex<double>(std::cos(a), -std::sin(a));
    }
    b_.assign(static_cast<std::size_t>(M), std::complex<float>());
    for (int i = 0; i < n; ++i) {
        const std::complex<double> c = std::conj(w_[static_cast<std::size_t>(i)]);
        b_[static_cast<std::size_t>(i)] = std::complex<float>(c);
        if (i) b_[static_cast<std::size_t>(M - i)] = std::complex<float>(c);
    }
    fft_.forward(&b_[0]);
    a_.assign(static_cast<std::size_t>(M), std::complex<float>());
}

void Dft::forward(const std::complex<double>* x, std::complex<double>* out) {
    if (pow2_) {
        for (int i = 0; i < n_; ++i) a_[static_cast<std::size_t>(i)] = std::complex<float>(x[i]);
        fft_.forward(&a_[0]);
        for (int i = 0; i < n_; ++i) out[i] = std::complex<double>(a_[static_cast<std::size_t>(i)]);
        return;
    }
    const std::size_t M = static_cast<std::size_t>(m_);
    for (std::size_t i = 0; i < M; ++i) a_[i] = std::complex<float>();
    for (int i = 0; i < n_; ++i) a_[static_cast<std::size_t>(i)] = std::complex<float>(x[i] * w_[static_cast<std::size_t>(i)]);
    fft_.forward(&a_[0]);
    // the convolution with the chirp, then back (an inverse FFT by conjugation)
    for (std::size_t i = 0; i < M; ++i) a_[i] = std::conj(a_[i] * b_[i]);
    fft_.forward(&a_[0]);
    const double sc = 1.0 / m_;
    for (int i = 0; i < n_; ++i)
        out[i] = std::conj(std::complex<double>(a_[static_cast<std::size_t>(i)])) * sc * w_[static_cast<std::size_t>(i)];
}

void Dft::inverse(const std::complex<double>* x, std::complex<double>* out) {
    // conj(DFT(conj(x))): the inverse transform without the 1/N
    for (int i = 0; i < n_; ++i) out[i] = std::conj(x[i]);
    forward(out, out);
    for (int i = 0; i < n_; ++i) out[i] = std::conj(out[i]);
}

}  // namespace drm
}  // namespace dec
