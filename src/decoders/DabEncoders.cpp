// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DabEncoders.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The demo DAB multiplex (DabEncoders.h): the FIC (FIGs 0/0, 1/0, 0/1,
//  0/13, 0/17, 0/2, 0/3, 1/1 in 12 FIBs with CRCs), energy dispersal,
//  the convolutional code, puncturing (the FIC's and each sub-channel's
//  EEP / UEP profile), time interleaving over the 480-CIF loop, then per
//  frame: the null symbol, the phase reference and 75 DQPSK symbols
//  (frequency interleaved) by inverse FFT, each with its guard interval.
//
//  How it fits in
//  --------------
//  A port of OFFgrid-SDR's JS (diddy-boy, https://github.com/solariun/
//  offgrid-sdr, index.html, "dablib": dabDemoSignal: conv, punct, bitsOf,
//  fig1, fig0, the FIB packing, ficGroup0, prep, frame).  The tables are
//  the receiver's (DabInternal.h, DabFec.cpp); the programme data
//  DabDemoData.cpp's.  Additions to the JS: a continuous generate(); a
//  UEP sub-channel whose punctured length falls short of its CUs is padded
//  with zeros explicitly (the JS's typed array did so implicitly).
//
//  Contributing
//  ------------
//  Keep in step with dabDemoSignal.  Run tests/test_decoders_dab.cpp.
// ============================================================================
#include "DabEncoders.h"

#include <cmath>
#include <cstring>

#include "DabInternal.h"
#include "dsp/Fft.h"

namespace dec {

using namespace dab;

namespace {

const int NC = 480;   // CIFs in the loop (120 frames)

typedef std::vector<unsigned char> Bytes;

// the convolutional code, 6 tail bits (JS conv)
Bytes conv(const Bytes& bits) {
    const Tables& T = tables();
    const std::size_t n = bits.size();
    Bytes out(4 * (n + 6));
    int reg = 0;
    for (std::size_t t = 0; t < n + 6; ++t) {
        const int b = t < n ? bits[t] : 0;
        reg = ((b << 6) | (reg >> 1)) & 0x7f;
        for (int j = 0; j < 4; ++j) out[4 * t + static_cast<std::size_t>(j)] = T.out[reg * 4 + j] > 0 ? 1 : 0;
    }
    return out;
}

// puncturing (JS punct)
Bytes punct(const Bytes& c, const Section* s, int ns) {
    const Tables& T = tables();
    Bytes out;
    std::size_t loc = 0;
    for (int k = 0; k < ns; ++k) {
        const unsigned char* pat = T.pcodes[s[k].pattern - 1];
        for (int i = 0; i < s[k].blocks; ++i)
            for (int j = 0; j < 128; ++j, ++loc)
                if (pat[j & 31]) out.push_back(c[loc]);
    }
    for (int k = 0; k < 24; ++k, ++loc)
        if (T.pix[k]) out.push_back(c[loc]);
    return out;
}

Bytes bitsOf(const unsigned char* by, std::size_t n) {
    Bytes b(n * 8);
    for (std::size_t i = 0; i < b.size(); ++i) b[i] = (by[i >> 3] >> (7 - (i & 7))) & 1;
    return b;
}

const unsigned EID = 0x0ff6;

Bytes fig1(int ext, unsigned id, const char* t) {
    Bytes g;
    g.push_back((1 << 5) | 21);
    g.push_back(static_cast<unsigned char>(ext));
    g.push_back(static_cast<unsigned char>(id >> 8));
    g.push_back(static_cast<unsigned char>(id & 255));
    const std::size_t n = std::strlen(t);
    for (std::size_t i = 0; i < 16; ++i) g.push_back(i < n ? static_cast<unsigned char>(t[i]) : 32);
    g.push_back(0xff);
    g.push_back(0x00);
    return g;
}
Bytes fig0(int ext, const unsigned char* body, std::size_t n) {
    Bytes g;
    g.push_back(static_cast<unsigned char>((0 << 5) | (n + 1)));
    g.push_back(static_cast<unsigned char>(ext));
    g.insert(g.end(), body, body + n);
    return g;
}

const Section kFicSec[2] = {{16, 21}, {15, 3}};

}  // namespace

struct DabDemoMultiplex::Impl {
    Bytes plus, mp2, spiData;
    std::vector<Bytes> fibs;
    Bytes fic;    // 9216 coded bits
    int c00;      // FIG 0/0's CIF count byte in FIB 0, or -1
    struct Chan {
        int start;
        std::vector<Bytes> il;   // NC interleaved CIFs, size * 64 bits each
    };
    std::vector<Chan> chans;
    dsp::detail::Fft fft;
    std::vector<Cf> x;
    Bytes bits;
    std::vector<std::complex<float> > cur;
    long curK;

    Impl() : c00(-1), fft(Tu), curK(-1) {
        plus = demoPlus();
        mp2 = demoMp2();
        spiData = demoSpi();
        plus.resize(static_cast<std::size_t>(NC) * 144, 0);
        mp2.resize(static_cast<std::size_t>(NC) * 144, 0);
        spiData.resize(static_cast<std::size_t>(NC) * 48, 0);
        buildFic();
        // the stations: each CIF's bytes scrambled, coded and punctured, then time-interleaved over the loop
        const Section plusSec[2] = {{8, 6 * 6 - 3}, {7, 3}};   // EEP A-3, 48 kbit/s (n = 6)
        prep(0, 36, plusSec, 2, 1152, plus, 144);
        const Section spiSec[2] = {{8, 6 * 2 - 3}, {7, 3}};    // the logos, 16 kbit/s
        prep(71, 12, spiSec, 2, 384, spiData, 48);
        int up[8];
        uepProfile(48, 3, up);
        Section uepSec[4];
        int nu = 0;
        for (int i = 0; i < 4; ++i)
            if (i < 3 || up[3]) {
                uepSec[nu].pattern = up[4 + i];
                uepSec[nu].blocks = up[i];
                ++nu;
            }
        prep(36, kUep[7].cus, uepSec, nu, 48 * 24, mp2, 144);
        x.resize(Tu);
        bits.resize(static_cast<std::size_t>(75) * kSymBits);
    }

    // the FIC: FIG 0/0, 1/0 (the multiplex's name), 0/1 (sub-channels), 0/2 (services), 1/1 (names), in 12 FIBs
    // (the multiplex's ID 0x0FF6: an ID's first digit is its country, and 0 belongs to none)
    void buildFic() {
        std::vector<Bytes> figs;
        figs.push_back(fig1(0, EID, "OFFgrid DAB"));
        {
            const unsigned char b[] = {EID >> 8, EID & 255, 0, 0};
            figs.push_back(fig0(0, b, sizeof b));
        }
        {   // sub-channel 1: CU 0, long form, EEP A-3, 36 CUs (48 kbit/s: room for text and a SlideShow)
            const unsigned char b[] = {(1 << 2) | 0, 0, 0x80 | (0 << 4) | (2 << 2) | 0, 36};
            figs.push_back(fig0(1, b, sizeof b));
        }
        {   // sub-channel 2: CU 36, short form, UEP index 7 (48 kbit/s, level 3, 35 CUs)
            const unsigned char b[] = {(2 << 2) | 0, 36, 7};
            figs.push_back(fig0(1, b, sizeof b));
        }
        {   // OFFgrid Plus's application: 2, the SlideShow, in its X-PAD (type 12, MOT)
            const unsigned char b[] = {0xf0, 0xd1, 0x01, 0x00, 0x42, 0x0c, 0x3c};
            figs.push_back(fig0(13, b, sizeof b));
        }
        {   // programme types: Pop Music, Other Music
            const unsigned char b[] = {0xf0, 0xd1, 0x00, 10, 0xf0, 0xd2, 0x00, 15};
            figs.push_back(fig0(17, b, sizeof b));
        }
        {   // the station logos: sub-channel 3: CU 71, EEP A-3, 12 CUs (16 kbit/s)
            const unsigned char b[] = {(3 << 2) | 0, 71, 0x80 | (0 << 4) | (2 << 2) | 0, 12};
            figs.push_back(fig0(1, b, sizeof b));
        }
        {   // data service E1F0D9FF: one packet-mode component, SCId 1
            const unsigned char b[] = {0xe1, 0xf0, 0xd9, 0xff, 1, 0xc0, 0x06};
            figs.push_back(fig0(0x20 | 2, b, sizeof b));
        }
        {   // SCId 1: MOT (DSCTy 60), sub-channel 3, packet address 1
            const unsigned char b[] = {0x00, 0x10, 60, (3 << 2) | 0, 1};
            figs.push_back(fig0(3, b, sizeof b));
        }
        {   // its application: 7, SPI
            const unsigned char b[] = {0xe1, 0xf0, 0xd9, 0xff, 0x01, 0x00, 0xe1, 0x01};
            figs.push_back(fig0(0x20 | 13, b, sizeof b));
        }
        {   // services: DAB+ (ASCTy 63) on 1; DAB (MP2) on 2
            const unsigned char b1[] = {0xf0, 0xd1, 1, 63, (1 << 2) | 2};
            figs.push_back(fig0(2, b1, sizeof b1));
            const unsigned char b2[] = {0xf0, 0xd2, 1, 0, (2 << 2) | 2};
            figs.push_back(fig0(2, b2, sizeof b2));
        }
        figs.push_back(fig1(1, 0xf0d1, "OFFgrid Plus"));
        figs.push_back(fig1(1, 0xf0d2, "OFFgrid Classic"));
        std::vector<Bytes> cur;
        std::size_t used = 0;
        for (std::size_t i = 0; i < figs.size(); ++i) {
            if (used + figs[i].size() > 30) {
                flush(cur);
                used = 0;
            }
            cur.push_back(figs[i]);
            used += figs[i].size();
        }
        flush(cur);
        while (fibs.size() < 12) flush(cur);
        // FIG 0/0's CIF count byte in the first FIB
        const Bytes& f0 = fibs[0];
        for (int p = 0; p < 30 && f0[static_cast<std::size_t>(p)] != 0xff;) {
            const int ln = f0[static_cast<std::size_t>(p)] & 31;
            if (!ln) break;
            if ((f0[static_cast<std::size_t>(p)] >> 5) == 0 && (f0[static_cast<std::size_t>(p) + 1] & 31) == 0) {
                c00 = p + 4;
                break;
            }
            p += 1 + ln;
        }
        fic.resize(kFicBits);
        for (int b = 0; b < 4; ++b) {
            const Bytes coded = ficBlock(b, 0, false);
            std::memcpy(&fic[static_cast<std::size_t>(b) * 2304], &coded[0], 2304);
        }
    }
    void flush(std::vector<Bytes>& cur) {
        Bytes d(32, 0);
        std::size_t p = 0;
        for (std::size_t i = 0; i < cur.size(); ++i) {
            std::memcpy(&d[p], &cur[i][0], cur[i].size());
            p += cur[i].size();
        }
        if (p < 30) d[p] = 0xff;
        const unsigned c = crc16(&d[0], 30);
        d[30] = static_cast<unsigned char>(c >> 8);
        d[31] = static_cast<unsigned char>(c & 255);
        fibs.push_back(d);
        cur.clear();
    }
    // a FIB group (3 FIBs), scrambled, coded, punctured: 2304 bits; group 0 with the CIF count when `count`
    Bytes ficBlock(int b, int cif, bool count) {
        const Tables& T = tables();
        Bytes u;
        u.reserve(768);
        for (int f = 0; f < 3; ++f) {
            Bytes fib = fibs[static_cast<std::size_t>(b * 3 + f) % fibs.size()];
            if (count && f == 0 && c00 >= 0) {
                fib[static_cast<std::size_t>(c00)] = static_cast<unsigned char>((fib[static_cast<std::size_t>(c00)] & 0xe0) | (cif / 250));
                fib[static_cast<std::size_t>(c00) + 1] = static_cast<unsigned char>(cif % 250);
                const unsigned c = crc16(&fib[0], 30);
                fib[30] = static_cast<unsigned char>(c >> 8);
                fib[31] = static_cast<unsigned char>(c & 255);
            }
            const Bytes bb = bitsOf(&fib[0], 32);
            u.insert(u.end(), bb.begin(), bb.end());
        }
        for (int i = 0; i < 768; ++i) u[static_cast<std::size_t>(i)] ^= T.prbs768[i];
        return punct(conv(u), kFicSec, 2);
    }
    void prep(int start, int size, const Section* sec, int ns, int nout, const Bytes& data, int cifBytes) {
        Bytes pr;
        prbs(nout, pr);
        std::vector<Bytes> coded(NC);
        for (int c = 0; c < NC; ++c) {
            Bytes u = bitsOf(&data[static_cast<std::size_t>(c) * static_cast<std::size_t>(cifBytes)], static_cast<std::size_t>(cifBytes));
            u.resize(static_cast<std::size_t>(nout), 0);
            for (int i = 0; i < nout; ++i) u[static_cast<std::size_t>(i)] ^= pr[static_cast<std::size_t>(i)];
            coded[static_cast<std::size_t>(c)] = punct(conv(u), sec, ns);
            coded[static_cast<std::size_t>(c)].resize(static_cast<std::size_t>(size) * 64, 0);   // (padded, as the JS's typed array)
        }
        Chan ch;
        ch.start = start;
        ch.il.resize(NC);
        const std::size_t n = static_cast<std::size_t>(size) * 64;
        for (int c = 0; c < NC; ++c) {
            Bytes& o = ch.il[static_cast<std::size_t>(c)];
            o.resize(n);
            for (std::size_t i = 0; i < n; ++i) o[i] = coded[static_cast<std::size_t>((c - kImap[i & 15] + NC) % NC)][i];
        }
        chans.push_back(Chan());
        chans.back().start = ch.start;
        chans.back().il.swap(ch.il);
    }

    // a frame: null, phase reference, 75 DQPSK symbols (FIC, then 4 CIFs), by inverse FFT (JS frame)
    void frame(long k, std::vector<std::complex<float> >& out) {
        const Tables& T = tables();
        std::memcpy(&bits[0], &fic[0], kFicBits);
        {
            const Bytes g0 = ficBlock(0, static_cast<int>((4 * k) % 5000), true);
            std::memcpy(&bits[0], &g0[0], 2304);
        }
        // (the filler: other stations' data, random; 32 bits a draw)
        unsigned seed = 0x9e3779b9u ^ static_cast<unsigned>(static_cast<unsigned long long>(k + 1) * 2654435761ull);
        for (std::size_t i = kFicBits; i < bits.size(); i += 32) {
            seed = seed * 1664525u + 1013904223u;
            unsigned w = seed;
            for (std::size_t j = 0; j < 32 && i + j < bits.size(); ++j, w >>= 1) bits[i + j] = w & 1;
        }
        for (int c = 0; c < 4; ++c)
            for (std::size_t q = 0; q < chans.size(); ++q) {
                const Bytes& src = chans[q].il[static_cast<std::size_t>((4 * k + c) % NC)];
                std::memcpy(&bits[static_cast<std::size_t>(kFicBits) + static_cast<std::size_t>(c) * kCifBits +
                                  static_cast<std::size_t>(chans[q].start) * 64],
                            &src[0], src.size());
            }
        out.assign(static_cast<std::size_t>(FR), std::complex<float>(0, 0));
        double zr[K], zi[K];
        for (int q = 0; q < K; ++q) {
            zr[q] = T.prs[q].real();
            zi[q] = T.prs[q].imag();
        }
        int o = Tn;
        symbol(zr, zi, out, o);
        o += Ts;
        const double r2 = 1 / std::sqrt(2.0);
        for (int l = 0; l < 75; ++l) {
            const std::size_t off = static_cast<std::size_t>(l) * kSymBits;
            for (int i = 0; i < K; ++i) {
                const int q = T.perm[i];
                const double yr = bits[off + static_cast<std::size_t>(i)] ? -1 : 1, yi = bits[off + K + static_cast<std::size_t>(i)] ? -1 : 1;
                const double a = zr[q], b = zi[q];
                zr[q] = (a * yr - b * yi) * r2;
                zi[q] = (a * yi + b * yr) * r2;
            }
            symbol(zr, zi, out, o);
            o += Ts;
        }
    }
    // (inverse FFT: conjugate, FFT, conjugate), then the guard interval
    void symbol(const double* zr, const double* zi, std::vector<std::complex<float> >& out, int o) {
        const Tables& T = tables();
        for (int j = 0; j < Tu; ++j) x[j] = Cf(0, 0);
        for (int q = 0; q < K; ++q) x[T.bin[q]] = Cf(static_cast<float>(zr[q]), static_cast<float>(-zi[q]));
        fft.forward(&x[0]);
        const float s = static_cast<float>(1 / std::sqrt(static_cast<double>(Tu)));
        for (int j = 0; j < Tu; ++j) out[static_cast<std::size_t>(o + Tg + j)] = std::complex<float>(x[j].real() * s, -x[j].imag() * s);
        for (int j = 0; j < Tg; ++j) out[static_cast<std::size_t>(o + j)] = out[static_cast<std::size_t>(o + Tu + j)];
    }
};

DabDemoMultiplex::DabDemoMultiplex() : impl_(new Impl), pos_(0) {}
DabDemoMultiplex::~DabDemoMultiplex() {}

void DabDemoMultiplex::frame(long k, std::vector<std::complex<float> >& out) { impl_->frame(k, out); }

void DabDemoMultiplex::generate(std::complex<float>* out, std::size_t n) {
    Impl& m = *impl_;
    while (n) {
        const long k = static_cast<long>(pos_ / FR);
        const std::size_t at = static_cast<std::size_t>(pos_ % FR);
        if (m.curK != k) {
            m.frame(k, m.cur);
            m.curK = k;
        }
        std::size_t take = static_cast<std::size_t>(FR) - at;
        if (take > n) take = n;
        std::memcpy(out, &m.cur[at], take * sizeof(std::complex<float>));
        out += take;
        n -= take;
        pos_ += take;
    }
}

const std::vector<unsigned char>& DabDemoMultiplex::plusData() const { return impl_->plus; }
const std::vector<unsigned char>& DabDemoMultiplex::mp2Data() const { return impl_->mp2; }
const std::vector<unsigned char>& DabDemoMultiplex::spiData() const { return impl_->spiData; }

}  // namespace dec
