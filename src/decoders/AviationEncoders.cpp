// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/AviationEncoders.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  ACARS MSK audio and VDL2 D8PSK bursts: see AviationEncoders.h.  Port of
//  OFFgrid-SDR's JS acarsBits / acarsAudio and its VDL Mode 2 transmitter
//  (vdlAddr, vdlAcars, vdlFrame, vdlBurstSymbols, vdlModulate) and demo
//  plans (diddy-boy, https://github.com/solariun/offgrid-sdr; the VDL2 side
//  "from the standard; checked against dumpvdl2").  Same arithmetic as the
//  JS; the only addition is vdl2AddBurst(), which places a burst in an IQ
//  block (the JS demo did that inline, from a 105 kHz copy).
//
//  How it fits in
//  --------------
//  Implements AviationEncoders.h with Acars.h (CRC, parity) and Vdl2.h (RS,
//  CRC, header parity, preamble, Gray map).  Standard library only.
//
//  Contributing
//  ------------
//  Deterministic output only.  Strict C++11.
// ============================================================================
#include "AviationEncoders.h"

#include "Acars.h"
#include "Vdl2.h"

#include <cmath>

namespace dec {

namespace {

const double kPi = 3.141592653589793;
const double kTwoPi = 6.283185307179586;

// JS s.padStart(len, pad).slice(-len)
std::string padStartLast(const std::string& s, std::size_t len, char pad) {
    std::string r = s.size() < len ? std::string(len - s.size(), pad) + s : s;
    return r.substr(r.size() - len);
}
// JS s.padEnd(len, pad).slice(0, len)
std::string padEndFirst(const std::string& s, std::size_t len, char pad) {
    std::string r = s;
    if (r.size() < len) r += std::string(len - r.size(), pad);
    return r.substr(0, len);
}

int fecOf(int L) { return L < 3 ? 0 : L < 31 ? 2 : L < 68 ? 4 : 6; }

}  // namespace

// ===========================================================================
// ACARS
// ===========================================================================
std::vector<unsigned char> acarsBits(const AcarsTx& m) {
    const std::string head = m.mode + padStartLast(m.reg, 7, '.') + m.ack + m.label + m.bid;
    std::vector<unsigned char> body;
    for (std::size_t i = 0; i < head.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(head[i]);
        body.push_back(c == 0x7f ? 0x7f : acarsOddParity(c));
    }
    if (!m.text.empty() || !m.msgno.empty()) {
        body.push_back(0x02);
        const std::string t = m.msgno + padEndFirst(m.flight, 6, ' ') + m.text;
        for (std::size_t i = 0; i < t.size(); ++i) body.push_back(acarsOddParity(static_cast<unsigned char>(t[i])));
    }
    body.push_back(0x83);
    const unsigned crc = acarsCrc(&body[0], body.size());
    std::vector<unsigned char> bytes;
    const unsigned char pre[5] = {0xab, 0x2a, 0x16, 0x16, 0x01};
    bytes.assign(pre, pre + 5);
    bytes.insert(bytes.end(), body.begin(), body.end());
    bytes.push_back(static_cast<unsigned char>(crc & 0xff));
    bytes.push_back(static_cast<unsigned char>(crc >> 8));
    bytes.push_back(0x7f);
    std::vector<unsigned char> bits(128, 1);   // pre-key
    for (std::size_t i = 0; i < bytes.size(); ++i)
        for (int k = 0; k < 8; ++k) bits.push_back(static_cast<unsigned char>((bytes[i] >> k) & 1));
    return bits;
}

// MSK audio: 2400 bits a second, built as offset-QPSK with half-cosine
// pulses around 1800 Hz (the same structure the decoder expects).
std::size_t synthAcars(const std::vector<unsigned char>& bits, double ar, std::vector<float>& out, double lead) {
    if (!(ar > 0)) return 0;
    const double T = 1.0 / 2400;
    const long nb = static_cast<long>(bits.size());
    const std::size_t n = static_cast<std::size_t>(std::ceil((lead + (nb + 2) * T) * ar));
    const std::size_t base = out.size();
    out.resize(base + n, 0.0f);
    for (std::size_t i = 0; i < n; ++i) {
        const double t = i / ar - lead;
        if (t < -T) continue;
        const long k0 = static_cast<long>(std::floor(t / T));
        double zr = 0, zi = 0;
        for (long k = k0; k <= k0 + 1; ++k) {
            if (k < 0 || k >= nb) continue;
            const double d = (t - k * T) / T;
            if (std::fabs(d) >= 1) continue;
            const double c = (bits[static_cast<std::size_t>(k)] ? 1 : -1) * ((k & 2) ? -1 : 1) * std::cos(kPi * d / 2);
            if (k & 1) zi += c;
            else zr += c;
        }
        const double ph = kTwoPi * 1800 * t;
        out[base + i] = static_cast<float>(zr * std::cos(ph) - zi * std::sin(ph));
    }
    return n;
}

// Airband: ACARS on 131.550 MHz.  Four short bursts in a 30 s loop: a
// downlink with text, the ground's acknowledgement, a weather uplink, and a
// ground station squitter.  Registrations and flights are made up.
std::vector<AcarsDemoItem> acarsDemoMessages() {
    std::vector<AcarsDemoItem> v(4);
    v[0].at = 1.5;
    v[0].msg.reg = "G-OFFG";
    v[0].msg.label = "H1";
    v[0].msg.bid = "3";
    v[0].msg.msgno = "M01A";
    v[0].msg.flight = "OG0123";
    v[0].msg.text = "OFFGRID-SDR DEMO. CRUISE FL350, FUEL 12.4T, ETA 1435Z";
    v[1].at = 4.0;
    v[1].msg.reg = "G-OFFG";
    v[1].msg.ack = "3";
    v[1].msg.label = "_\x7f";
    v[1].msg.bid = "A";
    v[2].at = 12.0;
    v[2].msg.reg = "G-OFFG";
    v[2].msg.label = "H1";
    v[2].msg.bid = "B";
    v[2].msg.text = "WX REQ RESPONSE\nDEST 141420Z 24012KT 9999 SCT030 15/09 Q1018";
    v[3].at = 21.0;
    v[3].msg.reg = "";
    v[3].msg.label = "SQ";
    v[3].msg.bid = " ";
    v[3].msg.text = "02XAOFFGRID DEMO GROUND STATION";
    return v;
}

void acarsDemoLoop(double rate, std::vector<float>& audio, std::vector<float>& gate) {
    const std::size_t N = static_cast<std::size_t>(30 * rate);
    audio.assign(N, 0.0f);
    gate.assign(N, 0.0f);
    const std::vector<AcarsDemoItem> msgs = acarsDemoMessages();
    std::vector<float> a;
    const long g = static_cast<long>(std::floor(0.02 * rate + 0.5));
    for (std::size_t m = 0; m < msgs.size(); ++m) {
        a.clear();
        synthAcars(acarsBits(msgs[m].msg), rate, a);
        const long i0 = static_cast<long>(std::floor(msgs[m].at * rate + 0.5));
        for (long i = 0; i < static_cast<long>(a.size()) && i0 + i < static_cast<long>(N); ++i)
            audio[static_cast<std::size_t>(i0 + i)] = a[static_cast<std::size_t>(i)];
        for (long i = i0 - g < 0 ? 0 : i0 - g; i < static_cast<long>(N) && i < i0 + static_cast<long>(a.size()) + g; ++i)
            gate[static_cast<std::size_t>(i)] = 1.0f;
    }
}

// ===========================================================================
// VDL2
// ===========================================================================
// (24-bit address, 3-bit type above it, then a status bit; reversed; 7 bits
//  a byte in bits 1-7)
Bytes vdl2Address(unsigned addr24, int type, int status, bool last) {
    const unsigned v = (addr24 & 0xffffffu) | ((static_cast<unsigned>(type) & 7u) << 24) |
                       ((static_cast<unsigned>(status) & 1u) << 27);
    unsigned r = 0;
    for (int i = 0; i < 28; ++i)
        if ((v >> i) & 1) r |= 1u << (27 - i);
    Bytes b(4);
    for (int k = 0; k < 4; ++k)
        b[static_cast<std::size_t>(k)] = static_cast<unsigned char>((((r >> (7 * k)) & 0x7f) << 1) | (last && k == 3 ? 1 : 0));
    return b;
}

// an ACARS message for ACARS over AVLC: odd parity on each character, ETX,
// the CRC (CRC-CCITT from 0, low byte first), DEL
Bytes vdl2AcarsInfo(const std::string& text) {
    Bytes body;
    for (std::size_t i = 0; i < text.size(); ++i) body.push_back(acarsOddParity(static_cast<unsigned char>(text[i])));
    body.push_back(acarsOddParity(0x03));
    const unsigned c = acarsCrc(&body[0], body.size());
    Bytes out;
    out.push_back(0xff);
    out.push_back(0xff);
    out.push_back(0x01);
    out.insert(out.end(), body.begin(), body.end());
    out.push_back(static_cast<unsigned char>(c & 255));
    out.push_back(static_cast<unsigned char>(c >> 8));
    out.push_back(0x7f);
    return out;
}

std::string vdl2AcarsDownlink(const std::string& reg, const std::string& flight, const std::string& label,
                              const std::string& block, const std::string& msgNo, const std::string& text) {
    return "2" + padStartLast("." + reg, 7, '.') + "\x15" + label + block + "\x02" + msgNo + padEndFirst(flight, 6, ' ') +
           text;
}

Bytes vdl2Frame(const Bytes& dst, const Bytes& src, unsigned char control, const Bytes& info) {
    Bytes f(dst);
    f.insert(f.end(), src.begin(), src.end());
    f.push_back(control);
    f.insert(f.end(), info.begin(), info.end());
    const unsigned c = vdl2Crc(&f[0], f.size());
    f.push_back(static_cast<unsigned char>(c & 255));
    f.push_back(static_cast<unsigned char>(c >> 8));
    return f;
}

// a burst: frames -> HDLC bits (flags, stuffing) -> octets -> RS blocks,
// interleaved -> header + scrambler -> symbols
std::vector<int> vdl2BurstSymbols(const std::vector<Bytes>& frames) {
    std::vector<unsigned char> bits;
    for (int k = 0; k < 8; ++k) bits.push_back(static_cast<unsigned char>((0x7e >> k) & 1));
    for (std::size_t fi = 0; fi < frames.size(); ++fi) {
        int ones = 0;
        const Bytes& f = frames[fi];
        for (std::size_t i = 0; i < f.size(); ++i)
            for (int k = 0; k < 8; ++k) {
                const unsigned char v = static_cast<unsigned char>((f[i] >> k) & 1);
                bits.push_back(v);
                if (v) {
                    if (++ones == 5) {
                        bits.push_back(0);
                        ones = 0;
                    }
                } else {
                    ones = 0;
                }
            }
        for (int k = 0; k < 8; ++k) bits.push_back(static_cast<unsigned char>((0x7e >> k) & 1));
    }
    const int datalen = static_cast<int>(bits.size()), octets = (datalen + 7) / 8;
    Bytes data(static_cast<std::size_t>(octets), 0);
    for (int i = 0; i < datalen; ++i) data[static_cast<std::size_t>(i >> 3)] |= static_cast<unsigned char>(bits[static_cast<std::size_t>(i)] << (i & 7));
    const int nb = (octets + 248) / 249, lastLen = octets - (nb - 1) * 249;
    std::vector<Bytes> blocks(static_cast<std::size_t>(nb), Bytes(249, 0));
    int row = 0, col = 0;
    for (int i = 0; i < octets; ++i) {
        if (row == nb - 1 && col >= lastLen) {
            row = 0;
            ++col;
        }
        blocks[static_cast<std::size_t>(row++)][static_cast<std::size_t>(col)] = data[static_cast<std::size_t>(i)];
        if (row == nb) {
            row = 0;
            ++col;
        }
    }
    std::vector<Bytes> fecs(static_cast<std::size_t>(nb));
    for (int r = 0; r < nb; ++r) {
        unsigned char p[6];
        vdl2RsEncode(&blocks[static_cast<std::size_t>(r)][0], p);
        fecs[static_cast<std::size_t>(r)].assign(p, p + (r == nb - 1 ? fecOf(lastLen) : 6));
    }
    const int fecRows = fecOf(lastLen) ? nb : nb - 1;
    Bytes fec;
    for (int c = 0; c < 6; ++c)   // (round-robin, as the data)
        for (int r = 0; r < fecRows; ++r)
            if (c < static_cast<int>(fecs[static_cast<std::size_t>(r)].size())) fec.push_back(fecs[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)]);
    unsigned hdr = 0;
    for (int i = 0; i < 17; ++i)   // (the length sent LSB first)
        if ((datalen >> i) & 1) hdr |= 1u << (16 - i);
    hdr <<= 5;
    hdr |= vdl2HeaderSyndrome(hdr);   // (each parity bit makes its check even)
    std::vector<unsigned char> out;
    for (int i = 24; i >= 0; --i) out.push_back(static_cast<unsigned char>((hdr >> i) & 1));
    for (std::size_t i = 0; i < data.size(); ++i)
        for (int k = 0; k < 8; ++k) out.push_back(static_cast<unsigned char>((data[i] >> k) & 1));
    for (std::size_t i = 0; i < fec.size(); ++i)
        for (int k = 0; k < 8; ++k) out.push_back(static_cast<unsigned char>((fec[i] >> k) & 1));
    unsigned lfsr = 0x6959;
    for (std::size_t i = 0; i < out.size(); ++i) {
        const unsigned b = (lfsr ^ (lfsr >> 14)) & 1;
        lfsr = (lfsr >> 1) | (b << 14);
        out[i] ^= static_cast<unsigned char>(b);
    }
    while (out.size() % 3) out.push_back(0);
    std::vector<int> syms(vdl2Preamble, vdl2Preamble + 16);
    int ph = vdl2Preamble[15];
    for (std::size_t i = 0; i < out.size(); i += 3) {
        const int v = (out[i] << 2) | (out[i + 1] << 1) | out[i + 2];
        int q = 0;
        while (vdl2Gray[q] != v) ++q;
        ph = (ph + q) % 8;
        syms.push_back(ph);
    }
    return syms;
}

namespace {
// raised-cosine pulse, alpha 0.6, x in symbols
double rcPulse(double x) {
    const double a = 0.6, d = 1 - (2 * a * x) * (2 * a * x);
    if (std::fabs(x) < 1e-9) return 1;
    const double sinc = std::sin(kPi * x) / (kPi * x);
    if (std::fabs(d) < 1e-6) return kPi / 4 * sinc;
    return sinc * std::cos(kPi * a * x) / d;
}
}  // namespace

std::size_t vdl2Modulate(const std::vector<int>& syms, double rate, double f0, double amp,
                         std::vector<std::complex<float> >& out) {
    if (!(rate > 0)) return 0;
    const double sps = rate / 10500;
    const int span = 6;
    const std::size_t n = static_cast<std::size_t>(std::ceil((syms.size() + 2 * span) * sps));
    std::vector<double> re(n, 0.0), im(n, 0.0);
    for (std::size_t k = 0; k < syms.size(); ++k) {
        const double c = std::cos(syms[k] * kPi / 4) * amp, s = std::sin(syms[k] * kPi / 4) * amp;
        const double t0 = (k + span) * sps;
        double lo = std::floor(t0 - span * sps), hi = std::ceil(t0 + span * sps);
        if (lo < 0) lo = 0;
        if (hi > static_cast<double>(n)) hi = static_cast<double>(n);
        for (std::size_t i = static_cast<std::size_t>(lo); i < static_cast<std::size_t>(hi); ++i) {
            const double w = rcPulse((i - t0) / sps);
            re[i] += c * w;
            im[i] += s * w;
        }
    }
    const std::size_t base = out.size();
    out.resize(base + n);
    for (std::size_t i = 0; i < n; ++i) {
        double r = re[i], q = im[i];
        if (f0 != 0) {
            const double w = kTwoPi * f0 * static_cast<double>(i) / rate, c = std::cos(w), s = std::sin(w);
            const double rr = r * c - q * s;
            q = r * s + q * c;
            r = rr;
        }
        out[base + i] = std::complex<float>(static_cast<float>(r), static_cast<float>(q));
    }
    return n;
}

void vdl2AddBurst(const std::vector<Bytes>& frames, double freqHz, double centerHz, double rate, double amp,
                  std::size_t at, std::vector<std::complex<float> >& iq) {
    std::vector<std::complex<float> > b;
    vdl2Modulate(vdl2BurstSymbols(frames), rate, freqHz - centerHz, amp, b);
    if (iq.size() < at + b.size()) iq.resize(at + b.size());
    for (std::size_t i = 0; i < b.size(); ++i) iq[at + i] += b[i];
}

// VDL2: aircraft datalink bursts on three channels (136.975, 136.675 and
// 136.775 MHz; the JS demo station sits on 136.825 with +150, -150 and
// -50 kHz offsets), a 30 s cycle; the registrations and flights are made up.
std::vector<Vdl2DemoBurst> vdl2DemoBursts() {
    const unsigned GS = 0x10A0FA;
    struct Ac {
        static Bytes make(unsigned gs, unsigned addr, const char* reg, const char* flight, const char* label,
                          const char* blk, const char* msg, const char* text) {
            return vdl2Frame(vdl2Address(gs, 4), vdl2Address(addr, 1, 0, true), 0x00,
                             vdl2AcarsInfo(vdl2AcarsDownlink(reg, flight, label, blk, msg, text)));
        }
    };
    std::vector<Vdl2DemoBurst> v(5);
    v[0].at = 2.0;
    v[0].freqHz = 136.975e6;
    v[0].frames.push_back(Ac::make(GS, 0x400ABC, "G-OFFG", "OG0123", "H1", "1", "M01A",
                                   "OFFGRID-SDR DEMO. VDL2 POSITION 5212N 00008E FL350"));
    v[1].at = 3.2;
    v[1].freqHz = 136.975e6;
    v[1].frames.push_back(vdl2Frame(vdl2Address(0x400ABC, 1), vdl2Address(GS, 4, 0, true), 0x01, Bytes()));
    v[2].at = 9.0;
    v[2].freqHz = 136.675e6;
    v[2].frames.push_back(Ac::make(GS, 0x406F21, "G-EZXY", "EZY12B", "16", "3", "M12A",
                                   "121530,33000,1702, 154,N 51.512 E 0.123"));
    v[3].at = 15.0;
    v[3].freqHz = 136.775e6;
    const unsigned char atn[] = {0x10, 0x01, 0xff, 0xe0, 0x45, 0x6f, 0x46, 0x46, 0x67, 0x72, 0x69, 0x64};
    v[3].frames.push_back(
        vdl2Frame(vdl2Address(0x2651D7, 4), vdl2Address(0x408064, 1, 0, true), 0x00, Bytes(atn, atn + sizeof atn)));
    v[4].at = 21.0;
    v[4].freqHz = 136.975e6;
    v[4].frames.push_back(Ac::make(GS, 0x3C6544, "D-AIBA", "LH0920", "H1", "5", "M33A", "WX REQ EGSS EGLL"));
    return v;
}

}  // namespace dec
