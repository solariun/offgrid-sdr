// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Ax25Encoders.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  AX.25 frame building, HDLC bits and AFSK / G3RUH audio (Ax25Encoders.h).
//  Frames use AX25Toolkit's ax25::Addr / ax25::Frame (Gustavo Campos); the
//  bits its hdlc::Encoder (FCS from Dire Wolf, WB2OSZ).  The Mic-E encoder
//  and the demo packets are ports of OFFgrid-SDR's JS aprsMicEEncode and
//  demo "aprs" station (diddy-boy, https://github.com/solariun/offgrid-sdr).
//  The audio synthesis is new here: AX25Toolkit's Modulator sends a whole
//  number of samples per bit (36 at 44100 Hz for 1200 Bd: 2% fast), so this
//  one keeps the bit timing exact.
//
//  How it fits in
//  --------------
//  Implements Ax25Encoders.h; uses ax25/hdlc.h and ax25/ax25frame.h.
//
//  Contributing
//  ------------
//  Deterministic output only.  Keep aprsMicEEncode in step with the JS.
// ============================================================================
#include "Ax25Encoders.h"

#include "ax25/ax25frame.h"
#include "ax25/hdlc.h"

#include <cmath>

namespace dec {

namespace {

const double kTwoPi = 6.283185307179586;

// "CALL-SSID" or "CALL-SSID*" -> address (H bit from the '*').
ax25::Addr addr(std::string s) {
    bool h = false;
    if (!s.empty() && s[s.size() - 1] == '*') { h = true; s.erase(s.size() - 1); }
    ax25::Addr a = ax25::Addr::make(s);
    a.repeated = h;
    return a;
}

}  // namespace

std::vector<unsigned char> ax25UiFrame(const std::string& source, const std::string& destination,
                                       const std::vector<std::string>& path, const std::string& info) {
    ax25::Frame f;
    f.dest = addr(destination);
    f.dest.repeated = false;
    f.src = addr(source);
    f.src.repeated = false;
    for (std::size_t i = 0; i < path.size(); ++i) f.digis.push_back(addr(path[i]));
    f.ctrl = 0x03;
    f.pid = 0xF0;
    f.has_pid = true;
    f.info.assign(info.begin(), info.end());
    const std::vector<uint8_t> b = f.encode();
    return std::vector<unsigned char>(b.begin(), b.end());
}

unsigned short ax25Fcs(const unsigned char* bytes, std::size_t n) { return hdlc::fcs_calc(bytes, n); }

std::vector<int> ax25HdlcBits(const std::vector<unsigned char>& frame, int preFlags, int postFlags, bool appendFcs) {
    std::vector<int> bits;
    bits.reserve(static_cast<std::size_t>(preFlags + postFlags) * 8 + (frame.size() + 2) * 10);
    hdlc::Encoder e;
    e.set_on_bit([&bits](int b) { bits.push_back(b); });
    e.send_frame(frame.empty() ? 0 : &frame[0], frame.size(), preFlags, postFlags, appendFcs);
    return bits;
}

std::size_t synthAx25Bits(const std::vector<int>& levels, double rate, std::vector<float>& out, const Ax25Tx& tx) {
    if (!(rate > 0) || levels.empty() || !(tx.baud > 0)) return 0;
    const double bitRate = tx.baud * tx.clockScale;
    const std::size_t nbits = levels.size();
    const std::size_t total = static_cast<std::size_t>(std::ceil(nbits * rate / bitRate));
    out.reserve(out.size() + total);
    if (tx.baud >= 4800) {
        // G3RUH: scramble the NRZI levels (x^17 + x^12 + 1, the output fed
        // back), NRZ, raised-cosine edges a quarter of a bit either side.
        std::vector<float> lv(nbits);
        unsigned int st = 0;
        for (std::size_t k = 0; k < nbits; ++k) {
            const unsigned int o = (static_cast<unsigned int>(levels[k]) ^ (st >> 16) ^ (st >> 11)) & 1u;
            st = ((st << 1) | o) & 0x1FFFFu;
            lv[k] = o ? 1.0f : -1.0f;
        }
        for (std::size_t i = 0; i < total; ++i) {
            const double pos = i * bitRate / rate;
            std::size_t k = static_cast<std::size_t>(pos);
            if (k >= nbits) k = nbits - 1;
            const double u = pos - static_cast<double>(k);
            double v = lv[k];
            if (u < 0.25 && k > 0) {
                const double w = 0.5 * (1 + std::sin(kTwoPi * u));          // 0.5 .. 1
                v = lv[k - 1] + (lv[k] - lv[k - 1]) * w;
            } else if (u > 0.75 && k + 1 < nbits) {
                const double w = 0.5 * (1 + std::sin(kTwoPi * (u - 1)));    // 0 .. 0.5
                v = lv[k] + (lv[k + 1] - lv[k]) * w;
            }
            out.push_back(static_cast<float>(tx.amplitude * v));
        }
        return total;
    }
    // AFSK, phase-continuous: the tone changes, the phase carries on.
    const double wm = kTwoPi * tx.markHz * tx.toneScale / rate, ws = kTwoPi * tx.spaceHz * tx.toneScale / rate;
    double ph = 0;
    for (std::size_t i = 0; i < total; ++i) {
        std::size_t k = static_cast<std::size_t>(i * bitRate / rate);
        if (k >= nbits) k = nbits - 1;
        out.push_back(static_cast<float>(tx.amplitude * std::cos(ph)));
        ph += levels[k] ? wm : ws;
        if (ph > kTwoPi) ph -= kTwoPi;
    }
    return total;
}

std::size_t synthAx25(const std::vector<unsigned char>& frame, double rate, std::vector<float>& out, const Ax25Tx& tx) {
    return synthAx25Bits(ax25HdlcBits(frame, tx.preFlags, tx.postFlags), rate, out, tx);
}

// The JS aprsMicEEncode: latitude and the message bits in the destination,
// longitude, speed and course in the information field (TAPR's Mic-E
// format: degrees 0-9 and 100-179 use the "+100" bit, as aprslib decodes
// them).
void aprsMicEEncode(double lat, double lon, double speedKmh, int course, char sym, char table, int msg,
                    std::string& destination, std::string& info) {
    const double la = std::fabs(lat), lo = std::fabs(lon);
    const int ld = static_cast<int>(std::floor(la));
    const double lm = (la - ld) * 60;
    const int lmi = static_cast<int>(std::floor(lm)), lh = static_cast<int>(std::floor((lm - lmi) * 100 + 0.5));
    int digs[6] = {ld / 10, ld % 10, lmi / 10, lmi % 10, lh / 10, lh % 10};
    for (int i = 0; i < 6; ++i)
        if (digs[i] > 9) digs[i] = 9;
    const int deg = static_cast<int>(std::floor(lo));
    const bool off = deg < 10 || deg >= 100;
    const int a[3] = {msg >> 2 & 1, msg >> 1 & 1, msg & 1};
    destination.clear();
    for (int i = 0; i < 6; ++i) {
        const bool on = i < 3 ? a[i] != 0 : i == 3 ? lat >= 0 : i == 4 ? off : lon < 0;
        destination += on ? static_cast<char>('P' + digs[i]) : static_cast<char>('0' + digs[i]);
    }
    const int dd = deg < 10 ? deg + 90 : deg < 100 ? deg : deg < 110 ? deg - 20 : deg - 100;
    const double mins = (lo - deg) * 60;
    const int mi = static_cast<int>(std::floor(mins)), mh = static_cast<int>(std::floor((mins - mi) * 100 + 0.5));
    const int m2 = mi < 10 ? mi + 60 : mi;
    const int kn = static_cast<int>(std::floor(speedKmh / 1.852 + 0.5)), sp = kn / 10,
              dc = (kn % 10) * 10 + course / 100, se = course % 100;
    info = "`";
    info += static_cast<char>(dd + 28);
    info += static_cast<char>(m2 + 28);
    info += static_cast<char>((mh < 99 ? mh : 99) + 28);
    info += static_cast<char>(sp + 28);
    info += static_cast<char>(dc + 28);
    info += static_cast<char>(se + 28);
    info += sym;
    info += table;
}

std::vector<AprsSample> aprsDemoPackets() {
    // APRS on 144.800 MHz: a car (Mic-E, as vehicle radios send it), a
    // weather station, a message, a digipeater's status, a repeater object,
    // a walker and a balloon (the JS demo's packets).
    std::vector<AprsSample> v;
    AprsSample s;
    std::string dst, info;
    aprsMicEEncode(53.4808, -2.2426, 62, 135, '>', '/', 6, dst, info);
    s.what = "mic-e car"; s.source = "M0OGR-9"; s.destination = dst;
    s.path.clear(); s.path.push_back("WIDE1-1"); s.path.push_back("WIDE2-1");
    s.info = info + "OFFgrid-SDR demo car";
    v.push_back(s);
    s.what = "weather"; s.source = "MB7OGW"; s.destination = "APRS";
    s.path.assign(1, "WIDE2-1");
    s.info = "@221815z5328.50N/00213.80W_270/012g018t054r000p000P000h78b10142OFFgrid-SDR demo weather";
    v.push_back(s);
    s.what = "message"; s.source = "G4OGR-7"; s.destination = "APRS";
    s.path.assign(1, "WIDE1-1");
    s.info = ":M0OGR-9  :See you at the radio club tonight{17";
    v.push_back(s);
    s.what = "status"; s.source = "GB7OGD"; s.destination = "APRS";
    s.path.clear();
    s.info = ">OFFgrid-SDR demo digipeater, 144.800 MHz";
    v.push_back(s);
    s.what = "object"; s.source = "G4OGR"; s.destination = "APRS";
    s.path.assign(1, "WIDE2-1");
    s.info = ";GB3OG    *221815z5330.00N/00215.00Wr145.6000MHz T082 R40k demo repeater";
    v.push_back(s);
    s.what = "walker"; s.source = "2E0OGB-7"; s.destination = "APRS";
    s.path.clear(); s.path.push_back("WIDE1-1"); s.path.push_back("WIDE2-1");
    s.info = "!5126.40N/00007.30W[Out walking on the hills";
    v.push_back(s);
    s.what = "balloon"; s.source = "M0OGB-11"; s.destination = "APRS";
    s.path.assign(1, "WIDE2-1");
    s.info = "/221815h5240.00N/00130.00WO000/045/A=031200 OFFgrid-SDR demo balloon";
    v.push_back(s);
    return v;
}

}  // namespace dec
