// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/AisEncoders.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  AisEncoders.h: AIS message builders, HDLC/NRZI framing, GMSK synthesis
//  and the demo band's plan, ported from OFFgrid-SDR's JS encoder
//  (diddy-boy, https://github.com/solariun/offgrid-sdr, index.html: aisPack,
//  aisFrame, aisPosMsg, aisStaticMsg, aisClassB, aisB24, aisAton, aisBase,
//  aisGmsk, the "ais" demo station).
//
//  How it fits in
//  --------------
//  Implements AisEncoders.h; the CRC and bit reversal come from Ais.cpp.
//  Standard library only.
//
//  Contributing
//  ------------
//  Field layouts: ITU-R M.1371 (and the JS).  Keep it deterministic.
// ============================================================================
#include "AisEncoders.h"

#include "Ais.h"

#include <cctype>
#include <algorithm>
#include <cmath>

namespace dec {

namespace {
const double kPi = 3.141592653589793;
// Addition to the JS: bursts are built at 40 samples a bit (384 kHz), not
// the JS's 5 (48 kHz): a frequency held for 1/48000 s makes images of the
// signal every 48 kHz, and one of channel B's lands 2 kHz from channel A.
const int kBurstSps = 40;
const double kBurstRate = 9600.0 * kBurstSps;
}  // namespace

// ---------------------------------------------------------------------------
// aisPack
// ---------------------------------------------------------------------------
void AisPacker::field(double value, int width) {
    double v = std::floor(value + 0.5);   // Math.round
    if (v < 0) v += std::pow(2.0, width);
    for (int k = width - 1; k >= 0; --k)
        bits_.push_back(static_cast<unsigned char>(static_cast<long long>(std::floor(v / std::pow(2.0, k))) % 2));
}

void AisPacker::text(const std::string& t, int chars) {
    for (int i = 0; i < chars; ++i) {
        int c = i < static_cast<int>(t.size()) ? std::toupper(static_cast<unsigned char>(t[i])) : '@';
        c = c >= 64 ? c - 64 : c;
        c &= 63;
        for (int k = 5; k >= 0; --k) bits_.push_back(static_cast<unsigned char>((c >> k) & 1));
    }
}

std::vector<unsigned char> AisPacker::bytes(int nbits) const {
    std::vector<unsigned char> b(bits_);
    while (static_cast<int>(b.size()) < nbits) b.push_back(0);
    std::vector<unsigned char> out;
    for (std::size_t i = 0; i < b.size(); i += 8) {
        unsigned v = 0;
        for (std::size_t k = 0; k < 8; ++k) v = v * 2 + (i + k < b.size() ? b[i + k] : 0);
        out.push_back(static_cast<unsigned char>(v));
    }
    return out;
}

// ---------------------------------------------------------------------------
// The message builders (the JS layouts)
// ---------------------------------------------------------------------------
std::vector<unsigned char> aisPositionMsg(unsigned mmsi, double lat, double lon, double sogKn, double cogDeg,
                                          int heading, int status, int type) {
    AisPacker p;
    p.field(type, 6); p.field(0, 2); p.field(mmsi, 30); p.field(status, 4); p.field(-128, 8);
    p.field(std::floor(sogKn * 10 + 0.5), 10); p.field(1, 1);
    p.field(std::floor(lon * 600000 + 0.5), 28); p.field(std::floor(lat * 600000 + 0.5), 27);
    p.field(std::floor(cogDeg * 10 + 0.5), 12); p.field(heading, 9); p.field(30, 6); p.field(0, 2); p.field(0, 3);
    p.field(0, 1); p.field(0, 19);
    return p.bytes(168);
}

std::vector<unsigned char> aisStaticMsg(unsigned mmsi, unsigned imo, const std::string& callsign,
                                        const std::string& name, int shipType, int lengthM, int beamM,
                                        double draughtM, const std::string& destination, int etaMonth, int etaDay,
                                        int etaHour, int etaMinute) {
    AisPacker p;
    const double bow = std::floor(lengthM * 0.7 + 0.5), port = std::floor(beamM / 2.0 + 0.5);
    p.field(5, 6); p.field(0, 2); p.field(mmsi, 30); p.field(0, 2); p.field(imo, 30);
    p.text(callsign, 7); p.text(name, 20); p.field(shipType, 8);
    p.field(bow, 9); p.field(lengthM - bow, 9); p.field(port, 6); p.field(beamM - port, 6);
    p.field(1, 4); p.field(etaMonth, 4); p.field(etaDay, 5); p.field(etaHour, 5); p.field(etaMinute, 6);
    p.field(std::floor(draughtM * 10 + 0.5), 8); p.text(destination, 20); p.field(0, 1); p.field(0, 1);
    return p.bytes(424);
}

std::vector<unsigned char> aisClassBMsg(unsigned mmsi, double lat, double lon, double sogKn, double cogDeg,
                                        int heading) {
    AisPacker p;
    p.field(18, 6); p.field(0, 2); p.field(mmsi, 30); p.field(0, 8); p.field(std::floor(sogKn * 10 + 0.5), 10);
    p.field(0, 1); p.field(std::floor(lon * 600000 + 0.5), 28); p.field(std::floor(lat * 600000 + 0.5), 27);
    p.field(std::floor(cogDeg * 10 + 0.5), 12); p.field(heading, 9); p.field(30, 6); p.field(0, 2);
    for (int k = 0; k < 5; ++k) p.field(1, 1);
    p.field(0, 1); p.field(0, 1); p.field(0, 20);
    return p.bytes(168);
}

std::vector<unsigned char> aisStaticBNameMsg(unsigned mmsi, const std::string& name) {
    AisPacker p;
    p.field(24, 6); p.field(0, 2); p.field(mmsi, 30); p.field(0, 2); p.text(name, 20);
    return p.bytes(160);
}

std::vector<unsigned char> aisStaticBTypeMsg(unsigned mmsi, int shipType, const std::string& callsign) {
    AisPacker p;
    p.field(24, 6); p.field(0, 2); p.field(mmsi, 30); p.field(1, 2); p.field(shipType, 8); p.text("OFF", 3);
    p.field(0, 4); p.field(0, 20); p.text(callsign, 7); p.field(0, 30); p.field(0, 6);
    return p.bytes(168);
}

std::vector<unsigned char> aisAtonMsg(unsigned mmsi, int atonType, const std::string& name, double lat, double lon) {
    AisPacker p;
    p.field(21, 6); p.field(0, 2); p.field(mmsi, 30); p.field(atonType, 5); p.text(name, 20); p.field(1, 1);
    p.field(std::floor(lon * 600000 + 0.5), 28); p.field(std::floor(lat * 600000 + 0.5), 27); p.field(0, 30);
    p.field(1, 4); p.field(60, 6); p.field(0, 1); p.field(0, 8);
    for (int k = 0; k < 4; ++k) p.field(0, 1);
    return p.bytes(272);
}

std::vector<unsigned char> aisBaseMsg(unsigned mmsi, double lat, double lon, int year, int month, int day, int hour,
                                      int minute, int second) {
    AisPacker p;
    p.field(4, 6); p.field(0, 2); p.field(mmsi, 30); p.field(year, 14); p.field(month, 4); p.field(day, 5);
    p.field(hour, 5); p.field(minute, 6); p.field(second, 6); p.field(1, 1);
    p.field(std::floor(lon * 600000 + 0.5), 28); p.field(std::floor(lat * 600000 + 0.5), 27); p.field(1, 4);
    p.field(0, 10); p.field(0, 1); p.field(0, 19);
    return p.bytes(168);
}

// Addition to the JS: type 27.
std::vector<unsigned char> aisLongRangeMsg(unsigned mmsi, double lat, double lon, int sogKn, int cogDeg, int status) {
    AisPacker p;
    p.field(27, 6); p.field(0, 2); p.field(mmsi, 30); p.field(0, 1); p.field(0, 1); p.field(status, 4);
    p.field(std::floor(lon * 600 + 0.5), 18); p.field(std::floor(lat * 600 + 0.5), 17); p.field(sogKn, 6);
    p.field(cogDeg, 9); p.field(0, 1); p.field(0, 1);
    return p.bytes(96);
}

// ---------------------------------------------------------------------------
// aisFrame: -> the NRZI levels to send, training sequence first
// ---------------------------------------------------------------------------
std::vector<unsigned char> aisFrameLevels(const std::vector<unsigned char>& msg, unsigned crcXor) {
    std::vector<unsigned char> all;
    for (std::size_t i = 0; i < msg.size(); ++i) all.push_back(aisReverse(msg[i]));
    const unsigned c = (aisCrc(all.empty() ? 0 : &all[0], all.size()) ^ crcXor) & 0xffff;
    all.push_back(static_cast<unsigned char>(c & 255));
    all.push_back(static_cast<unsigned char>(c >> 8));
    std::vector<unsigned char> raw;
    for (int k = 0; k < 24; ++k) raw.push_back(static_cast<unsigned char>(k & 1));   // (training: 0101...)
    for (int k = 0; k < 8; ++k) raw.push_back(static_cast<unsigned char>((0x7e >> k) & 1));
    int ones = 0;
    for (std::size_t i = 0; i < all.size(); ++i)
        for (int k = 0; k < 8; ++k) {
            const unsigned char b = static_cast<unsigned char>((all[i] >> k) & 1);
            raw.push_back(b);
            if (b) {
                if (++ones == 5) {
                    raw.push_back(0);
                    ones = 0;
                }
            } else {
                ones = 0;
            }
        }
    for (int k = 0; k < 8; ++k) raw.push_back(static_cast<unsigned char>((0x7e >> k) & 1));
    for (int k = 0; k < 4; ++k) raw.push_back(0);
    std::vector<unsigned char> lv;
    unsigned char l = 0;
    for (std::size_t i = 0; i < raw.size(); ++i) {
        if (!raw[i]) l ^= 1;
        lv.push_back(l);
    }
    return lv;
}

// aisGmsk: 5 samples a bit at 48 kHz, a Gaussian filter (sigma 0.3313 of a
// bit, edges held), ±2400 Hz.
namespace {
std::vector<float> gmskFreq(const std::vector<unsigned char>& lv, int sps) {
    const double sg = 0.3313 * sps;
    const int L = static_cast<int>(std::ceil(3 * sg));
    std::vector<double> h;
    double hs = 0;
    for (int k = -L; k <= L; ++k) {
        const double v = std::exp(-double(k) * k / (2 * sg * sg));
        h.push_back(v);
        hs += v;
    }
    std::vector<double> x;
    for (std::size_t i = 0; i < lv.size(); ++i)
        for (int k = 0; k < sps; ++k) x.push_back(lv[i] ? 1.0 : -1.0);
    const long nx = static_cast<long>(x.size());
    std::vector<float> y(x.size());
    for (long i = 0; i < nx; ++i) {
        double a = 0;
        for (int k = -L; k <= L; ++k) {
            long j = i + k;
            j = j < 0 ? 0 : j > nx - 1 ? nx - 1 : j;
            a += h[k + L] * x[j];
        }
        y[i] = static_cast<float>(2400 * a / hs);
    }
    return y;
}
}  // namespace

std::vector<float> aisGmskFreq(const std::vector<unsigned char>& lv) { return gmskFreq(lv, 5); }

AisBurst aisMakeBurst(const std::vector<unsigned char>& msg, unsigned crcXor) {
    const std::vector<float> fr = gmskFreq(aisFrameLevels(msg, crcXor), kBurstSps);
    AisBurst b;
    b.cycles.resize(fr.size() + 1);
    double c = 0;
    b.cycles[0] = 0;
    for (std::size_t k = 0; k < fr.size(); ++k) {
        c += fr[k] / kBurstRate;
        b.cycles[k + 1] = c;
    }
    return b;
}

void aisAddBurst(std::complex<float>* iq, std::size_t n, double rate, double tStart, double offsetHz, double amp,
                 const AisBurst& burst, double phase0) {
    if (!iq || !n || burst.cycles.size() < 2 || !(rate > 0)) return;
    const double len = burst.seconds();
    // samples j with 0 <= j / rate - tStart < len
    double jf0 = std::ceil(tStart * rate), jf1 = std::ceil((tStart + len) * rate);
    if (jf0 < 0) jf0 = 0;
    if (jf1 > static_cast<double>(n)) jf1 = static_cast<double>(n);
    const std::size_t nc = burst.cycles.size();
    for (std::size_t j = static_cast<std::size_t>(jf0); static_cast<double>(j) < jf1; ++j) {
        const double t = j / rate - tStart;
        const double u = t * kBurstRate;
        std::size_t k = static_cast<std::size_t>(u);
        if (k + 1 >= nc) k = nc - 2;
        const double f = u - k;
        const double cyc = burst.cycles[k] + (burst.cycles[k + 1] - burst.cycles[k]) * f;
        const double ph = phase0 + 2 * kPi * (cyc + offsetHz * t);
        // Addition to the JS: the power ramps up and down over 8 bits
        // (raised cosine), as a transmitter's does (M.1371's ramp-up time);
        // switched on and off abruptly, a burst splatters into the other
        // channel 50 kHz away.
        const double ramp = 8 / 9600.0, edge = std::min(t, len - t);
        const double g = edge >= ramp ? 1.0 : 0.5 - 0.5 * std::cos(kPi * edge / ramp);
        iq[j] += std::complex<float>(static_cast<float>(amp * g * std::cos(ph)), static_cast<float>(amp * g * std::sin(ph)));
    }
}

void aisAddBurst(std::complex<float>* iq, std::size_t n, double rate, std::size_t at, double offsetHz, double amp,
                 const std::vector<unsigned char>& msg) {
    aisAddBurst(iq, n, rate, at / rate, offsetHz, amp, aisMakeBurst(msg), 0);
}

// ---------------------------------------------------------------------------
// The demo: AIS in Liverpool Bay, a 30 s cycle (the JS demo station's plan)
// ---------------------------------------------------------------------------
std::vector<AisDemoBurst> aisDemoPlan(int year, int month, int day, int hour, int minute, int second) {
    struct P { double t; int ch; std::vector<unsigned char> m; };
    std::vector<P> plan;
    const unsigned ex = 235012345, pl = 232004567, rv = 250001234, wf = 235098765;
    const double exLat = 53.5210, exLon = -3.3520, plLat = 53.4640, plLon = -3.1050, rvLat = 53.5480, rvLon = -3.2460,
                 wfLat = 53.4020, wfLon = -3.0470;
    int k = 0;
    for (double t = 0.3; t < 30; t += 5, ++k)
        plan.push_back(P{t, k % 2, aisPositionMsg(ex, exLat, exLon, 12.3, 285.4, 284, 0)});
    k = 0;
    for (double t = 1.1; t < 30; t += 6, ++k)
        plan.push_back(P{t, (k + 1) % 2, aisPositionMsg(pl, plLat, plLon, 9.8, 121.0, 119, 0)});
    k = 0;
    for (double t = 3.7; t < 30; t += 10, ++k)
        plan.push_back(P{t, k % 2, aisPositionMsg(rv, rvLat, rvLon, 0.1, 0, 211, 1)});
    k = 0;
    for (double t = 4.4; t < 30; t += 10, ++k)
        plan.push_back(P{t, (k + 1) % 2, aisClassBMsg(wf, wfLat, wfLon, 5.6, 210.0, 211)});
    plan.push_back(P{2.0, 0, aisStaticMsg(ex, 9876543, "MOGR7", "OFFGRID EXPLORER", 70, 128, 20, 7.4, "DUBLIN", 11, 2, 14, 30)});
    plan.push_back(P{12.0, 1, aisStaticMsg(pl, 0, "MPLT4", "MERSEY PILOT", 50, 21, 6, 1.8, "LIVERPOOL", 0, 0, 24, 60)});
    plan.push_back(P{18.5, 0, aisStaticMsg(rv, 9765432, "EIRS5", "IRISH SEA ROVER", 80, 182, 32, 11.2, "LIVERPOOL", 10, 30, 6, 0)});
    plan.push_back(P{9.0, 1, aisStaticBNameMsg(wf, "WINDFALL")});
    plan.push_back(P{21.0, 0, aisStaticBTypeMsg(wf, 36, "MWND9")});
    plan.push_back(P{16.2, 1, aisAtonMsg(992351234, 21, "BAR LIGHT FLOAT", 53.5333, -3.3500)});
    plan.push_back(P{8.1, 0, aisBaseMsg(2320001, 53.3100, -4.6300, year, month, day, hour, minute, second)});
    plan.push_back(P{28.1, 1, aisBaseMsg(2320001, 53.3100, -4.6300, year, month, day, hour, minute, second)});
    std::vector<AisDemoBurst> out;
    for (std::size_t i = 0; i < plan.size(); ++i) {
        AisDemoBurst b;
        b.t = plan[i].t;
        b.channel = plan[i].ch ? 'B' : 'A';
        b.msg = plan[i].m;
        b.burst = aisMakeBurst(b.msg);
        out.push_back(b);
    }
    return out;
}

void aisDemoAdd(const std::vector<AisDemoBurst>& plan, std::complex<float>* iq, std::size_t n, double rate,
                double centerHz, double t0, double amp) {
    if (!iq || !n || !(rate > 0)) return;
    const double t1 = t0 + n / rate;
    for (long cyc = static_cast<long>(std::floor(t0 / 30)) - 1; cyc <= static_cast<long>(std::floor(t1 / 30)); ++cyc)
        for (std::size_t i = 0; i < plan.size(); ++i) {
            const AisDemoBurst& b = plan[i];
            const double bs = cyc * 30.0 + b.t, be = bs + b.burst.seconds();
            if (be <= t0 || bs >= t1) continue;
            const double off = (b.channel == 'B' ? kAisChannelB : kAisChannelA) - centerHz;
            if (std::fabs(off) >= rate * 0.45) continue;
            aisAddBurst(iq, n, rate, bs - t0, off, amp, b.burst, 0);
        }
}

}  // namespace dec
