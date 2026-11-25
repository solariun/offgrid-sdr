// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/WsprEncoders.cpp
//  Layer:  Library liboffgrid-decoders (test and demo signals)
//
//  What this file does
//  -------------------
//  The WSPR signal generators described in WsprEncoders.h: the JS demo
//  band's "ft8"-kind WSPR stations (diddy-boy, index.html: wsprst(), plain
//  tone changes, 8192/12000 s symbols, 12000/8192 Hz spacing, from the even
//  minute + 1 s), as audio.  Additions to the JS: drift, any delay, and
//  seeded Gaussian noise.
//
//  How it fits in
//  --------------
//  See WsprEncoders.h.
//
//  Contributing
//  ------------
//  Keep it deterministic (no clocks, no global random state).
// ============================================================================
#include "WsprEncoders.h"

#include <cmath>

namespace dec {
namespace enc {

namespace {
const double kPi = 3.14159265358979323846;

// the tone of symbol position `pos` (symbols, fractional) of a transmission
double toneAt(const WsprStation& st, const unsigned char* sym, double pos) {
    int i = static_cast<int>(std::floor(pos));
    if (i < 0) i = 0;
    if (i > 161) i = 161;
    return st.audioHz + (sym[i] - 1.5) * wspr::kToneHz + st.driftHz * (pos - 81) / 162.0;
}
}  // namespace

std::vector<float> wsprTransmission(const WsprStation& st, double rate) {
    std::vector<float> out;
    unsigned char sym[162];
    if (!(rate > 0) || !wspr::symbols(st.call, st.grid, st.dbm, sym)) return out;
    const std::size_t n = static_cast<std::size_t>(std::floor(wspr::kTxSeconds * rate));
    out.resize(n);
    double ph = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const double pos = (k / rate) / wspr::kSymbolSeconds;
        out[k] = static_cast<float>(st.amplitude * std::sin(ph));
        ph += 2 * kPi * toneAt(st, sym, pos) / rate;
        if (ph > 2 * kPi) ph -= 2 * kPi;
    }
    return out;
}

double wsprNoiseFor(double amplitude, double snrDb, double rate) {
    // signal power A^2/2 against the noise in 2500 Hz: sigma^2 * 2500 / (rate / 2)
    return std::sqrt(amplitude * amplitude / 2 / std::pow(10.0, snrDb / 10) * (rate / 2) / 2500);
}

std::vector<WsprStation> demoWsprStations(double scale) {
    std::vector<WsprStation> v;
    v.push_back(WsprStation("G4OGR", "IO91", 30, 1462, 0.00008 * scale));
    v.push_back(WsprStation("K1OGS", "FN42", 37, 1531, 0.00005 * scale));
    v.push_back(WsprStation("VK2OGS", "QF56", 23, 1574, 0.000034 * scale));
    return v;
}

WsprAir::WsprAir(double rate, const std::vector<WsprStation>& stations, double noiseRms, unsigned seed)
    : rate_(rate), noise_(noiseRms), rng_(0x9E3779B97F4A7C15ULL ^ seed), haveSpare_(false), spare_(0) {
    for (std::size_t i = 0; i < stations.size(); ++i) {
        Tx t;
        t.st = stations[i];
        t.ok = wspr::symbols(t.st.call, t.st.grid, t.st.dbm, t.sym);
        t.phase = 0;
        tx_.push_back(t);
    }
}

double WsprAir::gauss() {   // Box-Muller on a 64-bit xorshift*
    if (haveSpare_) { haveSpare_ = false; return spare_; }
    double u1, u2;
    do {
        rng_ ^= rng_ >> 12; rng_ ^= rng_ << 25; rng_ ^= rng_ >> 27;
        u1 = static_cast<double>((rng_ * 2685821657736338717ULL) >> 11) / 9007199254740992.0;
        rng_ ^= rng_ >> 12; rng_ ^= rng_ << 25; rng_ ^= rng_ >> 27;
        u2 = static_cast<double>((rng_ * 2685821657736338717ULL) >> 11) / 9007199254740992.0;
    } while (u1 <= 1e-300);
    const double r = std::sqrt(-2 * std::log(u1));
    spare_ = r * std::sin(2 * kPi * u2); haveSpare_ = true;
    return r * std::cos(2 * kPi * u2);
}

void WsprAir::render(double t0, float* out, std::size_t n) {
    for (std::size_t k = 0; k < n; ++k) {
        const double t = t0 + k / rate_;
        double v = noise_ > 0 ? noise_ * gauss() : 0;
        for (std::size_t j = 0; j < tx_.size(); ++j) {
            Tx& x = tx_[j];
            if (!x.ok) continue;
            const double into = t - std::floor(t / 120) * 120 - x.st.delay;   // seconds into this slot's transmission
            if (into < 0 || into >= wspr::kTxSeconds) continue;
            v += x.st.amplitude * std::sin(x.phase);
            x.phase += 2 * kPi * toneAt(x.st, x.sym, into / wspr::kSymbolSeconds) / rate_;
            if (x.phase > 2 * kPi) x.phase -= 2 * kPi;
        }
        out[k] = static_cast<float>(v);
    }
}

}  // namespace enc
}  // namespace dec
