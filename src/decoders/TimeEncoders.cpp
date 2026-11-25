// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/TimeEncoders.cpp
//  Layer:  Library liboffgrid-decoders (test and demo signals)
//
//  What this file does
//  -------------------
//  The time-station generators described in TimeEncoders.h: a port of
//  OFFgrid-SDR's demoChuBytes / demoWwvBits and its "timesig" demo
//  station (diddy-boy, index.html), as the audio a USB receiver gives.
//  Additions to the JS: WWV / WWVH as AM with both sidebands, WWVH, the
//  500 / 600 / 440 Hz tones, the subcarrier's 15 dB low level, CHU's hour
//  tone of 1 s and its upper-sideband-only signal, noise.
//
//  How it fits in
//  --------------
//  See TimeEncoders.h.
//
//  Contributing
//  ------------
//  Keep the code layouts in step with TimeDecoder (Time.cpp).
// ============================================================================
#include "TimeEncoders.h"

#include <cmath>

namespace dec {
namespace enc {

namespace {

const double kPi = 3.14159265358979323846;

// a phase-continuous oscillator
struct Osc {
    double ph;
    Osc() : ph(0) {}
    double step(double hz, double rate) {
        const double v = std::sin(ph);
        ph += 2 * kPi * hz / rate;
        if (ph > 2 * kPi) ph -= 2 * kPi;
        return v;
    }
};

// both sidebands of an AM tone `f` Hz on the carrier at `fc`, as USB audio: the one above, and the one below if it is
// still above 50 Hz (a USB filter takes away what is lower)
double amTone(Osc& up, Osc& dn, double fc, double f, double rate) {
    double v = up.step(fc + f, rate);
    if (fc - f > 50) v += dn.step(fc - f, rate);
    else dn.step(0, rate);
    return v;
}

}  // namespace

std::vector<int> chuCodeBytes(double utcSecond) {
    std::vector<int> b;
    int y, mo, d, doy, hh, mi, ss;
    utcFields(utcSecond, y, mo, d, doy, hh, mi, ss);
    int dig[10];
    if (ss == 31) {
        // format B: (0, 1), the year, TAI-UTC (37), (0, 0)
        const int v[10] = {0, 1, (y / 1000) % 10, (y / 100) % 10, (y / 10) % 10, y % 10, 3, 7, 0, 0};
        for (int i = 0; i < 10; ++i) dig[i] = v[i];
    } else if (ss >= 32 && ss <= 39) {
        // format A: 6, the day of the year, hour, minute, second
        const int v[10] = {6, doy / 100, (doy / 10) % 10, doy % 10, hh / 10, hh % 10, mi / 10, mi % 10, ss / 10, ss % 10};
        for (int i = 0; i < 10; ++i) dig[i] = v[i];
    } else {
        return b;
    }
    for (int i = 0; i < 10; i += 2) b.push_back(dig[i] | (dig[i + 1] << 4));
    for (int i = 0; i < 5; ++i) b.push_back(ss == 31 ? (b[i] ^ 255) : b[i]);
    return b;
}

std::vector<int> wwvCodeBits(double utcMinute) {
    std::vector<int> bits(60, 0);
    int y, mo, d, doy, hh, mi, ss;
    utcFields(utcMinute, y, mo, d, doy, hh, mi, ss);
    struct Put {
        std::vector<int>& bits;
        void operator()(const int* secs, const int* w, int n, int v) const {
            for (int i = 0; i < n; ++i) if (v & w[i]) bits[secs[i]] = 1;
        }
    } put = {bits};
    static const int w8[] = {1, 2, 4, 8}, w4[] = {1, 2, 4}, w2[] = {1, 2};
    static const int sMu[] = {10, 11, 12, 13}, sMt[] = {15, 16, 17}, sHu[] = {20, 21, 22, 23}, sHt[] = {25, 26};
    static const int sDu[] = {30, 31, 32, 33}, sDt[] = {35, 36, 37, 38}, sDh[] = {40, 41};
    static const int sYu[] = {4, 5, 6, 7}, sYt[] = {51, 52, 53, 54};
    const int yy = y % 100;
    put(sMu, w8, 4, mi % 10); put(sMt, w4, 3, mi / 10);
    put(sHu, w8, 4, hh % 10); put(sHt, w2, 2, hh / 10);
    put(sDu, w8, 4, doy % 10); put(sDt, w8, 4, (doy / 10) % 10); put(sDh, w2, 2, doy / 100);
    put(sYu, w8, 4, yy % 10); put(sYt, w8, 4, yy / 10);
    for (int s = 9; s <= 59; s += 10) bits[s] = 2;
    bits[0] = -1;
    return bits;
}

void addTimeNoise(std::vector<float>& audio, double rms, unsigned seed) {
    if (rms <= 0) return;
    unsigned long long st = 0x9E3779B97F4A7C15ULL ^ (static_cast<unsigned long long>(seed) * 0xD1B54A32D192ED03ULL);
    if (!st) st = 1;
    struct Rng {
        unsigned long long& s;
        double uni() {   // (0, 1]: xorshift64*
            s ^= s >> 12; s ^= s << 25; s ^= s >> 27;
            return (static_cast<double>((s * 2685821657736338717ULL) >> 11) + 1) / 9007199254740993.0;
        }
    } r = {st};
    for (std::size_t i = 0; i < audio.size(); i += 2) {
        const double u1 = r.uni(), u2 = r.uni(), m = rms * std::sqrt(-2 * std::log(u1));
        audio[i] += static_cast<float>(m * std::cos(2 * kPi * u2));
        if (i + 1 < audio.size()) audio[i + 1] += static_cast<float>(m * std::sin(2 * kPi * u2));
    }
}

std::vector<float> timeSignalAudio(TimeStation station, double utcStart, double seconds, double rate,
                                   const TimeSignalOptions& opt) {
    const std::size_t n = seconds > 0 && rate > 0 ? static_cast<std::size_t>(seconds * rate) : 0;
    std::vector<float> out(n, 0.f);
    const double A = opt.level, fc = opt.carrierHz;
    Osc car, tick, tickDn, fsk, sub, subDn, tone, toneDn;
    long long curSec = -1;
    int sm = 0, minuteOfHour = 0;
    std::vector<int> bytes, wbits;
    long long wMinute = -1;
    const bool wwvh = station == TimeStation::Wwvh, wwv = station == TimeStation::Wwv || wwvh;
    for (std::size_t i = 0; i < n; ++i) {
        const double t = utcStart + i / rate, secF = std::floor(t), u = t - secF;
        const long long sec = static_cast<long long>(secF);
        if (sec != curSec) {   // (the second's fields, once a second)
            curSec = sec;
            int y, mo, d, doy, hh, mi, ss;
            utcFields(secF, y, mo, d, doy, hh, mi, ss);
            sm = ss; minuteOfHour = mi;
            if (station == TimeStation::Chu) bytes = chuCodeBytes(secF);
            if (wwv && sec - sm != wMinute) { wMinute = sec - sm; wbits = wwvCodeBits(static_cast<double>(wMinute)); }
        }
        double v = 0;
        if (station == TimeStation::Chu) {
            // carrier, ticks, minute / hour tone, FSK code (upper sideband only)
            const double tickLen = sm == 0 ? (minuteOfHour == 0 ? 1.0 : 0.5) : sm == 29 ? 0
                                 : ((sm >= 31 && sm <= 39) || sm >= 51) ? 0.01 : 0.3;
            const double tk = u < tickLen ? 1 : 0;
            double fk = 0;
            bool mark = true;
            if (sm >= 31 && sm <= 39 && u >= 0.01 && u < 0.5 && bytes.size() == 10) {
                const double lead = 0.5 - 110.0 / 300, k = (u - lead) * 300;
                fk = 1;
                if (k >= 0) {
                    const int by = static_cast<int>(std::floor(k / 11)), bi = static_cast<int>(std::floor(std::fmod(k, 11.0)));
                    if (by < 10) mark = bi == 0 ? false : bi <= 8 ? ((bytes[by] >> (bi - 1)) & 1) != 0 : true;
                }
            }
            const double c = car.step(fc, rate), tt = tick.step(fc + 1000, rate), ff = fsk.step(fc + (mark ? 2225 : 2025), rate);
            v = A * (c + 0.6 * tk * tt + 0.6 * fk * ff);
        } else if (wwv) {
            // AM: carrier, ticks and minute tone (m = 1), subcarrier (m = 0.18, 15 dB lower between pulses), tones (m = 0.5)
            const double tickHz = sm == 0 && minuteOfHour == 0 ? 1500 : wwvh ? 1200 : 1000;
            const double tk = sm == 0 ? (u < 0.8 ? 1 : 0) : (sm == 29 || sm == 59) ? 0 : (u < 0.005 ? 1 : 0);
            const int code = wbits.empty() ? -1 : wbits[sm];
            const double len = code < 0 ? 0 : code == 2 ? 0.8 : code == 1 ? 0.5 : 0.2;
            const double subLv = u >= 0.03 && u < len ? 1.0 : 0.178;   // (-15 dB)
            double toneHz = 0;
            if (opt.tones && sm >= 1 && sm <= 44 && minuteOfHour != 29 && minuteOfHour != 59) {
                if ((wwvh && minuteOfHour == 1) || (!wwvh && minuteOfHour == 2)) toneHz = 440;
                else if (minuteOfHour != 0) toneHz = ((minuteOfHour % 2 == 0) != wwvh) ? 500 : 600;
            }
            const double quiet = (u < 0.03 || u > 0.99) ? 0 : 1;   // (the tones stop around each tick)
            const double c = car.step(fc, rate);
            const double tt = amTone(tick, tickDn, fc, tickHz, rate);
            const double ss = amTone(sub, subDn, fc, 100, rate);
            const double to = toneHz > 0 ? amTone(tone, toneDn, fc, toneHz, rate) : 0;
            v = A * (c + 0.5 * tk * tt + 0.09 * subLv * ss + 0.25 * quiet * to);
        } else {
            // RWM: the carrier keyed, 0.1 s a second, 0.5 s at the minute
            const double on = u < (sm == 0 ? 0.5 : 0.1) ? 1 : 0;
            v = A * on * car.step(fc, rate);
        }
        out[i] = static_cast<float>(v);
    }
    addTimeNoise(out, opt.noiseRms, opt.seed);
    return out;
}

}  // namespace enc
}  // namespace dec
