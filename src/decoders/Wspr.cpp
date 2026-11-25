// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Wspr.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The WSPR decoder described in Wspr.h: a port of OFFgrid-SDR's WSPR
//  library (diddy-boy, index.html: pack, unpack, encode, symbols, fano,
//  decode, decodeSlice, demod), of the WSPR parts of its FT8/FT4/WSPR slot
//  driver (ft8Feed(), ft8Done()) and of its calibration (wsprNoteFreq(),
//  wsprCalText(), wsprCalApply()).  The function names follow the
//  JavaScript.  WSPR itself is K1JT's (Joe Taylor; WSJT-X's wsprd); the
//  Fano decoder follows Phil Karn KA9Q's, as wsprd does.
//
//  Additions to the JS (each marked where it is):
//    - the audio path: the browser resamples to 12000 Hz real by straight
//      interpolation (relying on its USB filter) and transforms all of it
//      at once (2^21-2^22 points).  Here the audio is mixed down by 1500 Hz
//      and resampled to 3000 Hz complex with a Kaiser-windowed sinc filter
//      (alias-free 200-2800 Hz of audio, any input rate from 6 kHz), so the
//      ring holds 236 s in 5.7 MB and the decode transforms 2^20 points;
//      the 375 Hz slices and everything after them are the JS's;
//    - the audio is worked in chunks of 0.25 s, with the slot check after
//      each, so a large block cannot skip a slot's decode;
//    - demod()'s tone amplitudes: the four tones (375/256 Hz apart) taken
//      as DFT bins of each symbol after one mix-down, each symbol from
//      phase 0 (only amplitudes are used): the same values, ~1.6x faster;
//    - a drift search (as wsprd's, -4..+4 Hz over the transmission) after
//      the fine timing and frequency search; when it finds a better sync,
//      the drifting tones are tried first, then the JS's undrifted ones;
//      the drift is reported.  The candidates still come from the JS's
//      undrifted sync search, so drifts beyond about +-2 Hz are rarely
//      found;
//    - fano(): a NaN log-likelihood counts as unknown (0);
//    - NaN / infinite samples count as silence, samples are limited to
//      +-1e4, non-finite timestamps and clock offsets are ignored;
//    - a new sample rate starts the stream again (the JS resamples any);
//    - structured results, text lines, the CPU time of each decode, and the
//      calibration check result returned instead of set (setPpm()).
//
//  How it fits in
//  --------------
//  See Wspr.h.
//
//  Contributing
//  ------------
//  Keep the function names parallel to the JavaScript; mark deviations.
// ============================================================================
#include "Wspr.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>

namespace dec {

namespace {

const double kPi = 3.14159265358979323846;
const double kOutRate = 3000;    // (Addition to the JS) the decimated complex stream
const double kMixHz = 1500;      // ... centred here (audio 0..3000 Hz)
const double kBB = 375;          // the JS's BB: the slices' rate
const int kSPS = 256;            // samples a symbol at 375 Hz
const int kNF = 512, kH = 128;   // spectrogram: 512-point FFT, a frame every half symbol
const int kNBIN = 405;           // sync search bins (-205..199 half tones)
const double kSlot = 120, kTx = 1, kWin = 232, kMinLen = 113, kRingSec = 236;
const int kBigLog = 20;          // 2^20 points at 3000 Hz: up to 349 s
const double kChunkSec = 0.25;   // (Addition to the JS) slot check every 0.25 s of audio

bool finite(double v) { return v == v && v - v == 0; }   // not NaN, not +-inf
double jsRound(double v) { return std::floor(v + 0.5); }  // Math.round

const unsigned kPoly0 = 0xf2d05351u, kPoly1 = 0xe4613c47u;
int parity(unsigned v) {
    v ^= v >> 16; v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;
    return static_cast<int>(v & 1);
}

// the interleaving order: coded bit p goes to position ILV[p] (bit-reversed 8-bit counter, values under 162)
struct Ilv {
    int v[162];
    Ilv() {
        int n = 0;
        for (int k = 0; k < 256 && n < 162; ++k) {
            int r = 0;
            for (int j = 0; j < 8; ++j) r = (r << 1) | ((k >> j) & 1);
            if (r < 162) v[n++] = r;
        }
    }
};
const Ilv& ilv() { static const Ilv t; return t; }

int ch(char c) {
    return c >= '0' && c <= '9' ? c - '0' : c == ' ' ? 36 : c - 55;
}
bool isUpper(char c) { return c >= 'A' && c <= 'Z'; }
bool isDigit(char c) { return c >= '0' && c <= '9'; }

std::string trimUpper(const std::string& s) {
    std::size_t a = 0, b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t')) ++a;
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t')) --b;
    std::string r = s.substr(a, b - a);
    for (std::size_t i = 0; i < r.size(); ++i)
        if (r[i] >= 'a' && r[i] <= 'z') r[i] = static_cast<char>(r[i] - 32);
    return r;
}

double besselI0(double x) {
    double s = 1, t = 1;
    for (int k = 1; k < 60; ++k) {
        t *= (x / (2 * k)) * (x / (2 * k));
        s += t;
        if (t < 1e-12 * s) break;
    }
    return s;
}

std::string slotClock(double unix) {   // ft8SlotTime(): "HHMMSS"
    long long s = static_cast<long long>(std::floor(unix));
    long long d = s % 86400;
    if (d < 0) d += 86400;
    char buf[16];
    std::snprintf(buf, sizeof buf, "%02d%02d%02d", static_cast<int>(d / 3600), static_cast<int>(d / 60 % 60),
                  static_cast<int>(d % 60));
    return buf;
}

const WsprDecoder::Band kBands[] = {
    {"2200 m", 0.136e6},   {"630 m", 0.4742e6},  {"160 m", 1.8366e6},  {"80 m", 3.5686e6}, {"60 m", 5.2872e6},
    {"40 m", 7.0386e6},    {"30 m", 10.1387e6},  {"20 m", 14.0956e6},  {"17 m", 18.1046e6}, {"15 m", 21.0946e6},
    {"12 m", 24.9246e6},   {"10 m", 28.1246e6},  {"6 m", 50.293e6},    {"2 m", 144.489e6}};

}  // namespace

// ============================================================================
//  The codec
// ============================================================================
namespace wspr {

const unsigned char kPr3[162] = {
    1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,
    0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,1,1,0,0,0,1,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,1,
    0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,0,0};

bool validDbm(int dbm) {
    if (dbm < 0 || dbm > 60) return false;
    const int r = dbm % 10;
    return r == 0 || r == 3 || r == 7;
}

double dbmToWatts(int dbm) { return std::pow(10.0, (dbm - 30) / 10.0); }

std::string wattsText(int dbm) {   // wattsOf(): "1 W", "200 mW" (two significant figures)
    const double w = dbmToWatts(dbm);
    const double v = w >= 1 ? w : w * 1000;
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.2g", v);
    // (%.2g may give "1e+03": write those out)
    if (std::strchr(buf, 'e')) std::snprintf(buf, sizeof buf, "%.0f", v);
    return std::string(buf) + (w >= 1 ? " W" : " mW");
}

// ---- message: callsign (6 characters, the digit third), locator (4), power (dBm) -> 50 bits
bool pack(const std::string& callIn, const std::string& locIn, int dbm, unsigned char bits[50]) {
    const std::string call = trimUpper(callIn);
    std::string loc = locIn;
    for (std::size_t i = 0; i < loc.size(); ++i)
        if (loc[i] >= 'a' && loc[i] <= 'z') loc[i] = static_cast<char>(loc[i] - 32);
    if (call.size() < 3 || call.size() > 6) return false;
    for (std::size_t i = 0; i < call.size(); ++i)
        if (!isUpper(call[i]) && !isDigit(call[i])) return false;
    if (loc.size() != 4 || loc[0] < 'A' || loc[0] > 'R' || loc[1] < 'A' || loc[1] > 'R' || !isDigit(loc[2]) ||
        !isDigit(loc[3]))
        return false;
    std::string c;
    if (isDigit(call[2])) c = call;
    else if (isDigit(call[1])) c = " " + call;
    else return false;
    if (c.size() > 6) return false;
    for (std::size_t i = 3; i < c.size(); ++i)
        if (!isUpper(c[i]) && c[i] != ' ') return false;
    while (c.size() < 6) c += ' ';
    unsigned long n1 = static_cast<unsigned long>(ch(c[0]));
    n1 = n1 * 36 + static_cast<unsigned long>(ch(c[1]));
    n1 = n1 * 10 + static_cast<unsigned long>(ch(c[2]));
    for (int i = 3; i < 6; ++i) n1 = n1 * 27 + static_cast<unsigned long>(c[i] == ' ' ? 26 : c[i] - 65);
    const long ng = 180L * (179 - 10 * (loc[0] - 65) - (loc[2] - 48)) + 10 * (loc[1] - 65) + (loc[3] - 48);
    static const int adj[10] = {0, -1, 1, 0, -1, 2, 1, 0, -1, 1};
    int p = std::max(0, std::min(60, dbm));
    p = p + adj[p % 10];
    const unsigned long n2 = static_cast<unsigned long>(ng) * 128 + static_cast<unsigned long>(p + 64);
    int k = 0;
    for (int i = 27; i >= 0; --i) bits[k++] = static_cast<unsigned char>((n1 >> i) & 1);
    for (int i = 21; i >= 0; --i) bits[k++] = static_cast<unsigned char>((n2 >> i) & 1);
    return true;
}

// 50 bits -> { call, grid, dbm }, or false for anything but a standard message
bool unpack(const unsigned char bits[50], Message& out) {
    unsigned long n1 = 0, n2 = 0;
    for (int i = 0; i < 28; ++i) n1 = n1 * 2 + (bits[i] & 1);
    for (int i = 28; i < 50; ++i) n2 = n2 * 2 + (bits[i] & 1);
    const int dbm = static_cast<int>(n2 & 127) - 64;
    const unsigned long ng = n2 >> 7;
    if (!validDbm(dbm) || ng >= 32400) return false;   // (type 2 and 3 messages: compound and hashed calls)
    char suf[3];
    unsigned long n = n1;
    for (int i = 2; i >= 0; --i) {
        const unsigned long v = n % 27;
        n /= 27;
        suf[i] = v == 26 ? ' ' : static_cast<char>(65 + v);
    }
    const unsigned long d = n % 10; n /= 10;
    const unsigned long c1 = n % 36; n /= 36;
    const unsigned long c0 = n;
    if (c0 > 36) return false;
    struct Al { static char f(unsigned long v) { return v < 10 ? static_cast<char>('0' + v) : v == 36 ? ' ' : static_cast<char>(55 + v); } };
    std::string raw;
    raw += Al::f(c0); raw += Al::f(c1); raw += static_cast<char>('0' + d); raw.append(suf, 3);
    const std::string call = trimUpper(raw);
    // /^[A-Z0-9]{1,2}[0-9][A-Z]{1,3}$/
    // try a prefix of 1 or 2 characters, then the digit, then 1-3 letters
    bool ok = false;
    for (std::size_t pre = 1; pre <= 2 && !ok; ++pre) {
        if (call.size() < pre + 2) continue;
        bool good = true;
        for (std::size_t j = 0; j < pre; ++j) good = good && (isUpper(call[j]) || isDigit(call[j]));
        good = good && isDigit(call[pre]);
        const std::size_t rest = call.size() - pre - 1;
        good = good && rest >= 1 && rest <= 3;
        for (std::size_t j = pre + 1; j < call.size(); ++j) good = good && isUpper(call[j]);
        ok = good;
    }
    if (!ok) return false;
    const unsigned long q = ng / 180, r = ng % 180;
    const long L0 = (179 - static_cast<long>(q)) / 10, d2 = (179 - static_cast<long>(q)) % 10;
    const long L1 = static_cast<long>(r / 10), d3 = static_cast<long>(r % 10);
    if (L0 < 0 || L0 > 17 || L1 > 17) return false;
    out.call = call;
    out.grid.clear();
    out.grid += static_cast<char>(65 + L0); out.grid += static_cast<char>(65 + L1);
    out.grid += static_cast<char>('0' + d2); out.grid += static_cast<char>('0' + d3);
    out.dbm = dbm;
    return true;
}

// -> 162 coded bits (in transmission order, before the sync)
void encode(const unsigned char bits[50], unsigned char coded[162]) {
    unsigned char out[162];
    unsigned st = 0;
    for (int i = 0; i < 81; ++i) {
        const unsigned b = i < 50 ? (bits[i] & 1u) : 0u;
        st = ((st << 1) | b) & 0xffffffffu;
        out[2 * i] = static_cast<unsigned char>(parity(st & kPoly0));
        out[2 * i + 1] = static_cast<unsigned char>(parity(st & kPoly1));
    }
    for (int p = 0; p < 162; ++p) coded[ilv().v[p]] = out[p];
}

bool symbols(const std::string& call, const std::string& grid, int dbm, unsigned char sym[162]) {
    unsigned char b[50], c[162];
    if (!pack(call, grid, dbm, b)) return false;
    encode(b, c);
    for (int i = 0; i < 162; ++i) sym[i] = static_cast<unsigned char>(kPr3[i] + 2 * c[i]);
    return true;
}

// ---- the Fano sequential decoder (after Phil Karn KA9Q's, as used by wsprd). soft[162]: for each coded bit (in
// transmission order) the log-likelihood that it's 1; the 50 message bits, or false
bool fano(const double soft[162], unsigned char bits[50], double delta, int cycles) {
    const int nb = 81, tail = nb - 31;
    if (!(delta > 0) || !finite(delta)) delta = 2.4;
    if (cycles < 1) cycles = 1;
    const long long maxCycles = static_cast<long long>(cycles) * nb;
    double sy[162];
    for (int p = 0; p < 162; ++p) {
        const double v = soft[ilv().v[p]];   // (de-interleaved)
        sy[p] = finite(v) ? v : 0;           // (Addition to the JS) a NaN LLR counts as unknown
    }
    // branch metrics for each node and each possible pair of coded bits: the Fano metric log2(P(r|x)/P(r)) - R
    struct Bm {
        static double f(double l, int b) {
            const double p1 = 1 / (1 + std::exp(-l)), p = b ? p1 : 1 - p1;
            return std::log(2 * std::max(p, 1e-12)) / std::log(2.0) - 0.5;
        }
    };
    double met[81 * 4];
    for (int k = 0; k < nb; ++k)
        for (int s = 0; s < 4; ++s) met[k * 4 + s] = Bm::f(sy[2 * k], s >> 1) + Bm::f(sy[2 * k + 1], s & 1);
    unsigned enc[82];
    double gam[82], tm0[81], tm1[81];
    unsigned char ii[81];
    const double NEG = -HUGE_VAL;
    struct Sym { static int f(unsigned st) { return (parity(st & kPoly0) << 1) | parity(st & kPoly1); } };
    // the two branches from node k, best first (only 0 in the tail)
    struct Node {
        static void set(int k, int tailK, unsigned* enc, const double* met, double* tm0, double* tm1, unsigned char* ii,
                        double neg) {
            const int l = Sym::f(enc[k]);
            const double m0 = met[k * 4 + l], m1 = met[k * 4 + (l ^ 3)];
            if (k >= tailK) { tm0[k] = m0; tm1[k] = neg; }
            else if (m0 >= m1) { tm0[k] = m0; tm1[k] = m1; }
            else { tm0[k] = m1; tm1[k] = m0; enc[k] = enc[k] | 1u; }
            ii[k] = 0;
        }
    };
    int k = 0;
    double t = 0;
    enc[0] = 0; gam[0] = 0;
    Node::set(0, tail, enc, met, tm0, tm1, ii, NEG);
    for (long long cyc = 0; cyc < maxCycles; ++cyc) {
        const double ng = gam[k] + (ii[k] ? tm1[k] : tm0[k]);
        if (ng >= t) {   // forward
            if (gam[k] < t + delta) while (ng >= t + delta) t += delta;
            gam[k + 1] = ng; enc[k + 1] = (enc[k] << 1) & 0xffffffffu;
            if (++k == nb) {
                for (int j = 0; j < 50; ++j) bits[j] = static_cast<unsigned char>(enc[j] & 1);
                return true;
            }
            Node::set(k, tail, enc, met, tm0, tm1, ii, NEG);
            continue;
        }
        for (;;) {   // can't go forward: back up, or lower the threshold
            if (k == 0 || gam[k - 1] < t) {
                t -= delta;
                if (ii[k]) { ii[k] = 0; enc[k] ^= 1u; }
                break;
            }
            --k;
            if (k < tail && !ii[k]) { ii[k] = 1; enc[k] ^= 1u; break; }
        }
    }
    return false;
}

}  // namespace wspr

// ============================================================================
//  The decoder
// ============================================================================
const std::vector<WsprDecoder::Band>& WsprDecoder::dialList() {
    static const std::vector<Band> v(kBands, kBands + sizeof(kBands) / sizeof(kBands[0]));
    return v;
}

double WsprDecoder::dialForFrequency(double hz) {   // wsprDial()
    if (!finite(hz)) return 0;
    for (std::size_t i = 0; i < sizeof(kBands) / sizeof(kBands[0]); ++i)
        if (std::fabs(hz - kBands[i].dialHz) <= 300) return kBands[i].dialHz;
    return 0;
}

WsprDecoder::WsprDecoder()
    : dial_(0), fLo_(200), fHi_(2800), delta_(2.4), cycles_(4000), tsPending_(false), ts_(0), anchorSet_(false),
      anchor_(0), clockOff_(0), lastSlotSet_(false), lastSlot_(0), rate_(0), half_(0), phases_(32), histMask_(0),
      inN_(0), outK_(0), ratio_(1), ncoC_(1), ncoS_(0), ncoStepC_(1), ncoStepS_(0), ncoCount_(0), outN_(0),
      slotDone_(false), measured_(false), measuredOff_(0), measuredN_(0), measuredSpread_(0), slots_(0), total_(0),
      lastCount_(0), bestSnr_(0), lastDecodeAt_(0), lastCpu_(0), lastWall_(0), maxCpu_(0), calDial_(0),
      offer_(false), offerErr_(0), offerPpm_(0), offerN_(0), checkSet_(false), checkReady_(false),
      checkFlipped_(false), checkPpm_(0) {
    check_.old = check_.err = check_.f = 0;
    ring_.assign(static_cast<std::size_t>(kRingSec * kOutRate), std::complex<float>());
    A_.assign(162 * 4, 0.0);
}

void WsprDecoder::reset() {
    tsPending_ = false; anchorSet_ = false; anchor_ = 0;
    lastSlotSet_ = false; lastSlot_ = 0;
    clearStream();
    pending_.clear(); text_.clear(); keys_.clear(); lastDts_.clear();
    slotDone_ = false; measured_ = false; measuredOff_ = 0; measuredN_ = 0; measuredSpread_ = 0;
    slots_ = total_ = lastCount_ = 0; bestSnr_ = 0; lastDecodeAt_ = 0;
    lastCpu_ = lastWall_ = maxCpu_ = 0;
    notes_.clear(); calDial_ = dial_; offer_ = false; checkSet_ = false; checkReady_ = false;
}

void WsprDecoder::setTimestamp(double unixSeconds) {
    if (!finite(unixSeconds)) return;
    ts_ = unixSeconds; tsPending_ = true;
}

void WsprDecoder::setClockOffset(double pcMinusTrue) {
    if (finite(pcMinusTrue)) clockOff_ = pcMinusTrue;
}

void WsprDecoder::setDialFrequency(double hz) {
    dial_ = finite(hz) && hz > 0 ? hz : 0;
}

void WsprDecoder::setSearchRange(double loHz, double hiHz) {
    if (!finite(loHz) || !finite(hiHz)) return;
    if (loHz > hiHz) std::swap(loHz, hiHz);
    fLo_ = std::max(100.0, std::min(2900.0, loHz));
    fHi_ = std::max(fLo_ + 1, std::min(2900.0, hiHz));
}

void WsprDecoder::setFano(double delta, int cycles) {
    if (finite(delta) && delta > 0) delta_ = delta;
    if (cycles > 0) cycles_ = cycles;
}

double WsprDecoder::secondsHeld() const {
    return static_cast<double>(std::min<long long>(outN_, static_cast<long long>(ring_.size()))) / kOutRate;
}

// (Addition to the JS) the resampler: mixed down by 1500 Hz, then a Kaiser-windowed sinc low-pass (cut-off
// 1500 Hz, beta 8, 12 ms long: passband to ~1290 Hz, stopband from ~1710 Hz, ~80 dB) evaluated at each
// output instant from a table of 32 phases.  Output k is the audio at k / 3000 s after the stream's start.
void WsprDecoder::setRate(double rate) {
    rate_ = rate;
    ratio_ = rate / kOutRate;
    half_ = std::max(16, static_cast<int>(std::ceil(rate * 0.006)));
    const int taps = 2 * half_;
    taps_.assign(static_cast<std::size_t>(phases_ * taps), 0.f);
    const double fc = 1500 / rate, beta = 8, i0b = besselI0(beta);
    for (int p = 0; p < phases_; ++p) {
        const double mu = static_cast<double>(p) / phases_;
        double sum = 0;
        std::vector<double> h(static_cast<std::size_t>(taps));
        for (int j = 0; j < taps; ++j) {
            const double t = mu + half_ - 1 - j;   // (x - n)
            const double u = t / half_;
            double w = 0;
            if (std::fabs(u) < 1) w = besselI0(beta * std::sqrt(1 - u * u)) / i0b;
            const double a = 2 * kPi * fc * t;
            const double sinc = std::fabs(a) < 1e-12 ? 1 : std::sin(a) / a;
            h[static_cast<std::size_t>(j)] = 2 * fc * sinc * w;
            sum += h[static_cast<std::size_t>(j)];
        }
        for (int j = 0; j < taps; ++j)
            taps_[static_cast<std::size_t>(p * taps + j)] = static_cast<float>(h[static_cast<std::size_t>(j)] / sum);
    }
    long long L = 1;
    while (L < taps + static_cast<long long>(std::ceil(ratio_)) + 8) L <<= 1;
    histMask_ = L - 1;
    hist_.assign(static_cast<std::size_t>(2 * L), std::complex<float>());
    const double w = 2 * kPi * kMixHz / rate;
    ncoStepC_ = std::cos(w); ncoStepS_ = -std::sin(w);
    clearStream();
}

void WsprDecoder::clearStream() {
    inN_ = 0; outK_ = 0; outN_ = 0;
    std::fill(hist_.begin(), hist_.end(), std::complex<float>());
    ncoC_ = 1; ncoS_ = 0; ncoCount_ = 0;
    anchorSet_ = false;
}

void WsprDecoder::feed(const float* x, std::size_t n) {
    const long long L = histMask_ + 1;
    const int taps = 2 * half_;
    const std::size_t cap = ring_.size();
    for (std::size_t i = 0; i < n; ++i) {
        double v = x[i];
        if (!finite(v)) v = 0;                        // (Addition to the JS) NaN / inf: silence
        else if (v > 1e4) v = 1e4;
        else if (v < -1e4) v = -1e4;
        // mix: audio f -> f - 1500 Hz
        const std::complex<float> z(static_cast<float>(v * ncoC_), static_cast<float>(v * ncoS_));
        const double c = ncoC_ * ncoStepC_ - ncoS_ * ncoStepS_;
        ncoS_ = ncoC_ * ncoStepS_ + ncoS_ * ncoStepC_;
        ncoC_ = c;
        if (++ncoCount_ >= 4096) {
            ncoCount_ = 0;
            const double m = std::sqrt(ncoC_ * ncoC_ + ncoS_ * ncoS_);
            ncoC_ /= m; ncoS_ /= m;
        }
        const long long idx = inN_ & histMask_;
        hist_[static_cast<std::size_t>(idx)] = z;
        hist_[static_cast<std::size_t>(idx + L)] = z;
        ++inN_;
        for (;;) {
            const double xo = static_cast<double>(outK_) * ratio_;
            const long long fl = static_cast<long long>(std::floor(xo));
            if (fl + half_ + 1 >= inN_) break;
            int p = static_cast<int>((xo - static_cast<double>(fl)) * phases_ + 0.5);
            long long n0 = fl - half_ + 1;
            if (p >= phases_) { p = 0; ++n0; }
            const long long start = ((n0 % L) + L) % L;
            const std::complex<float>* hp = &hist_[static_cast<std::size_t>(start)];
            const float* tp = &taps_[static_cast<std::size_t>(p * taps)];
            float ar = 0, ai = 0;
            for (int j = 0; j < taps; ++j) { ar += hp[j].real() * tp[j]; ai += hp[j].imag() * tp[j]; }
            ring_[static_cast<std::size_t>(outN_ % static_cast<long long>(cap))] = std::complex<float>(ar, ai);
            ++outN_; ++outK_;
        }
    }
}

void WsprDecoder::process(const float* audio, std::size_t n, double rate) {
    if (!audio || n == 0 || !finite(rate) || rate < 6000 || rate > 1e6) return;
    if (rate != rate_) setRate(rate);
    // ---- ft8Feed(): when was the first sample? Blocks can only arrive late, never early: keep the earliest
    // estimate, creeping up slowly to follow the sound card's drift; a jump of over a second (a gap) starts again
    if (tsPending_) {
        const double est = ts_ - static_cast<double>(inN_) / rate_;
        if (!anchorSet_ || std::fabs(est - anchor_) > 1) anchor_ = est;
        else anchor_ = std::min(anchor_ + 2e-5, est);
        anchorSet_ = true;
        tsPending_ = false;
    }
    // (Addition to the JS) in chunks of 0.25 s, the slot check after each
    const std::size_t chunk = std::max<std::size_t>(1, static_cast<std::size_t>(rate_ * kChunkSec));
    for (std::size_t off = 0; off < n; off += chunk) {
        const std::size_t m = std::min(chunk, n - off);
        feed(audio + off, m);
        slotCheck();
    }
}

// just after each slot ends (by the corrected clock): decode
void WsprDecoder::slotCheck() {
    const double now = anchor_ + static_cast<double>(inN_) / rate_ - clockOff_;
    const long long slot = static_cast<long long>(std::floor((now - 0.4) / kSlot));
    if (!lastSlotSet_) { lastSlotSet_ = true; lastSlot_ = slot; return; }
    if (slot == lastSlot_) return;
    lastSlot_ = slot;
    decodeNow();
}

int WsprDecoder::decodeNow() {
    const long long cap = static_cast<long long>(ring_.size());
    const long long len = std::min(std::min(outN_, static_cast<long long>(jsRound(kWin * kOutRate))), cap);
    if (len < kMinLen * kOutRate) return 0;
    const std::clock_t c0 = std::clock();
    const std::chrono::steady_clock::time_point w0 = std::chrono::steady_clock::now();
    const int N = 1 << kBigLog, M = N / 8;   // (3000 Hz / 8 = 375 Hz slices)
    if (static_cast<int>(big_.size()) != N) {
        big_.assign(static_cast<std::size_t>(N), std::complex<float>());
        slice_.assign(static_cast<std::size_t>(M), std::complex<float>());
        frame_.assign(kNF, std::complex<float>());
        fftBig_.resize(N); fftSlice_.resize(M); fft512_.resize(kNF);
    }
    // the whole window, transformed once
    const long long s0 = outN_ - len;
    for (long long i = 0; i < len; ++i) big_[static_cast<std::size_t>(i)] = ring_[static_cast<std::size_t>((s0 + i) % cap)];
    std::fill(big_.begin() + len, big_.end(), std::complex<float>());
    fftBig_.forward(&big_[0]);
    const int nb = static_cast<int>(len / 8);
    // (start times: as far as the audio allows)
    const double span = std::max(4.0, std::min(121.0, static_cast<double>(len) / kOutRate - 111.5));
    std::vector<WsprDecode> res;
    std::vector<std::string> seen;
    const float scale = 1.0f / static_cast<float>(N);
    for (double fc = fLo_ + 150; fc - 150 < fHi_; fc += 300) {
        // the slice around fc, back to the time domain: a complex signal at 375 Hz with fc at 0 Hz
        const long long cbin = static_cast<long long>(jsRound((fc - kMixHz) * N / kOutRate));
        const double fcExact = kMixHz + static_cast<double>(cbin) * kOutRate / N;
        std::fill(slice_.begin(), slice_.end(), std::complex<float>());
        for (int k = -M / 2; k < M / 2; ++k) {
            const long long src = cbin + k;
            if (src <= -N / 2 || src >= N / 2) continue;
            const std::size_t si = static_cast<std::size_t>(src < 0 ? src + N : src);
            slice_[static_cast<std::size_t>((k + M) % M)] = std::conj(big_[si]);
        }
        fftSlice_.forward(&slice_[0]);   // (inverse, by conjugates; scaled so the slice keeps the audio's level)
        for (int k = 0; k < M; ++k) slice_[static_cast<std::size_t>(k)] = std::conj(slice_[static_cast<std::size_t>(k)]) * scale;
        decodeSlice(&slice_[0], nb, fcExact, span, res, seen);
    }
    // ---- ft8Done() for WSPR
    const double t0 = anchor_ + static_cast<double>(s0) / kOutRate - clockOff_;   // the window's start, corrected
    const double now = anchor_ + static_cast<double>(inN_) / rate_ - clockOff_;
    lastDts_.clear();
    lastCount_ = 0;
    bestSnr_ = -99;
    for (std::size_t i = 0; i < res.size(); ++i) {
        WsprDecode& r = res[i];
        noteFreq(r.audioHz, now);
        const double t = t0 + r.dt;   // (r.dt holds the start in seconds from the window's start here)
        const long long slot = static_cast<long long>(jsRound((t - kTx) / kSlot));
        r.dt = t - kTx - static_cast<double>(slot) * kSlot;
        r.slot = slot;
        r.slotStart = static_cast<double>(slot) * kSlot;
        char kb[32];
        std::snprintf(kb, sizeof kb, "%lld|", slot);
        const std::string key = std::string(kb) + r.text;
        bool dup = false;
        for (std::size_t j = 0; j < keys_.size(); ++j)
            if (keys_[j].second == key) { dup = true; break; }
        if (dup) continue;                // (heard in the previous window already)
        keys_.push_back(std::make_pair(slot, key));
        r.rfHz = dial_ > 0 ? dial_ + r.audioHz : 0;
        char line[256];
        char rf[48] = "";
        if (dial_ > 0) std::snprintf(rf, sizeof rf, "  %.6f MHz", r.rfHz / 1e6);
        std::snprintf(line, sizeof line, "%s  %3d dB  %+5.1f s  %4.0f Hz%s  drift %+.0f  %s %s %d dBm (%s)\n",
                      slotClock(r.slotStart).c_str(), r.snrDb, r.dt, r.audioHz, rf, r.drift, r.call.c_str(),
                      r.grid.c_str(), r.dbm, wspr::wattsText(r.dbm).c_str());
        r.line = line;
        text_ += line;
        pending_.push_back(r);
        lastDts_.push_back(r.dt);
        ++lastCount_; ++total_;
        bestSnr_ = std::max(bestSnr_, r.snrDb);
    }
    // keep the keys of the last few slots only
    const long long nowSlot = static_cast<long long>(std::floor(now / kSlot));
    std::vector<std::pair<long long, std::string> > kept;
    for (std::size_t j = 0; j < keys_.size(); ++j)
        if (keys_[j].first >= nowSlot - 4) kept.push_back(keys_[j]);
    keys_.swap(kept);
    if (keys_.size() > 400) keys_.erase(keys_.begin(), keys_.begin() + static_cast<long>(keys_.size() - 400));
    // the clock as these decodes see it (the JS moves its correction by the median dt: see Clock.h)
    if (!lastDts_.empty()) {
        std::vector<double> d(lastDts_);
        std::sort(d.begin(), d.end());
        measured_ = true;
        measuredOff_ = clockOff_ + d[d.size() >> 1];
        measuredN_ = static_cast<int>(d.size());
        measuredSpread_ = d.back() - d.front();
    }
    ++slots_;
    slotDone_ = true;
    lastDecodeAt_ = now;
    lastCpu_ = static_cast<double>(std::clock() - c0) / CLOCKS_PER_SEC;
    lastWall_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count();
    maxCpu_ = std::max(maxCpu_, lastCpu_);
    return static_cast<int>(lastCount_);
}

void WsprDecoder::decodeSlice(const std::complex<float>* br, int len, double fc, double span,
                              std::vector<WsprDecode>& out, std::vector<std::string>& seen) {
    // a spectrogram: one symbol (256 samples) per frame, every half symbol, bins of half a tone (512-point FFT)
    const int nfr = (len - kSPS) / kH;
    if (nfr < 2 * 161 + 2) return;
    P_.resize(static_cast<std::size_t>(nfr) * kNF);
    for (int f = 0; f < nfr; ++f) {
        std::fill(frame_.begin(), frame_.end(), std::complex<float>());
        for (int i = 0; i < kSPS; ++i) frame_[static_cast<std::size_t>(i)] = br[f * kH + i];
        fft512_.forward(&frame_[0]);
        float* p = &P_[static_cast<std::size_t>(f) * kNF];
        for (int k = 0; k < kNF; ++k) p[k] = std::norm(frame_[static_cast<std::size_t>(k)]);
    }
    const double hz = wspr::kToneHz / 2;   // (bin k: k * 0.732 Hz from fc)
    // (the JS also works out the slice's median noise here, unused: left out)
    // the sync search: tone 0 at bin b (tones at b, b+2, b+4, b+6), symbol i at frame m + 2i. Start times are
    // searched across `span` seconds from the start of the audio. Each frame's sync value for each bin is worked
    // out once, so each start time tried costs only a multiply-add a symbol.
    const int mHi = std::min(static_cast<int>(jsRound(span / (kSPS / kBB) * 2)), nfr - 2 * 161 - 1);
    if (mHi < 0) return;
    V_.resize(static_cast<std::size_t>(kNBIN) * nfr);
    for (int f = 0; f < nfr; ++f) {
        const float* p = &P_[static_cast<std::size_t>(f) * kNF];
        for (int j = 0; j < kNBIN; ++j) {
            const int b = j - 205;
            const double p0 = p[(b + kNF) % kNF], p1 = p[(b + 2 + kNF) % kNF], p2 = p[(b + 4 + kNF) % kNF],
                         p3 = p[(b + 6 + kNF) % kNF];
            V_[static_cast<std::size_t>(j) * nfr + f] = static_cast<float>(((p1 + p3) - (p0 + p2)) / (p0 + p1 + p2 + p3 + 1e-30));
        }
    }
    float sg[162];
    for (int i = 0; i < 162; ++i) sg[i] = wspr::kPr3[i] ? 1.f : -1.f;
    cand_.clear();
    for (int j = 0; j < kNBIN - 6; ++j) {
        const float* row = &V_[static_cast<std::size_t>(j) * nfr];
        float best = -1;
        int bm = 0;
        for (int m = 0; m <= mHi; ++m) {
            float s = 0;
            const float* k = row + m;
            for (int i = 0; i < 162; ++i) s += sg[i] * k[2 * i];
            if (s > best) { best = s; bm = m; }
        }
        Cand c; c.sync = best / 162.0; c.b = j - 205; c.m = bm;
        cand_.push_back(c);
    }
    struct ByScore { bool operator()(const Cand& a, const Cand& b) const { return a.sync > b.sync; } };
    std::stable_sort(cand_.begin(), cand_.end(), ByScore());
    std::vector<Cand> kept;
    for (std::size_t i = 0; i < cand_.size(); ++i) {
        const Cand& c = cand_[i];
        if (c.sync < 0.08 || kept.size() >= 12) break;
        bool near = false;
        for (std::size_t j = 0; j < kept.size(); ++j) near = near || std::abs(kept[j].b - c.b) <= 3;
        if (near) continue;
        kept.push_back(c);
    }
    for (std::size_t ci = 0; ci < kept.size(); ++ci) {
        const Cand& c = kept[ci];
        // tone 0's offset from fc (Hz), symbol 0's sample
        int st = c.m * kH;
        double f0 = c.b * hz, score = 0;
        refine(br, len, st, f0, 0, score, 0);
        // (Addition to the JS) the drift search, as wsprd's: -4..+4 Hz over the transmission
        double bestDrift = 0, bestScore = score;
        for (int d = -4; d <= 4; ++d) {
            if (!d) continue;
            const double s = syncScore(br, len, st, f0, d);
            if (s > bestScore) { bestScore = s; bestDrift = d; }
        }
        unsigned char bits[50];
        bool ok = false;
        Demod res;
        if (bestDrift != 0 && bestScore > score * 1.03) {
            int st2 = st;
            double f2 = f0, sc2 = bestScore;
            refine(br, len, st2, f2, bestDrift, sc2, 2);
            finish(br, len, st2, f2, bestDrift, res);
            ok = wspr::fano(res.soft, bits, delta_, cycles_);
        }
        if (!ok) {   // (the JS's own: no drift)
            finish(br, len, st, f0, 0, res);
            ok = wspr::fano(res.soft, bits, delta_, cycles_);
        }
        if (!ok) continue;
        wspr::Message u;
        if (!wspr::unpack(bits, u)) continue;
        char kb[64];
        std::snprintf(kb, sizeof kb, "%s%s%d", u.call.c_str(), u.grid.c_str(), u.dbm);
        if (std::find(seen.begin(), seen.end(), std::string(kb)) != seen.end()) continue;
        seen.push_back(kb);
        WsprDecode d;
        d.audioHz = fc + res.f0 + 1.5 * wspr::kToneHz;   // (the signal's centre, as WSPR reports it)
        d.dt = res.start / kBB;                          // (seconds from the window's start; made a dt by the caller)
        d.snrDb = static_cast<int>(jsRound(10 * std::log10(std::max(1e-6, res.sig / res.noise)) -
                                           10 * std::log10(2500 / wspr::kToneHz)));
        d.sync = std::floor(c.sync * 100 + 0.5) / 100;
        d.drift = res.drift;
        d.call = u.call; d.grid = u.grid; d.dbm = u.dbm; d.watts = wspr::dbmToWatts(u.dbm);
        char tb[64];
        std::snprintf(tb, sizeof tb, "%s %s %d dBm", u.call.c_str(), u.grid.c_str(), u.dbm);
        d.text = tb;
        out.push_back(d);
    }
}

// each symbol's four tone amplitudes at the exact timing and frequency (tone t of symbol i at
// f + t * TONE + drift * (i - 81) / 162 Hz).  (Addition to the JS, same result: the tones are 375/256 Hz apart,
// so over a symbol they are the bins of a 256-point DFT: each sample is mixed down by f once and the four
// tones taken from a fixed table, and each symbol's phase starts at 0 (only the amplitudes are used), instead
// of four rotating phasors started at the absolute phase.)
void WsprDecoder::amps(const std::complex<float>* s, int len, int st, double f, double drift, int step, double* A) const {
    struct Tab {
        double c[4][kSPS], s[4][kSPS];
        Tab() {
            for (int t = 0; t < 4; ++t)
                for (int k = 0; k < kSPS; ++k) { c[t][k] = std::cos(2 * kPi * t * k / kSPS); s[t][k] = -std::sin(2 * kPi * t * k / kSPS); }
        }
    };
    static const Tab tab;
    double ar = 1, ai = 0, lastF = HUGE_VAL;
    for (int i = 0; i < 162; ++i) {
        const int o = st + i * kSPS;
        double* a = A + i * 4;
        if (o < 0 || o + kSPS > len) { a[0] = a[1] = a[2] = a[3] = 0; continue; }
        const double fi = f + drift * (i - 81) / 162.0;
        if (fi != lastF) { const double w = 2 * kPi * fi / kBB * step; ar = std::cos(w); ai = -std::sin(w); lastF = fi; }
        double pr = 1, pi = 0, c0r = 0, c0i = 0, c1r = 0, c1i = 0, c2r = 0, c2i = 0, c3r = 0, c3i = 0;
        for (int k = 0; k < kSPS; k += step) {
            const double xr = s[o + k].real(), xi = s[o + k].imag();
            const double vr = xr * pr - xi * pi, vi = xr * pi + xi * pr;   // mixed down by fi
            c0r += vr; c0i += vi;
            c1r += vr * tab.c[1][k] - vi * tab.s[1][k]; c1i += vr * tab.s[1][k] + vi * tab.c[1][k];
            c2r += vr * tab.c[2][k] - vi * tab.s[2][k]; c2i += vr * tab.s[2][k] + vi * tab.c[2][k];
            c3r += vr * tab.c[3][k] - vi * tab.s[3][k]; c3i += vr * tab.s[3][k] + vi * tab.c[3][k];
            const double nr = pr * ar - pi * ai;
            pi = pr * ai + pi * ar; pr = nr;
        }
        a[0] = std::sqrt(c0r * c0r + c0i * c0i); a[1] = std::sqrt(c1r * c1r + c1i * c1i);
        a[2] = std::sqrt(c2r * c2r + c2i * c2i); a[3] = std::sqrt(c3r * c3r + c3i * c3i);
    }
}

double WsprDecoder::syncScore(const std::complex<float>* s, int len, int st, double f, double drift) {
    amps(s, len, st, f, drift, 4, &A_[0]);
    double sc = 0;
    for (int i = 0; i < 162; ++i) {
        const double* a = &A_[static_cast<std::size_t>(i) * 4];
        sc += (wspr::kPr3[i] ? 1 : -1) * ((a[1] + a[3]) - (a[0] + a[2]));
    }
    return sc;
}

// fine-tuned on the sync bits: halving steps of time (samples) and frequency (Hz), greedy
void WsprDecoder::refine(const std::complex<float>* s, int len, int& bs, double& bf, double drift, double& bp,
                         int firstLevel) {
    static const int dsT[4] = {32, 16, 8, 4};
    static const double dfT[4] = {0.18, 0.09, 0.045, 0.02};
    bp = syncScore(s, len, bs, bf, drift);
    for (int l = firstLevel; l < 4; ++l) {
        const int da[4] = {-dsT[l], dsT[l], 0, 0};
        const double db[4] = {0, 0, -dfT[l], dfT[l]};
        for (int q = 0; q < 4; ++q) {
            const double p = syncScore(s, len, bs + da[q], bf + db[q], drift);
            if (p > bp) { bp = p; bs += da[q]; bf += db[q]; }
        }
    }
}

// each coded bit's log-likelihood (the sync bit decides which two tones carry it)
void WsprDecoder::finish(const std::complex<float>* s, int len, int bs, double bf, double drift, Demod& res) {
    double* A = &A_[0];
    amps(s, len, bs, bf, drift, 1, A);
    // bit LLRs: data tone (sync + 2) against (sync); amplitudes normalised, Gaussian approximation
    double z[162], s2 = 0, sig = 0, nz = 0;
    for (int i = 0; i < 162; ++i) {
        const int sy = wspr::kPr3[i];
        const double a1 = A[i * 4 + sy + 2], a0 = A[i * 4 + sy];
        z[i] = a1 - a0; s2 += z[i] * z[i];
        sig += std::max(a1, a0) * std::max(a1, a0);
        nz += A[i * 4 + (1 - sy)] * A[i * 4 + (1 - sy)] + A[i * 4 + (3 - sy)] * A[i * 4 + (3 - sy)];
    }
    double rms = std::sqrt(s2 / 162);
    if (!(rms > 0) || !finite(rms)) rms = 1;
    double mabs = 0;
    for (int i = 0; i < 162; ++i) { z[i] /= rms; mabs += std::fabs(z[i]); }
    mabs /= 162;
    // the bit values z are +-m plus Gaussian noise of spread sd, with m^2 + sd^2 = 1 (normalised): find the m whose
    // expected mean |z| matches what was measured (sd*sqrt(2/pi)*exp(-m^2/2sd^2) + m*(1 - 2*Phi(-m/sd)))
    struct F {
        static double Phi(double v) { return 0.5 * (1 + std::tanh(0.7978845608 * (v + 0.044715 * v * v * v))); }
        static double meanAbs(double mm) {
            const double sd = std::sqrt(std::max(1e-4, 1 - mm * mm));
            return sd * 0.7978845608 * std::exp(-mm * mm / (2 * sd * sd)) + mm * (1 - 2 * Phi(-mm / sd));
        }
    };
    double lo = 0, hi = 0.995;
    for (int it = 0; it < 40; ++it) {
        const double mid = (lo + hi) / 2;
        if (F::meanAbs(mid) < mabs) lo = mid; else hi = mid;
    }
    const double m = std::max(0.05, (lo + hi) / 2), sd2 = std::max(0.01, 1 - m * m);
    for (int i = 0; i < 162; ++i) res.soft[i] = std::max(-30.0, std::min(30.0, 2 * m * z[i] / sd2));
    double noise = nz / (2 * 162);
    if (!(noise > 0)) noise = 1e-20;
    res.noise = noise;
    res.sig = std::max(1e-20, sig / 162 - noise);
    res.start = bs; res.f0 = bf; res.drift = drift;
}

// ---- wsprNoteFreq(): where the stations land
void WsprDecoder::noteFreq(double f, double t) {
    if (dial_ != calDial_) { calDial_ = dial_; notes_.clear(); offer_ = false; }
    Note nt; nt.f = f; nt.t = t;
    notes_.push_back(nt);
    std::vector<Note> keep;
    for (std::size_t i = 0; i < notes_.size(); ++i)
        if (t - notes_[i].t < 20 * 60) keep.push_back(notes_[i]);
    if (keep.size() > 40) keep.erase(keep.begin(), keep.begin() + static_cast<long>(keep.size() - 40));
    notes_.swap(keep);
    if (notes_.size() < 4) return;
    std::vector<double> fs;
    for (std::size_t i = 0; i < notes_.size(); ++i) fs.push_back(notes_[i].f);
    std::sort(fs.begin(), fs.end());
    const double med = fs[fs.size() >> 1], err = med - 1500;
    if (checkSet_) {   // after a correction: did it help? (if not, the other sign, as the time-station calibration does)
        const Check c = check_;
        checkSet_ = false;
        checkReady_ = true;
        char buf[160];
        if (std::fabs(err) > std::fabs(c.err) * 0.6) {
            checkFlipped_ = true;
            checkPpm_ = c.f > 0 ? c.old - c.err / c.f * 1e6 : c.old;
            notes_.clear();
            std::snprintf(buf, sizeof buf, "WSPR calibration: corrected the other way, to %.2f ppm", checkPpm_);
        } else {
            checkFlipped_ = false;
            checkPpm_ = 0;
            std::snprintf(buf, sizeof buf, "WSPR calibration confirmed: the stations are now centred within %.0f Hz of 1500 Hz",
                          std::fabs(err));
        }
        checkText_ = buf;
        return;
    }
    offer_ = std::fabs(err) > 80 && dial_ > 0;
    offerErr_ = err;
    offerPpm_ = dial_ > 0 ? err / dial_ * 1e6 : 0;
    offerN_ = static_cast<int>(fs.size());
}

WsprCalibration WsprDecoder::calibration() const {
    WsprCalibration c;
    c.dialHz = dial_;
    c.count = static_cast<int>(notes_.size());
    if (!notes_.empty()) {
        std::vector<double> fs;
        for (std::size_t i = 0; i < notes_.size(); ++i) fs.push_back(notes_[i].f);
        std::sort(fs.begin(), fs.end());
        c.medianHz = fs[fs.size() >> 1];
        c.errorHz = c.medianHz - 1500;
        c.ppm = dial_ > 0 ? c.errorHz / dial_ * 1e6 : 0;
    }
    c.ready = notes_.size() >= 4;
    c.offer = offer_;
    if (offer_) { c.errorHz = offerErr_; c.ppm = offerPpm_; }
    // wsprCalText()
    char buf[200];
    if (!offer_) c.text = c.ready ? " \xC2\xB7 stick on frequency" : "";
    else {
        std::snprintf(buf, sizeof buf, " \xC2\xB7 stations centred on %.0f Hz, not 1500: the stick looks about %.1f ppm off",
                      1500 + offerErr_, std::fabs(offerPpm_));
        c.text = buf;
    }
    return c;
}

void WsprDecoder::calibrationApplied(double oldPpm) {   // wsprCalApply()
    if (!offer_ || !finite(oldPpm)) return;
    check_.old = oldPpm; check_.err = offerErr_; check_.f = dial_;
    checkSet_ = true; offer_ = false; notes_.clear();
}

bool WsprDecoder::takeCalibrationCheck(bool& flipped, double& suggestedPpm, std::string& text) {
    if (!checkReady_) return false;
    checkReady_ = false;
    flipped = checkFlipped_; suggestedPpm = checkPpm_; text = checkText_;
    return true;
}

void WsprDecoder::takeText(std::string& out) {
    out += text_;
    text_.clear();
}

void WsprDecoder::takeDecodes(std::vector<WsprDecode>& out) {
    out.insert(out.end(), pending_.begin(), pending_.end());
    pending_.clear();
}

bool WsprDecoder::newSlotDone() {
    const bool d = slotDone_;
    slotDone_ = false;
    return d;
}

bool WsprDecoder::measuredClockOffset(double& pcMinusTrue, int* count, double* spread) const {
    if (!measured_) return false;
    pcMinusTrue = measuredOff_;
    if (count) *count = measuredN_;
    if (spread) *spread = measuredSpread_;
    return true;
}

Status WsprDecoder::status() const {
    Status s;
    s.mode = "WSPR";
    s.baud = wspr::kToneHz;
    s.shiftHz = wspr::kToneHz;
    s.characters = total_;
    s.signal = s.locked = slots_ > 0 && lastCount_ > 0;
    s.snrDb = lastCount_ ? bestSnr_ : 0;
    const WsprCalibration c = calibration();
    s.toneHz = c.count ? c.medianHz : 1500;
    const double pcNow = rate_ > 0 ? anchor_ + static_cast<double>(inN_) / rate_ : 0, now = pcNow - clockOff_;
    if (measured_) {
        s.timeValid = true;
        s.clockOffset = -measuredOff_;
        s.clockError = std::max(0.1, measuredSpread_ / 2);
        s.stationTime = pcNow - measuredOff_;
        s.station = "WSPR";
    }
    // the box's info line: "next decode in N s · last: k new · clock OK" + wsprCalText()
    const double left = std::ceil(kSlot - std::fmod(std::fmod(now - 0.4, kSlot) + kSlot, kSlot));
    char buf[200];
    std::snprintf(buf, sizeof buf, "next decode in %.0f s", left);
    std::string info = buf;
    if (slots_) { std::snprintf(buf, sizeof buf, " \xC2\xB7 last: %u new", lastCount_); info += buf; }
    if (std::fabs(clockOff_) < 0.05) info += " \xC2\xB7 clock OK";
    else {
        std::snprintf(buf, sizeof buf, " \xC2\xB7 clock corrected %s%.1f s", clockOff_ > 0 ? "+" : "-", std::fabs(clockOff_));
        info += buf;
    }
    info += c.text;
    s.info = info;
    s.quality = lastCount_ ? std::min(1.0, std::max(0.0, (bestSnr_ + 30) / 30.0)) : 0;
    return s;
}

}  // namespace dec
