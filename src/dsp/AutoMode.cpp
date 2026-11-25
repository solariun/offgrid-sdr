// ============================================================================
//  offgrid-dsp - reusable C++11 radio signal processing (part of OFFgrid-SDR
//  TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/AutoMode.cpp
//  Layer:  Library liboffgrid-dsp
//
//  What this file does
//  -------------------
//  AutoDetector (see AutoMode.h): a port of OFFgrid-SDR's autoDetect,
//  fixedModeBand, dabChannelNear, fcalFeed (the listen), autoDrmCheck,
//  ft8Runs, ssbRefine, keyedRunMs, pskBitTiming / pskSpeed and the frequency
//  tables they use (diddy-boy, index.html).  The JS's diagnostics events are
//  left out; its "listen again" and fall-back are kept.
//
//  How it fits in
//  --------------
//  Uses ChannelExtractor and detail::fftDouble from SignalMonitor.cpp.
//  Everything else is local to this file.
//
//  Contributing
//  ------------
//  Keep the order of the rules: it matters (fax before carriers, PSK before
//  Morse, AM sidebands before RTTY ...).  The bracketed comments are the
//  JS's reasons.
// ============================================================================
#include "AutoMode.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace dsp {

namespace {

const double kPi = 3.14159265358979323846;
const double kNaN = std::numeric_limits<double>::quiet_NaN();
bool isNum(double x) { return x == x && std::fabs(x) <= 1.7e308; }
double jsRound(double x) { return std::floor(x + 0.5); }
std::string fmt(const char* f, double a) {
    char b[96];
    std::snprintf(b, sizeof b, f, a);
    return b;
}
const char* kDot = "\xC2\xB7";  // " · "

// ---- frequency tables (index.html) -------------------------------------------
const double kNavtex[] = {490e3, 518e3, 4209.5e3};
const double kDsc[] = {2187.5e3, 4207.5e3, 6312e3, 8414.5e3, 12577e3, 16804.5e3};
const double kSstv[] = {14230e3, 14233e3, 21340e3, 28680e3, 7171e3, 3845e3};
const double kPskF[] = {1838e3, 3580e3, 7040e3, 7070e3, 10142e3, 14070e3, 18100e3, 21070e3, 24920e3, 28120e3};
const double kFt8[] = {1.840e6, 3.573e6, 5.357e6, 7.074e6, 10.136e6, 14.074e6, 18.100e6, 21.074e6, 24.915e6, 28.074e6, 50.313e6, 144.174e6};
const double kFt4[] = {3.575e6, 7.0475e6, 10.140e6, 14.080e6, 18.104e6, 21.140e6, 24.919e6, 28.180e6, 50.318e6, 144.170e6};
const double kWspr[] = {0.136e6, 0.4742e6, 1.8366e6, 3.5686e6, 5.2872e6, 7.0386e6, 10.1387e6, 14.0956e6, 18.1046e6, 21.0946e6, 24.9246e6, 28.1246e6, 50.293e6, 144.489e6};
const double kAcars[] = {131.55e6, 131.525e6, 131.725e6, 131.825e6, 130.025e6, 129.125e6, 130.45e6, 131.45e6, 136.9e6, 136.75e6, 136.8e6};
const double kVdl[] = {136.650e6, 136.675e6, 136.700e6, 136.725e6, 136.775e6, 136.800e6, 136.825e6, 136.875e6, 136.900e6, 136.975e6};
const double kAis[] = {161.975e6, 162.025e6};
const double kAprs[] = {144.800e6, 144.390e6, 145.175e6, 144.575e6, 144.660e6, 144.640e6, 144.930e6, 145.570e6, 145.825e6};
// fax stations' frequencies (kHz): DWD, Northwood, Boston, New Orleans, Point Reyes, Kodiak, Honolulu
const double kFaxK[] = {3855, 7880, 13882.5, 2618.5, 4610, 8040, 11086.5, 4235, 6340.5, 9110, 12750, 4317.9, 8503.9, 12789.9,
                        17146.4, 4346, 8682, 12786, 17151.2, 22527, 2054, 4298, 8459, 12410.6, 9982.5, 11090, 16135};
struct TimeSt {
    double k;
    const char* name;
};
const TimeSt kTime[] = {{3330, "CHU, Canada"}, {7850, "CHU, Canada"}, {14670, "CHU, Canada"}, {2500, "WWV / WWVH, USA"},
                        {5000, "WWV / WWVH, USA"}, {10000, "WWV / WWVH, USA"}, {15000, "WWV / WWVH, USA"}, {20000, "WWV, USA"},
                        {25000, "WWV, USA"}, {4996, "RWM, Russia"}, {9996, "RWM, Russia"}, {14996, "RWM, Russia"}};
// HF amateur bands with their usual sideband (SW_BANDS' "ham" rows below 30 MHz), kHz
struct HamSb {
    double lo, hi;
    bool lsb;
};
const HamSb kHamSb[] = {{1800, 2000, true}, {3500, 4000, true}, {5351.5, 5366.5, false}, {7000, 7300, true},
                        {10100, 10150, false}, {14000, 14350, false}, {18068, 18168, false}, {21000, 21450, false},
                        {24890, 24990, false}, {28000, 29700, false}, {26960, 27410, false}, {27596, 27996, false}};
// HAM_BANDS (Hz): where Hellschreiber is sent
const double kHamBands[][2] = {{0.1357e6, 0.1378e6}, {0.472e6, 0.479e6}, {1.8e6, 2.0e6}, {3.5e6, 4.0e6}, {5.25e6, 5.45e6},
                               {7.0e6, 7.3e6}, {10.1e6, 10.15e6}, {14.0e6, 14.35e6}, {18.068e6, 18.168e6}, {21.0e6, 21.45e6},
                               {24.89e6, 24.99e6}, {28.0e6, 29.7e6}, {50e6, 54e6}, {69.9e6, 70.5e6}, {144e6, 148e6}};
// DAB channels (EN 300 401): name, centre kHz
struct DabCh {
    const char* name;
    double k;
};
const DabCh kDab[] = {{"5A", 174928}, {"5B", 176640}, {"5C", 178352}, {"5D", 180064}, {"6A", 181936}, {"6B", 183648},
                      {"6C", 185360}, {"6D", 187072}, {"7A", 188928}, {"7B", 190640}, {"7C", 192352}, {"7D", 194064},
                      {"8A", 195936}, {"8B", 197648}, {"8C", 199360}, {"8D", 201072}, {"9A", 202928}, {"9B", 204640},
                      {"9C", 206352}, {"9D", 208064}, {"10A", 209936}, {"10N", 210096}, {"10B", 211648}, {"10C", 213360},
                      {"10D", 215072}, {"11A", 216928}, {"11N", 217088}, {"11B", 218640}, {"11C", 220352}, {"11D", 222064},
                      {"12A", 223936}, {"12N", 224096}, {"12B", 225648}, {"12C", 227360}, {"12D", 229072}, {"13A", 230784},
                      {"13B", 232496}, {"13C", 234208}, {"13D", 235776}, {"13E", 237488}, {"13F", 239200}};

#define COUNT(a) (sizeof(a) / sizeof((a)[0]))

bool nearAny(const double* list, std::size_t n, double f, double tol) {
    for (std::size_t i = 0; i < n; ++i)
        if (std::fabs(f - list[i]) < tol) return true;
    return false;
}
// first in list with |f - x| <= tol (JS find), or NaN
double findWithin(const double* list, std::size_t n, double f, double tol) {
    for (std::size_t i = 0; i < n; ++i)
        if (std::fabs(f - list[i]) <= tol) return list[i];
    return kNaN;
}
// the HF ham band's usual sideband: 1 USB, -1 LSB, 0 not in one
int hamSideband(double dial) {
    if (dial >= 30e6) return 0;
    for (std::size_t i = 0; i < COUNT(kHamSb); ++i)
        if (dial / 1e3 >= kHamSb[i].lo && dial / 1e3 <= kHamSb[i].hi) return kHamSb[i].lsb ? -1 : 1;
    return 0;
}
// SSTV's sideband on shortwave: LSB on the LSB ham bands (40 m, 80 m, 160 m), USB elsewhere
bool sstvLsb(double f) { return hamSideband(f) == -1; }

// ---- result helpers ---------------------------------------------------------------
void setMode(AutoResult& r, const std::string& js, double dial) {
    r.jsMode = js;
    r.audioShiftHz = 0;
    r.decoder = "";
    r.mode = js;
    if (js == "NBFM") r.mode = "NFM";
    else if (js == "WBFM") r.mode = "WFM";
    else if (js == "CW") r.decoder = "CW";
    else if (js == "RTTY" || js == "NAVTEX" || js == "DSC") {
        r.mode = "USB";
        r.decoder = js;
        r.audioShiftHz = 1500;  // (RTTY_AUDIO: the listed (centre) frequency at 1500 Hz audio)
    } else if (js == "FAX") {
        r.mode = "USB";
        r.decoder = "FAX";
        r.audioShiftHz = 1900;  // (FAX_SHIFT)
    } else if (js == "PSK31" || js == "HELL") {
        r.mode = "USB";
        r.decoder = js == "HELL" ? "HELL" : "PSK";
        r.audioShiftHz = 1000;  // (PSK_AUDIO)
    } else if (js == "FT8" || js == "FT4" || js == "WSPR") {
        r.mode = "USB";
        r.decoder = js;
        r.audioShiftHz = 1500;  // (FTX_SHIFT)
    } else if (js == "TIME") {
        r.mode = "USB";
        r.decoder = "TIME";
        r.audioShiftHz = 1000;  // (TIME_SHIFT: its carrier heard as a 1000 Hz tone)
    } else if (js == "SSTV") {
        r.mode = dial < 30e6 ? (sstvLsb(dial) ? "LSB" : "USB") : "NFM";
        r.decoder = "SSTV";
    } else if (js == "ACARS" || js == "VDL2" || js == "DRM" || js == "DAB") {
        r.mode = "AM";  // (the JS's modeObject: those decode from the raw samples, or AM)
        r.decoder = js;
    } else if (js == "APRS") {
        r.mode = "NFM";
        r.decoder = "AX25";
    } else if (js == "AIS") {
        r.mode = "NFM";
        r.decoder = "AIS";
    }
}

AutoResult make(const std::string& js, const std::string& sb, double dial, double tune, const std::string& label,
                const std::string& sure, bool listened) {
    AutoResult r;
    r.ready = true;
    r.found = true;
    r.listened = listened;
    setMode(r, js, dial);
    r.sideband = sb;
    r.dialHz = dial;
    r.tuneHz = tune;
    r.retuneHz = tune - dial;
    r.label = label;
    r.sure = sure;
    r.confidence = !listened ? 1.0 : sure == "likely" ? 0.8 : 0.5;
    r.reason = "AUTO found " + label + (sure.empty() ? std::string() : " (" + sure + ")");
    return r;
}

// ---- signal helpers ------------------------------------------------------------------
typedef std::vector<double> Vec;

// windowed-sinc low-pass, then keep every k-th (no folding): the JS's decim
void decim(const Vec& xr, const Vec& xi, int k, Vec& orr, Vec& oi) {
    const int T = 8 * k + 1;
    std::vector<double> h(T);
    {
        const double fc = 0.42 / k, M = (T - 1) / 2.0;
        double sum = 0;
        for (int j = 0; j < T; ++j) {
            const double n = j - M, w = 0.42 - 0.5 * std::cos(2 * kPi * j / (T - 1)) + 0.08 * std::cos(4 * kPi * j / (T - 1));
            h[j] = (n == 0 ? 2 * fc : std::sin(2 * kPi * fc * n) / (kPi * n)) * w;
            sum += h[j];
        }
        for (int j = 0; j < T; ++j) h[j] /= sum;
    }
    orr.clear();
    oi.clear();
    for (std::size_t i = T - 1; i < xr.size(); i += k) {
        double a = 0, b = 0;
        for (int j = 0; j < T; ++j) {
            a += h[j] * xr[i - j];
            b += h[j] * xi[i - j];
        }
        orr.push_back(a);
        oi.push_back(b);
    }
}

double fold(double x, double fsv) { return std::fmod(std::fmod(x, fsv) + fsv * 1.5, fsv) - fsv / 2; }

// Welch-averaged spectrum accessor (the JS's acc, P, dB, meanDb)
struct Spec {
    Vec acc;
    int N;
    double bin, floor;
    double P(double f) const {
        long i = static_cast<long>(jsRound(f / bin)) % N;
        if (i < 0) i += N;
        return acc[i];
    }
    double dB(double f) const { return 10 * std::log10(P(f) / floor + 1e-12); }
    double meanDb(double a, double b) const {
        double s = 0;
        int n = 0;
        for (double f = a; f <= b; f += bin * 2) {
            s += P(f);
            n++;
        }
        return 10 * std::log10(s / n / floor + 1e-12);
    }
};

// a tone's strength (|sum|^2) in frames of fr samples
void tonePow(const Vec& zr, const Vec& zi, double fs, double f, int fr, Vec& out) {
    out.clear();
    const double wp = -2 * kPi * f / fs;
    for (std::size_t s0 = 0; s0 + fr <= zr.size(); s0 += fr) {
        double qr = 0, qi = 0;
        for (std::size_t m = s0; m < s0 + fr; ++m) {
            const double ph = wp * m, cr = std::cos(ph), ci = std::sin(ph);
            qr += zr[m] * cr - zi[m] * ci;
            qi += zr[m] * ci + zi[m] * cr;
        }
        out.push_back(qr * qr + qi * qi);
    }
}

// (2.5 ms slices: DSC's dot pattern swaps tones every 10 ms)
double toneCorr(const Vec& zr, const Vec& zi, double fs, double fa, double fb) {
    const int fr = std::max(1, static_cast<int>(jsRound(fs / 400)));
    Vec A, B;
    tonePow(zr, zi, fs, fa, fr, A);
    tonePow(zr, zi, fs, fb, fr, B);
    const std::size_t n = std::min(A.size(), B.size());
    if (n < 10) return 0;
    double ma = 0, mb = 0;
    for (std::size_t k = 0; k < n; ++k) {
        ma += A[k];
        mb += B[k];
    }
    ma /= n;
    mb /= n;
    double sab = 0, saa = 0, sbb = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const double x = A[k] - ma, y = B[k] - mb;
        sab += x * y;
        saa += x * x;
        sbb += y * y;
    }
    const double d = std::sqrt(saa * sbb);
    return sab / (d ? d : 1);
}

double envCv(const Vec& zr, const Vec& zi, double fs) {
    const int fr2 = static_cast<int>(jsRound(fs / 100));
    Vec pw;
    for (std::size_t s0 = 0; s0 + fr2 <= zr.size(); s0 += fr2) {
        double t = 0;
        for (std::size_t m = s0; m < s0 + fr2; ++m) t += zr[m] * zr[m] + zi[m] * zi[m];
        pw.push_back(std::sqrt(t));
    }
    double mn = 0;
    for (std::size_t i = 0; i < pw.size(); ++i) mn += pw[i];
    mn /= (pw.empty() ? 1 : pw.size());
    double v = 0;
    for (std::size_t i = 0; i < pw.size(); ++i) v += (pw[i] - mn) * (pw[i] - mn);
    return std::sqrt(v / (pw.empty() ? 1 : pw.size())) / (mn ? mn : 1);
}

// envelope of a tone p in frames of fr samples (|sum|)
void toneEnv(const Vec& zr, const Vec& zi, double fs, double p, int fr, Vec& env) {
    env.clear();
    const double wp = -2 * kPi * p / fs;
    for (std::size_t s = 0; s + fr <= zr.size(); s += fr) {
        double qr = 0, qi = 0;
        for (std::size_t m = s; m < s + fr; ++m) {
            const double ph = wp * m, c = std::cos(ph), d = std::sin(ph);
            qr += zr[m] * c - zi[m] * d;
            qi += zr[m] * d + zi[m] * c;
        }
        env.push_back(std::sqrt(qr * qr + qi * qi));
    }
}

// How long a keyed tone stays on, typically (the median "on" stretch, ms). Hellschreiber paints its letters in dots
// of about 4 ms; Morse elements are 30 ms or more up to 40 words a minute. On above half the usual peak, off below a
// quarter. NaN: too few.
double keyedRunMs(const Vec& zr, const Vec& zi, double fs, double p) {
    const int fr = std::max(1, static_cast<int>(jsRound(fs / 500)));
    Vec env;
    toneEnv(zr, zi, fs, p, fr, env);
    Vec srt(env);
    std::sort(srt.begin(), srt.end());
    const double hi = srt.empty() ? 0 : srt[static_cast<std::size_t>(std::floor(srt.size() * 0.9))];
    const double ms = fr / fs * 1000;
    Vec runs;
    bool on = false;
    int len = 0;
    for (std::size_t i = 0; i < env.size(); ++i) {
        const double v = env[i];
        if (!on && v > hi * 0.5) {
            on = true;
            len = 1;
        } else if (on && v < hi * 0.25) {
            on = false;
            runs.push_back(len * ms);
        } else if (on)
            len++;
    }
    if (runs.size() < 4) return kNaN;
    std::sort(runs.begin(), runs.end());
    return runs[runs.size() >> 1];
}

// PSK keys its phase on a fixed clock: 32 ms bits (PSK31), 16 ms (PSK63) or 8 ms (PSK125). Each phase flip dips the
// amplitude to nothing, so the dips fall on that clock's grid. Returns the share of gaps between dips that are a
// whole number of bits (± an eighth of a bit): PSK at its own speed about 0.9+, anything else far less.
double pskBitTiming(const Vec& zr, const Vec& zi, double fs, double p, double bitMs) {
    const int fr = std::max(1, static_cast<int>(jsRound(fs / 1000)));
    Vec env;
    toneEnv(zr, zi, fs, p, fr, env);
    const double fms = fr / fs * 1000, minGap = bitMs * 0.375 / fms;
    const int sw = std::max(1, static_cast<int>(jsRound(bitMs / 16 / fms)));
    const long E = static_cast<long>(env.size());
    Vec sm(E);
    for (long i = 0; i < E; ++i) {
        double a = 0;
        int n = 0;
        for (long k = std::max(0L, i - sw); k <= std::min(E - 1, i + sw); ++k) {
            a += env[k];
            n++;
        }
        sm[i] = a / n;
    }
    Vec srt(sm);
    std::sort(srt.begin(), srt.end());
    const double hi = srt.empty() ? 0 : srt[static_cast<std::size_t>(std::floor(srt.size() * 0.9))];
    std::vector<long> dips;
    for (long i = 1; i < E - 1; ++i)
        if (sm[i] < hi * 0.35 && sm[i] <= sm[i - 1] && sm[i] < sm[i + 1] && (dips.empty() || i - dips.back() > minGap))
            dips.push_back(i);
    int on = 0, n = 0;
    for (std::size_t k = 1; k < dips.size(); ++k) {
        const double gap = (dips[k] - dips[k - 1]) * fms;
        if (gap > bitMs * 10) continue;
        n++;
        const double r = gap / bitMs;
        if (std::fabs(r - jsRound(r)) * bitMs <= bitMs / 8 && jsRound(r) >= 1) on++;
    }
    return n >= 6 ? static_cast<double>(on) / n : 0;
}

// Which PSK speed, if any: the slowest clock the dips fit. "" = none.
std::string pskSpeed(const Vec& zr, const Vec& zi, double fs, double p, double* timing) {
    static const double ms[3] = {32, 16, 8};
    static const char* const names[3] = {"PSK31", "PSK63", "PSK125"};
    for (int i = 0; i < 3; ++i) {
        const double t = pskBitTiming(zr, zi, fs, p, ms[i]);
        if (t >= 0.8) {
            if (timing) *timing = t;
            return names[i];
        }
    }
    if (timing) *timing = 0;
    return "";
}

// FT8 in AUTO's 2 s listen: separate narrow signals from 150 Hz to 3.2 kHz above an FT8 dial; each signal's width
// measured within 15 dB of its own peak. Returns widths (Hz).
template <class NearDc>
std::vector<double> ft8Runs(const Spec& S, double base, const NearDc& nearDc, double thr) {
    std::vector<double> widths;
    bool has = false;
    double start = 0, last = 0;
    for (double f = base + 150; f <= base + 3200; f += 5) {
        const bool on = !nearDc(f) && S.meanDb(f - 2.5, f + 2.5) >= thr;
        if (on) {
            if (!has) {
                start = f;
                has = true;
            }
            last = f;
        } else if (has && f - last > 15) {
            double pk = -1e9;
            for (double g = start; g <= last; g += 5) pk = std::max(pk, S.meanDb(g - 2.5, g + 2.5));
            double a = kNaN, b = kNaN;
            for (double g = start; g <= last; g += 5)
                if (S.meanDb(g - 2.5, g + 2.5) >= pk - 15) {
                    if (!isNum(a)) a = g;
                    b = g;
                }
            widths.push_back(b - a + 5);
            has = false;
        }
    }
    if (has) {
        double pk = -1e9;
        for (double g = start; g <= last; g += 5) pk = std::max(pk, S.meanDb(g - 2.5, g + 2.5));
        double a = kNaN, b = kNaN;
        for (double g = start; g <= last; g += 5)
            if (S.meanDb(g - 2.5, g + 2.5) >= pk - 15) {
                if (!isNum(a)) a = g;
                b = g;
            }
        widths.push_back(b - a + 5);
    }
    return widths;
}

// SSB fine tuning from the voice itself: voiced speech is a comb of harmonics at whole multiples of the speaker's
// pitch; tuned off by d Hz, the whole comb is shifted by d. Over 2 s only the true offset keeps fitting (the pitch
// keeps changing). Search ±150 Hz around c0. Returns the carrier (Hz from the dial) or NaN.
double ssbRefine(const Vec& zr, const Vec& zi, double fs, double c0, bool usb) {
    const int L = static_cast<int>(jsRound(fs * 0.256));
    const int NF = 1 << static_cast<int>(std::ceil(std::log(L * 4.0) / std::log(2.0)));
    const int hop = L >> 1;
    const double bin = fs / NF;
    const double D0 = -150, DS = 5;
    const int ND = 61;
    Vec score(ND, 0.0), win(L), re(NF), im(NF), db(NF), avg(NF, 0.0), fb(ND), ref;
    for (int i = 0; i < L; ++i) win[i] = 0.5 - 0.5 * std::cos(2 * kPi * i / L);
    int used = 0;
    struct Interp {
        const Vec& db;
        int NF;
        double bin;
        double operator()(double f) const {
            const double x = f / bin, i0 = std::floor(x), fr = x - i0;
            long a0 = static_cast<long>(i0) % NF, b0 = (static_cast<long>(i0) + 1) % NF;
            if (a0 < 0) a0 += NF;
            if (b0 < 0) b0 += NF;
            const double a = db[a0], b = db[b0];
            return a + (b - a) * fr;
        }
    } A = {db, NF, bin};
    for (std::size_t s = 0; s + L <= zr.size(); s += hop) {
        std::fill(re.begin(), re.end(), 0.0);
        std::fill(im.begin(), im.end(), 0.0);
        for (int i = 0; i < L; ++i) {
            re[i] = zr[s + i] * win[i];
            im[i] = zi[s + i] * win[i];
        }
        detail::fftDouble(&re[0], &im[0], NF);
        for (int k = 0; k < NF; ++k) db[k] = 10 * std::log10(re[k] * re[k] + im[k] * im[k] + 1e-30);
        // the voice region's typical level, to measure the comb against
        ref.clear();
        for (double fa = 300; fa <= 2400; fa += 20) ref.push_back(A(c0 + (usb ? fa : -fa)));
        std::sort(ref.begin(), ref.end());
        const double med = ref[ref.size() >> 1];
        double fbMax = -1e9;
        for (int d = 0; d < ND; ++d) {
            const double c = c0 + D0 + d * DS;
            double best = -1e9;
            for (double f0 = 85; f0 <= 320; f0 += 1.5) {
                double h = 0, g = 0;
                int n = 0;
                const double sg = usb ? 1 : -1;
                for (int k = std::max(1, static_cast<int>(std::ceil(250 / f0))); k * f0 <= 2200; ++k) {
                    const double fa = k * f0, fg = fa + f0 / 2;
                    h += std::max(0.0, A(c + sg * fa) - med);
                    g += std::max(0.0, A(c + sg * fg) - med);
                    n++;
                }
                if (n >= 3) {
                    const double v = (h - g) / n;
                    if (v > best) best = v;
                }
            }
            fb[d] = best;
            if (best > fbMax) fbMax = best;
        }
        if (fbMax < 3) continue;  // not voiced (or too weak): no clear comb in this frame
        for (int k = 0; k < NF; ++k) avg[k] += std::pow(10.0, db[k] / 10);
        used++;
        double mean = 0;
        for (int d = 0; d < ND; ++d) mean += fb[d];
        mean /= ND;
        for (int d = 0; d < ND; ++d) score[d] += fb[d] - mean;
    }
    if (used < 3) return kNaN;
    int bd = 0;
    for (int d = 1; d < ND; ++d)
        if (score[d] > score[bd]) bd = d;
    // the best offset must stand out: on noise alone it hardly does (under 0.3), on a voice even at 0 dB it does
    Vec srt(score);
    std::sort(srt.begin(), srt.end());
    const double base = srt[ND >> 1];
    if ((score[bd] - base) / used < 0.5) return kNaN;
    // A comb fits one pitch either side too. SSB transmitters cut the audio below about 300 Hz, so the voice can't
    // start much closer to the carrier than that: among the near-best offsets, drop any that would put the voice
    // closer than 270 Hz to the carrier, if one that doesn't remains.
    struct BandAt {
        const Vec& avg;
        int NF;
        double bin;
        double operator()(double f) const {
            long i = static_cast<long>(jsRound(f / bin)) % NF;
            if (i < 0) i += NF;
            return avg[i];
        }
    } bandAt = {avg, NF, bin};
    double vmax = 0;
    for (double fa = 100; fa <= 2400; fa += 10) vmax = std::max(vmax, bandAt(c0 + (usb ? fa : -fa)));
    std::vector<int> cands, ok;
    for (int d = 1; d < ND - 1; ++d)
        if (score[d] >= score[d - 1] && score[d] >= score[d + 1] && score[d] - base >= (score[bd] - base) * 0.8) cands.push_back(d);
    for (std::size_t i = 0; i < cands.size(); ++i) {
        const double c = c0 + D0 + cands[i] * DS;
        double startFrom = 1500;
        for (double fa = 50; fa <= 1500; fa += 10)
            if (bandAt(c + (usb ? fa : -fa)) >= vmax * 0.1) {
                startFrom = fa;
                break;
            }
        if (startFrom >= 270) ok.push_back(cands[i]);
    }
    if (!ok.empty() && std::find(ok.begin(), ok.end(), bd) == ok.end()) {
        bd = ok[0];
        for (std::size_t i = 0; i < ok.size(); ++i)
            if (score[ok[i]] > score[bd]) bd = ok[i];
    }
    if (bd == 0 || bd == ND - 1) return kNaN;  // at the edge of the search: not trustworthy
    const double a = score[bd - 1], b = score[bd], e = score[bd + 1], den = 2 * (a - 2 * b + e);
    const double offv = (a - e) / (den ? den : 1);
    return c0 + D0 + (bd + std::max(-0.5, std::min(0.5, offv))) * DS;
}

// AUTO's DRM test, on the ±16 kHz view: a flat block 4.5-20 kHz wide with sharp edges and no carrier (the DC spike's
// hole bridged and left out of the levels; flatness from the quartiles, which fading notches hardly move).
bool drmCheck(const Vec& zr, const Vec& zi, double fs, double dcAt, bool hasDc, double* lo, double* hi) {
    const int NW = 4096;
    Vec accW(NW, 0.0), re(NW), im(NW);
    for (std::size_t st = 0; st + NW / 2 <= zr.size(); st += NW / 4) {
        std::fill(re.begin(), re.end(), 0.0);
        std::fill(im.begin(), im.end(), 0.0);
        const std::size_t L2 = std::min<std::size_t>(NW / 2, zr.size() - st);
        for (std::size_t m = 0; m < L2; ++m) {
            const double h = 0.5 - 0.5 * std::cos(2 * kPi * m / L2);
            re[m] = zr[st + m] * h;
            im[m] = zi[st + m] * h;
        }
        detail::fftDouble(&re[0], &im[0], NW);
        for (int j = 0; j < NW; ++j) accW[j] += re[j] * re[j] + im[j] * im[j];
    }
    const double bW = fs / NW;
    struct DbAt {
        const Vec& a;
        int NW;
        double bW;
        double operator()(double f) const {
            long i = static_cast<long>(jsRound(f / bW)) % NW;
            if (i < 0) i += NW;
            return 10 * std::log10(a[i] + 1e-30);
        }
    } dbAt = {accW, NW, bW};
    const bool useDc = hasDc;
    std::vector<std::pair<double, double> > vals;
    for (double f = -15000; f <= 15000; f += 50) vals.push_back(std::make_pair(f, dbAt(f)));
    Vec sorted;
    for (std::size_t i = 0; i < vals.size(); ++i) sorted.push_back(vals[i].second);
    std::sort(sorted.begin(), sorted.end());
    const double fl = sorted[static_cast<std::size_t>(std::floor(sorted.size() * 0.2))];
    Vec on;
    for (std::size_t i = 0; i < vals.size(); ++i)
        if (vals[i].second >= fl + 8 && !(useDc && std::fabs(vals[i].first - dcAt) <= 350)) on.push_back(vals[i].first);
    if (on.empty()) return false;
    std::vector<Vec> cl;
    Vec cur(1, on[0]);
    for (std::size_t i = 1; i < on.size(); ++i) {
        const double gap = on[i] - on[i - 1];
        const bool acrossDc = useDc && on[i - 1] < dcAt && on[i] > dcAt && gap <= 900;  // (the spike's hole: bridge it)
        if (gap <= 300 || acrossDc)
            cur.push_back(on[i]);
        else {
            cl.push_back(cur);
            cur.assign(1, on[i]);
        }
    }
    cl.push_back(cur);
    const Vec* blk = 0;
    for (std::size_t i = 0; i < cl.size(); ++i) {
        const Vec& c = cl[i];
        if (c.front() <= 6000 && c.back() >= -6000 && c.back() - c.front() >= 4500 && c.back() - c.front() <= 21000) {
            blk = &c;
            break;
        }
    }
    if (!blk) return false;
    const double l = blk->front(), h = blk->back();
    Vec inside;
    for (std::size_t i = 0; i < vals.size(); ++i)
        if (vals[i].first >= l + 300 && vals[i].first <= h - 300 && !(useDc && std::fabs(vals[i].first - dcAt) <= 350))
            inside.push_back(vals[i].second);
    if (inside.empty()) return false;
    std::sort(inside.begin(), inside.end());
    struct Q {
        const Vec& v;
        double operator()(double p) const { return v[std::min(v.size() - 1, static_cast<std::size_t>(std::floor(v.size() * p)))]; }
    } q = {inside};
    const double med = q(0.5), spread = q(0.75) - q(0.25), top = inside.back();
    const double edgeLo = dbAt(l - 400), edgeHi = dbAt(h + 400);
    if (spread >= 5) return false;                     // not flat enough
    if (top - med >= 9) return false;                  // a carrier or strong tone inside (AM, not DRM)
    if (med - edgeLo <= 6 || med - edgeHi <= 6) return false;  // edges not sharp
    *lo = l;
    *hi = h;
    return true;
}

struct Peak {
    double f, db;
};

}  // namespace

AutoResult::AutoResult()
    : ready(false), found(false), listened(false), needsConfirm(false), exact(false), dialHz(0), tuneHz(0), retuneHz(0),
      audioShiftHz(0), confidence(0) {}

// ===========================================================================
// by frequency alone (the JS's DAB check, fixedModeBand and onFmBroadcast)
// ===========================================================================
bool AutoDetector::byFrequency(double f, AutoResult& r) {
    // DAB (Band III): anywhere inside a channel's block, its exact centre in DAB mode; the DAB receiver then confirms
    // a multiplex is there, or AUTO goes back to narrowband FM
    if (f >= 174e6 && f <= 240.1e6) {
        const DabCh* best = 0;
        double bd = 0;
        for (std::size_t i = 0; i < COUNT(kDab); ++i) {
            const double d = std::fabs(f - kDab[i].k * 1e3);
            if (d <= 768e3 && (!best || d < bd)) {
                best = &kDab[i];
                bd = d;
            }
        }
        if (best) {
            r = make("DAB", "", f, best->k * 1e3, std::string("DAB channel ") + best->name, "likely", false);
            r.needsConfirm = true;
            r.reason = std::string("AUTO: DAB channel ") + best->name + " (" + fmt("%.3f", best->k / 1e3) +
                       " MHz): listening for a multiplex...";
            return true;
        }
    }
    if (f >= 118e6 && f <= 137e6) {
        // (VDL2 channels carry nothing else: within 5 kHz of one, that's the answer)
        const double v = findWithin(kVdl, COUNT(kVdl), f, 5000);
        if (isNum(v)) {
            r = make("VDL2", "", f, v, "VDL2", "likely", false);
            r.reason = "AUTO: a VDL2 channel (" + fmt("%.3f", v / 1e6) + " MHz): aircraft datalink in short digital bursts";
            return true;
        }
        // (ACARS channels carry nothing but ACARS: within 5 kHz of one, that's the answer, on the channel itself)
        const double ch = findWithin(kAcars, COUNT(kAcars), f, 5000);
        if (isNum(ch)) {
            r = make("ACARS", "", f, ch, "ACARS", "likely", false);
            r.reason = "AUTO: an ACARS channel (" + fmt("%.3f", ch / 1e6) + " MHz), aircraft text messages in short bursts";
            return true;
        }
        r = make("AM", "both", f, f, "airband AM", "", false);
        r.reason = "Airband is always AM, so AUTO has nothing to work out here";
        return true;
    }
    {
        const double a = findWithin(kAis, COUNT(kAis), f, 12500);
        if (isNum(a)) {
            r = make("AIS", "", f, a, "AIS", "likely", false);
            r.reason = "AUTO: an AIS channel: ships' positions, names and destinations in short data bursts";
            return true;
        }
    }
    {
        const double a = findWithin(kAprs, COUNT(kAprs), f, 5000);
        if (isNum(a)) {
            r = make("APRS", "", f, a, "APRS", "likely", false);
            r.reason = "AUTO: an APRS frequency (" + fmt("%.3f", a / 1e6) +
                       " MHz): radio amateurs' positions, weather and messages in short data bursts";
            return true;
        }
    }
    struct Fixed {
        double lo, hi;
        const char* mode;
        const char* why;
    };
    static const Fixed kFixed[] = {
        {108e6, 118e6 - 1e-3, "AM", "108-118 MHz is aircraft navigation (VOR and ILS beacons), always AM, so AUTO has nothing to work out here"},
        {446e6, 446.2e6, "NBFM", "PMR446 is always narrowband FM, so AUTO has nothing to work out here"},
        {27.596e6, 27.996e6, "NBFM", "UK CB (27.60125-27.99125 MHz) is FM only, so AUTO has nothing to work out here"},
        {156e6, 162.05e6, "NBFM", "Marine VHF is always narrowband FM, so AUTO has nothing to work out here"}};
    for (std::size_t i = 0; i < COUNT(kFixed); ++i)
        if (f >= kFixed[i].lo && f <= kFixed[i].hi) {
            const std::string m = kFixed[i].mode;
            r = make(m, m == "AM" ? "both" : "", f, f, m == "AM" ? "AM" : "narrowband FM", "", false);
            r.reason = kFixed[i].why;
            return true;
        }
    // the FM broadcast band: always wideband FM (stereo, RDS), nothing to listen for
    if (f >= 87.5e6 && f <= 108e6) {
        r = make("WBFM", "", f, f, "wideband FM (broadcast)", "likely", false);
        return true;
    }
    return false;
}

// ===========================================================================
// the listen
// ===========================================================================
AutoDetector::AutoDetector()
    : demo_(false), active_(false), retry_(false), audio_(false), dial_(0), centre_(kNaN), rate_(0), want_(0), got_(0),
      skip_(0), view_(0), hpos_(0) {}

void AutoDetector::reset() {
    active_ = false;
    retry_ = false;
    res_ = AutoResult();
    buf_.clear();
}

bool AutoDetector::begin(double dial) {
    reset();
    dial_ = dial;
    AutoResult r;
    if (byFrequency(dial, r)) {
        res_ = r;
        return true;
    }
    active_ = true;
    startListen();
    return false;
}

void AutoDetector::startListen() {
    skip_ = 2;  // (just after a retune: may still be the old signal)
    centre_ = kNaN;
    rate_ = 0;
    got_ = 0;
    want_ = 0;
    buf_.clear();
    audio_ = false;
}

double AutoDetector::progress() const {
    if (res_.ready) return 1;
    if (!active_ || want_ <= 0) return 0;
    return std::min(1.0, got_ / want_);  // (of the current listen: a second listen starts again from 0)
}

bool AutoDetector::feedIq(const Complex* iq, std::size_t n, double rate, double centre) {
    if (!active_ || res_.ready) return res_.ready;
    if (skip_ > 0) {
        --skip_;
        return false;
    }
    if (!isNum(centre_)) {  // the tuning the blocks themselves report
        centre_ = centre;
        rate_ = rate;
        const bool vhf = dial_ >= 30e6;
        view_ = vhf ? 1 : 0;
        // HF: a ±16 kHz view (~32 kHz). VHF/UHF: a ±32 kHz view (~64 kHz) first, for FM's width.
        const int D = std::max(1, static_cast<int>(jsRound(rate / (vhf ? 64000 : 32000))));
        ext_.configure(rate, dial_ - centre, D);
        want_ = jsRound(rate * 2);
        buf_.reserve(static_cast<std::size_t>(want_ / D) + 16);
    }
    if (centre != centre_ || rate != rate_) return false;  // (only blocks with the same tuning)
    ext_.process(iq, n, buf_);
    got_ += n;
    if (got_ >= want_) finishListen();
    return res_.ready;
}

bool AutoDetector::feedAudio(const float* audio, std::size_t n, double rate) {
    if (!active_ || res_.ready) return res_.ready;
    if (skip_ > 0) {
        --skip_;
        return false;
    }
    if (!isNum(centre_)) {
        centre_ = dial_;
        rate_ = rate;
        audio_ = true;
        view_ = 2;
        // a 129-tap Hilbert transformer (Blackman-windowed): the audio made analytic, then down to ~8 kHz
        const int T = 129, M = T / 2;
        hil_.assign(T, 0.0f);
        for (int j = 0; j < T; ++j) {
            const int k = j - M;
            if (k % 2) {
                const double w = 0.42 - 0.5 * std::cos(2 * kPi * j / (T - 1)) + 0.08 * std::cos(4 * kPi * j / (T - 1));
                hil_[j] = static_cast<float>(2.0 / (kPi * k) * w);
            }
        }
        hring_.assign(T, 0.0f);
        hpos_ = 0;
        hout_.reserve(4096);
        const int D = std::max(1, static_cast<int>(jsRound(rate / 8000)));
        ext_.configure(rate, 0, D);
        want_ = jsRound(rate * 2);
        buf_.reserve(static_cast<std::size_t>(want_ / D) + 16);
    }
    const int T = static_cast<int>(hil_.size()), M = T / 2;
    std::size_t i = 0;
    while (i < n) {
        hout_.clear();
        const std::size_t end = std::min(n, i + 4096);
        for (; i < end; ++i) {
            hring_[hpos_] = audio[i];
            double q = 0;
            std::size_t idx = hpos_;
            for (int j = 0; j < T; ++j) {
                q += hil_[j] * hring_[idx];
                idx = idx == 0 ? T - 1 : idx - 1;
            }
            const float re = hring_[(hpos_ + T - M) % T];
            hout_.push_back(Complex(re, static_cast<float>(q)));
            hpos_ = hpos_ + 1 == static_cast<std::size_t>(T) ? 0 : hpos_ + 1;
        }
        ext_.process(&hout_[0], hout_.size(), buf_);
    }
    got_ += n;
    if (got_ >= want_) finishListen();
    return res_.ready;
}

void AutoDetector::finishListen() {
    std::vector<float> zr(buf_.size()), zi(buf_.size());
    for (std::size_t i = 0; i < buf_.size(); ++i) {
        zr[i] = buf_[i].real();
        zi[i] = buf_[i].imag();
    }
    const double fs = ext_.outputRate();
    const double dcRel = audio_ ? kNaN : centre_ - dial_;
    AutoResult r = analyse(zr, zi, fs, view_, dial_, dcRel, rate_, demo_);
    if (r.found) {
        res_ = r;
        active_ = false;
        return;
    }
    if (!retry_) {  // nothing this time: listen once more
        retry_ = true;
        const bool wasAudio = audio_;
        startListen();
        skip_ = 0;
        if (wasAudio) {
            // (keep the Hilbert state and the rate: the audio carries on)
            centre_ = dial_;
            audio_ = true;
            want_ = jsRound(rate_ * 2);
            ext_.reset();
        } else {
            centre_ = kNaN;
        }
        return;
    }
    // nothing clear: fall back to AM (the most forgiving), where you are
    const bool vhf = dial_ >= 30e6, air = dial_ >= 118e6 && dial_ <= 137e6;
    const int hs = hamSideband(dial_);
    const std::string fb = vhf && !air ? "NBFM" : hs ? (hs < 0 ? "LSB" : "USB") : "AM";
    AutoResult f = make(fb, "both", dial_, dial_, fb == "NBFM" ? "narrowband FM" : fb, "", true);
    f.found = false;
    f.confidence = 0;
    f.reason = "No clear signal found: switched to " + f.label + " (try tuning closer)";
    res_ = f;
    active_ = false;
}

// ===========================================================================
// the analysis (the JS's autoDetect after the listen)
// ===========================================================================
AutoResult AutoDetector::analyse(const std::vector<float>& zrIn, const std::vector<float>& ziIn, double fsIn, int view,
                                 double dial, double dcRel, double rawRate, bool demo) {
    AutoResult none;
    none.dialHz = dial;
    none.tuneHz = dial;
    none.ready = false;
    const bool vhf = dial >= 30e6, air = dial >= 118e6 && dial <= 137e6;
    const bool hasDc = isNum(dcRel);
    const int hs = hamSideband(dial);  // (an HF ham band, with its usual sideband)
    Vec zr(zrIn.begin(), zrIn.end()), zi(ziIn.begin(), ziIn.end()), tr, ti;
    double fs = fsIn;
    const std::size_t minLen = static_cast<std::size_t>(fsIn);  // (at least a second)
    if (zr.size() < minLen || zr.size() != zi.size()) {
        none.reason = "AUTO couldn't listen long enough, try again";
        return none;
    }
    // The stick's centre carries a small artificial "DC spike": AUTO ignores ±200 Hz around it (relative to the dial:
    // dcRel), and at its folded positions in the narrower views.
    bool vhfOk = false;
    double vW = 0, vCen = 0, vCv = 1;
    if (view == 1) {
        // ±8 kHz (~16 kHz rate): FM is steady in strength (constant envelope) and 4-16 kHz wide; AM and SSB rise and fall
        Vec wr, wi;
        decim(zr, zi, 4, wr, wi);
        const double f16 = fs / 4;
        const int NW = 4096, LW = 1024;
        Vec accW(NW, 0.0), re(NW), im(NW);
        for (std::size_t st = 0; st + LW <= wr.size(); st += LW / 2) {
            std::fill(re.begin(), re.end(), 0.0);
            std::fill(im.begin(), im.end(), 0.0);
            for (int m = 0; m < LW; ++m) {
                const double h = 0.5 - 0.5 * std::cos(2 * kPi * m / LW);
                re[m] = wr[st + m] * h;
                im[m] = wi[st + m] * h;
            }
            detail::fftDouble(&re[0], &im[0], NW);
            for (int j = 0; j < NW; ++j) accW[j] += re[j] * re[j] + im[j] * im[j];
        }
        Spec SW;
        SW.acc = accW;
        SW.N = NW;
        SW.bin = f16 / NW;
        SW.floor = 1;
        Vec allW;
        for (double f = -7500; f <= 7500; f += SW.bin * 3) allW.push_back(SW.P(f));
        std::sort(allW.begin(), allW.end());
        const double flW = allW[allW.size() >> 1] ? allW[allW.size() >> 1] : 1e-30;
        struct Mdb {
            const Spec& S;
            double fl;
            double operator()(double a, double b) const {
                double t = 0;
                int n = 0;
                for (double f = a; f <= b; f += S.bin) {
                    t += S.P(f);
                    n++;
                }
                return 10 * std::log10(t / n / fl + 1e-12);
            }
        } mdb = {SW, flW};
        const double dcW = hasDc ? fold(dcRel, f16) : 0, dcW2 = hasDc ? fold(dcRel, fs) : 0;
        Vec occ;
        for (double f = -7800; f <= 7800; f += 50)
            if ((!hasDc || (std::fabs(f - dcRel) >= 300 && std::fabs(f - dcW) >= 300 && std::fabs(f - dcW2) >= 300)) &&
                mdb(f - 25, f + 25) >= 8)
                occ.push_back(f);
        if (!occ.empty()) {
            // (an FM tone is a comb of lines, ~1 kHz apart: join them)
            std::vector<Vec> cl;
            Vec cur(1, occ[0]);
            for (std::size_t i = 1; i < occ.size(); ++i) {
                if (occ[i] - occ[i - 1] < 1500)
                    cur.push_back(occ[i]);
                else {
                    cl.push_back(cur);
                    cur.assign(1, occ[i]);
                }
            }
            cl.push_back(cur);
            std::size_t bi = 0;
            double bm = 1e18;
            for (std::size_t i = 0; i < cl.size(); ++i) {
                double m = 1e18;
                for (std::size_t j = 0; j < cl[i].size(); ++j) m = std::min(m, std::fabs(cl[i][j]));
                if (m < bm) {
                    bm = m;
                    bi = i;
                }
            }
            vW = cl[bi].back() - cl[bi].front();
            vCen = (cl[bi].front() + cl[bi].back()) / 2;
        }
        const int fr = static_cast<int>(jsRound(f16 / 100));
        Vec pw;
        for (std::size_t s0 = 0; s0 + fr <= wr.size(); s0 += fr) {
            double t = 0;
            for (std::size_t m = s0; m < s0 + fr; ++m) t += wr[m] * wr[m] + wi[m] * wi[m];
            pw.push_back(std::sqrt(t));
        }
        double mean = 0;
        for (std::size_t i = 0; i < pw.size(); ++i) mean += pw[i];
        mean /= pw.empty() ? 1 : pw.size();
        double v = 0;
        for (std::size_t i = 0; i < pw.size(); ++i) v += (pw[i] - mean) * (pw[i] - mean);
        vCv = std::sqrt(v / (pw.empty() ? 1 : pw.size())) / (mean ? mean : 1);
        vhfOk = true;
        decim(zr, zi, 8, tr, ti);  // the usual ±4 kHz view for the checks below
        zr.swap(tr);
        zi.swap(ti);
        fs = fs / 8;
    }
    // HF: on the ±16 kHz view, look for DRM (a flat block 4.5-20 kHz wide, sharp edges, no carrier); then ±4 kHz
    bool drm = false;
    double drmLo = 0, drmHi = 0;
    if (view == 0 && !vhf && zr.size() > 4096) {
        drm = drmCheck(zr, zi, fs, hasDc ? fold(dcRel, fs) : 0, hasDc, &drmLo, &drmHi);
        decim(zr, zi, 4, tr, ti);
        zr.swap(tr);
        zi.swap(ti);
        fs = fs / 4;
    } else if (view == 0) {
        decim(zr, zi, 4, tr, ti);
        zr.swap(tr);
        zi.swap(ti);
        fs = fs / 4;
    }
    const double dA = hasDc ? fold(dcRel, fs) : 0, dB2 = hasDc && rawRate > 0 ? fold(dcRel, fsIn) : 0;
    struct NearDc {
        bool has;
        double a, b, c;
        bool operator()(double f) const { return has && (std::fabs(f - a) < 200 || std::fabs(f - b) < 200 || std::fabs(f - c) < 200); }
    } nearDc = {hasDc, hasDc ? dcRel : 0, dA, dB2};
    // averaged spectrum (Welch): ~1 Hz bins
    Spec S;
    {
        const int L = 2048, N = 8192;
        S.acc.assign(N, 0.0);
        S.N = N;
        Vec re(N), im(N), w(L);
        for (int m = 0; m < L; ++m) w[m] = 0.5 - 0.5 * std::cos(2 * kPi * m / L);
        int segs = 0;
        for (std::size_t st = 0; st + L <= zr.size(); st += L / 2) {
            std::fill(re.begin(), re.end(), 0.0);
            std::fill(im.begin(), im.end(), 0.0);
            for (int m = 0; m < L; ++m) {
                re[m] = zr[st + m] * w[m];
                im[m] = zi[st + m] * w[m];
            }
            detail::fftDouble(&re[0], &im[0], N);
            for (int j = 0; j < N; ++j) S.acc[j] += re[j] * re[j] + im[j] * im[j];
            segs++;
        }
        if (!segs) {
            none.reason = "AUTO couldn't listen long enough, try again";
            return none;
        }
        S.bin = fs / N;
        S.floor = 1;
        Vec all;
        for (double f = -3800; f <= 3800; f += S.bin * 4) all.push_back(S.P(f));
        std::sort(all.begin(), all.end());
        S.floor = all[all.size() >> 1] ? all[all.size() >> 1] : 1e-30;
    }
    const double bin = S.bin;
    // peaks near the dial (±1600 Hz), 15 dB or more over the noise, merged within 25 Hz
    std::vector<Peak> peaks;
    for (double f = -1600; f <= 1600; f += bin) {
        if (nearDc(f)) continue;
        const double v = S.dB(f);
        if (v >= 15 && v >= S.dB(f - bin) && v >= S.dB(f + bin)) {
            Peak* near = 0;
            for (std::size_t i = 0; i < peaks.size(); ++i)
                if (std::fabs(peaks[i].f - f) < 25) {
                    near = &peaks[i];
                    break;
                }
            if (near) {
                if (v > near->db) {
                    near->f = f;
                    near->db = v;
                }
            } else {
                Peak p = {f, v};
                peaks.push_back(p);
            }
        }
    }
    std::stable_sort(peaks.begin(), peaks.end(), [](const Peak& a, const Peak& b) { return b.db < a.db; });
    AutoResult r;
    bool have = false;
#define AT(x) (dial + (x))
#define RESULT(js, sb, tune, label, sure) (r = make(js, sb, dial, tune, label, sure, true), have = true)
    // VHF/UHF first: narrowband FM (steady strength, 4-16 kHz wide)
    bool r0 = false;
    AutoResult nbfm;
    if (vhfOk && vW >= 4000 && vW <= 16000 && vCv < 0.25) {
        nbfm = make("NBFM", "", dial, jsRound(AT(vCen) / 500) * 500, "narrowband FM", "likely", true);
        r0 = true;
    }
    if (drm)
        RESULT("DRM", "", jsRound(AT((drmLo + drmHi) / 2) / 1000) * 1000,
               "DRM digital radio, " + fmt("%.0f", (drmHi - drmLo) / 1000) + " kHz wide: reading its name", "likely");
    if (r0) {
        r = nbfm;
        have = true;
    }
    // (fskMaybe: a tone pair at a standard shift whose alternation wasn't clear)
    bool fskMaybe = false;
    double fmA = 0, fmB = 0;
    int fmWant = 0;
    bool fmNav = false;
    // the occupied cluster nearest the dial (used by the fax and SSB checks)
    bool clu = false;
    double cluLo = 0, cluHi = 0;
    {
        Vec occ;
        for (double f = -3500; f <= 3500; f += 10)
            if (!nearDc(f) && S.meanDb(f - 10, f + 10) >= 8) occ.push_back(f);
        if (!occ.empty()) {
            std::vector<Vec> cl;
            Vec cur(1, occ[0]);
            for (std::size_t i = 1; i < occ.size(); ++i) {
                if (occ[i] - occ[i - 1] < 150)
                    cur.push_back(occ[i]);
                else {
                    cl.push_back(cur);
                    cur.assign(1, occ[i]);
                }
            }
            cl.push_back(cur);
            std::size_t bi = 0;
            double bm = 1e18;
            for (std::size_t i = 0; i < cl.size(); ++i) {
                double m = 1e18;
                for (std::size_t j = 0; j < cl[i].size(); ++j) m = std::min(m, std::fabs(cl[i][j]));
                if (m < bm) {
                    bm = m;
                    bi = i;
                }
            }
            clu = true;
            cluLo = cl[bi].front();
            cluHi = cl[bi].back();
        }
    }
    const double cluW = cluHi - cluLo;
    std::vector<double> faxFreqs;
    for (std::size_t i = 0; i < COUNT(kFaxK); ++i) faxFreqs.push_back(kFaxK[i] * 1e3);
    if (demo) faxFreqs.push_back(7425e3);
    // 0. fax on a known fax frequency (a chart is mostly white: one steady tone, so check this before carriers)
    if (!have && !vhf && clu && cluW >= 150 && cluW <= 2600) {
        const double cf = AT((cluLo + cluHi) / 2);
        if (nearAny(&faxFreqs[0], faxFreqs.size(), cf, 1500)) {
            double b = faxFreqs[0];
            for (std::size_t i = 1; i < faxFreqs.size(); ++i)
                if (std::fabs(cf - faxFreqs[i]) < std::fabs(cf - b)) b = faxFreqs[i];
            RESULT("FAX", "", b, "weather fax", "likely");
        }
    }
    // An SSB voice as one block, wherever the dial sits in it: the level smoothed over 300 Hz, gaps up to 600 Hz
    // bridged, the block 0.9-3.6 kHz wide with the most energy within 2.5 kHz of the dial. Speech is strongest in its
    // low audio, next to the carrier: energy bunched at the low end means USB, at the high end LSB (unclear: the
    // band's convention). The carrier is ~250 Hz beyond where the voice's energy starts.
    struct SsbBlock {
        const Spec& S;
        const NearDc& nearDc;
        const std::vector<Peak>& peaks;
        int hs;
        double dial;
        bool operator()(double thr, const char* tag, AutoResult& out) const {
            Vec occ2;
            for (double f = -3700; f <= 3700; f += 25)
                if (!nearDc(f) && S.meanDb(f - 150, f + 150) >= thr) occ2.push_back(f);
            if (occ2.empty()) return false;
            std::vector<Vec> cls;
            Vec cur(1, occ2[0]);
            for (std::size_t i = 1; i < occ2.size(); ++i) {
                if (occ2[i] - occ2[i - 1] <= 600)
                    cur.push_back(occ2[i]);
                else {
                    cls.push_back(cur);
                    cur.assign(1, occ2[i]);
                }
            }
            cls.push_back(cur);
            bool any = false;
            double lo = 0, hi = 0, W = 0, bestE = 0;
            for (std::size_t i = 0; i < cls.size(); ++i) {
                const double l = cls[i].front() + 125, h = cls[i].back() - 125, w = h - l;  // (undo the smoothing's widening)
                const double nearest = std::min(std::min(std::fabs(l), std::fabs(h)), l < 0 && h > 0 ? 0.0 : 1e9);
                if (w < 900 || w > 3600 || nearest > 2500) continue;
                double e = 0;
                for (double f = l; f <= h; f += 10) e += S.P(f);
                if (e > bestE) {
                    bestE = e;
                    lo = l;
                    hi = h;
                    W = w;
                    any = true;
                }
            }
            if (!any) return false;
            double sw = 0, sf = 0;
            for (double f = lo; f <= hi; f += 10) {
                const double v = S.P(f);
                sw += v;
                sf += v * f;
            }
            const double pos = sw > 0 ? (sf / sw - lo) / W : 0.5;  // 0 = low edge, 1 = high edge
            // an even block with a strong carrier in its middle is AM (its two sidebands), not an SSB voice
            const double mid = (lo + hi) / 2;
            bool car = false;
            for (std::size_t i = 0; i < peaks.size(); ++i)
                if (peaks[i].db >= 25 && std::fabs(peaks[i].f - mid) <= std::max(150.0, W * 0.1)) car = true;
            if (car && pos > 0.4 && pos < 0.6) return false;
            const bool usb = pos < 0.42 ? true : pos > 0.58 ? false : hs ? hs > 0 : dial >= 10e6;
            double acc2 = 0, edge = usb ? lo : hi;
            if (usb) {
                for (double f = lo; f <= hi; f += 5) {
                    acc2 += S.P(f) * 0.5;
                    if (acc2 >= sw * 0.02) {
                        edge = f;
                        break;
                    }
                }
            } else {
                for (double f = hi; f >= lo; f -= 5) {
                    acc2 += S.P(f) * 0.5;
                    if (acc2 >= sw * 0.02) {
                        edge = f;
                        break;
                    }
                }
            }
            out = usb ? make("USB", "", dial, dial + edge - 250, std::string("SSB voice (USB") + tag + ")", "possibly", true)
                      : make("LSB", "", dial, dial + edge + 250, std::string("SSB voice (LSB") + tag + ")", "possibly", true);
            return true;
        }
    } ssbFromBlock = {S, nearDc, peaks, hs, dial};
    // 0b. FT8, on an FT8 frequency (within 3 kHz of one): two or more separate signals about 50 Hz wide each in the
    // 3 kHz above the dial, and no wide voice among them. (FT8 is silent for the last 2.4 s of each 15 s: a listen
    // that falls there finds nothing, and AUTO listens again.)
    if (!have && !vhf) {
        double fd = kNaN;
        for (std::size_t i = 0; i < COUNT(kFt8); ++i)
            if (dial >= kFt8[i] - 100 && dial <= kFt8[i] + 3000) {
                fd = kFt8[i];
                break;
            }
        if (isNum(fd)) {
            const std::vector<double> w = ft8Runs(S, fd - dial, nearDc, 3);
            int sigs = 0, wide = 0;
            for (std::size_t i = 0; i < w.size(); ++i) {
                if (w[i] >= 8 && w[i] <= 70) sigs++;
                if (w[i] > 110) wide++;
            }
            if (!wide && sigs >= 2) RESULT("FT8", "", fd, "FT8 (" + fmt("%.0f", sigs) + " signals seen)", "likely");
        }
    }
    // time stations by their frequencies (within 500 Hz): CHU, WWV / WWVH, RWM
    if (!have && !vhf)
        for (std::size_t i = 0; i < COUNT(kTime); ++i)
            if (std::fabs(dial - kTime[i].k * 1e3) <= 500) {
                RESULT("TIME", "", kTime[i].k * 1e3, std::string("time signal (") + kTime[i].name + ")", "likely");
                break;
            }
    // WSPR on its frequencies: by the frequency alone (its signals are 6 Hz wide and 2 minutes long)
    if (!have && (!vhf || dial > 140e6)) {
        const double fw = findWithin(kWspr, COUNT(kWspr), dial, 300);
        if (isNum(fw)) RESULT("WSPR", "", fw, "WSPR (weak beacons)", "likely");
    }
    // FT4, likewise on its own frequencies: its signals are about 90 Hz wide (4 tones 20.8 Hz apart)
    if (!have && !vhf) {
        double fd4 = kNaN;
        for (std::size_t i = 0; i < COUNT(kFt4); ++i)
            if (dial >= kFt4[i] - 100 && dial <= kFt4[i] + 3000) {
                fd4 = kFt4[i];
                break;
            }
        if (isNum(fd4)) {
            const std::vector<double> w = ft8Runs(S, fd4 - dial, nearDc, 3);
            int sigs = 0, wide = 0;
            for (std::size_t i = 0; i < w.size(); ++i) {
                if (w[i] >= 20 && w[i] <= 130) sigs++;
                if (w[i] > 200) wide++;
            }
            if (!wide && sigs >= 2) RESULT("FT4", "", fd4, "FT4 (" + fmt("%.0f", sigs) + " signals seen)", "likely");
        }
    }
    // Two tones that are an AM station's sidebands, not RTTY: each mirrored the other side of a stronger, steady
    // carrier (RTTY has no carrier, so no mirror images: this can't cost a real RTTY signal)
    struct AmSidebands {
        const Spec& S;
        const std::vector<Peak>& peaks;
        bool operator()(double a, double b) const {
            for (std::size_t i = 0; i < peaks.size() && i < 5; ++i) {
                const Peak& c = peaks[i];
                if (std::fabs(c.f - a) < 15 || std::fabs(c.f - b) < 15 || std::fabs(c.f - (a + b) / 2) > 4000) continue;
                const double la = S.meanDb(a - 6, a + 6), lb = S.meanDb(b - 6, b + 6);
                if (S.meanDb(c.f - 6, c.f + 6) < std::max(la, lb) + 3) continue;  // (a carrier outshines its sidebands)
                if (S.meanDb(2 * c.f - a - 6, 2 * c.f - a + 6) > la - 10 && S.meanDb(2 * c.f - b - 6, 2 * c.f - b + 6) > lb - 10)
                    return true;
            }
            return false;
        }
    } amSidebands = {S, peaks};
    static const double kShifts[4] = {170, 425, 450, 850};
    struct ShiftOf {
        int operator()(double sh) const {
            for (int i = 0; i < 4; ++i) {
                const double s = kShifts[i];
                if (std::fabs(sh - s) < (s == 170 ? 25 : std::max(10.0, s * 0.025))) return static_cast<int>(s);
            }
            return 0;
        }
    } shiftOf;
    // 1. two tones a standard shift apart, similar strength, little between them: RTTY / NAVTEX
    for (std::size_t i = 0; i < std::min<std::size_t>(5, peaks.size()) && !have && !vhf; ++i)
        for (std::size_t j = i + 1; j < std::min<std::size_t>(6, peaks.size()) && !have; ++j) {
            const double a = std::min(peaks[i].f, peaks[j].f), b = std::max(peaks[i].f, peaks[j].f), sh = b - a;
            const double centre = AT((a + b) / 2);
            const bool onNav = nearAny(kNavtex, COUNT(kNavtex), centre, 2000);
            // (100-baud NAVTEX spreads each tone, so its measured shift can read up to ~200 Hz; slow keying spreads
            //  each tone too: 170 Hz gets ±25 Hz, the wider shifts ±2.5%)
            const int want = onNav && sh >= 140 && sh <= 230 ? 170 : shiftOf(sh);
            if (!want || std::fabs(peaks[i].db - peaks[j].db) > 8) continue;
            const double mid = S.meanDb(a + sh * 0.3, b - sh * 0.3);
            if (mid > std::min(peaks[i].db, peaks[j].db) - 10) continue;
            if (amSidebands(a, b)) continue;
            // RTTY's two tones ALTERNATE; a Morse carrier and its keying sideband rise and fall TOGETHER
            // (real FSK measures about -0.9; with 2.5 ms slices chance can reach -0.3, so "likely" needs -0.5,
            //  "possibly" -0.35)
            const double corr = toneCorr(zr, zi, fs, a, b);
            if (corr > -0.5) {
                if (!fskMaybe && corr < -0.35 && std::min(peaks[i].db, peaks[j].db) >= 20) {
                    fskMaybe = true;
                    fmA = a;
                    fmB = b;
                    fmWant = want;
                    fmNav = onNav;
                }
                continue;
            }
            const bool nav = want == 170 && onNav, onDsc = want == 170 && nearAny(kDsc, COUNT(kDsc), centre, 2000);
            if (onDsc)
                RESULT("DSC", "", centre, "DSC (ships' selective calling)", "likely");
            else if (nav)
                RESULT("NAVTEX", "", centre, std::string("NAVTEX ") + kDot + " 170 Hz shift", "likely");
            else
                RESULT("RTTY", "", centre, std::string("RTTY ") + kDot + " " + fmt("%.0f", want) + " Hz shift", "likely");
        }
    // 1c. PSK idle (continuous phase flips): two equal peaks the baud rate apart; the bit timing then says which
    if (!have && !vhf && peaks.size() >= 2) {
        const Peak& p0 = peaks[0];
        const Peak* q = 0;
        static const double bds[3] = {31.25, 62.5, 125};
        for (std::size_t i = 1; i < peaks.size() && !q; ++i) {
            if (std::fabs(peaks[i].db - p0.db) >= 6) continue;
            for (int k = 0; k < 3; ++k)
                if (std::fabs(std::fabs(peaks[i].f - p0.f) - bds[k]) < bds[k] * 0.16) {
                    q = &peaks[i];
                    break;
                }
        }
        if (q) {
            const double sp2 = std::fabs(q->f - p0.f), mid = (p0.f + q->f) / 2, gap = std::max(90.0, sp2 * 2);
            const bool alone = S.meanDb(mid - 450 - sp2, mid - gap) < 8 && S.meanDb(mid + gap, mid + 450 + sp2) < 8;
            if (alone) {
                const std::string sp = pskSpeed(zr, zi, fs, mid, 0);
                if (!sp.empty()) RESULT("PSK31", "", AT(mid), sp + " (ham text)", "likely");
            }
        }
    }
    // 2. a strong narrow carrier: AM (steady, sidebands both sides), CW (keyed) or a plain carrier (AM).
    // A voice harmonic in SSB can look like a carrier, but it comes and goes: a real carrier is steady.
    if (!have && !peaks.empty() && peaks[0].db >= 20) {
        const double p = peaks[0].f, lo = S.meanDb(p - 3000, p - 150), hi = S.meanDb(p + 150, p + 3000);
        // its strength over time (10 ms frames): Morse switches on and off
        const int fr = static_cast<int>(jsRound(fs / 100));
        Vec pw;
        tonePow(zr, zi, fs, p, fr, pw);
        double mx = 0;
        for (std::size_t k = 0; k < pw.size(); ++k) mx = std::max(mx, pw[k]);
        int nlow = 0;
        for (std::size_t k = 0; k < pw.size(); ++k)
            if (pw[k] < mx / 10) nlow++;
        const double offFrac = static_cast<double>(nlow) / pw.size();
        // how often it JUMPS: Morse switches in a few ms, so from one 10 ms frame to the next its level leaps by 10 dB
        // or more; fading changes it smoothly, over tenths of a second
        int jumps = 0;
        for (std::size_t k = 1; k < pw.size(); ++k) {
            const double a = pw[k - 1], b = pw[k];
            if (std::max(a, b) < mx / 100) continue;
            if (std::fabs(10 * std::log10((a + 1e-30) / (b + 1e-30))) >= 10) jumps++;
        }
        const double jumpRate = jumps / (pw.size() * fr / fs);
        // phase coherence while it's on: a keyed carrier (CW, Hell) keeps one phase; PSK31 keeps flipping it
        double coh = 1, revFrac = 0;
        {
            const int fq = std::max(1, static_cast<int>(jsRound(fs / 200)));
            const double wp2 = -2 * kPi * p / fs;
            std::vector<std::pair<double, double> > sl;
            for (std::size_t s0 = 0; s0 + fq <= zr.size(); s0 += fq) {
                double qr = 0, qi = 0;
                for (std::size_t m = s0; m < s0 + fq; ++m) {
                    const double ph = wp2 * m, cr = std::cos(ph), ci = std::sin(ph);
                    qr += zr[m] * cr - zi[m] * ci;
                    qi += zr[m] * ci + zi[m] * cr;
                }
                sl.push_back(std::make_pair(qr, qi));
            }
            Vec mag(sl.size());
            double mxm = 0;
            for (std::size_t k = 0; k < sl.size(); ++k) {
                mag[k] = std::sqrt(sl[k].first * sl[k].first + sl[k].second * sl[k].second);
                mxm = std::max(mxm, mag[k]);
            }
            double sr = 0, sa = 0;
            int neg = 0, cnt = 0;
            for (std::size_t k = 1; k < sl.size(); ++k) {
                if (mag[k] < mxm * 0.3 || mag[k - 1] < mxm * 0.3) continue;
                sr += sl[k].first * sl[k - 1].first + sl[k].second * sl[k - 1].second;
                sa += mag[k] * mag[k - 1];
            }
            // PSK31's flips: 15 ms apart (spanning the dip at each flip), the phase is often reversed; a carrier never
            for (std::size_t k = 3; k < sl.size(); ++k) {
                if (mag[k] < mxm * 0.3 || mag[k - 3] < mxm * 0.3) continue;
                cnt++;
                if (sl[k].first * sl[k - 3].first + sl[k].second * sl[k - 3].second < 0) neg++;
            }
            coh = sa > 0 ? sr / sa : 1;
            revFrac = cnt ? static_cast<double>(neg) / cnt : 0;
        }
        double mean = 0;
        for (std::size_t k = 0; k < pw.size(); ++k) mean += std::sqrt(pw[k]);
        mean /= pw.size();
        double var = 0;
        for (std::size_t k = 0; k < pw.size(); ++k) var += (std::sqrt(pw[k]) - mean) * (std::sqrt(pw[k]) - mean);
        const double cv = std::sqrt(var / pw.size()) / (mean ? mean : 1);
        const double broad = std::max(S.meanDb(p - 1500, p - 250), S.meanDb(p + 250, p + 1500));  // energy spread around it: a voice
        // how fast is it switched? 2.5 ms frames: Hellschreiber's dots are ~4 ms, Morse elements tens of ms
        double flips = 0;
        {
            const int f2 = std::max(1, static_cast<int>(jsRound(fs / 400)));
            Vec pw2;
            tonePow(zr, zi, fs, p, f2, pw2);
            double mx2 = 0;
            for (std::size_t k = 0; k < pw2.size(); ++k) mx2 = std::max(mx2, pw2[k]);
            int fl = 0, prev = -1;
            for (std::size_t k = 0; k < pw2.size(); ++k) {
                const int onq = pw2[k] > mx2 / 6 ? 1 : 0;
                if (prev >= 0 && onq != prev) fl++;
                prev = onq;
            }
            flips = fl / (pw2.size() * f2 / fs);
        }
        const bool onPsk = nearAny(kPskF, COUNT(kPskF), AT(p), 3500);
        const bool narrow = S.meanDb(p - 60, p + 60) - std::max(S.meanDb(p + 150, p + 450), S.meanDb(p - 450, p - 150)) > 10;
        // PSK31 also needs its 32 ms bit clock: an off-tune SSB voice or a heterodyne wobbles too, but its dips come at
        // random times, never on a 32 ms grid
        std::string pskSp;
        if (!vhf && revFrac > 0.08 && narrow && broad < 20) pskSp = pskSpeed(zr, zi, fs, p, 0);
        // Hellschreiber also needs its short dots: its "on" stretches are mostly under 15 ms; Morse's 30 ms or more
        const double runMs = !vhf && flips > 25 && offFrac > 0.15 ? keyedRunMs(zr, zi, fs, p) : kNaN;
        bool inHam = false;
        for (std::size_t k = 0; k < COUNT(kHamBands); ++k)
            if (AT(p) >= kHamBands[k][0] - 5e3 && AT(p) <= kHamBands[k][1] + 5e3) inHam = true;
        // one RTTY tone also switches on and off: a partner tone at an exact RTTY shift that doesn't rise and fall with
        // it makes this RTTY, not Morse
        const Peak* partner = 0;
        for (std::size_t k = 1; k < peaks.size(); ++k)
            if (peaks[k].db >= 15 && shiftOf(std::fabs(peaks[k].f - p))) {
                partner = &peaks[k];
                break;
            }
        if (!pskSp.empty())
            RESULT("PSK31", "", AT(p), pskSp + " (ham text)", onPsk ? "likely" : "possibly");  // (the PSK mode decodes all three speeds)
        // (and only in an amateur band, where Hellschreiber is sent: a broadcast station deep in a fade flickers fast)
        else if (isNum(runMs) && runMs < 20 && coh >= 0.45 && revFrac < 0.05 && broad < 25 && inHam)
            RESULT("HELL", "", AT(p), "Hellschreiber", "possibly");
        else if (partner && !vhf && offFrac > 0.15 && !amSidebands(p, partner->f) && toneCorr(zr, zi, fs, p, partner->f) < -0.3) {
            const double mid = (p + partner->f) / 2;
            const int want = shiftOf(std::fabs(partner->f - p));
            const bool nav = want == 170 && nearAny(kNavtex, COUNT(kNavtex), AT(mid), 2000);
            // (Addition to the JS: on a DSC frequency a 170 Hz pair is DSC here too, as in rule 1; DSC's 100 baud
            //  spreads its tones so that rule 1's "little between them" test often misses it)
            if (want == 170 && nearAny(kDsc, COUNT(kDsc), AT(mid), 2000))
                RESULT("DSC", "", AT(mid), "DSC (ships' selective calling)", "possibly");
            else if (nav)
                RESULT("NAVTEX", "", AT(mid), std::string("NAVTEX ") + kDot + " 170 Hz shift", "possibly");
            else
                RESULT("RTTY", "", AT(mid), std::string("RTTY ") + kDot + " " + fmt("%.0f", want) + " Hz shift", "possibly");
        }
        // (a strong, crisply keyed Morse signal has keying sidebands well above the noise, but far below itself;
        //  Morse needs some sudden jumps: fading AM shows none at all, slow Morse at 8 wpm can show under 3 a second)
        else if (offFrac > 0.15 && cv > 0.5 && coh >= 0.45 && revFrac < 0.05 && jumpRate >= 1.2 &&
                 (broad < 6 || broad < peaks[0].db - 25))
            RESULT("CW", "", AT(p), "CW (Morse)", offFrac > 0.25 ? "likely" : "possibly");
        // a FADING AM station: its carrier rises and falls, but smoothly, never keyed; one frequency and phase, and
        // programme sidebands on both sides (or a strong, clean carrier, as long as nothing is lopsided like SSB)
        else if (cv >= 0.45 && jumpRate < 1.5 && coh >= 0.9 && revFrac < 0.05 && narrow && std::fabs(lo - hi) < 6 &&
                 ((lo > 3 && hi > 3) || peaks[0].db >= 30))
            RESULT("AM", "both", AT(p), "AM (fading)", "likely");
        else if (cv < 0.45) {
            const bool oneSided = std::max(lo, hi) > 6 && std::fabs(lo - hi) >= 6;  // lopsided: SSB voice, not AM
            if (!oneSided) {
                if (lo > 3 && hi > 3)
                    RESULT("AM", "both", AT(p), "AM", std::fabs(lo - hi) < 3 ? "likely" : "possibly");
                else
                    RESULT("AM", "both", AT(p), "a steady carrier (AM)", "possibly");
            }
        }  // (else: not steady, so not a carrier: see 3)
    }
    // 3. wide signals without a carrier: fax (~800 Hz flat band) or SSB voice (1.5-3.5 kHz)
    if (!have) {
        if (clu) {
            const double lo = cluLo, hi = cluHi, W = cluW, cf = AT((lo + hi) / 2);
            const bool faxNear = nearAny(&faxFreqs[0], faxFreqs.size(), cf, 2500);
            // (away from the fax frequencies, a fax signal must also be steady in strength: a piece of a speaking
            //  voice, between its formant gaps, can be just as wide)
            if (W >= 450 && W <= 1150 && (faxNear || (W >= 650 && envCv(zr, zi, fs) < 0.2)))
                RESULT("FAX", "", cf, "weather fax", faxNear ? "likely" : "possibly");
            else if (!vhf && W >= 700 && W <= 3200 && envCv(zr, zi, fs) < 0.15 && nearAny(kSstv, COUNT(kSstv), cf, 5000)) {
                // tuned from the centre of its energy (~1750 Hz audio): below the signal in USB, above it in LSB
                double sw2 = 0, sf2 = 0;
                for (double f = lo; f <= hi; f += 10) {
                    const double v = S.P(f);
                    sw2 += v;
                    sf2 += v * f;
                }
                const double cen = AT(sw2 ? sf2 / sw2 : (lo + hi) / 2);
                const bool lsb = sstvLsb(cen);
                RESULT("SSTV", "", jsRound((lsb ? cen + 1750 : cen - 1750) / 10) * 10,
                       std::string("SSTV (slow-scan TV") + (lsb ? ", LSB" : "") + ")", "possibly");
            }
        }
        // SSB voice: found as a whole block, however it's tuned
        if (!have && !vhf) have = ssbFromBlock(4, "", r);
    }
    // 3b. a weak SSB voice, too faint to stand out as a block: over the 2 s listen its speech still lifts the average
    // on one side of the carrier and not the other (noise averages out flat)
    if (!have && !vhf) have = ssbFromBlock(2, ", weak", r);
    if (!have && !vhf) {
        const double below = S.meanDb(-2700, -300), above = S.meanDb(300, 2700), side = std::max(below, above);
        if (side >= 2.5 && std::fabs(below - above) >= 1.5) {
            const bool usb = std::fabs(below - above) >= 2 ? above > below : hs ? hs > 0 : dial >= 10e6;
            // the voice's inner edge: the first 200 Hz slice (from 0 Hz outwards) that's nearly as far up as the
            // side's average (85 %: the carrier then comes out within about 50 Hz)
            double edge = usb ? 300 : -300;
            for (double f = 0; f <= 1500; f += 25) {
                const double d = usb ? S.meanDb(f, f + 200) : S.meanDb(-f - 200, -f);
                if (d >= std::max(1.5, side * 0.85)) {
                    edge = usb ? f : -f;
                    break;
                }
            }
            double tune = usb ? edge - 250 : edge + 250;
            const Peak* line = 0;  // a carrier (residual) right there: use it
            for (std::size_t i = 0; i < peaks.size(); ++i)
                if (peaks[i].db >= 15 && std::fabs(peaks[i].f - tune) <= 150) {
                    line = &peaks[i];
                    break;
                }
            if (line) tune = line->f;
            RESULT(usb ? "USB" : "LSB", "", AT(tune), std::string("SSB voice (") + (usb ? "USB" : "LSB") + ", weak)", "possibly");
            if (line) r.exact = true;  // (the station's own carrier: nothing to refine)
        }
    }
    // SSB: from the voice edge the carrier is only good to some tens of Hz; the harmonics of the speaker's voice pin
    // it down to a few Hz (see ssbRefine)
    if (have && (r.jsMode == "USB" || r.jsMode == "LSB") && !r.exact && !vhf) {
        const double c = ssbRefine(zr, zi, fs, r.tuneHz - dial, r.jsMode == "USB");
        if (isNum(c)) {
            r.tuneHz = AT(c);
            r.retuneHz = c;
        }
    }
    // a tone pair at a standard shift whose alternation wasn't clear-cut: still most likely RTTY
    if (!have && fskMaybe && !vhf) {
        if (fmWant == 170 && nearAny(kDsc, COUNT(kDsc), AT((fmA + fmB) / 2), 2000))  // (Addition to the JS, as above)
            RESULT("DSC", "", AT((fmA + fmB) / 2), "DSC (ships' selective calling)", "possibly");
        else if (fmWant == 170 && fmNav)
            RESULT("NAVTEX", "", AT((fmA + fmB) / 2), std::string("NAVTEX ") + kDot + " 170 Hz shift", "possibly");
        else
            RESULT("RTTY", "", AT((fmA + fmB) / 2), std::string("RTTY ") + kDot + " " + fmt("%.0f", fmWant) + " Hz shift", "possibly");
    }
    if (!have && air && !peaks.empty() && peaks[0].db >= 15) RESULT("AM", "both", AT(peaks[0].f), "airband AM", "likely");
#undef AT
#undef RESULT
    if (!have) {
        none.reason = "No clear signal found near here";
        return none;
    }
    r.retuneHz = r.tuneHz - dial;
    return r;
}

}  // namespace dsp
