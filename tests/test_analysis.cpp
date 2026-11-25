// ============================================================================
//  offgrid-dsp - reusable C++11 radio signal processing (part of OFFgrid-SDR
//  TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_analysis.cpp
//  Layer:  Tests (library liboffgrid-dsp)
//
//  What this file does
//  -------------------
//  Self-checking test of the signal-analysis engines ported from
//  OFFgrid-SDR (diddy-boy): AUTO (AutoMode.h), the Signal Monitor and the
//  deep analysis (SignalMonitor.h), "Likely bands open" (Propagation.h),
//  peak hold / averaging / new-signal alerts (SpectrumTools.h).  Prints
//  PASS/FAIL per check, returns nonzero if anything failed:
//    - AUTO on signals made by the repo's encoders and dsp::DemoSource's
//      stations, as u8 IQ like a stick (256 kS/s): CW, RTTY 45.45/170,
//      NAVTEX on 518 kHz, DSC on 2187.5 kHz, PSK31, Hellschreiber, weather
//      fax, SSTV, FT8 (two stations), AM, USB and LSB voice, narrowband FM,
//      DRM (a flat 10 kHz block), time station and WSPR (by frequency), the
//      frequency-only bands (FM broadcast, DAB, airband, AIS, APRS), noise
//      (the fall-back), and the USB-audio path (feedAudio): the right mode
//      and decoder, and the retune within tolerance;
//    - the Signal Monitor: levels, noise, S/N, occupancy, transmission
//      count / length / interval ("every 10 s", regular), CSV; drift and
//      Follow on a drifting carrier;
//    - SignalAnalyzer: RTTY 45.45/170, PSK31, CW WPM, AM, FM, SSB;
//    - the propagation guide: day vs night, summer vs winter, sane MUF;
//    - SpectrumExtras: an alert on a new carrier, none on a known one;
//    - CPU: the monitor with 8 signals and AUTO's listen + analysis.
//
//  How it fits in
//  --------------
//  Build from the repository root (scratch output anywhere):
//    c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors -Isrc src/dsp/*.cpp \
//        src/decoders/FskEncoders.cpp src/decoders/Codes.cpp \
//        src/decoders/PskEncoders.cpp src/decoders/Psk.cpp src/decoders/Hell.cpp \
//        src/decoders/PictureEncoders.cpp src/decoders/Fax.cpp src/decoders/Sstv.cpp \
//        src/decoders/Ftx.cpp tests/test_analysis.cpp -o test_analysis
//
//  Contributing
//  ------------
//  Keep it deterministic (DemoSource and the encoders are; noise has fixed
//  seeds) and under ~30 s.
// ============================================================================
#include "dsp/AutoMode.h"
#include "dsp/Propagation.h"
#include "dsp/Receiver.h"
#include "dsp/SignalMonitor.h"
#include "dsp/SpectrumTools.h"

#include "decoders/FskEncoders.h"
#include "decoders/Ftx.h"
#include "decoders/PictureEncoders.h"
#include "decoders/PskEncoders.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

int g_fail = 0, g_pass = 0;

void check(bool ok, const std::string& what) {
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
    if (ok)
        ++g_pass;
    else
        ++g_fail;
}

std::string f1(double x) {
    char b[64];
    std::snprintf(b, sizeof b, "%.1f", x);
    return b;
}

const double kRate = 256000;
const double kPi = 3.14159265358979323846;

double now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Rng0 {
    unsigned s;
    double u() {
        s = s * 1664525u + 1013904223u;
        return ((s >> 8) + 0.5) / 16777216.0;
    }
    double g() { return std::sqrt(-2 * std::log(u())) * std::cos(2 * kPi * u()); }
};

// a station's audio, played as USB with its suppressed carrier at hz
struct AudioSt {
    double hz;
    std::vector<float> audio;
    double rate;
    double levelDb;
    dsp::Mode mode;
};

// AUTO on DemoSource IQ; feeds 0.0625 s blocks until ready (at most 12 s)
dsp::AutoResult runAuto(double centre, double dial, const std::vector<AudioSt>& extra, double* ms = 0) {
    dsp::DemoSource ds(kRate, centre);
    for (std::size_t i = 0; i < extra.size(); ++i)
        ds.addAudioStation(extra[i].hz, extra[i].audio, extra[i].rate, extra[i].levelDb, extra[i].mode, "test");
    dsp::AutoDetector ad;
    if (ad.begin(dial)) return ad.result();
    std::vector<std::uint8_t> bytes;
    std::vector<dsp::Complex> iq;
    const std::size_t blk = 16384;
    double used = 0;
    for (int k = 0; k < 12 * kRate / blk && !ad.ready(); ++k) {
        ds.generate(blk, bytes);
        dsp::convertU8(&bytes[0], bytes.size(), iq);
        const double t0 = now();
        ad.feedIq(&iq[0], iq.size(), kRate, centre);
        used += now() - t0;
    }
    if (ms) *ms = used * 1000;
    return ad.result();
}

std::string describe(const dsp::AutoResult& r) {
    char b[256];
    std::snprintf(b, sizeof b, "%s/%s dec=%s tune=%.1f (retune %+.1f) [%s, %s]", r.jsMode.c_str(), r.mode.c_str(),
                  r.decoder.c_str(), r.tuneHz, r.retuneHz, r.label.c_str(), r.sure.c_str());
    return b;
}

void expectAuto(const std::string& what, const dsp::AutoResult& r, const std::string& mode, const std::string& decoder,
                double tune, double tol) {
    const bool ok = r.ready && r.found && r.mode == mode && r.decoder == decoder && std::fabs(r.tuneHz - tune) <= tol;
    char b[96];
    std::snprintf(b, sizeof b, " (want %s/%s at %.1f +-%.0f Hz)", mode.c_str(), decoder.c_str(), tune, tol);
    check(ok, "AUTO " + what + ": " + describe(r) + b);
}

std::vector<float> repeatTo(const std::vector<float>& a, double seconds, double rate) {
    std::vector<float> o;
    while (o.size() < seconds * rate) o.insert(o.end(), a.begin(), a.end());
    return o;
}

// A voice-like SSB signal, analytic (complex) at fs: syllables with a smooth envelope, a pitch that glides
// (110-190 Hz), harmonics weighted by three formants, only from 300 Hz to 2.9 kHz above the carrier (as a
// transmitter's audio filter passes); conj() of it is LSB.
std::vector<dsp::Complex> voiceIq(double fs, double seconds, unsigned seed, bool usb) {
    unsigned s = seed * 2654435761u + 1;
    struct R {
        unsigned& s;
        double u() {
            s = s * 1664525u + 1013904223u;
            return ((s >> 8) + 0.5) / 16777216.0;
        }
    } rng = {s};
    const std::size_t n = static_cast<std::size_t>(seconds * fs);
    std::vector<dsp::Complex> z(n);
    double ph[40] = {0};
    double f0 = 120 + 40 * rng.u(), sylT = 0, sylLen = 0.2, F1 = 700, F2 = 1200, F3 = 2500, gap = 0;
    double phase0 = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const double t = i / fs;
        if (t >= sylT + sylLen + gap) {  // a new syllable
            sylT = t;
            sylLen = 0.15 + 0.2 * rng.u();
            gap = rng.u() < 0.25 ? 0.15 + 0.3 * rng.u() : 0.02;
            static const double kV[5][3] = {{730, 1090, 2440}, {530, 1840, 2480}, {270, 2290, 3010}, {570, 840, 2410}, {300, 870, 2240}};
            const int v = static_cast<int>(rng.u() * 5) % 5;
            F1 = kV[v][0], F2 = kV[v][1], F3 = kV[v][2];
        }
        const double x = (t - sylT) / sylLen, env = x < 1 ? std::sin(kPi * x) : 0;
        // the pitch glides (intonation)
        f0 = 150 + 35 * std::sin(2 * kPi * 0.6 * t + seed) + 10 * std::sin(2 * kPi * 2.3 * t);
        phase0 += 2 * kPi * f0 / fs;
        double re = 0, im = 0;
        for (int k = 1; k < 40; ++k) {
            const double f = k * f0;
            if (f > 2900) break;
            if (f < 300) continue;
            const double d1 = (f - F1) / 110, d2 = (f - F2) / 110, d3 = (f - F3) / 110;
            const double a = (1.0 / (1 + d1 * d1) + 0.6 / (1 + d2 * d2) + 0.3 / (1 + d3 * d3) + 0.02) / std::sqrt(static_cast<double>(k));
            ph[k] = k * phase0;
            re += a * std::cos(ph[k]);
            im += a * std::sin(ph[k]);
        }
        z[i] = dsp::Complex(static_cast<float>(env * re), static_cast<float>(usb ? env * im : -env * im));
    }
    return z;
}

// AUTO on that voice, its carrier at carrierHz, IQ at 32 kS/s centred 12 kHz above the dial
dsp::AutoResult runVoice(double carrierHz, double dial, bool usb, unsigned seed) {
    const double fs = 32000, centre = dial + 12000;
    std::vector<dsp::Complex> v = voiceIq(fs, 5, seed, usb);
    double p = 0;
    for (std::size_t i = 0; i < v.size(); ++i) p += std::norm(v[i]);
    const double sc = std::sqrt(1e-4 / (p / v.size()));  // the voice at -40 dBFS on average
    Rng0 rng = {seed + 77};
    const double w = 2 * kPi * (carrierHz - centre) / fs;
    for (std::size_t i = 0; i < v.size(); ++i) {
        const double c = std::cos(w * i), s = std::sin(w * i);
        const double x = v[i].real() * sc, y = v[i].imag() * sc;
        v[i] = dsp::Complex(static_cast<float>(x * c - y * s + 0.0004 * rng.g()), static_cast<float>(x * s + y * c + 0.0004 * rng.g()));
    }
    dsp::AutoDetector ad;
    ad.begin(dial);
    for (std::size_t o = 0; o + 1024 <= v.size() && !ad.ready(); o += 1024) ad.feedIq(&v[o], 1024, fs, centre);
    return ad.result();
}

// smooth random shading (value noise: random levels on a 24-pixel grid, interpolated), photo-like
unsigned char valueNoise(int x, int y, int c) {
    struct H {
        static double at(int i, int j, int c) {
            unsigned h = (i * 73856093u) ^ (j * 19349663u) ^ (c * 83492791u);
            h = (h ^ (h >> 13)) * 1274126177u;
            return ((h >> 8) & 0xffff) / 65535.0;
        }
    };
    const double fx = x / 24.0, fy = y / 24.0;
    const int i = static_cast<int>(fx), j = static_cast<int>(fy);
    double u = fx - i, v = fy - j;
    u = u * u * (3 - 2 * u);
    v = v * v * (3 - 2 * v);
    const double a = H::at(i, j, c) * (1 - u) + H::at(i + 1, j, c) * u, b = H::at(i, j + 1, c) * (1 - u) + H::at(i + 1, j + 1, c) * u;
    return static_cast<unsigned char>(255 * (a * (1 - v) + b * v));
}

// ---------------------------------------------------------------- AUTO
void testAuto() {
    const double A = 48000;
    // CW 20 wpm, 700 Hz tone: the carrier at 7.030 MHz; tuned 150 Hz off
    {
        std::vector<float> a = dec::enc::cwAudio("CQ CQ CQ DE G4OGR G4OGR PSE K TEST TEST", 20, 700, A, 0.5, 0.1, 0.3);
        std::vector<AudioSt> ex(1);
        ex[0].hz = 7.030e6 - 700, ex[0].audio = a, ex[0].rate = A, ex[0].levelDb = -40, ex[0].mode = dsp::Mode::USB;
        expectAuto("CW 20 wpm", runAuto(7.0e6, 7.030e6 + 150, ex), "CW", "CW", 7.030e6, 10);
    }
    // RTTY 45.45 Bd / 170 Hz centred on 14.090 MHz, tuned 300 Hz off
    {
        std::vector<float> a = dec::enc::rttyAudio("RYRYRY CQ CQ DE G4OGR G4OGR THE QUICK BROWN FOX 0123456789 ", 45.45, 170, 1500, A);
        std::vector<AudioSt> ex(1);
        ex[0].hz = 14.090e6 - 1500, ex[0].audio = a, ex[0].rate = A, ex[0].levelDb = -40, ex[0].mode = dsp::Mode::USB;
        expectAuto("RTTY 45.45/170", runAuto(14.0e6, 14.090e6 + 300, ex), "USB", "RTTY", 14.090e6, 15);
    }
    // NAVTEX on 518 kHz (tuned 200 Hz low)
    {
        std::vector<float> a = dec::enc::navtexAudio("ZCZC GA01\nTEST MESSAGE FOR AUTO\nNNNN\n", A, 1500, 0.5, 24);
        std::vector<AudioSt> ex(1);
        ex[0].hz = 518e3 - 1500, ex[0].audio = a, ex[0].rate = A, ex[0].levelDb = -40, ex[0].mode = dsp::Mode::USB;
        expectAuto("NAVTEX 518 kHz", runAuto(600e3, 518e3 - 200, ex), "USB", "NAVTEX", 518e3, 20);
    }
    // DSC on 2187.5 kHz
    {
        std::vector<float> a = dec::enc::dscAudio(dec::enc::dscBits(dec::enc::dscAllShipsFields(108, "235012345")), A, 1500);
        std::vector<AudioSt> ex(1);
        ex[0].hz = 2187.5e3 - 1500, ex[0].audio = repeatTo(a, 3, A), ex[0].rate = A, ex[0].levelDb = -40, ex[0].mode = dsp::Mode::USB;
        expectAuto("DSC 2187.5 kHz", runAuto(2.1e6, 2187.5e3 + 250, ex), "USB", "DSC", 2187.5e3, 30);
    }
    // PSK31 at 14.0705 MHz (the tone), tuned 150 Hz off
    {
        std::vector<float> a;
        dec::synthPsk("CQ CQ DE G4OGR G4OGR PSE K  the quick brown fox jumps over the lazy dog 0123456789", 31.25, A, a, 1000, 0.5);
        std::vector<AudioSt> ex(1);
        ex[0].hz = 14.0695e6, ex[0].audio = a, ex[0].rate = A, ex[0].levelDb = -40, ex[0].mode = dsp::Mode::USB;
        expectAuto("PSK31", runAuto(14.0e6, 14.0705e6 + 150, ex), "USB", "PSK", 14.0705e6, 5);
    }
    // Hellschreiber on 7.080 MHz
    {
        std::vector<float> a;
        dec::synthHell("CQ CQ DE G4OGR G4OGR HELL TEST 73 ", A, a, 1000, 0.5);
        std::vector<AudioSt> ex(1);
        ex[0].hz = 7.079e6, ex[0].audio = a, ex[0].rate = A, ex[0].levelDb = -40, ex[0].mode = dsp::Mode::USB;
        expectAuto("Hellschreiber", runAuto(7.013e6, 7.080e6 + 100, ex), "USB", "HELL", 7.080e6, 10);
    }
    // weather fax on DWD's 7880 kHz (the picture, past the start tone and phasing)
    {
        std::vector<unsigned char> grey;
        dec::testChartGrey(576, 200, grey);
        std::vector<float> a;
        dec::FaxTransmission tx;
        dec::synthFax(grey, 576, 200, A, a, tx);
        std::vector<float> pic(a.begin() + static_cast<long>(16 * A), a.begin() + static_cast<long>(40 * A));
        std::vector<AudioSt> ex(1);
        ex[0].hz = 7880e3 - 1900, ex[0].audio = pic, ex[0].rate = A, ex[0].levelDb = -40, ex[0].mode = dsp::Mode::USB;
        expectAuto("weather fax 7880 kHz", runAuto(7.9e6, 7880e3 - 500, ex), "USB", "FAX", 7880e3, 1);
    }
    // SSTV (Scottie S1) on 14.230 MHz, past its VIS header; a photo-like picture (smooth random shading: it spreads its tones;
    // a picture of a few flat colours is two or three tones that take turns, and reads as RTTY, as in the JS)
    {
        std::vector<unsigned char> rgb(320 * 256 * 3);
        for (int y = 0; y < 256; ++y)
            for (int x = 0; x < 320; ++x)
                for (int c = 0; c < 3; ++c)
                    rgb[(y * 320 + x) * 3 + c] = valueNoise(x, y, c);
        std::vector<float> a;
        dec::synthSstv(60, rgb, A, a);
        std::vector<float> pic(a.begin() + static_cast<long>(5 * A), a.begin() + static_cast<long>(40 * A));
        std::vector<AudioSt> ex(1);
        // (AUTO's SSTV rule wants one block 0.7-3.2 kHz wide and wider than fax's 1.15 kHz: the 1200 Hz sync must
        //  join the picture's 1500-2300 Hz, which takes a fairly strong signal (-30 dBFS here; at -40 the sync
        //  stands apart and the picture alone reads as fax, as in the JS). Tuned from the energy's centre, assumed
        //  at 1750 Hz: within ~200 Hz.)
        ex[0].hz = 14230e3, ex[0].audio = pic, ex[0].rate = A, ex[0].levelDb = -30, ex[0].mode = dsp::Mode::USB;
        expectAuto("SSTV 14.230 MHz", runAuto(14.3e6, 14230e3 + 1000, ex), "USB", "SSTV", 14230e3, 200);
    }
    // FT8: two stations on the 20 m FT8 dial
    {
        std::vector<float> a, b;
        dec::ftx::synthesizeFt8("CQ G4OGR IO91", 1000, A, 0.3, a);
        dec::ftx::synthesizeFt8("CQ K1OGS FN42", 1600, A, 0.3, b);
        for (std::size_t i = 0; i < a.size() && i < b.size(); ++i) a[i] += b[i];
        std::vector<AudioSt> ex(1);
        ex[0].hz = 14.074e6, ex[0].audio = a, ex[0].rate = A, ex[0].levelDb = -40, ex[0].mode = dsp::Mode::USB;
        expectAuto("FT8 (2 stations)", runAuto(14.0e6, 14.074e6, ex), "USB", "FT8", 14.074e6, 1);
    }
    // DemoSource's own stations: AM music (6.15 MHz), USB voice (6.45), LSB voice (7.05), NFM tone (145.35)
    {
        std::vector<AudioSt> none;
        expectAuto("AM broadcast", runAuto(6.1e6, 6.15e6 + 200, none), "AM", "", 6.15e6, 5);
        expectAuto("USB voice", runVoice(14.2e6, 14.2e6 + 500, true, 1), "USB", "", 14.2e6, 30);
        expectAuto("USB voice, tuned 1 kHz low", runVoice(14.25e6, 14.25e6 - 1000, true, 2), "USB", "", 14.25e6, 30);
        expectAuto("LSB voice", runVoice(7.15e6, 7.15e6 - 500, false, 3), "LSB", "", 7.15e6, 30);
        expectAuto("narrowband FM (VHF)", runAuto(145.31e6, 145.35e6 + 300, none), "NFM", "", 145.35e6, 500);
        // by frequency, after the listen: a time station (WWV 10 MHz) and WSPR (20 m)
        expectAuto("time station 10 MHz", runAuto(10.1e6, 10.0e6 + 100, none), "USB", "TIME", 10.0e6, 1);
        expectAuto("WSPR 14.0956 MHz", runAuto(14.1e6, 14.0956e6, none), "USB", "WSPR", 14.0956e6, 1);
        // noise only: nothing twice, then the fall-back (AM on 31 m broadcast)
        dsp::AutoResult n = runAuto(9.4e6, 9.5e6, none);
        check(n.ready && !n.found && n.mode == "AM", "AUTO noise only: not found, fall-back " + n.mode + " (" + n.reason + ")");
    }
    // by frequency alone (no listening)
    {
        dsp::AutoDetector ad;
        struct Case {
            double f;
            const char* mode;
            const char* dec;
            double tune;
        } cases[] = {{98.3e6, "WFM", "", 98.3e6},        {224.0e6, "AM", "DAB", 223.936e6}, {124.0e6, "AM", "", 124.0e6},
                     {161.98e6, "NFM", "AIS", 161.975e6}, {144.801e6, "NFM", "AX25", 144.8e6}, {131.553e6, "AM", "ACARS", 131.55e6}};
        for (std::size_t i = 0; i < sizeof cases / sizeof cases[0]; ++i) {
            const bool now = ad.begin(cases[i].f);
            const dsp::AutoResult& r = ad.result();
            check(now && r.ready && !r.listened && r.mode == cases[i].mode && r.decoder == cases[i].dec &&
                      std::fabs(r.tuneHz - cases[i].tune) < 1,
                  "AUTO by frequency " + f1(cases[i].f / 1e6) + " MHz: " + describe(r));
        }
    }
    // DRM: a flat 10 kHz block (band-limited noise) centred 1 kHz above the dial, IQ at 32 kS/s
    {
        const double fs = 32000, dial = 9.5e6, centre = dial - 7000;
        const int N = 1 << 17;  // 4 s
        std::vector<double> re(N, 0.0), im(N, 0.0);
        unsigned s = 12345;
        struct G {
            unsigned& s;
            double u() {
                s = s * 1664525u + 1013904223u;
                return (s >> 8) / 16777216.0;
            }
            double g() { return std::sqrt(-2 * std::log(u() + 1e-12)) * std::cos(2 * kPi * u()); }
        } rng = {s};
        for (int k = 0; k < N; ++k) {
            const double f = (k < N / 2 ? k : k - N) * fs / N;  // relative to the IQ centre
            const double rel = f - (dial - centre) - 1000;
            if (std::fabs(rel) <= 5000) {
                re[k] = rng.g();
                im[k] = rng.g();
            }
        }
        for (int k = 0; k < N; ++k) im[k] = -im[k];
        dsp::detail::fftDouble(&re[0], &im[0], N);
        double p = 0;
        for (int i = 0; i < N; ++i) p += re[i] * re[i] + im[i] * im[i];
        const double sc = std::sqrt(0.01 / (p / N));  // -20 dBFS
        std::vector<dsp::Complex> iq(N);
        for (int i = 0; i < N; ++i) iq[i] = dsp::Complex(static_cast<float>(re[i] * sc + 0.0003 * rng.g()), static_cast<float>(-im[i] * sc + 0.0003 * rng.g()));
        dsp::AutoDetector ad;
        ad.begin(dial);
        for (int o = 0; o + 2048 <= N && !ad.ready(); o += 2048) ad.feedIq(&iq[o], 2048, fs, centre);
        expectAuto("DRM (flat 10 kHz block)", ad.result(), "AM", "DRM", 9.501e6, 1000);
    }
    // the USB-audio path (Addition to the JS): RTTY 170 Hz and CW from audio alone
    {
        std::vector<float> a = dec::enc::rttyAudio("RYRYRY CQ CQ DE G4OGR G4OGR THE QUICK BROWN FOX 0123456789 ", 45.45, 170, 1500, A);
        dec::enc::addWhiteNoise(a, 0.02, 7);
        dsp::AutoDetector ad;
        ad.begin(14.090e6 - 1400);  // the dial: the USB carrier, 100 Hz higher than "right"
        for (std::size_t o = 0; o + 1024 <= a.size() && !ad.ready(); o += 1024) ad.feedAudio(&a[o], 1024, A);
        expectAuto("RTTY from USB audio", ad.result(), "USB", "RTTY", 14.090e6 + 100, 15);
        std::vector<float> c = dec::enc::cwAudio("CQ CQ CQ DE G4OGR G4OGR PSE K TEST TEST", 20, 700, A, 0.5, 0.1, 0.3);
        dec::enc::addWhiteNoise(c, 0.02, 9);
        ad.begin(7.030e6);
        for (std::size_t o = 0; o + 1024 <= c.size() && !ad.ready(); o += 1024) ad.feedAudio(&c[o], 1024, A);
        expectAuto("CW from USB audio", ad.result(), "CW", "CW", 7.030e6 + 700, 10);
    }
}

// ---------------------------------------------------------------- Signal Monitor
// complex white noise of the given rms per component + a carrier
struct Rng {
    unsigned s;
    double u() {
        s = s * 1664525u + 1013904223u;
        return ((s >> 8) + 0.5) / 16777216.0;
    }
    double g() { return std::sqrt(-2 * std::log(u())) * std::cos(2 * kPi * u()); }
};

void testMonitor() {
    const double fs = 256000, centre = 7.0e6;
    dsp::SignalMonitor mon;
    // signal 1: a carrier at +20 kHz keyed 3 s on, 7 s off; signal 2: a steady carrier at -30 kHz; 3: empty band
    const int s1 = mon.add(centre + 20e3, 1000, "AM", "keyed, \"every 10 s\"");
    const int s2 = mon.add(centre - 30e3, 1000, "AM", "steady");
    const int s3 = mon.add(centre + 50e3, 1000, "AM", "empty");
    Rng rng = {99};
    const double amp = 0.01, nrms = 0.003;  // carrier -40 dBFS; noise -47.4 dBFS over 256 kHz
    std::vector<dsp::Complex> blk(2560);
    double ph1 = 0, ph2 = 0;
    std::size_t n = 0;
    const double T = 60.5;
    dsp::MonitorStats mid = dsp::MonitorStats();
    for (; n < T * fs; n += blk.size()) {
        if (n == static_cast<std::size_t>(52.5 * fs) / blk.size() * blk.size()) mid = mon.stats(s1);
        for (std::size_t i = 0; i < blk.size(); ++i) {
            const double t = (n + i) / fs;
            const bool on = std::fmod(t - 1.0, 10.0) < 3.0 && t >= 1.0;
            double x = nrms * rng.g(), y = nrms * rng.g();
            ph1 += 2 * kPi * 20e3 / fs;
            ph2 += 2 * kPi * -30e3 / fs;
            if (on) {
                x += amp * std::cos(ph1);
                y += amp * std::sin(ph1);
            }
            x += amp * std::cos(ph2);
            y += amp * std::sin(ph2);
            blk[i] = dsp::Complex(static_cast<float>(x), static_cast<float>(y));
        }
        mon.feedIq(&blk[0], blk.size(), fs, centre);
    }
    const dsp::MonitorStats a = mon.stats(s1), b = mon.stats(s2), c = mon.stats(s3);
    const double noiseIn1k = 10 * std::log10(2 * nrms * nrms * 1000 / fs);
    check(std::fabs(b.levelAvg + 40) < 1.5 && std::fabs(b.level + 40) < 1.5,
          "Monitor steady carrier level " + f1(b.level) + " dBFS, average " + f1(b.levelAvg) + " (want -40)");
    check(std::fabs(b.noise - noiseIn1k) < 3, "Monitor noise in 1 kHz " + f1(b.noise) + " dBFS (want " + f1(noiseIn1k) + ")");
    check(b.snr > 15 && b.on && b.nTx == 1 && b.duty > 0.95, "Monitor steady carrier: on, S/N " + f1(b.snr) + " dB, 1 tx, occupancy " + f1(b.duty * 100) + "%");
    check(a.nTx == 6, "Monitor keyed carrier: " + f1(a.nTx) + " transmissions (want 6)");
    check(std::fabs(a.duty - 0.3) < 0.04, "Monitor keyed carrier: occupancy " + f1(a.duty * 100) + "% (want 30)");
    check(std::fabs(a.txAvg - 3) < 0.25 && std::fabs(a.longest - 3) < 0.3, "Monitor keyed carrier: average " + f1(a.txAvg) + " s, longest " + f1(a.longest) + " s (want 3)");
    check(std::fabs(a.interval - 10) < 0.2 && a.regular, "Monitor keyed carrier: every " + f1(a.interval) + " s, regular " + (a.regular ? "yes" : "no"));
    check(std::fabs(a.active - 18) < 1.5 && std::fabs(a.silent - 42.5) < 2, "Monitor keyed carrier: active " + f1(a.active) + " s, silent " + f1(a.silent) + " s");
    check(std::fabs(a.levelPeak + 40) < 1.5, "Monitor keyed carrier: peak level " + f1(a.levelPeak) + " dBFS");
    check(c.nTx == 0 && !c.on && c.duty == 0, "Monitor empty band: no transmissions");
    check(mid.on && std::fabs(mid.fc - (centre + 20e3)) < 5 && std::fabs(mid.fpk - (centre + 20e3)) < 5 && mid.obw < 400,
          "Monitor keyed carrier (during a transmission): centre " + f1(mid.fc - centre) + " Hz, peak " + f1(mid.fpk - centre) +
              " Hz from the stick's centre, width " + f1(mid.obw) + " Hz (resolution " + f1(mid.obwRes) + ")");
    const std::string csv = mon.csv(1798200000.0);  // 2026-12-25T...
    int lines = 0;
    for (std::size_t i = 0; i < csv.size(); ++i)
        if (csv[i] == '\n') lines++;
    check(lines == 1 + 6 + 1 && csv.find("signal,name,frequency_hz,mode,start_utc,end_utc,duration_s,peak_snr_db\r\n") == 0 &&
              csv.find("\"keyed, \"\"every 10 s\"\"\"") != std::string::npos && csv.find("2026-12-25T") != std::string::npos,
          "Monitor CSV: " + f1(lines) + " lines (header, 6 + 1 under way)");
    std::printf("      CSV row 2: %s\n", csv.substr(csv.find("\r\n") + 2, csv.find("\r\n", csv.find("\r\n") + 2) - csv.find("\r\n") - 2).c_str());

    // drift: a carrier at +10 kHz drifting 2 Hz/s for 150 s; band 400 Hz wide with Follow, another without
    dsp::SignalMonitor m2;
    const int fol = m2.add(centre + 10e3, 400, "CW", "follow", true);
    const int stay = m2.add(centre + 10e3, 400, "CW", "no follow", false);
    double ph = 0;
    for (n = 0; n < 150 * fs; n += blk.size()) {
        for (std::size_t i = 0; i < blk.size(); ++i) {
            const double t = (n + i) / fs, f = 10e3 + 2 * t;
            ph += 2 * kPi * f / fs;
            blk[i] = dsp::Complex(static_cast<float>(amp * std::cos(ph) + nrms * rng.g()), static_cast<float>(amp * std::sin(ph) + nrms * rng.g()));
        }
        m2.feedIq(&blk[0], blk.size(), fs, centre);
    }
    const dsp::MonitorStats d = m2.stats(fol), e = m2.stats(stay);
    check(std::fabs(m2.centreHz(fol) - (centre + 10e3 + 300)) < 10,
          "Follow: the band moved to " + f1(m2.centreHz(fol) - centre - 10e3) + " Hz (the carrier +300 Hz)");
    check(std::fabs(d.driftNow - 2) < 0.3, "Drift now " + f1(d.driftNow) + " Hz/s (want 2)");
    check(std::fabs(d.driftPerMin - 120) < 10, "Drift over 10 min " + f1(d.driftPerMin) + " Hz/min (want 120)");
    check(std::fabs(d.shift - 300) < 15, "Total shift " + f1(d.shift) + " Hz (want ~300)");
    check(m2.centreHz(stay) == centre + 10e3 && e.snr < d.snr - 6, "Without Follow the band stays and the carrier slides out of it (S/N " + f1(e.snr) + " vs " + f1(d.snr) + " dB)");
}

// ---------------------------------------------------------------- SignalAnalyzer
dsp::AnalysisResult analyseDemo(double centre, double bandF, double bw, const std::vector<AudioSt>& extra) {
    dsp::DemoSource ds(kRate, centre);
    for (std::size_t i = 0; i < extra.size(); ++i)
        ds.addAudioStation(extra[i].hz, extra[i].audio, extra[i].rate, extra[i].levelDb, extra[i].mode, "test");
    dsp::SignalAnalyzer an;
    an.start(bandF, bw, kRate);
    std::vector<std::uint8_t> bytes;
    std::vector<dsp::Complex> iq;
    while (!an.ready()) {
        ds.generate(16384, bytes);
        dsp::convertU8(&bytes[0], bytes.size(), iq);
        an.feedIq(&iq[0], iq.size(), centre);
    }
    return an.result();
}

void testAnalyzer() {
    const double A = 48000;
    std::vector<AudioSt> ex(1);
    ex[0].rate = A, ex[0].levelDb = -40, ex[0].mode = dsp::Mode::USB;
    {
        ex[0].hz = 7.040e6 - 1500;
        ex[0].audio = dec::enc::rttyAudio("RYRYRY CQ CQ DE G4OGR G4OGR THE QUICK BROWN FOX 0123456789 ", 45.45, 170, 1500, A);
        dsp::AnalysisResult r = analyseDemo(7.0e6, 7.040e6, 500, ex);
        check(r.kind == "FSK" && r.name == "RTTY (amateur, 45.45 baud)" && std::fabs(r.shift - 170) <= 10 && r.sure == "likely",
              "Analyse RTTY: " + r.name + " / " + r.text);
    }
    {
        ex[0].hz = 7.040e6 - 1000;
        ex[0].audio.clear();
        dec::synthPsk("CQ CQ DE G4OGR G4OGR PSE K  the quick brown fox jumps over the lazy dog 0123456789", 31.25, A, ex[0].audio, 1000, 0.5);
        dsp::AnalysisResult r = analyseDemo(7.0e6, 7.040e6, 200, ex);
        check(r.kind == "PSK" && r.name == "PSK31", "Analyse PSK31: " + r.name + " / " + r.text);
    }
    {
        ex[0].hz = 7.040e6 - 700;
        ex[0].audio = dec::enc::cwAudio("CQ CQ CQ DE G4OGR G4OGR PSE K TEST TEST", 18, 700, A, 0.5, 0.1, 0.3);
        dsp::AnalysisResult r = analyseDemo(7.0e6, 7.040e6, 200, ex);
        check(r.kind == "CW" && std::fabs(r.wpm - 18) <= 2, "Analyse CW 18 wpm: " + r.name + " / " + r.text);
    }
    std::vector<AudioSt> none;
    {
        dsp::AnalysisResult r = analyseDemo(7.6e6, 7.65e6, 8000, none);
        check(r.kind == "AM", "Analyse AM (demo 7.65 MHz, a 1 kHz tone): " + r.name + " / " + r.text);
    }
    {
        dsp::AnalysisResult r = analyseDemo(145.3e6, 145.35e6, 12000, none);
        check(r.kind == "FM", "Analyse FM (demo 145.35 MHz): " + r.name + " / " + r.text);
    }
    {
        dsp::AnalysisResult r = analyseDemo(6.4e6, 6.45e6 + 1500, 3000, none);
        check(r.kind == "SSB", "Analyse SSB voice (demo 6.45 MHz USB): " + r.name + " / " + r.text);
    }
}

// ---------------------------------------------------------------- propagation
void testPropagation() {
    double lat = 0, lon = 0;
    check(dsp::locatorToLatLon("IO91wm", &lat, &lon) && std::fabs(lat - 51.52) < 0.05 && std::fabs(lon + 0.125) < 0.1,
          "Locator IO91wm -> " + f1(lat) + ", " + f1(lon));
    check(!dsp::locatorToLatLon("ZZ99", &lat, &lon), "Locator ZZ99 rejected");
    dsp::locatorToLatLon("IO91", &lat, &lon);  // London
    // 2026-06-21 12:00 UTC (summer noon), 2026-06-21 02:00 (summer night), 2026-12-21 12:00 (winter noon), 2026-12-21 23:00
    const double sumNoon = 1782043200.0, sumNight = sumNoon - 10 * 3600, winNoon = 1797854400.0, winNight = winNoon + 11 * 3600;
    const double e1 = dsp::sunElevation(sumNoon, lat, lon), e2 = dsp::sunElevation(winNoon, lat, lon);
    check(std::fabs(e1 - 62) < 2 && std::fabs(e2 - 15) < 2, "Sun at London noon: June " + f1(e1) + " deg, December " + f1(e2) + " deg (62, 15)");
    const dsp::PropGuide dn = dsp::propagationGuide(sumNoon, lat, lon, true), nn = dsp::propagationGuide(winNight, lat, lon, true);
    const dsp::PropGuide wn = dsp::propagationGuide(winNoon, lat, lon, true), sn = dsp::propagationGuide(sumNight, lat, lon, true);
    std::printf("      summer noon:  MUF %.1f LUF %.1f  %s\n", dn.limits.muf, dn.limits.luf, dn.line.c_str());
    std::printf("      winter night: MUF %.1f LUF %.1f  %s\n", nn.limits.muf, nn.limits.luf, nn.line.c_str());
    check(dn.limits.muf > nn.limits.muf + 5 && dn.limits.luf > nn.limits.luf, "Day MUF/LUF above night's");
    check(wn.limits.muf > dn.limits.muf && sn.limits.muf > nn.limits.muf, "Winter days and summer nights carry higher (MUF " +
                                                                                  f1(wn.limits.muf) + " > " + f1(dn.limits.muf) + ", " + f1(sn.limits.muf) + " > " + f1(nn.limits.muf) + ")");
    bool day20 = false, night80 = false, night10 = true;
    for (std::size_t i = 0; i < dn.bands.size(); ++i)
        if (dn.bands[i].name == "20 m") day20 = dn.bands[i].rating == dsp::PropRating::Open;
    for (std::size_t i = 0; i < nn.bands.size(); ++i) {
        if (nn.bands[i].name == "80 m") night80 = nn.bands[i].rating == dsp::PropRating::Open;
        if (nn.bands[i].name == "10 m") night10 = nn.bands[i].rating != dsp::PropRating::Unlikely;
    }
    check(day20 && night80 && !night10, "20 m open by day; 80 m open and 10 m closed at night");
    check(dn.hasNext && dn.nextDusk && nn.hasNext && !nn.nextDusk, "Next change: dusk after noon, dawn after midnight");
    const dsp::PropGuide bc = dsp::propagationGuide(winNight, lat, lon, false, false);
    check(bc.bands.size() == 14 && bc.line.find("Set your locator") != std::string::npos, "Broadcast group: 14 bands; " + bc.line);
    const double ss = dsp::predictedSunspots(1735689600.0);  // 2025-01-01: near the peak of cycle 25
    check(ss > 140 && dsp::predictedSunspots(2019.96 * 0 + 1577836800.0) < 15, "Sunspots: " + f1(ss) + " at 2025, low at 2020");
}

// ---------------------------------------------------------------- alerts
void testAlerts() {
    dsp::SpectrumExtras sx;
    sx.setAlerts(true);
    sx.setPeakHold(true);
    sx.setAveraging(true);
    const int W = 400;
    std::vector<float> pix(W);
    Rng rng = {5};
    int alertsLearn = 0, alertsAfter = 0, alertsKnown = 0;
    double where = 0;
    for (int k = 0; k < 30 * 30; ++k) {  // 30 s at 30 frames a second
        const double t = k / 30.0;
        for (int x = 0; x < W; ++x) pix[x] = static_cast<float>(-100 + 2 * rng.g());
        pix[100] = -60;               // a broadcaster, there all along (known while learning)
        if (t > 20) pix[300] = -70;   // a new carrier at 20 s
        sx.frame(&pix[0], W, 7.0e6, 7.4e6, t, 7.199e6, 7.201e6);
        for (std::size_t i = 0; i < sx.lastAlerts().size(); ++i) {
            const double f = sx.lastAlerts()[i].hz;
            if (t < 10) alertsLearn++;
            else if (std::fabs(f - 7.3e6) < 5e3) {
                alertsAfter++;
                where = f;
            } else
                alertsKnown++;
        }
    }
    check(alertsLearn == 0 && alertsKnown == 0 && alertsAfter == 1 && std::fabs(where - 7.3005e6) < 1500,
          "Alerts: one on the new carrier at " + f1(where / 1e3) + " kHz, none while learning or on the known station");
    check(sx.peak()[300] > -71 && sx.average()[300] > -72 && sx.average()[200] < -95, "Peak hold and averaging follow the carrier");
}

// ---------------------------------------------------------------- CPU
void testCpu() {
    const double fs = 2048000, centre = 14.1e6;
    dsp::SignalMonitor mon;
    for (int i = 0; i < 8; ++i) mon.add(centre - 400e3 + i * 100e3, 3000, "USB");
    std::vector<dsp::Complex> blk(16384);
    Rng rng = {3};
    for (std::size_t i = 0; i < blk.size(); ++i) blk[i] = dsp::Complex(static_cast<float>(0.01 * rng.g()), static_cast<float>(0.01 * rng.g()));
    const double secs = 20;
    const double t0 = now();
    for (std::size_t n = 0; n < secs * fs; n += blk.size()) mon.feedIq(&blk[0], blk.size(), fs, centre);
    const double t1 = now();
    // the speaker path: one ChannelExtractor to ~200 kHz
    dsp::ChannelExtractor ce;
    ce.configure(fs, 50e3, dsp::ChannelExtractor::decimationFor(fs, 200e3));
    std::vector<dsp::Complex> out;
    out.reserve(4096);
    const double t2 = now();
    for (std::size_t n = 0; n < secs * fs; n += blk.size()) {
        out.clear();
        ce.process(&blk[0], blk.size(), out);
    }
    const double t3 = now();
    double ms = 0;
    std::vector<AudioSt> none;
    dsp::DemoSource warm(kRate, 6.4e6);
    (void)warm;
    runAuto(6.4e6, 6.45e6 + 500, none, &ms);
    std::printf("CPU   Signal Monitor, 8 signals at 2.048 MS/s: %.2f %% of a core; a speaker's extractor (to %.0f kHz): %.2f %%; "
                "AUTO listen + analysis (USB voice, 2 s at 256 kS/s): %.0f ms\n",
                100 * (t1 - t0) / secs, ce.outputRate() / 1e3, 100 * (t3 - t2) / secs, ms);
    check((t1 - t0) / secs < 0.1, "CPU: the monitor under 10 % of a core");
}

}  // namespace

int main() {
    testAuto();
    testMonitor();
    testAnalyzer();
    testPropagation();
    testAlerts();
    testCpu();
    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
