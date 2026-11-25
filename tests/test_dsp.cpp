// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_dsp.cpp
//  Layer:  Tests
//
//  What this file does
//  -------------------
//  Self-checking test of the DSP library.  Prints PASS/FAIL per check and
//  returns nonzero if anything failed.  Checks:
//    - FFT / SpectrumAnalyzer: a tone peaks in the right bin at ~0 dBFS;
//    - convertU8 scaling;
//    - the resampler: exactly audioRate frames per second of input, for
//      several modes and stick rates, over 1 s and over 10 s (no drift);
//    - every DemoSource station demodulates: level, audio present, tone
//      frequency recovered, squelch closes on an empty channel;
//    - WFM: stereo pilot, left/right separation, RDS PS/PI/RT/clock;
//    - SSB: the opposite sideband is rejected;
//    - performance: MS/s processed per CPU second, per mode.
//
//  How it fits in
//  --------------
//  Build (one-off, from the repository root):
//    c++ -std=c++11 -O2 src/dsp/*.cpp tests/test_dsp.cpp -o test_dsp
// ============================================================================
#include "../src/dsp/Fft.h"
#include "../src/dsp/Receiver.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

using namespace dsp;

namespace {

int g_failures = 0, g_checks = 0;

void check(bool ok, const std::string& name, const std::string& detail = "") {
    ++g_checks;
    if (!ok) ++g_failures;
    std::printf("%s  %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : "  -- ", detail.c_str());
}

std::string fmt(const char* f, double a, double b = 0, double c = 0) {
    char buf[256];
    std::snprintf(buf, sizeof buf, f, a, b, c);
    return buf;
}

const double kPi = 3.14159265358979323846;

// One channel (0 = left, 1 = right) of interleaved stereo, from frame `from`.
std::vector<float> channelOf(const std::vector<float>& inter, int ch, std::size_t from, std::size_t count) {
    std::vector<float> out;
    for (std::size_t i = from; i < from + count && 2 * i + 1 < inter.size(); ++i) out.push_back(inter[2 * i + ch]);
    return out;
}

double rms(const std::vector<float>& x) {
    double s = 0;
    for (std::size_t i = 0; i < x.size(); ++i) s += x[i] * x[i];
    return x.empty() ? 0 : std::sqrt(s / x.size());
}

// Power spectrum (Hann, N = 16384) of x; returns |X|^2 per bin (first half).
std::vector<double> spectrum(const std::vector<float>& x) {
    const int N = 16384;
    detail::Fft fft(N);
    std::vector<Complex> buf(N);
    for (int i = 0; i < N; ++i) {
        const float w = static_cast<float>(0.5 - 0.5 * std::cos(2 * kPi * i / N));
        buf[i] = Complex(i < static_cast<int>(x.size()) ? x[i] * w : 0.0f, 0.0f);
    }
    fft.forward(&buf[0]);
    std::vector<double> p(N / 2);
    for (int k = 0; k < N / 2; ++k) p[k] = std::norm(buf[k]);
    return p;
}

// Frequency of the strongest component between lo and hi Hz (parabolic
// interpolation between bins).
double peakFrequency(const std::vector<float>& x, double rate, double lo = 50, double hi = 20000) {
    const std::vector<double> p = spectrum(x);
    const int N = 16384;
    int best = -1;
    for (int k = 1; k < N / 2 - 1; ++k) {
        const double f = k * rate / N;
        if (f < lo || f > hi) continue;
        if (best < 0 || p[k] > p[best]) best = k;
    }
    if (best < 1) return 0;
    const double a = std::log(p[best - 1] + 1e-30), b = std::log(p[best] + 1e-30), c = std::log(p[best + 1] + 1e-30);
    const double d = 0.5 * (a - c) / (a - 2 * b + c);
    return (best + d) * rate / N;
}

// Power within +-20 Hz of f.
double powerAt(const std::vector<float>& x, double rate, double f) {
    const std::vector<double> p = spectrum(x);
    const int N = 16384;
    double s = 0;
    for (int k = 0; k < N / 2; ++k)
        if (std::fabs(k * rate / N - f) <= 20) s += p[k];
    return s;
}

double cpuSeconds() { return static_cast<double>(std::clock()) / CLOCKS_PER_SEC; }

// A receiver under test, fed the same IQ as the others.
struct Probe {
    std::string name;
    Receiver rx;
    std::vector<float> audio;
    double levelSum;
    int levelN;
    double psTime;  // seconds of input until RDS PS matched (-1 = never)
    std::string expectPs;
    Probe() : rx(48000), levelSum(0), levelN(0), psTime(-1) {}
};

ReceiverConfig makeConfig(double rate, double offset, Mode mode) {
    ReceiverConfig c;
    c.sampleRate = rate;
    c.offsetHz = offset;
    c.mode = mode;
    return c;
}

// Runs `seconds` of the demo band through every probe.
void runDemo(DemoSource& demo, std::vector<Probe*>& probes, double rate, double seconds) {
    const std::size_t chunk = 65536;
    std::vector<std::uint8_t> bytes;
    std::vector<Complex> iq;
    const std::size_t total = static_cast<std::size_t>(seconds * rate);
    for (std::size_t done = 0; done < total; done += chunk) {
        demo.generate(chunk, bytes);
        convertU8(&bytes[0], bytes.size(), iq);
        const double t = static_cast<double>(done + chunk) / rate;
        for (std::size_t p = 0; p < probes.size(); ++p) {
            Probe& pr = *probes[p];
            pr.rx.process(&iq[0], iq.size(), pr.audio);
            if (t > 0.5) {
                pr.levelSum += pr.rx.channelDbfs();
                ++pr.levelN;
            }
            if (!pr.expectPs.empty() && pr.psTime < 0 && pr.rx.rds().ps == pr.expectPs) pr.psTime = t;
        }
    }
}

// ---------------------------------------------------------------------------
void testFft() {
    std::printf("\n== FFT / SpectrumAnalyzer\n");
    const int N = 2048;
    const int bins[3] = {100, -300, 0};
    for (int t = 0; t < 3; ++t) {
        SpectrumAnalyzer sa(N, 0.0);
        std::vector<Complex> x(N);
        for (int i = 0; i < N; ++i) {
            const double a = 2 * kPi * bins[t] * i / N;
            x[i] = Complex(static_cast<float>(std::cos(a)), static_cast<float>(std::sin(a)));
        }
        const bool ready = sa.push(&x[0], x.size(), 2048000, 1000);
        const std::vector<float>& p = sa.powerDb();
        int best = 0;
        for (int k = 1; k < N; ++k)
            if (p[k] > p[best]) best = k;
        const int expect = N / 2 + bins[t];
        check(ready && best == expect && std::fabs(p[best]) < 0.1,
              fmt("full-scale tone at bin %+.0f peaks at index N/2%+.0f", bins[t], best - N / 2),
              fmt("peak %.2f dBFS, expected index %.0f", p[best], expect));
        // Far from the tone the Blackman-Harris side lobes must be very low.
        const int far = (expect + N / 2) % N;
        check(p[far] < -90, fmt("leakage %.0f bins away below -90 dB", N / 2.0), fmt("%.1f dB", p[far]));
    }
    // Frame rate limit: 1 s of input at 30 frames/s -> 30 frames.
    SpectrumAnalyzer sa(1024, 0.5);
    std::vector<Complex> blk(8000, Complex(0.1f, 0.0f));
    int frames = 0;
    for (int i = 0; i < 256; ++i) frames += sa.push(&blk[0], blk.size(), 2048000, 30) ? 1 : 0;
    check(frames >= 29 && frames <= 31, "frame rate limit: ~30 frames per second of input", fmt("%.0f frames", frames));
}

void testConvert() {
    std::printf("\n== convertU8\n");
    const std::uint8_t b[4] = {0, 255, 128, 127};
    std::vector<Complex> out;
    convertU8(b, 4, out);
    check(out.size() == 2 && out[0].real() == -1.0f && out[0].imag() == 1.0f &&
              std::fabs(out[1].real() - 0.5f / 127.5f) < 1e-6 && std::fabs(out[1].imag() + 0.5f / 127.5f) < 1e-6,
          "0/255 -> -1/+1, 127/128 -> +-0.5 LSB around 0");
}

void testResamplerCount() {
    std::printf("\n== Resampler: frames per second of input\n");
    const double rates[2] = {2048000, 2400000};
    const Mode modes[4] = {Mode::WFM, Mode::NFM, Mode::USB, Mode::CW};
    unsigned seed = 1;
    for (int r = 0; r < 2; ++r) {
        for (int m = 0; m < 4; ++m) {
            Receiver rx(48000);
            rx.configure(makeConfig(rates[r], 12345, modes[m]));
            std::vector<Complex> noise(300000);
            for (std::size_t i = 0; i < noise.size(); ++i) {
                seed = seed * 1103515245u + 12345u;
                const float a = ((seed >> 8) & 0xFFFF) / 65536.0f - 0.5f;
                seed = seed * 1103515245u + 12345u;
                const float b = ((seed >> 8) & 0xFFFF) / 65536.0f - 0.5f;
                noise[i] = Complex(a * 0.1f, b * 0.1f);
            }
            std::vector<float> audio;
            const std::size_t total = static_cast<std::size_t>(rates[r]);
            std::size_t done = 0, frames1 = 0;
            for (int sec = 0; sec < 10; ++sec) {
                std::size_t end = total * (sec + 1);
                while (done < end) {
                    seed = seed * 1103515245u + 12345u;
                    std::size_t n = 1000 + (seed >> 8) % 250000;  // odd block sizes on purpose
                    if (n > end - done) n = end - done;
                    rx.process(&noise[0], n, audio);
                    done += n;
                }
                if (sec == 0) frames1 = audio.size() / 2;
            }
            // The average rate must be exact: 480000 +-1 frames after 10 s
            // (0.1 frame/s).  At any single instant the count can be a few
            // frames off: SSB/CW go through a ~12 kHz stage, one sample of
            // which is ~4 audio frames.
            const double f1 = static_cast<double>(frames1), f10 = audio.size() / 2.0;
            check(std::fabs(f1 - 48000) <= 4 && std::fabs(f10 - 480000) <= 1,
                  std::string(modeName(modes[m])) + fmt(" at %.3f MS/s: 48000 frames/s (10 s: 480000+-1, 1 s: +-4)", rates[r] / 1e6),
                  fmt("1 s: %.0f frames, 10 s: %.0f frames", f1, f10));
        }
    }
}

// Blocks of 1..13 samples (and empty ones) must work and give the same
// amount of audio as big blocks.
void testTinyBlocks() {
    std::printf("\n== Tiny and empty blocks\n");
    const double rate = 2048000;
    DemoSource demo(rate, 7.0e6);
    std::vector<std::uint8_t> bytes;
    std::vector<Complex> iq;
    demo.generate(204800, bytes);  // 0.1 s
    convertU8(&bytes[0], bytes.size(), iq);
    const Mode modes[7] = {Mode::WFM, Mode::NFM, Mode::AM, Mode::SAM, Mode::USB, Mode::LSB, Mode::CW};
    for (int m = 0; m < 7; ++m) {
        Receiver a(48000), b(48000);
        a.configure(makeConfig(rate, 50000, modes[m]));
        b.configure(makeConfig(rate, 50000, modes[m]));
        std::vector<float> outA, outB;
        a.process(&iq[0], iq.size(), outA);
        std::size_t i = 0, k = 0;
        while (i < iq.size()) {
            const std::size_t n = std::min<std::size_t>(k++ % 14, iq.size() - i);  // 0..13
            b.process(n ? &iq[i] : 0, n, outB);
            i += n;
        }
        bool finite = true;
        for (std::size_t j = 0; j < outB.size(); ++j)
            if (!(outB[j] >= -1.0f && outB[j] <= 1.0f)) finite = false;
        const double diff = std::fabs(static_cast<double>(outA.size()) - static_cast<double>(outB.size())) / 2;
        check(finite && diff <= 1, std::string(modeName(modes[m])) + fmt(": %.0f frames from tiny blocks vs %.0f from one block", outB.size() / 2.0, outA.size() / 2.0));
    }
}

void testFmBand() {
    std::printf("\n== Demo FM band (centre 100 MHz): WFM, stereo, RDS\n");
    const double rate = 2048000;
    DemoSource demo(rate, 100e6);
    const std::vector<DemoStation>& st = demo.stations();
    check(st.size() >= 3 && st.size() <= 4, fmt("3-4 WFM stations in view (%.0f)", st.size()));
    std::vector<Probe*> probes;
    for (std::size_t i = 0; i < st.size(); ++i) {
        Probe* p = new Probe;
        p->name = st[i].name;
        p->expectPs = st[i].name;
        p->rx.configure(makeConfig(rate, st[i].offsetHz, Mode::WFM));
        probes.push_back(p);
    }
    runDemo(demo, probes, rate, 5.0);
    for (std::size_t i = 0; i < st.size(); ++i) {
        Probe& p = *probes[i];
        const std::string nm = "WFM '" + p.name + "'";
        const double level = p.levelSum / std::max(1, p.levelN);
        check(std::fabs(level - st[i].levelDb) < 3, nm + fmt(" channel level %.1f dBFS (station %.0f)", level, st[i].levelDb));
        check(p.rx.noiseDbfs() < level - 15, nm + fmt(" noise estimate %.1f dBFS well below the signal", p.rx.noiseDbfs()));
        const bool stereoExpected = p.name != "CLASSIC ";
        check(p.rx.stereoPilot() == stereoExpected, nm + (stereoExpected ? " stereo pilot detected" : " mono: no pilot"));
        const RdsInfo r = p.rx.rds();
        check(p.psTime > 0 && p.psTime < 4.0, nm + " RDS PS == '" + p.name + "'",
              "got '" + r.ps + "'" + fmt(" after %.2f s, %.0f groups", p.psTime, r.groups));
        const std::vector<float> L = channelOf(p.audio, 0, 72000, 16384), R = channelOf(p.audio, 1, 72000, 16384);
        check(rms(L) > 0.05 && rms(R) > 0.05, nm + fmt(" audio present (RMS L %.3f, R %.3f)", rms(L), rms(R)));
        if (p.name == "OFFGRID ") {
            const double fl = peakFrequency(L, 48000), fr = peakFrequency(R, 48000);
            check(std::fabs(fl - 1000) < 3 && std::fabs(fr - 1500) < 3, nm + fmt(" left tone %.1f Hz, right tone %.1f Hz", fl, fr));
            const double sepL = 10 * std::log10(powerAt(L, 48000, 1000) / (powerAt(L, 48000, 1500) + 1e-20));
            const double sepR = 10 * std::log10(powerAt(R, 48000, 1500) / (powerAt(R, 48000, 1000) + 1e-20));
            check(sepL > 25 && sepR > 25, nm + fmt(" stereo separation L %.1f dB, R %.1f dB", sepL, sepR));
            char pi[16];
            std::snprintf(pi, sizeof pi, "0x%04X", static_cast<unsigned>(r.pi));
            check(r.pi == 0xC0DE, nm + " RDS PI 0xC0DE", pi);
            check(r.radioText == "OFFgrid-SDR demo station - left 1 kHz, right 1.5 kHz test tones",
                  nm + " RDS RadioText", "'" + r.radioText + "'");
            check(r.pty == 10 && r.ptyName == "Pop Music" && r.music, nm + " RDS PTY 10 Pop Music, MS=music", r.ptyName);
            check(r.clock == "2026-06-21 12:00", nm + " RDS clock (group 4A)", "'" + r.clock + "'");
        }
        if (p.name == "CLASSIC ") {
            double d = 0;
            for (std::size_t k = 0; k < L.size(); ++k) d += (L[k] - R[k]) * (L[k] - R[k]);
            check(std::sqrt(d / L.size()) < 1e-6, nm + " mono station: L == R");
        }
    }
    for (std::size_t i = 0; i < probes.size(); ++i) delete probes[i];
}

void testHfBand() {
    std::printf("\n== Demo HF band (centre 7 MHz): AM, SAM, USB, LSB, CW, NFM\n");
    const double rate = 2048000;
    DemoSource demo(rate, 7.0e6);
    const std::vector<DemoStation>& st = demo.stations();
    check(st.size() == 6, fmt("6 stations in view (%.0f)", st.size()));
    std::vector<Probe*> probes;
    for (std::size_t i = 0; i < st.size(); ++i) {
        Probe* p = new Probe;
        p->name = std::string(modeName(st[i].mode)) + " '" + st[i].name + "'";
        ReceiverConfig c = makeConfig(rate, st[i].offsetHz, st[i].mode);
        // Squelch at -60 dBFS on the continuous signals (voice and Morse
        // have pauses during which a squelch rightly closes).
        const bool bursty = st[i].mode == Mode::USB || st[i].mode == Mode::LSB || st[i].mode == Mode::CW;
        c.squelchDb = bursty ? -200 : -60;
        p->rx.configure(c);
        probes.push_back(p);
    }
    // Extra probes: AM demodulation of the AM tone station (listed as SAM),
    // and an empty channel (squelch).
    std::size_t amToneIdx = 0;
    for (std::size_t i = 0; i < st.size(); ++i)
        if (st[i].name == "AM tone") amToneIdx = i;
    Probe* amTone = new Probe;
    amTone->name = "AM 'AM tone'";
    amTone->rx.configure(makeConfig(rate, st[amToneIdx].offsetHz, Mode::AM));
    probes.push_back(amTone);
    Probe* empty = new Probe;
    empty->name = "AM on an empty channel";
    {
        ReceiverConfig c = makeConfig(rate, 200000, Mode::AM);
        c.squelchDb = -60;
        empty->rx.configure(c);
    }
    probes.push_back(empty);

    runDemo(demo, probes, rate, 3.0);
    const std::size_t from = 60000;  // skip 1.25 s (AGC, PLL, filters settle)
    for (std::size_t i = 0; i < probes.size(); ++i) {
        Probe& p = *probes[i];
        const double level = p.levelSum / std::max(1, p.levelN);
        const std::vector<float> L = channelOf(p.audio, 0, from, 48000);
        if (&p == empty) {
            check(!p.rx.squelchOpen() && rms(L) < 1e-4, p.name + fmt(": squelch closed (level %.1f dBFS, RMS %.5f)", level, rms(L)));
            continue;
        }
        const double expectLevel = i < st.size() ? st[i].levelDb : st[amToneIdx].levelDb;
        const Mode mode = i < st.size() ? st[i].mode : Mode::AM;
        // Voice and CW are bursty: their average level is below the peak.
        const double tol = (mode == Mode::USB || mode == Mode::LSB || mode == Mode::CW) ? 12 : 3;
        check(level < expectLevel + 3 && level > expectLevel - tol,
              p.name + fmt(": channel level %.1f dBFS (station %.0f)", level, expectLevel));
        if (!(mode == Mode::USB || mode == Mode::LSB || mode == Mode::CW) && i < st.size())
            check(p.rx.squelchOpen(), p.name + ": squelch open at -60 dBFS");
        check(rms(L) > 0.03, p.name + fmt(": audio present, RMS %.3f", rms(L)));
        const std::string nm = i < st.size() ? st[i].name : "";
        if (nm == "CW beacon") {
            const double f = peakFrequency(L, 48000);
            check(std::fabs(f - 700) < 5, p.name + fmt(": CW tone at the 700 Hz pitch (%.1f Hz)", f));
        }
        if (nm == "NFM tone" || nm == "AM tone" || &p == amTone) {
            const double f = peakFrequency(L, 48000);
            check(std::fabs(f - 1000) < 3, p.name + fmt(": 1 kHz tone recovered (%.1f Hz)", f));
        }
        if (mode == Mode::USB || mode == Mode::LSB) {
            // Speech-like: energy between 100 Hz and 3 kHz.
            const std::vector<double> sp = spectrum(L);
            double in = 0, all = 0;
            for (std::size_t k = 1; k < sp.size(); ++k) {
                const double f = k * 48000.0 / 16384;
                all += sp[k];
                if (f >= 100 && f <= 3000) in += sp[k];
            }
            check(in / all > 0.95, p.name + fmt(": %.1f%% of the audio energy in 100-3000 Hz", 100 * in / all));
        }
    }
    for (std::size_t i = 0; i < probes.size(); ++i) delete probes[i];
}

// A clean complex tone 1 kHz above (or below) the tuned frequency: USB must
// hear it as 1 kHz, LSB must not hear it at all (and vice versa).
void testSideband() {
    std::printf("\n== SSB sideband selection (synthetic tone, no noise)\n");
    const double rate = 2048000, offset = 250000;
    for (int side = 0; side < 2; ++side) {
        const double toneHz = offset + (side == 0 ? 1000 : -1000);
        std::vector<Complex> x(rate);
        for (std::size_t i = 0; i < x.size(); ++i) {
            const double a = 2 * kPi * toneHz * static_cast<double>(i) / rate;
            x[i] = Complex(static_cast<float>(0.1 * std::cos(a)), static_cast<float>(0.1 * std::sin(a)));
        }
        Receiver usb(48000), lsb(48000);
        usb.configure(makeConfig(rate, offset, Mode::USB));
        lsb.configure(makeConfig(rate, offset, Mode::LSB));
        std::vector<float> au, al;
        for (std::size_t i = 0; i < x.size(); i += 32768) {
            const std::size_t n = std::min<std::size_t>(32768, x.size() - i);
            usb.process(&x[i], n, au);
            lsb.process(&x[i], n, al);
        }
        Receiver& right = side == 0 ? usb : lsb;
        Receiver& wrong = side == 0 ? lsb : usb;
        const std::vector<float> a = channelOf(side == 0 ? au : al, 0, 24000, 16384);
        const double f = peakFrequency(a, 48000);
        const double rej = right.channelDbfs() - wrong.channelDbfs();
        check(std::fabs(right.channelDbfs() + 20) < 0.5 && std::fabs(f - 1000) < 2,
              fmt(side == 0 ? "USB hears a tone 1 kHz above as %.1f Hz at %.1f dBFS" : "LSB hears a tone 1 kHz below as %.1f Hz at %.1f dBFS",
                  f, right.channelDbfs()));
        check(rej > 50, fmt(side == 0 ? "LSB rejects it by %.1f dB" : "USB rejects it by %.1f dB", rej));
    }
}

void testPerformance() {
    std::printf("\n== Performance (input MS/s processed per CPU second; %% of one core at 2.4 MS/s)\n");
    const double rate = 2400000;
    const double seconds = 4.0;
    const std::size_t chunk = 65536;
    std::vector<std::vector<Complex> > fm, hf;
    std::vector<std::uint8_t> bytes;
    double t0 = cpuSeconds();
    {
        DemoSource demo(rate, 100e6);
        for (std::size_t done = 0; done < seconds * rate; done += chunk) {
            demo.generate(chunk, bytes);
            fm.push_back(std::vector<Complex>());
            convertU8(&bytes[0], bytes.size(), fm.back());
        }
    }
    const double genCpu = cpuSeconds() - t0;
    std::printf("INFO  DemoSource (FM band) generates %.1f MS/s per CPU second (%.1f%% of a core at 2.4 MS/s)\n",
                seconds * rate / genCpu / 1e6, 100 * genCpu / seconds);
    {
        DemoSource demo(rate, 7.0e6);
        for (std::size_t done = 0; done < seconds * rate; done += chunk) {
            demo.generate(chunk, bytes);
            hf.push_back(std::vector<Complex>());
            convertU8(&bytes[0], bytes.size(), hf.back());
        }
    }
    DemoSource fmDemo(rate, 100e6), hfDemo(rate, 7.0e6);
    const Mode modes[7] = {Mode::WFM, Mode::NFM, Mode::AM, Mode::SAM, Mode::USB, Mode::LSB, Mode::CW};
    for (int m = 0; m < 7; ++m) {
        const bool isFm = modes[m] == Mode::WFM;
        const std::vector<DemoStation>& st = isFm ? fmDemo.stations() : hfDemo.stations();
        double off = st[0].offsetHz;  // FM band: the first station (stereo, RDS)
        if (!isFm)
            for (std::size_t i = 0; i < st.size(); ++i)
                if (st[i].mode == modes[m]) off = st[i].offsetHz;
        Receiver rx(48000);
        rx.configure(makeConfig(rate, off, modes[m]));
        std::vector<float> audio;
        audio.reserve(48000 * 2 * 5);
        const std::vector<std::vector<Complex> >& data = isFm ? fm : hf;
        const double c0 = cpuSeconds();
        for (std::size_t b = 0; b < data.size(); ++b) rx.process(&data[b][0], data[b].size(), audio);
        const double cpu = cpuSeconds() - c0;
        const double msps = seconds * rate / cpu / 1e6;
        std::printf("PERF  %-4s %7.1f MS/s per CPU second  (%.1f%% of a core at 2.4 MS/s)%s\n", modeName(modes[m]), msps,
                    240.0 / msps, isFm ? "  [stereo + RDS]" : "");
        if (isFm) check(240.0 / msps < 25.0, fmt("WFM stereo+RDS uses %.1f%% of a core at 2.4 MS/s (< 25%%)", 240.0 / msps));
        if (isFm) {
            const RdsInfo r = rx.rds();
            check(rx.stereoPilot() && !r.ps.empty() && r.ps != "        ", "WFM at 2.4 MS/s: pilot and RDS decoded", "'" + r.ps + "'");
        }
    }
}

}  // namespace

int main() {
    std::printf("OFFgrid-SDR DSP self-test\n");
    testFft();
    testConvert();
    testResamplerCount();
    testTinyBlocks();
    testFmBand();
    testHfBand();
    testSideband();
    testPerformance();
    std::printf("\n%d checks, %d failed: %s\n", g_checks, g_failures, g_failures ? "FAIL" : "ALL PASS");
    return g_failures ? 1 : 0;
}
