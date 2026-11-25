// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/DemoSource.cpp
//  Layer:  Signal processing
//
//  What this file does
//  -------------------
//  dsp::DemoSource: a synthetic band, so the program (and the tests) can run
//  without a stick.  Everything is deterministic (fixed seeds).
//
//  FM broadcast band (centre 87.5-108 MHz): every MHz has stations at
//  +0.3 and +0.7 MHz (so 3-4 are visible in a 2 MHz window, at fixed
//  absolute frequencies that move correctly when the centre is retuned).
//  Each one is a real FM multiplex:
//      0.9 * [ (L+R)/2 + (L-R)/2 * sin(2 p) ] + 0.09 * sin(p) + 0.04 * rds * sin(3 p)
//  with p the 19 kHz pilot phase, L/R pre-emphasised (50 us), deviation
//  75 kHz per unit, and RDS: groups 0A (PS), 2A (RadioText), 4A (clock),
//  check words, differential coding, biphase symbols (one sine cycle per
//  bit, sign = symbol) on a 57 kHz carrier locked to the pilot.  One station
//  of the six names is mono (no pilot).
//
//  Elsewhere: every 2 MHz, six stations 300 kHz apart: AM with music, USB
//  and LSB "voices" (additive synthesis: harmonics of a gliding pitch shaped
//  by vowel formants, built directly as a one-sided complex signal), a CW
//  beacon keying "CQ CQ DE OFFGRID" in Morse, NFM with a 1 kHz tone, and AM
//  with a 1 kHz tone (good for trying SAM).
//
//  How samples are made: each station computes its program at a low rate
//  (WFM multiplex ~256 kHz, the others ~64 kHz), then at the full rate
//  either integrates the frequency (FM: phase accumulator + sine table) or
//  interpolates the complex envelope linearly and rotates it to the
//  station's offset.  Noise (Gaussian, -42 dBFS) is added and the sum is
//  quantised to unsigned 8-bit IQ exactly like an RTL-SDR, so the whole
//  receive chain is exercised.
//
//  How it fits in
//  --------------
//  --demo in the application, and tests/test_dsp.cpp.  RdsEncoder is in
//  Rds.cpp.
// ============================================================================
#include "Receiver.h"

#include "DspCommon.h"
#include "Rds.h"

#include <algorithm>
#include <cmath>

namespace dsp {

using namespace detail;

namespace {

// ---- Random numbers (xorshift64*), deterministic ---------------------------
struct Rng {
    std::uint64_t s;
    explicit Rng(std::uint64_t seed = 0x0FF6121D5D12ULL) : s(seed ? seed : 1) {}
    std::uint64_t next() {
        s ^= s >> 12;
        s ^= s << 25;
        s ^= s >> 27;
        return s * 2685821657736338717ULL;
    }
    double uniform() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
    // Approximately Gaussian, unit variance: sum of four 16-bit uniforms
    // (Irwin-Hall), much cheaper than Box-Muller at 4 million per second.
    float gauss() {
        const std::uint64_t r = next();
        const float sum = static_cast<float>((r & 0xFFFF) + ((r >> 16) & 0xFFFF) + ((r >> 32) & 0xFFFF) + (r >> 48)) * (1.0f / 65536.0f);
        return (sum - 2.0f) * 1.7320508f;
    }
};

// ---- Audio programs (real, one sample per call) --------------------------
struct Audio {
    virtual ~Audio() {}
    virtual float next() = 0;
};

struct Tone : Audio {
    const float* tab;
    std::uint32_t ph, inc;
    float amp;
    Tone(double hz, double rate, float a) : tab(sinTable()), ph(0), inc(phaseIncrement(hz, rate)), amp(a) {}
    float next() {
        const float v = amp * tableSin(tab, ph);
        ph += inc;
        return v;
    }
};

// A plucked melody: random notes of a pentatonic scale, each with an
// exponential decay and two harmonics.
struct Melody : Audio {
    const float* tab;
    double rate;
    Rng rng;
    float amp, env, decay;
    std::uint32_t ph, inc;
    long left;
    Melody(double r, std::uint64_t seed, float a)
        : tab(sinTable()), rate(r), rng(seed), amp(a), env(0), decay(static_cast<float>(std::exp(-1.0 / (0.25 * r)))),
          ph(0), inc(0), left(0) {}
    float next() {
        if (left <= 0) {
            static const double kNotes[10] = {261.6, 293.7, 329.6, 392.0, 440.0, 523.3, 587.3, 659.3, 784.0, 880.0};
            inc = phaseIncrement(kNotes[rng.next() % 10], rate);
            left = static_cast<long>((0.15 + 0.15 * static_cast<double>(rng.next() % 3)) * rate);
            env = 1.0f;
        }
        --left;
        env *= decay;
        const float v = tableSin(tab, ph) + 0.4f * tableSin(tab, ph * 2) + 0.2f * tableSin(tab, ph * 3);
        ph += inc;
        return amp * env * v * 0.62f;
    }
};

// ---- Complex-envelope programs (one sample per call) ------------------------
struct Envelope {
    virtual ~Envelope() {}
    virtual Complex next() = 0;
};

// AM: carrier * (1 + m * audio).
struct AmEnvelope : Envelope {
    Audio* audio;
    float m;
    AmEnvelope(Audio* a, float depth) : audio(a), m(depth) {}
    ~AmEnvelope() { delete audio; }
    Complex next() { return Complex(1.0f + m * audio->next(), 0.0f); }
};

// A voice-like SSB signal.  A talk spurt is a few syllables; a syllable is a
// vowel (three formants) with a raised-sine loudness and a falling pitch.
// The harmonics of the pitch are summed as complex exponentials, which gives
// the one-sided (analytic) signal directly: positive frequencies = USB, the
// conjugate = LSB.
struct VoiceEnvelope : Envelope {
    double rate, f0base, f0, phi;
    bool lsb;
    Rng rng;
    long sylLeft, sylLen, pause;
    int sylInSpurt;
    double F[3], amps[40];
    int harmonics, update;
    float loud;
    VoiceEnvelope(double r, double pitch, bool isLsb, std::uint64_t seed)
        : rate(r), f0base(pitch), f0(pitch), phi(0), lsb(isLsb), rng(seed), sylLeft(0), sylLen(1), pause(0),
          sylInSpurt(0), harmonics(0), update(0), loud(0) {
        F[0] = 700;
        F[1] = 1200;
        F[2] = 2500;
        for (int k = 0; k < 40; ++k) amps[k] = 0;
    }
    void newSyllable() {
        static const double kVowels[5][3] = {{730, 1090, 2440}, {530, 1840, 2480}, {270, 2290, 3010}, {570, 840, 2410}, {300, 870, 2240}};
        if (sylInSpurt <= 0) {  // between talk spurts: a pause
            sylInSpurt = 3 + static_cast<int>(rng.next() % 5);
            pause = static_cast<long>((0.3 + 0.5 * rng.uniform()) * rate);
        }
        --sylInSpurt;
        const int v = static_cast<int>(rng.next() % 5);
        for (int i = 0; i < 3; ++i) F[i] = kVowels[v][i];
        sylLen = sylLeft = static_cast<long>((0.12 + 0.16 * rng.uniform()) * rate);
        f0base *= 0.97 + 0.06 * rng.uniform();  // drift of the speaker's pitch
        if (f0base < 90) f0base = 90;
        if (f0base > 220) f0base = 220;
    }
    void updateAmps() {
        // Harmonic k at k*f0: glottal tilt 1/k times the formant resonances.
        harmonics = std::min(39, static_cast<int>(3000.0 / f0));
        static const double kGain[3] = {1.0, 0.6, 0.3};
        for (int k = 1; k <= harmonics; ++k) {
            const double f = k * f0;
            double a = 0;
            for (int i = 0; i < 3; ++i) {
                const double d = (f - F[i]) / 110.0;
                a += kGain[i] / (1.0 + d * d);
            }
            amps[k] = a / std::sqrt(static_cast<double>(k));
        }
        // Normalise: rms of the voice at full loudness = 0.7 of the
        // station's level, so levelDb is roughly its peak envelope power.
        double p = 0;
        for (int k = 1; k <= harmonics; ++k) p += amps[k] * amps[k];
        const double g = p > 0 ? 0.7 / std::sqrt(p) : 0.0;
        for (int k = 1; k <= harmonics; ++k) amps[k] *= g;
    }
    Complex next() {
        if (pause > 0) {
            --pause;
            loud = 0;
            return Complex();
        }
        if (sylLeft <= 0) {
            newSyllable();
            if (pause > 0) return Complex();
        }
        const double t = 1.0 - static_cast<double>(sylLeft) / sylLen;  // 0..1 through the syllable
        --sylLeft;
        f0 = f0base * (1.08 - 0.16 * t);
        if (--update <= 0) {
            updateAmps();
            update = 64;
        }
        loud = static_cast<float>(std::sin(kPi * t));
        phi += kTwoPi * f0 / rate;
        if (phi > kTwoPi) phi -= kTwoPi;
        const double wr = std::cos(phi), wi = std::sin(phi);
        double zr = wr, zi = wi, sr = 0, si = 0;
        for (int k = 1; k <= harmonics; ++k) {
            sr += amps[k] * zr;
            si += amps[k] * zi;
            const double nr = zr * wr - zi * wi;  // z *= w: next harmonic
            zi = zr * wi + zi * wr;
            zr = nr;
        }
        return Complex(static_cast<float>(sr * loud), static_cast<float>((lsb ? -si : si) * loud));
    }
};

// Morse keyer: "CQ CQ DE OFFGRID" at 18 words per minute, raised-cosine
// keying edges (5 ms) so the signal stays narrow.
struct CwEnvelope : Envelope {
    std::vector<unsigned char> keys;  // one entry per dot length: 1 = carrier on
    std::size_t pos;
    long dotLen, left;
    double ramp, rampStep;
    CwEnvelope(double rate, const char* text) : pos(0), left(0), ramp(0) {
        static const char* const kMorse[36] = {".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-", ".-..",
                                               "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "...-", ".--", "-..-",
                                               "-.--", "--..", "-----", ".----", "..---", "...--", "....-", ".....", "-....", "--...",
                                               "---..", "----."};
        for (const char* p = text; *p; ++p) {
            const char c = *p;
            int idx = -1;
            if (c >= 'A' && c <= 'Z') idx = c - 'A';
            else if (c >= '0' && c <= '9') idx = 26 + (c - '0');
            if (idx < 0) {  // word gap: 7 dots (3 already added after the letter)
                keys.insert(keys.end(), 4, 0);
                continue;
            }
            for (const char* m = kMorse[idx]; *m; ++m) {
                keys.insert(keys.end(), *m == '-' ? 3 : 1, 1);
                keys.push_back(0);  // gap inside a letter
            }
            keys.insert(keys.end(), 2, 0);  // letter gap = 3 dots
        }
        keys.insert(keys.end(), 14, 0);  // pause before repeating
        dotLen = static_cast<long>(1.2 / 18.0 * rate);
        rampStep = 1.0 / (0.005 * rate);
    }
    Complex next() {
        if (left <= 0) {
            pos = (pos + 1) % keys.size();
            left = dotLen;
        }
        --left;
        if (keys[pos]) ramp = std::min(1.0, ramp + rampStep);
        else ramp = std::max(0.0, ramp - rampStep);
        return Complex(static_cast<float>(0.5 - 0.5 * std::cos(kPi * ramp)), 0.0f);
    }
};

// ---- Stations ------------------------------------------------------------------
struct Station {
    virtual ~Station() {}
    virtual void add(float* re, float* im, std::size_t n) = 0;
};

// The FM multiplex of a broadcast station, one sample per call.
struct WfmProgram {
    Audio *left, *right;
    bool stereo;
    const float* tab;
    std::uint32_t pilot, pilotInc;
    float preP, preG, prevL, prevR;
    RdsEncoder rds;
    double bitPos, bitInc;
    int sym;
    WfmProgram(double rate, Audio* l, Audio* r, bool st)
        : left(l), right(r), stereo(st), tab(sinTable()), pilot(0), pilotInc(phaseIncrement(19000, rate)), prevL(0),
          prevR(0), bitPos(0), bitInc(1187.5 / rate), sym(1) {
        // Pre-emphasis (50 us), the inverse of the receiver's de-emphasis:
        // y = (x - p x[-1]) / (1 - p).
        preP = static_cast<float>(std::exp(-1.0 / (50e-6 * rate)));
        preG = 1.0f / (1.0f - preP);
    }
    ~WfmProgram() {
        delete left;
        delete right;
    }
    float next() {
        const float l0 = left->next(), r0 = right ? right->next() : l0;
        const float l = (l0 - preP * prevL) * preG, r = (r0 - preP * prevR) * preG;
        prevL = l0;
        prevR = r0;
        bitPos += bitInc;
        if (bitPos >= 1.0) {
            bitPos -= 1.0;
            sym = rds.nextSymbol();
        }
        // Biphase symbol: one sine cycle per bit (+half then -half), sign = symbol.
        const float shape = tableSin(tab, static_cast<std::uint32_t>(bitPos * 4294967296.0));
        const float rdsV = 0.04f * static_cast<float>(sym) * shape * tableSin(tab, pilot * 3);
        float mpx = 0.45f * (l + r) + rdsV;
        if (stereo) mpx += 0.45f * (l - r) * tableSin(tab, pilot * 2) + 0.09f * tableSin(tab, pilot);
        pilot += pilotInc;
        return mpx;
    }
};

// Frequency modulation at the full rate: the instantaneous frequency is
// interpolated between program samples (rate fs / D) and the phase
// accumulates every sample.
struct FmStation : Station {
    const float* tab;
    std::uint32_t phase, carrierInc;
    std::int64_t fmCur, fmStep;  // phase increment of the modulation, and its ramp
    float amp;
    int D, sub;
    double devScale;  // program unit -> phase increment
    WfmProgram* wfm;
    Audio* audio;     // NFM
    FmStation(double fs, double offset, float a, int d, double deviation, WfmProgram* w, Audio* au)
        : tab(sinTable()), phase(0), carrierInc(phaseIncrement(offset, fs)), fmCur(0), fmStep(0), amp(a), D(d), sub(0),
          devScale(deviation / fs * 4294967296.0), wfm(w), audio(au) {}
    ~FmStation() {
        delete wfm;
        delete audio;
    }
    void add(float* re, float* im, std::size_t n) {
        std::uint32_t ph = phase;
        for (std::size_t i = 0; i < n; ++i) {
            if (sub == 0) {
                // Next program sample: ramp the frequency to it over D
                // samples (linear interpolation; holding it in steps would
                // leave images of the multiplex every fs/D).
                const float m = wfm ? wfm->next() : audio->next();
                const std::int64_t target = static_cast<std::int64_t>(std::floor(m * devScale + 0.5));
                fmStep = (target - fmCur) / D;
            }
            if (++sub == D) sub = 0;
            fmCur += fmStep;
            ph += carrierInc + static_cast<std::uint32_t>(fmCur);
            re[i] += amp * tableCos(tab, ph);
            im[i] += amp * tableSin(tab, ph);
        }
        phase = ph;
    }
};

// A complex envelope at the program rate, interpolated linearly to the full
// rate and rotated to the station's offset.
struct EnvelopeStation : Station {
    const float* tab;
    std::uint32_t phase, inc;
    float amp, invD;
    int D, sub;
    Complex prev, cur;
    Envelope* env;
    EnvelopeStation(double fs, double offset, float a, int d, Envelope* e)
        : tab(sinTable()), phase(0), inc(phaseIncrement(offset, fs)), amp(a), invD(1.0f / d), D(d), sub(0), env(e) {}
    ~EnvelopeStation() { delete env; }
    void add(float* re, float* im, std::size_t n) {
        std::uint32_t ph = phase;
        for (std::size_t i = 0; i < n; ++i) {
            if (sub == 0) {
                prev = cur;
                cur = env->next() * amp;
            }
            const float t = sub * invD;
            if (++sub == D) sub = 0;
            const float vr = prev.real() + (cur.real() - prev.real()) * t;
            const float vi = prev.imag() + (cur.imag() - prev.imag()) * t;
            const float c = tableCos(tab, ph), s = tableSin(tab, ph);
            re[i] += vr * c - vi * s;
            im[i] += vr * s + vi * c;
            ph += inc;
        }
        phase = ph;
    }
};

// Unix time of a civil date (UTC), for the RDS clock's fixed epoch.
double unixTime(int y, int m, int d, int hh, int mm) {
    // days_from_civil (H. Hinnant)
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    const long days = era * 146097L + static_cast<long>(doe) - 719468L;
    return days * 86400.0 + hh * 3600.0 + mm * 60.0;
}

}  // namespace

// ============================================================================
//  DemoSource::Impl
// ============================================================================
struct DemoSource::Impl {
    double fs, center;
    Rng noiseRng;
    float noiseSigma;
    std::vector<DemoStation> list;
    std::vector<Station*> gens;
    std::vector<float> re, im;

    Impl(double rate, double c) : fs(rate > 0 ? rate : 2048000), center(c), noiseRng(0x5DEECE66DULL) {
        // -42 dBFS of noise in total: a realistic floor, and enough (0.7 LSB
        // rms) to dither the 8-bit quantiser.  With less, the quantiser's
        // intermodulation lands exactly on other stations of the regular
        // demo grid and garbles their RDS.
        noiseSigma = static_cast<float>(std::sqrt(std::pow(10.0, -42.0 / 10.0) / 2.0));
        build();
    }
    ~Impl() { clear(); }
    void clear() {
        for (std::size_t i = 0; i < gens.size(); ++i) delete gens[i];
        gens.clear();
        list.clear();
    }

    static bool isFmBand(double c) { return c >= 87.5e6 && c <= 108e6; }

    void build() {
        clear();
        const double half = 0.45 * fs;  // keep stations clear of the band edges
        const int dWfm = std::max(1, static_cast<int>(std::floor(fs / 256000.0 + 0.5)));
        const int dLow = std::max(1, static_cast<int>(std::floor(fs / 64000.0 + 0.5)));
        const double rWfm = fs / dWfm, rLow = fs / dLow;
        if (isFmBand(center)) {
            static const char* const kNames[6] = {"OFFGRID ", "DEMO FM ", "RADIO 1 ", "CLASSIC ", "JAZZ 24 ", "NEWS 24 "};
            static const char* const kText[6] = {
                "OFFgrid-SDR demo station - left 1 kHz, right 1.5 kHz test tones",
                "You are listening to the OFFgrid-SDR terminal receiver demo",
                "Radio One - stereo melodies, all day",
                "Classic - a mono station with RDS",
                "Jazz 24 - more synthetic music",
                "News 24 - the news is that this is a demo"};
            static const std::uint16_t kPi[6] = {0xC0DE, 0xD3F0, 0xC201, 0xC3A5, 0xC4B2, 0xC5C7};
            static const int kPty[6] = {10, 9, 10, 14, 24, 1};
            static const double kLevel[6] = {-15, -22, -28, -25, -31, -35};
            const long m0 = static_cast<long>(std::floor((center - half) / 1e6)) - 1;
            const long m1 = static_cast<long>(std::floor((center + half) / 1e6)) + 1;
            for (long m = m0; m <= m1; ++m) {
                for (int j = 0; j < 2; ++j) {
                    const double f = m * 1e6 + (j ? 0.7e6 : 0.3e6);
                    if (std::fabs(f - center) > half) continue;
                    const int idx = static_cast<int>((2 * m + j) % 6);
                    const std::uint64_t seed = 1000 + static_cast<std::uint64_t>(m) * 7 + j;
                    Audio *l, *r;
                    if (idx == 0) {
                        l = new Tone(1000, rWfm, 0.6f);
                        r = new Tone(1500, rWfm, 0.6f);
                    } else {
                        l = new Melody(rWfm, seed, 0.6f);
                        r = idx == 3 ? 0 : new Melody(rWfm, seed * 31 + 5, 0.6f);
                    }
                    WfmProgram* prog = new WfmProgram(rWfm, l, r, idx != 3);
                    prog->rds.setup(kPi[idx], kNames[idx], kText[idx], kPty[idx], idx == 5, false, idx != 5,
                                    unixTime(2026, 6, 21, 12, 0));
                    gens.push_back(new FmStation(fs, f - center, dbToLin(kLevel[idx]), dWfm, 75000.0, prog, 0));
                    DemoStation st;
                    st.offsetHz = f - center;
                    st.mode = Mode::WFM;
                    st.name = kNames[idx];
                    st.levelDb = kLevel[idx];
                    list.push_back(st);
                }
            }
        } else {
            static const double kPos[6] = {0.15e6, 0.45e6, 0.75e6, 1.05e6, 1.35e6, 1.65e6};
            static const Mode kMode[6] = {Mode::AM, Mode::USB, Mode::CW, Mode::LSB, Mode::NFM, Mode::SAM};
            static const char* const kNames[6] = {"AM music", "USB voice", "CW beacon", "LSB voice", "NFM tone", "AM tone"};
            static const double kLevel[6] = {-30, -40, -45, -42, -35, -38};
            const long m0 = static_cast<long>(std::floor((center - half) / 2e6)) - 1;
            const long m1 = static_cast<long>(std::floor((center + half) / 2e6)) + 1;
            for (long m = m0; m <= m1; ++m) {
                for (int j = 0; j < 6; ++j) {
                    const double f = m * 2e6 + kPos[j];
                    if (f <= 0 || std::fabs(f - center) > half) continue;
                    const double off = f - center;
                    const float amp = dbToLin(kLevel[j]);
                    const std::uint64_t seed = 5000 + static_cast<std::uint64_t>(m) * 13 + j;
                    Station* g = 0;
                    switch (j) {
                        case 0: g = new EnvelopeStation(fs, off, amp, dLow, new AmEnvelope(new Melody(rLow, seed, 1.0f), 0.7f)); break;
                        case 1: g = new EnvelopeStation(fs, off, amp, dLow, new VoiceEnvelope(rLow, 120, false, seed)); break;
                        case 2: g = new EnvelopeStation(fs, off, amp, dLow, new CwEnvelope(rLow, "CQ CQ DE OFFGRID ")); break;
                        case 3: g = new EnvelopeStation(fs, off, amp, dLow, new VoiceEnvelope(rLow, 170, true, seed)); break;
                        case 4: g = new FmStation(fs, off, amp, dLow, 3000.0, 0, new Tone(1000, rLow, 1.0f)); break;
                        default: g = new EnvelopeStation(fs, off, amp, dLow, new AmEnvelope(new Tone(1000, rLow, 1.0f), 0.5f)); break;
                    }
                    gens.push_back(g);
                    DemoStation st;
                    st.offsetHz = off;
                    st.mode = kMode[j];
                    st.name = kNames[j];
                    st.levelDb = kLevel[j];
                    list.push_back(st);
                }
            }
        }
    }

    void generate(std::size_t n, std::vector<std::uint8_t>& bytes) {
        if (re.size() < n) {
            re.resize(n);
            im.resize(n);
        }
        const float sg = noiseSigma;
        for (std::size_t i = 0; i < n; ++i) {
            re[i] = sg * noiseRng.gauss();
            im[i] = sg * noiseRng.gauss();
        }
        for (std::size_t g = 0; g < gens.size(); ++g) gens[g]->add(&re[0], &im[0], n);
        bytes.resize(2 * n);
        // Quantise like the stick: 127.5 + 127.5 * x, rounded, clipped.
        for (std::size_t i = 0; i < n; ++i) {
            int a = static_cast<int>(re[i] * 127.5f + 128.0f);
            int b = static_cast<int>(im[i] * 127.5f + 128.0f);
            bytes[2 * i] = static_cast<std::uint8_t>(a < 0 ? 0 : (a > 255 ? 255 : a));
            bytes[2 * i + 1] = static_cast<std::uint8_t>(b < 0 ? 0 : (b > 255 ? 255 : b));
        }
    }
};

DemoSource::DemoSource(double sampleRate, double centerHz) : impl_(new Impl(sampleRate, centerHz)) {}
DemoSource::~DemoSource() {}

void DemoSource::setCenter(double centerHz) {
    if (centerHz == impl_->center) return;
    impl_->center = centerHz;
    impl_->build();
}

const std::vector<DemoStation>& DemoSource::stations() const { return impl_->list; }

void DemoSource::generate(std::size_t n, std::vector<std::uint8_t>& bytes) { impl_->generate(n, bytes); }

}  // namespace dsp
