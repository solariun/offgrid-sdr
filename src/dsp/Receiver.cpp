// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/Receiver.cpp
//  Layer:  Signal processing
//
//  What this file does
//  -------------------
//  dsp::Receiver: one tuned signal, IQ in -> stereo audio out.  The chain:
//
//    IQ (fs, e.g. 2.048 MS/s)
//     -> Mixer: shift the tuned frequency (offsetHz) to 0 Hz
//     -> stage 1: FIR low-pass + decimate by D1
//          WFM:    to ~320-400 kHz (enough for the whole FM multiplex)
//          others: to ~48 kHz
//     -> stage 2 (SSB, CW): FIR low-pass + decimate by 4 (or 2) to ~12 kHz,
//          so the long, sharp channel filter runs at a low rate
//     -> channel filter (bandwidthHz): real low-pass, or for SSB a complex
//          band-pass on one side of 0 Hz (low-pass kernel shifted)
//          -> channel power (dBFS) and squelch
//     -> demodulator:
//          WFM  FM discriminator -> multiplex -> RDS decoder
//               19 kHz pilot PLL -> L-R from the 38 kHz subcarrier
//               decimate L+R, L-R (15 kHz low-pass, pilot removed)
//               matrix to L/R (mono fallback), de-emphasis
//          NFM  FM discriminator, DC removal
//          AM   envelope / carrier level - 1  (carrier = DC removal and AGC)
//          SAM  carrier PLL (port of OFFgrid-SDR's DemodSAM), in-phase output
//          USB/LSB  real part of the one-sided signal, SSB high-pass, AGC
//          CW   narrow low-pass, BFO at cwPitchHz, AGC
//     -> fractional Resampler to audioRate (exact on average)
//     -> squelch gate (ramped), volume, clip to -1..1, interleave L R.
//
//  How it fits in
//  --------------
//  Public API in Receiver.h.  Building blocks in Filters.h, RDS in Rds.h.
//  The chain design follows jtarrio/signals (Apache-2.0: demod-wbfm.ts,
//  demod-am.ts, demod-ssb.ts, demod-cw.ts, demod-nbfm.ts, the pilot detector
//  and stereo separator in filters.ts/demodulators.ts) and OFFgrid-SDR
//  (diddy-boy: DemodSAM, SsbHighPass, RdsDecoder), restructured for C++ with
//  multi-stage decimation so it runs comfortably in real time.
//
//  Contributing
//  ------------
//  process() must not allocate once its buffers have grown to the block
//  size in use (std::vector members that only ever grow).  Any parameter
//  that changes a filter design goes through build(); cheap ones (offset,
//  de-emphasis, squelch, volume) are applied in configure() directly.
// ============================================================================
#include "Receiver.h"

#include "DspCommon.h"
#include "Fft.h"
#include "Filters.h"
#include "Rds.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <limits>
#include <mutex>

namespace dsp {

using namespace detail;

// ============================================================================
//  Small public helpers
// ============================================================================
void convertU8(const std::uint8_t* bytes, std::size_t count, std::vector<Complex>& out) {
    // `count` is the number of bytes (I, Q, I, Q ...); out gets count/2
    // samples.  0..255 maps to -1..+1 around 127.5 (the stick's zero).
    struct Lut {
        float v[256];
        Lut() {
            for (int i = 0; i < 256; ++i) v[i] = (static_cast<float>(i) - 127.5f) / 127.5f;
        }
    };
    static const Lut table;  // built once, thread-safe (C++11 magic statics)
    const float* lut = table.v;
    const std::size_t n = count / 2;
    out.resize(n);
    for (std::size_t i = 0; i < n; ++i) out[i] = Complex(lut[bytes[2 * i]], lut[bytes[2 * i + 1]]);
}

const char* modeName(Mode mode) {
    switch (mode) {
        case Mode::WFM: return "WFM";
        case Mode::NFM: return "NFM";
        case Mode::AM: return "AM";
        case Mode::SAM: return "SAM";
        case Mode::USB: return "USB";
        case Mode::LSB: return "LSB";
        case Mode::CW: return "CW";
    }
    return "?";
}

bool parseMode(const std::string& name, Mode& mode) {
    std::string s;
    for (std::size_t i = 0; i < name.size(); ++i) s += static_cast<char>(std::toupper(static_cast<unsigned char>(name[i])));
    if (s == "WFM" || s == "WBFM") mode = Mode::WFM;
    else if (s == "NFM" || s == "NBFM" || s == "FM") mode = Mode::NFM;
    else if (s == "AM") mode = Mode::AM;
    else if (s == "SAM") mode = Mode::SAM;
    else if (s == "USB") mode = Mode::USB;
    else if (s == "LSB") mode = Mode::LSB;
    else if (s == "CW") mode = Mode::CW;
    else return false;
    return true;
}

double defaultBandwidth(Mode mode) {
    switch (mode) {
        case Mode::WFM: return 150000;
        case Mode::NFM: return 12500;
        case Mode::AM: return 10000;
        case Mode::SAM: return 10000;
        case Mode::USB: return 2800;
        case Mode::LSB: return 2800;
        case Mode::CW: return 500;
    }
    return 10000;
}

ReceiverConfig::ReceiverConfig()
    : sampleRate(2048000), offsetHz(0), mode(Mode::WFM), bandwidthHz(0), stereo(true), deemphasisUs(50),
      squelchDb(-200), cwPitchHz(700), agc(true), volume(1) {}

RdsInfo::RdsInfo() : pi(0), ps(8, ' '), pty(-1), tp(false), ta(false), music(false), groups(0) {}

// ============================================================================
//  Internal pieces
// ============================================================================
namespace {

const float kManualGain = 20.0f;  // AGC off: fixed audio gain (26 dB)
const double kSsbLowEdge = 100.0; // SSB pass band starts here (OFFgrid-SDR ssbEdges)

// ---- Noise floor estimate ----------------------------------------------------
// Every 50 ms, a 256-point FFT of the stage-1 output (the band around the
// tuned signal, before the channel filter).  The spectra are averaged and
// the 25th percentile of the bins outside the channel is taken as the noise
// density: signals occupy few bins, so a low percentile ignores them.
// A bin (Hann window normalised by sum w^2) reads the total noise power
// across the whole sample rate, so the noise in the channel is
// bin * bandwidth / rate.
class NoiseEstimator {
public:
    NoiseEstimator() : rate_(1), filled_(0), countdown_(0), have_(false), noiseDb_(-120.0) {}
    void setup(double rate, double inner, double outer, double channelBw) {
        const int N = 256;
        rate_ = rate;
        fft_.resize(N);
        win_.resize(N);
        double s2 = 0;
        for (int i = 0; i < N; ++i) {
            win_[i] = static_cast<float>(0.5 - 0.5 * std::cos(kTwoPi * i / N));
            s2 += win_[i] * win_[i];
        }
        frame_.assign(N, Complex());
        avg_.assign(N, 0.0f);
        bins_.clear();
        for (int k = 0; k < N; ++k) {
            const double f = std::fabs((k < N / 2 ? k : k - N) * rate / N);
            if (f >= inner && f <= outer) bins_.push_back(k);
        }
        if (bins_.size() < 8) {  // channel fills the band: use everything
            bins_.clear();
            for (int k = 0; k < N; ++k) bins_.push_back(k);
        }
        sorted_.resize(bins_.size());
        // 25th percentile of an averaged (~14 degrees of freedom) spectrum is
        // ~0.73 of the mean: correct for it.
        scale_ = channelBw / rate / s2 / 0.73;
        filled_ = 0;
        countdown_ = 0;
        have_ = false;
    }
    void feed(const float* re, const float* im, std::size_t n) {
        std::size_t i = 0;
        const int N = static_cast<int>(frame_.size());
        while (i < n) {
            if (countdown_ > 0) {
                const std::size_t skip = std::min<std::size_t>(static_cast<std::size_t>(countdown_), n - i);
                countdown_ -= static_cast<long>(skip);
                i += skip;
                continue;
            }
            frame_[filled_] = Complex(re[i] * win_[filled_], im[i] * win_[filled_]);
            ++i;
            if (++filled_ == N) {
                compute();
                filled_ = 0;
                countdown_ = static_cast<long>(rate_ / 20);
            }
        }
    }
    double noiseDb() const { return noiseDb_; }

private:
    void compute() {
        fft_.forward(&frame_[0]);
        const int N = static_cast<int>(frame_.size());
        for (int k = 0; k < N; ++k) {
            const float p = frame_[k].real() * frame_[k].real() + frame_[k].imag() * frame_[k].imag();
            avg_[k] = have_ ? avg_[k] + 0.25f * (p - avg_[k]) : p;
        }
        have_ = true;
        for (std::size_t j = 0; j < bins_.size(); ++j) sorted_[j] = avg_[bins_[j]];
        std::vector<float>::iterator q = sorted_.begin() + sorted_.size() / 4;
        std::nth_element(sorted_.begin(), q, sorted_.end());
        noiseDb_ = powerToDb(*q * scale_);
    }

    double rate_;
    Fft fft_;
    std::vector<float> win_, avg_, sorted_;
    std::vector<Complex> frame_;
    std::vector<int> bins_;
    double scale_;
    int filled_;
    long countdown_;
    bool have_;
    double noiseDb_;
};

// ---- 19 kHz pilot PLL (WFM stereo) ------------------------------------------
// Locks a local oscillator to the pilot (pilot = A sin(theta)).  Phase
// detector: mpx * cos(theta) contains A/2 sin(error) plus audio that sits at
// 4 kHz and up; two one-pole low-passes (1.5 kHz) remove most of it before
// the PI loop filter (30 Hz natural frequency).  Lock detector: the
// in-phase arm mpx * sin(theta), low-passed, is A/2 when locked.
// Outputs sin(2 theta) for the 38 kHz L-R subcarrier.  (The idea of
// checking that a pilot is really there before decoding stereo comes from
// jtarrio/signals' PilotDetector; this is a classic PLL instead.)
struct StereoPll {
    const float* tab;
    std::uint32_t phase;
    double freq, nominal, maxDev;  // radians per sample
    double alpha, beta;
    float lpA, e1, e2;             // phase-error low-pass
    float lockA, level;            // in-phase arm, ~A/2
    bool locked;

    StereoPll() : tab(sinTable()), phase(0), freq(0), nominal(0), maxDev(0), alpha(0), beta(0), lpA(0), e1(0), e2(0),
                  lockA(0), level(0), locked(false) {}
    void setup(double rate) {
        nominal = kTwoPi * 19000.0 / rate;
        maxDev = kTwoPi * 30.0 / rate;  // the pilot is 19 kHz +-2 Hz; allow +-30 Hz
        const double kd = 0.045;        // detector gain for a 9% pilot (A/2)
        const double wn = kTwoPi * 30.0 / rate, zeta = 0.707;
        alpha = 2.0 * zeta * wn / kd;
        beta = wn * wn / kd;
        lpA = static_cast<float>(decayCoef(rate, 1.0 / (kTwoPi * 1500.0)));
        lockA = static_cast<float>(decayCoef(rate, 0.02));
        reset();
    }
    void reset() {
        phase = 0;
        freq = nominal;
        e1 = e2 = level = 0;
        locked = false;
    }
    // mpx in, diff = mpx * 2 sin(2 theta) out (L-R at baseband plus junk
    // above 15 kHz that the audio filter removes).
    void process(const float* mpx, float* diff, std::size_t n) {
        const double toPhase = 4294967296.0 / kTwoPi;
        for (std::size_t i = 0; i < n; ++i) {
            const float s = tableSin(tab, phase), c = tableCos(tab, phase), x = mpx[i];
            e1 += lpA * (x * c - e1);
            e2 += lpA * (e1 - e2);
            level += lockA * (x * s - level);
            freq += beta * e2;
            if (freq > nominal + maxDev) freq = nominal + maxDev;
            else if (freq < nominal - maxDev) freq = nominal - maxDev;
            const double step = freq + alpha * e2;
            phase += static_cast<std::uint32_t>(static_cast<std::int64_t>(step * toPhase));
            diff[i] = x * 4.0f * s * c;  // 2 * sin(2 theta) = 4 sin cos
        }
        // Lock with hysteresis: pilot amplitude above 4% on, below 2.4% off.
        if (!locked && level > 0.020f) locked = true;
        else if (locked && level < 0.012f) locked = false;
    }
};

// ---- Synchronous AM (port of OFFgrid-SDR's DemodSAM) ------------------------
struct SamDemod {
    double rate, ph, fr, lock, carr, agc, dc, alpha, beta, dcA, agcA, lockA, carrA;
    bool locked;
    SamDemod() : rate(48000) { setup(48000); }
    void setup(double r) {
        rate = r;
        dcA = decayCoef(r, 0.05);
        agcA = decayCoef(r, 0.5);
        lockA = decayCoef(r, 0.05);
        carrA = decayCoef(r, 0.3);
        reset();
    }
    void reset() {
        ph = fr = lock = 0;
        locked = false;
        dc = agc = carr = 0;
        setLoop(false);
    }
    void setLoop(bool isLocked) {
        // Wide loop to acquire, narrow once locked.
        const double fn = isLocked ? 25 : 120, zeta = 0.707, wT = kTwoPi * fn / rate;
        alpha = 2 * zeta * wT;
        beta = wT * wT;
    }
    void process(const float* I, const float* Q, std::size_t n, float* out, bool useAgc) {
        const double maxFr = kTwoPi * 1000.0 / rate;  // +-1 kHz pull-in
        if (agc == 0 && n) {  // after a retune: start from this block's level, no overshoot
            double m = 0;
            for (std::size_t i = 0; i < n; ++i) m += std::sqrt(I[i] * I[i] + Q[i] * Q[i]);
            agc = m / n;
            dc = agc;
        }
        for (std::size_t i = 0; i < n; ++i) {
            const double c = std::cos(ph), s = std::sin(ph), x = I[i], y = Q[i];
            const double a = x * c + y * s, b = y * c - x * s;  // rotate by -phase
            // Linear phase detector normalised by the averaged carrier.
            // Unlike atan2, the sidebands then average out in the loop filter
            // even when a fade takes the carrier below them.  atan2 is used
            // only until a carrier estimate exists.
            carr += carrA * (a - carr);
            double err = carr > 1e-7 ? b / carr : std::atan2(b, a);
            if (err > 1) err = 1;
            else if (err < -1) err = -1;
            fr += beta * err;
            if (fr > maxFr) fr = maxFr;
            else if (fr < -maxFr) fr = -maxFr;
            ph += fr + alpha * err;
            if (ph > kPi) ph -= kTwoPi;
            else if (ph < -kPi) ph += kTwoPi;
            const double mag = std::sqrt(x * x + y * y);
            lock += lockA * ((mag > 0 ? a / mag : 0) - lock);  // ~1 when phase-aligned
            agc += agcA * (mag - agc);
            dc += dcA * (a - dc);  // the carrier (DC of the in-phase arm)
            const double g = useAgc ? (agc > 1e-9 ? 1.0 / agc : 0.0) : kManualGain;
            out[i] = static_cast<float>((a - dc) * g);
        }
        const bool nowLocked = locked ? lock > 0.5 : lock > 0.8;  // hysteresis
        if (nowLocked != locked) {
            locked = nowLocked;
            setLoop(nowLocked);
        }
    }
};

inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace

// ============================================================================
//  Receiver::Impl
// ============================================================================
struct Receiver::Impl {
    explicit Impl(int rate)
        : audioRate(rate > 0 ? rate : 48000), built(false), bw(0), rate1(0), rate2(0), rateA(0), d1(1), d2(1),
          tw(0), stage1Pass(0), fmGain(1), prevRe(0), prevIm(0), amCarrier(0), stereoBlend(0), bfoPhase(0),
          bfoInc(0), sqGain(1), sqOpen(true), chanPow(-1), chanDb(-200), noiseDb(-200), pilot(false),
          sqOpenAtomic(true), rdsChanges(~0u) {}

    int audioRate;
    ReceiverConfig cfg;
    bool built;

    // derived design
    double bw, rate1, rate2, rateA;
    int d1, d2;
    double tw, stage1Pass;

    // chain
    Mixer mixer;
    FirDecimator stage1, stage2, channel, wfmAudio;
    Resampler resampler;
    NoiseEstimator noise;
    StereoPll pll;
    RdsDecoder rdsDec;
    OnePole deemL, deemR, nfmDc;
    SamDemod sam;
    Agc agc;
    BiquadCascade ssbHp;
    float fmGain, prevRe, prevIm;
    float amCarrier;
    float stereoBlend;
    std::uint32_t bfoPhase, bfoInc;
    float sqGain;
    bool sqOpen;
    double chanPow;

    // buffers (grow only)
    std::vector<float> a0, a1, b0, b1, c0, c1, d0, d1v, mpx, diff, e0, e1, o0, o1;

    // results, readable from other threads
    std::atomic<double> chanDb, noiseDb;
    std::atomic<bool> pilot, sqOpenAtomic;
    mutable std::mutex rdsMutex;
    RdsInfo rdsSnap;
    unsigned rdsChanges;

    bool isNarrowSideband() const { return cfg.mode == Mode::USB || cfg.mode == Mode::LSB || cfg.mode == Mode::CW; }

    double effectiveBandwidth() const {
        double b = cfg.bandwidthHz > 0 ? cfg.bandwidthHz : defaultBandwidth(cfg.mode);
        const double fs = cfg.sampleRate;
        const double r1 = fs / std::max(1.0, std::floor(fs / 48000.0));
        switch (cfg.mode) {
            case Mode::WFM: return clampd(b, 30000, 220000);
            case Mode::NFM: return clampd(b, 3000, 0.6 * r1);
            case Mode::AM:
            case Mode::SAM: return clampd(b, 1000, 0.6 * r1);
            case Mode::USB:
            case Mode::LSB: return clampd(b, 500, 6000);
            case Mode::CW: return clampd(b, 50, 3000);
        }
        return b;
    }

    // Designs every filter for the current mode, bandwidth and rates.
    void build() {
        resetCarrier();
        const double fs = cfg.sampleRate;
        const Mode mode = cfg.mode;
        bw = effectiveBandwidth();
        const double pitch = clampd(cfg.cwPitchHz, 200, 2000);
        mixer.setFrequency(-cfg.offsetHz, fs);
        mixer.reset();
        if (mode == Mode::WFM) {
            // Stage 1: to 320-400 kHz.  Pass band wide enough for the channel
            // and for the noise estimate beside it; the stop band starts where
            // aliases would land back inside that pass band.
            d1 = std::max(1, static_cast<int>(std::floor(fs / 320000.0)));
            rate1 = fs / d1;
            tw = 0.15 * bw;
            stage1Pass = std::min(std::max(0.4 * rate1, bw / 2 + tw), 0.45 * rate1);
            if (d1 > 1) stage1.setup(designLowPass(fs, stage1Pass, rate1 - stage1Pass, 70), d1);
            rate2 = rate1;
            d2 = 1;
            channel.setup(designLowPass(rate1, bw / 2, bw / 2 + tw, 60), 1);
            // Audio: L+R and L-R are low-passed at 15 kHz (pilot at 19 kHz
            // removed) and decimated to ~44-50 kHz in one filter.
            const int d3 = std::max(1, static_cast<int>(std::floor(rate1 / 44000.0)));
            rateA = rate1 / d3;
            wfmAudio.setup(designLowPass(rate1, 15000, 19000, 50), d3);
            fmGain = static_cast<float>(rate1 / (kTwoPi * 75000.0));  // +-75 kHz deviation = +-1
            pll.setup(rate1);
            rdsDec.clear();
            rdsChanges = ~0u;
            {
                std::lock_guard<std::mutex> lock(rdsMutex);
                rdsSnap = RdsInfo();
            }
            stereoBlend = 0;
            setupDeemphasis();
            const double pass = 15000, stop = std::max(pass + 1000, std::min(rateA, double(audioRate)) - pass);
            resampler.setup(rateA, audioRate, pass, stop);
            noise.setup(rate1, bw / 2 + tw, 0.9 * stage1Pass, bw);
        } else {
            // Stage 1: to ~48 kHz.
            d1 = std::max(1, static_cast<int>(std::floor(fs / 48000.0)));
            rate1 = fs / d1;
            double edge;  // highest frequency of the channel, from 0 Hz
            if (mode == Mode::USB || mode == Mode::LSB) {
                tw = std::max(0.15 * bw, 150.0);
                edge = bw + tw;
            } else if (mode == Mode::CW) {
                tw = std::max(0.2 * bw, 50.0);
                edge = bw / 2 + tw;
            } else {
                tw = 0.15 * bw;
                edge = bw / 2 + tw;
            }
            stage1Pass = std::min(std::max(0.3 * rate1, edge), 0.45 * rate1);
            if (d1 > 1) stage1.setup(designLowPass(fs, stage1Pass, rate1 - stage1Pass, 70, 4095), d1);
            // Stage 2 (SSB/CW): down to ~12 kHz where the edge allows it.
            d2 = 1;
            if (isNarrowSideband()) {
                if (edge <= 0.4 * rate1 / 4) d2 = 4;
                else if (edge <= 0.4 * rate1 / 2) d2 = 2;
            }
            rate2 = rate1 / d2;
            if (d2 > 1) stage2.setup(designLowPass(rate1, edge, rate2 - edge, 70), d2);
            // Channel filter at rate2.
            if (mode == Mode::USB || mode == Mode::LSB) {
                // Pass band [100 Hz, bw] above (USB) or below (LSB) 0 Hz: a
                // low-pass of half that width, shifted to its centre.
                const double half = (bw - kSsbLowEdge) / 2, centre = (bw + kSsbLowEdge) / 2 * (mode == Mode::USB ? 1 : -1);
                const std::vector<float> proto = designLowPass(rate2, half, half + tw, 60, 2047);
                std::vector<float> hr(proto.size()), hi(proto.size());
                const int m = static_cast<int>(proto.size() - 1) / 2;
                for (int k = 0; k < static_cast<int>(proto.size()); ++k) {
                    const double a = kTwoPi * centre * (k - m) / rate2;
                    hr[k] = static_cast<float>(proto[k] * std::cos(a));
                    hi[k] = static_cast<float>(proto[k] * std::sin(a));
                }
                channel.setupComplex(hr, hi, 1);
                ssbHp.configureHighPass(kSsbLowEdge, rate2);
            } else {
                channel.setup(designLowPass(rate2, bw / 2, bw / 2 + tw, 60, 2047), 1);
            }
            rateA = rate2;
            // Audio low-pass inside the resampler.
            double pass;
            switch (mode) {
                case Mode::NFM: pass = std::min(4000.0, bw / 2); break;
                case Mode::AM:
                case Mode::SAM: pass = bw / 2; break;
                case Mode::USB:
                case Mode::LSB: pass = bw; break;
                default: pass = pitch + bw / 2; break;  // CW
            }
            pass = std::min(pass, 0.4 * std::min(rateA, double(audioRate)));
            const double stop = std::max(pass + 500, std::min(pass + 3000, std::min(rateA, double(audioRate)) - pass));
            resampler.setup(rateA, audioRate, pass, stop);
            // FM: +-(0.4 * bandwidth) deviation = +-1 (5 kHz for 12.5 kHz).
            fmGain = static_cast<float>(rate2 / (kTwoPi * 0.4 * bw));
            nfmDc.setup(rate2, 0.1);
            nfmDc.y = 0;
            sam.setup(rate2);
            amCarrier = 0;
            agc.setup(rateA, 0.002, mode == Mode::CW ? 0.5 : 0.3, 0.3, 0.35f, 3000.0f);
            bfoInc = phaseIncrement(pitch, rateA);
            bfoPhase = 0;
            noise.setup(rate1, edge, 0.9 * stage1Pass, mode == Mode::USB || mode == Mode::LSB ? bw - kSsbLowEdge : bw);
        }
        prevRe = 1;
        prevIm = 0;
        chanPow = -1;
        sqGain = 1;
        pilot = false;
        built = true;
    }

    void setupDeemphasis() {
        const double tau = clampd(cfg.deemphasisUs, 0, 1000) * 1e-6;
        if (tau <= 0) {
            deemL.a = deemR.a = 1.0f;
        } else {
            deemL.setup(rateA, tau);
            deemR.setup(rateA, tau);
        }
    }

    // FM discriminator: angle between consecutive samples, scaled.
    void discriminate(const float* re, const float* im, std::size_t n, float* out) {
        float pr = prevRe, pi = prevIm;
        const float g = fmGain;
        for (std::size_t i = 0; i < n; ++i) {
            const float r = re[i], q = im[i];
            out[i] = fastAtan2(q * pr - r * pi, r * pr + q * pi) * g;  // arg(x * conj(prev))
            pr = r;
            pi = q;
        }
        prevRe = pr;
        prevIm = pi;
    }

    std::vector<float> tap;  // decoder audio of the last process() call
    // Carrier offset measured by the demodulator (Hz from the tuned
    // frequency): FM = mean discriminator output, SAM = the PLL frequency.
    bool carrierValid = false;
    std::atomic<double> carrierHz{0.0};

    // Mean frequency of the discriminator output (before any DC removal):
    // the first 0.5 s after a (re)tune is skipped (filters settling), then
    // the mean of everything since, whose error falls as listening goes on.
    double carrierSum = 0, carrierCount = 0, carrierSkip = 0;
    void trackCarrier(const float* fm, std::size_t n, double rate) {
        if (n == 0 || fmGain == 0) return;
        if (!carrierValid && carrierCount == 0 && carrierSkip == 0) carrierSkip = 0.5 * rate;
        if (carrierSkip > 0) {
            carrierSkip -= static_cast<double>(n);
            if (carrierSkip > 0) return;
            carrierSkip = -1;  // done skipping (until the next reset)
        }
        double sum = 0;
        for (std::size_t i = 0; i < n; ++i) sum += fm[i];
        carrierSum += sum / fmGain * rate / kTwoPi;
        carrierCount += static_cast<double>(n);
        carrierHz = carrierSum / carrierCount;
        carrierValid = true;
    }
    void resetCarrier() {
        carrierValid = false;
        carrierSum = carrierCount = carrierSkip = 0;
    }

    void process(const Complex* iq, std::size_t n, std::vector<float>& audio) {
        tap.clear();
        if (!built) build();
        if (n == 0) return;
        const Mode mode = cfg.mode;

        // 1. Shift the tuned signal to 0 Hz (planar output).
        ensureSize(a0, n);
        ensureSize(a1, n);
        mixer.process(iq, n, &a0[0], &a1[0]);

        // 2. Stage 1 decimation.
        float *r1 = &a0[0], *i1 = &a1[0];
        std::size_t n1 = n;
        if (d1 > 1) {
            const std::size_t m = stage1.maxOutput(n);
            ensureSize(b0, m);
            ensureSize(b1, m);
            n1 = stage1.process(r1, i1, n, &b0[0], &b1[0]);
            r1 = &b0[0];
            i1 = &b1[0];
        }
        if (n1 == 0) return;
        noise.feed(r1, i1, n1);

        // 3. Stage 2 (SSB/CW).
        std::size_t n2 = n1;
        if (d2 > 1) {
            const std::size_t m = stage2.maxOutput(n1);
            ensureSize(c0, m);
            ensureSize(c1, m);
            n2 = stage2.process(r1, i1, n1, &c0[0], &c1[0]);
            r1 = &c0[0];
            i1 = &c1[0];
        }

        // 4. Channel filter + level.
        const std::size_t mc = channel.maxOutput(n2);
        ensureSize(d0, mc);
        ensureSize(d1v, mc);
        const std::size_t nc = channel.process(r1, i1, n2, &d0[0], &d1v[0]);
        if (nc) {
            double p = 0;
            for (std::size_t i = 0; i < nc; ++i) p += d0[i] * d0[i] + d1v[i] * d1v[i];
            p /= nc;
            const double a = 1.0 - std::exp(-static_cast<double>(nc) / (rate2 * 0.05));
            chanPow = chanPow < 0 ? p : chanPow + a * (p - chanPow);
            chanDb = powerToDb(chanPow);
        }
        noiseDb = noise.noiseDb();
        const double sq = cfg.squelchDb;
        if (sq <= -199) sqOpen = true;
        else if (sqOpen) sqOpen = chanDb >= sq - 1.5;  // 1.5 dB hysteresis
        else sqOpen = chanDb >= sq;
        sqOpenAtomic = sqOpen;
        if (nc == 0) return;  // tiny block: nothing reached the demodulator yet

        // 5. Demodulate to audio at rateA (e0 = left/mono, e1 = right).
        std::size_t na = 0;
        bool twoCh = false;
        const float* dr = &d0[0];
        const float* di = &d1v[0];
        if (mode == Mode::WFM) {
            ensureSize(mpx, nc);
            ensureSize(diff, nc);
            discriminate(dr, di, nc, &mpx[0]);
            trackCarrier(&mpx[0], nc, rate2);
            rdsDec.process(&mpx[0], nc, rate1);
            pll.process(&mpx[0], &diff[0], nc);
            pilot = pll.locked;
            const std::size_t m = wfmAudio.maxOutput(nc);
            ensureSize(e0, m);
            ensureSize(e1, m);
            na = wfmAudio.process(&mpx[0], &diff[0], nc, &e0[0], &e1[0]);
            // Stereo matrix with a 50 ms cross-fade to/from mono.
            const float target = (cfg.stereo && pll.locked) ? 1.0f : 0.0f;
            const float step = static_cast<float>(1.0 / (0.05 * rateA));
            float blend = stereoBlend;
            for (std::size_t i = 0; i < na; ++i) {
                if (blend < target) blend = std::min(target, blend + step);
                else if (blend > target) blend = std::max(target, blend - step);
                const float m0 = e0[i], s0 = e1[i] * blend;
                e0[i] = deemL.step(m0 + s0);
                e1[i] = deemR.step(m0 - s0);
            }
            stereoBlend = blend;
            twoCh = true;
            publishRds();
        } else {
            ensureSize(e0, nc);
            float* out = &e0[0];
            na = nc;
            switch (mode) {
                case Mode::NFM:
                    discriminate(dr, di, nc, out);
                    trackCarrier(out, nc, rate2);
                    for (std::size_t i = 0; i < nc; ++i) out[i] -= nfmDc.step(out[i]);  // remove a frequency offset
                    break;
                case Mode::AM: {
                    if (amCarrier <= 0) {  // after a retune: start from this block's level
                        double m = 0;
                        for (std::size_t i = 0; i < nc; ++i) m += std::sqrt(dr[i] * dr[i] + di[i] * di[i]);
                        amCarrier = static_cast<float>(m / nc);
                    }
                    const float a = static_cast<float>(decayCoef(rateA, 0.5));
                    float car = amCarrier;
                    for (std::size_t i = 0; i < nc; ++i) {
                        const float env = std::sqrt(dr[i] * dr[i] + di[i] * di[i]);
                        car += a * (env - car);  // carrier level = DC of the envelope
                        if (cfg.agc) out[i] = car > 1e-9f ? env / car - 1.0f : 0.0f;
                        else out[i] = (env - car) * kManualGain;
                    }
                    amCarrier = car;
                    break;
                }
                case Mode::SAM:
                    sam.process(dr, di, nc, out, cfg.agc);
                    carrierHz = sam.fr * sam.rate / kTwoPi;
                    carrierValid = true;
                    break;
                case Mode::USB:
                case Mode::LSB:
                    // The channel filter left only one side of 0 Hz: its real
                    // part is the audio.
                    for (std::size_t i = 0; i < nc; ++i) out[i] = dr[i];
                    ssbHp.process(out, nc);
                    if (cfg.agc) agc.process(out, nc);
                    else for (std::size_t i = 0; i < nc; ++i) out[i] *= kManualGain;
                    break;
                case Mode::CW: {
                    // BFO: shift the carrier up to the pitch, take the real part.
                    const float* tab = sinTable();
                    std::uint32_t ph = bfoPhase;
                    for (std::size_t i = 0; i < nc; ++i) {
                        out[i] = dr[i] * tableCos(tab, ph) - di[i] * tableSin(tab, ph);
                        ph += bfoInc;
                    }
                    bfoPhase = ph;
                    if (cfg.agc) agc.process(out, nc);
                    else for (std::size_t i = 0; i < nc; ++i) out[i] *= kManualGain;
                    break;
                }
                default: break;
            }
        }

        // 6. Resample to the audio rate.
        const std::size_t mo = resampler.maxOutput(na);
        ensureSize(o0, mo);
        ensureSize(o1, mo);
        const std::size_t no = twoCh ? resampler.process(&e0[0], &e1[0], na, &o0[0], &o1[0])
                                     : resampler.process(&e0[0], 0, na, &o0[0], 0);

        // 6b. Decoder tap: mono, before squelch and volume (decoders keep
        //     working while the sound is muted or squelched).
        {
            const std::size_t t0 = tap.size();
            tap.resize(t0 + no);
            for (std::size_t i = 0; i < no; ++i) tap[t0 + i] = twoCh ? 0.5f * (o0[i] + o1[i]) : o0[i];
        }

        // 7. Squelch gate (5 ms ramp), volume, clip, interleave.
        const float vol = static_cast<float>(clampd(cfg.volume, 0, 2));
        const float target = sqOpen ? 1.0f : 0.0f, step = 1.0f / (0.005f * audioRate);
        float g = sqGain;
        const std::size_t base = audio.size();
        audio.resize(base + 2 * no);
        float* outp = &audio[base];
        const float* right = twoCh ? &o1[0] : &o0[0];
        for (std::size_t i = 0; i < no; ++i) {
            if (g < target) g = std::min(target, g + step);
            else if (g > target) g = std::max(target, g - step);
            const float k = g * vol;
            outp[2 * i] = std::max(-1.0f, std::min(1.0f, o0[i] * k));
            outp[2 * i + 1] = std::max(-1.0f, std::min(1.0f, right[i] * k));
        }
        sqGain = g;
    }

    void publishRds() {
        // Only copy strings when a new group arrived (no allocation per block).
        const unsigned changes = rdsDec.changes();
        if (changes == rdsChanges) return;
        rdsChanges = changes;
        std::lock_guard<std::mutex> lock(rdsMutex);
        rdsDec.fill(rdsSnap, cfg.deemphasisUs >= 62.5);  // 75 us = Americas = RBDS names
    }
};

// ============================================================================
//  Receiver (public API)
// ============================================================================
Receiver::Receiver(int audioRate) : impl_(new Impl(audioRate)) {}
Receiver::~Receiver() {}

void Receiver::configure(const ReceiverConfig& c) {
    Impl& s = *impl_;
    const ReceiverConfig old = s.cfg;
    s.cfg = c;
    if (!s.built) return;  // first process() builds everything
    const bool rebuild = c.sampleRate != old.sampleRate || c.mode != old.mode ||
                         s.effectiveBandwidth() != s.bw ||
                         (c.mode == Mode::CW && c.cwPitchHz != old.cwPitchHz);
    if (rebuild) {
        s.build();
        return;
    }
    if (c.offsetHz != old.offsetHz) {
        // Retune inside the same band: new mixer frequency, and the state
        // that belongs to the old signal starts again.
        s.mixer.setFrequency(-c.offsetHz, c.sampleRate);
        s.sam.reset();
        s.resetCarrier();
        s.amCarrier = 0;
        s.agc.reset();
        s.chanPow = -1;
        if (c.mode == Mode::WFM) {
            s.pll.reset();
            s.stereoBlend = 0;
            s.rdsDec.clear();
            s.rdsChanges = ~0u;
            std::lock_guard<std::mutex> lock(s.rdsMutex);
            s.rdsSnap = RdsInfo();
        }
    }
    if (c.deemphasisUs != old.deemphasisUs) s.setupDeemphasis();
    if (c.agc != old.agc) s.agc.reset();
}

const ReceiverConfig& Receiver::config() const { return impl_->cfg; }

void Receiver::process(const Complex* iq, std::size_t n, std::vector<float>& audio) { impl_->process(iq, n, audio); }
const std::vector<float>& Receiver::decoderAudio() const { return impl_->tap; }
double Receiver::carrierOffsetHz() const {
    if (!impl_->carrierValid) return std::numeric_limits<double>::quiet_NaN();
    const Mode m = impl_->cfg.mode;
    if (m != Mode::WFM && m != Mode::NFM && m != Mode::SAM) return std::numeric_limits<double>::quiet_NaN();
    return impl_->carrierHz;
}

double Receiver::channelDbfs() const { return impl_->chanDb; }
double Receiver::noiseDbfs() const { return impl_->noiseDb; }
bool Receiver::stereoPilot() const { return impl_->pilot; }
bool Receiver::squelchOpen() const { return impl_->sqOpenAtomic; }

RdsInfo Receiver::rds() const {
    std::lock_guard<std::mutex> lock(impl_->rdsMutex);
    return impl_->rdsSnap;
}

void Receiver::resetRds() {
    Impl& s = *impl_;
    s.rdsDec.clear();
    s.rdsChanges = ~0u;
    std::lock_guard<std::mutex> lock(s.rdsMutex);
    s.rdsSnap = RdsInfo();
}

}  // namespace dsp
