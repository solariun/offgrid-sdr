// ============================================================================
//  offgrid-dsp - liboffgrid-dsp, the signal processing library of OFFgrid-SDR TUI.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr) by diddy-boy.
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/AudioFilters.h
//  Layer:  Library liboffgrid-dsp
//
//  What this file does
//  -------------------
//  OFFgrid-SDR's clean-up filters, ported from the browser app's JavaScript
//  (index.html) to C++11, one class per JS class:
//
//    dsp::DcBlocker       "DC spike removal": a one-pole high-pass on the raw
//                         IQ that removes the stick's DC offset (JS DcBlocker)
//    dsp::NoiseBlanker    impulse noise ("noise blanker (impulse noise)"):
//      dsp::WideBlanker     on the raw wideband IQ, threshold from the median
//                           sample power, blanks each spike plus a short tail
//      dsp::ChannelBlanker  on the slice around the tuned station (+-128 kHz),
//                           subtracts only that slice during a spike
//    dsp::SpectralNR      hiss ("spectral noise reduction"): STFT, minimum-
//                         statistics noise floor, Wiener-style gain
//    dsp::HumFilter       mains hum: notches at 50 or 60 Hz and 7 harmonics
//    dsp::AutoNotch       automatic notch: finds steady whistles every 0.1 s
//                         and notches up to three, following them
//    dsp::SmartNotch      the "smart notch": on request, finds steady whines
//                         in the last 3 s of audio; its notches are a list of
//                         frequencies the UI can show, add, move and remove
//    dsp::Limiter         "retune mute, fade-in and headphone limiter":
//                         look-ahead, volume-aware peak limiter, plus the
//                         mute-then-fade-in after a retune
//
//  The AI voice cleaner (RNNoise) is in VoiceCleaner.h.
//
//  How it fits in
//  --------------
//  dsp::Receiver (Receiver.cpp) runs them as OFFgrid-SDR does, when the
//  matching ReceiverConfig switch is on and only in the modes the JS allows:
//    raw IQ:  NoiseBlanker (AM, SAM, USB, LSB, CW), before the mixer
//    audio at the audio rate, mono, after the resampler, in the JS's order:
//             HumFilter (AM, SAM, USB, LSB, CW) -> AutoNotch -> SmartNotch
//             -> VoiceCleaner -> SpectralNR (these four: AM, SAM, USB, LSB)
//             -> retune mute/fade -> Limiter (every mode).
//  DcBlocker is not in the Receiver: like the JS, the application runs it on
//  the IQ before both the spectrum and the receiver.  Each class also works
//  on its own (the tests drive them directly).
//
//  Contributing
//  ------------
//  Keep each class a faithful port of its JS class: the comments quote the
//  JS's reasoning (and its "from 2.3" notes) because the numbers came from
//  testing on real signals.  Anything that is not in the JS is marked
//  "Addition to the JS".  No allocation in process(): buffers are sized at
//  construction or only ever grow.  Credits: the algorithms and tuning are
//  OFFgrid-SDR's (diddy-boy); this file is a translation.
// ============================================================================
#ifndef OFFGRID_DSP_AUDIOFILTERS_H
#define OFFGRID_DSP_AUDIOFILTERS_H

#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace dsp {

// ---- DC spike removal (JS DcBlocker) -----------------------------------------
// The RTL2832U's samples carry a small constant offset, which the FFT shows
// as a spike at the centre frequency.  A one-pole high-pass on I and Q
// (~25 ms time constant) tracks it and subtracts it; after a retune it
// starts from the new block's mean.
class DcBlocker {
public:
    explicit DcBlocker(double sampleRate = 2048000);
    void setSampleRate(double rate);
    void reset();
    // centerHz: the block's tuned centre frequency (a change = retuned).
    void process(std::complex<float>* iq, std::size_t n, double centerHz);

private:
    double a_, dcI_, dcQ_, freq_;
    bool haveFreq_;
};

// ---- Noise blanker (JS WideBlanker, ChannelBlanker, NoiseBlanker) -----------
struct BlankerStats {
    BlankerStats() : blanked(0), total(0), skipped(0) {}
    unsigned long long blanked;  // samples zeroed (wide) or touched (channel)
    unsigned long long total;    // samples seen
    unsigned long long skipped;  // blocks left alone (more than 5% "impulses")
};

class WideBlanker {
public:
    WideBlanker();
    void setSampleRate(double rate);
    void reset();
    // Addition to the JS: thresholdDb moves the threshold up (blanks less)
    // or down (blanks more) from the JS's (0 dB: 20 x median / ln 2).
    void setThresholdDb(double db);
    void process(std::complex<float>* iq, std::size_t n, BlankerStats* stats = 0);

private:
    int holdN_, back_, hold_;
    double thrScale_;
    std::vector<float> pw_;
    std::vector<int> pos_;
};

class ChannelBlanker {
public:
    ChannelBlanker();
    void setSampleRate(double rate);
    void reset();
    void setThresholdDb(double db);  // Addition to the JS, as WideBlanker
    // offsetHz: the tuned station minus the centre frequency.
    void process(std::complex<float>* iq, std::size_t n, double offsetHz, BlankerStats* stats = 0);

private:
    static const int kL = 8;  // moving-average length (3 stages)
    double rate_;
    int holdN_, hold_, k_;
    double ph_, lastOff_;
    bool haveOff_;
    double thrScale_;
    double v_[6][kL];  // stage 0..2 x (re, im) ring buffers
    double s_[6];
    std::vector<float> cre_, cim_, det_, pw_;
    std::vector<int> pos_;
};

class NoiseBlanker {  // the wideband stage first, then the channel slice
public:
    NoiseBlanker();
    void setSampleRate(double rate);
    void reset();
    void setThresholdDb(double db);
    void process(std::complex<float>* iq, std::size_t n, double offsetHz);
    const BlankerStats& stats() const { return stats_; }

private:
    WideBlanker w_;
    ChannelBlanker c_;
    BlankerStats stats_;
};

// ---- Spectral noise reduction (JS SpectralNR) --------------------------------
class SpectralNR {
public:
    explicit SpectralNR(int fftSize = 512);
    void reset();
    // Addition to the JS: the speech-clarity band (1.5-4.5 kHz, subtracted
    // only 30% as hard) is placed for this rate; the JS assumes 48 kHz.
    void setSampleRate(double rate);
    // level 1 (Low), 2 (Medium), 3 (High), as the JS's NR_LEVELS; in place.
    // The output is delayed by half an FFT (and is zero at the very start).
    void process(float* buf, std::size_t n, int level);

private:
    void processFrame(double alpha, double floor);
    int N_, H_, fill_;
    double rate_;
    std::vector<float> win_, frame_, re_, im_, ola_, ps_, noise_, gain_, aprof_, q_;
    std::vector<float> cos_, sin_;
    std::vector<int> rev_;
    unsigned qr_, qw_;
};

// ---- Hum filter (JS HumFilter) ---------------------------------------------------
class HumFilter {
public:
    HumFilter();
    // f0: 0 (off), 50 or 60 Hz; notches at f0 x 1..8, 6 to 20 Hz wide.
    void configure(double f0, double fs);
    void reset();
    void process(float* buf, std::size_t n);
    double mainsHz() const { return f0_; }

private:
    struct Section {
        double b0, b1, b2, a1, a2, z1, z2;
    };
    double f0_, fs_;
    std::vector<Section> sec_;
};

// ---- Automatic notch (JS AutoNotch) --------------------------------------------
class AutoNotch {
public:
    AutoNotch();
    void reset();  // a new station: start afresh
    // In place; the first call at a new rate resets (as the JS).
    void process(float* x, std::size_t n, double fs);
    // The frequencies (Hz) of the notches in use, for display.
    std::vector<double> notches() const;
    int count() const { return count_; }                      // (no allocation:
    double frequency(int i) const { return notches_[i].f; }  //  for the audio thread)

private:
    struct Notch {
        double f, b0, b1, b2, a1, a2, z1, z2;
    };
    struct Cand {  // a whistle candidate of one analysis
        double f, p;
        int k;
    };
    void analyse();
    void design(Notch& n, double f) const;
    static const int kN = 4096;
    double fs_;
    int pos_, sinceA_;
    std::vector<float> win_, ring_, re_, im_, L_, prom_;
    std::vector<double> cs_;
    std::vector<int> rev_;
    std::vector<Cand> cand_;  // sized once (no allocation per analysis)
    Notch notches_[3];
    int count_;
};

// ---- Smart notch (JS SmartNotch) --------------------------------------------------
class SmartNotch {
public:
    static const int kMaxNotches = 3;

    explicit SmartNotch(double fs = 48000);

    // A whine found by find().
    struct Whine {
        double f;     // Hz (between bins: parabolic interpolation)
        double prom;  // dB above its surroundings
    };

    // Records the audio (before notching: to find and show whines), then
    // applies the notches, in place.
    void process(float* x, std::size_t n, double fs);

    // ---- the notch list (what the UI shows as lines) ----
    std::vector<double> notches() const;
    int count() const { return count_; }
    double frequency(int i) const { return notches_[i].f; }
    int add(double f);            // a new notch (the oldest goes beyond 3); its index
    void move(int index, double f);  // clamped to 80..4500 Hz (as the JS)
    void remove(int index);       // (the JS's right-click on a line)
    void clear();                 // the JS's reset()
    double sampleRate() const { return fs_; }

    // ---- analysis (on a copy of the recorded audio: may run on another
    //      thread than process(), see Receiver) ----
    // The last n samples recorded (before notching), or after notching
    // with post = true (at most 24000 of those are kept).
    void recent(std::size_t n, std::vector<float>& out, bool post = false) const;

    // Whines in `x` (the last 3 s of audio): narrow peaks 12 dB over their
    // surroundings at the same pitch through 85% of the time.  Strongest
    // first, or nearest to `near` (> 0) first and only within 250 Hz of it.
    // Mains hum (a comb of 50/60 Hz) is left out; if it was all there was,
    // *mainsHum is set to 50 or 60 (else 0).
    static std::vector<Whine> find(const std::vector<float>& x, double fs, double near = 0, int* mainsHum = 0);

    // 50 or 60 if the last second (the end of x) has mains hum, else 0.
    static int mainsHum(const std::vector<float>& x, double fs);

    // The strip's spectrum (the JS's quick()): averaged 4096-point power
    // spectrum of x, in dB, for bins 0..top-1; returns the bin width in Hz.
    static double quickSpectrum(const std::vector<float>& x, double fs, int top, std::vector<float>& avgDb);

    // The JS's snSnap(): a notch dropped near a peak of the strip's spectrum
    // (within 40 Hz, 8 dB over the bins 6 away) moves onto that peak.
    static double snap(double f, const std::vector<float>& avgDb, double bin, int top);

private:
    struct Notch {
        double f, b0, b1, b2, a1, a2, x1, x2, y1, y2;
    };
    void coeffs(Notch& n) const;
    double fs_;
    std::vector<float> ring_, post_;
    std::size_t pos_, ppos_;
    Notch notches_[kMaxNotches];
    int count_;
};

// ---- Retune mute, fade-in and headphone limiter (JS gate + makeLimiter) ----
class Limiter {
public:
    explicit Limiter(double sampleRate = 48000);
    // Addition to the JS: the look-ahead (96 samples = 2 ms) and the times
    // are kept in seconds at other rates.
    void setSampleRate(double rate);
    void reset();

    // Peak limiter, look-ahead within the block and volume-aware: only acts
    // where |sample| would pass `ceiling` (the JS passes 0.95 / volume, so it
    // only acts on what would really reach full scale).  The gain eases down
    // ~2 ms ahead of a peak and recovers over ~0.3 s; it never lets a sample
    // past the ceiling.  right may be null or equal to left (mono).
    void process(float* left, float* right, std::size_t n, float ceiling);
    bool active() const { return active_; }
    float gain() const { return g_; }

    // Retune mute: silence for muteSeconds, then a 100 ms fade-in (the JS's
    // armRetuneMute; 0 s = nothing).  gate() applies it.
    void armRetune(double muteSeconds);
    void gate(float* left, float* right, std::size_t n);
    bool gating() const { return muteLeft_ > 0 || fade_ < fadeN_; }

private:
    double rate_;
    int look_;
    float atk_, rel_, g_;
    bool active_;
    std::vector<float> need_, m_;
    long muteLeft_, fade_, fadeN_;
};

}  // namespace dsp

#endif
