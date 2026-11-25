// ============================================================================
//  offgrid-dsp - reusable C++11 radio signal processing (part of OFFgrid-SDR
//  TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/SignalMonitor.h
//  Layer:  Library liboffgrid-dsp (public header)
//
//  What this file does
//  -------------------
//  The Signal Monitor of OFFgrid-SDR 2.4 (diddy-boy, index.html: "Signal
//  Monitor engine", "Signal Monitor: deep analysis of one signal" and the
//  page around them), without any user interface:
//
//    dsp::ChannelExtractor  one band of the stick's IQ mixed down to 0 Hz and
//                           decimated (an 8-sample average, then a
//                           Blackman-windowed sinc low-pass), streaming and
//                           allocation-free: what the monitor's per-signal
//                           speakers, recordings and the analysis capture are
//                           built on, and what AUTO listens through.
//    dsp::SignalMonitor     up to 8 signals (bands given by centre and width)
//                           measured 10 times a second from a fine spectrum
//                           of their own (about 100 Hz bins at any zoom):
//                           level (average, peak), noise, S/N, occupied
//                           bandwidth, transmissions (count, average and
//                           longest, active and silent time, duty cycle, the
//                           interval "every N s" and whether it is regular),
//                           fades, frequency and drift (now in Hz/s, over 10
//                           minutes in Hz/min, the total shift), Follow (the
//                           band moves with a drifting signal), and the
//                           transmission log as CSV text.
//    dsp::SignalAnalyzer    "works out a signal's modulation in about 6
//                           seconds": FSK (shift, baud, the standard modes
//                           RTTY / NAVTEX / DSC / AFSK), PSK31/63/125, Morse
//                           (WPM), on-off keying, FM, AM, a plain carrier,
//                           SSB voice.
//
//  How it fits in
//  --------------
//  Pure engines on numbers.  The application feeds SignalMonitor::feedIq()
//  with every IQ block (the raw samples, before any demodulator shift) and
//  shows stats(); for SDRconnect's "Lite" mode (no samples, only a
//  spectrum) it calls step() with that spectrum instead.  An analysis is
//  SignalAnalyzer::start() + feedIq() until ready(), then result().  A
//  per-signal speaker is a ChannelExtractor (to ~200 kHz or less) feeding
//  a dsp::Receiver of its own.  Times are seconds on the caller's clock
//  (feedIq: the samples' own clock, so it is deterministic in tests).
//
//  Contributing
//  ------------
//  Strict C++11, no allocation per sample.  The thresholds (6 dB on for
//  0.2 s, 3 dB off for 0.6 s, fades 10 dB / 6 dB ...) are the JS's: keep
//  them in step with index.html and say so when they differ ("Addition to
//  the JS").  Ported from OFFgrid-SDR by diddy-boy.
// ============================================================================
#ifndef OFFGRID_DSP_SIGNALMONITOR_H
#define OFFGRID_DSP_SIGNALMONITOR_H

#include <complex>
#include <cstddef>
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace dsp {

typedef std::complex<float> Complex;
namespace detail {
class Fft;
// In-place radix-2 complex FFT in double (n a power of two), the JS's
// monFft / fftC: X[k] = sum x[n] exp(-j 2 pi k n / N).  Shared with
// AutoMode.cpp; internal (not part of the stable API).
void fftDouble(double* re, double* im, int n);
}

// ---------------------------------------------------------------------------
// ChannelExtractor: mix a band to 0 Hz and decimate (the JS's fcalFeed and
// monCapFeed / monSpkFeed collectors, as one reusable block)
// ---------------------------------------------------------------------------
class ChannelExtractor {
public:
    ChannelExtractor();
    // inputRate: the IQ rate; offsetHz: the band's centre minus the IQ's
    // centre; decimation: integer factor (>= 1).  Two stages, as the JS: an
    // average of 8 samples when the factor is a multiple of 8 and at least
    // 16 (its nulls fall where signals would fold back), then a windowed
    // sinc low-pass (8 * rest + 1 taps, cut-off 0.42 of the output's
    // Nyquist... i.e. 0.42 / rest cycles a sample) keeping every rest-th.
    // (Addition to the JS: the output is scaled to unity gain; the JS's
    // collector kept the 8-sample sum.)
    void configure(double inputRate, double offsetHz, int decimation);
    // max(1, round(inputRate / wantRate)).
    static int decimationFor(double inputRate, double wantRate);
    // Moves the band without resetting the filter (phase continuous).
    void setOffset(double offsetHz);
    void reset();  // clears the filter and the oscillator
    double inputRate() const { return inRate_; }
    double outputRate() const { return inRate_ / (D1_ * D2_); }
    double offsetHz() const { return offset_; }
    int decimation() const { return D1_ * D2_; }
    // Processes n samples; appends the outputs to `out` (no allocation once
    // `out` has the capacity).  Returns how many were appended.
    std::size_t process(const Complex* in, std::size_t n, std::vector<Complex>& out);

private:
    double inRate_, offset_;
    int D1_, D2_, T_;
    double pr_, pi_, sr_, si_;  // oscillator phasor and its step
    double ar_, ai_;            // stage-1 sums
    int cnt_, ph2_, hp_;
    std::vector<double> h_, rr_, ri_;
};

// ---------------------------------------------------------------------------
// SignalMonitor
// ---------------------------------------------------------------------------
// One transmission: from a to b (seconds), its peak S/N (dB).
struct MonitorTx {
    double a, b, peakSnr;
};
struct MonitorFade {
    double a, b, deep;
};

// What the panel shows for one signal (the JS's monStats).  NaN = "none yet"
// (the JS's null).
struct MonitorStats {
    bool on;            // transmitting now
    bool away;          // out of the radio's view (paused, not counted)
    double level;       // dBFS, smoothed over 0.25 s
    double noise;       // dBFS in the band's width
    double snr;         // dB, smoothed
    int nTx;            // transmissions so far
    double duty;        // 0..1: share of the watched time it was on (occupancy)
    double levelAvg;    // dBFS: mean power while on (NaN before any)
    double levelPeak;   // dBFS: highest smoothed level while on
    double active;      // seconds on
    double silent;      // seconds off
    double longest;     // seconds: the longest transmission
    double txAvg;       // seconds: mean transmission length (NaN: none)
    double txNow;       // seconds of the current one (NaN when off)
    double interval;    // seconds between starts ("Every"), NaN: < 2 starts
    bool regular;       // >= 3 intervals and their spread < 15 % of the mean
    int nFade;          // fades (not counted in USB, LSB, CW)
    double fadeDeep;    // dB: the deepest
    bool fadesOk;       // fades are counted in this mode
    double fadeRate;    // fades a minute while on (NaN under 30 s on)
    double obw;         // Hz: occupied bandwidth (99 %), NaN before any
    double obwRes;      // Hz: its resolution (4 bins)
    double fc;          // Hz: the power-weighted centre (NaN before any)
    double fpk;         // Hz: the peak, to a fraction of a bin
    double driftNow;    // Hz/s over the last 20 s (0: no clear trend; NaN: too few)
    double driftPerMin; // Hz/min over the last 10 min (0: none; NaN: too few)
    double shift;       // Hz: fc now minus the first fc seen (NaN before)
    double watched;     // seconds in view
};

// The engine's own spectrum frame (or SDRconnect's, in Lite mode).
struct MonitorFrame {
    MonitorFrame() : db(0), N(0), binHz(0), centreHz(0), perBin(0), w2(1) {}
    const float* db;    // dB a bin, natural FFT order (DC at index 0); 10*log10(|X|^2/N^2)
    int N;              // bins (power of two)
    double binHz;       // rate / N
    double centreHz;    // the frequency of bin 0 (the stick's centre)
    double perBin;      // the noise in one bin, linear (median / ln 2)
    double w2;          // the window's mean square (1 for no window)
};

class SignalMonitor {
public:
    static const int kMaxSignals = 8;
    // The thresholds (the JS's MON_*).
    static const double kOnDb, kOffDb, kOnSeconds, kOffSeconds, kFadeDb, kFadeEndDb;

    SignalMonitor();

    // Adds a signal; returns its index or -1 (8 already).  width >= 50 Hz
    // (default 3000 as the JS).  mode: the receiver's mode name when it was
    // added ("USB", "LSB" and "CW" have no fades: their level follows the
    // voice or the keying).
    int add(double centreHz, double widthHz, const std::string& mode = "", const std::string& name = "",
            bool follow = false);
    void remove(int index);
    void clear();
    int count() const { return static_cast<int>(sigs_.size()); }
    // Starts a signal's measurements afresh (the JS's monFresh).
    void resetStats(int index);

    void setBand(int index, double centreHz, double widthHz);  // a drag on the spectrum
    void setFollow(int index, bool on);
    void setMode(int index, const std::string& mode);
    void setName(int index, const std::string& name);
    double centreHz(int index) const;  // moves with Follow
    double widthHz(int index) const;
    bool follow(int index) const;
    const std::string& mode(int index) const;
    const std::string& name(int index) const;

    // Feeds IQ (the raw samples, `centreHz` = the stick's centre).  Keeps the
    // last N samples (N = the power of two near rate/100, 4096..65536: about
    // 100 Hz bins) and measures every 0.1 s of samples.  Time is the samples'
    // own clock (seconds fed so far), unless setClock() was given.
    void feedIq(const Complex* iq, std::size_t n, double sampleRate, double centreHz);
    // Addition to the JS: the time feedIq() stamps measurements with (seconds).
    double now() const { return clock_; }
    void setClock(double seconds) { clock_ = seconds; }
    // One look at a spectrum, at time t (seconds): what feedIq does 10 times a
    // second; call it directly with an outside spectrum (SDRconnect Lite).
    void step(const MonitorFrame& frame, double t);
    // After a pause (radio off, Panorama, a scan, paused waterfall): the
    // paused time is left out (the JS's monPauseCheck on resuming).
    void resumeAfterGap(double gapSeconds);

    MonitorStats stats(int index) const;
    // The signal's finished transmissions (oldest first, at most 200) and fades.
    const std::deque<MonitorTx>& transmissions(int index) const;
    const std::deque<MonitorFade>& fades(int index) const;

    // The transmission log, CSV (CRLF lines) as the JS's monCsv: signal, name,
    // frequency_hz, mode, start_utc, end_utc, duration_s, peak_snr_db; one
    // under way has no end.  unixAtZero: the Unix time of t = 0.
    std::string csv(double unixAtZero) const;

    // ISO 8601 UTC "YYYY-MM-DDTHH:MM:SS(.sss)Z" (Addition: exposed for the app).
    static std::string isoUtc(double unixSeconds);

private:
    struct Sig;
    void stepOne(Sig& s, double t, const MonitorFrame& fr);
    MonitorStats statsOf(const Sig& s, double t) const;

    std::vector<Sig*> sigs_;
    // own spectrum
    std::vector<Complex> ring_;
    std::size_t ringPos_, ringFill_;
    std::vector<Complex> work_;
    std::vector<float> win_, db_, smp_;
    std::vector<double> ex_;
    int N_;
    double w2_, rate_, sinceFrame_, clock_, lastCentre_;
    detail::Fft* fft_;

public:
    ~SignalMonitor();

private:
    SignalMonitor(const SignalMonitor&);
    SignalMonitor& operator=(const SignalMonitor&);
};

// ---------------------------------------------------------------------------
// SignalAnalyzer: deep analysis of one signal (the JS's monAnalyse)
// ---------------------------------------------------------------------------
struct AnalysisPeak {
    double f, db;
};
struct AnalysisResult {
    AnalysisResult();
    // kind: "short", "none", "FSK", "PSK", "CW", "OOK", "FM", "AM", "CARRIER",
    // "SSB", "UNKNOWN".  name: e.g. "RTTY (amateur, 45.45 baud)", "PSK31",
    // "CW (Morse)", "AM".  sure: "likely", "possibly" or "".
    std::string kind, name, sure, text;
    double fs, secs;
    double obw, lo, hi, centre, bandDb;  // Hz from the band's centre; dB
    int snr;                              // the strongest tone over the floor, dB
    std::vector<AnalysisPeak> peaks;      // up to 6 tones (Hz from the band's centre, dB)
    // envelope: swing (dB), share of time low, coefficient of variation (2 ms and 0.25 ms)
    double swingDb, offFrac, cv, cvFast;
    double toneCorr;                      // FSK tones' correlation (NaN: not measured)
    double reversals;                     // PSK: share of brief dips that reversed phase
    int revDips;
    // per kind (NaN when not applicable)
    double shift, baud, wpm, tone, carrier, deviation, fskLo, fskHi;
    bool stop15;                          // RTTY 1.5 stop bits detected
    std::vector<std::string> standards;   // the standard modes matched (FSK)
};

class SignalAnalyzer {
public:
    SignalAnalyzer();
    // Starts a capture of `seconds` (JS: 6) of the band centred on
    // bandCentreHz, bwHz wide (40 kHz at most), from IQ at inputRate: mixed
    // down and averaged by D = floor(rate / max(4000, 2.5 bw)) (the JS's
    // boxcar).  Returns false if the band is too wide.
    bool start(double bandCentreHz, double bwHz, double inputRate, double seconds = 6);
    void cancel();
    bool capturing() const { return active_ && !full_; }
    // Feeds IQ (centreHz = the stick's centre).  A retune restarts the capture
    // (as the JS).  Returns true when the capture is complete.
    bool feedIq(const Complex* iq, std::size_t n, double centreHz);
    bool ready() const { return full_; }
    double progress() const;  // 0..1
    // Runs the analysis on the finished capture (a few hundred ms for a wide
    // band: the app runs it off its DSP thread).
    AnalysisResult result() const;

    // The analysis itself, on any capture: zr, zi at fs with the band's centre
    // at 0 Hz, bw the band's width (Hz).  The capture should be wider than
    // the band (about 2.5x) so the noise beside it can be measured.
    static AnalysisResult analyse(const std::vector<float>& zr, const std::vector<float>& zi, double fs, double bw);

private:
    bool active_, full_;
    double f_, bw_, rate_, fs_, hw_;
    int D_, k_;
    std::size_t n_, N_;
    double ph_, ar_, ai_;
    std::vector<float> zr_, zi_;
};

}  // namespace dsp

#endif
