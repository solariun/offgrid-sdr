// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/Receiver.h
//  Layer:  Signal processing
//
//  What this file does
//  -------------------
//  The public face of the DSP library, used by the application:
//    dsp::Receiver          IQ at the stick's rate -> stereo audio at 48 kHz,
//                           for one tuned signal (offset from the centre):
//                           WFM (stereo + RDS), NFM, AM, SAM, USB, LSB, CW.
//    dsp::SpectrumAnalyzer  IQ -> power spectrum in dB (windowed FFT,
//                           averaged), for the spectrum and waterfall.
//    dsp::DemoSource        synthetic IQ with stations of every mode (no
//                           hardware needed: --demo, and the tests).
//    dsp::RdsInfo           what the RDS decoder found.
//
//  How it fits in
//  --------------
//  The application converts the stick's unsigned 8-bit bytes with
//  dsp::convertU8() and passes the same block to SpectrumAnalyzer::push()
//  and Receiver::process(), both on its DSP thread.
// ============================================================================
#ifndef OFFGRID_DSP_RECEIVER_H
#define OFFGRID_DSP_RECEIVER_H

#include <complex>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace dsp {

typedef std::complex<float> Complex;

// Unsigned 8-bit IQ bytes (RTL-SDR) -> complex samples in -1..1.
void convertU8(const std::uint8_t* bytes, std::size_t count, std::vector<Complex>& out);

enum class Mode { WFM, NFM, AM, SAM, USB, LSB, CW };
const char* modeName(Mode mode);              // "WFM", "NFM", ...
bool parseMode(const std::string& name, Mode& mode);
double defaultBandwidth(Mode mode);           // Hz: WFM 150k, NFM 12.5k, AM 10k, SAM 10k, SSB 2.8k, CW 500

struct ReceiverConfig {
    ReceiverConfig();
    double sampleRate;   // input IQ rate, Hz (default 2048000)
    double offsetHz;     // tuned frequency minus the centre frequency
    Mode mode;           // default WFM
    double bandwidthHz;  // channel filter width (0 = defaultBandwidth(mode))
    bool stereo;         // WFM: decode stereo when the pilot is present (default true)
    double deemphasisUs; // WFM: 50 (Europe, default) or 75 (Americas)
    double squelchDb;    // mute below this channel level in dBFS (-200 = off, default)
    double cwPitchHz;    // CW: tone of the received carrier (default 700)
    bool agc;            // automatic audio level for AM/SAM/SSB/CW (default true)
    double volume;       // 0..2 linear (default 1)
};

struct RdsInfo {
    RdsInfo();
    std::uint16_t pi;       // programme identification (0 = none yet)
    std::string ps;         // programme service name (8 chars, spaces = unknown)
    std::string radioText;  // up to 64 chars
    int pty;                // programme type 0..31 (-1 unknown)
    std::string ptyName;
    bool tp, ta, music;
    std::string clock;      // "YYYY-MM-DD HH:MM" from group 4A (empty = none)
    unsigned groups;        // valid groups decoded so far
};

class Receiver {
public:
    explicit Receiver(int audioRate = 48000);
    ~Receiver();

    // May be called at any time between process() calls; rebuilds filters
    // only when something relevant changed.
    void configure(const ReceiverConfig& config);
    const ReceiverConfig& config() const;

    // Demodulates `n` IQ samples, appending interleaved stereo float audio
    // (L, R, L, R ... in -1..1) at audioRate to `audio`.
    void process(const Complex* iq, std::size_t n, std::vector<float>& audio);

    // The demodulated sound of the last process() call as mono at audioRate,
    // before squelch and volume (with the AGC): what text decoders listen to.
    const std::vector<float>& decoderAudio() const;

    // Where the received carrier really is, in Hz from the tuned frequency
    // (WFM/NFM: the mean FM discriminator output since the tune, after
    // 0.5 s of settling, so it gets more precise the longer it listens; SAM: the
    // carrier PLL).  NaN in other modes or before any signal.  Used for
    // frequency calibration: on a station whose frequency is known exactly,
    // the offset is the stick's error.
    double carrierOffsetHz() const;

    // Measurements (updated by process()).
    double channelDbfs() const;    // power in the channel filter, dBFS
    double noiseDbfs() const;      // estimate of the noise floor near it
    bool stereoPilot() const;      // WFM: 19 kHz pilot detected
    bool squelchOpen() const;
    RdsInfo rds() const;           // WFM only (copy, thread-safe enough for display)
    void resetRds();

private:
    Receiver(const Receiver&);
    Receiver& operator=(const Receiver&);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class SpectrumAnalyzer {
public:
    // size: FFT length (power of two, e.g. 2048).  averaging: 0..1 weight
    // of the previous frame (exponential average, 0 = none).
    explicit SpectrumAnalyzer(int size = 2048, double averaging = 0.5);
    ~SpectrumAnalyzer();

    void setSize(int size);
    void setAveraging(double averaging);
    int size() const;

    // Feeds samples; returns true when a new frame is ready (at most
    // framesPerSecond of them per second of input at sampleRate).
    bool push(const Complex* iq, std::size_t n, double sampleRate, double framesPerSecond = 30);

    // The latest frame: size() bins in dBFS, from -sampleRate/2 (index 0)
    // to +sampleRate/2 (DC in the middle, fftshifted).
    const std::vector<float>& powerDb() const;

private:
    SpectrumAnalyzer(const SpectrumAnalyzer&);
    SpectrumAnalyzer& operator=(const SpectrumAnalyzer&);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// A simulated band: a few stations around the centre frequency.
struct DemoStation {
    double offsetHz;   // from the centre
    Mode mode;
    std::string name;  // also the RDS PS for WFM stations
    double levelDb;    // carrier level, dBFS
};

class DemoSource {
public:
    // centerHz decides which demo band is generated:
    //   87.5-108 MHz: WFM stations (stereo, with RDS PS and RadioText)
    //   otherwise: AM, SAM-able AM, USB/LSB voice-like signals, CW Morse and
    //   NFM, spread over +-900 kHz.
    explicit DemoSource(double sampleRate = 2048000, double centerHz = 100000000);
    ~DemoSource();

    void setCenter(double centerHz);
    const std::vector<DemoStation>& stations() const;

    // Adds a station that sends `audio` (mono, at audioRate, looped) as an
    // upper-sideband signal whose suppressed carrier is at `hz` (a USB
    // receiver tuned to `hz` hears the audio), or, with mode NFM, as narrow
    // FM on a carrier at `hz` (5 kHz deviation per unit of audio).  Used for decoder signals
    // (RTTY, PSK, NAVTEX ...) made by the program.  levelDb: peak level, dBFS.
    // wallClockStart (optional): the computer's time (Unix seconds) of the
    // audio's first sample; the station then plays in step with the clock
    // (each rebuild starts it at the sample for "now"), e.g. time signals.
    void addAudioStation(double hz, const std::vector<float>& audio, double audioRate, double levelDb, Mode mode,
                         const std::string& name, double wallClockStart = 0);

    // Next `n` samples (with noise), as unsigned 8-bit IQ like a stick, so
    // the whole chain is exercised.
    void generate(std::size_t n, std::vector<std::uint8_t>& bytes);

private:
    DemoSource(const DemoSource&);
    DemoSource& operator=(const DemoSource&);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dsp

#endif
