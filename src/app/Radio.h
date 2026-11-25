// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/Radio.h
//  Layer:  Application (the receiver engine)
//
//  What this file does
//  -------------------
//  app::Radio ties a Source, the DSP (dsp::Receiver, dsp::SpectrumAnalyzer)
//  and an audio::AudioOutput together, on one DSP thread:
//
//      Source --u8 IQ--> SampleQueue --> [DSP thread] --+--> SpectrumAnalyzer --> spectrum frames
//                                                       +--> Receiver --> AudioOutput
//
//  It owns the tuning logic: the dial frequency is anywhere inside the
//  stick's span; the stick's centre only moves when the dial leaves the
//  middle 80% of the span (then it is put a quarter span away from the
//  dial, keeping the DC spike off the signal).  Everything the user
//  interface reads (spectrum, levels, RDS, status) is copied under a mutex
//  into a Snapshot, so the UI thread never touches the DSP's state.
// ============================================================================
#ifndef OFFGRID_APP_RADIO_H
#define OFFGRID_APP_RADIO_H

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../audio/AudioOutput.h"
#include "../decoders/Decoder.h"
#include "../decoders/Heard.h"
#include "../dsp/AutoMode.h"
#include "../dsp/Receiver.h"
#include "../dsp/SignalMonitor.h"
#include "Source.h"

namespace app {

struct RadioSettings {
    RadioSettings();
    double frequency;     // dial, Hz
    dsp::Mode mode;
    double bandwidth;     // Hz (0 = mode default)
    double volume;        // 0..2
    double squelchDb;     // dBFS, -200 = off
    double gainDb;        // < 0 = automatic
    double ppm;
    bool stereo;
    double deemphasisUs;
    bool biasTee;
    bool muted;
    int fftSize;
    int directSampling;   // 0 off, 1 I, 2 Q
    // Decoder modes (as the browser version): `frequency` is the signal's;
    // the receiver is tuned this much lower so its tones land at the
    // decoder's audio centre (RTTY/NAVTEX/DSC 1500 Hz, PSK/HELL 1000 Hz).
    double decoderShift;
    // Audio clean-up filters (dsp::Receiver; voice modes AM/SAM/USB/LSB, the
    // blanker and hum filter also CW, the limiter all modes).
    bool noiseBlanker;
    int noiseReduction;   // 0 off, 1 low, 2 medium, 3 high
    int humFilterHz;      // 0, 50, 60
    bool autoNotch;
    int voiceCleaner;     // AI voice cleaner (RNNoise): 0 off, 1 light, 2 medium, 3 strong, 4 full
    bool limiter;         // peak limiter + a short mute after a retune
};

// What the user interface shows, copied out of the DSP thread.
struct Snapshot {
    Snapshot();
    std::vector<float> spectrum;  // dBFS, full span, DC in the middle
    unsigned spectrumSerial;      // increments with every new frame
    double centerHz, sampleRate;
    double channelDbfs, noiseDbfs;
    bool stereoPilot, squelchOpen;
    dsp::RdsInfo rds;
    double audioQueued;           // seconds
    unsigned audioUnderruns, samplesDropped;
    unsigned decoderBlocksDropped;  // blocks the decoder fell too far behind to see
    double dspLoad;               // DSP thread busy fraction (0..1 of one core)
    double sampleRateMeasured;    // samples/s actually arriving
    bool overload;                // IQ samples hitting the rails (lower the gain)
    double carrierHz;             // measured carrier offset from the dial (NaN = none)
    float voiceProbability;       // AI voice cleaner: speech probability (0..1)
    bool decoderOn;
    dec::Kind decoderKind;
    dec::Status decoder;          // the decoder's status (when on)
};

class Radio {
public:
    Radio(std::unique_ptr<Source> source, std::unique_ptr<audio::AudioOutput> audio);
    ~Radio();

    bool start(const RadioSettings& settings, std::string& error);
    void stop();

    // ---- controls (any thread)
    void tune(double hz);                 // dial; may move the stick's centre
    void setMode(dsp::Mode mode);         // also resets the bandwidth to the mode default
    void setBandwidth(double hz);
    void setVolume(double v);
    void setSquelch(double dB);
    void setGain(double dB);              // < 0 = auto
    void setPpm(double ppm);
    void setStereo(bool on);
    void setDeemphasis(double us);
    void setBiasTee(bool on);
    void setMuted(bool on);
    void setFftSize(int n);
    // The clean-up filter fields of `f` (noiseBlanker ... limiter).
    void setFilters(const RadioSettings& f);

    // Text / picture decoder on the demodulated sound (liboffgrid-decoders).
    // setup (optional) runs on the new decoder before its first sample, on the
    // DSP thread: decoder-specific settings (AX.25 speed, time station ...).
    void setDecoder(bool on, dec::Kind kind = dec::Kind::Cw, std::function<void(dec::Decoder&)> setup = std::function<void(dec::Decoder&)>());
    void setDecoderShift(double hz);
    // The clock correction in use (PC - true seconds), passed to slot-based
    // decoders (FT8, FT4, WSPR) with every block.
    void setClockOffset(double pcMinusTrue);
    // Your Maidenhead locator (distances and bearings: FT8/FT4/WSPR, the
    // Heard list, AIS, APRS).  "" = unknown.
    void setLocator(const std::string& grid);
    // FT8 / FT4 / WSPR: a decoded slot's timings for the clock correction
    // (ClockCorrection::applyFtxTimings); false when there is none new.
    bool takeFtxTimings(std::string& proto, std::vector<double>& dts);
    // Runs `command` on the decoder, on the decoder's thread (e.g. choosing
    // a DAB service).  Ignored when no decoder is on.
    void decoderCommand(std::function<void(dec::Decoder&)> command);
    // Multi-service decoders (DAB, DRM): the services on offer (refreshed
    // about twice a second by the decoder thread).
    std::vector<dec::Service> decoderServices() const;
    // Whole pictures from the decoder (DAB/DRM slideshow) since the last call.
    void takeDecoderImages(std::vector<dec::Decoder::EncodedImage>& out);
    // Centre the stick exactly on the dial (DAB: the 1.5 MHz ensemble must
    // fit in the span) instead of a quarter span away.
    void setCenterOnDial(bool on);
    // An HF upconverter in front of the stick (Ham It Up etc.): frequencies
    // below 65 MHz are tuned at frequency + offset Hz.  0 = none.
    void setUpconverter(double offsetHz);
    // The Heard summary (every FT8 / FT4 / WSPR decode, band by band) over
    // the last windowSeconds, as text lines.
    std::string heardText(double now, double windowSeconds, bool details = false) const;
    void resetDecoder();
    // New decoded text / Hell columns since the last call (UI thread).
    void takeDecoderText(std::string& out);
    void takeDecoderColumns(std::vector<std::vector<float> >& out);
    void takeDecoderLines(std::vector<dec::PictureLine>& out);  // FAX, SSTV

    // AUTO: listen near the dial and work out what the signal is
    // (dsp::AutoDetector).  takeAutoResult gives the answer once.
    void startAuto();
    bool autoListening() const;
    bool takeAutoResult(dsp::AutoResult& out);

    // The Signal Monitor (dsp::SignalMonitor): up to 8 signals measured all
    // the time from the stick's samples.
    int monitorAdd(double centreHz, double widthHz, const std::string& mode, const std::string& name);
    void monitorRemove(int index);
    int monitorCount() const;
    dsp::MonitorStats monitorStats(int index) const;
    double monitorCentre(int index) const;
    double monitorWidth(int index) const;
    std::string monitorName(int index) const;
    std::string monitorCsv(double unixAtZero) const;
    void monitorFollow(int index, bool on);
    bool monitorFollowing(int index) const;

    // The last seconds of the stick's raw samples (u8 IQ), kept in memory:
    //  - saveIq writes them as a .cu8 file (play it with --file);
    //  - startReplay plays them again through a second receiver that follows
    //    the dial and mode (tune around in the past), instead of the live
    //    sound, until the end or stopReplay.
    static const int kHistorySeconds = 20;
    bool saveIq(const std::string& path, std::string& error);
    bool startReplay(std::string& error);
    void stopReplay();
    bool replaying() const { return replaying_; }
    double replayPosition() const;  // seconds into the replay

    // Records the demodulated sound (16-bit stereo WAV at the audio rate).
    bool startRecording(const std::string& path, std::string& error);
    void stopRecording();
    bool recording() const;
    double recordedSeconds() const;

    RadioSettings settings() const;
    void snapshot(Snapshot& out) const;
    std::string sourceName() const;
    std::string audioName() const;
    std::vector<double> gains() const;
    bool tunable() const;

private:
    void loop();
    void applyReceiver();  // call with mutex_ held
    double offset() const;

    std::unique_ptr<Source> source_;
    std::unique_ptr<audio::AudioOutput> audio_;
    std::unique_ptr<dsp::Receiver> receiver_;  // made in start() at the audio rate
    dsp::SpectrumAnalyzer spectrum_;

    mutable std::mutex mutex_;  // guards settings_, snap_, pending flags
    RadioSettings settings_;
    Snapshot snap_;
    bool receiverDirty_, fftDirty_, flushAudio_;
    double center_;

    std::atomic<bool> running_;
    std::thread thread_;

    // Decoder: owned by its own thread (decoderLoop), so a heavy decode
    // (FT8 / WSPR slots, ~1 s of CPU) never stalls the radio or the sound.
    // The DSP thread queues each block's decoder input (DecoderJob).
    struct DecoderJob {
        std::vector<float> audio;          // receiver tap (audio decoders)
        std::vector<dsp::Complex> iq;      // the stick's samples (IQ decoders)
        double audioRate, iqRate, timestamp, clockOffset, dial, center;
    };
    void decoderLoop();
    std::thread decoderThread_;
    std::mutex jobMutex_;
    std::condition_variable jobReady_;
    std::deque<DecoderJob> jobs_;
    unsigned jobsDropped_;
    std::atomic<bool> decoderActive_;      // the DSP thread queues jobs only when on
    std::atomic<bool> decoderIq_;          // ... with IQ instead of audio
    dec::Status decStatus_;                // guarded by mutex_
    bool decoderOn_;                       // guarded by mutex_
    std::unique_ptr<dec::Decoder> decoder_;  // decoder thread only
    bool decoderWanted_, decoderDirty_, decoderReset_;
    dec::Kind decoderKind_;
    std::function<void(dec::Decoder&)> decoderSetup_;
    double clockOffset_;
    std::string locator_;
    std::deque<std::function<void(dec::Decoder&)> > commands_;   // mutex_
    std::vector<dec::Service> services_;                         // mutex_
    std::vector<dec::Decoder::EncodedImage> images_;             // mutex_
    std::atomic<bool> decoderAudio_;   // the decoder's sound replaces the receiver's
    // The decoder's sound waiting for the sound card (decoder thread only):
    // DRM decodes in 1.2 s windows, DAB in 24-120 ms frames, so it is
    // buffered first and then kept topped up.
    std::deque<float> decFifo_;        // stereo pairs at the sound card's rate
    const dec::Decoder* decFifoOwner_;
    bool decPlaying_;
    bool centerOnDial_;
    double upconverter_;
    double hardwareFrequency(double dial) const;  // with the upconverter
    dec::HeardList heard_;                         // guarded by mutex_
    std::string ftxProto_;                         // pending slot timings (mutex_)
    std::vector<double> ftxDts_;
    bool ftxPending_;
    std::string decText_;                          // waiting for the UI
    std::vector<std::vector<float> > decColumns_;  // waiting for the UI
    std::vector<dec::PictureLine> decLines_;       // waiting for the UI

    // AUTO and the monitor (guarded by analysisMutex_, used by the DSP thread).
    mutable std::mutex analysisMutex_;
    dsp::AutoDetector auto_;
    bool autoListening_, autoReady_;
    dsp::SignalMonitor monitor_;

    // IQ history (ring of raw bytes) and replay.
    std::vector<std::uint8_t> history_;   // ring, guarded by historyMutex_
    size_t historyPos_, historyFill_;
    double historyCenter_, historyRate_;
    mutable std::mutex historyMutex_;
    void replayLoop(std::vector<std::uint8_t> bytes, double center, double rate);
    std::thread replayThread_;
    std::atomic<bool> replaying_, replayStop_;
    std::atomic<double> replayPos_;

    // WAV recording (guarded by mutex_; written by the DSP thread).
    void writeWav(const std::vector<float>& audio);
    void finishWav();
    FILE* wav_;
    unsigned long wavFrames_;
};

}  // namespace app

#endif
