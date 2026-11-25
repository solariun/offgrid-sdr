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
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../audio/AudioOutput.h"
#include "../decoders/Decoder.h"
#include "../dsp/Receiver.h"
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
    double dspLoad;               // DSP thread busy fraction (0..1 of one core)
    double sampleRateMeasured;    // samples/s actually arriving
    bool overload;                // IQ samples hitting the rails (lower the gain)
    double carrierHz;             // measured carrier offset from the dial (NaN = none)
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

    // Text / picture decoder on the demodulated sound (liboffgrid-decoders).
    void setDecoder(bool on, dec::Kind kind = dec::Kind::Cw);
    void setDecoderShift(double hz);
    void resetDecoder();
    // New decoded text / Hell columns since the last call (UI thread).
    void takeDecoderText(std::string& out);
    void takeDecoderColumns(std::vector<std::vector<float> >& out);

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

    // Decoder: created / replaced on the DSP thread when decoderDirty_.
    std::unique_ptr<dec::Decoder> decoder_;
    bool decoderWanted_, decoderDirty_, decoderReset_;
    dec::Kind decoderKind_;
    std::string decText_;                          // waiting for the UI
    std::vector<std::vector<float> > decColumns_;  // waiting for the UI

    // WAV recording (guarded by mutex_; written by the DSP thread).
    void writeWav(const std::vector<float>& audio);
    void finishWav();
    FILE* wav_;
    unsigned long wavFrames_;
};

}  // namespace app

#endif
