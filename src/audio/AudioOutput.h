// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/audio/AudioOutput.h
//  Layer:  Sound output
//
//  What this file does
//  -------------------
//  Declares audio::AudioOutput: plays interleaved stereo float samples.  The
//  receiver pushes audio with write() from its DSP thread; a playback thread
//  (or the OS callback) pulls it from a ring buffer, so a late block never
//  blocks the radio and a missing block becomes silence, not a stall.
//
//  How it fits in
//  --------------
//  Implementations, chosen at compile time:
//    AudioMac.cpp    macOS AudioQueue (AudioToolbox, part of the OS)
//    AudioLinux.cpp  Linux: the kernel's ALSA PCM interface directly (ioctl
//                    on /dev/snd/pcmC*D*p, no alsa-lib, so the binary can be
//                    fully static); when the device is busy (PipeWire /
//                    PulseAudio own it) it pipes the sound to pw-cat, paplay
//                    or aplay, whichever is installed.
//    AudioNull       (in AudioOutput.cpp) no sound: --no-audio
//  The application opens one and keeps the receiver's audio rate equal to
//  rate().
// ============================================================================
#ifndef OFFGRID_AUDIO_AUDIOOUTPUT_H
#define OFFGRID_AUDIO_AUDIOOUTPUT_H

#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace audio {

// Thread-safe ring buffer of interleaved stereo frames.
class FrameRing {
public:
    explicit FrameRing(std::size_t capacityFrames = 48000);
    // Appends frames (drops the oldest when full, counting an overflow).
    void push(const float* stereo, std::size_t frames);
    // Copies up to `frames` frames out; returns how many (the rest is up to
    // the caller, usually silence).
    std::size_t pop(float* stereo, std::size_t frames);
    std::size_t size() const;
    void clear();
    unsigned overflows() const;

private:
    mutable std::mutex mutex_;
    std::vector<float> data_;
    std::size_t capacity_, read_, count_;
    unsigned overflows_;
};

class AudioOutput {
public:
    virtual ~AudioOutput() {}

    // The best output for this system ("" = default device).  device: on
    // Linux "hw:CARD,DEV" or "pipe:<command>"; ignored on macOS.
    static std::unique_ptr<AudioOutput> create(const std::string& device = std::string());
    // An output that discards everything (keeps timing for --no-audio).
    static std::unique_ptr<AudioOutput> createNull();

    // Starts playback at about `rate` frames per second (stereo).  The
    // actual rate may differ (rate() tells).  false + error on failure.
    virtual bool open(int rate, std::string& error) = 0;
    virtual void close() = 0;
    virtual int rate() const = 0;
    virtual std::string name() const = 0;

    // Queues interleaved stereo frames (never blocks).
    virtual void write(const float* stereo, std::size_t frames) = 0;
    // Seconds of sound queued and not played yet.
    virtual double queuedSeconds() const = 0;
    // Times the output ran dry (gaps heard).
    virtual unsigned underruns() const = 0;
    // Drops what is queued (after retuning, so old sound isn't heard).
    virtual void flush() = 0;
};

}  // namespace audio

#endif
