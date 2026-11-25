// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/audio/AudioMac.cpp
//  Layer:  Sound output (macOS)
//
//  What this file does
//  -------------------
//  audio::AudioOutput on macOS with an AudioQueue (AudioToolbox framework,
//  part of the OS): 32-bit float stereo, 4 buffers of 1024 frames (~85 ms in
//  flight at 48 kHz).  The queue's callback refills each buffer from the
//  FrameRing; when the ring runs dry the rest of the buffer is silence and an
//  underrun is counted.
//
//  Contributing
//  ------------
//  Keep the callback short: it runs on the audio thread.  Buffers are always
//  re-enqueued with the same size (a changing size makes some devices
//  re-allocate and crackle).
// ============================================================================
#ifdef __APPLE__

#include <AudioToolbox/AudioToolbox.h>

#include <atomic>
#include <cstring>

#include "AudioOutput.h"

namespace audio {

namespace {

class MacOutput : public AudioOutput {
public:
    MacOutput() : ring_(48000), queue_(0), rate_(48000), underruns_(0), running_(false), primed_(false) {}
    ~MacOutput() { close(); }

    bool open(int rate, std::string& error) {
        close();
        rate_ = rate > 0 ? rate : 48000;
        AudioStreamBasicDescription f;
        std::memset(&f, 0, sizeof(f));
        f.mSampleRate = rate_;
        f.mFormatID = kAudioFormatLinearPCM;
        f.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
        f.mChannelsPerFrame = 2;
        f.mBitsPerChannel = 32;
        f.mBytesPerFrame = 8;
        f.mFramesPerPacket = 1;
        f.mBytesPerPacket = 8;
        OSStatus st = AudioQueueNewOutput(&f, &MacOutput::callback, this, 0, kCFRunLoopCommonModes, 0, &queue_);
        if (st != noErr) {
            error = "AudioQueueNewOutput failed (" + std::to_string(static_cast<long>(st)) + ")";
            queue_ = 0;
            return false;
        }
        for (int i = 0; i < kBuffers; ++i) {
            AudioQueueBufferRef b = 0;
            if (AudioQueueAllocateBuffer(queue_, kFrames * 8, &b) != noErr) {
                error = "AudioQueueAllocateBuffer failed";
                close();
                return false;
            }
            std::memset(b->mAudioData, 0, kFrames * 8);
            b->mAudioDataByteSize = kFrames * 8;
            AudioQueueEnqueueBuffer(queue_, b, 0, 0);
        }
        running_ = true;
        st = AudioQueueStart(queue_, 0);
        if (st != noErr) {
            error = "AudioQueueStart failed (" + std::to_string(static_cast<long>(st)) + ")";
            close();
            return false;
        }
        return true;
    }

    void close() {
        if (!queue_) return;
        running_ = false;
        AudioQueueStop(queue_, true);
        AudioQueueDispose(queue_, true);
        queue_ = 0;
        ring_.clear();
    }

    int rate() const { return rate_; }
    std::string name() const { return "CoreAudio"; }
    void write(const float* stereo, std::size_t frames) { ring_.push(stereo, frames); }
    double queuedSeconds() const { return static_cast<double>(ring_.size() + kFrames * kBuffers / 2) / rate_; }
    unsigned underruns() const { return underruns_; }
    void flush() {
        ring_.clear();
        primed_ = false;
    }

private:
    enum { kBuffers = 4, kFrames = 1024 };

    static void callback(void* user, AudioQueueRef queue, AudioQueueBufferRef buffer) {
        MacOutput* self = static_cast<MacOutput*>(user);
        float* out = static_cast<float*>(buffer->mAudioData);
        // Play only once two buffers' worth is queued (at the start, after a
        // flush, after running dry), so the start is silence, not gaps.
        std::size_t got = 0;
        if (!self->primed_ && self->ring_.size() >= 2 * kFrames) self->primed_ = true;
        if (self->primed_) {
            got = self->ring_.pop(out, kFrames);
            if (got < kFrames) {
                ++self->underruns_;
                self->primed_ = false;
            }
        }
        if (got < kFrames) std::memset(out + got * 2, 0, (kFrames - got) * 8);
        buffer->mAudioDataByteSize = kFrames * 8;
        if (self->running_) AudioQueueEnqueueBuffer(queue, buffer, 0, 0);
    }

    FrameRing ring_;
    AudioQueueRef queue_;
    int rate_;
    std::atomic<unsigned> underruns_;
    std::atomic<bool> running_;
    std::atomic<bool> primed_;
};

}  // namespace

std::unique_ptr<AudioOutput> AudioOutput::create(const std::string&) { return std::unique_ptr<AudioOutput>(new MacOutput()); }

}  // namespace audio

#endif
