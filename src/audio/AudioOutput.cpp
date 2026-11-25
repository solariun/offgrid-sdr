// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/audio/AudioOutput.cpp
//  Layer:  Sound output
//
//  What this file does
//  -------------------
//  The platform-independent parts of audio::AudioOutput: the FrameRing used
//  by every implementation, and the null output (no sound card, --no-audio),
//  which consumes samples at the right speed on a clock so the rest of the
//  program behaves the same.
// ============================================================================
#include "AudioOutput.h"

#include <algorithm>
#include <chrono>

namespace audio {

FrameRing::FrameRing(std::size_t capacityFrames)
    : data_(capacityFrames * 2), capacity_(capacityFrames), read_(0), count_(0), overflows_(0) {}

void FrameRing::push(const float* stereo, std::size_t frames) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frames > capacity_) {  // more than fits: keep the newest
        stereo += (frames - capacity_) * 2;
        frames = capacity_;
    }
    if (count_ + frames > capacity_) {
        std::size_t drop = count_ + frames - capacity_;
        read_ = (read_ + drop) % capacity_;
        count_ -= drop;
        ++overflows_;
    }
    std::size_t write = (read_ + count_) % capacity_;
    for (std::size_t i = 0; i < frames; ++i) {
        data_[write * 2] = stereo[i * 2];
        data_[write * 2 + 1] = stereo[i * 2 + 1];
        write = write + 1 == capacity_ ? 0 : write + 1;
    }
    count_ += frames;
}

std::size_t FrameRing::pop(float* stereo, std::size_t frames) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t n = std::min(frames, count_);
    for (std::size_t i = 0; i < n; ++i) {
        stereo[i * 2] = data_[read_ * 2];
        stereo[i * 2 + 1] = data_[read_ * 2 + 1];
        read_ = read_ + 1 == capacity_ ? 0 : read_ + 1;
    }
    count_ -= n;
    return n;
}

std::size_t FrameRing::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return count_;
}

void FrameRing::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    read_ = count_ = 0;
}

unsigned FrameRing::overflows() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return overflows_;
}

namespace {

// Plays nothing; drains the queue in real time so queuedSeconds() behaves.
class NullOutput : public AudioOutput {
public:
    NullOutput() : rate_(48000), queued_(0), last_(std::chrono::steady_clock::now()) {}
    bool open(int rate, std::string&) {
        rate_ = rate > 0 ? rate : 48000;
        return true;
    }
    void close() {}
    int rate() const { return rate_; }
    std::string name() const { return "none"; }
    void write(const float*, std::size_t frames) {
        drain();
        queued_ = std::min(queued_ + static_cast<double>(frames) / rate_, 0.5);
    }
    double queuedSeconds() const {
        const_cast<NullOutput*>(this)->drain();
        return queued_;
    }
    unsigned underruns() const { return 0; }
    void flush() { queued_ = 0; }

private:
    void drain() {
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        queued_ = std::max(0.0, queued_ - std::chrono::duration<double>(now - last_).count());
        last_ = now;
    }
    int rate_;
    double queued_;
    std::chrono::steady_clock::time_point last_;
};

}  // namespace

std::unique_ptr<AudioOutput> AudioOutput::createNull() { return std::unique_ptr<AudioOutput>(new NullOutput()); }

}  // namespace audio
