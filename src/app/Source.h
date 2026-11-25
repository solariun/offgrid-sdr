// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/Source.h
//  Layer:  Application (where the IQ samples come from)
//
//  What this file does
//  -------------------
//  app::Source: a stream of unsigned 8-bit IQ bytes at a sample rate and a
//  centre frequency, from one of:
//    RtlSource    an RTL-SDR stick (src/rtlsdr, our own driver)
//    TcpSource    an rtl_tcp server on the network (host:port)
//    FileSource   a recording of raw u8 IQ (.cu8 / .iq / .bin; rtl_sdr's
//                 output), played in real time, looping
//    DemoSource   simulated stations (dsp::DemoSource), in real time
//  Every source delivers its bytes through a SampleQueue that the DSP
//  thread reads, so the radio code is the same for all of them.
// ============================================================================
#ifndef OFFGRID_APP_SOURCE_H
#define OFFGRID_APP_SOURCE_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace app {

// Blocks of bytes from the source thread to the DSP thread.  Bounded: when
// the DSP falls behind, the oldest blocks are dropped (and counted) rather
// than letting latency grow.
class SampleQueue {
public:
    explicit SampleQueue(std::size_t maxBlocks = 64);
    void push(const std::uint8_t* data, std::size_t length);
    // Waits up to timeoutMs for a block; false on timeout.
    bool pop(std::vector<std::uint8_t>& block, int timeoutMs);
    void clear();
    unsigned dropped() const { return dropped_; }
    std::size_t size() const;

private:
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<std::vector<std::uint8_t> > blocks_;
    std::vector<std::vector<std::uint8_t> > spare_;  // recycled buffers
    std::size_t maxBlocks_;
    std::atomic<unsigned> dropped_;
};

class Source {
public:
    virtual ~Source() {}

    virtual bool start(std::string& error) = 0;
    virtual void stop() = 0;

    virtual std::string name() const = 0;        // shown in the status bar
    virtual double sampleRate() const = 0;
    virtual bool setSampleRate(double hz) = 0;
    virtual double centerFrequency() const = 0;
    virtual bool setCenterFrequency(double hz) = 0;
    // Gain: dB >= 0, or < 0 for automatic.  gains(): the steps (may be empty).
    virtual bool setGain(double dB) = 0;
    virtual std::vector<double> gains() const = 0;
    virtual bool setPpm(double) { return false; }
    virtual bool setBiasTee(bool) { return false; }
    virtual bool setDirectSampling(int) { return false; }
    virtual bool tunable() const { return true; }  // files: no
    virtual std::string lastError() const { return std::string(); }

    SampleQueue& queue() { return queue_; }

protected:
    SampleQueue queue_;
};

// Factories.
std::unique_ptr<Source> makeRtlSource(int index);  // index into RtlSdr::find()
std::unique_ptr<Source> makeTcpSource(const std::string& hostPort);
std::unique_ptr<Source> makeFileSource(const std::string& path, double sampleRate, double centerHz);
std::unique_ptr<Source> makeDemoSource(double sampleRate, double centerHz);

}  // namespace app

#endif
