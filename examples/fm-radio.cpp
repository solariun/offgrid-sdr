// ============================================================================
//  OFFgrid-SDR TUI - example: a complete FM radio in about 100 lines.
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   examples/fm-radio.cpp
//
//  What this file does
//  -------------------
//  Shows the libraries used without the offgrid program: liboffgrid-rtlsdr
//  (the stick), liboffgrid-dsp (WFM stereo + RDS) and liboffgrid-audio (the
//  speakers).  Plays a station and prints its RDS name and text.
//
//      make examples
//      bin/example-fm-radio 94.0          (MHz)
//
//  How it fits in
//  --------------
//  Everything here is public API: RtlSdr.h, Receiver.h, AudioOutput.h.  The
//  sample callback runs on the USB thread; it only queues the bytes, and
//  the main thread demodulates.
// ============================================================================
#include <signal.h>

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include "audio/AudioOutput.h"
#include "dsp/Receiver.h"
#include "rtlsdr/RtlSdr.h"

namespace {
volatile sig_atomic_t gStop = 0;
void onSignal(int) { gStop = 1; }
}  // namespace

int main(int argc, char** argv) {
    double mhz = argc > 1 ? std::atof(argv[1]) : 100.0;
    const double rate = 2048000, offset = 250000;  // tune 250 kHz below: no DC spike on the station
    signal(SIGINT, onSignal);

    std::vector<usb::DeviceInfo> sticks = rtlsdr::RtlSdr::find();
    if (sticks.empty()) {
        std::fprintf(stderr, "no RTL-SDR stick found\n");
        return 1;
    }
    rtlsdr::RtlSdr stick;
    if (!stick.open(sticks[0])) {
        std::fprintf(stderr, "%s\n", stick.error().c_str());
        return 1;
    }
    stick.setSampleRate(static_cast<std::uint32_t>(rate));
    stick.setCenterFrequency(static_cast<std::uint64_t>(mhz * 1e6 - offset));
    stick.setAutoGain();

    std::unique_ptr<audio::AudioOutput> speakers = audio::AudioOutput::create();
    std::string error;
    if (!speakers->open(48000, error)) {
        std::fprintf(stderr, "sound: %s\n", error.c_str());
        return 1;
    }
    dsp::Receiver radio(speakers->rate());
    dsp::ReceiverConfig config;
    config.sampleRate = stick.sampleRate();
    config.offsetHz = offset;
    config.mode = dsp::Mode::WFM;
    radio.configure(config);

    // USB thread -> main thread.
    std::mutex mutex;
    std::condition_variable ready;
    std::deque<std::vector<std::uint8_t> > blocks;
    stick.start([&](const std::uint8_t* data, std::size_t n) {
        std::lock_guard<std::mutex> lock(mutex);
        if (blocks.size() < 32) blocks.push_back(std::vector<std::uint8_t>(data, data + n));
        ready.notify_one();
    });

    std::printf("%.3f MHz on %s (Ctrl+C to stop)\n", mhz, stick.tunerName().c_str());
    std::vector<dsp::Complex> iq;
    std::vector<float> sound;
    std::string shown;
    while (!gStop) {
        std::vector<std::uint8_t> block;
        {
            std::unique_lock<std::mutex> lock(mutex);
            ready.wait_for(lock, std::chrono::milliseconds(100), [&] { return !blocks.empty(); });
            if (blocks.empty()) continue;
            block.swap(blocks.front());
            blocks.pop_front();
        }
        dsp::convertU8(&block[0], block.size(), iq);
        sound.clear();
        radio.process(&iq[0], iq.size(), sound);
        if (speakers->queuedSeconds() < 0.25) speakers->write(&sound[0], sound.size() / 2);

        dsp::RdsInfo rds = radio.rds();
        std::string line = rds.ps + "  " + rds.radioText;
        if (rds.groups > 0 && line != shown) {
            shown = line;
            std::printf("\r%-90s%s", line.c_str(), radio.stereoPilot() ? " [stereo]" : "");
            std::fflush(stdout);
        }
    }
    std::printf("\n");
    stick.stop();
    speakers->close();
    return 0;
}
