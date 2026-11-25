// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tools/rtl-probe.cpp
//  Layer:  Developer tool (not part of the receiver)
//
//  What this file does
//  -------------------
//  Smoke test for the in-software RTL-SDR driver: lists the sticks found,
//  opens the first one, prints its tuner, tunes 100 MHz at 2.048 MS/s,
//  streams for 2 seconds and reports the byte rate against the expected
//  one (2 bytes per sample), so dropped transfers show up as a shortfall.
//
//  How it fits in
//  --------------
//  Uses only rtlsdr::RtlSdr (src/rtlsdr) and the USB layer (src/usb).
//  Build on macOS:
//    c++ -std=c++11 -O2 -o rtl-probe tools/rtl-probe.cpp src/rtlsdr/*.cpp \
//        src/usb/UsbMac.cpp -framework IOKit -framework CoreFoundation
//  Build on Linux (static):
//    c++ -std=c++11 -O2 -static -pthread -o rtl-probe tools/rtl-probe.cpp \
//        src/rtlsdr/*.cpp src/usb/UsbLinux.cpp
//
//  Exit status: 0 = streamed fine, 1 = no stick / error, 2 = data shortfall.
// ============================================================================
#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

#include "../src/rtlsdr/RtlSdr.h"

namespace {

std::atomic<unsigned long long> gBytes(0);
std::atomic<unsigned long long> gTransfers(0);

void onSamples(const std::uint8_t* /*iq*/, std::size_t bytes) {
    gBytes += bytes;
    ++gTransfers;
}

}  // namespace

int main() {
    std::vector<usb::DeviceInfo> sticks = rtlsdr::RtlSdr::find();
    if (sticks.empty()) {
        std::printf("No RTL-SDR stick found.\n");
        return 1;
    }
    for (std::size_t i = 0; i < sticks.size(); ++i) {
        const usb::DeviceInfo& s = sticks[i];
        std::printf("[%zu] %04x:%04x  %s / %s  serial %s  at %s\n", i, s.vendorId, s.productId,
                    s.manufacturer.c_str(), s.product.c_str(), s.serial.c_str(), s.path.c_str());
    }

    rtlsdr::RtlSdr sdr;
    if (!sdr.open(sticks[0])) {
        std::printf("Open failed: %s\n", sdr.error().c_str());
        return 1;
    }
    std::printf("Tuner: %s%s\n", sdr.tunerName().c_str(), sdr.isBlogV4() ? " (Blog V4)" : "");

    const std::uint32_t rate = 2048000;
    if (!sdr.setSampleRate(rate) || !sdr.setCenterFrequency(100000000ULL) || !sdr.setAutoGain()) {
        std::printf("Setup failed: %s\n", sdr.error().c_str());
        return 1;
    }
    std::printf("Sample rate %u Hz, centre %llu Hz\n", sdr.sampleRate(),
                static_cast<unsigned long long>(sdr.centerFrequency()));

    const std::size_t transferBytes = 65536;
    if (!sdr.start(onSamples, transferBytes, 16)) {
        std::printf("Start failed: %s\n", sdr.error().c_str());
        return 1;
    }
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    sdr.stop();
    double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    double expected = 2.0 * sdr.sampleRate() * seconds;
    double got = static_cast<double>(gBytes.load());
    double lost = (expected - got) / transferBytes;
    std::printf("Received %llu bytes in %llu transfers over %.2f s: %.3f MB/s (expected %.3f MB/s)\n",
                gBytes.load(), gTransfers.load(), seconds, got / seconds / 1e6, expected / seconds / 1e6);
    // A couple of transfers are always still in flight when we stop.
    std::printf("Lost transfers (estimate): %.1f\n", lost > 0 ? lost : 0.0);
    sdr.close();
    return lost > 4 ? 2 : 0;
}
