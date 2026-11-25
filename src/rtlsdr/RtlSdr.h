// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/rtlsdr/RtlSdr.h
//  Layer:  RTL-SDR driver (in software, no librtlsdr)
//
//  What this file does
//  -------------------
//  Declares rtlsdr::RtlSdr: an RTL2832U USB demodulator chip with an R820T,
//  R820T2, R828D (RTL-SDR Blog V4) tuner, driven over usb::Device.  It opens
//  the stick, sets sample rate, frequency, gain, frequency correction,
//  direct sampling and bias-tee, and streams 8-bit unsigned IQ samples.
//
//  How it fits in
//  --------------
//  Ported from @jtarrio/webrtlsdr (Apache-2.0, Jacobo Tarrio Barreiro), the
//  driver OFFgrid-SDR's browser version uses, which itself follows librtlsdr.
//    Rtl2832u.cpp  the demodulator chip: registers, I2C repeater, sample
//                  rate resampler, IF frequency, streaming.
//    R82xx.cpp     the Rafael Micro tuners (PLL, filters, gain tables).
//  The application (src/app) owns one RtlSdr and feeds its samples to the
//  DSP receiver (src/dsp).
// ============================================================================
#ifndef OFFGRID_RTLSDR_RTLSDR_H
#define OFFGRID_RTLSDR_RTLSDR_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "../usb/UsbDevice.h"

namespace rtlsdr {

// Called with raw interleaved unsigned 8-bit I,Q bytes (on the USB thread).
typedef std::function<void(const std::uint8_t* iq, std::size_t bytes)> SampleCallback;

class RtlSdr {
public:
    RtlSdr();
    ~RtlSdr();

    // RTL2832U sticks on the bus (known vendor/product ids).
    static std::vector<usb::DeviceInfo> find();

    // Opens and initialises the stick.  false + error() on failure.
    bool open(const usb::DeviceInfo& info);
    void close();
    bool isOpen() const;

    std::string tunerName() const;   // "R820T", "R828D", ...
    bool isBlogV4() const;           // RTL-SDR Blog V4 (R828D + HF input)
    const std::string& error() const { return error_; }

    // 225001-300000 Hz or 900001-3200000 Hz.  Returns the exact rate set.
    bool setSampleRate(std::uint32_t hz);
    std::uint32_t sampleRate() const;

    // Centre frequency in Hz (with frequency correction applied).
    bool setCenterFrequency(std::uint64_t hz);
    std::uint64_t centerFrequency() const;

    bool setFrequencyCorrection(double ppm);
    double frequencyCorrection() const;

    // Tuner gain in dB (nearest supported step), or automatic (AGC) gain.
    std::vector<double> gains() const;
    bool setGain(double dB);
    bool setAutoGain();

    // 0 = off, 1 = I branch, 2 = Q branch (Q is the usual HF direct input).
    bool setDirectSampling(int mode);
    bool setBiasTee(bool on);

    // Streams samples until stop().  bufferBytes: bytes per USB transfer
    // (multiple of 512).
    bool start(SampleCallback callback, std::size_t bufferBytes = 65536, int transfers = 16);
    void stop();

private:
    RtlSdr(const RtlSdr&);
    RtlSdr& operator=(const RtlSdr&);

    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::string error_;
};

}  // namespace rtlsdr

#endif
