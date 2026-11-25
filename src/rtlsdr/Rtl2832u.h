// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/rtlsdr/Rtl2832u.h   (internal to src/rtlsdr)
//  Layer:  RTL-SDR driver (in software, no librtlsdr)
//
//  What this file does
//  -------------------
//  Declares rtlsdr::detail::Rtl2832u, the Realtek RTL2832U demodulator chip
//  as seen over USB vendor control transfers:
//    * register access to its four blocks (USB, SYS, demodulator, I2C),
//    * the I2C repeater that connects the tuner chip to the bus,
//    * GPIO pins (bias-tee, Blog V4 upconverter switch),
//    * chip initialisation for SDR mode, the sample-rate resampler, the
//      sampling-frequency (ppm) offset, the digital IF mixer, the digital
//      AGC, direct-sampling routing and the sample FIFO reset.
//
//  How it fits in
//  --------------
//  Ported from webrtlsdr's rtlcom.ts and rtl2832u.ts (@jtarrio/webrtlsdr,
//  Apache-2.0, Copyright 2024 Jacobo Tarrio Barreiro, Copyright 2013 Google
//  Inc.), which follow librtlsdr (GPL-2.0+, Steve Markgraf, Dimitri
//  Stolnikov, Hoernchen and others).  R82xx.cpp talks to the tuner through
//  the I2C functions here; RtlSdr.cpp orchestrates both.
//
//  Errors: every USB failure throws detail::IoError (after a few retries,
//  like the original).  RtlSdr's public methods catch it and turn it into
//  `false` + error().  Exceptions never cross the public API.
//
//  Contributing
//  ------------
//  Register addresses come with comments copied/condensed from the
//  reference; keep them when changing values, they are the only docs most
//  people will ever have for this chip.
// ============================================================================
#ifndef OFFGRID_RTLSDR_RTL2832U_H
#define OFFGRID_RTLSDR_RTL2832U_H

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "../usb/UsbDevice.h"

namespace rtlsdr {
namespace detail {

// A USB transfer to the stick failed (after retries).
class IoError : public std::runtime_error {
public:
    explicit IoError(const std::string& what) : std::runtime_error(what) {}
};

class Rtl2832u {
public:
    // Frequency of the RTL2832U's reference crystal (also feeds the tuner).
    static const std::uint32_t kXtalFrequency = 28800000;
    // Bulk IN endpoint carrying the samples.
    static const std::uint8_t kSampleEndpoint = 0x81;

    explicit Rtl2832u(usb::Device& device) : device_(device) {}

    // ---- Register access (rtlcom.ts) ------------------------------------
    // USB block register (little-endian value of `length` bytes).
    void setUsbReg(std::uint16_t address, std::uint32_t value, int length);
    // 8051 SYS block register (one byte).
    void setSysReg(std::uint16_t address, std::uint8_t value);
    std::uint8_t getSysReg(std::uint16_t address);
    // Demodulator register (big-endian value of `length` bytes), followed
    // by the dummy read librtlsdr also does to let the write settle.
    void setDemodReg(std::uint8_t page, std::uint16_t address, std::uint32_t value, int length);

    // I2C (through the repeater; call openI2C() first).
    std::uint8_t getI2CReg(std::uint8_t i2cAddress, std::uint8_t reg);
    void setI2CReg(std::uint8_t i2cAddress, std::uint8_t reg, std::uint8_t value);
    std::vector<std::uint8_t> getI2CRegBuffer(std::uint8_t i2cAddress, std::uint8_t reg, std::size_t length);
    void setI2CRegBuffer(std::uint8_t i2cAddress, std::uint8_t reg, const std::uint8_t* values, std::size_t length);

    // GPIO: make pin an output, then drive it.
    void setGpioOutput(int gpio);
    void setGpioBit(int gpio, bool high);

    // I2C repeater: connects the tuner to the I2C bus while open.
    void openI2C();
    void closeI2C();

    // ---- Demodulator operations (rtl2832u.ts) ---------------------------
    // Brings the chip up in SDR mode (no tuner yet).
    void init();
    // Effective crystal frequency for a given correction.
    static std::uint32_t xtalFrequency(double ppm);
    // Programs the resampler; returns the exact sample rate obtained.
    std::uint32_t setSampleRate(std::uint32_t rate, double ppm);
    // Programs the sampling-frequency offset registers for `ppm`.
    void setSampleFrequencyCorrection(double ppm);
    // Programs the digital IF mixer; returns the exact IF obtained.
    std::int64_t setIfFrequency(std::int64_t ifFrequency, double ppm);
    // Enables/disables the RTL2832U's own digital AGC (used only in direct
    // sampling mode, where the tuner is bypassed).
    void enableRtlAgc(bool enable);
    // Routes the I (mode 1) or Q (mode 2) ADC to the datapath, low-IF mode.
    void enterDirectSampling(int mode);
    // Back to tuner (zero-IF off, inverted spectrum, default datapath).
    void leaveDirectSampling();
    // Soft reset of the demodulator.
    void resetDemodulator();
    // Clears the sample FIFO; do it before streaming.
    void resetBuffer();

private:
    // Raw vendor control transfers with the original's stall retry policy.
    void writeControl(std::uint16_t value, std::uint16_t index, const std::uint8_t* data, std::uint16_t length);
    void readControl(std::uint16_t value, std::uint16_t index, std::uint8_t* data, std::uint16_t length);

    usb::Device& device_;
};

}  // namespace detail
}  // namespace rtlsdr

#endif
