// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/rtlsdr/Rtl2832u.cpp   (internal to src/rtlsdr)
//  Layer:  RTL-SDR driver (in software, no librtlsdr)
//
//  What this file does
//  -------------------
//  The RTL2832U demodulator: vendor control transfers to its register
//  blocks, I2C to the tuner, GPIOs, SDR-mode initialisation, resampler,
//  ppm offset, IF mixer, AGC, direct sampling routing and FIFO reset.
//
//  How it fits in
//  --------------
//  Port of webrtlsdr's rtlcom.ts + the register work in rtl2832u.ts
//  (@jtarrio/webrtlsdr, Apache-2.0, Copyright 2024 Jacobo Tarrio Barreiro,
//  Copyright 2013 Google Inc.), itself derived from librtlsdr.  Values and
//  ordering are kept identical to the reference.
//
//  The USB protocol, in one paragraph
//  ----------------------------------
//  Everything is a vendor control transfer with bRequest = 0.  wValue holds
//  the register address and wIndex selects the block:
//      block 0x0100 USB registers      (USB_SYSCTL, USB_EPA_CTL, ...)
//      block 0x0200 SYS registers      (demod power, GPIO)
//      block 0x0600 I2C bus            (wValue = 8-bit I2C address)
//      demodulator page p: wIndex = p, wValue = (reg << 8) | 0x20
//  Writes OR 0x10 into wIndex.  Multi-byte values: USB block little-endian,
//  demodulator big-endian.
//
//  Contributing
//  ------------
//  Keep this file chip-only; tuner logic goes to R82xx.cpp, policy (when to
//  enter direct sampling, what to re-apply after a ppm change) to RtlSdr.cpp.
// ============================================================================
#include "Rtl2832u.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace rtlsdr {
namespace detail {

const std::uint32_t Rtl2832u::kXtalFrequency;
const std::uint8_t Rtl2832u::kSampleEndpoint;

namespace {

const std::uint16_t kWriteFlag = 0x10;     // OR'ed into wIndex for writes
const std::uint16_t kBlockUsb = 0x0100;
const std::uint16_t kBlockSys = 0x0200;
const std::uint16_t kBlockI2C = 0x0600;
const unsigned kControlTimeoutMs = 300;   // librtlsdr's CTRL_TIMEOUT
const int kRetries = 5;                    // as in rtlcom.ts (on stall)

void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

std::string hex(unsigned value) {
    char buffer[16];
    std::snprintf(buffer, sizeof buffer, "0x%x", value);
    return buffer;
}

}  // namespace

// ---------------------------------------------------------------------------
//  Raw control transfers
// ---------------------------------------------------------------------------

void Rtl2832u::writeControl(std::uint16_t value, std::uint16_t index, const std::uint8_t* data, std::uint16_t length) {
    // usb::Device::control() takes a non-const buffer (it is used for both
    // directions); copy so callers can pass const data.
    std::uint8_t buffer[16];
    std::vector<std::uint8_t> big;
    std::uint8_t* p = buffer;
    if (length > sizeof buffer) {
        big.assign(data, data + length);
        p = &big[0];
    } else {
        for (std::uint16_t i = 0; i < length; ++i) buffer[i] = data[i];
    }
    // The original retries a stalled transfer up to 5 times, waiting
    // 5, 10, 15, 20, 25 ms.  Our USB layer does not tell stalls apart from
    // other errors, so any failure is retried the same way.
    for (int attempt = 0;; ++attempt) {
        int n = device_.control(0x40 /* vendor, host->device */, 0, value, index, p, length, kControlTimeoutMs);
        if (n == static_cast<int>(length)) return;
        if (attempt >= kRetries) break;
        sleepMs((attempt + 1) * 5);
    }
    throw IoError("USB write failed value=" + hex(value) + " index=" + hex(index) + ": " + device_.lastError());
}

void Rtl2832u::readControl(std::uint16_t value, std::uint16_t index, std::uint8_t* data, std::uint16_t length) {
    for (int attempt = 0;; ++attempt) {
        int n = device_.control(0xc0 /* vendor, device->host */, 0, value, index, data, length, kControlTimeoutMs);
        if (n == static_cast<int>(length)) return;
        if (attempt >= kRetries) break;
        sleepMs((attempt + 1) * 5);
    }
    throw IoError("USB read failed value=" + hex(value) + " index=" + hex(index) + ": " + device_.lastError());
}

// ---------------------------------------------------------------------------
//  Register blocks
// ---------------------------------------------------------------------------

void Rtl2832u::setUsbReg(std::uint16_t address, std::uint32_t value, int length) {
    std::uint8_t data[4];
    for (int i = 0; i < length; ++i) data[i] = static_cast<std::uint8_t>(value >> (8 * i));  // little-endian
    writeControl(address, kBlockUsb | kWriteFlag, data, static_cast<std::uint16_t>(length));
}

void Rtl2832u::setSysReg(std::uint16_t address, std::uint8_t value) {
    writeControl(address, kBlockSys | kWriteFlag, &value, 1);
}

std::uint8_t Rtl2832u::getSysReg(std::uint16_t address) {
    std::uint8_t value = 0;
    readControl(address, kBlockSys, &value, 1);
    return value;
}

void Rtl2832u::setDemodReg(std::uint8_t page, std::uint16_t address, std::uint32_t value, int length) {
    std::uint8_t data[4];
    for (int i = 0; i < length; ++i) {
        data[i] = static_cast<std::uint8_t>(value >> (8 * (length - 1 - i)));  // big-endian
    }
    writeControl(static_cast<std::uint16_t>((address << 8) | 0x20), static_cast<std::uint16_t>(page | kWriteFlag), data,
                 static_cast<std::uint16_t>(length));
    // Dummy read of demod page 0x0a register 0x01, as librtlsdr/webrtlsdr do
    // after every demodulator write.
    std::uint8_t dummy = 0;
    readControl(0x0120, 0x0a, &dummy, 1);
}

std::uint8_t Rtl2832u::getI2CReg(std::uint8_t i2cAddress, std::uint8_t reg) {
    writeControl(i2cAddress, kBlockI2C | kWriteFlag, &reg, 1);
    std::uint8_t value = 0;
    readControl(i2cAddress, kBlockI2C, &value, 1);
    return value;
}

void Rtl2832u::setI2CReg(std::uint8_t i2cAddress, std::uint8_t reg, std::uint8_t value) {
    std::uint8_t data[2] = {reg, value};
    writeControl(i2cAddress, kBlockI2C | kWriteFlag, data, 2);
}

std::vector<std::uint8_t> Rtl2832u::getI2CRegBuffer(std::uint8_t i2cAddress, std::uint8_t reg, std::size_t length) {
    writeControl(i2cAddress, kBlockI2C | kWriteFlag, &reg, 1);
    std::vector<std::uint8_t> data(length);
    if (length) readControl(i2cAddress, kBlockI2C, &data[0], static_cast<std::uint16_t>(length));
    return data;
}

void Rtl2832u::setI2CRegBuffer(std::uint8_t i2cAddress, std::uint8_t reg, const std::uint8_t* values,
                               std::size_t length) {
    // The I2C bridge takes at most 8 bytes per transfer: the register
    // number plus 7 values.  Larger blocks are split, advancing the
    // register number.
    std::uint8_t payload[8];
    for (std::size_t i = 0; i < length; i += 7) {
        std::size_t chunk = length - i < 7 ? length - i : 7;
        payload[0] = static_cast<std::uint8_t>(reg + i);
        for (std::size_t k = 0; k < chunk; ++k) payload[1 + k] = values[i + k];
        writeControl(i2cAddress, kBlockI2C | kWriteFlag, payload, static_cast<std::uint16_t>(chunk + 1));
    }
}

void Rtl2832u::setGpioOutput(int gpio) {
    std::uint8_t bit = static_cast<std::uint8_t>(1u << gpio);
    // SYS_GPIO_DIR (0x3004): clear bit = output
    std::uint8_t r = getSysReg(0x3004);
    setSysReg(0x3004, static_cast<std::uint8_t>(r & ~bit));
    // SYS_GPIO_OUT_EN (0x3003): set bit = output driver enabled
    r = getSysReg(0x3003);
    setSysReg(0x3003, static_cast<std::uint8_t>(r | bit));
}

void Rtl2832u::setGpioBit(int gpio, bool high) {
    std::uint8_t bit = static_cast<std::uint8_t>(1u << gpio);
    // SYS_GPIO_OUT_VAL (0x3001)
    std::uint8_t r = getSysReg(0x3001);
    r = high ? static_cast<std::uint8_t>(r | bit) : static_cast<std::uint8_t>(r & ~bit);
    setSysReg(0x3001, r);
}

void Rtl2832u::openI2C() {
    // Demod page 1, reg 0x01: set [3] IIC_REPEAT (0x18)
    setDemodReg(1, 0x01, 0x18, 1);
}

void Rtl2832u::closeI2C() {
    // Demod page 1, reg 0x01: clear [3] IIC_REPEAT (0x10)
    setDemodReg(1, 0x01, 0x10, 1);
}

// ---------------------------------------------------------------------------
//  Demodulator operations
// ---------------------------------------------------------------------------

void Rtl2832u::init() {
    // USB_SYSCTL (0x2000): [0] DMA enable, [3] full packet mode
    setUsbReg(0x2000, 0x09, 1);
    // USB_EPA_MAXPKT (0x2158): 512-byte packets (high-speed bulk)
    setUsbReg(0x2158, 0x0200, 2);
    // USB_EPA_CTL (0x2148): [4] stall endpoint, [9] FIFO reset
    setUsbReg(0x2148, 0x0210, 2);
    // SYS_DEMOD_CTL1 (0x300b): power on demodulator analog circuitry/clock
    setSysReg(0x300b, 0x22);
    // SYS_DEMOD_CTL (0x3000): [7] PLL, [6] ADC_I, [5] release reset, [3] ADC_Q
    setSysReg(0x3000, 0xe8);
    // Page 1 reg 0x01: soft reset (assert [2], then release)
    setDemodReg(1, 0x01, 0x14, 1);
    setDemodReg(1, 0x01, 0x10, 1);
    // Page 1 reg 0x15: no spectrum inversion, no adjacent channel rejection
    setDemodReg(1, 0x15, 0x00, 1);
    // Page 1 regs 0x16-0x18: carrier frequency offset = 0
    setDemodReg(1, 0x16, 0x00, 1);
    setDemodReg(1, 0x17, 0x00, 1);
    setDemodReg(1, 0x18, 0x00, 1);
    // Page 1 regs 0x19-0x1b: IF frequency = 0
    setDemodReg(1, 0x19, 0x00, 1);
    setDemodReg(1, 0x1a, 0x00, 1);
    setDemodReg(1, 0x1b, 0x00, 1);
    // Page 1 regs 0x1c-0x2f: decimation FIR coefficients (8 x 8-bit taps +
    // 8 x 12-bit packed taps = 20 bytes), librtlsdr's fir_default.
    static const std::uint8_t kFir[20] = {0xca, 0xdc, 0xd7, 0xd8, 0xe0, 0xf2, 0x0e, 0x35, 0x06, 0x50,
                                          0x9c, 0x0d, 0x71, 0x11, 0x14, 0x71, 0x74, 0x19, 0x41, 0xa5};
    for (int i = 0; i < 20; ++i) setDemodReg(1, static_cast<std::uint16_t>(0x1c + i), kFir[i], 1);
    // Page 0 reg 0x19: [2] SDR mode, [5] digital AGC off
    setDemodReg(0, 0x19, 0x05, 1);
    // Page 1 regs 0x93-0x94: FSM state-holding registers
    setDemodReg(1, 0x93, 0xf0, 1);
    setDemodReg(1, 0x94, 0x0f, 1);
    // Page 1 reg 0x11: DAGC off
    setDemodReg(1, 0x11, 0x00, 1);
    // Page 1 reg 0x04: RF/IF AGC loop off
    setDemodReg(1, 0x04, 0x00, 1);
    // Page 0 reg 0x61: pass error packets, PID filter off
    setDemodReg(0, 0x61, 0x60, 1);
    // Page 0 reg 0x06: default ADC datapath
    setDemodReg(0, 0x06, 0x80, 1);
    // Page 1 reg 0xb1: zero-IF input, DC estimation, IQ compensation/estimation
    setDemodReg(1, 0xb1, 0x1b, 1);
    // Page 0 reg 0x0d: disable 4.096 MHz clock output on TP_CK0
    setDemodReg(0, 0x0d, 0x83, 1);
}

std::uint32_t Rtl2832u::xtalFrequency(double ppm) {
    return static_cast<std::uint32_t>(std::floor(kXtalFrequency * (1.0 + ppm / 1000000.0)));
}

std::uint32_t Rtl2832u::setSampleRate(std::uint32_t rate, double ppm) {
    // ratio = floor(xtal * 2^22 / rate), low two bits cleared (RSAMP_RATIO[27:2])
    const std::uint64_t scaled = static_cast<std::uint64_t>(xtalFrequency(ppm)) << 22;
    std::uint32_t ratio = static_cast<std::uint32_t>(scaled / rate) & 0x0ffffffcu;
    std::uint32_t realRate = ratio ? static_cast<std::uint32_t>(scaled / ratio) : 0;
    // Page 1 regs 0x9f & 0xa1: resample ratio, high and low 16 bits
    setDemodReg(1, 0x9f, (ratio >> 16) & 0xffff, 2);
    setDemodReg(1, 0xa1, ratio & 0xffff, 2);
    resetDemodulator();
    return realRate;
}

void Rtl2832u::setSampleFrequencyCorrection(double ppm) {
    // offset = -floor(ppm * 2^24 / 1e6), 14 bits over regs 0x3e[5:0], 0x3f
    std::int32_t offset = -static_cast<std::int32_t>(std::floor(ppm * (1 << 24) / 1000000.0));
    std::uint32_t bits = static_cast<std::uint32_t>(offset);
    // Page 1 regs 0x3e [13:8] & 0x3f [7:0]: sampling frequency offset
    setDemodReg(1, 0x3e, (bits >> 8) & 0x3f, 1);
    setDemodReg(1, 0x3f, bits & 0xff, 1);
}

std::int64_t Rtl2832u::setIfFrequency(std::int64_t ifFrequency, double ppm) {
    const double xtal = xtalFrequency(ppm);
    std::int32_t multiplier =
        -static_cast<std::int32_t>(std::floor(static_cast<double>(ifFrequency) * (1 << 22) / xtal));
    std::uint32_t bits = static_cast<std::uint32_t>(multiplier);
    // Page 1 regs 0x19-0x1b: digital down-converter IF multiplier [21:0]
    setDemodReg(1, 0x19, (bits >> 16) & 0x3f, 1);
    setDemodReg(1, 0x1a, (bits >> 8) & 0xff, 1);
    setDemodReg(1, 0x1b, bits & 0xff, 1);
    return static_cast<std::int64_t>(std::floor(-static_cast<double>(multiplier) * xtal / (1 << 22)));
}

void Rtl2832u::enableRtlAgc(bool enable) {
    // Page 0 reg 0x19: [2] SDR mode, [5] DAGC (0x25 on, 0x05 off)
    setDemodReg(0, 0x19, enable ? 0x25 : 0x05, 1);
}

void Rtl2832u::enterDirectSampling(int mode) {
    // Page 1 reg 0xb1: zero-IF input off (low-IF direct sampling)
    setDemodReg(1, 0xb1, 0x1a, 1);
    // Page 1 reg 0x15: non-inverted spectrum
    setDemodReg(1, 0x15, 0x00, 1);
    // Page 0 reg 0x06: [5:4] datapath: 0x80 = I ADC, 0x90 = Q ADC routed to I
    setDemodReg(0, 0x06, mode == 1 ? 0x80 : 0x90, 1);
}

void Rtl2832u::leaveDirectSampling() {
    // Page 1 reg 0x15: inverted spectrum (tuner's low-IF mixing)
    setDemodReg(1, 0x15, 0x01, 1);
    // Page 0 reg 0x06: default datapath
    setDemodReg(0, 0x06, 0x80, 1);
}

void Rtl2832u::resetDemodulator() {
    // Page 1 reg 0x01: toggle [2] soft_rst
    setDemodReg(1, 0x01, 0x14, 1);
    setDemodReg(1, 0x01, 0x10, 1);
}

void Rtl2832u::resetBuffer() {
    // USB_EPA_CTL (0x2148): stall + FIFO reset, then clear
    setUsbReg(0x2148, 0x0210, 2);
    setUsbReg(0x2148, 0x0000, 2);
}

}  // namespace detail
}  // namespace rtlsdr
