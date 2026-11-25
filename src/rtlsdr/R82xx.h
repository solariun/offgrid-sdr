// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/rtlsdr/R82xx.h   (internal to src/rtlsdr)
//  Layer:  RTL-SDR driver (in software, no librtlsdr)
//
//  What this file does
//  -------------------
//  Declares rtlsdr::detail::R82xx, the Rafael Micro R820T / R820T2 / R828D
//  tuner (all share the register map), including the RTL-SDR Blog V4
//  variant: an R828D with three switched inputs and an HF upconverter.
//
//  The tuner mixes the RF signal down to a low IF of 3.57 MHz; the RTL2832U
//  then shifts that IF to 0 Hz digitally (see Rtl2832u::setIfFrequency).
//
//  How it fits in
//  --------------
//  Port of webrtlsdr's r8xx.ts, r820t.ts and r828d.ts (@jtarrio/webrtlsdr,
//  Apache-2.0, Copyright 2024 Jacobo Tarrio Barreiro, Copyright 2013 Google
//  Inc.), which follow librtlsdr's tuner_r82xx.c (GPL-2.0+, Mauro Carvalho
//  Chehab, Steve Markgraf, Dimitri Stolnikov) and the RTL-SDR Blog fork for
//  the V4.  All register access goes through Rtl2832u's I2C repeater.
//
//  Threading/errors: called only from RtlSdr with the device lock held;
//  USB failures throw detail::IoError.
//
//  Contributing
//  ------------
//  The tuner registers are write-only in practice (reads come back
//  bit-reversed and only for status), so the driver keeps a "shadow" copy
//  of registers 0x05..0x1f and writes masked updates from it.  Never write a
//  register without going through writeRegMask()/writeRegs().
// ============================================================================
#ifndef OFFGRID_RTLSDR_R82XX_H
#define OFFGRID_RTLSDR_R82XX_H

#include <cstdint>
#include <string>
#include <vector>

#include "Rtl2832u.h"

namespace rtlsdr {
namespace detail {

// One row of a band table: RF multiplexer / filter settings that apply
// from `freqMhz` (of the PLL frequency, RF + IF) up to the next row.
struct R82xxMuxConfig {
    double freqMhz;          // lower edge of the band
    std::uint8_t openDrain;  // reg 0x17 [3]
    std::uint8_t rfMuxPoly;  // reg 0x1a [7:6] tracking filter, [1:0] RF filter
    std::uint8_t tfC;        // reg 0x1b [7:4] LPNF, [3:0] LPF
};

class R82xx {
public:
    enum Chip { R820T, R828D };

    // I2C addresses (8-bit form as the RTL2832U bridge expects them).
    static const std::uint8_t kR820TAddress = 0x34;
    static const std::uint8_t kR828DAddress = 0x74;
    // Low IF the tuner mixes to.
    static const std::uint32_t kIfFrequency = 3570000;

    // Reads the chip id register at `i2cAddress` (opening/closing the I2C
    // repeater); true if it answers 0x69 (Rafael Micro R82xx).
    static bool probe(Rtl2832u& rtl, std::uint8_t i2cAddress);

    // `blogV4`: RTL-SDR Blog V4 (only meaningful for R828D).
    R82xx(Rtl2832u& rtl, Chip chip, bool blogV4);

    std::string name() const;
    bool isBlogV4() const { return blogV4_; }

    // Powers up and initialises the tuner (also sets the RTL2832U for the
    // tuner's low-IF output).  Opens/closes the I2C repeater itself.
    void open();
    // Powers the tuner down.  Opens/closes the I2C repeater itself.
    void close();

    // The remaining calls need the I2C repeater open (caller's job).
    // Tunes to `freq` Hz; returns the frequency actually tuned (0 if the
    // PLL cannot reach it).
    std::int64_t setFrequency(std::int64_t freq);
    void setAutoGain();
    void setManualGain(double dB);

    void setXtalFrequency(std::uint32_t hz) { xtalFrequency_ = hz; }
    std::uint32_t intermediateFrequency() const { return kIfFrequency; }
    // Below this, the RTL2832U's direct sampling may take over.
    std::uint32_t minimumFrequency() const;
    // Whether the last PLL programming achieved lock.
    bool pllLocked() const { return pllLocked_; }

    // The gain steps setManualGain() distinguishes, in dB.
    static std::vector<double> gainSteps();

private:
    std::int64_t setTunerFrequency(std::int64_t freq);  // the generic R8xx part
    int calibrateFilter();
    void setMux(std::int64_t freq);
    std::int64_t setPll(std::int64_t freq);
    void waitPllLock();
    void initElectronics();

    std::vector<std::uint8_t> readRegBuffer(std::uint8_t address, std::size_t length);
    void writeRegMask(std::uint8_t address, std::uint8_t value, std::uint8_t mask);
    void setRegMask(std::uint8_t address, std::uint8_t value, std::uint8_t mask);
    void writeRegs(std::uint8_t address, std::size_t length);

    Rtl2832u& rtl_;
    Chip chip_;
    bool blogV4_;
    std::uint8_t i2c_;
    const R82xxMuxConfig* mux_;
    std::size_t muxCount_;
    int vcoPowerRef_;
    std::uint32_t xtalFrequency_;
    bool pllLocked_;
    int input_;                   // R828D: current input, -1 = unknown
    std::uint8_t shadow_[27];     // registers 0x05..0x1f
};

}  // namespace detail
}  // namespace rtlsdr

#endif
