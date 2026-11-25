// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/rtlsdr/R82xx.cpp   (internal to src/rtlsdr)
//  Layer:  RTL-SDR driver (in software, no librtlsdr)
//
//  What this file does
//  -------------------
//  Rafael Micro R820T / R820T2 / R828D tuner driver: power-up register set,
//  filter calibration, RF multiplexer / tracking filter per band, the
//  fractional-N PLL, LNA/mixer/VGA gain, power-down, and the RTL-SDR Blog
//  V4 input switching (HF / VHF / UHF) with its HF upconverter.
//
//  How it fits in
//  --------------
//  Faithful port of webrtlsdr's r8xx.ts, r820t.ts and r828d.ts
//  (@jtarrio/webrtlsdr, Apache-2.0, Copyright 2024 Jacobo Tarrio Barreiro,
//  Copyright 2013 Google Inc.), derived from librtlsdr's tuner_r82xx.c
//  (GPL-2.0+) and the RTL-SDR Blog V4 fork.  Register values and their
//  ordering are kept identical; the comments are condensed from webrtlsdr.
//
//  Contributing
//  ------------
//  C++11 has no binary literals, so register values that webrtlsdr writes
//  as 0b... are in hex here; each line keeps the bit-field comment.
// ============================================================================
#include "R82xx.h"

#include <chrono>
#include <cmath>
#include <thread>

namespace rtlsdr {
namespace detail {

const std::uint8_t R82xx::kR820TAddress;
const std::uint8_t R82xx::kR828DAddress;
const std::uint32_t R82xx::kIfFrequency;

namespace {

void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// Initial values for registers 0x05..0x1f (librtlsdr's r82xx_init_array).
const std::uint8_t kInitRegisters[27] = {
    0x83,  // 0x05 [7] loop-through off, [5] LNA1 on, [4] LNA auto, [3:0] LNA gain 3
    0x32,  // 0x06 [7] PDET1 on, [6] PDET3 on, [5] filter +3dB, [4] mixer 6MHz filter, [2:0] LNA bias 2
    0x75,  // 0x07 [7] mixer lower sideband, [6] mixer on, [4] mixer gain auto, [3:0] mixer gain 5
    0xc0,  // 0x08 [7] mixer buffer on, [6] low current, [5:0] IMR gain 0
    0x40,  // 0x09 [7] IF filter on, [6] low current, [5:0] IMR phase 0
    0xd6,  // 0x0a [7] channel filter on, [6:5] power level 2, [4] high Q, [3:0] fine BW 6
    0x6c,  // 0x0b [7:5] coarse BW 3, [4] calibration idle, [3:0] HPF corner 12
    0xf5,  // 0x0c [6] IF VGA on, [4] VGA by pin, [3:0] VGA gain 5.5dB
    0x63,  // 0x0d LNA AGC thresholds high 0.94V / low 0.64V
    0x75,  // 0x0e mixer AGC thresholds high 1.04V / low 0.84V
    0x68,  // 0x0f [6:5] LDO 3.0V, [4] clock out off, [3] fixed 1, [1] internal AGC clock on
    0x64,  // 0x10 [7:5] PLL-to-mixer div 3, [4] refdiv 1:1, [3] xtal swing high, [1:0] cap 0pF
    0x83,  // 0x11 [7:6] PLL analog regulator 2.0V, [2:0] reference bias 3
    0x80,  // 0x12 [7:5] VCO current 4, [3] SDM on (fractional-N)
    0x00,  // 0x13 [7:6] VCO control, [5:0] VCO band 0
    0x0f,  // 0x14 [7:6] SI 0, [5:0] NI 15
    0x00,  // 0x15 SDM low byte
    0xc0,  // 0x16 SDM high byte
    0x30,  // 0x17 [7:6] PLL digital regulator on, [5:4] prescaler 150uA, [3] open drain high-Z
    0x48,  // 0x18 [7:6] ring VCO bias, [3:0] reserved
    0xcc,  // 0x19 [7] RF filter on, [6:5] polyphase current, [3:0] ring divider
    0x60,  // 0x1a [7:6] tracking filter bypass, [5:4] AGC clock 80ms, [3:2] PLL autotune 128kHz
    0x00,  // 0x1b [7:4] LPNF highest, [3:0] LPF highest
    0x54,  // 0x1c [7:4] PDET3 TOP 5, [2] LNA PDET low discharge
    0xae,  // 0x1d [7:6] LNA NB PDET BW, [5:3] PDET1 TOP 5, [2:0] PDET2 TOP 6
    0x4a,  // 0x1e [6] filter extension, [4:0] LNA discharge 10
    0xc0,  // 0x1f [7] loop-through attenuation bypass, [6] fixed 1
};

// Reverses the bit order of a nibble; tuner reads come back bit-reversed.
const std::uint8_t kBitRev[16] = {0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf};

}  // namespace

// ---------------------------------------------------------------------------
//  Multiplexer configurations per band (lower edge in MHz, applied to the
//  frequency the PLL is set to, i.e. RF + IF):
//      openDrain  reg 0x17 [3]      (1: low Z)
//      rfMuxPoly  reg 0x1a [7:6] tracking filter (01: bypass), [1:0] RF filter
//                 (00 high, 01 med, 10 low)
//      tfC        reg 0x1b [7:4] LPNF, [3:0] LPF (0000: highest)
// ---------------------------------------------------------------------------
namespace {

const R82xxMuxConfig kStdMux[] = {
    {0, 0x08, 0x02, 0xdf},   {50, 0x08, 0x02, 0xbe},  {55, 0x08, 0x02, 0x8b},  {60, 0x08, 0x02, 0x7b},
    {65, 0x08, 0x02, 0x69},  {70, 0x08, 0x02, 0x58},  {75, 0x00, 0x02, 0x44},  {90, 0x00, 0x02, 0x34},
    {110, 0x00, 0x02, 0x24}, {140, 0x00, 0x02, 0x14}, {180, 0x00, 0x02, 0x13}, {250, 0x00, 0x02, 0x11},
    {280, 0x00, 0x02, 0x00}, {310, 0x00, 0x41, 0x00}, {588, 0x00, 0x40, 0x00},
};

// RTL-SDR Blog V4 specific table.
const R82xxMuxConfig kBlogV4Mux[] = {
    {0, 0x00, 0x02, 0xdf},   {2.2, 0x08, 0x02, 0xdf}, {50, 0x08, 0x02, 0xbe},  {55, 0x08, 0x02, 0x8b},
    {60, 0x08, 0x02, 0x7b},  {65, 0x08, 0x02, 0x69},  {70, 0x08, 0x02, 0x58},  {75, 0x08, 0x02, 0x44},
    {85, 0x00, 0x02, 0x44},  {90, 0x00, 0x02, 0x34},  {110, 0x00, 0x02, 0x24}, {112, 0x08, 0x02, 0x24},
    {140, 0x08, 0x02, 0x14}, {172, 0x00, 0x02, 0x14}, {180, 0x00, 0x02, 0x13}, {242, 0x08, 0x02, 0x13},
    {250, 0x08, 0x02, 0x11}, {280, 0x08, 0x02, 0x00}, {310, 0x08, 0x41, 0x00}, {588, 0x08, 0x40, 0x00},
};

}  // namespace

// ---------------------------------------------------------------------------
//  Construction / probing
// ---------------------------------------------------------------------------

bool R82xx::probe(Rtl2832u& rtl, std::uint8_t i2cAddress) {
    rtl.openI2C();
    bool found = false;
    try {
        // Register 0: chip id, 0x69 for Rafael Micro R82xx (read raw).
        found = rtl.getI2CReg(i2cAddress, 0) == 0x69;
    } catch (const IoError&) {
        // No device at this address: the I2C read NAKs.  Not an error.
    }
    rtl.closeI2C();
    return found;
}

R82xx::R82xx(Rtl2832u& rtl, Chip chip, bool blogV4)
    : rtl_(rtl),
      chip_(chip),
      blogV4_(chip == R828D && blogV4),
      i2c_(chip == R828D ? kR828DAddress : kR820TAddress),
      mux_(0),
      muxCount_(0),
      vcoPowerRef_(chip == R828D ? 1 : 2),
      xtalFrequency_(Rtl2832u::kXtalFrequency),
      pllLocked_(false),
      input_(-1) {
    if (blogV4_) {
        mux_ = kBlogV4Mux;
        muxCount_ = sizeof kBlogV4Mux / sizeof kBlogV4Mux[0];
    } else {
        mux_ = kStdMux;
        muxCount_ = sizeof kStdMux / sizeof kStdMux[0];
    }
    for (int i = 0; i < 27; ++i) shadow_[i] = kInitRegisters[i];
}

std::string R82xx::name() const {
    if (chip_ == R828D) return blogV4_ ? "R828D (RTL-SDR Blog V4)" : "R828D";
    // R820T and R820T2 answer the same id and are driven identically.
    return "R820T";
}

std::uint32_t R82xx::minimumFrequency() const {
    // The Blog V4 reaches HF through its upconverter, so the tuner covers
    // everything; otherwise below the crystal frequency is direct sampling
    // territory.
    return blogV4_ ? 0 : Rtl2832u::kXtalFrequency;
}

// ---------------------------------------------------------------------------
//  Power up / down
// ---------------------------------------------------------------------------

void R82xx::open() {
    // Demod page 1 reg 0xb1: zero-IF off (low-IF), DC est., IQ comp./est.
    rtl_.setDemodReg(1, 0xb1, 0x1a, 1);
    // Demod page 0 reg 0x08: ADC_I off, ADC_Q on (tuner IF goes to Q ADC)
    rtl_.setDemodReg(0, 0x08, 0x4d, 1);
    // Demod page 1 reg 0x15: inverted spectrum (lower-sideband low-IF)
    rtl_.setDemodReg(1, 0x15, 0x01, 1);
    rtl_.openI2C();
    for (int i = 0; i < 27; ++i) shadow_[i] = kInitRegisters[i];
    rtl_.setI2CRegBuffer(i2c_, 0x05, shadow_, 27);
    sleepMs(5);
    initElectronics();
    rtl_.closeI2C();
    input_ = -1;  // R828D: force input selection on next tune
}

void R82xx::close() {
    rtl_.openI2C();
    writeRegMask(0x06, 0xb1, 0xff);  // PDET1/PDET3 off, LNA power 1
    writeRegMask(0x05, 0xb3, 0xff);  // loop-through off, LNA1 off, LNA manual 3
    writeRegMask(0x07, 0x3a, 0xff);  // mixer off, gain auto, gain 10
    writeRegMask(0x08, 0x40, 0xff);  // mixer buffer off
    writeRegMask(0x09, 0xc0, 0xff);  // IF filter off
    writeRegMask(0x0a, 0x3a, 0xff);  // channel filter off
    writeRegMask(0x0c, 0x35, 0xff);  // IF VGA off
    writeRegMask(0x0f, 0x68, 0xff);  // LDO 3.0V, clock out off
    writeRegMask(0x11, 0x03, 0xff);  // PLL analog regulator off
    writeRegMask(0x17, 0xf4, 0xff);  // PLL digital regulator off
    writeRegMask(0x19, 0x0c, 0xff);  // RF filter off
    rtl_.closeI2C();
}

// ---------------------------------------------------------------------------
//  Frequency
// ---------------------------------------------------------------------------

std::int64_t R82xx::setFrequency(std::int64_t freq) {
    if (chip_ != R828D) return setTunerFrequency(freq);

    // Blog V4: HF below 28.8 MHz goes through an upconverter that adds
    // 28.8 MHz, so tune the tuner that much higher.
    std::int64_t upconvert = 0;
    if (blogV4_ && freq < 28800000) upconvert = 28800000;
    std::int64_t actual = setTunerFrequency(freq + upconvert);

    if (blogV4_) {
        // Input path on Blog V4:
        //   2 (HF):  freq <  28.8 MHz, upconverted, via Cable 2 input
        //   1 (VHF): 28.8 MHz <= freq < 250 MHz, via Cable 1 input
        //   0 (UHF): freq >= 250 MHz, via Air input
        int input = freq < 28800000 ? 2 : freq < 250000000 ? 1 : 0;
        if (input_ != input) {
            input_ = input;
            if (input == 0) {
                writeRegMask(0x06, 0x00, 0x08);  // Cable 2 LNA off
                writeRegMask(0x05, 0x00, 0x60);  // Cable 1 LNA off, LNA1 on
            } else if (input == 1) {
                writeRegMask(0x06, 0x00, 0x08);  // Cable 2 LNA off
                writeRegMask(0x05, 0x60, 0x60);  // Cable 1 LNA on
            } else {
                writeRegMask(0x06, 0x08, 0x08);  // Cable 2 LNA on
                writeRegMask(0x05, 0x20, 0x60);  // Cable 1 LNA off
            }
            // RTL2832U GPIO 5 drives the upconverter (active low: 0 = on)
            rtl_.setGpioOutput(5);
            rtl_.setGpioBit(5, input != 2);
        }
    } else {
        // Plain R828D: Cable 1 LNA on below 345 MHz, off above.
        int input = freq > 345000000 ? 0 : 1;
        if (input_ != input) {
            input_ = input;
            writeRegMask(0x05, input == 0 ? 0x00 : 0x60, 0x60);
        }
    }
    return actual == 0 ? 0 : actual - upconvert;
}

std::int64_t R82xx::setTunerFrequency(std::int64_t freq) {
    setMux(freq + kIfFrequency);
    std::int64_t actual = setPll(freq + kIfFrequency);
    return actual == 0 ? 0 : actual - kIfFrequency;
}

void R82xx::setMux(std::int64_t freq) {
    double freqMhz = static_cast<double>(freq) / 1000000.0;
    std::size_t i;
    for (i = 0; i < muxCount_ - 1; ++i) {
        if (freqMhz < mux_[i + 1].freqMhz) break;
    }
    const R82xxMuxConfig& cfg = mux_[i];
    writeRegMask(0x17, cfg.openDrain, 0x08);  // [3] open drain
    writeRegMask(0x1a, cfg.rfMuxPoly, 0xc3);  // [7:6] tracking filter, [1:0] RF filter
    writeRegMask(0x1b, cfg.tfC, 0xff);        // LPNF / LPF
    writeRegMask(0x10, 0x00, 0x0b);           // [3] xtal swing high, [1:0] cap 0pF
    writeRegMask(0x08, 0x00, 0x3f);           // IMR gain 0
    writeRegMask(0x09, 0x00, 0x3f);           // IMR phase 0
}

std::int64_t R82xx::setPll(std::int64_t freq) {
    const std::int64_t pllRef = xtalFrequency_;
    writeRegMask(0x10, 0x00, 0x10);  // [4] reference divider 1:1
    writeRegMask(0x1a, 0x00, 0x0c);  // [3:2] PLL autotune 128 kHz
    writeRegMask(0x12, 0x80, 0xe0);  // [7:5] VCO current 4

    // Mixer divider so that the VCO lands at >= 1.77 GHz.
    int divNum = static_cast<int>(std::floor(std::log(1770000000.0 / static_cast<double>(freq)) / std::log(2.0)));
    if (divNum < 0) divNum = 0;
    if (divNum > 6) divNum = 6;
    const std::int64_t mixDiv = static_cast<std::int64_t>(1) << (divNum + 1);

    // VCO fine tune vs. the chip's power reference nudges the divider (as
    // librtlsdr, mixDiv itself is not recomputed).
    std::vector<std::uint8_t> status = readRegBuffer(0x00, 5);
    int vcoFineTune = (status[4] & 0x30) >> 4;
    if (vcoFineTune > vcoPowerRef_) {
        divNum = divNum - 1 < 0 ? 0 : divNum - 1;
    } else if (vcoFineTune < vcoPowerRef_) {
        divNum = divNum + 1 > 6 ? 6 : divNum + 1;
    }

    const std::int64_t vcoFreq = freq * mixDiv;
    const std::int64_t nint = vcoFreq / (2 * pllRef);
    const std::int64_t vcoFra = vcoFreq % (2 * pllRef);
    if (nint > 63) {
        pllLocked_ = false;
        return 0;
    }
    const std::int64_t ni = (nint - 13) / 4;
    const std::int64_t si = (nint - 13) % 4;
    std::int64_t sdm = (32768 * vcoFra) / pllRef;
    if (sdm > 65535) sdm = 65535;

    setRegMask(0x10, static_cast<std::uint8_t>(divNum << 5), 0xe0);           // [7:5] mixer divider
    setRegMask(0x12, vcoFra == 0 ? 0x08 : 0x00, 0x08);                         // [3] SDM off if integer
    setRegMask(0x14, static_cast<std::uint8_t>(ni + (si << 6)), 0xff);         // SI / NI
    setRegMask(0x15, static_cast<std::uint8_t>(sdm & 0xff), 0xff);             // SDM low
    setRegMask(0x16, static_cast<std::uint8_t>(sdm >> 8), 0xff);               // SDM high
    writeRegs(0x10, 7);                                                         // 0x10..0x16
    waitPllLock();
    writeRegMask(0x1a, 0x08, 0x08);  // [3:2] PLL autotune 8 kHz (settled)

    const double actual = (2.0 * static_cast<double>(pllRef) * (static_cast<double>(nint) + sdm / 65536.0)) /
                          static_cast<double>(mixDiv);
    return static_cast<std::int64_t>(std::floor(actual + 0.5));
}

void R82xx::waitPllLock() {
    bool firstTry = true;
    for (;;) {
        for (int i = 1; i <= 4; ++i) {
            std::vector<std::uint8_t> status = readRegBuffer(0x00, 3);
            if (status[2] & 0x40) {  // [6] PLL locked
                pllLocked_ = true;
                return;
            }
            sleepMs(i * 5);
        }
        if (!firstTry) {
            pllLocked_ = false;
            return;
        }
        writeRegMask(0x12, 0x60, 0xe0);  // boost VCO current to 3 and retry
        firstTry = false;
    }
}

// ---------------------------------------------------------------------------
//  Gain
// ---------------------------------------------------------------------------

void R82xx::setAutoGain() {
    writeRegMask(0x05, 0x00, 0x10);  // LNA gain auto
    writeRegMask(0x07, 0x10, 0x10);  // mixer gain auto
    writeRegMask(0x0c, 0x0b, 0x9f);  // VGA manual, 26.5 dB
}

void R82xx::setManualGain(double dB) {
    // webrtlsdr's model: LNA ~2.3 dB steps, mixer ~1.2 dB steps; LNA and
    // mixer are raised together in 3.5 dB "full steps", with an extra LNA
    // step for a 2.3 dB "half step".  (The 1e-9 guards against rounding.)
    int fullsteps = static_cast<int>(std::floor(dB / 3.5 + 1e-9));
    int halfsteps = dB - 3.5 * fullsteps >= 2.3 - 1e-9 ? 1 : 0;
    if (fullsteps < 0) fullsteps = 0;
    if (fullsteps > 15) fullsteps = 15;
    if (fullsteps == 15) halfsteps = 0;
    std::uint8_t lnaValue = static_cast<std::uint8_t>(fullsteps + halfsteps);
    std::uint8_t mixerValue = static_cast<std::uint8_t>(fullsteps);
    writeRegMask(0x05, 0x10, 0x10);       // LNA gain manual
    writeRegMask(0x07, 0x00, 0x10);       // mixer gain manual
    writeRegMask(0x0c, 0x08, 0x9f);       // VGA manual, 16 dB
    writeRegMask(0x05, lnaValue, 0x0f);   // LNA gain code
    writeRegMask(0x07, mixerValue, 0x0f); // mixer gain code
}

std::vector<double> R82xx::gainSteps() {
    std::vector<double> steps;
    for (int k = 0; k <= 15; ++k) {
        steps.push_back(3.5 * k);
        if (k < 15) steps.push_back(3.5 * k + 2.3);
    }
    return steps;
}

// ---------------------------------------------------------------------------
//  Initialisation
// ---------------------------------------------------------------------------

int R82xx::calibrateFilter() {
    bool firstTry = true;
    for (;;) {
        writeRegMask(0x0b, 0x60, 0x60);  // coarse BW for calibration
        writeRegMask(0x0f, 0x04, 0x04);  // calibration clock on
        writeRegMask(0x10, 0x00, 0x03);  // xtal cap 0 pF
        setPll(56000000);
        if (!pllLocked_) throw IoError("R82xx: PLL not locked -- cannot calibrate the tuner filter");
        writeRegMask(0x0b, 0x10, 0x10);  // start calibration
        writeRegMask(0x0b, 0x00, 0x10);  // clear start bit
        writeRegMask(0x0f, 0x00, 0x04);  // calibration clock off
        std::vector<std::uint8_t> status = readRegBuffer(0x00, 5);
        int filterCap = status[4] & 0x0f;  // measured capacitor code
        if (filterCap == 0x0f) filterCap = 0;
        if (filterCap == 0 || !firstTry) return filterCap;
        firstTry = false;
    }
}

void R82xx::initElectronics() {
    writeRegMask(0x0c, 0x00, 0x0f);  // VGA gain -12 dB
    writeRegMask(0x13, 0x31, 0x3f);  // VCO band 49
    writeRegMask(0x1d, 0x00, 0x38);  // PDET1 TOP 0
    int filterCap = calibrateFilter();
    writeRegMask(0x0a, static_cast<std::uint8_t>(0x10 | filterCap), 0x1f);  // high Q, fine BW = cal
    writeRegMask(0x0b, 0x6b, 0xef);  // coarse BW 3, HPF 11
    writeRegMask(0x07, 0x00, 0x80);  // mixer lower sideband
    writeRegMask(0x06, 0x10, 0x30);  // filter gain 0 dB, mixer 6 MHz filter
    writeRegMask(0x1e, 0x40, 0x60);  // filter extension
    writeRegMask(0x05, 0x00, 0x80);  // loop-through on
    writeRegMask(0x1f, 0x00, 0x80);  // loop-through attenuation
    writeRegMask(0x0f, 0x00, 0x80);  // filter extension widest off
    writeRegMask(0x19, 0x60, 0x60);  // RF polyphase current
    writeRegMask(0x1d, 0xe5, 0xc7);  // LNA NB PDET BW, PDET2 TOP 5
    writeRegMask(0x1c, 0x24, 0xf8);  // PDET3 TOP 4
    writeRegMask(0x0d, 0x53, 0xff);  // LNA AGC thresholds 0.84/0.64 V
    writeRegMask(0x0e, 0x75, 0xff);  // mixer AGC thresholds 1.04/0.84 V
    writeRegMask(0x05, 0x00, 0x60);  // Cable 1 LNA off, LNA1 on
    writeRegMask(0x06, 0x00, 0x08);  // Cable 2 LNA off
    writeRegMask(0x11, 0x38, 0x08);  // calibration bias
    writeRegMask(0x17, 0x30, 0x30);  // prescaler 150 uA
    writeRegMask(0x0a, 0x40, 0x60);  // filter power level 2
    writeRegMask(0x1d, 0x00, 0x38);  // PDET1 TOP 0
    writeRegMask(0x1c, 0x00, 0x04);  // LNA PDET discharge normal
    writeRegMask(0x06, 0x00, 0x40);  // LNA PDET narrow band off
    writeRegMask(0x1a, 0x30, 0x30);  // AGC clock 20 ms
    writeRegMask(0x1d, 0x18, 0x38);  // PDET1 TOP 3
    writeRegMask(0x1c, 0x24, 0x04);  // LNA PDET1 low discharge
    writeRegMask(0x1e, 0x0d, 0x1f);  // LNA discharge current 13
    writeRegMask(0x1a, 0x20, 0x30);  // AGC clock 80 ms
}

// ---------------------------------------------------------------------------
//  Register helpers
// ---------------------------------------------------------------------------

std::vector<std::uint8_t> R82xx::readRegBuffer(std::uint8_t address, std::size_t length) {
    std::vector<std::uint8_t> data = rtl_.getI2CRegBuffer(i2c_, address, length);
    for (std::size_t i = 0; i < data.size(); ++i) {
        std::uint8_t b = data[i];
        data[i] = static_cast<std::uint8_t>((kBitRev[b & 0xf] << 4) | kBitRev[b >> 4]);
    }
    return data;
}

void R82xx::writeRegMask(std::uint8_t address, std::uint8_t value, std::uint8_t mask) {
    setRegMask(address, value, mask);
    rtl_.setI2CReg(i2c_, address, shadow_[address - 5]);
}

void R82xx::setRegMask(std::uint8_t address, std::uint8_t value, std::uint8_t mask) {
    std::uint8_t& r = shadow_[address - 5];
    r = static_cast<std::uint8_t>((r & ~mask) | (value & mask));
}

void R82xx::writeRegs(std::uint8_t address, std::size_t length) {
    rtl_.setI2CRegBuffer(i2c_, address, &shadow_[address - 5], length);
}

}  // namespace detail
}  // namespace rtlsdr
