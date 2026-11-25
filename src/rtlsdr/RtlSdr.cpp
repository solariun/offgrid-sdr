// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/rtlsdr/RtlSdr.cpp
//  Layer:  RTL-SDR driver (in software, no librtlsdr)
//
//  What this file does
//  -------------------
//  Implements the public rtlsdr::RtlSdr class: finds sticks, opens one,
//  detects its tuner, and applies sample rate, frequency, ppm correction,
//  gain, direct sampling and bias-tee, then streams raw u8 IQ samples.
//
//  How it fits in
//  --------------
//  The policy half of webrtlsdr's rtl2832u.ts (@jtarrio/webrtlsdr,
//  Apache-2.0, Copyright 2024 Jacobo Tarrio Barreiro, Copyright 2013 Google
//  Inc.; itself following librtlsdr, GPL-2.0+):
//    * after a ppm change, the IF, the centre frequency and the sample rate
//      are re-applied, because all depend on the corrected crystal;
//    * direct sampling is a *method* chosen by the user that engages only
//      when the centre frequency is below the tuner's minimum (28.8 MHz;
//      never on a Blog V4, which has its own HF upconverter);
//    * in direct sampling the tuner is powered down and "gain" means the
//      RTL2832U's digital AGC on/off.
//  Register work lives in Rtl2832u.cpp and R82xx.cpp.
//
//  Threading
//  ---------
//  Control calls are serialised by a mutex, so the UI thread may retune
//  while samples flow.  The sample callback runs on the USB thread; it must
//  not call stop() or close() (that would join its own thread).
//
//  Contributing
//  ------------
//  Internally, USB errors are detail::IoError exceptions; every public
//  method catches them and reports `false` + error().  Keep it that way so
//  callers never see exceptions.
// ============================================================================
#include "RtlSdr.h"

#include <cmath>
#include <exception>
#include <mutex>

#include "R82xx.h"
#include "Rtl2832u.h"

namespace rtlsdr {

namespace {

// RTL2832U-based sticks known to librtlsdr (vendor, product).  Only those
// with an R820T/R820T2/R828D tuner are usable; others open and then report
// "unsupported tuner".
const std::uint16_t kKnownIds[][2] = {
    {0x0bda, 0x2832},  // Generic RTL2832U
    {0x0bda, 0x2838},  // Generic RTL2832U OEM (most RTL-SDR sticks, Blog V3/V4)
    {0x0413, 0x6680},  // DigitalNow Quad DVB-T PCI-E card
    {0x0413, 0x6f0f},  // Leadtek WinFast DTV Dongle mini D
    {0x0458, 0x707f},  // Genius TVGo DVB-T03 USB dongle (Ver. B)
    {0x0ccd, 0x00a9},  // Terratec Cinergy T Stick Black (rev 1)
    {0x0ccd, 0x00b3},  // Terratec NOXON DAB/DAB+ USB dongle (rev 1)
    {0x0ccd, 0x00b4},  // Terratec Deutschlandradio DAB Stick
    {0x0ccd, 0x00b5},  // Terratec NOXON DAB Stick - Radio Energy
    {0x0ccd, 0x00b7},  // Terratec Media Broadcast DAB Stick
    {0x0ccd, 0x00b8},  // Terratec BR DAB Stick
    {0x0ccd, 0x00b9},  // Terratec WDR DAB Stick
    {0x0ccd, 0x00c0},  // Terratec MuellerVerlag DAB Stick
    {0x0ccd, 0x00c6},  // Terratec Fraunhofer DAB Stick
    {0x0ccd, 0x00d3},  // Terratec Cinergy T Stick RC (Rev.3)
    {0x0ccd, 0x00d7},  // Terratec T Stick PLUS
    {0x0ccd, 0x00e0},  // Terratec NOXON DAB/DAB+ USB dongle (rev 2)
    {0x1554, 0x5020},  // PixelView PV-DT235U(RN)
    {0x15f4, 0x0131},  // Astrometa DVB-T/DVB-T2
    {0x15f4, 0x0133},  // HanfTek DAB+FM+DVB-T
    {0x185b, 0x0620},  // Compro Videomate U620F
    {0x185b, 0x0650},  // Compro Videomate U650F
    {0x185b, 0x0680},  // Compro Videomate U680F
    {0x1b80, 0xd393},  // GIGABYTE GT-U7300
    {0x1b80, 0xd394},  // DIKOM USB-DVBT HD
    {0x1b80, 0xd395},  // Peak 102569AGPK
    {0x1b80, 0xd397},  // KWorld KW-UB450-T USB DVB-T Pico TV
    {0x1b80, 0xd398},  // Zaapa ZT-MINDVBZP
    {0x1b80, 0xd39d},  // SVEON STV20 DVB-T USB & FM
    {0x1b80, 0xd3a4},  // Twintech UT-40
    {0x1b80, 0xd3a8},  // ASUS U3100MINI_PLUS_V2
    {0x1b80, 0xd3af},  // SVEON STV27 DVB-T USB & FM
    {0x1b80, 0xd3b0},  // SVEON STV21 DVB-T USB & FM
    {0x1d19, 0x1101},  // Dexatek DK DVB-T Dongle (Logilink VG0002A)
    {0x1d19, 0x1102},  // Dexatek DK DVB-T Dongle (MSI DigiVox mini II V3.0)
    {0x1d19, 0x1103},  // Dexatek Technology Ltd. DK 5217 DVB-T Dongle
    {0x1d19, 0x1104},  // MSI DigiVox Micro HD
    {0x1f4d, 0xa803},  // Sweex DVB-T USB
    {0x1f4d, 0xb803},  // GTek T803
    {0x1f4d, 0xc803},  // Lifeview LV5TDeluxe
    {0x1f4d, 0xd286},  // MyGica TD312
    {0x1f4d, 0xd803},  // PROlectrix DV107669
};

}  // namespace

// ---------------------------------------------------------------------------
//  Impl: the open device and the state that must be re-applied.
// ---------------------------------------------------------------------------
struct RtlSdr::Impl {
    std::unique_ptr<usb::Device> device;
    std::unique_ptr<detail::Rtl2832u> rtl;
    std::unique_ptr<detail::R82xx> tuner;
    std::mutex mutex;  // serialises all control traffic

    std::uint32_t requestedRate;   // what the user asked for (re-applied on ppm change)
    std::uint32_t realRate;        // what the resampler produces
    std::uint64_t centerFrequency;
    double ppm;
    bool autoGain;
    double gain;
    int directSamplingMethod;      // what the user chose (0 off, 1 I, 2 Q)
    int directSampling;            // what is engaged right now
    bool biasTee;
    bool streaming;

    Impl()
        : requestedRate(0),
          realRate(0),
          centerFrequency(0),
          ppm(0),
          autoGain(true),
          gain(0),
          directSamplingMethod(0),
          directSampling(0),
          biasTee(false),
          streaming(false) {}

    // setGain() in rtl2832u.ts.
    void applyGain() {
        if (directSampling != 0) {
            rtl->enableRtlAgc(autoGain);
        } else {
            rtl->openI2C();
            if (autoGain) {
                tuner->setAutoGain();
            } else {
                tuner->setManualGain(gain);
            }
            rtl->closeI2C();
        }
    }

    // _maybeSetDirectSampling() in rtl2832u.ts.
    void maybeSetDirectSampling(std::uint64_t frequency) {
        bool lowFrequency = frequency < tuner->minimumFrequency();
        int method = lowFrequency ? directSamplingMethod : 0;
        if (directSampling == method) return;
        bool tunerWasOn = directSampling == 0;
        directSampling = method;
        if (method != 0) {
            if (tunerWasOn) tuner->close();
            rtl->enterDirectSampling(method);
            rtl->enableRtlAgc(autoGain);
        } else {
            tuner->open();
            std::uint32_t ifFreq = tuner->intermediateFrequency();
            if (ifFreq != 0) rtl->setIfFrequency(ifFreq, ppm);
            rtl->leaveDirectSampling();
            rtl->enableRtlAgc(false);
            applyGain();
        }
    }

    // setCenterFrequency() in rtl2832u.ts; returns the frequency actually
    // tuned (0 if the tuner cannot reach it).
    std::int64_t tune(std::uint64_t frequency) {
        maybeSetDirectSampling(frequency);
        std::int64_t actual;
        if (directSampling != 0) {
            actual = rtl->setIfFrequency(static_cast<std::int64_t>(frequency), ppm);
        } else {
            rtl->openI2C();
            actual = tuner->setFrequency(static_cast<std::int64_t>(frequency));
            rtl->closeI2C();
        }
        return actual;
    }

    // setFrequencyCorrection() in rtl2832u.ts.
    void applyCorrection() {
        rtl->setSampleFrequencyCorrection(ppm);
        tuner->setXtalFrequency(detail::Rtl2832u::xtalFrequency(ppm));
        std::uint32_t ifFreq = tuner->intermediateFrequency();
        if (ifFreq != 0) rtl->setIfFrequency(ifFreq, ppm);
        if (centerFrequency != 0) tune(centerFrequency);
        if (requestedRate != 0) realRate = rtl->setSampleRate(requestedRate, ppm);
    }
};

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

RtlSdr::RtlSdr() : impl_(new Impl) {}

RtlSdr::~RtlSdr() { close(); }

std::vector<usb::DeviceInfo> RtlSdr::find() {
    std::vector<std::pair<std::uint16_t, std::uint16_t> > ids;
    for (std::size_t i = 0; i < sizeof kKnownIds / sizeof kKnownIds[0]; ++i) {
        ids.push_back(std::make_pair(kKnownIds[i][0], kKnownIds[i][1]));
    }
    return usb::Device::find(ids);
}

bool RtlSdr::open(const usb::DeviceInfo& info) {
    close();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::string usbError;
    impl_->device = usb::Device::open(info, 0, usbError);
    if (!impl_->device) {
        error_ = usbError.empty() ? std::string("Cannot open the RTL-SDR stick") : usbError;
        return false;
    }
    try {
        impl_->rtl.reset(new detail::Rtl2832u(*impl_->device));
        impl_->rtl->init();
        // Find the tuner: R820T/R820T2 answer at 0x34, R828D at 0x74.
        if (detail::R82xx::probe(*impl_->rtl, detail::R82xx::kR820TAddress)) {
            impl_->tuner.reset(new detail::R82xx(*impl_->rtl, detail::R82xx::R820T, false));
        } else if (detail::R82xx::probe(*impl_->rtl, detail::R82xx::kR828DAddress)) {
            // The Blog V4 identifies itself only by its USB strings.
            bool blogV4 = info.manufacturer == "RTLSDRBlog" && info.product == "Blog V4";
            impl_->tuner.reset(new detail::R82xx(*impl_->rtl, detail::R82xx::R828D, blogV4));
        } else {
            throw detail::IoError(
                "Sorry, this USB dongle has an unsupported tuner chip (only R820T, R820T2 and R828D are "
                "supported).");
        }
        impl_->tuner->open();
        // Defaults, as RTL2832U.open(): automatic gain, 0 ppm.
        impl_->autoGain = true;
        impl_->applyGain();
        impl_->ppm = 0;
        impl_->applyCorrection();
    } catch (const std::exception& e) {
        error_ = e.what();
        impl_->tuner.reset();
        impl_->rtl.reset();
        impl_->device.reset();
        return false;
    }
    error_.clear();
    return true;
}

void RtlSdr::close() {
    if (!impl_->device) return;
    stop();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    try {
        if (impl_->tuner && impl_->directSampling == 0) impl_->tuner->close();
    } catch (const std::exception&) {
        // Unplugged: nothing left to power down.
    }
    impl_->tuner.reset();
    impl_->rtl.reset();
    impl_->device.reset();  // releases the interface (and re-attaches the kernel driver on Linux)
    Impl fresh;
    impl_->requestedRate = fresh.requestedRate;
    impl_->realRate = fresh.realRate;
    impl_->centerFrequency = fresh.centerFrequency;
    impl_->ppm = fresh.ppm;
    impl_->autoGain = fresh.autoGain;
    impl_->gain = fresh.gain;
    impl_->directSamplingMethod = fresh.directSamplingMethod;
    impl_->directSampling = fresh.directSampling;
    impl_->biasTee = fresh.biasTee;
    impl_->streaming = fresh.streaming;
}

bool RtlSdr::isOpen() const { return static_cast<bool>(impl_->device); }

std::string RtlSdr::tunerName() const { return impl_->tuner ? impl_->tuner->name() : std::string(); }

bool RtlSdr::isBlogV4() const { return impl_->tuner && impl_->tuner->isBlogV4(); }

bool RtlSdr::setSampleRate(std::uint32_t hz) {
    if (!isOpen()) {
        error_ = "RTL-SDR not open";
        return false;
    }
    // The resampler's valid ranges (same limits as librtlsdr).
    if (!((hz > 225000 && hz <= 300000) || (hz > 900000 && hz <= 3200000))) {
        error_ = "Invalid sample rate: use 225001-300000 Hz or 900001-3200000 Hz";
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    try {
        impl_->requestedRate = hz;
        impl_->realRate = impl_->rtl->setSampleRate(hz, impl_->ppm);
    } catch (const std::exception& e) {
        error_ = e.what();
        return false;
    }
    return true;
}

std::uint32_t RtlSdr::sampleRate() const { return impl_->realRate; }

bool RtlSdr::setCenterFrequency(std::uint64_t hz) {
    if (!isOpen()) {
        error_ = "RTL-SDR not open";
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    try {
        std::int64_t actual = impl_->tune(hz);
        if (actual == 0 && hz != 0) {
            error_ = "Frequency out of the tuner's range";
            return false;
        }
        impl_->centerFrequency = hz;
    } catch (const std::exception& e) {
        error_ = e.what();
        return false;
    }
    return true;
}

std::uint64_t RtlSdr::centerFrequency() const { return impl_->centerFrequency; }

bool RtlSdr::setFrequencyCorrection(double ppm) {
    if (!isOpen()) {
        error_ = "RTL-SDR not open";
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    try {
        impl_->ppm = ppm;
        impl_->applyCorrection();
    } catch (const std::exception& e) {
        error_ = e.what();
        return false;
    }
    return true;
}

double RtlSdr::frequencyCorrection() const { return impl_->ppm; }

std::vector<double> RtlSdr::gains() const { return detail::R82xx::gainSteps(); }

bool RtlSdr::setGain(double dB) {
    if (!isOpen()) {
        error_ = "RTL-SDR not open";
        return false;
    }
    // Snap to the nearest supported step.
    std::vector<double> steps = detail::R82xx::gainSteps();
    double best = steps[0];
    for (std::size_t i = 1; i < steps.size(); ++i) {
        if (std::fabs(steps[i] - dB) < std::fabs(best - dB)) best = steps[i];
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    try {
        impl_->autoGain = false;
        impl_->gain = best;
        impl_->applyGain();
    } catch (const std::exception& e) {
        error_ = e.what();
        return false;
    }
    return true;
}

bool RtlSdr::setAutoGain() {
    if (!isOpen()) {
        error_ = "RTL-SDR not open";
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    try {
        impl_->autoGain = true;
        impl_->applyGain();
    } catch (const std::exception& e) {
        error_ = e.what();
        return false;
    }
    return true;
}

bool RtlSdr::setDirectSampling(int mode) {
    if (!isOpen()) {
        error_ = "RTL-SDR not open";
        return false;
    }
    if (mode < 0 || mode > 2) {
        error_ = "Direct sampling mode must be 0 (off), 1 (I) or 2 (Q)";
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->directSamplingMethod == mode) return true;
    try {
        impl_->directSamplingMethod = mode;
        // Takes effect when the centre frequency is below the tuner's range.
        if (impl_->centerFrequency != 0) impl_->tune(impl_->centerFrequency);
    } catch (const std::exception& e) {
        error_ = e.what();
        return false;
    }
    return true;
}

bool RtlSdr::setBiasTee(bool on) {
    if (!isOpen()) {
        error_ = "RTL-SDR not open";
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    try {
        // GPIO 0 switches the bias-tee (RTL-SDR Blog V3/V4 and clones).
        impl_->rtl->setGpioOutput(0);
        impl_->rtl->setGpioBit(0, on);
        impl_->biasTee = on;
    } catch (const std::exception& e) {
        error_ = e.what();
        return false;
    }
    return true;
}

bool RtlSdr::start(SampleCallback callback, std::size_t bufferBytes, int transfers) {
    if (!isOpen()) {
        error_ = "RTL-SDR not open";
        return false;
    }
    if (bufferBytes == 0 || bufferBytes % 512 != 0) {
        error_ = "Buffer size must be a non-zero multiple of 512 bytes";
        return false;
    }
    stop();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    try {
        impl_->rtl->resetBuffer();  // empty the FIFO so the stream starts fresh
    } catch (const std::exception& e) {
        error_ = e.what();
        return false;
    }
    if (!impl_->device->startStream(detail::Rtl2832u::kSampleEndpoint, bufferBytes, transfers,
                                    usb::BulkCallback(callback))) {
        error_ = impl_->device->lastError();
        return false;
    }
    impl_->streaming = true;
    return true;
}

void RtlSdr::stop() {
    if (!impl_->device) return;
    // Not under the mutex: the stream thread never takes it, and holding it
    // while joining would block concurrent control calls needlessly.
    impl_->device->stopStream();
    impl_->streaming = false;
}

}  // namespace rtlsdr
