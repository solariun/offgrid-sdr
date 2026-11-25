// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Ax25.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  dec::Ax25Decoder (see Ax25.h): the glue between the audio and the
//  AX25Toolkit modem (Gustavo Campos, https://github.com/solariun/AX25Toolkit;
//  DSP from Dire Wolf, John Langner WB2OSZ), HDLC framing and frame parser:
//
//    audio (any rate, float)
//      -> sanitised (NaN/Inf -> 0, clipped to +-16)
//      -> decimated by an integer factor when the rate is higher than the
//         modem needs (windowed-sinc FIR, computed only at output samples)
//      -> modem::Demodulator (n slicers, each its own bit clock and DCD)
//      -> one hdlc::Decoder per slicer (flags, unstuffing, FCS)
//      -> gotFrame(): the same frame from several slicers is kept once
//         (same bytes within 0.5 s of audio); parsed (ax25ParseFrame, APRS),
//         distance/bearing from setHome(), text and frame lists.
//
//  How it fits in
//  --------------
//  Implements Ax25.h.  Uses ax25/modem.h, ax25/hdlc.h (AX25Toolkit) and
//  Ax25Aprs.cpp (frame parsing, APRS).  Standard library only.
//
//  Contributing
//  ------------
//  No allocation per sample (only per decoded frame and on a rate change).
//  Run tests/test_decoders_ax25.cpp after any change.
// ============================================================================
#include "Ax25.h"

#include "ax25/hdlc.h"
#include "ax25/modem.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace dec {

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kPi = 3.141592653589793;
const double kDupWindow = 0.5;      // s: the same frame from another slicer
const int kRecent = 16;             // frames remembered for that

// Integer-factor decimator: a Hamming-windowed sinc low-pass (cut-off at
// 0.4 of the output rate), evaluated only at the samples kept.
class Decimator {
public:
    Decimator() : D_(1), taps_(1), pos_(0), phase_(0) {}
    void init(int D) {
        D_ = D < 1 ? 1 : D;
        taps_ = 16 * D_ + 1;
        h_.assign(taps_, 0.0f);
        buf_.assign(2 * taps_, 0.0f);
        pos_ = 0;
        phase_ = 0;
        const double fc = 0.4 / D_, c = 0.5 * (taps_ - 1);
        double sum = 0;
        for (int j = 0; j < taps_; ++j) {
            const double t = j - c;
            double v = std::fabs(t) < 1e-9 ? 2 * fc : std::sin(2 * kPi * fc * t) / (kPi * t);
            v *= 0.54 - 0.46 * std::cos(2 * kPi * j / (taps_ - 1));
            h_[j] = static_cast<float>(v);
            sum += v;
        }
        for (int j = 0; j < taps_; ++j) h_[j] = static_cast<float>(h_[j] / sum);
    }
    int factor() const { return D_; }
    // Pushes a sample; true (and y) when an output sample is due.
    bool push(float x, float& y) {
        if (D_ == 1) { y = x; return true; }
        pos_ = (pos_ == 0 ? taps_ : pos_) - 1;
        buf_[pos_] = x;
        buf_[pos_ + taps_] = x;
        if (++phase_ < D_) return false;
        phase_ = 0;
        const float* d = &buf_[pos_];
        float s = 0;
        for (int j = 0; j < taps_; ++j) s += h_[j] * d[j];
        y = s;
        return true;
    }

private:
    int D_, taps_, pos_, phase_;
    std::vector<float> h_, buf_;
};

unsigned fnv(const unsigned char* d, std::size_t n) {
    unsigned h = 2166136261u;
    for (std::size_t i = 0; i < n; ++i) h = (h ^ d[i]) * 16777619u;
    return h;
}

}  // namespace

struct Ax25Decoder::Impl {
    modem::Demodulator demod;
    hdlc::Decoder hdlc[modem::MAX_SLICERS];
    Decimator dec;
    bool active;            // the rate suits the speed
    double outRate;
    unsigned long long n;   // input samples since reset
    struct Seen {
        unsigned hash;
        std::size_t len;
        double t;
    } recent[kRecent];
    int recentPos;
    Impl() : active(false), outRate(0), n(0), recentPos(0) {
        for (int k = 0; k < kRecent; ++k) { recent[k].hash = 0; recent[k].len = 0; recent[k].t = -1e9; }
    }
};

Ax25Decoder::Ax25Decoder()
    : impl_(new Impl()), baud_(1200), slicers_(5), haveHome_(false), homeLat_(0), homeLon_(0), maxFrames_(100),
      rate_(0), audioT_(0), lastFrameT_(-1e9), count_(0) {}

Ax25Decoder::~Ax25Decoder() {}

void Ax25Decoder::setBaud(int baud) {
    if (baud != 1200 && baud != 300 && baud != 9600) return;
    baud_ = baud;
    reset();
}

void Ax25Decoder::setSlicers(int n) {
    slicers_ = n < 1 ? 1 : n > modem::MAX_SLICERS ? modem::MAX_SLICERS : n;
    reset();
}

void Ax25Decoder::setHome(double lat, double lon) {
    if (!(std::fabs(lat) <= 90 && std::fabs(lon) <= 180)) return;
    haveHome_ = true;
    homeLat_ = lat;
    homeLon_ = lon;
}

void Ax25Decoder::clearHome() { haveHome_ = false; }

void Ax25Decoder::setMaxFrames(std::size_t n) {
    maxFrames_ = n ? n : 1;
    while (frames_.size() > maxFrames_) frames_.pop_front();
}

double Ax25Decoder::audioCenterHz() const { return baud_ == 9600 ? 0.0 : 1700.0; }

void Ax25Decoder::reset() {
    text_.clear();
    frames_.clear();
    fresh_.clear();
    count_ = 0;
    audioT_ = 0;
    lastFrameT_ = -1e9;
    const double r = rate_;
    rate_ = 0;
    impl_.reset(new Impl());
    if (r > 0) setup(r);
}

void Ax25Decoder::setup(double rate) {
    rate_ = rate;
    Impl& I = *impl_;
    // The most the modem needs (and its filters' 480-tap limit allows).
    const double maxOut = baud_ == 9600 ? 50000.0 : baud_ == 300 ? 16000.0 : 26000.0;
    const double minOut = baud_ == 9600 ? 38000.0 : 6000.0;
    const int D = static_cast<int>(std::ceil(rate / maxOut - 1e-9));
    I.dec.init(D < 1 ? 1 : D);
    I.outRate = rate / I.dec.factor();
    I.active = I.outRate >= minOut;
    const modem::Type type = baud_ == 9600 ? modem::GMSK_9600 : baud_ == 300 ? modem::AFSK_300 : modem::AFSK_1200;
    I.demod.init(type, static_cast<int>(std::floor(I.outRate + 0.5)), slicers_);
    Ax25Decoder* self = this;
    for (int k = 0; k < modem::MAX_SLICERS; ++k) {
        I.hdlc[k].init();
        I.hdlc[k].set_on_frame([self, k](const uint8_t* d, size_t n) { self->gotFrame(k, d, n); });
    }
    hdlc::Decoder* h = I.hdlc;
    I.demod.set_on_bit([h](int k, int bit) { h[k].receive_bit(bit); });
}

void Ax25Decoder::process(const float* audio, std::size_t n, double rate) {
    if (!audio || !n) return;
    if (!(rate > 0) || rate - rate != 0) return;   // not a positive finite rate
    if (rate != rate_) setup(rate);
    Impl& I = *impl_;
    if (!I.active) {
        I.n += n;
        audioT_ = I.n / rate_;
        return;
    }
    for (std::size_t i = 0; i < n; ++i) {
        float x = audio[i];
        if (!(x == x) || x > 16.0f) x = x == x ? 16.0f : 0.0f;   // NaN -> 0, +Inf/huge -> clip
        else if (x < -16.0f) x = -16.0f;
        ++I.n;
        float y;
        if (!I.dec.push(x, y)) continue;
        audioT_ = I.n / rate_;   // (frames found in this sample are stamped with it)
        I.demod.process_sample(y);
    }
    audioT_ = I.n / rate_;
}

void Ax25Decoder::gotFrame(int slicer, const unsigned char* data, std::size_t len) {
    Impl& I = *impl_;
    // (each slicer may find the same packet: keep it once)
    const unsigned h = fnv(data, len);
    for (int k = 0; k < kRecent; ++k)
        if (I.recent[k].hash == h && I.recent[k].len == len && audioT_ - I.recent[k].t < kDupWindow) return;
    Ax25Frame f;
    if (!ax25ParseFrame(data, len, f)) return;
    I.recent[I.recentPos].hash = h;
    I.recent[I.recentPos].len = len;
    I.recent[I.recentPos].t = audioT_;
    I.recentPos = (I.recentPos + 1) % kRecent;

    f.time = audioT_;
    f.slicer = slicer;
    if (haveHome_ && aprsHas(f.aprs.lat)) aprsDistance(homeLat_, homeLon_, f.aprs.lat, f.aprs.lon, f.aprs.distanceKm, f.aprs.bearingDeg);
    else f.aprs.distanceKm = f.aprs.bearingDeg = kNaN;

    text_ += f.monitorLine();
    text_ += '\n';
    if (f.aprs.kind != AprsKind::None) {
        const std::string d = aprsDescribe(f.aprs);
        text_ += "  APRS " + aprsWhat(f.aprs) + (d.empty() ? "" : " \xC2\xB7 " + d) + "\n";
    }
    ++count_;
    lastFrameT_ = audioT_;
    frames_.push_back(f);
    while (frames_.size() > maxFrames_) frames_.pop_front();
    fresh_.push_back(f);
    if (fresh_.size() > maxFrames_) fresh_.erase(fresh_.begin());
}

void Ax25Decoder::takeText(std::string& out) {
    out += text_;
    text_.clear();
}

std::vector<Ax25Frame> Ax25Decoder::frames() const { return std::vector<Ax25Frame>(frames_.begin(), frames_.end()); }

void Ax25Decoder::takeFrames(std::vector<Ax25Frame>& out) {
    out.insert(out.end(), fresh_.begin(), fresh_.end());
    fresh_.clear();
}

bool Ax25Decoder::dcd() const { return impl_->active && impl_->demod.dcd(); }

Status Ax25Decoder::status() const {
    Status s;
    const Impl& I = *impl_;
    s.locked = dcd();
    s.signal = s.locked || (count_ && audioT_ - lastFrameT_ < 2.0);
    s.quality = I.active ? I.demod.dcd_count() / 32.0 : 0;
    s.baud = baud_;
    s.characters = count_;
    if (baud_ == 9600) {
        s.mode = "G3RUH 9600";
    } else {
        s.toneHz = baud_ == 300 ? 1600 : 1200;
        s.shiftHz = baud_ == 300 ? 200 : 1000;
        s.mode = baud_ == 300 ? "AFSK 300" : "AFSK 1200";
    }
    char b[96];
    if (rate_ > 0 && !I.active)
        std::snprintf(b, sizeof b, "%s  needs %s audio", s.mode.c_str(), baud_ == 9600 ? ">= 38.4 kHz" : ">= 6 kHz");
    else
        std::snprintf(b, sizeof b, "%s  %u frame%s%s", s.mode.c_str(), count_, count_ == 1 ? "" : "s",
                      s.locked ? "  DCD" : "");
    s.info = b;
    return s;
}

}  // namespace dec
