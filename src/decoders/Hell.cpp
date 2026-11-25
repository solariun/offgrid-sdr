// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Hell.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  HellDecoder: Feld Hell tone strength -> columns of 14 dots (see Hell.h).
//  Port of OFFgrid-SDR's JS HellDecoder (diddy-boy,
//  https://github.com/solariun/offgrid-sdr).
//
//  How it fits in
//  --------------
//  Implements Hell.h.  Standard library only.
//
//  Contributing
//  ------------
//  Keep in step with the JS; no allocation per sample; strict C++11.
// ============================================================================
#include "Hell.h"

#include <cmath>
#include <cstdio>
#include <utility>

namespace dec {

namespace {
const double kTwoPi = 6.283185307179586;
bool finite(double v) { return v == v && v - v == 0; }
}  // namespace

HellDecoder::HellDecoder() : rate_(0), per_(1), a_(0), n_(0) {
    pending_.reserve(64);
    rebuild();
}

void HellDecoder::rebuild() {
    ph_ = 0;
    for (int i = 0; i < 4; ++i) lp_[i] = 0;
    acc_ = 0;
    cnt_ = 0;
    peak_ = 1e-6;
    ncol_ = 0;
    contrast_ = 0;
    for (int b = 0; b < 2; ++b) {
        refPh_[b] = 0;
        for (int i = 0; i < 4; ++i) refLp_[b][i] = 0;
        pRef_[b] = 0;
    }
    pTone_ = 0;
    ratio_ = 0;
    holdLeft_ = 0;
}

void HellDecoder::reset() {
    rebuild();
    pending_.clear();
    n_ = 0;
}

// A dot is rate/245 samples (not a whole number: the remainder carries over);
// the low-pass is 150 Hz, enough for 4 ms dots.
void HellDecoder::setRate(double rate) {
    rate_ = rate;
    per_ = rate / hellDotsPerSecond;
    a_ = 1.0 - std::exp(-kTwoPi * 150.0 / rate);
}

void HellDecoder::process(const float* x, std::size_t n, double rate) {
    if (!(rate > 0) || !finite(rate) || !x) return;
    if (rate != rate_) setRate(rate);
    // Poisoned by a broken sample: rebuilt from scratch, keeping the columns.
    if (!finite(ph_ + lp_[0] + lp_[1] + lp_[2] + lp_[3] + acc_ + cnt_ + peak_ + contrast_ + pTone_ + pRef_[0] + pRef_[1] +
                refPh_[0] + refPh_[1] + refLp_[0][2] + refLp_[0][3] + refLp_[1][2] + refLp_[1][3] + ratio_)) rebuild();
    const double a = a_, w = kTwoPi * hellAudioHz / rate_;
    // Signal detector (addition to the JS): the same mixer and filter at
    // 500 and 1500 Hz measure the noise beside the tone.  Keyed lettering
    // puts its power at 1000 Hz; noise (whatever the receiver's AGC does to
    // it) has about as much power beside it as on it.
    const double wr[2] = {kTwoPi * (hellAudioHz - 500) / rate_, kTwoPi * (hellAudioHz + 500) / rate_};
    const double bp = 1.0 - std::exp(-1.0 / (0.3 * rate_));
    double* lp = lp_;
    for (std::size_t i = 0; i < n; ++i) {
        ph_ += w;
        if (ph_ > kTwoPi) ph_ -= kTwoPi;
        const double xi = x[i];
        lp[0] += a * (xi * std::cos(ph_) - lp[0]);
        lp[1] += a * (-xi * std::sin(ph_) - lp[1]);
        lp[2] += a * (lp[0] - lp[2]);
        lp[3] += a * (lp[1] - lp[3]);
        acc_ += std::sqrt(lp[2] * lp[2] + lp[3] * lp[3]);
        pTone_ += bp * (lp[2] * lp[2] + lp[3] * lp[3] - pTone_);
        for (int b = 0; b < 2; ++b) {
            double* r = refLp_[b];
            refPh_[b] += wr[b];
            if (refPh_[b] > kTwoPi) refPh_[b] -= kTwoPi;
            r[0] += a * (xi * std::cos(refPh_[b]) - r[0]);
            r[1] += a * (-xi * std::sin(refPh_[b]) - r[1]);
            r[2] += a * (r[0] - r[2]);
            r[3] += a * (r[1] - r[3]);
            pRef_[b] += bp * (r[2] * r[2] + r[3] * r[3] - pRef_[b]);
        }
        if (holdLeft_ > 0) holdLeft_ -= 1;
        if (++cnt_ >= per_) {
            const double v = acc_ / cnt_;
            acc_ = 0;
            cnt_ -= per_;
            // tone 6 dB over its surroundings: a signal; the flag holds 1 s
            // (through the gaps between words and overs)
            ratio_ = pTone_ / (0.5 * (pRef_[0] + pRef_[1]) + 1e-20);
            if (ratio_ > 4.0) holdLeft_ = rate_;
            peak_ = v > peak_ * 0.9995 ? v : peak_ * 0.9995;
            double d = v / peak_;
            if (!finite(d)) d = 0;   // (a NaN sample: the block's rest is dark)
            col_[ncol_++] = static_cast<float>(d);
            if (ncol_ == hellDotsPerColumn) {
                // (n counts every column ever made; pending stops growing at 600)
                ncol_ = 0;
                ++n_;
                float lo = col_[0], hi = col_[0];
                for (int r = 1; r < hellDotsPerColumn; ++r) {
                    if (col_[r] < lo) lo = col_[r];
                    if (col_[r] > hi) hi = col_[r];
                }
                contrast_ = contrast_ * 0.95 + (hi - lo) * 0.05;
                if (pending_.size() >= maxPending()) pending_.erase(pending_.begin());
                // Sent bottom dot first: stored top first (Decoder.h convention).
                pending_.push_back(std::vector<float>(col_, col_ + hellDotsPerColumn));
                std::vector<float>& c = pending_.back();
                for (int r = 0; r < hellDotsPerColumn / 2; ++r) std::swap(c[r], c[hellDotsPerColumn - 1 - r]);
            }
        }
    }
}

void HellDecoder::takeColumns(std::vector<std::vector<float> >& columns) {
    for (std::size_t i = 0; i < pending_.size(); ++i) columns.push_back(pending_[i]);
    pending_.clear();
}

Status HellDecoder::status() const {
    Status s;
    // (The JS shows only the strip.  "signal": the tone stood 6 dB over
    //  the noise beside it within the last second.  quality: how far over,
    //  0 at 6 dB, 1 at 20 dB and more.)
    s.signal = holdLeft_ > 0;
    s.locked = s.signal;
    const double db = 10 * std::log10(ratio_ + 1e-12);
    s.snrDb = db > 0 ? db : 0;
    s.quality = db <= 6 ? 0 : db >= 20 ? 1 : (db - 6) / 14;
    s.toneHz = hellAudioHz;
    s.baud = hellDotsPerSecond / 2;   // 122.5 baud (a dot is half a "symbol" pair)
    s.mode = "HELL";
    char buf[80];
    std::snprintf(buf, sizeof buf, "Feld Hell  %s  tone/noise %.0f dB  %lu columns", s.signal ? "receiving" : "searching", s.snrDb, n_);
    s.info = buf;
    return s;
}

}  // namespace dec
