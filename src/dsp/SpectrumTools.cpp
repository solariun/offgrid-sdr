// ============================================================================
//  offgrid-dsp - reusable C++11 radio signal processing (part of OFFgrid-SDR
//  TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/SpectrumTools.cpp
//  Layer:  Library liboffgrid-dsp
//
//  What this file does
//  -------------------
//  SpectrumExtras (see SpectrumTools.h): a port of the JS's specExtras
//  (diddy-boy, index.html), times in seconds instead of ms.
//
//  How it fits in
//  --------------
//  Standard library only.
//
//  Contributing
//  ------------
//  The bracketed comments are the JS's reasons.
// ============================================================================
#include "SpectrumTools.h"

#include <algorithm>
#include <cmath>

namespace dsp {

namespace {
// (zero power is -Infinity dB: averaging with it would poison a pixel)
inline float clean(float x) { return x == x && std::fabs(x) < 1e30f ? x : -200.0f; }
inline bool finite(float x) { return x == x && std::fabs(x) < 1e30f; }
const double kQuiet = 120;  // s: a spot empty this long counts as empty
}  // namespace

SpectrumExtras::SpectrumExtras()
    : avgOn_(false), peakOn_(false), alertsOn_(false), haveKey_(false), haveLast_(false), haveFrame_(false), keyLo_(0),
      keyHi_(0), keyW_(0), learnUntil_(0), lastFrame_(0), frames_(0), count_(0) {}

void SpectrumExtras::restart() { haveKey_ = false; }

void SpectrumExtras::frame(const float* pix, int W, double lo, double hi, double now, double pa, double pb) {
    fresh_.clear();
    if (W <= 0 || !pix) return;
    const double rlo = std::floor(lo + 0.5), rhi = std::floor(hi + 0.5);
    if (!haveKey_ || keyLo_ != rlo || keyHi_ != rhi || keyW_ != W) {
        // a new view: start afresh, and learn the band for 10 s
        haveKey_ = true;
        keyLo_ = rlo;
        keyHi_ = rhi;
        keyW_ = W;
        haveLast_ = false;
        avg_.resize(W);
        peak_.resize(W);
        sm_.resize(W);
        for (int x = 0; x < W; ++x) avg_[x] = peak_[x] = sm_[x] = clean(pix[x]);
        hit_.assign(W, 0);
        learnUntil_ = now + 10;
    }
    const double fps = 1.0 / std::max(0.008, now - (haveFrame_ ? lastFrame_ : now - 0.016));
    lastFrame_ = now;
    haveFrame_ = true;
    for (int x = 0; x < W; ++x) {
        const float p = clean(pix[x]);
        if (avgOn_) avg_[x] = avg_[x] * 0.8f + p * 0.2f;
        if (peakOn_) peak_[x] = std::max(static_cast<float>(peak_[x] - 3 / fps), p);  // falls ~3 dB a second
    }
    if (!alertsOn_) return;
    if (!haveLast_) {
        last_.assign(W, -1e12);
        pres_.assign(W, 0.0f);
        frames_ = 0;
        haveLast_ = true;
    }
    {
        int dead = 0;
        for (int x = 0; x < W; ++x)
            if (!finite(pix[x])) dead++;
        if (dead > W * 0.3) return;
    }
    const bool learning = now < learnUntil_;
    srt_.clear();
    for (int x = 0; x < W; ++x)
        if (finite(pix[x])) srt_.push_back(pix[x]);
    float floor = -120;
    if (!srt_.empty()) {
        std::nth_element(srt_.begin(), srt_.begin() + srt_.size() / 2, srt_.end());
        floor = srt_[srt_.size() / 2];
    }
    cands_.clear();
    if (learning) frames_++;
    for (int x = 0; x < W; ++x) {
        sm_[x] = sm_[x] * 0.6f + clean(pix[x]) * 0.4f;
        const bool on = sm_[x] > floor + 9;
        if (learning) {
            if (on) pres_[x]++;
            hit_[x] = 0;
            continue;
        }
        if (!on) {
            hit_[x] = 0;
            continue;
        }
        double lastNear = -1e12;
        for (int k = std::max(0, x - 2); k <= std::min(W - 1, x + 2); ++k) lastNear = std::max(lastNear, last_[k]);
        if (now - lastNear > kQuiet) {
            if (hit_[x] < 255) hit_[x]++;
            if (hit_[x] >= 3) cands_.push_back(x);
        } else {
            hit_[x] = 0;
            last_[x] = now;
        }
    }
    if (learning) return;
    if (frames_) {  // learning just ended: mark the spots occupied most of the time as known
        for (int x = 0; x < W; ++x)
            if (pres_[x] >= frames_ * 0.5) last_[x] = now;
        frames_ = 0;
    }
    {
        std::size_t k = 0;
        for (std::size_t i = 0; i < seen_.size(); ++i)
            if (now - seen_[i].t < 600) seen_[k++] = seen_[i];
        seen_.resize(k);
    }
    if (cands_.empty()) return;
    // group neighbouring pixels: one alert per signal, at its strongest pixel
    struct G {
        int x, end;
    };
    std::vector<G> groups;
    for (std::size_t i = 0; i < cands_.size(); ++i) {
        const int x = cands_[i];
        if (!groups.empty() && x - groups.back().end <= 2) {
            groups.back().end = x;
            if (pix[x] > pix[groups.back().x]) groups.back().x = x;
        } else {
            G g = {x, x};
            groups.push_back(g);
        }
    }
    const double hzPerPx = (hi - lo) / W;
    // (a strong new signal brings weak spurs and leakage with it: "new signals" appearing at the same moment more
    //  than 20 dB weaker than the strongest are its offspring, not stations)
    float top = -1e30f;
    for (std::size_t i = 0; i < groups.size(); ++i) top = std::max(top, sm_[groups[i].x]);
    for (std::size_t i = 0; i < groups.size(); ++i) {
        const G& g = groups[i];
        if (sm_[g.x] < top - 20) continue;
        for (int x = std::max(0, g.x - 3); x <= std::min(W - 1, g.end + 3); ++x) {
            hit_[x] = 0;
            last_[x] = now;
        }
        const double f = lo + (g.x + 0.5) * hzPerPx;
        if (f >= pa - hzPerPx * 2 && f <= pb + hzPerPx * 2) continue;  // (that's the station you're on)
        // (a spot flashes again only after it has been quiet: while active, each frame refreshes its time)
        SpectrumAlert a = {f, now, sm_[g.x]};
        seen_.push_back(a);
        fresh_.push_back(a);
        count_++;
    }
}

}  // namespace dsp
