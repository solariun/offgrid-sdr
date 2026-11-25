// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Sstv.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The SSTV format table and SstvDecoder: see Sstv.h for the method.  A
//  line-by-line port of OFFgrid-SDR's JS SSTV_MODES, sstvLayout() and
//  SstvDecoder (diddy-boy, https://github.com/solariun/offgrid-sdr); the JS
//  comments are kept (reworded) next to the code they explain.  Differences
//  from the JS (marked "Addition to the JS" below), none of which change
//  what is decoded:
//    - the pitch history is a fixed ring (64 k values, ~5 s) instead of an
//      array that grows for the whole picture;
//    - the 1750 Hz oscillator is a rotating phasor (no cos/sin per sample);
//    - rows are handed out as PictureLines (takeLines) instead of painted
//      into an ImageData; a picture number per VIS code;
//    - a NaN/Inf sample is taken as silence, and a sample-rate change during
//      a picture ends that picture (the JS kept indexing the old rate's
//      history at the new rate).
//
//  How it fits in
//  --------------
//  Implements Sstv.h.  Standard library only.
//
//  Contributing
//  ------------
//  No allocation per sample.  Keep in step with the JS; run
//  tests/test_decoders_picture.cpp.
// ============================================================================
#include "Sstv.h"

#include <cmath>
#include <cstdio>

namespace dec {

namespace {

const double kTwoPi = 6.283185307179586;

bool finite(double v) { return v == v && v - v == 0; }   // not NaN, not +-inf

// The JS SSTV_MODES (VIS code: name, kind, size, component length in ms).
const SstvMode kModes[] = {
    {44, "Martin M1", SstvFamily::Martin, 320, 256, 146.432},
    {40, "Martin M2", SstvFamily::Martin, 320, 256, 73.216},
    {60, "Scottie S1", SstvFamily::Scottie, 320, 256, 138.240},
    {56, "Scottie S2", SstvFamily::Scottie, 320, 256, 88.064},
    {99, "PD90", SstvFamily::Pd, 320, 256, 170.240},
    {95, "PD120", SstvFamily::Pd, 640, 496, 121.600},
};
const int kNModes = static_cast<int>(sizeof kModes / sizeof kModes[0]);

int clamp255(double v) {
    const double r = std::floor(v + 0.5);
    return r < 0 ? 0 : r > 255 ? 255 : static_cast<int>(r);
}

}  // namespace

int sstvModeCount() { return kNModes; }
const SstvMode& sstvModeAt(int i) { return kModes[i < 0 ? 0 : i >= kNModes ? kNModes - 1 : i]; }
const SstvMode* sstvModeByVis(int vis) {
    for (int i = 0; i < kNModes; ++i)
        if (kModes[i].vis == vis) return &kModes[i];
    return 0;
}

// One line's layout: [ms offset, ms length, what] and the line length.
SstvLayout sstvLayout(const SstvMode& m) {
    SstvLayout L;
    const double c = m.compMs;
    L.nParts = 0;
    if (m.family == SstvFamily::Martin) {
        // sync 4.862, gap 0.572, then G, B, R each followed by a 0.572 gap
        double t = 0;
        L.parts[L.nParts++] = SstvPart{t, 4.862, kSstvSync};
        t += 4.862 + 0.572;
        const SstvComponent k[3] = {kSstvG, kSstvB, kSstvR};
        for (int i = 0; i < 3; ++i) { L.parts[L.nParts++] = SstvPart{t, c, k[i]}; t += c + 0.572; }
        L.lenMs = t; L.syncAtMs = 0; L.syncLenMs = 4.862;
    } else if (m.family == SstvFamily::Scottie) {
        // gap 1.5, G, gap 1.5, B, sync 9, gap 1.5, R (the sync sits mid-line)
        double t = 1.5;
        L.parts[L.nParts++] = SstvPart{t, c, kSstvG};
        t += c + 1.5;
        L.parts[L.nParts++] = SstvPart{t, c, kSstvB};
        t += c;
        const double sy = t;
        L.parts[L.nParts++] = SstvPart{t, 9, kSstvSync};
        t += 9 + 1.5;
        L.parts[L.nParts++] = SstvPart{t, c, kSstvR};
        t += c;
        L.lenMs = t; L.syncAtMs = sy; L.syncLenMs = 9;
    } else {
        // PD: sync 20, porch 2.08, then Y0 Cr Cb Y1
        double t = 0;
        L.parts[L.nParts++] = SstvPart{t, 20, kSstvSync};
        t += 22.08;
        const SstvComponent k[4] = {kSstvY0, kSstvCr, kSstvCb, kSstvY1};
        for (int i = 0; i < 4; ++i) { L.parts[L.nParts++] = SstvPart{t, c, k[i]}; t += c; }
        L.lenMs = t; L.syncAtMs = 0; L.syncLenMs = 20;
    }
    return L;
}

// ===========================================================================
// SstvDecoder
// ===========================================================================
SstvDecoder::SstvDecoder()
    : rate_(0), fs_(12000), a_(0), D_(1), k_(0), phR_(1), phI_(0), wR_(1), wI_(0), pzR_(0), pzI_(0), havePz_(false),
      lastF_(1900), F_(kRing, 1900.0f), n_(-1), oldest_(0), st_(Hunt), lead_(0), startRun_(0), sm_(0), smValid_(false),
      visAt_(0), mode_(0), t0_(0), drift_(0), miss_(0), line_(0), rows_(0), picture_(0) {
    for (int i = 0; i < 8; ++i) comp_[i].assign(640, 0.0f);   // the widest format
    L_ = sstvLayout(kModes[0]);
    reset();
}

void SstvDecoder::clearPitch() {
    for (int i = 0; i < 8; ++i) lp_[i] = 0;
    phR_ = 1; phI_ = 0;
    havePz_ = false;
    k_ = 0;
    lastF_ = 1900;
}

void SstvDecoder::reset() {
    clearPitch();
    st_ = Hunt;
    n_ = -1;
    oldest_ = 0;
    lead_ = startRun_ = 0;
    smValid_ = false;
    sm_ = 0;
    mode_ = 0;
    drift_ = 0;
    miss_ = line_ = rows_ = 0;
    picture_ = 0;
    out_.clear();
}

// Decimate to ~12 kHz; the low-pass is 900 Hz (4 stages).
void SstvDecoder::setRate(double rate) {
    rate_ = rate;
    D_ = static_cast<int>(std::floor(rate / 12000 + 0.5));
    if (D_ < 1) D_ = 1;
    fs_ = rate / D_;
    a_ = 1 - std::exp(-kTwoPi * 900 / rate);
    const double w = kTwoPi * 1750 / rate;
    wR_ = std::cos(w); wI_ = std::sin(w);
    k_ = 0;
}

long SstvDecoder::ms(double x) const { return static_cast<long>(std::floor(x * fs_ / 1000 + 0.5)); }

float SstvDecoder::at(long long i) const {
    // (Addition to the JS: indices before the kept history read its oldest
    //  value, after the newest its newest; the JS read undefined -> NaN)
    if (i < oldest_) i = oldest_;
    if (i > n_) i = n_;
    if (i < 0) return 1900.0f;
    return F_[static_cast<std::size_t>(i & (kRing - 1))];
}

double SstvDecoder::mean(long long s, long long e) const {
    if (e <= s) return at(s);
    double t = 0;
    for (long long k = s; k < e; ++k) t += at(k);
    return t / static_cast<double>(e - s);
}

void SstvDecoder::process(const float* audio, std::size_t n, double rate) {
    if (!audio || !n || !finite(rate) || rate < 4000) return;
    if (rate != rate_) {
        // (Addition to the JS: a picture under way can't continue at another
        //  rate: its history is indexed in the old rate's samples)
        if (st_ != Hunt) end();
        setRate(rate);
    }
    const double a = a_;
    double* lp = lp_;
    for (std::size_t i = 0; i < n; ++i) {
        // pitch: mix 1750 Hz to 0, low-pass, then the phase step between
        // decimated samples
        const double t = phR_ * wR_ - phI_ * wI_;
        phI_ = phR_ * wI_ + phI_ * wR_;
        phR_ = t;
        // (four low-pass stages at 900 Hz: mixing also makes a mirror copy of
        //  the tone ~3 kHz away, which two stages left strong enough to drag
        //  the measured pitch towards the middle)
        double v = audio[i];
        if (!finite(v)) v = 0;      // (Addition to the JS: NaN/Inf taken as silence)
        lp[0] += a * (v * phR_ - lp[0]);
        lp[1] += a * (-v * phI_ - lp[1]);
        for (int s = 2; s < 8; s += 2) {
            lp[s] += a * (lp[s - 2] - lp[s]);
            lp[s + 1] += a * (lp[s - 1] - lp[s + 1]);
        }
        if (++k_ < D_) continue;
        k_ = 0;
        const double zr = lp[6], zi = lp[7];
        double f = lastF_;
        if (havePz_) {
            const double dr = zr * pzR_ + zi * pzI_, di = zi * pzR_ - zr * pzI_;
            if (dr * dr + di * di > 1e-24) f = 1750 + std::atan2(di, dr) * fs_ / kTwoPi;
        }
        pzR_ = zr; pzI_ = zi; havePz_ = true;
        f = f < 900 ? 900 : f > 2600 ? 2600 : f;
        lastF_ = f;
        step(f);
    }
    // keep the oscillator on the unit circle (rounding drifts it)
    const double m = std::sqrt(phR_ * phR_ + phI_ * phI_);
    if (m > 0 && finite(m)) { phR_ /= m; phI_ /= m; } else { phR_ = 1; phI_ = 0; }
    // (Addition to the JS: a filter state that went non-finite is cleared)
    bool ok = true;
    for (int s = 0; s < 8; ++s) if (!finite(lp[s])) ok = false;
    if (!ok || !finite(lastF_)) { clearPitch(); if (st_ != Hunt) end(); }
}

void SstvDecoder::step(double f) {
    ++n_;
    F_[static_cast<std::size_t>(n_ & (kRing - 1))] = static_cast<float>(f);
    if (n_ - oldest_ >= kRing) oldest_ = n_ - kRing + 1;
    const long long n = n_;

    if (st_ == Hunt) {
        // (the header detector uses the pitch smoothed over ~5 ms: in noise
        //  the raw pitch jitters and kept breaking the long 1900 Hz leader run;
        //  F keeps the raw pitch)
        sm_ = smValid_ ? sm_ + (f - sm_) / ms(5) : f;
        smValid_ = true;
        f = sm_;
        if (std::fabs(f - 1900) < 110) { ++lead_; startRun_ = 0; }
        else if (std::fabs(f - 1200) < 110 && lead_ > ms(200)) {
            if (++startRun_ >= ms(18)) {
                // the smoothed pitch is late by a few ms: find the exact
                // 1900 -> 1200 Hz edge in the raw pitch, as the whole
                // picture's timing hangs on it (the steepest drop within
                // -15..+10 ms)
                const long long guess = n - startRun_ + 1;
                const long w10 = ms(8);
                long long best = guess;
                double bd = -1e9;
                for (long long e = guess - ms(15); e <= guess + ms(10); ++e) {
                    if (e - w10 < oldest_ || e + w10 > n) continue;
                    double sa = 0, sb = 0;
                    for (long long k = e - w10; k < e; ++k) sa += at(k);
                    for (long long k = e; k < e + w10; ++k) sb += at(k);
                    if (sa - sb > bd) { bd = sa - sb; best = e; }
                }
                visAt_ = best;
                st_ = Vis;
            }
        } else {
            if (std::fabs(f - 1200) >= 110) lead_ = lead_ - 2 > 0 ? lead_ - 2 : 0;
            startRun_ = 0;
        }
        return;
    }

    if (st_ == Vis) {
        if (n < visAt_ + ms(300)) return;
        int code = 0, ones = 0;
        for (int b = 0; b < 8; ++b) {
            const long long s = visAt_ + ms(30 * (b + 1) + 6);
            const int bit = mean(s, s + ms(18)) < 1200 ? 1 : 0;
            if (b < 7) code |= bit << b;
            ones += bit;
        }
        const SstvMode* m = sstvModeByVis(code);
        if (!m || (ones % 2)) {   // not a VIS we know (or a parity error)
            st_ = Hunt; lead_ = 0; startRun_ = 0;
            return;
        }
        mode_ = m;
        L_ = sstvLayout(*m);
        t0_ = static_cast<double>(visAt_ + ms(300) + (m->family == SstvFamily::Scottie ? ms(9) : 0));
        ++picture_;
        drift_ = 0; miss_ = 0; line_ = 0; rows_ = 0;
        st_ = Image;
        return;
    }

    // image: decode each line once its samples (plus a little to search for
    // its sync) have arrived
    const double Ls = L_.lenMs * fs_ / 1000;
    while (st_ == Image) {
        const double start = t0_ + line_ * Ls + drift_;
        if (static_cast<double>(n) < start + Ls + ms(3)) return;
        // find this line's sync pulse near where it should be (re-aligns for slant)
        const long long e = static_cast<long long>(std::floor(start + L_.syncAtMs * fs_ / 1000 + 0.5));
        const long sl = ms(L_.syncLenMs), r = ms(1.5);
        long long best = e;
        double bv = 1e9;
        for (long long s = e - r; s <= e + r; ++s) {
            if (s < 0) continue;
            const double t = mean(s, s + sl);
            if (t < bv) { bv = t; best = s; }
        }
        if (bv < 1400) { drift_ += (best - e) * 0.5; miss_ = 0; }
        else if (++miss_ > 6) { end(); return; }
        decodeLine();
        if (line_ >= sstvLinesPerPicture(*mode_)) { end(); return; }
    }
}

// Paint line line_ (one row, or two for PD) and hand it out.
void SstvDecoder::decodeLine() {
    const SstvMode& m = *mode_;
    const double Ls = L_.lenMs * fs_ / 1000;
    const double st2 = t0_ + line_ * Ls + drift_;
    for (int p = 0; p < L_.nParts; ++p) {
        const SstvPart& part = L_.parts[p];
        if (part.what == kSstvSync) continue;
        std::vector<float>& vals = comp_[part.what];
        const double s0 = st2 + part.offMs * fs_ / 1000, dt = part.lenMs * fs_ / 1000 / m.width;
        for (int x = 0; x < m.width; ++x) {
            const long long a0 = static_cast<long long>(std::floor(s0 + x * dt + 0.5));
            long long a1 = static_cast<long long>(std::floor(s0 + (x + 1) * dt + 0.5));
            if (a1 < a0 + 1) a1 = a0 + 1;
            vals[x] = static_cast<float>(sstvValue(mean(a0, a1)));
        }
    }
    if (m.family == SstvFamily::Pd) {
        PictureLine r0, r1;
        r0.picture = r1.picture = picture_;
        r0.row = 2 * line_;
        r1.row = 2 * line_ + 1;
        r0.rgb.resize(m.width * 3);
        r1.rgb.resize(m.width * 3);
        for (int x = 0; x < m.width; ++x) {
            const double cr = comp_[kSstvCr][x] - 128.0, cb = comp_[kSstvCb][x] - 128.0;
            for (int h = 0; h < 2; ++h) {
                const double yv = 1.164 * ((h ? comp_[kSstvY1][x] : comp_[kSstvY0][x]) - 16.0);
                unsigned char* px = &(h ? r1 : r0).rgb[x * 3];
                px[0] = static_cast<unsigned char>(clamp255(yv + 1.596 * cr));
                px[1] = static_cast<unsigned char>(clamp255(yv - 0.813 * cr - 0.391 * cb));
                px[2] = static_cast<unsigned char>(clamp255(yv + 2.018 * cb));
            }
        }
        out_.push_back(r0);
        out_.push_back(r1);
        rows_ += 2;
    } else {
        PictureLine r;
        r.picture = picture_;
        r.row = line_;
        r.rgb.resize(m.width * 3);
        for (int x = 0; x < m.width; ++x) {
            r.rgb[x * 3 + 0] = static_cast<unsigned char>(clamp255(comp_[kSstvR][x]));
            r.rgb[x * 3 + 1] = static_cast<unsigned char>(clamp255(comp_[kSstvG][x]));
            r.rgb[x * 3 + 2] = static_cast<unsigned char>(clamp255(comp_[kSstvB][x]));
        }
        out_.push_back(r);
        rows_ += 1;
    }
    ++line_;
    // (Addition to the JS: a program that never calls takeLines() must not
    //  grow this without bound; the oldest rows go first)
    if (out_.size() > 4096) out_.erase(out_.begin(), out_.begin() + (out_.size() - 4096));
}

void SstvDecoder::end() {
    st_ = Hunt;
    oldest_ = n_ + 1;              // (the JS emptied F)
    lead_ = 0;
    startRun_ = 0;
    smValid_ = false;
}

void SstvDecoder::takeLines(std::vector<PictureLine>& lines) {
    if (out_.empty()) return;
    lines.insert(lines.end(), out_.begin(), out_.end());
    out_.clear();
}

Status SstvDecoder::status() const {
    Status s;
    s.locked = st_ == Image;
    s.signal = st_ != Hunt;
    s.toneHz = lastF_;
    if (mode_) {
        s.mode = mode_->name;
        s.pictureWidth = mode_->width;
        s.pictureHeight = mode_->height;
        s.pictureRow = rows_;
        s.quality = st_ == Image ? (miss_ == 0 ? 1.0 : 1.0 - miss_ / 7.0) : 0;
    }
    char buf[128];
    if (st_ == Image)
        std::snprintf(buf, sizeof buf, "%s  picture %u  row %d/%d  slant %+.1f", mode_->name, picture_, rows_,
                      mode_->height, drift_);
    else if (st_ == Vis)
        std::snprintf(buf, sizeof buf, "VIS code...");
    else if (mode_)
        std::snprintf(buf, sizeof buf, "waiting for VIS  (last: %s, %d rows)", mode_->name, rows_);
    else
        std::snprintf(buf, sizeof buf, "waiting for VIS");
    s.info = buf;
    return s;
}

}  // namespace dec
