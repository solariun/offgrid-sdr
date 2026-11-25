// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Fax.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  FaxDecoder: see Fax.h for the method.  A line-by-line port of
//  OFFgrid-SDR's JS FaxDecoder (diddy-boy,
//  https://github.com/solariun/offgrid-sdr); the JS comments are kept
//  (reworded) next to the code they explain.  Differences from the JS
//  (marked "Addition to the JS" below):
//    - automatic slant correction from the phasing lines, and a fixed one
//      (setSlantPpm);
//    - each output line is joined from two free-running rows (the JS
//      rotated each row on its own: right of the wrap point it showed the
//      line before), from an absolute line-start count (no line lost or
//      doubled when slant moves the start across a row's edge); a phasing
//      pulse straddling the row's edge starts the image one row earlier;
//    - startNow() allows for the discriminator's filter delay;
//    - lines are handed out (takeLines) ~1.25 s late instead of kept in an
//      array that the stop/start tone trimming cuts back; the trimming is
//      applied to the lines not yet handed out;
//    - the oscillators are rotating phasors (no cos/sin per sample);
//    - NaN/Inf samples are taken as silence; stop() ends a picture by hand;
//    - the JS kept the last 4000 lines; here rows simply keep counting.
//
//  How it fits in
//  --------------
//  Implements Fax.h.  Standard library only.
//
//  Contributing
//  ------------
//  No allocation per sample (a std::vector per output line).  Keep in step
//  with the JS; run tests/test_decoders_picture.cpp.
// ============================================================================
#include "Fax.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace dec {

namespace {

const double kTwoPi = 6.283185307179586;
const int W = faxWidth;

bool finite(double v) { return v == v && v - v == 0; }   // not NaN, not +-inf

}  // namespace

FaxDecoder::FaxDecoder()
    : lpm_(120), slantPpm_(0), autoSlant_(true), rate_(0), ncoR_(1), ncoI_(0), incR_(1), incI_(0), a_(0), pr_(1),
      pi_(0), renorm_(0), lineLen_(0), pxLen_(1), slantFactor_(1), pxAcc_(0), pxN_(0), row_(W, 0.0f), rowN_(0),
      sampleInLine_(0), c300R_(1), c300I_(0), w300R_(1), w300I_(0), c450R_(1), c450I_(0), w450R_(1), w450I_(0),
      s300R_(0), s300I_(0), s450R_(0), s450I_(0), win_(1), tw_(0), startRun_(0), stopRun_(0), startMark_(-1),
      stopMark_(-1), state_(Waiting), phaseAcc_(W, 0.0f), phRows_(static_cast<std::size_t>(kMaxPhaseRows) * W, 0.0f),
      phaseLines_(0), noPhase_(0), pendN_(0), offset_(0), phaseStrength_(0), autoPpm_(0), scratch_(W, 0.0f),
      rowIdx_(0), phaseBase_(0), nextStart_(0), manualPending_(false), prevRow_(W, 0.0f), havePrev_(false), prevIdx_(0), picture_(0), lines_(0), emitted_(0) {
    for (int i = 0; i < 3; ++i) zr_[i] = zi_[i] = 0;
    for (int i = 0; i < kMaxPhaseRows; ++i) phMed_[i] = 0;
    pend_[0] = pend_[1] = 0;
    reset();
}

void FaxDecoder::reset() {
    rate_ = 0;                      // (setup() on the next process())
    state_ = Waiting;
    offset_ = 0;
    phaseLines_ = noPhase_ = pendN_ = 0;
    startRun_ = stopRun_ = 0;
    startMark_ = stopMark_ = -1;
    phaseStrength_ = autoPpm_ = 0;
    slantFactor_ = 1;
    s300R_ = s300I_ = s450R_ = s450I_ = 0;
    c300R_ = c450R_ = 1;
    c300I_ = c450I_ = 0;
    havePrev_ = false;
    manualPending_ = false;
    picture_ = 0;
    lines_ = emitted_ = 0;
    held_.clear();
    out_.clear();
}

void FaxDecoder::clearFilters() {
    for (int k = 0; k < 3; ++k) zr_[k] = zi_[k] = 0;
    pr_ = 1; pi_ = 0;
    ncoR_ = 1; ncoI_ = 0;
}

void FaxDecoder::setup(double rate) {
    rate_ = rate;
    clearFilters();
    incR_ = std::cos(kTwoPi * faxAudioHz / rate);
    incI_ = std::sin(kTwoPi * faxAudioHz / rate);
    // complex low-pass (~1000 Hz) for the mixed-down signal: 3 cascaded one-pole sections
    a_ = 1 - std::exp(-kTwoPi * 1000 / rate);
    applyLineLen();                  // samples per line (24000 at 48 kHz, 120 LPM)
    pxAcc_ = 0; pxN_ = 0; rowN_ = 0;
    sampleInLine_ = 0;
    // (rows are numbered from here; a chart being received keeps its offset)
    rowIdx_ = 0;
    nextStart_ = offset_;
    havePrev_ = false;
    if (manualPending_) {
        manualPending_ = false;
        nextStart_ = filterDelayPx();
        offset_ = static_cast<int>(std::floor(nextStart_ + 0.5)) % W;
    }
    // tone detectors run on the discriminator output, in 0.25 s windows
    win_ = static_cast<long>(std::floor(rate * 0.25 + 0.5));
    if (win_ < 1) win_ = 1;
    tw_ = 0;
    s300R_ = s300I_ = s450R_ = s450I_ = 0;
    w300R_ = std::cos(kTwoPi * 300 / rate); w300I_ = std::sin(kTwoPi * 300 / rate);
    w450R_ = std::cos(kTwoPi * 450 / rate); w450I_ = std::sin(kTwoPi * 450 / rate);
}

void FaxDecoder::applyLineLen() {
    const double r = rate_ > 0 ? rate_ : 48000;
    // (Addition to the JS: the fixed and the measured slant corrections)
    lineLen_ = r * 60.0 / lpm_ * (1 + slantPpm_ * 1e-6) * slantFactor_;
    pxLen_ = lineLen_ / W;
}

bool FaxDecoder::setLpm(int lpm) {
    if (lpm != 60 && lpm != 90 && lpm != 120 && lpm != 240) return false;
    lpm_ = lpm;
    reset();
    return true;
}

void FaxDecoder::setSlantPpm(double ppm) {
    if (!finite(ppm) || std::fabs(ppm) > 5000) return;
    slantPpm_ = ppm;
    applyLineLen();
}

void FaxDecoder::process(const float* audio, std::size_t n, double rate) {
    if (!audio || !n || !finite(rate) || rate < 4000) return;
    if (rate != rate_) setup(rate);
    const double a = a_, k2pi = rate / kTwoPi;
    for (std::size_t i = 0; i < n; ++i) {
        double x = audio[i];
        if (!finite(x)) x = 0;      // (Addition to the JS: NaN/Inf taken as silence)
        double r = x * ncoR_, im = -x * ncoI_;
        const double t = ncoR_ * incR_ - ncoI_ * incI_;
        ncoI_ = ncoR_ * incI_ + ncoI_ * incR_;
        ncoR_ = t;
        for (int k = 0; k < 3; ++k) {
            zr_[k] += a * (r - zr_[k]);
            zi_[k] += a * (im - zi_[k]);
            r = zr_[k];
            im = zi_[k];
        }
        // instantaneous frequency from the phase change since the last sample
        const double dr = r * pr_ + im * pi_, di = im * pr_ - r * pi_;
        pr_ = r; pi_ = im;
        const double f = faxAudioHz + std::atan2(di, dr) * k2pi;
        double v = (f - 1500) / 800;   // 0 black .. 1 white
        v = v < 0 ? 0 : v > 1 ? 1 : v;
        toneStep(f - faxAudioHz);
        sample(v);
        if ((++renorm_ & 4095) == 0) {
            // keep the oscillators on the unit circle (rounding drifts them)
            double m = std::sqrt(ncoR_ * ncoR_ + ncoI_ * ncoI_); ncoR_ /= m; ncoI_ /= m;
            m = std::sqrt(c300R_ * c300R_ + c300I_ * c300I_); c300R_ /= m; c300I_ /= m;
            m = std::sqrt(c450R_ * c450R_ + c450I_ * c450I_); c450R_ /= m; c450I_ /= m;
        }
    }
    // (Addition to the JS: a filter state that went non-finite is cleared)
    bool ok = finite(pr_) && finite(pi_) && finite(ncoR_) && finite(ncoI_);
    for (int k = 0; k < 3; ++k) if (!finite(zr_[k]) || !finite(zi_[k])) ok = false;
    if (!ok) clearFilters();
}

// start (300 Hz) and stop (450 Hz) tones: the frequency swings between black
// and white at that rate
void FaxDecoder::toneStep(double d) {
    s300R_ += d * c300R_; s300I_ += d * c300I_;
    s450R_ += d * c450R_; s450I_ += d * c450I_;
    double t = c300R_ * w300R_ - c300I_ * w300I_;
    c300I_ = c300R_ * w300I_ + c300I_ * w300R_; c300R_ = t;
    t = c450R_ * w450R_ - c450I_ * w450I_;
    c450I_ = c450R_ * w450I_ + c450I_ * w450R_; c450R_ = t;
    if (++tw_ < win_) return;
    tw_ = 0;
    // Hz of swing at that rate
    const double a300 = 2 * std::sqrt(s300R_ * s300R_ + s300I_ * s300I_) / win_;
    const double a450 = 2 * std::sqrt(s450R_ * s450R_ + s450I_ * s450I_) / win_;
    s300R_ = s300I_ = s450R_ = s450I_ = 0;
    startRun_ = a300 > 250 ? startRun_ + 1 : 0;   // ~1 s of start tone
    stopRun_ = a450 > 250 ? stopRun_ + 1 : 0;
    // start tone heard for ~1 s: wait for it to end, then phasing lines follow
    // (also during a chart: if its stop tone was lost in a fade, the next
    //  chart's start tone ends it; the second or so of start tone already
    //  drawn is trimmed off first)
    if (startRun_ == 1) startMark_ = lines_;
    if (startRun_ >= 4 && (state_ == Waiting || state_ == Done || state_ == Receiving)) {
        if (state_ == Receiving) {
            if (startMark_ >= 0 && startMark_ <= lines_) trimTo(std::max(0, startMark_ - 1));
            endPicture();
        }
        state_ = StartTone;
    }
    if (state_ == StartTone && a300 < 150) beginPhasing();
    // stop tone: note where it began, then finish (the lines received during
    // it are dropped)
    if (stopRun_ == 1 && state_ == Receiving) stopMark_ = lines_;
    if (stopRun_ >= 4 && state_ == Receiving) finish();
}

void FaxDecoder::newPicture() {
    endPicture();
    ++picture_;
    lines_ = emitted_ = 0;
    havePrev_ = false;
}

void FaxDecoder::beginPhasing() {
    newPicture();                     // (the JS archived the old chart here: "new chart")
    state_ = Phasing;
    std::fill(phaseAcc_.begin(), phaseAcc_.end(), 0.0f);
    phaseLines_ = noPhase_ = pendN_ = 0;
    stopMark_ = -1;
    slantFactor_ = 1;                 // (measured again from these phasing lines)
    autoPpm_ = 0;
    applyLineLen();
}

// The discriminator's low-pass delays everything by ~3 x (1 - a) / a
// samples (1.6 px at 120 LPM); the phasing measures the line start through
// the same delay, but a manual start has nothing to measure.
double FaxDecoder::filterDelayPx() const {
    return a_ > 0 && pxLen_ > 0 ? 3 * (1 - a_) / a_ / pxLen_ : 0;
}

void FaxDecoder::startNow() {
    // manual start: no alignment (lines start with the free-running rows;
    // shift() lines them up)
    if (state_ != Receiving && state_ != Phasing) newPicture();
    state_ = Receiving;
    // (Addition to the JS: the filter delay allowed for, so a signal that
    //  starts with the rows comes out aligned)
    nextStart_ = static_cast<double>(rowIdx_) * W + filterDelayPx();
    offset_ = static_cast<int>(std::floor(filterDelayPx() + 0.5)) % W;
    manualPending_ = rate_ <= 0;      // (not set up yet: setup() places it)
    havePrev_ = false;
    pendN_ = 0;
}

void FaxDecoder::stop() {
    // (Addition to the JS: end the picture by hand)
    if (state_ == Receiving) { state_ = Done; endPicture(); }
    else if (state_ == Phasing || state_ == StartTone) state_ = Waiting;
    stopMark_ = -1;
}

void FaxDecoder::finish() {
    if (state_ == Receiving) {
        state_ = Done;
        if (stopMark_ >= 0) trimTo(std::max(0, stopMark_ - 1));
        endPicture();
    }
    stopMark_ = -1;
}

void FaxDecoder::shift(int dx) {   // manual alignment
    offset_ = ((offset_ + dx) % W + W) % W;
    nextStart_ += dx;
}

// pixels: average the samples for each pixel; rows are faxWidth pixels, one
// per line period
void FaxDecoder::sample(double v) {
    pxAcc_ += v;
    ++pxN_;
    sampleInLine_ += 1;
    if (sampleInLine_ >= pxLen_ * (rowN_ + 1)) {
        row_[rowN_++] = static_cast<float>(pxAcc_ / pxN_);
        pxAcc_ = 0;
        pxN_ = 0;
        if (rowN_ == W) {
            const double len = lineLen_;   // (the length this row was cut with; rowDone may change it)
            rowDone();
            ++rowIdx_;
            rowN_ = 0;
            sampleInLine_ -= len;
        }
    }
}

double FaxDecoder::segMean(const float* row, int start, int len) const {
    double s = 0;
    for (int k = 0; k < len; ++k) s += row[(start + k) % W];
    return s / len;
}

void FaxDecoder::rowDone() {
    if (state_ == Phasing) {
        // phasing lines: a short pulse marks the line start.  Its position in
        // our free-running rows is the offset to rotate every row by.
        // Accumulate a few lines, then look for a narrow feature.
        const int k = phaseLines_;
        if (k == 0) phaseBase_ = rowIdx_;
        float* keep = &phRows_[static_cast<std::size_t>(k < kMaxPhaseRows ? k : kMaxPhaseRows - 1) * W];
        std::copy(row_.begin(), row_.end(), keep);
        std::copy(row_.begin(), row_.end(), scratch_.begin());
        std::nth_element(scratch_.begin(), scratch_.begin() + (W >> 1), scratch_.end());
        const float med = scratch_[W >> 1];
        if (k < kMaxPhaseRows) phMed_[k] = med;
        for (int i = 0; i < W; ++i) phaseAcc_[i] += row_[i] - med;
        ++phaseLines_;
        // the phasing pattern ends when rows stop looking alike (image starts)
        if (phaseLines_ >= 6) {
            const int n = phaseLines_, win = static_cast<int>(std::floor(W * 0.05 + 0.5));
            const float* acc = &phaseAcc_[0];
            double best = 0, run = 0;
            int bi = 0;
            for (int i = 0; i < win; ++i) run += acc[i];
            for (int i = 0; i < W; ++i) {
                const double m = std::fabs(run);
                if (m > best) { best = m; bi = i; }
                run += acc[(i + win) % W] - acc[i];
            }
            offset_ = bi;                            // pulse starts here: that's the line start
            phaseStrength_ = best / (static_cast<double>(win) * n);   // how clearly the pulse stands out (0..1)
            // is this row still phasing? (the pulse region differs from the rest)
            const double p = segMean(&row_[0], bi, win), rest = segMean(&row_[0], (bi + win) % W, W - win);
            if (std::fabs(p - rest) < 0.35) {
                ++noPhase_;
                if (pendN_ < 2) pend_[pendN_++] = k;
            } else {
                noPhase_ = 0;
                pendN_ = 0;
            }
            if (noPhase_ >= 2 || phaseLines_ > 70) endPhasing();
        }
        return;
    }
    if (state_ != Receiving) return;
    pushRow(&row_[0], rowIdx_);
}

// The end of the phasing: measure the slant (Addition to the JS), then the
// image; the rows that already were the image go out first.
void FaxDecoder::endPhasing() {
    state_ = Receiving;
    const int win = static_cast<int>(std::floor(W * 0.05 + 0.5));
    const int bi = offset_;
    const int n = std::min(phaseLines_ - pendN_, static_cast<int>(kMaxPhaseRows));
    double slope = 0, kMean = (n - 1) / 2.0;
    if (autoSlant_ && n >= 8) {
        // (Addition to the JS: the slant.  The pulse's centre in each phasing
        //  row (its centroid within half a pulse width either side of the
        //  window found above) drifts by a constant number of pixels per
        //  line when the sender's and the receiver's sample clocks differ:
        //  fit a straight line through it (the first 2 rows, which may hold
        //  the end of the start tone, and the last, which may hold the
        //  image's first line, left out; one pass dropping rows more
        //  than 3 px off the line), and stretch the line length by the slope.)
        double sgnSum = 0;
        for (int j = 0; j < win; ++j) sgnSum += phaseAcc_[(bi + j) % W];
        const double sgn = sgnSum >= 0 ? 1.0 : -1.0;
        double pos[kMaxPhaseRows];
        bool use[kMaxPhaseRows];
        for (int k = 0; k < n; ++k) {
            use[k] = false;
            if (k < 2 || k >= n - 1) continue;   // (the last may hold the image's start)
            const float* row = &phRows_[static_cast<std::size_t>(k) * W];
            double sw = 0, sj = 0;
            for (int j = -win / 2; j < win + win / 2; ++j) {
                double w = sgn * (row[((bi + j) % W + W) % W] - phMed_[k]);
                if (w < 0) w = 0;
                sw += w;
                sj += j * w;
            }
            if (sw > 0.3 * win) { pos[k] = sj / sw; use[k] = true; }
        }
        for (int pass = 0; pass < 2; ++pass) {
            double s0 = 0, s1 = 0, s2 = 0, sy = 0, sxy = 0;
            for (int k = 0; k < n; ++k)
                if (use[k]) { s0 += 1; s1 += k; s2 += double(k) * k; sy += pos[k]; sxy += k * pos[k]; }
            const double den = s0 * s2 - s1 * s1;
            if (s0 < 6 || den <= 0) { slope = 0; break; }
            const double b = (s0 * sxy - s1 * sy) / den, c = (sy - b * s1) / s0;
            double ss = 0;
            for (int k = 0; k < n; ++k)
                if (use[k]) { const double e = pos[k] - (c + b * k); ss += e * e; }
            const double rms = std::sqrt(ss / s0);
            slope = b;
            if (pass == 0) {
                for (int k = 0; k < n; ++k)
                    if (use[k] && std::fabs(pos[k] - (c + b * k)) > 3) use[k] = false;
            } else if (rms > 2.5 || std::fabs(b) > W * 500e-6) {
                slope = 0;                // not a clean, plausible drift: leave it
            }
        }
        if (slope != 0) {
            slantFactor_ *= 1 + slope / W;
            autoPpm_ = (slantFactor_ - 1) * 1e6;
            applyLineLen();
        }
    }
    // where the first image line starts: the pulse's place in its row (bi is
    // where it was on average over the phasing rows, i.e. at row kMean)
    // (Addition to the JS: when the pulse window runs over the row's end,
    //  most of it is the start of the row, i.e. the line that began in the
    //  row before: that is the line the test above found to be the image.)
    const int kf = pendN_ > 0 ? pend_[0] : phaseLines_;
    const int ks = bi + win / 2 >= W ? kf - 1 : kf;      // the row the first line starts in
    nextStart_ = static_cast<double>(phaseBase_ + ks) * W + bi + slope * (ks - kMean);
    const double at = bi + slope * (phaseLines_ - kMean);
    offset_ = ((static_cast<int>(std::floor(at + 0.5)) % W) + W) % W;
    // the row before it (a line may start just before its row)
    havePrev_ = false;
    if (kf >= 1 && kf - 1 < kMaxPhaseRows) {
        const float* p = &phRows_[static_cast<std::size_t>(kf - 1) * W];
        std::copy(p, p + W, prevRow_.begin());
        prevIdx_ = phaseBase_ + kf - 1;
        havePrev_ = true;
    }
    // those rows were already the image
    for (int i = 0; i < pendN_; ++i) {
        const int k = pend_[i] < kMaxPhaseRows ? pend_[i] : kMaxPhaseRows - 1;
        pushRow(&phRows_[static_cast<std::size_t>(k) * W], phaseBase_ + k);
    }
    pendN_ = 0;
}

// (Addition to the JS: a line is taken from where it starts in one
//  free-running row to the same place in the next; the JS rotated a single
//  row, so the part right of the wrap point was the line before.  Line
//  starts are kept as an absolute pixel count, one line every faxWidth
//  pixels: when slant moves the start across a row's edge, a row holds two
//  line starts, or none, and no line is lost or doubled.)
void FaxDecoder::pushRow(const float* row, long long idx) {
    const bool prev = havePrev_ && prevIdx_ == idx - 1;
    const double base = static_cast<double>(idx - 1) * W;      // row idx-1's first pixel
    const double lo = prev ? base : base + W;                  // the oldest pixel we have
    if (nextStart_ < lo - 0.5) nextStart_ += W * std::ceil((lo - 0.5 - nextStart_) / W);   // (no data for those)
    while (nextStart_ + W <= base + 2 * W + 0.5) {
        int off = static_cast<int>(std::floor(nextStart_ - base + 0.5));
        off = off < 0 ? 0 : off > W ? W : off;
        emitLine(prev ? &prevRow_[0] : row, row, off);
        nextStart_ += W;
    }
    std::copy(row, row + W, prevRow_.begin());
    prevIdx_ = idx;
    havePrev_ = true;
}

void FaxDecoder::emitLine(const float* a, const float* b, int off) {
    PictureLine ln;
    ln.picture = picture_;
    ln.row = lines_;
    ln.rgb.resize(static_cast<std::size_t>(W) * 3);
    for (int i = 0; i < W; ++i) {
        const int j = i + off;
        const double v = j < W ? a[j] : b[j - W];
        const int g = static_cast<int>(std::floor(255 * v + 0.5));
        const unsigned char c = static_cast<unsigned char>(g < 0 ? 0 : g > 255 ? 255 : g);
        ln.rgb[i * 3] = ln.rgb[i * 3 + 1] = ln.rgb[i * 3 + 2] = c;
    }
    held_.push_back(ln);
    ++lines_;
    // (Addition to the JS: hand lines out ~1.25 s late, so the ones a
    //  stop/start tone spoils can still be dropped)
    const std::size_t hold = static_cast<std::size_t>(std::ceil(1.25 * lpm_ / 60.0)) + 1;
    while (held_.size() > hold) {
        out_.push_back(held_.front());
        held_.pop_front();
        ++emitted_;
    }
    if (out_.size() > 8192) out_.erase(out_.begin(), out_.begin() + (out_.size() - 8192));
}

void FaxDecoder::trimTo(int n) {
    if (n < emitted_) n = emitted_;   // (handed out already: can't be taken back)
    while (lines_ > n && !held_.empty()) { held_.pop_back(); --lines_; }
    havePrev_ = false;
}

void FaxDecoder::endPicture() {
    while (!held_.empty()) { out_.push_back(held_.front()); held_.pop_front(); }
    emitted_ = lines_;
    havePrev_ = false;
}

void FaxDecoder::takeLines(std::vector<PictureLine>& lines) {
    if (out_.empty()) return;
    lines.insert(lines.end(), out_.begin(), out_.end());
    out_.clear();
}

const char* FaxDecoder::stateName() const {
    switch (state_) {
    case Waiting: return "waiting";
    case StartTone: return "start tone";
    case Phasing: return "phasing";
    case Receiving: return "receiving";
    default: return "done";
    }
}

Status FaxDecoder::status() const {
    Status s;
    s.locked = state_ == Receiving;
    s.signal = state_ == StartTone || state_ == Phasing || state_ == Receiving;
    s.toneHz = faxAudioHz;
    s.shiftHz = 800;
    s.quality = phaseStrength_ < 0 ? 0 : phaseStrength_ > 1 ? 1 : phaseStrength_;
    s.pictureWidth = picture_ ? W : 0;
    s.pictureHeight = 0;
    s.pictureRow = lines_;
    char buf[160];
    std::snprintf(buf, sizeof buf, "FAX %d LPM", lpm_);
    s.mode = buf;
    if (state_ == Waiting) std::snprintf(buf, sizeof buf, "waiting for start tone");
    else if (state_ == StartTone) std::snprintf(buf, sizeof buf, "start tone");
    else if (state_ == Phasing) std::snprintf(buf, sizeof buf, "phasing  line %d", phaseLines_);
    else if (state_ == Receiving)
        std::snprintf(buf, sizeof buf, "receiving  line %d  offset %d  slant %+.0f ppm", lines_, offset_,
                      slantPpm_ + autoPpm_);
    else std::snprintf(buf, sizeof buf, "done  %d lines", lines_);
    s.info = buf;
    return s;
}

}  // namespace dec
