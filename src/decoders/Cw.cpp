// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Cw.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The Morse decoder described in Cw.h.  A line-by-line port of
//  OFFgrid-SDR's CwDecoder and its helpers cwFit/cwNear (diddy-boy,
//  index.html); comments in brackets are the browser version's reasoning.
//  Differences from the browser version:
//    - the optional tone filter in front (Cw.h, step 0);
//    - times are counted in audio (ms of samples processed), not wall time;
//    - the text is streamed: takeText() returns what is new, with trailing
//      spaces held back so that a line end can still remove them.
//
//  How it fits in
//  --------------
//  See Cw.h.
//
//  Contributing
//  ------------
//  Keep the structure parallel to the JavaScript (same function names:
//  step, keyStep, record, mark, check, space, learnGap, learnSpeed, emit,
//  newLine, put), so fixes can be carried across in either direction.
// ============================================================================
#include "Cw.h"
#include "Codes.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace dec {

namespace {

const double kPi = 3.14159265358979323846;
const int kHist = 3000;       // 6 s of 2 ms levels
const int kFftN = 2048;       // tone search: ~0.25 s at ~8 kHz

// How well a ratio fits k (within a factor 1.55 either way).
inline bool cwNear(double r, double k) { return r > k / 1.55 && r < k * 1.55; }

// 2-means on a set of logs: two centres, nominally dots and dashes (or
// within-letter and letter gaps).
void twoMeans(const std::vector<double>& lg, double& c1, double& c2) {
    c1 = *std::min_element(lg.begin(), lg.end());
    c2 = *std::max_element(lg.begin(), lg.end());
    for (int it = 0; it < 12; ++it) {
        double s1 = 0, s2 = 0;
        int n1 = 0, n2 = 0;
        for (std::size_t i = 0; i < lg.size(); ++i) {
            const double v = lg[i];
            if (std::fabs(v - c1) <= std::fabs(v - c2)) { s1 += v; ++n1; }
            else { s2 += v; ++n2; }
        }
        if (n1) c1 = s1 / n1;
        if (n2) c2 = s2 / n2;
    }
}

}  // namespace

CwDecoder::CwDecoder()
    : toneFilter_(true), filterWidth_(250), fixedTone_(0), rate_(0), stepN_(1), powA_(0), lpA_(0),
      fft_(kFftN) {
    reset();
}

void CwDecoder::setFilterWidth(double hz) {
    filterWidth_ = std::max(100.0, std::min(1000.0, hz));
    if (rate_ > 0) lpA_ = 1 - std::exp(-2 * kPi * (filterWidth_ / 2) / rate_);
}

void CwDecoder::reset() {
    toneHz_ = fixedTone_ > 0 ? fixedTone_ : 0;
    oscRe_ = 1; oscIm_ = 0; rotRe_ = 1; rotIm_ = 0; rotFor_ = -1;
    lp1_ = lp2_ = std::complex<double>(0, 0);
    oscCount_ = 0;
    decimN_ = 0; decimAcc_ = 0;
    tbuf_.assign(kFftN, 0.f);
    tbufPos_ = 0; tbufSince_ = 0;
    spec_.assign(kFftN / 2, 0.0);
    specInit_ = false;
    fftBuf_.resize(kFftN);

    pw_ = 0; accN_ = 0; env_ = 0; hi_ = 0; lo_ = 0; t_ = 0;
    hist_.assign(kHist, 0.f);
    hn_ = 0; hcount_ = 0;
    present_ = false; quietFor_ = 0; signal_ = false; snrDb_ = 0;
    key_ = false; pend_ = false; pendAt_ = 0; runStart_ = 0;
    mbuf_.clear();
    sym_.clear();
    dot_ = 60; sdot_ = 0; markThr_ = 0; wpm_ = 0;
    marks_.clear(); events_.clear(); recent_.clear();
    // (until enough marks have been heard to judge the speed, keep the raw
    //  timings and decode nothing; then decode them all at once)
    learning_ = true; validated_ = false; wordSpaced_ = true; lineBroken_ = true;
    lastMarkAt_ = -1e9;
    chars_ = 0;
    out_.clear(); heldSpaces_ = 0; lastChar_ = 0;
}

void CwDecoder::setRate(double rate) {
    rate_ = rate;
    stepN_ = std::max(1, static_cast<int>(std::floor(rate * 0.002 + 0.5)));
    powA_ = 1 - std::exp(-1 / (rate * 0.003));
    lpA_ = 1 - std::exp(-2 * kPi * (filterWidth_ / 2) / rate);
    decimD_ = std::max(1, static_cast<int>(std::floor(rate / 8000 + 0.5)));
    rotFor_ = -1;
    if (accN_ >= stepN_) accN_ = 0;
    if (decimN_ >= decimD_) { decimN_ = 0; decimAcc_ = 0; }
}

// The tone: average power spectra of the last ~0.25 s (decimated to about
// 8 kHz), take the strongest peak in 250-1500 Hz if it stands well clear of
// the median.  Moves only for a clearly different tone (> 25 Hz).
void CwDecoder::trackTone() {
    const int N = kFftN;
    for (int i = 0; i < N; ++i) {
        const double w = 0.5 - 0.5 * std::cos(2 * kPi * i / N);
        fftBuf_[i] = std::complex<float>(static_cast<float>(tbuf_[(tbufPos_ + i) % N] * w), 0.f);
    }
    fft_.forward(&fftBuf_[0]);
    for (int j = 0; j < N / 2; ++j) {
        const double p = std::norm(fftBuf_[j]);
        spec_[j] = specInit_ ? spec_[j] + 0.3 * (p - spec_[j]) : p;
    }
    specInit_ = true;
    if (fixedTone_ > 0) { toneHz_ = fixedTone_; return; }
    const double fs = rate_ / decimD_, bin = fs / N;
    const int lo = static_cast<int>(std::ceil(250 / bin)), hi = std::min(N / 2 - 2, static_cast<int>(1500 / bin));
    if (hi <= lo + 2) return;
    scratch_.assign(spec_.begin() + lo, spec_.begin() + hi);
    std::nth_element(scratch_.begin(), scratch_.begin() + scratch_.size() / 2, scratch_.end());
    const double med = std::max(scratch_[scratch_.size() / 2], 1e-30);
    int best = -1;
    for (int j = lo + 1; j < hi; ++j)
        if (spec_[j] >= spec_[j - 1] && spec_[j] >= spec_[j + 1] && (best < 0 || spec_[j] > spec_[best])) best = j;
    if (best < 0 || spec_[best] < med * 12) return;
    // parabolic interpolation between bins
    const double a = spec_[best - 1], b = spec_[best], c = spec_[best + 1], den = a - 2 * b + c;
    const double off = den != 0 ? 0.5 * (a - c) / den : 0;
    const double f = (best + std::max(-0.5, std::min(0.5, off))) * bin;
    if (toneHz_ <= 0 || std::fabs(f - toneHz_) > 25) toneHz_ = f;
    else toneHz_ += 0.2 * (f - toneHz_);
}

void CwDecoder::process(const float* x, std::size_t n, double rate) {
    if (rate <= 0) return;
    if (rate != rate_) setRate(rate);
    const double dt = stepN_ / rate_ * 1000, a = powA_;
    double pw = pw_;
    for (std::size_t i = 0; i < n; ++i) {
        const double v = x[i];
        // tone search buffer (decimated by averaging)
        decimAcc_ += v;
        if (++decimN_ >= decimD_) {
            tbuf_[tbufPos_] = static_cast<float>(decimAcc_ / decimD_);
            tbufPos_ = (tbufPos_ + 1) % kFftN;
            decimAcc_ = 0; decimN_ = 0;
            if (++tbufSince_ >= kFftN / 2) { tbufSince_ = 0; trackTone(); }
        }
        double p;
        if (toneFilter_ && toneHz_ > 0) {
            if (rotFor_ != toneHz_) {
                const double w = 2 * kPi * toneHz_ / rate_;
                rotRe_ = std::cos(w); rotIm_ = -std::sin(w); rotFor_ = toneHz_;
            }
            // local oscillator (a rotating phasor, renormalised now and then)
            const double r = oscRe_ * rotRe_ - oscIm_ * rotIm_;
            oscIm_ = oscRe_ * rotIm_ + oscIm_ * rotRe_; oscRe_ = r;
            if (++oscCount_ >= 1024) {
                oscCount_ = 0;
                const double m = std::sqrt(oscRe_ * oscRe_ + oscIm_ * oscIm_);
                oscRe_ /= m; oscIm_ /= m;
            }
            lp1_ += lpA_ * (std::complex<double>(v * oscRe_, v * oscIm_) - lp1_);
            lp2_ += lpA_ * (lp1_ - lp2_);
            p = 2 * std::norm(lp2_);   // (x 2: a tone of amplitude A reads A^2/2, as unfiltered)
        } else {
            p = v * v;
        }
        // tone level: a smooth running average of power (3 ms), read every 2 ms
        pw += a * (p - pw);
        if (++accN_ >= stepN_) { accN_ = 0; env_ = std::sqrt(pw); step(env_, dt); }
    }
    pw_ = pw;
}

void CwDecoder::step(double e, double dt) {
    t_ += dt;
    // Levels from the spread of the last 2 s: the 20th percentile is the key-up (noise) level,
    // the 90th the key-down level. No clear gap between them means no Morse: stay quiet.
    hist_[hn_] = static_cast<float>(e);
    hn_ = (hn_ + 1) % kHist;
    if (hcount_ < kHist) ++hcount_;
    // Fast mode when no signal is established, or its key-down level has faded: refresh every
    // 20 ms (10 steps) instead of 100 ms.
    const bool fast = !present_ || hi_ / std::max(lo_, 1e-12) < 2.8;
    if (hcount_ >= 100 && (hn_ % (fast ? 10 : 50) == 0 || hi_ == 0)) {
        // look back 30 dots (2-6 s): slow Morse needs a longer view
        const int want = std::min(hcount_, std::max(1000, std::min(3000, static_cast<int>(std::floor(30 * dot_ / 2 + 0.5)))));
        sortBuf_.resize(want);
        for (int k = 0; k < want; ++k) sortBuf_[k] = hist_[(hn_ - 1 - k + 2 * kHist) % kHist];
        const std::size_t i20 = static_cast<std::size_t>(want * 0.2), i90 = static_cast<std::size_t>(want * 0.9),
                          i97 = static_cast<std::size_t>(want * 0.97);
        std::nth_element(sortBuf_.begin(), sortBuf_.begin() + i90, sortBuf_.end());
        const double q90 = sortBuf_[i90];
        std::nth_element(sortBuf_.begin() + i90, sortBuf_.begin() + i97, sortBuf_.end());
        const double q97 = sortBuf_[i97];
        std::nth_element(sortBuf_.begin(), sortBuf_.begin() + i20, sortBuf_.begin() + i90);
        const double q20 = sortBuf_[i20];
        // key-down level: the 90th percentile when the history shows two levels, else the loudest 3%
        hi_ = q90 / std::max(std::max(q20, q90 / 30), 1e-12) >= 2.8 ? q90 : q97;
        lo_ = std::max(q20, hi_ / 30);   // at most 30 dB of range
    }
    const double ratio = hi_ / std::max(lo_, 1e-12);
    snrDb_ = 20 * std::log10(std::max(ratio, 1.0));
    // present above ~12 dB, lost below ~9 dB for more than a second (hysteresis)
    if (ratio > 4) { present_ = true; quietFor_ = 0; }
    else if (ratio < 2.8) {
        quietFor_ += dt;
        if (quietFor_ > std::max(1000.0, 10 * dot_)) present_ = false;
    }
    const bool wasPresent = signal_;
    signal_ = present_;
    if (wasPresent && !signal_ && !learning_) {
        emit(); newLine();
        learning_ = true; events_.clear(); recent_.clear(); sym_.clear();   // re-check before printing again
    }
    if (signal_ && !wasPresent) { key_ = false; pend_ = false; pendAt_ = t_; runStart_ = t_; }
    keyStep(e);
}

void CwDecoder::keyStep(double e) {
    // thresholds at the midpoint in dB between key-up and key-down levels: tolerant of fading
    const double r = hi_ / std::max(lo_, 1e-12);
    const double thrOn = lo_ * std::pow(r, 0.5), thrOff = lo_ * std::pow(r, 0.4);
    const bool raw = signal_ && r > 2.8 && (pend_ ? e > thrOff : e > thrOn);
    if (key_ && mbuf_.size() < 500) mbuf_.push_back(e);
    // debounce: a change must last a quarter of a dot (4-10 ms) before it counts
    if (raw != pend_) { pend_ = raw; pendAt_ = t_; }
    const double deb = std::min(10.0, std::max(4.0, dot_ * 0.25));
    if (pend_ != key_ && t_ - pendAt_ >= deb) {
        const double at = pendAt_, dur = at - runStart_;
        const double cv = key_ ? steadiness() : 0;
        if (pend_) mbuf_.clear();   // a new key-down starts
        if (learning_) record(key_, dur, cv);
        else if (check(key_, dur, cv)) { if (key_) mark(dur, false); else space(dur); }
        key_ = pend_; runStart_ = at;
    } else if (!key_ && !learning_) {   // show letters and spaces as the gap grows
        const double gap = t_ - runStart_, u = sdot_ > 0 ? sdot_ : dot_;
        if (!sym_.empty() && gap > 2 * u) emit();
        if (!wordSpaced_ && gap > 5 * u) { put(" "); wordSpaced_ = true; }
        // a long break (1.5 s, or 20 dots): the next transmission starts a new line
        if (!lineBroken_ && gap > std::max(1500.0, 20 * dot_)) newLine();
    }
}

// Level variation inside a key-down (plateau only): a keyed carrier is steady, a syllable isn't.
double CwDecoder::steadiness() {
    const std::vector<double>& b = mbuf_;
    if (b.size() < 4) return 0;
    const double top = *std::max_element(b.begin(), b.end());
    double s = 0, q = 0;
    int n = 0;
    for (std::size_t i = 0; i < b.size(); ++i)
        if (b[i] > 0.5 * top) { s += b[i]; q += b[i] * b[i]; ++n; }
    if (n < 3) return 0;
    const double m = s / n;
    return m > 0 ? std::sqrt(std::max(0.0, q / n - m * m)) / m : 1;
}

// How well events fit Morse timing, given a dot length (cwFit in the browser).
CwDecoder::Fit CwDecoder::fit(const std::vector<Event>& events, double dot, double sdot) {
    if (sdot <= 0) sdot = dot;
    int mOk = 0, mN = 0, sOk = 0, sN = 0, dots = 0;
    scratch_.clear();
    for (std::size_t i = 0; i < events.size(); ++i) {
        const Event& ev = events[i];
        const double r = ev.dur / dot;
        if (ev.mark) {
            ++mN;
            if (cwNear(r, 1) || cwNear(r, 3)) ++mOk;
            if (r < 1.6) ++dots;
            scratch_.push_back(ev.cv);
        } else {
            const double q = ev.dur / sdot;
            ++sN;
            if (cwNear(q, 1) || cwNear(q, 3) || q > 4.5) ++sOk;
        }
    }
    Fit f;
    f.marks = mN ? static_cast<double>(mOk) / mN : 0;
    f.spaces = sN ? static_cast<double>(sOk) / sN : 1;
    if (scratch_.empty()) f.cv = 1;
    else {   // steadiness: the median level variation within key-downs
        std::nth_element(scratch_.begin(), scratch_.begin() + scratch_.size() / 2, scratch_.end());
        f.cv = scratch_[scratch_.size() / 2];
    }
    f.dots = mN ? static_cast<double>(dots) / mN : 0;
    return f;
}

void CwDecoder::record(bool isMark, double d, double cv) {
    if (isMark && d < 6) return;
    Event ev = {isMark, d, cv};
    events_.push_back(ev);
    int nMarks = 0;
    double lastMark = 0;
    for (std::size_t i = 0; i < events_.size(); ++i)
        if (events_[i].mark) { ++nMarks; lastMark = events_[i].dur; }
    if (nMarks < (validated_ ? 6 : 10)) return;   // quicker once the speed is known
    marks_.clear();
    {
        int skip = nMarks - 24;
        for (std::size_t i = 0; i < events_.size(); ++i)
            if (events_[i].mark) { if (skip-- <= 0) marks_.push_back(events_[i].dur); }
    }
    learnSpeed(lastMark, true);
    sdot_ = learnGap();
    // Morse-like? timing fits, key-downs steady, plenty of dots, and some dashes.
    const Fit f = fit(events_, dot_, sdot_);
    int dashes = 0;
    for (std::size_t i = 0; i < events_.size(); ++i)
        if (events_[i].mark && events_[i].dur > markThr_) ++dashes;
    if (f.marks < 0.85 || f.spaces < 0.8 || f.cv > 0.15 || f.dots < 0.2 || dashes == 0) {
        // not Morse (yet): slide past one mark
        std::size_t drop = 0;
        for (std::size_t i = 1; i < events_.size(); ++i)
            if (events_[i].mark) { drop = i; break; }
        events_.erase(events_.begin(), events_.begin() + (drop > 0 ? drop : 1));
        return;
    }
    learning_ = false; recent_.clear(); validated_ = true;
    std::vector<Event> ev2;
    ev2.swap(events_);
    for (std::size_t i = 0; i < ev2.size(); ++i) {
        if (ev2[i].mark) mark(ev2[i].dur, true);
        else space(ev2[i].dur);
    }
}

void CwDecoder::mark(double d, bool replay) {
    if (d < 6) return;   // too short to be keying
    lastMarkAt_ = t_; lineBroken_ = false;
    if (!replay) {
        marks_.push_back(d);
        if (marks_.size() > 24) marks_.erase(marks_.begin());
        learnSpeed(d, false);
    }
    sym_ += d > (markThr_ > 0 ? markThr_ : 2 * dot_) ? '-' : '.';
    if (sym_.size() > 9) sym_.clear();   // not Morse: start again
    wordSpaced_ = false;
}

// live check: the last 12 elements must still fit Morse timing, or stop printing and listen again
bool CwDecoder::check(bool isMark, double d, double cv) {
    Event ev = {isMark, d, cv};
    recent_.push_back(ev);
    if (recent_.size() > 24) recent_.erase(recent_.begin());
    if (recent_.size() < 12) return true;
    const Fit f = fit(recent_, dot_, sdot_);
    if (f.marks < 0.6 || f.spaces < 0.5 || f.cv > 0.2 || f.dots < 0.1) {
        learning_ = true; events_.clear(); sym_.clear(); recent_.clear();
        if (lastChar_ != 0 && lastChar_ != ' ') put(" ");
        return false;
    }
    return true;
}

void CwDecoder::space(double d) {
    const double u = sdot_ > 0 ? sdot_ : dot_;
    if (d < 2 * u) sdot_ = u + (d - u) * 0.2;   // a within-letter gap: refine the unit
    else if (d < 5 * u) sdot_ = u + (d / 3 - u) * 0.1;
    if (d > 2 * u) emit();
    if (d > 5 * u && !wordSpaced_) { put(" "); wordSpaced_ = true; }
}

// Gap unit, learned from the gaps themselves (the threshold makes key-downs look a little longer
// and gaps a little shorter). Within-letter gaps are the shortest group (1 dot, letter gaps 3).
double CwDecoder::learnGap() {
    std::vector<double>& g = scratch_;
    g.clear();
    for (std::size_t i = 0; i < events_.size(); ++i)
        if (!events_[i].mark && events_[i].dur < 5 * dot_ && events_[i].dur > 0) g.push_back(std::log(events_[i].dur));
    if (g.size() < 3) return dot_;
    double c1, c2;
    twoMeans(g, c1, c2);
    const double u = c2 - c1 > std::log(1.8) ? (std::exp(c1) + std::exp(c2) / 3) / 2 : std::exp(c1);
    return std::max(dot_ * 0.5, std::min(dot_ * 1.5, u));
}

// Speed: split recent marks into two groups (2-means on log length): dots and dashes, nominally
// 1:3. The dot/dash threshold sits between the groups, so sloppy timing still sorts.
void CwDecoder::learnSpeed(double d, bool snap) {
    if (marks_.size() >= 4) {
        std::vector<double>& lg = scratch_;
        lg.resize(marks_.size());
        for (std::size_t i = 0; i < marks_.size(); ++i) lg[i] = std::log(marks_[i]);
        double c1, c2;
        twoMeans(lg, c1, c2);
        double est;
        if (c2 - c1 > std::log(1.8)) {   // two real groups
            const double dotLen = std::exp(c1), dashLen = std::exp(c2);
            est = (dotLen + dashLen / 3) / 2;
            markThr_ = std::sqrt(dotLen * dashLen);
        } else {                          // one group: dots or dashes?
            const double mean = std::exp((c1 + c2) / 2);
            est = mean > 2 * dot_ ? mean / 3 : mean;
            markThr_ = 2 * est;
        }
        dot_ = snap ? est : dot_ + (est - dot_) * 0.3;
    } else {
        dot_ = d; markThr_ = 2 * d;
    }
    dot_ = std::max(20.0, std::min(300.0, dot_));   // 60 down to 4 WPM
    markThr_ = std::max(1.6 * dot_, std::min(2.4 * dot_, markThr_ > 0 ? markThr_ : 2 * dot_));
    wpm_ = 1200 / ((dot_ + (sdot_ > 0 ? sdot_ : dot_)) / 2);
}

void CwDecoder::emit() {
    if (sym_.empty()) return;
    const char* c = codes::morseLookup(sym_);
    put(c ? c : "\xC2\xB7");   // (an unknown pattern prints as a middle dot)
    sym_.clear();
}

void CwDecoder::newLine() {
    lineBroken_ = true;
    if (lastChar_ == 0 || (lastChar_ == '\n' && heldSpaces_ == 0)) return;
    heldSpaces_ = 0;   // (trailing spaces are dropped)
    out_ += '\n';
    lastChar_ = '\n';
}

void CwDecoder::put(const char* s) {
    if (s[0] == ' ' && s[1] == 0) { ++heldSpaces_; lastChar_ = ' '; return; }
    if (heldSpaces_) { out_.append(static_cast<std::size_t>(heldSpaces_), ' '); heldSpaces_ = 0; }
    out_ += s;
    lastChar_ = out_[out_.size() - 1];
    ++chars_;
}

void CwDecoder::takeText(std::string& out) {
    out += out_;
    out_.clear();
}

Status CwDecoder::status() const {
    Status s;
    s.signal = signal_;
    s.locked = signal_ && !learning_;
    s.snrDb = signal_ ? snrDb_ : 0;
    s.toneHz = toneHz_;
    s.wpm = learning_ && !validated_ ? 0 : wpm_;
    s.characters = chars_;
    s.quality = s.locked ? std::min(1.0, std::max(0.0, (snrDb_ - 9) / 15)) : 0;
    s.mode = "CW";
    char buf[96];
    if (s.locked) std::snprintf(buf, sizeof buf, "%.0f WPM  SNR %.0f dB", wpm_, snrDb_);
    else if (signal_) std::snprintf(buf, sizeof buf, "learning the speed...  SNR %.0f dB", snrDb_);
    else std::snprintf(buf, sizeof buf, "listening");
    s.info = buf;
    return s;
}

double CwDecoder::audioCenterHz() const { return toneHz_ > 0 ? toneHz_ : 700; }

}  // namespace dec
