// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Rtty.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The RTTY decoder described in Rtty.h: detail::RttyCore is a port of
//  OFFgrid-SDR's RttyDecoder class (tuneUp, guessBaud, emit, process) and
//  RttyDecoder the port of its rttyFeed() (the twin decoder and the
//  automatic normal/reversed choice), by diddy-boy, index.html.
//  Differences from the browser version:
//    - oscillators are rotating phasors instead of cos/sin per sample (same
//      result, cheaper);
//    - times are audio time; the twin comparison runs every 50 ms of audio
//      (the browser: once per audio block), so the result does not depend
//      on how the caller slices the audio;
//    - text is streamed (see Rtty.h about swaps).
//
//  How it fits in
//  --------------
//  See Rtty.h.
//
//  Contributing
//  ------------
//  Keep the function names parallel to the JavaScript.
// ============================================================================
#include "Rtty.h"
#include "Codes.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace dec {

namespace {

const double kPi = 3.14159265358979323846;
const double kShifts[] = {170, 200, 425, 450, 850};
const double kBauds[] = {45.45, 50, 75};
const int kNA = 4096;
const int kRuns = 300;

// "Does this look like text?" for each decoded character: ordinary letters and spaces score
// well, rare letters a little less; codes for nothing and needless shifts score badly. Figures
// count as fine. A wrongly reversed signal decodes to a jumble that scores far lower.
double rttyCharScore(int code, bool figs) {
    if (code == 27 || code == 31) return -0.6;   // a letters/figures shift
    const char* ch = codes::ita2Char(code, figs);
    if (!*ch) return -2;                         // a code with no character
    if (ch[0] == ' ' || ch[0] == '\n') return 1;
    if (figs) return (ch[0] >= '0' && ch[0] <= '9') ? 0.8 : 0.2;
    if (std::strchr("ETAOINSRHLDCUMFPGWYB", ch[0])) return 1.5;
    if (std::strchr("QXZJKV", ch[0])) return -0.5;
    return 0.5;
}

}  // namespace

namespace detail {

RttyCore::RttyCore()
    : reverse(false), fixedShift(0), fixedBaud(0), rate_(0), D_(1), fs2_(8000), fft_(kNA) {
    fbuf_.resize(kNA);
    P_.resize(kNA / 2);
    runs_.assign(kRuns, 0);
    buf_.assign(kNA, 0.f);
    win_.resize(kNA);
    for (int i = 0; i < kNA; ++i) win_[i] = static_cast<float>(0.5 - 0.5 * std::cos(2 * kPi * i / kNA));
    reset();
}

void RttyCore::reset() {
    mark = 1725; space = 1275; shift = 450; baud = 50; lock = false; lostAt_ = 0;
    q = 0; qs = 0; fOk = 0; fBad = 0;
    text.clear(); lastChar = 0; figs = false; st = 0; cnt_ = 0; bits_ = 0; nb_ = 0; level_ = 1;
    snrDb = 0; chars = 0; errs = 0;
    pmRe_ = 1; pmIm_ = 0; psRe_ = 1; psIm_ = 0; rmRe_ = 1; rmIm_ = 0; rsRe_ = 1; rsIm_ = 0;
    rotMark_ = -1; rotSpace_ = -1; renorm_ = 0;
    mI_ = mQ_ = sI_ = sQ_ = mI2_ = mQ2_ = sI2_ = sQ2_ = 0;
    runPos_ = 0; runCount_ = 0; runLen_ = 0; pendingBaud_ = 0;
    totP_ = 0; toneP_ = 0; present_ = false; levelRun_ = 0; lastRun_ = 0; baudKnown_ = false; found_ = false;
    decim_ = 0; accN_ = 0;
    std::fill(buf_.begin(), buf_.end(), 0.f);
    bp_ = 0; since_ = 0;
}

void RttyCore::setRate(double rate) {
    rate_ = rate;
    D_ = std::max(1, static_cast<int>(std::floor(rate / 8000 + 0.5)));   // analysis at ~8 kHz
    fs2_ = rate / D_;
    std::fill(buf_.begin(), buf_.end(), 0.f);
    bp_ = 0; since_ = 0; decim_ = 0; accN_ = 0;
    rotMark_ = rotSpace_ = -1;
}

// find the tone pair: the two strongest peaks a standard shift apart (mark = the higher tone)
void RttyCore::tuneUp() {
    const int N = kNA;
    for (int i = 0; i < N; ++i) fbuf_[i] = std::complex<float>(buf_[(bp_ + i) % N] * win_[i], 0.f);
    fft_.forward(&fbuf_[0]);
    const double bin = fs2_ / N;
    for (int j = 0; j < N / 2; ++j) P_[j] = std::norm(fbuf_[j]);
    const int lo = static_cast<int>(std::ceil(250 / bin)), hi = std::min(N / 2, static_cast<int>(std::floor(3000 / bin)));
    if (hi - lo < 4) return;
    sorted_.assign(P_.begin() + lo, P_.begin() + hi);
    std::nth_element(sorted_.begin(), sorted_.begin() + sorted_.size() / 2, sorted_.end());
    double med = sorted_[sorted_.size() / 2];
    if (!(med > 0)) med = 1e-12;
    peaks_.clear();
    for (int j = lo + 1; j < hi - 1; ++j)
        if (P_[j] > med * 20 && P_[j] >= P_[j - 1] && P_[j] >= P_[j + 1]) peaks_.push_back(j);
    int ba = -1, bb = -1;
    double bscore = 0, bshift = 0;
    for (std::size_t i = 0; i < peaks_.size(); ++i)
        for (std::size_t k = 0; k < peaks_.size(); ++k) {
            const int a = peaks_[i], b = peaks_[k];
            if (b <= a) continue;
            const double sh = (b - a) * bin;
            double want = fixedShift;
            if (want <= 0)
                for (std::size_t s = 0; s < sizeof kShifts / sizeof kShifts[0]; ++s)
                    if (std::fabs(sh - kShifts[s]) < std::max(12.0, kShifts[s] * 0.04)) { want = kShifts[s]; break; }
            if (want <= 0 || std::fabs(sh - want) > std::max(12.0, want * 0.04)) continue;
            const double score = std::min(P_[a], P_[b]);
            if (ba < 0 || score > bscore) { ba = a; bb = b; bscore = score; bshift = want; }
        }
    if (ba >= 0 && bscore > med * 40) {
        const double fa = ba * bin, fb = bb * bin;
        // (changed in this port: the browser jumps to any pair more than 8 Hz away. The single-bin
        //  peaks of 45-75 baud data wander by +-10-15 Hz from one spectrum to the next, so the
        //  tones jumped about (a 170 Hz pair read as 182 Hz). A pair within 30 Hz of the current
        //  one now pulls it a third of the way; one further off is taken at once, as before.)
        if (!lock || std::fabs(fa - space) > 30 || std::fabs(fb - mark) > 30) { space = fa; mark = fb; shift = bshift; }
        else { space += (fa - space) / 3; mark += (fb - mark) / 3; shift = bshift; }
        lock = true; lostAt_ = 0; found_ = true;
        // (rough SNR in 3 kHz: each tone is on half the time; the median noise bin is ln 2 of the mean)
        snrDb = 10 * std::log10(std::max(1e-6, 2 * bscore / ((med / 0.693) * (3000 / bin) / 1.5)));
    } else if (lock && ++lostAt_ >= 4) {
        lock = false;   // ~1-2 s without the pair
        runCount_ = 0; runPos_ = 0; pendingBaud_ = 0; baudKnown_ = false;   // (the next signal's speed is measured afresh)
    }
}

// the speed: measure single-bit pulses. Ignore glitches shorter than any real bit, take the
// shortest common runs (single bits); their middle value is the bit time; then the nearest
// standard speed, if it's close.
void RttyCore::guessBaud() {
    if (fixedBaud > 0) { baud = fixedBaud; return; }
    const double R = rate_;
    sorted_.clear();
    for (int i = 0; i < runCount_; ++i)
        if (runs_[i] > R / 120) sorted_.push_back(runs_[i]);   // (a 100 Bd bit is R/100)
    if (sorted_.size() < 30) return;
    std::sort(sorted_.begin(), sorted_.end());
    const double T0 = sorted_[static_cast<std::size_t>(sorted_.size() * 0.1)];
    std::size_t first = sorted_.size(), count = 0;
    for (std::size_t i = 0; i < sorted_.size(); ++i)
        if (sorted_[i] > 0.7 * T0 && sorted_[i] < 1.4 * T0) { if (first == sorted_.size()) first = i; ++count; }
    if (count < 10) return;
    const double T = sorted_[first + (count >> 1)], B = R / T;
    double bestC = 0, bestE = 1e9;
    for (std::size_t i = 0; i < sizeof kBauds / sizeof kBauds[0]; ++i) {
        const double e = std::fabs(B - kBauds[i]) / kBauds[i];
        if (e < bestE) { bestE = e; bestC = kBauds[i]; }
    }
    // (added in this port) the first measurement on a signal is taken at once; after that a new
    // speed only when two measurements in a row agree on it
    if (bestE >= 0.06) return;
    if (!baudKnown_) { baud = bestC; baudKnown_ = true; pendingBaud_ = 0; return; }
    if (bestC == baud) { pendingBaud_ = 0; return; }
    if (pendingBaud_ == bestC) { baud = bestC; pendingBaud_ = 0; }
    else pendingBaud_ = bestC;
}

void RttyCore::emit(int code) {
    const double sc = rttyCharScore(code, figs);
    q = q * 0.97 + sc;     // how text-like the last few dozen characters were
    qs = qs * 0.99 + sc;   // ...and the last hundred or so (steadier)
    if (code == codes::kIta2Letters) { figs = false; return; }
    if (code == codes::kIta2Figures) { figs = true; return; }
    const char* ch = codes::ita2Char(code, figs);
    if (code == codes::kIta2Space) figs = false;   // unshift on space
    if (!*ch) return;
    if (ch[0] == '\n' && lastChar == '\n') return;
    text += ch;
    lastChar = ch[0];
    ++chars;
    if (text.size() > 4000) text.erase(0, text.size() - 3000);
}

void RttyCore::process(const float* x, std::size_t n, double rate) {
    if (rate <= 0) return;
    if (rate != rate_) setRate(rate);
    const double R = rate_;
    if (rotMark_ != mark) { const double w = 2 * kPi * mark / R; rmRe_ = std::cos(w); rmIm_ = std::sin(w); rotMark_ = mark; }
    if (rotSpace_ != space) { const double w = 2 * kPi * space / R; rsRe_ = std::cos(w); rsIm_ = std::sin(w); rotSpace_ = space; }
    const double a = 1 - std::exp(-2 * kPi * baud * 0.75 / R);   // the tone filters: ~0.75 x baud
    const double T = R / baud;
    const double pa = 1 - std::exp(-1 / (R * 0.03));   // presence: powers averaged over ~30 ms
    for (std::size_t i = 0; i < n; ++i) {
        const double v = x[i];
        // analysis buffer for tuneUp (decimated by averaging)
        decim_ += v;
        if (++accN_ >= D_) {
            buf_[bp_] = static_cast<float>(decim_ / D_);
            bp_ = (bp_ + 1) % kNA; decim_ = 0; accN_ = 0;
            if (++since_ >= kNA / 2) { since_ = 0; tuneUp(); guessBaud(); }
        }
        // two tone detectors (mix down, 2-pole low-pass), then the discriminator
        double t = pmRe_ * rmRe_ - pmIm_ * rmIm_;
        pmIm_ = pmRe_ * rmIm_ + pmIm_ * rmRe_; pmRe_ = t;
        t = psRe_ * rsRe_ - psIm_ * rsIm_;
        psIm_ = psRe_ * rsIm_ + psIm_ * rsRe_; psRe_ = t;
        if (++renorm_ >= 1024) {
            renorm_ = 0;
            double m = std::sqrt(pmRe_ * pmRe_ + pmIm_ * pmIm_); pmRe_ /= m; pmIm_ /= m;
            m = std::sqrt(psRe_ * psRe_ + psIm_ * psIm_); psRe_ /= m; psIm_ /= m;
        }
        mI_ += a * (v * pmRe_ - mI_); mQ_ += a * (v * pmIm_ - mQ_);
        sI_ += a * (v * psRe_ - sI_); sQ_ += a * (v * psIm_ - sQ_);
        mI2_ += a * (mI_ - mI2_); mQ2_ += a * (mQ_ - mQ2_); sI2_ += a * (sI_ - sI2_); sQ2_ += a * (sQ_ - sQ2_);
        const double m = mI2_ * mI2_ + mQ2_ * mQ2_, s = sI2_ * sI2_ + sQ2_ * sQ2_;
        double d = (m - s) / (m + s + 1e-20);
        if (reverse) d = -d;
        const int prev = level_;
        if (d > 0.15) level_ = 1;
        else if (d < -0.15) level_ = 0;   // (a little hysteresis)
        // (added in this port) Is the pair actually on the air? The two tone filters hold
        // nearly all of the audio's power when it is (2 (m + s) ~ the power of the audio), a
        // small share of it in noise (two ~40 Hz slices of a 3 kHz passband). The browser
        // measured pulse lengths in noise too: after a gap of loud noise (a receiver's AGC
        // lifts it to full level between overs) the noise's flicker made up most of the last
        // 300 runs, read as 75 baud, and with the filters then widened to suit 75 baud the
        // decoder stayed there. Runs are now kept, and characters started, only while the
        // pair is present.
        totP_ += pa * (v * v - totP_);
        toneP_ += pa * (2 * (m + s) - toneP_);
        if (present_) { if (toneP_ < 0.2 * totP_) present_ = false; }
        else if (toneP_ > 0.3 * totP_) present_ = true;
        ++runLen_;
        ++levelRun_;
        if (level_ != prev) { lastRun_ = levelRun_; levelRun_ = 0; }
        if (!present_) runLen_ = 0;
        else if (level_ != prev) {
            runs_[runPos_] = runLen_; runPos_ = (runPos_ + 1) % kRuns;
            if (runCount_ < kRuns) ++runCount_;
            runLen_ = 0;
        }
        // (changed in this port: the browser decodes only while the pair is in view of the last
        //  tone search. An over starts with mark alone and the search needs both tones, so after
        //  every gap it locked on mid-character and the first words came out misframed. Once the
        //  pair has been found, characters are also read on those tones whenever they are on the
        //  air.)
        if (!lock && !(found_ && present_)) { st = 0; continue; }
        // the character: start bit (space), 5 data bits (LSB first), then a stop bit (mark)
        if (st == 0) {
            // (added in this port: a start bit follows at least the stop bit's mark, so an edge
            //  after a shorter mark is inside a character, e.g. just after locking on mid-over)
            if (present_ && prev == 1 && level_ == 0 && lastRun_ >= 0.8 * T) { st = 1; cnt_ = T * 0.5; }
        } else if (--cnt_ <= 0) {
            if (st == 1) {
                if (level_ != 0 || !present_) { st = 0; continue; }
                st = 2; bits_ = 0; nb_ = 0; cnt_ = T;
            } else if (st == 2) {
                bits_ |= level_ << nb_;
                if (++nb_ == 5) st = 3;
                cnt_ = T;
            } else {
                const bool ok = level_ == 1 && present_;   // (a character running into noise is dropped)
                if (ok) emit(bits_); else ++errs;
                st = 0;
                // (the framing, as running counts over the last 30 or so characters: the right way
                //  round the stop bit nearly always is where it should be; the wrong way, about half)
                fOk = fOk * 0.97 + (ok ? 1 : 0);
                fBad = fBad * 0.97 + (ok ? 0 : 1);
            }
        }
    }
}

}  // namespace detail

// ---------------------------------------------------------------------------
RttyDecoder::RttyDecoder()
    : fixedShift_(0), fixedBaud_(0), reverseSetting_(false), autoReverse_(true), t_(0), swapAt_(0) {
    twin_.reverse = true;
    reset();
}

void RttyDecoder::setFixedShift(double hz) { fixedShift_ = hz > 0 ? hz : 0; main_.fixedShift = twin_.fixedShift = fixedShift_; }

void RttyDecoder::setFixedBaud(double b) {
    fixedBaud_ = b > 0 ? b : 0;
    main_.fixedBaud = twin_.fixedBaud = fixedBaud_;
    if (fixedBaud_ > 0) main_.baud = twin_.baud = fixedBaud_;
}

void RttyDecoder::setReverse(bool on) {
    reverseSetting_ = on;
    main_.reverse = on;
    twin_.reverse = !on;
}

void RttyDecoder::reset() {
    main_.reset(); twin_.reset();
    main_.fixedShift = twin_.fixedShift = fixedShift_;
    main_.fixedBaud = twin_.fixedBaud = fixedBaud_;
    if (fixedBaud_ > 0) main_.baud = twin_.baud = fixedBaud_;
    main_.reverse = reverseSetting_;
    twin_.reverse = !reverseSetting_;
    // (no swap for the first 5 s, while the tones are found: the text test, on the first few
    //  characters, could swap the wrong way)
    swapAt_ = t_ + 2.0;
    out_.clear();
}

void RttyDecoder::process(const float* audio, std::size_t n, double rate) {
    if (rate <= 0) return;
    const std::size_t step = std::max<std::size_t>(1, static_cast<std::size_t>(rate * 0.05));
    for (std::size_t i = 0; i < n; i += step) chunk(audio + i, std::min(step, n - i), rate);
}

void RttyDecoder::chunk(const float* x, std::size_t n, double rate) {
    t_ += n / rate;
    main_.process(x, n, rate);
    out_ += main_.text;
    main_.text.clear();
    if (!autoReverse_) return;
    twin_.reverse = !main_.reverse;
    twin_.process(x, n, rate);
    // The other way round reads clearly better: swap. Tests (as rttyFeed in the browser):
    //  - framing: the right way round nearly every stop bit is where it should be (2-8% not), the
    //    wrong way round a third to a half aren't;
    //  - text: the last few dozen characters clearly better (quick), or the last hundred steadily
    //    better; but the text test may not swap away from clean framing to bad.
    const double nT = twin_.fOk + twin_.fBad, nD = main_.fOk + main_.fBad;
    const double feT = nT >= 10 ? twin_.fBad / nT : -1, feD = nD >= 10 ? main_.fBad / nD : -1;
    const bool framed = feT >= 0 && feD >= 0 && feT < 0.15 && feD > 0.3 && feD > feT * 2.5;
    const bool framedNow = feT >= 0 && feD >= 0 && feD < 0.15 && feT > 0.3;
    const bool settled = t_ - swapAt_ > 3.0;
    const bool byText = !framedNow && ((twin_.q > 6 && twin_.q > main_.q * 1.25 + 3) ||
                                       (twin_.qs > 20 && twin_.qs > main_.qs * 1.15 + 5));
    if (settled && (framed || byText)) {
        swapAt_ = t_;
        // the twin's text (the same stretch, read the right way) replaces the wrong-way text;
        // as that was already handed out, the twin's goes on a new line
        if (!twin_.text.empty()) {
            if (!out_.empty() ? out_[out_.size() - 1] != '\n' : (main_.lastChar != 0 && main_.lastChar != '\n')) out_ += '\n';
            out_ += twin_.text;
            main_.lastChar = twin_.lastChar;
        }
        main_.reverse = !main_.reverse;
        main_.figs = twin_.figs;
        main_.st = 0;
        std::swap(main_.q, twin_.q); std::swap(main_.qs, twin_.qs);
        std::swap(main_.fOk, twin_.fOk); std::swap(main_.fBad, twin_.fBad);
        twin_.text.clear();
        twin_.reverse = !main_.reverse;
    }
}

void RttyDecoder::takeText(std::string& out) {
    out += out_;
    out_.clear();
}

Status RttyDecoder::status() const {
    Status s;
    const detail::RttyCore& c = main_;
    s.signal = c.lock;
    s.locked = c.lock;
    s.snrDb = c.lock ? c.snrDb : 0;
    const double n = c.fOk + c.fBad;
    s.quality = c.lock && n > 1 ? c.fOk / n : 0;
    s.toneHz = c.mark;
    s.shiftHz = c.mark - c.space;
    s.baud = c.baud;
    s.characters = c.chars;
    char buf[96];
    std::snprintf(buf, sizeof buf, "RTTY %g/%g", c.baud, c.shift);
    s.mode = buf;
    if (c.lock)
        std::snprintf(buf, sizeof buf, "%g Bd  %.0f Hz shift  %s  SNR %.0f dB", c.baud, c.mark - c.space,
                      c.reverse ? "reversed" : "normal", s.snrDb);
    else
        std::snprintf(buf, sizeof buf, "searching for the two tones");
    s.info = buf;
    return s;
}

double RttyDecoder::audioCenterHz() const { return (main_.mark + main_.space) / 2; }

}  // namespace dec
