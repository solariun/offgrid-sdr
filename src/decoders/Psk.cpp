// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Psk.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  PskDecoder (one speed) and PskAutoDecoder (PSK31/63/125 at once): see
//  Psk.h for the method.  Line-by-line port of OFFgrid-SDR's JS PskDecoder
//  and PskAuto (diddy-boy, https://github.com/solariun/offgrid-sdr); the JS
//  comments are kept (reworded) next to the code they explain.  Differences
//  from the JS, none of which change what is decoded:
//    - text is handed out incrementally (takeText) instead of kept as a
//      4000-character history;
//    - the NaN check looks at every running value, not a sample of them;
//    - process() ignores a rate that is not a positive finite number;
//    - two fixes found by the tests (marked "Addition to the JS" below): a
//      lone 0 before a character is dropped (the JS lost the first
//      character of an over half the time), the squelch closes 2 bits
//      after the tone goes (the JS took ~12 bits of noise and often printed
//      a stray character after each over) and does not reopen on the fading
//      dip pattern, and the AFC pulls in faster while the squelch is shut
//      (the JS garbled the first characters of a signal 30 Hz off).
//
//  How it fits in
//  --------------
//  Implements Psk.h.  Standard library only.
//
//  Contributing
//  ------------
//  No allocation per sample (strings grow only per decoded character).
//  Keep in step with the JS; run tests/test_decoders_psk.cpp.
// ============================================================================
#include "Psk.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace dec {

namespace {

const double kTwoPi = 6.283185307179586;

// Varicode, ASCII 0..127 (G3PLX's table, as in the JS VARICODE).
const char* const kVaricode[128] = {
    "1010101011", "1011011011", "1011101101", "1101110111", "1011101011", "1101011111", "1011101111", "1011111101",
    "1011111111", "11101111", "11101", "1101101111", "1011011101", "11111", "1101110101", "1110101011",
    "1011110111", "1011110101", "1110101101", "1110101111", "1101011011", "1101101011", "1101101101", "1101010111",
    "1101111011", "1101111101", "1110110111", "1101010101", "1101011101", "1110111011", "1011111011", "1101111111",
    "1", "111111111", "101011111", "111110101", "111011011", "1011010101", "1010111011", "101111111",
    "11111011", "11110111", "101101111", "111011111", "1110101", "110101", "1010111", "110101111",
    "10110111", "10111101", "11101101", "11111111", "101110111", "101011011", "101101011", "110101101",
    "110101011", "110110111", "11110101", "110111101", "111101101", "1010101", "111010111", "1010101111",
    "1010111101", "1111101", "11101011", "10101101", "10110101", "1110111", "11011011", "11111101",
    "101010101", "1111111", "111111101", "101111101", "11010111", "10111011", "11011101", "10101011",
    "11010101", "111011101", "10101111", "1101111", "1101101", "101010111", "110110101", "101011101",
    "101110101", "101111011", "1010101101", "111110111", "111101111", "111111011", "1010111111", "101101101",
    "1011011111", "1011", "1011111", "101111", "101101", "11", "111101", "1011011",
    "101011", "1101", "111101011", "10111111", "11011", "111011", "1111", "111",
    "111111", "110111111", "10101", "10111", "101", "110111", "1111011", "1101011",
    "11011111", "1011101", "111010101", "1010110111", "110111011", "1010110101", "1011010111", "1110110101",
};

const double kSpeeds[3] = {31.25, 62.5, 125.0};
const char* const kNames[3] = {"PSK31", "PSK63", "PSK125"};

bool finite(double v) { return v == v && v - v == 0; }   // not NaN, not +-inf

}  // namespace

const char* pskVaricode(int c) { return c >= 0 && c < 128 ? kVaricode[c] : 0; }

int pskVaricodeLookup(const char* bits, std::size_t n) {
    for (int c = 0; c < 128; ++c)
        if (std::strlen(kVaricode[c]) == n && std::memcmp(kVaricode[c], bits, n) == 0) return c;
    return -1;
}

const char* pskModeName(double baud) {
    for (int i = 0; i < 3; ++i)
        if (std::fabs(baud - kSpeeds[i]) < 1e-6) return kNames[i];
    return "PSK";
}

// ===========================================================================
// PskDecoder
// ===========================================================================
PskDecoder::PskDecoder(double baud)
    : baud_(baud > 0 ? baud : 31.25), rate_(0), D_(1), a_(0), chars_(0), lastChar_(0) {
    text_.reserve(256);
    rebuild();
}

void PskDecoder::rebuild() {
    f_ = pskAudioHz;
    ph_ = 0;
    for (int i = 0; i < 8; ++i) lp_[i] = 0;
    k_ = 0;
    t_ = 0;
    for (int i = 0; i < 16; ++i) { amp_[i] = 0; af_[i] = 0; }
    afInit_ = false;
    havePrev_ = haveSq_ = false;
    prevR_ = prevI_ = sqR_ = sqI_ = 0;
    ar2_ = ai2_ = 0;
    q_ = 0;
    pk_ = 0;
    mAvg_ = 0;
    pqA_ = 0.5;
    floor_ = 0;
    floorInit_ = false;
    openRun_ = pqRun_ = lowRun_ = 0;
    open_ = skipFirst_ = false;
    nbits_ = 0;
}

void PskDecoder::reset() {
    rebuild();
    text_.clear();
    chars_ = 0;
    lastChar_ = 0;
}

void PskDecoder::retune() {
    f_ = pskAudioHz;
    open_ = false;
    nbits_ = 0;
}

void PskDecoder::setToneHz(double hz) {
    if (finite(hz)) f_ = hz;
}

// 16 samples a bit after decimation (500 Hz for PSK31); the filter widens
// with the speed (40 Hz for PSK31).
void PskDecoder::setRate(double rate) {
    rate_ = rate;
    long d = std::lround(rate / (16.0 * baud_));
    D_ = d < 1 ? 1 : static_cast<int>(d);
    a_ = 1.0 - std::exp(-kTwoPi * 40.0 * (baud_ / 31.25) / rate);
}

bool PskDecoder::stateFinite() const {
    double s = f_ + ph_ + ar2_ + ai2_ + q_ + pk_ + mAvg_ + pqA_ + floor_ + prevR_ + prevI_ + sqR_ + sqI_;
    for (int i = 0; i < 8; ++i) s += lp_[i];
    for (int i = 0; i < 16; ++i) s += amp_[i] + af_[i];
    return finite(s);
}

void PskDecoder::process(const float* x, std::size_t n, double rate) {
    if (!(rate > 0) || !finite(rate) || !x) return;
    if (rate != rate_) setRate(rate);
    // Poisoned by a broken sample (a running value not a number): rebuilt
    // from scratch, keeping the text.
    if (!stateFinite()) rebuild();
    const double a = a_, w = kTwoPi * f_ / rate_;
    double* lp = lp_;
    double step = w;
    for (std::size_t i = 0; i < n; ++i) {
        ph_ += step;
        if (ph_ > kTwoPi) ph_ -= kTwoPi;
        const double xi = x[i];
        lp[0] += a * (xi * std::cos(ph_) - lp[0]);
        lp[1] += a * (-xi * std::sin(ph_) - lp[1]);
        for (int s = 2; s < 8; s += 2) {
            lp[s] += a * (lp[s - 2] - lp[s]);
            lp[s + 1] += a * (lp[s - 1] - lp[s + 1]);
        }
        if (++k_ < D_) continue;
        k_ = 0;
        sample(lp[6], lp[7]);
        step = kTwoPi * f_ / rate_;   // AFC may have moved f
    }
}

void PskDecoder::sample(double zr, double zi) {
    // AFC on every sample (16 a bit): the signal squared cancels the flips;
    // its rotation is twice the frequency error.  (Measured once a bit, an
    // error over ~8 Hz turned too far to tell from a flip.)
    {
        const double sr = zr * zr - zi * zi, si = 2 * zr * zi;
        if (haveSq_) {
            ar2_ = ar2_ * 0.98 + (sr * sqR_ + si * sqI_);
            ai2_ = ai2_ * 0.98 + (si * sqR_ - sr * sqI_);
        }
        sqR_ = sr; sqI_ = si; haveSq_ = true;
    }
    // Bit timing: the strength dips at every phase flip; the bit's centre is
    // the strongest of the 16 positions (the dips mark the bit boundaries,
    // only at flips: averaged slowly so a run of 1s, with no dips, doesn't
    // move it; the centre is half a bit after the dip).
    const int pos = static_cast<int>(t_ % 16);
    const double m = std::sqrt(zr * zr + zi * zi);
    ++t_;
    amp_[pos] = amp_[pos] * 0.995 + m * 0.005;
    // The switch-on check has its own fast measurement (~0.3 s): the slow one
    // keeps the timing steady but took ~6 s to show the idle's dips.
    if (!afInit_) { for (int j = 0; j < 16; ++j) af_[j] = 0; afInit_ = true; }
    af_[pos] = af_[pos] * 0.85 + m * 0.15;
    int dip = 0;
    for (int j = 1; j < 16; ++j)
        if (amp_[j] < amp_[dip]) dip = j;
    const int best = (dip + 8) % 16;
    if (pos != best) return;

    // A bit: compare its phase with the previous bit's.
    if (havePrev_) {
        const double pr = prevR_, pi = prevI_;
        const double dot = zr * pr + zi * pi, crs = zi * pr - zr * pi;
        double hiA = amp_[0], hiF = af_[0], loF = af_[0];
        for (int j = 1; j < 16; ++j) {
            if (amp_[j] > hiA) hiA = amp_[j];
            if (af_[j] > hiF) hiF = af_[j];
            if (af_[j] < loF) loF = af_[j];
        }
        const double hi = amp_[best];
        // AFC: the squared phase change (the flips cancel) shows the error.
        const double err = std::atan2(ai2_, ar2_ != 0 ? ar2_ : 1.0) / 2.0 * (rate_ / D_) / kTwoPi;   // Hz
        // Bit timing quality: ~1 when the bit centre is the strongest point.
        q_ = hiA > 0 ? hi / hiA : 0;
        // (Always: it has to pull onto a signal before the phase test below
        //  can see it; on noise it only wanders within its +-70 Hz.)
        // (Addition to the JS: while the squelch is shut the loop pulls in
        //  harder, 0.4 of the error a bit instead of 0.15, so a signal 30-50 Hz
        //  off is on frequency before its idle ends; the JS took most of a
        //  PSK31 idle to get within a few Hz and garbled or lost the first
        //  characters.)
        f_ += err * (open_ ? 0.15 : 0.4);
        if (f_ < pskAudioHz - 70) f_ = pskAudioHz - 70;
        if (f_ > pskAudioHz + 70) f_ = pskAudioHz + 70;
        // Signal or noise?  Open on PSK's idle pattern (strong dips: every
        // transmission starts with it), then stay with the signal while its
        // strength holds; text has runs of 1s with no dips, so dips alone
        // can't keep it open.
        pk_ = hi > pk_ * 0.995 ? hi : pk_ * 0.995;
        // Clear dips at the bit boundaries for 8 bits running (noise can make
        // a dip pattern for a moment, not steadily).  (No "signal over a
        // floor" test: tuned straight onto a signal, the floor was learned
        // from the signal itself, and it could never open.)
        // (Addition to the JS: also the bit centre itself strong, over half the
        //  peak, and the AFC settled within a tenth of the bit rate.  After a
        //  transmission the fading dip pattern alone reopened the squelch on
        //  the noise; and an idle decoded off frequency reads as rubbish.)
        openRun_ = hiF > loF * 2.2 && m > hiF * 0.5 && std::fabs(err) < 0.1 * baud_ ? openRun_ + 1 : 0;
        // ...or part-way through a transmission: PSK's phase only changes by
        // 0 or 180 degrees bit to bit, so |cos| of the change is near 1;
        // noise's is anything (~0.5 on average).  A clean phase for 20 bits
        // running opens it too; the first character is then dropped
        // (decoding starts part-way through one).
        const double pq = std::fabs(dot) / (std::fabs(dot) + std::fabs(crs) + 1e-30);
        pqA_ = pqA_ * 0.9 + pq * 0.1;
        pqRun_ = pqA_ > 0.85 ? pqRun_ + 1 : 0;
        if (!open_ && openRun_ >= 8) open_ = true;
        // (Its bit timing must show at this decoder's own speed: a decoder
        //  slower than the signal also sees clean 0/180-degree changes, but
        //  no dip pattern of its own, and would only print rubbish.)
        else if (!open_ && pqRun_ >= 20 && hiF > loF * 1.4) { open_ = true; skipFirst_ = true; }
        if (open_ && hi < pk_ * 0.25) open_ = false;
        // ...and it closes when the phase stops being clean: the transmission
        // has ended, only noise is left (its strength alone can take seconds
        // to show that, while printing rubbish).
        if (open_ && pqA_ < 0.65) open_ = false;
        // (Addition to the JS: the tone has gone.  The bit centre is always at
        //  full strength in a PSK signal; 2 centres in a row under a quarter
        //  of their own average (learned while open, frozen while weak) mean
        //  the transmission ended.  The two tests above take ~12 bits of
        //  noise to notice, enough to print a stray character or two after
        //  every over.  A deep fade can close it too; it reopens on the clean
        //  phase, as when tuned in mid-over.)
        if (!open_) { mAvg_ = hiF; lowRun_ = 0; }
        else if (m < mAvg_ * 0.25) { if (++lowRun_ >= 2) { open_ = false; lowRun_ = 0; } }
        else { lowRun_ = 0; mAvg_ = mAvg_ * 0.9 + m * 0.1; }
        if (!open_) {
            floor_ = (floorInit_ ? floor_ : hiF) * 0.99 + hiF * 0.01;
            floorInit_ = true;
        }
        if (open_) addBit(dot < 0 ? 0 : 1);
        else nbits_ = 0;
    }
    prevR_ = zr; prevI_ = zi; havePrev_ = true;
}

void PskDecoder::addBit(int b) {
    // (Addition to the JS: Varicode codes all start with a 1, so a 0 with
    //  nothing before it is the tail of a separator or of the idle.  The JS
    //  kept it, and when the squelch opened an odd number of idle bits
    //  before the text, the first character came out as "0"+code: lost.)
    if (nbits_ == 0 && b == 0) return;
    bits_[nbits_++] = static_cast<char>('0' + b);
    if (nbits_ >= 2 && bits_[nbits_ - 1] == '0' && bits_[nbits_ - 2] == '0') {
        const int len = nbits_ - 2;
        nbits_ = 0;
        if (len == 0) return;
        if (skipFirst_) { skipFirst_ = false; return; }   // (joined part-way through it)
        const int c = pskVaricodeLookup(bits_, static_cast<std::size_t>(len));
        if (c < 0) return;
        char ch = 0;
        if (c == 10 || c == 13) ch = '\n';
        else if (c >= 32 && c < 127) ch = static_cast<char>(c);
        if (!ch || (ch == '\n' && lastChar_ == '\n')) return;
        text_ += ch;
        lastChar_ = ch;
        ++chars_;
    } else if (nbits_ > 14) {
        nbits_ = 0;
    }
}

void PskDecoder::takeText(std::string& out) {
    out += text_;
    text_.clear();
}

Status PskDecoder::status() const {
    Status s;
    s.locked = open_;
    s.signal = open_;
    s.quality = q_;
    s.toneHz = f_;
    s.baud = baud_;
    s.characters = chars_;
    s.mode = pskModeName(baud_);
    char buf[96];
    const double off = f_ - pskAudioHz;
    if (open_) std::snprintf(buf, sizeof buf, "%s receiving  tone %+.0f Hz", s.mode.c_str(), off);
    else std::snprintf(buf, sizeof buf, "%s waiting for the signal", s.mode.c_str());
    s.info = buf;
    return s;
}

// ===========================================================================
// PskAutoDecoder
// ===========================================================================
PskAutoDecoder::PskAutoDecoder() : lastOut_(0), cur_(-1), t_(0), chars_(0) {
    decs_[0] = PskDecoder(kSpeeds[0]);
    decs_[1] = PskDecoder(kSpeeds[1]);
    decs_[2] = PskDecoder(kSpeeds[2]);
    out_.reserve(256);
    scratch_.reserve(256);
    for (int i = 0; i < 3; ++i) pend_[i].reserve(64);
    restartSearch();
}

// A new frequency: start the speed search again.
void PskAutoDecoder::restartSearch() {
    cur_ = -1;
    for (int i = 0; i < 3; ++i) {
        score_[i] = 0;
        lastAt_[i] = -1e9;
        pend_[i].clear();
        since_[i] = 0;
    }
    t_ = 0;
}

void PskAutoDecoder::reset() {
    for (int i = 0; i < 3; ++i) decs_[i].reset();
    restartSearch();
    out_.clear();
    lastOut_ = 0;
    chars_ = 0;
}

void PskAutoDecoder::retune() {
    for (int i = 0; i < 3; ++i) decs_[i].retune();
    restartSearch();
    if (lastOut_ && lastOut_ != '\n') { out_ += '\n'; lastOut_ = '\n'; }
}

void PskAutoDecoder::setToneHz(double hz) {
    for (int i = 0; i < 3; ++i) decs_[i].setToneHz(hz);
}

void PskAutoDecoder::process(const float* x, std::size_t n, double rate) {
    if (!(rate > 0) || !finite(rate) || !x) return;
    t_ += n / rate;
    for (int i = 0; i < 3; ++i) {
        PskDecoder& d = decs_[i];
        const unsigned n0 = d.characters();
        d.process(x, n, rate);
        scratch_.clear();
        d.takeText(scratch_);
        const unsigned got = d.characters() - n0;
        if (got == 0) { score_[i] *= 0.995; continue; }
        if (d.timingQuality() > 0.7) {
            score_[i] = score_[i] + got < 12 ? score_[i] + got : 12;
            lastAt_[i] = t_;
        }
        if (i == cur_) {
            out_ += scratch_;
            chars_ += static_cast<unsigned>(scratch_.size());
            if (!scratch_.empty()) lastOut_ = scratch_[scratch_.size() - 1];
            since_[0] = since_[1] = since_[2] = 0;
        } else {
            since_[i] += got;
            pend_[i] += scratch_;
            if (pend_[i].size() > 40) pend_[i].erase(0, pend_[i].size() - 40);
        }
    }
    // Choose: the fastest decoder with 3+ good characters in the last 3 s.
    int pick = -1;
    for (int i = 2; i >= 0; --i)
        if (score_[i] >= 3 && t_ - lastAt_[i] < 3) { pick = i; break; }
    // (A slower decoder can print a little rubbish on a faster signal: once a
    //  speed is chosen, dropping to a slower one also takes 5 characters from
    //  it since the chosen one last printed, so the end of an over isn't
    //  followed by a few stray characters.)
    if (pick >= 0 && pick != cur_ &&
        (cur_ < 0 || pick > cur_ || (t_ - lastAt_[cur_] > 3 && since_[pick] >= 5))) {
        if (lastOut_ && lastOut_ != '\n') { out_ += '\n'; lastOut_ = '\n'; }
        out_ += pend_[pick];
        chars_ += static_cast<unsigned>(pend_[pick].size());
        if (!pend_[pick].empty()) lastOut_ = pend_[pick][pend_[pick].size() - 1];
        for (int i = 0; i < 3; ++i) pend_[i].clear();
        cur_ = pick;
    }
}

void PskAutoDecoder::takeText(std::string& out) {
    out += out_;
    out_.clear();
}

Status PskAutoDecoder::status() const {
    const PskDecoder& d = decs_[cur_ < 0 ? 0 : cur_];
    Status s;
    s.locked = cur_ >= 0 && d.isOpen();
    s.signal = s.locked;
    s.quality = cur_ >= 0 ? d.timingQuality() : 0;
    s.toneHz = d.toneHz();
    s.baud = cur_ >= 0 ? d.baud() : 0;
    s.characters = chars_;
    s.mode = cur_ >= 0 ? kNames[cur_] : "";
    char buf[96];
    const double off = d.toneHz() - pskAudioHz;
    if (s.locked) std::snprintf(buf, sizeof buf, "%s receiving  tone %+.0f Hz", s.mode.c_str(), off);
    else if (cur_ >= 0) std::snprintf(buf, sizeof buf, "%s waiting for the signal", s.mode.c_str());
    else std::snprintf(buf, sizeof buf, "waiting for a signal: PSK31, PSK63 or PSK125");
    s.info = buf;
    return s;
}

}  // namespace dec
