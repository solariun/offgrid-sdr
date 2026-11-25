// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Navtex.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The NAVTEX and DSC decoders described in Navtex.h: a port of
//  OFFgrid-SDR's NavtexDecoder (tuneUp, process, bit, slot, char, header)
//  and DscDecoder (bit, unpack, slot, symbol, flush, parse), by diddy-boy,
//  index.html.  Comments in brackets carry the browser version's notes.
//  Differences from the browser version:
//    - the tone pair search, bits read while the pair is out of view, the
//      phasing tie-break, phasing-based DX/RX parity, and DSC clearing the
//      DX/RX evidence on sync loss (see Navtex.h; each is marked below);
//    - oscillators are rotating phasors instead of cos/sin per sample;
//    - bit histories are fixed ring windows (no arrays growing per bit);
//    - DscDecoder::reset() also forgets the polarity and DX/RX parity (the
//      browser keeps them across its reset);
//    - messages and calls are also streamed as text (takeText()); calls
//      carry no clock time (the library has no clock: the caller stamps).
//
//  How it fits in
//  --------------
//  See Navtex.h.
//
//  Contributing
//  ------------
//  Keep the function names parallel to the JavaScript.
// ============================================================================
#include "Navtex.h"
#include "Codes.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace dec {

namespace {

const double kPi = 3.14159265358979323846;
const int kNA = 4096;
const char* const kDot = "\xC2\xB7";   // the middle dot used as a separator

std::string trim(const std::string& s) {
    std::size_t a = 0, b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\n' || s[a] == '\r' || s[a] == '\t')) ++a;
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\n' || s[b - 1] == '\r' || s[b - 1] == '\t')) --b;
    return s.substr(a, b - a);
}

bool endsWith(const std::string& s, const char* t) {
    const std::size_t n = std::char_traits<char>::length(t);
    return s.size() >= n && s.compare(s.size() - n, n, t) == 0;
}

}  // namespace

std::string NavtexMessage::header() const {
    std::string id2;
    for (std::size_t i = 0; i < id.size(); ++i)
        if (id[i] != ' ' && id[i] != '\n' && id[i] != '\t') id2 += id[i];
    const char st = id2.size() > 0 ? id2[0] : '?';
    const char sub = id2.size() > 1 ? id2[1] : '?';
    const std::string num = id2.size() > 2 ? id2.substr(2, 2) : std::string();
    std::string h = "Station ";
    h += st;
    h += " ";
    h += kDot;
    h += " ";
    const char* name = codes::navtexSubject(sub);
    if (name) h += name;
    else { h += "Subject "; h += sub; }
    if (!num.empty()) { h += " "; h += kDot; h += " No. "; h += num; }
    return h;
}

// ---------------------------------------------------------------------------
NavtexDecoder::NavtexDecoder() : rate_(0), D_(1), fs2_(8000), fft_(kNA) {
    fbuf_.resize(kNA);
    P_.resize(kNA / 2);
    buf_.assign(kNA, 0.f);
    win_.resize(kNA);
    for (int i = 0; i < kNA; ++i) win_[i] = static_cast<float>(0.5 - 0.5 * std::cos(2 * kPi * i / kNA));
    spb_ = 480;
    NavtexDecoder::reset();
}

void NavtexDecoder::reset() {
    mark_ = 1585; space_ = 1415; lock_ = false; found_ = false; snrDb_ = 0; lostAt_ = 0; level_ = 1;
    mI_ = mQ_ = sI_ = sQ_ = mI2_ = mQ2_ = sI2_ = sQ2_ = 0;
    pmRe_ = 1; pmIm_ = 0; psRe_ = 1; psIm_ = 0; rmRe_ = 1; rmIm_ = 0; rsRe_ = 1; rsIm_ = 0;
    rotMark_ = rotSpace_ = -1; renorm_ = 0;
    ph_ = 0; bits_ = 0; nb_ = 0;
    for (int k = 0; k < 20; ++k) vHist_[k].clear();
    for (int k = 0; k < 14; ++k) pHist_[k].clear();
    sync_ = -1;
    for (int k = 0; k < 8; ++k) slotRing_[k] = 0;
    slotPos_ = 0; slotLen_ = 0; slotN_ = 0;
    mHist_[0].clear(); mHist_[1].clear();
    phaseVote_[0] = phaseVote_[1] = 0;
    figs_ = false; raw_.clear(); msgs_.clear(); curOpen_ = false;
    chars_ = 0; fixed_ = 0; lost_ = 0;
    accN_ = 0; decim_ = 0;
    std::fill(buf_.begin(), buf_.end(), 0.f);
    bp_ = 0; since_ = 0;
    out_.clear();
}

void NavtexDecoder::retune() { lock_ = false; found_ = false; sync_ = -1; }

void NavtexDecoder::setRate(double rate) {
    rate_ = rate;
    D_ = std::max(1, static_cast<int>(std::floor(rate / 8000 + 0.5)));
    fs2_ = rate / D_;
    std::fill(buf_.begin(), buf_.end(), 0.f);
    bp_ = 0; since_ = 0; accN_ = 0; decim_ = 0;
    spb_ = rate / 100;
    if (ph_ >= spb_) ph_ = 0;
    rotMark_ = rotSpace_ = -1;
}

// The 170 Hz tone pair.
// (changed in this port) The browser takes the two strongest single-bin peaks 170 +-14 Hz apart,
// as RTTY's tuneUp. With 100 baud data each tone is ~200 Hz wide, so those peaks wander by +-20 Hz
// from one 0.5 s spectrum to the next, and DSC's dot pattern (lines every 100 Hz around the
// centre) gave false pairs that pulled the tones away just before each call. Here the pair is
// found by energy: for every candidate space frequency, the power within +-20 Hz of it and
// within +-20 Hz of it + 170 Hz; the score is the smaller of the two (both tones must be there),
// the best candidate is refined between bins. A pair needs each window 4x its noise (6 dB).
// The shift is taken as exactly 170 Hz (NAVTEX and DSC both use it).
void NavtexDecoder::tuneUp() {
    const int N = kNA;
    for (int i = 0; i < N; ++i) fbuf_[i] = std::complex<float>(buf_[(bp_ + i) % N] * win_[i], 0.f);
    fft_.forward(&fbuf_[0]);
    const double bin = fs2_ / N;
    for (int j = 0; j < N / 2; ++j) P_[j] = std::norm(fbuf_[j]);
    const int lo = static_cast<int>(std::ceil(250 / bin)), hi = std::min(N / 2 - 1, static_cast<int>(std::floor(3000 / bin)));
    if (hi - lo < 4) return;
    sorted_.assign(P_.begin() + lo, P_.begin() + hi);
    std::nth_element(sorted_.begin(), sorted_.begin() + sorted_.size() / 2, sorted_.end());
    double med = sorted_[sorted_.size() / 2];
    if (!(med > 0)) med = 1e-30;
    // prefix sums of the power, for window sums
    cum_.resize(N / 2 + 1);
    cum_[0] = 0;
    for (int j = 0; j < N / 2; ++j) cum_[j + 1] = cum_[j] + P_[j];
    const int w = std::max(1, static_cast<int>(std::floor(20 / bin + 0.5)));
    const double sh = 170 / bin;
    const double noiseWin = (med / 0.693) * (2 * w + 1);   // (the median of chi-square(2) is ln 2 of its mean)
    int best = -1;
    double bsc = 0;
    for (int a = lo + w; ; ++a) {
        const int b = static_cast<int>(std::floor(a + sh + 0.5));
        if (b + w >= hi) break;
        const double ea = cum_[a + w + 1] - cum_[a - w], eb = cum_[b + w + 1] - cum_[b - w];
        const double sc = std::min(ea, eb);
        if (sc > bsc) { bsc = sc; best = a; }
    }
    if (best > lo + w && bsc > noiseWin * 4) {
        // refine: parabola through the summed energy of the neighbouring candidates
        double e[3];
        for (int k = -1; k <= 1; ++k) {
            const int a = best + k, b = static_cast<int>(std::floor(a + sh + 0.5));
            e[k + 1] = (cum_[a + w + 1] - cum_[a - w]) + (b + w + 1 <= N / 2 ? cum_[b + w + 1] - cum_[b - w] : 0);
        }
        const double den = e[0] - 2 * e[1] + e[2];
        const double off = den < 0 ? std::max(-0.5, std::min(0.5, 0.5 * (e[0] - e[2]) / den)) : 0;
        const double fa = (best + off) * bin, fb = fa + 170;
        if (!lock_ || std::fabs(fa - space_) > 30) { space_ = fa; mark_ = fb; }   // a different signal: at once
        else { space_ += 0.5 * (fa - space_); mark_ = space_ + 170; }            // the same one: smoothly
        lock_ = true; lostAt_ = 0; found_ = true;
        snrDb_ = 10 * std::log10(std::max(1e-6, 2 * bsc / (noiseWin * 3000 / ((2 * w + 1) * bin))));
    } else if (lock_ && ++lostAt_ >= 4) {
        lock_ = false; sync_ = -1;
    }
}

void NavtexDecoder::process(const float* x, std::size_t n, double rate) {
    if (rate <= 0) return;
    if (rate != rate_) setRate(rate);
    const double R = rate_, SPB = spb_, a = 1 - std::exp(-2 * kPi * 75 / R);
    if (rotMark_ != mark_) { const double w = 2 * kPi * mark_ / R; rmRe_ = std::cos(w); rmIm_ = std::sin(w); rotMark_ = mark_; }
    if (rotSpace_ != space_) { const double w = 2 * kPi * space_ / R; rsRe_ = std::cos(w); rsIm_ = std::sin(w); rotSpace_ = space_; }
    for (std::size_t i = 0; i < n; ++i) {
        const double v = x[i];
        decim_ += v;
        if (++accN_ >= D_) {
            buf_[bp_] = static_cast<float>(decim_ / D_);
            bp_ = (bp_ + 1) % kNA; decim_ = 0; accN_ = 0;
            if (++since_ >= kNA / 2) {
                since_ = 0; tuneUp();
                // (new tones take effect at once: the browser picks them up at its next block)
                if (rotMark_ != mark_) { const double w = 2 * kPi * mark_ / R; rmRe_ = std::cos(w); rmIm_ = std::sin(w); rotMark_ = mark_; }
                if (rotSpace_ != space_) { const double w = 2 * kPi * space_ / R; rsRe_ = std::cos(w); rsIm_ = std::sin(w); rotSpace_ = space_; }
            }
        }
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
        const double m = mI2_ * mI2_ + mQ2_ * mQ2_, s = sI2_ * sI2_ + sQ2_ * sQ2_, d = (m - s) / (m + s + 1e-20);
        const int prev = level_;
        if (d > 0.1) level_ = 1;
        else if (d < -0.1) level_ = 0;
        // bit clock: transitions should fall on bit boundaries; nudge the clock towards them
        if (level_ != prev) {
            double e = ph_;
            if (e > SPB / 2) e -= SPB;
            ph_ -= e * 0.15;
        }
        const double before = ph_;
        ph_ += 1;
        // (changed in this port: the browser reads bits only while the tone pair is in view. Here also
        //  after it has been found once, on the last tones: a DSC call follows a dot pattern that
        //  hides the pair, and the call is over before a new search would find it. Noise cannot
        //  fake text: sync needs 26 of 40 (NAVTEX) or 9 of 12 (DSC) valid codes.)
        if (before < SPB / 2 && ph_ >= SPB / 2 && (lock_ || found_)) bit(level_);
        if (ph_ >= SPB) ph_ -= SPB;
    }
}

void NavtexDecoder::bit(int b) {
    bits_ = ((bits_ << 1) | b) & 0x7F;
    const unsigned long long n = nb_++;
    if (n < 6) return;
    // every boundary (n mod 7) and both polarities: how often is the last 7 bits a valid code?
    const int r = static_cast<int>(n % 7);
    for (int pol = 0; pol < 2; ++pol) {
        const int k = r + 7 * pol, code = pol ? (~bits_ & 0x7F) : bits_;
        vHist_[k].push(codes::pop7(code) == 4 ? 1 : 0, 40);
        pHist_[k].push(code == codes::kCcirAlpha || code == codes::kCcirBeta ? 1 : 0, 40);
    }
    // (added in this port: ties are broken by the phasing signals. The phasing pattern reads as
    //  valid codes at several boundaries; the browser took the first, often a wrong one, and lost
    //  the first characters of the message ("ZCZC") while the sync moved over. Only the right
    //  boundary and polarity show alpha and beta.)
    int best = 0;
    for (int k = 1; k < 14; ++k)
        if (vHist_[k].sum > vHist_[best].sum || (vHist_[k].sum == vHist_[best].sum && pHist_[k].sum > pHist_[best].sum)) best = k;
    if (sync_ >= 0 && sync_ != best && vHist_[sync_].sum == vHist_[best].sum && pHist_[sync_].sum >= pHist_[best].sum) best = sync_;
    if (vHist_[best].sum >= 26) sync_ = best;
    else if (vHist_[best].sum < 16) sync_ = -1;
    if (sync_ >= 0 && r == sync_ % 7) slot(sync_ >= 7 ? (~bits_ & 0x7F) : bits_);
}

// a new 7-bit slot code (polarity and boundary already chosen)
void NavtexDecoder::slot(int code) {
    pushSlot(code);
    const unsigned n = slotN_++;
    if (slotLen_ < 6) return;
    // which slots are first copies (DX)? the ones repeated 5 slots later
    const int a = slotAgo(6), b = slotAgo(1), p = static_cast<int>((n - 5) & 1);
    const int m = a == b && codes::pop7(a) == 4 && a != codes::kCcirAlpha && a != codes::kCcirBeta ? 1 : 0;
    mHist_[p].push(m, 30);
    int dxPar = mHist_[0].sum >= mHist_[1].sum ? 0 : 1;
    bool known = std::max(mHist_[0].sum, mHist_[1].sum) >= 3;
    // (added in this port) Before three repeated characters have shown which slots are first
    // copies, the phasing that starts every transmission tells: phasing signal 1 (beta) goes in
    // the DX slots, phasing signal 2 (alpha) in the RX slots. Without this the first characters
    // of a message heard from its start ("ZCZC") were lost.
    const int np = static_cast<int>(n & 1);
    phaseVote_[0] *= 0.97; phaseVote_[1] *= 0.97;
    if (code == codes::kCcirBeta) phaseVote_[np] += 1;
    else if (code == codes::kCcirAlpha) phaseVote_[np ^ 1] += 1;
    if (!known && std::fabs(phaseVote_[0] - phaseVote_[1]) >= 8) {
        dxPar = phaseVote_[0] > phaseVote_[1] ? 0 : 1;
        known = true;
    }
    if (p == dxPar && known) {
        const bool okDx = codes::pop7(a) == 4, okRx = codes::pop7(b) == 4;
        if (okDx) character(a);
        else if (okRx) { character(b); ++fixed_; }
        else { character(-1); ++lost_; }
    }
}

void NavtexDecoder::character(int code) {
    using namespace codes;
    if (code == kCcirAlpha || code == kCcirBeta || code == kCcirRepeat || code == kCcirBlank || code == kCcirCarriageReturn) return;
    if (code == kCcirLetters) { figs_ = false; return; }
    if (code == kCcirFigures) { figs_ = true; return; }
    const char* ch = code < 0 ? "_" : code == kCcirSpace ? " " : code == kCcirLineFeed ? "\n" : ccirChar(code, figs_);
    if (!ch || !*ch || (ch[0] == '\x07' && ch[1] == 0)) return;
    if (code == kCcirSpace) figs_ = false;   // (unshift on space, as most NAVTEX decoders)
    ++chars_;
    raw_ += ch;
    if (raw_.size() > 8000) raw_.erase(0, raw_.size() - 6000);
    // messages: ZCZC B1B2B3B4 ... NNNN
    if (endsWith(raw_, "ZCZC")) {
        msgs_.push_back(NavtexMessage());
        if (msgs_.size() > 20) msgs_.pop_front();
        curOpen_ = true;
        return;
    }
    if (!curOpen_ || msgs_.empty()) return;
    NavtexMessage& cur = msgs_.back();
    if (cur.done) return;
    if (!cur.idDone) {   // the header (e.g. "SA23") ends at the line break
        if (ch[0] == '\n' || (ch[0] == ' ' && cur.id.size() >= 4)) { cur.idDone = true; return; }
        if (ch[0] != ' ') cur.id += ch;
        if (cur.id.size() > 8) cur.idDone = true;
        return;
    }
    cur.text += ch;
    if (endsWith(cur.text, "NNNN")) {
        cur.text = trim(cur.text.substr(0, cur.text.size() - 4));
        cur.done = true;
        curOpen_ = false;
        out_ += cur.header();
        out_ += "\n";
        out_ += cur.text;
        out_ += "\n\n";
    }
}

void NavtexDecoder::takeText(std::string& out) {
    out += out_;
    out_.clear();
}

Status NavtexDecoder::status() const {
    Status s;
    s.signal = lock_;
    s.locked = lock_ && sync_ >= 0;
    s.snrDb = lock_ ? snrDb_ : 0;
    s.toneHz = mark_;
    s.shiftHz = mark_ - space_;
    s.baud = 100;
    s.characters = chars_;
    const double tot = chars_ + lost_;
    s.quality = s.locked && tot > 0 ? 1 - lost_ / tot : 0;
    s.mode = "NAVTEX";
    char buf[128];
    if (!lock_) std::snprintf(buf, sizeof buf, "searching for the two tones");
    else if (sync_ < 0) std::snprintf(buf, sizeof buf, "tones found, finding the characters");
    else std::snprintf(buf, sizeof buf, "in sync  %u repaired  %u lost", fixed_, lost_);
    s.info = buf;
    return s;
}

// ---------------------------------------------------------------------------
DscDecoder::DscDecoder() { DscDecoder::reset(); }

void DscDecoder::reset() {
    NavtexDecoder::reset();
    calls_.clear(); sym_.clear(); syms_ = 0;
    w10_ = 0; pol_ = 0; polLocked_ = false; phHits_[0] = phHits_[1] = 0; dxPar_ = -1;
}

// bits as received -> (check << 7) | info, info LSB first, check MSB first
int DscDecoder::unpack(int w) {
    const int info = w & 0x7F, c = (w >> 7) & 7, chk = ((c & 1) << 2) | (c & 2) | ((c >> 2) & 1);
    return (chk << 7) | info;
}

// 10-bit words; received bit by bit, the first bit is the least significant data bit
void DscDecoder::bit(int b) {
    w10_ = ((w10_ >> 1) | (b << 9)) & 0x3FF;
    const unsigned long long n = nb_++;
    if (n < 9) return;
    const int r = static_cast<int>(n % 10);
    for (int pol = 0; pol < 2; ++pol) {   // every boundary, both polarities: how often is it a valid symbol?
        const int k = r + 10 * pol, w = pol ? (~w10_ & 0x3FF) : w10_;
        vHist_[k].push(codes::dscCheckOk(unpack(w)) ? 1 : 0, 12);   // (12 symbols: a DSC call is only ~5 s)
    }
    // the boundary: where symbols come out valid. The polarity can't come from that (the check
    // counts the zeros, so an inverted symbol checks out too): it comes from the phasing symbols
    // (125, 104-111) that start every call, which only appear the right way up.
    int best = 0;
    for (int k = 1; k < 10; ++k)
        if (vHist_[k].sum > vHist_[best].sum) best = k;
    if (vHist_[best].sum >= 9) sync_ = best;
    else if (vHist_[best].sum < 5) {
        if (sync_ >= 0) flush();
        sync_ = -1; polLocked_ = false; phHits_[0] = phHits_[1] = 0; dxPar_ = -1;
        // (added in this port: the DX/RX evidence goes too; the slots counted after the next
        //  sync have no fixed relation to these. The browser kept it, and the next call's
        //  first symbols were then paired wrongly)
        mHist_[0].clear(); mHist_[1].clear();
    }
    if (sync_ >= 0 && r == sync_) {
        int vals[2];
        for (int pol = 0; pol < 2; ++pol) {
            const int w = unpack(pol ? (~w10_ & 0x3FF) : w10_);
            vals[pol] = codes::dscCheckOk(w) ? (w & 0x7F) : -1;
        }
        // (once the phasing has settled it, the polarity is kept for the call: some ordinary
        //  symbols read upside down equal phasing values, e.g. 23 -> 104)
        if (!polLocked_) {
            for (int pol = 0; pol < 2; ++pol) {
                const int v = vals[pol];
                phHits_[pol] = phHits_[pol] * 0.98 + ((v == 125 || (v >= 104 && v <= 111)) ? 1 : 0);
            }
            pol_ = phHits_[1] > phHits_[0] + 0.5 ? 1 : phHits_[0] > phHits_[1] + 0.5 ? 0 : pol_;
            if (std::fabs(phHits_[1] - phHits_[0]) >= 4) polLocked_ = true;
        }
        dscSlot(vals[pol_]);
    }
}

// DX/RX: each symbol again 5 slots later; use whichever copy is valid
void DscDecoder::dscSlot(int v) {
    pushSlot(v);
    const unsigned n = slotN_++;
    if (v == 125) dxPar_ = static_cast<int>(n & 1);   // phasing: 125 is always in the DX slots
    // (added in this port) and 104-111 always in the RX slots, once the polarity is settled
    else if (dxPar_ < 0 && polLocked_ && v >= 104 && v <= 111) dxPar_ = static_cast<int>((n & 1) ^ 1);
    if (slotLen_ < 6) return;
    const int a = slotAgo(6), b = slotAgo(1), p = static_cast<int>((n - 5) & 1);
    const int m = a == b && a >= 0 && a != 125 && !(a >= 104 && a <= 111) ? 1 : 0;
    mHist_[p].push(m, 30);
    const int dx = dxPar_ >= 0 ? dxPar_ : mHist_[0].sum >= mHist_[1].sum ? 0 : 1;
    if (p == dx && (dxPar_ >= 0 || std::max(mHist_[0].sum, mHist_[1].sum) >= 3)) {
        // the first copy if it's valid, else the second; but if both are valid and DISAGREE,
        // neither can be trusted (two bit errors can turn one valid symbol into another)
        const int v2 = a >= 0 && b >= 0 && a != b ? -1 : a >= 0 ? a : b;
        if (a < 0 && b >= 0) ++fixed_;
        if (v2 < 0) ++lost_;
        symbol(v2);
    }
}

// collect a call: format specifier (twice) ... end of sequence (117/122/127)
void DscDecoder::symbol(int v) {
    if (sym_.empty() && !codes::dscFormat(v)) return;   // (wait for a format symbol: the call starts there)
    sym_.push_back(v); ++syms_; ++chars_;
    if (sym_.size() > 3 && (v == 117 || v == 122 || v == 127)) { parse(sym_); sym_.clear(); return; }
    if (sym_.size() >= 26) { parse(sym_); sym_.clear(); }   // the end was damaged: show what came
}

void DscDecoder::flush() {   // the signal ended mid-call
    if (sym_.size() >= 8) parse(sym_);
    sym_.clear();
}

void DscDecoder::parse(const std::vector<int>& s) {
    // the fields start after the format symbols (normally two; one if the other copy was damaged)
    const int fmt = s[0];
    std::size_t at = 0;
    while (at < s.size() && (s[at] == fmt || (at == 1 && s[at] < 0))) ++at;
    struct Mmsi {
        static std::string get(const std::vector<int>& s, std::size_t k) {
            std::string r;
            for (std::size_t i = k; i < k + 5 && i < s.size(); ++i) {
                char b[8];
                if (s[i] < 0) r += "??";
                else { std::snprintf(b, sizeof b, "%02d", s[i]); r += b; }
            }
            return r.substr(0, 9);
        }
    };
    struct Look {
        static const char* at(const std::vector<int>& s, std::size_t k, const char* (*f)(int)) {
            if (k >= s.size()) return "";
            const char* r = f(s[k]);
            return r ? r : "";
        }
    };
    DscCall c;
    const char* fname = codes::dscFormat(fmt);
    if (fname) c.format = fname;
    else { char b[24]; std::snprintf(b, sizeof b, "Format %d", fmt); c.format = b; }
    if (fmt == 112) {
        c.category = "Distress"; c.from = Mmsi::get(s, at); c.nature = Look::at(s, at + 5, codes::dscNature); c.to = "all ships";
    } else if (fmt == 116) {
        c.to = "all ships"; c.category = Look::at(s, at, codes::dscCategory); c.from = Mmsi::get(s, at + 1);
    } else {
        c.to = fmt == 102 ? std::string("ships in an area") : "MMSI " + Mmsi::get(s, at);
        c.category = Look::at(s, at + 5, codes::dscCategory);
        c.from = Mmsi::get(s, at + 6);
    }
    // (a call joined part-way through, with most of the caller unreadable, would only confuse)
    if (std::count(c.from.begin(), c.from.end(), '?') > 4) return;
    c.text = c.category.empty() ? c.format : c.category;
    if (!c.nature.empty()) c.text += " (" + c.nature + ")";
    c.text += std::string(" ") + kDot + " " + c.to + " " + kDot + " from MMSI " + c.from;
    c.symbols = s;
    out_ += c.text;
    out_ += "\n";
    calls_.push_back(c);
    if (calls_.size() > 20) calls_.pop_front();
}

Status DscDecoder::status() const {
    Status s;
    s.signal = lock_;
    s.locked = lock_ && sync_ >= 0;
    s.snrDb = lock_ ? snrDb_ : 0;
    s.toneHz = mark_;
    s.shiftHz = mark_ - space_;
    s.baud = 100;
    s.characters = static_cast<unsigned>(calls_.size());
    const double tot = syms_ + lost_;
    s.quality = s.locked && tot > 0 ? 1 - lost_ / tot : 0;
    s.mode = "DSC";
    s.info = !lock_ ? "searching for the two tones" : sync_ < 0 ? "tones found, waiting for a call" : "receiving a call";
    return s;
}

}  // namespace dec
