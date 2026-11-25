// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/Rds.cpp
//  Layer:  Signal processing
//
//  What this file does
//  -------------------
//  The RDS decoder, ported line by line from OFFgrid-SDR's RdsDecoder
//  (index.html, "RDS decoder begin", by diddy-boy), and an RDS encoder for
//  the demo source.
//
//  Decoder, signal path (all per sample of the FM multiplex):
//    1. mix the 57 kHz subcarrier down to 0 Hz (NCO);
//    2. decimate by 16 with three cascaded moving averages (their nulls fall
//       on every band that would alias onto 0 Hz), then a 63-tap low-pass
//       (2.6 kHz, Blackman);
//    3. linear resampling to 19 kHz = 16 samples per RDS bit (1187.5 bit/s);
//    4. AGC + Costas loop: recovers the suppressed BPSK carrier phase;
//    5. biphase matched filter (two half-sines of opposite sign per bit),
//       bit timing = the sample phase (of 16) with the strongest output;
//    6. differential decoding (bit = this symbol XOR the previous one);
//    7. 26-bit block sync: a block's 10 check bits minus the CRC of its 16
//       data bits must equal one of the offset words A, B, C, C', D, and two
//       such blocks must arrive a whole number of blocks apart in the right
//       order.  Once synced, a damaged block is repaired when its syndrome
//       matches a burst of up to 5 bit errors;
//    8. groups: 0A/0B PS name, 2A/2B RadioText, 4A clock, plus PI, PTY, TP,
//       TA, MS.  Repaired characters are only shown once received the same
//       way twice, so noise never shows as wrong letters.
//
//  Encoder: the reverse of 6-8 (biphase shaping and the 57 kHz carrier are
//  done by DemoSource).
//
//  How it fits in
//  --------------
//  See Rds.h.
// ============================================================================
#include "Rds.h"
#include "DspCommon.h"

#include <algorithm>
#include <cstdio>

namespace dsp {
namespace detail {

namespace {

const char* const kPtyRds[32] = {
    "None", "News", "Current Affairs", "Information", "Sport", "Education", "Drama", "Culture",
    "Science", "Varied", "Pop Music", "Rock Music", "Easy Listening", "Light Classical", "Serious Classical", "Other Music",
    "Weather", "Finance", "Children's", "Social Affairs", "Religion", "Phone-in", "Travel", "Leisure",
    "Jazz Music", "Country Music", "National Music", "Oldies Music", "Folk Music", "Documentary", "Alarm Test", "Alarm"};
const char* const kPtyRbds[32] = {
    "None", "News", "Information", "Sports", "Talk", "Rock", "Classic Rock", "Adult Hits",
    "Soft Rock", "Top 40", "Country", "Oldies", "Soft", "Nostalgia", "Jazz", "Classical",
    "Rhythm and Blues", "Soft R&B", "Language", "Religious Music", "Religious Talk", "Personality", "Public", "College",
    "Spanish Talk", "Spanish Music", "Hip Hop", "Unassigned", "Unassigned", "Weather", "Emergency Test", "Emergency"};

// RDS character set (IEC 62106 table E.1) for codes 0x80-0xFF, as UTF-8;
// 0x20-0x7E is ASCII.  Same table as OFFgrid-SDR's RDS_HI.
const char* const kRdsHi[128] = {
    "\xc3\xa1", "\xc3\xa0", "\xc3\xa9", "\xc3\xa8", "\xc3\xad", "\xc3\xac", "\xc3\xb3", "\xc3\xb2",
    "\xc3\xba", "\xc3\xb9", "\xc3\x91", "\xc3\x87", "\xc5\x9e", "\xce\xb2", "\xc2\xa1", "\xc4\xb2",
    "\xc3\xa2", "\xc3\xa4", "\xc3\xaa", "\xc3\xab", "\xc3\xae", "\xc3\xaf", "\xc3\xb4", "\xc3\xb6",
    "\xc3\xbb", "\xc3\xbc", "\xc3\xb1", "\xc3\xa7", "\xc5\x9f", "\xc4\x9f", "\xc4\xb1", "\xc4\xb3",
    "\xc2\xaa", "\xc2\xa9", "\xe2\x80\xb0", "\xc4\x9e", "\xc4\x9b", "\xc5\x88", "\xc5\x91", "\xcf\x80",
    "\xe2\x82\xac", "\xc2\xa3", "\x24", "\xe2\x86\x90", "\xe2\x86\x91", "\xe2\x86\x92", "\xe2\x86\x93", "\xc2\xba",
    "\xc2\xb9", "\xc2\xb2", "\xc2\xb3", "\xc2\xb1", "\xc4\xb0", "\xc5\x84", "\xc5\xb1", "\xc2\xb5",
    "\xc2\xbf", "\xc3\xb7", "\xc2\xb0", "\xc2\xbc", "\xc2\xbd", "\xc2\xbe", "\xc2\xa7", "\xc3\x81",
    "\xc3\x80", "\xc3\x89", "\xc3\x88", "\xc3\x8d", "\xc3\x8c", "\xc3\x93", "\xc3\x92", "\xc3\x9a",
    "\xc3\x99", "\xc5\x98", "\xc4\x8c", "\xc5\xa0", "\xc5\xbd", "\xc3\x90", "\xc4\xbf", "\xc3\x82",
    "\xc3\x84", "\xc3\x8a", "\xc3\x8b", "\xc3\x8e", "\xc3\x8f", "\xc3\x94", "\xc3\x96", "\xc3\x9b",
    "\xc3\x9c", "\xc5\x99", "\xc4\x8d", "\xc5\xa1", "\xc5\xbe", "\xc4\x91", "\xc5\x80", "\xc3\x83",
    "\xc3\x85", "\xc3\x86", "\xc5\x92", "\xc5\xb7", "\xc3\x9d", "\xc3\x95", "\xc3\x98", "\xc3\x9e",
    "\xc5\x8a", "\xc5\x94", "\xc4\x86", "\xc5\x9a", "\xc5\xb9", "\xc5\xa6", "\xc3\xb0", "\xc3\xa3",
    "\xc3\xa5", "\xc3\xa6", "\xc5\x93", "\xc5\xb5", "\xc3\xbd", "\xc3\xb5", "\xc3\xb8", "\xc3\xbe",
    "\xc5\x8b", "\xc5\x95", "\xc4\x87", "\xc5\x9b", "\xc5\xba", "\xc5\xa7", " ", " "};

std::string rdsChar(int c) {
    if (c >= 0x20 && c < 0x7f) return std::string(1, static_cast<char>(c));
    if (c >= 0x80 && c <= 0xff) return kRdsHi[c - 0x80];
    return " ";
}

// Offset word -> block number (A=0, B=1, C/C'=2, D=3), -1 if none.
int offsetType(std::uint32_t syndrome) {
    switch (syndrome) {
        case kRdsOffsetA: return 0;
        case kRdsOffsetB: return 1;
        case kRdsOffsetC: return 2;
        case kRdsOffsetCp: return 2;
        case kRdsOffsetD: return 3;
        default: return -1;
    }
}

// Syndrome of an error pattern e (26 bits): the check it leaves after
// removing the offset word.
std::uint32_t rdsSyn(std::uint32_t e) { return (e & 0x3FF) ^ rdsCrc(e >> 10); }

// Syndrome -> burst error pattern (bursts of 1..5 bits), 0 = not repairable.
struct FixTable {
    std::uint32_t e[1024];
    FixTable() {
        for (int i = 0; i < 1024; ++i) e[i] = 0;
        for (int len = 1; len <= 5; ++len) {
            const int mids = len > 2 ? 1 << (len - 2) : 1;
            for (int mid = 0; mid < mids; ++mid) {
                const std::uint32_t pat = len == 1 ? 1u : ((1u << (len - 1)) | (static_cast<std::uint32_t>(mid) << 1) | 1u);
                for (int sh = 0; sh + len <= 26; ++sh) {
                    const std::uint32_t err = pat << sh, sy = rdsSyn(err);
                    if (sy && !e[sy]) e[sy] = err;
                }
            }
        }
    }
};
const FixTable& fixTable() {
    static const FixTable t;
    return t;
}

// Matched filter for one biphase bit at 16 samples/bit: a positive half-sine
// then a negative one (RDS_MF in the JavaScript).
struct MatchedFilter {
    double v[16];
    MatchedFilter() {
        for (int j = 0; j < 16; ++j) v[j] = (j < 8 ? 1.0 : -1.0) * std::sin(kPi * ((j % 8) + 0.5) / 8.0);
    }
};
const MatchedFilter& matchedFilter() {
    static const MatchedFilter m;
    return m;
}

// Modified Julian Day -> calendar date (IEC 62106 annex G).
void mjdToDate(int mjd, int& y, int& m, int& d) {
    const int yp = static_cast<int>((mjd - 15078.2) / 365.25);
    const int mp = static_cast<int>((mjd - 14956.1 - static_cast<int>(yp * 365.25)) / 30.6001);
    d = mjd - 14956 - static_cast<int>(yp * 365.25) - static_cast<int>(mp * 30.6001);
    const int k = (mp == 14 || mp == 15) ? 1 : 0;
    y = yp + k + 1900;
    m = mp - 1 - k * 12;
}

}  // namespace

std::uint32_t rdsCrc(std::uint32_t info) {
    std::uint32_t r = (info & 0xFFFF) << 10;
    for (int b = 25; b >= 10; --b)
        if (r & (1u << b)) r ^= 0x5B9u << (b - 10);
    return r & 0x3FF;
}

const char* rdsPtyName(int pty, bool rbds) {
    if (pty < 0 || pty > 31) return "";
    return rbds ? kPtyRbds[pty] : kPtyRds[pty];
}

// ============================================================================
//  Decoder
// ============================================================================
RdsDecoder::RdsDecoder() : rate_(0.0) { clear(); }

void RdsDecoder::clear() {
    pi_ = -1;
    for (int i = 0; i < 8; ++i) {
        ps_[i] = " ";
        psKnown_[i] = false;
        psPend_[i] = -1;
    }
    for (int i = 0; i < 64; ++i) {
        rt_[i] = " ";
        rtPend_[i] = -1;
    }
    rtLen_ = 64;
    rtAB_ = -1;
    pty_ = -1;
    tp_ = ta_ = ms_ = false;
    hasCt_ = false;
    ctMjd_ = ctMin_ = ctOffsetHalfHours_ = 0;
    groups_ = 0;
    changes_ = 0;
    cleanB_.clear();
    reg_ = 0;
    bitN_ = 0;
    haveCand_ = false;
    candN_ = 0;
    candT_ = 0;
    synced_ = false;
    left_ = expect_ = prevRaw_ = 0;
    for (int i = 0; i < 4; ++i) {
        blk_[i] = 0;
        ok_[i] = rep_[i] = false;
    }
    qLen_ = qPos_ = qSum_ = qGood_ = 0;
    fixed_ = 0;
}

void RdsDecoder::setRate(double rate) {
    // Decimate by 16 with the moving averages, then a short FIR.
    rate_ = rate;
    M_ = 16;
    r2_ = rate / M_;
    const int N = 63;
    const double fc = 2600.0 / r2_;
    h_.assign(N, 0.0);
    double sum = 0;
    for (int i = 0; i < N; ++i) {
        const double x = i - (N - 1) / 2.0;
        const double w = 0.42 - 0.5 * std::cos(kTwoPi * i / (N - 1)) + 0.08 * std::cos(2 * kTwoPi * i / (N - 1));
        h_[i] = w * (x == 0 ? 2 * fc : std::sin(kTwoPi * fc * x) / (kPi * x));
        sum += h_[i];
    }
    for (int i = 0; i < N; ++i) h_[i] /= sum;
    firRe_.assign(2 * N, 0.0);
    firIm_.assign(2 * N, 0.0);
    firP_ = 0;
    box_.assign(6 * M_, 0.0);
    for (int i = 0; i < 6; ++i) bs_[i] = 0;
    bk_ = dec_ = 0;
    nco_ = 0;
    ncoInc_ = kTwoPi * 57000.0 / rate;
    rsStep_ = r2_ / 19000.0;
    rsPos_ = 0;
    prevZr_ = prevZi_ = 0;
    th_ = fr_ = amp_ = 0;
    for (int i = 0; i < 16; ++i) win_[i] = metric_[i] = 0;
    wn_ = best_ = 0;
    sn_ = 0;
    lastEmit_ = -16;
}

void RdsDecoder::process(const float* mpx, std::size_t n, double rate) {
    if (rate != rate_) setRate(rate);
    const int M = M_, o1 = 2 * M, o2 = 4 * M, N = static_cast<int>(h_.size());
    const double iM = 1.0 / M;
    double* B = &box_[0];
    double s0r = bs_[0], s0i = bs_[1], s1r = bs_[2], s1i = bs_[3], s2r = bs_[4], s2i = bs_[5];
    int k = bk_, dec = dec_;
    const double ph = nco_, inc = ncoInc_;
    double cr = std::cos(ph), ci = -std::sin(ph);
    const double dcr = std::cos(inc), dci = -std::sin(inc);
    for (std::size_t i = 0; i < n; ++i) {
        // 1. mix 57 kHz to 0 Hz
        const double x = mpx[i], xr = x * cr, xi = x * ci;
        const double t = cr * dcr - ci * dci;
        ci = cr * dci + ci * dcr;
        cr = t;
        // 2. three moving averages of length M (running sums)
        const int k2 = 2 * k;
        s0r += xr - B[k2];
        s0i += xi - B[k2 + 1];
        B[k2] = xr;
        B[k2 + 1] = xi;
        const double a1 = s0r * iM, b1 = s0i * iM;
        s1r += a1 - B[o1 + k2];
        s1i += b1 - B[o1 + k2 + 1];
        B[o1 + k2] = a1;
        B[o1 + k2 + 1] = b1;
        const double a2 = s1r * iM, b2 = s1i * iM;
        s2r += a2 - B[o2 + k2];
        s2i += b2 - B[o2 + k2 + 1];
        B[o2 + k2] = a2;
        B[o2 + k2 + 1] = b2;
        k = k + 1 == M ? 0 : k + 1;
        if (++dec == M) {  // one output per M inputs: FIR low-pass (2.6 kHz)
            dec = 0;
            const int p = firP_;
            const double vr = s2r * iM, vi = s2i * iM;
            firRe_[p] = vr;
            firRe_[p + N] = vr;
            firIm_[p] = vi;
            firIm_[p + N] = vi;
            firP_ = p + 1 == N ? 0 : p + 1;
            double yr = 0, yi = 0;
            for (int j = 0, q = p + 1; j < N; ++j, ++q) {
                yr += h_[j] * firRe_[q];
                yi += h_[j] * firIm_[q];
            }
            // 3. linear resampling to 19 kHz (16 samples per RDS bit)
            while (rsPos_ <= 1.0) {
                const double u = rsPos_;
                symbolSample(prevZr_ + (yr - prevZr_) * u, prevZi_ + (yi - prevZi_) * u);
                rsPos_ += rsStep_;
            }
            rsPos_ -= 1.0;
            prevZr_ = yr;
            prevZi_ = yi;
        }
    }
    bs_[0] = s0r;
    bs_[1] = s0i;
    bs_[2] = s1r;
    bs_[3] = s1i;
    bs_[4] = s2r;
    bs_[5] = s2i;
    bk_ = k;
    dec_ = dec;
    nco_ = std::fmod(ph + inc * static_cast<double>(n), kTwoPi);
}

void RdsDecoder::symbolSample(double zr, double zi) {
    // 4. AGC + Costas loop (BPSK, suppressed carrier)
    const double mag = std::sqrt(zr * zr + zi * zi);
    amp_ += 0.0005 * (mag - amp_);
    const double g = amp_ > 1e-12 ? 1.0 / amp_ : 0.0, c = std::cos(th_), s = std::sin(th_);
    const double ur = (zr * c + zi * s) * g, ui = (zi * c - zr * s) * g;
    const double err = std::max(-1.0, std::min(1.0, ur * ui));
    fr_ += 4e-6 * err;
    fr_ = std::max(-0.02, std::min(0.02, fr_));
    th_ += fr_ + 0.008 * err;
    if (th_ > kPi) th_ -= kTwoPi;
    else if (th_ < -kPi) th_ += kTwoPi;
    // 5. biphase bit timing: pick the sample phase (of 16) with the strongest
    //    half-bit contrast.
    win_[wn_] = ur;
    wn_ = (wn_ + 1) & 15;
    const int p = static_cast<int>(sn_ & 15);
    ++sn_;
    // matched filter: each half-bit is a half-sine, so weight the samples by that shape
    const MatchedFilter& mf = matchedFilter();
    double soft = 0;
    for (int j = 0; j < 16; ++j) soft += mf.v[j] * win_[(wn_ + j) & 15];
    metric_[p] = 0.995 * metric_[p] + 0.005 * std::fabs(soft);
    // Timing: follow the peak one sample at a time (smooth, so no skipped or
    // repeated bits), but jump straight to the true peak if it is clearly
    // stronger.  Biphase coding also gives a weaker false peak half a bit
    // away, and one-step climbing alone could stick on it.
    if (p == 15) {
        const double* m = metric_;
        const int b = best_, up = (b + 1) & 15, dn = (b + 15) & 15;
        int gi = 0;
        for (int j = 1; j < 16; ++j)
            if (m[j] > m[gi]) gi = j;
        if (m[gi] > m[b] * 1.3) best_ = gi;
        else if (m[up] > m[b] * 1.05 && m[up] >= m[dn]) best_ = up;
        else if (m[dn] > m[b] * 1.05) best_ = dn;
    }
    if (p == best_ && sn_ - lastEmit_ >= 12) {
        lastEmit_ = sn_;
        const int raw = soft > 0 ? 1 : 0;
        bit(raw ^ prevRaw_);  // 6. differential decoding
        prevRaw_ = raw;
    }
}

void RdsDecoder::bit(int b) {
    // 7. block synchronisation
    reg_ = ((reg_ << 1) | static_cast<std::uint32_t>(b)) & 0x3FFFFFF;
    ++bitN_;
    const int type = offsetType((reg_ & 0x3FF) ^ rdsCrc(reg_ >> 10));
    if (!synced_) {
        if (type < 0) return;
        const long d = bitN_ - candN_;
        if (haveCand_ && d % 26 == 0 && d <= 26 * 6 && (candT_ + d / 26) % 4 == type) {
            synced_ = true;
            left_ = 0;
            expect_ = type;
            qLen_ = qPos_ = qSum_ = qGood_ = 0;
        } else {
            haveCand_ = true;
            candN_ = bitN_;
            candT_ = type;
            return;
        }
    } else if (++left_ < 26) {
        return;
    }
    // One block every 26 bits once synced; repair up to 5-bit error bursts
    // using the check bits.
    left_ = 0;
    std::uint32_t word = reg_;
    bool good = type == expect_, repaired = false;
    if (!good) {
        const std::uint32_t offs[4] = {kRdsOffsetA, kRdsOffsetB, kRdsOffsetC, kRdsOffsetD};
        const std::uint32_t tryOffs[2] = {offs[expect_], kRdsOffsetCp};
        const int tries = expect_ == 2 ? 2 : 1;
        for (int t = 0; t < tries; ++t) {
            const std::uint32_t e = fixTable().e[rdsSyn(word ^ tryOffs[t])];
            if (e) {
                word ^= e;
                good = true;
                ++fixed_;
                repaired = true;
                break;
            }
        }
    }
    blk_[expect_] = word >> 10;
    ok_[expect_] = good;
    rep_[expect_] = repaired;
    // Quality over the last 50 blocks.  Scores: 2 = clean, 1 = repaired,
    // 0 = bad.  Sync is lost when the average falls below 0.7 (of 2).
    // (Change from the JavaScript, which counted repaired blocks as fully
    // good and dropped sync below 25% good: about a third of random words
    // look "repairable" as a burst error, so a decoder that had synced on
    // the wrong bit -- e.g. during the start-up transient -- could stay
    // stuck there for ever.  Clean blocks are almost never random.)
    if (qLen_ == 50) {
        qSum_ -= q_[qPos_];
        qGood_ -= q_[qPos_] ? 1 : 0;
    } else {
        ++qLen_;
    }
    q_[qPos_] = static_cast<unsigned char>(good ? (repaired ? 1 : 2) : 0);
    qSum_ += q_[qPos_];
    qGood_ += good ? 1 : 0;
    qPos_ = (qPos_ + 1) % 50;
    if (qLen_ >= 20 && qSum_ < 0.7 * qLen_) {
        synced_ = false;
        haveCand_ = false;
        qLen_ = qPos_ = qSum_ = qGood_ = 0;
        return;
    }
    if (expect_ == 3) {
        ++changes_;
        group();
        for (int i = 0; i < 4; ++i) ok_[i] = rep_[i] = false;
    }
    expect_ = (expect_ + 1) & 3;
}

// Puts one character: clean blocks go straight in, repaired ones need to be
// seen twice.
bool RdsDecoder::setChar(std::string* arr, int* pend, int pos, int code, bool clean, bool isPs) {
    if (clean || pend[pos] == code) {
        arr[pos] = rdsChar(code);
        pend[pos] = -1;
        if (isPs) psKnown_[pos] = true;
        return true;
    }
    pend[pos] = code;
    return false;
}

bool RdsDecoder::putRt(int pos, int code, bool clean) {
    if (code == 0x0D) {  // carriage return: end of the text
        if (clean || rtPend_[pos] == code) rtLen_ = std::min(rtLen_, pos);
        else rtPend_[pos] = code;
        return true;
    }
    return setChar(rt_, rtPend_, pos, code, clean, false);
}

void RdsDecoder::group() {
    // 8. group decoding
    const std::uint32_t A = blk_[0], Bw = blk_[1], C = blk_[2], D = blk_[3];
    const bool okA = ok_[0], okB = ok_[1], okC = ok_[2], okD = ok_[3];
    const bool rA = rep_[0], rB = rep_[1], rC = rep_[2], rD = rep_[3];
    if (okA && !rA) pi_ = static_cast<int>(A);
    if (!okB) return;
    // A repaired block B could carry the wrong group type.  Trust it only if
    // its type, TP and PTY (bits 15-5) match a combination recently received
    // clean; stations only use a few.
    const int hi = static_cast<int>(Bw >> 5);
    std::vector<int>::iterator it = std::find(cleanB_.begin(), cleanB_.end(), hi);
    if (!rB) {
        if (it != cleanB_.end()) cleanB_.erase(it);
        cleanB_.push_back(hi);
        if (cleanB_.size() > 8) cleanB_.erase(cleanB_.begin());
    } else if (it == cleanB_.end()) {
        return;
    }
    ++groups_;
    const int type = static_cast<int>(Bw >> 12), ver = static_cast<int>((Bw >> 11) & 1);
    tp_ = ((Bw >> 10) & 1) != 0;
    pty_ = static_cast<int>((Bw >> 5) & 31);
    if (type == 0) {
        ta_ = ((Bw >> 4) & 1) != 0;
        ms_ = ((Bw >> 3) & 1) != 0;
        if (okD) {
            const int seg = static_cast<int>(Bw & 3);
            const bool clean = !rD && !rB;
            setChar(ps_, psPend_, seg * 2, static_cast<int>(D >> 8), clean, true);
            setChar(ps_, psPend_, seg * 2 + 1, static_cast<int>(D & 255), clean, true);
        }
    } else if (type == 2) {
        const int ab = static_cast<int>((Bw >> 4) & 1), seg = static_cast<int>(Bw & 15);
        // The A/B flag says "new text": only act on it from a clean block B.
        if (rtAB_ >= 0 && ab != rtAB_) {
            if (rB) return;
            for (int i = 0; i < 64; ++i) {
                rt_[i] = " ";
                rtPend_[i] = -1;
            }
            rtLen_ = 64;
        }
        rtAB_ = ab;
        if (ver == 0) {
            if (okC && okD) {
                const bool cc = !rC && !rB, cd = !rD && !rB;
                putRt(seg * 4, static_cast<int>(C >> 8), cc);
                putRt(seg * 4 + 1, static_cast<int>(C & 255), cc);
                putRt(seg * 4 + 2, static_cast<int>(D >> 8), cd);
                putRt(seg * 4 + 3, static_cast<int>(D & 255), cd);
            }
        } else if (okD) {
            const bool cd = !rD && !rB;
            putRt(seg * 2, static_cast<int>(D >> 8), cd);
            putRt(seg * 2 + 1, static_cast<int>(D & 255), cd);
        }
    } else if (type == 4 && ver == 0 && okC && okD && !rC && !rD) {
        const int mjd = static_cast<int>(((Bw & 3) << 15) | (C >> 1));
        const int hh = static_cast<int>(((C & 1) << 4) | (D >> 12)), mm = static_cast<int>((D >> 6) & 63);
        const int offHalfHours = static_cast<int>(D & 31) * (((D >> 5) & 1) ? -1 : 1);
        if (hh < 24 && mm < 60 && mjd > 15079) {
            hasCt_ = true;
            ctMjd_ = mjd;
            ctMin_ = hh * 60 + mm;
            ctOffsetHalfHours_ = offHalfHours;
        }
    }
}

double RdsDecoder::quality() const { return qLen_ ? static_cast<double>(qGood_) / qLen_ : 0.0; }

void RdsDecoder::fill(RdsInfo& out, bool rbds) const {
    out.pi = pi_ < 0 ? 0 : static_cast<std::uint16_t>(pi_);
    out.ps.clear();
    for (int i = 0; i < 8; ++i) out.ps += psKnown_[i] ? ps_[i] : std::string(" ");
    out.radioText.clear();
    for (int i = 0; i < rtLen_ && i < 64; ++i) out.radioText += rt_[i];
    while (!out.radioText.empty() && out.radioText[out.radioText.size() - 1] == ' ')
        out.radioText.erase(out.radioText.size() - 1);
    out.pty = pty_;
    out.ptyName = rdsPtyName(pty_, rbds);
    out.tp = tp_;
    out.ta = ta_;
    out.music = ms_;
    out.groups = groups_;
    out.clock.clear();
    if (hasCt_) {
        // Local time = UTC + the station's offset (in half hours).
        int mjd = ctMjd_, minutes = ctMin_ + ctOffsetHalfHours_ * 30;
        while (minutes < 0) {
            minutes += 1440;
            --mjd;
        }
        while (minutes >= 1440) {
            minutes -= 1440;
            ++mjd;
        }
        int y, m, d;
        mjdToDate(mjd, y, m, d);
        char buf[32];
        std::snprintf(buf, sizeof buf, "%04d-%02d-%02d %02d:%02d", y, m, d, minutes / 60, minutes % 60);
        out.clock = buf;
    }
}

// ============================================================================
//  Encoder (demo source)
// ============================================================================
RdsEncoder::RdsEncoder()
    : pi_(0), pty_(0), rtSegs_(1), tp_(false), ta_(false), ms_(true), epoch_(0), pos_(0), prev_(0), groupNo_(0),
      psSeg_(0), rtSeg_(0), bitsSent_(0) {}

void RdsEncoder::setup(std::uint16_t pi, const std::string& ps, const std::string& radioText, int pty, bool tp,
                       bool ta, bool music, double epochUnix) {
    pi_ = pi;
    ps_ = ps;
    ps_.resize(8, ' ');
    rt_ = radioText.substr(0, 64);
    if (rt_.size() < 64) rt_ += '\r';  // a carriage return ends a short text
    rtSegs_ = static_cast<int>((rt_.size() + 3) / 4);
    rt_.resize(static_cast<std::size_t>(rtSegs_) * 4, ' ');
    pty_ = pty & 31;
    tp_ = tp;
    ta_ = ta;
    ms_ = music;
    epoch_ = epochUnix;
    bits_.clear();
    pos_ = 0;
    prev_ = 0;
    groupNo_ = psSeg_ = rtSeg_ = 0;
    bitsSent_ = 0;
}

void RdsEncoder::pushBlock(std::uint32_t info, std::uint32_t offset) {
    // 16 information bits then 10 check bits (CRC xor offset word), MSB first.
    const std::uint32_t word = ((info & 0xFFFF) << 10) | (rdsCrc(info) ^ offset);
    for (int b = 25; b >= 0; --b) bits_.push_back(static_cast<unsigned char>((word >> b) & 1));
}

void RdsEncoder::makeGroup() {
    const unsigned g = groupNo_++;
    const std::uint32_t common = (tp_ ? 1u << 10 : 0u) | (static_cast<std::uint32_t>(pty_) << 5);
    std::uint32_t B, C, D;
    if (g % 20 == 19) {
        // 4A clock time: Modified Julian Day, UTC hour/minute, offset 0.
        const double t = epoch_ + bitsSent_ / 1187.5;
        const long days = static_cast<long>(std::floor(t / 86400.0));
        const int secOfDay = static_cast<int>(t - days * 86400.0);
        const std::uint32_t mjd = static_cast<std::uint32_t>(40587 + days);
        const std::uint32_t hh = static_cast<std::uint32_t>(secOfDay / 3600), mm = static_cast<std::uint32_t>((secOfDay / 60) % 60);
        B = (4u << 12) | common | ((mjd >> 15) & 3);
        C = ((mjd & 0x7FFF) << 1) | ((hh >> 4) & 1);
        D = ((hh & 15) << 12) | (mm << 6);
    } else if (g % 3 == 2) {
        // 2A RadioText: four characters per group.
        const unsigned seg = rtSeg_;
        rtSeg_ = (rtSeg_ + 1) % static_cast<unsigned>(rtSegs_);
        B = (2u << 12) | common | seg;  // A/B flag 0
        C = (static_cast<unsigned char>(rt_[seg * 4]) << 8) | static_cast<unsigned char>(rt_[seg * 4 + 1]);
        D = (static_cast<unsigned char>(rt_[seg * 4 + 2]) << 8) | static_cast<unsigned char>(rt_[seg * 4 + 3]);
    } else {
        // 0A basic tuning: two PS characters per group.
        const unsigned seg = psSeg_;
        psSeg_ = (psSeg_ + 1) & 3;
        B = common | (ta_ ? 1u << 4 : 0u) | (ms_ ? 1u << 3 : 0u) | seg;
        C = 0xE0CD;  // AF: "no alternative frequencies" + filler
        D = (static_cast<unsigned char>(ps_[seg * 2]) << 8) | static_cast<unsigned char>(ps_[seg * 2 + 1]);
    }
    pushBlock(pi_, kRdsOffsetA);
    pushBlock(B, kRdsOffsetB);
    pushBlock(C, kRdsOffsetC);
    pushBlock(D, kRdsOffsetD);
}

int RdsEncoder::nextSymbol() {
    if (pos_ >= bits_.size()) {
        bits_.clear();
        pos_ = 0;
        makeGroup();
    }
    prev_ ^= bits_[pos_++];  // differential coding: a 1 flips the symbol
    ++bitsSent_;
    return prev_ ? 1 : -1;
}

}  // namespace detail
}  // namespace dsp
