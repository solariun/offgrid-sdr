// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Acars.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  AcarsDecoder (MSK demodulator, framing, parity repair, CRC) and the ACARS
//  block parser acarsParse(): see Acars.h for the method.  Line-by-line port
//  of OFFgrid-SDR's JS AcarsDecoder (diddy-boy,
//  https://github.com/solariun/offgrid-sdr), whose demodulator follows the
//  scheme of the open-source acarsdec; the JS comments are kept (reworded)
//  next to the code they explain.  Differences from the JS, none of which
//  change what is decoded from clean input:
//    - messages are handed out incrementally (takeMessages / takeText);
//      each carries the time in seconds of audio, not a wall-clock Date;
//    - Addition to the JS: non-finite samples (NaN/Inf) are replaced by 0,
//      and a non-finite loop state is reset, so one bad sample cannot stop
//      the decoder (the JS app stops a decoder that throws);
//    - Addition to the JS: process() ignores a rate that is not a positive
//      finite number;
//    - the parity repair works in place on the block (the JS copies it for
//      each CRC trial): same result, no allocation;
//    - Addition to the JS (found by the tests: the JS lost ~3% of messages
//      in light noise, under 1% now): a second framer reads the other axis
//      of the matched filter (the loop can leave the pre-key, a pure tone
//      that hides the bit timing, locked half a bit off); SOH straight
//      after a single SYN starts a message; the loop stays quick until the
//      SOH (the JS: until the first SYN).  A message both framers decode is
//      reported once.
//
//  How it fits in
//  --------------
//  Implements Acars.h.  Standard library only.
//
//  Contributing
//  ------------
//  No allocation per sample (the block buffer is reserved once; strings grow
//  per decoded message only).  Keep in step with the JS; run
//  tests/test_decoders_aviation.cpp.
// ============================================================================
#include "Acars.h"

#include <cmath>
#include <cstdio>

namespace dec {

namespace {

const double kPi = 3.141592653589793;
const double kTwoPi = 6.283185307179586;

bool finite(double v) { return v == v && v - v == 0; }   // not NaN, not +-inf


// CRC table: reflected 0x8408 (the JS ACARS_CRC).
struct CrcTable {
    unsigned short t[256];
    CrcTable() {
        for (unsigned i = 0; i < 256; ++i) {
            unsigned c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? (c >> 1) ^ 0x8408u : c >> 1;
            t[i] = static_cast<unsigned short>(c);
        }
    }
};
const CrcTable& crcTable() {
    static const CrcTable table;
    return table;
}

int popcount8(unsigned r) {
    int n = 0;
    for (unsigned x = r & 0xffu; x; x &= x - 1) ++n;
    return n;
}
bool popOdd(unsigned r) { return (popcount8(r) & 1) != 0; }

// JS str(): printable ASCII kept, CR/LF as "\n", everything else dropped.
std::string str(const unsigned char* b, std::size_t n, std::size_t a, std::size_t z) {
    std::string s;
    for (std::size_t i = a; i < z && i < n; ++i) {
        const unsigned c = b[i] & 0x7fu;
        if (c >= 32 && c < 127) s += static_cast<char>(c);
        else if (c == 10 || c == 13) s += '\n';
    }
    return s;
}

std::string trim(const std::string& s) {
    std::size_t a = 0, z = s.size();
    while (a < z && (s[a] == ' ' || s[a] == '\n' || s[a] == '\t' || s[a] == '\r')) ++a;
    while (z > a && (s[z - 1] == ' ' || s[z - 1] == '\n' || s[z - 1] == '\t' || s[z - 1] == '\r')) --z;
    return s.substr(a, z - a);
}

const char* const kLabels[][2] = {
    {"SQ", "ground station squitter"},
    {"_d", "no information (acknowledgement)"},
    {"H1", "message to or from the crew or the aircraft's systems"},
    {"Q0", "link test"},
    {"5Z", "airline message"},
};

}  // namespace

// ===========================================================================
// Tables and helpers
// ===========================================================================
const std::vector<double>& acarsChannels() {
    static const double f[] = {131.55e6, 131.525e6, 131.725e6, 131.825e6, 130.025e6, 129.125e6,
                               130.45e6, 131.45e6,  136.9e6,   136.75e6,  136.8e6};
    static const std::vector<double> v(f, f + sizeof f / sizeof f[0]);
    return v;
}

unsigned acarsCrc(const unsigned char* bytes, std::size_t n, unsigned crc) {
    const CrcTable& T = crcTable();
    unsigned c = crc & 0xffffu;
    for (std::size_t i = 0; i < n; ++i) c = (c >> 8) ^ T.t[(c ^ bytes[i]) & 0xffu];
    return c;
}

unsigned char acarsOddParity(unsigned char c) {
    c &= 0x7f;
    return static_cast<unsigned char>(popOdd(c) ? c : (c | 0x80));
}

const char* acarsLabelMeaning(const std::string& label) {
    for (std::size_t i = 0; i < sizeof kLabels / sizeof kLabels[0]; ++i)
        if (label == kLabels[i][0]) return kLabels[i][1];
    return "";
}

std::string AcarsMessage::headLine() const {
    std::string s = reg.empty() ? std::string("(no reg)") : reg;
    if (!flight.empty()) s += "  " + flight;
    s += "  " + label + (down ? " from aircraft" : " to aircraft");
    return s;
}

std::string AcarsMessage::body() const {
    if (!text.empty()) return text;
    const char* m = acarsLabelMeaning(label);
    if (*m) return std::string("(") + m + ")";
    return "(no text)";
}

// The fields of a block (the JS finish(), after the CRC check).  `n` is the
// JS b.length - 1 (the ETX excluded).
bool acarsParse(const unsigned char* b, std::size_t n, AcarsMessage& m) {
    if (n < 12) return false;
    const unsigned c10 = b[10] & 0x7fu;
    m.mode = std::string(1, static_cast<char>(b[0] & 0x7f));
    std::string reg = str(b, n, 1, 8), r2;
    for (std::size_t i = 0; i < reg.size(); ++i)
        if (reg[i] != '.') r2 += reg[i];
    m.reg = r2;
    m.ack = (b[8] & 0x7f) == 0x15 ? std::string("NAK") : std::string(1, static_cast<char>(b[8] & 0x7f));
    m.label = std::string(1, static_cast<char>(b[9] & 0x7f)) + (c10 == 0x7f ? 'd' : static_cast<char>(c10));
    m.blockId = std::string(1, static_cast<char>(b[11] & 0x7f));
    m.down = m.blockId[0] >= '0' && m.blockId[0] <= '9';
    const bool hasText = n > 12 && (b[12] & 0x7f) == 0x02;
    std::size_t k = 13;
    m.msgNo.clear();
    m.flight.clear();
    if (hasText && m.down) {
        m.msgNo = str(b, n, 13, 17);
        m.flight = trim(str(b, n, 17, 23));
        k = 23;
    }
    m.text.clear();
    if (hasText) {
        // (CR already gone: str() made it "\n"; runs of line breaks become one)
        const std::string raw = str(b, n, k, n);
        std::string t;
        for (std::size_t i = 0; i < raw.size(); ++i) {
            if (raw[i] == '\n' && !t.empty() && t[t.size() - 1] == '\n') continue;
            t += raw[i];
        }
        m.text = trim(t);
    }
    return true;
}

// ===========================================================================
// AcarsDecoder
// ===========================================================================
AcarsDecoder::AcarsDecoder()
    : rate_(0), fs_(0), D_(1), FL_(0), OV_(12), idx_(0), t_(0), lastMsgT_(-1e9), ok_(0), bad_(0), freq_(0) {
    fr_[0].blk.reserve(256);
    fr_[1].blk.reserve(256);
    last_.reserve(256);
    restartState();
}

void AcarsDecoder::restartState() {
    p_ = 0;
    clk_ = 0;
    S_ = 0;
    df_ = 0;
    for (int i = 0; i < 2; ++i) {
        Framer& f = fr_[i];
        f.reg = 0;
        f.nb = 1;
        f.st = 0;
        f.inv = false;
        f.blk.clear();
        f.perr = 0;
        f.crc0 = 0;
        f.lvlSum = 0;
        f.lvlN = 0;
    }
    acc_ = 0;
    k_ = 0;
    dc_ = 0;
    for (std::size_t i = 0; i < ring_.size(); ++i) ring_[i] = 0;
    idx_ = 0;
}

void AcarsDecoder::reset() {
    restartState();
    t_ = 0;
    lastMsgT_ = -1e9;
}

void AcarsDecoder::clearMessages() {
    msgs_.clear();
    fresh_.clear();
    text_.clear();
    ok_ = bad_ = 0;
}

// Work at about 12 kHz (48 kHz audio / 4).  One 1200 Hz cycle = two bits:
// the matched filter spans it (FL samples), 12x oversampled.
void AcarsDecoder::setRate(double r) {
    rate_ = r;
    D_ = static_cast<int>(std::floor(r / 12000 + 0.5));
    if (D_ < 1) D_ = 1;
    fs_ = r / D_;
    FL_ = static_cast<int>(std::floor(fs_ / 1200 + 0.5)) + 1;
    OV_ = 12;
    h_.assign(static_cast<std::size_t>(FL_ * OV_ + 1), 0.0);
    const double mid = (static_cast<double>(h_.size()) - 1) / 2;
    for (std::size_t i = 0; i < h_.size(); ++i) {
        const double v = std::cos(kTwoPi * 600 / fs_ / OV_ * (static_cast<double>(i) - mid));
        h_[i] = v > 0 ? v : 0;
    }
    ring_.assign(static_cast<std::size_t>(FL_ * 2), 0.0);
    idx_ = 0;
}

void AcarsDecoder::process(const float* x, std::size_t n, double rate) {
    if (!(rate > 0) || !finite(rate) || !x) return;   // Addition to the JS
    if (rate != rate_) setRate(rate);
    const int D = D_;
    for (std::size_t i = 0; i < n; ++i) {
        double v = x[i];
        if (!finite(v)) v = 0;                       // Addition to the JS: NaN/Inf -> 0
        acc_ += v;
        if (++k_ < D) continue;
        const double v0 = acc_ / D;
        acc_ = 0;
        k_ = 0;
        dc_ += 0.002 * (v0 - dc_);
        sample(v0 - dc_);
    }
    t_ += n / rate;
    // Addition to the JS: a non-finite loop state (it cannot arise from
    // finite input, but be sure) starts the receiver over.
    if (!finite(df_) || !finite(p_) || !finite(dc_) || !finite(clk_)) {
        const double t = t_;
        restartState();
        t_ = t;
    }
}

void AcarsDecoder::sample(double v) {
    const double s = kTwoPi * 1800 / fs_ + df_;
    const int FL = FL_;
    double* ring = &ring_[0];
    p_ += s;
    if (p_ >= kTwoPi) p_ -= kTwoPi;
    ring[2 * idx_] = v * std::cos(p_);
    ring[2 * idx_ + 1] = -v * std::sin(p_);
    idx_ = (idx_ + 1) % FL;
    clk_ += s;
    if (clk_ < 1.5 * kPi - s / 2) return;
    clk_ -= 1.5 * kPi;
    // the matched filter, lined up to the exact bit instant (the oversampled
    // filter takes up the fraction of a sample)
    int o = static_cast<int>(std::floor(OV_ * (clk_ / s + 0.5) + 0.5));
    if (o > OV_) o = OV_;
    if (o < 0) o = 0;
    double vr = 0, vi = 0;
    for (int j = 0; j < FL; ++j, o += OV_) {
        int q = j + idx_;
        if (q >= FL) q -= FL;
        const double w = h_[static_cast<std::size_t>(o)];
        vr += w * ring[2 * q];
        vi += w * ring[2 * q + 1];
    }
    const double lvl = std::sqrt(vr * vr + vi * vi) + 1e-12;
    vr /= lvl;
    vi /= lvl;
    for (int i = 0; i < 2; ++i) {
        fr_[i].lvlSum += lvl * lvl;
        ++fr_[i].lvlN;
    }
    double vo, vb, dphi;
    if (S_ & 1) {
        vo = vi;
        vb = vr;
        dphi = vo >= 0 ? -vr : vr;
    } else {
        vo = vr;
        vb = vi;
        dphi = vo >= 0 ? vi : -vi;
    }
    bit(fr_[0], 0, (S_ & 2) ? -vo : vo);
    // Addition to the JS: a second framer reads the other axis.  A pure tone
    // (the pre-key) does not show the bit timing, so the loop can come out of
    // it locked up to half a bit off: the data then sits on the axis not
    // read (the JS lost such a message, ~2% of them in light noise; the loop
    // pulls in too slowly to save it).  Both framers' messages must pass the
    // CRC; a message both decode is reported once.
    bit(fr_[1], 1, ((S_ + 1) & 2) ? -vb : vb);   // (half a bit later: the sign pattern one bit on)
    ++S_;
    // the loop filter: quicker while looking for a message, so it locks
    // within the pre-key, then steadier.  Addition to the JS: quick until
    // the SOH (the JS: until the first SYN), since the bit timing can only
    // be pulled in once the data after the pre-key starts; steady while
    // either framer is inside a message.  (Messages lost in noise, at the
    // test's levels, went from ~3% to under 1%.)
    const bool search = fr_[0].st < 3 && fr_[1].st < 3;
    df_ = 0.52 * df_ + 0.48 * (search ? 4 * 38e-4 : 38e-4) * dphi;
}

void AcarsDecoder::bit(Framer& f, int which, double v) {
    if (f.inv) v = -v;
    f.reg = (f.reg >> 1) | (v > 0 ? 0x80u : 0u);
    if (--f.nb > 0) return;
    byte(f, which, f.reg);
}

void AcarsDecoder::restart(Framer& f, int which) {
    f.st = 0;
    f.nb = 1;
    if (which == 0) df_ = 0;   // (the JS's framer restarts the loop too)
}

void AcarsDecoder::begin(Framer& f) {
    f.st = 3;
    f.blk.clear();
    f.perr = 0;
    f.nb = 8;
    f.lvlSum = 0;
    f.lvlN = 0;
}

// (The JS flips the bit sign of everything after an inverted SYN by
//  toggling S's second bit; here each framer has its own `inv` flag, which
//  is the same thing.)
void AcarsDecoder::byte(Framer& f, int which, unsigned r) {
    const unsigned SYN = 0x16, NSYN = ~SYN & 0xffu;
    switch (f.st) {
    case 0:   // looking for SYN, bit by bit
        if (r == SYN) { f.st = 1; f.nb = 8; return; }
        if (r == NSYN) { f.inv = !f.inv; f.st = 1; f.nb = 8; return; }
        f.nb = 1;
        return;
    case 1:   // the second SYN
        if (r == SYN) { f.st = 2; f.nb = 8; return; }
        if (r == NSYN) { f.inv = !f.inv; f.nb = 8; return; }
        // Addition to the JS: SOH straight after one SYN starts the message
        // too (when the loop is still settling at the end of the pre-key the
        // first SYN can be lost; the second then reads as the first).
        if (r == 0x01) { begin(f); return; }
        restart(f, which);
        return;
    case 2:   // SOH: the message starts
        if (r == 0x01) { begin(f); return; }
        restart(f, which);
        return;
    case 3:   // the message, up to ETX / ETB
        f.blk.push_back(static_cast<unsigned char>(r));
        if (!popOdd(r)) {
            if (++f.perr > 6) { ++bad_; restart(f, which); return; }
        }
        if (r == 0x83 || r == 0x97) { f.st = 4; f.nb = 8; return; }
        if (f.blk.size() > 240) { ++bad_; restart(f, which); return; }
        f.nb = 8;
        return;
    case 4:
        f.crc0 = static_cast<unsigned char>(r);
        f.st = 5;
        f.nb = 8;
        return;
    case 5:
        finish(f, f.crc0, static_cast<unsigned char>(r));
        f.st = 6;
        f.nb = 8;
        return;
    default:
        restart(f, which);
        f.nb = 8;
    }
}

bool AcarsDecoder::crcOk(const std::vector<unsigned char>& blk, unsigned char c0, unsigned char c1) {
    unsigned c = acarsCrc(blk.empty() ? 0 : &blk[0], blk.size(), 0);
    const unsigned char tail[2] = {c0, c1};
    return acarsCrc(tail, 2, c) == 0;
}

// A bit error shows up as a character with the wrong parity: try flipping
// each bit of up to two such characters until the CRC checks out (single
// errors are the common case on a weak signal).
bool AcarsDecoder::repair(std::vector<unsigned char>& blk, unsigned char c0, unsigned char c1) {
    int badAt[3], nBad = 0;
    for (std::size_t i = 0; i < blk.size(); ++i)
        if (!popOdd(blk[i])) {
            if (nBad < 3) badAt[nBad] = static_cast<int>(i);
            ++nBad;
        }
    if (!nBad || nBad > 2) return false;
    const int i = badAt[0];
    for (int x = 0; x < 8; ++x) {
        blk[i] ^= static_cast<unsigned char>(1u << x);
        if (nBad == 1) {
            if (crcOk(blk, c0, c1)) return true;
        } else {
            const int j = badAt[1];
            for (int y = 0; y < 8; ++y) {
                blk[j] ^= static_cast<unsigned char>(1u << y);
                if (crcOk(blk, c0, c1)) return true;
                blk[j] ^= static_cast<unsigned char>(1u << y);
            }
        }
        blk[i] ^= static_cast<unsigned char>(1u << x);
    }
    return false;
}

void AcarsDecoder::finish(Framer& f, unsigned char c0, unsigned char c1) {
    std::vector<unsigned char>& b = f.blk;
    if (b.size() < 13) { ++bad_; return; }
    if (!crcOk(b, c0, c1) && !repair(b, c0, c1)) { ++bad_; return; }
    // (the same message from the other framer: once is enough)
    if (t_ - lastMsgT_ < 0.5 && b == last_) return;
    last_ = b;
    lastMsgT_ = t_;
    ++ok_;
    AcarsMessage m;
    acarsParse(&b[0], b.size() - 1, m);
    m.levelDb = f.lvlN ? 10 * std::log10(f.lvlSum / f.lvlN) : 0;
    m.time = t_;
    m.freqHz = freq_;
    msgs_.push_back(m);
    if (msgs_.size() > 200) msgs_.pop_front();
    fresh_.push_back(m);
    if (fresh_.size() > 200) fresh_.erase(fresh_.begin());

    // the readable lines (the JS ACARS box: acarsLine, then acarsBody)
    text_ += "ACARS  " + m.headLine();
    if (!m.msgNo.empty()) text_ += "  [" + m.msgNo + "]";
    const char* meaning = acarsLabelMeaning(m.label);
    if (*meaning && !m.text.empty()) text_ += std::string("  (") + meaning + ")";
    text_ += '\n';
    const std::string body = m.body();
    text_ += "  ";
    for (std::size_t i = 0; i < body.size(); ++i) {
        text_ += body[i];
        if (body[i] == '\n') text_ += "  ";
    }
    text_ += '\n';
}

void AcarsDecoder::takeMessages(std::vector<AcarsMessage>& out) {
    out.insert(out.end(), fresh_.begin(), fresh_.end());
    fresh_.clear();
}

void AcarsDecoder::takeText(std::string& out) {
    out += text_;
    text_.clear();
}

Status AcarsDecoder::status() const {
    Status s;
    s.locked = (fr_[0].st >= 3 && fr_[0].st <= 5) || (fr_[1].st >= 3 && fr_[1].st <= 5);
    s.signal = s.locked || (fr_[0].st >= 1 && fr_[0].st <= 5);
    s.toneHz = acarsAudioHz;
    s.shiftHz = 1200;
    s.baud = 2400;
    s.characters = ok_;
    s.mode = "ACARS";
    s.quality = ok_ + bad_ ? static_cast<double>(ok_) / (ok_ + bad_) : 0;
    char buf[96];
    if (ok_)
        std::snprintf(buf, sizeof buf, "%u message%s (%u rejected)%s", ok_, ok_ == 1 ? "" : "s", bad_,
                      s.locked ? "  receiving" : "");
    else
        std::snprintf(buf, sizeof buf, "listening%s", s.locked ? ": receiving" : "");
    s.info = buf;
    return s;
}

}  // namespace dec
