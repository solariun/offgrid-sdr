// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/Rds.h
//  Layer:  Signal processing (internal header)
//
//  What this file does
//  -------------------
//  RDS (Radio Data System, IEC 62106) on the 57 kHz subcarrier of FM
//  broadcasts:
//    RdsDecoder  FM multiplex -> RdsInfo (PI, PS, RadioText, PTY, TP/TA/MS,
//                clock time).  A port of OFFgrid-SDR's RdsDecoder (by
//                diddy-boy), JavaScript -> C++.
//    RdsEncoder  the other direction, for DemoSource: builds groups 0A, 2A and
//                4A, adds the check words and differential coding, and hands
//                out one +-1 symbol per RDS bit.
//
//  How it fits in
//  --------------
//  Receiver feeds the FM discriminator output (the "multiplex", before stereo
//  decoding and de-emphasis) to RdsDecoder::process().  DemoSource asks
//  RdsEncoder for symbols and shapes them into the biphase 57 kHz signal.
//
//  Contributing
//  ------------
//  The decoder is kept close to the JavaScript so fixes can be carried
//  between the two versions; names in comments match the original.
// ============================================================================
#ifndef OFFGRID_DSP_RDS_H
#define OFFGRID_DSP_RDS_H

#include "Receiver.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace dsp {
namespace detail {

// 10-bit RDS check (CRC) of a 16-bit information word, generator
// x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1 (0x5B9).
std::uint32_t rdsCrc(std::uint32_t info);

// Offset words added to the check of blocks A, B, C, C' and D.
const std::uint32_t kRdsOffsetA = 0x0FC, kRdsOffsetB = 0x198, kRdsOffsetC = 0x168,
                    kRdsOffsetCp = 0x350, kRdsOffsetD = 0x1B4;

// Programme type names: European RDS and North-American RBDS tables.
const char* rdsPtyName(int pty, bool rbds);

class RdsDecoder {
public:
    RdsDecoder();
    void clear();
    // mpx: FM discriminator output at `rate` (any rate above ~150 kHz).
    void process(const float* mpx, std::size_t n, double rate);
    // Copies what is known so far into `out`.
    void fill(RdsInfo& out, bool rbds) const;
    double quality() const;  // fraction of good blocks recently (0..1)
    // Counts decoded groups (good or not): fill() only needs calling when
    // this changed.
    unsigned changes() const { return changes_; }

private:
    void setRate(double rate);
    void symbolSample(double zr, double zi);
    void bit(int b);
    void group();
    bool setChar(std::string* arr, int* pend, int pos, int code, bool clean, bool isPs);
    bool putRt(int pos, int code, bool clean);

    // --- what has been decoded
    int pi_;  // -1 = none
    std::string ps_[8];
    bool psKnown_[8];
    std::string rt_[64];
    int rtLen_, rtAB_;  // rtAB_ -1 = none yet
    int pty_;
    bool tp_, ta_, ms_;
    bool hasCt_;
    int ctMjd_, ctMin_, ctOffsetHalfHours_;
    unsigned groups_, changes_;
    // repaired characters wait here until seen the same way twice
    int psPend_[8], rtPend_[64];
    std::vector<int> cleanB_;  // recent clean block-B tops (type/TP/PTY), oldest first

    // --- block synchroniser
    std::uint32_t reg_;
    long bitN_;
    bool haveCand_;
    long candN_;
    int candT_;
    bool synced_;
    int left_, expect_, prevRaw_;
    std::uint32_t blk_[4];
    bool ok_[4], rep_[4];
    unsigned char q_[50];
    int qLen_, qPos_, qSum_, qGood_;
    unsigned fixed_;

    // --- demodulator (57 kHz -> baseband -> 19 kHz symbol-rate samples)
    double rate_, r2_;
    int M_;
    std::vector<double> h_;           // 63-tap low-pass at rate/16
    std::vector<double> firRe_, firIm_;  // doubled delay line (no wrap in the loop)
    int firP_;
    std::vector<double> box_;         // three moving averages of length M, re/im
    double bs_[6];
    int bk_, dec_;
    double nco_, ncoInc_;
    double rsStep_, rsPos_, prevZr_, prevZi_;
    double th_, fr_, amp_;
    double win_[16], metric_[16];
    int wn_, best_;
    long sn_, lastEmit_;
};

// Builds the RDS bit stream of one station for the demo source.
class RdsEncoder {
public:
    RdsEncoder();
    // epoch: clock time (Unix seconds, UTC) at the first bit; groups 4A count
    // on from there.
    void setup(std::uint16_t pi, const std::string& ps, const std::string& radioText, int pty, bool tp, bool ta,
               bool music, double epochUnix);
    // Next differentially-coded symbol: +1 or -1, one per bit (1187.5/s).
    int nextSymbol();

private:
    void makeGroup();
    void pushBlock(std::uint32_t info, std::uint32_t offset);

    std::uint16_t pi_;
    std::string ps_, rt_;
    int pty_, rtSegs_;
    bool tp_, ta_, ms_;
    double epoch_;
    std::vector<unsigned char> bits_;
    std::size_t pos_;
    int prev_;
    unsigned groupNo_, psSeg_, rtSeg_;
    long bitsSent_;
};

}  // namespace detail
}  // namespace dsp

#endif
