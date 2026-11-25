// ============================================================================
//  offgrid-codecs - MPEG Audio Layer II decoder for DAB (part of OFFgrid-SDR
//  TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from pl_mpeg's MP2 decoder (MIT, notice below), with
//  OFFgrid-SDR's DAB changes (licenses/pi_mpeg/pl_mpeg-dab-changes.diff).
// ----------------------------------------------------------------------------
//  File:   src/codecs/Mp2.cpp
//  Layer:  Library liboffgrid-codecs
//
//  What this file does
//  -------------------
//  codec::Mp2Decoder (see Mp2.h): reads a Layer II frame - header, bit
//  allocation, scale factor selection and scale factors, then 12 granules of
//  3 samples per subband - dequantises with integer arithmetic, and runs the
//  32-band synthesis filter bank (a 32-point DCT, "idct36", then the 512-tap
//  window) into 1152 stereo samples.  Then it collects the ancillary bytes
//  after the audio, where DAB carries its PAD.
//
//  What changed from pl_mpeg
//  -------------------------
//  pl_mpeg decodes a byte stream (a ring buffer, searching for the next sync
//  word); DAB hands over whole frames, so this decodes one frame per call
//  from a bounded bit reader, with the header at the first byte.  Kept from
//  OFFgrid-SDR's patched pl_mpeg: MPEG-2 half sample rates with their bit-rate
//  table and allocation table (ISO 13818-3 B.1).  Not kept: the check that
//  a header matches the previous one (a resync guard for byte streams; a
//  frame-by-frame caller knows its frames).  Fixed: a free-format bit-rate
//  index (0) read pl_mpeg's table at -1; it is refused here.  New: a frame
//  whose audio runs past its own end is refused (it is damaged), and the
//  ancillary data and DAB's PAD split are exposed.  The dequantisation, DCT
//  and window arithmetic are pl_mpeg's, operation for operation.
//
//  How it fits in
//  --------------
//  Used by the DAB receiver for classic (MP2) stations; independent of the
//  AAC decoder.
//
//  Contributing
//  ------------
//  Keep the arithmetic as it is (the tests compare with pl_mpeg sample for
//  sample).  Strict C++11, no warnings with -Wall -Wextra -pedantic-errors.
// ============================================================================
/*
   pl_mpeg's MP2 decoder: PL_MPEG - MPEG1 Video decoder, MP2 Audio decoder, MPEG-PS demuxer
   Dominic Szablewski - https://phoboslab.org (commit c871f2be022ece7ef4f64230b4fb8e1fb9eb6023)
   Based on kjmp2 by Martin J. Fiedler, http://keyj.emphy.de/kjmp2/

   The MIT License (MIT)

   Copyright (c) 2019 Dominic Szablewski

   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
   documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
   permit persons to whom the Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all copies or substantial portions of
   the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
   WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
   COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include "Mp2.h"

#include <cstring>

namespace codec {

namespace {

const int kMpeg2 = 0x2;
const int kMpeg1 = 0x3;
const int kLayerII = 0x2;

// Sample rates, MPEG-1 then MPEG-2's halves (index + 4).
const unsigned short kSampleRate[] = {
    44100, 48000, 32000, 0, // MPEG-1
    22050, 24000, 16000, 0  // MPEG-2
};

// Bit rates in kbit/s, MPEG-1 then MPEG-2 Layer II (index + 14).
const short kBitRate[] = {
    32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384, // MPEG-1
     8, 16, 24, 32, 40, 48,  56,  64,  80,  96, 112, 128, 144, 160  // MPEG-2
};

const int kScalefactorBase[] = {
    0x02000000, 0x01965FEA, 0x01428A30
};

// The synthesis window (ISO 11172-3 table 3-B.3), scaled and folded as kjmp2 does.
const float kSynthesisWindow[] = {
         0.0,     -0.5,     -0.5,     -0.5,     -0.5,     -0.5,
        -0.5,     -1.0,     -1.0,     -1.0,     -1.0,     -1.5,
        -1.5,     -2.0,     -2.0,     -2.5,     -2.5,     -3.0,
        -3.5,     -3.5,     -4.0,     -4.5,     -5.0,     -5.5,
        -6.5,     -7.0,     -8.0,     -8.5,     -9.5,    -10.5,
       -12.0,    -13.0,    -14.5,    -15.5,    -17.5,    -19.0,
       -20.5,    -22.5,    -24.5,    -26.5,    -29.0,    -31.5,
       -34.0,    -36.5,    -39.5,    -42.5,    -45.5,    -48.5,
       -52.0,    -55.5,    -58.5,    -62.5,    -66.0,    -69.5,
       -73.5,    -77.0,    -80.5,    -84.5,    -88.0,    -91.5,
       -95.0,    -98.0,   -101.0,   -104.0,    106.5,    109.0,
       111.0,    112.5,    113.5,    114.0,    114.0,    113.5,
       112.0,    110.5,    107.5,    104.0,    100.0,     94.5,
        88.5,     81.5,     73.0,     63.5,     53.0,     41.5,
        28.5,     14.5,     -1.0,    -18.0,    -36.0,    -55.5,
       -76.5,    -98.5,   -122.0,   -147.0,   -173.5,   -200.5,
      -229.5,   -259.5,   -290.5,   -322.5,   -355.5,   -389.5,
      -424.0,   -459.5,   -495.5,   -532.0,   -568.5,   -605.0,
      -641.5,   -678.0,   -714.0,   -749.0,   -783.5,   -817.0,
      -849.0,   -879.5,   -908.5,   -935.0,   -959.5,   -981.0,
     -1000.5,  -1016.0,  -1028.5,  -1037.5,  -1042.5,  -1043.5,
     -1040.0,  -1031.5,   1018.5,   1000.0,    976.0,    946.5,
       911.0,    869.5,    822.0,    767.5,    707.0,    640.0,
       565.5,    485.0,    397.0,    302.5,    201.0,     92.5,
       -22.5,   -144.0,   -272.5,   -407.0,   -547.5,   -694.0,
      -846.0,  -1003.0,  -1165.0,  -1331.5,  -1502.0,  -1675.5,
     -1852.5,  -2031.5,  -2212.5,  -2394.0,  -2576.5,  -2758.5,
     -2939.5,  -3118.5,  -3294.5,  -3467.5,  -3635.5,  -3798.5,
     -3955.0,  -4104.5,  -4245.5,  -4377.5,  -4499.0,  -4609.5,
     -4708.0,  -4792.5,  -4863.5,  -4919.0,  -4958.0,  -4979.5,
     -4983.0,  -4967.5,  -4931.5,  -4875.0,  -4796.0,  -4694.5,
     -4569.5,  -4420.0,  -4246.0,  -4046.0,  -3820.0,  -3567.0,
      3287.0,   2979.5,   2644.0,   2280.5,   1888.0,   1467.5,
      1018.5,    541.0,     35.0,   -499.0,  -1061.0,  -1650.0,
     -2266.5,  -2909.0,  -3577.0,  -4270.0,  -4987.5,  -5727.5,
     -6490.0,  -7274.0,  -8077.5,  -8899.5,  -9739.0, -10594.5,
    -11464.5, -12347.0, -13241.0, -14144.5, -15056.0, -15973.5,
    -16895.5, -17820.0, -18744.5, -19668.0, -20588.0, -21503.0,
    -22410.5, -23308.5, -24195.0, -25068.5, -25926.5, -26767.0,
    -27589.0, -28389.0, -29166.5, -29919.0, -30644.5, -31342.0,
    -32009.5, -32645.0, -33247.0, -33814.5, -34346.0, -34839.5,
    -35295.0, -35710.0, -36084.5, -36417.5, -36707.5, -36954.0,
    -37156.5, -37315.0, -37428.0, -37496.0,  37519.0,  37496.0,
     37428.0,  37315.0,  37156.5,  36954.0,  36707.5,  36417.5,
     36084.5,  35710.0,  35295.0,  34839.5,  34346.0,  33814.5,
     33247.0,  32645.0,  32009.5,  31342.0,  30644.5,  29919.0,
     29166.5,  28389.0,  27589.0,  26767.0,  25926.5,  25068.5,
     24195.0,  23308.5,  22410.5,  21503.0,  20588.0,  19668.0,
     18744.5,  17820.0,  16895.5,  15973.5,  15056.0,  14144.5,
     13241.0,  12347.0,  11464.5,  10594.5,   9739.0,   8899.5,
      8077.5,   7274.0,   6490.0,   5727.5,   4987.5,   4270.0,
      3577.0,   2909.0,   2266.5,   1650.0,   1061.0,    499.0,
       -35.0,   -541.0,  -1018.5,  -1467.5,  -1888.0,  -2280.5,
     -2644.0,  -2979.5,   3287.0,   3567.0,   3820.0,   4046.0,
      4246.0,   4420.0,   4569.5,   4694.5,   4796.0,   4875.0,
      4931.5,   4967.5,   4983.0,   4979.5,   4958.0,   4919.0,
      4863.5,   4792.5,   4708.0,   4609.5,   4499.0,   4377.5,
      4245.5,   4104.5,   3955.0,   3798.5,   3635.5,   3467.5,
      3294.5,   3118.5,   2939.5,   2758.5,   2576.5,   2394.0,
      2212.5,   2031.5,   1852.5,   1675.5,   1502.0,   1331.5,
      1165.0,   1003.0,    846.0,    694.0,    547.5,    407.0,
       272.5,    144.0,     22.5,    -92.5,   -201.0,   -302.5,
      -397.0,   -485.0,   -565.5,   -640.0,   -707.0,   -767.5,
      -822.0,   -869.5,   -911.0,   -946.5,   -976.0,  -1000.0,
      1018.5,   1031.5,   1040.0,   1043.5,   1042.5,   1037.5,
      1028.5,   1016.0,   1000.5,    981.0,    959.5,    935.0,
       908.5,    879.5,    849.0,    817.0,    783.5,    749.0,
       714.0,    678.0,    641.5,    605.0,    568.5,    532.0,
       495.5,    459.5,    424.0,    389.5,    355.5,    322.5,
       290.5,    259.5,    229.5,    200.5,    173.5,    147.0,
       122.0,     98.5,     76.5,     55.5,     36.0,     18.0,
         1.0,    -14.5,    -28.5,    -41.5,    -53.0,    -63.5,
       -73.0,    -81.5,    -88.5,    -94.5,   -100.0,   -104.0,
      -107.5,   -110.5,   -112.0,   -113.5,   -114.0,   -114.0,
      -113.5,   -112.5,   -111.0,   -109.0,    106.5,    104.0,
       101.0,     98.0,     95.0,     91.5,     88.0,     84.5,
        80.5,     77.0,     73.5,     69.5,     66.0,     62.5,
        58.5,     55.5,     52.0,     48.5,     45.5,     42.5,
        39.5,     36.5,     34.0,     31.5,     29.0,     26.5,
        24.5,     22.5,     20.5,     19.0,     17.5,     15.5,
        14.5,     13.0,     12.0,     10.5,      9.5,      8.5,
         8.0,      7.0,      6.5,      5.5,      5.0,      4.5,
         4.0,      3.5,      3.5,      3.0,      2.5,      2.5,
         2.0,      2.0,      1.5,      1.5,      1.0,      1.0,
         1.0,      1.0,      0.5,      0.5,      0.5,      0.5,
         0.5,      0.5
};

// Quantizer lookup, step 1: bitrate classes
const std::uint8_t kQuantLutStep1[2][16] = {
    // 32, 48, 56, 64, 80, 96,112,128,160,192,224,256,320,384 <- bitrate
    { 0,  0,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  2 }, // mono
    // 16, 24, 28, 32, 40, 48, 56, 64, 80, 96,112,128,160,192 <- bitrate / chan
    { 0,  0,  0,  0,  0,  0,  1,  1,  1,  2,  2,  2,  2,  2 }  // stereo
};

// Quantizer lookup, step 2: bitrate class, sample rate -> B2 table idx, sblimit
const int kQuantTabA = 27 | 64;   // Table 3-B.2a: high-rate, sblimit = 27
const int kQuantTabB = 30 | 64;   // Table 3-B.2b: high-rate, sblimit = 30
const int kQuantTabC = 8;         // Table 3-B.2c:  low-rate, sblimit =  8
const int kQuantTabD = 12;        // Table 3-B.2d:  low-rate, sblimit = 12

const std::uint8_t kQuantLutStep2[3][3] = {
    //44.1 kHz,  48 kHz,     32 kHz
    { kQuantTabC, kQuantTabC, kQuantTabD }, // 32 - 48 kbit/sec/ch
    { kQuantTabA, kQuantTabA, kQuantTabA }, // 56 - 80 kbit/sec/ch
    { kQuantTabB, kQuantTabA, kQuantTabB }  // 96+    kbit/sec/ch
};

// Quantizer lookup, step 3: B2 table, subband -> nbal, row index
// (upper 4 bits: nbal, lower 4 bits: row index)
const std::uint8_t kQuantLutStep3[3][32] = {
    // Low-rate table (3-B.2c and 3-B.2d)
    {
        0x44,0x44,
        0x34,0x34,0x34,0x34,0x34,0x34,0x34,0x34,0x34,0x34
    },
    // High-rate table (3-B.2a and 3-B.2b)
    {
        0x43,0x43,0x43,
        0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,
        0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,
        0x20,0x20,0x20,0x20,0x20,0x20,0x20
    },
    // MPEG-2 LSR table (B.2 in ISO 13818-3)
    {
        0x45,0x45,0x45,0x45,
        0x34,0x34,0x34,0x34,0x34,0x34,0x34,
        0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,
        0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24
    }
};

// Quantizer lookup, step 4: table row, allocation[] value -> quant table index
const std::uint8_t kQuantLutStep4[6][16] = {
    { 0, 1, 2, 17 },
    { 0, 1, 2,  3, 4, 5, 6, 17 },
    { 0, 1, 2,  3, 4, 5, 6,  7,  8,  9, 10, 11, 12, 13, 14, 17 },
    { 0, 1, 3,  5, 6, 7, 8,  9, 10, 11, 12, 13, 14, 15, 16, 17 },
    { 0, 1, 2,  4, 5, 6, 7,  8,  9, 10, 11, 12, 13, 14, 15, 17 },
    { 0, 1, 2,  3, 4, 5, 6,  7,  8,  9, 10, 11, 12, 13, 14, 15 }
};

} // namespace

// The quantizer classes (levels, grouped?, bits per sample or group).
const Mp2Decoder::Quant Mp2Decoder::kQuantTab[17] = {
    {     3, 1,  5 },  //  1
    {     5, 1,  7 },  //  2
    {     7, 0,  3 },  //  3
    {     9, 1, 10 },  //  4
    {    15, 0,  4 },  //  5
    {    31, 0,  5 },  //  6
    {    63, 0,  6 },  //  7
    {   127, 0,  7 },  //  8
    {   255, 0,  8 },  //  9
    {   511, 0,  9 },  // 10
    {  1023, 0, 10 },  // 11
    {  2047, 0, 11 },  // 12
    {  4095, 0, 12 },  // 13
    {  8191, 0, 13 },  // 14
    { 16383, 0, 14 },  // 15
    { 32767, 0, 15 },  // 16
    { 65535, 0, 16 }   // 17
};

// pl_mpeg's plm_buffer_read(), bounded by the frame: a read that would run
// past the end gives 0 and does not move (as pl_mpeg's does when its buffer
// lacks the bits); decode() then refuses the frame.
int Mp2Decoder::Bits::read(int count) {
    if (pos + (std::size_t)count > nbits) {
        pos = nbits + 1;   // (marks the overrun)
        return 0;
    }
    int value = 0;
    while (count) {
        int current_byte = p[pos >> 3];
        int remaining = 8 - (int)(pos & 7);              // Remaining bits in byte
        int rd = remaining < count ? remaining : count;  // Bits in this run
        int shift = remaining - rd;
        int mask = (0xff >> (8 - rd));
        value = (value << rd) | ((current_byte & (mask << shift)) >> shift);
        pos += (std::size_t)rd;
        count -= rd;
    }
    return value;
}

Mp2Decoder::Mp2Decoder() {
    reset();
}

void Mp2Decoder::reset() {
    rate_ = kbps_ = 0;
    mode_ = Stereo;
    mpeg2_ = crc_ = false;
    frameBytes_ = audioBytes_ = 0;
    anc_.clear();
    pad_ = DabPad();
    bound_ = 0;
    vPos_ = 0;
    std::memset(allocation_, 0, sizeof allocation_);
    std::memset(scaleFactorInfo_, 0, sizeof scaleFactorInfo_);
    std::memset(scaleFactor_, 0, sizeof scaleFactor_);
    std::memset(sample_, 0, sizeof sample_);
    std::memcpy(D_, kSynthesisWindow, 512 * sizeof(float));
    std::memcpy(D_ + 512, kSynthesisWindow, 512 * sizeof(float));
    std::memset(V_, 0, sizeof V_);
    std::memset(U_, 0, sizeof U_);
}

std::size_t Mp2Decoder::frameSize(const std::uint8_t *h, std::size_t len) {
    if (!h || len < 4) return 0;
    if (h[0] != 0xff || (h[1] & 0xf0) != 0xf0) return 0;
    int version = (h[1] >> 3) & 1 ? kMpeg1 : kMpeg2;
    if (((h[1] >> 1) & 3) != kLayerII) return 0;
    int bi = (h[2] >> 4) - 1, si = (h[2] >> 2) & 3, padding = (h[2] >> 1) & 1;
    if (bi < 0 || bi > 13 || si == 3) return 0;
    if (version == kMpeg2) { bi += 14; si += 4; }
    return (std::size_t)(144000 * kBitRate[bi] / kSampleRate[si] + padding);
}

bool Mp2Decoder::decode(const std::vector<std::uint8_t> &frame, std::vector<float> &pcm) {
    return decode(frame.empty() ? 0 : &frame[0], frame.size(), pcm);
}

bool Mp2Decoder::decode(const std::uint8_t *frame, std::size_t len, std::vector<float> &pcm) {
    pcm.clear();
    anc_.clear();
    pad_ = DabPad();
    if (!frame || len < 4) return false;

    // ---- the header (pl_mpeg's plm_audio_decode_header, at the frame's start)
    Bits b;
    b.p = frame;
    b.nbits = len * 8;
    b.pos = 0;
    if (b.read(11) != 0x7ff) return false;
    int version = b.read(2);
    int layer = b.read(2);
    int hasCRC = !b.read(1);
    // (OFFgrid-SDR: MPEG-2's half sample rates accepted too, as kjmp2 1.1 does: DAB uses 24 kHz for many speech stations)
    if ((version != kMpeg1 && version != kMpeg2) || layer != kLayerII) return false;
    int bitrate_index = b.read(4) - 1;
    if (bitrate_index < 0 || bitrate_index > 13) return false;   // (0: free format, 15: reserved)
    int samplerate_index = b.read(2);
    if (samplerate_index == 3) return false;
    if (version == kMpeg2) {   // (OFFgrid-SDR: the MPEG-2 halves of the rate tables)
        bitrate_index += 14;
        samplerate_index += 4;
    }
    int padding = b.read(1);
    b.read(1);   // private bit
    int mode = b.read(2);
    // the mode extension: the stereo bound
    int bound;
    if (mode == JointStereo) {
        bound = (b.read(2) + 1) << 2;
    } else {
        b.read(2);
        bound = (mode == Mono) ? 0 : 32;
    }
    b.read(4);   // copyright(1), original(1), emphasis(2)
    if (hasCRC) b.read(16);
    int bitrate = kBitRate[bitrate_index];
    int samplerate = kSampleRate[samplerate_index];
    std::size_t frame_size = (std::size_t)(144000 * bitrate / samplerate + padding);
    if (len < frame_size) return false;
    b.nbits = frame_size * 8;   // read no further than this frame

    // ---- the frame (pl_mpeg's plm_audio_decode_frame)
    int tab3 = 0;
    int sblimit = 0;
    if (samplerate_index >= 4) {
        // (OFFgrid-SDR: MPEG-2 half sample rate: one allocation table for every bit rate (ISO 13818-3 B.1), 30 subbands)
        tab3 = 2;
        sblimit = 30;
    } else {
        int tab1 = (mode == Mono) ? 0 : 1;
        int tab2 = kQuantLutStep1[tab1][bitrate_index];
        tab3 = kQuantLutStep2[tab2][samplerate_index];
        sblimit = tab3 & 63;
        tab3 >>= 6;
    }
    if (bound > sblimit) bound = sblimit;
    bound_ = bound;

    // the allocation information
    for (int sb = 0; sb < bound; sb++) {
        allocation_[0][sb] = readAllocation(b, sb, tab3);
        allocation_[1][sb] = readAllocation(b, sb, tab3);
    }
    for (int sb = bound; sb < sblimit; sb++) {
        allocation_[0][sb] = allocation_[1][sb] = readAllocation(b, sb, tab3);
    }

    // the scale factor selector information
    int channels = (mode == Mono) ? 1 : 2;
    for (int sb = 0; sb < sblimit; sb++) {
        for (int ch = 0; ch < channels; ch++) {
            if (allocation_[ch][sb]) scaleFactorInfo_[ch][sb] = (std::uint8_t)b.read(2);
        }
        if (mode == Mono) scaleFactorInfo_[1][sb] = scaleFactorInfo_[0][sb];
    }

    // the scale factors
    for (int sb = 0; sb < sblimit; sb++) {
        for (int ch = 0; ch < channels; ch++) {
            if (allocation_[ch][sb]) {
                int *sf = scaleFactor_[ch][sb];
                switch (scaleFactorInfo_[ch][sb]) {
                case 0:
                    sf[0] = b.read(6);
                    sf[1] = b.read(6);
                    sf[2] = b.read(6);
                    break;
                case 1:
                    sf[0] = sf[1] = b.read(6);
                    sf[2] = b.read(6);
                    break;
                case 2:
                    sf[0] = sf[1] = sf[2] = b.read(6);
                    break;
                case 3:
                    sf[0] = b.read(6);
                    sf[1] = sf[2] = b.read(6);
                    break;
                }
            }
        }
        if (mode == Mono) {
            scaleFactor_[1][sb][0] = scaleFactor_[0][sb][0];
            scaleFactor_[1][sb][1] = scaleFactor_[0][sb][1];
            scaleFactor_[1][sb][2] = scaleFactor_[0][sb][2];
        }
    }

    // coefficient input and reconstruction
    pcm.resize(1152 * 2);
    float *out = &pcm[0];
    int out_pos = 0;
    for (int part = 0; part < 3; part++) {
        for (int granule = 0; granule < 4; granule++) {
            // the samples
            for (int sb = 0; sb < bound; sb++) {
                readSamples(b, 0, sb, part);
                readSamples(b, 1, sb, part);
            }
            for (int sb = bound; sb < sblimit; sb++) {
                readSamples(b, 0, sb, part);
                sample_[1][sb][0] = sample_[0][sb][0];
                sample_[1][sb][1] = sample_[0][sb][1];
                sample_[1][sb][2] = sample_[0][sb][2];
            }
            for (int sb = sblimit; sb < 32; sb++) {
                sample_[0][sb][0] = 0;
                sample_[0][sb][1] = 0;
                sample_[0][sb][2] = 0;
                sample_[1][sb][0] = 0;
                sample_[1][sb][1] = 0;
                sample_[1][sb][2] = 0;
            }

            // the synthesis
            for (int p = 0; p < 3; p++) {
                vPos_ = (vPos_ - 64) & 1023;   // shifting step
                for (int ch = 0; ch < 2; ch++) {
                    idct36(sample_[ch], p, V_[ch], vPos_);

                    // build U, windowing, the output
                    std::memset(U_, 0, sizeof(U_));
                    int d_index = 512 - (vPos_ >> 1);
                    int v_index = (vPos_ % 128) >> 1;
                    while (v_index < 1024) {
                        for (int i = 0; i < 32; ++i) U_[i] += D_[d_index++] * V_[ch][v_index++];
                        v_index += 128 - 32;
                        d_index += 64 - 32;
                    }
                    d_index -= (512 - 32);
                    v_index = (128 - 32 + 1024) - v_index;
                    while (v_index < 1024) {
                        for (int i = 0; i < 32; ++i) U_[i] += D_[d_index++] * V_[ch][v_index++];
                        v_index += 128 - 32;
                        d_index += 64 - 32;
                    }
                    for (int j = 0; j < 32; j++) out[((out_pos + j) << 1) + ch] = U_[j] / -1090519040.0f;
                }
                out_pos += 32;
            }
        }
    }

    // A frame whose audio needs more bits than it has is damaged: refused.
    // (The filter bank has taken its samples in: that is what pl_mpeg would
    // have done too, so the next frame continues as it would there.)
    if (b.pos > b.nbits) {
        pcm.clear();
        return false;
    }

    // ---- the format, and the bytes after the audio (pl_mpeg's plm_buffer_align)
    rate_ = (unsigned)samplerate;
    kbps_ = (unsigned)bitrate;
    mode_ = mode;
    mpeg2_ = version == kMpeg2;
    crc_ = hasCRC != 0;
    frameBytes_ = frame_size;
    audioBytes_ = (b.pos + 7) >> 3;
    if (audioBytes_ < frameBytes_) anc_.assign(frame + audioBytes_, frame + frameBytes_);

    // DAB's PAD (ETSI EN 300 401 7.4.1), as OFFgrid-SDR's browser version reads it: the last two bytes are the F-PAD,
    // before them the scale-factor CRC (2 bytes for MPEG-1 below 56 kbit/s mono or 112 kbit/s two-channel, else 4),
    // and before that the X-PAD field, which the PAD reader takes backwards from its end.
    std::size_t scf = (!mpeg2_ && bitrate < (mode == Mono ? 56 : 112)) ? 2 : 4;
    std::size_t body = frame_size - 4;   // (the frame after its 4-byte header)
    if (body > 2 + scf) {
        pad_.valid = true;
        pad_.fpad[0] = frame[frame_size - 2];
        pad_.fpad[1] = frame[frame_size - 1];
        pad_.scfCrcBytes = scf;
        pad_.xpadEnd = frame_size - 2 - scf;
    }
    return true;
}

const Mp2Decoder::Quant *Mp2Decoder::readAllocation(Bits &b, int sb, int tab3) {
    int tab4 = kQuantLutStep3[tab3][sb];
    int qtab = kQuantLutStep4[tab4 & 15][b.read(tab4 >> 4)];
    return qtab ? &kQuantTab[qtab - 1] : 0;
}

void Mp2Decoder::readSamples(Bits &b, int ch, int sb, int part) {
    const Quant *q = allocation_[ch][sb];
    int sf = scaleFactor_[ch][sb][part];
    int *sample = sample_[ch][sb];
    int val = 0;

    if (!q) {
        // no bits allocated for this subband
        sample[0] = sample[1] = sample[2] = 0;
        return;
    }

    // resolve the scalefactor
    if (sf == 63) {
        sf = 0;
    } else {
        int shift = (sf / 3) | 0;
        sf = (kScalefactorBase[sf % 3] + ((1 << shift) >> 1)) >> shift;
    }

    // decode the samples
    int adj = q->levels;
    if (q->group) {
        // grouped samples
        val = b.read(q->bits);
        sample[0] = val % adj;
        val /= adj;
        sample[1] = val % adj;
        sample[2] = val / adj;
    } else {
        // direct samples
        sample[0] = b.read(q->bits);
        sample[1] = b.read(q->bits);
        sample[2] = b.read(q->bits);
    }

    // postmultiply the samples
    int scale = 65536 / (adj + 1);
    adj = ((adj + 1) >> 1) - 1;

    val = (adj - sample[0]) * scale;
    sample[0] = (val * (sf >> 12) + ((val * (sf & 4095) + 2048) >> 12)) >> 12;

    val = (adj - sample[1]) * scale;
    sample[1] = (val * (sf >> 12) + ((val * (sf & 4095) + 2048) >> 12)) >> 12;

    val = (adj - sample[2]) * scale;
    sample[2] = (val * (sf >> 12) + ((val * (sf & 4095) + 2048) >> 12)) >> 12;
}

// pl_mpeg's 32-point DCT into the V buffer (plm_audio_idct36), as written there.
void Mp2Decoder::idct36(int s[32][3], int ss, float *d, int dp) {
    float t01, t02, t03, t04, t05, t06, t07, t08, t09, t10, t11, t12,
        t13, t14, t15, t16, t17, t18, t19, t20, t21, t22, t23, t24,
        t25, t26, t27, t28, t29, t30, t31, t32, t33;

    t01 = (float)(s[0][ss] + s[31][ss]); t02 = (float)(s[0][ss] - s[31][ss]) * 0.500602998235f;
    t03 = (float)(s[1][ss] + s[30][ss]); t04 = (float)(s[1][ss] - s[30][ss]) * 0.505470959898f;
    t05 = (float)(s[2][ss] + s[29][ss]); t06 = (float)(s[2][ss] - s[29][ss]) * 0.515447309923f;
    t07 = (float)(s[3][ss] + s[28][ss]); t08 = (float)(s[3][ss] - s[28][ss]) * 0.53104259109f;
    t09 = (float)(s[4][ss] + s[27][ss]); t10 = (float)(s[4][ss] - s[27][ss]) * 0.553103896034f;
    t11 = (float)(s[5][ss] + s[26][ss]); t12 = (float)(s[5][ss] - s[26][ss]) * 0.582934968206f;
    t13 = (float)(s[6][ss] + s[25][ss]); t14 = (float)(s[6][ss] - s[25][ss]) * 0.622504123036f;
    t15 = (float)(s[7][ss] + s[24][ss]); t16 = (float)(s[7][ss] - s[24][ss]) * 0.674808341455f;
    t17 = (float)(s[8][ss] + s[23][ss]); t18 = (float)(s[8][ss] - s[23][ss]) * 0.744536271002f;
    t19 = (float)(s[9][ss] + s[22][ss]); t20 = (float)(s[9][ss] - s[22][ss]) * 0.839349645416f;
    t21 = (float)(s[10][ss] + s[21][ss]); t22 = (float)(s[10][ss] - s[21][ss]) * 0.972568237862f;
    t23 = (float)(s[11][ss] + s[20][ss]); t24 = (float)(s[11][ss] - s[20][ss]) * 1.16943993343f;
    t25 = (float)(s[12][ss] + s[19][ss]); t26 = (float)(s[12][ss] - s[19][ss]) * 1.48416461631f;
    t27 = (float)(s[13][ss] + s[18][ss]); t28 = (float)(s[13][ss] - s[18][ss]) * 2.05778100995f;
    t29 = (float)(s[14][ss] + s[17][ss]); t30 = (float)(s[14][ss] - s[17][ss]) * 3.40760841847f;
    t31 = (float)(s[15][ss] + s[16][ss]); t32 = (float)(s[15][ss] - s[16][ss]) * 10.1900081235f;

    t33 = t01 + t31; t31 = (t01 - t31) * 0.502419286188f;
    t01 = t03 + t29; t29 = (t03 - t29) * 0.52249861494f;
    t03 = t05 + t27; t27 = (t05 - t27) * 0.566944034816f;
    t05 = t07 + t25; t25 = (t07 - t25) * 0.64682178336f;
    t07 = t09 + t23; t23 = (t09 - t23) * 0.788154623451f;
    t09 = t11 + t21; t21 = (t11 - t21) * 1.06067768599f;
    t11 = t13 + t19; t19 = (t13 - t19) * 1.72244709824f;
    t13 = t15 + t17; t17 = (t15 - t17) * 5.10114861869f;
    t15 = t33 + t13; t13 = (t33 - t13) * 0.509795579104f;
    t33 = t01 + t11; t01 = (t01 - t11) * 0.601344886935f;
    t11 = t03 + t09; t09 = (t03 - t09) * 0.899976223136f;
    t03 = t05 + t07; t07 = (t05 - t07) * 2.56291544774f;
    t05 = t15 + t03; t15 = (t15 - t03) * 0.541196100146f;
    t03 = t33 + t11; t11 = (t33 - t11) * 1.30656296488f;
    t33 = t05 + t03; t05 = (t05 - t03) * 0.707106781187f;
    t03 = t15 + t11; t15 = (t15 - t11) * 0.707106781187f;
    t03 += t15;
    t11 = t13 + t07; t13 = (t13 - t07) * 0.541196100146f;
    t07 = t01 + t09; t09 = (t01 - t09) * 1.30656296488f;
    t01 = t11 + t07; t07 = (t11 - t07) * 0.707106781187f;
    t11 = t13 + t09; t13 = (t13 - t09) * 0.707106781187f;
    t11 += t13; t01 += t11;
    t11 += t07; t07 += t13;
    t09 = t31 + t17; t31 = (t31 - t17) * 0.509795579104f;
    t17 = t29 + t19; t29 = (t29 - t19) * 0.601344886935f;
    t19 = t27 + t21; t21 = (t27 - t21) * 0.899976223136f;
    t27 = t25 + t23; t23 = (t25 - t23) * 2.56291544774f;
    t25 = t09 + t27; t09 = (t09 - t27) * 0.541196100146f;
    t27 = t17 + t19; t19 = (t17 - t19) * 1.30656296488f;
    t17 = t25 + t27; t27 = (t25 - t27) * 0.707106781187f;
    t25 = t09 + t19; t19 = (t09 - t19) * 0.707106781187f;
    t25 += t19;
    t09 = t31 + t23; t31 = (t31 - t23) * 0.541196100146f;
    t23 = t29 + t21; t21 = (t29 - t21) * 1.30656296488f;
    t29 = t09 + t23; t23 = (t09 - t23) * 0.707106781187f;
    t09 = t31 + t21; t31 = (t31 - t21) * 0.707106781187f;
    t09 += t31;    t29 += t09;    t09 += t23;    t23 += t31;
    t17 += t29;    t29 += t25;    t25 += t09;    t09 += t27;
    t27 += t23;    t23 += t19; t19 += t31;
    t21 = t02 + t32; t02 = (t02 - t32) * 0.502419286188f;
    t32 = t04 + t30; t04 = (t04 - t30) * 0.52249861494f;
    t30 = t06 + t28; t28 = (t06 - t28) * 0.566944034816f;
    t06 = t08 + t26; t08 = (t08 - t26) * 0.64682178336f;
    t26 = t10 + t24; t10 = (t10 - t24) * 0.788154623451f;
    t24 = t12 + t22; t22 = (t12 - t22) * 1.06067768599f;
    t12 = t14 + t20; t20 = (t14 - t20) * 1.72244709824f;
    t14 = t16 + t18; t16 = (t16 - t18) * 5.10114861869f;
    t18 = t21 + t14; t14 = (t21 - t14) * 0.509795579104f;
    t21 = t32 + t12; t32 = (t32 - t12) * 0.601344886935f;
    t12 = t30 + t24; t24 = (t30 - t24) * 0.899976223136f;
    t30 = t06 + t26; t26 = (t06 - t26) * 2.56291544774f;
    t06 = t18 + t30; t18 = (t18 - t30) * 0.541196100146f;
    t30 = t21 + t12; t12 = (t21 - t12) * 1.30656296488f;
    t21 = t06 + t30; t30 = (t06 - t30) * 0.707106781187f;
    t06 = t18 + t12; t12 = (t18 - t12) * 0.707106781187f;
    t06 += t12;
    t18 = t14 + t26; t26 = (t14 - t26) * 0.541196100146f;
    t14 = t32 + t24; t24 = (t32 - t24) * 1.30656296488f;
    t32 = t18 + t14; t14 = (t18 - t14) * 0.707106781187f;
    t18 = t26 + t24; t24 = (t26 - t24) * 0.707106781187f;
    t18 += t24; t32 += t18;
    t18 += t14; t26 = t14 + t24;
    t14 = t02 + t16; t02 = (t02 - t16) * 0.509795579104f;
    t16 = t04 + t20; t04 = (t04 - t20) * 0.601344886935f;
    t20 = t28 + t22; t22 = (t28 - t22) * 0.899976223136f;
    t28 = t08 + t10; t10 = (t08 - t10) * 2.56291544774f;
    t08 = t14 + t28; t14 = (t14 - t28) * 0.541196100146f;
    t28 = t16 + t20; t20 = (t16 - t20) * 1.30656296488f;
    t16 = t08 + t28; t28 = (t08 - t28) * 0.707106781187f;
    t08 = t14 + t20; t20 = (t14 - t20) * 0.707106781187f;
    t08 += t20;
    t14 = t02 + t10; t02 = (t02 - t10) * 0.541196100146f;
    t10 = t04 + t22; t22 = (t04 - t22) * 1.30656296488f;
    t04 = t14 + t10; t10 = (t14 - t10) * 0.707106781187f;
    t14 = t02 + t22; t02 = (t02 - t22) * 0.707106781187f;
    t14 += t02;    t04 += t14;    t14 += t10;    t10 += t02;
    t16 += t04;    t04 += t08;    t08 += t14;    t14 += t28;
    t28 += t10;    t10 += t20;    t20 += t02;    t21 += t16;
    t16 += t32;    t32 += t04;    t04 += t06;    t06 += t08;
    t08 += t18;    t18 += t14;    t14 += t30;    t30 += t28;
    t28 += t26;    t26 += t10;    t10 += t12;    t12 += t20;
    t20 += t24;    t24 += t02;

    d[dp + 48] = -t33;
    d[dp + 49] = d[dp + 47] = -t21;
    d[dp + 50] = d[dp + 46] = -t17;
    d[dp + 51] = d[dp + 45] = -t16;
    d[dp + 52] = d[dp + 44] = -t01;
    d[dp + 53] = d[dp + 43] = -t32;
    d[dp + 54] = d[dp + 42] = -t29;
    d[dp + 55] = d[dp + 41] = -t04;
    d[dp + 56] = d[dp + 40] = -t03;
    d[dp + 57] = d[dp + 39] = -t06;
    d[dp + 58] = d[dp + 38] = -t25;
    d[dp + 59] = d[dp + 37] = -t08;
    d[dp + 60] = d[dp + 36] = -t11;
    d[dp + 61] = d[dp + 35] = -t18;
    d[dp + 62] = d[dp + 34] = -t09;
    d[dp + 63] = d[dp + 33] = -t14;
    d[dp + 32] = -t05;
    d[dp + 0] = t05; d[dp + 31] = -t30;
    d[dp + 1] = t30; d[dp + 30] = -t27;
    d[dp + 2] = t27; d[dp + 29] = -t28;
    d[dp + 3] = t28; d[dp + 28] = -t07;
    d[dp + 4] = t07; d[dp + 27] = -t26;
    d[dp + 5] = t26; d[dp + 26] = -t23;
    d[dp + 6] = t23; d[dp + 25] = -t10;
    d[dp + 7] = t10; d[dp + 24] = -t15;
    d[dp + 8] = t15; d[dp + 23] = -t12;
    d[dp + 9] = t12; d[dp + 22] = -t19;
    d[dp + 10] = t19; d[dp + 21] = -t20;
    d[dp + 11] = t20; d[dp + 20] = -t13;
    d[dp + 12] = t13; d[dp + 19] = -t24;
    d[dp + 13] = t24; d[dp + 18] = -t31;
    d[dp + 14] = t31; d[dp + 17] = -t02;
    d[dp + 15] = t02; d[dp + 16] = 0.0f;
}

} // namespace codec
