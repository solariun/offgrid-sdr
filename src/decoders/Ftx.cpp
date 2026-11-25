// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Ftx.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  FT8 / FT4 messages, codes, tones and the slot decoder of Ftx.h: a port of
//  OFFgrid-SDR's `ft8lib` script (diddy-boy, index.html: crc14, ldpcEncode,
//  ldpcCheck, bpDecode, pack28, packGrid, pack77, tones, tones4, unpack28,
//  unpackGrid, unpack77, ft8Precise, decodeOnce, decode4Once,
//  gfskPhaseSteps, matchPower, refine, subtractSignal, decode, decode4),
//  whose protocol tables, CRC, message formats and method follow ft8_lib
//  by Kārlis Goba (MIT licence, see Ftx.h; https://github.com/kgoba/ft8_lib).
//  The LDPC tables below are the browser's (generated from its GEN_HEX, NM,
//  MN and NROWS), which are ft8_lib's.
//
//  Additions to the JS (each marked where it is):
//    - the message types the JS only names ("(message type 0.1)" ...):
//      DXpedition (0.1), ARRL Field Day (0.3, 0.4), ARRL RTTY Roundup (3)
//      and EU VHF contest (5) are packed and unpacked as WSJT-X defines them
//      (the section and state lists are WSJT-X's); nonstandard calls (4),
//      telemetry (0.5) and hashed calls "<CALL>" in standard messages can
//      also be packed (the JS packs standard messages and free text only);
//    - reports outside -30..+99 are not packed as reports (the JS packed
//      e.g. "-45" into a value that reads back as a locator);
//    - the hash table is bounded (the JS's grows for ever);
//    - every decode's start and frequency are fine-tuned on the audio (the
//      subtraction's refine(), run for the last pass too) and reported, so
//      DT is to a few ms rather than to the 80 ms (FT8) / 24 ms (FT4)
//      spectrogram step; refine() correlates every 8th / 2nd sample (the
//      JS every 32nd / 16th, which loses 12-15 dB to folded noise: weak
//      signals' timing wandered), made cheap by tabulating the GFSK phase's
//      cosine and sine and turning the carrier as a phasor;
//    - FT4 gets the exact second measurement of a failed candidate too (the
//      JS: FT8 only), worth about 2 dB;
//    - the exact measurement works the tones (and the sync's trial
//      frequencies) side by side, single precision for the tones: the same
//      numbers to ~1e-3 dB, a few times faster;
//    - NaN / infinite samples are taken as silence;
//    - the GFSK pulse uses the exact erf (the JS: a 1.5e-7 approximation);
//      the synthesiser works at any rate (fractional samples per symbol).
//
//  How it fits in
//  --------------
//  See Ftx.h.  Ft8.cpp feeds it slots of live audio.
//
//  Contributing
//  ------------
//  Keep the function names parallel to the JavaScript; mark deviations;
//  keep the credit to ft8_lib.  No allocation per sample: the Engine's
//  buffers are sized on first use and kept.
// ============================================================================
#include "Ftx.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace dec {
namespace ftx {

const u8 kFt8Costas[7] = {3, 1, 4, 0, 6, 5, 2};
const u8 kFt8Gray[8] = {0, 1, 3, 2, 5, 6, 4, 7};
const u8 kFt4Costas[4][4] = {{0, 1, 3, 2}, {1, 0, 2, 3}, {2, 3, 1, 0}, {3, 2, 0, 1}};
const u8 kFt4Gray[4] = {0, 1, 3, 2};
const u8 kFt4Xor[10] = {0x4A, 0x5E, 0x89, 0xB4, 0xB0, 0x8A, 0x79, 0x55, 0xBE, 0x28};

namespace {

const double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------- the LDPC(174,91) tables (ft8_lib's)
// The generator matrix: 83 rows of 91 bits (12 bytes each, MSB first).
const unsigned char kGen[83][12] = {
    {0x83, 0x29, 0xce, 0x11, 0xbf, 0x31, 0xea, 0xf5, 0x09, 0xf2, 0x7f, 0xc0},
    {0x76, 0x1c, 0x26, 0x4e, 0x25, 0xc2, 0x59, 0x33, 0x54, 0x93, 0x13, 0x20},
    {0xdc, 0x26, 0x59, 0x02, 0xfb, 0x27, 0x7c, 0x64, 0x10, 0xa1, 0xbd, 0xc0},
    {0x1b, 0x3f, 0x41, 0x78, 0x58, 0xcd, 0x2d, 0xd3, 0x3e, 0xc7, 0xf6, 0x20},
    {0x09, 0xfd, 0xa4, 0xfe, 0xe0, 0x41, 0x95, 0xfd, 0x03, 0x47, 0x83, 0xa0},
    {0x07, 0x7c, 0xcc, 0xc1, 0x1b, 0x88, 0x73, 0xed, 0x5c, 0x3d, 0x48, 0xa0},
    {0x29, 0xb6, 0x2a, 0xfe, 0x3c, 0xa0, 0x36, 0xf4, 0xfe, 0x1a, 0x9d, 0xa0},
    {0x60, 0x54, 0xfa, 0xf5, 0xf3, 0x5d, 0x96, 0xd3, 0xb0, 0xc8, 0xc3, 0xe0},
    {0xe2, 0x07, 0x98, 0xe4, 0x31, 0x0e, 0xed, 0x27, 0x88, 0x4a, 0xe9, 0x00},
    {0x77, 0x5c, 0x9c, 0x08, 0xe8, 0x0e, 0x26, 0xdd, 0xae, 0x56, 0x31, 0x80},
    {0xb0, 0xb8, 0x11, 0x02, 0x8c, 0x2b, 0xf9, 0x97, 0x21, 0x34, 0x87, 0xc0},
    {0x18, 0xa0, 0xc9, 0x23, 0x1f, 0xc6, 0x0a, 0xdf, 0x5c, 0x5e, 0xa3, 0x20},
    {0x76, 0x47, 0x1e, 0x83, 0x02, 0xa0, 0x72, 0x1e, 0x01, 0xb1, 0x2b, 0x80},
    {0xff, 0xbc, 0xcb, 0x80, 0xca, 0x83, 0x41, 0xfa, 0xfb, 0x47, 0xb2, 0xe0},
    {0x66, 0xa7, 0x2a, 0x15, 0x8f, 0x93, 0x25, 0xa2, 0xbf, 0x67, 0x17, 0x00},
    {0xc4, 0x24, 0x36, 0x89, 0xfe, 0x85, 0xb1, 0xc5, 0x13, 0x63, 0xa1, 0x80},
    {0x0d, 0xff, 0x73, 0x94, 0x14, 0xd1, 0xa1, 0xb3, 0x4b, 0x1c, 0x27, 0x00},
    {0x15, 0xb4, 0x88, 0x30, 0x63, 0x6c, 0x8b, 0x99, 0x89, 0x49, 0x72, 0xe0},
    {0x29, 0xa8, 0x9c, 0x0d, 0x3d, 0xe8, 0x1d, 0x66, 0x54, 0x89, 0xb0, 0xe0},
    {0x4f, 0x12, 0x6f, 0x37, 0xfa, 0x51, 0xcb, 0xe6, 0x1b, 0xd6, 0xb9, 0x40},
    {0x99, 0xc4, 0x72, 0x39, 0xd0, 0xd9, 0x7d, 0x3c, 0x84, 0xe0, 0x94, 0x00},
    {0x19, 0x19, 0xb7, 0x51, 0x19, 0x76, 0x56, 0x21, 0xbb, 0x4f, 0x1e, 0x80},
    {0x09, 0xdb, 0x12, 0xd7, 0x31, 0xfa, 0xee, 0x0b, 0x86, 0xdf, 0x6b, 0x80},
    {0x48, 0x8f, 0xc3, 0x3d, 0xf4, 0x3f, 0xbd, 0xee, 0xa4, 0xea, 0xfb, 0x40},
    {0x82, 0x74, 0x23, 0xee, 0x40, 0xb6, 0x75, 0xf7, 0x56, 0xeb, 0x5f, 0xe0},
    {0xab, 0xe1, 0x97, 0xc4, 0x84, 0xcb, 0x74, 0x75, 0x71, 0x44, 0xa9, 0xa0},
    {0x2b, 0x50, 0x0e, 0x4b, 0xc0, 0xec, 0x5a, 0x6d, 0x2b, 0xdb, 0xdd, 0x00},
    {0xc4, 0x74, 0xaa, 0x53, 0xd7, 0x02, 0x18, 0x76, 0x16, 0x69, 0x36, 0x00},
    {0x8e, 0xba, 0x1a, 0x13, 0xdb, 0x33, 0x90, 0xbd, 0x67, 0x18, 0xce, 0xc0},
    {0x75, 0x38, 0x44, 0x67, 0x3a, 0x27, 0x78, 0x2c, 0xc4, 0x20, 0x12, 0xe0},
    {0x06, 0xff, 0x83, 0xa1, 0x45, 0xc3, 0x70, 0x35, 0xa5, 0xc1, 0x26, 0x80},
    {0x3b, 0x37, 0x41, 0x78, 0x58, 0xcc, 0x2d, 0xd3, 0x3e, 0xc3, 0xf6, 0x20},
    {0x9a, 0x4a, 0x5a, 0x28, 0xee, 0x17, 0xca, 0x9c, 0x32, 0x48, 0x42, 0xc0},
    {0xbc, 0x29, 0xf4, 0x65, 0x30, 0x9c, 0x97, 0x7e, 0x89, 0x61, 0x0a, 0x40},
    {0x26, 0x63, 0xae, 0x6d, 0xdf, 0x8b, 0x5c, 0xe2, 0xbb, 0x29, 0x48, 0x80},
    {0x46, 0xf2, 0x31, 0xef, 0xe4, 0x57, 0x03, 0x4c, 0x18, 0x14, 0x41, 0x80},
    {0x3f, 0xb2, 0xce, 0x85, 0xab, 0xe9, 0xb0, 0xc7, 0x2e, 0x06, 0xfb, 0xe0},
    {0xde, 0x87, 0x48, 0x1f, 0x28, 0x2c, 0x15, 0x39, 0x71, 0xa0, 0xa2, 0xe0},
    {0xfc, 0xd7, 0xcc, 0xf2, 0x3c, 0x69, 0xfa, 0x99, 0xbb, 0xa1, 0x41, 0x20},
    {0xf0, 0x26, 0x14, 0x47, 0xe9, 0x49, 0x0c, 0xa8, 0xe4, 0x74, 0xce, 0xc0},
    {0x44, 0x10, 0x11, 0x58, 0x18, 0x19, 0x6f, 0x95, 0xcd, 0xd7, 0x01, 0x20},
    {0x08, 0x8f, 0xc3, 0x1d, 0xf4, 0xbf, 0xbd, 0xe2, 0xa4, 0xea, 0xfb, 0x40},
    {0xb8, 0xfe, 0xf1, 0xb6, 0x30, 0x77, 0x29, 0xfb, 0x0a, 0x07, 0x8c, 0x00},
    {0x5a, 0xfe, 0xa7, 0xac, 0xcc, 0xb7, 0x7b, 0xbc, 0x9d, 0x99, 0xa9, 0x00},
    {0x49, 0xa7, 0x01, 0x6a, 0xc6, 0x53, 0xf6, 0x5e, 0xcd, 0xc9, 0x07, 0x60},
    {0x19, 0x44, 0xd0, 0x85, 0xbe, 0x4e, 0x7d, 0xa8, 0xd6, 0xcc, 0x7d, 0x00},
    {0x25, 0x1f, 0x62, 0xad, 0xc4, 0x03, 0x2f, 0x0e, 0xe7, 0x14, 0x00, 0x20},
    {0x56, 0x47, 0x1f, 0x87, 0x02, 0xa0, 0x72, 0x1e, 0x00, 0xb1, 0x2b, 0x80},
    {0x2b, 0x8e, 0x49, 0x23, 0xf2, 0xdd, 0x51, 0xe2, 0xd5, 0x37, 0xfa, 0x00},
    {0x6b, 0x55, 0x0a, 0x40, 0xa6, 0x6f, 0x47, 0x55, 0xde, 0x95, 0xc2, 0x60},
    {0xa1, 0x8a, 0xd2, 0x8d, 0x4e, 0x27, 0xfe, 0x92, 0xa4, 0xf6, 0xc8, 0x40},
    {0x10, 0xc2, 0xe5, 0x86, 0x38, 0x8c, 0xb8, 0x2a, 0x3d, 0x80, 0x75, 0x80},
    {0xef, 0x34, 0xa4, 0x18, 0x17, 0xee, 0x02, 0x13, 0x3d, 0xb2, 0xeb, 0x00},
    {0x7e, 0x9c, 0x0c, 0x54, 0x32, 0x5a, 0x9c, 0x15, 0x83, 0x6e, 0x00, 0x00},
    {0x36, 0x93, 0xe5, 0x72, 0xd1, 0xfd, 0xe4, 0xcd, 0xf0, 0x79, 0xe8, 0x60},
    {0xbf, 0xb2, 0xce, 0xc5, 0xab, 0xe1, 0xb0, 0xc7, 0x2e, 0x07, 0xfb, 0xe0},
    {0x7e, 0xe1, 0x82, 0x30, 0xc5, 0x83, 0xcc, 0xcc, 0x57, 0xd4, 0xb0, 0x80},
    {0xa0, 0x66, 0xcb, 0x2f, 0xed, 0xaf, 0xc9, 0xf5, 0x26, 0x64, 0x12, 0x60},
    {0xbb, 0x23, 0x72, 0x5a, 0xbc, 0x47, 0xcc, 0x5f, 0x4c, 0xc4, 0xcd, 0x20},
    {0xde, 0xd9, 0xdb, 0xa3, 0xbe, 0xe4, 0x0c, 0x59, 0xb5, 0x60, 0x9b, 0x40},
    {0xd9, 0xa7, 0x01, 0x6a, 0xc6, 0x53, 0xe6, 0xde, 0xcd, 0xc9, 0x03, 0x60},
    {0x9a, 0xd4, 0x6a, 0xed, 0x5f, 0x70, 0x7f, 0x28, 0x0a, 0xb5, 0xfc, 0x40},
    {0xe5, 0x92, 0x1c, 0x77, 0x82, 0x25, 0x87, 0x31, 0x6d, 0x7d, 0x3c, 0x20},
    {0x4f, 0x14, 0xda, 0x82, 0x42, 0xa8, 0xb8, 0x6d, 0xca, 0x73, 0x35, 0x20},
    {0x8b, 0x8b, 0x50, 0x7a, 0xd4, 0x67, 0xd4, 0x44, 0x1d, 0xf7, 0x70, 0xe0},
    {0x22, 0x83, 0x1c, 0x9c, 0xf1, 0x16, 0x94, 0x67, 0xad, 0x04, 0xb6, 0x80},
    {0x21, 0x3b, 0x83, 0x8f, 0xe2, 0xae, 0x54, 0xc3, 0x8e, 0xe7, 0x18, 0x00},
    {0x5d, 0x92, 0x6b, 0x6d, 0xd7, 0x1f, 0x08, 0x51, 0x81, 0xa4, 0xe1, 0x20},
    {0x66, 0xab, 0x79, 0xd4, 0xb2, 0x9e, 0xe6, 0xe6, 0x95, 0x09, 0xe5, 0x60},
    {0x95, 0x81, 0x48, 0x68, 0x2d, 0x74, 0x8a, 0x38, 0xdd, 0x68, 0xba, 0xa0},
    {0xb8, 0xce, 0x02, 0x0c, 0xf0, 0x69, 0xc3, 0x2a, 0x72, 0x3a, 0xb1, 0x40},
    {0xf4, 0x33, 0x1d, 0x6d, 0x46, 0x16, 0x07, 0xe9, 0x57, 0x52, 0x74, 0x60},
    {0x6d, 0xa2, 0x3b, 0xa4, 0x24, 0xb9, 0x59, 0x61, 0x33, 0xcf, 0x9c, 0x80},
    {0xa6, 0x36, 0xbc, 0xbc, 0x7b, 0x30, 0xc5, 0xfb, 0xea, 0xe6, 0x7f, 0xe0},
    {0x5c, 0xb0, 0xd8, 0x6a, 0x07, 0xdf, 0x65, 0x4a, 0x90, 0x89, 0xa2, 0x00},
    {0xf1, 0x1f, 0x10, 0x68, 0x48, 0x78, 0x0f, 0xc9, 0xec, 0xdd, 0x80, 0xa0},
    {0x1f, 0xbb, 0x53, 0x64, 0xfb, 0x8d, 0x2c, 0x9d, 0x73, 0x0d, 0x5b, 0xa0},
    {0xfc, 0xb8, 0x6b, 0xc7, 0x0a, 0x50, 0xc9, 0xd0, 0x2a, 0x5d, 0x03, 0x40},
    {0xa5, 0x34, 0x43, 0x30, 0x29, 0xea, 0xc1, 0x5f, 0x32, 0x2e, 0x34, 0xc0},
    {0xc9, 0x89, 0xd9, 0xc7, 0xc3, 0xd3, 0xb8, 0xc5, 0x5d, 0x75, 0x13, 0x00},
    {0x7b, 0xb3, 0x8b, 0x2f, 0x01, 0x86, 0xd4, 0x66, 0x43, 0xae, 0x96, 0x20},
    {0x26, 0x44, 0xeb, 0xad, 0xeb, 0x44, 0xb9, 0x46, 0x7d, 0x1f, 0x42, 0xc0},
    {0x60, 0x8c, 0xc8, 0x57, 0x59, 0x4b, 0xfb, 0xb5, 0x5d, 0x69, 0x60, 0x00},
};
// Nm: for each of the 83 checks, the (1-based) bits it covers (6 or 7; 0 = none).
const unsigned char kNm[83][7] = {
    {4, 31, 59, 91, 92, 96, 153},
    {5, 32, 60, 93, 115, 146, 0},
    {6, 24, 61, 94, 122, 151, 0},
    {7, 33, 62, 95, 96, 143, 0},
    {8, 25, 63, 83, 93, 96, 148},
    {6, 32, 64, 97, 126, 138, 0},
    {5, 34, 65, 78, 98, 107, 154},
    {9, 35, 66, 99, 139, 146, 0},
    {10, 36, 67, 100, 107, 126, 0},
    {11, 37, 67, 87, 101, 139, 158},
    {12, 38, 68, 102, 105, 155, 0},
    {13, 39, 69, 103, 149, 162, 0},
    {8, 40, 70, 82, 104, 114, 145},
    {14, 41, 71, 88, 102, 123, 156},
    {15, 42, 59, 106, 123, 159, 0},
    {1, 33, 72, 106, 107, 157, 0},
    {16, 43, 73, 108, 141, 160, 0},
    {17, 37, 74, 81, 109, 131, 154},
    {11, 44, 75, 110, 121, 166, 0},
    {45, 55, 64, 111, 130, 161, 173},
    {8, 46, 71, 112, 119, 166, 0},
    {18, 36, 76, 89, 113, 114, 143},
    {19, 38, 77, 104, 116, 163, 0},
    {20, 47, 70, 92, 138, 165, 0},
    {2, 48, 74, 113, 128, 160, 0},
    {21, 45, 78, 83, 117, 121, 151},
    {22, 47, 58, 118, 127, 164, 0},
    {16, 39, 62, 112, 134, 158, 0},
    {23, 43, 79, 120, 131, 145, 0},
    {19, 35, 59, 73, 110, 125, 161},
    {20, 36, 63, 94, 136, 161, 0},
    {14, 31, 79, 98, 132, 164, 0},
    {3, 44, 80, 124, 127, 169, 0},
    {19, 46, 81, 117, 135, 167, 0},
    {7, 49, 58, 90, 100, 105, 168},
    {12, 50, 61, 118, 119, 144, 0},
    {13, 51, 64, 114, 118, 157, 0},
    {24, 52, 76, 129, 148, 149, 0},
    {25, 53, 69, 90, 101, 130, 156},
    {20, 46, 65, 80, 120, 140, 170},
    {21, 54, 77, 100, 140, 171, 0},
    {35, 82, 133, 142, 171, 174, 0},
    {14, 30, 83, 113, 125, 170, 0},
    {4, 29, 68, 120, 134, 173, 0},
    {1, 4, 52, 57, 86, 136, 152},
    {26, 51, 56, 91, 122, 137, 168},
    {52, 84, 110, 115, 145, 168, 0},
    {7, 50, 81, 99, 132, 173, 0},
    {23, 55, 67, 95, 172, 174, 0},
    {26, 41, 77, 109, 141, 148, 0},
    {2, 27, 41, 61, 62, 115, 133},
    {27, 40, 56, 124, 125, 126, 0},
    {18, 49, 55, 124, 141, 167, 0},
    {6, 33, 85, 108, 116, 156, 0},
    {28, 48, 70, 85, 105, 129, 158},
    {9, 54, 63, 131, 147, 155, 0},
    {22, 53, 68, 109, 121, 174, 0},
    {3, 13, 48, 78, 95, 123, 0},
    {31, 69, 133, 150, 155, 169, 0},
    {12, 43, 66, 89, 97, 135, 159},
    {5, 39, 75, 102, 136, 167, 0},
    {2, 54, 86, 101, 135, 164, 0},
    {15, 56, 87, 108, 119, 171, 0},
    {10, 44, 82, 91, 111, 144, 149},
    {23, 34, 71, 94, 127, 153, 0},
    {11, 49, 88, 92, 142, 157, 0},
    {29, 34, 87, 97, 147, 162, 0},
    {30, 50, 60, 86, 137, 142, 162},
    {10, 53, 66, 84, 112, 128, 165},
    {22, 57, 85, 93, 140, 159, 0},
    {28, 32, 72, 103, 132, 166, 0},
    {28, 29, 84, 88, 117, 143, 150},
    {1, 26, 45, 80, 128, 147, 0},
    {17, 27, 89, 103, 116, 153, 0},
    {51, 57, 98, 163, 165, 172, 0},
    {21, 37, 73, 138, 152, 169, 0},
    {16, 47, 76, 130, 137, 154, 0},
    {3, 24, 30, 72, 104, 139, 0},
    {9, 40, 90, 106, 134, 151, 0},
    {15, 58, 60, 74, 111, 150, 163},
    {18, 42, 79, 144, 146, 152, 0},
    {25, 38, 65, 99, 122, 160, 0},
    {17, 42, 75, 129, 170, 172, 0},
};
// Mn: for each of the 174 bits, the (1-based) three checks it is in.
const unsigned char kMn[174][3] = {
    {16, 45, 73}, {25, 51, 62}, {33, 58, 78}, {1, 44, 45}, {2, 7, 61}, {3, 6, 54},
    {4, 35, 48}, {5, 13, 21}, {8, 56, 79}, {9, 64, 69}, {10, 19, 66}, {11, 36, 60},
    {12, 37, 58}, {14, 32, 43}, {15, 63, 80}, {17, 28, 77}, {18, 74, 83}, {22, 53, 81},
    {23, 30, 34}, {24, 31, 40}, {26, 41, 76}, {27, 57, 70}, {29, 49, 65}, {3, 38, 78},
    {5, 39, 82}, {46, 50, 73}, {51, 52, 74}, {55, 71, 72}, {44, 67, 72}, {43, 68, 78},
    {1, 32, 59}, {2, 6, 71}, {4, 16, 54}, {7, 65, 67}, {8, 30, 42}, {9, 22, 31},
    {10, 18, 76}, {11, 23, 82}, {12, 28, 61}, {13, 52, 79}, {14, 50, 51}, {15, 81, 83},
    {17, 29, 60}, {19, 33, 64}, {20, 26, 73}, {21, 34, 40}, {24, 27, 77}, {25, 55, 58},
    {35, 53, 66}, {36, 48, 68}, {37, 46, 75}, {38, 45, 47}, {39, 57, 69}, {41, 56, 62},
    {20, 49, 53}, {46, 52, 63}, {45, 70, 75}, {27, 35, 80}, {1, 15, 30}, {2, 68, 80},
    {3, 36, 51}, {4, 28, 51}, {5, 31, 56}, {6, 20, 37}, {7, 40, 82}, {8, 60, 69},
    {9, 10, 49}, {11, 44, 57}, {12, 39, 59}, {13, 24, 55}, {14, 21, 65}, {16, 71, 78},
    {17, 30, 76}, {18, 25, 80}, {19, 61, 83}, {22, 38, 77}, {23, 41, 50}, {7, 26, 58},
    {29, 32, 81}, {33, 40, 73}, {18, 34, 48}, {13, 42, 64}, {5, 26, 43}, {47, 69, 72},
    {54, 55, 70}, {45, 62, 68}, {10, 63, 67}, {14, 66, 72}, {22, 60, 74}, {35, 39, 79},
    {1, 46, 64}, {1, 24, 66}, {2, 5, 70}, {3, 31, 65}, {4, 49, 58}, {1, 4, 5},
    {6, 60, 67}, {7, 32, 75}, {8, 48, 82}, {9, 35, 41}, {10, 39, 62}, {11, 14, 61},
    {12, 71, 74}, {13, 23, 78}, {11, 35, 55}, {15, 16, 79}, {7, 9, 16}, {17, 54, 63},
    {18, 50, 57}, {19, 30, 47}, {20, 64, 80}, {21, 28, 69}, {22, 25, 43}, {13, 22, 37},
    {2, 47, 51}, {23, 54, 74}, {26, 34, 72}, {27, 36, 37}, {21, 36, 63}, {29, 40, 44},
    {19, 26, 57}, {3, 46, 82}, {14, 15, 58}, {33, 52, 53}, {30, 43, 52}, {6, 9, 52},
    {27, 33, 65}, {25, 69, 73}, {38, 55, 83}, {20, 39, 77}, {18, 29, 56}, {32, 48, 71},
    {42, 51, 59}, {28, 44, 79}, {34, 60, 62}, {31, 45, 61}, {46, 68, 77}, {6, 24, 76},
    {8, 10, 78}, {40, 41, 70}, {17, 50, 53}, {42, 66, 68}, {4, 22, 72}, {36, 64, 81},
    {13, 29, 47}, {2, 8, 81}, {56, 67, 73}, {5, 38, 50}, {12, 38, 64}, {59, 72, 80},
    {3, 26, 79}, {45, 76, 81}, {1, 65, 74}, {7, 18, 77}, {11, 56, 59}, {14, 39, 54},
    {16, 37, 66}, {10, 28, 55}, {15, 60, 70}, {17, 25, 82}, {20, 30, 31}, {12, 67, 68},
    {23, 75, 80}, {27, 32, 62}, {24, 69, 75}, {19, 21, 71}, {34, 53, 61}, {35, 46, 47},
    {33, 59, 76}, {40, 43, 83}, {41, 42, 63}, {49, 75, 83}, {20, 44, 48}, {42, 49, 57},
};
const unsigned char kNumRows[83] = {
    7, 6, 6, 6, 7, 6, 7, 6, 6, 7, 6, 6, 7, 7, 6, 6, 6, 7, 6, 7,
    6, 7, 6, 6, 6, 7, 6, 6, 6, 7, 6, 6, 6, 6, 7, 6, 6, 6, 7, 7,
    6, 6, 6, 6, 7, 7, 6, 6, 6, 6, 7, 6, 6, 6, 7, 6, 6, 6, 6, 7,
    6, 6, 6, 7, 6, 6, 6, 7, 7, 6, 6, 7, 6, 6, 6, 6, 6, 6, 6, 7,
    6, 6, 6,
};

// ---------------------------------------------------------------- characters and numbers
const char T_FULL[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ+-./?";
const char T_AN_SP[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const char T_AN[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const char T_NUM[] = "0123456789";
const char T_LET_SP[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const char T_AN_SP_SL[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";

const long NTOKENS = 2063592, MAX22 = 4194304, MAXGRID4 = 32400;

// (Addition to the JS) WSJT-X's lists for the contest messages: ARRL / RAC sections (Field Day) and US states and
// Canadian provinces (RTTY Roundup), 1-based in the messages
const char* const kSections[] = {
    "AB", "AK", "AL", "AR", "AZ", "BC", "CO", "CT", "DE", "EB", "EMA", "ENY", "EPA", "EWA", "GA", "GTA", "IA",
    "ID", "IL", "IN", "KS", "KY", "LA", "LAX", "MAR", "MB", "MDC", "ME", "MI", "MN", "MO", "MS", "MT", "NC",
    "ND", "NE", "NFL", "NH", "NL", "NLI", "NM", "NNJ", "NNY", "NT", "NTX", "NV", "OH", "OK", "ONE", "ONN",
    "ONS", "OR", "ORG", "PAC", "PR", "QC", "RI", "SB", "SC", "SCV", "SD", "SDG", "SF", "SFL", "SJV", "SK",
    "SNJ", "STX", "SV", "TN", "UT", "VA", "VI", "VT", "WCF", "WI", "WMA", "WNY", "WPA", "WTX", "WV", "WWA",
    "WY", "DX"};
const int kNSections = static_cast<int>(sizeof(kSections) / sizeof(kSections[0]));
const char* const kStates[] = {
    "AL", "AK", "AZ", "AR", "CA", "CO", "CT", "DE", "FL", "GA", "HI", "ID", "IL", "IN", "IA", "KS", "KY",
    "LA", "ME", "MD", "MA", "MI", "MN", "MS", "MO", "MT", "NE", "NV", "NH", "NJ", "NM", "NY", "NC", "ND",
    "OH", "OK", "OR", "PA", "RI", "SC", "SD", "TN", "TX", "UT", "VT", "VA", "WA", "WV", "WI", "WY", "NB",
    "NS", "QC", "ON", "MB", "SK", "AB", "BC", "NWT", "NF", "LB", "NU", "YT", "PEI", "DC"};
const int kNStates = static_cast<int>(sizeof(kStates) / sizeof(kStates[0]));

int indexIn(const char* table, char c) {
    if (!c) return -1;
    const char* p = std::strchr(table, c);
    return p ? static_cast<int>(p - table) : -1;
}

bool finite(double v) { return v == v && v - v == 0; }
bool isDigit(char c) { return c >= '0' && c <= '9'; }
bool isLetter(char c) { return c >= 'A' && c <= 'Z'; }
bool allDigits(const std::string& s) {
    if (s.empty()) return false;
    for (std::size_t i = 0; i < s.size(); ++i) if (!isDigit(s[i])) return false;
    return true;
}
bool allLetters(const std::string& s) {
    if (s.empty()) return false;
    for (std::size_t i = 0; i < s.size(); ++i) if (!isLetter(s[i])) return false;
    return true;
}
bool endsWith(const std::string& s, const char* t) {
    const std::size_t n = std::strlen(t);
    return s.size() >= n && s.compare(s.size() - n, n, t) == 0;
}
bool bracketed(const std::string& s) { return s.size() >= 3 && s[0] == '<' && s[s.size() - 1] == '>'; }
std::string unbracket(const std::string& s) { return bracketed(s) ? s.substr(1, s.size() - 2) : s; }
std::string trim(const std::string& s) {
    std::size_t a = 0, b = s.size();
    while (a < b && s[a] == ' ') ++a;
    while (b > a && s[b - 1] == ' ') --b;
    return s.substr(a, b - a);
}
// a report as FT8 shows it: "+05", "-12"
std::string reportText(int v) {
    char b[16];
    std::snprintf(b, sizeof b, "%c%02d", v >= 0 ? '+' : '-', v >= 0 ? v : -v);
    return b;
}
std::vector<std::string> words(const std::string& s) {
    std::vector<std::string> w;
    std::size_t a = 0;
    while (a < s.size()) {
        std::size_t b = s.find(' ', a);
        if (b == std::string::npos) b = s.size();
        if (b > a) w.push_back(s.substr(a, b - a));
        a = b + 1;
    }
    return w;
}
// the JS ftxInfo's isCall(): looks like a callsign (letters and a digit, 3+ characters)
bool callLike(const std::string& t) {
    bool d = false, l = false;
    for (std::size_t i = 0; i < t.size(); ++i) {
        if (isDigit(t[i])) d = true;
        else if (isLetter(t[i])) l = true;
        else if (t[i] != '/') return false;
    }
    return d && l && t.size() >= 3;
}

// ---- bit fields of a 77-bit payload (MSB first)
int getBit(const u8* a, int i) { return (a[i >> 3] >> (7 - (i & 7))) & 1; }
void setBit(u8* a, int i, int v) {
    if (v) a[i >> 3] = static_cast<u8>(a[i >> 3] | (0x80 >> (i & 7)));
    else a[i >> 3] = static_cast<u8>(a[i >> 3] & ~(0x80 >> (i & 7)));
}
unsigned long long getBits(const u8* a, int start, int n) {
    unsigned long long v = 0;
    for (int i = 0; i < n; ++i) v = (v << 1) | static_cast<unsigned long long>(getBit(a, start + i));
    return v;
}
void setBits(u8* a, int start, int n, unsigned long long v) {
    for (int i = 0; i < n; ++i) setBit(a, start + i, static_cast<int>((v >> (n - 1 - i)) & 1));
}

// ---- 71-bit numbers (free text, telemetry): 9 bytes, big-endian (the top bit always 0)
struct Big71 {
    u8 b[9];
    Big71() { std::memset(b, 0, sizeof b); }
    void mulAdd(unsigned mul, unsigned add) {
        unsigned carry = add;
        for (int i = 8; i >= 0; --i) {
            const unsigned t = b[i] * mul + carry;
            b[i] = static_cast<u8>(t & 0xff);
            carry = t >> 8;
        }
    }
    unsigned divMod(unsigned d) {
        unsigned rem = 0;
        for (int i = 0; i < 9; ++i) {
            const unsigned t = rem * 256 + b[i];
            b[i] = static_cast<u8>(t / d);
            rem = t % d;
        }
        return rem;
    }
    bool fits71() const { return (b[0] & 0x80) == 0; }
    void toPayload(u8* a) const {   // payload bits 0..70 = the number's bits 70..0
        for (int i = 0; i < 71; ++i) {
            const int j = 70 - i;
            setBit(a, i, (b[8 - j / 8] >> (j % 8)) & 1);
        }
    }
    void fromPayload(const u8* a) {
        std::memset(b, 0, sizeof b);
        for (int i = 0; i < 71; ++i) {
            const int j = 70 - i;
            if (getBit(a, i)) b[8 - j / 8] = static_cast<u8>(b[8 - j / 8] | (1 << (j % 8)));
        }
    }
};

// ---------------------------------------------------------------- packing
// port of packBasecall(): a standard callsign (2 or 3 characters before the digit... as WSJT-X) -> n, or -1
long packBasecall(const std::string& c) {
    std::string c6;
    if (c.compare(0, 4, "3DA0") == 0 && c.size() > 4 && c.size() <= 7) c6 = "3D0" + c.substr(4);
    else if (c.size() >= 3 && c.compare(0, 2, "3X") == 0 && isLetter(c[2]) && c.size() <= 7) c6 = "Q" + c.substr(2);
    else if (c.size() >= 3 && isDigit(c[2]) && c.size() <= 6) c6 = c;
    else if (c.size() >= 2 && isDigit(c[1]) && c.size() <= 5) c6 = " " + c;
    if (c6.empty()) return -1;
    while (c6.size() < 6) c6 += ' ';
    const int i0 = indexIn(T_AN_SP, c6[0]), i1 = indexIn(T_AN, c6[1]), i2 = indexIn(T_NUM, c6[2]);
    const int i3 = indexIn(T_LET_SP, c6[3]), i4 = indexIn(T_LET_SP, c6[4]), i5 = indexIn(T_LET_SP, c6[5]);
    if (i0 < 0 || i1 < 0 || i2 < 0 || i3 < 0 || i4 < 0 || i5 < 0) return -1;
    return ((((static_cast<long>(i0) * 36 + i1) * 10 + i2) * 27 + i3) * 27 + i4) * 27 + i5;
}

bool isStandardCall(const std::string& c) {
    std::string base = c;
    if (endsWith(c, "/P") || endsWith(c, "/R")) base = c.substr(0, c.size() - 2);
    return packBasecall(base) >= 0;
}

// port of pack28(): a call or token -> n28 and the /P-/R flag.  `save` gets the call to remember (the JS saves
// at once; here only once the whole message packs).  (Addition to the JS: "<CALL>" -> its 22-bit hash.)
bool pack28(const std::string& c, long& n28, int& ip, std::string* save) {
    ip = 0;
    if (c == "DE") { n28 = 0; return true; }
    if (c == "QRZ") { n28 = 1; return true; }
    if (c == "CQ") { n28 = 2; return true; }
    if (c.compare(0, 3, "CQ ") == 0) {
        const std::string m = c.substr(3);
        if (m.size() == 3 && allDigits(m)) { n28 = 3 + std::atol(m.c_str()); return true; }
        if (m.size() >= 1 && m.size() <= 4 && allLetters(m)) {
            long v = 0;
            for (std::size_t i = 0; i < m.size(); ++i) v = v * 27 + (m[i] - 'A' + 1);
            n28 = 3 + 1000 + v;
            return true;
        }
        return false;
    }
    if (bracketed(c)) {   // (Addition to the JS)
        unsigned h;
        const std::string call = unbracket(c);
        if (!CallHashTable::hash22(call, h)) return false;
        n28 = NTOKENS + static_cast<long>(h);
        if (save) *save = call;
        return true;
    }
    std::string base = c;
    if (endsWith(c, "/P") || endsWith(c, "/R")) { ip = 1; base = c.substr(0, c.size() - 2); }
    const long n = packBasecall(base);
    if (n < 0) return false;
    n28 = NTOKENS + MAX22 + n;
    if (save) *save = c;
    return true;
}

// port of packGrid(): the third part of a standard message -> igrid4 and the R flag
bool packGrid(const std::string& g, long& v, int& ir) {
    ir = 0;
    if (g.empty()) { v = MAXGRID4 + 1; return true; }
    if (g == "RRR") { v = MAXGRID4 + 2; return true; }
    if (g == "RR73") { v = MAXGRID4 + 3; return true; }
    if (g == "73") { v = MAXGRID4 + 4; return true; }
    std::string s = g;
    if (s.compare(0, 2, "R ") == 0) { ir = 1; s = s.substr(2); }
    if (s.size() == 4 && s[0] >= 'A' && s[0] <= 'R' && s[1] >= 'A' && s[1] <= 'R' && isDigit(s[2]) && isDigit(s[3])) {
        v = ((s[0] - 'A') * 18 + (s[1] - 'A')) * 100 + (s[2] - '0') * 10 + (s[3] - '0');
        return true;
    }
    if (ir) return false;
    s = g;
    if (!s.empty() && s[0] == 'R') { ir = 1; s = s.substr(1); }
    if (s.size() == 3 && (s[0] == '+' || s[0] == '-') && isDigit(s[1]) && isDigit(s[2])) {
        const int r = (s[0] == '-' ? -1 : 1) * ((s[1] - '0') * 10 + (s[2] - '0'));
        if (r < -30) return false;   // (Addition to the JS: below -30 the value would read back as a locator)
        v = MAXGRID4 + 35 + r;
        return true;
    }
    return false;
}

bool packStandard(const std::vector<std::string>& w, u8* a, std::vector<std::string>& saves) {
    std::string p0, p1, p2;
    long n0 = 0, n1 = 0;
    int ip0 = 0, ip1 = 0;
    std::string s0, s1;
    bool ok = false;
    // "CQ DX K1ABC FN42": the CQ's modifier is part of the first field
    if (w.size() >= 3 && w[0] == "CQ" &&
        ((w[1].size() == 3 && allDigits(w[1])) || (w[1].size() >= 1 && w[1].size() <= 4 && allLetters(w[1]))) &&
        pack28(w[2], n1, ip1, &s1)) {
        p0 = "CQ " + w[1];
        p1 = w[2];
        for (std::size_t i = 3; i < w.size(); ++i) p2 += (i > 3 ? " " : "") + w[i];
        ok = true;
    } else if (w.size() >= 2) {
        p0 = w[0];
        p1 = w[1];
        for (std::size_t i = 2; i < w.size(); ++i) p2 += (i > 2 ? " " : "") + w[i];
        ok = true;
    }
    if (!ok) return false;
    long g = 0;
    int ir = 0;
    if (!pack28(p0, n0, ip0, &s0) || !pack28(p1, n1, ip1, &s1) || !packGrid(p2, g, ir)) return false;
    const bool p = endsWith(p0, "/P") || endsWith(p1, "/P"), r = endsWith(p0, "/R") || endsWith(p1, "/R");
    if (p && r) return false;
    const int i3 = p ? 2 : 1;   // (/P calls: type 2; /R: type 1)
    std::memset(a, 0, kPayloadBytes);
    setBits(a, 0, 28, static_cast<unsigned long long>(n0));
    setBit(a, 28, ip0);
    setBits(a, 29, 28, static_cast<unsigned long long>(n1));
    setBit(a, 57, ip1);
    setBit(a, 58, ir);
    setBits(a, 59, 15, static_cast<unsigned long long>(g));
    setBits(a, 74, 3, static_cast<unsigned long long>(i3));
    if (!s0.empty()) saves.push_back(s0);
    if (!s1.empty()) saves.push_back(s1);
    return true;
}

// (Addition to the JS) 0.5: "telemetry" and up to 18 hex digits (71 bits)
bool packTelemetry(const std::vector<std::string>& w, u8* a) {
    if (w.size() != 2 || w[0] != "TELEMETRY" || w[1].empty() || w[1].size() > 18) return false;
    Big71 v;
    for (std::size_t i = 0; i < w[1].size(); ++i) {
        const char c = w[1][i];
        const int d = isDigit(c) ? c - '0' : (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
        if (d < 0) return false;
        v.mulAdd(16, static_cast<unsigned>(d));
    }
    if (!v.fits71()) return false;
    std::memset(a, 0, kPayloadBytes);
    v.toPayload(a);
    setBits(a, 71, 3, 5);
    setBits(a, 74, 3, 0);
    return true;
}

// (Addition to the JS) 0.1 DXpedition: "K1ABC RR73; W9XYZ <KH1/KH7Z> -08"
bool packDxpedition(const std::vector<std::string>& w, u8* a, std::vector<std::string>& saves) {
    if (w.size() != 5 || w[1] != "RR73;" || !bracketed(w[3])) return false;
    long n0, n1;
    int ip0, ip1;
    std::string s0, s1;
    if (!pack28(w[0], n0, ip0, &s0) || !pack28(w[2], n1, ip1, &s1) || ip0 || ip1) return false;
    if (n0 < NTOKENS || n1 < NTOKENS) return false;
    unsigned h;
    const std::string dx = unbracket(w[3]);
    if (!CallHashTable::hash22(dx, h)) return false;
    const std::string& r = w[4];
    if (r.size() != 3 || (r[0] != '+' && r[0] != '-') || !isDigit(r[1]) || !isDigit(r[2])) return false;
    const int rpt = (r[0] == '-' ? -1 : 1) * std::atoi(r.c_str() + 1);
    if (rpt < -30 || rpt > 32) return false;
    std::memset(a, 0, kPayloadBytes);
    setBits(a, 0, 28, static_cast<unsigned long long>(n0));
    setBits(a, 28, 28, static_cast<unsigned long long>(n1));
    setBits(a, 56, 10, h >> 12);
    setBits(a, 66, 5, static_cast<unsigned long long>((rpt + 30) / 2));
    setBits(a, 71, 3, 1);
    setBits(a, 74, 3, 0);
    if (!s0.empty()) saves.push_back(s0);
    if (!s1.empty()) saves.push_back(s1);
    saves.push_back(dx);
    return true;
}

// (Addition to the JS) 3, ARRL RTTY Roundup: "[TU;] W9XYZ K1ABC [R] 579 MA|0013"
bool packRttyRu(const std::vector<std::string>& w0, u8* a, std::vector<std::string>& saves) {
    std::vector<std::string> w = w0;
    int tu = 0, ir = 0;
    if (!w.empty() && w[0] == "TU;") { tu = 1; w.erase(w.begin()); }
    if (w.size() == 5 && w[2] == "R") { ir = 1; w.erase(w.begin() + 2); }
    if (w.size() != 4) return false;
    long n0, n1;
    int ip0, ip1;
    std::string s0, s1;
    if (!pack28(w[0], n0, ip0, &s0) || !pack28(w[1], n1, ip1, &s1) || ip0 || ip1) return false;
    if (n0 < NTOKENS || n1 < NTOKENS) return false;
    const std::string& rst = w[2];
    if (rst.size() != 3 || rst[0] != '5' || rst[2] != '9' || rst[1] < '2' || rst[1] > '9') return false;
    long ex = -1;
    if (allDigits(w[3]) && w[3].size() <= 4) {
        ex = std::atol(w[3].c_str());
        if (ex < 1 || ex > 7999) return false;
    } else {
        for (int i = 0; i < kNStates; ++i) if (w[3] == kStates[i]) ex = 8001 + i;
        if (ex < 0) return false;
    }
    std::memset(a, 0, kPayloadBytes);
    setBit(a, 0, tu);
    setBits(a, 1, 28, static_cast<unsigned long long>(n0));
    setBits(a, 29, 28, static_cast<unsigned long long>(n1));
    setBit(a, 57, ir);
    setBits(a, 58, 3, static_cast<unsigned long long>(rst[1] - '2'));
    setBits(a, 61, 13, static_cast<unsigned long long>(ex));
    setBits(a, 74, 3, 3);
    if (!s0.empty()) saves.push_back(s0);
    if (!s1.empty()) saves.push_back(s1);
    return true;
}

// (Addition to the JS) 0.3 / 0.4, ARRL Field Day: "WA9XYZ KA1ABC [R] 16A EMA"
bool packFieldDay(const std::vector<std::string>& w0, u8* a, std::vector<std::string>& saves) {
    std::vector<std::string> w = w0;
    int ir = 0;
    if (w.size() == 5 && w[2] == "R") { ir = 1; w.erase(w.begin() + 2); }
    if (w.size() != 4) return false;
    long n0, n1;
    int ip0, ip1;
    std::string s0, s1;
    if (!pack28(w[0], n0, ip0, &s0) || !pack28(w[1], n1, ip1, &s1) || ip0 || ip1) return false;
    if (n0 < NTOKENS || n1 < NTOKENS) return false;
    const std::string& c = w[2];
    if (c.size() < 2 || c.size() > 3 || !allDigits(c.substr(0, c.size() - 1))) return false;
    const char cls = c[c.size() - 1];
    if (cls < 'A' || cls > 'H') return false;
    const int ntx = std::atoi(c.substr(0, c.size() - 1).c_str());
    if (ntx < 1 || ntx > 32) return false;
    int sec = -1;
    for (int i = 0; i < kNSections; ++i) if (w[3] == kSections[i]) sec = i + 1;
    if (sec < 0) return false;
    std::memset(a, 0, kPayloadBytes);
    setBits(a, 0, 28, static_cast<unsigned long long>(n0));
    setBits(a, 28, 28, static_cast<unsigned long long>(n1));
    setBit(a, 56, ir);
    setBits(a, 57, 4, static_cast<unsigned long long>(ntx <= 16 ? ntx - 1 : ntx - 17));
    setBits(a, 61, 3, static_cast<unsigned long long>(cls - 'A'));
    setBits(a, 64, 7, static_cast<unsigned long long>(sec));
    setBits(a, 71, 3, ntx <= 16 ? 3 : 4);
    setBits(a, 74, 3, 0);
    if (!s0.empty()) saves.push_back(s0);
    if (!s1.empty()) saves.push_back(s1);
    return true;
}

bool isGrid6(const std::string& g) {
    return g.size() == 6 && g[0] >= 'A' && g[0] <= 'R' && g[1] >= 'A' && g[1] <= 'R' && isDigit(g[2]) &&
           isDigit(g[3]) && g[4] >= 'A' && g[4] <= 'X' && g[5] >= 'A' && g[5] <= 'X';
}

// (Addition to the JS) 5, EU VHF contest: "<PA3XYZ> <G4ABC/P> [R] 590003 IO91NP"
bool packEuVhf(const std::vector<std::string>& w0, u8* a, std::vector<std::string>& saves) {
    std::vector<std::string> w = w0;
    int ir = 0;
    if (w.size() == 5 && w[2] == "R") { ir = 1; w.erase(w.begin() + 2); }
    if (w.size() != 4 || !bracketed(w[0]) || !bracketed(w[1])) return false;
    unsigned h0, h1;
    const std::string c0 = unbracket(w[0]), c1 = unbracket(w[1]);
    if (!CallHashTable::hash22(c0, h0) || !CallHashTable::hash22(c1, h1)) return false;
    const std::string& ex = w[2];
    if (ex.size() != 6 || !allDigits(ex) || ex[0] != '5' || ex[1] < '2') return false;
    const int serial = std::atoi(ex.substr(2).c_str());
    if (serial > 2047 || !isGrid6(w[3])) return false;
    const std::string& g = w[3];
    const long g25 = (((((g[0] - 'A') * 18L + (g[1] - 'A')) * 10 + (g[2] - '0')) * 10 + (g[3] - '0')) * 24 +
                      (g[4] - 'A')) * 24 + (g[5] - 'A');
    std::memset(a, 0, kPayloadBytes);
    setBits(a, 0, 12, h0 >> 10);
    setBits(a, 12, 22, h1);
    setBit(a, 34, ir);
    setBits(a, 35, 3, static_cast<unsigned long long>(ex[1] - '2'));
    setBits(a, 38, 11, static_cast<unsigned long long>(serial));
    setBits(a, 49, 25, static_cast<unsigned long long>(g25));
    setBits(a, 74, 3, 5);
    saves.push_back(c0);
    saves.push_back(c1);
    return true;
}

bool pack58(const std::string& call, unsigned long long& n58) {
    if (call.size() < 3 || call.size() > 11) return false;
    n58 = 0;
    for (std::size_t i = 0; i < call.size(); ++i) {
        const int j = indexIn(T_AN_SP_SL, call[i]);
        if (j < 0) return false;
        n58 = n58 * 38 + static_cast<unsigned long long>(j);
    }
    return true;
}

// (Addition to the JS) 4, nonstandard calls: "CQ PJ4/K1ABC", "<W9XYZ> PJ4/K1ABC RR73", "PJ4/K1ABC <W9XYZ> 73"
// (the call in <> is sent as a 12-bit hash; without brackets, the standard one is)
bool packNonStandard(const std::vector<std::string>& w, u8* a, std::vector<std::string>& saves) {
    int icq = 0, iflip = 0, nrpt = 0;
    unsigned n12 = 0;
    std::string full;
    if (w.size() == 2 && w[0] == "CQ") {
        icq = 1;
        full = w[1];
        if (bracketed(full) || !callLike(full)) return false;
    } else if (w.size() == 2 || w.size() == 3) {
        if (w.size() == 3) {
            if (w[2] == "RRR") nrpt = 1;
            else if (w[2] == "RR73") nrpt = 2;
            else if (w[2] == "73") nrpt = 3;
            else return false;
        }
        const bool b0 = bracketed(w[0]), b1 = bracketed(w[1]);
        if (b0 && b1) return false;
        std::string hashed;
        if (b0) { hashed = unbracket(w[0]); full = w[1]; iflip = 0; }
        else if (b1) { hashed = unbracket(w[1]); full = w[0]; iflip = 1; }
        else {
            const bool st0 = isStandardCall(w[0]), st1 = isStandardCall(w[1]);
            if (st0 == st1) return false;   // (both standard: a standard message; neither: not a pair of calls)
            if (st0) { hashed = w[0]; full = w[1]; iflip = 0; }
            else { hashed = w[1]; full = w[0]; iflip = 1; }
        }
        if (!callLike(full) || !callLike(hashed)) return false;
        unsigned h;
        if (!CallHashTable::hash22(hashed, h)) return false;
        n12 = h >> 10;
        saves.push_back(hashed);
    } else {
        return false;
    }
    unsigned long long n58;
    if (!pack58(full, n58)) return false;
    std::memset(a, 0, kPayloadBytes);
    setBits(a, 0, 12, n12);
    setBits(a, 12, 58, n58);
    setBit(a, 70, iflip);
    setBits(a, 71, 2, static_cast<unsigned long long>(nrpt));
    setBit(a, 73, icq);
    setBits(a, 74, 3, 4);
    saves.push_back(full);
    return true;
}

bool packFree(const std::string& text, u8* a) {
    if (text.empty() || text.size() > 13) return false;
    Big71 v;
    std::string t = text;
    while (t.size() < 13) t += ' ';
    for (std::size_t i = 0; i < 13; ++i) {
        const int j = indexIn(T_FULL, t[i]);
        if (j < 0) return false;
        v.mulAdd(42, static_cast<unsigned>(j));
    }
    std::memset(a, 0, kPayloadBytes);
    v.toPayload(a);
    return true;   // (n3 = 0, i3 = 0)
}

// ---------------------------------------------------------------- unpacking
std::string lookupCall(const CallHashTable* ht, int bits, unsigned h) {
    std::string c;
    if (ht && ht->lookup(bits, h, c)) return "<" + c + ">";
    return "<...>";
}

// port of unpack28(): false for an invalid field
bool unpack28(long n, int ip, int i3, CallHashTable* ht, std::string& out) {
    if (n < NTOKENS) {
        if (n == 0) { out = "DE"; return true; }
        if (n == 1) { out = "QRZ"; return true; }
        if (n == 2) { out = "CQ"; return true; }
        if (n <= 1002) { char b[16]; std::snprintf(b, sizeof b, "CQ %03ld", n - 3); out = b; return true; }
        if (n <= 532443) {
            long v = n - 1003;
            std::string s;
            for (int i = 0; i < 4; ++i) { s = T_LET_SP[v % 27] + s; v /= 27; }
            out = "CQ " + trim(s);
            return true;
        }
        return false;
    }
    n -= NTOKENS;
    if (n < MAX22) { out = lookupCall(ht, 22, static_cast<unsigned>(n)); return true; }
    n -= MAX22;
    std::string c(6, ' ');
    c[5] = T_LET_SP[n % 27]; n /= 27;
    c[4] = T_LET_SP[n % 27]; n /= 27;
    c[3] = T_LET_SP[n % 27]; n /= 27;
    c[2] = T_NUM[n % 10]; n /= 10;
    c[1] = T_AN[n % 36]; n /= 36;
    if (n >= 37) return false;
    c[0] = T_AN_SP[n % 37];
    std::string r;
    if (c.compare(0, 3, "3D0") == 0 && c[3] != ' ') r = "3DA0" + trim(c.substr(3));
    else if (c[0] == 'Q' && isLetter(c[1])) r = "3X" + trim(c.substr(1));
    else r = trim(c);
    if (r.size() < 3 || r.find(' ') != std::string::npos) return false;
    if (ip) {
        if (i3 == 1) r += "/R";
        else if (i3 == 2) r += "/P";
        else return false;
    }
    if (ht) ht->save(r);
    out = r;
    return true;
}

// port of unpackGrid()
std::string unpackGrid(long g, int ir) {
    if (g <= MAXGRID4) {
        long n = g;
        const int d4 = static_cast<int>(n % 10); n /= 10;
        const int d3 = static_cast<int>(n % 10); n /= 10;
        const int l2 = static_cast<int>(n % 18); n /= 18;
        std::string s = ir ? "R " : "";
        s += static_cast<char>('A' + n % 18);
        s += static_cast<char>('A' + l2);
        s += static_cast<char>('0' + d3);
        s += static_cast<char>('0' + d4);
        return s;
    }
    const long r = g - MAXGRID4;
    if (r == 1) return "";
    if (r == 2) return "RRR";
    if (r == 3) return "RR73";
    if (r == 4) return "73";
    return (ir ? "R" : "") + reportText(static_cast<int>(r - 35));
}

std::string joinNonEmpty(const std::string& a, const std::string& b, const std::string& c) {
    std::string s = a;
    if (!b.empty()) s += (s.empty() ? "" : " ") + b;
    if (!c.empty()) s += (s.empty() ? "" : " ") + c;
    return s;
}

}  // namespace

// ---------------------------------------------------------------- CallHashTable
CallHashTable::CallHashTable(std::size_t capacity) : cap_(capacity < 16 ? 16 : capacity) {}

bool CallHashTable::hash22(const std::string& call, unsigned& h22) {
    unsigned long long n58 = 0;
    std::size_t i = 0;
    if (call.empty()) return false;
    for (; i < call.size() && i < 11; ++i) {
        const int j = indexIn(T_AN_SP_SL, call[i]);
        if (j < 0) return false;
        n58 = n58 * 38 + static_cast<unsigned long long>(j);
    }
    if (call.size() > 11) return false;
    for (; i < 11; ++i) n58 *= 38;
    const unsigned long long p = 47055833459ULL * n58;   // (mod 2^64, as WSJT-X)
    h22 = static_cast<unsigned>((p >> 42) & 0x3fffff);
    return true;
}

void CallHashTable::save(const std::string& call) {
    unsigned h;
    if (!hash22(call, h)) return;
    h22_[h] = call;
    h12_[h >> 10] = call;
    h10_[h >> 12] = call;
    if (known_.insert(call).second) order_.push_back(call);
    // (Addition to the JS: bounded; the oldest call's entries go, unless a newer call has taken them)
    while (order_.size() > cap_) {
        const std::string old = order_.front();
        order_.pop_front();
        known_.erase(old);
        unsigned ho;
        if (!hash22(old, ho)) continue;
        std::map<unsigned, std::string>::iterator it;
        if ((it = h22_.find(ho)) != h22_.end() && it->second == old) h22_.erase(it);
        if ((it = h12_.find(ho >> 10)) != h12_.end() && it->second == old) h12_.erase(it);
        if ((it = h10_.find(ho >> 12)) != h10_.end() && it->second == old) h10_.erase(it);
    }
}

bool CallHashTable::lookup(int bits, unsigned hash, std::string& call) const {
    const std::map<unsigned, std::string>& m = bits == 22 ? h22_ : bits == 12 ? h12_ : h10_;
    std::map<unsigned, std::string>::const_iterator it = m.find(hash);
    if (it == m.end()) return false;
    call = it->second;
    return true;
}

void CallHashTable::clear() {
    h22_.clear();
    h12_.clear();
    h10_.clear();
    order_.clear();
    known_.clear();
}

// ---------------------------------------------------------------- messages
const char* messageTypeName(MessageType t) {
    switch (t) {
        case MessageType::FreeText: return "free text";
        case MessageType::Dxpedition: return "DXpedition";
        case MessageType::FieldDay: return "ARRL Field Day";
        case MessageType::Telemetry: return "telemetry";
        case MessageType::Standard: return "standard";
        case MessageType::RttyRoundup: return "ARRL RTTY Roundup";
        case MessageType::NonStandard: return "nonstandard calls";
        case MessageType::EuVhf: return "EU VHF contest";
        default: return "unknown";
    }
}

std::string normalizeMessage(const std::string& text) {
    std::string s;
    bool space = false;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(text[i]);
        if (std::isspace(c)) { space = true; continue; }
        if (space && !s.empty()) s += ' ';
        space = false;
        s += static_cast<char>(std::toupper(c));
    }
    return s;
}

int messageI3(const u8 a77[kPayloadBytes]) { return static_cast<int>(getBits(a77, 74, 3)); }
int messageN3(const u8 a77[kPayloadBytes]) { return static_cast<int>(getBits(a77, 71, 3)); }

bool pack77(const std::string& text0, u8 a77[kPayloadBytes], CallHashTable* ht, MessageType* type) {
    const std::string text = normalizeMessage(text0);
    const std::vector<std::string> w = words(text);
    std::vector<std::string> saves;
    MessageType t = MessageType::Unknown;
    u8 a[kPayloadBytes];
    std::memset(a, 0, sizeof a);
    bool ok = false;
    if (packTelemetry(w, a)) { ok = true; t = MessageType::Telemetry; }
    else if (packDxpedition(w, a, saves)) { ok = true; t = MessageType::Dxpedition; }
    else if ((saves.clear(), packStandard(w, a, saves))) { ok = true; t = MessageType::Standard; }
    else if ((saves.clear(), packRttyRu(w, a, saves))) { ok = true; t = MessageType::RttyRoundup; }
    else if ((saves.clear(), packFieldDay(w, a, saves))) { ok = true; t = MessageType::FieldDay; }
    else if ((saves.clear(), packEuVhf(w, a, saves))) { ok = true; t = MessageType::EuVhf; }
    else if ((saves.clear(), packNonStandard(w, a, saves))) { ok = true; t = MessageType::NonStandard; }
    else if ((saves.clear(), packFree(text, a))) { ok = true; t = MessageType::FreeText; }
    if (!ok) return false;
    if (ht) for (std::size_t i = 0; i < saves.size(); ++i) ht->save(saves[i]);
    std::memcpy(a77, a, kPayloadBytes);
    if (type) *type = t;
    return true;
}

bool unpack77(const u8 b[kPayloadBytes], std::string& text, CallHashTable* ht, MessageType* type) {
    const int i3 = messageI3(b);
    MessageType t = MessageType::Unknown;
    char tmp[64];
    if (i3 == 1 || i3 == 2) {
        std::string a, c;
        if (!unpack28(static_cast<long>(getBits(b, 0, 28)), getBit(b, 28), i3, ht, a)) return false;
        if (!unpack28(static_cast<long>(getBits(b, 29, 28)), getBit(b, 57), i3, ht, c)) return false;
        const std::string g = unpackGrid(static_cast<long>(getBits(b, 59, 15)), getBit(b, 58));
        text = joinNonEmpty(a, c, g);
        t = MessageType::Standard;
    } else if (i3 == 0) {
        const int n3 = messageN3(b);
        if (n3 == 0) {
            Big71 v;
            v.fromPayload(b);
            std::string s(13, ' ');
            for (int i = 12; i >= 0; --i) s[static_cast<std::size_t>(i)] = T_FULL[v.divMod(42)];
            s = trim(s);
            if (s.empty()) return false;
            text = s;
            t = MessageType::FreeText;
        } else if (n3 == 5) {
            Big71 v;
            v.fromPayload(b);
            std::string s = "telemetry ";
            for (int i = 0; i < 9; ++i) { std::snprintf(tmp, sizeof tmp, "%02X", v.b[i]); s += tmp; }
            text = s;
            t = MessageType::Telemetry;
        } else if (n3 == 1) {   // (Addition to the JS: the JS shows "(message type 0.1)")
            std::string c1, c2;
            if (!unpack28(static_cast<long>(getBits(b, 0, 28)), 0, 0, ht, c1)) return false;
            if (!unpack28(static_cast<long>(getBits(b, 28, 28)), 0, 0, ht, c2)) return false;
            const std::string dx = lookupCall(ht, 10, static_cast<unsigned>(getBits(b, 56, 10)));
            text = c1 + " RR73; " + c2 + " " + dx + " " + reportText(2 * static_cast<int>(getBits(b, 66, 5)) - 30);
            t = MessageType::Dxpedition;
        } else if (n3 == 3 || n3 == 4) {   // (Addition to the JS)
            std::string c1, c2;
            if (!unpack28(static_cast<long>(getBits(b, 0, 28)), 0, 0, ht, c1)) return false;
            if (!unpack28(static_cast<long>(getBits(b, 28, 28)), 0, 0, ht, c2)) return false;
            const int ir = getBit(b, 56), ntx = static_cast<int>(getBits(b, 57, 4)) + (n3 == 3 ? 1 : 17);
            const int cls = static_cast<int>(getBits(b, 61, 3)), sec = static_cast<int>(getBits(b, 64, 7));
            if (sec < 1 || sec > kNSections) return false;
            std::snprintf(tmp, sizeof tmp, "%d%c", ntx, 'A' + cls);
            text = c1 + " " + c2 + (ir ? " R " : " ") + tmp + " " + kSections[sec - 1];
            t = MessageType::FieldDay;
        } else {
            std::snprintf(tmp, sizeof tmp, "(message type 0.%d)", n3);
            text = tmp;
        }
    } else if (i3 == 4) {
        const unsigned n12 = static_cast<unsigned>(getBits(b, 0, 12));
        unsigned long long n58 = getBits(b, 12, 58);
        const int iflip = getBit(b, 70), nrpt = static_cast<int>(getBits(b, 71, 2)), icq = getBit(b, 73);
        std::string s(11, ' ');
        for (int i = 10; i >= 0; --i) { s[static_cast<std::size_t>(i)] = T_AN_SP_SL[n58 % 38]; n58 /= 38; }
        const std::string full = trim(s);
        if (full.size() >= 3 && ht) ht->save(full);
        if (icq) text = "CQ " + full;
        else {
            const std::string other = lookupCall(ht, 12, n12);
            static const char* const kR[4] = {"", "RRR", "RR73", "73"};
            text = joinNonEmpty(iflip ? full : other, iflip ? other : full, kR[nrpt]);
        }
        t = MessageType::NonStandard;
    } else if (i3 == 3) {   // (Addition to the JS)
        std::string c1, c2;
        if (!unpack28(static_cast<long>(getBits(b, 1, 28)), 0, 3, ht, c1)) return false;
        if (!unpack28(static_cast<long>(getBits(b, 29, 28)), 0, 3, ht, c2)) return false;
        const int tu = getBit(b, 0), ir = getBit(b, 57), r3 = static_cast<int>(getBits(b, 58, 3));
        const int ex = static_cast<int>(getBits(b, 61, 13));
        std::string exch;
        if (ex > 8000) {
            if (ex - 8000 > kNStates) return false;
            exch = kStates[ex - 8001];
        } else {
            std::snprintf(tmp, sizeof tmp, "%04d", ex);
            exch = tmp;
        }
        std::snprintf(tmp, sizeof tmp, "5%c9", '2' + r3);
        text = std::string(tu ? "TU; " : "") + c1 + " " + c2 + (ir ? " R " : " ") + tmp + " " + exch;
        t = MessageType::RttyRoundup;
    } else if (i3 == 5) {   // (Addition to the JS)
        const std::string c1 = lookupCall(ht, 12, static_cast<unsigned>(getBits(b, 0, 12)));
        const std::string c2 = lookupCall(ht, 22, static_cast<unsigned>(getBits(b, 12, 22)));
        const int ir = getBit(b, 34), r3 = static_cast<int>(getBits(b, 35, 3)), serial = static_cast<int>(getBits(b, 38, 11));
        long g = static_cast<long>(getBits(b, 49, 25));
        char g6[7];
        g6[6] = 0;
        g6[5] = static_cast<char>('A' + g % 24); g /= 24;
        g6[4] = static_cast<char>('A' + g % 24); g /= 24;
        g6[3] = static_cast<char>('0' + g % 10); g /= 10;
        g6[2] = static_cast<char>('0' + g % 10); g /= 10;
        g6[1] = static_cast<char>('A' + g % 18); g /= 18;
        if (g >= 18) return false;
        g6[0] = static_cast<char>('A' + g);
        std::snprintf(tmp, sizeof tmp, "%d%04d", 52 + r3, serial);
        text = c1 + " " + c2 + (ir ? " R " : " ") + tmp + " " + g6;
        t = MessageType::EuVhf;
    } else {
        std::snprintf(tmp, sizeof tmp, "(message type %d)", i3);
        text = tmp;
    }
    if (type) *type = t;
    return true;
}

// ---------------------------------------------------------------- CRC, LDPC
unsigned crc14(const u8* bytes, int nbits) {
    unsigned r = 0;
    for (int i = 0; i < nbits; ++i) {
        r ^= static_cast<unsigned>(getBit(bytes, i)) << 13;
        r = (r & 0x2000) ? ((r << 1) ^ 0x2757) & 0x3fff : (r << 1) & 0x3fff;
    }
    return r;
}

void addCrc(const u8 a77[kPayloadBytes], u8 a91[12]) {
    std::memset(a91, 0, 12);
    for (int i = 0; i < 77; ++i) setBit(a91, i, getBit(a77, i));
    const unsigned c = crc14(a91, 82);   // (the 77 bits and 5 zeros)
    setBits(a91, 77, 14, c);
}

bool checkCrc(const u8 a91[12]) {
    u8 t[12];
    std::memset(t, 0, sizeof t);
    for (int i = 0; i < 77; ++i) setBit(t, i, getBit(a91, i));
    return crc14(t, 82) == static_cast<unsigned>(getBits(a91, 77, 14));
}

void ldpcEncode(const u8 a91[12], u8 cw[kN]) {
    for (int j = 0; j < kK; ++j) cw[j] = static_cast<u8>(getBit(a91, j));
    for (int i = 0; i < kM; ++i) {
        int p = 0;
        for (int j = 0; j < kK; ++j)
            if (cw[j] && ((kGen[i][j >> 3] >> (7 - (j & 7))) & 1)) p ^= 1;
        cw[kK + i] = static_cast<u8>(p);
    }
}

int ldpcCheck(const u8 b[kN]) {
    int e = 0;
    for (int m = 0; m < kM; ++m) {
        int x = 0;
        for (int i = 0; i < kNumRows[m]; ++i) x ^= b[kNm[m][i] - 1];
        e += x;
    }
    return e;
}

bool bpDecode(const float llr[kN], int iters, u8 plain[kN]) {
    float tov[kN][3], toc[kM][7];
    std::memset(tov, 0, sizeof tov);
    std::memset(toc, 0, sizeof toc);
    for (int it = 0; it < iters; ++it) {
        int sum = 0;
        for (int n = 0; n < kN; ++n) {
            plain[n] = llr[n] + tov[n][0] + tov[n][1] + tov[n][2] > 0 ? 1 : 0;
            sum += plain[n];
        }
        if (!sum) return false;
        if (ldpcCheck(plain) == 0) return true;
        for (int m = 0; m < kM; ++m)
            for (int i = 0; i < kNumRows[m]; ++i) {
                const int n = kNm[m][i] - 1;
                float T = llr[n];
                for (int j = 0; j < 3; ++j) if (kMn[n][j] - 1 != m) T += tov[n][j];
                toc[m][i] = std::tanh(-T / 2);
            }
        for (int n = 0; n < kN; ++n)
            for (int j = 0; j < 3; ++j) {
                const int m = kMn[n][j] - 1;
                float P = 1;
                for (int i = 0; i < kNumRows[m]; ++i) if (kNm[m][i] - 1 != n) P *= toc[m][i];
                P = std::max(-0.9999999f, std::min(0.9999999f, P));
                tov[n][j] = -2 * std::atanh(P);
            }
    }
    return false;
}

void packBits(const u8* bits, int n, u8* bytes) {
    std::memset(bytes, 0, static_cast<std::size_t>((n + 7) / 8));
    for (int i = 0; i < n; ++i) setBit(bytes, i, bits[i]);
}

// ---------------------------------------------------------------- tones
void ft8TonesFromPayload(const u8 a77[kPayloadBytes], u8 out[kFt8Symbols]) {
    u8 a91[12], cw[kN];
    addCrc(a77, a91);
    ldpcEncode(a91, cw);
    for (int s = 0, d = 0; s < kFt8Symbols; ++s) {
        if (s < 7) out[s] = kFt8Costas[s];
        else if (s >= 36 && s < 43) out[s] = kFt8Costas[s - 36];
        else if (s >= 72) out[s] = kFt8Costas[s - 72];
        else { out[s] = kFt8Gray[cw[3 * d] * 4 + cw[3 * d + 1] * 2 + cw[3 * d + 2]]; ++d; }
    }
}

void ft4TonesFromPayload(const u8 a77[kPayloadBytes], u8 out[kFt4Symbols]) {
    u8 px[kPayloadBytes], a91[12], cw[kN];
    for (int i = 0; i < kPayloadBytes; ++i) px[i] = static_cast<u8>(a77[i] ^ kFt4Xor[i]);
    px[9] &= 0xF8;   // (only 77 bits)
    addCrc(px, a91);
    ldpcEncode(a91, cw);
    for (int s = 0, d = 0; s < kFt4Symbols; ++s) {
        if (s == 0 || s == 104) out[s] = 0;
        else if (s < 5) out[s] = kFt4Costas[0][s - 1];
        else if (s >= 34 && s < 38) out[s] = kFt4Costas[1][s - 34];
        else if (s >= 67 && s < 71) out[s] = kFt4Costas[2][s - 67];
        else if (s >= 100 && s < 104) out[s] = kFt4Costas[3][s - 100];
        else { out[s] = kFt4Gray[cw[2 * d] * 2 + cw[2 * d + 1]]; ++d; }
    }
}

bool ft8Tones(const std::string& text, u8 tones[kFt8Symbols], CallHashTable* ht) {
    u8 a[kPayloadBytes];
    if (!pack77(text, a, ht)) return false;
    ft8TonesFromPayload(a, tones);
    return true;
}

bool ft4Tones(const std::string& text, u8 tones[kFt4Symbols], CallHashTable* ht) {
    u8 a[kPayloadBytes];
    if (!pack77(text, a, ht)) return false;
    ft4TonesFromPayload(a, tones);
    return true;
}

// ---------------------------------------------------------------- GFSK
double gfskPulse(double t, double bt) {
    if (t <= -1.5 || t >= 1.5) return 0;
    const double K = kPi * std::sqrt(2 / std::log(2.0));
    return (std::erf(K * bt * (t + 0.5)) - std::erf(K * bt * (t - 0.5))) / 2;
}

void synthesize(const u8* tones, int n, double symbolSeconds, double spacingHz, double bt, double f0, double rate,
                double amplitude, std::vector<float>& out) {
    if (!tones || n <= 0 || !(rate > 0) || !(symbolSeconds > 0)) return;
    const double sps = symbolSeconds * rate;
    const long total = static_cast<long>(std::floor(n * sps + 0.5));
    out.reserve(out.size() + static_cast<std::size_t>(total));
    double phase = 0;
    for (long k = 0; k < total; ++k) {
        out.push_back(static_cast<float>(amplitude * std::cos(phase)));
        // the frequency at this sample: the tones around it weighed by the GFSK pulse (the first and last held)
        const double u = k / sps;
        const long s0 = static_cast<long>(std::floor(u));
        double dev = 0;
        if (bt <= 0) dev = tones[std::max(0L, std::min(static_cast<long>(n - 1), s0))];
        else
            for (long i = s0 - 1; i <= s0 + 1; ++i)
                dev += tones[std::max(0L, std::min(static_cast<long>(n - 1), i))] * gfskPulse(u - i - 0.5, bt);
        phase += 2 * kPi * (f0 + spacingHz * dev) / rate;
        if (phase > 2 * kPi) phase -= 2 * kPi * std::floor(phase / (2 * kPi));
    }
}

bool synthesizeFt8(const std::string& text, double f0, double rate, double amplitude, std::vector<float>& out,
                   CallHashTable* ht) {
    u8 t[kFt8Symbols];
    if (!ft8Tones(text, t, ht)) return false;
    synthesize(t, kFt8Symbols, 0.16, 6.25, 2, f0, rate, amplitude, out);
    return true;
}

bool synthesizeFt4(const std::string& text, double f0, double rate, double amplitude, std::vector<float>& out,
                   CallHashTable* ht) {
    u8 t[kFt4Symbols];
    if (!ft4Tones(text, t, ht)) return false;
    synthesize(t, kFt4Symbols, 0.048, 1 / 0.048, 1, f0, rate, amplitude, out);
    return true;
}

// ---------------------------------------------------------------- the slot decoder
const ProtocolInfo& protocolInfo(Protocol p) {
    static const ProtocolInfo kFt8 = {"FT8", 12800, 2048, 0.16, 6.25, 79, 8, 2, 15, 0.5, 27.6, 13.5};
    static const ProtocolInfo kFt4 = {"FT4", 32000.0 / 3, 512, 0.048, 1 / 0.048, 105, 4, 1, 7.5, 0.5, 13, 6};
    return p == Protocol::Ft8 ? kFt8 : kFt4;
}

Engine::Engine() : noise_(1e-20) {
    for (int i = 0; i < kN; ++i) llr_[i] = 0;
}

void Engine::spectrogram(int nfft, int hop, int b0, int nb, int nFrames) {
    if (static_cast<int>(win_.size()) != nfft) {
        win_.resize(static_cast<std::size_t>(nfft));
        for (int i = 0; i < nfft; ++i) win_[static_cast<std::size_t>(i)] = static_cast<float>(0.5 - 0.5 * std::cos(2 * kPi * i / nfft));
        fft_.resize(nfft);
        buf_.resize(static_cast<std::size_t>(nfft));
    }
    const std::size_t total = static_cast<std::size_t>(nFrames) * static_cast<std::size_t>(nb);
    if (db_.size() < total) { db_.resize(total); lin_.resize(total); }
    for (int j = 0; j < nFrames; ++j) {
        const std::size_t o = static_cast<std::size_t>(j) * static_cast<std::size_t>(hop);
        for (int i = 0; i < nfft; ++i) buf_[static_cast<std::size_t>(i)] = std::complex<float>(y_[o + static_cast<std::size_t>(i)] * win_[static_cast<std::size_t>(i)], 0.f);
        fft_.forward(&buf_[0]);
        float* L = &lin_[static_cast<std::size_t>(j) * static_cast<std::size_t>(nb)];
        float* D = &db_[static_cast<std::size_t>(j) * static_cast<std::size_t>(nb)];
        for (int k = 0; k < nb; ++k) {
            const std::complex<float> v = buf_[static_cast<std::size_t>(b0 + k)];
            const float p = v.real() * v.real() + v.imag() * v.imag() + 1e-20f;
            L[k] = p;
            D[k] = 10 * std::log10(p);
        }
    }
    // noise: the median power over the band (every 37th value, as the JS)
    std::vector<float> smp;
    smp.reserve(total / 37 + 1);
    for (std::size_t i = 0; i < total; i += 37) smp.push_back(lin_[i]);
    if (smp.empty()) { noise_ = 1e-20; return; }
    std::nth_element(smp.begin(), smp.begin() + static_cast<long>(smp.size() >> 1), smp.end());
    noise_ = smp[smp.size() >> 1] > 0 ? smp[smp.size() >> 1] : 1e-20;
}

// the bit likelihoods normalised (variance 24, as the JS / ft8_lib), then belief propagation
bool Engine::llrDecode(int iters, u8 cw[kN]) {
    double su = 0, su2 = 0;
    for (int i = 0; i < kN; ++i) { su += llr_[i]; su2 += static_cast<double>(llr_[i]) * llr_[i]; }
    const double vr = (su2 - su * su / kN) / kN, nf = std::sqrt(24 / (vr > 0 ? vr : 1));
    for (int i = 0; i < kN; ++i) llr_[i] = static_cast<float>(llr_[i] * nf);
    if (!bpDecode(llr_, iters, cw)) return false;
    u8 a91[12];
    packBits(cw, kK, a91);
    return checkCrc(a91);
}

// port of ft8Precise(): a candidate lined up exactly on its Costas sync blocks (FT8 three of 7 symbols, FT4
// four of 4), then each symbol's tone powers (dB) measured at the exact frequencies and symbol boundaries into
// prec_ (symbols x tones).  (Addition to the JS: FT4 gets it too, with the same steps scaled to its symbol and
// tone spacing; the JS tries it for FT8 only.)
void Engine::precise(Protocol p, double start, double f0) {
    const ProtocolInfo& P = protocolInfo(p);
    const bool ft8 = p == Protocol::Ft8;
    const int SYM = P.symbolSamples, NT = P.tones, NS = P.symbols;
    const double FS = P.sampleRate, sp = P.toneSpacing, bw = sp / 2;
    const long len = static_cast<long>(y_.size());
    const float* x = &y_[0];
    // the known sync symbols and their tones
    int syncSym[24], syncTone[24], nSync = 0;   // (FT8 21, FT4 16)
    if (ft8) {
        for (int g = 0; g < 3; ++g)
            for (int k = 0; k < 7; ++k) { syncSym[nSync] = 36 * g + k; syncTone[nSync++] = kFt8Costas[k]; }
    } else {
        for (int g = 0; g < 4; ++g)
            for (int k = 0; k < 4; ++k) { syncSym[nSync] = 1 + 33 * g + k; syncTone[nSync++] = kFt4Costas[g][k]; }
    }
    // the summed power of the sync tones for a start s0 and up to 8 frequencies at once (worked side by side)
    struct Sync {
        static void pow(const float* x, long len, int SYM, double FS, double sp, const int* ss, const int* st, int ns,
                        double s0, const double* f, int nf, double* out) {
            for (int k = 0; k < nf; ++k) out[k] = 0;
            for (int q = 0; q < ns; ++q) {
                const long o = static_cast<long>(s0) + static_cast<long>(ss[q]) * SYM;
                if (o < 0 || o + SYM > len) continue;
                double ar[8], ai[8], cr[8], ci[8], pr[8], pi[8];
                for (int k = 0; k < nf; ++k) {
                    const double w = 2 * kPi * (f[k] + st[q] * sp) / FS * 4;
                    ar[k] = std::cos(w); ai[k] = -std::sin(w);
                    cr[k] = 0; ci[k] = 0; pr[k] = 1; pi[k] = 0;
                }
                for (int i = 0; i < SYM; i += 4) {
                    const double v = x[o + i];
                    for (int k = 0; k < nf; ++k) {
                        cr[k] += v * pr[k];
                        ci[k] += v * pi[k];
                        const double nr = pr[k] * ar[k] - pi[k] * ai[k];
                        pi[k] = pr[k] * ai[k] + pi[k] * ar[k];
                        pr[k] = nr;
                    }
                }
                for (int k = 0; k < nf; ++k) out[k] += cr[k] * cr[k] + ci[k] * ci[k];
            }
        }
    };
    double bs = start, bf = f0, bp = -1, fq[8], pw[8];
    for (int a = -SYM / 4; a <= SYM / 4; a += SYM / 16) {
        int nf = 0;
        for (double b = -bw / 2; b <= bw / 2 + 1e-4 && nf < 8; b += bw / 4) fq[nf++] = f0 + b;
        Sync::pow(x, len, SYM, FS, sp, syncSym, syncTone, nSync, start + a, fq, nf, pw);
        for (int k = 0; k < nf; ++k)
            if (pw[k] > bp) { bp = pw[k]; bs = start + a; bf = fq[k]; }
    }
    double ds = SYM / 32.0, df = 0.128 * bw;   // (FT8: 64 samples, 0.4 Hz, as the JS)
    for (int it = 0; it < 3; ++it, ds /= 2, df /= 2) {
        for (int sg = -1; sg <= 1; sg += 2) {
            Sync::pow(x, len, SYM, FS, sp, syncSym, syncTone, nSync, bs + sg * ds, &bf, 1, pw);
            if (pw[0] > bp) { bp = pw[0]; bs += sg * ds; }
        }
        fq[0] = bf - df;
        fq[1] = bf + df;
        Sync::pow(x, len, SYM, FS, sp, syncSym, syncTone, nSync, bs, fq, 2, pw);
        // (the JS tries -df, then +df from wherever that left it; both from bf here, the better kept)
        if (pw[0] > bp && pw[0] >= pw[1]) { bp = pw[0]; bf = fq[0]; }
        else if (pw[1] > bp) { bp = pw[1]; bf = fq[1]; }
    }
    // the tone powers: each tone a rotating phasor (no trig in the loop)
    if (prec_.size() < static_cast<std::size_t>(NS * NT)) prec_.resize(static_cast<std::size_t>(NS * NT));
    double rr[8], ri[8];
    for (int t = 0; t < 8; ++t) {   // (FT4: tones 4-7 worked but not used)
        const double w = 2 * kPi * (bf + t * sp) / FS;
        rr[t] = std::cos(w);
        ri[t] = -std::sin(w);
    }
    // (the tones side by side in the inner loop, so they are worked together)
    for (int s = 0; s < NS; ++s) {
        const long o = static_cast<long>(bs) + static_cast<long>(s) * SYM;
        // (single precision here: over one symbol the phasors drift by under 1e-3, and it runs twice as fast)
        float cr[8] = {0}, ci[8] = {0}, pr[8], pi[8], fr[8], fi[8];
        for (int t = 0; t < 8; ++t) { pr[t] = 1; pi[t] = 0; fr[t] = static_cast<float>(rr[t]); fi[t] = static_cast<float>(ri[t]); }
        if (o >= 0 && o + SYM <= len)
            for (int i = 0; i < SYM; ++i) {
                const float v = x[o + i];
                for (int t = 0; t < 8; ++t) {
                    cr[t] += v * pr[t];
                    ci[t] += v * pi[t];
                    const float nr = pr[t] * fr[t] - pi[t] * fi[t];
                    pi[t] = pr[t] * fi[t] + pi[t] * fr[t];
                    pr[t] = nr;
                }
            }
        for (int t = 0; t < NT; ++t)
            prec_[static_cast<std::size_t>(s * NT + t)] = static_cast<float>(10 * std::log10(static_cast<double>(cr[t]) * cr[t] + static_cast<double>(ci[t]) * ci[t] + 1e-20));
    }
}

void Engine::decodeOnce(Protocol p, const DecodeOptions& opt0, int pass, CallHashTable* ht, std::set<std::string>& seen,
                        std::vector<Found>& found) {
    const bool ft8 = p == Protocol::Ft8;
    const ProtocolInfo& P = protocolInfo(p);
    const int NFFT = ft8 ? 4096 : 1024, H = ft8 ? 1024 : 256;
    const double bw = P.sampleRate / NFFT;   // half a tone
    const int span = 2 * (P.tones - 1);       // bins from tone 0 to the top tone
    const int nsym = P.symbols;
    // (the later passes: a slightly lower threshold, as what's left is quieter; FT8 fewer exact tries)
    const double minScore = pass ? opt0.minScore - 0.4 : opt0.minScore;
    const int maxCand = opt0.maxCandidates > 0 ? opt0.maxCandidates : ft8 ? 300 : 220;
    const int maxPrecise = pass ? std::min(opt0.precise, 20) : opt0.precise;
    const int maxRank = pass ? std::min(opt0.preciseRank, 60) : opt0.preciseRank;
    const double fLo = std::max(0.0, opt0.fLo), fHi = std::max(fLo, opt0.fHi);
    const int b0 = std::max(1, static_cast<int>(std::floor(fLo / bw)));
    const int b1 = std::min(NFFT / 2 - 1, static_cast<int>(std::ceil(fHi / bw)) + span);
    const int NB = b1 - b0 + 1;
    const long n = static_cast<long>(y_.size());
    const int nFrames = n >= NFFT ? static_cast<int>((n - NFFT) / H + 1) : 0;
    if (NB <= span + 1 || nFrames < 2 * nsym + 2) return;
    spectrogram(NFFT, H, b0, NB, nFrames);
    const float* db = &db_[0];
    const float* lin = &lin_[0];
#define AT(j, k) db[static_cast<std::size_t>(j) * static_cast<std::size_t>(NB) + static_cast<std::size_t>(k)]
    // sync: message start m (in half symbols; symbol s is frame m-1+2s), base bin b (tone k at bin b+2k)
    cands_.clear();
    for (int m = 1; m - 1 + 2 * (nsym - 1) < nFrames; ++m) {
        for (int b = 0; b + span < NB; ++b) {
            double sc = 0;
            int na = 0;
            if (ft8) {
                for (int g = 0; g < 3; ++g)
                    for (int k = 0; k < 7; ++k) {
                        const int s = g * 36 + k, j = m - 1 + 2 * s, kk = b + 2 * kFt8Costas[k];
                        const float v = AT(j, kk);
                        if (kFt8Costas[k] > 0) { sc += v - AT(j, kk - 2); ++na; }
                        if (kFt8Costas[k] < 7) { sc += v - AT(j, kk + 2); ++na; }
                        if (k > 0) { sc += v - AT(j - 2, kk); ++na; }
                        if (k < 6) { sc += v - AT(j + 2, kk); ++na; }
                    }
            } else {
                for (int g = 0; g < 4; ++g)
                    for (int k = 0; k < 4; ++k) {
                        const int s = 1 + 33 * g + k, j = m - 1 + 2 * s, t = kFt4Costas[g][k], kk = b + 2 * t;
                        const float v = AT(j, kk);
                        if (t > 0) { sc += v - AT(j, kk - 2); ++na; }
                        if (t < 3) { sc += v - AT(j, kk + 2); ++na; }
                        if (k > 0) { sc += v - AT(j - 2, kk); ++na; }
                        if (k < 3) { sc += v - AT(j + 2, kk); ++na; }
                    }
            }
            sc /= na;
            if (sc >= minScore) {
                Cand c;
                c.score = static_cast<float>(sc);
                c.m = m;
                c.b = b;
                cands_.push_back(c);
            }
        }
    }
    struct ByScore {
        bool operator()(const Cand& a, const Cand& b) const { return a.score > b.score; }
    };
    std::stable_sort(cands_.begin(), cands_.end(), ByScore());
    kept_.clear();
    for (std::size_t i = 0; i < cands_.size() && static_cast<int>(kept_.size()) < maxCand; ++i) {
        const Cand& c = cands_[i];
        bool near = false;
        for (std::size_t k = 0; k < kept_.size() && !near; ++k)
            near = std::abs(kept_[k].m - c.m) <= 1 && std::abs(kept_[k].b - c.b) <= 1;
        if (!near) kept_.push_back(c);
    }
    // decode each candidate
    int tries = 0;
    u8 cw[kN];
    for (std::size_t ci = 0; ci < kept_.size(); ++ci) {
        const int m = kept_[ci].m, b = kept_[ci].b;
        bool ok = false;
        if (ft8) {
            float s2[8];
            for (int attempt = 0; attempt < 2 && !ok; ++attempt) {
                if (attempt == 1) {   // the precise second try (the most promising candidates)
                    if (tries >= maxPrecise || static_cast<int>(ci) >= maxRank) break;
                    ++tries;
                    precise(p, static_cast<double>(m) * H, (b0 + b) * bw);
                }
                for (int s = 7, d = 0; s < 72; ++s) {
                    if (s >= 36 && s < 43) continue;
                    for (int t = 0; t < 8; ++t)
                        s2[t] = attempt ? prec_[static_cast<std::size_t>(s * 8 + kFt8Gray[t])] : AT(m - 1 + 2 * s, b + 2 * kFt8Gray[t]);
                    llr_[3 * d] = std::max(std::max(s2[4], s2[5]), std::max(s2[6], s2[7])) - std::max(std::max(s2[0], s2[1]), std::max(s2[2], s2[3]));
                    llr_[3 * d + 1] = std::max(std::max(s2[2], s2[3]), std::max(s2[6], s2[7])) - std::max(std::max(s2[0], s2[1]), std::max(s2[4], s2[5]));
                    llr_[3 * d + 2] = std::max(std::max(s2[1], s2[3]), std::max(s2[5], s2[7])) - std::max(std::max(s2[0], s2[2]), std::max(s2[4], s2[6]));
                    ++d;
                }
                ok = llrDecode(opt0.iters, cw);
            }
        } else {
            float s2[4];
            for (int attempt = 0; attempt < 2 && !ok; ++attempt) {
                if (attempt == 1) {   // (Addition to the JS: FT4's precise second try)
                    if (tries >= maxPrecise || static_cast<int>(ci) >= maxRank) break;
                    ++tries;
                    precise(p, static_cast<double>(m) * H, (b0 + b) * bw);
                }
                for (int k = 0; k < 87; ++k) {
                    const int s = k + (k < 29 ? 5 : k < 58 ? 9 : 13), j = m - 1 + 2 * s;
                    for (int t = 0; t < 4; ++t)
                        s2[t] = attempt ? prec_[static_cast<std::size_t>(s * 4 + kFt4Gray[t])] : AT(j, b + 2 * kFt4Gray[t]);
                    llr_[2 * k] = std::max(s2[2], s2[3]) - std::max(s2[0], s2[1]);
                    llr_[2 * k + 1] = std::max(s2[1], s2[3]) - std::max(s2[0], s2[2]);
                }
                ok = llrDecode(opt0.iters, cw);   // (the CRC covers the scrambled message)
            }
        }
        if (!ok) continue;
        u8 a77[kPayloadBytes];
        packBits(cw, kPayloadBits, a77);
        if (!ft8) for (int i = 0; i < kPayloadBytes; ++i) a77[i] = static_cast<u8>(a77[i] ^ kFt4Xor[i]);
        a77[9] &= 0xF8;
        std::string text;
        if (!unpack77(a77, text, ht)) continue;
        if (seen.count(text)) continue;
        seen.insert(text);
        // signal strength: the power in the tones actually sent (from the codeword)
        Found f;
        f.ntones = nsym;
        if (ft8) ft8TonesFromPayload(a77, f.tones);
        else ft4TonesFromPayload(a77, f.tones);
        double sig = 0;
        for (int s = 0; s < nsym; ++s)
            sig += lin[static_cast<std::size_t>(m - 1 + 2 * s) * static_cast<std::size_t>(NB) + static_cast<std::size_t>(b + 2 * f.tones[s])];
        sig /= nsym;
        // per-bin noise is noise_; a Hann window's noise bandwidth is 1.5 bins
        const double snr = 10 * std::log10(std::max(1e-6, sig / noise_ - 1)) - 10 * std::log10(2500 / (1.5 * bw)) + opt0.snrCal;
        f.r.text = text;
        std::memcpy(f.r.payload, a77, kPayloadBytes);
        f.r.start = static_cast<double>(m) * H;   // (symbol s is centred on frame m-1+2s)
        f.r.freq = (b0 + b) * bw;
        f.r.snr = static_cast<int>(std::floor(snr + 0.5));
        f.r.score = kept_[ci].score;
        f.r.pass = pass + 1;
        found.push_back(f);
    }
#undef AT
}

// port of gfskPhaseSteps() (the running total: the GFSK phase at each sample, without the carrier), kept as its
// cosine and sine (Addition to the JS: so matchPower() and the subtraction need no trig per sample)
void Engine::phaseSteps(const u8* tones, int ntones, int sps, double bt, std::vector<double>& phi) {
    const std::size_t n = static_cast<std::size_t>(ntones) * static_cast<std::size_t>(sps);
    phi.resize(n);
    cphi_.resize(n);
    sphi_.resize(n);
    const double pk = 2 * kPi / sps;
    // the pulse tabulated once per call (3 symbols)
    pulse_.resize(static_cast<std::size_t>(3 * sps));
    for (int i = 0; i < 3 * sps; ++i) pulse_[static_cast<std::size_t>(i)] = gfskPulse(static_cast<double>(i) / sps - 1.5, bt);
    double acc = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const long s0 = static_cast<long>(k / static_cast<std::size_t>(sps));
        const long r = static_cast<long>(k % static_cast<std::size_t>(sps));
        double dev = 0;
        // symbol i contributes pulse((k - i*sps)/sps - 0.5) = pulse_[k - i*sps + sps] (i = s0-1 .. s0+1)
        for (long i = s0 - 1; i <= s0 + 1; ++i) {
            const long j = r + (s0 - i) * sps + sps;
            if (j < 0 || j >= 3 * sps) continue;
            dev += tones[std::max(0L, std::min(static_cast<long>(ntones - 1), i))] * pulse_[static_cast<std::size_t>(j)];
        }
        acc += pk * dev;
        phi[k] = acc;
        cphi_[k] = static_cast<float>(std::cos(acc));
        sphi_[k] = static_cast<float>(std::sin(acc));
    }
}

// port of matchPower(): how well a rebuilt signal at (start, f0) matches the audio: the summed power of each
// symbol's correlation, from every step-th sample.  (Addition to the JS: the carrier as a rotating phasor and the
// GFSK phase from phaseSteps()'s table, so a finer step costs little; see refine().)
double Engine::matchPower(double start, double f0, double fs, const std::vector<double>& phi, int sps, int step) const {
    const double w0 = 2 * kPi * f0 / fs;
    const long n = static_cast<long>(phi.size()), len = static_cast<long>(y_.size()), st = static_cast<long>(start);
    const long k0 = step >> 1;
    double tot = 0, cr = 0, ci = 0;
    double pr = std::cos(w0 * (k0 + 1)), pi = std::sin(w0 * (k0 + 1));   // the carrier's phase w0 * (k + 1)
    const double rr = std::cos(w0 * step), ri = std::sin(w0 * step);
    long sym = 0;
    int renorm = 0;
    for (long k = k0; k < n; k += step) {
        const long s = k / sps;
        if (s != sym) { tot += cr * cr + ci * ci; cr = 0; ci = 0; sym = s; }
        const long xi = st + k;
        if (xi >= 0 && xi < len) {
            const double c = cphi_[static_cast<std::size_t>(k)], sn = sphi_[static_cast<std::size_t>(k)];
            const double v = y_[static_cast<std::size_t>(xi)];
            cr += v * (pr * c - pi * sn);   // cos(w0 (k+1) + phi)
            ci -= v * (pi * c + pr * sn);   // sin(w0 (k+1) + phi)
        }
        const double t = pr * rr - pi * ri;
        pi = pr * ri + pi * rr;
        pr = t;
        if (++renorm == 4096) {   // (keep the phasor's length 1)
            renorm = 0;
            const double m = 1 / std::sqrt(pr * pr + pi * pi);
            pr *= m;
            pi *= m;
        }
    }
    return tot + cr * cr + ci * ci;
}

// port of refine(): timing and frequency fine-tuned around the decoder's estimate (it's within half a symbol and
// half a bin): a coarse grid, then the two refined in turn, each by a few steps that halve.  (Addition to the JS:
// every 8th sample for the grid and every 2nd for the steps, not every 32nd and 16th: a correlation over every
// D-th sample loses 10 log D dB to the noise folded in, which made weak signals' timing and frequency wander.)
void Engine::refine(double start, double f0, double fs, const std::vector<double>& phi, int sps, double binHz, double& bs,
                    double& bf) const {
    bs = start;
    bf = f0;
    double bp = -1;
    for (int a = -sps / 4; a <= sps / 4; a += sps / 8)
        for (double b = -binHz / 2; b <= binHz / 2 + 1e-9; b += binHz / 4) {
            const double p = matchPower(start + a, f0 + b, fs, phi, sps, 8);
            if (p > bp) { bp = p; bs = start + a; bf = f0 + b; }
        }
    bp = matchPower(bs, bf, fs, phi, sps, 2);
    double ds = sps / 16.0, df = binHz / 8;
    for (int it = 0; it < 4; ++it, ds /= 2, df /= 2) {
        for (int sg = -1; sg <= 1; sg += 2) {
            const double c = std::floor(bs + sg * ds + 0.5);
            const double p = matchPower(c, bf, fs, phi, sps, 2);
            if (p > bp) { bp = p; bs = c; }
        }
        for (int sg = -1; sg <= 1; sg += 2) {
            const double p = matchPower(bs, bf + sg * df, fs, phi, sps, 2);
            if (p > bp) { bp = p; bf += sg * df; }
        }
    }
}

// port of subtractSignal(): the decoded signal rebuilt and taken out of the audio
void Engine::subtract(const Found& f, Protocol p, double& rs, double& rf) {
    const ProtocolInfo& P = protocolInfo(p);
    const int sps = P.symbolSamples, ns = f.ntones;
    const double fs = P.sampleRate, binHz = fs / (p == Protocol::Ft8 ? 4096 : 1024);
    phaseSteps(f.tones, ns, sps, P.bt, phi_);
    refine(f.r.start, f.r.freq, fs, phi_, sps, binHz, rs, rf);
    const double w0 = 2 * kPi * rf / fs;
    const long n = static_cast<long>(phi_.size()), len = static_cast<long>(y_.size()), st = static_cast<long>(rs);
    cosW_.resize(static_cast<std::size_t>(n));
    sinW_.resize(static_cast<std::size_t>(n));
    ar_.assign(static_cast<std::size_t>(ns), 0.0);
    ai_.assign(static_cast<std::size_t>(ns), 0.0);
    // the rebuilt waveform once (cos and sin), then each symbol's complex amplitude (strength and phase there)
    for (long k = 0; k < n; ++k) {
        const double ph = w0 * (k + 1) + phi_[static_cast<std::size_t>(k)];
        cosW_[static_cast<std::size_t>(k)] = static_cast<float>(std::cos(ph));
        sinW_[static_cast<std::size_t>(k)] = static_cast<float>(std::sin(ph));
        const long xi = st + k;
        if (xi < 0 || xi >= len) continue;
        const double v = y_[static_cast<std::size_t>(xi)];
        const std::size_t q = static_cast<std::size_t>(k / sps);
        ar_[q] += v * cosW_[static_cast<std::size_t>(k)];
        ai_[q] -= v * sinW_[static_cast<std::size_t>(k)];
    }
    // smoothed over three symbols (signals fade, but not that fast)
    sr_.assign(static_cast<std::size_t>(ns), 0.0);
    si_.assign(static_cast<std::size_t>(ns), 0.0);
    for (int q = 0; q < ns; ++q) {
        double a = 0, b = 0, w = 0;
        for (int t = std::max(0, q - 1); t <= std::min(ns - 1, q + 1); ++t) {
            const double wt = t == q ? 2 : 1;
            a += wt * ar_[static_cast<std::size_t>(t)];
            b += wt * ai_[static_cast<std::size_t>(t)];
            w += wt;
        }
        sr_[static_cast<std::size_t>(q)] = 2 * a / w / sps;
        si_[static_cast<std::size_t>(q)] = 2 * b / w / sps;
    }
    // taken out, with the amplitude eased from one symbol's centre to the next (no steps)
    for (long k = 0; k < n; ++k) {
        const long xi = st + k;
        if (xi < 0 || xi >= len) continue;
        const double u = static_cast<double>(k) / sps - 0.5;
        const int s0 = std::max(0, std::min(ns - 1, static_cast<int>(std::floor(u))));
        const int s1 = std::min(ns - 1, s0 + 1);
        const double fr = std::max(0.0, std::min(1.0, u - s0));
        const double re = sr_[static_cast<std::size_t>(s0)] + (sr_[static_cast<std::size_t>(s1)] - sr_[static_cast<std::size_t>(s0)]) * fr;
        const double im = si_[static_cast<std::size_t>(s0)] + (si_[static_cast<std::size_t>(s1)] - si_[static_cast<std::size_t>(s0)]) * fr;
        y_[static_cast<std::size_t>(xi)] -= static_cast<float>(re * cosW_[static_cast<std::size_t>(k)] - im * sinW_[static_cast<std::size_t>(k)]);
    }
}

// port of decode() / decode4(): up to three passes, each decoded signal subtracted before the next
void Engine::decode(Protocol p, const float* x, std::size_t n, const DecodeOptions& opt, CallHashTable* ht,
                    std::vector<RawDecode>& out) {
    if (!x || n == 0) return;
    y_.resize(n);
    for (std::size_t i = 0; i < n; ++i) y_[i] = finite(x[i]) ? x[i] : 0.f;   // (Addition to the JS)
    std::set<std::string> seen;
    const int passes = std::max(1, std::min(5, opt.passes));
    const ProtocolInfo& P = protocolInfo(p);
    const double binHz = P.sampleRate / (p == Protocol::Ft8 ? 4096 : 1024);
    std::vector<Found> res;
    for (int ps = 0; ps < passes; ++ps) {
        res.clear();
        decodeOnce(p, opt, ps, ht, seen, res);
        for (std::size_t i = 0; i < res.size(); ++i) {
            double rs = res[i].r.start, rf = res[i].r.freq;
            if (ps < passes - 1) subtract(res[i], p, rs, rf);
            else if (opt.refine) {   // (Addition to the JS: the last pass's timing fine-tuned too)
                phaseSteps(res[i].tones, res[i].ntones, P.symbolSamples, P.bt, phi_);
                refine(res[i].r.start, res[i].r.freq, P.sampleRate, phi_, P.symbolSamples, binHz, rs, rf);
            }
            RawDecode r = res[i].r;
            if (opt.refine) { r.start = rs; r.freq = rf; }
            out.push_back(r);
        }
        if (res.empty()) break;
    }
}

}  // namespace ftx
}  // namespace dec
