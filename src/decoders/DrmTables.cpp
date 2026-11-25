// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DrmTables.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  DRM's tables (ETSI ES 201 980), exactly as OFFgrid-SDR's JS has them in
//  DRM_T (diddy-boy, https://github.com/solariun/offgrid-sdr, index.html,
//  the "drmlib" block), which took them from Dream's CellMappingTable and
//  MLC tables (TU Darmstadt, Volker Fischer and others; GPL-2.0-or-later,
//  licenses/dream-COPYING.txt):
//    - per robustness mode: the frequency pilots and the time references
//      ([carrier, phase in 1/1024 turns] pairs), the gain references'
//      scattering (x, y, k0) and phase tables W, Z, Q, the boosted pilots
//      of each spectrum occupancy (4 each: lower pair, upper pair), and the
//      FAC cells ([symbol, carrier] pairs);
//    - kmin/kmax: the carriers of each occupancy (index so * 4 + mode);
//    - the puncturing patterns (rate index: [group length, coded bits a
//      group, then the generator set of each step]) and the tail patterns;
//    - names: languages, programme types, sampling rates.
//
//  How it fits in
//  --------------
//  Declared in DrmInternal.h; used by the receiver (DrmRx.cpp) and the
//  generator (DrmEncoders.cpp).
//
//  Contributing
//  ------------
//  Generated from the JS's DRM_T JSON: do not edit the numbers by hand; if
//  the JS's table changes, regenerate (keep the order of the pairs).
// ============================================================================
#include "decoders/DrmInternal.h"

namespace dec {
namespace drm {

const int kTu[4] = {1152, 1024, 704, 448};
const int kTg[4] = {128, 256, 256, 352};
const int kSymbols[4] = {15, 15, 20, 24};
const char kModeLetter[4] = {'A', 'B', 'C', 'D'};
const double kOccupancyKhz[6] = {4.5, 5, 9, 10, 18, 20};

namespace {

const int kA_freq[] = {18, 205, 54, 836, 72, 215};
const int kA_time[] = {17, 973, 18, 205, 19, 717, 21, 264, 28, 357, 29, 357, 32, 952, 33, 440, 39, 856, 40, 88, 41, 88, 53, 68, 54, 836, 55, 836, 56, 836, 60, 1008, 61, 1008, 63, 752, 71, 215, 72, 215, 73, 727};
const int kA_W[] = {228, 341, 455, 455, 569, 683, 683, 796, 910, 910, 0, 114, 114, 228, 341};
const int kA_Z[] = {0, 81, 248, 18, 106, 106, 122, 116, 31, 129, 129, 39, 33, 32, 111};
const int kA_fac[] = {2, 26, 2, 46, 2, 66, 2, 86, 3, 10, 3, 30, 3, 50, 3, 70, 3, 90, 4, 14, 4, 22, 4, 34, 4, 62, 4, 74, 4, 94, 5, 26, 5, 38, 5, 58, 5, 66, 5, 78, 6, 22, 6, 30, 6, 42, 6, 62, 6, 70, 6, 82, 7, 26, 7, 34, 7, 46, 7, 66, 7, 74, 7, 86, 8, 10, 8, 30, 8, 38, 8, 50, 8, 58, 8, 70, 8, 78, 8, 90, 9, 14, 9, 22, 9, 34, 9, 42, 9, 62, 9, 74, 9, 82, 9, 94, 10, 26, 10, 38, 10, 46, 10, 66, 10, 86, 11, 10, 11, 30, 11, 50, 11, 70, 11, 90, 12, 14, 12, 34, 12, 74, 12, 94, 13, 38, 13, 58, 13, 78};
const int kB_freq[] = {16, 331, 48, 651, 64, 555};
const int kB_time[] = {14, 304, 16, 331, 18, 108, 20, 620, 24, 192, 26, 704, 32, 44, 36, 432, 42, 588, 44, 844, 48, 651, 49, 651, 50, 651, 54, 460, 56, 460, 62, 944, 64, 555, 66, 940, 68, 428};
const int kB_W[] = {512, 0, 512, 0, 512, 0, 512, 0, 512, 0, 512, 0, 512, 0, 512};
const int kB_Z[] = {0, 57, 164, 64, 12, 168, 255, 161, 106, 118, 25, 232, 132, 233, 38};
const int kB_fac[] = {2, 13, 2, 25, 2, 43, 2, 55, 2, 67, 3, 15, 3, 27, 3, 45, 3, 57, 3, 69, 4, 17, 4, 29, 4, 47, 4, 59, 4, 71, 5, 19, 5, 31, 5, 49, 5, 61, 5, 73, 6, 9, 6, 21, 6, 33, 6, 51, 6, 63, 6, 75, 7, 11, 7, 23, 7, 35, 7, 53, 7, 65, 7, 77, 8, 13, 8, 25, 8, 37, 8, 55, 8, 67, 8, 79, 9, 15, 9, 27, 9, 39, 9, 57, 9, 69, 9, 81, 10, 17, 10, 29, 10, 41, 10, 59, 10, 71, 10, 83, 11, 19, 11, 31, 11, 43, 11, 61, 11, 73, 12, 21, 12, 33, 12, 45, 12, 63, 12, 75, 13, 23, 13, 35, 13, 47, 13, 65, 13, 77};
const int kC_freq[] = {11, 214, 33, 392, 44, 242};
const int kC_time[] = {8, 722, 10, 466, 11, 214, 12, 214, 14, 479, 16, 516, 18, 260, 22, 577, 24, 662, 28, 3, 30, 771, 32, 392, 33, 392, 36, 37, 38, 37, 42, 474, 44, 242, 45, 242, 46, 754};
const int kC_W[] = {465, 372, 279, 186, 93, 0, 931, 838, 745, 652, 931, 838, 745, 652, 559, 465, 372, 279, 186, 93};
const int kC_Z[] = {0, 76, 29, 76, 9, 190, 161, 248, 33, 108, 179, 178, 83, 253, 127, 105, 101, 198, 250, 145};
const int kC_fac[] = {3, 9, 3, 21, 3, 45, 3, 57, 4, 23, 4, 35, 4, 47, 5, 13, 5, 25, 5, 37, 5, 49, 6, 15, 6, 27, 6, 39, 6, 51, 7, 5, 7, 17, 7, 29, 7, 41, 7, 53, 8, 7, 8, 19, 8, 31, 8, 43, 8, 55, 9, 9, 9, 21, 9, 45, 9, 57, 10, 23, 10, 35, 10, 47, 11, 13, 11, 25, 11, 37, 11, 49, 12, 15, 12, 27, 12, 39, 12, 51, 13, 5, 13, 17, 13, 29, 13, 41, 13, 53, 14, 7, 14, 19, 14, 31, 14, 43, 14, 55, 15, 9, 15, 21, 15, 45, 15, 57, 16, 23, 16, 35, 16, 47, 17, 13, 17, 25, 17, 37, 17, 49, 18, 15, 18, 27, 18, 39, 18, 51};
const int kD_freq[] = {7, 788, 21, 1014, 28, 332};
const int kD_time[] = {5, 636, 6, 124, 7, 788, 8, 788, 9, 200, 11, 688, 12, 152, 14, 920, 15, 920, 17, 644, 18, 388, 20, 652, 21, 1014, 23, 176, 24, 176, 26, 752, 27, 496, 28, 332, 29, 432, 30, 964, 32, 452};
const int kD_W[] = {366, 439, 512, 585, 658, 731, 805, 878, 731, 805, 878, 951, 0, 73, 146, 219, 73, 146, 219, 293, 366, 439, 512, 585};
const int kD_Z[] = {0, 240, 17, 60, 220, 38, 151, 101, 110, 7, 78, 82, 175, 150, 106, 25, 165, 7, 252, 124, 253, 177, 197, 142};
const int kD_fac[] = {3, 9, 3, 18, 3, 27, 4, 10, 4, 19, 5, 11, 5, 20, 5, 29, 6, 12, 6, 30, 7, 13, 7, 22, 7, 31, 8, 5, 8, 14, 8, 23, 8, 32, 9, 6, 9, 15, 9, 24, 9, 33, 10, 16, 10, 25, 10, 34, 11, 8, 11, 17, 11, 26, 11, 35, 12, 9, 12, 18, 12, 27, 12, 36, 13, 10, 13, 19, 13, 37, 14, 11, 14, 20, 14, 29, 15, 12, 15, 30, 16, 13, 16, 22, 16, 31, 17, 5, 17, 14, 17, 23, 17, 32, 18, 6, 18, 15, 18, 24, 18, 33, 19, 16, 19, 25, 19, 34, 20, 8, 20, 17, 20, 26, 20, 35, 21, 9, 21, 18, 21, 27, 21, 36, 22, 10, 22, 19, 22, 37};

const int kKmin[24] = {2, 1, 0, 0, 2, 1, 0, 0, -102, -91, 0, 0, -114, -103, -69, -44, -98, -87, 0, 0, -110, -99, -67, -43};
const int kKmax[24] = {102, 91, 0, 0, 114, 103, 0, 0, 102, 91, 0, 0, 114, 103, 69, 44, 314, 279, 0, 0, 350, 311, 213, 135};
const int kPunct[13][10] = {{1, 4, 1, 0, 0, 0, 0, 0, 0, 0}, {3, 10, 1, 2, 2, 0, 0, 0, 0, 0}, {1, 3, 2, 0, 0, 0, 0, 0, 0, 0}, {4, 11, 2, 2, 2, 3, 0, 0, 0, 0}, {1, 2, 3, 0, 0, 0, 0, 0, 0, 0}, {4, 7, 3, 5, 3, 4, 0, 0, 0, 0}, {3, 5, 3, 4, 3, 0, 0, 0, 0, 0}, {2, 3, 3, 4, 0, 0, 0, 0, 0, 0}, {8, 11, 3, 4, 4, 3, 4, 4, 3, 4}, {3, 4, 3, 4, 4, 0, 0, 0, 0, 0}, {4, 5, 3, 4, 4, 4, 0, 0, 0, 0}, {7, 8, 3, 4, 4, 4, 4, 4, 4, 0}, {8, 9, 3, 4, 4, 4, 4, 4, 4, 4}};
const int kTail[12][6] = {{3, 3, 3, 3, 3, 3}, {2, 3, 3, 3, 3, 3}, {2, 3, 3, 2, 3, 3}, {2, 2, 3, 2, 3, 3}, {2, 2, 3, 2, 2, 3}, {2, 2, 2, 2, 2, 3}, {2, 2, 2, 2, 2, 2}, {1, 2, 2, 2, 2, 2}, {1, 2, 2, 1, 2, 2}, {1, 1, 2, 1, 2, 2}, {1, 1, 2, 1, 2, 1}, {1, 1, 1, 1, 2, 1}};

// the four modes: {freq, nFreq, time, nTime, scat, W, nW, Z, nZ, Q, boost, fac, nFac}
const ModeTable kModes[4] = {
    {kA_freq, 3, kA_time, 21, {4, 5, 2}, kA_W, 15, kA_Z, 15, 36, {2, 6, 98, 102, 2, 6, 110, 114, -102, -98, 98, 102, -114, -110, 110, 114, -98, -94, 310, 314, -110, -106, 346, 350}, kA_fac, 65},
    {kB_freq, 3, kB_time, 19, {2, 3, 1}, kB_W, 15, kB_Z, 15, 12, {1, 3, 89, 91, 1, 3, 101, 103, -91, -89, 89, 91, -103, -101, 101, 103, -87, -85, 277, 279, -99, -97, 309, 311}, kB_fac, 65},
    {kC_freq, 3, kC_time, 19, {2, 2, 1}, kC_W, 20, kC_Z, 20, 12, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -69, -67, 67, 69, 0, 0, 0, 0, -67, -65, 211, 213}, kC_fac, 65},
    {kD_freq, 3, kD_time, 21, {1, 3, 1}, kD_W, 24, kD_Z, 24, 14, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -44, -43, 43, 44, 0, 0, 0, 0, -43, -42, 134, 135}, kD_fac, 65},
};

const char* const kLang[16] = {"", "Arabic", "Bengali", "Chinese", "Dutch", "English", "French", "German",
                               "Hindi", "Japanese", "Javanese", "Korean", "Portuguese", "Russian", "Spanish", "other"};
// Programme types (as DAB's / RDS's, ETSI TS 101 756); DRM uses the same list.
const char* const kPty[32] = {"", "News", "Current Affairs", "Information", "Sport", "Education", "Drama", "Culture",
                              "Science", "Varied", "Pop Music", "Rock Music", "Easy Listening", "Light Classical",
                              "Serious Classical", "Other Music", "Weather", "Finance", "Children's",
                              "Social Affairs", "Religion", "Phone In", "Travel", "Leisure", "Jazz Music",
                              "Country Music", "National Music", "Oldies Music", "Folk Music", "Documentary", "", ""};
const char* const kAacRates[8] = {"8 kHz", "12 kHz", "16 kHz", "24 kHz", "32 kHz", "48 kHz", "", ""};
const char* const kXheRates[8] = {"9.6 kHz", "12 kHz", "16 kHz", "19.2 kHz", "24 kHz", "32 kHz", "38.4 kHz", "48 kHz"};

}  // namespace

const ModeTable& modeTable(int mi) { return kModes[mi & 3]; }
int kMin(int so, int mi) { return (so < 0 || so > 5) ? 0 : kKmin[so * 4 + (mi & 3)]; }
int kMax(int so, int mi) { return (so < 0 || so > 5) ? 0 : kKmax[so * 4 + (mi & 3)]; }
const int* punct(int rate) { return kPunct[rate < 0 ? 0 : rate > 12 ? 12 : rate]; }
const int* tailPattern(int tp) { return kTail[tp < 0 ? 0 : tp > 11 ? 11 : tp]; }
const char* languageName(int code) { return kLang[code & 15]; }
const char* programmeTypeName(int code) { return kPty[code & 31]; }
const char* aacRateName(int code) { return kAacRates[code & 7]; }
const char* xheRateName(int code) { return kXheRates[code & 7]; }

}  // namespace drm
}  // namespace dec
