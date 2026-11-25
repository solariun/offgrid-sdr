// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DrmEncoders.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  A DRM transmitter, for tests and the demo band: real DRM signals (FAC,
//  SDC with the name, the multiplex description and the audio settings, the
//  main channel with AAC or xHE-AAC audio super frames, the text message, a
//  MOT SlideShow picture and Journaline pages), built with the receiver's
//  own tables and coding (DrmInternal.h) so the two can't drift apart:
//
//      #include "decoders/DrmEncoders.h"
//      std::vector<std::complex<float> > iq;
//      dec::drmSignal(dec::drmDemoStation(0), iq);   // "OFFgrid DRM", 12 s
//      // iq: complex samples at 48 kHz, the station's reference carrier
//      // (k = 0) on 0 Hz, peak |I|,|Q| = 1: mix it to the station's
//      // frequency (11.700 MHz) at the amplitude wanted, and loop it.
//
//  The demo's two stations (as the JS's demo band, 25 m):
//    0: "OFFgrid DRM", 11.700 MHz: mode B, 10 kHz, 64-QAM (protection
//       level 1), short interleaving, 4-QAM SDC, AAC 24 kHz mono (the JS's
//       12-second jingle, encoded with FAAC's DRM encoder), the station text,
//       a SlideShow PNG and four Journaline pages; 10 superframes (12 s)
//       that loop seamlessly; amplitude 0.05 in the JS's band.
//    1: "OFFgrid xHE", 11.750 MHz: the same in xHE-AAC (the jingle's 150
//       frames, three times over, filling 32 superframes, 38.4 s, exactly),
//       text and picture (no Journaline: no room in its description).
//  Output is IQ (not USB audio): DRM's reference carrier at 0 Hz.
//
//  Beyond the demo (Addition to the JS, which made mode B 10 kHz 64-QAM
//  only): any robustness mode A-D, any spectrum occupancy the mode has,
//  16- or 64-QAM, protection levels, long interleaving, a 16-QAM SDC, AAC
//  at 12 or 24 kHz with your own access units.
//
//  How it fits in
//  --------------
//  Port of OFFgrid-SDR's JS drmDemoSignal and drmXhePack (diddy-boy,
//  https://github.com/solariun/offgrid-sdr, index.html), following Dream
//  (GPL-2.0-or-later, licenses/dream-COPYING.txt) as the receiver does.
//  The demo's sound, picture and xHE-AAC frames are in DrmDemoData.cpp.
//
//  Contributing
//  ------------
//  Deterministic (fixed seeds).  Whatever the receiver learns to read, the
//  generator should learn to send, so the tests can check it.
// ============================================================================
#ifndef OFFGRID_DECODERS_DRM_ENCODERS_H
#define OFFGRID_DECODERS_DRM_ENCODERS_H

#include <complex>
#include <string>
#include <vector>

namespace dec {

struct DrmTxOptions {
    DrmTxOptions()
        : mode(1), occupancy(3), qam16(false), protection(-1), longInterleave(false), sdc16(false),
          label("OFFgrid DRM"), serviceId(0xf06001), language(5), programmeType(0), superframes(10), extras(false),
          slideshow(true), journaline(true), aacRateCode(3), sbr(false), audioMode(0), xhe(false) {}
    int mode;              // 0..3 = A..D
    int occupancy;         // 0..5 = 4.5, 5, 9, 10, 18, 20 kHz
    bool qam16;            // main channel 16-QAM (else 64-QAM)
    int protection;        // protection level (64-QAM 0..3, 16-QAM 0..1); -1: 1 with extras, else 0
    bool longInterleave;   // long (2 s) cell interleaving (else short)
    bool sdc16;            // SDC in 16-QAM (else 4-QAM)
    std::string label;
    unsigned serviceId;
    int language;          // FAC language code (5 English)
    int programmeType;     // FAC programme type
    int superframes;       // the loop's length, 1.2 s each
    bool extras;           // the station text, the SlideShow, Journaline
    bool slideshow;        //  (with extras) the SlideShow picture
    bool journaline;       //  (with extras, AAC only) the Journaline pages
    std::string text;      // the text message ("" = the JS's demo text)
    std::vector<unsigned char> picture;   // PNG (empty = the demo's)
    std::string pictureName;              // ("" = "OFFgrid-SDR.png")
    // AAC: access units, each [the DRM CRC byte][the AU]; empty = the demo
    // jingle (300 frames, 24 kHz mono, no SBR); taken in turn, 10 a
    // multiplex frame at rate code 3 (24 kHz), 5 at rate code 1 (12 kHz).
    std::vector<std::vector<unsigned char> > aac;
    int aacRateCode;
    bool sbr;
    int audioMode;         // 0 mono, 1 parametric stereo, 2 stereo
    // xHE-AAC instead: frames (no CRC) and DRM's compact configuration
    bool xhe;
    std::vector<std::vector<unsigned char> > xheFrames;
    std::vector<unsigned char> xheConfig;
};

// What drmSignal() put in, for tests.
struct DrmTxReport {
    DrmTxReport() : lenBytes(0), lenAudio(0), nmux(0), muxFrames(0) {}
    int lenBytes;     // bytes a multiplex frame
    int lenAudio;     // of them, the audio stream's
    int nmux;         // cells a multiplex frame
    int muxFrames;    // multiplex frames in the loop
    // the audio sent in each multiplex frame of the loop: AAC access units
    // ([CRC][AU]) or xHE-AAC super frames (whole)
    std::vector<std::vector<std::vector<unsigned char> > > audio;
    std::string text;
};

// Builds the loop: complex samples at 48 kHz, peak 1.  False (and why) if
// the options don't fit (e.g. the audio too big for the channel).
bool drmSignal(const DrmTxOptions& opt, std::vector<std::complex<float> >& iq, std::string* error = 0,
               DrmTxReport* report = 0);

// The demo's stations: 0 "OFFgrid DRM" (AAC), 1 "OFFgrid xHE" (xHE-AAC).
DrmTxOptions drmDemoStation(int which);
double drmDemoStationHz(int which);           // 11.700e6, 11.750e6
double drmDemoStationAmplitude(int which);    // 0.05

// The demo's material (DrmDemoData.cpp)
std::vector<std::vector<unsigned char> > drmDemoAacFrames();  // 300 x [CRC][AU]
std::vector<std::vector<unsigned char> > drmDemoXheFrames();  // 150 frames
std::vector<unsigned char> drmDemoSlidePng();                 // 128 x 128 PNG

}  // namespace dec

#endif
