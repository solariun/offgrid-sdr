// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Dab.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  DAB / DAB+ digital radio receiver (Kind::Dab): a whole multiplex
//  (ensemble) from the stick's IQ, its stations, and the chosen station's
//  sound, text and pictures:
//
//      #include "decoders/Dab.h"
//      dec::DabDecoder dab;
//      // the stick at 2.048 MS/s, centred on a channel, e.g. 12C:
//      dab.processIq(iq, n, 2048000, 227.360e6, 227.360e6);
//      std::vector<dec::Service> s = dab.services();   // the stations
//      dab.selectService(s[1].id);                     // default: the first
//      std::vector<float> pcm; dab.takeAudio(pcm);     // stereo at audioRate()
//      std::string t; dab.takeText(t);                 // ensemble, station text
//      std::vector<dec::Decoder::EncodedImage> im; dab.takeImages(im);
//
//  Text lines (takeText): the ensemble when first heard ("DAB ensemble
//  OFFgrid DAB (0FF6)"), the station list once complete, the station
//  chosen, and each new dynamic label (DLS) as "<station>: <text>".
//  Images (takeImages): the chosen station's SlideShow pictures ("slide
//  F0D1 <name>") and the multiplex's station logos from its SPI service
//  ("logo F0D1 <station> 128x128"), JPEG or PNG bytes as sent.
//
//  Tuning (what the radio must deliver)
//  ------------------------------------
//  Complex IQ at exactly 2.048 MS/s (another rate is refused: status says
//  so), the multiplex (1.536 MHz wide) in the middle: the stick's centre
//  `centerHz` on the channel's centre frequency (dabChannels(): 5A 174.928
//  to 13F 239.200 MHz; the browser version and the TUI centre the stick on
//  the dial in DAB mode).  `dialHz` is where the user is tuned: if it lies
//  inside a channel's block (within 768 kHz of its centre) that channel's
//  centre is taken, otherwise the dial itself, and when the stick is not
//  centred on it the difference is mixed out here (Addition to the JS),
//  so a stick tuned a little off (up to about ±200 kHz, the block still
//  inside the 2.048 MHz) still decodes.  The receiver itself finds the
//  carrier offset to ±40 kHz (whole carriers, then the fraction), so a
//  stick's ppm error needs no correction.  A change of dial of more than
//  1 Hz starts afresh (as the JS's retune); of more than 800 kHz also
//  forgets the chosen station.  IQ scale does not matter.
//
//  How it works
//  ------------
//  A port of OFFgrid-SDR's JS DAB receiver (diddy-boy, https://github.com/
//  solariun/offgrid-sdr, index.html: the "dablib" block, "DAB / DAB+
//  receiver (mode I, 2.048 MHz) ... Tables from ETSI EN 300 401, as in
//  welle.io", its worker, and the front end "DAB / DAB+ (from 2.1)"):
//    - lock: the null symbol (the quietest 2,656 samples in a frame's
//      length), the phase reference's guard-interval peak, the fractional
//      frequency from the guard correlation of all 76 symbols, whole
//      carriers from neighbouring carriers' phase steps against the phase
//      reference's (±40 carriers);
//    - track: frame by frame from the guard peak near where it is expected;
//      the frequency followed slowly (good frames only, 20 Hz a frame at
//      most); up to 3 poor frames coasted; a watchdog lets a lock go that
//      reads no FIC; the "forgiving re-lock" (from 2.4): samples lost on the
//      way (a USB transfer dropped) are found again from the next null
//      symbol, the frames lost counted from FIG 0/0's CIF count and fed to
//      the time interleaver as "unknown", so DAB+ keeps its super frames;
//      symbols a gap put out of place are erased; single-frequency-network
//      echoes are allowed for (guard thresholds, null-symbol test at 0.75);
//    - only the symbols needed are transformed (FFT 2048): the FIC's and
//      the chosen sub-channel's (and the SPI service's);
//    - DQPSK: differential, the common phase removed per symbol, each
//      carrier weighted by its reliability over the frame; frequency
//      deinterleaving;
//    - FIC: depuncturing, Viterbi, energy dispersal, FIB CRC; FIGs 0/0
//      (ensemble ID, CIF count), 0/1 (sub-channels), 0/2 (services and
//      components, also data services), 0/3 (packet components), 0/13
//      (user applications), 0/17 (programme type), 1/0 and 1/1 (labels);
//      the FIC is read one frame in four once the list is complete;
//    - MSC: the sub-channel's 16-CIF time de-interleaving, depuncturing
//      (EEP A/B, UEP), Viterbi, energy dispersal; then DAB (MPEG Layer II
//      frames, 48 or 24 kHz, joined in pairs at 24 kHz) or DAB+ (super
//      frames of 5 CIFs found by the fire code, Reed-Solomon (120,110),
//      AU CRCs, HE-AAC v2 format);
//    - PAD: dynamic label and MOT SlideShow; the SPI service's logos.
//  The sound comes from liboffgrid-codecs (codec::Mp2Decoder,
//  codec::AacDecoder; DabAudio.cpp): build with src/codecs/*.cpp.  Built
//  without them (OFFGRID_DAB_NO_CODECS) everything up to the audio units
//  is still decoded and counted (stats(), takeUnits()); takeAudio() then
//  gives nothing.
//
//  Additions to the JS (each marked in Dab.cpp / DabFec.cpp / DabPad.cpp):
//  the dial mixed to the channel centre; status() and stats(); text lines;
//  AU CRC failures counted; bounds checks; the Viterbi's scores kept small.
//
//  How it fits in
//  --------------
//  Implements dec::Decoder (Decoder.h): wantsIq() is true; call
//  processIq() (process() with audio does nothing).  DabEncoders.h builds
//  the demo multiplex (tests, the demo band).  Depends on liboffgrid-dsp's
//  FFT (src/dsp/Fft.h) and, for sound, liboffgrid-codecs.
//
//  Contributing
//  ------------
//  Keep in step with the JS (named in each comment); mark anything new
//  "Addition to the JS"; no allocation per sample; strict C++11.  Run
//  tests/test_decoders_dab.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_DAB_H
#define OFFGRID_DECODERS_DAB_H

#include <complex>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "Decoder.h"

namespace dec {

// The Band III channel plan (EN 300 401 annex), 5A to 13F with the N
// channels some countries use (as the JS's band chips).
struct DabChannel {
    const char* name;   // "12C"
    double hz;          // its centre, e.g. 227.360e6
};
const std::vector<DabChannel>& dabChannels();
// The channel whose block holds `hz` (within 768 kHz of its centre, the
// nearest): false if none (JS dabChannelNear).
bool dabChannelNear(double hz, DabChannel& out);
// By name, case-insensitive ("12c").
bool dabChannelByName(const std::string& name, DabChannel& out);

// What the receiver has done since reset() (for tests and diagnostics).
struct DabStats {
    DabStats()
        : frames(0), fibs(0), fibsOk(0), lost(0), gaps(0), resyncs(0), superframesOk(0), superframesBad(0),
          rsFixed(0), unitsOk(0), unitsBad(0), mp2Frames(0), mp2Bad(0), pcmFrames(0), decodeErrors(0) {}
    unsigned frames;           // 96 ms transmission frames decoded
    unsigned fibs, fibsOk;     // FIBs read, and with a good CRC
    unsigned lost;             // locks lost
    unsigned gaps;             // gaps in the samples bridged
    unsigned resyncs;          // DAB+ super frame alignment searched afresh
    unsigned superframesOk, superframesBad;
    unsigned rsFixed;          // bytes Reed-Solomon corrected
    unsigned unitsOk, unitsBad;   // DAB+ audio units: CRC right / wrong
    unsigned mp2Frames, mp2Bad;   // DAB: Layer II frames accepted / refused
    unsigned long pcmFrames;   // stereo frames of sound produced
    unsigned decodeErrors;     // units / frames the codec could not decode
};

// One audio unit (DAB+) or Layer II frame (DAB), as it leaves the
// receiver (setKeepUnits(true)): for a program with its own codec.
struct DabUnit {
    bool aac;                  // DAB+ access unit (else an MP2 frame)
    int sampleRate;            // DAB+: 48000 or 32000 (with SBR the core runs at half)
    bool sbr, ps, stereo;
    std::vector<unsigned char> bytes;
};

class DabDecoder : public Decoder {
public:
    DabDecoder();
    ~DabDecoder();

    Kind kind() const { return Kind::Dab; }
    void reset();
    void process(const float* audio, std::size_t n, double rate);   // not used (IQ decoder)
    void takeText(std::string& out);
    Status status() const;
    double audioCenterHz() const { return 0; }

    bool wantsIq() const { return true; }
    void processIq(const std::complex<float>* iq, std::size_t n, double rate, double centerHz, double dialHz);

    bool hasAudio() const { return true; }
    int audioRate() const;
    void takeAudio(std::vector<float>& stereo);
    std::vector<Service> services() const;
    bool selectService(unsigned id);
    void takeImages(std::vector<EncodedImage>& images);

    // ---- DAB's own
    std::string ensembleLabel() const;   // "" until heard
    unsigned ensembleId() const;         // 0 until heard
    unsigned selectedService() const;    // SId, 0 if none yet
    std::string dynamicLabel() const;    // the chosen station's text now
    DabStats stats() const;
    bool codecsAvailable() const;        // liboffgrid-codecs built in (sound)
    // Keep each audio unit / MP2 frame for takeUnits() (off by default).
    void setKeepUnits(bool on);
    void takeUnits(std::vector<DabUnit>& units);

    struct Impl;

private:
    DabDecoder(const DabDecoder&);
    DabDecoder& operator=(const DabDecoder&);
    std::unique_ptr<Impl> impl_;
};

}  // namespace dec

#endif
