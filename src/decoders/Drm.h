// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Drm.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  DRM (Digital Radio Mondiale, ETSI ES 201 980) receiver (Kind::Drm): the
//  station's name, its settings, its services, its sound (AAC, through
//  liboffgrid-codecs), its text message, its SlideShow pictures and its
//  Journaline news pages, from the stick's IQ samples:
//
//      #include "decoders/Drm.h"
//      dec::DrmDecoder drm;
//      // the stick anywhere near (e.g. 2.048 MS/s centred 200 kHz away), the
//      // dial on the DRM station's frequency (its centre, k = 0):
//      drm.processIq(iq, n, 2048000, centerHz, 11.700e6);
//      dec::Status s = drm.status();      // "OFFgrid DRM · B 10 kHz 64-QAM ..."
//      std::vector<dec::Service> sv = drm.services();
//      std::vector<float> pcm; drm.takeAudio(pcm);   // stereo at audioRate()
//      std::string t; drm.takeText(t);               // the text message
//      std::vector<dec::Decoder::EncodedImage> im; drm.takeImages(im);
//      dec::DrmInfo info = drm.info();               // everything read
//
//  Tuning (what the radio must deliver)
//  ------------------------------------
//  Complex IQ at any rate from 48 kS/s up (2.048 MS/s is the usual), with
//  `centerHz` the stick's centre and `dialHz` the station's frequency: the
//  DRM signal's reference (carrier k = 0, its centre for 10 and 20 kHz
//  signals; the lower edge for the 4.5/5 kHz half-channel ones; 18/20 kHz
//  signals reach from -4.5/-5 to +13.5/+15 kHz of it), anywhere the stick
//  sees it.  The dial must be within about ±1.6 kHz of the true frequency
//  (±40 carriers are searched; mode-dependent).  Mirrored spectra (LSB
//  tuning, IQ leads swapped) are found and read too.  Robustness modes A-D;
//  spectrum occupancies 4.5/5/9/10/18/20 kHz (as the mode allows).  The IQ
//  scale does not matter.  A dial change resets the decoder.
//
//  How it works
//  ------------
//  A port of OFFgrid-SDR's JS DRM receiver (diddy-boy,
//  https://github.com/solariun/offgrid-sdr, index.html: the "drmlib" block
//  and `class DrmFront`), written from the ETSI standard following Dream
//  (TU Darmstadt, Volker Fischer, Alexander Kurpiers and others;
//  GPL-2.0-or-later, licenses/dream-COPYING.txt) for its tables and the
//  details of its cell mapping and coding:
//    - the front end mixes the dial down to 0 Hz, averages to about 240 kHz,
//      low-passes and resamples to exactly 48 kHz (DRM's clock);
//    - every 1.2 s (a superframe) of signal, the last 6 s (3.6 s at first,
//      for a quick name) are read FROM SCRATCH: robustness mode and timing
//      from the guard intervals, the frequency, the frame start from the
//      time references, the channel from the gain references, the FAC and
//      SDC (each checked by its CRC), then the main channel (cell
//      de-interleaving, 16/64-QAM multilevel decoding with Viterbi, energy
//      dispersal); fades and retunes recover by themselves, and the
//      multiplex frames, numbered by their place in the stream, join up
//      across the overlapping windows without repeats;
//    - the audio super frames give AAC access units (each with its DRM CRC
//      byte) for liboffgrid-codecs' AAC decoder (faad2's DRM mode), the
//      text message (its last 4 bytes), and the data streams: MOT SlideShow
//      pictures and Journaline pages (packet mode).
//  The sound comes out about 5 s behind the signal (long interleaving and
//  the windowing).  xHE-AAC (USAC) stations are recognised and shown, their
//  text and pictures decoded, but their sound is not played yet
//  ("xHE-AAC: not supported yet").  Not decoded (as the JS): hierarchical
//  64-QAM, unequal error protection (part A), CELP and HVXC speech.
//
//  Additions to the JS: see Drm.cpp, DrmRx.cpp and DrmData.cpp (each
//  marked "Addition to the JS"), among them all four services (the JS
//  showed one), the service's own audio stream, the main channel decoded
//  for text and pictures even when its sound can't be played, a wider
//  front-end filter for 18/20 kHz signals, and compressed Journaline pages.
//
//  How it fits in
//  --------------
//  Implements Decoder (Decoder.h).  Uses liboffgrid-dsp's FFT and
//  liboffgrid-codecs' AAC decoder (src/codecs/Aac.h, through DrmAac.cpp;
//  link liboffgrid-codecs).  No threads:
//  each 1.2 s of signal costs one analysis inside processIq() (see the test's
//  CPU line).  Signals for tests and the demo band: DrmEncoders.h.
//
//  Contributing
//  ------------
//  Keep in step with the JS (the comments follow its wording).  The tests
//  (tests/test_decoders_drm.cpp) run DrmEncoders.h's signals through it.
// ============================================================================
#ifndef OFFGRID_DECODERS_DRM_H
#define OFFGRID_DECODERS_DRM_H

#include <complex>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "decoders/Decoder.h"

namespace dec {

// One Journaline (news) page.
struct DrmJournalinePage {
    DrmJournalinePage() : id(0) {}
    unsigned id;          // object ID (0: the main menu)
    std::string type;     // "menu", "text", "title", "list"
    std::string title;
    std::string text;     // a text page's body ('\n' between paragraphs; *emphasis*)
    std::vector<std::pair<unsigned, std::string> > links;  // a menu's items: target ID, text
    std::vector<std::vector<std::string> > rows;           // a list's rows of cells
};

// One audio access unit, as taken from the audio super frame (for programs
// with a codec of their own, and for the tests).
struct DrmAccessUnit {
    DrmAccessUnit() : service(0), codec(0), frame(0) {}
    unsigned service;     // short ID 0..3
    int codec;            // 0 AAC (bytes: the DRM CRC byte, then the AU), 3 xHE-AAC (the frame)
    long long frame;      // the multiplex frame's number in the stream
    std::vector<unsigned char> bytes;
};

// Everything read about the signal and the station.
struct DrmInfo {
    DrmInfo()
        : signal(false), locked(false), metric(0), mode(0), occupancy(-1), occupancyKhz(0), longInterleave(false),
          hasMer(false), merDb(0), freqOffsetHz(0), mirrored(false), facOk(0), facTried(0), sdcOk(0), sdcTried(0),
          serviceId(0), codec(-1), sbr(false), stereoMode(0), rateCode(0), streamBytes(0), audioGood(0), audioBad(0),
          accessUnits(0), muxFrames(0), pictures(0), journalinePages(0) {}
    bool signal;               // DRM's OFDM structure is there
    bool locked;               // the FAC was read (in the last window)
    double metric;             // the guard-interval peak (>= 0.4: DRM)
    char mode;                 // 'A'..'D' (0: none)
    int occupancy;             // 0..5
    double occupancyKhz;       // 4.5, 5, 9, 10, 18, 20
    std::string msc;           // "64-QAM", "16-QAM", "64-QAM (hier.)"
    std::string sdc;           // "4-QAM", "16-QAM"
    bool longInterleave;       // long (2 s) or short (0.4 s)
    bool hasMer;
    double merDb;              // signal quality from the FAC cells
    double freqOffsetHz;       // the signal's offset from the dial
    bool mirrored;             // read with the spectrum turned round
    int facOk, facTried, sdcOk, sdcTried;   // last window
    std::string state;         // e.g. "station name read", "no DRM signal"
    // the selected service
    std::string label;
    unsigned serviceId;
    std::string language, programmeType;
    int codec;                 // 0 AAC, 1 CELP, 2 HVXC, 3 xHE-AAC, -1 unknown
    std::string codecName;
    bool sbr;
    int stereoMode;            // 0 mono, 1 parametric stereo, 2 stereo
    int rateCode;
    std::string audioText;     // "AAC, mono, 24 kHz"
    std::string unsupported;   // why the sound can't be played ("" if it can)
    std::string text;          // the text message
    int streamBytes;           // the audio stream's bytes a multiplex frame
    // counts since reset()
    long long audioGood, audioBad;   // AAC frames decoded / lost
    long long accessUnits, muxFrames;
    int pictures, journalinePages;
};

class DrmDecoder : public Decoder {
public:
    DrmDecoder();
    ~DrmDecoder();

    Kind kind() const { return Kind::Drm; }
    void reset();
    // DRM needs IQ: process() does nothing (use processIq()).
    void process(const float* audio, std::size_t n, double rate);
    void takeText(std::string& out);
    Status status() const;
    double audioCenterHz() const { return 0; }

    bool wantsIq() const { return true; }
    void processIq(const std::complex<float>* iq, std::size_t n, double rate, double centerHz, double dialHz);

    // The sound (liboffgrid-codecs' AAC decoder): interleaved
    // stereo float at audioRate().
    bool hasAudio() const;
    int audioRate() const;
    void takeAudio(std::vector<float>& stereo);

    // DRM's up to four services (Service::id = the short ID 0..3); the
    // first audio service plays unless another is chosen.
    std::vector<Service> services() const;
    bool selectService(unsigned id);

    // SlideShow pictures (MOT), JPEG or PNG, named as sent.
    void takeImages(std::vector<EncodedImage>& images);

    // ---- DRM's own --------------------------------------------------------------
    DrmInfo info() const;
    // The Journaline pages received from the station (by ID).
    std::vector<DrmJournalinePage> journaline() const;
    // Keep the access units for takeAccessUnits() (off by default).
    void setKeepAccessUnits(bool on);
    void takeAccessUnits(std::vector<DrmAccessUnit>& out);
    // Whether an AAC decoder is there (always, now liboffgrid-codecs is required).
    static bool aacAvailable();

private:
    class Impl;
    std::unique_ptr<Impl> d_;
    DrmDecoder(const DrmDecoder&);
    DrmDecoder& operator=(const DrmDecoder&);
};

}  // namespace dec

#endif
