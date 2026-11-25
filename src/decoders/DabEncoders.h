// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DabEncoders.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  The demo DAB multiplex, as OFFgrid-SDR's browser version puts it in its
//  demo band (on 12A, 223.936 MHz): a mode I OFDM signal at 2.048 MS/s,
//  "OFFgrid DAB" (ensemble ID 0FF6) with
//    - "OFFgrid Plus" (SId F0D1): the demo jingle in DAB+ (AAC), sub-channel
//      1, 48 kbit/s EEP 3-A, with station text (DLS) and a SlideShow in its
//      PAD; programme type Pop Music;
//    - "OFFgrid Classic" (SId F0D2): the jingle in the original DAB (MPEG
//      Layer II, 48 kbit/s), sub-channel 2, UEP index 7 (level 3); Other
//      Music;
//    - an SPI data service (E1F0D9FF, application 7): the two stations'
//      logos, MOT in packet mode (address 1), sub-channel 3, 16 kbit/s;
//    - FIG 0/0 with the CIF count (4 more each frame), so the receiver's
//      forgiving re-lock can count frames lost in a gap;
//    - the rest of the multiplex's capacity filled with random bits.
//  It loops every 120 frames (11.52 s), which hold both stations' audio
//  exactly.
//
//      dec::DabDemoMultiplex mux;
//      std::vector<std::complex<float> > iq(65536);
//      mux.generate(&iq[0], iq.size());     // the next samples, endlessly
//      dab.processIq(&iq[0], iq.size(), 2048000, 223.936e6, 223.936e6);
//
//  How it fits in
//  --------------
//  A port of OFFgrid-SDR's JS (diddy-boy, https://github.com/solariun/
//  offgrid-sdr, index.html, "dablib": dabDemoSignal), from the same
//  embedded programme data (DabDemoData.cpp: the "dabdemoplus",
//  "dabdemomp2" and "dabdemospi" blocks, whose heavy parts were done ahead:
//  the super frames' Reed-Solomon parity, the MP2 encoding, the SPI
//  packets).  Here: the FIC, the convolutional coding, puncturing and time
//  interleaving once (in the constructor, about 15 ms), then each 96 ms
//  frame built on demand by inverse FFT (1 s of signal in about 20 ms).
//  Uses the receiver's own tables (DabInternal.h), so the two can't
//  drift apart.  Needs liboffgrid-dsp's FFT.
//
//  Contributing
//  ------------
//  Keep it in step with dabDemoSignal, so the TUI's demo matches the
//  browser's.  Run tests/test_decoders_dab.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_DAB_ENCODERS_H
#define OFFGRID_DECODERS_DAB_ENCODERS_H

#include <complex>
#include <cstddef>
#include <memory>
#include <vector>

namespace dec {

class DabDemoMultiplex {
public:
    DabDemoMultiplex();
    ~DabDemoMultiplex();

    static const int kFrames = 120;          // the loop: 120 frames of 96 ms
    static const int kFrameSamples = 196608; // samples a frame at 2.048 MS/s
    static double sampleRate() { return 2048000.0; }
    // What it carries (for tests and the demo's labels).
    static const unsigned kEnsembleId = 0x0FF6;
    static const unsigned kPlusSid = 0xF0D1;
    static const unsigned kClassicSid = 0xF0D2;

    // Transmission frame k (any k >= 0; frames repeat every kFrames, the CIF
    // count keeps counting): kFrameSamples samples, the null symbol first,
    // written to out (resized).  RMS about 0.87 (sqrt(0.75)).
    void frame(long k, std::vector<std::complex<float> >& out);
    // A continuous stream: the next n samples, frame after frame.
    void generate(std::complex<float>* out, std::size_t n);
    // Where generate() is: samples since the start.
    unsigned long long position() const { return pos_; }
    void restart() { pos_ = 0; }

    // The programme data as sent (each CIF's bytes before coding):
    // DAB+ super frames (480 CIFs x 144 bytes), MP2 frames (480 x 144), SPI
    // packets (480 x 48).  For tests that compare what is decoded.
    const std::vector<unsigned char>& plusData() const;
    const std::vector<unsigned char>& mp2Data() const;
    const std::vector<unsigned char>& spiData() const;

    struct Impl;

private:
    DabDemoMultiplex(const DabDemoMultiplex&);
    DabDemoMultiplex& operator=(const DabDemoMultiplex&);
    std::unique_ptr<Impl> impl_;
    unsigned long long pos_;
};

}  // namespace dec

#endif
