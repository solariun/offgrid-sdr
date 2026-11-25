// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Vdl2.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  VDL Mode 2 receiver (VHF Data Link Mode 2, the digital successor to
//  ACARS) for 136.6-137 MHz: every 25 kHz VDL2 channel within the stick's IQ
//  span is decoded at once.
//
//  The signal: D8PSK at 10 500 symbols a second (3 bits a symbol, Gray
//  coded, the data in the phase step from one symbol to the next),
//  raised-cosine pulses.  One burst:
//
//      preamble (16 known symbols)  header (25 bits: 3 reserved, the
//      data length in bits (17, sent LSB first), 5 parity bits of a
//      single-error-correcting code)  data octets (HDLC: flags, bit
//      stuffing)  Reed-Solomon parity (RS(255,249) over GF(256), blocks
//      interleaved, the last block's parity shortened to 0/2/4/6 bytes)
//
//  everything after the preamble scrambled (LFSR, seed 0x6959).  The HDLC
//  frames are AVLC: destination and source addresses (24-bit ICAO aircraft
//  address or ground station, with a type), a control byte (I, S or U frame),
//  information, a CRC-16.  ACARS travels in I frames as 0xFF 0xFF 0x01 +
//  the ACARS block (parsed with acarsParse(), Acars.h); other I frames are
//  ATN network data (summarised), S frames acknowledgements, U frames link
//  management (XID) or link control.
//
//  Use alone:
//
//      #include "decoders/Vdl2.h"
//      dec::Vdl2Decoder vdl;
//      vdl.processIq(iq, n, 2048000, centerHz, dialHz);   // the stick's IQ
//      std::vector<dec::Vdl2Frame> f; vdl.takeFrames(f);
//      std::string t; vdl.takeText(t);                    // readable lines
//
//  Tuning: OFFgrid-SDR tunes the dial to 136.975 MHz (the common channel
//  worldwide) when VDL2 is chosen off 136.6-137 MHz; the stick's centre is
//  wherever the program put it (the TUI: a quarter span below the dial).
//  Each configured channel within 0.45 x rate of centerHz is decoded;
//  dialHz is not used.  The channels: setChannels(), or setLocation() for the
//  JS's choice by region (Europe: 136.675, .725, .775, .825, .875, .975;
//  North America: 136.650, .700, .800, .975; elsewhere 136.975 alone).
//
//  How it works (as the JS vdllib worker)
//  --------------------------------------
//   1. Shared first stage: the whole block of channels mixed down once,
//      around the middle of the channels in view, and reduced by 4 with a
//      second-order CIC (two boxcars).
//   2. Per channel: mixed to 0 Hz, a boxcar to about 256 kHz, a 31-tap
//      low-pass FIR (20 kHz) dividing by ~5: about 51.2 kHz (at 2.048 MS/s).
//   3. VdlRx: a low-pass to the signal's own width (9 kHz windowed sinc);
//      bursts found by their power against a running noise floor (16x to
//      start, ends after 3 symbols under 4x), each decoded whole:
//   4. timing and frequency offset from the preamble (the timing whose 15
//      phase steps best fit the known pattern, cubic interpolation), then
//      each phase step less the offset to the nearest eighth of a turn, the
//      offset tracked;
//   5. descramble, header (syndrome table), Reed-Solomon with the unsent
//      parity as erasures, de-interleave, HDLC (unstuff), CRC, AVLC parse.
//
//  Ported from OFFgrid-SDR (diddy-boy, https://github.com/solariun/offgrid-sdr)
//  index.html: the vdllib block (VdlRx, VdlChan, the worker's shared stage,
//  vdlRsDecode, vdlParseFrame), vdlChannels(), ICAO_BLOCKS / icaoCountry,
//  vdlLine / vdlBody (the readable lines).  The JS was written "from the
//  standard; checked against dumpvdl2" (dumpvdl2 by Tomasz Lemiech,
//  https://github.com/szpajder/dumpvdl2); the country blocks are ICAO's
//  allocation (Annex 10 vol. III) as tar1090 lists them.
//
//  How it fits in
//  --------------
//  Implements dec::Decoder (Decoder.h), Kind::Vdl2, an IQ decoder
//  (wantsIq() is true: call processIq(), not process()).  Uses Acars.h for
//  ACARS over AVLC.  Standard library only.  AviationEncoders.h synthesises
//  bursts for tests and the demo.
//
//  Contributing
//  ------------
//  Keep it a faithful port of the JS receiver; deviations are marked
//  "Addition to the JS" in the .cpp.  No allocation per sample (buffers grow
//  to the largest block seen, then are reused); strict C++11.  Run
//  tests/test_decoders_aviation.cpp after any change.
// ============================================================================
#ifndef OFFGRID_DECODERS_VDL2_H
#define OFFGRID_DECODERS_VDL2_H

#include "Acars.h"
#include "Decoder.h"

#include <complex>
#include <cstddef>
#include <deque>
#include <string>
#include <vector>

namespace dec {

// --- channels (the JS VDL_ALL and vdlChannels()) ---
// Every VDL2 channel the JS knows (136.650 .. 136.975 MHz), Hz.
const std::vector<double>& vdl2AllChannels();
// The JS choice by where you are: Europe, North America, else 136.975 alone.
std::vector<double> vdl2ChannelsFor(double latDeg, double lonDeg);
// The common channel worldwide; the dial the JS app goes to for VDL2.
const double vdl2CommonChannelHz = 136.975e6;

// --- coding pieces, shared with the encoders (AviationEncoders.h) ---
// RS(255,249) over GF(256) (field polynomial 0x187, first root 120; data in
// positions 0..248, parity 249..254).  Encode: parity of 249 data bytes.
void vdl2RsEncode(const unsigned char data[249], unsigned char parity[6]);
// Decode in place with erasures (positions); the number of symbols
// corrected, or -1 when it cannot be corrected.
int vdl2RsDecode(unsigned char c[255], const int* erasures, int nErasures);
// AVLC frame CRC (CRC-16/X.25: reflected 0x8408, from 0xFFFF, inverted).
unsigned vdl2Crc(const unsigned char* bytes, std::size_t n);
// The header's 5 parity bits as a syndrome of a 25-bit header word (0 = ok).
unsigned vdl2HeaderSyndrome(unsigned h25);
// The preamble (16 symbol phases, eighths of a turn) and the Gray map
// (3 bits -> phase step: index = step, value = bits).
extern const int vdl2Preamble[16];
extern const int vdl2Gray[8];

// An aircraft's country from its 24-bit ICAO address ("" when unknown).
const char* icaoCountry(unsigned addr24);

// One AVLC frame as received.
struct Vdl2Frame {
    Vdl2Frame()
        : freqHz(0), time(0), src(0), dst(0), srcType(0), dstType(0), kind('?'), control(0), isAcars(false), atn(false) {}
    double freqHz;            // the channel it came on
    double time;              // seconds of IQ since reset()
    unsigned src, dst;        // 24-bit addresses
    int srcType, dstType;     // 1 aircraft, 4/5 ground station, 7 all
    char kind;                // 'I' information, 'S' supervisory, 'U' unnumbered
    unsigned char control;
    bool isAcars;             // an I frame carrying ACARS: `acars` is filled
    AcarsMessage acars;
    bool atn;                 // an I frame with other (ATN) network data
    std::string what;         // S: "Receive Ready", "Reject", "Receive Not
                              // Ready", "Selective Reject"; U: "XID" or
                              // "link control"
    std::vector<unsigned char> bytes;   // the frame, CRC included

    // As the JS vdlLine without the time:
    // "136.975 · 400ABC (UK) → 10A0FA (ground) · G-OFFG · OG0123 · label H1"
    std::string headLine() const;
    // As the JS vdlBody: the ACARS text, "ATN data (...)", "Link management
    // (XID)", "Acknowledgement (Receive Ready)" or "Link control".
    std::string body() const;
};

// Parses one frame (addresses, control, information, the 2 CRC bytes; the
// CRC is not checked here).  False when shorter than 11 bytes.
bool vdl2ParseFrame(const unsigned char* f, std::size_t n, Vdl2Frame& out);

// ---------------------------------------------------------------------------
// The decoder (Kind::Vdl2).
// ---------------------------------------------------------------------------
class Vdl2Decoder : public Decoder {
public:
    Vdl2Decoder();
    ~Vdl2Decoder();

    Kind kind() const { return Kind::Vdl2; }
    void reset();
    // Audio is not used (VDL2 needs the IQ): ignored.
    void process(const float* audio, std::size_t n, double rate);
    bool wantsIq() const { return true; }
    void processIq(const std::complex<float>* iq, std::size_t n, double rate, double centerHz, double dialHz);
    // Each new frame as two lines (headLine, then the body indented):
    //   "VDL2 136.975 · 400ABC (UK) → 10A0FA (ground) · G-OFFG · OG0123 · label H1\n"
    //   "  OFFGRID-SDR DEMO. VDL2 POSITION 5212N 00008E FL350\n"
    void takeText(std::string& out);
    Status status() const;
    double audioCenterHz() const { return 0; }

    // --- settings (kept by reset()) ---
    // The channels to decode, Hz (those in the IQ span are).  Default: every
    // channel the JS knows (vdl2AllChannels()); see the .cpp.
    void setChannels(const std::vector<double>& hz);
    const std::vector<double>& channels() const { return chans_; }
    // The JS choice by region (vdl2ChannelsFor) as the channel list.
    void setLocation(double latDeg, double lonDeg);
    // "Messages only" (the JS vdlOnly): takeText leaves out frames that are
    // not I frames (acknowledgements, link management).  takeFrames and
    // frames() always have everything.
    void setMessagesOnly(bool on) { messagesOnly_ = on; }
    // How many frames frames() keeps (default 300, as the JS).
    void setMaxFrames(std::size_t n);

    // --- results ---
    const std::deque<Vdl2Frame>& frames() const { return frames_; }
    void takeFrames(std::vector<Vdl2Frame>& out);
    unsigned frameCount() const { return count_; }
    // Frames per channel since reset() (the JS channel buttons' counts).
    unsigned countOn(double hz) const;
    // Diagnostics: bursts seen, header/RS failures, CRC failures (all channels).
    unsigned bursts() const;
    unsigned rsFailures() const;
    unsigned crcFailures() const;
    // The channels being decoded in the last block (in view).
    std::size_t channelsInView() const { return inView_.size(); }

private:
    struct Chan;
    Vdl2Decoder(const Vdl2Decoder&);
    Vdl2Decoder& operator=(const Vdl2Decoder&);
    void gotFrame(double freq, const unsigned char* f, std::size_t n);
    void resetShared();

    std::vector<double> chans_;
    std::vector<Chan*> rx_;          // one per channel in chans_
    std::vector<std::size_t> inView_;
    bool messagesOnly_;
    std::size_t maxFrames_;

    // the shared first stage (the JS `vs`)
    double mid_, rate_;
    double mr_, mi_;                 // mixer rotator
    double cr_[2], ci_[2];           // CIC as a 7-tap triangle: this block, carried part
    double pr_, pi_;                 // previous block's carried part
    int k_;
    std::vector<float> sr_, si_;     // stage output (grown, reused)

    double t_;                       // seconds of IQ since reset()
    double lastFrameT_;
    unsigned count_;
    std::deque<Vdl2Frame> frames_;
    std::vector<Vdl2Frame> fresh_;
    std::string text_;
    std::vector<double> countF_;
    std::vector<unsigned> countN_;
};

}  // namespace dec

#endif
