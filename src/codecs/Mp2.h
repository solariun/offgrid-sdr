// ============================================================================
//  offgrid-codecs - MPEG Audio Layer II decoder for DAB (part of OFFgrid-SDR
//  TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from pl_mpeg's MP2 decoder (Dominic Szablewski, MIT; based
//  on kjmp2 by Martin J. Fiedler), with OFFgrid-SDR's DAB changes: see
//  licenses/pi_mpeg.  pl_mpeg's notice is in Mp2.cpp.
// ----------------------------------------------------------------------------
//  File:   src/codecs/Mp2.h
//  Layer:  Library liboffgrid-codecs (public header)
//
//  What this file does
//  -------------------
//  codec::Mp2Decoder - classic DAB's audio: MPEG-1 Layer II at 48 kHz and
//  MPEG-2 Layer II at its half rate, 24 kHz (also 32/44.1 and 16/22.05 kHz),
//  mono, stereo, joint stereo and dual channel.  One whole frame in, 1152
//  stereo sample frames out, plus the frame's ancillary bytes, where DAB
//  carries the programme-associated data (X-PAD and F-PAD).
//
//      codec::Mp2Decoder dec;
//      std::vector<float> pcm;
//      // DAB: a 48 kHz frame is one CIF's sub-channel bytes; a 24 kHz frame
//      // is two CIFs' joined (the first starts with the header).
//      if (dec.decode(frame, len, pcm)) {
//          play(pcm.data(), pcm.size() / 2, dec.sampleRate());   // stereo pairs
//          const codec::Mp2Decoder::DabPad &pad = dec.dabPad();
//          // pad.fpad[0..1]: F-PAD; frame[0 .. pad.xpadEnd): what the PAD
//          // reader scans backwards for X-PAD (ETSI EN 300 401 7.4)
//      }
//
//  The output
//  ----------
//  Always 2 channels, interleaved (a mono frame gives the same samples on
//  both), 1152 per channel, floats about -1..1 (pl_mpeg's scaling).  The
//  rate is the frame's own (sampleRate()).  The synthesis filter bank keeps
//  its state from frame to frame, as a stream decoder must.
//
//  Ancillary data
//  --------------
//  After a Layer II frame's audio bits comes ancillary data, up to the
//  frame's end (its size follows from the header: 144000 * kbit/s / rate
//  bytes, + 1 when padded).  ancillary() gives those bytes (from the first
//  whole byte after the audio).  DAB fills the end of every frame as
//      ... audio | X-PAD (read backwards) | ScF-CRC (2 or 4 bytes) | F-PAD (2)
//  and dabPad() splits that the way OFFgrid-SDR's browser version does: the
//  ScF-CRC is 2 bytes for MPEG-1 below 56 kbit/s mono / 112 kbit/s for two
//  channels, else 4.
//
//  How it fits in
//  --------------
//  The DAB receiver (src/decoders/Dab*) hands it each sub-channel frame; this
//  turns it into sound.  No threads, no allocation per frame once warm, no
//  global state; never reads past the bytes it is given.
//
//  Contributing
//  ------------
//  The arithmetic follows pl_mpeg's exactly (the tests compare sample for
//  sample with the original C).  Strict C++11, no warnings with -Wall
//  -Wextra -pedantic-errors.
// ============================================================================
#ifndef OFFGRID_CODECS_MP2_H
#define OFFGRID_CODECS_MP2_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace codec {

class Mp2Decoder {
public:
    enum Mode { Stereo = 0, JointStereo = 1, DualChannel = 2, Mono = 3 };

    // DAB's view of a frame's last bytes (see the header).
    struct DabPad {
        bool valid;              // the frame was long enough to hold them
        std::uint8_t fpad[2];    // F-PAD: the frame's last two bytes
        std::size_t scfCrcBytes; // ScF-CRC length before the F-PAD: 2 or 4
        std::size_t xpadEnd;     // offset in the frame where the X-PAD field ends (it is read backwards from here)
        DabPad() : valid(false), scfCrcBytes(0), xpadEnd(0) { fpad[0] = fpad[1] = 0; }
    };

    Mp2Decoder();

    // Decode one whole frame (it must start with the frame header; bytes past
    // the frame's size are ignored).  pcm is replaced by 1152 x 2 interleaved
    // samples.  False (pcm empty) when the header is not Layer II, the frame is
    // shorter than its header says, or the format is free/reserved.
    bool decode(const std::uint8_t *frame, std::size_t len, std::vector<float> &pcm);
    bool decode(const std::vector<std::uint8_t> &frame, std::vector<float> &pcm);

    // Forget the filter bank's history (after a gap or a retune).
    void reset();

    // The last decoded frame's format (0 / false before the first).
    unsigned sampleRate() const { return rate_; }
    unsigned bitrateKbps() const { return kbps_; }
    unsigned channels() const { return 2; }               // output channels (always 2)
    Mode mode() const { return (Mode)mode_; }             // the frame's channel mode
    bool mpeg2() const { return mpeg2_; }                 // MPEG-2 half sample rate
    bool hasCrc() const { return crc_; }                  // the frame carries a header CRC
    std::size_t frameBytes() const { return frameBytes_; } // the frame's size from its header
    std::size_t audioBytes() const { return audioBytes_; } // header + audio data, whole bytes

    // The bytes after the audio data to the frame's end, and DAB's split of them.
    const std::vector<std::uint8_t> &ancillary() const { return anc_; }
    const DabPad &dabPad() const { return pad_; }

    // The size of the frame a header announces, without decoding (0 if not a
    // Layer II header): to cut frames from a byte stream.  Needs 4 bytes.
    static std::size_t frameSize(const std::uint8_t *header, std::size_t len);

private:
    struct Quant { unsigned short levels; unsigned char group; unsigned char bits; };
    static const Quant kQuantTab[17];   // the quantizer classes (levels, grouped?, bits)

    // A bounded big-endian bit reader over one frame: reading past the end
    // gives zeros, it never touches memory beyond.
    struct Bits {
        const std::uint8_t *p;
        std::size_t nbits, pos;
        int read(int n);
    };

    const Quant *readAllocation(Bits &b, int sb, int tab3);
    void readSamples(Bits &b, int ch, int sb, int part);
    static void idct36(int s[32][3], int ss, float *d, int dp);

    unsigned rate_, kbps_;
    int mode_;
    bool mpeg2_, crc_;
    std::size_t frameBytes_, audioBytes_;
    std::vector<std::uint8_t> anc_;
    DabPad pad_;

    int bound_, vPos_;
    const Quant *allocation_[2][32];
    std::uint8_t scaleFactorInfo_[2][32];
    int scaleFactor_[2][32][3];
    int sample_[2][32][3];
    float D_[1024];
    float V_[2][1024];
    float U_[32];
};

} // namespace codec

#endif // OFFGRID_CODECS_MP2_H
