// ============================================================================
//  offgrid-codecs - AAC decoder for DAB+ and DRM (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/codecs/Aac.h
//  Layer:  Library liboffgrid-codecs (public header)
//
//  What this file does
//  -------------------
//  codec::AacDecoder - the AAC decoder digital radio needs: AAC-LC, with SBR
//  (HE-AAC) and parametric stereo (HE-AAC v2), 960- or 1024-sample frames,
//  and DRM's error-resilient AAC.  One access unit (AU) in, interleaved float
//  PCM out.  Underneath is a C++11 port of FAAD2 (files Aac*.cpp, namespace
//  codec::faad), configured as OFFgrid-SDR's browser version builds it.
//
//      codec::AacDecoder dec;
//      // DAB+: from the super frame header (dac_rate, sbr_flag,
//      // aac_channel_mode, ps_flag)
//      dec.init(codec::AacParams::dabPlus(48000, true, false, true));
//      // ... or from an AudioSpecificConfig:   dec.init(ascBytes, ascLen);
//      // ... or DRM, from the SDC's audio parameters:
//      //     dec.init(codec::AacParams::drmAudio(24000, true, false, true));
//      std::vector<float> pcm;
//      for (each AU) {
//          if (dec.decode(au, auLen, pcm)) {
//              play(pcm.data(), pcm.size() / dec.channels(), dec.sampleRate());
//          } else {
//              // a damaged AU: nothing came out (dec.lastErrorMessage() says
//              // why); the decoder is ready for the next one.  Fill the gap
//              // with silence of dec.frameSamples() frames if you need an
//              // unbroken timeline.
//          }
//      }
//
//  The output
//  ----------
//  Interleaved float samples, -1..1 (FAAD2's float output: 16-bit full scale
//  = 1.0; loud material can exceed 1 slightly, clip when converting).
//  Channels: always 2 for mono and stereo streams.  FAAD2, built with
//  parametric stereo, turns a mono stream into two identical channels,
//  because PS can switch on at any frame and the output must not change
//  shape; this port keeps that.  Rate: the AAC rate, or twice it with SBR.
//  The first AU of an SBR stream gives no samples (SBR's start-up delay):
//  decode() returns true with pcm empty.
//
//  What each system hands the decoder
//  ----------------------------------
//  DAB+ (ETSI TS 102 563): each AU as cut from the RS-corrected super frame
//    by its au_start offsets, WITHOUT its 2-byte CRC.  Params from the super
//    frame header: AacParams::dabPlus(dac_rate ? 48000 : 32000, sbr_flag,
//    aac_channel_mode, ps_flag).  Frames are 960 samples (core).
//  DRM (ETSI ES 201 980): each audio frame of the audio super frame,
//    PRECEDED by its 8-bit CRC byte (the byte from the higher-protected part)
//    - exactly what FAAD2's NeAACDecInitDRM mode expects (the CRC is read and
//    skipped, as FAAD2 does).  Params from the SDC: AacParams::drmAudio(audio
//    sampling rate 12000/24000/48000, SBR flag, stereo, PS).
//  Anything else: an AudioSpecificConfig (as in MP4 or LATM) with init(asc),
//    or an ADTS stream with initAdts() and then each ADTS frame to decode().
//
//  How it fits in
//  --------------
//  The DAB and DRM receivers (src/decoders/Dab*, Drm*) find the AUs; this
//  only turns them into sound.  No threads, no allocation per AU once warm,
//  no global state: one decoder per stream, decoders independent.  Stack use
//  per decode() peaks at about 110 kB (HE-AAC v2), fine for any thread.
//
//  Contributing
//  ------------
//  The FAAD2 port (Aac*.cpp other than this file's Aac.cpp) is kept close to
//  FAAD2 so tests can compare it sample-exactly with the original; change it
//  only to fix a bug, and say so in a comment marked "OFFgrid-SDR".  Strict
//  C++11, no warnings with -Wall -Wextra -pedantic-errors.
// ============================================================================
#ifndef OFFGRID_CODECS_AAC_H
#define OFFGRID_CODECS_AAC_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace codec {

// How a stream is coded, as DAB+ and DRM signal it (instead of an
// AudioSpecificConfig).
struct AacParams {
    unsigned sampleRate;   // the AAC (core) sampling rate in Hz: WITHOUT SBR's doubling
    unsigned channels;     // coded channels: 1 or 2 (a PS stream codes 1)
    bool sbr;              // SBR present: output at twice sampleRate
    bool ps;               // parametric stereo present (needs sbr)
    unsigned frameLength;  // samples per AAC frame: 960 (DAB+, DRM) or 1024
    bool drm;              // DRM's error-resilient AAC (CRC byte first; frameLength is 960)

    AacParams() : sampleRate(48000), channels(2), sbr(false), ps(false), frameLength(1024), drm(false) {}

    // DAB+ from its super frame header: dacRate = 48000 (dac_rate 1) or 32000
    // (dac_rate 0) - the OUTPUT rate; with SBR the AAC core runs at half.
    // stereo = aac_channel_mode.  960-sample frames.
    static AacParams dabPlus(unsigned dacRate, bool sbr, bool stereo, bool ps);

    // DRM from the SDC's audio information: audioRate = the AAC sampling rate
    // (12000, 24000 or 48000; with SBR the output is twice it); stereo = audio
    // mode 2 (with PS: audio mode 1, coded mono).
    static AacParams drmAudio(unsigned audioRate, bool sbr, bool stereo, bool ps);
};

class AacDecoder {
public:
    AacDecoder();
    ~AacDecoder();

    // Set up (or set up again: any earlier stream is forgotten).  False if the
    // configuration is not one this decoder handles.
    bool init(const std::uint8_t *asc, std::size_t len);      // MPEG-4 AudioSpecificConfig
    bool init(const std::vector<std::uint8_t> &asc);
    bool init(const AacParams &p);                              // explicit (DAB+, DRM)
    bool initAdts(const std::uint8_t *data, std::size_t len);  // convenience: from an ADTS frame's header;
                                                                // then decode() takes whole ADTS frames
    void close();                                               // back to not set up

    // Decode one access unit.  pcm is replaced by its samples (interleaved,
    // channels() per frame; may be empty, see the header).  False on a damaged
    // or truncated AU, or if not set up: pcm is then empty.  Never reads past
    // len, whatever the bytes say.  The decoder stays usable after any AU:
    // the states FAAD2 never leaves (a damaged AU that changed the stream's
    // element layout, error 21, or switched implicit SBR on, error 23, after
    // which every AU fails), and 8 failures in a row, make it rebuild itself
    // from its configuration (counted by restarts()).
    bool decode(const std::uint8_t *au, std::size_t len, std::vector<float> &pcm);
    bool decode(const std::vector<std::uint8_t> &au, std::vector<float> &pcm);

    bool ready() const { return h_ != 0; }
    unsigned sampleRate() const { return rate_; }      // output rate (after SBR), Hz
    unsigned channels() const { return ch_; }           // output channels (2 for mono, see the header)
    unsigned frameSamples() const { return frame_; }    // samples per channel per AU (once known)
    bool sbr() const { return sbr_; }                   // the last AU used SBR
    bool ps() const { return ps_; }                     // the last AU used parametric stereo
    std::size_t bytesConsumed() const { return used_; } // bytes the last AU took (ADTS: the frame)
    int lastError() const { return err_; }              // FAAD2's error code (0: none; -1: not set up / bad input)
    unsigned restarts() const { return restarts_; }     // times the decoder rebuilt itself after damage (see decode())
    const char *lastErrorMessage() const;

    // The AudioSpecificConfig init(AacParams) uses for a non-DRM stream
    // (AAC-LC, explicit hierarchical SBR/PS signalling, as DAB+ receivers do).
    static std::vector<std::uint8_t> makeAsc(const AacParams &p);

private:
    AacDecoder(const AacDecoder &);             // not copyable
    AacDecoder &operator=(const AacDecoder &);
    bool open();                                // (re)create FAAD2's state from the saved configuration

    enum Kind { None, Asc, Drm, Adts };
    void *h_;                         // the FAAD2 decoder state (codec::faad::NeAACDecStruct)
    Kind kind_;                       // how it was set up, kept to rebuild it:
    bool noSbr_;                      //   explicitly without SBR (init(AacParams), sbr false),
    std::vector<std::uint8_t> cfg_;   //   the AudioSpecificConfig or ADTS header,
    unsigned drmRate_;                //   or DRM's AAC rate
    unsigned char drmCode_;           //   and channel code (FAAD2's DRMCH_*)
    std::vector<std::uint8_t> buf_;   // the AU, copied with zero padding behind it
    unsigned rate_, ch_, frame_;
    bool sbr_, ps_;
    std::size_t used_;
    int err_;
    unsigned restarts_, failures_;
};

} // namespace codec

#endif // OFFGRID_CODECS_AAC_H
