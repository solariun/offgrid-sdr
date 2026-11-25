// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Decoder.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  The one interface every decoder implements, so a program can drive any
//  of them the same way:
//
//      std::unique_ptr<dec::Decoder> d = dec::makeDecoder(dec::Kind::Rtty);
//      for (;;) {
//          d->process(audio, n, 48000);      // mono demodulated audio, any rate
//          std::string text;
//          d->takeText(text);                 // new characters since last time
//          std::fputs(text.c_str(), stdout);
//          dec::Status s = d->status();       // locked? tones, speed, quality
//      }
//
//  Each decoder also has its own header (Cw.h, Rtty.h, Navtex.h (with DSC),
//  Psk.h, Hell.h) with its own settings; a program that needs one decoder
//  only includes that header.
//
//  Audio conventions (what the radio must deliver)
//  -----------------------------------------------
//  Mono float samples in about -1..1 at any rate from 8 kHz up, demodulated
//  in SSB (USB) or, for CW, with the beat note: the decoders look for their
//  tones in the audio (see audioCenterHz() and each header for where):
//    CW      the tone anywhere 250-1500 Hz (found by the decoder; the TUI
//            receiver's CW pitch is 700 Hz, the browser version's 600 Hz)
//    RTTY    a tone pair 170/200/425/450/850 Hz apart in 250-3000 Hz;
//            OFFgrid-SDR centres it on 1500 Hz (USB dial 1.5 kHz below the
//            listed frequency): 170 Hz shift = 1415/1585 Hz, mark the higher
//    NAVTEX  a 170 Hz pair around 1500 Hz, 1415/1585 Hz (dial 1.5 kHz below
//            518/490/4209.5 kHz)
//    DSC     the same signal as NAVTEX: 1415/1585 Hz, dial 1.5 kHz below the
//            DSC frequency (a 1 bit, "B", is the lower tone)
//    PSK     the tone at 1000 Hz ±50 Hz (dial 1 kHz below the PSK frequency)
//    HELL    the tone at 1000 Hz
//
//  How it fits in
//  --------------
//  The library depends only on the C++11 standard library and
//  liboffgrid-dsp's FFT (src/dsp/Fft.h).  It never touches audio devices,
//  threads or the terminal: call process() from any one thread.
// ============================================================================
#ifndef OFFGRID_DECODERS_DECODER_H
#define OFFGRID_DECODERS_DECODER_H

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace dec {

enum class Kind { Cw, Rtty, Psk, Navtex, Dsc, Hell };
const char* kindName(Kind kind);              // "CW", "RTTY", "PSK", "NAVTEX", "DSC", "HELL"
bool parseKind(const std::string& name, Kind& kind);  // case-insensitive

// What a decoder reports about the signal it is following.
struct Status {
    Status() : locked(false), signal(false), snrDb(0), quality(0), toneHz(0), shiftHz(0), baud(0), wpm(0), characters(0) {}
    bool locked;          // decoding (in sync with a signal)
    bool signal;          // a signal of the right kind is present
    double snrDb;         // signal to noise, dB (0 when unknown)
    double quality;       // 0..1: how clean the decoding is
    double toneHz;        // the tone followed (CW, PSK, HELL) or the mark tone (FSK)
    double shiftHz;       // FSK: mark - space (Hz)
    double baud;          // symbols per second (RTTY, NAVTEX, DSC, PSK)
    double wpm;           // CW words per minute
    unsigned characters;  // characters decoded since reset()
    std::string mode;     // detail, e.g. "PSK63", "RTTY 45.45/170", "NAVTEX"
    std::string info;     // one line for a status bar, e.g. "24 WPM  SNR 18 dB"
};

class Decoder {
public:
    virtual ~Decoder() {}

    virtual Kind kind() const = 0;
    // Forget everything (call after retuning).  Settings are kept.
    virtual void reset() = 0;
    // Feeds n mono samples at `rate` Hz (the rate may change between calls;
    // the decoder adapts).
    virtual void process(const float* audio, std::size_t n, double rate) = 0;
    // Appends the text decoded since the last call (UTF-8; '\n' starts a
    // line).  Complete messages (NAVTEX, DSC) arrive as formatted lines.
    virtual void takeText(std::string& out) = 0;
    virtual Status status() const = 0;
    // Where the decoder expects the signal in the audio, Hz (for tuning aids).
    virtual double audioCenterHz() const = 0;

    // Picture decoders (Hellschreiber): new columns of dots since the last
    // call, each a vector of brightness values 0..1 from top to bottom.
    virtual bool hasImage() const { return false; }
    virtual void takeColumns(std::vector<std::vector<float> >& columns) { (void)columns; }
};

// A decoder of the given kind with its default settings.
std::unique_ptr<Decoder> makeDecoder(Kind kind);
// All kinds, in menu order.
std::vector<Kind> allKinds();

}  // namespace dec

#endif
