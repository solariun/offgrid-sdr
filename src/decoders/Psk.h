// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Psk.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  PSK31 / PSK63 / PSK125 receivers: BPSK keyboard-to-keyboard ham text.
//
//  The signal is one steady tone.  Every bit (31.25, 62.5 or 125 a second)
//  the transmitter either keeps the tone's phase (a 1) or turns it over by
//  180 degrees (a 0); the strength eases down to zero through each turn
//  (raised cosine), so the signal's strength dips once per 0 bit.  The text
//  is Varicode: common letters have short codes, no code contains "00", and
//  characters are separated by "00".  Between overs the transmitter sends
//  "idle": nothing but 0s (a phase turn every bit).
//
//    dec::PskDecoder      one speed (constructor argument).
//    dec::PskAutoDecoder  all three speeds at once; prints the text of the
//                         one that matches the signal.  This is what
//                         makeDecoder(Kind::Psk) returns.
//
//  Use alone:
//
//      #include "decoders/Psk.h"
//      dec::PskAutoDecoder psk;
//      psk.process(audio, n, 48000);    // USB audio, the tone near 1000 Hz
//      std::string t; psk.takeText(t);  // new characters
//      psk.status().mode;               // "PSK31", "PSK63", "PSK125" or ""
//
//  Audio: the tone is expected at 1000 Hz (pskAudioHz); the decoder follows
//  it within +-70 Hz (meant for +-50 Hz: click near the signal and it pulls
//  in).  In OFFgrid-SDR the frequency shown is the signal's; the USB dial is
//  1 kHz below it, so the tone lands at 1000 Hz in the audio.
//
//  How it works (per speed)
//  ------------------------
//   1. Mix the audio down by the tracked tone frequency f (a local
//      oscillator) and low-pass it (4 one-pole stages, 40 Hz for PSK31,
//      wider in proportion to the speed): a complex baseband value z.
//   2. Decimate to 16 samples a bit.
//   3. Bit timing: the average strength |z| at each of the 16 positions in
//      the bit; the weakest is the flip (the bit boundary), the bit's centre
//      is 8 positions after it.
//   4. At the centre, compare z with the previous centre: a negative dot
//      product is a phase turn, a 0; otherwise a 1.
//   5. AFC: z squared loses the 180-degree turns; its slow rotation per
//      sample is twice the frequency error, so f is nudged by it (0.15 of
//      the error a bit; 0.4 while the squelch is shut, added in this port).
//   6. Squelch ("open"): opens on the idle pattern (clear dips for 8 bits
//      running) or on 20 bits of clean 0/180-degree phase changes with some
//      dip pattern (joined mid-transmission; that first character is
//      dropped); closes when the strength falls under a quarter of its peak,
//      the phase stops being clean, or (added in this port) 2 bit centres
//      in a row are far weaker than usual: the tone has gone.
//   7. Varicode: collect bits up to "00", look the code up.
//
//  PskAutoDecoder runs one PskDecoder per speed.  A decoder faster than the
//  signal never sees a "00" (each of the signal's bits looks like 2 or 4 of
//  its own, the first always a 1): it prints nothing.  A slower one may print
//  rubbish, but with poor bit timing quality.  So the fastest decoder that
//  produces characters with good timing (quality > 0.7, 3+ in the last 3 s)
//  is chosen; the characters it decoded before being chosen (the last 40)
//  are printed then, so the start of the text is not lost.
//
//  Ported from OFFgrid-SDR (diddy-boy, https://github.com/solariun/offgrid-sdr)
//  index.html: class PskDecoder, class PskAuto, VARICODE.
//
//  How it fits in
//  --------------
//  Implements dec::Decoder (Decoder.h).  Depends only on the C++11 standard
//  library.  PskEncoders.h synthesises test/demo signals for it.
//
//  Contributing
//  ------------
//  Keep it a faithful port of the JS decoder (the two should decode the same
//  audio the same way); no allocation per sample; strict C++11.  Run
//  tests/test_decoders_psk.cpp after any change.
// ============================================================================
#ifndef OFFGRID_DECODERS_PSK_H
#define OFFGRID_DECODERS_PSK_H

#include "Decoder.h"

#include <cstddef>
#include <string>

namespace dec {

// The audio frequency where PSK (and Hell) tones are expected, Hz.
const double pskAudioHz = 1000.0;

// Varicode for ASCII code c (0..127): a string of '0'/'1' (no "00" inside),
// or nullptr for c outside 0..127.
const char* pskVaricode(int c);
// The ASCII code for a Varicode string (without the "00" separator), or -1.
int pskVaricodeLookup(const char* bits, std::size_t n);

// PSK name for a speed: "PSK31" (31.25), "PSK63" (62.5), "PSK125" (125).
const char* pskModeName(double baud);

// ---------------------------------------------------------------------------
// One speed.
// ---------------------------------------------------------------------------
class PskDecoder : public Decoder {
public:
    // baud: 31.25 (PSK31), 62.5 (PSK63) or 125 (PSK125).
    explicit PskDecoder(double baud = 31.25);

    Kind kind() const { return Kind::Psk; }
    void reset();
    void process(const float* audio, std::size_t n, double rate);
    void takeText(std::string& out);
    Status status() const;
    double audioCenterHz() const { return pskAudioHz; }

    // --- extras (used by PskAutoDecoder, useful for tuning aids) ---
    double baud() const { return baud_; }
    double toneHz() const { return f_; }          // the tracked tone
    void setToneHz(double hz);                    // e.g. click-to-tune
    bool isOpen() const { return open_; }         // squelch open: decoding
    double timingQuality() const { return q_; }   // ~1 when the bit timing is good
    unsigned characters() const { return chars_; }
    // A new frequency: back to 1000 Hz, squelch closed, partial character
    // dropped (bit timing and text kept), as the JS PskAuto.retune() does.
    void retune();

private:
    void setRate(double rate);
    void rebuild();               // all running state from scratch (text kept)
    void sample(double zr, double zi);
    void addBit(int bit);
    bool stateFinite() const;

    double baud_;
    double rate_;
    int D_;                       // decimation: ~16 samples a bit
    double a_;                    // one-pole low-pass coefficient

    // running state (rebuilt by rebuild())
    double f_, ph_;
    double lp_[8];
    int k_;
    unsigned long t_;             // decimated samples since start
    double amp_[16];              // slow strength per bit position
    double af_[16];               // fast strength per bit position
    bool afInit_;
    bool havePrev_, haveSq_;
    double prevR_, prevI_, sqR_, sqI_;
    double ar2_, ai2_;            // AFC: z^2 rotation accumulator
    double q_, pk_, pqA_, floor_;
    double mAvg_;                 // bit-centre strength while open (tone-gone test)
    int openRun_, pqRun_;
    int lowRun_;                  // weak bit centres in a row (tone-gone test)
    bool open_, skipFirst_, floorInit_;
    char bits_[16];               // bits of the character being received
    int nbits_;

    // kept across a NaN rebuild
    std::string text_;            // decoded, not yet taken
    unsigned chars_;
    char lastChar_;               // last character decoded (for "\n\n")
};

// ---------------------------------------------------------------------------
// PSK31, PSK63 and PSK125 at once (Kind::Psk; makeDecoder's choice).
// ---------------------------------------------------------------------------
class PskAutoDecoder : public Decoder {
public:
    PskAutoDecoder();

    Kind kind() const { return Kind::Psk; }
    void reset();
    void process(const float* audio, std::size_t n, double rate);
    void takeText(std::string& out);
    Status status() const;
    double audioCenterHz() const { return pskAudioHz; }

    // --- extras ---
    // -1 while searching, else 0/1/2 = PSK31/PSK63/PSK125.
    int speedIndex() const { return cur_; }
    const PskDecoder& decoder(int i) const { return decs_[i]; }
    // As the JS app does on a new frequency: restart the speed search,
    // all decoders back to 1000 Hz, and start a new line of text.
    void retune();
    void setToneHz(double hz);

private:
    void restartSearch();

    PskDecoder decs_[3];
    std::string out_;             // text for takeText
    char lastOut_;                // last character appended to out_ (0: none)
    int cur_;
    double score_[3], lastAt_[3];
    std::string pend_[3];         // last 40 characters of each not-chosen decoder
    double t_;                    // seconds of audio seen
    unsigned since_[3];
    unsigned chars_;
    std::string scratch_;
};

}  // namespace dec

#endif
