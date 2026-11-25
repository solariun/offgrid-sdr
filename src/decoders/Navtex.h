// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Navtex.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  Two maritime decoders that share one receiver (170 Hz FSK at 100 baud,
//  every symbol sent twice):
//
//  dec::NavtexDecoder - NAVTEX (SITOR-B / CCIR 476 FEC): maritime safety
//    and weather text on 518 kHz (English), 490 kHz (local languages) and
//    4209.5 kHz.  7-bit characters, each with exactly four 1 bits (a
//    damaged one shows), each sent twice five character slots apart (DX
//    then RX): if the first copy is bad, the second is used.  No start or
//    stop bits: the character boundary is found by trying all seven (and
//    both polarities) and keeping the one where codes come out valid.
//    Messages run "ZCZC B1B2B3B4" ... "NNNN"; B1 = station, B2 = subject
//    (A navigational warning, B meteorological warning, E forecast ...),
//    B3B4 = number.  takeText() gives each complete message:
//
//        Station S · Navigational warning · No. 23
//        <the message text>
//        <empty line>
//
//  dec::DscDecoder - HF DSC (Digital Selective Calling, ITU-R M.493):
//    ships' automated distress, urgency, safety and routine calls on
//    2187.5, 4207.5, 6312, 8414.5, 12577 and 16804.5 kHz.  10-bit symbols:
//    7 data bits (least significant first) and 3 check bits (the number of
//    zeros, most significant first); each symbol twice, five slots apart.
//    The polarity comes from the phasing symbols (125, 104-111) that start
//    every call.  takeText() gives one line per call, e.g.
//
//        Distress (sinking) · all ships · from MMSI 232123456
//        Safety · all ships · from MMSI 235009876
//
//  Use alone:
//
//      #include "decoders/Navtex.h"
//      dec::NavtexDecoder nav;              // or dec::DscDecoder dsc;
//      nav.process(audio, n, 48000);        // USB audio, the pair ~1500 Hz
//      std::string t; nav.takeText(t);      // complete messages
//      nav.messages();                      // also as structures
//
//  How it works (shared receiver)
//  ------------------------------
//   1. Tone pair: every ~0.3 s an FFT (4096 points at ~8 kHz); the space
//      frequency f in 250-2830 Hz where the power within +-20 Hz of f and
//      of f + 170 Hz (the smaller of the two) is greatest, at least 6 dB
//      over the noise.  Bits are read once a pair has been found, even
//      while it is out of view (DSC's dot pattern hides it).
//   2. Two tone detectors (mix down, two one-pole low-passes at 75 Hz),
//      discriminator with hysteresis (+-0.1) -> level (1 = higher tone).
//   3. Bit clock at 100 baud, nudged towards level changes (15% of the
//      error each time); a bit is read in the middle of each bit time.
//   4. NAVTEX: for each of the 7 boundaries x 2 polarities, how many of the
//      last 40 codes read there were valid (four 1s); sync on the best
//      (>= 26 of 40, about 2 s; lost below 16), ties going to the one that
//      reads the phasing signals alpha/beta.  DSC: the same over 10
//      boundaries with the DSC check, 12 symbols (a call is only ~5 s).
//   5. DX/RX: which slot parity holds first copies is learned from codes
//      repeated five slots later, or from the phasing (NAVTEX: beta in DX,
//      alpha in RX; DSC: 125 in DX, 104-111 in RX).
//
//  Changes from the browser version (each marked in Navtex.cpp): the tone
//  search of step 1 (the browser: strongest single-bin peaks, which wander
//  with 100 baud data), reading bits while the pair is out of view, the
//  phasing tie-break and phasing-based DX/RX parity, and DSC forgetting
//  the DX/RX evidence when sync is lost.  Without them the first message
//  or call heard from its start was lost (the browser's demo loops, so it
//  had always seen the signal before).
//
//  How it fits in
//  --------------
//  Implements dec::Decoder.  Ported from OFFgrid-SDR's NavtexDecoder and
//  DscDecoder (diddy-boy, index.html); DscDecoder derives from
//  NavtexDecoder there and here.  Depends on Codes.h and src/dsp/Fft.h.
//  No allocation per sample (per character / per message only).
//
//  Contributing
//  ------------
//  Keep the thresholds in step with the browser version and run
//  tests/test_decoders_fsk.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_NAVTEX_H
#define OFFGRID_DECODERS_NAVTEX_H

#include "Decoder.h"
#include "../dsp/Fft.h"

#include <complex>
#include <deque>
#include <string>
#include <vector>

namespace dec {

// A NAVTEX message (as received so far).
struct NavtexMessage {
    NavtexMessage() : idDone(false), done(false) {}
    std::string id;     // "SA23": station, subject, number
    std::string text;   // the body (complete once done)
    bool idDone;
    bool done;          // NNNN received
    // "Station S · Navigational warning · No. 23"
    std::string header() const;
};

// A DSC call.
struct DscCall {
    std::string format;    // "Distress", "All ships", "Individual call", ...
    std::string category;  // "Distress", "Urgency", "Safety", "Routine" or ""
    std::string to;        // "all ships", "ships in an area", "MMSI 123456789"
    std::string from;      // nine digits ("?" for unreadable ones)
    std::string nature;    // nature of distress, e.g. "sinking", or ""
    std::string text;      // the one-line summary takeText() gives
    std::vector<int> symbols;   // the call's symbols as received (-1 = damaged)
};

class NavtexDecoder : public Decoder {
public:
    NavtexDecoder();

    Kind kind() const override { return Kind::Navtex; }
    void reset() override;
    void process(const float* audio, std::size_t n, double rate) override;
    void takeText(std::string& out) override;
    Status status() const override;
    double audioCenterHz() const override { return (mark_ + space_) / 2; }

    // Forget the tones and the sync (after retuning), keep the messages.
    void retune();

    // ---- readings ----
    const std::deque<NavtexMessage>& messages() const { return msgs_; }   // the last 20
    const std::string& rawText() const { return raw_; }   // every character (last ~6000)
    double markHz() const { return mark_; }
    double spaceHz() const { return space_; }
    bool tonesFound() const { return lock_; }
    bool inSync() const { return sync_ >= 0; }
    unsigned repaired() const { return fixed_; }   // first copy bad, second used
    unsigned lostChars() const { return lost_; }   // both copies bad

protected:
    // A window of the last `len` 0/1 values and their sum.
    struct Window {
        Window() : pos(0), count(0), sum(0) {}
        unsigned char v[40];
        int pos, count, sum;
        void clear() { pos = count = sum = 0; }
        void push(int x, int len) {
            if (count == len) { sum -= v[pos]; } else ++count;
            v[pos] = static_cast<unsigned char>(x); sum += x;
            pos = (pos + 1) % len;
        }
    };

    virtual void bit(int b);
    // The last slot values (ring of 8; s(1) = newest, s(6) = 5 before it).
    void pushSlot(int v) { slotRing_[slotPos_] = v; slotPos_ = (slotPos_ + 1) & 7; if (slotLen_ < 64) ++slotLen_; }
    int slotAgo(int k) const { return slotRing_[(slotPos_ - k + 8) & 7]; }

    double rate_;
    double mark_, space_;
    bool lock_;
    bool found_;    // the pair has been found since reset()/retune()
    double snrDb_;
    int sync_;
    unsigned long long nb_;
    Window vHist_[20];
    Window pHist_[14];   // NAVTEX: phasing signals seen per boundary/polarity
    Window mHist_[2];
    int slotRing_[8];
    int slotPos_, slotLen_;
    unsigned slotN_;
    unsigned fixed_, lost_, chars_;
    std::string out_;

private:
    void setRate(double rate);
    void tuneUp();
    void slot(int code);
    void character(int code);

    int lostAt_, level_;
    double ph_, spb_;
    int bits_;
    bool figs_;
    std::string raw_;
    std::deque<NavtexMessage> msgs_;
    bool curOpen_;
    double phaseVote_[2];   // phasing beta/alpha seen at each slot parity (decaying)
    // tone detectors
    double pmRe_, pmIm_, psRe_, psIm_, rmRe_, rmIm_, rsRe_, rsIm_, rotMark_, rotSpace_;
    int renorm_;
    double mI_, mQ_, sI_, sQ_, mI2_, mQ2_, sI2_, sQ2_;
    // analysis
    int D_;
    double fs2_, decim_;
    int accN_;
    std::vector<float> buf_, win_;
    int bp_, since_;
    dsp::detail::Fft fft_;
    std::vector<std::complex<float> > fbuf_;
    std::vector<double> P_, sorted_, cum_;
};

class DscDecoder : public NavtexDecoder {
public:
    DscDecoder();

    Kind kind() const override { return Kind::Dsc; }
    void reset() override;
    Status status() const override;

    const std::deque<DscCall>& calls() const { return calls_; }   // the last 20

protected:
    void bit(int b) override;

private:
    static int unpack(int w);
    void dscSlot(int v);
    void symbol(int v);
    void flush();
    void parse(const std::vector<int>& s);

    int w10_;
    int pol_;
    bool polLocked_;
    double phHits_[2];
    int dxPar_;   // -1 = not known
    std::vector<int> sym_;
    unsigned syms_;
    std::deque<DscCall> calls_;
};

}  // namespace dec

#endif
