// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Acars.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  ACARS receiver: short text messages between aircraft and the ground on
//  the VHF airband (131.550 MHz and others), sent in AM.
//
//  The AM audio carries MSK at 2400 bits a second: tones of 1200 and
//  2400 Hz, i.e. 1800 Hz +- 600.  Characters are 7-bit ASCII with odd
//  parity, least-significant bit first.  One transmission:
//
//      pre-key (1s)  + *  SYN SYN SOH  mode  registration(7)  ack  label(2)
//      block-id  [STX [message number(4) flight(6), downlinks only] text]
//      ETX (or ETB)  CRC (2 bytes, low first)  DEL
//
//  The CRC is CRC-16 (polynomial 0x1021 reflected, 0x8408), from 0, over
//  everything from the mode to the ETX; run over the CRC bytes too it gives
//  0.  Only messages whose CRC checks out are reported (a single character
//  with a bad parity bit, or two, are repaired first by trying every bit
//  until the CRC fits), so noise never turns into text.
//
//  Use alone:
//
//      #include "decoders/Acars.h"
//      dec::AcarsDecoder acars;
//      acars.process(audio, n, 48000);     // AM-demodulated audio
//      std::vector<dec::AcarsMessage> m; acars.takeMessages(m);
//      std::string t; acars.takeText(t);   // the same, as readable lines
//
//  Audio and tuning: AM, the dial ON the ACARS channel, with an AM filter
//  wide enough for the tones: OFFgrid-SDR uses 8 kHz (acarsAmBandwidthHz;
//  the MSK reaches +-3.6 kHz).  The channels the JS app knows are in
//  acarsChannels() (131.550 first: the app jumps there when ACARS is chosen
//  off a channel; "AUTO" picks ACARS within 5 kHz of one).
//
//  How it works (as the JS AcarsDecoder, the scheme acarsdec also uses)
//  ------------------------------------------------------------------
//   1. Average down to about 12 kHz (48 kHz audio / 4), remove DC.
//   2. Mix with a local 1800 Hz oscillator (complex), keep a ring of the
//      last ~1 symbol pair of products.
//   3. Every three quarters of an 1800 Hz cycle (one bit), a matched filter
//      (half a 600 Hz cosine, 12x oversampled so it lines up with the exact
//      bit instant) gives a complex value; bits come alternately from its
//      real and imaginary parts, the sign flipping every second pair
//      (offset-QPSK view of MSK).
//   4. A phase-locked loop keeps the oscillator on the signal (quicker while
//      looking for a message, steadier inside one).
//   5. Framing: SYN searched bit by bit (an inverted SYN means the loop
//      locked upside down: the bit sign is flipped from there), the second
//      SYN, SOH, then characters up to ETX/ETB, the two CRC bytes.  (This
//      port runs a second framer on the other axis, for a loop that left the
//      pre-key half a bit off; see Acars.cpp.)
//   6. CRC check (with parity repair), then the fields (acarsParse()).
//
//  Ported from OFFgrid-SDR (diddy-boy, https://github.com/solariun/offgrid-sdr)
//  index.html: class AcarsDecoder, acarsCrc, ACARS_LABELS, ACARS_FREQS, and
//  the ACARS box (acarsLine / acarsBody) for the text lines.  The JS credits
//  the demodulator scheme to the open-source acarsdec (Thierry Leconte).
//
//  How it fits in
//  --------------
//  Implements dec::Decoder (Decoder.h), Kind::Acars.  acarsParse() is also
//  used by the VDL2 decoder (Vdl2.h) for ACARS carried over AVLC.  Depends
//  only on the C++11 standard library.  AviationEncoders.h synthesises test
//  and demo signals for it.
//
//  Contributing
//  ------------
//  Keep it a faithful port of the JS decoder (the two should decode the same
//  audio the same way); deviations are marked "Addition to the JS" in the
//  .cpp.  No allocation per sample; strict C++11.  Run
//  tests/test_decoders_aviation.cpp after any change.
// ============================================================================
#ifndef OFFGRID_DECODERS_ACARS_H
#define OFFGRID_DECODERS_ACARS_H

#include "Decoder.h"

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

namespace dec {

// The audio frequency at the centre of the ACARS tones (1200/2400 Hz), Hz.
const double acarsAudioHz = 1800.0;
// The AM filter OFFgrid-SDR uses in ACARS mode (JS ACARS_BW), Hz.
const double acarsAmBandwidthHz = 8000.0;

// The ACARS channels the JS app knows (ACARS_FREQS), Hz; 131.550 MHz first.
const std::vector<double>& acarsChannels();

// CRC-16 as ACARS uses it (reflected 0x8408, no final xor), continuing from
// `crc` (start with 0).  Over a message plus its two CRC bytes it is 0.
unsigned acarsCrc(const unsigned char* bytes, std::size_t n, unsigned crc = 0);
// `c` (7-bit) with the odd-parity bit set as bit 7.
unsigned char acarsOddParity(unsigned char c);
// The meaning of a label, or "" when the JS table (ACARS_LABELS) lacks it:
// "SQ", "_d", "H1", "Q0", "5Z".
const char* acarsLabelMeaning(const std::string& label);

// One ACARS message.
struct AcarsMessage {
    AcarsMessage() : ack(), down(false), levelDb(0), time(0), freqHz(0) {}
    std::string mode;        // mode character, e.g. "2"
    std::string reg;         // aircraft registration, the leading dots dropped
                             // ("" for a ground station's squitter)
    std::string ack;         // "NAK", or the acknowledged block id
    std::string label;       // two characters, e.g. "H1"; a DEL second
                             // character reads as "d" ("_d")
    std::string blockId;     // block id: a digit on downlinks (from aircraft)
    bool down;               // from an aircraft (block id 0-9)
    std::string msgNo;       // message number (downlinks with text), e.g. "M01A"
    std::string flight;      // flight (downlinks with text), e.g. "OG0123"
    std::string text;        // the text: CR dropped, blank lines merged, trimmed
    double levelDb;          // matched-filter level, dB (ACARS decoder only)
    double time;             // seconds of input since reset()
    double freqHz;           // the channel, when the caller knows it (VDL2), else 0

    // As the JS ACARS box: "G-OFFG  OG0123  H1 from aircraft" (reg or
    // "(no reg)", flight if any, label, direction).
    std::string headLine() const;
    // The body: the text, or "(label meaning)", or "(no text)".
    std::string body() const;
};

// Parses an ACARS block: the bytes from the mode character up to, not
// including, the ETX/ETB (CRC not included; parity bits may be set: they are
// masked off).  At least 12 bytes (mode .. block id).  As the JS
// AcarsDecoder.finish(): reg without dots, label DEL -> 'd', ack 0x15 ->
// "NAK", STX at byte 12 starts the text, downlinks carry the message number
// and flight first; non-printable characters dropped (CR/LF kept as line
// breaks).  False when too short.
bool acarsParse(const unsigned char* bytes, std::size_t n, AcarsMessage& out);

// ---------------------------------------------------------------------------
// The decoder (Kind::Acars).
// ---------------------------------------------------------------------------
class AcarsDecoder : public Decoder {
public:
    AcarsDecoder();

    Kind kind() const { return Kind::Acars; }
    // Forgets the signal state (as the JS reset(); the app calls it on a new
    // frequency).  The message list and counters are kept; clearMessages()
    // empties them.
    void reset();
    void process(const float* audio, std::size_t n, double rate);
    // Each new message as (JS acarsLine / acarsBody, without the time):
    //   "ACARS  G-OFFG  OG0123  H1 from aircraft  [M01A]  (message to or
    //    from the crew or the aircraft's systems)\n"      (one line)
    //   "  OFFGRID-SDR DEMO. CRUISE FL350, ...\n"
    // The message number in [], the label's meaning in () when the JS table
    // has one and there is text (else the body is the meaning); every body
    // line indented by two spaces.
    void takeText(std::string& out);
    Status status() const;
    double audioCenterHz() const { return acarsAudioHz; }

    // --- results ---
    // The last messages (oldest first), at most 200 (as the JS).
    const std::deque<AcarsMessage>& messages() const { return msgs_; }
    // Appends the messages decoded since the last call.
    void takeMessages(std::vector<AcarsMessage>& out);
    void clearMessages();
    unsigned okCount() const { return ok_; }    // messages that passed the CRC
    unsigned badCount() const { return bad_; }  // blocks rejected
    // Tagged onto each new message's freqHz (e.g. the dial), 0 = unknown.
    void setFrequency(double hz) { freq_ = hz; }

private:
    void setRate(double rate);
    void restartState();      // the JS reset() body
    // One byte framer (SYN search, SOH, characters, CRC); two run at once.
    struct Framer {
        Framer() : st(0), nb(1), reg(0), inv(false), perr(0), crc0(0), lvlSum(0), lvlN(0) {}
        int st, nb;               // state; bits to the next byte decision
        unsigned reg;             // the last 8 bits
        bool inv;                 // bit sign flipped (an inverted SYN seen)
        std::vector<unsigned char> blk;   // the message, mode .. ETX
        int perr;                 // characters with bad parity
        unsigned char crc0;
        double lvlSum;            // level since SOH
        unsigned lvlN;
    };
    void sample(double v);
    void bit(Framer& f, int which, double v);
    void restart(Framer& f, int which);
    void begin(Framer& f);
    void byte(Framer& f, int which, unsigned r);
    static bool crcOk(const std::vector<unsigned char>& blk, unsigned char c0, unsigned char c1);
    static bool repair(std::vector<unsigned char>& blk, unsigned char c0, unsigned char c1);
    void finish(Framer& f, unsigned char c0, unsigned char c1);

    double rate_, fs_;
    int D_, FL_, OV_;
    std::vector<double> h_;       // matched filter, FL*OV+1 taps
    std::vector<double> ring_;    // FL complex products (re, im)
    int idx_;

    double p_, clk_, df_;
    unsigned S_;                  // bits since start: axis (bit 0), sign (bit 1)
    Framer fr_[2];                // [0] the JS's, [1] the other axis
    std::vector<unsigned char> last_;   // the last message (for the duplicate check)
    double acc_, dc_;
    int k_;
    double t_;                    // seconds since reset (for message times)
    double lastMsgT_;

    std::deque<AcarsMessage> msgs_;
    std::vector<AcarsMessage> fresh_;
    std::string text_;
    unsigned ok_, bad_;
    double freq_;
};

}  // namespace dec

#endif
