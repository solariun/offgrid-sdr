// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/AviationEncoders.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  Deterministic signal synthesis for the aviation decoders (the counterpart
//  of Acars.h and Vdl2.h), for the demo band and the tests:
//
//    ACARS: the bits of a transmission (acarsBits: pre-key, + * SYN SYN SOH,
//    the message with odd parity, ETX, CRC, DEL) and their MSK audio
//    (synthAcars: 2400 bit/s, 1200/2400 Hz, built as offset-QPSK with
//    half-cosine pulses around 1800 Hz), i.e. what an AM receiver tuned to
//    the channel hears.  acarsDemoLoop() is the JS demo station: four bursts
//    in a 30 s loop, with the carrier gate (the carrier is only on during a
//    burst, as on the real channel; the JS demo AM-modulates the audio at
//    m = 0.8 under that gate on 131.550 MHz).
//
//    VDL2: AVLC frames (vdl2Address, vdl2Frame; ACARS in an I frame with
//    vdl2AcarsInfo), a burst's symbols (vdl2BurstSymbols: HDLC flags and
//    stuffing, RS(255,249) blocks interleaved, header with its parity,
//    scrambler, preamble, D8PSK Gray-coded phase steps) and their complex
//    baseband (vdl2Modulate: raised-cosine pulses, alpha 0.6, at any rate and
//    offset).  vdl2DemoBursts() is the JS demo plan: five bursts in a 30 s
//    loop on 136.975, 136.675 and 136.775 MHz.
//
//  Same input, same samples (no random numbers, no clocks).
//
//  Ported from OFFgrid-SDR (diddy-boy, https://github.com/solariun/offgrid-sdr)
//  index.html: acarsBits, acarsAudio, the demo's ACARS station; "a VDL Mode
//  2 transmitter (from the standard; checked against dumpvdl2)": vdlAddr,
//  vdlAcars, vdlFrame, vdlBurstSymbols, vdlModulate, and the demo's VDL2
//  plan.
//
//  How it fits in
//  --------------
//  Uses Acars.h (CRC, parity) and Vdl2.h (RS encoder, CRC, header parity,
//  preamble, Gray map); standard library only.  Not needed to decode.
//
//  Contributing
//  ------------
//  Keep the output deterministic: the tests compare decoded fields exactly.
//  Strict C++11.
// ============================================================================
#ifndef OFFGRID_DECODERS_AVIATIONENCODERS_H
#define OFFGRID_DECODERS_AVIATIONENCODERS_H

#include <complex>
#include <cstddef>
#include <string>
#include <vector>

namespace dec {

// ---------------------------------------------------------------------------
// ACARS
// ---------------------------------------------------------------------------
// One message to send (the JS acarsBits argument).  `reg` is padded with
// leading dots to 7 characters; `label` may contain '\x7f' (sent without
// parity: "_\x7f" reads back as "_d").  The STX and text part is sent when
// text or msgno is not empty; msgno (4) and flight (padded to 6) go first.
struct AcarsTx {
    AcarsTx() : mode("2"), ack("\x15") {}
    std::string mode, reg, ack, label, bid, msgno, flight, text;
};

// The bits of one ACARS transmission: 128 pre-key 1s, then the bytes
// 0xAB 0x2A SYN SYN SOH, the message, ETX, CRC (low first), DEL, each least
// significant bit first.  Values 0/1.
std::vector<unsigned char> acarsBits(const AcarsTx& m);
// Appends MSK audio for `bits` at `rate`, starting `lead` seconds in
// (silence before): peak about 1.  Returns the number of samples appended.
std::size_t synthAcars(const std::vector<unsigned char>& bits, double rate, std::vector<float>& out, double lead = 0);

// The JS demo station's messages: when (seconds into the 30 s loop) and what.
struct AcarsDemoItem {
    double at;
    AcarsTx msg;
};
std::vector<AcarsDemoItem> acarsDemoMessages();
// The demo's 30 s loop at `rate`: the audio (bursts, silence between) and
// the carrier gate (1 from 20 ms before each burst to 20 ms after, else 0).
void acarsDemoLoop(double rate, std::vector<float>& audio, std::vector<float>& gate);

// ---------------------------------------------------------------------------
// VDL2
// ---------------------------------------------------------------------------
typedef std::vector<unsigned char> Bytes;

// An AVLC address: the 24-bit address, its type (1 aircraft, 4 ground
// station), the status bit, and `last` (the address-extension bit of the
// last address byte, set on the source address).
Bytes vdl2Address(unsigned addr24, int type, int status = 0, bool last = false);
// ACARS for ACARS over AVLC: 0xFF 0xFF 0x01, the text with odd parity,
// ETX, CRC (from 0, low byte first), DEL.  `text` starts with the mode.
Bytes vdl2AcarsInfo(const std::string& text);
// The ACARS text the JS demo builds for an aircraft's downlink:
// "2" + ("." + reg) padded/cut to 7 + NAK + label + block + STX + msg +
// flight padded/cut to 6 + text.
std::string vdl2AcarsDownlink(const std::string& reg, const std::string& flight, const std::string& label,
                              const std::string& block, const std::string& msgNo, const std::string& text);
// A frame: destination, source, control, information, CRC (low first).
Bytes vdl2Frame(const Bytes& dst, const Bytes& src, unsigned char control, const Bytes& info);
// A burst's symbols: each the cumulative phase in eighths of a turn (the
// preamble's as listed, some negative).
std::vector<int> vdl2BurstSymbols(const std::vector<Bytes>& frames);
// Appends the symbols as complex samples at `rate`, offset from the centre
// by `f0` Hz, peak about `amp`: raised-cosine pulses (alpha 0.6, 6 symbols
// either side).  Returns the number of samples appended.
std::size_t vdl2Modulate(const std::vector<int>& syms, double rate, double f0, double amp,
                         std::vector<std::complex<float> >& out);
// Adds a burst into `iq` (which is at `rate`, centred on `centerHz`)
// starting at sample `at`, on channel `freqHz` (grows iq if needed).
void vdl2AddBurst(const std::vector<Bytes>& frames, double freqHz, double centerHz, double rate, double amp,
                  std::size_t at, std::vector<std::complex<float> >& iq);

// The JS demo plan (30 s loop): when, on which channel (Hz), which frames.
struct Vdl2DemoBurst {
    double at;
    double freqHz;
    std::vector<Bytes> frames;
};
std::vector<Vdl2DemoBurst> vdl2DemoBursts();

}  // namespace dec

#endif
