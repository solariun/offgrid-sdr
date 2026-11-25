// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Ax25Encoders.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  Builds AX.25 / APRS packets and synthesises their audio, for the demo
//  band and the tests (the counterpart of Ax25.h):
//
//      std::vector<unsigned char> f = dec::ax25UiFrame("N0CALL-9", "APRS",
//          {"WIDE1-1", "WIDE2-1"}, "!5126.40N/00007.30W[Out walking");
//      std::vector<float> a;
//      dec::synthAx25(f, 48000, a);               // AFSK 1200, appends
//      dec::synthAx25(f, 48000, a, dec::Ax25Tx::hf300());
//
//  Frame: addresses (a "*" after a digipeater sets its H bit, "has
//  repeated"), control 0x03 (UI), PID 0xF0, the information field; as the
//  JS aprsFrame (OFFgrid-SDR, diddy-boy).  HDLC: AX25Toolkit's
//  hdlc::Encoder (Gustavo Campos; FCS from Dire Wolf, WB2OSZ): flags, the
//  frame and its FCS bit-stuffed, NRZI (a 0 changes the tone).
//
//  Audio: phase-continuous AFSK with exact (fractional) bit timing, 1200 Bd
//  1200/2200 Hz or 300 Bd 1600/1800 Hz, the first bit on a mark tone at
//  phase 0; or G3RUH 9600 baseband (scrambled x^17 + x^12 + 1, NRZ levels
//  with raised-cosine edges a quarter of a bit wide).  `toneScale` and
//  `clockScale` multiply the tones and the bit rate (to test a transmitter
//  that is off by a few per cent).  No noise.  Same input, same samples.
//
//  Also: the JS aprsMicEEncode (a Mic-E position: latitude and message in
//  the destination) and the demo band's APRS packets (as the JS demo
//  station on 144.800 MHz).
//
//  How it fits in
//  --------------
//  Uses ax25/hdlc.h and ax25/ax25frame.h; standard library only.  Not needed
//  to decode.
//
//  Contributing
//  ------------
//  Keep the output deterministic (no random numbers, no clocks): the tests
//  compare decoded packets exactly.  Strict C++11.
// ============================================================================
#ifndef OFFGRID_DECODERS_AX25ENCODERS_H
#define OFFGRID_DECODERS_AX25ENCODERS_H

#include <cstddef>
#include <string>
#include <vector>

namespace dec {

// The bytes of a UI frame (no FCS): destination, source, path ("CALL-SSID";
// a trailing "*" on a path entry sets its H bit), 0x03, 0xF0, info.
std::vector<unsigned char> ax25UiFrame(const std::string& source, const std::string& destination,
                                       const std::vector<std::string>& path, const std::string& info);

// The FCS of a frame (CRC-16-CCITT as AX.25 sends it: low byte first on
// the air), e.g. to build a frame with a wrong one for a test.
unsigned short ax25Fcs(const unsigned char* bytes, std::size_t n);

// The on-air bit levels of a frame (1 = mark): preFlags flags, the frame and
// its FCS bit-stuffed, postFlags flags, all NRZI-encoded.  `appendFcs`
// false sends the bytes as given (the caller supplies the 2 FCS bytes, e.g.
// a wrong one).
std::vector<int> ax25HdlcBits(const std::vector<unsigned char>& frame, int preFlags = 40, int postFlags = 3,
                              bool appendFcs = true);

// How to send.
struct Ax25Tx {
    Ax25Tx() : baud(1200), markHz(1200), spaceHz(2200), amplitude(0.5), toneScale(1), clockScale(1),
               preFlags(40), postFlags(3) {}
    static Ax25Tx vhf1200() { return Ax25Tx(); }
    static Ax25Tx hf300() {
        Ax25Tx t;
        t.baud = 300; t.markHz = 1600; t.spaceHz = 1800; t.preFlags = 20;
        return t;
    }
    static Ax25Tx g3ruh9600() {
        Ax25Tx t;
        t.baud = 9600; t.markHz = 0; t.spaceHz = 0; t.preFlags = 60;
        return t;
    }
    double baud;          // 1200, 300 (AFSK) or 9600 (G3RUH baseband)
    double markHz, spaceHz;
    double amplitude;     // peak
    double toneScale;     // multiplies both tones (1.02 = 2% high)
    double clockScale;    // multiplies the bit rate
    int preFlags, postFlags;
};

// Appends the audio of NRZI bit levels at `rate` Hz.  Returns the samples
// appended.
std::size_t synthAx25Bits(const std::vector<int>& levels, double rate, std::vector<float>& out,
                          const Ax25Tx& tx = Ax25Tx());
// Appends the audio of a frame (ax25HdlcBits with tx's flags).
std::size_t synthAx25(const std::vector<unsigned char>& frame, double rate, std::vector<float>& out,
                      const Ax25Tx& tx = Ax25Tx());

// Mic-E (as the JS aprsMicEEncode): msg 7 Off duty, 6 En route, 5 In
// service, 4 Returning, 3 Committed, 2 Special, 1 Priority, 0 Emergency.
void aprsMicEEncode(double lat, double lon, double speedKmh, int course, char sym, char table, int msg,
                    std::string& destination, std::string& info);

// The demo band's APRS packets (the JS demo station on 144.800 MHz): a car
// (Mic-E), a weather station, a message, a digipeater's status, a repeater
// object, a walker and a balloon.
struct AprsSample {
    std::string what;      // "mic-e car", "weather", "message", ...
    std::string source, destination;
    std::vector<std::string> path;
    std::string info;
};
std::vector<AprsSample> aprsDemoPackets();

}  // namespace dec

#endif
