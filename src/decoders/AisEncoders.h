// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/AisEncoders.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  Builds AIS messages and synthesises their radio signal as the stick
//  would see it, for the demo band and the tests (the counterpart of
//  Ais.h).  Deterministic (no clock, no randomness):
//
//      message fields -> bits (aisPack)                     AisPacker, aisXxxMsg
//      -> HDLC frame: training 0101.., flag, bytes LSB first with bit
//         stuffing, CRC-16, flag, NRZI levels               aisFrameLevels
//      -> GMSK frequency (Gaussian-filtered levels, ±2400 Hz, at 48 kHz)
//                                                           aisGmskFreq
//      -> complex IQ at any rate and channel offset         aisAddBurst
//
//      std::vector<unsigned char> m = dec::aisPositionMsg(235012345, 53.521,
//                                       -3.352, 12.3, 285.4, 284, 0);
//      dec::aisAddBurst(iq, n, 2048000, 1000, -25000, 0.5, m);   // channel A,
//                                                 // the stick on 162.000 MHz
//
//  aisDemoPlan() is the JS demo's 30 s cycle of reports from Liverpool Bay
//  (a cargo ship, the pilot boat, a tanker at anchor, a Class B yacht, a
//  light vessel and a coast station on both channels in turn);
//  aisDemoAdd() adds that cycle to any block of IQ.
//
//  How it works
//  ------------
//  A port of OFFgrid-SDR's JS encoder (diddy-boy, https://github.com/
//  solariun/offgrid-sdr, index.html: "an encoder (for the demo and the
//  tests)": aisPack, aisFrame, aisPosMsg, aisStaticMsg, aisClassB, aisB24,
//  aisAton, aisBase, aisGmsk, and the demo station's plan).  The JS
//  synthesiser holds the frequency per chunk of samples; here the phase is
//  the exact running integral of the frequency, interpolated (Addition to
//  the JS), so any block of samples can be made on its own.
//
//  How it fits in
//  --------------
//  Uses aisCrc() / aisReverse() from Ais.h.  Standard library only.
//
//  Contributing
//  ------------
//  Keep the field layouts in step with the JS encoder and the parser in
//  Ais.cpp.  Run tests/test_decoders_ais.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_AIS_ENCODERS_H
#define OFFGRID_DECODERS_AIS_ENCODERS_H

#include <complex>
#include <cstddef>
#include <string>
#include <vector>

namespace dec {

// The JS aisPack: fields most significant bit first, text as 6-bit
// characters ('@' pads), then padded with 0 bits to `nbits` and cut into
// bytes.
class AisPacker {
public:
    // An unsigned (or, negative, two's complement) value in `width` bits.
    void field(double value, int width);
    // Text: upper-cased, padded with '@' / cut to `chars` characters.
    void text(const std::string& t, int chars);
    std::vector<unsigned char> bytes(int nbits) const;
    int bits() const { return static_cast<int>(bits_.size()); }

private:
    std::vector<unsigned char> bits_;
};

// ---- messages (bytes, most significant bit first): the JS builders ----
// 1/2/3: class A position.  sog knots, cog degrees, heading 0..359 (511 =
// n/a), status 0..15.
std::vector<unsigned char> aisPositionMsg(unsigned mmsi, double lat, double lon, double sogKn, double cogDeg,
                                          int heading, int status = 0, int type = 1);
// 5: static and voyage.  length/beam in metres (split bow/stern 70/30 and
// port/starboard in half, as the JS); ETA month, day, hour, minute (0, 0,
// 24, 60: not given).
std::vector<unsigned char> aisStaticMsg(unsigned mmsi, unsigned imo, const std::string& callsign,
                                        const std::string& name, int shipType, int lengthM, int beamM,
                                        double draughtM, const std::string& destination, int etaMonth, int etaDay,
                                        int etaHour, int etaMinute);
// 18: class B position.
std::vector<unsigned char> aisClassBMsg(unsigned mmsi, double lat, double lon, double sogKn, double cogDeg,
                                        int heading);
// 24 part A (the name) and part B (ship type, call sign; vendor "OFF").
std::vector<unsigned char> aisStaticBNameMsg(unsigned mmsi, const std::string& name);
std::vector<unsigned char> aisStaticBTypeMsg(unsigned mmsi, int shipType, const std::string& callsign);
// 21: aid to navigation.
std::vector<unsigned char> aisAtonMsg(unsigned mmsi, int atonType, const std::string& name, double lat, double lon);
// 4: base station, its UTC time given (the JS uses the clock).
std::vector<unsigned char> aisBaseMsg(unsigned mmsi, double lat, double lon, int year, int month, int day, int hour,
                                      int minute, int second);
// Addition to the JS: 27, long-range position (1/10 minute, whole knots
// and degrees).
std::vector<unsigned char> aisLongRangeMsg(unsigned mmsi, double lat, double lon, int sogKn, int cogDeg, int status);

// ---- the radio side ----
// The JS aisFrame: NRZI levels (0/1) to send: 24 training bits, flag, the
// message bytes bit-reversed (sent LSB first) with the CRC, bit-stuffed,
// flag, 4 more bits.  crcXor (Addition to the JS, for tests) is XORed into
// the CRC sent: nonzero makes a frame with a bad CRC.
std::vector<unsigned char> aisFrameLevels(const std::vector<unsigned char>& msg, unsigned crcXor = 0);
// The JS aisGmsk: the instantaneous frequency (Hz) at 48 kHz, 5 samples a
// bit, Gaussian-filtered ±2400 Hz.
std::vector<float> aisGmskFreq(const std::vector<unsigned char>& levels);

// A burst ready to be placed in IQ: its phase (cycles), the running
// integral of the GMSK frequency.  Addition to the JS: worked out at 40
// samples a bit (384 kHz) rather than aisGmskFreq's 5, so the phase, read
// by interpolation at any rate, makes no images 48 kHz away (where the
// other channel is, seen from one channel); the power ramps up and down
// over 8 bits.
struct AisBurst {
    std::vector<double> cycles;       // cycles[k]: phase at k / 384000 s
    double seconds() const { return cycles.empty() ? 0 : (cycles.size() - 1) / 384000.0; }
};
AisBurst aisMakeBurst(const std::vector<unsigned char>& msg, unsigned crcXor = 0);
// Adds the burst to iq[0..n) (at `rate`) starting at burst time `tStart`
// seconds relative to iq[0] (negative: already under way), `offsetHz` from
// the stick's centre, amplitude `amp`, starting phase `phase0` (radians).
void aisAddBurst(std::complex<float>* iq, std::size_t n, double rate, double tStart, double offsetHz, double amp,
                 const AisBurst& burst, double phase0 = 0);
// Convenience: the message's burst starting at sample `at`.
void aisAddBurst(std::complex<float>* iq, std::size_t n, double rate, std::size_t at, double offsetHz, double amp,
                 const std::vector<unsigned char>& msg);

// ---- the demo band ----
struct AisDemoBurst {
    double t;                         // seconds into the 30 s cycle
    char channel;                     // 'A' or 'B'
    std::vector<unsigned char> msg;
    AisBurst burst;
};
// The JS demo's plan (Liverpool Bay).  The base station's time is given
// (the JS uses the clock); the default is fixed for reproducibility.
std::vector<AisDemoBurst> aisDemoPlan(int year = 2026, int month = 1, int day = 1, int hour = 12, int minute = 0,
                                      int second = 0);
// Adds the plan's bursts that overlap [t0, t0 + n / rate) (t0: seconds on
// the demo's clock; the plan repeats every 30 s) for a stick centred on
// centerHz (a channel more than 45% of the rate away is left out).
void aisDemoAdd(const std::vector<AisDemoBurst>& plan, std::complex<float>* iq, std::size_t n, double rate,
                double centerHz, double t0, double amp);

}  // namespace dec

#endif
