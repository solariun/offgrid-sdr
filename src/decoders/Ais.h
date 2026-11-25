// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Ais.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  AIS receiver (Kind::Ais): ships' positions, courses, names and
//  destinations (ITU-R M.1371), 9600-baud GMSK on the two marine VHF AIS
//  channels, decoded at once from the stick's IQ samples:
//
//      #include "decoders/Ais.h"
//      dec::AisDecoder ais;
//      ais.setHome(53.48, -2.24);                 // optional: distance, bearing
//      // the stick centred on 162.000 MHz, e.g. at 2.048 MS/s:
//      ais.processIq(iq, n, 2048000, 162.000e6, 162.000e6);
//      std::string t; ais.takeText(t);            // one line per message
//      std::vector<dec::AisMessage> m; ais.takeMessages(m);   // as structs
//      std::vector<std::string> s; ais.takeNmea(s);          // !AIVDM lines
//      std::vector<dec::AisVessel> v = ais.vessels();        // the AIS list
//
//  One text line per message whose frame check (CRC-16) is right:
//
//      AIS A 235012345 OFFGRID EXPLORER (cargo, UK): position: 53.5210°N
//        3.3520°W · 12.3 kn, course 285° · heading 284° · under way ·
//        74 km, bearing 279°
//
//  (one line in fact; the distance and bearing only after setHome()).
//
//  Tuning (what the radio must deliver)
//  ------------------------------------
//  Complex IQ from the stick (any rate from about 150 kS/s up; 2.048 and
//  2.4 MS/s are the usual) with `centerHz` its centre frequency.  Channel A
//  (161.975 MHz, 87B) and channel B (162.025 MHz, 88B) are both decoded
//  whenever they lie within ±45% of the rate of the centre: centre the
//  stick between them, on 162.000 MHz (the TUI does), and both are 25 kHz
//  off centre.  (The browser version puts its dial on 161.975, channel A,
//  in NFM so the bursts can be heard, and decodes both channels from the
//  same IQ.)  `dialHz` is not needed and is ignored.  A stick up to a few
//  kHz off frequency (±1 kHz tested) is followed.  IQ scale does not
//  matter (the FM discriminator only sees phase).
//
//  How it works
//  ------------
//  A port of OFFgrid-SDR's JS (diddy-boy, https://github.com/solariun/
//  offgrid-sdr, index.html: "AIS: ships' positions", class AisChannel,
//  aisBits, parseAis, "names for numbers", the AIS list): per channel,
//  mixed down to 0 Hz, summed in blocks (boxcar) to about 240 kHz, low-passed
//  (41 taps) and kept every 5th sample (about 48 kHz), FM discriminator
//  (phase step), the carrier offset removed, a bit clock pulled by each
//  transition, NRZI (no change = 1), HDLC flags and bit stuffing, CRC-16
//  (as AX.25).  Bits arrive least significant first within each byte; the
//  message is read most significant first (bytes reversed).
//
//  Additions to the JS (see Ais.cpp, each marked): the low-pass cuts at
//  7 kHz (not 10) and the discriminator is averaged over two samples; the
//  carrier offset is measured at the start of each burst (found by its
//  power) and then held, rather than averaged all the time; the bit clock
//  interpolates each crossing and the bit's value; NaN/Inf samples count as
//  0; frames up to 160 bytes; types 0 and above 27 rejected; type 27 read;
//  !AIVDM sentences.  In the test (both channels at once, the stick centred
//  and up to 1 kHz off, SNR measured in 20 kHz) about 85% of messages
//  decode at 14 dB, 98% at 16 dB and all at 18 dB; with the JS's settings
//  the same messages needed well over 20 dB.
//
//  How it fits in
//  --------------
//  Implements dec::Decoder (Decoder.h): wantsIq() is true, call processIq()
//  (process() with audio does nothing).  AisEncoders.h builds messages and
//  synthesises their IQ (tests, demo band).  Standard library only.
//
//  Contributing
//  ------------
//  Keep the parser and the names in step with the JS; mark anything new
//  "Addition to the JS"; no allocation per sample; strict C++11.
//  Run tests/test_decoders_ais.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_AIS_H
#define OFFGRID_DECODERS_AIS_H

#include "Decoder.h"

#include <complex>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace dec {

// The two AIS channels (Hz).
const double kAisChannelA = 161.975e6;   // 87B, "A"
const double kAisChannelB = 162.025e6;   // 88B, "B"
// Where to centre the stick to hear both (Hz).
const double kAisCenter = 162.000e6;

// Numbers a message did not give (or gave as "not available") are NaN:
// test them with aisHas().
inline bool aisHas(double v) { return v == v; }

// ---------------------------------------------------------------------------
// One message (the JS parseAis result).
// ---------------------------------------------------------------------------
enum class AisKind {
    Position,   // 1/2/3 (class A), 18/19 (class B), 27 (long range)
    Base,       // 4: base (coast) station, with its UTC time
    Static,     // 5 (static and voyage), 24 (class B static, part A or B)
    Aton,       // 21: aid to navigation
    Other       // any other type (the bits are kept; no fields read)
};
// "position", "base", "static", "aton", "other".
const char* aisKindName(AisKind k);

struct AisMessage {
    AisMessage();
    int type;                  // 1..27
    int repeat;                // repeat indicator 0..3
    unsigned mmsi;             // never 0
    AisKind kind;
    char channel;              // 'A' or 'B' ('?' when parsed by aisParse())
    double time;               // when it ended: seconds of IQ since reset(),
                               // or Unix seconds after setTimestamp()
    bool classB;               // 18, 19 (as the JS: not set by 24)

    // position (NaN when absent or not available: 181° / 91°)
    double lat, lon;
    // motion: hasMotion when the message carries speed and course (they may
    // still be NaN: "not available"); hasHeading likewise for the heading.
    bool hasMotion, hasHeading;
    double sogKn;              // speed over ground, knots (102.3 = n/a -> NaN)
    double cogDeg;             // course over ground, degrees (360 n/a -> NaN)
    double headingDeg;         // true heading, degrees (511 n/a -> NaN)
    int status;                // navigational status 0..15, -1 when absent
    int rot;                   // rate of turn as sent (-127..127), -128 = none

    // static (5, 19, 24)
    unsigned imo;              // 0 = none
    std::string callsign, name, destination;
    int shipType;              // 0..99, -1 when absent
    double lengthM, beamM, draughtM;   // NaN when absent
    std::string eta;           // "14/11 02:30 UTC" ("" when not given)
    int etaMonth, etaDay, etaHour, etaMinute;   // as sent (-1 when absent)

    // aids to navigation (21)
    int atonType;              // 0..31, -1 when absent

    // base station (4): its UTC time
    std::string utc;           // "2026-11-25 12:00:00 UTC"
    int year, month, day, hour, minute, second;   // -1 when absent

    std::vector<unsigned char> bytes;  // the message, MSB first, CRC removed
    std::vector<std::string> nmea;     // its !AIVDM sentence(s)

    // from setHome() (NaN when no home or no position)
    double distanceKm, bearingDeg;
};

// ---------------------------------------------------------------------------
// One vessel (or station, or aid) in the AIS list: everything heard from
// one MMSI, merged as it arrives (positions and names come in different
// messages), as the JS aisGot().
// ---------------------------------------------------------------------------
struct AisVessel {
    AisVessel();
    unsigned mmsi;
    double lat, lon;                   // NaN until a position is heard
    double sogKn, cogDeg, headingDeg;  // NaN when not known
    int status;                        // -1 when not known
    std::string name, callsign, destination, eta;
    unsigned imo;
    int shipType;                      // -1 when not known
    double lengthM, beamM, draughtM;
    int atonType;                      // -1 when not known
    bool classB, base, aton;
    double firstHeard, lastHeard;      // same clock as AisMessage::time
    unsigned messages;
    bool heardA, heardB;               // on which channel(s)
    int lastType;                      // type of the last message
    double distanceKm, bearingDeg;     // from setHome() (NaN otherwise)
};

// ---------------------------------------------------------------------------
// Message parsing and "names for numbers" (free functions).
// ---------------------------------------------------------------------------
// CRC-16 of the frame as sent (HDLC: bytes LSB first), as the JS AisCrc.
unsigned aisCrc(const unsigned char* bytes, std::size_t n);
// Bit reversal of a byte (HDLC order <-> message order), the JS aisRev.
unsigned char aisReverse(unsigned char b);
// Parses a message (bytes MSB first, CRC removed).  False when the MMSI is 0
// or (Addition to the JS) the type is 0 or above 27.  Sets bytes and nmea
// (channel '?' unless out.channel was 'A' / 'B' beforehand).
bool aisParse(const unsigned char* bytes, std::size_t n, AisMessage& out);
// The !AIVDM sentence(s) for a message (bytes MSB first, all of each byte's
// bits sent; fill bits pad to the 6-bit boundary).  Up to 60 payload
// characters per sentence; longer messages are split, with `seqId` (0..9)
// as the sequential message id.  channel 'A' / 'B' (or 0: left empty).
std::vector<std::string> aisNmea(const unsigned char* bytes, std::size_t n, char channel, int seqId = 0);
// The NMEA checksum of the text between '!' and '*' (XOR of the bytes).
unsigned aisNmeaChecksum(const std::string& body);

// "under way", "at anchor", ... (statuses 0..8; "" otherwise), JS AIS_STATUS.
const char* aisStatusName(int status);
// "cargo", "tanker", "pilot", ... ("" for 0 / unknown), JS aisTypeName.
std::string aisShipTypeName(int shipType);
// "light vessel", "buoy", ... ("" when not in the table), JS AIS_ATON.
const char* aisAtonName(int atonType);
// The flag from the MMSI's maritime identification digits ("" unknown).
const char* aisFlag(unsigned mmsi);
// What it is: "coast station", "buoy", "cargo", "small craft", "vessel"
// (the JS aisWhat).
std::string aisWhat(const AisVessel& v);
// The list's detail: "12.3 kn, course 285° · at anchor · to DUBLIN" (JS
// aisDetail).
std::string aisDetail(const AisVessel& v);
// One readable line for a message (without the "AIS A " prefix).
std::string aisDescribe(const AisMessage& m);
// Great-circle distance (km) and initial bearing (degrees) from 1 to 2 (the
// JS distBrg, not rounded).
void aisDistance(double lat1, double lon1, double lat2, double lon2, double& km, double& deg);

// ---------------------------------------------------------------------------
// The decoder (Kind::Ais).
// ---------------------------------------------------------------------------
class AisDecoder : public Decoder {
public:
    AisDecoder();
    ~AisDecoder();

    Kind kind() const { return Kind::Ais; }
    void reset();                          // forgets the vessels too
    // AIS needs IQ: audio is ignored.
    void process(const float* audio, std::size_t n, double rate);
    void takeText(std::string& out);
    Status status() const;
    double audioCenterHz() const { return 0; }
    bool wantsIq() const { return true; }
    // iq at `rate` (S/s), the stick centred on `centerHz`; dialHz ignored.
    void processIq(const std::complex<float>* iq, std::size_t n, double rate, double centerHz, double dialHz);
    // Addition to the JS: the computer's clock (Unix seconds) at the first
    // sample of the next processIq(); times are then Unix seconds.
    void setTimestamp(double unixSeconds);

    // --- settings (kept by reset()) ---
    void setHome(double lat, double lon);
    void clearHome();
    // Addition to the JS: also put each message's !AIVDM sentence(s) in
    // takeText() (after its readable line).  Default off.
    void setNmeaInText(bool on) { nmeaInText_ = on; }
    // How many messages messages() keeps (default 200) and how many vessels
    // the list keeps (default 1000; the least recently heard go first).
    void setMaxMessages(std::size_t n);
    void setMaxVessels(std::size_t n);

    // --- results ---
    // The AIS list: one row per MMSI, the most recently heard first.
    std::vector<AisVessel> vessels() const;
    bool vessel(unsigned mmsi, AisVessel& out) const;
    void clearVessels();
    // The last messages (oldest first) / those since the last call.
    std::vector<AisMessage> messages() const;
    void takeMessages(std::vector<AisMessage>& out);
    // The !AIVDM sentences since the last call (each without CR/LF).
    void takeNmea(std::vector<std::string>& out);
    unsigned messageCount() const { return count_; }
    unsigned messageCount(char channel) const { return channel == 'B' ? countB_ : countA_; }
    // Whether each channel was within the IQ span at the last processIq().
    bool channelInView(char channel) const;

private:
    struct Impl;
    AisDecoder(const AisDecoder&);             // not copyable
    AisDecoder& operator=(const AisDecoder&);
    void gotFrame(int ch, const unsigned char* bytes, std::size_t n, double t);

    std::unique_ptr<Impl> impl_;
    bool haveHome_;
    double homeLat_, homeLon_;
    bool nmeaInText_;
    std::size_t maxMessages_, maxVessels_;
    unsigned count_, countA_, countB_;
    int seqId_;
    std::string text_;
    std::vector<std::string> nmea_;
};

}  // namespace dec

#endif
