// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Ax25.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  AX.25 packet radio receiver with APRS interpretation (Kind::Ax25):
//
//      #include "decoders/Ax25.h"
//      dec::Ax25Decoder ax;              // AFSK 1200 (VHF APRS) by default
//      ax.setBaud(300);                  // or HF packet, or 9600 (G3RUH)
//      ax.setHome(53.48, -2.24);         // optional: distance and bearing
//      ax.process(audio, n, 48000);      // mono audio, any rate
//      std::string t; ax.takeText(t);    // one or two lines per packet
//      std::vector<dec::Ax25Frame> f; ax.takeFrames(f);   // as structs
//
//  Each packet whose frame check (CRC-16) is right prints its monitor line,
//  as TNC2 / Dire Wolf / aprs.fi show packets, then (APRS) what it says:
//
//      M0OGR-9>US2XXU,WIDE1-1*,WIDE2-1: `x*T<0x1f>;?>/OFFgrid-SDR demo car
//        APRS car · 53.4808°N 2.2427°W · 61 km/h heading 135° · en route ·
//        OFFgrid-SDR demo car · <distance> km, bearing <deg>°
//
//  (one line in fact; the distance and bearing only after setHome()).
//
//  (bytes that are not printable ASCII show as <0xNN>; '*' marks each
//  digipeater that has repeated the packet, its H bit.)
//
//  Audio and tuning (what the radio must deliver)
//  ----------------------------------------------
//    1200 baud  Bell 202 AFSK, mark 1200 Hz, space 2200 Hz: VHF/UHF APRS
//               and packet (144.800, 144.390 MHz ...).  Demodulate in NFM
//               with the dial ON the channel; normal (de-emphasised) audio
//               is fine: several slicers cover the 2200 Hz tone being up to
//               ~6 dB weaker or stronger than the 1200 Hz one.
//    300 baud   AFSK, mark 1600 Hz, space 1800 Hz: HF packet / HF APRS
//               (e.g. 10.1476 MHz, 14.105 MHz).  Demodulate in USB with the
//               dial on the listed (dial) frequency, so the tones land at
//               1600/1800 Hz; audioCenterHz() = 1700.  Tune within ~±30 Hz.
//    9600 baud  G3RUH scrambled baseband (K9NG/G3RUH), UHF packet: needs the
//               FM discriminator's flat audio (no de-emphasis, no 3 kHz
//               audio filter) at 38.4 kHz or more; the usual NFM speaker
//               audio will not do.  audioCenterHz() = 0 (baseband).
//  Any input rate from 8 kHz up (9600: 38.4 kHz up); rates above what the
//  modem needs are decimated internally (1200 baud runs at <= 26 kHz, 300
//  at <= 16 kHz, 9600 at <= 50 kHz).  The bit clock follows the signal's
//  rate within a few per cent; the tones may be off by a few per cent.
//
//  How it works
//  ------------
//  The demodulator, HDLC framing and AX.25 address parsing are AX25Toolkit's
//  (Gustavo Campos, https://github.com/solariun/AX25Toolkit), whose DSP comes
//  from Dire Wolf (John Langner, WB2OSZ, https://github.com/wb2osz/direwolf):
//  band-pass, mark and space tone strengths, several slicers each with its
//  own bit clock (PLL) and DCD, NRZI, HDLC flags and bit stuffing, the FCS
//  (see src/decoders/ax25/).  A frame several slicers find is printed once.
//  The APRS interpretation is a port of OFFgrid-SDR's JS parser (diddy-boy,
//  https://github.com/solariun/offgrid-sdr, index.html: parseAx25,
//  parseAprs, aprsLatLon, aprsCompressed, aprsMicE, aprsWx, aprsWhat,
//  aprsDetail): uncompressed, compressed and Mic-E positions (course, speed,
//  altitude), weather (in a position or positionless), messages, bulletins
//  and acks, status, objects, items, telemetry (named only) and third-party
//  packets ("}").
//
//  How it fits in
//  --------------
//  Implements dec::Decoder (Decoder.h).  Ax25Encoders.h builds frames and
//  synthesises their audio (tests, demo band).  Standard library only.
//
//  Contributing
//  ------------
//  Keep the APRS parser in step with the JS one; keep the copied AX25Toolkit
//  code close to its origin (changes marked "OFFgrid:" there); no allocation
//  per sample; strict C++11.  Run tests/test_decoders_ax25.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_AX25_H
#define OFFGRID_DECODERS_AX25_H

#include "Decoder.h"

#include <cstddef>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace dec {

// ---------------------------------------------------------------------------
// APRS: what a packet's information field says.
// Numbers that a packet did not give are NaN: test them with aprsHas().
// ---------------------------------------------------------------------------
inline bool aprsHas(double v) { return v == v; }

struct AprsWeather {
    AprsWeather();
    double windDirDeg;    // where the wind comes from, degrees
    double windKmh;       // sustained wind, km/h
    double gustKmh;       // gusts, km/h
    double tempC;         // temperature, °C (0.1)
    double rain1hMm;      // rain in the last hour, mm
    double rain24hMm;     // rain in the last 24 hours, mm
    double rainDayMm;     // rain since midnight, mm
    double humidityPct;   // relative humidity, %
    double pressureHpa;   // barometric pressure, hPa (0.1)
    bool any() const;     // any value present
};

enum class AprsKind {
    None,       // not an APRS packet (not UI / PID F0)
    Position,   // ! = / @ ` ' (uncompressed, compressed, Mic-E)
    Weather,    // a position with weather, or _ positionless weather
    Message,    // :ADDRESSEE:text{id
    Bulletin,   // a message to BLN...
    Ack,        // a message that is "ackNN" / "rejNN"
    Status,     // >text
    Object,     // ;name.....*time position
    Item,       // )name!position
    Telemetry,  // T#... (named, not decoded)
    Other       // anything else (the raw field is shown)
};
// "position", "weather", "message", "bulletin", "ack", "status", "object",
// "item", "telemetry", "other", "" for None.
const char* aprsKindName(AprsKind k);

struct AprsReport {
    AprsReport();
    AprsKind kind;
    std::string raw;           // the information field (bytes as chars)
    // position (lat/lon NaN when none)
    double lat, lon;           // degrees, + = N / E
    char symTable, symCode;    // APRS symbol ('/' primary table, '\\' alternate,
                               // or an overlay); 0 when none
    std::string symbolName;    // "car", "house", ... (primary table) or ""
    std::string timestamp;     // "221815z" / "221815h" when given
    double courseDeg;          // 0..359 (NaN when not given)
    double speedKmh;
    double altitudeM;
    std::string micE;          // Mic-E message: "Off duty", "En route", ...,
                               // "Emergency", "custom" ("" when not Mic-E)
    AprsWeather wx;            // wx.any() when weather was given
    // message / bulletin / ack
    std::string to, text, msgId;   // text also holds a status's text
    // object / item
    std::string name;
    bool killed;               // object / item deleted ("_")
    std::string comment;
    std::string thirdParty;    // "}" packets: the station that relayed it
    std::vector<std::string> innerPath;   // "}" packets: "(via the internet)"
    std::string innerSource;   // "}" packets: the original source
    // from setHome() (NaN when no home or no position)
    double distanceKm, bearingDeg;
};

// ---------------------------------------------------------------------------
// One AX.25 frame as received.
// ---------------------------------------------------------------------------
struct Ax25Address {
    Ax25Address() : ssid(0), repeated(false) {}
    std::string call;          // "N0CALL" (up to 6 characters)
    int ssid;                  // 0..15
    bool repeated;             // H bit (digipeaters only): has repeated it
    std::string str() const;   // "N0CALL-9" (no "-0")
};

struct Ax25Frame {
    Ax25Frame() : control(0), hasPid(false), pid(0), time(0), slicer(0) {}
    Ax25Address destination, source;
    std::vector<Ax25Address> path;     // digipeaters, in order
    unsigned char control;             // 0x03 = UI (APRS)
    bool hasPid;
    unsigned char pid;                 // 0xF0 = no layer 3 (APRS)
    std::vector<unsigned char> info;
    std::string frameType;             // "UI", "I", "RR", "RNR", "REJ", "SABM",
                                       // "DISC", "UA", "DM", "FRMR", "S", "?"
    double time;                       // seconds of audio since reset()
    int slicer;                        // which slicer decoded it first
    AprsReport aprs;                   // kind None when not APRS

    // "SRC>DEST,DIGI1*,DIGI2: info" (non-UI frames: "SRC>DEST,PATH <SABM>"
    // and the info, if any, after ": ").
    std::string monitorLine() const;
    // The info bytes as text: printable ASCII kept, other bytes as <0xNN>,
    // trailing CR/LF dropped.
    std::string infoText() const;
};

// Parses the bytes between the flags (FCS removed).  False when they are not
// a plausible AX.25 frame (address field, callsigns A-Z 0-9).  APRS is
// interpreted for UI frames with PID F0.
bool ax25ParseFrame(const unsigned char* bytes, std::size_t n, Ax25Frame& out);

// APRS interpretation of an information field, as the JS parseAprs (the
// destination is needed for Mic-E, the source names the relaying station of
// a third-party "}" packet).
void aprsParse(const std::string& source, const std::string& destination, const std::string& info,
               AprsReport& out);
// What the report is, one or two words: "car", "weather", "message", ...
// (the JS aprsWhat).
std::string aprsWhat(const AprsReport& r);
// The readable line (without the leading "APRS <what>"): the JS aprsDetail
// plus the position and, if known, distance and bearing.
std::string aprsDescribe(const AprsReport& r);
// Great-circle distance (km) and initial bearing (degrees) from 1 to 2.
void aprsDistance(double lat1, double lon1, double lat2, double lon2, double& km, double& deg);

// ---------------------------------------------------------------------------
// The decoder (Kind::Ax25).
// ---------------------------------------------------------------------------
class Ax25Decoder : public Decoder {
public:
    Ax25Decoder();
    ~Ax25Decoder();

    Kind kind() const { return Kind::Ax25; }
    void reset();
    void process(const float* audio, std::size_t n, double rate);
    void takeText(std::string& out);
    Status status() const;
    double audioCenterHz() const;   // 1700 (AFSK 1200 and 300), 0 (9600)

    // --- settings (kept by reset()) ---
    // 1200 (default), 300 or 9600; anything else is ignored.  Resets.
    void setBaud(int baud);
    int baud() const { return baud_; }
    // Slicers for AFSK, 1..5 (default 5: one AGC-normalised + four with the
    // space tone scaled 0.5, 1, 2, 4).  Resets.
    void setSlicers(int n);
    // Where you are, for each position's distance and bearing.
    void setHome(double lat, double lon);
    void clearHome();
    // How many frames frames() keeps (default 100).
    void setMaxFrames(std::size_t n);

    // --- results ---
    // The last frames (oldest first), up to setMaxFrames().
    std::vector<Ax25Frame> frames() const;
    // Appends the frames decoded since the last call.
    void takeFrames(std::vector<Ax25Frame>& out);
    unsigned frameCount() const { return count_; }
    bool dcd() const;

private:
    struct Impl;
    Ax25Decoder(const Ax25Decoder&);             // not copyable
    Ax25Decoder& operator=(const Ax25Decoder&);
    void setup(double rate);
    void gotFrame(int slicer, const unsigned char* data, std::size_t len);

    std::unique_ptr<Impl> impl_;
    int baud_;
    int slicers_;
    bool haveHome_;
    double homeLat_, homeLon_;
    std::size_t maxFrames_;

    double rate_;           // input rate (0 = not set up)
    double audioT_;         // seconds of audio since reset
    double lastFrameT_;
    unsigned count_;
    std::string text_;
    std::deque<Ax25Frame> frames_;
    std::vector<Ax25Frame> fresh_;
};

}  // namespace dec

#endif
