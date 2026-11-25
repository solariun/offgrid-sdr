// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Ft8.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  dec::Ft8Decoder (Kind::Ft8) and dec::Ft4Decoder (Kind::Ft4): FT8 and FT4
//  from live audio, slot by slot, as OFFgrid-SDR's FT8 mode does it (a port
//  of ft8Feed, ft8Done, ftxInfo and the FT8 box, diddy-boy, index.html;
//  the decoding itself is Ftx.h's, after ft8_lib by Kārlis Goba, MIT):
//
//      #include "decoders/Ft8.h"
//      dec::Ft8Decoder d;
//      d.setHome("IO91");                      // optional: distance, bearing
//      d.setDial(14.074e6);                    // optional: the band in the text
//      for (;;) {
//          d.setClockOffset(clock.offsetAt(pcNow));   // dec::ClockCorrection
//          d.setTimestamp(pcClockAtFirstSample);      // Unix seconds
//          d.process(audio, n, 48000);                // USB audio, any rate
//          std::string t; d.takeText(t);              // a line per decode
//          std::vector<dec::FtxDecode> list; d.takeDecodes(list);
//          std::vector<double> dts;
//          if (d.takeSlotTimings(dts))                // after each slot decoded
//              clock.applyFtxTimings(dts, "FT8", pcNow);
//      }
//
//  Text lines (as the browser's FT8 box rows: time, SNR, DT, audio
//  frequency, message, country, distance and bearing):
//
//      143015 -12 +0.3 1240  CQ G4OGR IO81           England · 1234 km 287°
//
//  ("◆" after the message: found by the deeper decoding, under a stronger
//  station; "★": calling you, see setMyCall()).
//
//  Audio and dial convention
//  -------------------------
//  USB audio, the dial ON the listed FT8 / FT4 frequency (ft8Dials(),
//  ft4Dials(): 14.074 / 14.080 MHz ...), any rate from 8 kHz up; the
//  stations' base (lowest) tones searched over 200-3000 Hz (setBand()).
//  Reported frequencies are Hz above the dial, as WSJT-X and the browser
//  show them.  (The browser listens 1.5 kHz below the dial and puts the
//  frequencies back; OFFgrid-SDR TUI tunes the dial on the frequency.)
//
//  Timing
//  ------
//  Call setTimestamp() before every process() with the computer's clock
//  (Unix seconds) at the block's first sample, and setClockOffset() with
//  the correction in use (computer - true, ClockCorrection's sign): the
//  slots follow the true time.  As in ft8Feed(), the stream's start on the
//  computer's clock is the earliest estimate (blocks only arrive late),
//  creeping up slowly; a jump over 1 s starts it again.  Just after each
//  slot ends (0.4 s, by the corrected clock) the last 27.6 s (FT8) / 13 s
//  (FT4) are decoded: always one whole transmission, wherever the clock
//  stands within it, so a clock seconds out still decodes, and every
//  decode's DT (+ = late) measures the clock.  Without timestamps the
//  stream is taken to start at Unix time 0.
//
//  Decoding a slot takes a few hundred ms of one core (FT8, 3 passes, on a
//  laptop: lastDecodeMs()); it runs inside process() at the slot's end.  As
//  in the browser, the passes adapt: one fewer after a decode that took over
//  60% of the slot, one more after one under 30% (setMaxPasses(), 3).
//
//  How it fits in
//  --------------
//  Implements dec::Decoder.  Uses Ftx.h (messages, codes, the Engine) and
//  Callsign.h (country, locator, distance).  No allocation per sample:
//  the buffers are sized when the rate changes; a slot's decode allocates
//  its results.  HeardList (Heard.h) can take every decode.
//
//  Contributing
//  ------------
//  Keep it in step with the browser's ft8Feed / ft8Done / ftxInfo / FT8 box;
//  mark deviations "Addition to the JS".  Tests:
//  tests/test_decoders_ftx.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_FT8_H
#define OFFGRID_DECODERS_FT8_H

#include "Decoder.h"
#include "Ftx.h"

#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace dec {

// What a message says (port of ftxInfo()).
struct FtxInfo {
    FtxInfo() : hasReport(false), report(0) {}
    // "CQ" (also QRZ), "reply" (with a grid), "report", "R report", "RRR",
    // "RR73", "73", "call" (two calls, nothing else known) or "text".
    std::string type;
    std::string from;    // the caller (sender); "" for text
    std::string to;      // the callee; "" for CQ and text
    std::string grid;    // 4-character locator, if sent
    std::string cqTo;    // "DX", "EU", "POTA" ... of "CQ DX K1ABC FN42"
    bool hasReport;
    int report;          // dB
};
FtxInfo ftxInfo(const std::string& text);

// One decode, worked out (a row of the browser's FT8 box).
struct FtxDecode {
    FtxDecode()
        : slotTime(0), slotSeconds(15), snr(0), dt(0), freq(0), pass(1), cq(false), gridFromEarlier(false),
          hasReport(false), report(0), hasDistance(false), km(0), bearing(0), toMe(false) {}
    std::string proto;       // "FT8", "FT4"
    double slotTime;         // the slot's start, Unix seconds (true UTC)
    double slotSeconds;      // 15 or 7.5
    std::string utc;         // "HHMMSS" of the slot's start
    int snr;                 // dB in 2500 Hz
    double dt;               // seconds late (+) or early (-) against 0.5 s into the slot
    double freq;             // Hz above the dial (tone 0)
    std::string text;        // the message, e.g. "CQ G4OGR IO81"
    int pass;                // 1, or 2-3: found once stronger signals were taken out ("deeper")
    // the message's fields (ftxInfo)
    std::string type;
    std::string caller, callee, cqTo;
    bool cq;
    std::string grid;        // locator: sent in this message or (gridFromEarlier) an earlier one
    bool gridFromEarlier;
    bool hasReport;
    int report;
    // where the caller is
    std::string country, continent;  // from the callsign ("" if unknown)
    bool hasDistance;        // home set and a locator known
    int km, bearing;         // rounded, as the browser shows them
    bool toMe;               // the callee is setMyCall()'s call
};

// A listed dial frequency (the browser's FTX_BANDS).
struct FtxDial {
    const char* band;  // "20 m"
    double hz;         // 14.074e6
};
const std::vector<FtxDial>& ft8Dials();  // 160 m .. 2 m
const std::vector<FtxDial>& ft4Dials();  // 80 m .. 2 m

// FT8 and FT4 share everything but their numbers (ftx::protocolInfo()).
class FtxDecoder : public Decoder {
public:
    explicit FtxDecoder(ftx::Protocol p);

    Kind kind() const override;
    // Forgets the audio, the timing and the decodes (and the hash table and
    // the grids heard).  Settings are kept.
    void reset() override;
    void process(const float* audio, std::size_t n, double rate) override;
    void takeText(std::string& out) override;
    Status status() const override;
    double audioCenterHz() const override { return 1500; }
    void setTimestamp(double unixSeconds) override;
    void setClockOffset(double pcMinusTrue) override;

    // ---- settings ----
    // Your locator ("IO91", "IO91wm"): distances and bearings.  False (and
    // cleared) if not a locator.
    bool setHome(const std::string& grid);
    void setHomeLatLon(double lat, double lon);
    void clearHome();
    // Your callsign: decodes calling it are marked (toMe, "★").
    void setMyCall(const std::string& call);
    // The dial frequency (Hz), for the text only (0 = not shown).
    void setDial(double hz) { dial_ = hz; }
    // Base frequencies searched, Hz (default 200-3000).
    void setBand(double loHz, double hiHz);
    // Deeper decoding (decode, subtract, decode again) on/off; and its most
    // passes (1-3; the browser: 3, or 2 in low-power mode).
    void setDeep(bool on) { deep_ = on; }
    void setMaxPasses(int n);

    // ---- results ----
    const std::deque<FtxDecode>& decodes() const { return list_; }   // the last 400, oldest first
    void takeDecodes(std::vector<FtxDecode>& out);                    // new since the last call
    // After each slot decoded: true once, with the decodes' DTs (seconds,
    // + = late), for ClockCorrection::applyFtxTimings(dts, protoName()).
    bool takeSlotTimings(std::vector<double>& dts);
    const std::vector<double>& lastSlotTimings() const { return lastDts_; }
    const char* protoName() const { return info_.name; }
    // The grid last heard from a call ("" if none).
    std::string gridOf(const std::string& call) const;
    // Decodes the audio buffered so far now, as at a slot's end (Addition
    // to the JS: for recordings and tests).  The number of new decodes.
    int decodeNow();

    // ---- performance ----
    double lastDecodeMs() const { return lastMs_; }     // wall time of the last slot's decode
    double maxDecodeMs() const { return maxMs_; }
    unsigned slotsDecoded() const { return slots_; }
    int passesInUse() const { return passes_; }
    unsigned lastNew() const { return lastNew_; }
    unsigned lastDeeper() const { return lastDeeper_; }

    ftx::CallHashTable& hashTable() { return hashes_; }

private:
    void setRate(double rate);
    void chunk(const float* x, std::size_t n, double t0);
    void decodeWindow();
    void addDecode(const ftx::RawDecode& r, double t0, std::vector<double>& dts);
    void fillPlace(FtxDecode& d) const;
    std::string line(const FtxDecode& d) const;

    ftx::Protocol proto_;
    ftx::ProtocolInfo info_;
    ftx::Engine engine_;
    ftx::CallHashTable hashes_;
    // settings
    bool homeSet_;
    double homeLat_, homeLon_;
    std::string myCall_;
    double dial_, fLo_, fHi_;
    bool deep_;
    int maxPasses_, passes_;
    // timing
    bool tsPending_;
    double ts_, nextTs_;
    double offset_;
    bool anchorSet_;
    double anchor_;          // computer time of output sample 0
    bool lastSlotSet_;
    long long lastSlot_;
    // resampling to the decoder's rate (an anti-alias low-pass first when the rate is higher)
    double rate_, step_, frac_;
    float prev_;
    bool lowpass_;
    double bq_[3][5], z_[3][2];
    // the audio at info_.sampleRate: a ring
    std::vector<float> ring_;
    long long n_;
    std::vector<float> work_;
    std::vector<ftx::RawDecode> raw_;
    // results
    std::deque<FtxDecode> list_;
    std::set<std::string> keys_;
    std::vector<FtxDecode> fresh_;
    std::map<std::string, std::string> grids_;
    std::deque<std::string> gridOrder_;
    std::vector<double> lastDts_;
    bool dtsPending_;
    std::string out_;
    double lastMs_, maxMs_;
    unsigned slots_, total_, lastNew_, lastDeeper_;
    int bestSnr_;
};

class Ft8Decoder : public FtxDecoder {
public:
    Ft8Decoder() : FtxDecoder(ftx::Protocol::Ft8) {}
};

class Ft4Decoder : public FtxDecoder {
public:
    Ft4Decoder() : FtxDecoder(ftx::Protocol::Ft4) {}
};

}  // namespace dec

#endif
