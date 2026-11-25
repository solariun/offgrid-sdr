// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Wspr.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  dec::WsprDecoder: WSPR (Weak Signal Propagation Reporter) beacons, the
//  radio amateurs' 2-minute weak-signal reports (callsign, locator, power).
//  A port of OFFgrid-SDR's WSPR library (diddy-boy, index.html: the block
//  ending `globalThis.WSPR = { PR3, pack, unpack, encode, symbols, fano,
//  decode, VALID_DBM, TONE }`), of its slot driver (ft8Feed()/ft8Done() with
//  FTX.WSPR: "2-minute slots from each even minute, transmissions 110.6 s
//  starting 1 s in; decoded once the slot is over") and of its stick
//  calibration (wsprNoteFreq(), wsprCalText(), wsprCalApply()).
//
//  The protocol is K1JT's (Joe Taylor), as used by WSJT-X's wsprd; the JS
//  was written from the published protocol and checked against WsprryPi.
//  The Fano sequential decoder follows Phil Karn KA9Q's, as wsprd does.
//
//    message    50 bits: a callsign (up to 6 characters, a digit third),
//               a 4-character locator and the power in dBm (type 1).  The
//               JS decodes type 1 messages only, and so does this port
//               (types 2 and 3, compound calls and hashed calls with
//               6-character locators, are recognised and skipped).
//    coding     K=32 rate-1/2 convolutional code (Layland-Lushbaugh
//               polynomials 0xf2d05351, 0xe4613c47), 31 tail bits: 162
//               coded bits, bit-reversal interleaved.
//    symbols    162 4-FSK symbols, 1.4648 Hz apart (12000/8192), each
//               8192/12000 s (110.6 s in all), symbol = sync bit + 2 x data
//               bit (the sync bits are the known PR3 sequence).
//    timing     2-minute slots from each even UTC minute; transmissions
//               start 1 s in.
//
//  How it is found (as the JS, see Wspr.cpp for the details): the decoder
//  keeps the last ~4 minutes of audio (decimated, see below) and, just
//  after each slot ends (by the corrected clock), searches 232 s of it for
//  start times across two whole minutes, and the whole audio passband for
//  frequencies (not only WSPR's usual 1400-1600 Hz: a stick off frequency
//  moves the signals).  So it decodes with the clock out by up to about a
//  minute, and the decodes' timings then say how far out it is.
//
//  Use:
//
//      #include "decoders/Wspr.h"
//      dec::WsprDecoder w;
//      w.setDialFrequency(14.0956e6);           // optional: RF of each decode
//      for (;;) {
//          w.setClockOffset(clock.offsetAt(pcNow));   // PC - true, seconds
//          w.setTimestamp(pcClockAtFirstSample);      // Unix seconds (PC)
//          w.process(audio, n, 48000);                // USB audio
//          std::string t; w.takeText(t);              // one line per decode
//          std::vector<dec::WsprDecode> d; w.takeDecodes(d);
//          if (w.newSlotDone()) clock.applyFtxTimings(w.lastSlotDts(), "WSPR", pcNow);
//          dec::WsprCalibration c = w.calibration();  // c.offer: c.ppm
//      }
//
//  Audio and dial convention: USB with the dial ON the listed WSPR dial
//  frequency (WsprDecoder::dialList(): 136.0 kHz, 474.2 kHz, 1836.6 kHz,
//  3568.6 kHz, 5287.2 kHz, 7038.6 kHz, 10138.7 kHz, 14095.6 kHz,
//  18104.6 kHz, 21094.6 kHz, 24924.6 kHz, 28124.6 kHz, 50.293 MHz,
//  144.489 MHz); the stations then sit at 1400-1600 Hz in the audio
//  (centre 1500 Hz) and a decode's RF frequency is dial + audio Hz.
//  The browser listens 1.5 kHz lower (FTX_SHIFT) with a 6 kHz filter and
//  subtracts the 1500 Hz again; this port takes the audio in the usual
//  terms.  The search covers 200-2800 Hz of audio by default
//  (setSearchRange()), so a stick up to ~1300 Hz off still has them in view.
//
//  Timing: call setTimestamp() before each process() with the computer's
//  clock (Unix seconds) at the block's first sample, and setClockOffset()
//  with the correction in use (PC - true, Decoder.h's sign).  As in
//  ft8Feed(), the stream's start on that clock is the earliest estimate
//  (blocks only arrive late), creeping up by 20 us a block to follow the
//  sound card, and a jump of over 1 s starts it again.  Without timestamps
//  the stream is taken to start at Unix time 0.
//
//  The decode at the end of a slot is heavy (one 2^20-point FFT, nine
//  375 Hz slices, a sync search over 2 minutes of start times, up to 12
//  candidates a slice through the Fano decoder): it runs inside the
//  process() call that crosses the slot end, about 0.7-0.8 s of CPU on a
//  recent desktop (lastDecodeCpuSeconds()), half of it Fano decoding
//  candidates that turn out to be noise.  A program that must not stall
//  its audio thread for that long calls process() from a worker.  Memory:
//  ~6 MB for the audio ring, ~20 MB more for the decode's buffers (made
//  on the first decode and kept).
//
//  Calibration (wsprNoteFreq()): every decode's audio frequency is noted
//  (the last 40 within 20 minutes, per dial frequency).  With 4 or more,
//  their median's distance from 1500 Hz is the stick's error: over 80 Hz
//  and an offer to correct is made (calibration().offer), ppm = error /
//  dial * 1e6 (stations landing high: + ppm, added to the correction in
//  OFFgrid-SDR's sign).  After calibrationApplied(), the next estimate
//  checks it and, if it did not help, suggests the other sign.
//
//  How it fits in
//  --------------
//  Implements dec::Decoder (Kind::Wspr).  Depends on Decoder.h and
//  src/dsp/Fft.h.  No allocation per sample (the audio path keeps a fixed
//  ring; buffers for the decode are made once, on the first decode).
//  Clock.h's ClockCorrection::applyFtxTimings() takes lastSlotDts().
//  WsprEncoders.h makes transmissions for tests and the demo band.
//
//  Contributing
//  ------------
//  Thresholds and search parameters are the browser's (opt.minSync 0.08,
//  opt.maxCand 12, Fano delta 2.4 and 4000 cycles a bit).  Deviations are
//  marked "Addition to the JS" in Wspr.cpp.  Keep both versions in step;
//  run tests/test_decoders_wspr.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_WSPR_H
#define OFFGRID_DECODERS_WSPR_H

#include "Decoder.h"
#include "../dsp/Fft.h"

#include <complex>
#include <string>
#include <utility>
#include <vector>

namespace dec {

// ---- the WSPR codec (the JS's pack, unpack, encode, symbols, fano) ----
namespace wspr {

const int kSymbols = 162;                  // channel symbols
const double kToneHz = 12000.0 / 8192.0;   // tone spacing = baud (1.4648)
const double kSymbolSeconds = 8192.0 / 12000.0;
const double kTxSeconds = 162 * 8192.0 / 12000.0;  // 110.592 s
extern const unsigned char kPr3[162];      // the sync sequence (PR3)

// The message.
struct Message {
    Message() : dbm(0) {}
    std::string call;   // e.g. "K1ABC", "G4OGR"
    std::string grid;   // 4-character locator, e.g. "FN42"
    int dbm;            // power, one of validDbm()
};
// Power levels a type 1 message can carry: 0, 3, 7, 10, ... 60 dBm.
bool validDbm(int dbm);
// Watts for a power in dBm, and the JS's wattsOf() text ("1 W", "200 mW").
double dbmToWatts(int dbm);
std::string wattsText(int dbm);

// pack(): callsign, locator, dBm -> 50 bits (0/1); false for what a type 1
// message cannot hold.  The power is rounded to a valid level (as the JS).
bool pack(const std::string& call, const std::string& grid, int dbm, unsigned char bits[50]);
// unpack(): 50 bits -> message; false for anything but a well-formed type 1
// message (types 2 and 3 included).
bool unpack(const unsigned char bits[50], Message& out);
// encode(): 50 bits -> 162 coded bits in transmission (interleaved) order.
void encode(const unsigned char bits[50], unsigned char coded[162]);
// symbols(): the 162 channel symbols 0..3 (PR3 + 2 x coded); false if the
// message cannot be packed.
bool symbols(const std::string& call, const std::string& grid, int dbm, unsigned char sym[162]);

// fano(): soft[162] are each coded bit's log-likelihood of being 1, in
// transmission order; on success the 50 message bits.  `delta` the
// threshold step, `cycles` the effort per bit (the JS's opt.delta,
// opt.cycles).
bool fano(const double soft[162], unsigned char bits[50], double delta = 2.4, int cycles = 4000);

}  // namespace wspr

// One decoded beacon.
struct WsprDecode {
    WsprDecode()
        : slot(0), slotStart(0), snrDb(0), dt(0), audioHz(0), rfHz(0), drift(0), dbm(0), watts(0), sync(0) {}
    long long slot;       // 2-minute slot number (true time / 120 s)
    double slotStart;     // its start, Unix seconds (UTC, true time)
    int snrDb;            // signal to noise in 2500 Hz, dB (WSJT's convention)
    double dt;            // start time - (slot start + 1 s), seconds, by the
                          // corrected clock (+ = late: the clock is fast)
    double audioHz;       // the signal's centre in the audio (Hz above the dial)
    double rfHz;          // dial + audioHz (0 when no dial is set)
    double drift;         // frequency change over the transmission, Hz
                          // (Addition to the JS: the JS reports none)
    std::string call;     // callsign
    std::string grid;     // 4-character locator
    int dbm;              // power, dBm
    double watts;         // the same in watts
    double sync;          // the sync search's score (0..1)
    std::string text;     // "G4OGR IO91 30 dBm" (the JS's r.text)
    std::string line;     // the readable line takeText() gives
};

// The stick's frequency error from where the stations land (wsprNoteFreq()).
struct WsprCalibration {
    WsprCalibration() : ready(false), offer(false), count(0), medianHz(0), errorHz(0), ppm(0), dialHz(0) {}
    bool ready;          // 4 or more decodes noted (on this dial, last 20 min)
    bool offer;          // |error| > 80 Hz with a dial set: worth correcting
    int count;           // decodes noted
    double medianHz;     // their median audio frequency
    double errorHz;      // medianHz - 1500
    double ppm;          // errorHz / dial * 1e6 (0 without a dial)
    double dialHz;
    std::string text;    // the JS's wsprCalText(), e.g. " · stick on frequency"
};

class WsprDecoder : public Decoder {
public:
    WsprDecoder();

    Kind kind() const override { return Kind::Wspr; }
    // Forgets the audio, timing, decodes and calibration notes.  Settings
    // (dial, search range, Fano effort) are kept.
    void reset() override;
    void process(const float* audio, std::size_t n, double rate) override;
    // One line per decode (see WsprDecode::line), e.g.
    //   "140200  -21 dB  +0.3 s  1462 Hz  14.097062 MHz  drift 0  G4OGR IO91 30 dBm (1 W)"
    void takeText(std::string& out) override;
    Status status() const override;
    double audioCenterHz() const override { return 1500; }
    void setTimestamp(double unixSeconds) override;
    void setClockOffset(double pcMinusTrue) override;

    // ---- settings ----
    // The dial frequency (Hz) for RF frequencies and calibration; 0 = not
    // known.  A different dial starts the calibration notes again (as the
    // JS); retuning should also call reset(), as for every decoder.
    void setDialFrequency(double hz);
    double dialFrequency() const { return dial_; }
    // The audio range searched (Hz above the dial); default 200-2800,
    // limited to 100-2900 (what the 3 kHz decimated stream holds).
    void setSearchRange(double loHz, double hiHz);
    // Fano effort (the JS's opt.delta, opt.cycles).
    void setFano(double delta, int cycles);

    // The WSPR dial frequencies (the browser's WSPR_DIALS / FTX_BANDS.WSPR).
    struct Band { const char* name; double dialHz; };
    static const std::vector<Band>& dialList();
    // The listed dial within 300 Hz of `hz` (wsprDial()); 0 if none.
    static double dialForFrequency(double hz);

    // ---- results ----
    // The decodes since the last call (structured).
    void takeDecodes(std::vector<WsprDecode>& out);
    // True once after each slot decode (whether or not anything was found).
    bool newSlotDone();
    // The dts of the last slot's new decodes, for
    // ClockCorrection::applyFtxTimings(dts, "WSPR", pcNow).
    const std::vector<double>& lastSlotDts() const { return lastDts_; }
    // The clock as WSPR measured it at the last slot with decodes: PC - true
    // seconds (= the offset in use then + the decodes' median dt).  False
    // if nothing was decoded yet.  `count` decodes, their dts' `spread`.
    bool measuredClockOffset(double& pcMinusTrue, int* count = 0, double* spread = 0) const;
    // The stick's frequency error (wsprNoteFreq(), wsprCalText()).
    WsprCalibration calibration() const;
    // wsprCalApply(): the program set its correction from oldPpm to
    // oldPpm + calibration().ppm; the notes start again and the next
    // estimate checks it.
    void calibrationApplied(double oldPpm);
    // The check after calibrationApplied(), once 4 more decodes are in:
    // true once; `flipped` if the correction made it worse (then
    // `suggestedPpm` is the other sign: old - error/dial*1e6), else it was
    // confirmed; `text` the JS's status line.
    bool takeCalibrationCheck(bool& flipped, double& suggestedPpm, std::string& text);

    // ---- work ----
    unsigned slotsDecoded() const { return slots_; }
    unsigned decodesTotal() const { return total_; }
    double lastDecodeCpuSeconds() const { return lastCpu_; }   // std::clock() time
    double lastDecodeWallSeconds() const { return lastWall_; } // steady clock
    double maxDecodeCpuSeconds() const { return maxCpu_; }
    // Seconds of audio held (at most 236).
    double secondsHeld() const;
    // Decodes the audio held now, as at a slot end (for tools and tests;
    // process() calls it by itself).  Returns the number of new decodes.
    int decodeNow();

private:
    struct Cand { double sync; int b; int m; };
    struct Demod { double soft[162]; int start; double f0; double sig; double noise; double drift; };
    struct Note { double f; double t; };
    struct Check { double old; double err; double f; };

    void setRate(double rate);
    void clearStream();
    void feed(const float* x, std::size_t n);
    void slotCheck();
    void decodeSlice(const std::complex<float>* s, int len, double fc, double span, std::vector<WsprDecode>& out,
                     std::vector<std::string>& seen);
    // demod() of the JS, in parts: amps(), the sync score, the fine timing and
    // frequency search (refine()), the drift search, and the bit LLRs (finish())
    void amps(const std::complex<float>* s, int len, int st, double f, double drift, int step, double* A) const;
    double syncScore(const std::complex<float>* s, int len, int st, double f, double drift);
    void refine(const std::complex<float>* s, int len, int& st, double& f, double drift, double& score, int firstLevel);
    void finish(const std::complex<float>* s, int len, int st, double f, double drift, Demod& res);
    void noteFreq(double f, double t);

    // settings
    double dial_, fLo_, fHi_, delta_; int cycles_;
    // timing (ft8Feed)
    bool tsPending_; double ts_; bool anchorSet_; double anchor_;
    double clockOff_;
    bool lastSlotSet_; long long lastSlot_;
    // resampler: real audio -> 3000 Hz complex around 1500 Hz (Addition to the JS)
    double rate_; int half_; int phases_; std::vector<float> taps_;
    std::vector<std::complex<float> > hist_; long long histMask_;
    long long inN_; long long outK_; double ratio_;
    double ncoC_, ncoS_, ncoStepC_, ncoStepS_; unsigned ncoCount_;
    // the decimated ring (at most 236 s)
    std::vector<std::complex<float> > ring_; long long outN_;
    // decode buffers (made on the first decode)
    std::vector<std::complex<float> > big_, slice_, frame_;
    dsp::detail::Fft fftBig_, fftSlice_, fft512_;
    std::vector<float> P_, V_, smp_;
    std::vector<Cand> cand_;
    std::vector<double> A_;
    // results
    std::vector<WsprDecode> pending_;
    std::string text_;
    std::vector<std::pair<long long, std::string> > keys_;
    std::vector<double> lastDts_;
    bool slotDone_;
    bool measured_; double measuredOff_; int measuredN_; double measuredSpread_;
    unsigned slots_, total_, lastCount_;
    int bestSnr_; double lastDecodeAt_;
    double lastCpu_, lastWall_, maxCpu_;
    // calibration
    double calDial_; std::vector<Note> notes_;
    bool offer_; double offerErr_, offerPpm_; int offerN_;
    bool checkSet_; Check check_;
    bool checkReady_; bool checkFlipped_; double checkPpm_; std::string checkText_;
};

}  // namespace dec

#endif
