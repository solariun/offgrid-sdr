// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/WsprEncoders.h
//  Layer:  Library liboffgrid-decoders (public header; test and demo signals)
//
//  What this file does
//  -------------------
//  Deterministic WSPR audio, as a receiver in USB with the dial on the WSPR
//  dial frequency hears it: a message -> 162 symbols (wspr::symbols(), the
//  JS's WSPR.symbols) -> continuous-phase 4-FSK, tones 1.4648 Hz apart,
//  1.4648 baud, 110.6 s, at any sample rate and audio frequency (the
//  frequency given is the signal's centre, as WSPR reports it: tone 0 lies
//  1.5 tones below it).
//
//  WsprAir renders a band: stations that transmit every 2-minute slot, 1 s
//  after each even minute (plus their own delay), with an optional drift,
//  and white Gaussian noise, block by block for any span of time.  The
//  demo stations are OFFgrid-SDR's demo band on 14.0956 MHz (diddy-boy,
//  index.html: G4OGR IO91 30 dBm at 1462 Hz, K1OGS FN42 37 dBm at 1531 Hz,
//  VK2OGS QF56 23 dBm at 1574 Hz; the JS's demo sets "late: 1.2", taken
//  here as their start 1 s after the minute, as WSPR's).
//
//  How it fits in
//  --------------
//  Used by tests/test_decoders_wspr.cpp and by the TUI's demo band.
//  Depends on Wspr.h and the C++11 standard library.  Deterministic: the
//  same arguments (and the same sequence of render() calls) give the same
//  samples.
//
//  Contributing
//  ------------
//  Keep the symbol timing and tone layout in step with WsprDecoder.
// ============================================================================
#ifndef OFFGRID_DECODERS_WSPRENCODERS_H
#define OFFGRID_DECODERS_WSPRENCODERS_H

#include "Wspr.h"

#include <string>
#include <vector>

namespace dec {
namespace enc {

// One WSPR transmitter.
struct WsprStation {
    WsprStation() : dbm(0), audioHz(1500), amplitude(0.01), delay(1), driftHz(0) {}
    WsprStation(const std::string& c, const std::string& g, int d, double hz, double amp)
        : call(c), grid(g), dbm(d), audioHz(hz), amplitude(amp), delay(1), driftHz(0) {}
    std::string call, grid;
    int dbm;
    double audioHz;     // the signal's centre in the audio (Hz above the dial)
    double amplitude;   // peak amplitude of the tone
    double delay;       // start after the even minute, seconds (WSPR: 1)
    double driftHz;     // frequency change over the transmission (Hz)
};

// One transmission, 110.592 s at `rate` (empty if the message cannot be
// packed).  The tone of symbol i is audioHz + (sym - 1.5) * 1.4648
// + driftHz * (i - 81) / 162, with continuous phase.
std::vector<float> wsprTransmission(const WsprStation& st, double rate);

// The noise rms that puts a station of amplitude `amplitude` at `snrDb`
// in WSPR's 2500 Hz reference bandwidth, for white noise at `rate`.
double wsprNoiseFor(double amplitude, double snrDb, double rate);

// The demo band's stations (14.0956 MHz).  The JS's amplitudes are levels
// on the radio's input; `scale` turns them into audio levels.
std::vector<WsprStation> demoWsprStations(double scale = 100);

// A band of stations plus noise, rendered for any span of (true) time.
class WsprAir {
public:
    WsprAir(double rate, const std::vector<WsprStation>& stations, double noiseRms, unsigned seed = 12345);
    // Writes n samples, sample i at Unix time t0 + i / rate (true time).
    // Consecutive calls should continue in time (the phases carry on).
    void render(double t0, float* out, std::size_t n);
    double rate() const { return rate_; }

private:
    struct Tx { WsprStation st; unsigned char sym[162]; bool ok; double phase; };
    double gauss();
    double rate_, noise_;
    std::vector<Tx> tx_;
    unsigned long long rng_;
    bool haveSpare_; double spare_;
};

}  // namespace enc
}  // namespace dec

#endif
