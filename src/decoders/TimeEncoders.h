// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/TimeEncoders.h
//  Layer:  Library liboffgrid-decoders (public header; test and demo signals)
//
//  What this file does
//  -------------------
//  Deterministic audio of the time stations CHU, WWV, WWVH and RWM for a
//  given UTC start time and length, as a receiver in USB with the dial
//  1 kHz below the station hears them (the carrier a 1000 Hz tone; see
//  Time.h).  Based on OFFgrid-SDR's demo time stations (diddy-boy,
//  index.html: demoChuBytes, demoWwvBits and the "timesig" generator), made
//  closer to the real stations ("Addition to the JS"):
//
//    CHU   carrier with its upper sideband only (CHU sends H3E): 1000 Hz
//          ticks of 300 ms (10 ms in seconds 31-39 and 51-59, none in 29),
//          0.5 s minute tone (1 s at the hour), and in seconds 31-39 the
//          Bell 103 FSK code (mark 2225 / space 2025 Hz above the carrier,
//          300 baud, the mark tone from 10 ms, ten bytes ending at 500 ms;
//          second 31 format B with the year, 32-39 format A).
//    WWV   AM (both sidebands, those above 0 Hz kept): 5 ms 1000 Hz ticks
//          (none in 29 and 59), 0.8 s minute tone (1000 Hz; 1500 Hz at the
//          hour), the 100 Hz BCD subcarrier (high for 170 / 470 / 770 ms
//          from 30 ms in, 15 dB lower otherwise), and with `tones` the
//          500 / 600 Hz tones in seconds 1-44 (silent from 10 ms before to
//          30 ms after each tick; 440 Hz in minute 2; none in minutes 29
//          and 59).  Which minutes carry 500 or 600 Hz, and the
//          subcarrier's 18% depth, are approximations of NIST's format.
//    WWVH  the same with 1200 Hz ticks and minute tone, 500/600 swapped
//          and 440 Hz in minute 1.
//    RWM   the carrier keyed: 100 ms each second, 500 ms at the minute
//          (no DUT1 doubled pulses, no call sign).
//
//  No voice announcements.  Levels, the carrier's audio frequency (to
//  imitate a stick off frequency) and white noise are adjustable.
//
//  How it fits in
//  --------------
//  Used by tests/test_decoders_time.cpp and by the TUI's demo band.
//  Depends on Time.h (TimeStation and the UTC helpers) and the C++11
//  standard library.  Deterministic: the same arguments give the same
//  samples.
//
//  Contributing
//  ------------
//  Keep the code layouts in step with TimeDecoder (Time.cpp).
// ============================================================================
#ifndef OFFGRID_DECODERS_TIMEENCODERS_H
#define OFFGRID_DECODERS_TIMEENCODERS_H

#include "Time.h"

#include <vector>

namespace dec {
namespace enc {

struct TimeSignalOptions {
    TimeSignalOptions() : carrierHz(1000), level(0.25), noiseRms(0), seed(1), tones(true) {}
    double carrierHz;  // the carrier's audio frequency (1000 = on frequency)
    double level;      // the carrier's amplitude (the sidebands scale with it)
    double noiseRms;   // white Gaussian noise added (0 = none)
    unsigned seed;     // the noise's seed
    bool tones;        // WWV / WWVH: the 440 / 500 / 600 Hz tones
};

// `seconds` of audio at `rate` Hz, the first sample at UTC `utcStart` (Unix
// seconds, any fraction) of the station's own clock.  Wwv and Wwvh are the
// two stations (not the decoder's setting).
std::vector<float> timeSignalAudio(TimeStation station, double utcStart, double seconds, double rate,
                                   const TimeSignalOptions& opt = TimeSignalOptions());

// CHU's ten bytes for the second starting at UTC `utcSecond` (whole): second
// 31 format B (0, 1, year, TAI-UTC 37, 0, 0; then inverted), 32-39 format A
// ("6 ddd hh mm ss", twice); empty for other seconds.  Each byte holds two
// digits, the first in its low half.
std::vector<int> chuCodeBytes(double utcSecond);

// WWV's 60 code values for the minute starting at UTC `utcMinute`: 0 / 1
// bits, 2 a position marker (seconds 9, 19 .. 59), -1 no pulse (second 0).
std::vector<int> wwvCodeBits(double utcMinute);

// White Gaussian noise of the given rms added to `audio` (deterministic).
void addTimeNoise(std::vector<float>& audio, double rms, unsigned seed);

}  // namespace enc
}  // namespace dec

#endif
