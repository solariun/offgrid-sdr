// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/Bands.h
//  Layer:  Application (band plan)
//
//  What this file does
//  -------------------
//  The band plan shown in the Bands list: each band's range, the mode and
//  tuning step it opens with, and the frequency it opens on.  Follows the
//  browser version's bands (broadcast, amateur, airband, marine, PMR446, CB,
//  weather) with the usual worldwide allocations.
//
//  How it fits in
//  --------------
//  app::Ui lists them and, on a choice, asks app::Radio to tune to the
//  band's last (or default) frequency in its mode and step.  Each band
//  remembers the last frequency used in it for this session.
// ============================================================================
#ifndef OFFGRID_APP_BANDS_H
#define OFFGRID_APP_BANDS_H

#include <cstdint>
#include <string>
#include <vector>

#include "../dsp/Receiver.h"

namespace app {

struct Band {
    std::string group;    // "Broadcast", "Amateur", "Air & sea", "Other"
    std::string name;     // "FM", "40 m", "Airband" ...
    double low, high;     // Hz
    double start;         // Hz: where it opens
    dsp::Mode mode;
    double step;          // Hz
};

// The whole plan, in display order.
const std::vector<Band>& bands();
// The band containing `hz` (the narrowest one), or nullptr.
const Band* bandAt(double hz);

// DAB Band III channels (5A ... 13F), name and centre frequency (Hz).
struct DabChannel {
    const char* name;
    double hz;
};
const std::vector<DabChannel>& dabChannels();
// The channel at (within 1 kHz of) hz, or nullptr; the nearest channel.
const DabChannel* dabChannelAt(double hz);
const DabChannel& dabNearest(double hz);

// "145.800.000" style (MHz . kHz . Hz groups) and "145.8 MHz" short form.
std::string formatFrequency(double hz);
std::string formatShort(double hz);
// Parses "100.3", "100.3M", "7074k", "1090 MHz", "162025000" (no unit:
// below 2000 = MHz, else Hz).  false if not a frequency.
bool parseFrequency(const std::string& text, double& hz);

}  // namespace app

#endif
