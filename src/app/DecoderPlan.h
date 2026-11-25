// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/DecoderPlan.h
//  Layer:  Application (how the radio is set for each decoder)
//
//  What this file does
//  -------------------
//  app::planDecoder(kind, frequency): the mode, filter, dial shift and
//  decoder settings for listening with a decoder at a frequency, following
//  the browser version (modeShift, sstvSb, the AX.25 speeds):
//    CW                CW mode, no shift
//    RTTY/NAVTEX/DSC   USB, 3 kHz, dial 1.5 kHz below the signal
//    PSK, HELL         USB, 3 kHz, dial 1 kHz below
//    FAX               USB, 3 kHz, dial 1.9 kHz below
//    SSTV              on the listed frequency: LSB below 10 MHz (40/80/160 m),
//                      NFM above 30 MHz (e.g. the ISS 145.800), USB between
//    AX.25             above 30 MHz: NFM 12.5 kHz, 1200 Bd (APRS 144.800 /
//                      144.390); below: USB, 300 Bd, dial on the listed dial
//    TIME              USB, dial 1 kHz below the station; the station from
//                      the frequency (CHU, WWV, RWM)
//  The interface (Ui) and the headless mode (main) both use it.
// ============================================================================
#ifndef OFFGRID_APP_DECODERPLAN_H
#define OFFGRID_APP_DECODERPLAN_H

#include <functional>

#include "../decoders/Decoder.h"
#include "../dsp/Receiver.h"

namespace app {

struct DecoderPlan {
    dsp::Mode mode;
    double bandwidth;   // 0 = the mode's default
    double shift;       // Hz the receiver sits below the dial
    std::function<void(dec::Decoder&)> setup;  // decoder settings (may be empty)
};

DecoderPlan planDecoder(dec::Kind kind, double hz);

}  // namespace app

#endif
