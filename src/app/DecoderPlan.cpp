// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/DecoderPlan.cpp
//  Layer:  Application (how the radio is set for each decoder)
//
//  What this file does
//  -------------------
//  Implementation of DecoderPlan.h (the table is in the header comment).
// ============================================================================
#include "DecoderPlan.h"

#include "../decoders/Ax25.h"
#include "../decoders/Time.h"

namespace app {

DecoderPlan planDecoder(dec::Kind kind, double hz) {
    DecoderPlan p;
    p.mode = dsp::Mode::USB;
    p.bandwidth = 3000;
    p.shift = 0;
    p.centerOnDial = false;
    switch (kind) {
        case dec::Kind::Cw:
            p.mode = dsp::Mode::CW;
            p.bandwidth = 0;
            break;
        case dec::Kind::Rtty:
        case dec::Kind::Navtex:
        case dec::Kind::Dsc: p.shift = 1500; break;
        case dec::Kind::Psk:
        case dec::Kind::Hell: p.shift = 1000; break;
        case dec::Kind::Time: {
            p.shift = 1000;
            dec::TimeStation st;
            if (dec::TimeDecoder::stationForFrequency(hz, st)) {
                p.setup = [st](dec::Decoder& d) {
                    if (dec::TimeDecoder* t = dynamic_cast<dec::TimeDecoder*>(&d)) t->setStation(st);
                };
            }
            break;
        }
        case dec::Kind::Fax: p.shift = 1900; break;
        case dec::Kind::Sstv:
            if (hz > 30e6) {
                p.mode = dsp::Mode::NFM;
                p.bandwidth = 12500;
            } else if (hz < 10e6) {
                p.mode = dsp::Mode::LSB;
            }
            break;
        case dec::Kind::Acars:  // AM, on the channel (131.550, 131.725 ... MHz)
            p.mode = dsp::Mode::AM;
            p.bandwidth = 0;
            break;
        case dec::Kind::Vdl2:   // IQ: every channel 136.6-137 MHz in the span
            p.mode = dsp::Mode::AM;
            p.bandwidth = 0;
            break;
        case dec::Kind::Ais:    // IQ: both channels; the dial between them (162.000)
            p.mode = dsp::Mode::NFM;
            p.bandwidth = 0;
            break;
        case dec::Kind::Ft8:
        case dec::Kind::Ft4:
        case dec::Kind::Wspr: break;  // USB, 3 kHz, the dial on the listed frequency
        case dec::Kind::Dab:
            p.mode = dsp::Mode::WFM;
            p.bandwidth = 0;
            p.centerOnDial = true;
            break;
        case dec::Kind::Drm:
            p.mode = dsp::Mode::AM;
            p.bandwidth = 20000;
            break;
        case dec::Kind::Ax25: {
            const bool vhf = hz > 30e6;
            if (vhf) {
                p.mode = dsp::Mode::NFM;
                p.bandwidth = 12500;
            }
            const int baud = vhf ? 1200 : 300;
            p.setup = [baud](dec::Decoder& d) {
                if (dec::Ax25Decoder* a = dynamic_cast<dec::Ax25Decoder*>(&d)) a->setBaud(baud);
            };
            break;
        }
    }
    return p;
}

}  // namespace app
