// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/Ui.h
//  Layer:  Application (the terminal user interface)
//
//  What this file does
//  -------------------
//  app::Ui is the whole screen, built from Graphicx widgets
//  (external/Graphicx, widgets.h):
//
//   ┌ big frequency ─ mode buttons ─ band / step ─ lights ── S-meter ─ clock ┐
//   │ spectrum (braille plot, passband shaded, dial marker)  │ Bands list    │
//   │ waterfall (half blocks, colormap)                      │ Controls      │
//   │ RDS: station name, programme type, radio text                          │
//   └ status: source, rate, DSP load, audio, keys ──────────────────────────┘
//
//  Keyboard and mouse both drive everything (see the key list in
//  Options.cpp's usage() and the F1 help).  With no widget focused, keys
//  tune the radio; Tab focuses the side controls, Esc gives the keys back
//  to tuning.
//
//  How it fits in
//  --------------
//  main.cpp builds the Radio and runs Ui::run(); the Ui reads a Snapshot of
//  the Radio 30 times a second and sends commands back through Radio's
//  setters.  It never touches the DSP directly.
// ============================================================================
#ifndef OFFGRID_APP_UI_H
#define OFFGRID_APP_UI_H

#include <memory>

#include "Options.h"
#include "Radio.h"

namespace app {

class Ui {
public:
    Ui(Radio& radio, const Options& options);
    ~Ui();
    // Runs until the user quits (q, Ctrl+C).  Returns the exit code.
    int run();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace app

#endif
