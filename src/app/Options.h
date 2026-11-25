// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/Options.h
//  Layer:  Application (command line)
//
//  What this file does
//  -------------------
//  app::Options: everything that can be set on the command line, parsed by
//  parse() and documented by usage() (offgrid --help).
// ============================================================================
#ifndef OFFGRID_APP_OPTIONS_H
#define OFFGRID_APP_OPTIONS_H

#include <string>

#include "Radio.h"

namespace app {

enum class SourceKind { Rtl, Tcp, File, Demo };

struct Options {
    Options();

    SourceKind source;
    int device;              // --device N (RTL-SDR index)
    std::string tcp;         // --tcp host[:port]
    std::string file;        // --file path.cu8
    double fileCenter;       // --center Hz (file's centre frequency)
    double sampleRate;       // --rate Hz
    RadioSettings radio;     // --freq, --mode, --gain, --ppm, --bias-tee ...

    std::string theme;       // --theme
    std::string colormap;    // --colormap ("" = the theme's)
    bool trueColor;          // default true; --256 for xterm-256 colors
    bool mouse;              // default true; --no-mouse
    bool audio;              // default true; --no-audio
    std::string audioDevice; // --audio hw:0,0 | pipe:<cmd>

    bool list;               // --list
    bool help, version;
    double headlessSeconds;  // --headless N: no interface, print status lines
    std::string wav;         // --wav out.wav: record the demodulated sound
};

// Parses argv into opts.  Returns "" or an error message.
std::string parse(int argc, char** argv, Options& opts);
std::string usage();
const char* versionString();

}  // namespace app

#endif
