// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/Options.cpp
//  Layer:  Application (command line)
//
//  What this file does
//  -------------------
//  Command-line parsing and the --help text.  Every option is documented in
//  usage(); keep the two in step (and the README's option table).
// ============================================================================
#include "Options.h"

#include <cstdlib>

#include "Bands.h"

namespace app {

Options::Options()
    : source(SourceKind::Rtl),
      device(0),
      fileCenter(100e6),
      sampleRate(2048000),
      theme("amber"),
      trueColor(true),
      mouse(true),
      audio(true),
      list(false),
      help(false),
      version(false),
      headlessSeconds(0) {}

const char* versionString() { return "offgrid 0.1.0 (C++11 terminal port of OFFgrid-SDR 2.5)"; }

std::string parse(int argc, char** argv, Options& o) {
    bool freqGiven = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        bool hasValue = i + 1 < argc;
        std::string v = hasValue ? argv[i + 1] : std::string();
        if (a == "-h" || a == "--help") o.help = true;
        else if (a == "-V" || a == "--version") o.version = true;
        else if (a == "--list") o.list = true;
        else if (a == "--demo") o.source = SourceKind::Demo;
        else if (a == "--device" && hasValue) {
            o.source = SourceKind::Rtl;
            o.device = std::atoi(v.c_str());
            ++i;
        } else if (a == "--tcp" && hasValue) {
            o.source = SourceKind::Tcp;
            o.tcp = v;
            ++i;
        } else if (a == "--file" && hasValue) {
            o.source = SourceKind::File;
            o.file = v;
            ++i;
        } else if (a == "--center" && hasValue) {
            if (!parseFrequency(v, o.fileCenter)) return "--center: not a frequency: " + v;
            ++i;
        } else if ((a == "-f" || a == "--freq") && hasValue) {
            if (!parseFrequency(v, o.radio.frequency)) return "--freq: not a frequency: " + v;
            freqGiven = true;
            ++i;
        } else if ((a == "-m" || a == "--mode") && hasValue) {
            if (!dsp::parseMode(v, o.radio.mode)) return "--mode: one of wfm, nfm, am, sam, usb, lsb, cw";
            ++i;
        } else if ((a == "-r" || a == "--rate") && hasValue) {
            if (!parseFrequency(v, o.sampleRate)) return "--rate: not a rate: " + v;  // "2.4" = 2.4 MS/s
            ++i;
        } else if ((a == "-g" || a == "--gain") && hasValue) {
            o.radio.gainDb = (v == "auto") ? -1 : std::atof(v.c_str());
            ++i;
        } else if (a == "--ppm" && hasValue) {
            o.radio.ppm = std::atof(v.c_str());
            ++i;
        } else if (a == "--bias-tee") o.radio.biasTee = true;
        else if (a == "--direct" && hasValue) {
            o.radio.directSampling = std::atoi(v.c_str());
            ++i;
        } else if (a == "--squelch" && hasValue) {
            o.radio.squelchDb = std::atof(v.c_str());
            ++i;
        } else if (a == "--volume" && hasValue) {
            o.radio.volume = std::atof(v.c_str()) / 100.0;
            ++i;
        } else if (a == "--deemph" && hasValue) {
            o.radio.deemphasisUs = std::atof(v.c_str());
            ++i;
        } else if (a == "--mono") o.radio.stereo = false;
        else if (a == "--fft" && hasValue) {
            int n = std::atoi(v.c_str());
            if (n < 256 || n > 65536 || (n & (n - 1))) return "--fft: a power of two from 256 to 65536";
            o.radio.fftSize = n;
            ++i;
        } else if (a == "--theme" && hasValue) {
            o.theme = v;
            ++i;
        } else if (a == "--colormap" && hasValue) {
            o.colormap = v;
            ++i;
        } else if (a == "--256") o.trueColor = false;
        else if (a == "-t" || a == "--truecolor") o.trueColor = true;
        else if (a == "--no-mouse") o.mouse = false;
        else if (a == "--no-audio") o.audio = false;
        else if (a == "--audio" && hasValue) {
            o.audioDevice = v;
            ++i;
        } else if (a == "--headless" && hasValue) {
            o.headlessSeconds = std::atof(v.c_str());
            ++i;
        } else if (a == "--wav" && hasValue) {
            o.wav = v;
            ++i;
        } else {
            return "unknown option: " + a + " (see --help)";
        }
    }
    // Without --freq, a band's start frequency fitting the mode.
    if (!freqGiven && o.source == SourceKind::File) o.radio.frequency = o.fileCenter;
    return std::string();
}

std::string usage() {
    return std::string(versionString()) +
           "\n\n"
           "A software-defined radio receiver for the terminal: RTL-SDR sticks (own driver, no\n"
           "librtlsdr), rtl_tcp servers, IQ recordings and a demo band; spectrum, waterfall,\n"
           "WFM stereo with RDS, NFM, AM, SAM, USB, LSB and CW.\n"
           "\n"
           "Usage: offgrid [options]\n"
           "\n"
           "Source (default: the first RTL-SDR stick)\n"
           "  --list                  list the RTL-SDR sticks and exit\n"
           "  --device N              use stick N (from --list)\n"
           "  --tcp HOST[:PORT]       an rtl_tcp server (port 1234 by default)\n"
           "  --file PATH             raw unsigned 8-bit IQ (rtl_sdr output), looped\n"
           "  --center FREQ           the recording's centre frequency (with --file)\n"
           "  --demo                  simulated stations, no hardware needed\n"
           "\n"
           "Radio\n"
           "  -f, --freq FREQ         dial frequency: 100.3, 7074k, 145.8M, 1090MHz (default 100 MHz)\n"
           "  -m, --mode MODE         wfm, nfm, am, sam, usb, lsb, cw (default wfm)\n"
           "  -r, --rate RATE         sample rate, e.g. 2.048 or 2400000 (default 2.048 MS/s)\n"
           "  -g, --gain DB|auto      tuner gain (default auto)\n"
           "  --ppm N                 frequency correction in ppm\n"
           "  --bias-tee              power an active antenna through the coax\n"
           "  --direct 0|1|2          direct sampling for HF below 28.8 MHz (2 = Q branch)\n"
           "  --squelch DB            mute below this channel level (dBFS)\n"
           "  --volume PCT            0-200 (default 80)\n"
           "  --deemph US             FM de-emphasis: 50 (Europe, default) or 75 (Americas)\n"
           "  --mono                  WFM in mono\n"
           "  --fft N                 spectrum resolution, power of two (default 2048)\n"
           "\n"
           "Display and sound\n"
           "  --theme NAME            amber, orange, lime, green, cyan, vfd, blue, violet, rose,\n"
           "                          mono, light (default amber; t cycles)\n"
           "  --colormap NAME         waterfall colors: classic, turbo, inferno, viridis, ice,\n"
           "                          grey, amber, green, hot (default: the theme's; c cycles)\n"
           "  -t, --truecolor         24-bit colors (default)\n"
           "  --256                   xterm 256-color palette (older terminals, slow links)\n"
           "  --no-mouse              don't capture the mouse\n"
           "  --no-audio              no sound\n"
           "  --audio DEV             Linux: hw:CARD,DEV (ALSA) or pipe:COMMAND\n"
           "\n"
           "Other\n"
           "  --headless SECONDS      no interface: print the signal, RDS and status each second\n"
           "  --wav FILE              record the sound to a WAV file (also R in the interface)\n"
           "  -V, --version           version\n"
           "  -h, --help              this text\n"
           "\n"
           "Keys (F1 in the program shows them all)\n"
           "  Left/Right tune by the step   Up/Down step size   PgUp/PgDn 1 MHz\n"
           "  Enter or f type a frequency   1-7 WFM NFM AM SAM USB LSB CW   b bands\n"
           "  +/- volume  space mute  g/G gain (a auto)  s/S squelch  z/Z zoom\n"
           "  [ ] waterfall floor  { } waterfall range  t theme  c colors  R record  q quit\n"
           "  Mouse: click the spectrum or waterfall to tune, wheel to step.\n";
}

}  // namespace app
