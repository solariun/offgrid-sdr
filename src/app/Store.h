// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/Store.h
//  Layer:  Application (what is kept between runs)
//
//  What this file does
//  -------------------
//  app::Store keeps, in $XDG_CONFIG_HOME/offgrid (or ~/.config/offgrid):
//    settings.ini   the last frequency, mode, volume, theme ... ([settings]),
//                   the memories ([memories]), the frequency labels
//                   ([labels]) and each band's last frequency ([bands])
//    logbook.csv    the log, one line per entry (UTC time, frequency, mode,
//                   level, SNR, station, notes), appended, never rewritten,
//                   readable by any spreadsheet
//
//  Format of settings.ini (plain text, safe to edit by hand):
//      [settings]
//      frequency=94000000
//      mode=WFM
//      [memories]
//      GLOW FM|94000000|WFM|0
//      [labels]
//      99400000|Veronica
//  Text fields never contain '|' or line breaks (they are replaced by
//  spaces on the way in).
// ============================================================================
#ifndef OFFGRID_APP_STORE_H
#define OFFGRID_APP_STORE_H

#include <map>
#include <string>
#include <vector>

#include "../dsp/Receiver.h"

namespace app {

struct Memory {
    std::string name;
    double hz;
    dsp::Mode mode;
    double bandwidth;  // 0 = the mode's default
};

struct Label {
    double hz;
    std::string text;
};

struct LogEntry {
    std::string utc;      // "2026-11-25 14:03:07"
    double hz;
    std::string mode;     // "WFM", or "RTTY" etc. when a decoder was on
    double levelDb, snrDb;
    std::string station;  // RDS name, or what the user typed
    std::string notes;
};

class Store {
public:
    // The directory (created on save).  OFFGRID_CONFIG overrides it (tests).
    static std::string directory();

    // Reads settings.ini; a missing file is not an error.
    bool load(std::string& error);
    bool save(std::string& error) const;

    // [settings] values.
    std::string get(const std::string& key, const std::string& fallback = std::string()) const;
    double getNumber(const std::string& key, double fallback) const;
    void set(const std::string& key, const std::string& value);
    void setNumber(const std::string& key, double value);
    bool has(const std::string& key) const { return values.count(key) != 0; }

    std::map<std::string, std::string> values;
    std::map<std::string, double> bandMemory;  // band name -> last frequency
    std::vector<Memory> memories;              // in the order added
    std::vector<Label> labels;                 // sorted by frequency

    // Memories and labels helpers.
    void addMemory(const Memory& m);           // replaces one with the same name
    void addLabel(double hz, const std::string& text);  // replaces one within 500 Hz
    bool removeLabelNear(double hz, double within);

    // The logbook.
    static bool appendLog(const LogEntry& entry, std::string& error);
    static std::vector<LogEntry> readLog(std::size_t newestCount);
    static std::string logPath();
};

// "2026-11-25 14:03:07" (UTC now).
std::string utcNow();
// A field made safe for the files (no '|', ',', '"', line breaks).
std::string clean(const std::string& text);

}  // namespace app

#endif
