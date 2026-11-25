// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/Store.cpp
//  Layer:  Application (what is kept between runs)
//
//  What this file does
//  -------------------
//  Implementation of Store.h: the settings file (a small INI reader and
//  writer) and the CSV logbook.  Saving writes settings.ini.tmp and renames
//  it over the old file, so a crash never leaves a half-written file.
// ============================================================================
#include "Store.h"

#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>

namespace app {

namespace {

std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

std::vector<std::string> split(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::string cur;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == sep) {
            out.push_back(cur);
            cur.clear();
        } else {
            cur += s[i];
        }
    }
    out.push_back(cur);
    return out;
}

std::string number(double v) {
    char b[32];
    if (v == std::floor(v) && std::fabs(v) < 1e15) std::snprintf(b, sizeof(b), "%.0f", v);
    else std::snprintf(b, sizeof(b), "%.6g", v);
    return b;
}

// mkdir -p for one or two levels (~/.config/offgrid).
void makeDirs(const std::string& path) {
    for (size_t i = 1; i <= path.size(); ++i) {
        if (i == path.size() || path[i] == '/') mkdir(path.substr(0, i).c_str(), 0755);
    }
}

}  // namespace

std::string clean(const std::string& text) {
    std::string s = text;
    for (size_t i = 0; i < s.size(); ++i)
        if (s[i] == '|' || s[i] == ',' || s[i] == '"' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t') s[i] = ' ';
    return trim(s);
}

std::string utcNow() {
    std::time_t now = std::time(0);
    char b[32];
    std::strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", std::gmtime(&now));
    return b;
}

std::string Store::directory() {
    if (const char* o = std::getenv("OFFGRID_CONFIG")) return o;
    if (const char* x = std::getenv("XDG_CONFIG_HOME"))
        if (*x) return std::string(x) + "/offgrid";
    const char* home = std::getenv("HOME");
    return std::string(home ? home : ".") + "/.config/offgrid";
}

std::string Store::logPath() { return directory() + "/logbook.csv"; }

bool Store::load(std::string& error) {
    std::ifstream in((directory() + "/settings.ini").c_str());
    if (!in) return true;  // first run
    std::string line, section;
    int lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        line = trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        if (line[0] == '[') {
            section = line.substr(1, line.find(']') == std::string::npos ? std::string::npos : line.find(']') - 1);
            continue;
        }
        if (section == "settings") {
            size_t eq = line.find('=');
            if (eq != std::string::npos) values[trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
        } else if (section == "bands") {
            size_t eq = line.find('=');
            if (eq != std::string::npos) bandMemory[trim(line.substr(0, eq))] = std::atof(line.substr(eq + 1).c_str());
        } else if (section == "memories") {
            std::vector<std::string> f = split(line, '|');
            if (f.size() < 3) continue;
            Memory m;
            m.name = f[0];
            m.hz = std::atof(f[1].c_str());
            if (!dsp::parseMode(f[2], m.mode)) m.mode = dsp::Mode::AM;
            m.bandwidth = f.size() > 3 ? std::atof(f[3].c_str()) : 0;
            if (m.hz > 0) memories.push_back(m);
        } else if (section == "labels") {
            size_t bar = line.find('|');
            if (bar == std::string::npos) continue;
            Label l = {std::atof(line.substr(0, bar).c_str()), line.substr(bar + 1)};
            if (l.hz > 0) labels.push_back(l);
        } else {
            error = "settings.ini line " + std::to_string(lineNo) + ": outside a known section (ignored)";
        }
    }
    std::sort(labels.begin(), labels.end(), [](const Label& a, const Label& b) { return a.hz < b.hz; });
    return true;
}

bool Store::save(std::string& error) const {
    std::string dir = directory();
    makeDirs(dir);
    std::string tmp = dir + "/settings.ini.tmp";
    {
        std::ofstream out(tmp.c_str());
        if (!out) {
            error = "cannot write " + tmp;
            return false;
        }
        out << "# offgrid settings (written on exit; safe to edit while offgrid is closed)\n\n[settings]\n";
        for (std::map<std::string, std::string>::const_iterator i = values.begin(); i != values.end(); ++i)
            out << i->first << "=" << i->second << "\n";
        out << "\n[bands]\n";
        for (std::map<std::string, double>::const_iterator i = bandMemory.begin(); i != bandMemory.end(); ++i)
            out << i->first << "=" << number(i->second) << "\n";
        out << "\n[memories]\n# name|frequency Hz|mode|filter Hz (0 = default)\n";
        for (size_t i = 0; i < memories.size(); ++i)
            out << clean(memories[i].name) << "|" << number(memories[i].hz) << "|" << dsp::modeName(memories[i].mode) << "|"
                << number(memories[i].bandwidth) << "\n";
        out << "\n[labels]\n# frequency Hz|text\n";
        for (size_t i = 0; i < labels.size(); ++i) out << number(labels[i].hz) << "|" << clean(labels[i].text) << "\n";
        if (!out) {
            error = "cannot write " + tmp;
            return false;
        }
    }
    if (std::rename(tmp.c_str(), (dir + "/settings.ini").c_str()) != 0) {
        error = "cannot replace " + dir + "/settings.ini";
        return false;
    }
    return true;
}

std::string Store::get(const std::string& key, const std::string& fallback) const {
    std::map<std::string, std::string>::const_iterator i = values.find(key);
    return i == values.end() ? fallback : i->second;
}

double Store::getNumber(const std::string& key, double fallback) const {
    std::map<std::string, std::string>::const_iterator i = values.find(key);
    if (i == values.end() || i->second.empty()) return fallback;
    char* end = 0;
    double v = std::strtod(i->second.c_str(), &end);
    return end && *end == '\0' ? v : fallback;
}

void Store::set(const std::string& key, const std::string& value) { values[key] = clean(value); }
void Store::setNumber(const std::string& key, double value) { values[key] = number(value); }

void Store::addMemory(const Memory& m) {
    Memory c = m;
    c.name = clean(m.name);
    for (size_t i = 0; i < memories.size(); ++i) {
        if (memories[i].name == c.name) {
            memories[i] = c;
            return;
        }
    }
    memories.push_back(c);
}

void Store::addLabel(double hz, const std::string& text) {
    removeLabelNear(hz, 500);
    Label l = {hz, clean(text)};
    if (l.text.empty()) return;
    labels.push_back(l);
    std::sort(labels.begin(), labels.end(), [](const Label& a, const Label& b) { return a.hz < b.hz; });
}

bool Store::removeLabelNear(double hz, double within) {
    for (size_t i = 0; i < labels.size(); ++i) {
        if (std::fabs(labels[i].hz - hz) <= within) {
            labels.erase(labels.begin() + static_cast<long>(i));
            return true;
        }
    }
    return false;
}

bool Store::appendLog(const LogEntry& e, std::string& error) {
    std::string dir = directory();
    makeDirs(dir);
    std::string path = logPath();
    bool fresh = !std::ifstream(path.c_str());
    std::ofstream out(path.c_str(), std::ios::app);
    if (!out) {
        error = "cannot write " + path;
        return false;
    }
    if (fresh) out << "utc,frequency_hz,mode,level_dbfs,snr_db,station,notes\n";
    char nums[96];
    std::snprintf(nums, sizeof(nums), "%.0f,%s,%.1f,%.1f", e.hz, clean(e.mode).c_str(), e.levelDb, e.snrDb);
    out << clean(e.utc) << "," << nums << "," << clean(e.station) << "," << clean(e.notes) << "\n";
    return static_cast<bool>(out);
}

std::vector<LogEntry> Store::readLog(std::size_t newestCount) {
    std::vector<LogEntry> all;
    std::ifstream in(logPath().c_str());
    std::string line;
    bool header = true;
    while (std::getline(in, line)) {
        if (header) {
            header = false;
            if (line.compare(0, 4, "utc,") == 0) continue;
        }
        std::vector<std::string> f = split(line, ',');
        if (f.size() < 7) continue;
        LogEntry e;
        e.utc = f[0];
        e.hz = std::atof(f[1].c_str());
        e.mode = f[2];
        e.levelDb = std::atof(f[3].c_str());
        e.snrDb = std::atof(f[4].c_str());
        e.station = f[5];
        e.notes = f[6];
        all.push_back(e);
    }
    if (all.size() > newestCount) all.erase(all.begin(), all.end() - static_cast<long>(newestCount));
    std::reverse(all.begin(), all.end());  // newest first
    return all;
}

}  // namespace app
