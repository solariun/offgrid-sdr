// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/Bands.cpp
//  Layer:  Application (band plan)
//
//  What this file does
//  -------------------
//  The band table and the frequency text helpers (see Bands.h).
//
//  Contributing
//  ------------
//  Add a band with one line in the table; keep `start` inside [low, high]
//  and the step a sensible channel spacing for the mode.
// ============================================================================
#include "Bands.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace app {

namespace {

Band band(const char* group, const char* name, double lowKHz, double highKHz, double startKHz, dsp::Mode mode, double stepHz) {
    Band b;
    b.group = group;
    b.name = name;
    b.low = lowKHz * 1e3;
    b.high = highKHz * 1e3;
    b.start = startKHz * 1e3;
    b.mode = mode;
    b.step = stepHz;
    return b;
}

std::vector<Band> makeBands() {
    using dsp::Mode;
    std::vector<Band> v;
    // Broadcast
    v.push_back(band("Broadcast", "FM broadcast", 87500, 108000, 100000, Mode::WFM, 100000));
    v.push_back(band("Broadcast", "LW", 148.5, 283.5, 198, Mode::AM, 9000));
    v.push_back(band("Broadcast", "MW", 526.5, 1705, 1000, Mode::AM, 9000));
    v.push_back(band("Broadcast", "120 m", 2300, 2495, 2400, Mode::AM, 5000));
    v.push_back(band("Broadcast", "90 m", 3200, 3400, 3300, Mode::AM, 5000));
    v.push_back(band("Broadcast", "75 m", 3900, 4000, 3950, Mode::AM, 5000));
    v.push_back(band("Broadcast", "60 m", 4750, 5060, 4900, Mode::AM, 5000));
    v.push_back(band("Broadcast", "49 m", 5900, 6200, 6000, Mode::AM, 5000));
    v.push_back(band("Broadcast", "41 m", 7200, 7450, 7300, Mode::AM, 5000));
    v.push_back(band("Broadcast", "31 m", 9400, 9900, 9600, Mode::AM, 5000));
    v.push_back(band("Broadcast", "25 m", 11600, 12100, 11800, Mode::AM, 5000));
    v.push_back(band("Broadcast", "22 m", 13570, 13870, 13700, Mode::AM, 5000));
    v.push_back(band("Broadcast", "19 m", 15100, 15800, 15400, Mode::AM, 5000));
    v.push_back(band("Broadcast", "16 m", 17480, 17900, 17700, Mode::AM, 5000));
    v.push_back(band("Broadcast", "13 m", 21450, 21850, 21600, Mode::AM, 5000));
    v.push_back(band("Broadcast", "11 m", 25670, 26100, 25800, Mode::AM, 5000));
    v.push_back(band("Broadcast", "DAB III", 174000, 240000, 225648, Mode::WFM, 16000));
    // Amateur
    v.push_back(band("Amateur", "160 m", 1810, 2000, 1840, Mode::LSB, 500));
    v.push_back(band("Amateur", "80 m", 3500, 3800, 3700, Mode::LSB, 500));
    v.push_back(band("Amateur", "60 m", 5351.5, 5366.5, 5357, Mode::USB, 500));
    v.push_back(band("Amateur", "40 m", 7000, 7200, 7100, Mode::LSB, 500));
    v.push_back(band("Amateur", "30 m", 10100, 10150, 10136, Mode::CW, 100));
    v.push_back(band("Amateur", "20 m", 14000, 14350, 14200, Mode::USB, 500));
    v.push_back(band("Amateur", "17 m", 18068, 18168, 18130, Mode::USB, 500));
    v.push_back(band("Amateur", "15 m", 21000, 21450, 21300, Mode::USB, 500));
    v.push_back(band("Amateur", "12 m", 24890, 24990, 24950, Mode::USB, 500));
    v.push_back(band("Amateur", "10 m", 28000, 29700, 28500, Mode::USB, 500));
    v.push_back(band("Amateur", "6 m", 50000, 54000, 50150, Mode::USB, 500));
    v.push_back(band("Amateur", "4 m", 70000, 70500, 70200, Mode::USB, 500));
    v.push_back(band("Amateur", "2 m", 144000, 148000, 145500, Mode::NFM, 12500));
    v.push_back(band("Amateur", "70 cm", 430000, 440000, 433500, Mode::NFM, 12500));
    // Air & sea
    v.push_back(band("Air & sea", "Airband", 118000, 137000, 121500, Mode::AM, 8333.333));
    v.push_back(band("Air & sea", "Marine VHF", 156000, 162025, 156800, Mode::NFM, 25000));
    v.push_back(band("Air & sea", "AIS", 161950, 162050, 162000, Mode::NFM, 25000));
    v.push_back(band("Air & sea", "ACARS", 129125, 131850, 131725, Mode::AM, 25000));
    v.push_back(band("Air & sea", "VDL2", 136600, 137000, 136800, Mode::AM, 25000));
    v.push_back(band("Air & sea", "NOAA weather", 162400, 162550, 162550, Mode::NFM, 25000));
    v.push_back(band("Air & sea", "NAVTEX", 490, 518, 518, Mode::USB, 100));
    // Other
    v.push_back(band("Other", "PMR446", 446000, 446200, 446006.25, Mode::NFM, 12500));
    v.push_back(band("Other", "CB", 26965, 27405, 27185, Mode::AM, 10000));
    v.push_back(band("Other", "Time WWV", 2500, 20000, 10000, Mode::AM, 5000));
    v.push_back(band("Other", "The Buzzer", 4625, 4625, 4625, Mode::USB, 100));
    v.push_back(band("Other", "ISM 433", 433050, 434790, 433920, Mode::NFM, 25000));
    return v;
}

}  // namespace

const std::vector<Band>& bands() {
    static const std::vector<Band> table = makeBands();
    return table;
}

const Band* bandAt(double hz) {
    const Band* best = 0;
    const std::vector<Band>& b = bands();
    for (size_t i = 0; i < b.size(); ++i) {
        if (hz < b[i].low || hz > b[i].high) continue;
        if (!best || b[i].high - b[i].low < best->high - best->low) best = &b[i];
    }
    return best;
}

const std::vector<DabChannel>& dabChannels() {
    static const DabChannel table[] = {
        {"5A", 174928e3},  {"5B", 176640e3},  {"5C", 178352e3},  {"5D", 180064e3},  {"6A", 181936e3},  {"6B", 183648e3},
        {"6C", 185360e3},  {"6D", 187072e3},  {"7A", 188928e3},  {"7B", 190640e3},  {"7C", 192352e3},  {"7D", 194064e3},
        {"8A", 195936e3},  {"8B", 197648e3},  {"8C", 199360e3},  {"8D", 201072e3},  {"9A", 202928e3},  {"9B", 204640e3},
        {"9C", 206352e3},  {"9D", 208064e3},  {"10A", 209936e3}, {"10N", 210096e3}, {"10B", 211648e3}, {"10C", 213360e3},
        {"10D", 215072e3}, {"11A", 216928e3}, {"11N", 217088e3}, {"11B", 218640e3}, {"11C", 220352e3}, {"11D", 222064e3},
        {"12A", 223936e3}, {"12N", 224096e3}, {"12B", 225648e3}, {"12C", 227360e3}, {"12D", 229072e3}, {"13A", 230784e3},
        {"13B", 232496e3}, {"13C", 234208e3}, {"13D", 235776e3}, {"13E", 237488e3}, {"13F", 239200e3}};
    static const std::vector<DabChannel> v(table, table + sizeof(table) / sizeof(table[0]));
    return v;
}

const DabChannel* dabChannelAt(double hz) {
    const std::vector<DabChannel>& c = dabChannels();
    for (size_t i = 0; i < c.size(); ++i)
        if (std::fabs(c[i].hz - hz) < 1000) return &c[i];
    return 0;
}

const DabChannel& dabNearest(double hz) {
    const std::vector<DabChannel>& c = dabChannels();
    size_t best = 0;
    for (size_t i = 1; i < c.size(); ++i)
        if (std::fabs(c[i].hz - hz) < std::fabs(c[best].hz - hz)) best = i;
    return c[best];
}

std::string formatFrequency(double hz) {
    if (hz < 0) hz = 0;
    unsigned long long v = static_cast<unsigned long long>(hz + 0.5);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%llu.%03llu.%03llu", v / 1000000ull, (v / 1000ull) % 1000ull, v % 1000ull);
    return buf;
}

std::string formatShort(double hz) {
    char buf[32];
    if (hz >= 1e6) std::snprintf(buf, sizeof(buf), "%.4g MHz", hz / 1e6);
    else if (hz >= 1e3) std::snprintf(buf, sizeof(buf), "%.4g kHz", hz / 1e3);
    else std::snprintf(buf, sizeof(buf), "%.0f Hz", hz);
    return buf;
}

bool parseFrequency(const std::string& text, double& hz) {
    std::string t;
    for (size_t i = 0; i < text.size(); ++i)
        if (!std::isspace(static_cast<unsigned char>(text[i]))) t += static_cast<char>(std::tolower(static_cast<unsigned char>(text[i])));
    if (t.empty()) return false;
    // Allow "145.800.000": more than one dot = dots are digit-group marks.
    size_t dots = 0;
    for (size_t i = 0; i < t.size(); ++i) dots += t[i] == '.';
    if (dots > 1) {
        std::string d;
        for (size_t i = 0; i < t.size(); ++i)
            if (t[i] != '.') d += t[i];
        t = d;
    }
    char* end = 0;
    double v = std::strtod(t.c_str(), &end);
    if (end == t.c_str() || !(v > 0)) return false;
    std::string unit(end);
    if (unit.size() > 2 && unit.compare(unit.size() - 2, 2, "hz") == 0) unit.erase(unit.size() - 2);
    if (unit == "g") v *= 1e9;
    else if (unit == "m") v *= 1e6;
    else if (unit == "k") v *= 1e3;
    else if (unit.empty()) v = (dots <= 1 && v < 2000) ? v * 1e6 : v;
    else return false;
    hz = v;
    return true;
}

}  // namespace app
