// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/main.cpp
//  Layer:  Application (entry point)
//
//  What this file does
//  -------------------
//  Parses the command line, lists sticks (--list), builds the source and
//  the sound output, starts the Radio, and runs either the terminal
//  interface (Ui) or the headless loop (--headless N: one status line per
//  second, for tests, scripts and remote checks).
// ============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <thread>

#include <signal.h>

#include <ctime>
#include <graphicx/png.h>

#include "../rtlsdr/RtlSdr.h"
#include "Bands.h"
#include "Options.h"
#include "Radio.h"
#include "DecoderPlan.h"
#include "Store.h"
#include "Ui.h"

namespace {

volatile sig_atomic_t gStop = 0;
void onSignal(int) { gStop = 1; }

int listSticks() {
    std::vector<usb::DeviceInfo> sticks = rtlsdr::RtlSdr::find();
    if (sticks.empty()) {
        std::printf("No RTL-SDR stick found.\n");
        return 1;
    }
    for (size_t i = 0; i < sticks.size(); ++i) {
        const usb::DeviceInfo& d = sticks[i];
        std::printf("[%zu] %04x:%04x  %s / %s  serial %s\n", i, d.vendorId, d.productId, d.manufacturer.c_str(),
                    d.product.c_str(), d.serial.c_str());
    }
    return 0;
}

// Headless pictures (FAX, SSTV): kept row by row and saved as PNG when the
// next picture starts and at the end (unattended fax archiving).
struct PictureSaver {
    unsigned id;
    int width;
    std::vector<std::vector<unsigned char> > rows;
    PictureSaver() : id(~0u), width(0) {}
    void save(const std::string& kind) {
        if (rows.empty() || width <= 0) return;
        gfx::Image img(width, static_cast<int>(rows.size()));
        for (size_t y = 0; y < rows.size(); ++y)
            for (int x = 0; x < width; ++x)
                img.set(x, static_cast<int>(y), gfx::rgb(rows[y][x * 3], rows[y][x * 3 + 1], rows[y][x * 3 + 2]));
        std::time_t now = std::time(0);
        char name[64];
        std::strftime(name, sizeof(name), "%Y%m%d-%H%M%S", std::localtime(&now));
        std::string path = "offgrid-" + kind + "-" + name + "-" + std::to_string(id) + ".png", error;
        if (gfx::savePng(path, img, error)) std::printf("picture saved: %s (%dx%d)\n", path.c_str(), width, static_cast<int>(rows.size()));
        else std::printf("picture not saved: %s\n", error.c_str());
        rows.clear();
    }
    void add(const std::vector<dec::PictureLine>& lines, const std::string& kind) {
        for (size_t i = 0; i < lines.size(); ++i) {
            const dec::PictureLine& l = lines[i];
            if (l.picture != id) {
                save(kind);
                id = l.picture;
                width = static_cast<int>(l.rgb.size() / 3);
            }
            if (l.row < 0 || width <= 0) continue;
            if (static_cast<size_t>(l.row) >= rows.size())
                rows.resize(static_cast<size_t>(l.row) + 1, std::vector<unsigned char>(static_cast<size_t>(width) * 3, 0));
            rows[static_cast<size_t>(l.row)] = l.rgb;
            rows[static_cast<size_t>(l.row)].resize(static_cast<size_t>(width) * 3, 0);
        }
    }
};

int headless(app::Radio& radio, const app::Options& o) {
    PictureSaver pictures;
    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);
    app::RadioSettings s = radio.settings();
    std::printf("source: %s\naudio: %s\ntuned: %s Hz %s\n", radio.sourceName().c_str(), radio.audioName().c_str(),
                app::formatFrequency(s.frequency).c_str(), dsp::modeName(s.mode));
    std::fflush(stdout);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int second = 0;
    while (!gStop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
        if (elapsed >= second + 1) {
            ++second;
            app::Snapshot snap;
            radio.snapshot(snap);
            std::printf("t=%2ds level %6.1f dBFS noise %6.1f  rate %.3f MS/s  dsp %3.0f%%  audio %3.0f ms gaps %u  drops %u%s%s",
                        second, snap.channelDbfs, snap.noiseDbfs, snap.sampleRateMeasured / 1e6, snap.dspLoad * 100,
                        snap.audioQueued * 1000, snap.audioUnderruns, snap.samplesDropped, snap.stereoPilot ? "  STEREO" : "",
                        snap.overload ? "  OVERLOAD" : "");
            if (!std::isnan(snap.carrierHz))
                std::printf("  carrier %+.1f Hz (%+.2f ppm)", snap.carrierHz, -snap.carrierHz / radio.settings().frequency * 1e6);
            if (snap.decoderOn)
                std::printf("  %s %s%s", snap.decoder.mode.empty() ? dec::kindName(snap.decoderKind) : snap.decoder.mode.c_str(),
                            snap.decoder.locked ? "locked" : "searching", snap.decoder.info.empty() ? "" : (" " + snap.decoder.info).c_str());
            if (snap.rds.groups > 0)
                std::printf("  RDS groups %u PI %04X PS \"%s\" PTY %s RT \"%s\"", snap.rds.groups, snap.rds.pi, snap.rds.ps.c_str(),
                            snap.rds.ptyName.c_str(), snap.rds.radioText.c_str());
            // The strongest peaks of the spectrum (15 dB over the median).
            if (!snap.spectrum.empty()) {
                std::vector<float> sorted(snap.spectrum);
                std::nth_element(sorted.begin(), sorted.begin() + static_cast<long>(sorted.size() / 2), sorted.end());
                float median = sorted[sorted.size() / 2];
                std::vector<std::pair<float, int> > peaks;
                int n = static_cast<int>(snap.spectrum.size());
                for (int i = 2; i < n - 2; ++i) {
                    float v = snap.spectrum[static_cast<size_t>(i)];
                    if (std::abs(i - n / 2) < 3) continue;  // the DC spike
                    if (v > median + 15 && v >= snap.spectrum[static_cast<size_t>(i - 1)] && v >= snap.spectrum[static_cast<size_t>(i + 1)] &&
                        v >= snap.spectrum[static_cast<size_t>(i - 2)] && v >= snap.spectrum[static_cast<size_t>(i + 2)])
                        peaks.push_back(std::make_pair(v, i));
                }
                std::sort(peaks.rbegin(), peaks.rend());
                if (!peaks.empty()) std::printf("  peaks:");
                for (size_t k = 0; k < peaks.size() && k < 5; ++k) {
                    double f = snap.centerHz + (peaks[k].second - n / 2) * snap.sampleRate / n;
                    std::printf(" %.3f MHz (%+.0f dB)", f / 1e6, peaks[k].first - median);
                }
            }
            std::printf("\n");
            std::string text;
            radio.takeDecoderText(text);
            if (!text.empty()) std::printf("decoded: %s\n", text.c_str());
            std::vector<dec::PictureLine> lines;
            radio.takeDecoderLines(lines);
            if (!lines.empty()) pictures.add(lines, dec::kindName(snap.decoderKind));
            std::fflush(stdout);
        }
        if (elapsed >= o.headlessSeconds) break;
    }
    app::Snapshot last;
    radio.snapshot(last);
    pictures.save(dec::kindName(last.decoderKind));  // the picture in progress
    return 0;
}

}  // namespace

// Saved settings become the defaults; the command line still wins (it is
// parsed after).
void applyStore(const app::Store& st, app::Options& o) {
    o.radio.frequency = st.getNumber("frequency", o.radio.frequency);
    dsp::Mode m;
    if (dsp::parseMode(st.get("mode"), m)) o.radio.mode = m;
    o.radio.bandwidth = st.getNumber("bandwidth", o.radio.bandwidth);
    o.radio.volume = st.getNumber("volume", o.radio.volume);
    o.radio.squelchDb = st.getNumber("squelch", o.radio.squelchDb);
    o.radio.gainDb = st.getNumber("gain", o.radio.gainDb);
    o.radio.ppm = st.getNumber("ppm", o.radio.ppm);
    o.radio.stereo = st.get("stereo", o.radio.stereo ? "1" : "0") == "1";
    o.radio.deemphasisUs = st.getNumber("deemphasis", o.radio.deemphasisUs);
    if (!st.get("theme").empty()) o.theme = st.get("theme");
    o.colormap = st.get("colormap", o.colormap);
}

int main(int argc, char** argv) {
    app::Options o;
    app::Store store;
    std::string error;
    bool fresh = false;
    for (int i = 1; i < argc; ++i) fresh = fresh || std::string(argv[i]) == "--fresh";
    if (!fresh) {
        store.load(error);
        applyStore(store, o);
    }
    error = app::parse(argc, argv, o);
    if (!error.empty()) {
        std::fprintf(stderr, "offgrid: %s\n", error.c_str());
        return 2;
    }
    if (o.help) {
        std::fputs(app::usage().c_str(), stdout);
        return 0;
    }
    if (o.version) {
        std::printf("%s\n", app::versionString());
        return 0;
    }
    if (o.list) return listSticks();

    std::unique_ptr<app::Source> source;
    switch (o.source) {
        case app::SourceKind::Demo: source = app::makeDemoSource(o.sampleRate, o.radio.frequency); break;
        case app::SourceKind::Tcp: source = app::makeTcpSource(o.tcp); break;
        case app::SourceKind::File: source = app::makeFileSource(o.file, o.sampleRate, o.fileCenter); break;
        default: source = app::makeRtlSource(o.device); break;
    }
    source->setSampleRate(o.sampleRate);
    std::unique_ptr<audio::AudioOutput> out = o.audio ? audio::AudioOutput::create(o.audioDevice) : audio::AudioOutput::createNull();

    app::Radio radio(std::move(source), std::move(out));
    if (!radio.start(o.radio, error)) {
        std::fprintf(stderr, "offgrid: %s\n", error.c_str());
        return 1;
    }
    if (!o.wav.empty() && !radio.startRecording(o.wav, error)) std::fprintf(stderr, "offgrid: %s\n", error.c_str());
    dec::Kind kind;
    if (o.headlessSeconds > 0 && !o.decoder.empty() && dec::parseKind(o.decoder, kind)) {
        // As in the interface (DecoderPlan.h).
        app::DecoderPlan plan = app::planDecoder(kind, radio.settings().frequency);
        radio.setMode(plan.mode);
        if (plan.bandwidth > 0) radio.setBandwidth(plan.bandwidth);
        radio.setDecoderShift(plan.shift);
        radio.setDecoder(true, kind, plan.setup);
    }

    int code = 0;
    if (o.headlessSeconds > 0) {
        code = headless(radio, o);
    } else {
        app::Ui ui(radio, o, store);
        code = ui.run();  // saves the settings on the way out
    }
    radio.stop();
    return code;
}
