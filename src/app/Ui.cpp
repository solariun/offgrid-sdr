// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/Ui.cpp
//  Layer:  Application (the terminal user interface)
//
//  What this file does
//  -------------------
//  Implementation of app::Ui (see Ui.h): the widgets, their layout, the
//  key and mouse bindings, and the per-frame update from the Radio's
//  Snapshot (spectrum window and zoom, waterfall rows, meter, lights, RDS,
//  decoder panel, status line); plus the phase 2 features: memories,
//  frequency labels, logbook, scanning and frequency calibration.
//
//  The spectrum window
//  -------------------
//  The FFT covers the whole span (sample rate) around the stick's centre.
//  With zoom z the plot and waterfall show span / z, centred near the dial;
//  the window moves in quarter-window jumps (not with every tuning step), so
//  the waterfall history stays aligned until a jump clears it.
//
//  Scanning
//  --------
//  n scans the current band (its range, in its step), N the memories.  Each
//  channel is listened to for 0.2 s; a level above the squelch (or, with the
//  squelch off, 12 dB over the noise floor) stops the scan there, and it
//  carries on 2 s after the signal goes.  Any tuning key stops it.
//
//  Calibration
//  -----------
//  K, on a station whose frequency is known exactly (FM broadcast in WFM,
//  a time signal or AM broadcaster in SAM): the receiver measures where the
//  carrier really is (Receiver::carrierOffsetHz); the stick's error is
//  -offset / frequency, which is added to the ppm correction.
//
//  Contributing
//  ------------
//  New controls: create the widget in Impl, add() it in build(), place it in
//  layout(), and keep a key for it in bindKeys() and the help text.
// ============================================================================
#include "Ui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>

#include <graphicx/canvas.h>
#include <graphicx/image_io.h>
#include <graphicx/png.h>
#include <graphicx/terminal.h>
#include <graphicx/widgets.h>

#include <deque>
#include <fstream>
#include <sstream>
#include <sys/time.h>

#include "../decoders/Callsign.h"
#include "../decoders/Clock.h"
#include "../decoders/Ft8.h"
#include "../dsp/Propagation.h"
#include "../dsp/SpectrumTools.h"
#include "Bands.h"
#include "DecoderPlan.h"

namespace app {

using namespace gfx;
using namespace gfx::ui;
using terminal::Event;

namespace {

const double kSteps[] = {1, 10, 50, 100, 500, 1000, 2500, 5000, 8333.333333, 9000, 10000, 12500, 25000, 50000, 100000, 200000, 1000000};
const int kStepCount = sizeof(kSteps) / sizeof(kSteps[0]);
const dsp::Mode kModes[] = {dsp::Mode::WFM, dsp::Mode::NFM, dsp::Mode::AM, dsp::Mode::SAM, dsp::Mode::USB, dsp::Mode::LSB, dsp::Mode::CW};
const int kModeCount = 7;

std::string fmt(const char* f, double v) {
    char b[64];
    std::snprintf(b, sizeof(b), f, v);
    return b;
}

std::string stepName(double s) {
    if (std::fabs(s - 8333.333333) < 1) return "8.33 kHz";
    return formatShort(s);
}

int modeIndex(dsp::Mode m) {
    for (int i = 0; i < kModeCount; ++i)
        if (kModes[i] == m) return i;
    return 0;
}

// Bandwidth slider range per mode: min, max, step (Hz).
void bandwidthRange(dsp::Mode m, double& lo, double& hi, double& step) {
    switch (m) {
        case dsp::Mode::WFM: lo = 50000; hi = 250000; step = 10000; break;
        case dsp::Mode::NFM: lo = 4000; hi = 30000; step = 500; break;
        case dsp::Mode::AM:
        case dsp::Mode::SAM: lo = 2000; hi = 20000; step = 500; break;
        case dsp::Mode::USB:
        case dsp::Mode::LSB: lo = 1000; hi = 4500; step = 100; break;
        default: lo = 100; hi = 2000; step = 50; break;
    }
}

// A one-line text prompt in a framed popup (frequency, memory name, label,
// log notes).  The caller sets title / label / onSubmit, then opens it.
class Prompt : public Widget {
public:
    Prompt() : okX(0), cancelX(0) {
        input.maxLength = 60;
    }
    std::string title, hint, error;
    TextInput input;
    int okX, cancelX;  // where the buttons were drawn (local columns, row 4)
    bool focusable() const { return true; }
    void draw(Painter& p, const Theme& t, bool) {
        p.panel(p.bounds(), t.focus, t.panel, Border::Double, " " + title + " ");
        Painter in = p.sub(rect(2, 2, p.width() - 4, 1));
        input.draw(in, t, true);
        if (!error.empty()) p.text(rect(2, 3, p.width() - 4, 1), error, t.bad, Align::Left);
        else p.text(rect(2, 3, p.width() - 4, 1), hint.empty() ? std::string("Enter accepts, Esc cancels") : hint, t.dim, Align::Left);
        // Clickable buttons (mouse users).
        cancelX = p.width() - 2 - 10;
        okX = cancelX - 1 - 6;
        p.text(okX, 4, "[ OK ]", t.accentText, t.accent, Bold);
        p.text(cancelX, 4, "[ Cancel ]", t.text, lerp(t.panel, t.text, 0.12));
    }
    bool handle(const Event& e) {
        if (e.type == terminal::EventType::Mouse) {
            if (e.button == terminal::MouseButton::Left && e.action == terminal::MouseAction::Release && e.y == 4) {
                if (e.x >= okX && e.x < okX + 6 && input.onSubmit) input.onSubmit(input.text);
                else if (e.x >= cancelX && e.x < cancelX + 10 && input.onCancel) input.onCancel();
            } else if (e.y == 2) {
                Event local = e;
                local.x -= 2;
                local.y = 0;
                input.handle(local);  // place the cursor
            }
            return true;
        }
        return input.handle(e);
    }
};

// The decoder's output: scrolling text, or the Hellschreiber picture.
class DecoderView : public Widget {
public:
    DecoderView() : picture(false), lineMode(false), pictureId(~0u), pictureWidth(0), serviceMode(false) {}
    std::string title;
    std::string text;                              // everything decoded (last 16 kB)
    std::vector<std::vector<float> > columns;      // Hell columns (last 2000)
    bool picture;                                  // Hell strip
    bool lineMode;                                 // FAX / SSTV picture
    // FAX / SSTV: the picture being received, row by row (RGB bytes).
    unsigned pictureId;
    int pictureWidth;
    std::vector<std::vector<unsigned char> > rows;
    std::function<void()> onTitleClick;
    // Multi-service decoders (DAB, DRM): the station list on the left, the
    // text in the middle, the latest slideshow picture on the right.
    bool serviceMode;
    std::vector<dec::Service> services;
    std::function<void(unsigned)> onSelectService;
    Image slide;
    // Called with the finished picture before a new one starts.
    std::function<void(const Image&)> onPictureDone;

    Image pictureImage() const {
        Image img(pictureWidth, static_cast<int>(rows.size()), rgb(0, 0, 0));
        for (size_t y = 0; y < rows.size(); ++y)
            for (int x = 0; x < pictureWidth && static_cast<size_t>(x) * 3 + 2 < rows[y].size(); ++x)
                img.set(x, static_cast<int>(y), rgb(rows[y][x * 3], rows[y][x * 3 + 1], rows[y][x * 3 + 2]));
        return img;
    }
    void addLines(const std::vector<dec::PictureLine>& lines) {
        for (size_t i = 0; i < lines.size(); ++i) {
            const dec::PictureLine& l = lines[i];
            if (l.picture != pictureId) {
                if (!rows.empty() && onPictureDone) onPictureDone(pictureImage());
                rows.clear();
                pictureId = l.picture;
                pictureWidth = static_cast<int>(l.rgb.size() / 3);
            }
            if (l.row < 0 || pictureWidth <= 0) continue;
            if (static_cast<size_t>(l.row) >= rows.size()) rows.resize(static_cast<size_t>(l.row) + 1, std::vector<unsigned char>(static_cast<size_t>(pictureWidth) * 3, 0));
            rows[static_cast<size_t>(l.row)] = l.rgb;
            rows[static_cast<size_t>(l.row)].resize(static_cast<size_t>(pictureWidth) * 3, 0);
        }
    }
    int listWidth() const { return std::min(34, std::max(18, bounds.width / 4)); }
    bool handle(const Event& e) {
        if (e.type == terminal::EventType::Mouse && e.y == 0 && e.button == terminal::MouseButton::Left &&
            e.action == terminal::MouseAction::Press && onTitleClick) {
            onTitleClick();
            return true;
        }
        // A click on a station in the list chooses it.
        if (serviceMode && e.type == terminal::EventType::Mouse && e.button == terminal::MouseButton::Left &&
            e.action == terminal::MouseAction::Press && e.x >= 1 && e.x < 1 + listWidth() && e.y >= 1) {
            size_t i = static_cast<size_t>(e.y - 1);
            if (i < services.size() && onSelectService) onSelectService(services[i].id);
            return true;
        }
        return false;
    }
    void addText(const std::string& s) {
        text += s;
        if (text.size() > 16384) text.erase(0, text.size() - 16384);
    }
    void addColumns(const std::vector<std::vector<float> >& c) {
        columns.insert(columns.end(), c.begin(), c.end());
        if (columns.size() > 2000) columns.erase(columns.begin(), columns.end() - 2000);
    }
    void clear() {
        text.clear();
        columns.clear();
        rows.clear();
        pictureId = ~0u;
        pictureWidth = 0;
    }
    void draw(Painter& p, const Theme& t, bool focused) {
        p.panel(p.bounds(), focused ? t.focus : t.border, t.panel, Border::Rounded, std::string());
        if (p.width() > 4) p.text(rect(2, 0, p.width() - 4, 1), " " + title + " ", t.accent, Align::Left, Bold);
        Rect in = inset(p.bounds(), 1);
        in.x += 1;
        in.width -= 2;
        if (in.empty()) return;
        if (serviceMode) {
            // Left: the stations; right: the slide; middle: the text.
            int lw = std::min(listWidth(), in.width / 2);
            for (size_t i = 0; i < services.size() && static_cast<int>(i) < in.height; ++i) {
                const dec::Service& sv = services[i];
                Rgb fg = sv.selected ? t.accentText : (sv.audio ? t.text : t.dim);
                Rgb bg = sv.selected ? t.accent : t.panel;
                p.fill(rect(in.x, in.y + static_cast<int>(i), lw, 1), bg);
                p.text(rect(in.x, in.y + static_cast<int>(i), lw, 1), (sv.selected ? "\xE2\x96\xB6 " : "  ") + sv.label, fg, Align::Left,
                       sv.selected ? Bold : Plain);
            }
            if (services.empty()) p.text(in.x, in.y, "looking for stations...", t.dim);
            int sx = in.x + lw + 1, sw = in.right() - sx;
            if (!slide.empty() && sw > 20) {
                int iw = std::min(sw / 2, in.height * 2 * slide.width() / std::max(1, slide.height()));
                int ih = std::max(1, std::min(in.height, iw * slide.height() / std::max(1, slide.width()) / 2));
                Painter img = p.sub(rect(in.right() - iw, in.y, iw, ih));
                img.image(img.bounds(), slide, PixelMode::HalfBlocks, Filter::Box);
                sw -= iw + 1;
            }
            Rect textArea = rect(sx, in.y, std::max(1, sw), in.height);
            // Selected service's detail, then the newest text lines.
            for (size_t i = 0; i < services.size(); ++i)
                if (services[i].selected) p.text(rect(textArea.x, textArea.y, textArea.width, 1), services[i].detail, t.dim, Align::Left);
            std::vector<std::string> lines;
            size_t start = 0;
            while (start < text.size()) {
                size_t nl = text.find('\n', start);
                std::vector<std::string> w = wrap(text.substr(start, nl == std::string::npos ? std::string::npos : nl - start), textArea.width);
                lines.insert(lines.end(), w.begin(), w.end());
                if (nl == std::string::npos) break;
                start = nl + 1;
            }
            int rows = textArea.height - 1;
            int first = std::max(0, static_cast<int>(lines.size()) - rows);
            for (int i = first; i < static_cast<int>(lines.size()); ++i)
                p.text(textArea.x, textArea.y + 1 + i - first, lines[static_cast<size_t>(i)], t.text);
            return;
        }
        if (lineMode) {
            // FAX / SSTV: the newest part of the picture that fits, at the
            // picture's own aspect ratio (half blocks: 1x2 pixels a cell).
            if (rows.empty() || pictureWidth <= 0) {
                p.text(in.x, in.y, "waiting for a picture...", t.dim);
                return;
            }
            int cols = in.width, pixRows = in.height * 2;
            double scale = static_cast<double>(cols) / pictureWidth;  // cells per pixel
            int show = std::min(static_cast<int>(rows.size()), std::max(1, static_cast<int>(pixRows / scale)));
            int w = cols;
            int h = std::max(1, std::min(in.height, static_cast<int>(show * scale / 2 + 0.5)));
            if (h * 2 > pixRows) h = in.height;
            Image img(pictureWidth, show, rgb(0, 0, 0));
            for (int y = 0; y < show; ++y) {
                const std::vector<unsigned char>& r = rows[rows.size() - show + y];
                for (int x = 0; x < pictureWidth; ++x) img.set(x, y, rgb(r[x * 3], r[x * 3 + 1], r[x * 3 + 2]));
            }
            // Narrow pictures (SSTV) keep their shape, centred.
            if (show * scale / 2 < in.height) {
                double fit = std::min(static_cast<double>(in.width) / pictureWidth, in.height * 2.0 / show);
                w = std::max(1, static_cast<int>(pictureWidth * fit));
                h = std::max(1, static_cast<int>(show * fit / 2));
            }
            Painter area = p.sub(rect(in.x + (in.width - w) / 2, in.y, w, h));
            area.image(area.bounds(), img, PixelMode::HalfBlocks, Filter::Box);
            return;
        }
        if (picture) {
            // Hell: 14 dots a column, newest at the right; one pixel per dot,
            // half blocks (2 dots a cell row), stretched to the panel height.
            int w = std::min(static_cast<int>(columns.size()), in.width);
            if (w <= 0) return;
            Image img(w, 14, t.panel);
            for (int x = 0; x < w; ++x) {
                const std::vector<float>& col = columns[columns.size() - w + x];
                for (int y = 0; y < 14 && y < static_cast<int>(col.size()); ++y)
                    img.set(x, y, lerp(t.panel, t.text, std::max(0.0f, std::min(1.0f, col[static_cast<size_t>(y)]))));
            }
            int rows = std::min(in.height, 14);
            Painter area = p.sub(rect(in.right() - w, in.y, w, rows));
            area.image(area.bounds(), img, PixelMode::HalfBlocks, Filter::Box);
            return;
        }
        // Text: wrap every line, show the last ones that fit.
        std::vector<std::string> lines;
        size_t start = 0;
        while (start <= text.size()) {
            size_t nl = text.find('\n', start);
            std::string line = text.substr(start, nl == std::string::npos ? std::string::npos : nl - start);
            std::vector<std::string> w = wrap(line, in.width);
            if (w.empty()) w.push_back(std::string());
            lines.insert(lines.end(), w.begin(), w.end());
            if (nl == std::string::npos) break;
            start = nl + 1;
        }
        int first = std::max(0, static_cast<int>(lines.size()) - in.height);
        for (int i = first; i < static_cast<int>(lines.size()); ++i)
            p.text(in.x, in.y + i - first, lines[static_cast<size_t>(i)], t.text);
    }
};

}  // namespace

// ==================================================================== Impl ==

struct Ui::Impl {
    Impl(Radio& r, const Options& o, Store& s);

    Radio& radio;
    Options options;
    Store& store;
    App app;

    // header
    BigText freqText;
    gfx::ui::Label unitLabel, infoLabel, clockLabel;
    RadioGroup modes;
    Led ledStereo, ledRds, ledSquelch, ledOverload, ledRecord, ledMute, ledScan, ledDecoder;
    Meter meter;
    Sparkline levelHistory;
    // main
    Plot spectrum;
    Waterfall waterfall;
    DecoderView decoderView;
    gfx::ui::Label rdsLabel;
    // side
    Panel listPanel, controlsPanel;
    Tabs listTabs;
    ListBox list;
    Slider volume, squelch, gain, bandwidth, zoom, wfFloor, wfRange;
    Toggle autoGain, stereo, biasTee;
    StatusBar status;
    // popups
    Prompt prompt;
    std::function<void(const std::string&)> promptAction;
    Dialog help, message, confirm;
    std::function<void(int)> confirmAction;
    Menu decoderMenu, filterMenu, monitorMenu, viewMenu;

    // state
    int stepIndex;
    std::vector<double> gainSteps;
    std::vector<std::string> themes;
    size_t themeIndex;
    std::vector<Colormap> maps;
    int mapIndex;              // -1 = the theme's
    double winLow, winHigh;    // Hz shown by the spectrum / waterfall
    unsigned lastSerial;
    bool autoFloor;
    int framesSinceStart;
    Snapshot snap;
    std::vector<float> wfHold; // peak of the frames since the last waterfall row
    unsigned wfPhase;
    std::vector<LogEntry> logCache;
    std::unique_ptr<dec::Decoder> probe;  // asked for the decoder's audio centre (tuning marker)
    std::deque<std::vector<float> > wfRows;   // the waterfall's rows (dB), newest first: Snapshot
    // toolbar (mouse access to everything the keys do)
    std::vector<std::unique_ptr<Button> > toolbar;
    // scan
    enum ScanKind { ScanOff, ScanBand, ScanMemories } scanKind;
    double scanLow, scanHigh, scanStep;
    size_t scanIndex;
    int scanDwell;             // frames left on this channel before judging
    int scanHold;              // frames of silence left before moving on
    bool scanStopped;          // a signal holds the scan here
    std::string notice;        // last action's result, shown in the status bar for a while
    int noticeFrames;
    bool showMessageLater;     // first run: the welcome message once the screen exists
    dec::ClockCorrection clock;  // the corrected time (time signals), kept in clock.txt

    void build();
    void layout(const Rect& screen);
    void bindKeys();
    void update();
    void tune(double hz, bool snap = true);
    void setMode(dsp::Mode m);
    void syncControls();
    void showMessage(const std::string& title, const std::string& text);
    void ask(const std::string& title, const std::string& label, const std::string& initial,
             std::function<void(const std::string&)> action, const std::string& hint = std::string());
    void toggleRecording();
    double step() const { return kSteps[stepIndex]; }
    void passband(double& lo, double& hi) const;
    void fillList();
    void startDecoder(dec::Kind kind);
    void stopDecoder();
    void startScan(ScanKind kind);
    void stopScan();
    void scanStep_();
    void calibrate();
    void saveState();
    // Actions shared by keys, toolbar and mouse.
    void askFrequency();
    void openDecoderMenu();
    void saveMemory();
    void labelHere();
    void logHere();
    void showHelp();
    void savePicture(const Image& img, const std::string& kind);
    void setClock();
    // Band survey: FT8 on each band from 80 m to 10 m in turn, two slots
    // each, so the Heard list shows which bands are open.
    std::vector<double> surveyDials;
    size_t surveyIndex;
    double surveyNext;   // corrected time of the next move (0 = off)
    void toggleSurvey();
    void surveyStep();
    void snapshotPicture();
    // Clean-up filters, AUTO, the Signal Monitor, peak hold and alerts.
    void openMenu(Menu& m, int x);
    void fillFilterMenu();
    void fillViewMenu();
    void startAuto();
    void autoStep();
    bool autoWaiting;          // AUTO is listening
    double autoDabUntil;       // AUTO picked DAB: it must lock by then (else NFM), 0 = no
    void monitorHere();
    void saveMonitorCsv();
    double monitorStart;       // Unix time of the monitor's first signal
    dsp::SpectrumExtras extras;
    bool peakHold, averaging, alerts;
    void setup();
    static double pcNow() {
        timeval tv;
        gettimeofday(&tv, 0);
        return tv.tv_sec + tv.tv_usec * 1e-6;
    }
    void say(const std::string& text) {
        notice = text;
        noticeFrames = 150;
    }
    Rect screen() { return rect(0, 0, app.surface().width(), app.surface().height()); }
};

Ui::Impl::Impl(Radio& r, const Options& o, Store& s)
    : radio(r),
      options(o),
      store(s),
      app(theme(o.theme)),
      ledStereo("ST"),
      ledRds("RDS"),
      ledSquelch("SQL"),
      ledOverload("OVL"),
      ledRecord("REC"),
      ledMute("MUTE"),
      ledScan("SCAN"),
      ledDecoder("DEC"),
      listPanel(""),
      controlsPanel("Controls"),
      volume("Volume", 0, 200, 80),
      squelch("Squelch", -120, 0, -120),
      gain("RF gain", 0, 1, 0),
      bandwidth("Filter", 1000, 250000, 150000),
      zoom("Zoom", 0, 5, 0),
      wfFloor("WF floor", -140, -20, -90),
      wfRange("WF range", 10, 100, 50),
      autoGain("Auto gain", true),
      stereo("Stereo", true),
      biasTee("Bias-T", false),
      stepIndex(14),
      themeIndex(0),
      mapIndex(-1),
      winLow(0),
      winHigh(1),
      lastSerial(0),
      autoFloor(true),
      framesSinceStart(0),
      wfPhase(0),
      scanKind(ScanOff),
      scanLow(0),
      scanHigh(0),
      scanStep(0),
      scanIndex(0),
      scanDwell(0),
      scanHold(0),
      scanStopped(false),
      noticeFrames(0),
      showMessageLater(false),
      surveyIndex(0),
      surveyNext(0),
      autoWaiting(false),
      autoDabUntil(0),
      monitorStart(0),
      peakHold(false),
      averaging(false),
      alerts(false) {
    themes = themeNames();
    for (size_t i = 0; i < themes.size(); ++i)
        if (themes[i] == o.theme) themeIndex = i;
    maps = allColormaps();
    Colormap m;
    if (!o.colormap.empty() && parseColormap(o.colormap, m))
        for (size_t i = 0; i < maps.size(); ++i)
            if (maps[i] == m) mapIndex = static_cast<int>(i);
    app.setColorMode(o.trueColor ? ColorMode::TrueColor : ColorMode::Palette256);
    app.autoFocus = false;
    gainSteps = radio.gains();
    // The step: the saved one, else one that suits the band we start in.
    const Band* b = bandAt(radio.settings().frequency);
    if (b)
        for (int i = 0; i < kStepCount; ++i)
            if (std::fabs(kSteps[i] - b->step) < 1) stepIndex = i;
    if (store.has("step")) stepIndex = std::max(0, std::min(kStepCount - 1, static_cast<int>(store.getNumber("step", stepIndex))));
    if (store.has("zoom")) zoom.value = store.getNumber("zoom", 0);
    if (store.has("wf_range")) wfRange.value = store.getNumber("wf_range", 50);
}

void Ui::Impl::passband(double& lo, double& hi) const {
    RadioSettings s = radio.settings();
    double bw = s.bandwidth > 0 ? s.bandwidth : dsp::defaultBandwidth(s.mode);
    double f = s.frequency - s.decoderShift;  // where the receiver really is
    if (s.mode == dsp::Mode::USB) {
        lo = f;
        hi = f + bw;
    } else if (s.mode == dsp::Mode::LSB) {
        lo = f - bw;
        hi = f;
    } else {
        lo = f - bw / 2;
        hi = f + bw / 2;
    }
}

void Ui::Impl::tune(double hz, bool snapToStep) {
    if (snapToStep) {
        double s = step();
        hz = std::floor(hz / s + 0.5) * s;
    }
    radio.tune(hz);
    const Band* b = bandAt(radio.settings().frequency);
    if (b) store.bandMemory[b->name] = radio.settings().frequency;
}

void Ui::Impl::setMode(dsp::Mode m) {
    radio.setMode(m);
    modes.selected = modeIndex(m);
    syncControls();
}

void Ui::Impl::syncControls() {
    RadioSettings s = radio.settings();
    double lo, hi, st;
    bandwidthRange(s.mode, lo, hi, st);
    bandwidth.min = lo;
    bandwidth.max = hi;
    bandwidth.step = st;
    bandwidth.value = s.bandwidth > 0 ? s.bandwidth : dsp::defaultBandwidth(s.mode);
    modes.selected = modeIndex(s.mode);
    volume.value = s.volume * 100;
    squelch.value = s.squelchDb < -120 ? -120 : s.squelchDb;
    autoGain.checked = s.gainDb < 0;
    if (s.gainDb >= 0 && !gainSteps.empty()) {
        size_t best = 0;
        for (size_t i = 0; i < gainSteps.size(); ++i)
            if (std::fabs(gainSteps[i] - s.gainDb) < std::fabs(gainSteps[best] - s.gainDb)) best = i;
        gain.value = static_cast<double>(best);
    }
    stereo.checked = s.stereo;
    stereo.enabled = s.mode == dsp::Mode::WFM;
    biasTee.checked = s.biasTee;
}

void Ui::Impl::showMessage(const std::string& title, const std::string& text) {
    message.title = title;
    message.message = text;
    message.buttons.assign(1, "OK");
    message.place(screen(), 64);
    app.openPopup(message, true);
}

void Ui::Impl::ask(const std::string& title, const std::string& label, const std::string& initial,
                   std::function<void(const std::string&)> action, const std::string& hint) {
    prompt.title = title;
    prompt.hint = hint;
    prompt.error.clear();
    prompt.input.label = label;
    prompt.input.accept = std::function<bool(std::uint32_t)>();
    prompt.input.setText(initial);
    promptAction = action;
    prompt.bounds = centered(screen(), std::min(64, app.surface().width() - 2), 6);
    app.openPopup(prompt, true);
}

void Ui::Impl::toggleRecording() {
    if (radio.recording()) {
        radio.stopRecording();
        return;
    }
    std::time_t now = std::time(0);
    char name[64];
    std::strftime(name, sizeof(name), "offgrid-%Y%m%d-%H%M%S.wav", std::localtime(&now));
    std::string error;
    if (!radio.startRecording(name, error)) showMessage("Recording", error);
}

// ------------------------------------------------------- decoders, scan, cal --

void Ui::Impl::startDecoder(dec::Kind kind) {
    // Each decoder listens in the mode it needs; the dial stays where it is
    // (the spectrum marks where the signal should sit: dial + the decoder's
    // audio centre).
    DecoderPlan plan = planDecoder(kind, radio.settings().frequency);
    if (radio.settings().mode != plan.mode) setMode(plan.mode);
    if (plan.bandwidth > 0) radio.setBandwidth(plan.bandwidth);
    radio.setDecoderShift(plan.shift);  // the dial shows the signal's frequency
    radio.setCenterOnDial(plan.centerOnDial);
    syncControls();
    radio.setDecoder(true, kind, plan.setup);
    if (decoderView.lineMode && !decoderView.rows.empty()) savePicture(decoderView.pictureImage(), dec::kindName(snap.decoderKind));
    decoderView.clear();
    decoderView.picture = kind == dec::Kind::Hell;
    decoderView.lineMode = kind == dec::Kind::Fax || kind == dec::Kind::Sstv;
    decoderView.serviceMode = kind == dec::Kind::Dab || kind == dec::Kind::Drm;
    decoderView.services.clear();
    decoderView.slide = Image();
    if (kind == dec::Kind::Dab && !dabChannelAt(radio.settings().frequency)) {
        // As the browser version: DAB opens on a channel (the nearest one).
        tune(dabNearest(radio.settings().frequency).hz, false);
    }
    decoderView.visible = true;
    store.set("decoder", dec::kindName(kind));
    app.relayout();
}

void Ui::Impl::stopDecoder() {
    radio.setCenterOnDial(false);
    if (decoderView.lineMode && !decoderView.rows.empty()) savePicture(decoderView.pictureImage(), dec::kindName(snap.decoderKind));
    decoderView.lineMode = false;
    decoderView.serviceMode = false;
    radio.setDecoder(false);
    radio.setDecoderShift(0);
    decoderView.visible = false;
    store.set("decoder", "");
    app.relayout();
}

void Ui::Impl::startScan(ScanKind kind) {
    if (kind == ScanMemories && store.memories.empty()) {
        showMessage("Scan", "There are no memories to scan yet: M saves the frequency you are on.");
        return;
    }
    if (kind == ScanBand) {
        const Band* b = bandAt(radio.settings().frequency);
        if (!b || b->high - b->low < step()) {
            showMessage("Scan", "Tune into a band first (b opens the band list): n scans the band you are in, in steps of its channel spacing.");
            return;
        }
        scanLow = b->low;
        scanHigh = b->high;
        scanStep = b->step;
        scanIndex = static_cast<size_t>(std::max(0.0, std::floor((radio.settings().frequency - scanLow) / scanStep + 0.5)));
    } else {
        scanIndex = 0;
    }
    scanKind = kind;
    scanStopped = false;
    scanDwell = 0;
    scanHold = 0;
    scanStep_();
}

void Ui::Impl::stopScan() { scanKind = ScanOff; }

// Moves the scan to the next channel.
void Ui::Impl::scanStep_() {
    if (scanKind == ScanBand) {
        size_t count = static_cast<size_t>((scanHigh - scanLow) / scanStep) + 1;
        scanIndex = (scanIndex + 1) % count;
        radio.tune(scanLow + scanIndex * scanStep);
    } else if (scanKind == ScanMemories) {
        scanIndex = (scanIndex + 1) % store.memories.size();
        const Memory& m = store.memories[scanIndex];
        if (radio.settings().mode != m.mode) setMode(m.mode);
        radio.tune(m.hz);
        if (m.bandwidth > 0) radio.setBandwidth(m.bandwidth);
    }
    scanDwell = 6;  // 0.2 s at 30 frames a second
}

void Ui::Impl::calibrate() {
    RadioSettings s = radio.settings();
    if (s.mode != dsp::Mode::WFM && s.mode != dsp::Mode::NFM && s.mode != dsp::Mode::SAM) {
        showMessage("Calibrate",
                    "Tune exactly to a station whose frequency you know (an FM broadcaster in WFM, a time signal "
                    "or AM broadcaster in SAM), wait a few seconds, then press K.");
        return;
    }
    if (std::isnan(snap.carrierHz) || snap.channelDbfs - snap.noiseDbfs < 10) {
        showMessage("Calibrate", "No steady carrier here (the signal must be at least 10 dB over the noise).");
        return;
    }
    double errorPpm = -snap.carrierHz / s.frequency * 1e6;
    double proposed = s.ppm + errorPpm;
    confirm.title = "Calibrate";
    confirm.message = "The carrier is " + fmt("%+.1f Hz", snap.carrierHz) + " from " + formatShort(s.frequency) +
                      ": the stick is off by " + fmt("%+.2f ppm", errorPpm) + ".\nCorrection now " + fmt("%+.2f ppm", s.ppm) +
                      ", new " + fmt("%+.2f ppm", proposed) + " (kept for next time).";
    confirm.buttons.clear();
    confirm.buttons.push_back("Apply");
    confirm.buttons.push_back("Cancel");
    confirm.selected = 0;
    confirmAction = [this, proposed](int button) {
        if (button != 0) return;
        radio.setPpm(proposed);
        store.setNumber("ppm", proposed);
        radio.resetDecoder();
    };
    confirm.place(screen(), 64);
    app.openPopup(confirm, true);
}

// ----------------------------------------------------------------- actions --

void Ui::Impl::askFrequency() {
    ask("Frequency", "MHz / kHz / Hz:", std::string(), [this](const std::string& text) {
        double hz;
        if (!parseFrequency(text, hz)) {
            prompt.error = "not a frequency: " + text;
            return;
        }
        stopScan();
        const Band* band = bandAt(hz);
        if (band && band->mode != radio.settings().mode && (band->name == "FM broadcast" || band->name == "Airband"))
            setMode(band->mode);
        tune(hz, false);
    }, "e.g. 100.3  7074k  145.8M   Enter tunes, Esc cancels");
    prompt.input.accept = [](std::uint32_t c) {
        return (c >= '0' && c <= '9') || c == '.' || c == 'k' || c == 'K' || c == 'm' || c == 'M' || c == 'g' || c == 'G' ||
               c == 'h' || c == 'H' || c == 'z' || c == 'Z' || c == ' ';
    };
}

void Ui::Impl::openDecoderMenu() {
    decoderMenu.place(2, 4, screen());
    app.openPopup(decoderMenu);
}

void Ui::Impl::saveMemory() {
    RadioSettings s = radio.settings();
    std::string name = snap.rds.ps;
    while (!name.empty() && name[name.size() - 1] == ' ') name.erase(name.size() - 1);
    if (name.empty()) name = formatShort(s.frequency) + " " + dsp::modeName(s.mode);
    ask("Save memory", "Name:", name, [this, s](const std::string& text) {
        Memory m = {text.empty() ? formatShort(s.frequency) : text, s.frequency, s.mode, s.bandwidth};
        store.addMemory(m);
        listTabs.selected = 1;
        fillList();
        say("memory saved: " + m.name);
    });
}

void Ui::Impl::labelHere() {
    double hz = radio.settings().frequency;
    std::string current;
    for (size_t i = 0; i < store.labels.size(); ++i)
        if (std::fabs(store.labels[i].hz - hz) <= 500) current = store.labels[i].text;
    ask("Label " + formatShort(hz), "Text:", current, [this, hz](const std::string& text) {
        if (text.empty()) store.removeLabelNear(hz, 500);
        else store.addLabel(hz, text);
    }, "shown on the spectrum; empty removes it");
}

void Ui::Impl::logHere() {
    RadioSettings s = radio.settings();
    Snapshot at = snap;
    std::string station = at.rds.ps;
    while (!station.empty() && station[station.size() - 1] == ' ') station.erase(station.size() - 1);
    ask("Log " + formatShort(s.frequency), "Notes:", std::string(), [this, s, at, station](const std::string& text) {
        LogEntry e;
        e.utc = utcNow();
        e.hz = s.frequency;
        e.mode = at.decoderOn ? dec::kindName(at.decoderKind) : dsp::modeName(s.mode);
        e.levelDb = at.channelDbfs;
        e.snrDb = at.channelDbfs - at.noiseDbfs;
        e.station = station;
        e.notes = text;
        std::string error;
        if (!Store::appendLog(e, error)) prompt.error = error;
        else {
            if (listTabs.selected == 2) fillList();
            say("logged " + formatShort(s.frequency));
        }
    }, "station " + (station.empty() ? std::string("-") : station) + "; written to " + Store::logPath());
}

// Snapshot: the spectrum and the recent waterfall as one PNG, with a strip
// along the top saying what it shows (as the browser version's Picture).
void Ui::Impl::snapshotPicture() {
    if (spectrum.series.empty() || spectrum.series[0].y.empty()) {
        say("nothing to picture yet");
        return;
    }
    const int w = 1024, strip = 14, specH = 200, wfH = std::min(300, static_cast<int>(wfRows.size()));
    Image img(w, strip + specH + wfH, theme(themes[themeIndex]).background);
    Canvas c(img);
    const Theme t = app.theme();
    RadioSettings s = radio.settings();
    std::time_t now = static_cast<std::time_t>(clock.now(pcNow()));
    char when[40];
    std::strftime(when, sizeof(when), "%Y-%m-%d %H:%M UTC", std::gmtime(&now));
    std::string head = formatFrequency(s.frequency) + " " + dsp::modeName(s.mode) +
                       (snap.decoderOn ? std::string(" ") + dec::kindName(snap.decoderKind) : std::string()) +
                       fmt("  %.1f dBFS  ", snap.channelDbfs) + when;
    c.fillRect(rect(0, 0, w, strip), t.panel);
    c.text(4, 4, head, t.accent);
    const std::vector<double>& y = spectrum.series[0].y;
    const double lo = spectrum.yMin, hi = spectrum.yMax;
    int prev = -1;
    for (int x = 0; x < w; ++x) {
        double v = y[static_cast<size_t>(static_cast<double>(x) / w * y.size())];
        int py = strip + specH - 1 - static_cast<int>((v - lo) / (hi - lo) * (specH - 1));
        py = std::max(strip, std::min(strip + specH - 1, py));
        c.vline(x, py, strip + specH - 1, lerp(t.background, t.accent, 0.3));
        if (prev >= 0) c.line(x - 1, prev, x, py, t.accent);
        prev = py;
    }
    std::vector<Rgb> map = colormapTable(mapIndex < 0 ? t.colormap : maps[static_cast<size_t>(mapIndex)]);
    for (int r = 0; r < wfH; ++r) {
        const std::vector<float>& row = wfRows[static_cast<size_t>(r)];
        for (int x = 0; x < w; ++x) {
            float v = row[static_cast<size_t>(static_cast<double>(x) / w * row.size())];
            double f = (v - waterfall.low) / std::max(1.0, waterfall.high - waterfall.low);
            img.set(x, strip + specH + r, map[static_cast<size_t>(std::max(0.0, std::min(1.0, f)) * 255)]);
        }
    }
    savePicture(img, "snapshot");
}

void Ui::Impl::toggleSurvey() {
    if (surveyNext > 0) {
        surveyNext = 0;
        say("band survey stopped");
        return;
    }
    surveyDials.clear();
    const std::vector<dec::FtxDial>& all = dec::ft8Dials();
    for (size_t i = 0; i < all.size(); ++i)
        if (all[i].hz >= 3.5e6 && all[i].hz <= 29.7e6) surveyDials.push_back(all[i].hz);  // 80 m to 10 m
    if (surveyDials.empty()) return;
    surveyIndex = 0;
    if (!(snap.decoderOn && snap.decoderKind == dec::Kind::Ft8)) startDecoder(dec::Kind::Ft8);
    tune(surveyDials[0], false);
    double now = clock.now(pcNow());
    surveyNext = (std::floor(now / 15.0) + 3) * 15.0 + 1;  // this slot's rest, then two whole slots
    listTabs.selected = 3;
    fillList();
    say("band survey: FT8 on each band, two slots each (Survey again: stop)");
}

void Ui::Impl::surveyStep() {
    if (surveyNext <= 0 || clock.now(pcNow()) < surveyNext) return;
    surveyIndex = (surveyIndex + 1) % surveyDials.size();
    tune(surveyDials[surveyIndex], false);
    surveyNext += 30;
}

// Setup: where you are (locator or a city), for distances and bearings;
// the clock and the stick's correction follow from T and K.
void Ui::Impl::setup() {
    ask("Setup: your location", "Locator or city:", store.get("locator"), [this](const std::string& text) {
        std::string grid = text;
        double lat, lon;
        if (!dec::Locator::toLatLon(grid, lat, lon)) {
            const dec::City* city = dec::findCity(text);
            if (!city) {
                prompt.error = "not a locator (e.g. JO21) and not a city I know: " + text;
                return;
            }
            grid = dec::Locator::fromLatLon(city->lat, city->lon);
        }
        store.set("locator", grid);
        radio.setLocator(grid);
        say("location " + grid + "; next: K calibrates the stick on a known station, T sets the clock from a time signal");
    }, "a Maidenhead locator (JO21ek) or a city (Amsterdam, London, Sao Paulo)");
}

// T / "Set clock": the decoded time code becomes the clock's correction.
void Ui::Impl::setClock() {
    const dec::Status& d = snap.decoder;
    if (!snap.decoderOn || snap.decoderKind != dec::Kind::Time) {
        showMessage("Set clock", "Start the time-signal decoder (d, Time signal) on CHU (3330/7850/14670 kHz), WWV/WWVH "
                                 "(2.5/5/10/15/20 MHz) or RWM (4996/9996/14996 kHz), wait for a full minute's time code, then press T.");
        return;
    }
    if (!d.timeValid) {
        showMessage("Set clock", "No full time code yet (" + d.info + "). CHU and WWV send one every minute.");
        return;
    }
    double pc = pcNow();
    clock.set(-d.clockOffset, d.station + " " + formatShort(radio.settings().frequency), dec::ClockSetKind::Code, pc);
    say("clock set from " + d.station + fmt(": the computer was %+.3f s off", -d.clockOffset));
}

void Ui::Impl::showHelp() {
    help.place(screen(), std::min(90, app.surface().width() - 2));
    help.bounds.height = std::min(app.surface().height() - 2, 21);
    app.openPopup(help, true);
}

// Pictures are saved where the program was started, like WAV recordings.
void Ui::Impl::savePicture(const Image& img, const std::string& kind) {
    if (img.empty()) return;
    std::time_t now = std::time(0);
    char name[96];
    std::strftime(name, sizeof(name), "%Y%m%d-%H%M%S.png", std::localtime(&now));
    std::string path = "offgrid-" + kind + "-" + name;
    std::string error;
    if (savePng(path, img, error)) say("picture saved: " + path);
    else say(error);
}

// ------------------------------------------- filters, AUTO, monitor, view --

void Ui::Impl::openMenu(Menu& m, int x) {
    m.place(x, 4, screen());
    app.openPopup(m);
}

void Ui::Impl::fillFilterMenu() {
    static const char* nr[] = {"off", "low", "medium", "high"};
    static const char* vc[] = {"off", "light", "medium", "strong", "full"};
    RadioSettings f = radio.settings();
    filterMenu.items.clear();
    filterMenu.addItem("Noise blanker (clicks, ignition)", std::string(), f.noiseBlanker);
    filterMenu.addItem(std::string("Noise reduction: ") + nr[std::max(0, std::min(3, f.noiseReduction))], std::string(),
                       f.noiseReduction > 0);
    filterMenu.addItem(f.humFilterHz ? fmt("Hum filter: %d Hz", f.humFilterHz) : std::string("Hum filter: off"), std::string(),
                       f.humFilterHz > 0);
    filterMenu.addItem("Auto notch (whistles)", std::string(), f.autoNotch);
    filterMenu.addItem(std::string("AI voice cleaner: ") + vc[std::max(0, std::min(4, f.voiceCleaner))], std::string(),
                       f.voiceCleaner > 0);
    filterMenu.addItem("Limiter", std::string(), f.limiter);
    filterMenu.addSeparator();
    filterMenu.addItem("All off");
}

void Ui::Impl::fillViewMenu() {
    viewMenu.items.clear();
    viewMenu.addItem("Peak hold", std::string(), peakHold);
    viewMenu.addItem("Averaging", std::string(), averaging);
    viewMenu.addItem("New-signal alerts", std::string(), alerts);
}

void Ui::Impl::startAuto() {
    stopScan();
    autoDabUntil = 0;
    radio.startAuto();
    autoWaiting = true;
    say("AUTO: listening...");
}

// Applies AUTO's verdict (as the browser version: the mode, the decoder and
// where to tune); DAB must then lock within 6 s, else NFM.
void Ui::Impl::autoStep() {
    if (autoDabUntil > 0) {
        if (snap.decoderOn && snap.decoderKind == dec::Kind::Dab && snap.decoder.locked) {
            autoDabUntil = 0;
            say("AUTO: DAB ensemble found");
        } else if (pcNow() > autoDabUntil) {
            autoDabUntil = 0;
            stopDecoder();
            setMode(dsp::Mode::NFM);
            say("AUTO: no DAB ensemble here; NFM");
        }
    }
    if (!autoWaiting) return;
    dsp::AutoResult r;
    if (!radio.takeAutoResult(r)) return;
    autoWaiting = false;
    dec::Kind kind;
    dsp::Mode mode;
    if (!r.decoder.empty() && dec::parseKind(r.decoder, kind)) {
        // AUTO's tuneHz is the signal's frequency, as the dial in decoder
        // modes (the receiver sits the decoder's shift lower).
        if (kind != dec::Kind::Dab && r.tuneHz > 0) tune(r.tuneHz, false);
        if (!(snap.decoderOn && snap.decoderKind == kind)) startDecoder(kind);
        if (r.needsConfirm) autoDabUntil = pcNow() + 6;
    } else {
        if (snap.decoderOn) stopDecoder();
        if (dsp::parseMode(r.mode, mode)) setMode(mode);
        if (r.tuneHz > 0) tune(r.tuneHz, false);
    }
    syncControls();
    std::string text = "AUTO: " + (r.label.empty() ? r.mode : r.label);
    if (!r.sure.empty()) text += " (" + r.sure + ")";
    if (r.tuneHz > 0) text += " at " + formatFrequency(r.tuneHz);
    if (!r.found) text = "AUTO: nothing recognised; " + (r.label.empty() ? r.mode : r.label);
    say(text);
}

void Ui::Impl::monitorHere() {
    if (radio.monitorCount() >= 8) {
        say("the monitor watches 8 signals at most (Del removes one from its list)");
        return;
    }
    RadioSettings s = radio.settings();
    double lo, hi;
    passband(lo, hi);
    std::string name = snap.rds.ps;
    while (!name.empty() && name[name.size() - 1] == ' ') name.erase(name.size() - 1);
    for (size_t i = 0; i < store.labels.size() && name.empty(); ++i)
        if (std::fabs(store.labels[i].hz - s.frequency) <= 500) name = store.labels[i].text;
    if (name.empty()) name = formatShort(s.frequency) + " " + dsp::modeName(s.mode);
    if (radio.monitorAdd((lo + hi) / 2, hi - lo, dsp::modeName(s.mode), name) < 0) {
        say("cannot watch that signal");
        return;
    }
    if (monitorStart == 0) monitorStart = clock.now(pcNow());
    listTabs.selected = 4;
    fillList();
    say("watching " + name + " (Monitor list; Monitor menu: follow, save the log)");
}

void Ui::Impl::saveMonitorCsv() {
    if (radio.monitorCount() == 0) {
        say("nothing watched yet (O watches the signal you are on)");
        return;
    }
    std::time_t now = std::time(0);
    char name[64];
    std::strftime(name, sizeof(name), "offgrid-monitor-%Y%m%d-%H%M%S.csv", std::localtime(&now));
    std::ofstream f(name);
    f << radio.monitorCsv(monitorStart);
    if (f) say(std::string("saved ") + name);
    else say(std::string("cannot write ") + name);
}

// ------------------------------------------------------------------- lists --

void Ui::Impl::fillList() {
    list.items.clear();
    if (listTabs.selected == 0) {
        const std::vector<Band>& b = bands();
        for (size_t i = 0; i < b.size(); ++i) {
            ListBox::Item it = {b[i].name, b[i].group.substr(0, 5), false, rgb(0, 0, 0)};
            list.items.push_back(it);
        }
    } else if (listTabs.selected == 1) {
        for (size_t i = 0; i < store.memories.size(); ++i) {
            const Memory& m = store.memories[i];
            ListBox::Item it = {m.name, formatShort(m.hz) + " " + dsp::modeName(m.mode), false, rgb(0, 0, 0)};
            list.items.push_back(it);
        }
        if (store.memories.empty()) {
            ListBox::Item it = {"(M saves a memory)", std::string(), true, app.theme().dim};
            list.items.push_back(it);
        }
    } else if (listTabs.selected == 4) {
        // The Signal Monitor: one line a signal (Enter tunes to it).
        for (int i = 0; i < radio.monitorCount(); ++i) {
            dsp::MonitorStats st = radio.monitorStats(i);
            std::string name = radio.monitorName(i);
            if (radio.monitorFollowing(i)) name = "\xE2\x86\x94" + name;  // following
            std::string right = st.away ? std::string("out of view")
                                        : std::string(st.on ? "\xE2\x97\x8F" : " ") + fmt("%2.0f dB", st.snr) +
                                              fmt(" %3.0f%%", st.duty * 100);
            ListBox::Item it = {name, right, false, rgb(0, 0, 0)};
            list.items.push_back(it);
            std::string detail = fmt("  %d tx", st.nTx);
            if (st.interval == st.interval) detail += fmt(", every %.1f s", st.interval);
            if (st.driftPerMin == st.driftPerMin && st.driftPerMin != 0) detail += fmt(", %+.1f Hz/min", st.driftPerMin);
            ListBox::Item d = {detail, std::string(), true, app.theme().dim};
            list.items.push_back(d);
        }
        if (radio.monitorCount() == 0) {
            ListBox::Item it = {"(O watches the signal you are on)", std::string(), true, app.theme().dim};
            list.items.push_back(it);
        }
    } else if (listTabs.selected == 3) {
        // Heard: likely bands open (from the locator, or the time zone), then
        // who this station hears, band by band, the last hour (every FT8 /
        // FT4 / WSPR decode).
        {
            double lat = 0, lon = 0;
            bool fromLocator = dsp::locatorToLatLon(store.get("locator"), &lat, &lon);
            if (!fromLocator) {
                std::time_t t = std::time(0);
                std::tm local = *std::localtime(&t), utc = *std::gmtime(&t);
                local.tm_isdst = 0;
                int east = static_cast<int>(std::difftime(std::mktime(&local), std::mktime(&utc)) / 60);
                dsp::placeFromUtcOffset(east, &lat, &lon);
            }
            double now = clock.now(pcNow());
            dsp::PropGuide ham = dsp::propagationGuide(now, lat, lon, true, fromLocator);
            dsp::PropGuide bc = dsp::propagationGuide(now, lat, lon, false, fromLocator);
            std::string open, sw;
            for (size_t i = 0; i < ham.open.size(); ++i) open += (i ? ", " : "") + ham.open[i];
            for (size_t i = 0; i < bc.open.size(); ++i) sw += (i ? ", " : "") + bc.open[i];
            ListBox::Item a = {"Open: " + (open.empty() ? std::string("none likely") : open), std::string("ham"), true, app.theme().good};
            list.items.push_back(a);
            if (!ham.possible.empty()) {
                std::string p;
                for (size_t i = 0; i < ham.possible.size(); ++i) p += (i ? ", " : "") + ham.possible[i];
                ListBox::Item b = {"Possible: " + p, std::string(), true, app.theme().dim};
                list.items.push_back(b);
            }
            ListBox::Item c = {"Open: " + (sw.empty() ? std::string("none likely") : sw), std::string("SW"), true, app.theme().good};
            list.items.push_back(c);
            if (!fromLocator) {
                ListBox::Item d = {"(Setup: your locator for a better guess)", std::string(), true, app.theme().dim};
                list.items.push_back(d);
            }
        }
        size_t guideLines = list.items.size();
        std::string text = radio.heardText(pcNow() - clock.offsetAt(pcNow()), 3600, false);
        size_t start = 0;
        while (start < text.size()) {
            size_t nl = text.find('\n', start);
            std::string line = text.substr(start, nl == std::string::npos ? std::string::npos : nl - start);
            if (!line.empty()) {
                ListBox::Item it = {line, std::string(), false, rgb(0, 0, 0)};
                list.items.push_back(it);
            }
            if (nl == std::string::npos) break;
            start = nl + 1;
        }
        if (list.items.size() == guideLines) {
            ListBox::Item it = {"(FT8, FT4 and WSPR decodes show here)", std::string(), true, app.theme().dim};
            list.items.push_back(it);
        }
    } else {
        logCache = Store::readLog(200);
        for (size_t i = 0; i < logCache.size(); ++i) {
            const LogEntry& e = logCache[i];
            ListBox::Item it = {e.utc.substr(5, 11) + " " + (e.station.empty() ? e.notes : e.station), formatShort(e.hz), false,
                                rgb(0, 0, 0)};
            list.items.push_back(it);
        }
        if (logCache.empty()) {
            ListBox::Item it = {"(L logs what you hear)", std::string(), true, app.theme().dim};
            list.items.push_back(it);
        }
    }
    if (list.selected >= static_cast<int>(list.items.size())) list.selected = static_cast<int>(list.items.size()) - 1;
    if (list.selected < 0 && !list.items.empty()) list.selected = 0;
}

void Ui::Impl::saveState() {
    RadioSettings s = radio.settings();
    store.setNumber("frequency", s.frequency);
    store.set("mode", dsp::modeName(s.mode));
    store.setNumber("bandwidth", s.bandwidth);
    store.setNumber("volume", s.volume);
    store.setNumber("squelch", s.squelchDb);
    store.setNumber("gain", s.gainDb);
    store.setNumber("ppm", s.ppm);
    store.set("stereo", s.stereo ? "1" : "0");
    store.setNumber("deemphasis", s.deemphasisUs);
    store.set("noise_blanker", s.noiseBlanker ? "1" : "0");
    store.setNumber("noise_reduction", s.noiseReduction);
    store.setNumber("hum_filter", s.humFilterHz);
    store.set("auto_notch", s.autoNotch ? "1" : "0");
    store.setNumber("voice_cleaner", s.voiceCleaner);
    store.set("limiter", s.limiter ? "1" : "0");
    store.set("theme", themes[themeIndex]);
    store.set("colormap", mapIndex < 0 ? std::string() : std::string(colormapName(maps[static_cast<size_t>(mapIndex)])));
    store.setNumber("step", stepIndex);
    store.setNumber("zoom", zoom.value);
    store.setNumber("wf_range", wfRange.value);
    std::string error;
    store.save(error);  // nothing useful to do with an error at exit
}

// ------------------------------------------------------------------- build --

void Ui::Impl::build() {
    freqText.font = Font::Font3x5;
    freqText.mode = PixelMode::Quadrants;
    unitLabel.text = "MHz";
    for (int i = 0; i < kModeCount; ++i) modes.options.push_back(dsp::modeName(kModes[i]));
    modes.onChange = [this](int i) {
        stopScan();
        setMode(kModes[i]);
    };

    meter.label = "S";
    meter.min = -120;
    meter.max = -10;
    meter.warnAt = -40;
    meter.badAt = -20;
    meter.segmented = true;
    meter.scale.push_back(std::make_pair(-100.0, std::string("-100")));
    meter.scale.push_back(std::make_pair(-80.0, std::string("-80")));
    meter.scale.push_back(std::make_pair(-60.0, std::string("-60")));
    meter.scale.push_back(std::make_pair(-40.0, std::string("-40")));
    meter.scale.push_back(std::make_pair(-20.0, std::string("-20 dB")));
    levelHistory.min = -120;
    levelHistory.max = -10;

    spectrum.showGrid = true;
    spectrum.maxDecimation = true;
    spectrum.xTicks = 6;
    spectrum.yTicks = 4;
    spectrum.formatX = [](double hz) {
        char b[32];
        if (hz >= 1e6) std::snprintf(b, sizeof(b), "%.3f", hz / 1e6);
        else std::snprintf(b, sizeof(b), "%.1fk", hz / 1e3);
        return std::string(b);
    };
    spectrum.formatY = [](double v) { return fmt("%.0f", v); };
    Plot::Series series;
    series.fill = true;
    series.useColor = false;
    series.color = rgb(0, 0, 0);
    spectrum.series.push_back(series);
    Plot::Series peak;  // peak hold (View menu)
    peak.fill = false;
    peak.useColor = true;
    peak.color = rgb(0, 0, 0);
    spectrum.series.push_back(peak);
    spectrum.onMouse = [this](double x, double, const Event& e) {
        using terminal::MouseAction;
        using terminal::MouseButton;
        if (e.button == MouseButton::WheelUp) tune(radio.settings().frequency + step());
        else if (e.button == MouseButton::WheelDown) tune(radio.settings().frequency - step());
        else if (e.button == MouseButton::Left && (e.action == MouseAction::Press || e.action == MouseAction::Drag)) {
            stopScan();
            tune(x);
        }
    };
    waterfall.onMouse = [this](double f, const Event& e) {
        using terminal::MouseAction;
        using terminal::MouseButton;
        if (e.button == MouseButton::WheelUp) tune(radio.settings().frequency + step());
        else if (e.button == MouseButton::WheelDown) tune(radio.settings().frequency - step());
        else if (e.button == MouseButton::Left && (e.action == MouseAction::Press || e.action == MouseAction::Drag)) {
            stopScan();
            tune(winLow + f * (winHigh - winLow));
        }
    };
    decoderView.visible = false;
    decoderView.onTitleClick = [this]() { openDecoderMenu(); };
    decoderView.onSelectService = [this](unsigned id) {
        radio.decoderCommand([id](dec::Decoder& d) { d.selectService(id); });
        say("station chosen");
    };
    decoderView.onPictureDone = [this](const Image& img) {
        savePicture(img, snap.decoderOn ? std::string(dec::kindName(snap.decoderKind)) : std::string("picture"));
    };
    // The toolbar: one click for each action that has a key.
    struct Tool {
        const char* label;
        std::function<void()> action;
    };
    Tool tools[] = {
        {"Decoder \xE2\x96\xBE", [this]() { openDecoderMenu(); }},
        {"AUTO", [this]() { startAuto(); }},
        {"Filters \xE2\x96\xBE", [this]() {
             fillFilterMenu();
             openMenu(filterMenu, 12);
         }},
        {"Monitor \xE2\x96\xBE", [this]() { openMenu(monitorMenu, 22); }},
        {"View \xE2\x96\xBE", [this]() {
             fillViewMenu();
             openMenu(viewMenu, 32);
         }},
        {"Memory +", [this]() { saveMemory(); }},
        {"Label", [this]() { labelHere(); }},
        {"Log", [this]() { logHere(); }},
        {"Scan", [this]() {
             if (scanKind != ScanOff) stopScan();
             else startScan(ScanBand);
         }},
        {"Scan mem", [this]() {
             if (scanKind != ScanOff) stopScan();
             else startScan(ScanMemories);
         }},
        {"Calibrate", [this]() { calibrate(); }},
        {"Record", [this]() { toggleRecording(); }},
        {"Save picture", [this]() {
             if (decoderView.lineMode) savePicture(decoderView.pictureImage(), dec::kindName(snap.decoderKind));
             else if (decoderView.serviceMode && !decoderView.slide.empty()) savePicture(decoderView.slide, dec::kindName(snap.decoderKind));
             else say("no picture: start the FAX or SSTV decoder (d)");
         }},
        {"Set clock", [this]() { setClock(); }},
        {"Replay", [this]() {
             if (radio.replaying()) {
                 radio.stopReplay();
                 say("live again");
                 return;
             }
             std::string error;
             if (radio.startReplay(error)) say("replaying the last seconds (tune around; Replay again: live)");
             else say(error);
         }},
        {"Save IQ", [this]() {
             std::time_t now = std::time(0);
             char name[96];
             RadioSettings s = radio.settings();
             std::strftime(name, sizeof(name), "%Y%m%d-%H%M%S", std::localtime(&now));
             std::string path = std::string("offgrid-") + name + fmt("-%.0fHz", snap.centerHz) + fmt("-%.0fsps.cu8", snap.sampleRate);
             std::string error;
             if (radio.saveIq(path, error)) say("saved " + path + " (play: --file it --center " + fmt("%.0f", snap.centerHz) + ")");
             else say(error);
             (void)s;
         }},
        {"Snapshot", [this]() { snapshotPicture(); }},
        {"Setup", [this]() { setup(); }},
        {"Survey", [this]() { toggleSurvey(); }},
        {"Theme", [this]() {
             themeIndex = (themeIndex + 1) % themes.size();
             app.setTheme(theme(themes[themeIndex]));
             waterfall.clear();
         }},
        {"Help", [this]() { showHelp(); }},
        {"Quit", [this]() { app.quit(); }},
    };
    for (size_t i = 0; i < sizeof(tools) / sizeof(tools[0]); ++i) {
        std::unique_ptr<Button> b(new Button(tools[i].label));
        b->brackets = false;
        b->onPress = tools[i].action;
        toolbar.push_back(std::move(b));
    }

    listTabs.tabs.push_back("Bands");
    listTabs.tabs.push_back("Memories");
    listTabs.tabs.push_back("Log");
    listTabs.tabs.push_back("Heard");
    listTabs.tabs.push_back("Monitor");
    listTabs.onChange = [this](int) {
        list.selected = 0;
        fillList();
    };
    list.border = false;
    list.onActivate = [this](int i) {
        stopScan();
        if (listTabs.selected == 0) {
            const Band& band = bands()[static_cast<size_t>(i)];
            std::map<std::string, double>::const_iterator mem = store.bandMemory.find(band.name);
            double f = mem != store.bandMemory.end() && mem->second >= band.low && mem->second <= band.high ? mem->second : band.start;
            radio.setMode(band.mode);
            for (int k = 0; k < kStepCount; ++k)
                if (std::fabs(kSteps[k] - band.step) < 1) stepIndex = k;
            tune(f, false);
        } else if (listTabs.selected == 1 && i < static_cast<int>(store.memories.size())) {
            const Memory& m = store.memories[static_cast<size_t>(i)];
            radio.setMode(m.mode);
            if (m.bandwidth > 0) radio.setBandwidth(m.bandwidth);
            tune(m.hz, false);
        } else if (listTabs.selected == 2 && i < static_cast<int>(logCache.size())) {
            tune(logCache[static_cast<size_t>(i)].hz, false);
        } else if (listTabs.selected == 4 && i / 2 < radio.monitorCount()) {  // two lines a signal
            tune(radio.monitorCentre(i / 2), false);
        }
        syncControls();
        app.clearFocus();
    };

    volume.step = 5;
    volume.format = [](double v) { return fmt("%.0f%%", v); };
    volume.onChange = [this](double v) { radio.setVolume(v / 100.0); };
    squelch.step = 1;
    squelch.format = [](double v) { return v <= -120 ? std::string("off") : fmt("%.0f dB", v); };
    squelch.onChange = [this](double v) { radio.setSquelch(v <= -120 ? -200 : v); };
    gain.min = 0;
    gain.max = gainSteps.empty() ? 1 : static_cast<double>(gainSteps.size() - 1);
    gain.step = 1;
    gain.value = gain.max * 0.7;
    gain.format = [this](double v) {
        if (radio.settings().gainDb < 0) return std::string("auto");
        size_t i = static_cast<size_t>(v + 0.5);
        return gainSteps.empty() ? std::string("-") : fmt("%.1f dB", gainSteps[std::min(i, gainSteps.size() - 1)]);
    };
    gain.onChange = [this](double v) {
        if (gainSteps.empty()) return;
        size_t i = std::min(static_cast<size_t>(v + 0.5), gainSteps.size() - 1);
        radio.setGain(gainSteps[i]);
        autoGain.checked = false;
    };
    autoGain.onChange = [this](bool on) {
        if (on) radio.setGain(-1);
        else if (!gainSteps.empty()) radio.setGain(gainSteps[std::min(static_cast<size_t>(gain.value + 0.5), gainSteps.size() - 1)]);
    };
    bandwidth.format = [](double v) { return formatShort(v); };
    bandwidth.onChange = [this](double v) { radio.setBandwidth(v); };
    zoom.step = 1;
    zoom.format = [](double v) { return fmt("x%.0f", std::pow(2.0, std::floor(v + 0.5))); };
    wfFloor.step = 1;
    wfFloor.format = [](double v) { return fmt("%.0f dB", v); };
    wfFloor.onChange = [this](double) { autoFloor = false; };
    wfRange.step = 1;
    wfRange.format = [](double v) { return fmt("%.0f dB", v); };
    stereo.onChange = [this](bool on) { radio.setStereo(on); };
    biasTee.onChange = [this](bool on) { radio.setBiasTee(on); };

    prompt.input.onSubmit = [this](const std::string& text) {
        std::function<void(const std::string&)> action = promptAction;
        prompt.error.clear();
        if (action) action(text);
        if (prompt.error.empty()) app.closePopup(prompt);
    };
    prompt.input.onCancel = [this]() { app.closePopup(prompt); };
    confirm.onClose = [this](int button) {
        if (confirmAction) confirmAction(button);
    };

    decoderMenu.title = "Decoder";
    std::vector<dec::Kind> kinds = dec::allKinds();
    for (size_t i = 0; i < kinds.size(); ++i) {
        std::string name = dec::kindName(kinds[i]);
        switch (kinds[i]) {
            case dec::Kind::Cw: name = "CW (Morse)"; break;
            case dec::Kind::Psk: name = "PSK31 / 63 / 125"; break;
            case dec::Kind::Dsc: name = "DSC (ships' calls)"; break;
            case dec::Kind::Hell: name = "Hellschreiber"; break;
            case dec::Kind::Fax: name = "Weather fax"; break;
            case dec::Kind::Sstv: name = "SSTV (pictures)"; break;
            case dec::Kind::Time: name = "Time signal (CHU/WWV/RWM)"; break;
            case dec::Kind::Dab: name = "DAB / DAB+ (digital radio)"; break;
            case dec::Kind::Drm: name = "DRM (digital shortwave)"; break;
            default: break;
        }
        decoderMenu.addItem(name);
    }
    decoderMenu.addSeparator();
    decoderMenu.addItem("Off", "D");
    decoderMenu.onPick = [this](int i) {
        std::vector<dec::Kind> k = dec::allKinds();
        if (i >= 0 && i < static_cast<int>(k.size())) startDecoder(k[static_cast<size_t>(i)]);
        else stopDecoder();
    };

    filterMenu.title = "Clean-up filters";
    filterMenu.onPick = [this](int i) {
        RadioSettings f = radio.settings();
        std::string what;
        static const char* nr[] = {"off", "low", "medium", "high"};
        static const char* vc[] = {"off", "light", "medium", "strong", "full"};
        switch (i) {
            case 0: f.noiseBlanker = !f.noiseBlanker; what = std::string("noise blanker ") + (f.noiseBlanker ? "on" : "off"); break;
            case 1: f.noiseReduction = (f.noiseReduction + 1) % 4; what = std::string("noise reduction ") + nr[f.noiseReduction]; break;
            case 2: f.humFilterHz = f.humFilterHz == 0 ? 50 : (f.humFilterHz == 50 ? 60 : 0);
                    what = f.humFilterHz ? fmt("hum filter %d Hz", f.humFilterHz) : std::string("hum filter off"); break;
            case 3: f.autoNotch = !f.autoNotch; what = std::string("auto notch ") + (f.autoNotch ? "on" : "off"); break;
            case 4: f.voiceCleaner = (f.voiceCleaner + 1) % 5; what = std::string("AI voice cleaner ") + vc[f.voiceCleaner]; break;
            case 5: f.limiter = !f.limiter; what = std::string("limiter ") + (f.limiter ? "on" : "off"); break;
            case 7:
                f.noiseBlanker = f.autoNotch = false;
                f.noiseReduction = f.humFilterHz = f.voiceCleaner = 0;
                what = "filters off (limiter kept)";
                break;
            default: return;
        }
        radio.setFilters(f);
        dsp::Mode m = f.mode;
        bool voice = m == dsp::Mode::AM || m == dsp::Mode::SAM || m == dsp::Mode::USB || m == dsp::Mode::LSB;
        if (!voice && i != 5 && !((i == 0 || i == 2) && m == dsp::Mode::CW)) what += " (works in AM, SAM, USB, LSB)";
        say(what);
    };
    monitorMenu.title = "Signal Monitor";
    monitorMenu.addItem("Watch this signal", "O");
    monitorMenu.addItem("Show the list");
    monitorMenu.addItem("Follow the selected signal on/off");
    monitorMenu.addItem("Stop watching the selected signal", "Del");
    monitorMenu.addItem("Save the log (CSV)");
    monitorMenu.onPick = [this](int i) {
        int sel = listTabs.selected == 4 && list.selected >= 0 ? list.selected / 2 : -1;
        if (i == 0) monitorHere();
        else if (i == 1) {
            listTabs.selected = 4;
            fillList();
        } else if (i == 2 || i == 3) {
            if (sel < 0 || sel >= radio.monitorCount()) {
                say("choose a signal in the Monitor list first");
                return;
            }
            if (i == 3) {
                radio.monitorRemove(sel);
                say("stopped watching");
            } else {
                bool on = !radio.monitorFollowing(sel);
                radio.monitorFollow(sel, on);
                say(on ? "following its drift" : "fixed in place");
            }
            fillList();
        } else if (i == 4) saveMonitorCsv();
    };
    viewMenu.title = "Spectrum";
    viewMenu.onPick = [this](int i) {
        if (i == 0) peakHold = !peakHold;
        else if (i == 1) averaging = !averaging;
        else if (i == 2) {
            alerts = !alerts;
            if (alerts) extras.restart();
            say(alerts ? "alerts on: learning the band for 10 s, then new signals are announced" : "alerts off");
        }
        extras.setPeakHold(peakHold);
        extras.setAveraging(averaging);
        extras.setAlerts(alerts);
        if (spectrum.series.size() > 1 && !peakHold) spectrum.series[1].y.clear();
    };

    help.title = "Keys";
    help.buttons.assign(1, "Close");
    help.message =
        "Left/Right  tune one step (Shift: 10 steps)      Up/Down  change the step\n"
        "PgUp/PgDn   tune 1 MHz                           Enter, f type a frequency\n"
        "1-7         WFM NFM AM SAM USB LSB CW            m  next mode\n"
        "b           the side list (Bands/Memories/Log)   Tab / Esc  side controls / back\n"
        "d / D       decoder menu / decoder off           R  record the sound to a WAV file\n"
        "M           save a memory here                   Del  delete the selected memory\n"
        "l           label this frequency (empty: remove) L  log what you hear\n"
        "n / N       scan this band / the memories        K  calibrate on a known station\n"
        "T           set the clock from a time signal (time decoder)\n"
        "A           AUTO: work out what the signal is      F  clean-up filters (blanker, NR, AI voice)\n"
        "O           watch this signal (Signal Monitor)    Del  stop watching (Monitor list)\n"
        "j / k       DAB / DRM: next / previous station (or click it); DAB: Left/Right step channels\n"
        "+ / -, space  volume, mute                       g / G, a  RF gain down / up, auto\n"
        "s / S       squelch down / up                    w / W  filter narrower / wider\n"
        "z / Z       zoom in / out                        [ ] { }  waterfall floor / range\n"
        "t / c       theme / waterfall colors             F1, h, ?  this help    q  quit\n"
        "\n"
        "Mouse: click or drag on the spectrum or waterfall to tune, the wheel steps.";

    Widget* all[] = {&freqText, &unitLabel, &infoLabel, &clockLabel, &modes, &ledStereo, &ledRds, &ledSquelch, &ledOverload,
                     &ledRecord, &ledMute, &ledScan, &ledDecoder, &meter, &levelHistory, &spectrum, &waterfall, &decoderView,
                     &rdsLabel, &listPanel, &listTabs, &list, &controlsPanel, &volume, &squelch, &gain, &autoGain,
                     &bandwidth, &zoom, &wfFloor, &wfRange, &stereo, &biasTee, &status};
    for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); ++i) app.add(*all[i]);
    for (size_t i = 0; i < toolbar.size(); ++i) app.add(*toolbar[i]);
    app.onLayout = [this](const Rect& r) { layout(r); };
    app.onDrawBackground = [this](Painter& p) { p.fill(rect(0, 0, p.width(), 5), lerp(app.theme().background, app.theme().panel, 0.8)); };
    syncControls();
    fillList();
    bindKeys();
    if (store.values.empty() && options.decoder.empty())
        showMessageLater = true;
    dec::Kind k;
    std::string want = !options.decoder.empty() ? options.decoder : store.get("decoder");
    if (!want.empty() && dec::parseKind(want, k)) startDecoder(k);
}

// ------------------------------------------------------------------ layout --

void Ui::Impl::layout(const Rect& r) {
    int w = r.width, h = r.height;
    int side = w >= 110 ? 34 : (w >= 90 ? 28 : 0);
    int fw, fh;
    freqText.text = formatFrequency(radio.settings().frequency);
    freqText.measure(fw, fh);
    freqText.bounds = rect(1, 0, fw, 3);
    unitLabel.bounds = rect(fw + 2, 2, 4, 1);
    modes.bounds = rect(fw + 7, 0, 46, 1);
    infoLabel.bounds = rect(fw + 7, 1, std::max(10, w - fw - 7 - 46), 1);
    int lx = fw + 7;
    Led* leds[] = {&ledStereo, &ledRds, &ledSquelch, &ledOverload, &ledRecord, &ledMute, &ledScan, &ledDecoder};
    for (size_t i = 0; i < 8; ++i) {
        int lw = leds[i] == &ledDecoder ? 9 : static_cast<int>(leds[i]->label.size()) + 3;  // its label changes
        leds[i]->bounds = rect(lx, 2, lw, 1);
        lx += lw + 1;
    }
    int meterX = std::max(lx + 1, w - 46);
    meter.bounds = rect(meterX, 0, w - meterX - 1, 2);
    levelHistory.bounds = rect(meterX + 2, 2, std::max(0, w - meterX - 16), 1);
    clockLabel.bounds = rect(w - 13, 2, 12, 1);
    int tx = 1, ty = 3;  // toolbar on row 3, wrapping onto row 4
    for (size_t i = 0; i < toolbar.size(); ++i) {
        int bw = columns(toolbar[i]->label) + 2;
        if (tx + bw > w && ty == 3) {
            tx = 1;
            ty = 4;
        }
        toolbar[i]->visible = tx + bw <= w;
        toolbar[i]->bounds = rect(tx, ty, bw, 1);
        tx += bw + 1;
    }
    status.bounds = rect(0, h - 1, w, 1);
    rdsLabel.bounds = rect(1, h - 2, w - 2, 1);
    Rect main = rect(0, 5, w - side - (side ? 1 : 0), h - 5 - 2);
    int decH = decoderView.visible ? std::max(5, main.height / 3) : 0;
    int rest = main.height - decH;
    int specH = std::max(4, rest * 2 / 5);
    spectrum.bounds = rect(main.x, main.y, main.width, specH);
    Rect pa = spectrum.plotArea();
    waterfall.bounds = rect(main.x + pa.x, main.y + specH, std::max(1, main.width - pa.x), rest - specH);
    decoderView.bounds = rect(main.x, main.y + rest, main.width, decH);
    bool showSide = side > 0;
    Widget* sideWidgets[] = {&listPanel, &listTabs, &list, &controlsPanel, &volume, &squelch, &gain, &autoGain, &bandwidth,
                             &zoom, &wfFloor, &wfRange, &stereo, &biasTee};
    for (size_t i = 0; i < sizeof(sideWidgets) / sizeof(sideWidgets[0]); ++i) sideWidgets[i]->visible = showSide;
    if (showSide) {
        int sx = w - side;
        int controlsH = std::min(15, main.height - 8);
        int listH = main.height - controlsH;
        listPanel.bounds = rect(sx, main.y, side, listH);
        listTabs.bounds = rect(sx + 1, main.y + 1, side - 2, 1);
        list.bounds = rect(sx + 1, main.y + 2, side - 2, listH - 3);
        controlsPanel.bounds = rect(sx, main.y + listH, side, controlsH);
        Rect in = inset(controlsPanel.bounds, 1);
        in.x += 1;
        in.width -= 2;
        Widget* rows[] = {&volume, &squelch, &gain, &autoGain, &bandwidth, &zoom, &wfFloor, &wfRange};
        Slider* sliders[] = {&volume, &squelch, &gain, &bandwidth, &zoom, &wfFloor, &wfRange};
        for (size_t i = 0; i < 7; ++i) sliders[i]->labelWidth = 9;
        int y = in.y;
        for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); ++i, ++y)
            rows[i]->bounds = y < in.bottom() ? rect(in.x, y, in.width, 1) : rect(0, 0, 0, 0);
        stereo.bounds = y < in.bottom() ? rect(in.x, y, in.width / 2, 1) : rect(0, 0, 0, 0);
        biasTee.bounds = y < in.bottom() ? rect(in.x + in.width / 2, y, in.width - in.width / 2, 1) : rect(0, 0, 0, 0);
    }
    waterfall.clear();
}

// -------------------------------------------------------------------- keys --

void Ui::Impl::bindKeys() {
    using namespace terminal;
    // Tuning keys also stop a scan.
    // (With the side list focused, Left/Right change its tab instead.)
    app.bindKey(KeyRight, [this]() {
        if (app.focused() == &list) {
            listTabs.selected = (listTabs.selected + 1) % 5;
            listTabs.onChange(listTabs.selected);
            return;
        }
        stopScan();
        if (snap.decoderOn && snap.decoderKind == dec::Kind::Dab) {  // DAB: the next channel
            const std::vector<DabChannel>& c = dabChannels();
            const DabChannel& cur = dabNearest(radio.settings().frequency);
            size_t i = static_cast<size_t>(&cur - &c[0]);
            tune(c[std::min(c.size() - 1, i + 1)].hz, false);
            return;
        }
        tune(radio.settings().frequency + step());
    });
    app.bindKey(KeyLeft, [this]() {
        if (app.focused() == &list) {
            listTabs.selected = (listTabs.selected + 4) % 5;
            listTabs.onChange(listTabs.selected);
            return;
        }
        stopScan();
        if (snap.decoderOn && snap.decoderKind == dec::Kind::Dab) {  // DAB: the previous channel
            const std::vector<DabChannel>& c = dabChannels();
            const DabChannel& cur = dabNearest(radio.settings().frequency);
            size_t i = static_cast<size_t>(&cur - &c[0]);
            tune(c[i > 0 ? i - 1 : 0].hz, false);
            return;
        }
        tune(radio.settings().frequency - step());
    });
    app.bindKey(KeyUp, [this]() { stepIndex = std::min(kStepCount - 1, stepIndex + 1); });
    app.bindKey(KeyDown, [this]() { stepIndex = std::max(0, stepIndex - 1); });
    app.bindKey(KeyPageUp, [this]() {
        stopScan();
        tune(radio.settings().frequency + 1e6, false);
    });
    app.bindKey(KeyPageDown, [this]() {
        stopScan();
        tune(radio.settings().frequency - 1e6, false);
    });
    std::function<void()> askFrequency = [this]() { this->askFrequency(); };
    app.bindKey(KeyEnter, askFrequency);
    app.bindKey('f', askFrequency);
    for (int i = 0; i < kModeCount; ++i) {
        dsp::Mode m = kModes[i];
        app.bindKey('1' + i, [this, m]() { setMode(m); });
    }
    app.bindKey('m', [this]() { setMode(kModes[(modeIndex(radio.settings().mode) + 1) % kModeCount]); });
    app.bindKey('b', [this]() {
        if (list.visible) app.focus(list);
    });
    app.bindKey(KeyTab, [this]() { app.focusNext(false); });
    app.bindKey(KeyEscape, [this]() {
        if (scanKind != ScanOff) stopScan();
        else app.clearFocus();
    });
    app.bindKey('+', [this]() { volume.set(volume.value + 5); });
    app.bindKey('=', [this]() { volume.set(volume.value + 5); });
    app.bindKey('-', [this]() { volume.set(volume.value - 5); });
    app.bindKey(' ', [this]() { radio.setMuted(!radio.settings().muted); });
    app.bindKey('g', [this]() { gain.set(gain.value - 1); });
    app.bindKey('G', [this]() { gain.set(gain.value + 1); });
    app.bindKey('a', [this]() {
        autoGain.checked = true;
        radio.setGain(-1);
    });
    app.bindKey('s', [this]() { squelch.set(squelch.value - 2); });
    app.bindKey('S', [this]() { squelch.set(squelch.value + 2); });
    app.bindKey('w', [this]() { bandwidth.set(bandwidth.value - bandwidth.step); });
    app.bindKey('W', [this]() { bandwidth.set(bandwidth.value + bandwidth.step); });
    app.bindKey('z', [this]() { zoom.set(zoom.value + 1); });
    app.bindKey('Z', [this]() { zoom.set(zoom.value - 1); });
    app.bindKey('[', [this]() {
        autoFloor = false;
        wfFloor.set(wfFloor.value - 2);
    });
    app.bindKey(']', [this]() {
        autoFloor = false;
        wfFloor.set(wfFloor.value + 2);
    });
    app.bindKey('{', [this]() { wfRange.set(wfRange.value - 5); });
    app.bindKey('}', [this]() { wfRange.set(wfRange.value + 5); });
    app.bindKey('t', [this]() {
        themeIndex = (themeIndex + 1) % themes.size();
        app.setTheme(theme(themes[themeIndex]));
        waterfall.clear();
    });
    app.bindKey('c', [this]() {
        mapIndex = mapIndex + 1 >= static_cast<int>(maps.size()) ? -1 : mapIndex + 1;
        waterfall.clear();
    });
    app.bindKey('R', [this]() { toggleRecording(); });
    app.bindKey('A', [this]() { startAuto(); });
    app.bindKey('F', [this]() {
        fillFilterMenu();
        openMenu(filterMenu, 12);
    });
    app.bindKey('O', [this]() { monitorHere(); });
    app.bindKey('d', [this]() { openDecoderMenu(); });
    app.bindKey('D', [this]() { stopDecoder(); });
    app.bindKey('M', [this]() { saveMemory(); });
    app.bindKey(KeyDelete, [this]() {
        if (listTabs.selected == 1 && list.selected >= 0 && list.selected < static_cast<int>(store.memories.size())) {
            store.memories.erase(store.memories.begin() + list.selected);
            fillList();
        } else if (listTabs.selected == 4 && list.selected >= 0 && list.selected / 2 < radio.monitorCount()) {
            radio.monitorRemove(list.selected / 2);
            fillList();
        }
    });
    app.bindKey('l', [this]() { labelHere(); });
    app.bindKey('L', [this]() { logHere(); });
    app.bindKey('n', [this]() {
        if (scanKind == ScanBand) stopScan();
        else startScan(ScanBand);
    });
    app.bindKey('N', [this]() {
        if (scanKind == ScanMemories) stopScan();
        else startScan(ScanMemories);
    });
    app.bindKey('K', [this]() { calibrate(); });
    // DAB / DRM: next / previous station.
    std::function<void(int)> stepService = [this](int dir) {
        const std::vector<dec::Service>& sv = decoderView.services;
        if (sv.empty()) return;
        int cur = 0;
        for (size_t i = 0; i < sv.size(); ++i)
            if (sv[i].selected) cur = static_cast<int>(i);
        int next = (cur + dir + static_cast<int>(sv.size())) % static_cast<int>(sv.size());
        unsigned id = sv[static_cast<size_t>(next)].id;
        radio.decoderCommand([id](dec::Decoder& d) { d.selectService(id); });
    };
    app.bindKey('j', [stepService]() { stepService(1); });
    app.bindKey('k', [stepService]() { stepService(-1); });
    app.bindKey('T', [this]() { setClock(); });
    std::function<void()> showHelp = [this]() { this->showHelp(); };
    app.bindKey(KeyF1, showHelp);
    app.bindKey('h', showHelp);
    app.bindKey('?', showHelp);
    app.bindKey('q', [this]() { app.quit(); });
    app.bindKey('Q', [this]() { app.quit(); });
    app.onUnhandled = [this](const Event& e) {
        // The frequency display: click types a new one, the wheel tunes.
        if (e.type == EventType::Mouse && freqText.bounds.contains(e.x, e.y)) {
            if (e.button == MouseButton::WheelUp) tune(radio.settings().frequency + step());
            else if (e.button == MouseButton::WheelDown) tune(radio.settings().frequency - step());
            else if (e.button == MouseButton::Left && e.action == MouseAction::Press) this->askFrequency();
            return;
        }
        if (e.type != EventType::Key || !e.shift) return;
        if (e.key == KeyRight) tune(radio.settings().frequency + 10 * step());
        if (e.key == KeyLeft) tune(radio.settings().frequency - 10 * step());
    };
}

// ------------------------------------------------------------------ update --

void Ui::Impl::update() {
    radio.snapshot(snap);
    RadioSettings s = radio.settings();
    const Theme& t = app.theme();
    ++framesSinceStart;
    if (showMessageLater && framesSinceStart == 3) {
        showMessageLater = false;
        showMessage("Welcome to offgrid",
                    "Setup (toolbar) tells offgrid where you are, for distances and bearings. K calibrates the stick on "
                    "a station of known frequency, T sets the clock from a time signal (FT8, FT4 and WSPR need it right). "
                    "F1 lists every key; the mouse works everywhere.");
    }

    // Scan: judge the channel once the dwell time is over.
    if (scanKind != ScanOff) {
        if (scanDwell > 0) {
            --scanDwell;
        } else {
            double threshold = s.squelchDb > -200 ? s.squelchDb : snap.noiseDbfs + 12;
            bool signal = snap.channelDbfs >= threshold;
            if (signal) {
                scanStopped = true;
                scanHold = 60;  // 2 s after the signal goes
            } else if (scanStopped && --scanHold > 0) {
                // keep listening
            } else {
                scanStopped = false;
                scanStep_();
            }
        }
    }

    // Header.
    std::string ft = formatFrequency(s.frequency);
    if (ft != freqText.text) {
        freqText.text = ft;
        app.relayout();
    }
    const Band* band = bandAt(s.frequency);
    const DabChannel* dabCh = dabChannelAt(s.frequency);
    char info[200];
    std::snprintf(info, sizeof(info), "%s   step %s   filter %s   %s%s", dabCh && snap.decoderOn ? (std::string("DAB ") + dabCh->name).c_str() : (band ? band->name.c_str() : "-"), stepName(step()).c_str(),
                  formatShort(s.bandwidth > 0 ? s.bandwidth : dsp::defaultBandwidth(s.mode)).c_str(),
                  s.ppm != 0 ? (fmt("%+.2f ppm   ", s.ppm)).c_str() : "", app.focused() ? "[Esc: keys back to tuning]" : "");
    infoLabel.text = info;
    infoLabel.useColor = true;
    infoLabel.color = t.dim;
    modes.selected = modeIndex(s.mode);
    ledStereo.on = s.mode == dsp::Mode::WFM && snap.stereoPilot;
    ledRds.on = s.mode == dsp::Mode::WFM && snap.rds.groups > 0;
    ledSquelch.on = s.squelchDb > -200 && !snap.squelchOpen;
    ledSquelch.useColor = true;
    ledSquelch.color = t.warn;
    ledOverload.on = snap.overload;
    ledOverload.useColor = true;
    ledOverload.color = t.bad;
    ledRecord.on = radio.recording();
    ledRecord.useColor = true;
    ledRecord.color = t.bad;
    ledMute.on = s.muted;
    ledMute.useColor = true;
    ledMute.color = t.warn;
    ledScan.on = scanKind != ScanOff;
    ledScan.useColor = true;
    ledScan.color = scanStopped ? t.good : t.warn;
    ledDecoder.on = snap.decoderOn && snap.decoder.locked;
    ledDecoder.label = snap.decoderOn ? dec::kindName(snap.decoderKind) : "DEC";
    meter.update(snap.channelDbfs, 0.4);
    meter.valueText = fmt("%5.1f dB", snap.channelDbfs) + fmt("  SNR %4.1f", snap.channelDbfs - snap.noiseDbfs);
    if (framesSinceStart % 3 == 0) levelHistory.push(snap.channelDbfs, 512);
    if (listTabs.selected == 3 && framesSinceStart % 90 == 0) fillList();
    if (listTabs.selected == 4 && framesSinceStart % 10 == 0) fillList();
    surveyStep();
    autoStep();
    {
        std::string proto;
        std::vector<double> dts;
        if (radio.takeFtxTimings(proto, dts) && clock.applyFtxTimings(dts, proto, pcNow()))
            say("clock corrected from " + proto + " stations");
    }
    // The clock: corrected by time signals when set (marked with a dot);
    // slot-based decoders (FT8, FT4, WSPR) follow the corrected time.
    radio.setClockOffset(clock.offsetAt(pcNow()));
    double corrected = clock.now(pcNow());
    std::time_t now = static_cast<std::time_t>(corrected);
    char clockText[24];
    std::strftime(clockText, sizeof(clockText), "%H:%M:%S UTC", std::gmtime(&now));
    clockLabel.text = std::string(clock.baseAt() > 0 ? "\xE2\x80\xA2" : "") + clockText;
    clockLabel.tooltip = clock.sourceText(pcNow());
    clockLabel.align = Align::Right;

    // Spectrum window.
    if (!snap.spectrum.empty() && snap.sampleRate > 0) {
        int n = static_cast<int>(snap.spectrum.size());
        double span = snap.sampleRate;
        double z = std::pow(2.0, std::floor(zoom.value + 0.5));
        double win = span / z;
        double lowEdge = snap.centerHz - span / 2;
        double c = snap.centerHz;
        if (z > 1) {
            double q = win / 4;
            c = std::floor(s.frequency / q + 0.5) * q;
            c = std::max(lowEdge + win / 2, std::min(lowEdge + span - win / 2, c));
        }
        double newLow = c - win / 2, newHigh = c + win / 2;
        if (std::fabs(newLow - winLow) > 1 || std::fabs(newHigh - winHigh) > 1) waterfall.clear();
        winLow = newLow;
        winHigh = newHigh;
        int i0 = std::max(0, static_cast<int>((winLow - lowEdge) / span * n));
        int i1 = std::min(n, static_cast<int>((winHigh - lowEdge) / span * n + 0.5));
        if (snap.spectrumSerial != lastSerial && i1 > i0) {
            lastSerial = snap.spectrumSerial;
            std::vector<float> view(snap.spectrum.begin() + i0, snap.spectrum.begin() + i1);
            int dc = n / 2 - i0;  // hide the DC spike
            if (dc >= 1 && dc + 1 < static_cast<int>(view.size()))
                view[static_cast<size_t>(dc)] = 0.5f * (view[static_cast<size_t>(dc - 1)] + view[static_cast<size_t>(dc + 1)]);
            spectrum.series[0].y.assign(view.begin(), view.end());
            if (peakHold || averaging || alerts) {
                double plo, phi;
                passband(plo, phi);
                extras.frame(&view[0], static_cast<int>(view.size()), winLow, winHigh, pcNow(), plo, phi);
                if (averaging && extras.average().size() == view.size())
                    spectrum.series[0].y.assign(extras.average().begin(), extras.average().end());
                if (peakHold && extras.peak().size() == view.size())
                    spectrum.series[1].y.assign(extras.peak().begin(), extras.peak().end());
                const std::vector<dsp::SpectrumAlert>& fresh = extras.lastAlerts();
                if (alerts && !fresh.empty()) say("new signal at " + formatShort(fresh.back().hz) + fmt(" (%.0f dB)", fresh.back().level));
            }
            if (autoFloor && framesSinceStart > 10) {
                std::vector<float> sorted(view);
                std::nth_element(sorted.begin(), sorted.begin() + static_cast<long>(sorted.size() / 5), sorted.end());
                double p20 = sorted[sorted.size() / 5];
                wfFloor.value = wfFloor.value + 0.05 * ((p20 - 3) - wfFloor.value);
            }
            waterfall.low = wfFloor.value;
            waterfall.high = wfFloor.value + wfRange.value;
            waterfall.useThemeMap = mapIndex < 0;
            if (mapIndex >= 0) waterfall.map = maps[static_cast<size_t>(mapIndex)];
            if (wfHold.size() != view.size()) wfHold.assign(view.begin(), view.end());
            else
                for (size_t k = 0; k < view.size(); ++k) wfHold[k] = std::max(wfHold[k], view[k]);
            if (++wfPhase % 2 == 0) {
                if (wfRows.size() && wfRows.front().size() != wfHold.size()) wfRows.clear();
                waterfall.addRow(wfHold);
                wfRows.push_front(wfHold);
                if (wfRows.size() > 300) wfRows.pop_back();
                wfHold.clear();
            }
        }
        spectrum.xMin = winLow;
        spectrum.xMax = winHigh;
        spectrum.yMin = wfFloor.value - 10;
        spectrum.yMax = wfFloor.value + wfRange.value + 20;
        double lo, hi;
        passband(lo, hi);
        spectrum.bands.clear();
        Plot::BandArea pb = {lo, hi, lerp(t.background, t.accent, 0.22)};
        spectrum.bands.push_back(pb);
        spectrum.markers.clear();
        waterfall.markers.clear();
        // Personal labels in view (dim), then the decoder's tone, then the dial.
        for (size_t i = 0; i < store.labels.size(); ++i) {
            const app::Label& l = store.labels[i];
            if (l.hz < winLow || l.hz > winHigh) continue;
            Plot::Marker m = {l.hz, lerp(t.background, t.text, 0.55), l.text};
            spectrum.markers.push_back(m);
        }
        if (snap.decoderOn && s.mode == dsp::Mode::USB && s.decoderShift > 0) {
            if (!probe || probe->kind() != snap.decoderKind) probe = dec::makeDecoder(snap.decoderKind);
            double toneHz = snap.decoder.toneHz > 0 ? snap.decoder.toneHz : (probe ? probe->audioCenterHz() : s.decoderShift);
            Plot::Marker m = {s.frequency - s.decoderShift + toneHz, t.good, std::string()};
            spectrum.markers.push_back(m);
        }
        spectrum.series[1].color = lerp(t.background, t.warn, 0.7);
        // Watched signals (Signal Monitor) and recent new-signal alerts.
        for (int i = 0; i < radio.monitorCount(); ++i) {
            double mc = radio.monitorCentre(i), mw = radio.monitorWidth(i);
            if (mc + mw / 2 < winLow || mc - mw / 2 > winHigh) continue;
            Plot::BandArea ba = {mc - mw / 2, mc + mw / 2, lerp(t.background, t.good, 0.18)};
            spectrum.bands.push_back(ba);
        }
        if (alerts) {
            const std::vector<dsp::SpectrumAlert>& seen = extras.recentAlerts();
            for (size_t i = 0; i < seen.size(); ++i) {
                if (pcNow() - seen[i].t > 60 || seen[i].hz < winLow || seen[i].hz > winHigh) continue;
                Plot::Marker m = {seen[i].hz, t.warn, "new"};
                spectrum.markers.push_back(m);
            }
        }
        Plot::Marker dial = {s.frequency, t.accent, std::string()};
        spectrum.markers.push_back(dial);
        Plot::Marker wm = {(s.frequency - winLow) / (winHigh - winLow), t.accent, std::string()};
        waterfall.markers.push_back(wm);
    }

    // Decoder panel.
    if (decoderView.visible) {
        std::string text;
        radio.takeDecoderText(text);
        decoderView.addText(text);
        if (decoderView.serviceMode) {
            decoderView.services = radio.decoderServices();
            std::vector<dec::Decoder::EncodedImage> images;
            radio.takeDecoderImages(images);
            for (size_t i = 0; i < images.size(); ++i) {
                try {
                    decoderView.slide = loadImageFromMemory(&images[i].bytes[0], images[i].bytes.size());
                } catch (const std::exception&) {
                    // a broken slide: keep the previous one
                }
            }
        }
        if (decoderView.lineMode) {
            std::vector<dec::PictureLine> lines;
            radio.takeDecoderLines(lines);
            decoderView.addLines(lines);
        }
        if (decoderView.picture) {
            std::vector<std::vector<float> > cols;
            radio.takeDecoderColumns(cols);
            decoderView.addColumns(cols);
        }
        const dec::Status& d = snap.decoder;
        std::string title = d.mode.empty() ? std::string(dec::kindName(snap.decoderKind)) : d.mode;
        title += d.locked ? "  locked" : (d.signal ? "  signal" : "  searching");
        std::string info = d.info;
        if (!d.mode.empty() && info.compare(0, d.mode.size(), d.mode) == 0) info = info.substr(d.mode.size());  // no "PSK31 ... PSK31"
        while (!info.empty() && info[0] == ' ') info.erase(0, 1);
        if (!info.empty()) title += "  " + info;
        title += "   (d: other decoder, D: off)";
        decoderView.title = title;
    }

    // RDS line.
    if (s.mode == dsp::Mode::WFM && snap.rds.groups > 0) {
        std::string line = "RDS  " + (snap.rds.ps.empty() ? std::string("        ") : snap.rds.ps);
        if (snap.rds.pi) {
            char pi[16];
            std::snprintf(pi, sizeof(pi), "   PI %04X", static_cast<unsigned>(snap.rds.pi));
            line += pi;
        }
        if (!snap.rds.ptyName.empty()) line += "   " + snap.rds.ptyName;
        if (!snap.rds.radioText.empty()) line += "   \xE2\x80\x9C" + snap.rds.radioText + "\xE2\x80\x9D";
        if (!snap.rds.clock.empty()) line += "   " + snap.rds.clock;
        rdsLabel.text = line;
        rdsLabel.useColor = true;
        rdsLabel.color = t.accent;
        rdsLabel.attrs = Bold;
    } else {
        rdsLabel.text = s.mode == dsp::Mode::WFM ? "RDS  (waiting for data)" : std::string();
        rdsLabel.useColor = true;
        rdsLabel.color = t.dim;
        rdsLabel.attrs = Plain;
    }

    // Status bar.
    status.clear();
    status.addLeft(radio.sourceName(), t.accentText, t.accent);
    status.addLeft(fmt("%.3f MS/s", snap.sampleRate / 1e6));
    status.addLeft(fmt("DSP %3.0f%%", snap.dspLoad * 100));
    status.addLeft("audio " + radio.audioName() + fmt(" %3.0f ms", snap.audioQueued * 1000) +
                   (snap.audioUnderruns ? fmt(" gaps %.0f", static_cast<double>(snap.audioUnderruns)) : std::string()));
    if (snap.decoderBlocksDropped)
        status.addLeft(fmt("decoder behind %.0f", static_cast<double>(snap.decoderBlocksDropped)), t.accentText, t.warn);
    if (snap.samplesDropped) status.addLeft(fmt("dropped %.0f", static_cast<double>(snap.samplesDropped)), t.accentText, t.warn);
    if (radio.recording()) status.addLeft(fmt("REC %.0f s", radio.recordedSeconds()), t.accentText, t.bad);
    if (surveyNext > 0) status.addLeft("SURVEY " + formatShort(surveyDials[surveyIndex]), t.accentText, t.warn);
    if (radio.replaying()) status.addLeft(fmt("REPLAY %.1f s", radio.replayPosition()), t.accentText, t.warn);
    if (noticeFrames > 0) {
        --noticeFrames;
        status.addLeft(notice, t.accentText, t.good);
    }
    if (scanKind != ScanOff)
        status.addLeft(scanKind == ScanBand ? "scanning band (n stops)" : "scanning memories (N stops)", t.accentText, t.warn);
    status.addRight("F1 help  Enter freq  d decoder  M memory  b lists  q quit");
}

// --------------------------------------------------------------------- Ui --

Ui::Ui(Radio& radio, const Options& options, Store& store) : impl_(new Impl(radio, options, store)) { impl_->build(); }
Ui::~Ui() {}

int Ui::run() {
    {
        std::ifstream in((Store::directory() + "/clock.txt").c_str());
        std::stringstream text;
        text << in.rdbuf();
        if (in) impl_->clock.load(text.str());
    }
    terminal::Session session;
    if (impl_->options.mouse) session.enableMouse(true);
    impl_->app.run(session, 30, [this]() { impl_->update(); });
    if (impl_->decoderView.lineMode && !impl_->decoderView.rows.empty())  // a picture still on screen
        impl_->savePicture(impl_->decoderView.pictureImage(), dec::kindName(impl_->snap.decoderKind));
    impl_->saveState();
    {
        std::ofstream out((Store::directory() + "/clock.txt").c_str());
        out << impl_->clock.save();
    }
    return 0;
}

}  // namespace app
