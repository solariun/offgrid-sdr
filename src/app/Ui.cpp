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
//  status line).
//
//  The spectrum window
//  -------------------
//  The FFT covers the whole span (sample rate) around the stick's centre.
//  With zoom z the plot and waterfall show span / z, centred near the dial;
//  the window moves in quarter-window jumps (not with every tuning step), so
//  the waterfall history stays aligned until a jump clears it.
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

#include <graphicx/terminal.h>
#include <graphicx/widgets.h>

#include "Bands.h"

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

// Frequency entry popup: a framed TextInput.
class FrequencyPrompt : public Widget {
public:
    FrequencyPrompt() {
        input.label = "MHz / kHz / Hz:";
        input.placeholder = "e.g. 100.3  7074k  145.8M";
        input.maxLength = 20;
        input.accept = [](std::uint32_t c) {
            return (c >= '0' && c <= '9') || c == '.' || c == 'k' || c == 'K' || c == 'm' || c == 'M' || c == 'g' || c == 'G' ||
                   c == 'h' || c == 'H' || c == 'z' || c == 'Z' || c == ' ';
        };
    }
    TextInput input;
    std::string error;
    bool focusable() const { return true; }
    void draw(Painter& p, const Theme& t, bool) {
        p.panel(p.bounds(), t.focus, t.panel, Border::Double, " Frequency ");
        Painter in = p.sub(rect(2, 2, p.width() - 4, 1));
        input.draw(in, t, true);
        if (!error.empty()) p.text(2, 3, error, t.bad);
        else p.text(2, 3, "Enter tunes, Esc cancels", t.dim);
    }
    bool handle(const Event& e) {
        if (e.type == terminal::EventType::Mouse) return true;
        return input.handle(e);
    }
};

}  // namespace

// ==================================================================== Impl ==

struct Ui::Impl {
    Impl(Radio& r, const Options& o);

    Radio& radio;
    Options options;
    App app;
    terminal::Session* session;

    // header
    BigText freqText;
    Label unitLabel, infoLabel, clockLabel;
    RadioGroup modes;
    Led ledStereo, ledRds, ledSquelch, ledOverload, ledRecord, ledMute;
    Meter meter;
    Sparkline levelHistory;
    // main
    Plot spectrum;
    Waterfall waterfall;
    Label rdsLabel;
    // side
    Panel bandsPanel, controlsPanel;
    ListBox bandList;
    Slider volume, squelch, gain, bandwidth, zoom, wfFloor, wfRange;
    Toggle autoGain, stereo, biasTee;
    StatusBar status;
    // popups
    FrequencyPrompt prompt;
    Dialog help, message;

    // state
    int stepIndex;
    std::vector<double> gainSteps;
    std::vector<std::string> themes;
    size_t themeIndex;
    std::vector<Colormap> maps;
    int mapIndex;  // -1 = the theme's
    double winLow, winHigh;  // Hz shown by the spectrum / waterfall
    unsigned lastSerial;
    bool autoFloor;
    int framesSinceStart;
    Snapshot snap;
    std::vector<double> bandMemory;
    std::vector<float> wfHold;  // peak of the frames since the last waterfall row
    unsigned wfPhase;  // last frequency used in each band

    void build();
    void layout(const Rect& screen);
    void bindKeys();
    void update();
    void tune(double hz, bool snap = true);
    void setMode(dsp::Mode m);
    void syncControls();
    void showMessage(const std::string& title, const std::string& text);
    void toggleRecording();
    double step() const { return kSteps[stepIndex]; }
    void passband(double& lo, double& hi) const;
};

Ui::Impl::Impl(Radio& r, const Options& o)
    : radio(r),
      options(o),
      app(theme(o.theme)),
      session(0),
      ledStereo("ST"),
      ledRds("RDS"),
      ledSquelch("SQL"),
      ledOverload("OVL"),
      ledRecord("REC"),
      ledMute("MUTE"),
      bandsPanel("Bands"),
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
      bandMemory(bands().size(), 0),
      wfPhase(0) {
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
    // A step that suits the band we start in.
    const Band* b = bandAt(radio.settings().frequency);
    if (b)
        for (int i = 0; i < kStepCount; ++i)
            if (std::fabs(kSteps[i] - b->step) < 1) stepIndex = i;
}

void Ui::Impl::passband(double& lo, double& hi) const {
    RadioSettings s = radio.settings();
    double bw = s.bandwidth > 0 ? s.bandwidth : dsp::defaultBandwidth(s.mode);
    double f = s.frequency;
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
    // Remember it for its band.
    const Band* b = bandAt(radio.settings().frequency);
    if (b) bandMemory[static_cast<size_t>(b - &bands()[0])] = radio.settings().frequency;
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
    stereo.checked = s.stereo;
    stereo.enabled = s.mode == dsp::Mode::WFM;
    biasTee.checked = s.biasTee;
}

void Ui::Impl::showMessage(const std::string& title, const std::string& text) {
    message.title = title;
    message.message = text;
    message.buttons.assign(1, "OK");
    message.place(rect(0, 0, app.surface().width(), app.surface().height()), 60);
    app.openPopup(message, true);
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

// ------------------------------------------------------------------- build --

void Ui::Impl::build() {
    freqText.font = Font::Font3x5;
    freqText.mode = PixelMode::Quadrants;
    unitLabel.text = "MHz";
    for (int i = 0; i < kModeCount; ++i) modes.options.push_back(dsp::modeName(kModes[i]));
    modes.onChange = [this](int i) { setMode(kModes[i]); };

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
    spectrum.onMouse = [this](double x, double, const Event& e) {
        using terminal::MouseAction;
        using terminal::MouseButton;
        if (e.button == MouseButton::WheelUp) tune(radio.settings().frequency + step());
        else if (e.button == MouseButton::WheelDown) tune(radio.settings().frequency - step());
        else if (e.button == MouseButton::Left && (e.action == MouseAction::Press || e.action == MouseAction::Drag)) tune(x);
    };
    waterfall.onMouse = [this](double f, const Event& e) {
        using terminal::MouseAction;
        using terminal::MouseButton;
        if (e.button == MouseButton::WheelUp) tune(radio.settings().frequency + step());
        else if (e.button == MouseButton::WheelDown) tune(radio.settings().frequency - step());
        else if (e.button == MouseButton::Left && (e.action == MouseAction::Press || e.action == MouseAction::Drag))
            tune(winLow + f * (winHigh - winLow));
    };

    bandList.border = false;
    const std::vector<Band>& b = bands();
    for (size_t i = 0; i < b.size(); ++i) {
        ListBox::Item it = {b[i].name, b[i].group.substr(0, 5), false, rgb(0, 0, 0)};
        bandList.items.push_back(it);
    }
    bandList.selected = 0;
    bandList.onActivate = [this](int i) {
        const Band& band = bands()[static_cast<size_t>(i)];
        double f = bandMemory[static_cast<size_t>(i)] > 0 ? bandMemory[static_cast<size_t>(i)] : band.start;
        radio.setMode(band.mode);
        for (int k = 0; k < kStepCount; ++k)
            if (std::fabs(kSteps[k] - band.step) < 1) stepIndex = k;
        tune(f, false);
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
        double hz;
        if (!parseFrequency(text, hz)) {
            prompt.error = "not a frequency: " + text;
            return;
        }
        app.closePopup(prompt);
        const Band* band = bandAt(hz);
        if (band && band->mode != radio.settings().mode) {
            // Typing a broadcast-FM frequency while in SSB etc.: switch.
            if (band->name == "FM broadcast" || band->name == "Airband") setMode(band->mode);
        }
        tune(hz, false);
    };
    prompt.input.onCancel = [this]() { app.closePopup(prompt); };

    help.title = "Keys";
    help.buttons.assign(1, "Close");
    help.message =
        "Left/Right  tune one step (Shift: 10 steps)      Up/Down  change the step\n"
        "PgUp/PgDn   tune 1 MHz                           Enter, f type a frequency\n"
        "1-7         WFM NFM AM SAM USB LSB CW            m  next mode\n"
        "b           band list (Enter picks)              Tab / Esc  side controls / back\n"
        "+ / -       volume                               space  mute\n"
        "g / G       RF gain down / up                    a  automatic gain\n"
        "s / S       squelch down / up                    w / W  filter narrower / wider\n"
        "z / Z       zoom in / out                        [ ]  waterfall floor   { }  range\n"
        "t           next theme                           c  next waterfall colors\n"
        "R           record the sound to a WAV file\n"
        "F1, h, ?    this help                            q  quit\n"
        "\n"
        "Mouse: click or drag on the spectrum or waterfall to tune, the wheel steps.\n"
        "Every control on the right works with the mouse too.";

    app.add(freqText);
    app.add(unitLabel);
    app.add(infoLabel);
    app.add(clockLabel);
    app.add(modes);
    app.add(ledStereo);
    app.add(ledRds);
    app.add(ledSquelch);
    app.add(ledOverload);
    app.add(ledRecord);
    app.add(ledMute);
    app.add(meter);
    app.add(levelHistory);
    app.add(spectrum);
    app.add(waterfall);
    app.add(rdsLabel);
    app.add(bandsPanel);
    app.add(bandList);
    app.add(controlsPanel);
    app.add(volume);
    app.add(squelch);
    app.add(gain);
    app.add(autoGain);
    app.add(bandwidth);
    app.add(zoom);
    app.add(wfFloor);
    app.add(wfRange);
    app.add(stereo);
    app.add(biasTee);
    app.add(status);
    app.onLayout = [this](const Rect& screen) { layout(screen); };
    app.onDrawBackground = [this](Painter& p) {
        // Header background band.
        p.fill(rect(0, 0, p.width(), 4), lerp(app.theme().background, app.theme().panel, 0.8));
    };
    syncControls();
    bindKeys();
}

// ------------------------------------------------------------------ layout --

void Ui::Impl::layout(const Rect& screen) {
    int w = screen.width, h = screen.height;
    int side = w >= 110 ? 34 : (w >= 90 ? 28 : 0);
    // Header: rows 0-3.
    int fw, fh;
    freqText.text = formatFrequency(radio.settings().frequency);
    freqText.measure(fw, fh);
    freqText.bounds = rect(1, 0, fw, 3);
    unitLabel.bounds = rect(fw + 2, 2, 4, 1);
    modes.bounds = rect(fw + 7, 0, 46, 1);
    infoLabel.bounds = rect(fw + 7, 1, std::max(10, w - fw - 7 - 46), 1);
    int lx = fw + 7;
    ledStereo.bounds = rect(lx, 2, 5, 1);
    ledRds.bounds = rect(lx + 6, 2, 6, 1);
    ledSquelch.bounds = rect(lx + 13, 2, 6, 1);
    ledOverload.bounds = rect(lx + 20, 2, 6, 1);
    ledRecord.bounds = rect(lx + 27, 2, 6, 1);
    ledMute.bounds = rect(lx + 34, 2, 7, 1);
    int meterX = std::max(lx + 42, w - 46);
    meter.bounds = rect(meterX, 0, w - meterX - 1, 2);
    levelHistory.bounds = rect(meterX + 2, 2, w - meterX - 16, 1);
    clockLabel.bounds = rect(w - 13, 2, 12, 1);
    // Bottom: RDS row and status.
    status.bounds = rect(0, h - 1, w, 1);
    rdsLabel.bounds = rect(1, h - 2, w - 2, 1);
    // Main area.
    Rect main = rect(0, 4, w - side - (side ? 1 : 0), h - 4 - 2);
    int specH = std::max(4, main.height * 2 / 5);
    spectrum.bounds = rect(main.x, main.y, main.width, specH);
    Rect pa = spectrum.plotArea();
    waterfall.bounds = rect(main.x + pa.x, main.y + specH, std::max(1, main.width - pa.x), main.height - specH);
    // Side panels.
    bool showSide = side > 0;
    Widget* sideWidgets[] = {&bandsPanel, &bandList, &controlsPanel, &volume, &squelch, &gain, &autoGain, &bandwidth,
                             &zoom, &wfFloor, &wfRange, &stereo, &biasTee};
    for (size_t i = 0; i < sizeof(sideWidgets) / sizeof(sideWidgets[0]); ++i) sideWidgets[i]->visible = showSide;
    if (showSide) {
        int sx = w - side;
        int controlsH = std::min(15, main.height - 6);
        int bandsH = main.height - controlsH;
        bandsPanel.bounds = rect(sx, main.y, side, bandsH);
        bandList.bounds = inset(bandsPanel.bounds, 1);
        controlsPanel.bounds = rect(sx, main.y + bandsH, side, controlsH);
        Rect in = inset(controlsPanel.bounds, 1);
        in.x += 1;
        in.width -= 2;
        Widget* rows[] = {&volume, &squelch, &gain, &autoGain, &bandwidth, &zoom, &wfFloor, &wfRange, &stereo, &biasTee};
        Slider* sliders[] = {&volume, &squelch, &gain, &bandwidth, &zoom, &wfFloor, &wfRange};
        for (size_t i = 0; i < 7; ++i) sliders[i]->labelWidth = 9;
        int y = in.y;
        for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); ++i) {
            if (rows[i] == &stereo) {
                stereo.bounds = rect(in.x, y, in.width / 2, 1);
                biasTee.bounds = rect(in.x + in.width / 2, y, in.width - in.width / 2, 1);
                ++y;
                break;
            }
            rows[i]->bounds = y < in.bottom() ? rect(in.x, y, in.width, 1) : rect(0, 0, 0, 0);
            ++y;
        }
    }
    waterfall.clear();
}

// -------------------------------------------------------------------- keys --

void Ui::Impl::bindKeys() {
    using namespace terminal;
    app.bindKey(KeyRight, [this]() { tune(radio.settings().frequency + step()); });
    app.bindKey(KeyLeft, [this]() { tune(radio.settings().frequency - step()); });
    app.bindKey(KeyUp, [this]() { stepIndex = std::min(kStepCount - 1, stepIndex + 1); });
    app.bindKey(KeyDown, [this]() { stepIndex = std::max(0, stepIndex - 1); });
    app.bindKey(KeyPageUp, [this]() { tune(radio.settings().frequency + 1e6, false); });
    app.bindKey(KeyPageDown, [this]() { tune(radio.settings().frequency - 1e6, false); });
    std::function<void()> askFrequency = [this]() {
        prompt.error.clear();
        prompt.input.setText(std::string());
        int pw = std::min(52, app.surface().width() - 2);
        prompt.bounds = centered(rect(0, 0, app.surface().width(), app.surface().height()), pw, 5);
        app.openPopup(prompt, true);
    };
    app.bindKey(KeyEnter, askFrequency);
    app.bindKey('f', askFrequency);
    for (int i = 0; i < kModeCount; ++i) {
        dsp::Mode m = kModes[i];
        app.bindKey('1' + i, [this, m]() { setMode(m); });
    }
    app.bindKey('m', [this]() { setMode(kModes[(modeIndex(radio.settings().mode) + 1) % kModeCount]); });
    app.bindKey('b', [this]() {
        if (bandList.visible) app.focus(bandList);
    });
    app.bindKey(KeyTab, [this]() { app.focusNext(false); });
    app.bindKey(KeyEscape, [this]() { app.clearFocus(); });
    app.bindKey('+', [this]() {
        volume.set(volume.value + 5);
    });
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
    std::function<void()> showHelp = [this]() {
        help.place(rect(0, 0, app.surface().width(), app.surface().height()), std::min(86, app.surface().width() - 2));
        help.bounds.height = std::min(app.surface().height() - 2, 20);
        app.openPopup(help, true);
    };
    app.bindKey(KeyF1, showHelp);
    app.bindKey('h', showHelp);
    app.bindKey('?', showHelp);
    app.bindKey('q', [this]() { app.quit(); });
    app.bindKey('Q', [this]() { app.quit(); });
    // Shift+arrows: 10 steps.
    app.onUnhandled = [this](const Event& e) {
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

    // Header.
    std::string ft = formatFrequency(s.frequency);
    if (ft != freqText.text) {
        freqText.text = ft;
        app.relayout();
    }
    const Band* band = bandAt(s.frequency);
    char info[160];
    std::snprintf(info, sizeof(info), "%s   step %s   filter %s   %s", band ? band->name.c_str() : "-", stepName(step()).c_str(),
                  formatShort(s.bandwidth > 0 ? s.bandwidth : dsp::defaultBandwidth(s.mode)).c_str(),
                  app.focused() ? "[Esc: keys back to tuning]" : "");
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
    meter.update(snap.channelDbfs, 0.4);
    meter.valueText = fmt("%5.1f dB", snap.channelDbfs) + fmt("  SNR %4.1f", snap.channelDbfs - snap.noiseDbfs);
    if (framesSinceStart % 3 == 0) levelHistory.push(snap.channelDbfs, 512);
    std::time_t now = std::time(0);
    char clock[16];
    std::strftime(clock, sizeof(clock), "%H:%M:%S UTC", std::gmtime(&now));
    clockLabel.text = clock;
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
            // Hide the DC spike at the stick's centre (one or two bins).
            int dc = n / 2 - i0;
            if (dc >= 1 && dc + 1 < static_cast<int>(view.size())) {
                float fill = 0.5f * (view[static_cast<size_t>(dc - 1)] + view[static_cast<size_t>(dc + 1)]);
                view[static_cast<size_t>(dc)] = fill;
            }
            spectrum.series[0].y.assign(view.begin(), view.end());
            if (autoFloor && framesSinceStart > 10) {
                // Floor: a low percentile of the view, followed slowly.
                std::vector<float> sorted(view);
                std::nth_element(sorted.begin(), sorted.begin() + static_cast<long>(sorted.size() / 5), sorted.end());
                double p20 = sorted[sorted.size() / 5];
                wfFloor.value = wfFloor.value + 0.05 * ((p20 - 3) - wfFloor.value);
            }
            waterfall.low = wfFloor.value;
            waterfall.high = wfFloor.value + wfRange.value;
            waterfall.useThemeMap = mapIndex < 0;
            if (mapIndex >= 0) waterfall.map = maps[static_cast<size_t>(mapIndex)];
            // 15 rows a second (every second frame), keeping the peak of the
            // two frames so short signals still show.
            if (wfHold.size() != view.size()) wfHold.assign(view.begin(), view.end());
            else
                for (size_t k = 0; k < view.size(); ++k) wfHold[k] = std::max(wfHold[k], view[k]);
            if (++wfPhase % 2 == 0) {
                waterfall.addRow(wfHold);
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
        Plot::Marker dial = {s.frequency, t.accent, std::string()};
        spectrum.markers.push_back(dial);
        waterfall.markers.clear();
        Plot::Marker wm = {(s.frequency - winLow) / (winHigh - winLow), t.accent, std::string()};
        waterfall.markers.push_back(wm);
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
    if (snap.samplesDropped) status.addLeft(fmt("dropped %.0f", static_cast<double>(snap.samplesDropped)), t.accentText, t.warn);
    if (radio.recording()) status.addLeft(fmt("REC %.0f s", radio.recordedSeconds()), t.accentText, t.bad);
    status.addRight("F1 help  Enter freq  1-7 mode  b bands  q quit");
}

// --------------------------------------------------------------------- Ui --

Ui::Ui(Radio& radio, const Options& options) : impl_(new Impl(radio, options)) { impl_->build(); }
Ui::~Ui() {}

int Ui::run() {
    terminal::Session session;
    if (impl_->options.mouse) session.enableMouse(true);
    impl_->session = &session;
    impl_->app.run(session, 30, [this]() { impl_->update(); });
    impl_->session = 0;
    return 0;
}

}  // namespace app
