// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/PictureEncoders.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  Fax and SSTV audio synthesis and the test pictures: see
//  PictureEncoders.h.  Ports of OFFgrid-SDR's JS demo fax station (its
//  faxPixel() segments) and sstvTones() (diddy-boy,
//  https://github.com/solariun/offgrid-sdr).  The JS demo looks the pixel
//  up every 64 samples and mixes it onto an RF carrier; here every sample
//  is computed, directly as audio (Addition to the JS: exact timing).
//
//  How it fits in
//  --------------
//  Implements PictureEncoders.h.  Standard library only.
//
//  Contributing
//  ------------
//  Deterministic output only.  Strict C++11.
// ============================================================================
#include "PictureEncoders.h"

#include <cmath>

namespace dec {

namespace {

const double kTwoPi = 6.283185307179586;

// A phase-continuous oscillator.
struct Osc {
    Osc() : ph(0) {}
    double ph;
    float next(double hz, double rate, double amp) {
        const float s = static_cast<float>(amp * std::sin(ph));
        ph += kTwoPi * hz / rate;
        if (ph > kTwoPi) ph -= kTwoPi;
        return s;
    }
};

double clamp01(double v) { return v < 0 ? 0 : v > 1 ? 1 : v; }

}  // namespace

// ===========================================================================
// Fax: the JS demo's segments: start, phase, image, stop, black.
// ===========================================================================
std::size_t synthFax(const std::vector<unsigned char>& grey, int width, int height, double rate,
                     std::vector<float>& out, const FaxTransmission& tx) {
    if (width <= 0 || height <= 0 || grey.size() < static_cast<std::size_t>(width) * height || !(rate > 0) ||
        !(tx.lpm > 0))
        return 0;
    const double line = 60.0 / tx.lpm;   // seconds per line
    const double tStart = tx.startSec, tPhase = tStart + tx.phasingLines * line, tImage = tPhase + height * line,
                 tStop = tImage + tx.stopSec, tEnd = tStop + tx.blackSec;
    const std::size_t n = static_cast<std::size_t>(std::floor(tEnd * rate));
    const std::size_t first = out.size();
    out.reserve(first + n);
    Osc o;
    for (std::size_t i = 0; i < n; ++i) {
        const double t = i / rate;
        double v;
        if (t < tStart) v = static_cast<long>(std::floor(t * 600)) % 2;              // 300 Hz black/white
        else if (t < tPhase) {
            const double u = std::fmod(t - tStart, line) / line;
            v = u < 0.05 ? 1 : 0;                                                       // the line-start pulse
        } else if (t < tImage) {
            const double u = t - tPhase;
            int y = static_cast<int>(std::floor(u / line));
            if (y >= height) y = height - 1;
            int x = static_cast<int>(std::floor(std::fmod(u, line) / line * width));
            if (x >= width) x = width - 1;
            if (x < 0) x = 0;
            v = grey[static_cast<std::size_t>(y) * width + x] / 255.0;
        } else if (t < tStop) v = static_cast<long>(std::floor((t - tImage) * 900)) % 2;   // 450 Hz
        else v = 0;                                                                     // black
        out.push_back(o.next(1900 + 400 * (2 * clamp01(v) - 1), rate, tx.amplitude));
    }
    return n;
}

// ===========================================================================
// SSTV
// ===========================================================================
void sstvTones(const SstvMode& m, const std::vector<unsigned char>& rgb, std::vector<SstvTone>& seq) {
    const SstvLayout L = sstvLayout(m);
    const SstvTone head[4] = {{1900, 300}, {1200, 10}, {1900, 300}, {1200, 30}};
    for (int i = 0; i < 4; ++i) seq.push_back(head[i]);
    int ones = 0;
    for (int b = 0; b < 7; ++b) {
        const int x = (m.vis >> b) & 1;
        ones += x;
        seq.push_back(SstvTone{x ? 1100.0 : 1300.0, 30});
    }
    seq.push_back(SstvTone{ones % 2 ? 1100.0 : 1300.0, 30});
    seq.push_back(SstvTone{1200, 30});
    if (m.family == SstvFamily::Scottie) seq.push_back(SstvTone{1200, 9});   // Scottie's starting sync
    const int w = m.width;
    const int rows = sstvLinesPerPicture(m);
    for (int ln = 0; ln < rows; ++ln) {
        double t = 0;
        for (int p = 0; p < L.nParts; ++p) {
            const SstvPart& part = L.parts[p];
            if (part.offMs > t) { seq.push_back(SstvTone{1500, part.offMs - t}); t = part.offMs; }   // porches / separators
            if (part.what == kSstvSync) { seq.push_back(SstvTone{1200, part.lenMs}); t += part.lenMs; continue; }
            const double dt = part.lenMs / w;
            for (int x = 0; x < w; ++x) {
                double v;
                if (m.family == SstvFamily::Pd) {
                    const int y0 = 2 * ln, y = part.what == kSstvY1 ? y0 + 1 : y0;
                    const unsigned char* a = &rgb[(static_cast<std::size_t>(y0) * w + x) * 3];
                    const unsigned char* b = &rgb[(static_cast<std::size_t>(y0 + 1) * w + x) * 3];
                    const unsigned char* c = &rgb[(static_cast<std::size_t>(y) * w + x) * 3];
                    const double R = c[0], G = c[1], B = c[2];
                    const double Ra = (a[0] + b[0]) / 2.0, Ga = (a[1] + b[1]) / 2.0, Ba = (a[2] + b[2]) / 2.0;
                    if (part.what == kSstvY0 || part.what == kSstvY1) v = 16 + (65.738 * R + 129.057 * G + 25.064 * B) / 256;
                    else if (part.what == kSstvCr) v = 128 + (112.439 * Ra - 94.154 * Ga - 18.285 * Ba) / 256;
                    else v = 128 + (-37.945 * Ra - 74.494 * Ga + 112.439 * Ba) / 256;
                } else {
                    const int k = part.what == kSstvR ? 0 : part.what == kSstvG ? 1 : 2;
                    v = rgb[(static_cast<std::size_t>(ln) * w + x) * 3 + k];
                }
                seq.push_back(SstvTone{sstvFreq(v), dt});
            }
            t += part.lenMs;
        }
        if (L.lenMs > t) seq.push_back(SstvTone{1500, L.lenMs - t});
    }
}

std::size_t synthTones(const std::vector<SstvTone>& seq, double rate, std::vector<float>& out, double amplitude) {
    if (seq.empty() || !(rate > 0)) return 0;
    double total = 0;
    for (std::size_t k = 0; k < seq.size(); ++k) total += seq[k].ms;
    const std::size_t n = static_cast<std::size_t>(std::floor(total / 1000 * rate));
    out.reserve(out.size() + n);
    Osc o;
    std::size_t k = 0;
    double end = seq[0].ms;               // end of tone k, ms
    for (std::size_t i = 0; i < n; ++i) {
        const double t = i * 1000.0 / rate;
        while (t >= end && k + 1 < seq.size()) end += seq[++k].ms;
        out.push_back(o.next(seq[k].hz, rate, amplitude));
    }
    return n;
}

std::size_t synthSstv(int vis, const std::vector<unsigned char>& rgb, double rate, std::vector<float>& out,
                      double amplitude) {
    const SstvMode* m = sstvModeByVis(vis);
    if (!m || rgb.size() < static_cast<std::size_t>(m->width) * m->height * 3) return 0;
    std::vector<SstvTone> seq;
    seq.reserve(static_cast<std::size_t>(sstvLinesPerPicture(*m)) * (4 * m->width + 8) + 16);
    sstvTones(*m, rgb, seq);
    return synthTones(seq, rate, out, amplitude);
}

// ===========================================================================
// Test pictures
// ===========================================================================
void testCardRgb(int w, int h, std::vector<unsigned char>& rgb) {
    rgb.assign(static_cast<std::size_t>(w > 0 ? w : 0) * (h > 0 ? h : 0) * 3, 0);
    static const unsigned char bars[8][3] = {{255, 255, 255}, {255, 255, 0}, {0, 255, 255}, {0, 255, 0},
                                             {255, 0, 255},   {255, 0, 0},   {0, 0, 255},   {0, 0, 0}};
    const int yBars = h * 45 / 100, yRamp = h * 60 / 100;
    const double cx = w * 0.5, cy = (yRamp + h) * 0.5, rDisc = (h - yRamp) * 0.28, rRing = (h - yRamp) * 0.42;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            unsigned char* p = &rgb[(static_cast<std::size_t>(y) * w + x) * 3];
            if (y < yBars) {
                const int b = x * 8 / w;
                p[0] = bars[b][0]; p[1] = bars[b][1]; p[2] = bars[b][2];
            } else if (y < yRamp) {
                const unsigned char g = static_cast<unsigned char>(w > 1 ? 255 * x / (w - 1) : 0);
                p[0] = p[1] = p[2] = g;
            } else {
                const double d = std::sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
                if (d < rDisc) { p[0] = 255; p[1] = 176; p[2] = 0; }                     // orange disc
                else if (std::fabs(d - rRing) < 2.5) { p[0] = p[1] = p[2] = 255; }      // white ring
                else { p[0] = 28; p[1] = 63; p[2] = 120; }                              // dark blue field
            }
        }
}

void testChartGrey(int w, int h, std::vector<unsigned char>& grey) {
    grey.assign(static_cast<std::size_t>(w > 0 ? w : 0) * (h > 0 ? h : 0), 255);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            unsigned char& p = grey[static_cast<std::size_t>(y) * w + x];
            // a frame and a grid (as the JS demo chart)
            if (x < 4 || x >= w - 4 || y < 2 || y >= h - 2) { p = 0; continue; }
            if (x % 150 < 3 || y % 55 < 2) p = 0;
            // a grey ramp band across the chart
            if (y >= h * 40 / 100 && y < h * 52 / 100) p = static_cast<unsigned char>(255 * x / (w - 1));
            // solid blocks: black, mid grey, dark grey
            if (y >= h * 60 / 100 && y < h * 80 / 100) {
                if (x >= w * 10 / 100 && x < w * 20 / 100) p = 0;
                if (x >= w * 30 / 100 && x < w * 40 / 100) p = 128;
                if (x >= w * 50 / 100 && x < w * 60 / 100) p = 64;
            }
            // wavy "isobars" and rings around two centres
            for (int k = 0; k < 3; ++k) {
                const double yy = h * (0.15 + 0.08 * k) + h * 0.05 * std::sin(x / 190.0 + k * 0.9);
                if (std::fabs(y - yy) < 1.6) p = 0;
            }
            const double dx = (x - w * 0.75) / 1.6, dy = y - h * 0.85;
            const double d = std::sqrt(dx * dx + dy * dy);
            for (int k = 1; k <= 3; ++k)
                if (std::fabs(d - h * 0.035 * k) < 1.2) p = 0;
        }
}

}  // namespace dec
