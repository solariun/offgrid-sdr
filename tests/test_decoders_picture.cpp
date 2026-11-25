// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_decoders_picture.cpp
//  Layer:  Tests (Library liboffgrid-decoders)
//
//  What this file does
//  -------------------
//  Self-checking test of the FAX and SSTV decoders against the encoders in
//  PictureEncoders.h.  Prints PASS/FAIL per check, returns nonzero on
//  failure.
//    - FAX 120 LPM at 48000 and 12000 Hz, white noise (SNR 20 dB in 3 kHz),
//      sender clock 0 / +100 / -100 ppm off: one picture, 1809 wide, the
//      right number of rows, pixel error small with no shift (start and
//      line-start alignment), slant measured from the phasing;
//    - FAX 240 and 60 LPM; manual start (startNow) without start tone and
//      phasing; two charts in a row are pictures 1 and 2;
//    - "aligned": the best fit over row shifts -2..2 and column shifts
//      -12..12 (fax) / -6..6 (SSTV) is no row shift and at most 2 px (fax,
//      0.1 % of a line) / 1 px (SSTV) column shift: the picture starts on
//      the right row and the line start is found;
//    - every SSTV format at 48000 Hz (+100 ppm) and 12000 Hz (-100 ppm),
//      same noise: the format named, picture size and every row there, pixel
//      error small; three pictures in a row get numbers 1, 2, 3;
//    - no picture from noise alone (both decoders);
//    - NaN/Inf samples do not poison either decoder;
//    - CPU: seconds of audio per CPU second.
//
//  Pixel error: the mean absolute difference per colour channel, 0..255,
//  over the rows both pictures have.  The limits (see kFaxMae, kSstvMae)
//  come from what the method itself loses, not the noise: the decoders'
//  low-pass filters (1000 Hz fax, 900 Hz SSTV) blur each sharp edge over a
//  few pixels, so a picture of sharp-edged blocks loses a few levels on
//  average even with no noise; 20 dB adds 1-3 levels.  A misaligned
//  picture (a line start off by 1 % or a row off) scores 20-60.
//
//  How it fits in
//  --------------
//  Build (from the repository root):
//    c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors -Isrc src/dsp/Fft.cpp \
//        src/decoders/Fax.cpp src/decoders/Sstv.cpp \
//        src/decoders/PictureEncoders.cpp tests/test_decoders_picture.cpp \
//        -o test_decoders_picture
//
//  Contributing
//  ------------
//  Keep it deterministic (own noise generator, fixed seeds).  Algorithms
//  under test are ports from OFFgrid-SDR (diddy-boy).
// ============================================================================
#include "decoders/Fax.h"
#include "decoders/PictureEncoders.h"
#include "decoders/Sstv.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace {

int g_failures = 0, g_checks = 0;

void check(bool ok, const std::string& name, const std::string& detail = "") {
    ++g_checks;
    if (!ok) ++g_failures;
    std::printf("%s  %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : "  -- ", detail.c_str());
    std::fflush(stdout);
}

// Deterministic Gaussian noise (xorshift64 + Box-Muller): same on every platform.
class Noise {
public:
    explicit Noise(unsigned long long seed) : s_(seed ? seed : 1), have_(false), spare_(0) {}
    double uniform() {
        s_ ^= s_ << 13; s_ ^= s_ >> 7; s_ ^= s_ << 17;
        return ((s_ >> 11) + 0.5) / 9007199254740992.0;
    }
    double gauss() {
        if (have_) { have_ = false; return spare_; }
        const double u = uniform(), v = uniform(), r = std::sqrt(-2 * std::log(u));
        spare_ = r * std::sin(6.283185307179586 * v);
        have_ = true;
        return r * std::cos(6.283185307179586 * v);
    }
private:
    unsigned long long s_;
    bool have_;
    double spare_;
};

const double kAmp = 0.3;          // tone amplitude
#ifndef PIC_SNR_DB
#define PIC_SNR_DB 20             // (build with -DPIC_SNR_DB=80 to see the noise-free baseline)
#endif
const double kSnrDb = PIC_SNR_DB; // in 3 kHz
const double kFaxMae = 16;        // pixel error limits, levels of 255 (see the header)
const double kSstvMae = 14;

// Noise standard deviation for a tone of amplitude A at snrDb in 3 kHz.
double noiseSigma(double A, double snrDb, double rate) {
    const double sig = A * A / 2, n3k = sig / std::pow(10.0, snrDb / 10);
    return std::sqrt(n3k * (rate / 2) / 3000);
}

// lead s of silence, the signal, tail s of silence; plus noise.
std::vector<float> withNoise(const std::vector<float>& sig, double rate, double lead, double tail, double sigma,
                             unsigned long long seed) {
    const std::size_t nl = static_cast<std::size_t>(lead * rate), nt = static_cast<std::size_t>(tail * rate);
    std::vector<float> a(nl + sig.size() + nt, 0.0f);
    for (std::size_t i = 0; i < sig.size(); ++i) a[nl + i] = sig[i];
    Noise g(seed);
    for (std::size_t i = 0; i < a.size(); ++i) a[i] += static_cast<float>(sigma * g.gauss());
    return a;
}

// Feeds in blocks of uneven size (as a sound card would), collecting lines.
void feed(dec::Decoder& d, const std::vector<float>& a, double rate, std::vector<dec::PictureLine>& lines) {
    static const std::size_t sizes[] = {1024, 333, 4096, 7, 2048, 960};
    std::size_t i = 0, k = 0;
    while (i < a.size()) {
        std::size_t n = sizes[k++ % 6];
        if (i + n > a.size()) n = a.size() - i;
        d.process(&a[i], n, rate);
        d.takeLines(lines);
        i += n;
    }
}

// The rows of picture `pic` (row -> rgb), and how many distinct pictures.
struct Got {
    std::map<int, std::vector<unsigned char> > rows;
    int width;
    bool ordered;      // rows arrived 0, 1, 2, ... without gaps or repeats
};
Got picture(const std::vector<dec::PictureLine>& lines, unsigned pic) {
    Got g;
    g.width = 0;
    g.ordered = true;
    int next = 0;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (lines[i].picture != pic) continue;
        if (lines[i].row != next) g.ordered = false;
        next = lines[i].row + 1;
        g.rows[lines[i].row] = lines[i].rgb;
        g.width = static_cast<int>(lines[i].rgb.size() / 3);
    }
    return g;
}
std::vector<unsigned> pictureNumbers(const std::vector<dec::PictureLine>& lines) {
    std::vector<unsigned> v;
    for (std::size_t i = 0; i < lines.size(); ++i)
        if (v.empty() || v.back() != lines[i].picture) v.push_back(lines[i].picture);
    return v;
}

// Mean absolute error per channel between got (rows shifted by dy, columns
// by dx) and the reference (rgb, or grey when channels == 1).
double mae(const Got& g, const std::vector<unsigned char>& ref, int w, int h, int channels, int dy = 0, int dx = 0) {
    double s = 0;
    long n = 0;
    for (std::map<int, std::vector<unsigned char> >::const_iterator it = g.rows.begin(); it != g.rows.end(); ++it) {
        const int y = it->first + dy;
        if (y < 0 || y >= h || static_cast<int>(it->second.size()) < w * 3) continue;
        for (int x = 0; x < w; ++x) {
            const int xs = x + dx;
            if (xs < 0 || xs >= w) continue;
            for (int c = 0; c < channels; ++c) {
                const int r = ref[(static_cast<std::size_t>(y) * w + xs) * channels + c];
                s += std::abs(static_cast<int>(it->second[x * 3 + c]) - r);
                ++n;
            }
        }
    }
    return n ? s / n : 1e9;
}

// The best (dy, dx) alignment, for the diagnostics and the alignment checks.
int g_bestDy = 0, g_bestDx = 0;
std::string bestShift(const Got& g, const std::vector<unsigned char>& ref, int w, int h, int channels, int maxDx) {
    double best = 1e9;
    int by = 0, bx = 0;
    for (int dy = -2; dy <= 2; ++dy)
        for (int dx = -maxDx; dx <= maxDx; dx += (maxDx > 8 ? 2 : 1)) {
            const double m = mae(g, ref, w, h, channels, dy, dx);
            if (m < best) { best = m; by = dy; bx = dx; }
        }
    g_bestDy = by;
    g_bestDx = bx;
    char b[96];
    std::snprintf(b, sizeof b, "best fit dy %+d dx %+d: %.1f", by, bx, best);
    return b;
}

// ===========================================================================
// FAX
// ===========================================================================
const int kFaxH = 80;

void testFax() {
    std::vector<unsigned char> chart;
    dec::testChartGrey(dec::faxWidth, kFaxH, chart);
    const double rates[2] = {48000, 12000};
    const double ppms[3] = {0, 100, -100};
    for (int r = 0; r < 2; ++r)
        for (int p = 0; p < 3; ++p) {
            const double rate = rates[r], ppm = ppms[p];
            std::vector<float> sig;
            dec::synthFax(chart, dec::faxWidth, kFaxH, rate * (1 + ppm * 1e-6), sig, dec::FaxTransmission());
            // (the lead-in varies so the line start falls at different
            //  places in the decoder's free-running rows, the row edge too)
            const double lead = 2 + 0.17 * (r * 3 + p);
            const std::vector<float> a = withNoise(sig, rate, lead, 3, noiseSigma(0.5, kSnrDb, rate), 11 + r * 3 + p);
            dec::FaxDecoder fax;
            std::vector<dec::PictureLine> lines;
            feed(fax, a, rate, lines);
            const Got g = picture(lines, 1);
            const std::vector<unsigned> pics = pictureNumbers(lines);
            const double m = mae(g, chart, dec::faxWidth, kFaxH, 1);
            const dec::Status st = fax.status();
            char name[160], det[256];
            std::snprintf(name, sizeof name, "FAX 120 LPM @%.0f Hz, %+.0f ppm, %.0f dB: one picture, %d rows, MAE %.1f < %.0f, aligned",
                          rate, ppm, kSnrDb, static_cast<int>(g.rows.size()), m, kFaxMae);
            std::snprintf(det, sizeof det, "pictures %u, width %d, ordered %d, state %s, slant %.0f ppm, %s",
                          static_cast<unsigned>(pics.size()), g.width, g.ordered ? 1 : 0, fax.stateName(),
                          fax.measuredSlantPpm(), bestShift(g, chart, dec::faxWidth, kFaxH, 1, 12).c_str());
            const int nr = static_cast<int>(g.rows.size());
            check(pics.size() == 1 && pics[0] == 1 && g.width == dec::faxWidth && g.ordered && nr >= kFaxH - 3 &&
                      nr <= kFaxH + 1 && m < kFaxMae && st.pictureWidth == dec::faxWidth && st.pictureHeight == 0 &&
                      st.mode == "FAX 120 LPM" && fax.state() == dec::FaxDecoder::Done && g_bestDy == 0 &&
                      std::abs(g_bestDx) <= 2,
                  name, det);
            if (ppm != 0) {
                std::snprintf(name, sizeof name, "FAX @%.0f Hz: slant measured %+.0f ppm (sent %+.0f)", rate,
                              fax.measuredSlantPpm(), ppm);
                check(std::fabs(fax.measuredSlantPpm() - ppm) < 35, name);
            }
        }

    // Other speeds (48 kHz, clean but for the noise).
    const int lpms[2] = {240, 60};
    for (int i = 0; i < 2; ++i) {
        const double rate = 48000;
        dec::FaxTransmission tx;
        tx.lpm = lpms[i];
        const int h = lpms[i] == 60 ? 30 : kFaxH;
        std::vector<unsigned char> c2;
        dec::testChartGrey(dec::faxWidth, h, c2);
        std::vector<float> sig;
        dec::synthFax(c2, dec::faxWidth, h, rate, sig, tx);
        const std::vector<float> a = withNoise(sig, rate, 1, 2, noiseSigma(0.5, kSnrDb, rate), 21 + i);
        dec::FaxDecoder fax;
        check(fax.setLpm(lpms[i]) && !fax.setLpm(100) && fax.lpm() == lpms[i], "FAX setLpm accepts 60/90/120/240 only");
        std::vector<dec::PictureLine> lines;
        feed(fax, a, rate, lines);
        const Got g = picture(lines, 1);
        const double m = mae(g, c2, dec::faxWidth, h, 1);
        char name[160];
        std::snprintf(name, sizeof name, "FAX %d LPM @48000 Hz: %d rows of %d, MAE %.1f, mode \"%s\"", lpms[i],
                      static_cast<int>(g.rows.size()), h, m, fax.status().mode.c_str());
        const int nr = static_cast<int>(g.rows.size());
        char want[32];
        std::snprintf(want, sizeof want, "FAX %d LPM", lpms[i]);
        const std::string bs = bestShift(g, c2, dec::faxWidth, h, 1, 12);
        check(nr >= h - 3 && nr <= h + 1 && m < kFaxMae && fax.status().mode == want && g_bestDy == 0 &&
                  std::abs(g_bestDx) <= 2,
              name, bs);
    }

    // Manual start: no start tone, no phasing; the audio starts with the
    // image, so the decoder's free-running rows start with the lines.
    {
        const double rate = 12000;
        dec::FaxTransmission tx;
        tx.startSec = 0;
        tx.phasingLines = 0;
        std::vector<float> sig;
        dec::synthFax(chart, dec::faxWidth, kFaxH, rate, sig, tx);
        Noise gn(31);
        const double sg = noiseSigma(0.5, kSnrDb, rate);
        for (std::size_t i = 0; i < sig.size(); ++i) sig[i] += static_cast<float>(sg * gn.gauss());
        dec::FaxDecoder fax;
        fax.startNow();
        std::vector<dec::PictureLine> lines;
        feed(fax, sig, rate, lines);
        const Got g = picture(lines, 1);
        const double m = mae(g, chart, dec::faxWidth, kFaxH, 1);
        char name[160];
        std::snprintf(name, sizeof name, "FAX startNow() (no start tone/phasing): %d rows, MAE %.1f, ends on the stop tone",
                      static_cast<int>(g.rows.size()), m);
        const std::string bs = bestShift(g, chart, dec::faxWidth, kFaxH, 1, 12);
        check(static_cast<int>(g.rows.size()) >= kFaxH - 3 && m < kFaxMae && fax.state() == dec::FaxDecoder::Done &&
                  g_bestDy == 0 && std::abs(g_bestDx) <= 2,
              name, bs);
    }

    // Two charts in a row (20 s of noise between): pictures 1 and 2.
    {
        const double rate = 12000;
        std::vector<unsigned char> c2;
        dec::testChartGrey(dec::faxWidth, 30, c2);
        std::vector<float> sig;
        dec::synthFax(c2, dec::faxWidth, 30, rate, sig);
        sig.resize(sig.size() + static_cast<std::size_t>(20 * rate), 0.0f);
        dec::synthFax(c2, dec::faxWidth, 30, rate, sig);
        const std::vector<float> a = withNoise(sig, rate, 1, 2, noiseSigma(0.5, kSnrDb, rate), 41);
        dec::FaxDecoder fax;
        std::vector<dec::PictureLine> lines;
        feed(fax, a, rate, lines);
        const std::vector<unsigned> pics = pictureNumbers(lines);
        const Got g1 = picture(lines, 1), g2 = picture(lines, 2);
        char name[160];
        std::snprintf(name, sizeof name, "FAX two charts: pictures %u (%d rows, MAE %.1f) and then %u (%d rows, MAE %.1f)",
                      pics.size() > 0 ? pics[0] : 0, static_cast<int>(g1.rows.size()), mae(g1, c2, dec::faxWidth, 30, 1),
                      pics.size() > 1 ? pics[1] : 0, static_cast<int>(g2.rows.size()), mae(g2, c2, dec::faxWidth, 30, 1));
        check(pics.size() == 2 && pics[0] == 1 && pics[1] == 2 && g1.rows.size() >= 27 && g2.rows.size() >= 27 &&
                  mae(g1, c2, dec::faxWidth, 30, 1) < kFaxMae && mae(g2, c2, dec::faxWidth, 30, 1) < kFaxMae &&
                  fax.pictures() == 2,
              name);
    }

    // Noise only (an AGC-loud band: noise at the level of a signal).
    {
        const double rate = 48000;
        std::vector<float> none;
        const std::vector<float> a = withNoise(none, rate, 120, 0, 0.3, 51);
        dec::FaxDecoder fax;
        std::vector<dec::PictureLine> lines;
        feed(fax, a, rate, lines);
        check(lines.empty() && fax.pictures() == 0 && fax.state() == dec::FaxDecoder::Waiting &&
                  !fax.status().signal && fax.status().pictureWidth == 0,
              "FAX: no picture from 120 s of noise", fax.status().info);
    }

    // NaN/Inf: in the lead-in and inside the image.
    {
        const double rate = 12000;
        std::vector<float> sig;
        dec::synthFax(chart, dec::faxWidth, kFaxH, rate, sig);
        std::vector<float> a = withNoise(sig, rate, 2, 3, noiseSigma(0.5, kSnrDb, rate), 61);
        const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
        a[100] = nan; a[101] = inf; a[102] = -inf;
        const std::size_t img = static_cast<std::size_t>((2 + 5 + 10 + 20) * rate);   // 20 s into the image
        for (int k = 0; k < 5; ++k) a[img + k * 17] = nan;
        dec::FaxDecoder fax;
        std::vector<dec::PictureLine> lines;
        feed(fax, a, rate, lines);
        const Got g = picture(lines, 1);
        const double m = mae(g, chart, dec::faxWidth, kFaxH, 1);
        char name[160];
        std::snprintf(name, sizeof name, "FAX: NaN/Inf samples do not poison it (%d rows, MAE %.1f)",
                      static_cast<int>(g.rows.size()), m);
        check(static_cast<int>(g.rows.size()) >= kFaxH - 3 && m < kFaxMae, name);
    }
}

// ===========================================================================
// SSTV
// ===========================================================================
void testSstv() {
    for (int i = 0; i < dec::sstvModeCount(); ++i) {
        const dec::SstvMode& md = dec::sstvModeAt(i);
        std::vector<unsigned char> card;
        dec::testCardRgb(md.width, md.height, card);
        for (int r = 0; r < 2; ++r) {
            const double rate = r == 0 ? 48000 : 12000, ppm = r == 0 ? 100 : -100;
            std::vector<float> sig;
            dec::synthSstv(md.vis, card, rate * (1 + ppm * 1e-6), sig, kAmp);
            const std::vector<float> a = withNoise(sig, rate, 1.5, 2, noiseSigma(kAmp, kSnrDb, rate), 100 + i * 2 + r);
            dec::SstvDecoder sstv;
            std::vector<dec::PictureLine> lines;
            std::string modeSeen;
            // (feed, noting the mode while the picture is received)
            {
                static const std::size_t sizes[] = {1024, 333, 4096, 7, 2048, 960};
                std::size_t j = 0, k = 0;
                while (j < a.size()) {
                    std::size_t n = sizes[k++ % 6];
                    if (j + n > a.size()) n = a.size() - j;
                    sstv.process(&a[j], n, rate);
                    sstv.takeLines(lines);
                    if (sstv.receiving()) modeSeen = sstv.status().mode;
                    j += n;
                }
            }
            const Got g = picture(lines, 1);
            const std::vector<unsigned> pics = pictureNumbers(lines);
            const double m = mae(g, card, md.width, md.height, 3);
            const dec::Status st = sstv.status();
            char name[200], det[256];
            std::snprintf(name, sizeof name, "SSTV %-10s @%.0f Hz, %+.0f ppm, %.0f dB: %dx%d, %d rows, MAE %.1f < %.0f, aligned",
                          md.name, rate, ppm, kSnrDb, g.width, st.pictureHeight, static_cast<int>(g.rows.size()), m, kSstvMae);
            std::snprintf(det, sizeof det, "mode \"%s\", pictures %u, ordered %d, drift %.1f, %s", modeSeen.c_str(),
                          static_cast<unsigned>(pics.size()), g.ordered ? 1 : 0, sstv.driftSamples(),
                          bestShift(g, card, md.width, md.height, 3, 6).c_str());
            check(modeSeen == md.name && st.mode == md.name && pics.size() == 1 && pics[0] == 1 &&
                      g.width == md.width && st.pictureWidth == md.width && st.pictureHeight == md.height &&
                      static_cast<int>(g.rows.size()) == md.height && g.ordered && st.pictureRow == md.height &&
                      m < kSstvMae && !sstv.receiving() && g_bestDy == 0 && std::abs(g_bestDx) <= 1,
                  name, det);
        }
    }

    // Three pictures in a row (Martin M2, Scottie S2, Martin M2), 3 s apart.
    {
        const double rate = 12000;
        std::vector<unsigned char> card;
        dec::testCardRgb(320, 256, card);
        std::vector<float> sig;
        const int seq[3] = {40, 56, 40};
        for (int k = 0; k < 3; ++k) {
            dec::synthSstv(seq[k], card, rate, sig, kAmp);
            sig.resize(sig.size() + static_cast<std::size_t>(3 * rate), 0.0f);
        }
        const std::vector<float> a = withNoise(sig, rate, 1, 1, noiseSigma(kAmp, kSnrDb, rate), 201);
        dec::SstvDecoder sstv;
        std::vector<dec::PictureLine> lines;
        feed(sstv, a, rate, lines);
        const std::vector<unsigned> pics = pictureNumbers(lines);
        bool ok = pics.size() == 3 && sstv.pictures() == 3;
        std::string det;
        for (unsigned p = 1; p <= 3; ++p) {
            const Got g = picture(lines, p);
            const double m = mae(g, card, 320, 256, 3);
            char b[64];
            std::snprintf(b, sizeof b, "#%u %d rows MAE %.1f  ", p, static_cast<int>(g.rows.size()), m);
            det += b;
            if (g.rows.size() != 256 || m >= kSstvMae || (pics.size() >= p && pics[p - 1] != p)) ok = false;
        }
        check(ok, "SSTV three pictures in a row: numbers 1, 2, 3, each complete", det);
    }

    // Noise only.
    {
        const double rate = 48000;
        std::vector<float> none;
        const std::vector<float> a = withNoise(none, rate, 180, 0, 0.3, 301);
        dec::SstvDecoder sstv;
        std::vector<dec::PictureLine> lines;
        feed(sstv, a, rate, lines);
        check(lines.empty() && sstv.pictures() == 0 && sstv.status().pictureWidth == 0,
              "SSTV: no picture from 180 s of noise", sstv.status().info);
    }

    // NaN/Inf: in the lead-in and inside the picture.
    {
        const double rate = 48000;
        std::vector<unsigned char> card;
        dec::testCardRgb(320, 256, card);
        std::vector<float> sig;
        dec::synthSstv(40, card, rate, sig, kAmp);
        std::vector<float> a = withNoise(sig, rate, 1, 1, noiseSigma(kAmp, kSnrDb, rate), 401);
        const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
        a[10] = nan; a[11] = inf; a[12] = -inf;
        const std::size_t mid = static_cast<std::size_t>(30 * rate);
        for (int k = 0; k < 5; ++k) a[mid + k * 13] = nan;
        dec::SstvDecoder sstv;
        std::vector<dec::PictureLine> lines;
        feed(sstv, a, rate, lines);
        const Got g = picture(lines, 1);
        const double m = mae(g, card, 320, 256, 3);
        char name[160];
        std::snprintf(name, sizeof name, "SSTV: NaN/Inf samples do not poison it (%d rows, MAE %.1f)",
                      static_cast<int>(g.rows.size()), m);
        check(g.rows.size() == 256 && m < kSstvMae && sstv.status().toneHz == sstv.status().toneHz, name);
    }
}

void testEncoders() {
    std::vector<unsigned char> c;
    dec::testCardRgb(320, 256, c);
    std::vector<float> a, b;
    dec::synthSstv(44, c, 12000, a);
    dec::synthSstv(44, c, 12000, b);
    check(a == b && !a.empty(), "SSTV synthesis is deterministic");
    double ms = 0;
    std::vector<dec::SstvTone> seq;
    dec::sstvTones(*dec::sstvModeByVis(44), c, seq);
    for (std::size_t i = 0; i < seq.size(); ++i) ms += seq[i].ms;
    char name[128];
    std::snprintf(name, sizeof name, "Martin M1 transmission %.3f s (VIS 910 ms + 256 x 446.446 ms)", ms / 1000);
    check(std::fabs(ms - (910 + 256 * 446.446)) < 1, name);
    check(dec::sstvModeByVis(95) && dec::sstvModeByVis(95)->width == 640 && dec::sstvModeByVis(95)->height == 496 &&
              !dec::sstvModeByVis(8),
          "SSTV format table (PD120 640x496; VIS 8 (Robot 36) not supported, as in the JS)");
}

void testCpu() {
    const double rate = 48000;
    {
        std::vector<unsigned char> chart;
        dec::testChartGrey(dec::faxWidth, 40, chart);
        std::vector<float> sig;
        dec::synthFax(chart, dec::faxWidth, 40, rate, sig);
        const std::vector<float> a = withNoise(sig, rate, 0, 0, noiseSigma(0.5, kSnrDb, rate), 501);
        dec::FaxDecoder fax;
        std::vector<dec::PictureLine> lines;
        const std::clock_t t0 = std::clock();
        feed(fax, a, rate, lines);
        const double cpu = double(std::clock() - t0) / CLOCKS_PER_SEC, secs = a.size() / rate;
        char name[128];
        std::snprintf(name, sizeof name, "CPU: FAX @48 kHz: %.0fx real time (%.2f%% of a core)",
                      secs / (cpu > 1e-9 ? cpu : 1e-9), 100 * cpu / secs);
        check(true, name);
    }
    {
        std::vector<unsigned char> card;
        dec::testCardRgb(320, 256, card);
        std::vector<float> sig;
        dec::synthSstv(44, card, rate, sig, kAmp);
        const std::vector<float> a = withNoise(sig, rate, 0, 0, noiseSigma(kAmp, kSnrDb, rate), 502);
        dec::SstvDecoder sstv;
        std::vector<dec::PictureLine> lines;
        const std::clock_t t0 = std::clock();
        feed(sstv, a, rate, lines);
        const double cpu = double(std::clock() - t0) / CLOCKS_PER_SEC, secs = a.size() / rate;
        char name[128];
        std::snprintf(name, sizeof name, "CPU: SSTV (Martin M1) @48 kHz: %.0fx real time (%.2f%% of a core)",
                      secs / (cpu > 1e-9 ? cpu : 1e-9), 100 * cpu / secs);
        check(true, name);
    }
}

}  // namespace

int main() {
    testEncoders();
    testFax();
    testSstv();
    testCpu();
    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
