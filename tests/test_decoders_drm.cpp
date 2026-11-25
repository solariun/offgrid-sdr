// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_decoders_drm.cpp
//  Layer:  Tests (Library liboffgrid-decoders)
//
//  What this file does
//  -------------------
//  Self-checking test of the DRM receiver (Drm.h) against the transmitter in
//  DrmEncoders.h.  Prints PASS/FAIL per check, returns nonzero on failure:
//    - the coding pieces: CRCs, interleaver permutations, the mother code
//      through the Viterbi decoder (clean and noisy), raw inflate, the text
//      message, MOT, Journaline;
//    - the demo's "OFFgrid DRM" (mode B, 10 kHz, 64-QAM, AAC + text +
//      SlideShow + Journaline) in noise (SNR 25 dB in the signal's band) and
//      237 Hz off: FAC/SDC fields, label, service ID, language, codec, the
//      text message, the PNG byte for byte, the Journaline pages, every AAC
//      access unit byte for byte with its DRM CRC byte (the CRC covers the
//      AAC core's bits, so the AAC decoder checks it), then the sound through
//      liboffgrid-codecs: every frame good, 960 samples each, level, mono;
//    - the same through a two-path channel (an echo of 0.5 at 1 ms), at 480
//      kS/s with the station 37 kHz from the stick's centre, and mirrored;
//    - the demo's "OFFgrid xHE": xHE-AAC reported as unsupported ("xHE-AAC:
//      not supported yet") in the status and the services, its frames
//      recovered byte for byte, its text and picture decoded;
//    - other robustness modes and occupancies (A 10 kHz 16-QAM long
//      interleaving 16-QAM SDC; B 20 kHz; C 10 kHz; D 10 kHz; A 5 kHz;
//      B 9 kHz; A 18 kHz), each in noise and off frequency;
//    - nothing from noise, nothing from an AM station; service selection;
//    - CPU: the demo station at 2.048 MS/s, % of a core.
//
//  How it fits in
//  --------------
//  Build (from the repository root; add src/codecs/*.cpp once present):
//    c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors -Isrc \
//        src/dsp/Fft.cpp src/decoders/Drm*.cpp tests/test_decoders_drm.cpp \
//        build/lib/liboffgrid-codecs.a -o test_decoders_drm
//    (or src/codecs/*.cpp in place of the library)
//
//  Contributing
//  ------------
//  Keep it deterministic (own noise generator, fixed seeds).  The receiver
//  and the transmitter are ports of OFFgrid-SDR's JS (diddy-boy), which
//  follows Dream (GPL-2.0-or-later).
// ============================================================================
#include "decoders/Drm.h"
#include "decoders/DrmEncoders.h"
#include "decoders/DrmInternal.h"

#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <limits>
#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace {

int g_failures = 0, g_checks = 0;

void check(bool ok, const std::string& name, const std::string& detailIn = "") {
    std::string detail(detailIn);
    while (!detail.empty() && detail[detail.size() - 1] == '\n') detail.erase(detail.size() - 1);
    for (std::size_t i = 0; i < detail.size(); ++i)
        if (detail[i] == '\n') detail[i] = ' ';
    ++g_checks;
    if (!ok) ++g_failures;
    std::printf("%s  %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : "  -- ", detail.c_str());
    std::fflush(stdout);
}

std::string str(const char* f, double a) {
    char b[160];
    std::snprintf(b, sizeof b, f, a);
    return b;
}
std::string hex(unsigned v) {
    char b[32];
    std::snprintf(b, sizeof b, "0x%X", v);
    return b;
}
std::string str2(const char* f, double a, double b2) {
    char b[160];
    std::snprintf(b, sizeof b, f, a, b2);
    return b;
}

// Deterministic Gaussian noise (xorshift + Box-Muller).
class Noise {
public:
    explicit Noise(unsigned long long seed) : s_(seed ? seed : 1), have_(false), spare_(0) {}
    double uniform() {
        s_ ^= s_ << 13;
        s_ ^= s_ >> 7;
        s_ ^= s_ << 17;
        return ((s_ >> 11) + 0.5) / 9007199254740992.0;
    }
    double gauss() {
        if (have_) {
            have_ = false;
            return spare_;
        }
        const double u = uniform(), v = uniform(), r = std::sqrt(-2 * std::log(u)), a = 6.283185307179586 * v;
        spare_ = r * std::sin(a);
        have_ = true;
        return r * std::cos(a);
    }

private:
    unsigned long long s_;
    bool have_;
    double spare_;
};

typedef std::vector<std::complex<float> > Iq;

struct Channel {
    Channel() : seconds(16), snrDb(99), offsetHz(0), mirror(false), echo(0), echoDelay(48), rate(48000), stationHz(0), seed(7) {}
    double seconds, snrDb, offsetHz;
    bool mirror;
    double echo;
    int echoDelay;
    double rate;        // the stick's rate (48000: the signal as made)
    double stationHz;   // the station's distance from the stick's centre
    unsigned long long seed;
};

// The loop repeated for `seconds`, through the channel, at 48 kHz.
Iq channel48(const Iq& loop, const Channel& c, double bwKhz) {
    const std::size_t n = static_cast<std::size_t>(c.seconds * 48000);
    Iq out(n);
    double p = 0;
    for (std::size_t i = 0; i < loop.size(); ++i) p += std::norm(loop[i]);
    p /= loop.size();
    // noise power over 48 kHz such that the SNR holds in the signal's band
    const double sigma = std::sqrt(p / std::pow(10.0, c.snrDb / 10) * (48.0 / bwKhz) / 2);
    Noise nz(c.seed);
    const double w = 2 * 3.14159265358979323846 * c.offsetHz / 48000;
    for (std::size_t i = 0; i < n; ++i) {
        std::complex<double> v(loop[i % loop.size()]);
        if (c.echo != 0 && i >= static_cast<std::size_t>(c.echoDelay))
            v += c.echo * std::complex<double>(0.6, 0.8) * std::complex<double>(loop[(i - c.echoDelay) % loop.size()]);
        v *= std::polar(1.0, w * static_cast<double>(i));
        if (c.mirror) v = std::conj(v);
        if (c.snrDb < 90) v += std::complex<double>(sigma * nz.gauss(), sigma * nz.gauss());
        out[i] = std::complex<float>(v);
    }
    return out;
}

// 48 kHz -> rate (an integer multiple, windowed-sinc interpolation), the
// station moved to stationHz.
Iq upsample(const Iq& x, int factor, double stationHz) {
    const int taps = 16;  // each side, in input samples
    std::vector<double> h(static_cast<std::size_t>(2 * taps * factor + 1));
    for (int i = -taps * factor; i <= taps * factor; ++i) {
        const double t = static_cast<double>(i) / factor, wnd = 0.5 + 0.5 * std::cos(3.14159265358979323846 * i / (taps * factor + 1));
        h[static_cast<std::size_t>(i + taps * factor)] = (i == 0 ? 1 : std::sin(3.14159265358979323846 * t * 0.9) / (3.14159265358979323846 * t)) * wnd;
        if (i == 0) h[static_cast<std::size_t>(taps * factor)] = 0.9;
    }
    Iq out(x.size() * factor);
    const double w = 2 * 3.14159265358979323846 * stationHz / (48000.0 * factor);
    std::complex<double> rot(1, 0);
    const std::complex<double> step = std::polar(1.0, w);
    for (std::size_t j = 0; j < out.size(); ++j) {
        const long long m = static_cast<long long>(j / factor), ph = static_cast<long long>(j % factor);
        std::complex<double> acc;
        for (int q = -taps; q <= taps; ++q) {
            const long long src = m - q;
            if (src < 0 || src >= static_cast<long long>(x.size())) continue;
            acc += h[static_cast<std::size_t>(q * factor + ph + taps * factor)] * std::complex<double>(x[static_cast<std::size_t>(src)]);
        }
        out[j] = std::complex<float>(acc * rot);
        rot *= step;
        if ((j & 1023) == 0) rot /= std::abs(rot);
    }
    return out;
}

void feed(dec::DrmDecoder& d, const Iq& x, double rate, double centerHz, double dialHz) {
    const std::size_t chunk = static_cast<std::size_t>(rate / 20);
    for (std::size_t i = 0; i < x.size(); i += chunk) d.processIq(&x[i], std::min(chunk, x.size() - i), rate, centerHz, dialHz);
}

bool startsWithZeros(const std::vector<unsigned char>& got, const std::vector<unsigned char>& want) {
    if (got.size() < want.size()) return false;
    if (std::memcmp(&got[0], &want[0], want.size()) != 0) return false;
    for (std::size_t i = want.size(); i < got.size(); ++i)
        if (got[i]) return false;
    return true;
}

// the AAC access units against what was sent (frame number -> the loop's
// multiplex frame); returns matched, sets bad
int checkAacAus(const std::vector<dec::DrmAccessUnit>& aus, const dec::DrmTxReport& rep, int& bad) {
    int ok = 0;
    bad = 0;
    long long cur = -1;
    int j = 0;
    for (std::size_t q = 0; q < aus.size(); ++q) {
        if (aus[q].codec != 0) continue;
        if (aus[q].frame != cur) {
            cur = aus[q].frame;
            j = 0;
        }
        const std::size_t g = static_cast<std::size_t>(((cur % rep.muxFrames) + rep.muxFrames) % rep.muxFrames);
        if (j < static_cast<int>(rep.audio[g].size()) && startsWithZeros(aus[q].bytes, rep.audio[g][static_cast<std::size_t>(j)]))
            ++ok;
        else
            ++bad;
        ++j;
    }
    return ok;
}

std::vector<std::vector<unsigned char> > syntheticAus(int count, int size, unsigned long long seed) {
    Noise r(seed);
    std::vector<std::vector<unsigned char> > out;
    for (int i = 0; i < count; ++i) {
        std::vector<unsigned char> a(static_cast<std::size_t>(size));
        for (int k = 0; k < size; ++k) a[static_cast<std::size_t>(k)] = static_cast<unsigned char>(r.uniform() * 256);
        a[static_cast<std::size_t>(size - 1)] |= 1;  // (no trailing zero: the receiver's last AU is padded with them)
        out.push_back(a);
    }
    return out;
}

// ---- the pieces --------------------------------------------------------------------
void testPieces() {
    using namespace dec::drm;
    // CRCs: the CCITT CRC-16 of "123456789" (0xFFFF start, inverted) = 0xD64E
    const unsigned char nine[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    check(crc16(nine, 9) == 0xD64E, "CRC-16 (text, packets) of \"123456789\"", hex(crc16(nine, 9)));
    check(crcDream(nine, 9, 16) == 0xD64E, "Dream's CRC-16 equals the CCITT CRC-16", hex(crcDream(nine, 9, 16)));
    // interleavers are permutations
    bool perm = true;
    const int sizes[4] = {130, 2398, 1199, 461};
    const int t0s[3] = {5, 13, 21};
    for (int a = 0; a < 4; ++a)
        for (int b = 0; b < 3; ++b) {
            std::vector<int> t;
            ilTable(sizes[a], t0s[b], t);
            std::vector<int> seen(static_cast<std::size_t>(sizes[a]), 0);
            for (std::size_t i = 0; i < t.size(); ++i)
                if (t[i] < 0 || t[i] >= sizes[a] || seen[static_cast<std::size_t>(t[i])]++) perm = false;
        }
    check(perm, "interleaver tables are permutations");
    // the mother code through the Viterbi decoder, for every rate
    Noise nz(3);
    Viterbi v;
    bool clean = true, noisy = true;
    for (int rate = 0; rate <= 12; ++rate) {
        const int N2 = 600, M = punct(rate)[0] * ((2 * N2 - 12) / punct(rate)[1]);
        Pattern pat;
        makePattern(rate, M, N2, false, pat);
        if (patternBits(pat) != 2 * N2) clean = false;
        Bytes bits(static_cast<std::size_t>(M)), code, dec;
        for (int i = 0; i < M; ++i) bits[static_cast<std::size_t>(i)] = nz.uniform() < 0.5;
        convEncode(&bits[0], bits.size(), pat, code);
        std::vector<double> soft(code.size()), soft2(code.size());
        for (std::size_t i = 0; i < code.size(); ++i) {
            soft[i] = code[i] ? -1 : 1;
            soft2[i] = soft[i] + 0.45 * nz.gauss();
        }
        v.decode(&soft[0], pat, dec);
        if (dec != bits) clean = false;
        if (rate <= 4) {  // (the strong rates: noisy too)
            v.decode(&soft2[0], pat, dec);
            if (dec != bits) noisy = false;
        }
    }
    check(clean, "the punctured mother code decodes (13 rates, coded length = 2 x cells)");
    check(noisy, "... and through noise (sigma 0.45, rates 0-4)");
    // raw inflate: "hello hello hello hello\n" compressed (fixed Huffman)
    const unsigned char defl[] = {0xcb, 0x48, 0xcd, 0xc9, 0xc9, 0x57, 0xc8, 0x40, 0x27, 0xb9, 0x00};
    Bytes out;
    const bool okInf = inflateRaw(defl, sizeof defl, out, 1000);
    check(okInf && std::string(out.begin(), out.end()) == "hello hello hello hello\n", "raw inflate (fixed codes)",
          std::string(out.begin(), out.end()));
    {
        const unsigned char dyn[] = {0xed, 0x8c, 0xb1, 0x0d, 0x00, 0x30, 0x0c, 0xc2, 0x6e, 0xb5, 0xa2, 0x4a, 0x24, 0xf9, 0x7f, 0x2f, 0xfc,
                                     0xd0, 0xb1, 0x13, 0x32, 0x46, 0xb0, 0x2d, 0xb1, 0x9c, 0x61, 0x8e, 0x53, 0xea, 0x40, 0x0d, 0xd8, 0x10,
                                     0xa0, 0x6c, 0xf0, 0xac, 0x89, 0x71, 0xb1, 0x4d, 0x66, 0x4c, 0xcc, 0x3f, 0x78, 0x79, 0x70, 0x01};
        Bytes want;
        for (int i = 0; i < 400; ++i) want.push_back(static_cast<unsigned char>('a' + (i % 5 ? (i * i * 7 + i * 3) % 13 : 0)));
        const bool okD = inflateRaw(dyn, sizeof dyn, out, 1000);
        check(okD && out == want, "raw inflate (dynamic codes)", str("%.0f bytes", out.size()));
    }
    // Journaline NML: a compressed object (stored block) parses
    {
        const unsigned char nml[] = {0x01, 'T', 'i', 't', 'l', 'e', 0x03, 'B', 'o', 'd', 'y', 0x10, 'x', 0x12, 'y', 0x13};
        Bytes st;
        st.push_back(0x01);  // a final stored block
        st.push_back(sizeof nml);
        st.push_back(0);
        st.push_back(static_cast<unsigned char>(~sizeof nml));
        st.push_back(0xff);
        st.insert(st.end(), nml, nml + sizeof nml);
        Bytes g;
        g.push_back(0x40);
        g.push_back(0);
        g.push_back(0x01);
        g.push_back(0x07);
        g.push_back((2 << 5) | 0x08 | 1);  // text, compressed, revision 1
        g.push_back(0x08);
        g.insert(g.end(), st.begin(), st.end());
        const unsigned c = crc16(&g[0], g.size());
        g.push_back(static_cast<unsigned char>(c >> 8));
        g.push_back(static_cast<unsigned char>(c & 255));
        Journaline jl;
        jl.group(g);
        const bool okJ = jl.pages.count(0x0107) && jl.pages[0x0107].title == "Title" && jl.pages[0x0107].text == "Body\nx*y*";
        check(okJ, "Journaline: a compressed text page", okJ ? "" : (jl.pages.count(0x0107) ? jl.pages[0x0107].text : "none"));
    }
}

// ---- a station through the receiver ------------------------------------------------------
struct Run {
    dec::DrmInfo info;
    std::vector<dec::DrmAccessUnit> aus;
    std::vector<dec::Decoder::EncodedImage> images;
    std::vector<dec::DrmJournalinePage> pages;
    std::vector<dec::Service> services;
    dec::Status status;
    std::string text;
    std::size_t pcm, pcmFrames;
    bool hasAudio, monoPairs;
    double rate, rms, peak, silentShare;
    double seconds;  // processing time
};

Run receive(const Iq& loop, const Channel& c, double bwKhz) {
    Run r;
    dec::DrmDecoder d;
    d.setKeepAccessUnits(true);
    Iq x = channel48(loop, c, bwKhz);
    const double dial = 11.700e6;
    double rate = 48000, center = dial;
    if (c.rate != 48000) {
        const int factor = static_cast<int>(c.rate / 48000);
        x = upsample(x, factor, c.stationHz);
        rate = 48000.0 * factor;
        center = dial - c.stationHz;
    }
    const std::clock_t t0 = std::clock();
    feed(d, x, rate, center, dial);
    r.seconds = static_cast<double>(std::clock() - t0) / CLOCKS_PER_SEC;
    r.info = d.info();
    d.takeAccessUnits(r.aus);
    d.takeImages(r.images);
    r.pages = d.journaline();
    r.services = d.services();
    r.status = d.status();
    d.takeText(r.text);
    std::vector<float> pcm;
    d.takeAudio(pcm);
    r.pcm = pcm.size();
    r.pcmFrames = pcm.size() / 2;
    r.hasAudio = d.hasAudio();
    r.rate = d.audioRate();
    double e = 0, pk = 0;
    r.monoPairs = true;
    for (std::size_t q = 0; q + 1 < pcm.size(); q += 2) {
        e += pcm[q] * pcm[q];
        pk = std::max(pk, static_cast<double>(std::fabs(pcm[q])));
        if (pcm[q] != pcm[q + 1]) r.monoPairs = false;
    }
    r.rms = r.pcmFrames ? std::sqrt(e / r.pcmFrames) : 0;
    r.peak = pk;
    const std::size_t blk = 480;  // 20 ms at 24 kHz
    std::size_t nb = 0, silent = 0;
    for (std::size_t b = 0; (b + 1) * blk <= r.pcmFrames; ++b, ++nb) {
        double be = 0;
        for (std::size_t q = b * blk; q < (b + 1) * blk; ++q) be += pcm[2 * q] * pcm[2 * q];
        if (std::sqrt(be / blk) < 1e-4) ++silent;
    }
    r.silentShare = nb ? static_cast<double>(silent) / nb : 1;
    return r;
}

void testDemoAac() {
    dec::DrmTxReport rep;
    Iq loop;
    std::string err;
    const bool built = dec::drmSignal(dec::drmDemoStation(0), loop, &err, &rep);
    check(built && loop.size() == 10u * 57600, "demo station 0 built: 12 s loop", err + str(" %.0f bytes a multiplex frame", rep.lenBytes));
    if (!built) return;
    Channel c;
    c.seconds = 30;
    c.snrDb = 25;
    c.offsetHz = 237;
    const Run r = receive(loop, c, 10);
    const dec::DrmInfo& i = r.info;
    check(i.label == "OFFgrid DRM", "AAC demo: label", i.label);
    check(i.mode == 'B' && i.occupancy == 3 && i.occupancyKhz == 10, "FAC: mode B, 10 kHz", std::string(1, i.mode ? i.mode : '?'));
    check(i.msc == "64-QAM" && !i.longInterleave && i.sdc == "4-QAM", "FAC: 64-QAM, short interleaving, 4-QAM SDC", i.msc + " / " + i.sdc);
    check(i.serviceId == 0xf06001 && i.language == "English", "FAC: service ID F06001, English", hex(i.serviceId) + " " + i.language);
    check(i.codec == 0 && i.rateCode == 3 && i.stereoMode == 0 && !i.sbr, "SDC: AAC, 24 kHz, mono, no SBR", i.audioText);
    check(i.hasMer && i.merDb > 18, "MER measured", str("%.1f dB", i.merDb));
    check(std::fabs(i.freqOffsetHz - 237) < 3, "frequency offset found", str("%.1f Hz", i.freqOffsetHz));
    check(i.facOk > 0 && i.sdcOk > 0, "FAC and SDC CRCs pass", str2("FAC %.0f, SDC %.0f", i.facOk, i.sdcOk));
    check(i.text == "OFFgrid-SDR demo: DRM digital radio with station text and a SlideShow picture", "text message", i.text);
    check(r.text.find("OFFgrid DRM: OFFgrid-SDR demo") != std::string::npos, "takeText gives the text message", r.text);
    const std::vector<unsigned char> png = dec::drmDemoSlidePng();
    check(r.images.size() >= 1 && r.images[0].bytes == png && r.images[0].mime == "image/png" && r.images[0].name == "OFFgrid-SDR.png",
          "SlideShow: the PNG byte for byte", r.images.empty() ? "none" : r.images[0].name + str(" %.0f bytes", r.images[0].bytes.size()));
    bool jlOk = r.pages.size() == 4;
    std::string jlInfo;
    for (std::size_t q = 0; q < r.pages.size(); ++q) jlInfo += "[" + r.pages[q].type + ": " + r.pages[q].title + "]";
    if (jlOk) {
        jlOk = r.pages[0].type == "menu" && r.pages[0].links.size() == 3 && r.pages[0].links[1].first == 0x0102 && r.pages[1].type == "text" &&
               r.pages[1].text.find("*News*") != std::string::npos && r.pages[2].type == "list" && r.pages[2].rows.size() == 3 &&
               r.pages[2].rows[1][1] == "OFFgrid DRM (AAC)";
    }
    check(jlOk, "Journaline: 4 pages (menu, text, list, text)", jlInfo);
    int bad = 0;
    const int ok = checkAacAus(r.aus, rep, bad);
    check(ok >= 200 && bad == 0, "AAC access units byte for byte, with their DRM CRC bytes", str2("%.0f good, %.0f bad", ok, bad));
    check(r.services.size() == 1 && r.services[0].label == "OFFgrid DRM" && r.services[0].audio && r.services[0].selected,
          "services(): one audio service", r.services.empty() ? "" : r.services[0].label + " / " + r.services[0].detail);
    check(r.status.locked && r.status.station == "OFFgrid DRM" && r.status.info.find("OFFgrid DRM") == 0, "status()", r.status.mode + " | " + r.status.info);
    {
        // the sound: 24 kHz mono AAC, 960 samples a frame, out as stereo pairs;
        // every AU after the first gives a frame (the codec's start-up delay)
        const std::size_t frames = r.pcmFrames, expect = static_cast<std::size_t>(i.audioGood > 0 ? i.audioGood - 1 : 0) * 960;
        check(r.hasAudio && r.rate == 24000 && i.audioGood >= 600 && i.audioBad == 0, "AAC decoded (liboffgrid-codecs): every frame good",
              str2("%.0f good, %.0f bad", i.audioGood, i.audioBad) + str(", %.0f Hz", r.rate));
        check(frames == expect, "PCM: 960 stereo frames per AU", str2("%.0f frames, expected %.0f", frames, expect));
        check(r.rms > 0.01 && r.peak > 0.1 && r.peak < 1.5 && r.silentShare < 0.5, "PCM has sound (not silence, not overload)",
              str2("RMS %.3f, peak %.3f", r.rms, r.peak) + str(", %.0f%% of 20 ms blocks silent", 100 * r.silentShare));
        check(r.monoPairs, "mono station: left = right");
    }
    std::printf("INFO  30 s of signal at 48 kS/s decoded in %.2f s (%.1f%% of a core)\n", r.seconds, 100 * r.seconds / 30);

    // multipath: an echo of 0.5 after 1 ms
    {
        Channel m = c;
        m.echo = 0.5;
        m.echoDelay = 48;
        m.seed = 11;
        const Run e = receive(loop, m, 10);
        int b2 = 0;
        const int g2 = checkAacAus(e.aus, rep, b2);
        check(e.info.label == "OFFgrid DRM" && g2 >= 200 && b2 <= g2 / 50 && e.images.size() >= 1, "two-path channel (echo 0.5 at 1 ms)",
              str2("%.0f AUs good, %.0f bad", g2, b2));
    }
    // the front end: 480 kS/s, the station 37 kHz from the stick's centre
    {
        Channel m = c;
        m.seconds = 20;
        m.rate = 480000;
        m.stationHz = 37000;
        m.seed = 12;
        const Run e = receive(loop, m, 10);
        int b2 = 0;
        const int g2 = checkAacAus(e.aus, rep, b2);
        check(e.info.label == "OFFgrid DRM" && g2 >= 100 && b2 == 0, "480 kS/s, station 37 kHz off the stick's centre",
              str2("%.0f AUs good, %.0f bad", g2, b2));
    }
    // mirrored (LSB tuning, IQ swapped)
    {
        Channel m = c;
        m.seconds = 16;
        m.mirror = true;
        m.seed = 13;
        const Run e = receive(loop, m, 10);
        int b2 = 0;
        const int g2 = checkAacAus(e.aus, rep, b2);
        check(e.info.label == "OFFgrid DRM" && e.info.mirrored && g2 > 50 && b2 == 0, "mirrored spectrum read", str2("%.0f AUs good, %.0f bad", g2, b2));
    }
    // how weak: the share of AUs at lower SNRs
    for (int snr = 20; snr >= 14; snr -= 3) {
        Channel m = c;
        m.seconds = 16;
        m.snrDb = snr;
        m.seed = 20 + snr;
        const Run e = receive(loop, m, 10);
        int b2 = 0;
        const int g2 = checkAacAus(e.aus, rep, b2);
        std::printf("INFO  SNR %d dB: label \"%s\", %d AUs good, %d bad, MER %.1f dB; codec: %lld frames decoded, %lld refused (CRC/damage)\n", snr,
                    e.info.label.c_str(), g2, b2, e.info.merDb, e.info.audioGood, e.info.audioBad);
        if (snr == 20) check(e.info.label == "OFFgrid DRM" && g2 > 50 && b2 <= g2 / 20, "SNR 20 dB: name and sound", str2("%.0f good, %.0f bad", g2, b2));
    }
}

void testDemoXhe() {
    dec::DrmTxReport rep;
    Iq loop;
    std::string err;
    const bool built = dec::drmSignal(dec::drmDemoStation(1), loop, &err, &rep);
    check(built && loop.size() == 32u * 57600, "demo station 1 (xHE-AAC) built: 38.4 s loop", err);
    if (!built) return;
    Channel c;
    c.seconds = 24;
    c.snrDb = 25;
    c.offsetHz = -151;
    c.seed = 5;
    const Run r = receive(loop, c, 10);
    const dec::DrmInfo& i = r.info;
    check(i.label == "OFFgrid xHE" && i.serviceId == 0xf06002, "xHE demo: label, service ID", i.label);
    check(i.codec == 3 && i.codecName == "xHE-AAC" && i.rateCode == 4, "SDC: xHE-AAC, 24 kHz", i.audioText);
    check(i.unsupported == "xHE-AAC: not supported yet", "reported as not supported", i.unsupported);
    check(r.status.info.find("xHE-AAC: not supported yet") != std::string::npos, "... in status()", r.status.info);
    check(!r.services.empty() && r.services[0].detail.find("xHE-AAC: not supported yet") != std::string::npos, "... in services()",
          r.services.empty() ? "" : r.services[0].detail);
    check(r.pcm == 0 && i.audioGood == 0, "no sound (xHE-AAC stays unsupported)");
    check(i.text == "OFFgrid-SDR demo: DRM in xHE-AAC, with station text and a SlideShow picture", "its text message", i.text);
    check(r.images.size() >= 1 && r.images[0].bytes == dec::drmDemoSlidePng(), "its SlideShow picture", str("%.0f pictures", r.images.size()));
    // the xHE-AAC frames: each one of the jingle's (with the packer's zero padding)
    const std::vector<std::vector<unsigned char> > src = dec::drmDemoXheFrames();
    int ok = 0, bad = 0;
    for (std::size_t q = 0; q < r.aus.size(); ++q) {
        bool found = false;
        for (std::size_t s = 0; s < src.size() && !found; ++s)
            if (startsWithZeros(r.aus[q].bytes, src[s])) found = true;
        if (found)
            ++ok;
        else
            ++bad;
    }
    check(ok > 100 && bad == 0, "xHE-AAC frames recovered byte for byte (super frame directory, header CRC)", str2("%.0f good, %.0f bad", ok, bad));
}

struct ModeCase {
    const char* name;
    int mode, so;
    bool q16;
    int prot;
    bool longIl, sdc16;
    int rateCode;
    double offset;
};

void testModes() {
    const ModeCase cases[] = {
        {"A 10 kHz 16-QAM, long interleaving, 16-QAM SDC", 0, 3, true, 0, true, true, 3, 310},
        {"B 20 kHz 64-QAM", 1, 5, false, 0, false, false, 3, -420},
        {"C 10 kHz 64-QAM prot 2", 2, 3, false, 2, false, false, 3, 88},
        {"D 10 kHz 16-QAM prot 1, 16-QAM SDC, AAC 12 kHz", 3, 3, true, 1, false, true, 1, -60},
        {"A 5 kHz 64-QAM, AAC 12 kHz", 0, 1, false, 0, false, false, 1, 150},
        {"B 9 kHz 16-QAM, long interleaving", 1, 2, true, 1, true, false, 3, 0},
        {"A 18 kHz 16-QAM", 0, 4, true, 0, false, false, 3, -200},
        {"C 20 kHz 64-QAM prot 3", 2, 5, false, 3, false, false, 3, 500},
    };
    for (std::size_t q = 0; q < sizeof cases / sizeof cases[0]; ++q) {
        const ModeCase& m = cases[q];
        dec::DrmTxOptions o;
        o.mode = m.mode;
        o.occupancy = m.so;
        o.qam16 = m.q16;
        o.protection = m.prot;
        o.longInterleave = m.longIl;
        o.sdc16 = m.sdc16;
        o.label = "Test " + std::string(1, "ABCD"[m.mode]);
        o.serviceId = 0x123456 + static_cast<unsigned>(q);
        o.language = 7;
        o.programmeType = 10;
        o.superframes = 5;
        o.aacRateCode = m.rateCode;
        // access units that fit: first how big a frame is, then fill most of it
        dec::DrmTxReport probe;
        Iq loop;
        std::string err;
        o.aac = syntheticAus(4, 8, 1);
        if (!dec::drmSignal(o, loop, &err, &probe)) {
            check(false, std::string("mode ") + m.name + ": built", err);
            continue;
        }
        const int nf = m.rateCode == 3 ? 10 : 5, hdr = (12 * (nf - 1) + (nf == 10 ? 4 : 0)) / 8;
        const int size = (probe.lenBytes - hdr - nf) / nf - 1;
        o.aac = syntheticAus(37, size, 100 + q);
        dec::DrmTxReport rep;
        if (!dec::drmSignal(o, loop, &err, &rep)) {
            check(false, std::string("mode ") + m.name + ": built", err);
            continue;
        }
        Channel c;
        c.seconds = m.longIl ? 18 : 14;
        c.snrDb = m.q16 ? 22 : 28;
        c.offsetHz = m.offset;
        c.seed = 40 + q;
        const double bw = dec::drm::kOccupancyKhz[m.so];
        const Run r = receive(loop, c, bw);
        int bad = 0;
        const int ok = checkAacAus(r.aus, rep, bad);
        const dec::DrmInfo& i = r.info;
        const bool fields = i.label == o.label && i.mode == "ABCD"[m.mode] && i.occupancy == m.so && i.msc == (m.q16 ? "16-QAM" : "64-QAM") &&
                            i.longInterleave == m.longIl && i.sdc == (m.sdc16 ? "16-QAM" : "4-QAM") && i.serviceId == o.serviceId &&
                            i.language == "German" && i.programmeType == "Pop Music" && i.rateCode == m.rateCode;
        char det[256];
        std::snprintf(det, sizeof det, "\"%s\" mode %c, occupancy %d, MSC %s, SDC %s, %d AUs good, %d bad (%d B/frame, AU %d B), MER %.1f, %.1f Hz", i.label.c_str(),
                      i.mode ? i.mode : '?', i.occupancy, i.msc.c_str(), i.sdc.c_str(), ok, bad, rep.lenBytes, size, i.merDb, i.freqOffsetHz);
        check(fields && ok >= 40 && bad == 0, std::string("mode ") + m.name, det);
    }
}

void testNothing() {
    // noise alone
    {
        dec::DrmDecoder d;
        Noise nz(99);
        Iq x(48000 * 20);
        for (std::size_t i = 0; i < x.size(); ++i) x[i] = std::complex<float>(static_cast<float>(0.1 * nz.gauss()), static_cast<float>(0.1 * nz.gauss()));
        feed(d, x, 48000, 6.0e6, 6.0e6);
        const dec::DrmInfo i = d.info();
        std::string t;
        d.takeText(t);
        std::vector<dec::Decoder::EncodedImage> im;
        d.takeImages(im);
        check(!i.signal && !i.locked && i.label.empty() && d.services().empty() && t.empty() && im.empty() && i.accessUnits == 0,
              "nothing from 20 s of noise", i.state + str(" (metric %.2f)", i.metric));
    }
    // an AM station (a carrier and a tone)
    {
        dec::DrmDecoder d;
        Noise nz(98);
        Iq x(48000 * 12);
        for (std::size_t i = 0; i < x.size(); ++i) {
            const double t = static_cast<double>(i) / 48000, a = 1 + 0.7 * std::sin(2 * 3.14159265358979323846 * 400 * t);
            x[i] = std::complex<float>(static_cast<float>(a + 0.01 * nz.gauss()), static_cast<float>(0.01 * nz.gauss()));
        }
        feed(d, x, 48000, 6.0e6, 6.0e6);
        const dec::DrmInfo i = d.info();
        check(!i.signal && i.label.empty(), "an AM station is not DRM", i.state + str(" (metric %.2f)", i.metric));
    }
    // NaN samples do no harm
    {
        dec::DrmDecoder d;
        Iq x(48000 * 5, std::complex<float>(std::numeric_limits<float>::quiet_NaN(), 1.0f));
        feed(d, x, 48000, 6.0e6, 6.0e6);
        check(!d.info().signal, "NaN samples do no harm");
    }
}

void testSelection() {
    Iq loop;
    dec::drmSignal(dec::drmDemoStation(0), loop);
    dec::DrmDecoder d;
    Channel c;
    c.seconds = 6;
    feed(d, channel48(loop, c, 10), 48000, 11.7e6, 11.7e6);
    check(d.selectService(0) && !d.selectService(2) && !d.selectService(7), "selectService: 0 yes, 2 and 7 no");
}

void testCpu() {
    // the demo station at 2.048 MS/s (linear interpolation from 48 kHz, the
    // station 200 kHz from the stick's centre), as the TUI feeds it
    Iq loop;
    dec::drmSignal(dec::drmDemoStation(0), loop);
    Channel c;
    c.seconds = 15;
    c.snrDb = 30;
    c.seed = 77;
    const Iq x48 = channel48(loop, c, 10);
    const double rate = 2048000, off = 200000;
    Iq x(static_cast<std::size_t>(c.seconds * rate));
    const double w = 2 * 3.14159265358979323846 * off / rate;
    for (std::size_t j = 0; j < x.size(); ++j) {
        const double t = j * 48000.0 / rate;
        const std::size_t m = static_cast<std::size_t>(t);
        const double f = t - m;
        const std::complex<double> a(x48[m]), b(x48[std::min(m + 1, x48.size() - 1)]);
        x[j] = std::complex<float>((a + (b - a) * f) * std::polar(1.0, w * static_cast<double>(j % 1000000)));
    }
    dec::DrmDecoder d;
    const std::clock_t t0 = std::clock();
    feed(d, x, rate, 11.5e6, 11.7e6);
    const double s = static_cast<double>(std::clock() - t0) / CLOCKS_PER_SEC;
    const dec::DrmInfo i = d.info();
    check(i.label == "OFFgrid DRM", "2.048 MS/s, station 200 kHz off centre: decoded", i.label + str(", %.0f AUs", i.accessUnits));
    std::printf("CPU   DRM at 2.048 MS/s: %.2f s for %.0f s of signal = %.1f%% of a core\n", s, c.seconds, 100 * s / c.seconds);
}

}  // namespace

int main() {
    std::printf("DRM receiver tests (AAC decoder %s)\n", dec::DrmDecoder::aacAvailable() ? "built in" : "not built in");
    testPieces();
    testDemoAac();
    testDemoXhe();
    testModes();
    testNothing();
    testSelection();
    testCpu();
    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
