// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_decoders_dab.cpp
//  Layer:  Tests (Library liboffgrid-decoders)
//
//  What this file does
//  -------------------
//  Self-checking test of the DAB / DAB+ receiver (Dab.h) against the demo
//  multiplex (DabEncoders.h).  Prints PASS/FAIL per check, returns nonzero
//  on failure:
//    - the pieces: CRC-16, Reed-Solomon (5 errors mended, 6 refused), the
//      demo's 96 DAB+ super frames (fire code, RS, AU CRCs), EBU Latin
//      text, EEP/UEP protection, the channel table;
//    - the multiplex, clean, at 2.048 MS/s: ensemble name and ID, both
//      stations with format and bit rate, programme types, the default
//      station (the first in the list), sub-channels; "OFFgrid Classic":
//      MP2 frames identical to those sent; then "OFFgrid Plus": super
//      frames, Reed-Solomon, every audio unit identical to those sent (AU
//      CRCs), the dynamic label (DLS), a SlideShow picture whose bytes are a
//      JPEG / PNG, the SPI logos; once liboffgrid-codecs is built in, sound;
//    - the same with noise (10 dB in the block) and the stick 12.3 kHz off;
//      with the stick centred 150 kHz away from the channel (mixed in the
//      decoder); with a single-frequency-network echo (a second
//      transmitter, half as strong, 60 us later); with 50,000 samples lost
//      on the way (the forgiving re-lock);
//    - nothing from 10 s of noise; a wrong sample rate is refused;
//    - CPU: real time at 2.048 MS/s with the margin printed.
//
//  How it fits in
//  --------------
//  Build (from the repository root):
//    c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors -Isrc \
//        src/dsp/Fft.cpp src/decoders/Dab*.cpp src/codecs/*.cpp \
//        tests/test_decoders_dab.cpp -o test_decoders_dab
//
//  Contributing
//  ------------
//  Keep it deterministic (own noise generator, fixed seeds).  The receiver
//  and the multiplex are ports of OFFgrid-SDR's JS (diddy-boy).
// ============================================================================
#include "decoders/Dab.h"
#include "decoders/DabEncoders.h"
#include "decoders/DabInternal.h"

#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <ctime>
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

void note(const std::string& s) {
    std::printf("      %s\n", s.c_str());
    std::fflush(stdout);
}

std::string fmt(const char* f, double a, double b = 0, double c = 0, double d = 0) {
    char buf[256];
    std::snprintf(buf, sizeof buf, f, a, b, c, d);
    return buf;
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

typedef std::complex<float> cf;
typedef std::vector<unsigned char> Bytes;

const double kRate = 2048000;
const double k12A = 223.936e6;

// ---- the reference: the audio units and MP2 frames the demo sends, in loop order
std::vector<Bytes> referenceAus(const Bytes& plus, int& sfOk, int& rsOk) {
    std::vector<Bytes> aus;
    sfOk = rsOk = 0;
    const int s = 6;   // 48 kbit/s
    for (std::size_t o = 0; o + 720 <= plus.size(); o += 720) {
        Bytes sf(plus.begin() + static_cast<std::ptrdiff_t>(o), plus.begin() + static_cast<std::ptrdiff_t>(o + 720));
        bool rs = true;
        for (int i = 0; i < s; ++i) {
            unsigned char col[120];
            for (int p = 0; p < 120; ++p) col[p] = sf[static_cast<std::size_t>(p * s + i)];
            if (dec::dab::rsDecode(col, 120) != 0) rs = false;
        }
        if (rs) ++rsOk;
        if (!dec::dab::superframeOk(&sf[0], s)) continue;
        ++sfOk;
        const int dac = (sf[2] >> 6) & 1, sbr = (sf[2] >> 5) & 1, n = dac ? (sbr ? 3 : 6) : (sbr ? 2 : 4);
        int st[7] = {dac ? (sbr ? 6 : 11) : (sbr ? 5 : 8), (sf[3] << 4) | (sf[4] >> 4), ((sf[4] & 15) << 8) | sf[5],
                     (sf[6] << 4) | (sf[7] >> 4),          ((sf[7] & 15) << 8) | sf[8], (sf[9] << 4) | (sf[10] >> 4), 0};
        st[n] = 110 * s;
        for (int j = 0; j < n; ++j) {
            int len = st[j + 1] - st[j];
            const unsigned char* a = &sf[static_cast<std::size_t>(st[j])];
            if (j == n - 1)
                for (int L = len; L > 2; --L)
                    if (dec::dab::crcOk(a, static_cast<std::size_t>(L))) {
                        len = L;
                        break;
                    }
            if (len > 2 && dec::dab::crcOk(a, static_cast<std::size_t>(len))) aus.push_back(Bytes(a, a + len - 2));
        }
    }
    return aus;
}

// how many of `got` follow `ref` cyclically, from where the first one is found
int matchCyclic(const std::vector<Bytes>& got, const std::vector<Bytes>& ref, std::size_t from = 0) {
    if (got.size() <= from || ref.empty()) return 0;
    for (std::size_t i0 = 0; i0 < ref.size(); ++i0) {
        if (ref[i0] != got[from]) continue;
        int m = 0;
        for (std::size_t j = from; j < got.size(); ++j)
            if (got[j] == ref[(i0 + (j - from)) % ref.size()]) ++m;
        return m;
    }
    return 0;
}

// ---- a scenario: the demo multiplex through a channel into the decoder
struct Channel {
    Channel()
        : seconds(20), snrDb(1000), freqHz(0), stickOffsetHz(0), echoGain(0), echoDelay(0), gapAt(-1), gapLen(0),
          selectAt(-1), selectSid(0), seed(7) {}
    double seconds;
    double snrDb;          // in the 1.536 MHz block (signal power / noise power there)
    double freqHz;         // the stick's frequency error
    double stickOffsetHz;  // the stick centred this far below the channel (dial on the channel)
    double echoGain;
    int echoDelay;         // samples
    double gapAt;          // seconds: samples lost here
    int gapLen;
    double selectAt;       // seconds: selectService(selectSid) here (0: before anything)
    unsigned selectSid;
    unsigned long long seed;
};

struct Result {
    std::string text;
    std::vector<dec::Decoder::EncodedImage> images;
    std::vector<dec::DabUnit> units, unitsAfterSelect;
    std::size_t pcm;
    double energy;   // sum of squares of the sound
    double cpu;
    dec::DabStats stats;
};

void run(dec::DabDecoder& d, dec::DabDemoMultiplex& mux, const Channel& c, Result& r) {
    const std::size_t block = 65536;
    std::vector<cf> sig(block), out(block), hist(static_cast<std::size_t>(c.echoDelay > 0 ? c.echoDelay : 1));
    std::size_t histPos = 0;
    Noise nz(c.seed);
    const double sigma = std::sqrt(std::pow(10.0, -c.snrDb / 10) / 2);   // (the block's carriers: power 0.75 of 2.048 MHz)
    const double sh = c.freqHz + c.stickOffsetHz, w = 2 * 3.14159265358979 * sh / kRate;
    double ph = 0;
    const double center = k12A - c.stickOffsetHz;
    const std::size_t total = static_cast<std::size_t>(c.seconds * kRate);
    const std::size_t gapAt = c.gapAt < 0 ? total + 1 : static_cast<std::size_t>(c.gapAt * kRate);
    const std::size_t selAt = c.selectAt < 0 ? total + 1 : static_cast<std::size_t>(c.selectAt * kRate);
    bool gapDone = false, selDone = false;
    r.pcm = 0;
    r.energy = 0;
    r.cpu = 0;
    mux.restart();
    for (std::size_t done = 0; done < total;) {
        if (!selDone && done >= selAt) {
            selDone = true;
            d.selectService(c.selectSid);
            std::vector<dec::DabUnit> u;
            d.takeUnits(u);
            r.units.insert(r.units.end(), u.begin(), u.end());
        }
        mux.generate(&sig[0], block);
        std::size_t n = block;
        for (std::size_t i = 0; i < block; ++i) {
            cf s = sig[i];
            if (c.echoDelay > 0) {   // a second transmitter: the same signal, later
                const cf e = hist[histPos];
                hist[histPos] = s;
                histPos = (histPos + 1) % hist.size();
                s += static_cast<float>(c.echoGain) * e;
            }
            if (sh != 0) {
                s *= cf(static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
                ph = std::fmod(ph + w, 2 * 3.14159265358979);
            }
            if (c.snrDb < 999) s += cf(static_cast<float>(sigma * nz.gauss()), static_cast<float>(sigma * nz.gauss()));
            out[i] = s;
        }
        std::size_t first = 0;
        if (!gapDone && done + block > gapAt) {   // samples lost on the way
            gapDone = true;
            first = std::min<std::size_t>(block, static_cast<std::size_t>(c.gapLen));
            n = block - first;
        }
        const std::clock_t t0 = std::clock();
        if (n) d.processIq(&out[first], n, kRate, center, k12A);
        r.cpu += double(std::clock() - t0) / CLOCKS_PER_SEC;
        done += block;
        std::string t;
        d.takeText(t);
        r.text += t;
        d.takeImages(r.images);
        std::vector<dec::DabUnit> u;
        d.takeUnits(u);
        (selDone ? r.unitsAfterSelect : r.units).insert((selDone ? r.unitsAfterSelect : r.units).end(), u.begin(), u.end());
        std::vector<float> pcm;
        d.takeAudio(pcm);
        r.pcm += pcm.size() / 2;
        for (std::size_t i = 0; i < pcm.size(); ++i) r.energy += double(pcm[i]) * pcm[i];
    }
    r.stats = d.stats();
}

void showText(const std::string& t) {
    std::size_t a = 0;
    int lines = 0;
    while (a < t.size() && lines < 14) {
        std::size_t b = t.find('\n', a);
        if (b == std::string::npos) b = t.size();
        note("| " + t.substr(a, b - a));
        a = b + 1;
        ++lines;
    }
}

std::vector<Bytes> bytesOf(const std::vector<dec::DabUnit>& u, bool aac) {
    std::vector<Bytes> v;
    for (std::size_t i = 0; i < u.size(); ++i)
        if (u[i].aac == aac) v.push_back(u[i].bytes);
    return v;
}

bool contains(const std::string& s, const std::string& x) { return s.find(x) != std::string::npos; }

// the decode of the Plus station in a scenario: AUs identical, superframes, text, pictures
void checkPlus(const std::string& tag, const Result& r, const std::vector<Bytes>& refAus, bool wantPictures, double minShare) {
    const std::vector<Bytes> aus = bytesOf(r.unitsAfterSelect, true);
    // (the first super frames after the switch may straddle it: matched from the 3rd unit on)
    const int m = matchCyclic(aus, refAus, aus.size() > 3 ? 3 : 0);
    const double share = aus.size() > 3 ? double(m) / double(aus.size() - 3) : 0;
    check(aus.size() > 3 && share >= minShare, tag + ": DAB+ audio units identical to those sent (AU CRCs right)",
          fmt("%.0f units, %.0f matched, super frames %.0f ok / %.0f bad", double(aus.size()), double(m),
              r.stats.superframesOk, r.stats.superframesBad) +
              fmt(", RS fixed %.0f bytes, AU CRC failures %.0f", r.stats.rsFixed, r.stats.unitsBad));
    if (wantPictures) {
        bool slide = false, logo = false;
        std::string names;
        for (std::size_t i = 0; i < r.images.size(); ++i) {
            const dec::Decoder::EncodedImage& im = r.images[i];
            const bool jpeg = im.bytes.size() > 4 && im.bytes[0] == 0xff && im.bytes[1] == 0xd8;
            const bool png = im.bytes.size() > 8 && im.bytes[0] == 0x89 && im.bytes[1] == 'P' && im.bytes[2] == 'N' && im.bytes[3] == 'G';
            const bool okb = (im.mime == "image/jpeg" && jpeg) || (im.mime == "image/png" && png);
            if (im.name.compare(0, 5, "slide") == 0 && okb) slide = true;
            if (im.name.compare(0, 4, "logo") == 0 && okb) logo = true;
            if (names.size() < 300) names += (names.empty() ? "" : "; ") + im.name + fmt(" (%.0f bytes)", double(im.bytes.size()));
        }
        check(slide, tag + ": a SlideShow picture (MOT, X-PAD), JPEG/PNG bytes", names);
        check(logo, tag + ": station logos from the SPI service", names);
    }
}

}  // namespace

int main() {
    using namespace dec;

    // ---- the pieces ----
    {
        const unsigned char abc[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
        check(dab::crc16(abc, 9) == (0x29b1 ^ 0xffff), "CRC-16 (CCITT, inverted) of \"123456789\"");
    }
    {
        Noise nz(3);
        unsigned char cw[120], orig[120];
        bool okAll = true, refused = true;
        for (int trial = 0; trial < 200; ++trial) {
            for (int i = 0; i < 110; ++i) orig[i] = static_cast<unsigned char>(nz.uniform() * 256);
            dab::rsEncode(orig, orig + 110);
            std::memcpy(cw, orig, 120);
            if (dab::rsDecode(cw, 120) != 0) okAll = false;
            const int ne = trial % 6 + 1;   // 1..6 errors
            int pos[6];
            for (int e = 0; e < ne; ++e) {
                bool fresh;
                do {
                    pos[e] = static_cast<int>(nz.uniform() * 120);
                    fresh = true;
                    for (int f = 0; f < e; ++f) fresh = fresh && pos[f] != pos[e];
                } while (!fresh);
                cw[pos[e]] = static_cast<unsigned char>(cw[pos[e]] ^ (1 + static_cast<int>(nz.uniform() * 255)));
            }
            const int r = dab::rsDecode(cw, 120);
            if (ne <= 5 && (r != ne || std::memcmp(cw, orig, 120) != 0)) okAll = false;
            if (ne == 6 && r >= 0 && std::memcmp(cw, orig, 120) == 0) refused = false;
        }
        check(okAll, "Reed-Solomon RS(120,110): 1-5 byte errors corrected (200 codewords)");
        check(refused, "Reed-Solomon: 6 errors never 'corrected' to the original");
    }
    {
        const unsigned char ebu[] = {'O', 'F', 'F', 0x82, 0x9a, ' ', ' '};
        check(dab::text(ebu, sizeof ebu, 0) == "OFF\xc3\xa9\xc3\xb1", "EBU Latin text to UTF-8 (0x82 = e acute, 0x9A = n tilde), trimmed");
        dab::SubChannel sc;
        sc.size = 36;
        sc.lev = 2;
        dab::Protection p;
        const bool e3a = dab::protectionOf(sc, p) && p.kbps == 48 && dab::puncturedLength(p.sections, p.nSections) == 36 * 64;
        sc.uep = true;
        sc.uepIndex = 7;
        sc.size = 35;
        const bool u7 = dab::protectionOf(sc, p) && p.kbps == 48;
        check(e3a && u7, "protection: EEP 3-A 36 CUs = 48 kbit/s (2304 bits), UEP index 7 = 48 kbit/s");
        DabChannel ch;
        check(dabChannelNear(227.5e6, ch) && std::string(ch.name) == "12C" && dabChannelByName("11c", ch) && ch.hz == 220.352e6 &&
                  !dabChannelNear(100e6, ch) && dabChannels().size() == 41,
              "channel table: 41 channels, 227.5 MHz is in 12C, 11C = 220.352 MHz");
    }
    DabDemoMultiplex mux;
    int sfOk = 0, rsOk = 0;
    const std::vector<Bytes> refAus = referenceAus(mux.plusData(), sfOk, rsOk);
    check(sfOk == 96 && rsOk == 96 && refAus.size() >= 96 * 2, "the demo's DAB+ data: 96 super frames, fire code and RS clean, AU CRCs",
          fmt("%.0f super frames ok, %.0f units", sfOk, double(refAus.size())));
    std::vector<Bytes> refMp2;
    for (std::size_t o = 0; o + 144 <= mux.mp2Data().size(); o += 144)
        refMp2.push_back(Bytes(mux.mp2Data().begin() + static_cast<std::ptrdiff_t>(o), mux.mp2Data().begin() + static_cast<std::ptrdiff_t>(o + 144)));

    std::printf("\ncodecs (liboffgrid-codecs) %s\n", DabDecoder().codecsAvailable() ? "built in: sound checked" : "not built in: decoded to audio units, no sound");

    // ---- clean: the list, the default station (MP2), then OFFgrid Plus ----
    {
        DabDecoder d;
        d.setKeepUnits(true);
        Channel c;
        c.seconds = 36;
        c.selectAt = 10;
        c.selectSid = DabDemoMultiplex::kPlusSid;
        Result r;
        run(d, mux, c, r);
        std::printf("\n-- clean signal, 36 s: \n");
        showText(r.text);
        const Status st = d.status();
        note("status: " + st.info);
        check(d.ensembleLabel() == "OFFgrid DAB" && d.ensembleId() == 0x0FF6, "clean: ensemble \"OFFgrid DAB\", ID 0FF6", d.ensembleLabel());
        const std::vector<Service> sv = d.services();
        bool plus = false, classic = false;
        for (std::size_t i = 0; i < sv.size(); ++i) {
            { char idb[16]; std::snprintf(idb, sizeof idb, "%04X ", sv[i].id); note(std::string("service ") + idb + sv[i].label + " -- " + sv[i].detail + (sv[i].selected ? "  [selected]" : "")); }
            if (sv[i].id == 0xF0D1 && sv[i].label == "OFFgrid Plus" && sv[i].bitrateKbps == 48 && contains(sv[i].detail, "DAB+") &&
                contains(sv[i].detail, "Pop Music"))
                plus = true;
            if (sv[i].id == 0xF0D2 && sv[i].label == "OFFgrid Classic" && sv[i].bitrateKbps == 48 && contains(sv[i].detail, "DAB ") &&
                contains(sv[i].detail, "Other Music"))
                classic = true;
        }
        check(sv.size() == 2 && plus && classic, "clean: two services, labels, DAB+ / DAB, 48 kbit/s, programme types");
        check(contains(r.text, "Station OFFgrid Classic"), "clean: default station = the first in the list (OFFgrid Classic)");
        const std::vector<Bytes> mp2 = bytesOf(r.units, false);
        const int m = matchCyclic(mp2, refMp2, mp2.size() > 2 ? 2 : 0);
        check(mp2.size() > 50 && m >= static_cast<int>(mp2.size()) - 3, "clean: MP2 frames identical to those sent (OFFgrid Classic, sub-channel 2, UEP)",
              fmt("%.0f frames, %.0f matched, %.0f refused", double(mp2.size()), double(m), r.stats.mp2Bad));
        check(d.selectedService() == 0xF0D1, "clean: selectService(F0D1) switched to OFFgrid Plus");
        checkPlus("clean", r, refAus, true, 0.999);
        check(contains(r.text, "OFFgrid Plus: ") && !d.dynamicLabel().empty(), "clean: dynamic label (DLS) text from OFFgrid Plus", d.dynamicLabel());
        check(r.stats.fibs > 0 && r.stats.fibsOk == r.stats.fibs, "clean: every FIB's CRC right",
              fmt("%.0f / %.0f", r.stats.fibsOk, r.stats.fibs));
        check(st.locked && st.snrDb > 30 && st.quality > 0.99, "clean: status locked, SNR, FIC quality", st.info);
        if (d.codecsAvailable())
            {
            const double rms = r.pcm ? std::sqrt(r.energy / (2.0 * double(r.pcm))) : 0;
            check(r.pcm > 48000 * 10 && r.stats.decodeErrors < 5 && rms > 0.01 && rms < 1, "clean: sound (PCM) from both stations, not silent",
                  fmt("%.0f stereo frames, now at %.0f Hz, %.0f codec errors, rms %.3f", double(r.pcm), d.audioRate(), r.stats.decodeErrors, rms));
        }
        else
            note("(sound not checked: liboffgrid-codecs not built in)");
        const double margin = c.seconds / r.cpu;
        check(margin > 3, "CPU: DAB+ station + FIC + SPI at 2.048 MS/s",
              fmt("%.1fx real time (%.1f%% of a core)", margin, 100 * r.cpu / c.seconds));
    }

    // ---- noise and a frequency error ----
    {
        DabDecoder d;
        d.setKeepUnits(true);
        Channel c;
        c.seconds = 30;
        c.snrDb = 10;
        c.freqHz = 12345;
        c.selectAt = 0;
        c.selectSid = DabDemoMultiplex::kPlusSid;
        c.seed = 11;
        Result r;
        run(d, mux, c, r);
        std::printf("\n-- 10 dB SNR, stick 12.345 kHz off:\n");
        note("status: " + d.status().info);
        check(d.ensembleLabel() == "OFFgrid DAB" && d.services().size() == 2, "noise+offset: ensemble and services");
        checkPlus("noise+offset", r, refAus, true, 0.99);
        check(!d.dynamicLabel().empty(), "noise+offset: DLS text", d.dynamicLabel());
        check(std::fabs(d.status().snrDb - 10) < 3, "noise+offset: SNR estimate near 10 dB", fmt("%.1f dB", d.status().snrDb));
    }
    {
        DabDecoder d;
        d.setKeepUnits(true);
        Channel c;
        c.seconds = 14;
        c.snrDb = 6;
        c.freqHz = -3210;
        c.selectAt = 0;
        c.selectSid = DabDemoMultiplex::kPlusSid;
        c.seed = 12;
        Result r;
        run(d, mux, c, r);
        std::printf("\n-- 6 dB SNR, stick 3.21 kHz off:\n");
        note("status: " + d.status().info);
        checkPlus("6 dB", r, refAus, false, 0.95);
    }
    // ---- the stick centred away from the channel (dial on it) ----
    {
        DabDecoder d;
        d.setKeepUnits(true);
        Channel c;
        c.seconds = 10;
        c.stickOffsetHz = 150e3;
        c.snrDb = 15;
        c.selectAt = 0;
        c.selectSid = DabDemoMultiplex::kPlusSid;
        Result r;
        run(d, mux, c, r);
        std::printf("\n-- stick centred 150 kHz below the channel:\n");
        note("status: " + d.status().info);
        checkPlus("stick off centre", r, refAus, false, 0.99);
    }
    // ---- a single-frequency network: a second transmitter, half as strong, 60 us later ----
    {
        DabDecoder d;
        d.setKeepUnits(true);
        Channel c;
        c.seconds = 12;
        c.snrDb = 15;
        c.echoGain = 0.5;
        c.echoDelay = 123;
        c.freqHz = 700;
        c.selectAt = 0;
        c.selectSid = DabDemoMultiplex::kPlusSid;
        Result r;
        run(d, mux, c, r);
        std::printf("\n-- SFN echo (0.5, 123 samples), 15 dB:\n");
        note("status: " + d.status().info);
        checkPlus("SFN", r, refAus, false, 0.99);
    }
    // ---- samples lost on the way ----
    {
        DabDecoder d;
        d.setKeepUnits(true);
        Channel c;
        c.seconds = 14;
        c.snrDb = 20;
        c.gapAt = 7.3;
        c.gapLen = 50000;
        c.selectAt = 0;
        c.selectSid = DabDemoMultiplex::kPlusSid;
        Result r;
        run(d, mux, c, r);
        std::printf("\n-- 50,000 samples lost at 7.3 s:\n");
        note("status: " + d.status().info);
        const DabStats s = d.stats();
        check(s.gaps >= 1 && s.lost == 0, "gap: bridged by the forgiving re-lock, lock kept",
              fmt("%.0f gaps, %.0f locks lost, %.0f super frames bad, %.0f resyncs", s.gaps, s.lost, s.superframesBad, s.resyncs));
        checkPlus("gap", r, refAus, false, 0.9);
    }

    // ---- noise alone; a wrong rate ----
    {
        DabDecoder d;
        Noise nz(99);
        std::vector<cf> blk(65536);
        std::string t;
        for (int b = 0; b < 320; ++b) {   // about 10 s
            for (std::size_t i = 0; i < blk.size(); ++i) blk[i] = cf(static_cast<float>(0.3 * nz.gauss()), static_cast<float>(0.3 * nz.gauss()));
            d.processIq(&blk[0], blk.size(), kRate, k12A, k12A);
            d.takeText(t);
        }
        std::vector<Decoder::EncodedImage> im;
        d.takeImages(im);
        check(t.empty() && d.services().empty() && im.empty() && !d.status().locked && d.stats().fibsOk == 0,
              "noise: nothing decoded from 10 s of noise", d.status().info);
        DabDecoder w;
        w.processIq(&blk[0], blk.size(), 2400000, k12A, k12A);
        check(w.status().info == "DAB needs 2.048 MS/s", "a rate other than 2.048 MS/s is refused (status says so)");
    }

    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
