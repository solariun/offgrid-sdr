// ============================================================================
//  offgrid-codecs - self-test of the AAC and MP2 decoders (part of OFFgrid-SDR
//  TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_codecs.cpp
//  Layer:  Tests
//
//  What this file does
//  -------------------
//  Self-checking test of liboffgrid-codecs.  Prints PASS/FAIL per check and
//  returns nonzero if anything failed.  Checks:
//    - the real streams in tests/data decode without errors, at the right
//      rate, channel count and frame count, with finite, sane-level samples:
//        codec_dabplus_aus.bin   OFFgrid-SDR's DAB+ demo station: AAC-LC,
//                                32 kHz mono, 960-sample frames (AUs cut
//                                from its super frames, CRCs checked)
//        codec_drm_aac.bin       its DRM demo station: DRM AAC, 24 kHz mono
//        codec_heaac_adts.aac    HE-AAC (SBR), 48 kHz stereo, ADTS
//        codec_heaacv2_adts.aac  HE-AAC v2 (SBR + parametric stereo), ADTS
//        codec_aac_stereo_adts.aac  AAC-LC stereo, 24 kHz core, ADTS
//        codec_dab_mp2.bin       the DAB demo's classic station: MP2 48 kHz mono
//        codec_mp2_stereo48.mp2, codec_mp2_stereo24.mp2, codec_mp2_mono24.mp2
//    - each frame's level (RMS per channel) matches the ORIGINAL decoders'
//      (FAAD2 and pl_mpeg, compiled as C from licenses/; their figures are in
//      codec_reference.txt) to 1e-4 relative; and the whole output's hash
//      says whether it is bit-identical to them (it is, with clang; another
//      compiler may round a little differently, so that line informs only);
//    - the same HE-AAC v2 AUs as raw AUs with explicit parameters (the DAB+ /
//      DRM way) decode like the ADTS stream;
//    - the configuration paths: AacParams::dabPlus's AudioSpecificConfig is
//      the one OFFgrid-SDR's browser version builds, bad configurations are
//      refused;
//    - damaged input: thousands of mutated AUs and frames (bit flips, cuts,
//      random bytes, odd lengths) in every configuration - no crash, finite
//      output, and afterwards the clean stream decodes exactly again (the
//      decoder state stays sane).  Build with -fsanitize=address,undefined
//      to make this a memory-safety test;
//    - speed: the real-time factor of each decoder.
//
//  How it fits in
//  --------------
//  Build and run from the repository root:
//    c++ -std=c++11 -O2 -Isrc src/codecs/*.cpp tests/test_codecs.cpp -o test_codecs
//    ./test_codecs [data directory, default tests/data] [fuzz rounds, default 3000]
//
//  Contributing
//  ------------
//  A new stream: put it in tests/data as codec_*, add its reference figures
//  (from the original decoder) to codec_reference.txt, add a line below.
// ============================================================================
#include "codecs/Aac.h"
#include "codecs/Mp2.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <map>
#include <string>
#include <vector>

namespace {

int g_failures = 0, g_checks = 0;
std::string g_dir = "tests/data";

void check(bool ok, const std::string &name, const std::string &detail = "") {
    ++g_checks;
    if (!ok) ++g_failures;
    std::printf("%s  %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : "  -- ", detail.c_str());
}

std::string fmt(const char *f, double a = 0, double b = 0, double c = 0, double d = 0) {
    char buf[256];
    std::snprintf(buf, sizeof buf, f, a, b, c, d);
    return buf;
}

std::vector<unsigned char> readFile(const std::string &name) {
    std::vector<unsigned char> b;
    std::FILE *f = std::fopen((g_dir + "/" + name).c_str(), "rb");
    if (!f) return b;
    unsigned char tmp[65536];
    std::size_t n;
    while ((n = std::fread(tmp, 1, sizeof tmp, f)) > 0) b.insert(b.end(), tmp, tmp + n);
    std::fclose(f);
    return b;
}

// ---- the reference figures (codec_reference.txt) ----
struct RefFrame { long samples; double rms[2]; };
struct RefStream { std::vector<RefFrame> frames; long samples; unsigned rate, channels; unsigned long long fnv; };
std::map<std::string, RefStream> g_ref;

bool loadReference() {
    std::vector<unsigned char> t = readFile("codec_reference.txt");
    if (t.empty()) return false;
    std::string s(t.begin(), t.end());
    std::size_t p = 0;
    std::string cur;
    while (p < s.size()) {
        std::size_t e = s.find('\n', p);
        if (e == std::string::npos) e = s.size();
        std::string line = s.substr(p, e - p);
        p = e + 1;
        char name[64];
        RefFrame f;
        int idx;
        long n;
        unsigned rate, ch;
        unsigned long long fnv;
        int frames;
        if (std::sscanf(line.c_str(), "stream %63s", name) == 1) {
            cur = name;
            g_ref[cur] = RefStream();
        } else if (std::sscanf(line.c_str(), "f %d %ld %lf %lf", &idx, &f.samples, &f.rms[0], &f.rms[1]) == 4) {
            g_ref[cur].frames.push_back(f);
        } else if (std::sscanf(line.c_str(), "end %63s frames %d samples %ld rate %u channels %u fnv %llx", name, &frames, &n, &rate,
                               &ch, &fnv) == 6) {
            RefStream &r = g_ref[cur];
            r.samples = n;
            r.rate = rate;
            r.channels = ch;
            r.fnv = fnv;
        }
    }
    return !g_ref.empty();
}

// ---- what a decoded stream gave ----
struct Result {
    std::vector<std::vector<float> > frames;   // each AU's / frame's samples
    int errors;
    unsigned rate, channels;
    double seconds;                            // CPU time decoding
    Result() : errors(0), rate(0), channels(0), seconds(0) {}
};

unsigned long long fnvOf(const Result &r) {
    unsigned long long h = 1469598103934665603ULL;
    for (std::size_t i = 0; i < r.frames.size(); i++)
        for (std::size_t j = 0; j < r.frames[i].size(); j++) {
            std::uint32_t u;
            std::memcpy(&u, &r.frames[i][j], 4);
            for (int k = 0; k < 4; k++) {
                h ^= (u >> (8 * k)) & 255;
                h *= 1099511628211ULL;
            }
        }
    return h;
}

void rmsOf(const std::vector<float> &x, unsigned ch, double out[2]) {
    double s[2] = {0, 0};
    for (std::size_t j = 0; j < x.size(); j++) s[(j % ch) > 0 ? 1 : 0] += (double)x[j] * x[j];
    std::size_t m = ch ? x.size() / ch : 0;
    out[0] = m ? std::sqrt(s[0] / m) : 0;
    out[1] = m ? std::sqrt(s[1] / m) : 0;
}

// Compare a decoded stream with its reference and check that it is sane.
void judge(const std::string &name, const Result &r, unsigned wantRate, double audioSeconds) {
    std::map<std::string, RefStream>::const_iterator it = g_ref.find(name);
    if (it == g_ref.end()) {
        check(false, name + ": reference figures", "missing from codec_reference.txt");
        return;
    }
    const RefStream &ref = it->second;
    long total = 0;
    bool finite = true;
    double peak = 0, sum2 = 0;
    for (std::size_t i = 0; i < r.frames.size(); i++)
        for (std::size_t j = 0; j < r.frames[i].size(); j++) {
            float v = r.frames[i][j];
            if (!(v == v) || std::fabs(v) > 1e6f) finite = false;
            if (std::fabs(v) > peak) peak = std::fabs(v);
            sum2 += (double)v * v;
            total++;
        }
    double rms = total ? std::sqrt(sum2 / total) : 0;
    check(r.errors == 0, name + ": every frame decodes", fmt("%.0f errors in %.0f frames", r.errors, (double)r.frames.size()));
    check(r.rate == wantRate && r.channels == 2, name + ": rate and channels", fmt("%.0f Hz, %.0f channels", r.rate, r.channels));
    check(r.frames.size() == ref.frames.size() && total == ref.samples, name + ": frame and sample count as the original decoder",
          fmt("%.0f frames, %.0f samples (original: %.0f, %.0f)", (double)r.frames.size(), (double)total, (double)ref.frames.size(),
              (double)ref.samples));
    check(finite && rms > 0.01 && rms < 0.7 && peak < 2.0, name + ": samples finite, level sane", fmt("RMS %.3f, peak %.3f", rms, peak));
    // frame by frame against the original
    double worst = 0;
    std::size_t n = r.frames.size() < ref.frames.size() ? r.frames.size() : ref.frames.size(), bad = 0;
    for (std::size_t i = 0; i < n; i++) {
        double got[2];
        rmsOf(r.frames[i], 2, got);
        if ((long)r.frames[i].size() != ref.frames[i].samples) {
            bad++;
            continue;
        }
        for (int c = 0; c < 2; c++) {
            double want = ref.frames[i].rms[c];
            double rel = std::fabs(got[c] - want) / (want > 1e-3 ? want : 1e-3);
            if (rel > worst) worst = rel;
            if (rel > 1e-4) bad++;
        }
    }
    check(bad == 0 && n == ref.frames.size(), name + ": each frame's level matches the original decoder",
          fmt("worst relative difference %.2g over %.0f frames", worst, (double)n));
    bool exact = fnvOf(r) == ref.fnv;
    std::printf("info  %s: %s the original decoder's output (hash %016llx)\n", name.c_str(),
                exact ? "bit-identical to" : "NOT bit-identical (rounding) to", fnvOf(r));
    if (audioSeconds > 0 && r.seconds > 0)
        std::printf("info  %s: %.1f s of audio decoded in %.3f s CPU: %.0fx real time\n", name.c_str(), audioSeconds, r.seconds,
                    audioSeconds / r.seconds);
}

// ---- the AU containers ----
// codec_dabplus_aus.bin: "DABP", u32 dac rate, sbr, stereo, ps, ASC length, ASC, then [u16 length][AU]...
// codec_drm_aac.bin: [u16 length][CRC byte + audio frame]...   (both little-endian lengths)
std::vector<std::vector<unsigned char> > lengthPrefixed(const std::vector<unsigned char> &b, std::size_t p) {
    std::vector<std::vector<unsigned char> > out;
    while (p + 2 <= b.size()) {
        std::size_t L = b[p] | (b[p + 1] << 8);
        if (p + 2 + L > b.size()) break;
        out.push_back(std::vector<unsigned char>(b.begin() + p + 2, b.begin() + p + 2 + L));
        p += 2 + L;
    }
    return out;
}

std::vector<std::vector<unsigned char> > adtsFrames(const std::vector<unsigned char> &b) {
    std::vector<std::vector<unsigned char> > out;
    std::size_t p = 0;
    while (p + 7 <= b.size() && b[p] == 0xff && (b[p + 1] & 0xf0) == 0xf0) {
        std::size_t L = ((b[p + 3] & 3) << 11) | (b[p + 4] << 3) | (b[p + 5] >> 5);
        if (L < 7 || p + L > b.size()) break;
        out.push_back(std::vector<unsigned char>(b.begin() + p, b.begin() + p + L));
        p += L;
    }
    return out;
}

std::vector<std::vector<unsigned char> > mp2Frames(const std::vector<unsigned char> &b) {
    std::vector<std::vector<unsigned char> > out;
    std::size_t p = 0;
    while (p + 4 <= b.size()) {
        std::size_t L = codec::Mp2Decoder::frameSize(&b[p], b.size() - p);
        if (!L || p + L > b.size()) break;
        out.push_back(std::vector<unsigned char>(b.begin() + p, b.begin() + p + L));
        p += L;
    }
    return out;
}

Result decodeAac(codec::AacDecoder &d, const std::vector<std::vector<unsigned char> > &aus) {
    Result r;
    std::vector<float> pcm;
    std::clock_t t0 = std::clock();
    for (std::size_t i = 0; i < aus.size(); i++) {
        if (!d.decode(aus[i], pcm)) r.errors++;
        r.frames.push_back(pcm);
    }
    r.seconds = (double)(std::clock() - t0) / CLOCKS_PER_SEC;
    r.rate = d.sampleRate();
    r.channels = d.channels();
    return r;
}

Result decodeMp2(codec::Mp2Decoder &d, const std::vector<std::vector<unsigned char> > &frames) {
    Result r;
    std::vector<float> pcm;
    std::clock_t t0 = std::clock();
    for (std::size_t i = 0; i < frames.size(); i++) {
        if (!d.decode(frames[i], pcm)) r.errors++;
        r.frames.push_back(pcm);
    }
    r.seconds = (double)(std::clock() - t0) / CLOCKS_PER_SEC;
    r.rate = d.sampleRate();
    r.channels = d.channels();
    return r;
}

double samplesSeconds(const Result &r) {
    double n = 0;
    for (std::size_t i = 0; i < r.frames.size(); i++) n += r.frames[i].size();
    return r.rate && r.channels ? n / r.channels / r.rate : 0;
}

// ---- a small deterministic random generator for the damage tests ----
struct Rng {
    unsigned long long s;
    explicit Rng(unsigned long long seed) : s(seed) {}
    unsigned next() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return (unsigned)(s >> 33);
    }
    unsigned below(unsigned n) { return n ? next() % n : 0; }
};

// One damaged copy of a unit: bit flips, a byte burst, a cut, an extension
// with random bytes, all random, or a few bytes only.
std::vector<unsigned char> damage(const std::vector<unsigned char> &in, Rng &g) {
    std::vector<unsigned char> b = in;
    switch (g.below(7)) {
    case 0: {
        unsigned n = 1 + g.below(8);
        for (unsigned i = 0; i < n && !b.empty(); i++) b[g.below((unsigned)b.size())] ^= (unsigned char)(1u << g.below(8));
        break;
    }
    case 1: {
        if (b.empty()) break;
        std::size_t at = g.below((unsigned)b.size()), n = 1 + g.below(16);
        for (std::size_t i = at; i < at + n && i < b.size(); i++) b[i] = (unsigned char)g.next();
        break;
    }
    case 2:
        b.resize(g.below((unsigned)b.size() + 1));
        break;
    case 3: {
        std::size_t n = 1 + g.below(400);
        for (std::size_t i = 0; i < n; i++) b.push_back((unsigned char)g.next());
        break;
    }
    case 4:
        for (std::size_t i = 0; i < b.size(); i++) b[i] = (unsigned char)g.next();
        break;
    case 5:
        b.resize(1 + g.below(4));
        for (std::size_t i = 0; i < b.size(); i++) b[i] = (unsigned char)g.next();
        break;
    default:
        for (std::size_t i = g.below((unsigned)b.size() + 1); i < b.size(); i++) b[i] = 0xff;
        break;
    }
    return b;
}

bool allFinite(const std::vector<float> &x) {
    for (std::size_t i = 0; i < x.size(); i++)
        if (!(x[i] == x[i]) || std::fabs(x[i]) > 1e9f) return false;
    return true;
}

// ============================================================================

std::vector<std::vector<unsigned char> > g_dabAus, g_drmAus, g_heAdts, g_he2Adts, g_lcAdts, g_he2Raw;
std::vector<unsigned char> g_dabAsc;

void testAacStreams() {
    std::printf("\n-- AAC (FAAD2 port) --\n");
    // DAB+
    std::vector<unsigned char> b = readFile("codec_dabplus_aus.bin");
    check(b.size() > 12 && std::memcmp(&b[0], "DABP", 4) == 0, "codec_dabplus_aus.bin present");
    if (b.size() > 12) {
        unsigned dac = b[4] | (b[5] << 8) | (b[6] << 16) | ((unsigned)b[7] << 24);
        bool sbr = b[8] != 0, stereo = b[9] != 0, ps = b[10] != 0;
        g_dabAsc.assign(b.begin() + 12, b.begin() + 12 + b[11]);
        g_dabAus = lengthPrefixed(b, 12 + b[11]);
        codec::AacParams p = codec::AacParams::dabPlus(dac, sbr, stereo, ps);
        std::vector<unsigned char> asc = codec::AacDecoder::makeAsc(p);
        std::string hex;
        for (std::size_t i = 0; i < asc.size(); i++) {
            hex += "0123456789abcdef"[asc[i] >> 4];
            hex += "0123456789abcdef"[asc[i] & 15];
        }
        check(asc == g_dabAsc, "AacParams::dabPlus gives the browser version's AudioSpecificConfig", hex);
        codec::AacDecoder d;
        check(d.init(p), "DAB+ decoder set up from explicit parameters (32 kHz, AAC-LC, 960)");
        Result r = decodeAac(d, g_dabAus);
        judge("dabplus", r, 32000, samplesSeconds(r));
        codec::AacDecoder d2;
        check(d2.init(g_dabAsc), "DAB+ decoder set up from the AudioSpecificConfig bytes");
        Result r2 = decodeAac(d2, g_dabAus);
        check(fnvOf(r2) == fnvOf(r), "the two set-ups decode identically");
    }
    // DRM
    b = readFile("codec_drm_aac.bin");
    g_drmAus = lengthPrefixed(b, 0);
    check(g_drmAus.size() == 300, "codec_drm_aac.bin: 300 audio frames", fmt("%.0f", (double)g_drmAus.size()));
    {
        codec::AacDecoder d;
        check(d.init(codec::AacParams::drmAudio(24000, false, false, false)), "DRM decoder set up (24 kHz, mono, no SBR)");
        Result r = decodeAac(d, g_drmAus);
        judge("drm", r, 24000, samplesSeconds(r));
    }
    // ADTS streams
    struct { const char *file, *name; std::vector<std::vector<unsigned char> > *keep; } adts[] = {
        {"codec_heaac_adts.aac", "heaac", &g_heAdts},
        {"codec_heaacv2_adts.aac", "heaacv2", &g_he2Adts},
        {"codec_aac_stereo_adts.aac", "aacstereo", &g_lcAdts},
    };
    for (int i = 0; i < 3; i++) {
        *adts[i].keep = adtsFrames(readFile(adts[i].file));
        const std::vector<std::vector<unsigned char> > &fr = *adts[i].keep;
        codec::AacDecoder d;
        check(!fr.empty() && d.initAdts(&fr[0][0], fr[0].size()), std::string(adts[i].file) + ": set up from its ADTS header");
        Result r = decodeAac(d, fr);
        judge(adts[i].name, r, 48000, samplesSeconds(r));
        if (i == 1) check(d.sbr() && d.ps(), "heaacv2: SBR and parametric stereo in use", fmt("sbr %.0f ps %.0f", d.sbr(), d.ps()));
        if (i == 0) check(d.sbr() && !d.ps(), "heaac: SBR in use, no PS");
    }
    // HE-AAC v2 as raw AUs with explicit parameters (the DAB+/DRM way of signalling)
    if (!g_he2Adts.empty()) {
        std::vector<std::vector<unsigned char> > raw;
        for (std::size_t i = 0; i < g_he2Adts.size(); i++) {
            std::size_t h = (g_he2Adts[i][1] & 1) ? 7 : 9;
            raw.push_back(std::vector<unsigned char>(g_he2Adts[i].begin() + h, g_he2Adts[i].end()));
        }
        g_he2Raw = raw;
        codec::AacParams p;
        p.sampleRate = 24000;
        p.channels = 1;
        p.sbr = true;
        p.ps = true;
        p.frameLength = 1024;
        codec::AacDecoder d;
        check(d.init(p), "HE-AAC v2 set up from explicit parameters (24 kHz core, SBR, PS, 1024)");
        Result r = decodeAac(d, raw);
        judge("heaacv2", r, 48000, 0);
    }
}

void testAacConfig() {
    std::printf("\n-- AAC set-up --\n");
    codec::AacDecoder d;
    std::vector<float> pcm;
    unsigned char au[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    check(!d.decode(au, sizeof au, pcm) && pcm.empty() && !d.ready(), "decode before set-up is refused");
    codec::AacParams p;
    p.frameLength = 512;
    check(!d.init(p), "frame length 512 refused");
    p = codec::AacParams::drmAudio(22050, false, false, false);
    check(!d.init(p), "DRM at 22.05 kHz refused");
    unsigned char badAsc[2] = {0xf8, 0x00};
    check(!d.init(badAsc, 2), "an AudioSpecificConfig of an unsupported object type refused");
    check(!d.init(badAsc, 1), "a 1-byte AudioSpecificConfig refused");
    // each DRM channel mode, and DAB+ with every flag, set up
    bool all = true;
    unsigned rates[3] = {12000, 24000, 48000};
    for (int r = 0; r < 3; r++)
        for (int m = 0; m < 5; m++) {
            bool sbr = m >= 2, stereo = m == 1 || m == 3, ps = m == 4;
            if (!d.init(codec::AacParams::drmAudio(rates[r], sbr, stereo, ps))) all = false;
        }
    for (int m = 0; m < 16; m++)
        if (!d.init(codec::AacParams::dabPlus(m & 1 ? 48000 : 32000, (m & 2) != 0, (m & 4) != 0, (m & 8) != 0))) all = false;
    check(all, "every DRM (3 rates x 5 modes) and DAB+ (16 flag combinations) configuration sets up");
    d.init(codec::AacParams::dabPlus(48000, true, false, true));
    check(d.sampleRate() == 48000, "DAB+ 48 kHz with SBR: output rate 48 kHz", fmt("%.0f", d.sampleRate()));
}

// Thousands of damaged units through every configuration; then the clean
// stream again, which must decode exactly as at first.
void testAacDamage(int rounds) {
    std::printf("\n-- AAC: damaged input --\n");
    struct Case { const char *name; codec::AacParams p; int adts; const std::vector<std::vector<unsigned char> > *aus; };
    std::vector<Case> cases;
    Case c;
    c.adts = 0;
    c.name = "DAB+ AAC-LC 32 kHz";
    c.p = codec::AacParams::dabPlus(32000, false, false, false);
    c.aus = &g_dabAus;
    cases.push_back(c);
    c.name = "DAB+ HE-AAC v2 48 kHz (960, SBR + PS)";
    c.p = codec::AacParams::dabPlus(48000, true, false, true);
    cases.push_back(c);
    c.name = "DAB+ HE-AAC v2 48 kHz (960, SBR + PS), fed HE-AAC v2 AUs";
    c.aus = &g_he2Raw;
    cases.push_back(c);
    c.aus = &g_dabAus;
    c.name = "DAB+ HE-AAC stereo 32 kHz (960, SBR)";
    c.p = codec::AacParams::dabPlus(32000, true, true, false);
    cases.push_back(c);
    c.name = "DRM AAC 24 kHz mono";
    c.p = codec::AacParams::drmAudio(24000, false, false, false);
    c.aus = &g_drmAus;
    cases.push_back(c);
    c.name = "DRM AAC 24 kHz SBR + PS";
    c.p = codec::AacParams::drmAudio(24000, true, false, true);
    cases.push_back(c);
    c.name = "DRM AAC 12 kHz SBR stereo";
    c.p = codec::AacParams::drmAudio(12000, true, true, false);
    cases.push_back(c);
    c.name = "HE-AAC v2 ADTS";
    c.adts = 1;
    c.aus = &g_he2Adts;
    cases.push_back(c);
    c.name = "AAC-LC stereo ADTS";
    c.aus = &g_lcAdts;
    cases.push_back(c);

    Rng g(20261125);
    for (std::size_t k = 0; k < cases.size(); k++) {
        const std::vector<std::vector<unsigned char> > &aus = *cases[k].aus;
        if (aus.empty()) continue;
        codec::AacDecoder d;
        bool ok = cases[k].adts ? d.initAdts(&aus[0][0], aus[0].size()) : d.init(cases[k].p);
        if (!ok) {
            check(false, std::string(cases[k].name) + ": set up");
            continue;
        }
        std::vector<float> pcm;
        int n = rounds / (int)cases.size() + 1, good = 0, refused = 0;
        bool finite = true;
        std::clock_t t0 = std::clock();
        for (int i = 0; i < n; i++) {
            std::vector<unsigned char> au = damage(aus[g.below((unsigned)aus.size())], g);
            if (d.decode(au, pcm)) {
                good++;
                if (!allFinite(pcm)) finite = false;
            } else {
                refused++;
                if (!pcm.empty()) finite = false;
            }
        }
        double secs = (double)(std::clock() - t0) / CLOCKS_PER_SEC;
        check(finite, std::string(cases[k].name) + ": " + fmt("%.0f damaged AUs, no crash, output finite", n),
              fmt("%.0f decoded, %.0f refused, %.2f s", good, refused, secs));
    }
    // damaged and clean AUs interleaved (as on a weak signal): every clean AU must still decode. (Without the
    // decoder's rebuild on FAAD2's error 23, one damaged DAB+ AU that looks like SBR data stops all that follow.)
    for (int which = 0; which < 2; which++) {
        const std::vector<std::vector<unsigned char> > &aus = which ? g_drmAus : g_dabAus;
        if (aus.empty()) continue;
        codec::AacDecoder d;
        d.init(which ? codec::AacParams::drmAudio(24000, false, false, false) : codec::AacParams::dabPlus(32000, false, false, false));
        std::vector<float> pcm;
        int clean = 0, cleanBad = 0;
        for (int i = 0; i < 2000; i++) {
            std::vector<unsigned char> au = aus[g.below((unsigned)aus.size())];
            bool damaged = (i & 1) != 0;
            if (damaged)
                for (int k = 0; k < 3; k++) au[g.below((unsigned)au.size())] ^= (unsigned char)(1u << g.below(8));
            bool ok = d.decode(au, pcm);
            if (!damaged) {
                clean++;
                if (!ok) cleanBad++;
            }
        }
        check(cleanBad == 0, std::string(which ? "DRM" : "DAB+") + ": clean AUs between damaged ones all decode",
              fmt("%.0f of %.0f failed; decoder rebuilt %.0f times", cleanBad, clean, d.restarts()));
    }
    // recovery: a decoder that has eaten garbage decodes the clean DAB+ and DRM streams exactly as a fresh one
    for (int which = 0; which < 2; which++) {
        const std::vector<std::vector<unsigned char> > &aus = which ? g_drmAus : g_dabAus;
        if (aus.empty()) continue;
        codec::AacParams p = which ? codec::AacParams::drmAudio(24000, false, false, false) : codec::AacParams::dabPlus(32000, false, false, false);
        codec::AacDecoder fresh, used;
        fresh.init(p);
        used.init(p);
        std::vector<float> pcm;
        for (int i = 0; i < 300; i++) used.decode(damage(aus[g.below((unsigned)aus.size())], g), pcm);
        Result a = decodeAac(fresh, aus), b = decodeAac(used, aus);
        // the overlap-add of the first clean frame still holds the garbage's tail: compare from the third frame
        std::size_t same = 0, cmp = 0;
        for (std::size_t i = 2; i < a.frames.size() && i < b.frames.size(); i++, cmp++)
            if (a.frames[i] == b.frames[i]) same++;
        check(b.errors == 0 && same == cmp, std::string(which ? "DRM" : "DAB+") + ": after 300 damaged AUs, the clean stream decodes exactly as on a fresh decoder",
              fmt("%.0f of %.0f frames identical, %.0f errors", same, cmp, b.errors));
    }
}

std::vector<std::vector<unsigned char> > g_mp2[4];

void testMp2() {
    std::printf("\n-- MP2 (pl_mpeg port) --\n");
    const char *files[4] = {"codec_dab_mp2.bin", "codec_mp2_stereo48.mp2", "codec_mp2_stereo24.mp2", "codec_mp2_mono24.mp2"};
    const char *names[4] = {"mp2dab", "mp2stereo48", "mp2stereo24", "mp2mono24"};
    unsigned rates[4] = {48000, 48000, 24000, 24000};
    codec::Mp2Decoder::Mode modes[4] = {codec::Mp2Decoder::Mono, codec::Mp2Decoder::Stereo, codec::Mp2Decoder::Stereo, codec::Mp2Decoder::Mono};
    for (int i = 0; i < 4; i++) {
        g_mp2[i] = mp2Frames(readFile(files[i]));
        check(!g_mp2[i].empty(), std::string(files[i]) + ": frames found", fmt("%.0f", (double)g_mp2[i].size()));
        codec::Mp2Decoder d;
        Result r = decodeMp2(d, g_mp2[i]);
        judge(names[i], r, rates[i], samplesSeconds(r));
        check(d.mode() == modes[i] && d.mpeg2() == (rates[i] == 24000), std::string(names[i]) + ": mode and MPEG version");
    }
    // DAB: the PAD at each frame's end (48 kbit/s mono at 48 kHz: 144-byte frames, ScF-CRC 2 bytes, then F-PAD)
    if (!g_mp2[0].empty()) {
        codec::Mp2Decoder d;
        std::vector<float> pcm;
        bool ok = true;
        std::size_t xpadFrames = 0;
        for (std::size_t i = 0; i < g_mp2[0].size(); i++) {
            const std::vector<unsigned char> &f = g_mp2[0][i];
            if (!d.decode(f, pcm)) { ok = false; continue; }
            const codec::Mp2Decoder::DabPad &pad = d.dabPad();
            if (!pad.valid || pad.scfCrcBytes != 2 || pad.xpadEnd != 140 || pad.fpad[0] != f[142] || pad.fpad[1] != f[143]) ok = false;
            const std::vector<unsigned char> &a = d.ancillary();
            if (d.audioBytes() + a.size() != d.frameBytes() || d.frameBytes() != 144) ok = false;
            if (a.size() && std::memcmp(&a[0], &f[d.audioBytes()], a.size()) != 0) ok = false;
            if ((pad.fpad[0] >> 6) == 0 && (pad.fpad[0] >> 4 & 3) != 0) xpadFrames++;   // F-PAD type 00 with an X-PAD indicator
        }
        check(ok, "mp2dab: ancillary bytes and DAB PAD split (F-PAD = last 2 bytes, ScF-CRC 2, X-PAD field up to byte 140)");
        std::printf("info  mp2dab: %zu of %zu frames announce X-PAD in their F-PAD; ancillary bytes after the audio: %zu in the last frame\n",
                    xpadFrames, g_mp2[0].size(), d.ancillary().size());
    }
    // refusals
    codec::Mp2Decoder d;
    std::vector<float> pcm;
    unsigned char hdr[4] = {0xff, 0xfd, 0x24, 0xc4};   // MPEG-1 layer II 48 kbit/s 48 kHz mono
    check(codec::Mp2Decoder::frameSize(hdr, 4) == 144, "frameSize of a 48 kbit/s 48 kHz header: 144 bytes");
    check(!d.decode(hdr, 4, pcm) && pcm.empty(), "a frame shorter than its header says is refused");
    unsigned char freeFmt[4] = {0xff, 0xfd, 0x04, 0xc4};
    check(codec::Mp2Decoder::frameSize(freeFmt, 4) == 0, "free-format bit rate refused");
    unsigned char layer3[4] = {0xff, 0xfb, 0x24, 0xc4};
    check(codec::Mp2Decoder::frameSize(layer3, 4) == 0, "layer III refused");
}

void testMp2Damage(int rounds) {
    std::printf("\n-- MP2: damaged input --\n");
    Rng g(77);
    for (int k = 0; k < 4; k++) {
        if (g_mp2[k].empty()) continue;
        codec::Mp2Decoder d;
        std::vector<float> pcm;
        int n = rounds / 4 + 1, good = 0, refused = 0;
        bool finite = true;
        for (int i = 0; i < n; i++) {
            std::vector<unsigned char> f = damage(g_mp2[k][g.below((unsigned)g_mp2[k].size())], g);
            if (d.decode(f, pcm)) {
                good++;
                if (!allFinite(pcm) || pcm.size() != 2304) finite = false;
            } else {
                refused++;
                if (!pcm.empty()) finite = false;
            }
        }
        check(finite, fmt("MP2 stream %.0f: ", k) + fmt("%.0f damaged frames, no crash, output finite", n), fmt("%.0f decoded, %.0f refused", good, refused));
    }
    // after garbage, reset() gives a decoder that decodes exactly like a fresh one
    if (!g_mp2[0].empty()) {
        codec::Mp2Decoder fresh, used;
        std::vector<float> pcm;
        for (int i = 0; i < 200; i++) used.decode(damage(g_mp2[0][g.below((unsigned)g_mp2[0].size())], g), pcm);
        used.reset();
        Result a = decodeMp2(fresh, g_mp2[0]), b = decodeMp2(used, g_mp2[0]);
        check(fnvOf(a) == fnvOf(b), "MP2: after damaged frames and reset(), the clean stream decodes exactly as on a fresh decoder");
    }
}

}  // namespace

int main(int argc, char **argv) {
    if (argc > 1) g_dir = argv[1];
    int rounds = argc > 2 ? std::atoi(argv[2]) : 3000;
    std::printf("offgrid-codecs self-test (data: %s)\n", g_dir.c_str());
    if (!loadReference()) {
        std::FILE *f = std::fopen(("../" + g_dir + "/codec_reference.txt").c_str(), "rb");
        if (f) {
            std::fclose(f);
            g_dir = "../" + g_dir;
            loadReference();
        }
    }
    check(!g_ref.empty(), "reference figures loaded (codec_reference.txt)", fmt("%.0f streams", (double)g_ref.size()));
    testAacStreams();
    testAacConfig();
    testMp2();
    testAacDamage(rounds);
    testMp2Damage(rounds);
    std::printf("\n%d checks, %d failed: %s\n", g_checks, g_failures, g_failures ? "FAIL" : "ALL PASS");
    return g_failures ? 1 : 0;
}
