// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Drm.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  DrmDecoder (Drm.h): the front end (the stick's samples near the dial:
//  mixed down, averaged to about 240 kHz, low-passed and resampled to exactly
//  48 kHz, by interpolating between filtered samples about 5 times
//  oversampled), a 6 s history, and every 1.2 s (a superframe) the last 6 s
//  (3.6 s at first) through the receiver (DrmRx.cpp): the station's name and
//  settings, then the main channel's multiplex frames not decoded before ->
//  the selected service's audio super frames (AAC access units -> the AAC
//  decoder; xHE-AAC frames, recognised only), the text message, the
//  SlideShow and Journaline streams.
//
//  Port of OFFgrid-SDR's JS (diddy-boy, https://github.com/solariun/
//  offgrid-sdr, index.html: `class DrmFront`, drmFeed, the DRM worker's
//  onmessage, drmAudioUnsupported, drmTextFeed, drmSlideFeed, drmJlFeed,
//  faadFor, drmGot), which credits Dream (TU Darmstadt, Volker Fischer and
//  others; GPL-2.0-or-later, licenses/dream-COPYING.txt).
//
//  Differences from the JS:
//    - Addition to the JS: windows are taken every 57,600 samples of signal
//      (1.2 s at 48 kHz) instead of every 1.2 s of wall-clock time with the
//      worker idle: the same rhythm, deterministic, and nothing is skipped
//      on a slow computer (the analysis runs inside processIq());
//    - Addition to the JS: the front end's low-pass reaches 16 kHz (the JS:
//      11 kHz, which cut the upper carriers of 18/20 kHz signals, reaching
//      +13.5/+15 kHz of the reference), and non-finite samples are 0;
//    - Addition to the JS: all four services are listed (label, language,
//      programme type, codec) and any can be chosen; the chosen service's
//      own audio stream is used (the JS: stream 0); the main channel is
//      decoded whenever it can be (16/64-QAM, equal protection), so the
//      text message and pictures work for stations whose sound can't be
//      played (xHE-AAC, CELP, no AAC decoder) (the JS decoded it only
//      while it could play the sound);
//    - Addition to the JS: what is known about the station is merged
//      window by window and kept through a fade (the JS kept the last
//      labelled window);
//    - the JS restarted the sound (re-decoding the window's frames) when the
//      AAC settings changed; here decoding just carries on with the new
//      settings.
//
//  How it fits in
//  --------------
//  Implements Drm.h on DrmInternal.h; the AAC decoder through
//  drm::makeAacCodec() (DrmAac.cpp).
//
//  Contributing
//  ------------
//  No allocation per sample (the front end and the history are fixed); the
//  per-window work reuses the receiver's buffers.
// ============================================================================
#include "decoders/Drm.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>

#include "decoders/DrmInternal.h"

namespace dec {

namespace {

const double kPi = 3.14159265358979323846;
const int kTaps = 63;
const std::size_t kHistory = 48000 * 6;      // the window: 6 s
const long long kFirstWindow = 48000 * 36 / 10;  // 3.6 s, for a quick name
const long long kStep = drm::kSuperframeSamples;  // every superframe

const char* mscName(int code) {
    return code == 3 ? "16-QAM" : code == 0 ? "64-QAM" : "64-QAM (hier.)";
}
const char* codecName(int c) {
    static const char* const n[4] = {"AAC", "CELP", "HVXC", "xHE-AAC"};
    return c >= 0 && c < 4 ? n[c] : "";
}
const char* stereoName(int m) { return m == 2 ? "stereo" : m == 1 ? "parametric stereo" : "mono"; }

std::string fmt(const char* f, double v) {
    char b[64];
    std::snprintf(b, sizeof b, f, v);
    return b;
}

}  // namespace

class DrmDecoder::Impl {
public:
    Impl() : ring(kHistory), keepAus(false) {
        codec.reset(drm::makeAacCodec());
        reset();
    }

    // ---- the front end (DrmFront) -------------------------------------------
    double feRate;
    int D1;
    double fs1, step;
    double h[kTaps];
    std::complex<double> hist[256];
    long long n1;
    double t;
    std::complex<double> acc, osc;
    int kk;
    bool haveDial;
    double lastDial, lastCenter;

    // ---- the history and the windows -------------------------------------------
    std::vector<std::complex<float> > ring, win;
    std::size_t w;
    long long n, lastAt;
    drm::Rx rx;
    drm::RxResult r;
    bool haveR;
    drm::Rx::Hint hint;
    long long winCount;
    long long lastId;
    std::vector<drm::MuxFrame> mux;
    std::vector<drm::AacAu> aus;

    // ---- what is known (merged window by window) ---------------------------------
    drm::RxResult known;
    bool haveKnown;
    int selected;  // -1: the first audio service

    // ---- data ----------------------------------------------------------------------
    drm::TextMessage txt;
    std::string txtStation, text, textOut;
    drm::PacketAssembler slidePk, jlPk;
    std::string slideKey, jlKey;
    drm::MotSlides slides;
    drm::Journaline jl;
    std::vector<Decoder::EncodedImage> images;
    std::vector<drm::Bytes> groups;
    std::vector<drm::Picture> pics;
    int pictures;

    // ---- the sound ---------------------------------------------------------------------
    std::unique_ptr<drm::AudioCodec> codec;
    std::string codecKey;
    bool codecOk;
    std::size_t frameLen;
    std::vector<float> pcm;
    int prate;
    drm::XheFrames xhe;
    std::vector<drm::Bytes> xframes;
    bool keepAus;
    std::vector<DrmAccessUnit> auOut;
    long long audioGood, audioBad, accessUnits, muxFrames;
    int playingSid;

    void reset() {
        feRate = 0;
        haveDial = false;
        lastDial = lastCenter = 0;
        std::fill(ring.begin(), ring.end(), std::complex<float>());
        w = 0;
        n = 0;
        lastAt = 0;
        r.clear();
        haveR = false;
        hint = drm::Rx::Hint();
        winCount = 0;
        lastId = LLONG_MIN;
        known.clear();
        haveKnown = false;
        selected = -1;
        txt.reset();
        txtStation.clear();
        text.clear();
        textOut.clear();
        slidePk = drm::PacketAssembler();
        jlPk = drm::PacketAssembler();
        slideKey.clear();
        jlKey.clear();
        slides.reset();
        jl.reset();
        images.clear();
        pictures = 0;
        codecKey.clear();
        codecOk = false;
        frameLen = 0;
        pcm.clear();
        prate = 48000;
        xhe.reset();
        auOut.clear();
        audioGood = audioBad = accessUnits = muxFrames = 0;
        playingSid = -1;
    }

    void setup(double rate) {
        feRate = rate;
        D1 = std::max(1, static_cast<int>(std::floor(rate / 240000 + 0.5)));
        fs1 = rate / D1;
        step = fs1 / drm::kRate;
        // Addition to the JS: 16 kHz (the JS: 11 kHz), for 18/20 kHz signals
        const double fc = std::min(16000.0, 0.45 * fs1) / fs1;
        double sum = 0;
        for (int i = 0; i < kTaps; ++i) {
            const double x = i - (kTaps - 1) / 2.0, wnd = 0.54 - 0.46 * std::cos(2 * kPi * i / (kTaps - 1));
            h[i] = (x == 0 ? 2 * fc : std::sin(2 * kPi * fc * x) / (kPi * x)) * wnd;
            sum += h[i];
        }
        for (int i = 0; i < kTaps; ++i) h[i] /= sum;
        for (int i = 0; i < 256; ++i) hist[i] = std::complex<double>();
        n1 = 0;
        t = kTaps;
        acc = std::complex<double>();
        kk = 0;
        osc = std::complex<double>(1, 0);
    }

    std::complex<double> fir(long long m) const {
        std::complex<double> y;
        for (int j = 0; j < kTaps; ++j) y += h[j] * hist[(m - j) & 255];
        return y;
    }

    void processIq(const std::complex<float>* iq, std::size_t cnt, double rate, double centerHz, double dialHz) {
        if (!(rate > 0) || !std::isfinite(rate) || !iq) return;
        // (a new station: everything heard so far forgotten)
        if (haveDial && dialHz != lastDial) {
            const bool keep = keepAus;
            reset();
            keepAus = keep;
        }
        haveDial = true;
        lastDial = dialHz;
        lastCenter = centerHz;
        if (rate != feRate) setup(rate);
        const double wv = -2 * kPi * (dialHz - centerHz) / rate;
        const std::complex<double> rot(std::cos(wv), std::sin(wv));
        for (std::size_t i = 0; i < cnt; ++i) {
            float re = iq[i].real(), im = iq[i].imag();
            if (!std::isfinite(re)) re = 0;  // Addition to the JS
            if (!std::isfinite(im)) im = 0;
            acc += std::complex<double>(re, im) * osc;
            osc *= rot;
            if (++kk == D1) {
                kk = 0;
                hist[n1 & 255] = acc / static_cast<double>(D1);
                ++n1;
                acc = std::complex<double>();
                osc *= 1.5 - 0.5 * std::norm(osc);
                while (t + 1 < n1) {
                    const long long m = static_cast<long long>(std::floor(t));
                    const double f = t - m;
                    const std::complex<double> y0 = fir(m), y1 = fir(m + 1);
                    push(std::complex<float>(y0 + (y1 - y0) * f));
                    t += step;
                }
                if (n1 > (1LL << 24)) {  // (kept small, so t keeps its fraction)
                    n1 -= 1LL << 23;
                    t -= static_cast<double>(1LL << 23);
                }
            }
        }
    }

    void push(const std::complex<float>& v) {
        ring[w] = v;
        w = (w + 1) % ring.size();
        ++n;
        if (n >= kFirstWindow && n - lastAt >= kStep) {
            lastAt = n;
            analyseWindow();
        }
    }

    // ---- what is known about the station, merged -----------------------------------
    void merge(const drm::RxResult& x) {
        if (!x.settings) return;
        if (!haveKnown) known.clear();
        haveKnown = true;
        known.mode = x.mode;
        known.metric = x.metric;
        known.settings = true;
        known.so = x.so;
        known.mscCode = x.mscCode;
        known.longIl = x.longIl;
        known.sdc16 = x.sdc16;
        known.services = x.services;
        known.freqOffset = x.freqOffset;
        known.mirrored = x.mirrored;
        if (x.hasMer) {
            known.hasMer = true;
            known.mer = x.mer;
        }
        for (int s = 0; s < 4; ++s) {
            if (x.svcSeen[s]) {
                known.svcSeen[s] = true;
                known.svc[s] = x.svc[s];
            }
            if (x.haveLabel[s]) {
                known.haveLabel[s] = true;
                known.labels[s] = x.labels[s];
            }
            if (x.audio[s].present) known.audio[s] = x.audio[s];
        }
        if (!x.label.empty()) known.label = x.label;
        if (x.mux.present) known.mux = x.mux;
        if (!x.apps.empty()) known.apps = x.apps;
    }

    // the service played: the chosen one, else the first audio service
    int pickService(const drm::RxResult& x) const {
        if (selected >= 0 && selected < 4 && (x.audio[selected].present || x.svcSeen[selected] || x.haveLabel[selected])) return selected;
        for (int s = 0; s < 4; ++s)
            if (x.audio[s].present && !(x.svcSeen[s] && x.svc[s].data)) return s;
        for (int s = 0; s < 4; ++s)
            if (x.svcSeen[s]) return s;
        return -1;
    }

    // Can this station's audio be played here?  "" if so (or not known yet),
    // otherwise the reason, in plain words (drmAudioUnsupported)
    std::string unsupported(const drm::RxResult& x, int sid) const {
        if (sid < 0) return "";
        const drm::AudioParams& a = x.audio[sid];
        if (!a.present) return x.svcSeen[sid] && x.svc[sid].data ? "it's a data service, without audio" : "";
        if (a.codec == 3) return "xHE-AAC: not supported yet";
        if (a.codec == 1 || a.codec == 2) return std::string("it uses the ") + codecName(a.codec) + " speech codec, which can't be played";
        if (x.mscCode == 1 || x.mscCode == 2) return "it uses hierarchical modulation, which can't be decoded yet";
        for (std::size_t q = 0; q < x.mux.streams.size(); ++q)
            if (x.mux.streams[q].first > 0) return "it uses two protection levels (unequal error protection), which can't be decoded yet";
        if (!codec) return "no AAC decoder in this build (liboffgrid-codecs)";
        return "";
    }

    void streamAt(const drm::RxResult& x, int s, std::size_t& off, std::size_t& len) const {
        off = 0;
        len = 0;
        if (s < 0 || s >= static_cast<int>(x.mux.streams.size())) return;
        for (std::size_t q = 0; q < x.mux.streams.size(); ++q) off += static_cast<std::size_t>(x.mux.streams[q].first);  // (parts A first, then B)
        for (int q = 0; q < s; ++q) off += static_cast<std::size_t>(x.mux.streams[static_cast<std::size_t>(q)].second);
        len = static_cast<std::size_t>(x.mux.streams[static_cast<std::size_t>(s)].first + x.mux.streams[static_cast<std::size_t>(s)].second);
    }

    const drm::DataApp* findApp(const drm::RxResult& x, int app) const {
        for (std::size_t q = 0; q < x.apps.size(); ++q)
            if (x.apps[q].packet && x.apps[q].domain == 1 && x.apps[q].app == app) return &x.apps[q];
        return 0;
    }

    void analyseWindow() {
        const std::size_t L = ring.size(), nw = static_cast<std::size_t>(std::min<long long>(n, static_cast<long long>(L)));
        win.resize(nw);
        for (std::size_t j = 0; j < nw; ++j) win[j] = ring[(w + L - nw + j) % L];
        const long long t0 = n - static_cast<long long>(nw);
        rx.keep = true;
        ++winCount;
        // (the last good reading's mode, mirroring and carrier offset, except
        //  every 10th window, read in full)
        rx.hint = hint.valid && winCount % 10 != 0 ? hint : drm::Rx::Hint();
        rx.analyse(&win[0], nw, r);
        if (rx.hint.valid && !r.facOk) {  // (the hint failed: in full)
            rx.hint = drm::Rx::Hint();
            rx.analyse(&win[0], nw, r);
        }
        haveR = true;
        if (r.facOk) {
            hint.valid = true;
            hint.mode = r.mode;
            hint.mirrored = r.mirrored;
            hint.sh = r.sh;
        } else
            hint = drm::Rx::Hint();
        merge(r);
        if (!r.settings || !r.mux.present) return;
        const int sid = pickService(r);
        rx.mscFrames(r, t0, lastId, mux);
        for (std::size_t q = 0; q < mux.size(); ++q) {
            const drm::MuxFrame& mf = mux[q];
            if (mf.id <= lastId) continue;
            const bool gap = lastId != LLONG_MIN && mf.id != lastId + 1;
            lastId = mf.id;
            ++muxFrames;
            frame(mf, sid, gap);
        }
    }

    void frame(const drm::MuxFrame& mf, int sid, bool gap) {
        const drm::Bytes& all = mf.all;
        // the audio service: its stream's audio super frame
        if (sid >= 0 && r.audio[sid].present) {
            const drm::AudioParams& a = r.audio[sid];
            int st = a.streamId;
            if (st >= static_cast<int>(r.mux.streams.size())) st = 0;
            std::size_t off, len;
            streamAt(r, st, off, len);
            if (len >= 8 && off + len <= all.size()) {
                const unsigned char* by = &all[off];
                if (sid != playingSid) {  // (another service: its own sound and text)
                    playingSid = sid;
                    codecKey.clear();
                    xhe.reset();
                    txt.reset();
                    txtStation.clear();
                }
                // the text message: the last 4 bytes of the audio super frame
                if (a.text) {
                    if (r.label != txtStation) {
                        txtStation = r.label;
                        txt.reset();
                    }
                    if (txt.feed(by + len - 4) && txt.text != text) {
                        text = txt.text;
                        textOut += (r.label.empty() ? std::string("DRM") : r.label) + ": " + text + "\n";
                    }
                }
                if (a.codec == 0) audioAac(a, by, static_cast<int>(len), mf.id, sid);
                if (a.codec == 3) {
                    // xHE-AAC: frames run on from one super frame into the
                    // next; a gap starts afresh, so no frame is stitched from
                    // two places.  Recognised and counted; not played yet.
                    if (gap) xhe.reset();
                    xhe.feed(by, static_cast<int>(len) - (a.text ? 4 : 0), xframes);
                    for (std::size_t j = 0; j < xframes.size(); ++j) {
                        ++accessUnits;
                        if (keepAus) {
                            DrmAccessUnit u;
                            u.service = static_cast<unsigned>(sid);
                            u.codec = 3;
                            u.frame = mf.id;
                            u.bytes = xframes[j];
                            auOut.push_back(u);
                        }
                    }
                }
            }
        }
        // the SlideShow: a packet-mode data stream
        if (const drm::DataApp* ap = findApp(r, 2)) {
            char key[96];
            std::snprintf(key, sizeof key, ":%d:%d", ap->stream, ap->packetId);
            const std::string k = r.label + key;
            if (k != slideKey) {
                slideKey = k;
                slidePk = drm::PacketAssembler();
                slidePk.packetLen = ap->packetLen;
                slidePk.packetId = ap->packetId;
                slides.reset();
            }
            std::size_t off, len;
            streamAt(r, ap->stream, off, len);
            if (len && off + len <= all.size()) {
                groups.clear();
                slidePk.frame(&all[off], len, groups);
                for (std::size_t j = 0; j < groups.size(); ++j) {
                    pics.clear();
                    slides.group(groups[j], pics);
                    for (std::size_t p = 0; p < pics.size(); ++p) {
                        Decoder::EncodedImage im;
                        im.name = pics[p].name;
                        im.mime = pics[p].mime;
                        im.bytes = pics[p].bytes;
                        images.push_back(im);
                        ++pictures;
                    }
                }
            }
        }
        // Journaline: the same, its own packet ID
        if (const drm::DataApp* ap = findApp(r, 0x44a)) {
            char key[96];
            std::snprintf(key, sizeof key, ":%d:%d", ap->stream, ap->packetId);
            const std::string k = r.label + key;
            if (k != jlKey) {
                jlKey = k;
                jlPk = drm::PacketAssembler();
                jlPk.packetLen = ap->packetLen;
                jlPk.packetId = ap->packetId;
                jlPk.maxGroup = 8000;
                jlPk.resetOnBad = true;
                jl.reset();
            }
            std::size_t off, len;
            streamAt(r, ap->stream, off, len);
            if (len && off + len <= all.size()) {
                groups.clear();
                jlPk.frame(&all[off], len, groups);
                for (std::size_t j = 0; j < groups.size(); ++j) jl.group(groups[j]);
            }
        }
    }

    void audioAac(const drm::AudioParams& a, const unsigned char* by, int len, long long id, int sid) {
        drm::aacFrames(by, a, len, aus);
        // (re)started when the audio settings change: AAC rate and channel mode
        char key[64];
        std::snprintf(key, sizeof key, "%d/%d/%d", a.rateCode, a.sbr ? 1 : 0, a.mode);
        if (codec && key != codecKey) {
            codecKey = key;
            codecOk = codec->configure(a);
            frameLen = 0;
        }
        const bool canPlay = codec && codecOk && unsupported(r, sid).empty();
        for (std::size_t j = 0; j < aus.size(); ++j) {
            ++accessUnits;
            if (keepAus || canPlay) {
                DrmAccessUnit u;
                u.service = static_cast<unsigned>(sid);
                u.codec = 0;
                u.frame = id;
                u.bytes.resize(static_cast<std::size_t>(aus[j].n) + 1);
                u.bytes[0] = aus[j].crc;
                std::copy(aus[j].p, aus[j].p + aus[j].n, u.bytes.begin() + 1);
                if (canPlay) {
                    const std::size_t before = pcm.size();
                    int rate = prate;
                    if (codec->decode(&u.bytes[0], u.bytes.size(), pcm, rate)) {
                        // (the first AU after (re)starting gives no samples: the
                        //  codec's start-up delay, not a loss)
                        ++audioGood;
                        prate = rate;
                        if (pcm.size() > before) frameLen = pcm.size() - before;
                    } else {
                        // (a lost frame: its length in silence, so the timeline stays exact)
                        pcm.resize(before);
                        ++audioBad;
                        if (frameLen) pcm.resize(before + frameLen, 0.0f);
                    }
                }
                if (keepAus) auOut.push_back(u);
            }
        }
    }
};

DrmDecoder::DrmDecoder() : d_(new Impl) {}
DrmDecoder::~DrmDecoder() {}

void DrmDecoder::reset() {
    const bool keep = d_->keepAus;
    d_->reset();
    d_->keepAus = keep;
}

void DrmDecoder::process(const float* audio, std::size_t n, double rate) {
    (void)audio;
    (void)n;
    (void)rate;
}

void DrmDecoder::processIq(const std::complex<float>* iq, std::size_t n, double rate, double centerHz, double dialHz) {
    d_->processIq(iq, n, rate, centerHz, dialHz);
}

void DrmDecoder::takeText(std::string& out) {
    out += d_->textOut;
    d_->textOut.clear();
}

bool DrmDecoder::hasAudio() const { return d_->codec.get() != 0; }
int DrmDecoder::audioRate() const { return d_->prate; }

void DrmDecoder::takeAudio(std::vector<float>& stereo) {
    stereo.insert(stereo.end(), d_->pcm.begin(), d_->pcm.end());
    d_->pcm.clear();
}

void DrmDecoder::takeImages(std::vector<EncodedImage>& images) {
    images.insert(images.end(), d_->images.begin(), d_->images.end());
    d_->images.clear();
}

void DrmDecoder::setKeepAccessUnits(bool on) { d_->keepAus = on; }

void DrmDecoder::takeAccessUnits(std::vector<DrmAccessUnit>& out) {
    out.insert(out.end(), d_->auOut.begin(), d_->auOut.end());
    d_->auOut.clear();
}

bool DrmDecoder::aacAvailable() {
    std::unique_ptr<drm::AudioCodec> c(drm::makeAacCodec());
    return c.get() != 0;
}

std::vector<Service> DrmDecoder::services() const {
    std::vector<Service> out;
    const drm::RxResult& k = d_->known;
    if (!d_->haveKnown) return out;
    const int sel = d_->pickService(k);
    for (int s = 0; s < 4; ++s) {
        if (!k.svcSeen[s] && !k.haveLabel[s] && !k.audio[s].present) continue;
        Service sv;
        sv.id = static_cast<unsigned>(s);
        sv.label = k.haveLabel[s] ? k.labels[s] : "";
        const drm::AudioParams& a = k.audio[s];
        sv.audio = !(k.svcSeen[s] && k.svc[s].data);
        std::string det;
        if (a.present) {
            det = codecName(a.codec);
            if (a.sbr && a.codec == 0) det += " with SBR";
            det += std::string(", ") + stereoName(a.mode) + ", " + (a.codec == 3 ? drm::xheRateName(a.rateCode) : drm::aacRateName(a.rateCode));
            int st = a.streamId < static_cast<int>(k.mux.streams.size()) ? a.streamId : 0;
            if (!k.mux.streams.empty()) {
                const int bytes = k.mux.streams[static_cast<std::size_t>(st)].first + k.mux.streams[static_cast<std::size_t>(st)].second;
                sv.bitrateKbps = static_cast<int>(std::floor(bytes * 8 / 400.0 + 0.5));  // (a frame every 400 ms)
            }
        } else if (!sv.audio)
            det = "data";
        if (k.svcSeen[s]) {
            const char* lang = drm::languageName(k.svc[s].lang);
            if (*lang) det += std::string(det.empty() ? "" : " · ") + lang;
            if (!k.svc[s].data) {
                const char* pt = drm::programmeTypeName(k.svc[s].ptype);
                if (*pt) det += std::string(det.empty() ? "" : " · ") + pt;
            }
        }
        const std::string why = d_->unsupported(k, s);
        if (!why.empty()) det += std::string(det.empty() ? "" : " · ") + (a.codec == 3 && a.present ? why : "no sound: " + why);
        sv.detail = det;
        sv.selected = s == sel;
        out.push_back(sv);
    }
    return out;
}

bool DrmDecoder::selectService(unsigned id) {
    if (id > 3) return false;
    const drm::RxResult& k = d_->known;
    if (d_->haveKnown && !k.svcSeen[id] && !k.haveLabel[id] && !k.audio[id].present) return false;
    d_->selected = static_cast<int>(id);
    d_->pcm.clear();
    return true;
}

DrmInfo DrmDecoder::info() const {
    DrmInfo i;
    const Impl& d = *d_;
    const drm::RxResult& k = d.known;
    if (d.haveR) {
        i.metric = d.r.metric;
        i.signal = d.r.metric >= 0.4;
        i.locked = d.r.facOk > 0;
        i.facOk = d.r.facOk;
        i.facTried = d.r.facTried;
        i.sdcOk = d.r.sdcOk;
        i.sdcTried = d.r.sdcTried;
        i.state = d.r.state;
        if (d.r.mode >= 0 && i.signal) i.mode = drm::kModeLetter[d.r.mode];
    } else
        i.state = d.n < kFirstWindow ? "listening (the first read takes about 4 seconds)" : "reading";
    if (d.haveKnown) {
        i.mode = drm::kModeLetter[k.mode];
        i.occupancy = k.so;
        i.occupancyKhz = k.so >= 0 && k.so < 6 ? drm::kOccupancyKhz[k.so] : 0;
        i.msc = mscName(k.mscCode);
        i.sdc = k.sdc16 ? "16-QAM" : "4-QAM";
        i.longInterleave = k.longIl;
        i.hasMer = k.hasMer;
        i.merDb = k.mer;
        i.freqOffsetHz = d.haveR && d.r.settings ? d.r.freqOffset : k.freqOffset;
        i.mirrored = k.mirrored;
        const int s = d.pickService(k);
        if (s >= 0) {
            i.label = k.haveLabel[s] ? k.labels[s] : k.label;
            if (k.svcSeen[s]) {
                i.serviceId = static_cast<unsigned>(k.svc[s].serviceId);
                i.language = drm::languageName(k.svc[s].lang);
                if (!k.svc[s].data) i.programmeType = drm::programmeTypeName(k.svc[s].ptype);
            }
            const drm::AudioParams& a = k.audio[s];
            if (a.present) {
                i.codec = a.codec;
                i.codecName = codecName(a.codec);
                i.sbr = a.sbr;
                i.stereoMode = a.mode;
                i.rateCode = a.rateCode;
                i.audioText = std::string(codecName(a.codec)) + (a.sbr && a.codec == 0 ? " with SBR" : "") + ", " + stereoName(a.mode) + ", " +
                              (a.codec == 3 ? drm::xheRateName(a.rateCode) : drm::aacRateName(a.rateCode));
                const int st = a.streamId < static_cast<int>(k.mux.streams.size()) ? a.streamId : 0;
                if (!k.mux.streams.empty())
                    i.streamBytes = k.mux.streams[static_cast<std::size_t>(st)].first + k.mux.streams[static_cast<std::size_t>(st)].second;
            }
            i.unsupported = d.unsupported(k, s);
        } else
            i.label = k.label;
    }
    i.text = d.text;
    i.audioGood = d.audioGood;
    i.audioBad = d.audioBad;
    i.accessUnits = d.accessUnits;
    i.muxFrames = d.muxFrames;
    i.pictures = d.pictures;
    i.journalinePages = static_cast<int>(d.jl.pages.size());
    return i;
}

Status DrmDecoder::status() const {
    const DrmInfo i = info();
    Status s;
    s.signal = i.signal;
    s.locked = i.locked;
    s.snrDb = i.hasMer ? i.merDb : 0;
    s.quality = i.facTried ? static_cast<double>(i.facOk) / i.facTried : 0;
    s.characters = static_cast<unsigned>(i.text.size());
    s.station = i.label;
    if (i.mode && i.occupancy >= 0) {
        s.mode = std::string("DRM ") + i.mode + " " + fmt("%g", i.occupancyKhz) + " kHz " + i.msc;
        std::string line = i.label.empty() ? std::string("(name not read yet)") : i.label;
        if (!i.locked) line += " (fading)";
        line += std::string(" · mode ") + i.mode + ", " + fmt("%g", i.occupancyKhz) + " kHz, " + i.msc + ", " + (i.longInterleave ? "long" : "short") +
                " interleaving";
        if (i.hasMer) line += fmt(" · MER %.0f dB", i.merDb);
        if (!i.audioText.empty()) line += " · " + i.audioText;
        if (!i.unsupported.empty()) line += " · " + i.unsupported;
        if (i.audioGood + i.audioBad > 0) line += fmt(" · %.0f%% audio frames good", 100.0 * i.audioGood / (i.audioGood + i.audioBad));
        if (std::fabs(i.freqOffsetHz) > 30) line += fmt(" · %+.0f Hz off centre", i.freqOffsetHz);
        if (i.mirrored) line += " · mirrored";
        s.info = line;
    } else {
        s.mode = "DRM";
        s.info = i.state == "no DRM signal" ? "no DRM signal here: tune to the centre of a DRM station (a flat block about 10 kHz wide)" : i.state;
    }
    return s;
}

std::vector<DrmJournalinePage> DrmDecoder::journaline() const {
    std::vector<DrmJournalinePage> out;
    static const char* const types[5] = {"", "menu", "text", "title", "list"};
    for (std::map<unsigned, drm::JournalinePage>::const_iterator it = d_->jl.pages.begin(); it != d_->jl.pages.end(); ++it) {
        DrmJournalinePage p;
        p.id = it->second.id;
        p.type = types[it->second.type >= 1 && it->second.type <= 4 ? it->second.type : 0];
        p.title = it->second.title;
        p.text = it->second.text;
        p.links = it->second.links;
        p.rows = it->second.rows;
        out.push_back(p);
    }
    return out;
}

}  // namespace dec
