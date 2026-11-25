// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DrmData.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  What DRM's main channel carries, once its bytes are out:
//    - the AAC audio super frame (ETSI ES 201 980 6.4.3; Dream's
//      AACSuperFrame): frame borders (12 bits each), one CRC byte per frame,
//      then the frames -> AAC access units, each with its CRC byte (the CRC
//      covers the AAC core's bits only, so the AAC decoder checks it: faad2's
//      DRM mode, liboffgrid-codecs);
//    - the xHE-AAC audio super frame (Dream's XHEAACSuperFrame): a 2-byte
//      header (frame border count, bit reservoir level, CRC-8), the payload,
//      then a directory at the end (2 bytes per frame border, in reverse
//      order); frames run on from one super frame into the next;
//    - DRM's compact xHE-AAC configuration -> an MPEG-4 AudioSpecificConfig
//      with a UsacConfig (following Fraunhofer FDK's DRM parser);
//    - the text message: segments of up to 16 bytes, each after four 0xFF
//      bytes, with a 2-byte header and a CRC, 4 bytes a super frame;
//    - packet mode (ES 201 980 6.6.1: a 1-byte header (first, last, packet
//      ID, padding indicator, continuity), the data, a CRC-16) carrying MSC
//      data groups (as DAB's);
//    - MOT in header mode (EN 301 234): the SlideShow's pictures (a header
//      object, type 3, with the body's size, content type and name; the
//      body, type 4; by transport ID);
//    - Journaline (ETSI TS 102 979): NML objects (menus, plain text, lists),
//      compressed ones through a raw inflate (RFC 1951).
//
//  Port of OFFgrid-SDR's JS (diddy-boy, https://github.com/solariun/
//  offgrid-sdr, index.html "drmlib": drmAacFrames, drmXheFrames, drmXheAsc,
//  drmTextFeed, DrmSlides, DrmJournaline, drmNmlParse), itself following
//  Dream (TU Darmstadt, Volker Fischer and others; GPL-2.0-or-later,
//  licenses/dream-COPYING.txt) and, for Journaline, Fraunhofer's reference
//  decoder as found in Dream.
//
//  Differences from the JS:
//    - Addition to the JS: the xHE-AAC super frame's header CRC-8 is
//      checked (the JS ignored it); a bad one starts the frames afresh;
//    - Addition to the JS: compressed Journaline objects are inflated here
//      (the JS used the browser's DecompressionStream);
//    - Journaline's emphasis is marked *like this* (the JS bracketed the
//      stars with invisible separators for its page);
//    - every read is bounds-checked (the JS read undefined past the end).
//
//  How it fits in
//  --------------
//  Declared in DrmInternal.h; used by DrmDecoder (Drm.cpp).
//
//  Contributing
//  ------------
//  The generator (DrmEncoders.cpp) builds every one of these; the tests
//  check that what goes in comes out.
// ============================================================================
#include "decoders/DrmInternal.h"

#include <algorithm>
#include <cstdio>

namespace dec {
namespace drm {

namespace {

unsigned bitsAt(const unsigned char* by, std::size_t len, std::size_t pos, int n) {
    unsigned v = 0;
    for (int j = 0; j < n; ++j) {
        const std::size_t p = pos + static_cast<std::size_t>(j);
        v = v * 2 + ((p >> 3) < len ? ((by[p >> 3] >> (7 - (p & 7))) & 1) : 0);
    }
    return v;
}

std::string trimText(const Bytes& b) {
    std::size_t a = 0, e = b.size();
    while (a < e && (b[a] == ' ' || b[a] == '\t' || b[a] == '\n' || b[a] == '\r')) ++a;
    while (e > a && (b[e - 1] == ' ' || b[e - 1] == '\t' || b[e - 1] == '\n' || b[e - 1] == '\r' || b[e - 1] == 0)) --e;
    return std::string(b.begin() + static_cast<std::ptrdiff_t>(a), b.begin() + static_cast<std::ptrdiff_t>(e));
}

}  // namespace

// ---- AAC ---------------------------------------------------------------------
int aacFrames(const unsigned char* by, const AudioParams& a, int lenBytes, std::vector<AacAu>& out) {
    out.clear();
    const int nf = a.rateCode == 3 ? 10 : 5, payload = lenBytes - (a.text ? 4 : 0), nb = nf - 1,
              hdr = (12 * nb + (nb == 9 ? 4 : 0)) / 8, avail = payload - hdr - nf;
    if (avail <= 0) return 0;
    int lens[10], prev = 0;
    for (int j = 0; j < nb; ++j) {
        int bd = static_cast<int>(bitsAt(by, static_cast<std::size_t>(lenBytes), static_cast<std::size_t>(12 * j), 12));
        if (bd < prev) bd += 4096;
        lens[j] = bd - prev;
        prev = bd;
    }
    lens[nb] = avail - prev;
    int pos = hdr + nf;
    for (int j = 0; j < nf; ++j) {
        const int L = lens[j];
        if (L <= 0 || pos + L > payload) break;
        AacAu au;
        au.crc = by[hdr + j];
        au.p = by + pos;
        au.n = L;
        out.push_back(au);
        pos += L;
    }
    return static_cast<int>(out.size());
}

// ---- xHE-AAC -------------------------------------------------------------------
void XheFrames::feed(const unsigned char* by, int superFrameSize, std::vector<Bytes>& out) {
    out.clear();
    if (superFrameSize < 2) return;
    const int fbc = by[0] >> 4;
    // Addition to the JS: the header's CRC-8 (over its first byte)
    if (crcDream(by, 1, 8) != by[1]) {
        ++headerBad;
        reset();
        return;
    }
    const int dirOff = superFrameSize - 2 * fbc;
    if (dirOff < 2) {
        payload_.clear();
        return;
    }
    const int start = static_cast<int>(payload_.size());
    payload_.insert(payload_.end(), by + 2, by + dirOff);
    if (!fbc) return;
    int borders[16], size[16];
    bool ok = true;
    for (int i = fbc - 1, pos = dirOff * 8; i >= 0; --i, pos += 16) {
        borders[i] = static_cast<int>(bitsAt(by, static_cast<std::size_t>(superFrameSize), static_cast<std::size_t>(pos), 12));
        if (static_cast<int>(bitsAt(by, static_cast<std::size_t>(superFrameSize), static_cast<std::size_t>(pos + 12), 4)) != fbc) ok = false;
    }
    if (!ok) {  // (a damaged directory: start afresh)
        payload_.clear();
        return;
    }
    if (borders[0] == 0xffe) {
        if (start < 2) {
            payload_.clear();
            return;
        }
        borders[0] = start - 2;
    } else if (borders[0] == 0xfff) {
        if (start < 1) {
            payload_.clear();
            return;
        }
        borders[0] = start - 1;
    } else
        borders[0] += start;
    size[0] = borders[0];
    int total = size[0];
    bool neg = size[0] < 0;
    for (int i = 1; i < fbc; ++i) {
        borders[i] += start;
        size[i] = borders[i] - borders[i - 1];
        total += size[i];
        if (size[i] < 0) neg = true;
    }
    if (neg || total > static_cast<int>(payload_.size())) {
        payload_.clear();
        return;
    }
    int p = 0;
    for (int i = 0; i < fbc; ++i) {
        out.push_back(Bytes(payload_.begin() + p, payload_.begin() + p + size[i]));
        p += size[i];
    }
    payload_.erase(payload_.begin(), payload_.begin() + p);
    // (the first frame of a fresh start is only the tail of a frame whose
    //  start was missed: dropped)
    if (!started_) {
        started_ = true;
        out.erase(out.begin());
    }
}

// DRM sends the codec's configuration in a compact form of its own in the
// SDC (after the two bytes of the audio entity); a USAC decoder wants a
// standard AudioSpecificConfig with a UsacConfig.  Translated field by field,
// following Fraunhofer FDK's DRM parser (tpdec_asc.cpp, sac_bitdec.cpp).  DRM
// leaves out the element count and type (one element: mono SCE or stereo
// CPE), the time-warped MDCT flag (off) and, in MPS212, the decorrelation
// configuration (0), and sends the temporal shaping as one bit (0 or 3).
Bytes xheAsc(const AudioParams& a) {
    Bytes res;
    if (a.xheConfig.empty()) return res;
    static const int kSr[8] = {9600, 12000, 16000, 19200, 24000, 32000, 38400, 48000};
    static const int kUsacIdx[8] = {0x1b, 0x09, 0x08, 0x17, 0x06, 0x05, 0x12, 0x03};
    static const int kSf4[13] = {96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350};
    std::size_t ip = 0;
    const std::size_t nin = a.xheConfig.size() * 8;
    struct Rd {
        const Bytes* cfg;
        std::size_t* ip;
        std::size_t nin;
        int operator()(int n) const {
            int v = 0;
            for (int i = 0; i < n; ++i, ++*ip) v = v * 2 + (*ip < nin ? (((*cfg)[*ip >> 3] >> (7 - (*ip & 7))) & 1) : 0);
            return v;
        }
    };
    Rd rd = {&a.xheConfig, &ip, nin};
    std::vector<unsigned char> ob;
    struct Wr {
        std::vector<unsigned char>* ob;
        void operator()(long v, int n) const {
            for (int i = n - 1; i >= 0; --i) ob->push_back(static_cast<unsigned char>((v >> i) & 1));
        }
    };
    Wr wr = {&ob};
    const bool stereo = a.mode == 2;
    const int fs = kSr[a.rateCode & 7];
    int sfIdx4 = -1;
    for (int i = 0; i < 13; ++i)
        if (kSf4[i] == fs) sfIdx4 = i;
    // AudioSpecificConfig: object type 42 (USAC, escaped: 31 then 42-32),
    // sampling frequency, channel configuration
    wr(31, 5);
    wr(42 - 32, 6);
    if (sfIdx4 >= 0)
        wr(sfIdx4, 4);
    else {
        wr(15, 4);
        wr(fs, 24);
    }
    wr(stereo ? 2 : 1, 4);
    // UsacConfig
    const int csfl = rd(2) + 1;  // (DRM sends the index less one)
    const bool sbr = csfl >= 2;
    wr(kUsacIdx[a.rateCode & 7], 5);
    wr(csfl, 3);
    wr(stereo ? 2 : 1, 5);
    wr(0, 4);              // UsacDecoderConfig: one element
    wr(stereo ? 1 : 0, 2);  // ID_USAC_CPE or ID_USAC_SCE
    wr(0, 1);              // UsacCoreConfig: tw_mdct off
    wr(rd(1), 1);          //  noiseFilling
    if (sbr) {
        wr(rd(3), 3);  // harmonicSBR, bs_interTes, bs_pvc
        wr(rd(4), 4);  // SbrDfltHeader
        wr(rd(4), 4);
        const int e1 = rd(1), e2 = rd(1);
        wr(e1, 1);
        wr(e2, 1);
        if (e1) {
            wr(rd(2), 2);
            wr(rd(1), 1);
            wr(rd(2), 2);
        }
        if (e2) {
            wr(rd(2), 2);
            wr(rd(2), 2);
            wr(rd(1), 1);
            wr(rd(1), 1);
        }
    }
    if (stereo) {
        const int sci = rd(2);
        wr(sci, 2);
        if (sci > 0) {  // Mps212Config, DRM's form to MPEG's
            wr(rd(3), 3);
            wr(rd(3), 3);              // bsFreqRes, bsFixedGainDMX
            wr(rd(1) ? 3 : 0, 2);      // bsTempShapeConfig (one bit in DRM)
            wr(0, 2);                  // bsDecorrConfig 0
            wr(rd(1), 1);              // high-rate mode
            wr(rd(1), 1);              // phase coding
            const int opp = rd(1);
            wr(opp, 1);
            if (opp) wr(rd(5), 5);     // OTT bands phase
            if (sci > 1) {
                wr(rd(5), 5);          // residual bands
                wr(rd(1), 1);          // pseudo-LR
            }
        }
    }
    wr(0, 1);  // usacConfigExtensionPresent: none
    while (ob.size() % 8) ob.push_back(0);
    res.assign(ob.size() / 8, 0);
    for (std::size_t i = 0; i < ob.size(); ++i) res[i >> 3] |= static_cast<unsigned char>(ob[i] << (7 - (i & 7)));
    return res;
}

// ---- the text message ------------------------------------------------------------
void TextMessage::reset() {
    buf_.clear();
    segs_.clear();
    toggle_ = -1;
    text.clear();
    have = false;
}

bool TextMessage::feed(const unsigned char* four) {
    Bytes& B = buf_;
    B.insert(B.end(), four, four + 4);
    bool done = false;
    for (;;) {
        int m = -1;
        for (std::size_t i = 0; i + 3 < B.size(); ++i)
            if (B[i] == 0xff && B[i + 1] == 0xff && B[i + 2] == 0xff && B[i + 3] == 0xff) {
                m = static_cast<int>(i);
                break;
            }
        if (m < 0) {
            if (B.size() > 3) B.erase(B.begin(), B.end() - 3);
            return done;
        }
        const std::size_t um = static_cast<std::size_t>(m);
        if (B.size() < um + 6) {
            B.erase(B.begin(), B.begin() + m);
            return done;
        }
        const int h0 = B[um + 4], h1 = B[um + 5], cmd = (h0 >> 4) & 1, len = cmd ? 0 : (h0 & 15) + 1;
        if (B.size() < um + 6 + static_cast<std::size_t>(len) + 2) {
            B.erase(B.begin(), B.begin() + m);
            return done;
        }
        const Bytes seg(B.begin() + m + 4, B.begin() + m + 8 + len);
        B.erase(B.begin(), B.begin() + m + 4);
        if (crc16(&seg[0], static_cast<std::size_t>(len + 2)) != ((static_cast<unsigned>(seg[static_cast<std::size_t>(len + 2)]) << 8) | seg[static_cast<std::size_t>(len + 3)]))
            continue;  // (damaged: dropped)
        if (cmd) {
            if ((h0 & 15) == 1) {  // (1: the message removed)
                segs_.clear();
                text.clear();
                have = true;
                done = true;
            }
            continue;
        }
        const int tog = h0 >> 7, firstF = (h0 >> 6) & 1, lastF = (h0 >> 5) & 1, num = firstF ? 0 : (h1 >> 4) & 7;
        if (tog != toggle_) {
            segs_.clear();
            toggle_ = tog;
        }
        segs_[num] = std::make_pair(lastF != 0, Bytes(seg.begin() + 2, seg.begin() + 2 + len));
        Bytes parts;
        bool whole = false;
        for (int i = 0; i < 8; ++i) {
            std::map<int, std::pair<bool, Bytes> >::const_iterator it = segs_.find(i);
            if (it == segs_.end()) break;
            parts.insert(parts.end(), it->second.second.begin(), it->second.second.end());
            if (it->second.first) {
                whole = true;
                break;
            }
        }
        if (whole) {
            text = trimText(parts);
            have = true;
            done = true;
        }
    }
}

// ---- packet mode -----------------------------------------------------------------
void PacketAssembler::frame(const unsigned char* st, std::size_t n, std::vector<Bytes>& groups) {
    const std::size_t P = static_cast<std::size_t>(packetLen) + 3;
    if (packetLen <= 0) return;
    for (std::size_t p = 0; p + P <= n; p += P) {
        const unsigned char* pk = st + p;
        if (crc16(pk, P - 2) != ((static_cast<unsigned>(pk[P - 2]) << 8) | pk[P - 1])) {
            if (resetOnBad) have_ = false;
            continue;
        }
        ++packets;
        const int h = pk[0], firstF = h >> 7, lastF = (h >> 6) & 1, pid = (h >> 4) & 3, ppi = (h >> 3) & 1;
        if (pid != packetId) continue;
        const unsigned char* d = pk + 1;
        std::size_t dn = P - 3;
        if (ppi) {  // (a padded packet: its first byte says how much is data)
            const std::size_t useful = std::min<std::size_t>(d[0], dn - 1);
            d = d + 1;
            dn = useful;
        }
        if (firstF) {
            cur_.clear();
            have_ = true;
        }
        if (have_) {
            cur_.insert(cur_.end(), d, d + dn);
            if (cur_.size() > maxGroup) have_ = false;
        }
        if (lastF && have_) {
            groups.push_back(cur_);
            have_ = false;
        }
    }
}

// ---- MOT SlideShow -----------------------------------------------------------------
void MotSlides::reset() {
    objs_.clear();
    done_.clear();
}

void MotSlides::group(const Bytes& g, std::vector<Picture>& ready) {
    if (g.size() < 4) return;
    const int ext = g[0] >> 7, crcF = (g[0] >> 6) & 1, segF = (g[0] >> 5) & 1, uaF = (g[0] >> 4) & 1, type = g[0] & 15;
    if (crcF && crc16(&g[0], g.size() - 2) != ((static_cast<unsigned>(g[g.size() - 2]) << 8) | g[g.size() - 1])) return;
    std::size_t q = 2 + (ext ? 2 : 0);
    int last = 0, seg = 0, tid = -1;
    if (segF) {
        if (q + 2 > g.size()) return;
        last = g[q] >> 7;
        seg = ((g[q] & 127) << 8) | g[q + 1];
        q += 2;
    }
    if (uaF) {
        if (q + 1 > g.size()) return;
        const int L = g[q] & 15;
        if (((g[q] >> 4) & 1) && q + 3 <= g.size()) tid = (g[q + 1] << 8) | g[q + 2];
        q += 1 + static_cast<std::size_t>(L);
    }
    if ((type != 3 && type != 4) || tid < 0 || done_.count(tid)) return;
    const std::size_t end = g.size() - (crcF ? 2 : 0);
    if (q + 2 > end) return;
    const std::size_t size = std::min<std::size_t>(static_cast<std::size_t>(((g[q] & 31) << 8) | g[q + 1]), end - q - 2);
    Obj& o = objs_[tid];
    const Bytes part(g.begin() + static_cast<std::ptrdiff_t>(q + 2), g.begin() + static_cast<std::ptrdiff_t>(q + 2 + size));
    if (type == 3) {
        o.hdr[seg] = part;
        if (last) o.hl = seg;
    } else {
        o.body[seg] = part;
        if (last) o.bl = seg;
    }
    if (o.hl < 0 || static_cast<int>(o.hdr.size()) != o.hl + 1 || o.bl < 0 || static_cast<int>(o.body.size()) != o.bl + 1) return;
    Bytes h, b;
    for (int i = 0; i <= o.hl; ++i) h.insert(h.end(), o.hdr[i].begin(), o.hdr[i].end());
    for (int i = 0; i <= o.bl; ++i) b.insert(b.end(), o.body[i].begin(), o.body[i].end());
    objs_.erase(tid);
    done_.insert(tid);
    if (done_.size() > 200) done_.clear();
    if (h.size() < 7) return;
    const int ctype = (h[5] >> 1) & 63, csub = ((h[5] & 1) << 8) | h[6];
    std::size_t r = 7;
    std::string name;
    while (r < h.size()) {
        const int pli = h[r] >> 6, id = h[r] & 63;
        ++r;
        static const int kLen[4] = {0, 1, 4, -1};
        int len = kLen[pli];
        if (len < 0) {
            if (r >= h.size()) break;
            if (h[r] & 0x80) {
                if (r + 1 >= h.size()) break;
                len = ((h[r] & 127) << 8) | h[r + 1];
                r += 2;
            } else {
                len = h[r];
                r += 1;
            }
        }
        if (id == 12 && len > 1)
            for (std::size_t i = r + 1; i < r + static_cast<std::size_t>(len) && i < h.size(); ++i) name += static_cast<char>(h[i]);
        r += static_cast<std::size_t>(len);
    }
    if (ctype == 2 && (csub == 1 || csub == 3)) {
        Picture pic;
        pic.name = name;
        pic.mime = csub == 3 ? "image/png" : "image/jpeg";
        pic.bytes = b;
        ready.push_back(pic);
    }
}

// ---- Journaline ---------------------------------------------------------------------
// A packet-mode stream (as the SlideShow's) of MSC data groups (type 0, CRC),
// each an NML object: a 3-byte header (object ID; type 3 bits: 1 menu, 2 plain
// text, 3 title only, 4 list; static; compressed; revision), then, if
// compressed, 0x08 and a raw deflate body.  Inside: 0x01 + the title; then a
// plain text's 0x03 + text, a menu's 0x02 + a 2-byte link + text each, or a
// list's 0x04 (a new row) / 0x05 (the next column) + text.  A text runs to the
// next byte below 0x10; data sections (0x1a/0x1b + length) are skipped; within
// text, 0x10 is a line break, 0x12/0x13 emphasis, 0x1c/0x1d skip the next
// byte.  Each object is kept when new or revised.
void Journaline::group(const Bytes& g) {
    if (g.size() < 6) return;
    const int ext = g[0] >> 7, crcF = (g[0] >> 6) & 1, segF = (g[0] >> 5) & 1, uaF = (g[0] >> 4) & 1, type = g[0] & 15;
    if (type != 0 || (crcF && crc16(&g[0], g.size() - 2) != ((static_cast<unsigned>(g[g.size() - 2]) << 8) | g[g.size() - 1]))) return;
    std::size_t q = 2 + (ext ? 2 : 0) + (segF ? 2 : 0);
    if (uaF) {
        if (q >= g.size()) return;
        q += 1 + (g[q] & 15);
    }
    const std::size_t end = g.size() - (crcF ? 2 : 0);
    if (q + 4 > end) return;
    const unsigned char* b = &g[q];
    const std::size_t bn = end - q;
    const unsigned id = (static_cast<unsigned>(b[0]) << 8) | b[1];
    const int ot = b[2] >> 5, comp = (b[2] >> 3) & 1, rev = b[2] & 7;
    char key[64];
    std::snprintf(key, sizeof key, "%u:%d:%u", id, rev, static_cast<unsigned>(bn));
    if (ot < 1 || ot > 4) return;
    std::map<unsigned, std::string>::const_iterator it = seen_.find(id);
    if (it != seen_.end() && it->second == key) return;
    JournalinePage page;
    bool ok;
    if (!comp)
        ok = nmlParse(id, ot, b + 3, bn - 3, page);
    else {
        Bytes raw;
        ok = b[3] == 0x08 && inflateRaw(b + 4, bn - 4, raw, 65536) && nmlParse(id, ot, raw.empty() ? 0 : &raw[0], raw.size(), page);
    }
    if (!ok) return;
    seen_[id] = key;
    pages[id] = page;
    ++newPages;
}

namespace {

struct NmlReader {
    const unsigned char* p;
    std::size_t n, i;
    std::string text() {
        std::string out;
        bool emph = false;
        while (i < n) {
            const unsigned c = p[i];
            if (c == 0x1a || c == 0x1b) {
                i += 3 + (i + 1 < n ? p[i + 1] : 0);
                continue;
            }
            if (c < 0x10) break;
            if (c == 0x10)
                out += '\n';
            else if (c == 0x12) {
                if (!emph) {
                    out += '*';
                    emph = true;
                }
            } else if (c == 0x13) {
                if (emph) {
                    out += '*';
                    emph = false;
                }
            } else if (c == 0x1c || c == 0x1d)
                ++i;
            else if (c >= 0x20)
                out += static_cast<char>(c);
            ++i;
        }
        if (emph) out += '*';
        // trim
        std::size_t a = 0, e = out.size();
        while (a < e && (out[a] == ' ' || out[a] == '\n')) ++a;
        while (e > a && (out[e - 1] == ' ' || out[e - 1] == '\n')) --e;
        return out.substr(a, e - a);
    }
};

}  // namespace

bool nmlParse(unsigned id, int ot, const unsigned char* p, std::size_t n, JournalinePage& o) {
    NmlReader r = {p, n, 0};
    if (n < 1 || p[0] != 0x01) return false;
    r.i = 1;
    o = JournalinePage();
    o.id = id;
    o.type = ot;
    o.title = r.text();
    if (ot == 2) {
        if (r.i < n && p[r.i] == 0x03) {
            ++r.i;
            o.text = r.text();
        }
    } else if (ot == 1) {
        while (r.i + 3 <= n && p[r.i] == 0x02) {
            const unsigned link = (static_cast<unsigned>(p[r.i + 1]) << 8) | p[r.i + 2];
            r.i += 3;
            o.links.push_back(std::make_pair(link, r.text()));
        }
    } else if (ot == 4) {
        bool haveRow = false;
        while (r.i < n && (p[r.i] == 0x04 || p[r.i] == 0x05)) {
            const bool nr = p[r.i] == 0x04;
            ++r.i;
            const std::string t = r.text();
            if (nr || !haveRow) {
                o.rows.push_back(std::vector<std::string>(1, t));
                haveRow = true;
            } else
                o.rows.back().push_back(t);
        }
    }
    return true;
}

// ---- raw inflate (RFC 1951), after Mark Adler's puff ------------------------------------
namespace {

struct Huffman {
    short count[16];
    short symbol[320];
};

struct Inflater {
    const unsigned char* in;
    std::size_t n, pos;
    unsigned long bitbuf;
    int bitcnt;
    Bytes* out;
    std::size_t maxOut;
    bool err;

    int bits(int need) {
        unsigned long val = bitbuf;
        while (bitcnt < need) {
            if (pos >= n) {
                err = true;
                return 0;
            }
            val |= static_cast<unsigned long>(in[pos++]) << bitcnt;
            bitcnt += 8;
        }
        bitbuf = val >> need;
        bitcnt -= need;
        return static_cast<int>(val & ((1UL << need) - 1));
    }
    int decode(const Huffman& h) {
        int code = 0, first = 0, index = 0;
        for (int len = 1; len < 16; ++len) {
            code |= bits(1);
            if (err) return -1;
            const int count = h.count[len];
            if (code - count < first) return h.symbol[index + (code - first)];
            index += count;
            first += count;
            first <<= 1;
            code <<= 1;
        }
        return -1;
    }
};

int construct(Huffman& h, const short* length, int n) {
    for (int len = 0; len < 16; ++len) h.count[len] = 0;
    for (int s = 0; s < n; ++s) h.count[length[s]]++;
    if (h.count[0] == n) return 0;
    int left = 1;
    for (int len = 1; len < 16; ++len) {
        left <<= 1;
        left -= h.count[len];
        if (left < 0) return left;
    }
    short offs[16];
    offs[1] = 0;
    for (int len = 1; len < 15; ++len) offs[len + 1] = static_cast<short>(offs[len] + h.count[len]);
    for (int s = 0; s < n; ++s)
        if (length[s] != 0) h.symbol[offs[length[s]]++] = static_cast<short>(s);
    return left;
}

bool codes(Inflater& s, const Huffman& lencode, const Huffman& distcode) {
    static const short lens[29] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
    static const short lext[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
    static const short dists[30] = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
    static const short dext[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};
    for (;;) {
        int symbol = s.decode(lencode);
        if (symbol < 0 || s.err) return false;
        if (symbol < 256) {
            if (s.out->size() >= s.maxOut) return false;
            s.out->push_back(static_cast<unsigned char>(symbol));
        } else if (symbol == 256)
            return true;
        else {
            symbol -= 257;
            if (symbol >= 29) return false;
            const int len = lens[symbol] + s.bits(lext[symbol]);
            const int ds = s.decode(distcode);
            if (ds < 0 || ds >= 30 || s.err) return false;
            const std::size_t dist = static_cast<std::size_t>(dists[ds] + s.bits(dext[ds]));
            if (s.err || dist > s.out->size() || s.out->size() + static_cast<std::size_t>(len) > s.maxOut) return false;
            for (int i = 0; i < len; ++i) s.out->push_back((*s.out)[s.out->size() - dist]);
        }
    }
}

}  // namespace

bool inflateRaw(const unsigned char* in, std::size_t n, Bytes& out, std::size_t maxOut) {
    out.clear();
    Inflater s = {in, n, 0, 0, 0, &out, maxOut, false};
    int last;
    do {
        last = s.bits(1);
        const int type = s.bits(2);
        if (s.err) return false;
        if (type == 0) {  // stored
            s.bitbuf = 0;
            s.bitcnt = 0;
            if (s.pos + 4 > n) return false;
            const unsigned len = in[s.pos] | (in[s.pos + 1] << 8);
            const unsigned nlen = in[s.pos + 2] | (in[s.pos + 3] << 8);
            s.pos += 4;
            if (len != (~nlen & 0xffff) || s.pos + len > n || out.size() + len > maxOut) return false;
            out.insert(out.end(), in + s.pos, in + s.pos + len);
            s.pos += len;
        } else if (type == 1) {  // fixed codes
            Huffman lc, dc;
            short lengths[320];
            int sym = 0;
            for (; sym < 144; ++sym) lengths[sym] = 8;
            for (; sym < 256; ++sym) lengths[sym] = 9;
            for (; sym < 280; ++sym) lengths[sym] = 7;
            for (; sym < 288; ++sym) lengths[sym] = 8;
            construct(lc, lengths, 288);
            for (sym = 0; sym < 30; ++sym) lengths[sym] = 5;
            construct(dc, lengths, 30);
            if (!codes(s, lc, dc)) return false;
        } else if (type == 2) {  // dynamic codes
            static const short order[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
            short lengths[320];
            const int nlen = s.bits(5) + 257, ndist = s.bits(5) + 1, ncode = s.bits(4) + 4;
            if (s.err || nlen > 286 || ndist > 30) return false;
            int index = 0;
            for (; index < ncode; ++index) lengths[order[index]] = static_cast<short>(s.bits(3));
            for (; index < 19; ++index) lengths[order[index]] = 0;
            Huffman lc, dc;
            if (construct(lc, lengths, 19) != 0) return false;
            index = 0;
            while (index < nlen + ndist) {
                int symbol = s.decode(lc);
                if (symbol < 0 || s.err) return false;
                if (symbol < 16)
                    lengths[index++] = static_cast<short>(symbol);
                else {
                    int len = 0;
                    if (symbol == 16) {
                        if (index == 0) return false;
                        len = lengths[index - 1];
                        symbol = 3 + s.bits(2);
                    } else if (symbol == 17)
                        symbol = 3 + s.bits(3);
                    else
                        symbol = 11 + s.bits(7);
                    if (s.err || index + symbol > nlen + ndist) return false;
                    while (symbol--) lengths[index++] = static_cast<short>(len);
                }
            }
            if (lengths[256] == 0) return false;
            const int e1 = construct(lc, lengths, nlen);
            if (e1 < 0 || (e1 > 0 && nlen - lc.count[0] != 1)) return false;
            const int e2 = construct(dc, lengths + nlen, ndist);
            if (e2 < 0 || (e2 > 0 && ndist - dc.count[0] != 1)) return false;
            if (!codes(s, lc, dc)) return false;
        } else
            return false;
    } while (!last);
    return true;
}

}  // namespace drm
}  // namespace dec
