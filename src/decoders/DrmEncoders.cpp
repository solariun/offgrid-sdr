// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DrmEncoders.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  drmSignal() (DrmEncoders.h): a real DRM signal, built with the receiver's
//  own library (its tables, coding, interleaving, scrambling and CRCs):
//    - the FAC (72 bits: the signal's settings, one service's parameters,
//      CRC-8) in every frame, 4-QAM;
//    - the SDC (symbols 0-1, or 0-2 in modes C and D, of each superframe's
//      first frame): the multiplex description (type 0), the SlideShow and
//      Journaline applications (type 5), the label (type 1), the audio
//      settings (type 9, with xHE-AAC's configuration), CRC-16;
//    - the main channel: per multiplex frame (400 ms), the audio super frame
//      (AAC: 12-bit frame borders, the CRC bytes, the access units; xHE-AAC:
//      packed on schedule by drmXhePack), the text message's 4 bytes, the
//      packets of the data stream; scrambled, coded on the levels (16-QAM 2,
//      64-QAM 3; all but 64-QAM's first bit-interleaved), mapped, then the
//      cell interleaving (short, or long over 5 frames);
//    - every cell in Dream's order of precedence (gain references, time
//      references, frequency pilots, the empty DC carrier, FAC, SDC), then
//      each symbol by an inverse DFT, with its guard interval.
//
//  Port of OFFgrid-SDR's JS drmDemoSignal and drmXhePack (diddy-boy,
//  https://github.com/solariun/offgrid-sdr, index.html), which follow
//  Dream (TU Darmstadt, Volker Fischer and others; GPL-2.0-or-later,
//  licenses/dream-COPYING.txt).
//
//  Differences from the JS (each an Addition to the JS, which built mode B,
//  10 kHz, 64-QAM, 4-QAM SDC, short interleaving only):
//    - any robustness mode and occupancy (mode A's carriers +-1 and the DC
//      carrier empty), 16-QAM, any protection level, long interleaving, a
//      16-QAM SDC (two levels, rates 2 and 7, as the receiver reads it);
//    - AAC at 12 kHz (5 frames a super frame) as well as 24 kHz (10), SBR
//      and stereo flags, and the caller's own access units;
//    - the dummy cells (past the multiplex frames) in the main channel's
//      own constellation (the JS: always 64-QAM).
//
//  How it fits in
//  --------------
//  Uses DrmInternal.h (the receiver's tables and coding) and DrmDemoData.cpp
//  (the demo's jingle, xHE-AAC frames and picture).
//
//  Contributing
//  ------------
//  Keep the demo stations exactly as the JS's (drmDemoStation): the lead's
//  demo band and the tests rely on them.
// ============================================================================
#include "decoders/DrmEncoders.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "decoders/DrmInternal.h"

namespace dec {

namespace {

using drm::Bytes;

const double kPi = 3.14159265358979323846;
const double kSqrt2 = 1.41421356237309504880;

void putBits(Bytes& v, unsigned long long val, int n) {
    for (int i = n - 1; i >= 0; --i) v.push_back(static_cast<unsigned char>((val >> i) & 1));
}

Bytes utf8(const std::string& s) { return Bytes(s.begin(), s.end()); }

Bytes withCrc16(const Bytes& b) {
    Bytes o(b);
    const unsigned c = drm::crc16(b.empty() ? 0 : &b[0], b.size());
    o.push_back(static_cast<unsigned char>(c >> 8));
    o.push_back(static_cast<unsigned char>(c & 255));
    return o;
}

// MSC data groups -> packets of PL data bytes (header, data, CRC-16)
void toPackets(const std::vector<Bytes>& groups, int PL, int pid, std::vector<Bytes>& out) {
    int ci = 0;
    for (std::size_t q = 0; q < groups.size(); ++q) {
        const Bytes& g = groups[q];
        for (std::size_t i = 0; i < g.size(); i += static_cast<std::size_t>(PL)) {
            const std::size_t e = std::min(g.size(), i + static_cast<std::size_t>(PL));
            const bool first = i == 0, last = i + static_cast<std::size_t>(PL) >= g.size(), pad = e - i < static_cast<std::size_t>(PL);
            Bytes body;
            if (pad) {
                body.push_back(static_cast<unsigned char>(e - i));
                body.insert(body.end(), g.begin() + static_cast<std::ptrdiff_t>(i), g.begin() + static_cast<std::ptrdiff_t>(e));
                body.resize(static_cast<std::size_t>(PL), 0);
            } else
                body.assign(g.begin() + static_cast<std::ptrdiff_t>(i), g.begin() + static_cast<std::ptrdiff_t>(e));
            Bytes pk;
            pk.push_back(static_cast<unsigned char>((first ? 0x80 : 0) | (last ? 0x40 : 0) | (pid << 4) | (pad ? 0x08 : 0) | (ci++ & 7)));
            pk.insert(pk.end(), body.begin(), body.end());
            out.push_back(withCrc16(pk));
        }
    }
}

// The other way from the receiver's XheFrames: xHE-AAC frames packed into
// nMux audio super frames of lenBytes each, filling them exactly, so a loop
// of them restarts on a frame boundary with nothing carried over.  The spare
// capacity goes as zero bytes after each frame's audio (decoders ignore what
// follows a frame's last element).  A frame's end needs its 2-byte
// directory entry in the super frame where it ends.  Each frame should end
// near its due time (an even share of the loop's bytes), as from a real
// encoder: a small frame is padded up to it; a big one runs a little late,
// made up by the small ones after it.
bool xhePack(const std::vector<Bytes>& frames, int nMux, int lenBytes, std::vector<Bytes>& out, std::string& why) {
    const int F = static_cast<int>(frames.size());
    const long long total = static_cast<long long>(nMux) * (lenBytes - 2) - 2LL * F;
    long long raw = 0;
    for (int i = 0; i < F; ++i) raw += static_cast<long long>(frames[static_cast<std::size_t>(i)].size());
    if (F == 0 || raw > total) {
        why = "xHE-AAC frames too big for the channel";
        return false;
    }
    for (int jit = 0; jit < 64; ++jit) {  // (an end that can't take its entry in time: due times nudged)
        std::vector<long long> ends;
        long long e = 0;
        for (int i = 0; i < F; ++i) {
            const long long due = i == F - 1 ? total : static_cast<long long>(std::floor((i + 1) * static_cast<double>(total) / F + 0.5)) - (jit % 8) * (i % 3);
            e = std::max(e + static_cast<long long>(frames[static_cast<std::size_t>(i)].size()), due);
            ends.push_back(e);
        }
        if (e != total) continue;
        struct Sf {
            long long S, P;
            std::vector<long long> borders;
        };
        std::vector<Sf> sfs;
        long long S = 0;
        int j = 0;
        bool ok = true;
        for (int k = 0; k < nMux && ok; ++k) {
            std::vector<long long> mine;
            while (j < F && ends[static_cast<std::size_t>(j)] - S <= lenBytes - 2 - 2 * static_cast<long long>(mine.size() + 1)) mine.push_back(ends[static_cast<std::size_t>(j++)]);
            const long long P = lenBytes - 2 - 2 * static_cast<long long>(mine.size());
            if (mine.size() > 15) {
                ok = false;
                break;
            }
            Sf sf;
            sf.S = S;
            sf.P = P;
            for (std::size_t q = 0; q < mine.size(); ++q) sf.borders.push_back(mine[q] - S);
            sfs.push_back(sf);
            S += P;
        }
        if (!ok || j != F || S != e) continue;
        Bytes stream(static_cast<std::size_t>(e), 0);
        long long q = 0;
        for (int i = 0; i < F; ++i) {
            const Bytes& f = frames[static_cast<std::size_t>(i)];
            std::copy(f.begin(), f.end(), stream.begin() + q);
            q = ends[static_cast<std::size_t>(i)];
        }
        out.clear();
        for (std::size_t k = 0; k < sfs.size(); ++k) {
            Bytes by(static_cast<std::size_t>(lenBytes), 0);
            const int fbc = static_cast<int>(sfs[k].borders.size());
            by[0] = static_cast<unsigned char>(fbc << 4);
            by[1] = static_cast<unsigned char>(drm::crcDream(&by[0], 1, 8));
            std::copy(stream.begin() + sfs[k].S, stream.begin() + sfs[k].S + sfs[k].P, by.begin() + 2);
            for (int i = fbc - 1, at = lenBytes - 2 * fbc; i >= 0; --i, at += 2) {
                const long long v = (sfs[k].borders[static_cast<std::size_t>(i)] << 4) | fbc;
                by[static_cast<std::size_t>(at)] = static_cast<unsigned char>((v >> 8) & 255);
                by[static_cast<std::size_t>(at + 1)] = static_cast<unsigned char>(v & 255);
            }
            out.push_back(by);
        }
        return true;
    }
    why = "couldn't pack the xHE-AAC frames";
    return false;
}

struct Rng {  // (the JS's: Park-Miller)
    long long sd;
    double next() {
        sd = (sd * 16807) % 2147483647;
        return static_cast<double>(sd) / 2147483647;
    }
};

}  // namespace

DrmTxOptions drmDemoStation(int which) {
    DrmTxOptions o;
    o.extras = true;
    if (which == 1) {
        o.label = "OFFgrid xHE";
        o.serviceId = 0xf06002;
        o.superframes = 32;
        o.text = "OFFgrid-SDR demo: DRM in xHE-AAC, with station text and a SlideShow picture";
        o.xhe = true;
        const std::vector<std::vector<unsigned char> > f = drmDemoXheFrames();
        for (int r = 0; r < 3; ++r) o.xheFrames.insert(o.xheFrames.end(), f.begin(), f.end());
        o.xheConfig.push_back(0xa0);
        o.xheConfig.push_back(0x10);
        o.aacRateCode = 4;  // (DRM's xHE-AAC rate table: 24 kHz)
        o.journaline = false;
    }
    return o;
}

double drmDemoStationHz(int which) { return which == 1 ? 11750e3 : 11700e3; }
double drmDemoStationAmplitude(int which) {
    (void)which;
    return 0.05;
}

bool drmSignal(const DrmTxOptions& opt, std::vector<std::complex<float> >& iq, std::string* error, DrmTxReport* report) {
    std::string why;
    struct Fail {
        std::string* e;
        bool operator()(const std::string& w) const {
            if (e) *e = w;
            return false;
        }
    };
    Fail fail = {error};
    const int mi = opt.mode, so = opt.occupancy;
    if (mi < 0 || mi > 3) return fail("no such robustness mode");
    if (so < 0 || so > 5) return fail("no such spectrum occupancy");
    const int kmin = drm::kMin(so, mi), kmax = drm::kMax(so, mi);
    if (kmin == 0 && kmax == 0) return fail("this occupancy isn't defined for this robustness mode");
    const int Tu = drm::kTu[mi], Tg = drm::kTg[mi], LS = drm::kSymbols[mi], NK = kmax - kmin + 1;
    const drm::ModeTable& tb = drm::modeTable(mi);
    const int* boost = &tb.boost[so * 4];
    Rng rng = {12345};
    double A64[8], A16[4];
    {
        static const int a64[8] = {7, -1, 3, -5, 5, -3, 1, -7};
        static const int a16[4] = {3, -1, 1, -3};
        for (int j = 0; j < 8; ++j) A64[j] = a64[j] / std::sqrt(42.0);
        for (int j = 0; j < 4; ++j) A16[j] = a16[j] / std::sqrt(10.0);
    }
    const bool q16 = opt.qam16;
    const int prot = opt.protection >= 0 ? opt.protection : (opt.extras ? 1 : 0);
    if (q16 ? prot > 1 : prot > 3) return fail("no such protection level");
    struct Q {
        Rng* r;
        const double* a;
        int n;
        std::complex<double> operator()() const {
            const double x = a[static_cast<int>(r->next() * n)], y = a[static_cast<int>(r->next() * n)];
            return std::complex<double>(x, y);
        }
    };
    Q dummy = {&rng, q16 ? A16 : A64, q16 ? 4 : 8};
    const std::complex<double> qpsk[4] = {std::complex<double>(1 / kSqrt2, 1 / kSqrt2), std::complex<double>(1 / kSqrt2, -1 / kSqrt2),
                                          std::complex<double>(-1 / kSqrt2, 1 / kSqrt2), std::complex<double>(-1 / kSqrt2, -1 / kSqrt2)};

    // ---- the cells: which are the SDC's and which the main channel's --------------
    std::set<int> fps, tps;
    for (int j = 0; j < tb.nFreq; ++j) fps.insert(tb.freq[2 * j]);
    for (int j = 0; j < tb.nTime; ++j) tps.insert(tb.time[2 * j]);
    std::set<long long> facSet;
    for (int j = 0; j < tb.nFac; ++j) facSet.insert(static_cast<long long>(tb.fac[2 * j]) * 65536 + tb.fac[2 * j + 1] + 32768);
    std::vector<std::set<int> > gain(static_cast<std::size_t>(LS));
    std::vector<std::vector<drm::IntPair> > gainPh(static_cast<std::size_t>(LS));
    for (int s = 0; s < LS; ++s) {
        drm::gainRefs(mi, s, kmin, kmax, gainPh[static_cast<std::size_t>(s)]);
        for (std::size_t q = 0; q < gainPh[static_cast<std::size_t>(s)].size(); ++q) gain[static_cast<std::size_t>(s)].insert(gainPh[static_cast<std::size_t>(s)][q].first);
    }
    const int sdcSyms = mi == 0 || mi == 1 ? 2 : 3;
    std::vector<drm::IntPair> sdcPos;
    std::set<long long> sdcSet;
    for (int s = 0; s < sdcSyms; ++s)
        for (int k = kmin; k <= kmax; ++k)
            if (k && !(mi == 0 && std::abs(k) == 1) && !gain[static_cast<std::size_t>(s)].count(k) && !fps.count(k) && !(s == 0 && tps.count(k))) {
                sdcPos.push_back(drm::IntPair(s, k));
                sdcSet.insert(static_cast<long long>(s) * 65536 + k + 32768);
            }
    // the main channel's cells, in order: frame, symbol, carrier
    std::vector<int> mscIndex(static_cast<std::size_t>(3 * LS * NK), -1);
    int mscCount = 0;
    for (int f = 0; f < 3; ++f)
        for (int s = 0; s < LS; ++s)
            for (int k = kmin; k <= kmax; ++k) {
                const long long key = static_cast<long long>(s) * 65536 + k + 32768;
                if (k && !(mi == 0 && std::abs(k) == 1) && !gain[static_cast<std::size_t>(s)].count(k) && !fps.count(k) && !(s == 0 && tps.count(k)) &&
                    !facSet.count(key) && !(f == 0 && sdcSet.count(key)))
                    mscIndex[static_cast<std::size_t>((f * LS + s) * NK + k - kmin)] = mscCount++;
            }

    // ---- the main channel's code ------------------------------------------------------
    static const int r64[4][3] = {{0, 4, 9}, {2, 7, 10}, {4, 9, 11}, {7, 10, 12}};
    static const int r16[2][2] = {{2, 7}, {4, 9}};
    const int nl = q16 ? 2 : 3, NMUX = mscCount / 3;
    if (NMUX < 64) return fail("too few cells for a main channel");
    int rates[3], Ms[3], Mtot = 0;
    drm::Pattern pats[3];
    std::vector<int> ils[3];
    bool hasIl[3] = {false, false, false};
    for (int j = 0; j < nl; ++j) {
        rates[j] = q16 ? r16[prot][j] : r64[prot][j];
        Ms[j] = drm::punct(rates[j])[0] * ((2 * NMUX - 12) / drm::punct(rates[j])[1]);
        drm::makePattern(rates[j], Ms[j], NMUX, false, pats[j]);
        Mtot += Ms[j];
        if (q16 || j > 0) {
            drm::ilTable(2 * NMUX, (q16 ? j == 0 : j == 1) ? 13 : 21, ils[j]);
            hasIl[j] = true;
        }
    }
    std::vector<int> cit;
    drm::ilTable(NMUX, 5, cit);
    const int lenBytes = Mtot / 8;
    Bytes mpr;
    drm::prbs(static_cast<std::size_t>(Mtot), mpr);
    // (the extras: a data stream of 3 packets a frame, each 58 bytes of data +
    //  header and CRC; the audio stream the rest, its last 4 bytes the text)
    const bool ex = opt.extras;
    const bool wantSlides = ex && opt.slideshow, wantJl = ex && opt.journaline && !opt.xhe;
    const int PL = 58, NPK = 3, lenData = (wantSlides || wantJl) ? NPK * (PL + 3) : 0, lenAudio = lenBytes - lenData,
              sfLen = ex ? lenAudio - 4 : lenBytes;
    if (lenAudio < 40) return fail("the channel is too small for the audio and the data");

    // ---- the text message, the SlideShow, Journaline ---------------------------------------
    Bytes textBytes;
    std::vector<Bytes> pk;
    std::string textSent;
    if (ex) {
        // the text message: segments of up to 16 bytes, each after four 0xFF
        // bytes, with a 2-byte header and a CRC
        textSent = opt.text.empty() ? std::string("OFFgrid-SDR demo: DRM digital radio with station text and a SlideShow picture") : opt.text;
        const Bytes msg = utf8(textSent);
        for (std::size_t i = 0, n = 0; i < msg.size(); i += 16, ++n) {
            const std::size_t e = std::min(msg.size(), i + 16);
            const bool first = i == 0, last = i + 16 >= msg.size();
            Bytes seg;
            seg.push_back(static_cast<unsigned char>((first ? 0x40 : 0) | (last ? 0x20 : 0) | (e - i - 1)));
            seg.push_back(static_cast<unsigned char>(first ? 0 : (n << 4)));
            seg.insert(seg.end(), msg.begin() + static_cast<std::ptrdiff_t>(i), msg.begin() + static_cast<std::ptrdiff_t>(e));
            seg = withCrc16(seg);
            for (int q = 0; q < 4; ++q) textBytes.push_back(0xff);
            textBytes.insert(textBytes.end(), seg.begin(), seg.end());
        }
        while (textBytes.size() % 4) textBytes.push_back(0);
        if (wantSlides) {
            // the picture: MOT in header mode (a header object naming the PNG,
            // then its body), in data groups with CRCs, in packets
            const Bytes png = opt.picture.empty() ? drmDemoSlidePng() : opt.picture;
            const Bytes name = utf8(opt.pictureName.empty() ? std::string("OFFgrid-SDR.png") : opt.pictureName);
            Bytes cn(1, 0);
            cn.insert(cn.end(), name.begin(), name.end());
            const unsigned long long hs = 7 + 2 + cn.size();
            const unsigned long long v = (static_cast<unsigned long long>(png.size()) << 28) | (hs << 15) | (2ULL << 9) | 3ULL;
            Bytes hdr;
            for (int i = 6; i >= 0; --i) hdr.push_back(static_cast<unsigned char>((v >> (8 * i)) & 255));
            hdr.push_back((3 << 6) | 12);
            hdr.push_back(static_cast<unsigned char>(cn.size()));
            hdr.insert(hdr.end(), cn.begin(), cn.end());
            const int TID = 0x2a51;
            std::vector<Bytes> dgs;
            for (int type = 3; type <= 4; ++type) {
                const Bytes& data = type == 3 ? hdr : png;
                const std::size_t seg = 900;
                for (std::size_t i = 0, n = 0; i < data.size() || n == 0; i += seg, ++n) {
                    const std::size_t e = std::min(data.size(), i + seg);
                    const bool lastSeg = i + seg >= data.size();
                    Bytes g;
                    g.push_back(static_cast<unsigned char>(0x70 | type));
                    g.push_back(0);
                    g.push_back(static_cast<unsigned char>((lastSeg ? 0x80 : 0) | (n >> 8)));
                    g.push_back(static_cast<unsigned char>(n & 255));
                    g.push_back(0x12);
                    g.push_back(static_cast<unsigned char>(TID >> 8));
                    g.push_back(static_cast<unsigned char>(TID & 255));
                    g.push_back(static_cast<unsigned char>(((e - i) >> 8) & 31));
                    g.push_back(static_cast<unsigned char>((e - i) & 255));
                    g.insert(g.end(), data.begin() + static_cast<std::ptrdiff_t>(i), data.begin() + static_cast<std::ptrdiff_t>(e));
                    dgs.push_back(withCrc16(g));
                }
            }
            toPackets(dgs, PL, 0, pk);
        }
        if (wantJl) {
            // Journaline pages in the same stream as packet ID 1: a menu, an
            // article, a table and some tips, built to the standard (ETSI TS
            // 102 979), uncompressed, each an NML object in an MSC data group
            // with a CRC
            struct J {
                static Bytes hdr(unsigned id, int type) {
                    Bytes b;
                    b.push_back(static_cast<unsigned char>(id >> 8));
                    b.push_back(static_cast<unsigned char>(id & 255));
                    b.push_back(static_cast<unsigned char>((type << 5) | 0x10));
                    return b;
                }
                static void add(Bytes& b, int c) { b.push_back(static_cast<unsigned char>(c)); }
                static void add(Bytes& b, const char* s) {
                    const Bytes u = utf8(s);
                    b.insert(b.end(), u.begin(), u.end());
                }
            };
            std::vector<Bytes> objs(4);
            {
                Bytes& o = objs[0];
                o = J::hdr(0x0000, 1);
                J::add(o, 0x01);
                J::add(o, "OFFgrid-SDR demo news");
                J::add(o, 0x02), J::add(o, 0x01), J::add(o, 0x01), J::add(o, "About Journaline");
                J::add(o, 0x02), J::add(o, 0x01), J::add(o, 0x02), J::add(o, "The demo's DRM stations");
                J::add(o, 0x02), J::add(o, 0x01), J::add(o, 0x03), J::add(o, "Off-grid listening tips");
            }
            {
                Bytes& o = objs[1];
                o = J::hdr(0x0101, 2);
                J::add(o, 0x01);
                J::add(o, "Journaline: news pages with the sound");
                J::add(o, 0x03);
                J::add(o, "Journaline sends pages of text alongside a DRM station's sound: menus, articles and lists, rather like teletext. "
                          "OFFgrid-SDR collects them as they arrive; open them with the ");
                J::add(o, 0x12), J::add(o, "News"), J::add(o, 0x13), J::add(o, " button.");
                J::add(o, 0x10);
                J::add(o, "These pages come from OFFgrid-SDR's demo, built to the standard. On air, All India Radio is Journaline's best-known user.");
            }
            {
                Bytes& o = objs[2];
                o = J::hdr(0x0102, 4);
                J::add(o, 0x01);
                J::add(o, "The demo's DRM stations");
                J::add(o, 0x04), J::add(o, 0x12), J::add(o, "Frequency"), J::add(o, 0x13), J::add(o, 0x05), J::add(o, 0x12), J::add(o, "Station"), J::add(o, 0x13);
                J::add(o, 0x04), J::add(o, "11.700 MHz"), J::add(o, 0x05), J::add(o, "OFFgrid DRM (AAC)");
                J::add(o, 0x04), J::add(o, "11.750 MHz"), J::add(o, 0x05), J::add(o, "OFFgrid xHE (xHE-AAC)");
            }
            {
                Bytes& o = objs[3];
                o = J::hdr(0x0103, 2);
                J::add(o, 0x01);
                J::add(o, "Off-grid listening tips");
                J::add(o, 0x03);
                J::add(o, "A long wire and a good earth often beat a short active antenna on shortwave.");
                J::add(o, 0x10);
                J::add(o, "DRM spreads each moment over 2 seconds of signal, so short fades pass unheard; deep ones are bridged by the next good window.");
            }
            std::vector<Bytes> gs;
            for (std::size_t q = 0; q < objs.size(); ++q) {
                Bytes g0;
                g0.push_back(0x40);
                g0.push_back(0);
                g0.insert(g0.end(), objs[q].begin(), objs[q].end());
                gs.push_back(withCrc16(g0));
            }
            toPackets(gs, PL, 1, pk);
        }
    }

    // ---- the SDC: the multiplex description, the applications, the label, the
    //      audio settings --------------------------------------------------------------
    const int N = static_cast<int>(sdcPos.size());
    int M0 = 0, M1 = 0;
    if (opt.sdc16) {
        M0 = drm::punct(2)[0] * ((2 * N - 12) / drm::punct(2)[1]);
        M1 = drm::punct(7)[0] * ((2 * N - 12) / drm::punct(7)[1]);
    } else
        M0 = drm::punct(4)[0] * ((2 * N - 12) / drm::punct(4)[1]);
    const int M = M0 + M1, useful = 20 + 8 * ((M - 20) / 8);
    Bytes sb;
    sb.reserve(static_cast<std::size_t>(M));
    putBits(sb, 0, 4);  // (the AFS index)
    const Bytes label = utf8(opt.label);
    if (label.size() > 64) return fail("the label is too long");
    const bool streams2 = lenData > 0;
    if (!streams2) {  // type 0: one stream, part B only
        putBits(sb, 3, 7), putBits(sb, 0, 1), putBits(sb, 0, 4), putBits(sb, 0, 2), putBits(sb, static_cast<unsigned>(prot), 2);
        putBits(sb, 0, 12), putBits(sb, static_cast<unsigned>(ex ? lenAudio : lenBytes), 12);
    } else {  // type 0: the audio and the data stream
        putBits(sb, 6, 7), putBits(sb, 0, 1), putBits(sb, 0, 4), putBits(sb, 0, 2), putBits(sb, static_cast<unsigned>(prot), 2);
        putBits(sb, 0, 12), putBits(sb, static_cast<unsigned>(lenAudio), 12), putBits(sb, 0, 12), putBits(sb, static_cast<unsigned>(lenData), 12);
    }
    if (wantSlides) {  // type 5: stream 1, packet mode, DAB's SlideShow
        putBits(sb, 4, 7), putBits(sb, 0, 1), putBits(sb, 5, 4), putBits(sb, 1, 2), putBits(sb, 1, 2), putBits(sb, 1, 1), putBits(sb, 1, 1);
        putBits(sb, 0, 2), putBits(sb, 1, 4), putBits(sb, PL, 8), putBits(sb, 0, 5), putBits(sb, 2, 11);
    }
    if (wantJl) {  // type 5 again: Journaline, packet ID 1, the same stream
        putBits(sb, 4, 7), putBits(sb, 0, 1), putBits(sb, 5, 4), putBits(sb, 2, 2), putBits(sb, 1, 2), putBits(sb, 1, 1), putBits(sb, 1, 1);
        putBits(sb, 1, 2), putBits(sb, 1, 4), putBits(sb, PL, 8), putBits(sb, 0, 5), putBits(sb, 0x44a, 11);
    }
    putBits(sb, label.size(), 7), putBits(sb, 0, 1), putBits(sb, 1, 4), putBits(sb, 0, 4);  // type 1: the label
    for (std::size_t q = 0; q < label.size(); ++q) putBits(sb, label[q], 8);
    if (!opt.xhe) {  // type 9: AAC
        putBits(sb, 2, 7), putBits(sb, 0, 1), putBits(sb, 9, 4), putBits(sb, 0, 4), putBits(sb, 0, 2), putBits(sb, opt.sbr ? 1 : 0, 1);
        putBits(sb, static_cast<unsigned>(opt.audioMode & 3), 2), putBits(sb, static_cast<unsigned>(opt.aacRateCode & 7), 3), putBits(sb, ex ? 1 : 0, 1),
            putBits(sb, 0, 7);
    } else {  // type 9: xHE-AAC, and its configuration
        putBits(sb, 2 + opt.xheConfig.size(), 7), putBits(sb, 0, 1), putBits(sb, 9, 4), putBits(sb, 0, 4), putBits(sb, 3, 2), putBits(sb, opt.sbr ? 1 : 0, 1);
        putBits(sb, static_cast<unsigned>(opt.audioMode & 3), 2), putBits(sb, static_cast<unsigned>(opt.aacRateCode & 7), 3), putBits(sb, ex ? 1 : 0, 1),
            putBits(sb, 0, 7);
        for (std::size_t q = 0; q < opt.xheConfig.size(); ++q) putBits(sb, opt.xheConfig[q], 8);
    }
    if (static_cast<int>(sb.size()) > useful - 16) return fail("the SDC is too long for its channel");
    sb.resize(static_cast<std::size_t>(M), 0);
    {
        Bytes bytes(1, static_cast<unsigned char>(drm::bitsNum(&sb[0], 0, 4)));
        for (int j = 0; j < (useful - 4) / 8 - 2; ++j) bytes.push_back(static_cast<unsigned char>(drm::bitsNum(&sb[0], static_cast<std::size_t>(4 + 8 * j), 8)));
        const unsigned c = drm::crcDream(&bytes[0], bytes.size(), 16);
        for (int j = 0; j < 16; ++j) sb[static_cast<std::size_t>(useful - 16 + j)] = static_cast<unsigned char>((c >> (15 - j)) & 1);
    }
    std::vector<std::complex<double> > sdcCells(static_cast<std::size_t>(N));
    {
        Bytes spr;
        drm::prbs(static_cast<std::size_t>(M), spr);
        for (int j = 0; j < M; ++j) sb[static_cast<std::size_t>(j)] ^= spr[static_cast<std::size_t>(j)];
        if (!opt.sdc16) {
            drm::Pattern pat;
            drm::makePattern(4, M0, N, false, pat);
            Bytes sc;
            drm::convEncode(&sb[0], sb.size(), pat, sc);
            std::vector<int> sil;
            drm::ilTable(2 * N, 21, sil);
            for (int j = 0; j < N; ++j)
                sdcCells[static_cast<std::size_t>(j)] = qpsk[(sc[static_cast<std::size_t>(sil[static_cast<std::size_t>(2 * j)])] << 1) |
                                                            sc[static_cast<std::size_t>(sil[static_cast<std::size_t>(2 * j + 1)])]];
        } else {
            drm::Pattern p0, p1;
            drm::makePattern(2, M0, N, false, p0);
            drm::makePattern(7, M1, N, false, p1);
            Bytes c0, c1;
            drm::convEncode(&sb[0], static_cast<std::size_t>(M0), p0, c0);
            drm::convEncode(&sb[static_cast<std::size_t>(M0)], static_cast<std::size_t>(M1), p1, c1);
            std::vector<int> il0, il1;
            drm::ilTable(2 * N, 13, il0);
            drm::ilTable(2 * N, 21, il1);
            for (int j = 0; j < N; ++j) {
                double v[2];
                for (int h = 0; h < 2; ++h) {
                    const int p = 2 * j + h;
                    v[h] = A16[(c0[static_cast<std::size_t>(il0[static_cast<std::size_t>(p)])] << 1) | c1[static_cast<std::size_t>(il1[static_cast<std::size_t>(p)])]];
                }
                sdcCells[static_cast<std::size_t>(j)] = std::complex<double>(v[0], v[1]);
            }
        }
    }

    // ---- the FAC, per frame identity 0..2 ------------------------------------------------
    std::vector<std::complex<double> > facCells[3];
    {
        Bytes pr;
        drm::prbs(72, pr);
        drm::Pattern fp;
        drm::makePattern(6, 72, 65, true, fp);
        std::vector<int> il;
        drm::ilTable(130, 21, il);
        for (int fr = 0; fr < 3; ++fr) {
            Bytes b;
            putBits(b, 0, 1), putBits(b, static_cast<unsigned>(fr), 2), putBits(b, static_cast<unsigned>(so), 4), putBits(b, opt.longInterleave ? 0 : 1, 1);
            putBits(b, q16 ? 3 : 0, 2), putBits(b, opt.sdc16 ? 0 : 1, 1), putBits(b, 4, 4), putBits(b, 0, 3), putBits(b, 0, 1), putBits(b, 0, 1);
            putBits(b, opt.serviceId & 0xffffff, 24), putBits(b, 0, 2), putBits(b, 0, 1), putBits(b, static_cast<unsigned>(opt.language & 15), 4);
            putBits(b, 0, 1), putBits(b, static_cast<unsigned>(opt.programmeType & 31), 5), putBits(b, 0, 1), putBits(b, 0, 6);
            unsigned char by[8];
            for (int j = 0; j < 8; ++j) by[j] = static_cast<unsigned char>(drm::bitsNum(&b[0], static_cast<std::size_t>(8 * j), 8));
            putBits(b, drm::crcDream(by, 8, 8), 8);
            for (int j = 0; j < 72; ++j) b[static_cast<std::size_t>(j)] ^= pr[static_cast<std::size_t>(j)];
            Bytes c;
            drm::convEncode(&b[0], b.size(), fp, c);
            for (int j = 0; j < tb.nFac; ++j)
                facCells[fr].push_back(qpsk[(c[static_cast<std::size_t>(il[static_cast<std::size_t>(2 * j)])] << 1) | c[static_cast<std::size_t>(il[static_cast<std::size_t>(2 * j + 1)])]]);
        }
    }

    // ---- the multiplex frames' bytes and cells ------------------------------------------------
    const int NSF = opt.superframes > 0 ? opt.superframes : 10, F = NSF * 3;
    std::vector<std::vector<unsigned char> > aac = opt.aac;
    if (!opt.xhe && aac.empty()) aac = drmDemoAacFrames();
    std::vector<Bytes> xhePacked;
    if (opt.xhe && !xhePack(opt.xheFrames, F, sfLen, xhePacked, why)) return fail(why);
    if (report) {
        report->lenBytes = lenBytes;
        report->lenAudio = ex ? lenAudio : lenBytes;
        report->nmux = NMUX;
        report->muxFrames = F;
        report->audio.assign(static_cast<std::size_t>(F), std::vector<std::vector<unsigned char> >());
        report->text = textSent;
    }
    std::vector<std::vector<std::complex<double> > > orig(static_cast<std::size_t>(F));
    {
        Bytes by, mb, lv[3], code[3];
        for (int g = 0; g < F; ++g) {
            by.assign(static_cast<std::size_t>(lenBytes), 0);
            if (opt.xhe) {
                const Bytes& x = xhePacked[static_cast<std::size_t>(g) % xhePacked.size()];
                std::copy(x.begin(), x.begin() + sfLen, by.begin());
                if (report) report->audio[static_cast<std::size_t>(g)].push_back(x);
            } else {
                // the audio super frame: frame borders (12 bits each), the CRC
                // bytes, then the frames
                const int nf = opt.aacRateCode == 3 ? 10 : 5, hdr = (12 * (nf - 1) + (nf == 10 ? 4 : 0)) / 8;
                std::vector<const std::vector<unsigned char>*> fr;
                int sum = 0;
                for (int j = 0; j < nf; ++j) {
                    fr.push_back(&aac[(static_cast<std::size_t>(g) * nf + j) % aac.size()]);
                    if (fr.back()->empty()) return fail("an empty access unit");
                    sum += static_cast<int>(fr.back()->size()) - 1;
                }
                if (hdr + nf + sum > sfLen) return fail("the audio is too big for the channel");
                int acc = 0, bp = 0;
                for (int j = 0; j + 1 < nf; ++j) {
                    acc += static_cast<int>(fr[static_cast<std::size_t>(j)]->size()) - 1;
                    for (int i = 11; i >= 0; --i, ++bp)
                        if (((acc % 4096) >> i) & 1) by[static_cast<std::size_t>(bp >> 3)] |= static_cast<unsigned char>(0x80 >> (bp & 7));
                }
                int o = hdr;
                for (int j = 0; j < nf; ++j) by[static_cast<std::size_t>(o++)] = (*fr[static_cast<std::size_t>(j)])[0];
                for (int j = 0; j < nf; ++j) {
                    std::copy(fr[static_cast<std::size_t>(j)]->begin() + 1, fr[static_cast<std::size_t>(j)]->end(), by.begin() + o);
                    o += static_cast<int>(fr[static_cast<std::size_t>(j)]->size()) - 1;
                    if (report) report->audio[static_cast<std::size_t>(g)].push_back(*fr[static_cast<std::size_t>(j)]);
                }
            }
            // (the extras, after either kind of audio: the text's 4 bytes, then
            //  the data stream's packets)
            if (ex) {
                for (int j = 0; j < 4; ++j) by[static_cast<std::size_t>(sfLen + j)] = textBytes[(static_cast<std::size_t>(g) * 4 + j) % textBytes.size()];
                if (!pk.empty())
                    for (int j = 0; j < NPK; ++j) {
                        const Bytes& p = pk[(static_cast<std::size_t>(g) * NPK + j) % pk.size()];
                        std::copy(p.begin(), p.end(), by.begin() + lenAudio + j * (PL + 3));
                    }
            }
            // scrambled, coded on the levels, mapped
            mb.assign(static_cast<std::size_t>(Mtot), 0);
            for (int j = 0; j < Mtot; ++j)
                mb[static_cast<std::size_t>(j)] = static_cast<unsigned char>(((j < 8 * lenBytes ? (by[static_cast<std::size_t>(j >> 3)] >> (7 - (j & 7))) & 1 : 0)) ^ mpr[static_cast<std::size_t>(j)]);
            int at = 0;
            for (int j = 0; j < nl; ++j) {
                drm::convEncode(&mb[static_cast<std::size_t>(at)], static_cast<std::size_t>(Ms[j]), pats[j], code[j]);
                at += Ms[j];
            }
            std::vector<std::complex<double> >& og = orig[static_cast<std::size_t>(g)];
            og.resize(static_cast<std::size_t>(NMUX));
            for (int j = 0; j < NMUX; ++j) {
                double v[2];
                for (int h = 0; h < 2; ++h) {
                    const int p = 2 * j + h;
                    int ix = 0;
                    for (int l = 0; l < nl; ++l) ix = (ix << 1) | code[l][static_cast<std::size_t>(hasIl[l] ? ils[l][static_cast<std::size_t>(p)] : p)];
                    v[h] = q16 ? A16[ix] : A64[ix];
                }
                og[static_cast<std::size_t>(j)] = std::complex<double>(v[0], v[1]);
            }
        }
    }

    // ---- the superframes: every cell, then each symbol by an inverse DFT ------------------------
    drm::Dft dft;
    dft.init(Tu);
    std::vector<std::complex<double> > X(static_cast<std::size_t>(Tu)), x(static_cast<std::size_t>(Tu)), row(static_cast<std::size_t>(NK)),
        cells(static_cast<std::size_t>(3 * NMUX));
    std::vector<std::complex<double> > out(static_cast<std::size_t>(NSF) * 3 * LS * (Tu + Tg));
    std::size_t o = 0;
    double pkv = 0;
    for (int sf = 0; sf < NSF; ++sf) {
        // (the cell interleaving: the receiver puts cell i of a frame back at
        //  cit[i]; long interleaving also delays it by i mod 5 frames)
        for (int m = 0; m < 3; ++m) {
            const int t = sf * 3 + m;
            for (int i = 0; i < NMUX; ++i) {
                const int g = opt.longInterleave ? ((t - i % 5) % F + F) % F : t;
                cells[static_cast<std::size_t>(m * NMUX + i)] = orig[static_cast<std::size_t>(g)][static_cast<std::size_t>(cit[static_cast<std::size_t>(i)])];
            }
        }
        for (int fr = 0; fr < 3; ++fr)
            for (int s = 0; s < LS; ++s) {
                for (int k = kmin; k <= kmax; ++k) {
                    const int ix = mscIndex[static_cast<std::size_t>((fr * LS + s) * NK + k - kmin)];
                    row[static_cast<std::size_t>(k - kmin)] = ix >= 0 && ix < 3 * NMUX ? cells[static_cast<std::size_t>(ix)] : dummy();  // (the last few: dummy cells)
                }
                const std::vector<drm::IntPair>& gp = gainPh[static_cast<std::size_t>(s)];
                for (std::size_t q = 0; q < gp.size(); ++q) {
                    const int k = gp[q].first;
                    bool b = false;
                    for (int j = 0; j < 4; ++j)
                        if (boost[j] == k) b = true;
                    const double a = b ? 2 : kSqrt2, wph = 2 * kPi * gp[q].second / 1024;
                    row[static_cast<std::size_t>(k - kmin)] = std::polar(a, wph);
                }
                if (s == 0)
                    for (int j = 0; j < tb.nTime; ++j) {
                        const int k = tb.time[2 * j];
                        if (k >= kmin && k <= kmax) row[static_cast<std::size_t>(k - kmin)] = std::polar(kSqrt2, 2 * kPi * tb.time[2 * j + 1] / 1024);
                    }
                for (int j = 0; j < tb.nFreq; ++j) {
                    const int k = tb.freq[2 * j];
                    const int ph = (tb.freq[2 * j + 1] + (mi == 3 && s % 2 == 1 && j < 2 ? 512 : 0)) % 1024;
                    if (k >= kmin && k <= kmax) row[static_cast<std::size_t>(k - kmin)] = std::polar(kSqrt2, 2 * kPi * ph / 1024);
                }
                for (int k = -1; k <= 1; ++k)
                    if (k >= kmin && k <= kmax && (k == 0 || mi == 0)) row[static_cast<std::size_t>(k - kmin)] = 0;
                for (int j = 0; j < tb.nFac; ++j)
                    if (tb.fac[2 * j] == s) row[static_cast<std::size_t>(tb.fac[2 * j + 1] - kmin)] = facCells[fr][static_cast<std::size_t>(j)];
                if (fr == 0)
                    for (int j = 0; j < N; ++j)
                        if (sdcPos[static_cast<std::size_t>(j)].first == s) row[static_cast<std::size_t>(sdcPos[static_cast<std::size_t>(j)].second - kmin)] = sdcCells[static_cast<std::size_t>(j)];
                std::fill(X.begin(), X.end(), std::complex<double>());
                for (int k = kmin; k <= kmax; ++k) X[static_cast<std::size_t>(((k % Tu) + Tu) % Tu)] = row[static_cast<std::size_t>(k - kmin)];
                dft.inverse(&X[0], &x[0]);
                for (int j = 0; j < Tu + Tg; ++j) {
                    const std::complex<double> v = x[static_cast<std::size_t>((j - Tg + Tu) % Tu)];
                    out[o++] = v;
                    pkv = std::max(pkv, std::max(std::fabs(v.real()), std::fabs(v.imag())));
                }
            }
    }
    iq.resize(o);
    for (std::size_t j = 0; j < o; ++j) iq[j] = std::complex<float>(out[j] / (pkv > 0 ? pkv : 1));
    return true;
}

}  // namespace dec
