// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Dab.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The DAB / DAB+ receiver (Dab.h): the stream receiver (lock, frame
//  tracking, the forgiving re-lock, DQPSK demodulation, SNR), the FIC and
//  its FIGs, the station list, the chosen sub-channel (time interleaving,
//  depuncturing, Viterbi, then MP2 frames or DAB+ super frames and audio
//  units), PAD and SPI hand-off, and the worker's part: selecting a
//  station (kept until the list has it), the sound through the codecs,
//  slides and logos, the text lines and the status.
//
//  How it fits in
//  --------------
//  A port of OFFgrid-SDR's JS (diddy-boy, https://github.com/solariun/
//  offgrid-sdr, index.html, the "dablib" block: class DabRx (push, guard,
//  guardScan, nullRatio, findFrame, step, fftAt, demod, fic, figs, forget,
//  stations, spiChannel, dataChannel, select, audio) and its worker
//  (self.onmessage, dabTrySelect, dabLogoStation); the main script's
//  "DAB / DAB+ (from 2.1)": dabFeed's retune, dabAutoPlay, dabChannelNear).
//  The error correction, tables and PAD live in DabFec.cpp and DabPad.cpp
//  (DabInternal.h); the codecs behind DabAudio.cpp.  The FFT is
//  liboffgrid-dsp's (src/dsp/Fft.h).
//
//  Contributing
//  ------------
//  The comments in parentheses are the JS's own (its reasons for each
//  threshold): keep them with the code.  Mark changes "Addition to the
//  JS".  No allocation per sample; strict C++11.  Run
//  tests/test_decoders_dab.cpp, and a live multiplex if you can.
// ============================================================================
#include "Dab.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <limits>
#include <map>

#include "DabInternal.h"
#include "dsp/Fft.h"

namespace dec {

using namespace dab;

namespace {

const double kPi = 3.14159265358979323846;

// programme types (as RDS in Europe): their names by number (JS DAB_PTY)
const char* const kPty[32] = {"",
                              "News",
                              "Current Affairs",
                              "Information",
                              "Sport",
                              "Education",
                              "Drama",
                              "Culture",
                              "Science",
                              "Varied",
                              "Pop Music",
                              "Rock Music",
                              "Easy Listening",
                              "Light Classical",
                              "Serious Classical",
                              "Other Music",
                              "Weather",
                              "Finance",
                              "Children's",
                              "Social Affairs",
                              "Religion",
                              "Phone In",
                              "Travel",
                              "Leisure",
                              "Jazz Music",
                              "Country Music",
                              "National Music",
                              "Oldies Music",
                              "Folk Music",
                              "Documentary",
                              "",
                              ""};

struct Component {
    int tmid, ascty, subch, primary;
};
struct DataComp {
    int tmid, scid;
};
struct PacketComp {
    int dscty, sub, addr;
};

// a station in the list (JS stations())
struct Station {
    unsigned sid;
    std::string label, format;   // "DAB+", "DAB", "other", "?"
    bool hasSub;
    SubChannel sc;
    int pty;   // -1 unknown
};

// a decoded sub-channel's state (JS select / dataChannel's object)
struct Channel {
    Channel() : sid(0), nout(0), ilk(0), ilCount(0), sfSynced(false), misses(0), mp2HoldOn(false), addr(0), fresh(false) {
        for (int i = 0; i < 16; ++i) ilValid[i] = false;
    }
    unsigned sid;
    std::string label, format;   // "DAB+", "DAB", "data"
    SubChannel sc;
    Protection prot;
    int nout;                    // bits a CIF (24 x kbit/s)
    std::vector<unsigned char> prbs;
    std::vector<float> il[16];   // the time interleaver: the last 16 CIFs' soft bits
    bool ilValid[16];
    int ilk, ilCount;
    std::deque<std::vector<unsigned char> > cifs;   // DAB+: the CIFs of a super frame
    bool sfSynced;
    int misses;
    bool mp2HoldOn;
    std::vector<unsigned char> mp2Hold;
    Pad pad;
    // SPI
    int addr;
    std::unique_ptr<Spi> spi;
    bool fresh;
    void clear() {   // (JS: ch.il.fill(null); ch.ilCount = 0; ch.cifs = []; ch.sfSynced = false; ch.misses = 0)
        for (int i = 0; i < 16; ++i) ilValid[i] = false;
        ilCount = 0;
        cifs.clear();
        sfSynced = false;
        misses = 0;
    }
};

std::string hex4(unsigned v) {
    char b[16];
    std::snprintf(b, sizeof b, "%04X", v);
    return b;
}

// the channel plan (JS SW_BANDS' "dab" rows)
const DabChannel kChannels[] = {
    {"5A", 174.928e6},  {"5B", 176.640e6},  {"5C", 178.352e6},  {"5D", 180.064e6},  {"6A", 181.936e6},
    {"6B", 183.648e6},  {"6C", 185.360e6},  {"6D", 187.072e6},  {"7A", 188.928e6},  {"7B", 190.640e6},
    {"7C", 192.352e6},  {"7D", 194.064e6},  {"8A", 195.936e6},  {"8B", 197.648e6},  {"8C", 199.360e6},
    {"8D", 201.072e6},  {"9A", 202.928e6},  {"9B", 204.640e6},  {"9C", 206.352e6},  {"9D", 208.064e6},
    {"10A", 209.936e6}, {"10N", 210.096e6}, {"10B", 211.648e6}, {"10C", 213.360e6}, {"10D", 215.072e6},
    {"11A", 216.928e6}, {"11N", 217.088e6}, {"11B", 218.640e6}, {"11C", 220.352e6}, {"11D", 222.064e6},
    {"12A", 223.936e6}, {"12N", 224.096e6}, {"12B", 225.648e6}, {"12C", 227.360e6}, {"12D", 229.072e6},
    {"13A", 230.784e6}, {"13B", 232.496e6}, {"13C", 234.208e6}, {"13D", 235.776e6}, {"13E", 237.488e6},
    {"13F", 239.200e6}};

}  // namespace

const std::vector<DabChannel>& dabChannels() {
    static const std::vector<DabChannel> v(kChannels, kChannels + sizeof kChannels / sizeof kChannels[0]);
    return v;
}

bool dabChannelNear(double hz, DabChannel& out) {
    if (hz < 174e6 || hz > 240.1e6) return false;
    double best = 1e18;
    for (std::size_t i = 0; i < sizeof kChannels / sizeof kChannels[0]; ++i) {
        const double d = std::fabs(hz - kChannels[i].hz);
        if (d <= 768e3 && d < best) {
            best = d;
            out = kChannels[i];
        }
    }
    return best < 1e18;
}

bool dabChannelByName(const std::string& name, DabChannel& out) {
    for (std::size_t i = 0; i < sizeof kChannels / sizeof kChannels[0]; ++i) {
        const char* c = kChannels[i].name;
        std::size_t k = 0;
        while (c[k] && k < name.size() && std::toupper(static_cast<unsigned char>(name[k])) == c[k]) ++k;
        if (!c[k] && k == name.size()) {
            out = kChannels[i];
            return true;
        }
    }
    return false;
}

// ============================================================================
//  The receiver
// ============================================================================
struct DabDecoder::Impl {
    Impl() : fft(Tu), codecs(newAudioCodecs()), keepUnits(false), userSid(0), userChose(false) {
        fftBuf.resize(Tu);
        syms.resize(static_cast<std::size_t>(L) * K);
        soft.resize(static_cast<std::size_t>(L - 1) * kSymBits);
        dif.resize(static_cast<std::size_t>(L - 1) * K);
        bitsBuf.resize(16384);
        byteBuf.resize(2048);
        sfBuf.resize(120 * 48);
        sfTmp.resize(120 * 48);
        clearAll();
        lastDial = std::numeric_limits<double>::quiet_NaN();
    }

    // ---- state (JS DabRx's fields; reset() restores them)
    dsp::detail::Fft fft;
    std::vector<Cf> fftBuf;
    std::vector<Cf> buf;   // samples; local index i is buf[off + i]
    std::size_t off;
    std::size_t skipIn;    // samples to skip as they arrive
    long long base;        // the absolute index of local 0
    bool locked;
    double f;              // the frequency correction, Hz
    int next;              // where the next frame's phase reference is expected (local)
    double lockQ, lockMetric;
    bool lockFic;
    int lockN, poorRun, coasted;
    unsigned lostCount, watchdog, gaps, sfResyncs;
    bool hasGRef;
    double gRef;
    std::vector<int> badSyms;
    std::vector<double> snrHist;
    double snrDb;
    bool hasSnr;
    int cifNow, cifRead, cifTrust;
    bool ficForce;
    int bridgeCheck;
    unsigned frames, fibs, fibsOk;
    bool everFic, ficLight, ficSkipped;
    int noFicRun, figBlock;
    double ficRate;   // (Addition to the JS: the FIB CRC rate, smoothed, for status())
    std::string state;

    std::vector<Cf> syms;         // carriers of each symbol needed (L x K)
    bool symOk[L];
    std::vector<float> soft;      // soft bits of symbols 1..75 (75 x 3072)
    std::vector<Cf> dif;          // demod's differences
    bool difOk[L];
    double err[K], amp[K];
    float wgt[K];

    // the multiplex (JS ens, labels, subch, services, ptys, dataSvc, packetComps, userApps, eid)
    bool hasEns;
    unsigned ensId;
    std::string ensLabel;
    bool hasEid;
    unsigned eid;
    std::map<unsigned, std::string> labels;
    std::map<int, SubChannel> subch;
    std::map<unsigned, std::vector<Component> > services;
    std::map<unsigned, int> ptys;
    std::map<unsigned, std::vector<DataComp> > dataSvc;
    std::map<int, PacketComp> packetComps;
    std::map<unsigned, std::vector<int> > userApps;

    std::unique_ptr<Channel> sel, spi;
    bool hasPending, hasReselect;
    unsigned pending, reselect;

    // error correction workspace
    Viterbi viterbi;
    std::vector<float> vb, temp;
    std::vector<unsigned char> bitsBuf, byteBuf, sfBuf, sfTmp;
    std::vector<double> scanCr, scanCi, scanEe;

    // ---- the worker's part
    std::unique_ptr<AudioCodecs> codecs;
    AacFormat aacFmt;
    bool hasAacFmt;
    std::string audioDesc;
    std::vector<float> audio;
    int audioRate;
    std::vector<float> pcmTmp;
    std::string text;
    std::vector<Decoder::EncodedImage> images;
    bool keepUnits;
    std::vector<DabUnit> units;
    DabStats st;
    bool ensAnnounced, listAnnounced;
    int listFrames;           // frames since the list first had an audio station (auto choice after 3 s)
    unsigned userSid;         // the station the program chose (kept over a small retune)
    bool userChose;
    std::map<unsigned, bool> logosSent;

    // front end
    double lastDial;
    bool rateOk;
    double mixPhase;

    void clearAll() {
        buf.clear();
        off = 0;
        skipIn = 0;
        base = 0;
        locked = false;
        f = 0;
        next = 0;
        lockQ = lockMetric = 0;
        lockFic = false;
        lockN = poorRun = coasted = 0;
        lostCount = watchdog = gaps = sfResyncs = 0;
        hasGRef = false;
        gRef = 0;
        badSyms.clear();
        snrHist.clear();
        snrDb = 0;
        hasSnr = false;
        cifNow = cifRead = -1;
        cifTrust = 0;
        ficForce = false;
        bridgeCheck = 0;
        frames = fibs = fibsOk = 0;
        everFic = ficLight = ficSkipped = false;
        noFicRun = 0;
        figBlock = 0;
        ficRate = 0;
        state = "searching";
        hasEns = false;
        ensId = 0;
        ensLabel.clear();
        hasEid = false;
        eid = 0;
        labels.clear();
        subch.clear();
        services.clear();
        ptys.clear();
        dataSvc.clear();
        packetComps.clear();
        userApps.clear();
        sel.reset();
        spi.reset();
        hasPending = hasReselect = false;
        pending = reselect = 0;
        hasAacFmt = false;
        audioDesc.clear();
        audio.clear();
        audioRate = 48000;
        text.clear();
        images.clear();
        units.clear();
        st = DabStats();
        ensAnnounced = listAnnounced = false;
        listFrames = -1;
        logosSent.clear();
        rateOk = true;
        mixPhase = 0;
        if (codecs) codecs->reset();
    }

    // ---- samples
    int avail() const { return static_cast<int>(buf.size() - off); }
    const Cf* data() const { return &buf[off]; }
    void drop(int n) {
        off += static_cast<std::size_t>(n);
        base += n;
        // (past what has arrived: the samples still to come are skipped as they arrive, as the JS's slice)
        if (off > buf.size()) {
            skipIn += off - buf.size();
            off = buf.size();
        }
    }
    double pw(const Cf* x, int i) const { return double(x[i].real()) * x[i].real() + double(x[i].imag()) * x[i].imag(); }

    // the guard interval against the symbol's end, at local index i (JS guard)
    void guard(int i, double& cr, double& ci, double& e) const {
        const Cf* x = data();
        cr = ci = e = 0;
        for (int j = i; j < i + Tg; ++j) {
            const double ar = x[j].real(), ai = x[j].imag(), br = x[j + Tu].real(), bi = x[j + Tu].imag();
            cr += ar * br + ai * bi;
            ci += ai * br - ar * bi;
            e += std::sqrt((ar * ar + ai * ai) * (br * br + bi * bi));
        }
    }

    // the guard-interval match at every second position from lo to hi, in one pass: running sums over the range
    // instead of 504 samples summed afresh for each (JS guardScan): the best position and its metric
    void guardScan(int lo, int hi, int& at, double& best) {
        lo = std::max(0, lo);
        const int n = avail();
        const int end = std::min(hi + Tg, n - Tu), Lw = end - lo;
        at = lo;
        best = 0;
        if (Lw <= Tg) return;
        scanCr.resize(static_cast<std::size_t>(Lw) + 1);
        scanCi.resize(static_cast<std::size_t>(Lw) + 1);
        scanEe.resize(static_cast<std::size_t>(Lw) + 1);
        const Cf* x = data();
        scanCr[0] = scanCi[0] = scanEe[0] = 0;
        for (int j = 0; j < Lw; ++j) {
            const int a = lo + j, b = a + Tu;
            const double ar = x[a].real(), ai = x[a].imag(), br = x[b].real(), bi = x[b].imag();
            scanCr[j + 1] = scanCr[j] + ar * br + ai * bi;
            scanCi[j + 1] = scanCi[j] + ai * br - ar * bi;
            scanEe[j + 1] = scanEe[j] + std::sqrt((ar * ar + ai * ai) * (br * br + bi * bi));
        }
        best = -1;
        for (int i = lo; i <= hi && i - lo + Tg <= Lw; i += 2) {
            const int k = i - lo;
            const double xr = scanCr[k + Tg] - scanCr[k], yi = scanCi[k + Tg] - scanCi[k], e = scanEe[k + Tg] - scanEe[k];
            const double g = std::sqrt(xr * xr + yi * yi) / (e != 0 ? e : 1);
            // (Addition to the JS: of equal matches the latest is taken. A noiseless null symbol matches its guard as
            //  well as the phase reference does (zeros add nothing to either sum), so a clean signal gave a plateau up to
            //  48 samples early, and the window slid into the symbol before: seen with the generated multiplex)
            if (g > best + 1e-9) {
                best = g;
                at = i;
            } else if (g >= best - 1e-9)
                at = i;
        }
        if (best < 0) best = 0;
    }

    // (from 2.4) how quiet the 2,048 samples before position i are, against the two symbols from i: a phase reference
    // has the null symbol before it (noise alone, about N / (S + N): under 0.5 down to a 0 dB signal), any other symbol
    // hasn't (1). Taken in 8 parts and the loudest part kept: a null symbol only partly there (a gap took the rest) is
    // caught by the part that isn't quiet (JS nullRatio)
    double nullRatio(int i) const {
        const Cf* x = data();
        if (i < 2048 || i + 2 * Ts > avail()) return 0;
        double b = 0, worst = 0;
        for (int j = i; j < i + 2 * Ts; ++j) b += pw(x, j);
        b /= 2 * Ts;
        for (int k = 0; k < 8; ++k) {
            double a = 0;
            const int j0 = i - 2048 + 64 + k * 240;
            for (int j = j0; j < j0 + 240; ++j) a += pw(x, j);
            worst = std::max(worst, a / 240);
        }
        return worst / (b != 0 ? b : 1e-30);
    }

    // the quietest Tn samples in a frame's length from lo (the null symbol)
    int nullSearch(int lo) const {
        const Cf* x = data();
        double best = std::numeric_limits<double>::infinity(), s = 0;
        int at = lo;
        for (int i = lo; i < lo + Tn; ++i) s += pw(x, i);
        for (int i = lo; i < lo + FR; ++i) {
            if (s < best) {
                best = s;
                at = i;
            }
            s += pw(x, i + Tn) - pw(x, i);
        }
        return at;
    }

    // (from 2.4) the next frame from position lo: its null symbol, then the phase reference's guard peak (JS findFrame)
    void findFrame(int lo, int& p, double& g) {
        const int at = nullSearch(lo);
        int q;
        double gq;
        guardScan(at + Tn - 600, at + Tn + 600, q, gq);
        // (a null symbol cut short by the start of what's kept is less quiet than the next one, a frame on: so the frame
        //  a frame earlier is tried too, and taken if it's there. Otherwise a frame merely retimed by an echo read as one lost)
        if (q - FR >= lo + 64) {
            int q0;
            double g0;
            guardScan(q - FR - 600, q - FR + 600, q0, g0);
            if (g0 >= 0.3 && g0 >= 0.8 * gq && nullRatio(q0) < 0.5) {
                p = q0;
                g = g0;
                return;
            }
        }
        p = q;
        g = gq;
    }

    // the symbol whose guard starts at local index i, frequency-corrected, FFT window 32 samples into the guard (JS
    // fftAt); into fftBuf (all bins)
    void fftAt(int i, double fc) {
        const int s = i + Tg - 32;
        const Cf* x = data() + s;
        const double w = -2 * kPi * fc / FS;
        // (a rotating phasor: cos and sin once a symbol, from the absolute sample position, then one complex step a sample)
        const double ph0 = std::fmod(w * static_cast<double>(base + s), 2 * kPi);
        const double sc = std::cos(w), ss = std::sin(w);
        double c = std::cos(ph0), sn = std::sin(ph0);
        for (int j = 0; j < Tu; ++j) {
            const double xr = x[j].real(), yi = x[j].imag();
            fftBuf[j] = Cf(static_cast<float>(xr * c - yi * sn), static_cast<float>(xr * sn + yi * c));
            const double t = c * sc - sn * ss;
            sn = c * ss + sn * sc;
            c = t;
        }
        fft.forward(&fftBuf[0]);
    }
    // the same, carriers only, each turned back by its 32-sample delay, into syms row l
    void symbolAt(int i, double fc, int l) {
        fftAt(i, fc);
        const Tables& T = tables();
        Cf* o = &syms[static_cast<std::size_t>(l) * K];
        for (int q = 0; q < K; ++q) {
            const Cf a = fftBuf[T.bin[q]], r = T.win[q];
            o[q] = Cf(a.real() * r.real() - a.imag() * r.imag(), a.real() * r.imag() + a.imag() * r.real());
        }
        symOk[l] = true;
    }

    // ---- one step of the stream receiver (JS step): false when more samples are needed
    bool step() {
        const int n = avail();
        const Tables& T = tables();
        if (!locked) {
            if (n < 2 * FR + Tn) return false;
            // the null symbol: the quietest 2,656 samples in a frame's length
            int at = nullSearch(0);
            int prs = at + Tn;
            double gbest;
            guardScan(prs - 600, prs + 600, at, gbest);   // refined to the phase reference's guard peak
            if (gbest < 0.3) {
                drop(FR);
                state = "no DAB signal";
                lockMetric = gbest;
                return true;
            }
            prs = at;
            double cr = 0, ci = 0;
            for (int l = 0; l < L; ++l) {
                double a, b, e;
                guard(prs + l * Ts, a, b, e);
                cr += a;
                ci += b;
            }
            const double ff = -std::atan2(ci, cr) / (2 * kPi) * FS / Tu;   // (the signal's fractional offset)
            // whole carriers: neighbouring carriers' phase steps against the phase reference's (timing can't spoil them)
            fftAt(prs, ff);
            double bestM = -1;
            int sh = 0;
            for (int d = -40; d <= 40; ++d) {
                double vr = 0, vi = 0, a = 0;
                for (int i = 0; i + 1 < K; ++i) {
                    const int b1 = (((T.carriers[i] + d) % Tu) + Tu) % Tu, b2 = (((T.carriers[i + 1] + d) % Tu) + Tu) % Tu;
                    const double x1r = fftBuf[b1].real(), x1i = fftBuf[b1].imag(), x2r = fftBuf[b2].real(), x2i = fftBuf[b2].imag();
                    const double dr = x2r * x1r + x2i * x1i, di = x2i * x1r - x2r * x1i;
                    const double p1r = T.prs[i].real(), p1i = T.prs[i].imag(), p2r = T.prs[i + 1].real(), p2i = T.prs[i + 1].imag();
                    const double rr = p2r * p1r + p2i * p1i, ri = p2i * p1r - p2r * p1i;
                    vr += dr * rr + di * ri;
                    vi += di * rr - dr * ri;
                    a += std::sqrt(dr * dr + di * di);
                }
                const double m = std::sqrt(vr * vr + vi * vi) / (a != 0 ? a : 1);
                if (m > bestM) {
                    bestM = m;
                    sh = d;
                }
            }
            if (bestM < 0.3) {
                drop(FR);
                state = "no DAB signal";
                lockMetric = bestM;
                return true;
            }
            f = ff + sh * FS / Tu;
            locked = true;
            next = prs;
            lockQ = bestM;
            lockFic = false;
            lockN = 0;
            state = "locked";
        }
        // a whole frame: track its timing from the phase reference's guard peak near where it's expected
        if (next + L * Ts + 64 > n) return false;
        int at;
        double gbest;
        guardScan(next - 48, next + 48, at, gbest);
        // (from 2.4) the forgiving re-lock. A group of samples lost on the way (a busy computer, a USB transfer dropped,
        //  a gap in SDRconnect's stream) moves every frame after it earlier by that many samples: the phase reference is
        //  no longer where it's expected, and what comes before the place it's looked for isn't the quiet null symbol.
        //  Rather than coast for 3 frames, give up and search the whole multiplex afresh, the next frame is found at once
        //  from its null symbol, the frequency kept. The frames that went missing are counted, from the FIC's CIF count,
        //  and go into the time interleavers as "unknown", so the error correction mends what it can and the DAB+ super
        //  frames keep their alignment: a blip at most
        bool gap = false;
        int gapFrames = -1;
        // (the null symbol's test at 0.75: an echo arriving before the main signal puts some of its power there, about
        //  half when the two are equally strong; a gap puts in a whole symbol's, about 1)
        // (only a lock that has read the FIC is carried over a gap: one that hasn't may be on the wrong multiplex)
        if (lockFic && (gbest < 0.25 || nullRatio(at) > 0.75)) {
            const int lo = next - 2048;
            if (n < lo + FR + Tn + 600 + L * Ts + 64) return false;   // (a frame more is needed, to look in)
            int p;
            double g;
            findFrame(lo, p, g);
            // (a move of under 600 samples, within reach of an echo, only retimes the frame: nothing counted as lost.
            //  Beyond that it's a gap; how many frames went is then taken only from the CIF count (below))
            if (g >= 0.3 && std::abs(p - at) > 64 && nullRatio(p) < 0.5) {
                if (std::abs(p - next) >= 600) {
                    gap = true;
                    ficForce = true;
                    bridgeCheck = 8;
                }
                at = p;
                gbest = g;
                poorRun = 0;
            }
        }
        // (a poor frame: up to 3 in a row, the receiver coasts on the expected timing rather than searching from
        //  scratch, so a brief dip in the signal costs at most those frames, not the lock)
        if (gbest < 0.25) {
            if (++poorRun > 3) {
                locked = false;
                poorRun = 0;
                ++lostCount;
                // (the lock really lost: the chosen station's time interleaver is cleared of stale data and DAB+'s
                //  super frame alignment searched for afresh at once, rather than assumed)
                if (sel) sel->clear();
                if (spi) spi->clear();
                state = "lost";
                return true;
            }
            at = next;
            ++coasted;
        } else
            poorRun = 0;
        // only the symbols needed: the phase reference and the FIC (0-3), and the chosen station's sub-channel in each
        // CIF, each with the symbol before it (the differential)
        bool need[L];
        for (int l = 0; l < L; ++l) need[l] = l <= 3;
        Channel* chs[2] = {sel.get(), spi.get()};
        for (int k = 0; k < 2; ++k) {
            const Channel* ch = chs[k];
            if (!ch) continue;
            for (int c = 0; c < 4; ++c) {
                const int b0 = c * kCifBits + ch->sc.start * 64, b1 = b0 + ch->sc.size * 64 - 1;
                for (int l = 4 + b0 / kSymBits; l <= 4 + b1 / kSymBits && l < L; ++l) need[l] = need[l - 1] = true;
            }
        }
        for (int l = 0; l < L; ++l) {
            symOk[l] = false;
            if (need[l]) symbolAt(at + l * Ts, f, l);
        }
        // (from 2.4) where a gap in the samples falls inside this frame, every symbol after it is out of place and its
        //  guard no longer matches: those are decoded as "unknown", not as confidently wrong data. Only a run of poor
        //  symbols that lasts to the end of the frame counts, at least 3 of them: on a single-frequency network the
        //  echoes make single symbols' guard matches dip at random
        if (gbest >= 0.25) {
            gRef = hasGRef ? gRef + 0.1 * (gbest - gRef) : gbest;
            hasGRef = true;
        }
        //  (and under 0.12 as well: a symbol a gap put out of place matches its guard hardly at all, about 0.05)
        const double gMin = std::min(0.12, 0.4 * std::max(gbest, hasGRef ? gRef : 0.0));
        {
            int chkL[L];
            double chkG[L];
            int nchk = 0;
            for (int l = 1; l < L; ++l)
                if (symOk[l]) {
                    double a, b, e;
                    guard(at + l * Ts, a, b, e);
                    chkL[nchk] = l;
                    chkG[nchk++] = std::sqrt(a * a + b * b) / (e != 0 ? e : 1);
                }
            int run = nchk;
            while (run > 0 && chkG[run - 1] < gMin) --run;
            //  (and only after at least 3 good ones: a frame poor all through is an echo's doing or a weak moment)
            badSyms.clear();
            if (nchk - run >= 3 && run >= 3)
                for (int k = run; k < nchk; ++k) badSyms.push_back(chkL[k]);
        }
        // the frequency: followed slowly, from this frame's guard correlation (after the correction)
        {
            double cr = 0, ci = 0;
            for (int l = 1; l < L; l += 3) {
                double a, b, e;
                guard(at + l * Ts, a, b, e);
                cr += a;
                ci += b;
            }
            const double w = 2 * kPi * f * Tu / FS, c = std::cos(w), sn = std::sin(w), x = cr * c - ci * sn;
            ci = cr * sn + ci * c;
            cr = x;
            // (only good frames adjust it, by 20 Hz a frame at most: a poor frame's estimate is mostly noise)
            if (gbest >= 0.5) f += std::max(-20.0, std::min(20.0, 0.2 * (-std::atan2(ci, cr) / (2 * kPi) * FS / Tu)));
        }
        snr(at);
        const unsigned fib0 = fibsOk;
        // (from 2.4) the CIF count: 4 more each frame (mode I), read from FIG 0/0 whenever the FIC is decoded. Trusted
        //  once it has twice matched what was expected
        if (cifNow >= 0) cifNow = (cifNow + 4) % 5000;
        cifRead = -1;
        demod();
        fic();
        ++frames;
        lockMetric = gbest;
        int pre = 0;
        if (cifRead >= 0) {
            const int m = cifNow >= 0 ? (cifRead - cifNow + 5000) % 5000 : -1;
            if (gap && cifTrust >= 2 && m >= 0 && m % 4 == 0 && m <= 400) gapFrames = m / 4;
            cifTrust = m == 0 ? cifTrust + 1 : gap ? cifTrust : 0;
            cifNow = cifRead;
        }
        if (gap) {
            ++gaps;
            const int missed = gapFrames < 0 ? -1 : 4 * gapFrames;
            // (more than 3 frames gone, or how many unknown: the interleavers (16 CIFs) are cleared)
            if (missed < 0 || missed > 12) {
                if (sel) sel->clear();
                if (spi) spi->clear();
            } else
                pre = missed;
        }
        if (sel) channelAudio(*sel, pre);
        // the multiplex's SPI service (station logos), decoded alongside the station once it's been described
        // (found partway through a frame, after its symbols were chosen: decoded from the next frame)
        if (!spi) {
            int sub, addr;
            if (spiChannel(sub, addr)) {
                spi = dataChannel(sub);
                if (spi) {
                    spi->addr = addr;
                    spi->spi.reset(new Spi(addr));
                    spi->fresh = true;
                }
            }
        } else if (spi->fresh)
            spi->fresh = false;
        else
            channelAudio(*spi, pre);
        // the watchdog: the FIC is strongly protected and reads down to about 3 dB, so locked but without a single FIB
        // for 2 s (21 frames) means the lock itself is wrong (a carrier or a symbol out): found again from scratch
        if (!ficLight && frames > 60) {
            std::vector<Station> stl = stations();
            bool all = !stl.empty();
            for (std::size_t i = 0; i < stl.size(); ++i)
                if (stl[i].format == "?" || !stl[i].hasSub) all = false;
            ficLight = all && (spi || frames > 300);
        }
        if (ficSkipped) {
        } else if (fibsOk > fib0)
            noFicRun = 0;
        else if (++noFicRun > (ficLight ? 5 : 20) && everFic) {
            noFicRun = 0;
            locked = false;
            ++lostCount;
            ++watchdog;
            if (sel) sel->clear();
        }
        if (fibsOk > fib0) {
            everFic = true;
            lockFic = true;
            bridgeCheck = 0;
        }
        // (from 2.4) after a gap is bridged, the FIC must come through within 8 frames: if it doesn't, what was bridged
        //  to isn't this multiplex as it was locked, and the lock is let go to be found afresh
        else if (bridgeCheck > 0 && --bridgeCheck == 0) {
            locked = false;
            ++lostCount;
            ++watchdog;
            if (sel) sel->clear();
            if (spi) spi->clear();
        }
        // (from 2.4) and a lock that has read no FIC at all in 3 s (30 frames) is let go too
        else if (!lockFic && locked && ++lockN > 30) {
            locked = false;
            lockN = 0;
            ++lostCount;
            ++watchdog;
            if (sel) sel->clear();
        }
        if (!locked) state = "lost";
        next = at + FR;
        const int keep = std::max(0, next - 2048);
        drop(keep);
        next -= keep;
        return true;
    }

    // the signal's quality in dB, as for DRM: the null symbol (the transmitter silent) gives the noise in the carriers'
    // frequencies, the phase reference symbol the signal and the noise; per carrier, S/N = (P_ref - P_null) / P_null
    void snr(int at) {
        const int s0 = at - Tu;
        if (s0 < 0 || !symOk[0]) return;
        fftAt(s0 - (Tg - 32), f);   // (the same derotation, from the absolute position, as the JS)
        const Tables& T = tables();
        double pn = 0, ps = 0;
        const Cf* x = &syms[0];
        for (int q = 0; q < K; ++q) {
            const Cf a = fftBuf[T.bin[q]];
            pn += double(a.real()) * a.real() + double(a.imag()) * a.imag();
            ps += double(x[q].real()) * x[q].real() + double(x[q].imag()) * x[q].imag();
        }
        // (a frame whose "silent" window holds as much power as the signal is a timing glitch, not noise: skipped, and
        //  the figure is the median of the last 9 good frames, so a glitch can't drag it down)
        const double r = ps / std::max(pn, 1e-30);
        if (r > 1.05) {
            snrHist.push_back(10 * std::log10(r - 1));
            if (snrHist.size() > 9) snrHist.erase(snrHist.begin());
            std::vector<double> h(snrHist);
            std::sort(h.begin(), h.end());
            snrDb = h[h.size() >> 1];
            hasSnr = true;
        }
    }

    // differential, common phase removed per symbol, each carrier weighted by its reliability over the frame. (The
    // error per carrier from the fourth power: for a small error e, 1 - Re(-u^4) = 1 - cos 4e, about 8 e^2) (JS demod)
    void demod() {
        const Tables& T = tables();
        for (int q = 0; q < K; ++q) err[q] = amp[q] = 0;
        int nPairs = 0;
        for (int l = 1; l < L; ++l) {
            difOk[l] = symOk[l - 1] && symOk[l];
            if (!difOk[l]) continue;
            ++nPairs;
            const Cf* a = &syms[static_cast<std::size_t>(l - 1) * K];
            const Cf* b = &syms[static_cast<std::size_t>(l) * K];
            Cf* d = &dif[static_cast<std::size_t>(l - 1) * K];
            double sr = 0, si = 0;
            for (int q = 0; q < K; ++q) {
                const double dr = double(b[q].real()) * a[q].real() + double(b[q].imag()) * a[q].imag();
                const double di = double(b[q].imag()) * a[q].real() - double(b[q].real()) * a[q].imag();
                d[q] = Cf(static_cast<float>(dr), static_cast<float>(di));
                const double m2 = dr * dr + di * di + 1e-24, m = std::sqrt(m2), r2 = (dr * dr - di * di) / m2, i2 = 2 * dr * di / m2;
                const double r4 = r2 * r2 - i2 * i2, i4 = 2 * r2 * i2;
                sr -= r4 * m;
                si -= i4 * m;
                amp[q] += m;
            }
            const double cpe = std::atan2(si, sr) / 4, c = std::cos(-cpe), s = std::sin(-cpe);
            for (int q = 0; q < K; ++q) {
                const double xr = d[q].real(), xi = d[q].imag();
                const double dr = xr * c - xi * s, di = xr * s + xi * c;
                d[q] = Cf(static_cast<float>(dr), static_cast<float>(di));
                // (1 - Re(-u^4), turned by the common phase; u^4 recomputed from the turned value: the same thing)
                const double m2 = dr * dr + di * di + 1e-24, r2 = (dr * dr - di * di) / m2, i2 = 2 * dr * di / m2;
                err[q] += 1 + (r2 * r2 - i2 * i2);
            }
        }
        double wm = 0;
        if (nPairs) {
            for (int q = 0; q < K; ++q) {
                wgt[q] = static_cast<float>((amp[q] / nPairs) / (err[q] / 8 / nPairs + 1e-3));
                wm += wgt[q];
            }
            wm /= K;
        }
        if (wm <= 0) wm = 1;
        for (int l = 1; l < L; ++l) {
            float* o = &soft[static_cast<std::size_t>(l - 1) * kSymBits];
            if (!difOk[l]) {
                std::fill(o, o + kSymBits, 0.f);
                continue;
            }
            const Cf* d = &dif[static_cast<std::size_t>(l - 1) * K];
            for (int i = 0; i < K; ++i) {
                const int q = T.perm[i];
                const double m = std::sqrt(double(d[q].real()) * d[q].real() + double(d[q].imag()) * d[q].imag()) + 1e-12;
                const double g = wgt[q] / wm / m;
                o[i] = static_cast<float>(-d[q].real() * g);
                o[K + i] = static_cast<float>(-d[q].imag() * g);
            }
        }
        // (from 2.4: a symbol out of place, see step: both differences it's part of become "unknown")
        for (std::size_t k = 0; k < badSyms.size(); ++k) {
            const int l = badSyms[k];
            // (the pair (l-1, l) is row l - 1, the pair (l, l+1) row l)
            std::fill(&soft[static_cast<std::size_t>(l - 1) * kSymBits], &soft[static_cast<std::size_t>(l) * kSymBits], 0.f);
            if (l < L - 1) std::fill(&soft[static_cast<std::size_t>(l) * kSymBits], &soft[static_cast<std::size_t>(l + 1) * kSymBits], 0.f);
        }
    }

    // ---- the FIC (JS fic)
    void fic() {
        // (once the station list, its formats and the logo service are known, the FIC is decoded in one frame of four:
        //  it rarely changes, and decoding it every frame was a good part of the receiver's work)
        ficSkipped = ficLight && frames % 4 != 0 && !ficForce && !bridgeCheck;
        ficForce = false;
        if (ficSkipped) return;
        const Tables& T = tables();
        static const Section kFicSections[2] = {{16, 21}, {15, 3}};
        int ok = 0;
        for (int b = 0; b < 4; ++b) {
            depuncture(&soft[static_cast<std::size_t>(b) * 2304], kFicSections, 2, 768, vb);
            viterbi.decode(&vb[0], 768, &bitsBuf[0]);
            for (int fb = 0; fb < 3; ++fb) {
                unsigned char by[32];
                std::memset(by, 0, sizeof by);
                for (int i = 0; i < 256; ++i)
                    by[i >> 3] = static_cast<unsigned char>(by[i >> 3] | (((bitsBuf[fb * 256 + i] ^ T.prbs768[fb * 256 + i]) & 1) << (7 - (i & 7))));
                ++fibs;
                if (crcOk(by, 32)) {
                    ++fibsOk;
                    ++ok;
                    figBlock = b;
                    figs(by);
                }
            }
        }
        ficRate += 0.25 * (ok / 12.0 - ficRate);
    }

    void figs(const unsigned char* by) {
        int p = 0;
        while (p < 30 && by[p] != 0xff) {
            const int typ = by[p] >> 5, ln = by[p] & 31;
            const unsigned char* d = by + p + 1;
            const int dl = std::min(ln, 31 - p);   // (Addition to the JS: inside the FIB's 32 bytes)
            p += 1 + ln;
            if (!ln) break;
            if (typ == 1 && dl >= 19) {
                const int ext = d[0] & 7;
                const unsigned id = (static_cast<unsigned>(d[1]) << 8) | d[2];
                const std::string txt = dab::text(d + 3, 16, d[0] >> 4);
                if (ext == 0) {
                    hasEns = true;
                    ensId = id;
                    ensLabel = txt;
                } else if (ext == 1)
                    labels[id] = txt;
            }
            if (typ == 0 && dl > 1) fig0(d, dl);
        }
    }

    void fig0(const unsigned char* d, int len) {
        const int ext = d[0] & 31, pd = (d[0] >> 5) & 1, cn = d[0] >> 7;
        int q = 1;
        // (from 2.4) FIG 0/0: the CIF count (0-4999), of the CIF its FIB group goes with: this frame's first is that,
        // less the group. (And its multiplex ID: a different one from what's been heard means everything learned so far
        // came from another multiplex (the last channel's samples, still arriving just after a retune): forgotten)
        if (ext == 0 && len >= 5) {
            const unsigned e = (static_cast<unsigned>(d[1]) << 8) | d[2];
            if (hasEid && e != eid) forget();
            hasEid = true;
            eid = e;
            if ((d[3] & 31) < 20) cifRead = ((d[3] & 31) * 250 + d[4] - figBlock + 5000) % 5000;
        }
        if (ext == 1 && !cn)
            while (q + 2 < len) {
                SubChannel sc;
                sc.id = d[q] >> 2;
                sc.start = ((d[q] & 3) << 8) | d[q + 1];
                if (d[q + 2] & 0x80) {
                    if (q + 3 >= len) break;
                    sc.uep = false;
                    sc.opt = (d[q + 2] >> 4) & 7;
                    sc.lev = (d[q + 2] >> 2) & 3;
                    sc.size = ((d[q + 2] & 3) << 8) | d[q + 3];
                    q += 4;
                } else {
                    sc.uep = true;
                    sc.uepIndex = d[q + 2] & 63;
                    sc.size = kUep[sc.uepIndex].cus;
                    q += 3;
                }
                subch[sc.id] = sc;
            }
        // FIG 0/17: each station's programme type (as welle.io: SId, a flags byte saying whether a language byte or
        // extra codes follow, and the type in the last 5 bits)
        if (ext == 17 && !pd)
            while (q + 3 < len) {
                const unsigned sid = (static_cast<unsigned>(d[q]) << 8) | d[q + 1];
                const int Lf = (d[q + 2] >> 5) & 1, CC = (d[q + 2] >> 4) & 1;
                if (q + 3 + Lf >= len) break;
                ptys[sid] = d[q + 3 + Lf] & 31;
                q += 4 + Lf + CC;
            }
        // data services (32-bit IDs), their packet-mode components (FIG 0/3) and applications (FIG 0/13): SPI is
        // application 7, carried as MOT (DSCTy 60) in packet mode: the station logos
        if (ext == 2 && pd)
            while (q + 4 < len) {
                const unsigned sid = (static_cast<unsigned>(d[q]) << 24) | (static_cast<unsigned>(d[q + 1]) << 16) |
                                     (static_cast<unsigned>(d[q + 2]) << 8) | d[q + 3];
                const int nc = d[q + 4] & 15;
                q += 5;
                std::vector<DataComp> comps;
                for (int c = 0; c < nc && q + 1 < len; ++c, q += 2) {
                    DataComp dc = {d[q] >> 6, ((d[q] & 63) << 6) | (d[q + 1] >> 2)};
                    comps.push_back(dc);
                }
                dataSvc[sid] = comps;
            }
        if (ext == 3)
            while (q + 4 < len) {
                const int scid = (d[q] << 4) | (d[q + 1] >> 4), scca = (d[q + 1] >> 1) & 1;
                PacketComp pc = {d[q + 2] & 63, d[q + 3] >> 2, ((d[q + 3] & 3) << 8) | d[q + 4]};
                packetComps[scid] = pc;
                q += 5 + (scca ? 2 : 0);
            }
        if (ext == 13) {
            const int w = pd ? 4 : 2;
            while (q + w < len) {
                const unsigned sid = pd ? ((static_cast<unsigned>(d[q]) << 24) | (static_cast<unsigned>(d[q + 1]) << 16) |
                                           (static_cast<unsigned>(d[q + 2]) << 8) | d[q + 3])
                                        : ((static_cast<unsigned>(d[q]) << 8) | d[q + 1]);
                q += w;
                const int na = d[q] & 15;
                ++q;
                std::vector<int> apps;
                for (int a = 0; a < na && q + 1 < len; ++a) {
                    apps.push_back((d[q] << 3) | (d[q + 1] >> 5));
                    q += 2 + (d[q + 1] & 31);
                }
                userApps[sid] = apps;
            }
        }
        if (ext == 2 && !pd)
            while (q + 2 < len) {
                const unsigned sid = (static_cast<unsigned>(d[q]) << 8) | d[q + 1];
                const int nc = d[q + 2] & 15;
                q += 3;
                std::vector<Component> comps;
                for (int c = 0; c < nc && q + 1 < len; ++c, q += 2) {
                    Component cp = {d[q] >> 6, d[q] & 63, d[q + 1] >> 2, (d[q + 1] >> 1) & 1};
                    comps.push_back(cp);
                }
                services[sid] = comps;
            }
    }

    void forget() {   // (JS forget: another multiplex's data)
        if (sel) {
            hasReselect = true;
            reselect = sel->sid;
        }
        sel.reset();
        spi.reset();
        hasEns = false;
        ensLabel.clear();
        ensId = 0;
        ficLight = false;
        labels.clear();
        subch.clear();
        services.clear();
        ptys.clear();
        dataSvc.clear();
        packetComps.clear();
        userApps.clear();
        cifNow = -1;
        cifTrust = 0;
        ensAnnounced = listAnnounced = false;
        listFrames = -1;
        logosSent.clear();
    }

    // the station list: each labelled audio service, with its sub-channel and format (JS stations())
    std::vector<Station> stations() const {
        std::vector<Station> out;
        for (std::map<unsigned, std::string>::const_iterator it = labels.begin(); it != labels.end(); ++it) {
            Station s;
            s.sid = it->first;
            s.label = it->second;
            s.hasSub = false;
            s.pty = -1;
            s.format = "?";
            std::map<unsigned, std::vector<Component> >::const_iterator sv = services.find(s.sid);
            if (sv != services.end())
                for (std::size_t i = 0; i < sv->second.size(); ++i)
                    if (sv->second[i].tmid == 0) {
                        const Component& c = sv->second[i];
                        s.format = c.ascty == 63 ? "DAB+" : c.ascty == 0 ? "DAB" : "other";
                        std::map<int, SubChannel>::const_iterator sc = subch.find(c.subch);
                        if (sc != subch.end()) {
                            s.hasSub = true;
                            s.sc = sc->second;
                        } else
                            s.sc.id = c.subch;
                        break;
                    }
            std::map<unsigned, int>::const_iterator pt = ptys.find(s.sid);
            if (pt != ptys.end()) s.pty = pt->second;
            out.push_back(s);
        }
        struct ByLabel {
            bool operator()(const Station& a, const Station& b) const {
                return a.label != b.label ? a.label < b.label : a.sid < b.sid;
            }
        };
        std::sort(out.begin(), out.end(), ByLabel());
        return out;
    }

    // the SPI service's sub-channel and packet address, once described (JS spiChannel)
    bool spiChannel(int& sub, int& addr) const {
        for (std::map<unsigned, std::vector<DataComp> >::const_iterator it = dataSvc.begin(); it != dataSvc.end(); ++it) {
            std::map<unsigned, std::vector<int> >::const_iterator ua = userApps.find(it->first);
            if (ua == userApps.end() || std::find(ua->second.begin(), ua->second.end(), 7) == ua->second.end()) continue;
            for (std::size_t i = 0; i < it->second.size(); ++i) {
                if (it->second[i].tmid != 3) continue;
                std::map<int, PacketComp>::const_iterator pc = packetComps.find(it->second[i].scid);
                if (pc != packetComps.end() && subch.count(pc->second.sub)) {
                    sub = pc->second.sub;
                    addr = pc->second.addr;
                    return true;
                }
            }
        }
        return false;
    }

    std::unique_ptr<Channel> makeChannel(const SubChannel& sc) {
        Protection p;
        if (!protectionOf(sc, p)) return std::unique_ptr<Channel>();
        std::unique_ptr<Channel> c(new Channel);
        c->sc = sc;
        c->prot = p;
        c->nout = 24 * p.kbps;
        prbs(c->nout, c->prbs);
        for (int i = 0; i < 16; ++i) c->il[i].assign(static_cast<std::size_t>(sc.size) * 64, 0.f);
        return c;
    }
    // a data sub-channel (packet mode: SPI, for station logos), alongside the station (JS dataChannel)
    std::unique_ptr<Channel> dataChannel(int id) {
        std::map<int, SubChannel>::const_iterator it = subch.find(id);
        if (it == subch.end()) return std::unique_ptr<Channel>();
        std::unique_ptr<Channel> c = makeChannel(it->second);
        if (c) c->format = "data";
        return c;
    }
    // (JS select)
    bool select(unsigned sid) {
        std::vector<Station> stl = stations();
        for (std::size_t i = 0; i < stl.size(); ++i) {
            if (stl[i].sid != sid) continue;
            if (!stl[i].hasSub || (stl[i].format != "DAB" && stl[i].format != "DAB+")) return false;
            std::unique_ptr<Channel> c = makeChannel(stl[i].sc);
            if (!c) return false;
            c->sid = sid;
            c->label = stl[i].label;
            c->format = stl[i].format;
            sel.swap(c);
            if (codecs) codecs->reset();
            hasAacFmt = false;
            audioDesc.clear();
            char line[160];
            std::snprintf(line, sizeof line, "Station %s (%s %d kbit/s, %s)\n", sel->label.c_str(), sel->format.c_str(),
                          sel->prot.kbps, sel->prot.level.c_str());
            text += line;
            return true;
        }
        return false;
    }

    // ---- a sub-channel's CIFs: time de-interleaving -> depuncturing -> Viterbi -> descrambling; then MP2 frames,
    // DAB+ super frames, or (data) packets (JS audio). pre: that many CIFs lost in a gap go in first, as zeros:
    // "unknown" to the Viterbi decoder, so the time interleaver stays in step
    void channelAudio(Channel& ch, int pre) {
        const int len = ch.sc.size * 64;
        temp.resize(static_cast<std::size_t>(len));
        for (int c = -pre; c < 4; ++c) {
            const float* cif = c >= 0 ? &soft[static_cast<std::size_t>(3) * kSymBits + static_cast<std::size_t>(c) * kCifBits +
                                              static_cast<std::size_t>(ch.sc.start) * 64]
                                      : 0;
            if (c >= 0 && static_cast<std::size_t>(3) * kSymBits + static_cast<std::size_t>(c) * kCifBits +
                                  static_cast<std::size_t>(ch.sc.start + ch.sc.size) * 64 > soft.size())
                return;   // (Addition to the JS: a sub-channel past the CIF's end is ignored)
            for (int i = 0; i < len; ++i) {
                const int k = (ch.ilk + kImap[i & 15]) & 15;
                temp[static_cast<std::size_t>(i)] = ch.ilValid[k] ? ch.il[k][static_cast<std::size_t>(i)] : 0.f;
            }
            if (cif)
                std::copy(cif, cif + len, ch.il[ch.ilk].begin());
            else
                std::fill(ch.il[ch.ilk].begin(), ch.il[ch.ilk].end(), 0.f);
            ch.ilValid[ch.ilk] = true;
            ch.ilk = (ch.ilk + 1) & 15;
            if (ch.ilCount < 16) {
                ++ch.ilCount;
                continue;
            }
            const int nout = ch.nout;
            depuncture(&temp[0], ch.prot.sections, ch.prot.nSections, nout, vb);
            if (bitsBuf.size() < static_cast<std::size_t>(nout)) bitsBuf.resize(static_cast<std::size_t>(nout));
            viterbi.decode(&vb[0], nout, &bitsBuf[0]);
            const std::size_t nb = static_cast<std::size_t>(nout >> 3);
            std::vector<unsigned char> by(nb, 0);   // (one CIF's bytes: a few hundred, once each 24 ms)
            for (int i = 0; i < nout; ++i)
                by[static_cast<std::size_t>(i >> 3)] = static_cast<unsigned char>(by[static_cast<std::size_t>(i >> 3)] |
                                                                                  (((bitsBuf[i] ^ ch.prbs[static_cast<std::size_t>(i)]) & 1) << (7 - (i & 7))));
            if (ch.format == "data") {   // (a data sub-channel: its bytes, for packet mode)
                if (ch.spi) ch.spi->cif(&by[0], by.size());
                continue;
            }
            if (ch.format == "DAB") {
                mp2Cif(ch, by);
                continue;
            }
            plusCif(ch, by);
        }
        if (ch.spi && !ch.spi->ready.empty()) logos(*ch.spi);
    }

    // DAB: MPEG Layer II frames (JS audio's "DAB" branch)
    void mp2Cif(Channel& ch, std::vector<unsigned char>& by) {
        std::vector<unsigned char> fr;
        // (MPEG-2's half sample rate, 24 kHz: each frame lasts 48 ms, two CIFs: the first, which starts with its header,
        //  is held and the second joined to it. A second half on its own (after a lost CIF) is skipped, so the pairing
        //  realigns at the next header)
        if (ch.mp2HoldOn) {
            fr.swap(ch.mp2Hold);
            fr.insert(fr.end(), by.begin(), by.end());
            ch.mp2HoldOn = false;
        } else if (by.size() > 2 && by[0] == 0xff && (by[1] & 0xf0) == 0xf0 && !((by[1] >> 3) & 1)) {
            ch.mp2Hold = by;
            ch.mp2HoldOn = true;
            return;
        } else
            fr.swap(by);
        if (fr.size() < 8 || fr[0] != 0xff || (fr[1] & 0xf0) != 0xf0) return;
        // (the frame's header: MPEG-1 at 48 kHz, or MPEG-2's 24 kHz; its bit rate; mono, stereo, joint stereo or dual)
        const int mpeg1 = (fr[1] >> 3) & 1, bi = fr[2] >> 4, si = (fr[2] >> 2) & 3, md = fr[3] >> 6;
        static const int br1[16] = {0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384, 0};
        static const int br2[16] = {0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, 0};
        static const int sr1[4] = {44100, 48000, 32000, 0}, sr2[4] = {22050, 24000, 16000, 0};
        const int br = mpeg1 ? br1[bi] : br2[bi], rate = mpeg1 ? sr1[si] : sr2[si];
        const bool layer2 = ((fr[1] >> 1) & 3) == 2;
        // (a real DAB MP2 frame states its sub-channel's bit rate and one of DAB's two sample rates, 48 or 24 kHz, Layer
        //  II: a damaged frame that merely starts like one is dropped, not played or shown)
        if (!layer2 || br != ch.prot.kbps || (rate != 48000 && rate != 24000)) {
            ++st.mp2Bad;
            return;
        }
        ++st.mp2Frames;
        static const char* const modes[4] = {"stereo", "joint stereo", "dual channel", "mono"};
        char desc[96];
        std::snprintf(desc, sizeof desc, "MPEG Layer II %d kbit/s %d kHz %s", br, rate / 1000, modes[md]);
        audioDesc = desc;
        if (keepUnits) {
            DabUnit u;
            u.aac = false;
            u.sampleRate = rate;
            u.sbr = u.ps = false;
            u.stereo = md != 3;
            u.bytes = fr;
            units.push_back(u);
        }
        decodeMp2(fr);
        const unsigned char* body = &fr[4];
        const int Lb = static_cast<int>(fr.size()) - 4, scf = mpeg1 && br < (md == 3 ? 56 : 112) ? 2 : 4;
        if (Lb > 2 + scf) {
            ch.pad.process(body, Lb - 2 - scf, false, body[Lb - 2], body[Lb - 1]);
            padOut(ch);
        }
    }

    // DAB+: super frames of 5 CIFs. Searching: a sliding window of 5, tested by the fire code (after Reed-Solomon);
    // synchronised: each next 5, corrected by Reed-Solomon, then checked by the fire code (JS audio's DAB+ branch)
    void plusCif(Channel& ch, std::vector<unsigned char>& by) {
        ch.cifs.push_back(std::vector<unsigned char>());
        ch.cifs.back().swap(by);
        if (!ch.sfSynced) {
            if (ch.cifs.size() > 5) ch.cifs.pop_front();
            if (ch.cifs.size() < 5) return;
        } else if (ch.cifs.size() < 5)
            return;
        const int s = ch.prot.kbps / 8;
        if (s < 1 || ch.prot.kbps % 8) return;
        const std::size_t sfLen = static_cast<std::size_t>(120 * s);
        if (sfBuf.size() < sfLen) {
            sfBuf.resize(sfLen);
            sfTmp.resize(sfLen);
        }
        unsigned char* sf = &sfBuf[0];
        {
            std::size_t o = 0;
            for (std::size_t i = 0; i < ch.cifs.size(); ++i)
                for (std::size_t j = 0; j < ch.cifs[i].size() && o < sfLen; ++j) sf[o++] = ch.cifs[i][j];
            if (o < sfLen) return;
        }
        unsigned char col[120];
        // (searching: each position tested after Reed-Solomon, as when aligned: on a weak signal the raw bytes nearly
        //  always hold a few errors; a copy, so a wrong one isn't kept)
        if (!ch.sfSynced && !superframeOk(sf, s)) {
            unsigned char* t = &sfTmp[0];
            std::memcpy(t, sf, sfLen);
            for (int i = 0; i < s; ++i) {
                for (int p = 0; p < 120; ++p) col[p] = t[p * s + i];
                if (rsDecode(col, 120) >= 0)
                    for (int p = 0; p < 120; ++p) t[p * s + i] = col[p];
            }
            if (!superframeOk(t, s)) return;
        }
        int fixed = 0;
        for (int i = 0; i < s; ++i) {
            for (int p = 0; p < 120; ++p) col[p] = sf[p * s + i];
            const int r = rsDecode(col, 120);
            if (r >= 0) {
                fixed += r;
                for (int p = 0; p < 120; ++p) sf[p * s + i] = col[p];
            }
        }
        if (!superframeOk(sf, s)) {
            ++st.superframesBad;
            if (++ch.misses > 3) {
                ch.sfSynced = false;
                ch.cifs.pop_front();
                ++sfResyncs;
            } else
                ch.cifs.clear();
            return;
        }
        ch.sfSynced = true;
        ch.misses = 0;
        ++st.superframesOk;
        st.rsFixed += static_cast<unsigned>(fixed);
        const int dac = (sf[2] >> 6) & 1, sbr = (sf[2] >> 5) & 1, n = dac ? (sbr ? 3 : 6) : (sbr ? 2 : 4);
        AacFormat fmt;
        fmt.rate = dac ? 48000 : 32000;
        fmt.sbr = sbr != 0;
        fmt.stereo = ((sf[2] >> 4) & 1) != 0;
        fmt.ps = ((sf[2] >> 3) & 1) != 0;
        if (!hasAacFmt || fmt != aacFmt) {
            aacFmt = fmt;
            hasAacFmt = true;
            char desc[96];
            std::snprintf(desc, sizeof desc, "%s %d kHz %s", fmt.sbr ? (fmt.ps ? "HE-AAC v2" : "HE-AAC") : "AAC-LC",
                          fmt.rate / 1000, fmt.stereo || fmt.ps ? "stereo" : "mono");
            audioDesc = desc;
        }
        int starts[7] = {dac ? (sbr ? 6 : 11) : (sbr ? 5 : 8), (sf[3] << 4) | (sf[4] >> 4), ((sf[4] & 15) << 8) | sf[5],
                         (sf[6] << 4) | (sf[7] >> 4),          ((sf[7] & 15) << 8) | sf[8], (sf[9] << 4) | (sf[10] >> 4), 0};
        starts[n] = 110 * s;
        for (int j = 0; j < n; ++j) {
            const unsigned char* a = sf + starts[j];
            int alen = starts[j + 1] - starts[j];
            if (alen <= 0) continue;
            if (j == n - 1)
                for (int Lx = alen; Lx > 2; --Lx)
                    if (crc16(a, static_cast<std::size_t>(Lx - 2)) == ((static_cast<unsigned>(a[Lx - 2]) << 8) | a[Lx - 1])) {
                        alen = Lx;
                        break;
                    }
            if (alen > 2 && crcOk(a, static_cast<std::size_t>(alen))) {
                ++st.unitsOk;
                const int aulen = alen - 2;
                if (keepUnits) {
                    DabUnit u;
                    u.aac = true;
                    u.sampleRate = fmt.rate;
                    u.sbr = fmt.sbr;
                    u.ps = fmt.ps;
                    u.stereo = fmt.stereo;
                    u.bytes.assign(a, a + aulen);
                    units.push_back(u);
                }
                decodeAac(a, aulen);
                // PAD: a data stream element (ID 4) at the unit's start
                if (aulen >= 3 && (a[0] >> 5) == 4) {
                    int ps = 2, pl = a[1];
                    if (pl == 255) {
                        pl += a[2];
                        ++ps;
                    }
                    if (pl >= 2 && aulen >= ps + pl) {
                        ch.pad.process(a + ps, pl - 2, true, a[ps + pl - 2], a[ps + pl - 1]);
                        padOut(ch);
                    }
                }
            } else
                ++st.unitsBad;   // (Addition to the JS: counted)
        }
        ch.cifs.clear();
    }

    // the station's text and pictures, from its PAD
    void padOut(Channel& ch) {
        if (ch.pad.changed) {
            ch.pad.changed = false;
            if (!ch.pad.label.empty()) text += ch.label + ": " + ch.pad.label + "\n";
        }
        std::vector<Picture>& r = ch.pad.slides.ready;
        for (std::size_t i = 0; i < r.size(); ++i) {
            Decoder::EncodedImage im;
            im.name = "slide " + hex4(ch.sid) + (r[i].name.empty() ? "" : " " + r[i].name);
            im.mime = r[i].mime;
            im.bytes.swap(r[i].bytes);
            images.push_back(im);
        }
        r.clear();
    }

    // logos completed by the SPI decoder, matched to their stations: the BBC names each "si" + the station's ID (4 hex
    // digits) + a size number (JS dabLogoStation)
    void logos(Spi& s) {
        std::vector<Station> stl = stations();
        for (std::size_t i = 0; i < s.ready.size(); ++i) {
            Picture& p = s.ready[i];
            std::string n = p.name;
            const std::size_t slash = n.rfind('/');
            if (slash != std::string::npos) n = n.substr(slash + 1);
            for (std::size_t k = 0; k < n.size(); ++k) n[k] = static_cast<char>(std::toupper(static_cast<unsigned char>(n[k])));
            for (std::size_t k = 0; k < stl.size(); ++k) {
                const std::string want = "SI" + hex4(stl[k].sid);
                if (n.compare(0, want.size(), want) != 0) continue;
                Decoder::EncodedImage im;
                char dims[32];
                std::snprintf(dims, sizeof dims, " %dx%d", p.w, p.h);
                im.name = "logo " + hex4(stl[k].sid) + " " + stl[k].label + dims;
                im.mime = p.mime;
                im.bytes.swap(p.bytes);
                images.push_back(im);
                break;
            }
        }
        s.ready.clear();
    }

    // ---- the sound
    void addPcm(int rate) {
        if (pcmTmp.empty()) return;
        if (rate > 0) audioRate = rate;
        st.pcmFrames += pcmTmp.size() / 2;
        // (Addition to the JS: at most 10 s waiting, if the program doesn't take it)
        if (audio.size() > static_cast<std::size_t>(audioRate) * 20) audio.erase(audio.begin(), audio.begin() + static_cast<std::ptrdiff_t>((audio.size() / 2) & ~static_cast<std::size_t>(1)));
        audio.insert(audio.end(), pcmTmp.begin(), pcmTmp.end());
    }
    void decodeAac(const unsigned char* au, int n) {
        if (!codecs || !codecs->available()) return;
        pcmTmp.clear();
        int rate = 0;
        if (!codecs->aac(aacFmt, au, static_cast<std::size_t>(n), pcmTmp, rate)) {
            ++st.decodeErrors;
            return;
        }
        addPcm(rate);
    }
    void decodeMp2(const std::vector<unsigned char>& fr) {
        if (!codecs || !codecs->available()) return;
        pcmTmp.clear();
        int rate = 0;
        if (!codecs->mp2(&fr[0], fr.size(), pcmTmp, rate)) {
            ++st.decodeErrors;
            return;
        }
        addPcm(rate);
    }

    // ---- after each frame: the worker's part (selecting, the list, auto choice)
    void afterFrame(bool frame) {
        if (hasEns && !ensAnnounced) {
            ensAnnounced = true;
            text += "DAB ensemble " + ensLabel + " (" + hex4(ensId) + ")\n";
        }
        if (hasReselect) {   // (after forget: chosen again)
            if (!hasPending) {
                hasPending = true;
                pending = reselect;
            }
            hasReselect = false;
        }
        if (hasPending && select(pending)) hasPending = false;
        std::vector<Station> stl = stations();
        bool anyAudio = false;
        for (std::size_t i = 0; i < stl.size(); ++i)
            if ((stl[i].format == "DAB" || stl[i].format == "DAB+") && stl[i].hasSub) anyAudio = true;
        if (anyAudio && listFrames < 0) listFrames = 0;
        if (listFrames >= 0 && frame) ++listFrames;
        if (ficLight && !listAnnounced && hasEns) {
            listAnnounced = true;
            for (std::size_t i = 0; i < stl.size(); ++i) {
                Protection p;
                const bool okp = stl[i].hasSub && protectionOf(stl[i].sc, p);
                char line[200];
                std::snprintf(line, sizeof line, "  %s  %-16s  %s", hex4(stl[i].sid).c_str(), stl[i].label.c_str(),
                              stl[i].format.c_str());
                text += line;
                if (okp) {
                    std::snprintf(line, sizeof line, " %d kbit/s", p.kbps);
                    text += line;
                }
                if (stl[i].pty > 0 && stl[i].pty < 32 && kPty[stl[i].pty][0]) text += std::string(", ") + kPty[stl[i].pty];
                text += "\n";
            }
        }
        // Tuned to a multiplex but no station chosen: once the names have been arriving for 3 s (so the list has
        // filled in), the first audio station in the list's order (JS dabAutoPlay)
        if (!sel && !hasPending && listFrames >= 31) {
            for (std::size_t i = 0; i < stl.size(); ++i)
                if ((stl[i].format == "DAB" || stl[i].format == "DAB+") && stl[i].hasSub && select(stl[i].sid)) break;
        }
        st.frames = frames;
        st.fibs = fibs;
        st.fibsOk = fibsOk;
        st.lost = lostCount;
        st.gaps = gaps;
        st.resyncs = sfResyncs;
    }

    // ---- the front end (Addition to the JS: the dial's channel centre mixed to 0 Hz when the stick isn't on it)
    void push(const std::complex<float>* iq, std::size_t n, double shiftHz) {
        if (off > 0 && off >= buf.size() / 2) {
            buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(off));
            off = 0;
        }
        const std::size_t at = buf.size();
        buf.resize(at + n);
        Cf* o = &buf[at];
        if (std::fabs(shiftHz) < 0.5) {
            for (std::size_t i = 0; i < n; ++i) {
                float r = iq[i].real(), m = iq[i].imag();
                if (!(r == r) || std::fabs(r) > 1e30f) r = 0;   // (NaN / Inf: 0)
                if (!(m == m) || std::fabs(m) > 1e30f) m = 0;
                o[i] = Cf(r, m);
            }
        } else {
            const double w = -2 * kPi * shiftHz / FS, sc = std::cos(w), ss = std::sin(w);
            double c = std::cos(mixPhase), s = std::sin(mixPhase);
            for (std::size_t i = 0; i < n; ++i) {
                float r = iq[i].real(), m = iq[i].imag();
                if (!(r == r) || std::fabs(r) > 1e30f) r = 0;
                if (!(m == m) || std::fabs(m) > 1e30f) m = 0;
                o[i] = Cf(static_cast<float>(r * c - m * s), static_cast<float>(r * s + m * c));
                const double t = c * sc - s * ss;
                s = c * ss + s * sc;
                c = t;
            }
            mixPhase = std::fmod(mixPhase + w * static_cast<double>(n), 2 * kPi);
        }
        if (skipIn) {
            const std::size_t k = std::min(skipIn, buf.size() - off);
            off += k;
            skipIn -= k;
        }
    }
};

// ============================================================================
//  DabDecoder
// ============================================================================
DabDecoder::DabDecoder() : impl_(new Impl) {}
DabDecoder::~DabDecoder() {}

void DabDecoder::reset() {
    const bool keep = impl_->keepUnits;
    impl_->clearAll();
    impl_->keepUnits = keep;
    impl_->userChose = false;
    impl_->lastDial = std::numeric_limits<double>::quiet_NaN();
}

void DabDecoder::process(const float* audio, std::size_t n, double rate) {
    (void)audio;
    (void)n;
    (void)rate;
}

void DabDecoder::processIq(const std::complex<float>* iq, std::size_t n, double rate, double centerHz, double dialHz) {
    Impl& m = *impl_;
    if (!n || !iq) return;
    if (std::fabs(rate - FS) > 1) {   // (the receiver needs 2.048 MS/s exactly, as the JS)
        m.rateOk = false;
        return;
    }
    m.rateOk = true;
    // (dabFeed: a retune starts afresh; a big one forgets the station chosen)
    if (m.lastDial == m.lastDial && std::fabs(dialHz - m.lastDial) > 1) {
        const bool far = std::fabs(dialHz - m.lastDial) > 800e3;
        const bool chose = m.userChose && !far;
        const unsigned sid = m.userSid;
        const bool keep = m.keepUnits;
        m.clearAll();
        m.keepUnits = keep;
        m.userChose = chose;
        if (chose) {
            m.hasPending = true;
            m.pending = sid;
        }
    }
    m.lastDial = dialHz;
    double target = dialHz;
    DabChannel ch;
    if (dabChannelNear(dialHz, ch)) target = ch.hz;
    m.push(iq, n, target - centerHz);
    while (m.step()) m.afterFrame(true);
    m.afterFrame(false);
}

void DabDecoder::takeText(std::string& out) {
    out += impl_->text;
    impl_->text.clear();
}

Status DabDecoder::status() const {
    const Impl& m = *impl_;
    Status s;
    s.locked = m.locked && m.everFic;
    s.signal = m.locked;
    s.snrDb = m.hasSnr ? m.snrDb : 0;
    s.quality = m.everFic ? m.ficRate : 0;
    s.mode = m.sel ? m.sel->format : "DAB";
    s.station = m.ensLabel;
    char b[256];
    if (!m.rateOk) {
        s.info = "DAB needs 2.048 MS/s";
        return s;
    }
    if (!m.locked) {
        s.info = m.state == "no DAB signal" ? "no DAB signal" : "DAB: searching";
        return s;
    }
    std::string info;
    if (m.hasEns)
        info = m.ensLabel + " (" + hex4(m.ensId) + ")";
    else
        info = "DAB locked";
    std::snprintf(b, sizeof b, " · %u stations · FIC %d%%", static_cast<unsigned>(m.labels.size()),
                  static_cast<int>(m.ficRate * 100 + 0.5));
    info += b;
    if (m.hasSnr) {
        std::snprintf(b, sizeof b, " · SNR %.0f dB", m.snrDb);
        info += b;
    }
    if (m.sel) {
        std::snprintf(b, sizeof b, " · %s %s %d kbit/s", m.sel->label.c_str(), m.sel->format.c_str(), m.sel->prot.kbps);
        info += b;
        if (!m.audioDesc.empty()) info += " " + m.audioDesc;
    }
    s.info = info;
    return s;
}

int DabDecoder::audioRate() const { return impl_->audioRate; }

void DabDecoder::takeAudio(std::vector<float>& stereo) {
    stereo.insert(stereo.end(), impl_->audio.begin(), impl_->audio.end());
    impl_->audio.clear();
}

std::vector<Service> DabDecoder::services() const {
    const Impl& m = *impl_;
    std::vector<Service> out;
    std::vector<Station> stl = m.stations();
    for (std::size_t i = 0; i < stl.size(); ++i) {
        if (stl[i].format != "DAB" && stl[i].format != "DAB+") continue;
        Service s;
        s.id = stl[i].sid;
        s.label = stl[i].label;
        s.audio = true;
        Protection p;
        std::string d = stl[i].format;
        if (stl[i].hasSub && protectionOf(stl[i].sc, p)) {
            s.bitrateKbps = p.kbps;
            char b[48];
            std::snprintf(b, sizeof b, " %d kbit/s", p.kbps);
            d += b;
        }
        s.selected = m.sel && m.sel->sid == s.id;
        if (s.selected && !m.audioDesc.empty()) d += " " + m.audioDesc;
        if (stl[i].pty > 0 && stl[i].pty < 32 && kPty[stl[i].pty][0]) d += std::string(", ") + kPty[stl[i].pty];
        s.detail = d;
        out.push_back(s);
    }
    return out;
}

bool DabDecoder::selectService(unsigned id) {
    Impl& m = *impl_;
    // (a station chosen before the list has it, after a retune say, is kept and selected as soon as it's heard of)
    m.sel.reset();
    m.userChose = true;
    m.userSid = id;
    m.hasPending = true;
    m.pending = id;
    if (m.select(id)) {
        m.hasPending = false;
        return true;
    }
    return false;
}

void DabDecoder::takeImages(std::vector<EncodedImage>& images) {
    images.insert(images.end(), impl_->images.begin(), impl_->images.end());
    impl_->images.clear();
}

std::string DabDecoder::ensembleLabel() const { return impl_->ensLabel; }
unsigned DabDecoder::ensembleId() const { return impl_->hasEns ? impl_->ensId : 0; }
unsigned DabDecoder::selectedService() const { return impl_->sel ? impl_->sel->sid : 0; }
std::string DabDecoder::dynamicLabel() const { return impl_->sel ? impl_->sel->pad.label : std::string(); }
DabStats DabDecoder::stats() const { return impl_->st; }
bool DabDecoder::codecsAvailable() const { return impl_->codecs && impl_->codecs->available(); }
void DabDecoder::setKeepUnits(bool on) { impl_->keepUnits = on; }
void DabDecoder::takeUnits(std::vector<DabUnit>& units) {
    units.insert(units.end(), impl_->units.begin(), impl_->units.end());
    impl_->units.clear();
}

}  // namespace dec
