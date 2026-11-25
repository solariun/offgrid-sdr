// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DrmRx.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The DRM receiver proper: Rx::analyse() reads one window of complex
//  samples at 48 kHz centred on the station (DRM's reference carrier k = 0
//  on 0 Hz) from scratch, as far as the station's name and settings:
//    1. the robustness mode and the symbol timing, from the guard intervals
//       (x[n] x*[n+Tu] summed over a guard, folded at the symbol period: a
//       sharp peak once a symbol);
//    2. the fine frequency from the guard correlation's phase, followed
//       through the window (a straight line fitted to each symbol's value,
//       or the smoothed values themselves after a jump); each symbol's
//       useful part through a DFT;
//    3. whole carriers: the frequency pilots' phase carries on from symbol
//       to symbol (every other symbol in mode D);
//    4. the frame start: the time references, neighbour against neighbour;
//    5. the channel, from the gain references (linear interpolation in
//       time along each pilot carrier, then across the carriers), then the
//       FAC (Viterbi, energy dispersal, CRC-8): robustness, occupancy,
//       interleaving, MSC mode, SDC mode, the services; the signal quality
//       (MER) from the FAC cells;
//    6. the SDC in each superframe's first frame (4-QAM or 16-QAM,
//       multilevel), checked by its CRC-16: the multiplex description,
//       labels, audio parameters (codec, rate, SBR, stereo, text, xHE-AAC
//       configuration), applications (SlideShow, Journaline).
//  Rx::mscFrames() then decodes the main channel of that window: the cells
//  in order across the superframe, cut into 3 multiplex frames, the cell
//  de-interleaver (short, or long over 5 frames), each carrier weighted by
//  its own reliability, multilevel decoding of part B (16/64-QAM, levels in
//  turn, each re-encoded to narrow the next one's choice; two more passes
//  for a frame in trouble), energy dispersal, the bytes.
//
//  Port of OFFgrid-SDR's JS (diddy-boy, https://github.com/solariun/
//  offgrid-sdr, index.html "drmlib": DrmRx.analyse / analyseOnce,
//  drmMscFrames), written from ETSI ES 201 980 following Dream (TU
//  Darmstadt, Volker Fischer and others; GPL-2.0-or-later,
//  licenses/dream-COPYING.txt).  The JS comments are kept, reworded, next to
//  the code they explain.
//
//  Differences from the JS:
//    - Addition to the JS: the FAC's short ID and programme type (or
//      application) are read too, and every service's FAC entry, label and
//      audio parameters are kept by short ID (the JS kept one service);
//    - Addition to the JS: a gain reference on carrier 0 (mode D has them)
//      is not used for the channel: the DC carrier is sent empty, and using
//      it pulled the channel estimate of its neighbours towards zero;
//    - Addition to the JS: an SDC entity that runs past the SDC's end stops
//      the parsing (the JS read past the end);
//    - the JS's disabled trials (fine timing from the pilots' slope,
//      smoothing of the channel estimate: both measured not to help) are
//      left out;
//    - the extra decoding passes are always allowed (the JS allowed them
//      while its worker kept up: a wall-clock test, see Rx::iterate).
//
//  How it fits in
//  --------------
//  Declared in DrmInternal.h; driven by DrmDecoder (Drm.cpp), which feeds it
//  a window every superframe (1.2 s) and joins the multiplex frames.
//
//  Contributing
//  ------------
//  Buffers are members, reused window after window.  Keep in step with the
//  JS and with the generator (DrmEncoders.cpp): the tests run one into the
//  other.
// ============================================================================
#include "decoders/DrmInternal.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dec {
namespace drm {

namespace {

const double kPi = 3.14159265358979323846;
const double kSqrt2 = 1.41421356237309504880;

inline long long cellKey(int s, int k) { return static_cast<long long>(s) * 65536 + k + 32768; }

std::string trimLabel(const Bytes& b) {
    std::string s(b.begin(), b.end());
    while (!s.empty() && s[s.size() - 1] == '\0') s.erase(s.size() - 1);
    std::size_t a = 0, e = s.size();
    while (a < e && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) ++a;
    while (e > a && (s[e - 1] == ' ' || s[e - 1] == '\t' || s[e - 1] == '\r' || s[e - 1] == '\n')) --e;
    std::string out;
    for (std::size_t i = a; i < e; ++i)
        if (static_cast<unsigned char>(s[i]) >= 0x20 || s[i] == '\t') out += s[i];  // (no control characters)
    return out;
}

// lin(): linear interpolation between the marked points of a strided row,
// held at the ends (as numpy's interp).
void interpolate(std::complex<float>* v, unsigned char* mark, int len, int stride, std::vector<int>& pts) {
    pts.clear();
    for (int j = 0; j < len; ++j)
        if (mark[static_cast<std::size_t>(j) * stride]) pts.push_back(j);
    if (pts.size() < 2) return;
    std::size_t p = 0;
    for (int j = 0; j < len; ++j) {
        while (p < pts.size() - 2 && pts[p + 1] < j) ++p;
        const int a = pts[p], b = pts[p + 1];
        float w = static_cast<float>(j - a) / static_cast<float>(b - a);
        if (w < 0) w = 0;
        if (w > 1) w = 1;
        const std::complex<float> va = v[static_cast<std::size_t>(a) * stride], vb = v[static_cast<std::size_t>(b) * stride];
        v[static_cast<std::size_t>(j) * stride] = va * (1 - w) + vb * w;
        mark[static_cast<std::size_t>(j) * stride] = 1;
    }
}

}  // namespace

void RxResult::clear() {
    metric = 0;
    mode = -1;
    state.clear();
    facOk = facTried = sdcOk = sdcTried = 0;
    hasMer = false;
    mer = 0;
    freqOffset = 0;
    sh = 0;
    drift = 0;
    mirrored = false;
    frames.clear();
    settings = false;
    so = mscCode = services = 0;
    longIl = sdc16 = false;
    for (int i = 0; i < 4; ++i) {
        svcSeen[i] = false;
        svc[i] = FacFrame();
        haveLabel[i] = false;
        labels[i].clear();
        audio[i] = AudioParams();
    }
    label.clear();
    mux = Mux();
    apps.clear();
}

Rx::Rx()
    : keep(false), iterate(true), mi_(0), Tu_(0), Tg_(0), L_(0), c0_(0), sh_(0), S0_(0), nsym_(0), nframes_(0),
      start_(0), kmin_(0), kmax_(0), W2_(0) {}

// A clear DRM signal whose settings can't be read is tried again mirrored
// (the spectrum turned round, as a receiver tuned as lower sideband, or IQ
// leads swapped, gives it): the FAC's CRC says which way round is right, so
// a wrong guess can't give false settings.
void Rx::analyse(const std::complex<float>* x, std::size_t n, RxResult& out) {
    // (a station last found mirrored is tried mirrored first)
    if (hint.valid && hint.mirrored) {
        analyseOnce(x, n, true, out);
        if (out.facOk) {
            out.mirrored = true;
            return;
        }
        analyseOnce(x, n, false, out);
        return;
    }
    analyseOnce(x, n, false, out);
    if (out.facOk || !(out.metric >= 0.4)) return;
    RxResult m;
    analyseOnce(x, n, true, m);
    if (m.facOk) {
        m.mirrored = true;
        out = m;
    }
    // (else the first reading stands; it has no settings, so the cells kept,
    //  the mirrored ones, are never asked for)
}

void Rx::chan(int kmin, int kmax, const int* boost, int nboost, std::vector<std::complex<float> >& H) {
    const ModeTable& tb = modeTable(mi_);
    const int W = kmax - kmin + 1, rows = nframes_ * L_;
    H.assign(static_cast<std::size_t>(rows) * W, std::complex<float>());
    mark_.assign(static_cast<std::size_t>(rows) * W, 0);
    std::vector<std::vector<IntPair> > g(static_cast<std::size_t>(L_));
    for (int s = 0; s < L_; ++s) gainRefs(mi_, s, kmin, kmax, g[static_cast<std::size_t>(s)]);
    // (where a gain reference shares its cell with a frequency pilot, or in
    //  symbol 0 a time reference, that pilot is what's sent there, with its own
    //  phase: as Dream's mapping, where they take precedence.  Modes A, C and D
    //  have such cells; taking them as gain references put the channel
    //  estimate wrong around them)
    for (int s = 0; s < rows; ++s) {
        const std::vector<IntPair>& gs = g[static_cast<std::size_t>(s % L_)];
        for (std::size_t q = 0; q < gs.size(); ++q) {
            const int k = gs[q].first;
            if (k == 0) continue;  // Addition to the JS: the empty DC carrier
            int ph = gs[q].second;
            if (s % L_ == 0)
                for (int j = 0; j < tb.nTime; ++j)
                    if (tb.time[2 * j] == k) ph = tb.time[2 * j + 1];
            for (int j = 0; j < tb.nFreq; ++j)
                if (tb.freq[2 * j] == k)
                    ph = (tb.freq[2 * j + 1] + (mi_ == 3 && (s % L_) % 2 == 1 && j < 2 ? 512 : 0)) % 1024;
            bool boosted = false;
            for (int j = 0; j < nboost; ++j)
                if (boost[j] == k) boosted = true;
            const double amp = boosted ? 2 : kSqrt2, a = -2 * kPi * ph / 1024;
            const std::complex<double> h = C(s, k) * std::complex<double>(std::cos(a), std::sin(a)) / amp;
            const std::size_t at = static_cast<std::size_t>(s) * W + (k - kmin);
            H[at] = std::complex<float>(h);
            mark_[at] = 1;
        }
    }
    std::vector<int> pts;
    // along each carrier in time, then across the carriers in each symbol
    for (int c = 0; c < W; ++c) interpolate(&H[static_cast<std::size_t>(c)], &mark_[static_cast<std::size_t>(c)], rows, W, pts);
    for (int s = 0; s < rows; ++s)
        interpolate(&H[static_cast<std::size_t>(s) * W], &mark_[static_cast<std::size_t>(s) * W], W, 1, pts);
}

void Rx::analyseOnce(const std::complex<float>* x, std::size_t n, bool mirror, RxResult& out) {
    out.clear();
    xin_.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        float re = x[i].real(), im = x[i].imag();
        if (!(re == re) || std::fabs(re) > 1e30f) re = 0;  // (non-finite samples: nothing)
        if (!(im == im) || std::fabs(im) > 1e30f) im = 0;
        xin_[i] = std::complex<double>(re, mirror ? -im : im);
    }
    const std::complex<double>* X = n ? &xin_[0] : 0;

    // 1. the mode and symbol timing, from the guard intervals: x[n] x*[n+Tu]
    //    summed over a guard, folded at the symbol period
    bool found = false;
    int bMode = 0, bStart = 0;
    double bMetric = 0, bPh = 0;
    for (int mi = 0; mi < 4; ++mi) {
        if (hint.valid && mi != hint.mode) continue;  // (only the known mode, with a hint)
        const int Tu = kTu[mi], Tg = kTg[mi], Ts = Tu + Tg;
        if (n < static_cast<std::size_t>(Tu + 2 * Ts)) continue;
        std::vector<double> cr(static_cast<std::size_t>(Ts)), ci(static_cast<std::size_t>(Ts)), en(static_cast<std::size_t>(Ts));
        double sr = 0, si = 0, se = 0;
        const long long span = std::min<long long>(static_cast<long long>(n) - Tu, 48000LL * 16 / 10);
        for (long long i = 0; i < span; ++i) {
            const std::complex<double> p = X[i] * std::conj(X[i + Tu]);
            sr += p.real();
            si += p.imag();
            se += 0.5 * (std::norm(X[i]) + std::norm(X[i + Tu]));
            if (i >= Tg) {
                const long long j = i - Tg;
                const std::complex<double> q = X[j] * std::conj(X[j + Tu]);
                sr -= q.real();
                si -= q.imag();
                se -= 0.5 * (std::norm(X[j]) + std::norm(X[j + Tu]));
            }
            if (i >= Tg - 1) {
                const std::size_t k = static_cast<std::size_t>((i - Tg + 1) % Ts);
                cr[k] += sr;
                ci[k] += si;
                en[k] += se;
            }
        }
        // (the peak above the typical level: DRM's guard intervals make a sharp
        //  peak once a symbol; a steady carrier, as an AM station's, correlates
        //  with itself at every delay, high everywhere with no peak, and mustn't
        //  count as DRM)
        std::vector<double> ms(static_cast<std::size_t>(Ts));
        int bk = 0;
        double bv = 0;
        for (int k = 0; k < Ts; ++k) {
            const double e = en[static_cast<std::size_t>(k)];
            ms[static_cast<std::size_t>(k)] = std::sqrt(cr[static_cast<std::size_t>(k)] * cr[static_cast<std::size_t>(k)] +
                                                        ci[static_cast<std::size_t>(k)] * ci[static_cast<std::size_t>(k)]) /
                                              (e != 0 ? e : 1);
            if (ms[static_cast<std::size_t>(k)] > bv) {
                bv = ms[static_cast<std::size_t>(k)];
                bk = k;
            }
        }
        std::vector<double> sorted(ms);
        std::nth_element(sorted.begin(), sorted.begin() + (Ts >> 1), sorted.end());
        const double peak = bv - sorted[static_cast<std::size_t>(Ts >> 1)];
        if (!found || peak > bMetric) {
            found = true;
            bMode = mi;
            bMetric = peak;
            bStart = bk;
            bPh = std::atan2(ci[static_cast<std::size_t>(bk)], cr[static_cast<std::size_t>(bk)]);
        }
    }
    if (!found) {
        out.state = "no DRM signal";
        return;
    }
    out.metric = bMetric;
    out.mode = bMode;
    if (bMetric < 0.4) {
        out.state = "no DRM signal";
        return;
    }
    const int mi = bMode, Tu = kTu[mi], Tg = kTg[mi], L = kSymbols[mi], Ts = Tu + Tg;
    const ModeTable& tb = modeTable(mi);
    mi_ = mi;
    Tu_ = Tu;
    Tg_ = Tg;
    L_ = L;
    start_ = bStart;
    const double fo = -bPh * kRate / (2 * kPi * Tu);

    // 2. each symbol's carriers, the fine frequency taken out.  FOLLOWED
    //    through the window: each symbol's guard interval measured (smoothed
    //    over +-4 symbols), its phase unwrapped from the window's own estimate,
    //    and a straight line fitted, weighted by strength; then followed with a
    //    continuous phase.  A drifting transmitter or stick no longer spoils a
    //    window; a steady signal gives a flat line.  If the line disagrees with
    //    the window's estimate at its start, the window's single value is used.
    Dft& dft = dft_[Tu];
    if (dft.size() != Tu) dft.init(Tu);
    const long long nsymL = (static_cast<long long>(n) - bStart - Ts) / Ts;
    const int nsym = nsymL > 0 ? static_cast<int>(nsymL) : 0;
    nsym_ = nsym;
    const int c0 = Tu / 2;
    c0_ = c0;
    double fA = fo, fB = 0;
    bool local = false;
    std::vector<double> fsym(static_cast<std::size_t>(nsym)), wsym(static_cast<std::size_t>(nsym));
    if (nsym > 8) {
        std::vector<std::complex<double> > gc(static_cast<std::size_t>(nsym));
        for (int s = 0; s < nsym; ++s) {
            const long long g0 = bStart + static_cast<long long>(s) * Ts;
            std::complex<double> acc;
            for (int j = 0; j < Tg; ++j) acc += X[g0 + j] * std::conj(X[g0 + j + Tu]);
            gc[static_cast<std::size_t>(s)] = acc;
        }
        const int Wn = 4;
        const double step = static_cast<double>(kRate) / Tu;
        double prevF = fo, sw = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int s = 0; s < nsym; ++s) {
            std::complex<double> a;
            for (int k = std::max(0, s - Wn); k <= std::min(nsym - 1, s + Wn); ++k) a += gc[static_cast<std::size_t>(k)];
            double f = -std::atan2(a.imag(), a.real()) * kRate / (2 * kPi * Tu);
            while (f - prevF > step / 2) f -= step;
            while (f - prevF < -step / 2) f += step;
            prevF = f;
            const double w = std::abs(a);
            fsym[static_cast<std::size_t>(s)] = f;
            wsym[static_cast<std::size_t>(s)] = w;
            sw += w;
            sx += w * s;
            sy += w * f;
            sxx += w * s * s;
            sxy += w * s * f;
        }
        const double den = sw * sxx - sx * sx, d = den > 0 ? (sw * sxy - sx * sy) / den : 0,
                     a0 = sw > 0 ? (sy - d * sx) / sw : fo;
        if (std::isfinite(d) && std::isfinite(a0) && std::fabs(a0 - fo) < step / 4) {
            fA = a0;
            fB = d;
            // (a jump rather than a drift: the measurements stray from the line.
            //  Then the smoothed per-symbol values are followed directly)
            double r2 = 0;
            for (int s = 0; s < nsym; ++s) {
                const double e = fsym[static_cast<std::size_t>(s)] - (a0 + d * s);
                r2 += wsym[static_cast<std::size_t>(s)] * e * e;
            }
            if (sw > 0 && std::sqrt(r2 / sw) > step / 10) local = true;
        }
        out.drift = std::floor(fB * kRate / Ts * 100 + 0.5) / 100;  // (Hz a second)
    }
    S_.resize(static_cast<std::size_t>(nsym) * Tu);
    tx_.resize(static_cast<std::size_t>(Tu));
    tX_.resize(static_cast<std::size_t>(Tu));
    double th = 0;  // (the followed phase at each symbol's start, per-symbol case)
    for (int s = 0; s < nsym; ++s) {
        const long long a = bStart + static_cast<long long>(s) * Ts + Tg;
        if (local) {
            const double f = fsym[static_cast<std::size_t>(s)];
            for (int j = 0; j < Tu; ++j) {
                const double ph = th - 2 * kPi * f * (Tg + j) / kRate;
                tx_[static_cast<std::size_t>(j)] = X[a + j] * std::complex<double>(std::cos(ph), std::sin(ph));
            }
            th -= 2 * kPi * f * Ts / kRate;
        } else {
            for (int j = 0; j < Tu; ++j) {
                const double u = static_cast<double>(a + j - bStart);
                const double ph = -2 * kPi * (fA * u + fB * u * u / (2.0 * Ts)) / kRate;
                tx_[static_cast<std::size_t>(j)] = X[a + j] * std::complex<double>(std::cos(ph), std::sin(ph));
            }
        }
        dft.forward(&tx_[0], &tX_[0]);
        std::complex<float>* row = &S_[static_cast<std::size_t>(s) * Tu];
        for (int k = 0; k < Tu; ++k) row[k] = std::complex<float>(tX_[static_cast<std::size_t>((k + c0) % Tu)]);  // (carrier k at c0 + k)
    }

    // 3. whole carriers: the frequency pilots' phase carries on from symbol to
    //    symbol (two symbols in mode D).  (Each bin's steadiness worked out
    //    only where the search looks, and once)
    const int lag = mi == 3 ? 2 : 1;
    steady_.assign(static_cast<std::size_t>(Tu), -1);
    struct Steady {
        const Rx* rx;
        std::vector<double>* cache;
        int lag, nsym, Tu;
        double at(int k) const {
            if (k < 0 || k >= Tu) return 0;
            double& c = (*cache)[static_cast<std::size_t>(k)];
            if (c >= 0) return c;
            std::complex<double> acc;
            double am = 0;
            for (int s = lag; s < nsym; ++s) {
                const std::complex<double> z = std::complex<double>(rx->S_[static_cast<std::size_t>(s) * Tu + k]) *
                                               std::conj(std::complex<double>(rx->S_[static_cast<std::size_t>(s - lag) * Tu + k]));
                acc += z;
                am += std::abs(z);
            }
            return (c = std::abs(acc) / (am != 0 ? am : 1));
        }
    };
    Steady st = {this, &steady_, lag, nsym, Tu};
    int sh = 0;
    double shv = -1;
    const bool hs = hint.valid;
    for (int h = hs ? hint.sh - 2 : -40; h <= (hs ? hint.sh + 2 : 40); ++h) {
        double on = 0, nearv = 0;
        for (int j = 0; j < tb.nFreq; ++j) {
            const int k = tb.freq[2 * j];
            on += st.at(c0 + k + h);
            static const int dd[4] = {-3, -2, 2, 3};
            for (int q = 0; q < 4; ++q) nearv += st.at(c0 + k + h + dd[q]);
        }
        const double v = on / tb.nFreq - nearv / (tb.nFreq * 4.0);
        if (v > shv) {
            shv = v;
            sh = h;
        }
    }
    out.freqOffset = fo + static_cast<double>(sh) * kRate / Tu;
    out.sh = sh;
    sh_ = sh;
    // (a whole-carrier offset also turns each symbol's phase by sh x Ts/Tu
    //  turns: taken out)
    for (int s = 0; s < nsym; ++s) {
        const double a = -2 * kPi * sh * static_cast<double>(Ts) / Tu * s;
        const std::complex<float> r(static_cast<float>(std::cos(a)), static_cast<float>(std::sin(a)));
        std::complex<float>* row = &S_[static_cast<std::size_t>(s) * Tu];
        for (int k = 0; k < Tu; ++k) row[k] *= r;
    }
    // (the carriers used must lie inside the transform, whatever the offset)
    int kLo = 0, kHi = 0;
    for (int so = 0; so < 6; ++so) {
        kLo = std::min(kLo, kMin(so, mi));
        kHi = std::max(kHi, kMax(so, mi));
    }
    if (c0 + sh + kLo - 1 < 0 || c0 + sh + kHi + 1 >= Tu) {
        out.state = "DRM signal found, settings not readable (weak or fading)";
        return;
    }

    // 4. the frame start: the time references, neighbour against neighbour
    //    (the channel's phase drops out)
    S0_ = 0;
    std::vector<double> score(static_cast<std::size_t>(L), 0.0);
    for (int s = 0; s < nsym; ++s) {
        std::complex<double> acc;
        double e = 0;
        for (int j = 0; j + 1 < tb.nTime; ++j) {
            const int k1 = tb.time[2 * j], p1 = tb.time[2 * j + 1], k2 = tb.time[2 * j + 2], p2 = tb.time[2 * j + 3];
            const std::complex<double> a1 = C(s, k1), a2 = C(s, k2);
            const double ph = -2 * kPi * (p1 - p2) / 1024;
            acc += a1 * std::conj(a2) * std::complex<double>(std::cos(ph), std::sin(ph));
            e += std::norm(a1);
        }
        score[static_cast<std::size_t>(s % L)] += std::abs(acc) / (e != 0 ? e : 1);
    }
    int f0 = 0;
    for (int s = 1; s < L; ++s)
        if (score[static_cast<std::size_t>(s)] > score[static_cast<std::size_t>(f0)]) f0 = s;

    // 5. the channel, from the gain references (for 10 kHz first: the FAC is
    //    inside every width), then the FAC
    const int frames = nsym > f0 ? (nsym - f0) / L : 0;
    nframes_ = frames;
    S0_ = f0;
    const int so3 = 3, k3min = kMin(so3, mi), k3max = kMax(so3, mi), W1 = k3max - k3min + 1;
    chan(k3min, k3max, 0, 0, H1_);
    std::vector<int> il130;
    ilTable(130, 21, il130);
    Pattern facPat;
    makePattern(6, 72, 65, true, facPat);
    Bytes pr72, b;
    prbs(72, pr72);
    std::vector<double> soft(130);
    int firstIdx = -1;
    for (int fr = 1; fr < frames - 1; ++fr) {
        for (int j = 0; j < tb.nFac; ++j) {
            const int sym = tb.fac[2 * j], k = tb.fac[2 * j + 1], s = fr * L + sym;
            const std::complex<double> z = C(s, k) * std::conj(std::complex<double>(H1_[static_cast<std::size_t>(s) * W1 + (k - k3min)]));
            soft[static_cast<std::size_t>(il130[static_cast<std::size_t>(2 * j)])] = z.real();
            soft[static_cast<std::size_t>(il130[static_cast<std::size_t>(2 * j + 1)])] = z.imag();
        }
        vit_.decode(&soft[0], facPat, b);
        for (int j = 0; j < 72; ++j) b[static_cast<std::size_t>(j)] ^= pr72[static_cast<std::size_t>(j)];
        unsigned char bytes[8];
        for (int j = 0; j < 8; ++j) bytes[j] = static_cast<unsigned char>(bitsNum(&b[0], static_cast<std::size_t>(8 * j), 8));
        if (crcDream(bytes, 8, 8) != bitsNum(&b[0], 64, 8)) continue;
        ++out.facOk;
        FacFrame F;
        F.fr = fr;
        F.frame = static_cast<int>(bitsNum(&b[0], 1, 2));
        F.so = static_cast<int>(bitsNum(&b[0], 3, 4));
        F.longIl = !bitsNum(&b[0], 7, 1);
        F.mscCode = static_cast<int>(bitsNum(&b[0], 8, 2));
        F.sdc16 = !bitsNum(&b[0], 10, 1);
        F.services = static_cast<int>(bitsNum(&b[0], 11, 4));
        F.serviceId = static_cast<int>(bitsNum(&b[0], 20, 24));
        F.shortId = static_cast<int>(bitsNum(&b[0], 44, 2));  // Addition to the JS
        F.lang = static_cast<int>(bitsNum(&b[0], 47, 4));
        F.data = bitsNum(&b[0], 51, 1) != 0;
        F.ptype = static_cast<int>(bitsNum(&b[0], 52, 5));    // Addition to the JS
        out.frames.push_back(F);
        if (firstIdx < 0) firstIdx = static_cast<int>(out.frames.size()) - 1;
        out.svcSeen[F.shortId] = true;
        out.svc[F.shortId] = F;
    }
    out.facTried = std::max(0, frames - 2);
    // the signal quality (MER, dB), from the FAC cells of the frames that
    // passed: each frame's cells scaled to 4-QAM's power, then the error from
    // the nearest point.  16-QAM needs about 10 dB, 64-QAM about 16
    {
        double es = 0;
        long long ps = 0;
        std::vector<std::complex<double> > pts(static_cast<std::size_t>(tb.nFac));
        for (std::size_t q = 0; q < out.frames.size(); ++q) {
            double pw = 0;
            for (int j = 0; j < tb.nFac; ++j) {
                const int sym = tb.fac[2 * j], k = tb.fac[2 * j + 1], s = out.frames[q].fr * L + sym;
                const std::complex<double> h(H1_[static_cast<std::size_t>(s) * W1 + (k - k3min)]);
                double m = std::norm(h);
                if (m == 0) m = 1e-12;
                pts[static_cast<std::size_t>(j)] = C(s, k) * std::conj(h) / m;
                pw += std::norm(pts[static_cast<std::size_t>(j)]);
            }
            pw /= tb.nFac;
            const double g = 1 / std::sqrt(pw != 0 ? pw : 1);
            for (int j = 0; j < tb.nFac; ++j) {
                const double X1 = pts[static_cast<std::size_t>(j)].real() * g, Y1 = pts[static_cast<std::size_t>(j)].imag() * g;
                const double sx = X1 > 0 ? 1 : X1 < 0 ? -1 : 0, sy = Y1 > 0 ? 1 : Y1 < 0 ? -1 : 0;
                es += (X1 - sx / kSqrt2) * (X1 - sx / kSqrt2) + (Y1 - sy / kSqrt2) * (Y1 - sy / kSqrt2);
                ++ps;
            }
        }
        if (ps) {
            out.hasMer = true;
            out.mer = 10 * std::log10(static_cast<double>(ps) / (es != 0 ? es : 1e-9));
        }
    }
    if (firstIdx < 0) {
        out.state = "DRM signal found, settings not readable (weak or fading)";
        return;
    }
    const FacFrame first = out.frames[static_cast<std::size_t>(firstIdx)];
    out.settings = true;
    out.so = first.so;
    out.mscCode = first.mscCode;
    out.longIl = first.longIl;
    out.sdc16 = first.sdc16;
    out.services = first.services;
    out.state = "settings read";

    // 6. the SDC, in each superframe's first frame (frame 0, or 3), with the
    //    channel for the real width and its boosted pilots
    const int so = first.so, kmin = kMin(so, mi), kmax = kMax(so, mi);
    kmin_ = kmin;
    kmax_ = kmax;
    if (kmin == 0 && kmax == 0) {  // (an occupancy this mode doesn't have)
        out.state = "settings read (an occupancy this mode does not have)";
        return;
    }
    const int W2 = kmax - kmin + 1;
    W2_ = W2;
    chan(kmin, kmax, &tb.boost[so * 4], 4, H2_);
    fp_.clear();
    tp_.clear();
    for (int j = 0; j < tb.nFreq; ++j) fp_.insert(tb.freq[2 * j]);
    for (int j = 0; j < tb.nTime; ++j) tp_.insert(tb.time[2 * j]);
    facCells_.clear();
    for (int j = 0; j < tb.nFac; ++j) facCells_.insert(cellKey(tb.fac[2 * j], tb.fac[2 * j + 1]));
    sdcCells_.clear();
    std::vector<IntPair> cells, g;
    for (int s = 0; s < (mi == 0 || mi == 1 ? 2 : 3); ++s) {
        gainRefs(mi, s, kmin, kmax, g);
        std::set<int> gs;
        for (std::size_t q = 0; q < g.size(); ++q) gs.insert(g[q].first);
        for (int k = kmin; k <= kmax; ++k) {
            if (k == 0 || (mi == 0 && std::abs(k) == 1) || gs.count(k) || fp_.count(k) || (s == 0 && tp_.count(k))) continue;
            cells.push_back(IntPair(s, k));
            sdcCells_.insert(cellKey(s, k));
        }
    }
    const int N = static_cast<int>(cells.size());
    const double a = 1 / std::sqrt(10.0);
    std::vector<double> xs(static_cast<std::size_t>(2 * N)), ws(static_cast<std::size_t>(2 * N)), y(static_cast<std::size_t>(2 * N));
    Bytes bits, b0, c0b, b1, pr;
    std::vector<int> il0, il1;
    Pattern pat0, pat1;
    if (first.sdc16) {
        const int M0 = punct(2)[0] * ((2 * N - 12) / punct(2)[1]), M1 = punct(7)[0] * ((2 * N - 12) / punct(7)[1]);
        makePattern(2, M0, N, false, pat0);
        makePattern(7, M1, N, false, pat1);
        ilTable(2 * N, 13, il0);
        ilTable(2 * N, 21, il1);
    } else {
        const int M = punct(4)[0] * ((2 * N - 12) / punct(4)[1]);
        makePattern(4, M, N, false, pat0);
        ilTable(2 * N, 21, il0);
    }
    for (std::size_t q = 0; q < out.frames.size() && N > 6; ++q) {
        const FacFrame& F = out.frames[q];
        if (F.frame != 0 && F.frame != 3) continue;
        for (int j = 0; j < N; ++j) {
            const int sym = cells[static_cast<std::size_t>(j)].first, k = cells[static_cast<std::size_t>(j)].second, s = F.fr * L + sym;
            const std::complex<double> h(H2_[static_cast<std::size_t>(s) * W2 + (k - kmin)]);
            double m = std::norm(h);
            if (m == 0) m = 1e-12;
            const std::complex<double> z = C(s, k) * std::conj(h) / m;
            xs[static_cast<std::size_t>(2 * j)] = z.real();
            xs[static_cast<std::size_t>(2 * j + 1)] = z.imag();
            ws[static_cast<std::size_t>(2 * j)] = ws[static_cast<std::size_t>(2 * j + 1)] = m;
        }
        if (first.sdc16) {
            for (int j = 0; j < 2 * N; ++j) {
                const double v = xs[static_cast<std::size_t>(j)];
                const double d0 = std::min(std::fabs(v - 3 * a), std::fabs(v + a)), d1 = std::min(std::fabs(v - a), std::fabs(v + 3 * a));
                y[static_cast<std::size_t>(il0[static_cast<std::size_t>(j)])] = (d1 * d1 - d0 * d0) * ws[static_cast<std::size_t>(j)];
            }
            vit_.decode(&y[0], pat0, b0);
            convEncode(b0.empty() ? 0 : &b0[0], b0.size(), pat0, c0b);
            for (int j = 0; j < 2 * N; ++j) {
                const int y0 = c0b[static_cast<std::size_t>(il0[static_cast<std::size_t>(j)])];
                const double p0 = y0 ? a : 3 * a, p1 = y0 ? -3 * a : -a, v = xs[static_cast<std::size_t>(j)];
                y[static_cast<std::size_t>(il1[static_cast<std::size_t>(j)])] = ((v - p1) * (v - p1) - (v - p0) * (v - p0)) * ws[static_cast<std::size_t>(j)];
            }
            vit_.decode(&y[0], pat1, b1);
            bits = b0;
            bits.insert(bits.end(), b1.begin(), b1.end());
        } else {
            for (int j = 0; j < 2 * N; ++j)
                y[static_cast<std::size_t>(il0[static_cast<std::size_t>(j)])] = xs[static_cast<std::size_t>(j)] * ws[static_cast<std::size_t>(j)];
            vit_.decode(&y[0], pat0, bits);
        }
        prbs(bits.size(), pr);
        for (std::size_t j = 0; j < bits.size(); ++j) bits[j] ^= pr[j];
        const int nb = static_cast<int>(bits.size());
        if (nb < 36) continue;
        const int useful = 20 + 8 * ((nb - 20) / 8);
        Bytes bytes;
        bytes.push_back(static_cast<unsigned char>(bitsNum(&bits[0], 0, 4)));
        for (int j = 0; j < (useful - 4) / 8 - 2; ++j) bytes.push_back(static_cast<unsigned char>(bitsNum(&bits[0], static_cast<std::size_t>(4 + 8 * j), 8)));
        ++out.sdcTried;
        if (crcDream(&bytes[0], bytes.size(), 16) != bitsNum(&bits[0], static_cast<std::size_t>(useful - 16), 16)) continue;
        ++out.sdcOk;
        // the data entities: 7-bit length (bytes after the first 4 bits of the
        // body), version, 4-bit type, body
        int pos = 4;
        const int end = useful - 16;
        const unsigned char* B = &bits[0];
        while (pos + 12 <= end) {
            const int len = static_cast<int>(bitsNum(B, static_cast<std::size_t>(pos), 7));
            if (!len) break;
            const int type = static_cast<int>(bitsNum(B, static_cast<std::size_t>(pos + 8), 4)), body = pos + 12;
            if (body + 4 + 8 * len > end) break;  // Addition to the JS: past the end
            const std::size_t ub = static_cast<std::size_t>(body);
            if (type == 1) {  // the label (of the service with this short ID)
                const int sid = static_cast<int>(bitsNum(B, ub, 2));
                Bytes lb;
                for (int j = 0; j < len; ++j) lb.push_back(static_cast<unsigned char>(bitsNum(B, ub + 4 + 8 * static_cast<std::size_t>(j), 8)));
                out.label = trimLabel(lb);
                out.labels[sid] = out.label;
                out.haveLabel[sid] = true;
            }
            if (type == 0 && !out.mux.present) {  // the multiplex description: protection levels, then each stream's bytes a frame (parts A and B)
                out.mux.present = true;
                out.mux.protA = static_cast<int>(bitsNum(B, ub, 2));
                out.mux.protB = static_cast<int>(bitsNum(B, ub + 2, 2));
                for (int j = 0; j < len / 3; ++j)
                    out.mux.streams.push_back(IntPair(static_cast<int>(bitsNum(B, ub + 4 + 24 * static_cast<std::size_t>(j), 12)),
                                                      static_cast<int>(bitsNum(B, ub + 16 + 24 * static_cast<std::size_t>(j), 12))));
            }
            // (type 5, application information, as Dream reads it: the stream,
            //  packet mode or not, and in packet mode the packet ID, the
            //  application domain (1 = DAB's) and packet length; for DAB's
            //  domain, the user application: 2 = SlideShow, 0x44A = Journaline)
            if (type == 5) {
                DataApp ap;
                ap.shortId = static_cast<int>(bitsNum(B, ub, 2));
                ap.stream = static_cast<int>(bitsNum(B, ub + 2, 2));
                ap.packet = bitsNum(B, ub + 4, 1) != 0;
                if (ap.packet) {
                    ap.dataUnits = bitsNum(B, ub + 5, 1) != 0;
                    ap.packetId = static_cast<int>(bitsNum(B, ub + 6, 2));
                    ap.domain = static_cast<int>(bitsNum(B, ub + 8, 4));
                    ap.packetLen = static_cast<int>(bitsNum(B, ub + 12, 8));
                    if (ap.domain == 1 && len >= 4) ap.app = static_cast<int>(bitsNum(B, ub + 25, 11));
                }
                bool dup = false;  // (each window reads the SDC several times)
                for (std::size_t z = 0; z < out.apps.size(); ++z)
                    if (out.apps[z].shortId == ap.shortId && out.apps[z].stream == ap.stream && out.apps[z].packetId == ap.packetId &&
                        out.apps[z].app == ap.app)
                        dup = true;
                if (!dup) out.apps.push_back(ap);
            }
            if (type == 9) {  // the audio parameters
                const int sid = static_cast<int>(bitsNum(B, ub, 2));
                AudioParams& A = out.audio[sid];
                if (!A.present) {
                    A.present = true;
                    A.streamId = static_cast<int>(bitsNum(B, ub + 2, 2));
                    A.codec = static_cast<int>(bitsNum(B, ub + 4, 2));
                    A.sbr = bitsNum(B, ub + 6, 1) != 0;
                    A.mode = static_cast<int>(bitsNum(B, ub + 7, 2));
                    A.rateCode = static_cast<int>(bitsNum(B, ub + 9, 3));
                    A.text = bitsNum(B, ub + 12, 1) != 0;
                    // (xHE-AAC: the codec's configuration follows the two bytes
                    //  above: DRM's compact form, kept as bytes)
                    if (A.codec == 3)
                        for (int j = 2; j < len; ++j) A.xheConfig.push_back(static_cast<unsigned char>(bitsNum(B, ub + 4 + 8 * static_cast<std::size_t>(j), 8)));
                }
            }
            pos = body + 4 + 8 * len;
        }
    }
    out.state = out.label.empty() ? "settings read" : "station name read";
}

// ---- the main channel (MSC), from a window analysed with keep = true.  Every
// multiplex frame gets a number from its position in the stream (a
// superframe is always 57,600 samples, 1.2 s), so that overlapping windows
// can be joined without repeats.  (ETSI 7.3-7.6, following Dream: cells in
// order across the superframe, cut into 3 multiplex frames of N_MUX cells,
// the cell de-interleaver, multilevel decoding of part B, energy dispersal,
// then the bytes)
void Rx::mscFrames(const RxResult& r, long long winStart, long long after, std::vector<MuxFrame>& out) {
    out.clear();
    if (!keep || !r.settings || !r.mux.present || r.mux.streams.empty() || kmin_ == kmax_) return;
    int fr0 = -1;
    for (std::size_t q = 0; q < r.frames.size(); ++q)
        if (r.frames[q].frame == 0 || r.frames[q].frame == 3) {
            fr0 = r.frames[q].fr;
            break;
        }
    if (fr0 < 0) return;
    while (fr0 - 3 >= 0) fr0 -= 3;
    const int L = L_, mi = mi_, kmin = kmin_, kmax = kmax_, NK = kmax - kmin + 1, Ts = Tu_ + Tg_;
    // (part A and hierarchical modulation: not yet)
    for (std::size_t q = 0; q < r.mux.streams.size(); ++q)
        if (r.mux.streams[q].first > 0) return;
    if (r.mscCode == 1 || r.mscCode == 2) return;
    const bool q64 = r.mscCode == 0;
    const int prot = r.mux.protB;
    if (!q64 && prot > 1) return;
    // (a table of which cells are audio data, made once)
    std::vector<unsigned char> mscTab(static_cast<std::size_t>(3 * L * NK), 0);
    std::vector<IntPair> g;
    int perSuper = 0;
    for (int sym = 0; sym < L; ++sym) {
        gainRefs(mi, sym, kmin, kmax, g);
        std::set<int> gs;
        for (std::size_t q = 0; q < g.size(); ++q) gs.insert(g[q].first);
        for (int f = 0; f < 3; ++f)
            for (int k = kmin; k <= kmax; ++k) {
                const bool no = k == 0 || (mi == 0 && std::abs(k) == 1) || gs.count(k) || fp_.count(k) || (sym == 0 && tp_.count(k)) ||
                                facCells_.count(cellKey(sym, k)) || (f == 0 && sdcCells_.count(cellKey(sym, k)));
                if (!no) {
                    mscTab[static_cast<std::size_t>((f * L + sym) * NK + k - kmin)] = 1;
                    ++perSuper;
                }
            }
    }
    const int NMUX = perSuper / 3;
    if (NMUX < 16) return;
    static const int r64[4][3] = {{0, 4, 9}, {2, 7, 10}, {4, 9, 11}, {7, 10, 12}};
    static const int r16[2][2] = {{2, 7}, {4, 9}};
    const int nl = q64 ? 3 : 2;
    int rates[3];
    for (int j = 0; j < nl; ++j) rates[j] = q64 ? r64[prot][j] : r16[prot][j];
    std::vector<int> ils[3];
    bool hasIl[3] = {false, false, false};
    if (q64) {
        ilTable(2 * NMUX, 13, ils[1]);
        ilTable(2 * NMUX, 21, ils[2]);
        hasIl[1] = hasIl[2] = true;
    } else {
        ilTable(2 * NMUX, 13, ils[0]);
        ilTable(2 * NMUX, 21, ils[1]);
        hasIl[0] = hasIl[1] = true;
    }
    Pattern pats[3];
    int Ms[3];
    for (int j = 0; j < nl; ++j) {
        Ms[j] = punct(rates[j])[0] * ((2 * NMUX - 12) / punct(rates[j])[1]);
        makePattern(rates[j], Ms[j], NMUX, false, pats[j]);
    }
    double amps[8];
    if (q64) {
        static const int a64[8] = {7, -1, 3, -5, 5, -3, 1, -7};
        for (int j = 0; j < 8; ++j) amps[j] = a64[j] / std::sqrt(42.0);
    } else {
        static const int a16[4] = {3, -1, 1, -3};
        for (int j = 0; j < 4; ++j) amps[j] = a16[j] / std::sqrt(10.0);
    }
    const int D = r.longIl ? 5 : 1;
    std::vector<int> cit;
    ilTable(NMUX, 5, cit);
    std::vector<std::complex<double> > memX(static_cast<std::size_t>(5 * NMUX));
    std::vector<double> memW(static_cast<std::size_t>(5 * NMUX));
    int idx[5] = {0, 1, 2, 3, 4};
    // each carrier's own noise, over the whole window: how far its cells land
    // from the nearest point.  Interference on a few carriers (a whistle, a
    // spur) is common on shortwave: weighting each carrier's soft values by its
    // reliability lets the error correction lean on the clean ones.
    const double u = q64 ? 1 / std::sqrt(42.0) : 1 / std::sqrt(10.0);
    const int top = q64 ? 7 : 3;
    std::vector<double> nEe(static_cast<std::size_t>(NK)), nEc(static_cast<std::size_t>(NK));
    // (each audio cell equalised once, here, and kept)
    const int nS = (nframes_ - 1) * L;
    if (nS <= 0) return;
    std::vector<std::complex<double> > eq(static_cast<std::size_t>(nS) * NK);
    std::vector<double> eqM(static_cast<std::size_t>(nS) * NK);
    for (int fr = fr0; fr < nframes_ - 1; ++fr)
        for (int sym = 0; sym < L; ++sym) {
            const int s = fr * L + sym, f = (fr - fr0) % 3, row = (f * L + sym) * NK;
            for (int k = kmin; k <= kmax; ++k) {
                if (mscTab[static_cast<std::size_t>(row + k - kmin)] != 1) continue;
                const std::complex<double> h(H2_[static_cast<std::size_t>(s) * W2_ + (k - kmin)]);
                double m = std::norm(h);
                if (m == 0) m = 1e-12;
                const std::complex<double> z = C(s, k) * std::conj(h) / m;
                const std::size_t ci0 = static_cast<std::size_t>(s) * NK + (k - kmin);
                eq[ci0] = z;
                eqM[ci0] = m;
                double nx = 2 * std::floor((z.real() / u - 1) / 2 + 0.5) + 1, ny = 2 * std::floor((z.imag() / u - 1) / 2 + 0.5) + 1;
                nx = std::max<double>(-top, std::min<double>(top, nx));
                ny = std::max<double>(-top, std::min<double>(top, ny));
                const double ex = z.real() - nx * u, ey = z.imag() - ny * u;
                nEe[static_cast<std::size_t>(k - kmin)] += (ex * ex + ey * ey) * m;
                nEc[static_cast<std::size_t>(k - kmin)] += m;
            }
        }
    double avgN = 0;
    int cntN = 0;
    for (int j = 0; j < NK; ++j)
        if (nEc[static_cast<std::size_t>(j)] != 0) {
            avgN += nEe[static_cast<std::size_t>(j)] / nEc[static_cast<std::size_t>(j)];
            ++cntN;
        }
    avgN = avgN / (cntN ? cntN : 1);
    if (!(avgN != 0)) avgN = 1;
    std::vector<double> relK(static_cast<std::size_t>(NK), 1.0);  // (1 for a typical carrier, less for a noisy one)
    for (int j = 0; j < NK; ++j)
        if (nEc[static_cast<std::size_t>(j)] != 0)
            relK[static_cast<std::size_t>(j)] = std::min(4.0, avgN / std::max(nEe[static_cast<std::size_t>(j)] / nEc[static_cast<std::size_t>(j)], avgN / 50));
    int warm = D - 1;
    std::vector<std::complex<double> > cc;
    std::vector<double> cw;
    std::vector<double> xv(static_cast<std::size_t>(2 * NMUX)), wv(static_cast<std::size_t>(2 * NMUX)), y(static_cast<std::size_t>(2 * NMUX));
    Bytes known[3], decs[3], code, pr;
    for (int sf = fr0; sf + 2 < nframes_ - 1; sf += 3) {
        const double at = static_cast<double>(winStart) + start_ + static_cast<double>(S0_ + sf * L) * Ts;
        const long long sfId = static_cast<long long>(std::floor(at / kSuperframeSamples + 0.5));
        cc.clear();
        cw.clear();
        for (int f = 0; f < 3; ++f)
            for (int sym = 0; sym < L; ++sym) {
                const int s = (sf + f) * L + sym;
                for (int k = kmin; k <= kmax; ++k)
                    if (mscTab[static_cast<std::size_t>((f * L + sym) * NK + k - kmin)] == 1) {
                        const std::size_t ci0 = static_cast<std::size_t>(s) * NK + (k - kmin);
                        cc.push_back(eq[ci0]);
                        cw.push_back(eqM[ci0] * relK[static_cast<std::size_t>(k - kmin)]);
                    }
            }
        for (int mf = 0; mf < 3; ++mf) {
            for (int i = 0; i < NMUX; ++i) {
                const std::size_t row = static_cast<std::size_t>(idx[i % D]), c = static_cast<std::size_t>(cit[static_cast<std::size_t>(i)]),
                                  j = static_cast<std::size_t>(mf * NMUX + i);
                memX[row * NMUX + c] = cc[j];
                memW[row * NMUX + c] = cw[j];
            }
            const std::size_t o = static_cast<std::size_t>(idx[D - 1]);
            const long long id = sfId * 3 + mf - (D - 1);
            for (int j = 0; j < D; ++j) idx[j] = (idx[j] - 1 + D) % D;
            if (warm > 0) {
                --warm;
                continue;
            }
            // (only frames not decoded before: with overlapping windows, most
            //  of each window's frames were decoded by the last one)
            if (id <= after) continue;
            for (int j = 0; j < NMUX; ++j) {
                xv[static_cast<std::size_t>(2 * j)] = memX[o * NMUX + j].real();
                xv[static_cast<std::size_t>(2 * j + 1)] = memX[o * NMUX + j].imag();
                wv[static_cast<std::size_t>(2 * j)] = wv[static_cast<std::size_t>(2 * j + 1)] = memW[o * NMUX + j];
            }
            // multistage decoding, iterated (as Dream): the first pass decodes
            // each level knowing only the levels before it; for a frame in
            // trouble (after the first pass, each level's decision re-encoded
            // and compared with the bits received), two more passes decode
            // every level again knowing all the others, so each bit is a
            // choice between just two points instead of four or eight
            bool have[3] = {false, false, false};
            double dis[3] = {0, 0, 0};
            int passes = 1;
            for (int j = 0; j < nl; ++j) known[j].assign(static_cast<std::size_t>(2 * NMUX), 0);
            for (int pass = 0; pass < passes; ++pass)
                for (int lv = 0; lv < nl; ++lv) {
                    for (int p = 0; p < 2 * NMUX; ++p) {
                        // the candidates: this level's bit 0 or 1; each other
                        // level fixed where it's known, free where it isn't yet
                        double d0 = std::numeric_limits<double>::infinity(), d1 = d0;
                        for (int ix = 0; ix < (1 << nl); ++ix) {
                            bool fits = true;
                            for (int q = 0; q < nl && fits; ++q)
                                if (q != lv && have[q] && (((ix >> (nl - 1 - q)) & 1) != known[q][static_cast<std::size_t>(p)])) fits = false;
                            if (!fits) continue;
                            const double e = xv[static_cast<std::size_t>(p)] - amps[ix], d = e * e;
                            if ((ix >> (nl - 1 - lv)) & 1) {
                                if (d < d1) d1 = d;
                            } else if (d < d0)
                                d0 = d;
                        }
                        const double v = (d1 - d0) * wv[static_cast<std::size_t>(p)];
                        y[static_cast<std::size_t>(hasIl[lv] ? ils[lv][static_cast<std::size_t>(p)] : p)] = v;
                    }
                    vit_.decode(&y[0], pats[lv], decs[lv]);
                    convEncode(decs[lv].empty() ? 0 : &decs[lv][0], decs[lv].size(), pats[lv], code);
                    if (static_cast<int>(code.size()) < 2 * NMUX) code.resize(static_cast<std::size_t>(2 * NMUX), 0);
                    for (int p = 0; p < 2 * NMUX; ++p)
                        known[lv][static_cast<std::size_t>(p)] = code[static_cast<std::size_t>(hasIl[lv] ? ils[lv][static_cast<std::size_t>(p)] : p)];
                    have[lv] = true;
                    if (pass == 0) {
                        int dd = 0, t = 0;
                        for (int p = 0; p < 2 * NMUX; ++p) {
                            const double yy = y[static_cast<std::size_t>(hasIl[lv] ? ils[lv][static_cast<std::size_t>(p)] : p)];
                            if (yy == 0) continue;
                            ++t;
                            if ((yy < 0 ? 1 : 0) != known[lv][static_cast<std::size_t>(p)]) ++dd;
                        }
                        dis[lv] = t ? static_cast<double>(dd) / t : 0;
                        // (clean frames: DeutschlandRadio ~11% / 3% / <1%)
                        if (lv == nl - 1 && nl > 1 && iterate && (dis[0] > 0.15 || dis[1] > 0.04 || (nl > 2 && dis[2] > 0.015))) passes = 3;
                    }
                }
            std::size_t total = 0;
            for (int lv = 0; lv < nl; ++lv) total += decs[lv].size();
            prbs(total, pr);
            MuxFrame mfo;
            mfo.id = id;
            mfo.all.assign(total >> 3, 0);
            std::size_t j = 0;
            for (int lv = 0; lv < nl; ++lv)
                for (std::size_t q = 0; q < decs[lv].size(); ++q, ++j)
                    if (j < 8 * mfo.all.size() && ((decs[lv][q] ^ pr[j]) & 1)) mfo.all[j >> 3] |= static_cast<unsigned char>(0x80 >> (j & 7));
            out.push_back(mfo);
        }
    }
}

}  // namespace drm
}  // namespace dec
