// ============================================================================
//  offgrid-dsp - reusable C++11 radio signal processing (part of OFFgrid-SDR
//  TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/SignalMonitor.cpp
//  Layer:  Library liboffgrid-dsp
//
//  What this file does
//  -------------------
//  ChannelExtractor, SignalMonitor and SignalAnalyzer (see SignalMonitor.h):
//  a line-by-line port of OFFgrid-SDR's monStep / monStats ("Signal Monitor
//  engine"), monFeed / monCsv / monCapFeed (the page around it) and
//  monAnalyse with monFft / monBand / monLine / monPct ("Signal Monitor:
//  deep analysis of one signal"), by diddy-boy, index.html 2.4/2.5.
//
//  How it fits in
//  --------------
//  Depends on Fft.h (the float FFT for the monitor's 10-a-second spectrum)
//  and the C++11 standard library.  AutoMode.cpp reuses ChannelExtractor
//  and detail::fftDouble from here.
//
//  Contributing
//  ------------
//  The comments in brackets are the JS's own reasons, kept with the code
//  they explain.  Keep the hot paths (feedIq, ChannelExtractor::process)
//  free of allocation; the analysis is a one-off and may allocate.
// ============================================================================
#include "SignalMonitor.h"

#include "Fft.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <limits>

namespace dsp {

namespace {

const double kPi = 3.14159265358979323846;
const double kLn2 = 0.69314718055994530942;
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kNegInf = -std::numeric_limits<double>::infinity();

bool isNum(double x) { return x == x && std::fabs(x) <= 1.7e308; }

// A Blackman-windowed sinc low-pass (fc in cycles per sample), unity gain:
// the JS's sincTaps.
void sincTaps(int T, double fc, std::vector<double>& h) {
    h.assign(T, 0.0);
    if (T == 1) {
        h[0] = 1;
        return;
    }
    const double M = (T - 1) / 2.0;
    double sum = 0;
    for (int j = 0; j < T; ++j) {
        const double n = j - M;
        const double w = 0.42 - 0.5 * std::cos(2 * kPi * j / (T - 1)) + 0.08 * std::cos(4 * kPi * j / (T - 1));
        h[j] = (n == 0 ? 2 * fc : std::sin(2 * kPi * fc * n) / (kPi * n)) * w;
        sum += h[j];
    }
    for (int j = 0; j < T; ++j) h[j] /= sum;
}

// p-th quantile (nearest rank) of a copy: the JS's monPct.
double pct(std::vector<double> a, double p) {
    if (a.empty()) return 0;
    std::sort(a.begin(), a.end());
    long i = static_cast<long>(std::floor(p * (a.size() - 1) + 0.5));
    if (i < 0) i = 0;
    if (i > static_cast<long>(a.size()) - 1) i = static_cast<long>(a.size()) - 1;
    return a[i];
}

int pow2AtLeast(std::size_t n) {
    int m = 1;
    while (static_cast<std::size_t>(m) < n) m <<= 1;
    return m;
}

double round1(double x) { return std::floor(x * 10 + 0.5) / 10; }
double round2(double x) { return std::floor(x * 100 + 0.5) / 100; }
long jsRound(double x) { return static_cast<long>(std::floor(x + 0.5)); }

std::string fmt(const char* f, double a) {
    char b[64];
    std::snprintf(b, sizeof b, f, a);
    return b;
}

}  // namespace

namespace detail {

void fftDouble(double* re, double* im, int n) {
    for (int i = 1, j = 0; i < n; ++i) {
        int b = n >> 1;
        for (; j & b; b >>= 1) j ^= b;
        j ^= b;
        if (i < j) {
            std::swap(re[i], re[j]);
            std::swap(im[i], im[j]);
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        const double a = -2 * kPi / len, wr = std::cos(a), wi = std::sin(a);
        const int h = len >> 1;
        for (int i = 0; i < n; i += len) {
            double cr = 1, ci = 0;
            for (int j = 0; j < h; ++j) {
                const int p = i + j, q = p + h;
                const double xr = re[q] * cr - im[q] * ci, xi = re[q] * ci + im[q] * cr;
                re[q] = re[p] - xr;
                im[q] = im[p] - xi;
                re[p] += xr;
                im[p] += xi;
                const double t = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = t;
            }
        }
    }
}

}  // namespace detail

// ===========================================================================
// ChannelExtractor
// ===========================================================================
ChannelExtractor::ChannelExtractor()
    : inRate_(1), offset_(0), D1_(1), D2_(1), T_(1), pr_(1), pi_(0), sr_(1), si_(0), ar_(0), ai_(0), cnt_(0), ph2_(0),
      hp_(0) {
    configure(1, 0, 1);
}

int ChannelExtractor::decimationFor(double inputRate, double wantRate) {
    if (wantRate <= 0) return 1;
    const long d = jsRound(inputRate / wantRate);
    return d < 1 ? 1 : static_cast<int>(d);
}

void ChannelExtractor::configure(double inputRate, double offsetHz, int decimation) {
    if (decimation < 1) decimation = 1;
    inRate_ = inputRate > 0 ? inputRate : 1;
    // (Streaming, in two steps: (1) average 8 samples (cheap; its nulls fall exactly where signals would fold back
    //  into the view), then (2) a proper windowed-sinc low-pass decimating the rest of the way. A plain average alone
    //  let strong signals outside the view fold in as phantoms)
    D1_ = decimation % 8 == 0 && decimation >= 16 ? 8 : 1;
    D2_ = decimation / D1_;
    T_ = D2_ > 1 ? 8 * D2_ + 1 : 1;
    sincTaps(T_, 0.42 / D2_, h_);
    rr_.assign(T_, 0.0);
    ri_.assign(T_, 0.0);
    setOffset(offsetHz);
    reset();
}

void ChannelExtractor::setOffset(double offsetHz) {
    offset_ = offsetHz;
    const double w = -2 * kPi * offsetHz / inRate_;
    sr_ = std::cos(w);
    si_ = std::sin(w);
}

void ChannelExtractor::reset() {
    pr_ = 1;
    pi_ = 0;
    ar_ = ai_ = 0;
    cnt_ = ph2_ = hp_ = 0;
    std::fill(rr_.begin(), rr_.end(), 0.0);
    std::fill(ri_.begin(), ri_.end(), 0.0);
}

std::size_t ChannelExtractor::process(const Complex* in, std::size_t n, std::vector<Complex>& out) {
    const std::size_t before = out.size();
    const double sr = sr_, si = si_, inv1 = 1.0 / D1_;
    double pr = pr_, pi = pi_, ar = ar_, ai = ai_;
    int cnt = cnt_;
    const int T = T_, D1 = D1_;
    for (std::size_t i = 0; i < n; ++i) {
        const double x = in[i].real(), y = in[i].imag();
        ar += x * pr - y * pi;
        ai += x * pi + y * pr;
        const double t = pr * sr - pi * si;
        pi = pr * si + pi * sr;
        pr = t;
        if (++cnt == D1) {
            cnt = 0;
            hp_ = hp_ + 1 == T ? 0 : hp_ + 1;
            rr_[hp_] = ar * inv1;
            ri_[hp_] = ai * inv1;
            ar = ai = 0;
            if (++ph2_ == D2_) {
                ph2_ = 0;
                double o_r = 0, o_i = 0;
                for (int j = 0, q = hp_; j < T; ++j) {
                    o_r += h_[j] * rr_[q];
                    o_i += h_[j] * ri_[q];
                    q = q == 0 ? T - 1 : q - 1;
                }
                out.push_back(Complex(static_cast<float>(o_r), static_cast<float>(o_i)));
            }
        }
    }
    const double m = std::sqrt(pr * pr + pi * pi);  // (keep the phasor on the unit circle)
    pr_ = m > 0 ? pr / m : 1;
    pi_ = m > 0 ? pi / m : 0;
    ar_ = ar;
    ai_ = ai;
    cnt_ = cnt;
    return out.size() - before;
}

// ===========================================================================
// SignalMonitor
// ===========================================================================
const double SignalMonitor::kOnDb = 6, SignalMonitor::kOffDb = 3, SignalMonitor::kOnSeconds = 0.2,
             SignalMonitor::kOffSeconds = 0.6, SignalMonitor::kFadeDb = 10, SignalMonitor::kFadeEndDb = 6;

// One watched signal and what monStep keeps for it (the JS's monNew + monFresh).
struct SignalMonitor::Sig {
    double f, bw;
    std::string name, mode;
    bool follow;
    // st
    bool hasT, hasT0;
    double t0, t, seen, lvl, lvlS, noise, snr, snrPk;
    bool on;
    double cand;
    bool hasOnSince;
    double onSince, onTime;
    std::deque<MonitorTx> txs;
    int nTx;
    std::deque<double> starts;
    std::deque<MonitorFade> fades;
    int nFade;
    double fadeDeep;
    bool inFade;
    double fadeA, fadeDeepNow, fadeRef;
    double ref, obw, fc, fpk;
    bool hasFc, hasFcFirst;
    double fcFirst;
    std::deque<std::pair<double, double> > drift;
    double lastDriftT;
    bool away;
    double lvlPk, lvlSum, lvlW, longest, binHz;
    bool hasRawUp, hasRawLast;
    double rawUp, rawLast;

    Sig(double f0, double bw0, const std::string& m, const std::string& n, bool fol)
        : f(f0), bw(std::max(50.0, bw0 > 0 ? bw0 : 3000.0)), name(n), mode(m), follow(fol) {
        fresh();
    }
    void fresh() {
        hasT = hasT0 = false;
        t0 = t = seen = 0;
        lvl = lvlS = noise = kNegInf;
        snr = snrPk = 0;
        on = false;
        cand = 0;
        hasOnSince = false;
        onSince = onTime = 0;
        txs.clear();
        nTx = 0;
        starts.clear();
        fades.clear();
        nFade = 0;
        fadeDeep = 0;
        inFade = false;
        fadeA = fadeDeepNow = fadeRef = 0;
        ref = obw = 0;
        fc = fpk = kNaN;
        hasFc = hasFcFirst = false;
        fcFirst = 0;
        drift.clear();
        lastDriftT = kNegInf;
        away = false;
        lvlPk = kNegInf;
        lvlSum = lvlW = longest = 0;
        binHz = 0;
        hasRawUp = hasRawLast = false;
        rawUp = rawLast = 0;
    }
    bool fadesOk() const { return !(mode == "USB" || mode == "LSB" || mode == "CW"); }
};

SignalMonitor::SignalMonitor()
    : ringPos_(0), ringFill_(0), N_(0), w2_(1), rate_(0), sinceFrame_(0), clock_(0), lastCentre_(kNaN), fft_(0) {}

SignalMonitor::~SignalMonitor() {
    clear();
    delete fft_;
}

int SignalMonitor::add(double centreHz, double widthHz, const std::string& mode, const std::string& name, bool follow) {
    if (count() >= kMaxSignals) return -1;
    sigs_.push_back(new Sig(centreHz, widthHz, mode, name, follow));
    return count() - 1;
}

void SignalMonitor::remove(int i) {
    if (i < 0 || i >= count()) return;
    delete sigs_[i];
    sigs_.erase(sigs_.begin() + i);
}

void SignalMonitor::clear() {
    for (std::size_t i = 0; i < sigs_.size(); ++i) delete sigs_[i];
    sigs_.clear();
}

void SignalMonitor::resetStats(int i) {
    if (i >= 0 && i < count()) sigs_[i]->fresh();
}
void SignalMonitor::setBand(int i, double f, double bw) {
    if (i < 0 || i >= count()) return;
    sigs_[i]->f = f;
    sigs_[i]->bw = std::max(50.0, bw);
}
void SignalMonitor::setFollow(int i, bool on) {
    if (i >= 0 && i < count()) sigs_[i]->follow = on;
}
void SignalMonitor::setMode(int i, const std::string& m) {
    if (i >= 0 && i < count()) sigs_[i]->mode = m;
}
void SignalMonitor::setName(int i, const std::string& n) {
    if (i >= 0 && i < count()) sigs_[i]->name = n;
}
double SignalMonitor::centreHz(int i) const { return i >= 0 && i < count() ? sigs_[i]->f : kNaN; }
double SignalMonitor::widthHz(int i) const { return i >= 0 && i < count() ? sigs_[i]->bw : kNaN; }
bool SignalMonitor::follow(int i) const { return i >= 0 && i < count() && sigs_[i]->follow; }
const std::string& SignalMonitor::mode(int i) const {
    static const std::string none;
    return i >= 0 && i < count() ? sigs_[i]->mode : none;
}
const std::string& SignalMonitor::name(int i) const {
    static const std::string none;
    return i >= 0 && i < count() ? sigs_[i]->name : none;
}
const std::deque<MonitorTx>& SignalMonitor::transmissions(int i) const {
    static const std::deque<MonitorTx> none;
    return i >= 0 && i < count() ? sigs_[i]->txs : none;
}
const std::deque<MonitorFade>& SignalMonitor::fades(int i) const {
    static const std::deque<MonitorFade> none;
    return i >= 0 && i < count() ? sigs_[i]->fades : none;
}

void SignalMonitor::resumeAfterGap(double gap) {
    for (std::size_t i = 0; i < sigs_.size(); ++i) {
        Sig& st = *sigs_[i];
        st.hasT = false;  // (the next step counts nothing)
        if (st.on && st.hasOnSince) st.onSince += gap;
        if (st.hasRawUp) st.rawUp += gap;
        if (st.hasRawLast) st.rawLast += gap;
    }
}

// The monitor's own spectrum (the JS's monFeed): about 100 Hz bins whatever the
// zoom, 10 a second, the noise in one bin from the median over the middle 90 %.
void SignalMonitor::feedIq(const Complex* iq, std::size_t n, double rate, double centreHz) {
    if (n == 0 || rate <= 0) return;
    int N = static_cast<int>(std::min(65536.0, std::max(4096.0, std::pow(2.0, std::ceil(std::log(rate / 100) / std::log(2.0))))));
    if (N != N_ || rate != rate_) {
        N_ = N;
        rate_ = rate;
        ring_.assign(N, Complex(0, 0));
        work_.assign(N, Complex(0, 0));
        win_.resize(N);
        db_.resize(N);
        double t = 0;
        for (int i = 0; i < N; ++i) {
            win_[i] = static_cast<float>(0.5 - 0.5 * std::cos(2 * kPi * i / N));
            t += static_cast<double>(win_[i]) * win_[i];
        }
        w2_ = t / N;
        delete fft_;
        fft_ = new detail::Fft(N);
        smp_.reserve(1100);
        ringPos_ = ringFill_ = 0;
    }
    // (Addition to the JS: a retune empties the buffer, so no frame mixes two tunings)
    if (!(centreHz == lastCentre_)) {
        lastCentre_ = centreHz;
        ringPos_ = ringFill_ = 0;
    }
    for (std::size_t i = 0; i < n; ++i) {
        ring_[ringPos_] = iq[i];
        ringPos_ = ringPos_ + 1 == static_cast<std::size_t>(N) ? 0 : ringPos_ + 1;
    }
    ringFill_ = std::min<std::size_t>(N, ringFill_ + n);
    const double dt = n / rate;
    clock_ += dt;
    sinceFrame_ += dt;
    if (sinceFrame_ < 0.1) return;
    sinceFrame_ = sinceFrame_ >= 0.2 ? 0 : sinceFrame_ - 0.1;
    if (ringFill_ < static_cast<std::size_t>(N) || sigs_.empty()) return;
    for (int i = 0; i < N; ++i) {
        const std::size_t k = (ringPos_ + i) % N;
        work_[i] = ring_[k] * win_[i];
    }
    fft_->forward(&work_[0]);
    const double inv = 1.0 / (static_cast<double>(N) * N);
    for (int k = 0; k < N; ++k) {
        const double p = (static_cast<double>(work_[k].real()) * work_[k].real() +
                          static_cast<double>(work_[k].imag()) * work_[k].imag()) * inv;
        db_[k] = static_cast<float>(10 * std::log10(p + 1e-30));
    }
    // the noise in one bin: the median bin over the middle 90% of what the radio sees (as the S-meter's), from 1,024 of them
    smp_.clear();
    const int stp = std::max(1, N / 1024), h = N / 2;
    for (int j = static_cast<int>(std::floor(N * 0.05)); j < N * 0.95; j += stp) smp_.push_back(db_[(j + h) % N]);
    std::nth_element(smp_.begin(), smp_.begin() + smp_.size() / 2, smp_.end());
    MonitorFrame fr;
    fr.db = &db_[0];
    fr.N = N;
    fr.binHz = rate / N;
    fr.centreHz = centreHz;
    fr.perBin = std::pow(10.0, smp_[smp_.size() / 2] / 10.0) / kLn2;
    fr.w2 = w2_;
    step(fr, clock_);
}

void SignalMonitor::step(const MonitorFrame& fr, double t) {
    if (!fr.db || fr.N < 8) return;
    for (std::size_t i = 0; i < sigs_.size(); ++i) stepOne(*sigs_[i], t, fr);
}

// one look (the JS's monStep)
void SignalMonitor::stepOne(Sig& st, double t, const MonitorFrame& fr) {
    const float* db = fr.db;
    const int N = fr.N, h = N / 2;
    const double binHz = fr.binHz, c = fr.centreHz, w2 = fr.w2;
    st.binHz = binHz;
    const double dt = !st.hasT ? 0 : std::max(0.0, std::min(2.0, t - st.t));
    st.t = t;
    st.hasT = true;
    if (!st.hasT0) {
        st.t0 = t;
        st.hasT0 = true;
    }
    // (outside what the radio can see now: paused, not counted as off)
    const long j0 = jsRound((st.f - st.bw / 2 - c) / binHz + h), j1 = jsRound((st.f + st.bw / 2 - c) / binHz + h);
    // (time out of view isn't counted: a transmission's length and the duty cycle are of the time it could be seen)
    if (j0 < N * 0.04 || j1 > N * 0.96 || j1 <= j0) {
        st.away = true;
        st.cand = 0;
        if (st.on && st.hasOnSince) st.onSince += dt;
        return;
    }
    st.away = false;
    st.seen += dt;
    double p = 0, pk = kNegInf, sw = 0, sp = 0, cw = 0;
    long jpk = j0;
    const double nb = fr.perBin;
    ex_.resize(j1 - j0 + 1);
    for (long j = j0; j <= j1; ++j) {
        const double v = db[(j + h) % N], lin = std::pow(10.0, v / 10);
        p += lin;
        if (v > pk) {
            pk = v;
            jpk = j;
        }
        const double e = std::max(0.0, lin - nb);
        ex_[j - j0] = e;
        sw += e;
    }
    // (the centre from the bins within 20 dB of the peak only: the noise in the rest of the band would pull it about)
    {
        const double lim = std::pow(10.0, pk / 10) * 0.01;
        for (std::size_t k = 0; k < ex_.size(); ++k)
            if (ex_[k] >= lim) {
                cw += ex_[k];
                sp += ex_[k] * (j0 + static_cast<long>(k));
            }
    }
    const double n = (j1 - j0 + 1) * nb, a = dt ? 1 - std::exp(-dt / 0.25) : 1;
    const double lvl = 10 * std::log10(p / w2), snr = std::max(0.0, 10 * std::log10(std::max(p / n - 1, 1e-3)));
    st.lvl = isNum(st.lvl) ? st.lvl + a * (lvl - st.lvl) : lvl;
    st.lvlS = isNum(st.lvlS) ? st.lvlS + (dt ? 1 - std::exp(-dt / 0.5) : 1) * (lvl - st.lvlS) : lvl;
    const bool fadesOk = st.fadesOk();
    st.noise = 10 * std::log10(n / w2);
    st.snr += a * (snr - st.snr);
    // (the unsmoothed reading's own edges: when it last rose above the "off" level, and the last moment it was above
    //  it. A transmission is timed by these, its start and end decided on the smoothed reading, which is steadier)
    if (snr >= kOffDb) {
        if (!st.hasRawUp) {
            st.rawUp = t;
            st.hasRawUp = true;
        }
        st.rawLast = t;
        st.hasRawLast = true;
    } else
        st.hasRawUp = false;
    // transmissions, with the hysteresis and the holds above
    const bool want = st.on ? st.snr >= kOffDb : st.snr >= kOnDb;
    if (want == st.on)
        st.cand = 0;
    else {
        st.cand += dt;
        if (st.cand >= (st.on ? kOffSeconds : kOnSeconds)) {
            st.cand = 0;
            st.on = want;
            // (from 2.4: timed from when the signal appeared to when it went (rawUp, rawLast), not when that was
            //  decided; never from before the watching began: the occupancy read 101%)
            if (want) {
                const double from = std::max(st.hasRawUp && st.rawUp <= t ? st.rawUp : t - kOnSeconds, t - st.seen);
                st.onTime += t - from;
                st.onSince = from;
                st.hasOnSince = true;
                st.nTx++;
                st.starts.push_back(t);
                if (st.starts.size() > 100) st.starts.pop_front();
                st.snrPk = st.snr;
                st.ref = st.lvlS;
            } else {
                const double end = st.hasRawLast && st.rawLast >= st.onSince && st.rawLast <= t ? st.rawLast + 0.05
                                                                                                 : t - kOffSeconds;
                st.onTime = std::max(0.0, st.onTime - (t - end));
                // (a flicker: under a quarter of a second of signal isn't a transmission. Taken back: not counted)
                if (end - st.onSince < 0.25) {
                    st.onTime = std::max(0.0, st.onTime - std::max(0.0, end - st.onSince));
                    st.nTx = std::max(0, st.nTx - 1);
                    if (!st.starts.empty()) st.starts.pop_back();
                } else {
                    st.longest = std::max(st.longest, end - st.onSince);
                    MonitorTx tx = {st.onSince, end, st.snrPk};
                    st.txs.push_back(tx);
                    if (st.txs.size() > 200) st.txs.pop_front();
                }
                st.hasOnSince = false;
                st.inFade = false;
            }
        }
    }
    if (st.on) {
        st.onTime += dt;
        st.snrPk = std::max(st.snrPk, st.snr);
        // (from 2.4: the average level over the time it's on (as power, not dB), and the peak)
        if (dt) {
            st.lvlSum += std::pow(10.0, st.lvl / 10) * dt;
            st.lvlW += dt;
        }
        if (isNum(st.lvl)) st.lvlPk = std::max(st.lvlPk, st.lvl);
        // fades: against the usual level, followed slowly (and not dragged down by the fade itself)
        if (!fadesOk)
            st.inFade = false;
        else if (!st.inFade) {
            st.ref += (dt ? 1 - std::exp(-dt / 4) : 0) * (st.lvlS - st.ref);
            if (st.lvlS < st.ref - kFadeDb) {
                st.inFade = true;
                st.fadeA = t;
                st.fadeDeepNow = st.ref - st.lvlS;
                st.fadeRef = st.ref;
            }
        } else {
            st.fadeDeepNow = std::max(st.fadeDeepNow, st.fadeRef - st.lvlS);
            if (st.lvlS > st.fadeRef - kFadeEndDb) {
                MonitorFade f = {st.fadeA, t, st.fadeDeepNow};
                st.fades.push_back(f);
                if (st.fades.size() > 200) st.fades.pop_front();
                st.nFade++;
                st.fadeDeep = std::max(st.fadeDeep, f.deep);
                st.inFade = false;
            }
        }
        // the occupied bandwidth (99%) and the centre, from the power above the noise
        if (sw > 0 && st.snr >= kOnDb) {
            double acc = 0;
            long lo = j0, hi = j1;
            for (std::size_t k = 0; k < ex_.size(); ++k) {
                acc += ex_[k];
                if (acc >= 0.005 * sw) {
                    lo = j0 + static_cast<long>(k);
                    break;
                }
            }
            acc = 0;
            for (long k = static_cast<long>(ex_.size()) - 1; k >= 0; --k) {
                acc += ex_[k];
                if (acc >= 0.005 * sw) {
                    hi = j0 + k;
                    break;
                }
            }
            const double obw = (hi - lo + 1) * binHz;
            st.obw = st.obw ? st.obw + 0.1 * (obw - st.obw) : obw;
            // (only when there's power to weigh: between a keyed signal's dots there can be none)
            const double fc = c + (sp / cw - h) * binHz;
            if (cw > 0 && isNum(fc)) st.fc = !st.hasFc || !isNum(st.fc) ? fc : st.fc + a * (fc - st.fc);
            if (cw > 0 && isNum(fc)) st.hasFc = true;
            // the peak, to a fraction of a bin (a parabola through the top three)
            const double y0 = db[((jpk - 1 + h) % N + N) % N], y1 = pk, y2 = db[(jpk + 1 + h) % N], d = y0 - 2 * y1 + y2;
            st.fpk = c + (jpk - h + (d < 0 ? 0.5 * (y0 - y2) / d : 0)) * binHz;
            // (from 2.4: every half second, for the drift over the last 20 s too (a satellite's Doppler changes by
            //  the second); and the first frequency seen, for the whole shift since)
            if (st.hasFc && !st.inFade && t - st.lastDriftT >= 0.5) {
                st.lastDriftT = t;
                st.drift.push_back(std::make_pair(t, st.fc));
                if (!st.hasFcFirst) {
                    st.fcFirst = st.fc;
                    st.hasFcFirst = true;
                }
                while (!st.drift.empty() && st.drift.front().first < t - 600) st.drift.pop_front();
            }
            // (catching up over about 1 s: a satellite near its closest approach moves some 50 Hz a second at
            //  145 MHz, more higher up)
            if (st.follow && dt && st.hasFc) {
                const double k = 1 - std::exp(-dt / 1), m = st.f + k * (st.fc - st.f);
                if (std::fabs(st.fc - st.f) < st.bw / 2) st.f = m;
            }
        }
    }
}

namespace {
// A straight line through (x, y) points: slope b, or NaN; *se gets its standard error.
double fitSlope(const std::deque<std::pair<double, double> >& D, std::size_t from, double* se) {
    const std::size_t n = D.size() - from;
    const double t0 = D[from].first, y0 = D[from].second;
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (std::size_t i = from; i < D.size(); ++i) {
        const double u = D[i].first - t0, v = D[i].second - y0;
        sx += u;
        sy += v;
        sxx += u * u;
        sxy += u * v;
    }
    const double den = n * sxx - sx * sx;
    if (den <= 0) return kNaN;
    const double b = (n * sxy - sx * sy) / den, a0 = (sy - b * sx) / n;
    double rss = 0;
    for (std::size_t i = from; i < D.size(); ++i) {
        const double r = D[i].second - y0 - a0 - b * (D[i].first - t0);
        rss += r * r;
    }
    *se = std::sqrt(rss / std::max<double>(1, static_cast<double>(n) - 2) / std::max(1e-9, sxx - sx * sx / n));
    return b;
}
}  // namespace

MonitorStats SignalMonitor::stats(int i) const {
    if (i < 0 || i >= count()) {
        MonitorStats z = MonitorStats();
        return z;
    }
    return statsOf(*sigs_[i], sigs_[i]->t);
}

// what the panel shows (the JS's monStats)
MonitorStats SignalMonitor::statsOf(const Sig& st, double t) const {
    MonitorStats r;
    const double el = std::max(1e-9, st.seen);
    double mean = kNaN, sd = kNaN;
    const std::size_t ni = st.starts.size() > 1 ? st.starts.size() - 1 : 0;
    if (ni) {
        double s = 0;
        for (std::size_t k = 1; k < st.starts.size(); ++k) s += st.starts[k] - st.starts[k - 1];
        mean = s / ni;
        if (ni > 1) {
            double v = 0;
            for (std::size_t k = 1; k < st.starts.size(); ++k) {
                const double d = st.starts[k] - st.starts[k - 1] - mean;
                v += d * d;
            }
            sd = std::sqrt(v / (ni - 1));
        }
    }
    // (the drift now: a straight line through the last 20 s, in Hz a second, as for a satellite's Doppler; only a
    //  clear trend, 3 times its standard error, and 0.5 Hz a second or more)
    double rate = kNaN;
    {
        std::size_t from = 0;
        while (from < st.drift.size() && st.drift[from].first < t - 20) ++from;
        if (st.drift.size() - from >= 8 && st.drift.back().first - st.drift[from].first >= 8) {
            double se = 0;
            const double b = fitSlope(st.drift, from, &se);
            if (isNum(b)) rate = std::fabs(b) >= 3 * se && std::fabs(b) >= 0.5 ? b : 0;
        }
    }
    double slope = kNaN;
    if (st.drift.size() >= 10 && st.drift.back().first - st.drift.front().first >= 20) {
        // (only a real trend: the slope at least 3 times its standard error, or a noise-like signal's wandering
        //  centre would read as drift; and steadier than 1 Hz a minute reads as none)
        double se = 0;
        const double b = fitSlope(st.drift, 0, &se);
        if (isNum(b)) slope = std::fabs(b) >= 3 * se && std::fabs(b * 60) >= 1 ? b * 60 : 0;
    }
    double dsum = 0;
    std::size_t dn = 0;
    for (std::size_t k = 0; k < st.txs.size(); ++k) {
        dsum += st.txs[k].b - st.txs[k].a;
        ++dn;
    }
    const bool cur = st.on && st.hasOnSince;
    if (cur) {
        dsum += t - st.onSince;
        ++dn;
    }
    r.on = st.on;
    r.away = st.away;
    r.level = st.lvl;
    r.noise = st.noise;
    r.snr = st.snr;
    r.nTx = st.nTx;
    r.duty = std::min(1.0, st.onTime / el);
    r.levelAvg = st.lvlW > 0 ? 10 * std::log10(st.lvlSum / st.lvlW) : kNaN;
    r.levelPeak = isNum(st.lvlPk) ? st.lvlPk : kNaN;
    r.active = std::min(st.onTime, st.seen);
    r.silent = std::max(0.0, st.seen - st.onTime);
    r.longest = std::min(std::max(st.longest, cur ? t - st.onSince : 0.0), st.seen + 0.05);
    r.driftNow = rate;
    r.shift = st.hasFcFirst && st.hasFc ? st.fc - st.fcFirst : kNaN;
    r.txAvg = dn ? dsum / dn : kNaN;
    r.txNow = cur ? t - st.onSince : kNaN;
    r.interval = mean;
    r.regular = ni >= 3 && mean > 0 && sd / mean < 0.15;
    // (a fade under way counts once it has outlasted the "off" hold: before that it may be the transmission ending)
    const bool fadeNow = st.inFade && st.on && t - st.fadeA > kOffSeconds;
    r.nFade = st.nFade + (fadeNow ? 1 : 0);
    r.fadeDeep = std::max(st.fadeDeep, st.inFade && t - st.fadeA > kOffSeconds ? st.fadeDeepNow : 0.0);
    r.fadesOk = st.fadesOk();
    r.fadeRate = st.onTime > 30 ? st.nFade / (st.onTime / 60) : kNaN;
    r.obw = st.obw ? st.obw : kNaN;
    r.obwRes = st.binHz ? 4 * st.binHz : kNaN;
    r.fc = st.hasFc ? st.fc : kNaN;
    r.fpk = st.fpk;
    r.driftPerMin = slope;
    r.watched = st.seen;
    return r;
}

std::string SignalMonitor::isoUtc(double unix) {
    const double fl = std::floor(unix);
    std::time_t tt = static_cast<std::time_t>(fl);
    std::tm g;
#if defined(_WIN32)
    gmtime_s(&g, &tt);
#else
    gmtime_r(&tt, &g);
#endif
    const int ms = static_cast<int>(std::floor((unix - fl) * 1000 + 0.5));
    char b[48];
    // (the JS's toISOString with ".000Z" shortened to "Z")
    if (ms == 0 || ms == 1000)
        std::snprintf(b, sizeof b, "%04d-%02d-%02dT%02d:%02d:%02dZ", g.tm_year + 1900, g.tm_mon + 1, g.tm_mday, g.tm_hour,
                      g.tm_min, g.tm_sec);
    else
        std::snprintf(b, sizeof b, "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ", g.tm_year + 1900, g.tm_mon + 1, g.tm_mday,
                      g.tm_hour, g.tm_min, g.tm_sec, ms);
    return b;
}

namespace {
std::string csvQuote(const std::string& v) {
    if (v.find_first_of("\",\n") == std::string::npos) return v;
    std::string o = "\"";
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (v[i] == '"') o += '"';
        o += v[i];
    }
    return o + "\"";
}
}  // namespace

// (from 2.4: the transmission log as CSV: every watched signal's transmissions, oldest first, times in UTC; a
//  transmission still under way has no end yet)
std::string SignalMonitor::csv(double unixAtZero) const {
    std::string out = "signal,name,frequency_hz,mode,start_utc,end_utc,duration_s,peak_snr_db\r\n";
    char b[64];
    for (std::size_t i = 0; i < sigs_.size(); ++i) {
        const Sig& s = *sigs_[i];
        std::vector<MonitorTx> txs(s.txs.begin(), s.txs.end());
        const std::size_t done = txs.size();
        if (s.on && s.hasOnSince) {
            MonitorTx x = {s.onSince, s.t, s.snrPk};
            txs.push_back(x);
        }
        for (std::size_t k = 0; k < txs.size(); ++k) {
            const MonitorTx& x = txs[k];
            std::snprintf(b, sizeof b, "%d,", static_cast<int>(i + 1));
            out += b;
            out += csvQuote(s.name) + ",";
            std::snprintf(b, sizeof b, "%ld,", jsRound(s.f));
            out += b;
            out += s.mode + ",";
            out += isoUtc(unixAtZero + x.a) + ",";
            out += (k < done ? isoUtc(unixAtZero + x.b) : std::string()) + ",";
            std::snprintf(b, sizeof b, "%.1f,%.1f\r\n", x.b - x.a, x.peakSnr);
            out += b;
        }
    }
    return out;
}

// ===========================================================================
// SignalAnalyzer
// ===========================================================================
AnalysisResult::AnalysisResult()
    : fs(0), secs(0), obw(kNaN), lo(kNaN), hi(kNaN), centre(kNaN), bandDb(kNaN), snr(0), swingDb(kNaN), offFrac(kNaN),
      cv(kNaN), cvFast(kNaN), toneCorr(kNaN), reversals(kNaN), revDips(0), shift(kNaN), baud(kNaN), wpm(kNaN),
      tone(kNaN), carrier(kNaN), deviation(kNaN), fskLo(kNaN), fskHi(kNaN), stop15(false) {}

SignalAnalyzer::SignalAnalyzer()
    : active_(false), full_(false), f_(0), bw_(0), rate_(0), fs_(0), hw_(kNaN), D_(1), k_(0), n_(0), N_(0), ph_(0),
      ar_(0), ai_(0) {}

bool SignalAnalyzer::start(double f, double bw, double rate, double seconds) {
    cancel();
    if (bw > 40e3 || rate <= 0) return false;  // (40 kHz at most: narrow it by dragging an edge)
    const double want = std::max(4000.0, 2.5 * bw);
    D_ = std::max(1, static_cast<int>(std::floor(rate / want)));
    rate_ = rate;
    fs_ = rate / D_;
    f_ = f;
    bw_ = bw;
    N_ = static_cast<std::size_t>(jsRound(fs_ * seconds));
    zr_.assign(N_, 0.0f);
    zi_.assign(N_, 0.0f);
    n_ = 0;
    k_ = 0;
    ph_ = ar_ = ai_ = 0;
    hw_ = kNaN;
    active_ = true;
    full_ = false;
    return true;
}

void SignalAnalyzer::cancel() {
    active_ = full_ = false;
    n_ = 0;
}

double SignalAnalyzer::progress() const { return N_ ? static_cast<double>(n_) / N_ : 0; }

bool SignalAnalyzer::feedIq(const Complex* iq, std::size_t n, double centreHz) {
    if (!active_ || full_) return full_;
    // (started again if the radio retunes partway: the band moved in the samples)
    if (!(centreHz == hw_)) {
        hw_ = centreHz;
        n_ = 0;
        k_ = 0;
        ar_ = ai_ = ph_ = 0;
    }
    const double w = -2 * kPi * (f_ - hw_) / rate_, wr = std::cos(w), wi = std::sin(w);
    double cr = std::cos(ph_), ci = std::sin(ph_), ar = ar_, ai = ai_;
    int k = k_;
    std::size_t m = n_;
    for (std::size_t i = 0; i < n && m < N_; ++i) {
        const double x = iq[i].real(), y = iq[i].imag();
        ar += x * cr - y * ci;
        ai += x * ci + y * cr;
        const double t = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = t;
        if (++k == D_) {
            zr_[m] = static_cast<float>(ar / D_);
            zi_[m] = static_cast<float>(ai / D_);
            ++m;
            ar = ai = 0;
            k = 0;
        }
    }
    ph_ = std::atan2(ci, cr);
    ar_ = ar;
    ai_ = ai;
    k_ = k;
    n_ = m;
    if (n_ >= N_) full_ = true;
    return full_;
}

AnalysisResult SignalAnalyzer::result() const {
    if (!full_) {
        std::vector<float> zr(zr_.begin(), zr_.begin() + n_), zi(zi_.begin(), zi_.begin() + n_);
        return analyse(zr, zi, fs_, bw_);
    }
    return analyse(zr_, zi_, fs_, bw_);
}

namespace {

// keep only f1..f2 Hz of a baseband signal (a cosine taper at each edge), by FFT (the JS's monBand)
void monBand(std::vector<double>& zr, std::vector<double>& zi, double fs, double f1, double f2) {
    const std::size_t N = zr.size();
    const int n2 = pow2AtLeast(N);
    std::vector<double> re(n2, 0.0), im(n2, 0.0);
    std::copy(zr.begin(), zr.end(), re.begin());
    std::copy(zi.begin(), zi.end(), im.begin());
    detail::fftDouble(&re[0], &im[0], n2);
    const double tp = std::max(5.0, (f2 - f1) * 0.05);
    for (int k = 0; k < n2; ++k) {
        const double f = (k < n2 / 2 ? k : k - n2) * fs / n2;
        const double g = f >= f1 && f <= f2 ? 1
                         : f < f1 - tp || f > f2 + tp
                             ? 0
                             : 0.5 + 0.5 * std::cos(kPi * (f < f1 ? f1 - f : f - f2) / tp);
        re[k] *= g;
        im[k] *= g;
    }
    for (int k = 0; k < n2; ++k) im[k] = -im[k];
    detail::fftDouble(&re[0], &im[0], n2);
    for (std::size_t i = 0; i < N; ++i) {
        zr[i] = static_cast<float>(re[i] / n2);
        zi[i] = static_cast<float>(-im[i] / n2);
    }
}

// the strongest line in x's spectrum between lo and hi Hz (x sampled at fs): its frequency, and how far it stands
// above the median of that range, in dB. Refined by a parabola through the top three bins (the JS's monLine)
std::pair<double, double> monLine(const std::vector<double>& x, double fs, double lo, double hi) {
    int n = pow2AtLeast(x.size());
    n = std::min(n, 1 << 18);
    std::vector<double> re(n, 0.0), im(n, 0.0);
    const std::size_t M = std::min<std::size_t>(n, x.size());
    double m = 0;
    for (std::size_t i = 0; i < M; ++i) m += x[i];
    m /= M;
    for (std::size_t i = 0; i < M; ++i) re[i] = (x[i] - m) * (0.5 - 0.5 * std::cos(2 * kPi * i / x.size()));
    detail::fftDouble(&re[0], &im[0], n);
    const int b0 = std::max(1, static_cast<int>(std::floor(lo / fs * n)));
    const int b1 = std::min(n / 2 - 2, static_cast<int>(std::ceil(hi / fs * n)));
    if (b1 <= b0 + 2) return std::make_pair(0.0, 0.0);
    std::vector<double> p(b1 - b0 + 1);
    int best = b0;
    for (int b = b0; b <= b1; ++b) {
        p[b - b0] = re[b] * re[b] + im[b] * im[b];
        if (p[b - b0] > p[best - b0]) best = b;
    }
    double med = pct(p, 0.5);
    if (!med) med = 1e-30;
    const int L = static_cast<int>(p.size());
    const double y0 = std::log(p[std::max(0, best - b0 - 1)] + 1e-30), y1 = std::log(p[best - b0] + 1e-30),
                 y2 = std::log(p[std::min(L - 1, best - b0 + 1)] + 1e-30), d = y0 - 2 * y1 + y2;
    return std::make_pair((best + (d < 0 ? 0.5 * (y0 - y2) / d : 0)) * fs / n, 10 * std::log10(p[best - b0] / med));
}

// standard symbol rates, and the modes each FSK shift and rate (or PSK rate) goes with
const double kBauds[] = {45.45, 50, 75, 100, 110, 150, 200, 300, 600, 1200};
struct FskStd {
    double shift, baud;
    const char* name;
};
const FskStd kFsk[] = {{170, 45.45, "RTTY (amateur, 45.45 baud)"},
                       {170, 50, "RTTY, 50 baud"},
                       {170, 75, "RTTY, 75 baud"},
                       {170, 100, "SITOR-B / NAVTEX, or DSC (100 baud)"},
                       {425, 50, "RTTY, 50 baud, 425 Hz shift"},
                       {450, 50, "RTTY, 50 baud, 450 Hz shift (weather)"},
                       {850, 50, "RTTY, 50 baud, 850 Hz shift (weather, press)"},
                       {850, 75, "RTTY, 75 baud, 850 Hz shift"},
                       {1000, 1200, "AFSK 1200 (packet, APRS)"}};
const int kNFsk = sizeof kFsk / sizeof kFsk[0];
struct PskStd {
    double rate;
    const char* name;
};
const PskStd kPsk[] = {{31.25, "PSK31"}, {62.5, "PSK63"}, {125, "PSK125"}, {250, "PSK250"}};

// NaN = null
double snapBaud(double b, bool* ok) {
    *ok = isNum(b) && b != 0;
    if (!*ok) return kNaN;
    for (std::size_t i = 0; i < sizeof kBauds / sizeof kBauds[0]; ++i)
        if (std::fabs(b - kBauds[i]) / kBauds[i] < 0.03) return kBauds[i];
    return std::floor(b * 10 + 0.5) / 10;
}
bool fskMatch(const FskStd& s, double shift, double baud) {
    return std::fabs(shift - s.shift) <= std::max(15.0, s.shift * 0.06) && isNum(baud) && baud &&
           std::fabs(baud - s.baud) / s.baud < 0.03;
}

std::string hzOrKhz(double obw) {
    if (obw >= 1000) return fmt("%.1f kHz", obw / 1000);
    return fmt("%.0f Hz", static_cast<double>(jsRound(obw)));
}

}  // namespace

AnalysisResult SignalAnalyzer::analyse(const std::vector<float>& zrIn, const std::vector<float>& ziIn, double fs,
                                       double bw) {
    AnalysisResult out;
    const std::size_t N = std::min(zrIn.size(), ziIn.size());
    out.fs = fs;
    out.secs = N / fs;
    if (N < fs) {
        out.kind = "short";
        out.name = "too short to tell";
        out.text = "Less than a second of signal";
        return out;
    }
    std::vector<double> zr(zrIn.begin(), zrIn.begin() + N), zi(ziIn.begin(), ziIn.begin() + N);
    // 0. the noise just outside the band (the capture is wider than the band), then everything outside the band
    //  removed: whatever else is in the capture would swamp the keying and the tone changes
    double outsideFloor = kNaN;
    {
        const int n2 = pow2AtLeast(N);
        std::vector<double> re(n2, 0.0), im(n2, 0.0), outs;
        std::copy(zr.begin(), zr.end(), re.begin());
        std::copy(zi.begin(), zi.end(), im.begin());
        detail::fftDouble(&re[0], &im[0], n2);
        const double binF = fs / n2;
        for (int k = 0; k < n2; ++k) {
            const double f = std::abs(k < n2 / 2 ? k : k - n2) * binF;
            if (f > bw / 2 * 1.15 && f < fs / 2 * 0.85) outs.push_back(re[k] * re[k] + im[k] * im[k]);
        }
        // (the noise's variance a sample: an exponential's median is ln 2 of its mean)
        if (outs.size() > 50) outsideFloor = pct(outs, 0.5) / kLn2 / n2;
        for (int k = 0; k < n2; ++k) {
            const double f = std::abs(k < n2 / 2 ? k : k - n2) * binF;
            const double g = f <= bw / 2 ? 1 : f >= bw / 2 * 1.1 ? 0 : 0.5 + 0.5 * std::cos(kPi * (f - bw / 2) / (bw * 0.05));
            re[k] *= g;
            im[k] *= g;
        }
        for (int k = 0; k < n2; ++k) im[k] = -im[k];
        detail::fftDouble(&re[0], &im[0], n2);
        for (std::size_t i = 0; i < N; ++i) {
            zr[i] = static_cast<float>(re[i] / n2);
            zi[i] = static_cast<float>(-im[i] / n2);
        }
    }
    // 1. the spectrum: averaged (Welch), about 4-8 Hz a bin, within the band
    int L = 1;
    while (L * 2 <= std::min(N / 4.0, fs / 3)) L <<= 1;
    std::vector<double> psd(L, 0.0), w(L), re(L), im(L);
    for (int i = 0; i < L; ++i) w[i] = 0.5 - 0.5 * std::cos(2 * kPi * i / L);
    int segs = 0;
    for (std::size_t o = 0; o + L <= N; o += L >> 1) {
        for (int i = 0; i < L; ++i) {
            re[i] = zr[o + i] * w[i];
            im[i] = zi[o + i] * w[i];
        }
        detail::fftDouble(&re[0], &im[0], L);
        for (int i = 0; i < L; ++i) psd[i] += re[i] * re[i] + im[i] * im[i];
        segs++;
    }
    const double binHz = fs / L;
#define F_OF(k) (((k) < L / 2 ? (k) : (k) - L) * binHz)
#define IN_BAND(k) (std::fabs(F_OF(k)) <= bw / 2)
    std::vector<double> band;
    for (int k = 0; k < L; ++k)
        if (IN_BAND(k)) band.push_back(psd[k] / segs);
    // (the floor: the noise outside the band where there's room to see it, as a signal can fill the band; else the
    //  band's quietest fifth. A Hann-windowed bin of white noise of variance v reads v times the window's energy, 3L/8)
    const double perBinOut = isNum(outsideFloor) ? outsideFloor * 3 * L / 8 : 0;
    double floor = perBinOut;
    if (!floor) floor = pct(band, 0.2);
    if (!floor) floor = 1e-30;
#define DB_K(k) (10 * std::log10(psd[k] / segs / floor))
    {
        double s = 0;
        for (std::size_t i = 0; i < band.size(); ++i) s += band[i];
        out.bandDb = round1(10 * std::log10(s / band.size() / floor));
    }
    // the tones: local peaks 10 dB or more above the floor, strongest first, each refined by a parabola
    std::vector<AnalysisPeak> peaks;
    for (int k = 0; k < L; ++k) {
        if (!IN_BAND(k)) continue;
        const double a = psd[(k - 1 + L) % L], b = psd[k], c = psd[(k + 1) % L];
        if (b >= a && b >= c && DB_K(k) >= 10) {
            const double y0 = std::log(a + 1e-30), y1 = std::log(b + 1e-30), y2 = std::log(c + 1e-30), d = y0 - 2 * y1 + y2;
            AnalysisPeak p = {F_OF(k) + (d < 0 ? 0.5 * (y0 - y2) / d : 0) * binHz, DB_K(k)};
            peaks.push_back(p);
        }
    }
    std::stable_sort(peaks.begin(), peaks.end(), [](const AnalysisPeak& x, const AnalysisPeak& y) { return y.db < x.db; });
    // the occupied bandwidth (99% of the power clearly above the floor: an averaged noise bin wanders some 15% either side)
    {
        std::vector<int> ks;
        for (int k = 0; k < L; ++k)
            if (IN_BAND(k)) ks.push_back(k);
        std::stable_sort(ks.begin(), ks.end(), [&](int x, int y) { return F_OF(x) < F_OF(y); });
        std::vector<double> e(ks.size());
        double tot = 0;
        for (std::size_t i = 0; i < ks.size(); ++i) {
            e[i] = std::max(0.0, psd[ks[i]] / segs - 1.3 * floor);
            tot += e[i];
        }
        double a = 0;
        long i0 = 0, i1 = static_cast<long>(e.size()) - 1;
        for (; i0 < static_cast<long>(e.size()); ++i0) {
            a += e[i0];
            if (a >= 0.005 * tot) break;
        }
        if (i0 >= static_cast<long>(e.size())) i0 = static_cast<long>(e.size()) - 1;
        a = 0;
        for (; i1 > 0; --i1) {
            a += e[i1];
            if (a >= 0.005 * tot) break;
        }
        out.obw = std::max(binHz, (i1 - i0 + 1) * binHz);
        out.lo = F_OF(ks[i0]);
        out.hi = F_OF(ks[i1]);
        // (its centre: the power-weighted middle, steadier on a weak signal than halfway between the edges)
        double sw = 0, sf = 0;
        for (long i = i0; i <= i1; ++i) {
            sw += e[i];
            sf += e[i] * F_OF(ks[i]);
        }
        out.centre = sw ? sf / sw : (out.lo + out.hi) / 2;
    }
    // (from here on, only the span the signal occupies, with a margin: a narrow signal in a wide band otherwise has
    //  most of the band's noise on it)
    if (out.hi - out.lo < bw * 0.7) {
        const double m = std::max(30.0, (out.hi - out.lo) * 0.25);
        monBand(zr, zi, fs, out.lo - m, out.hi + m);
    }
    for (std::size_t i = 0; i < peaks.size() && i < 6; ++i) {
        AnalysisPeak p = {static_cast<double>(jsRound(peaks[i].f)), static_cast<double>(jsRound(peaks[i].db))};
        out.peaks.push_back(p);
    }
    out.snr = peaks.empty() ? 0 : static_cast<int>(jsRound(peaks[0].db));
    if ((perBinOut && out.bandDb < 1.5 && peaks.empty()) || (!perBinOut && peaks.empty() && out.obw < bw * 0.1)) {
        out.kind = "none";
        out.name = "no signal";
        out.text = "Nothing above the noise in the band";
        return out;
    }
    // 2. the envelope, smoothed over 2 ms: keyed (on and off), or steady
    const int sm = std::max(1L, jsRound(fs * 0.002));
    std::vector<double> env(N), a2(N), eDb(N);
    {
        double acc = 0;
        for (std::size_t i = 0; i < N; ++i) a2[i] = zr[i] * zr[i] + zi[i] * zi[i];
        for (std::size_t i = 0; i < N; ++i) {
            acc += a2[i] - (i >= static_cast<std::size_t>(sm) ? a2[i - sm] : 0);
            env[i] = std::sqrt(std::max(0.0, acc / std::min<double>(i + 1, sm)));
        }
    }
    for (std::size_t i = 0; i < N; ++i) eDb[i] = 20 * std::log10(env[i] + 1e-12);
    const double hiDb = pct(eDb, 0.95), loDb = pct(eDb, 0.05), mid = (hiDb + loDb) / 2;
    std::size_t off = 0;
    for (std::size_t i = 0; i < N; ++i)
        if (eDb[i] < mid) off++;
    const double offFrac = static_cast<double>(off) / N, swing = hiDb - loDb;
    double em = 0, ev = 0;
    for (std::size_t i = 0; i < N; ++i) em += env[i];
    em /= N;
    for (std::size_t i = 0; i < N; ++i) ev += (env[i] - em) * (env[i] - em);
    const double cv = std::sqrt(ev / N) / (em ? em : 1);
    // (steadiness from a lightly smoothed envelope, 0.25 ms: the 2 ms one averages away AM at audio rates)
    double cvF;
    {
        const int q = std::max(1L, jsRound(fs * 0.00025));
        double acc = 0, m1 = 0, m2 = 0;
        for (std::size_t i = 0; i < N; ++i) a2[i] = std::sqrt(zr[i] * zr[i] + zi[i] * zi[i]);
        for (std::size_t i = 0; i < N; ++i) {
            acc += a2[i] - (i >= static_cast<std::size_t>(q) ? a2[i - q] : 0);
            const double v = acc / std::min<double>(i + 1, q);
            m1 += v;
            m2 += v * v;
        }
        m1 /= N;
        cvF = std::sqrt(std::max(0.0, m2 / N - m1 * m1)) / (m1 ? m1 : 1);
    }
    out.swingDb = round1(swing);
    out.offFrac = round2(offFrac);
    out.cv = round2(cv);
    out.cvFast = round2(cvF);
    const bool keyed = swing >= 12 && offFrac >= 0.1 && offFrac <= 0.9;
    // the on and off stretches (with a little hysteresis), for Morse's timing
    std::vector<std::pair<bool, double> > runs;
    {
        bool st = eDb[0] > mid;
        std::size_t t0 = 0;
        for (std::size_t i = 1; i < N; ++i) {
            const double v = eDb[i];
            const bool s2 = st ? v > mid - 3 : v > mid + 3;
            if (s2 != st) {
                runs.push_back(std::make_pair(st, (i - t0) / fs));
                st = s2;
                t0 = i;
            }
        }
    }
    // 3. the instantaneous frequency (where the signal is on), and FSK's tone pair
    std::vector<double> fi(N, 0.0), fis(N);
    for (std::size_t i = 1; i < N; ++i) {
        const double r = zr[i] * zr[i - 1] + zi[i] * zi[i - 1], q = zi[i] * zr[i - 1] - zr[i] * zi[i - 1];
        fi[i] = std::atan2(q, r) * fs / (2 * kPi);
    }
    {
        const int fsm = std::max(1L, jsRound(fs * 0.001));
        double acc = 0;
        for (std::size_t i = 0; i < N; ++i) {
            acc += fi[i] - (i >= static_cast<std::size_t>(fsm) ? fi[i - fsm] : 0);
            fis[i] = acc / std::min<double>(i + 1, fsm);
        }
    }
    // two tones of similar strength, a sensible shift apart, that take turns (the power around each, over 5 ms,
    // falls as the other's rises)
    bool fsk = false;
    double fskLo = 0, fskHi = 0;
    if (peaks.size() >= 2) {
        const AnalysisPeak& p1 = peaks[0];
        const AnalysisPeak* p2 = 0;
        for (std::size_t i = 1; i < peaks.size(); ++i) {
            const double d = std::fabs(peaks[i].f - p1.f);
            if (d >= 40 && d <= 1200 && p1.db - peaks[i].db <= 10) {
                p2 = &peaks[i];
                break;
            }
        }
        if (p2) {
            const int step = static_cast<int>(jsRound(fs * 0.005));
            std::vector<double> A, B;
            for (int pass = 0; pass < 2; ++pass) {
                const double f = pass ? p2->f : p1.f;
                std::vector<double>& o2 = pass ? B : A;
                double cr = 1, ci = 0;
                const double wr = std::cos(-2 * kPi * f / fs), wi = std::sin(-2 * kPi * f / fs);
                for (std::size_t o = 0; o + step <= N; o += step) {
                    double sr = 0, si = 0;
                    for (std::size_t i = o; i < o + step; ++i) {
                        sr += zr[i] * cr - zi[i] * ci;
                        si += zr[i] * ci + zi[i] * cr;
                        const double t = cr * wr - ci * wi;
                        ci = cr * wi + ci * wr;
                        cr = t;
                    }
                    o2.push_back(sr * sr + si * si);
                }
            }
            double ma = 0, mb = 0;
            for (std::size_t i = 0; i < A.size(); ++i) {
                ma += A[i];
                mb += B[i];
            }
            ma /= A.size();
            mb /= B.size();
            double sab = 0, saa = 0, sbb = 0;
            for (std::size_t i = 0; i < A.size(); ++i) {
                sab += (A[i] - ma) * (B[i] - mb);
                saa += (A[i] - ma) * (A[i] - ma);
                sbb += (B[i] - mb) * (B[i] - mb);
            }
            const double den = std::sqrt(saa * sbb);
            const double corr = sab / (den ? den : 1);
            out.toneCorr = round2(corr);
            if (corr < -0.3) {
                fsk = true;
                fskLo = std::min(p1.f, p2->f);
                fskHi = std::max(p1.f, p2->f);
            }
        }
    }
    // 4. the symbol rate: the strongest timing line, from what changes at each symbol
    struct Res {
        static AnalysisResult& set(AnalysisResult& o, const char* kind, const std::string& name, const char* sure,
                                   const std::string& text) {
            o.kind = kind;
            o.name = name;
            o.sure = sure;
            o.text = text;
            return o;
        }
    };
    if (fsk) {
        // the two tones, as the averages of the instantaneous frequency either side of the middle, then the runs
        // between changes of tone. Every run lasts a whole number of symbols, so the symbol rate is the slowest
        // that fits them all (any multiple of it fits too)
        double c = (fskHi + fskLo) / 2, lo = fskLo, hi = fskHi;
        for (int it = 0; it < 5; ++it) {
            double sa = 0, sb = 0;
            std::size_t na = 0, nb2 = 0;
            for (std::size_t i = 0; i < N; ++i) {
                if (env[i] < em * 0.5) continue;
                if (fis[i] < c) {
                    sa += fis[i];
                    na++;
                } else {
                    sb += fis[i];
                    nb2++;
                }
            }
            if (na && nb2) {
                lo = sa / na;
                hi = sb / nb2;
                c = (lo + hi) / 2;
            }
        }
        const double shift = hi - lo;
        std::vector<double> lens;
        {
            int prev = 0;
            std::size_t t0 = 0;
            for (std::size_t i = 0; i < N; ++i) {
                if (env[i] < em * 0.3) continue;
                const double d = fis[i] - c;
                const int b = d > 0 ? 1 : d < 0 ? -1 : 0;
                if (prev && b && b != prev) {
                    lens.push_back((i - t0) / fs);
                    t0 = i;
                }
                if (b) prev = b;
            }
        }
        std::vector<double> L2;
        for (std::size_t i = 1; i < lens.size(); ++i)
            if (lens[i] > 1.0 / 2000 && lens[i] < 0.5) L2.push_back(lens[i]);
        struct Fit {
            const std::vector<double>& v;
            double operator()(double b) const {
                double s2 = 0;
                for (std::size_t i = 0; i < v.size(); ++i) s2 += std::cos(2 * kPi * v[i] * b);
                return s2 / (v.empty() ? 1 : v.size());
            }
        } fit = {L2};
        double raw = kNaN;
        if (L2.size() >= 20) {
            double best = 0;
            std::vector<std::pair<double, double> > sc;
            for (double b = 30; b <= 1500; b *= 1.002) {
                const double v = fit(b);
                sc.push_back(std::make_pair(b, v));
                if (v > best) best = v;
            }
            for (std::size_t i = 0; i < sc.size(); ++i) {
                if (sc[i].second >= 0.85 * best && best > 0.5) {
                    double b = sc[i].first;
                    for (int k = 0; k < 30; ++k) {
                        const double bb = b * (1 + (k - 15) * 0.0005);
                        if (fit(bb) > fit(b)) b = bb;
                    }
                    raw = b;
                    break;
                }
            }
        }
        bool ok;
        double baud = snapBaud(raw, &ok);
        bool stop15 = false;
        // (RTTY's stop bit lasts 1.5 bits, so its runs come in half bits and the slowest rate fitting them all is
        //  twice the baud rate: where the rate found isn't a standard one with this shift but half of it is, it's
        //  half, with 1.5 stop bits)
        if (ok) {
            bool any = false;
            for (int i = 0; i < kNFsk; ++i) any = any || fskMatch(kFsk[i], shift, baud);
            if (!any) {
                bool ok2;
                const double h = snapBaud(raw / 2, &ok2);
                bool any2 = false;
                for (int i = 0; i < kNFsk; ++i) any2 = any2 || fskMatch(kFsk[i], shift, h);
                if (any2) {
                    baud = h;
                    stop15 = true;
                }
            }
        }
        // (the standard modes this shift and rate match; where two shifts are near, the nearer)
        std::vector<int> stdm;
        for (int i = 0; i < kNFsk; ++i)
            if (fskMatch(kFsk[i], shift, baud)) stdm.push_back(i);
        if (!stdm.empty()) {
            double dmin = 1e9;
            for (std::size_t i = 0; i < stdm.size(); ++i) dmin = std::min(dmin, std::fabs(shift - kFsk[stdm[i]].shift));
            std::vector<int> keep;
            for (std::size_t i = 0; i < stdm.size(); ++i)
                if (std::fabs(shift - kFsk[stdm[i]].shift) <= dmin + 1) keep.push_back(stdm[i]);
            stdm.swap(keep);
        }
        for (std::size_t i = 0; i < stdm.size(); ++i) out.standards.push_back(kFsk[stdm[i]].name);
        const double sh = std::floor(shift / 5 + 0.5) * 5;
        out.fskLo = static_cast<double>(jsRound(lo));
        out.fskHi = static_cast<double>(jsRound(hi));
        out.shift = sh;
        out.baud = baud;
        out.stop15 = stop15;
        out.centre = static_cast<double>(jsRound(c));
        std::string text = "FSK: " + fmt("%.0f", sh) + " Hz shift";
        if (isNum(baud))
            text += ", " + fmt("%g", baud) + " baud" + (stop15 ? " (1.5 stop bits)" : "");
        else
            text += ", baud rate unclear";
        if (!out.standards.empty()) {
            text += " \xC2\xB7 ";
            for (std::size_t i = 0; i < out.standards.size(); ++i) text += (i ? " or " : "") + out.standards[i];
        }
        return Res::set(out, "FSK", out.standards.empty() ? std::string("FSK") : out.standards[0],
                        out.standards.empty() ? "possibly" : "likely", text);
    }
    // the phase across each dip in the envelope: dips, and how many turned the phase over (by more than 2.2 rad)
    struct RevAt {
        const std::vector<double>& eDb;
        const std::vector<double>& zr;
        const std::vector<double>& zi;
        double mid, centre, fs;
        void operator()(int half, int* dipsOut, int* revOut) const {
            int dips = 0, rev = 0;
            const long N = static_cast<long>(eDb.size());
            for (long i = half; i < N - half; ++i) {
                if (eDb[i] < mid && eDb[i - 1] >= mid) {
                    const long j0 = i - half, j1 = std::min(N - 1, i + half * 2L);
                    // (only a brief dip, the signal back on at the far side: across a Morse gap there's only noise)
                    if (eDb[j1] < mid || eDb[j0] < mid) {
                        i += half;
                        continue;
                    }
                    // (less the signal's own turning between the two moments; its centre is the middle of what it
                    //  occupies: PSK31 has no carrier, its strongest lines are the two at +-15.6 Hz)
                    double d = std::atan2(zi[j1] * zr[j0] - zr[j1] * zi[j0], zr[j1] * zr[j0] + zi[j1] * zi[j0]) -
                               2 * kPi * centre * (j1 - j0) / fs;
                    d = std::atan2(std::sin(d), std::cos(d));
                    dips++;
                    if (std::fabs(d) > 2.2) rev++;
                    i += half;
                }
            }
            *dipsOut = dips;
            *revOut = rev;
        }
    } revAt = {eDb, zr, zi, mid, out.centre, fs};
    // PSK (before Morse: its envelope dips at each phase reversal, which looks like keying): one narrow tone whose
    //  envelope dips at a steady rate, the phase turning over (about half a cycle) across each dip
    // (a dominant line doesn't rule PSK out: PSK31 sending a run of 1 bits is a plain carrier for a while)
    if (!peaks.empty() && out.obw <= 600) {
        std::vector<double> e2(N);
        for (std::size_t i = 0; i < N; ++i) e2[i] = env[i] * env[i];
        const std::pair<double, double> ln = monLine(e2, fs, 15, std::min(1500.0, fs / 4));
        const double raw = ln.second >= 12 ? ln.first : kNaN;
        int dips = 0, rev = 0;
        revAt(isNum(raw) ? std::max(1L, jsRound(fs / raw / 3)) : std::max(1L, jsRound(fs * 0.008)), &dips, &rev);
        out.reversals = dips ? round2(static_cast<double>(rev) / dips) : 0;
        out.revDips = dips;
        if (isNum(raw)) {
            const PskStd* rate = 0;
            for (std::size_t i = 0; i < sizeof kPsk / sizeof kPsk[0]; ++i)
                if (std::fabs(raw - kPsk[i].rate) / kPsk[i].rate < 0.03) {
                    rate = &kPsk[i];
                    break;
                }
            // (PSK reverses at nearly every brief dip; noise dipping a Morse dash shows a random phase, ~30%)
            if ((dips >= 10 && static_cast<double>(rev) / dips >= 0.5) || (dips >= 4 && static_cast<double>(rev) / dips >= 0.7)) {
                out.baud = rate ? rate->rate : static_cast<double>(jsRound(raw));
                out.tone = static_cast<double>(jsRound(out.centre));
                return Res::set(out, "PSK", rate ? rate->name : "PSK", rate && dips >= 10 ? "likely" : "possibly",
                                rate ? "PSK: " + fmt("%g", rate->rate) + " symbols a second (" + rate->name + ")"
                                     : "Phase-shift keyed, about " + fmt("%.0f", static_cast<double>(jsRound(raw))) +
                                           " symbols a second");
            }
        }
    }
    // Morse: keyed, one narrow tone, its on-stretches falling into dots and dashes (about 1 : 3)
    std::vector<double> on;
    for (std::size_t i = 0; i < runs.size(); ++i)
        if (runs[i].first && runs[i].second >= 0.02) on.push_back(runs[i].second);
    // (not where nearly all of the brief dips turned the phase over: that's PSK, perhaps only a short burst of it)
    if (keyed && !peaks.empty() && out.obw <= 400 && on.size() >= 6 && !(out.reversals >= 0.7 && out.revDips >= 4)) {
        std::vector<double> lg(on.size());
        for (std::size_t i = 0; i < on.size(); ++i) lg[i] = std::log(on[i]);
        std::sort(lg.begin(), lg.end());
        double c1 = lg.front(), c2 = lg.back();
        for (int it = 0; it < 20; ++it) {
            double sA = 0, sB = 0;
            int nA = 0, nB = 0;
            for (std::size_t i = 0; i < lg.size(); ++i) {
                if (std::fabs(lg[i] - c1) <= std::fabs(lg[i] - c2)) {
                    sA += lg[i];
                    nA++;
                } else {
                    sB += lg[i];
                    nB++;
                }
            }
            if (nA) c1 = sA / nA;
            if (nB) c2 = sB / nB;
        }
        const double dot = std::exp(c1), dash = std::exp(c2), ratio = dash / dot;
        const bool two = ratio > 2 && ratio < 4.5;
        // (each "on" stretch is read a little long, by the smoothing and the filter, the same for dots and dashes:
        //  half the difference between a dash (3 units) and a dot (1) has none of that in it)
        const double unit = two ? (dash - dot) / 2 : dot, wpm = 1.2 / unit;
        out.tone = static_cast<double>(jsRound(peaks[0].f));
        if (two && wpm >= 4 && wpm <= 60) {
            out.wpm = static_cast<double>(jsRound(wpm));
            return Res::set(out, "CW", "CW (Morse)", "likely",
                            "Morse: about " + fmt("%.0f", out.wpm) + " words a minute (dots " +
                                fmt("%.0f", static_cast<double>(jsRound(dot * 1000))) + " ms, dashes " +
                                fmt("%.0f", static_cast<double>(jsRound(dash * 1000))) + " ms)");
        }
        return Res::set(out, "OOK", "on-off keying", "possibly",
                        "Keyed on and off, one tone; its timing isn't Morse's (stretches " +
                            fmt("%.0f", static_cast<double>(jsRound(dot * 1000))) + " and " +
                            fmt("%.0f", static_cast<double>(jsRound(dash * 1000))) + " ms)");
    }
    // FM: a steady envelope but a wandering frequency, wider than a carrier (before AM)
    if (cvF < 0.2 && out.obw > 1000) {
        double s = 0;
        std::size_t n = 0;
        for (std::size_t i = 0; i < N; ++i)
            if (env[i] > em * 0.5) {
                s += fis[i] * fis[i];
                n++;
            }
        const double dev = std::sqrt(s / (n ? n : 1));
        out.deviation = static_cast<double>(jsRound(dev * 1.4));
        return Res::set(out, "FM", "FM", "likely",
                        "FM: a constant envelope, its frequency swinging about \xC2\xB1" + fmt("%.0f", out.deviation) + " Hz");
    }
    // a carrier: one tone, steady, well above any other tone in the band; with matching sidebands it's AM
    const bool carrier = !peaks.empty() && peaks[0].db >= 20 && std::fabs(peaks[0].f) <= bw / 2 &&
                         peaks[0].db - (peaks.size() > 1 ? peaks[1].db : 0) >= 6;
    if (carrier) {
        const double c0 = peaks[0].f;
        double lo = 0, hi = 0;
        int nl = 0, nh = 0;
        for (int k = 0; k < L; ++k) {
            if (!IN_BAND(k)) continue;
            const double d = F_OF(k) - c0;
            if (std::fabs(d) < 150 || std::fabs(d) > 4000) continue;
            if (d < 0) {
                lo += psd[k];
                nl++;
            } else {
                hi += psd[k];
                nh++;
            }
        }
        const double sbL = 10 * std::log10((lo / (nl ? nl : 1)) / segs / floor),
                     sbH = 10 * std::log10((hi / (nh ? nh : 1)) / segs / floor);
        out.carrier = static_cast<double>(jsRound(c0));
        if (sbL > 3 && sbH > 3 && std::fabs(sbL - sbH) < 6)
            return Res::set(out, "AM", "AM", "likely", std::string("AM: a carrier with matching sidebands") + (cv > 0.3 ? ", fading" : ""));
        if (cvF < 0.2 && out.obw <= 300) return Res::set(out, "CARRIER", "unmodulated carrier", "likely", "A steady, unmodulated carrier");
    }
    // voice in SSB: no carrier, an envelope that comes and goes, a few kHz wide
    if (!carrier && cv > 0.5 && out.obw >= 1200 && out.obw <= 4000)
        return Res::set(out, "SSB", "SSB voice", "possibly", "Voice-like, with no carrier: SSB");
    std::string text = hzOrKhz(out.obw) + " wide";
    if (!peaks.empty()) {
        const std::size_t np = std::min<std::size_t>(peaks.size(), 6);
        text += ", " + fmt("%.0f", static_cast<double>(np)) + (peaks.size() > 1 ? " tones" : " tone");
    }
    text += keyed ? ", keyed on and off" : cv < 0.15 ? ", steady" : "";
#undef F_OF
#undef IN_BAND
#undef DB_K
    return Res::set(out, "UNKNOWN", "not recognised", "", text);
}

}  // namespace dsp
