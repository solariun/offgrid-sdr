// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Time.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The time-signal decoder described in Time.h: a port of OFFgrid-SDR's
//  TimeDecoder class (findCarrier, frame, secondPhase, minuteWork,
//  chuSecondBytes, chuBursts, wwvBits, wwvFrame, tickOffset,
//  minuteOffset), of the stream timing in timeFeed() and of timeBest(), by
//  diddy-boy, index.html.  The function names follow the JavaScript.
//
//  Additions to the JS (each marked where it is):
//    - the audio is worked in chunks of at most 50 ms, with the "once a
//      second" work after each, so the result does not depend on how the
//      caller slices the audio (the browser: once per audio block), and
//      CHU's 2 s ring always still holds the second being read;
//    - WWVH: the browser computes the 1200 Hz tick profile but never uses
//      it, so WWVH alone gave no ticks.  Here the stronger of the 1000 Hz
//      (WWV) and 1200 Hz (WWVH) tick profiles sets the second (with a
//      1.5x hysteresis) and names the station; the minute tone is also
//      looked for at 1500 Hz (both stations' hour tone);
//    - CHU's FSK space tone (2025 Hz above the carrier) lies 25 Hz from
//      the tick tone, so seconds 31-39 looked like minute tones to the
//      minute test; a candidate where the FSK mark tone (2225 Hz) is as
//      strong as half the tick-frequency energy is not taken for one;
//    - CHU's bytes: the tone sums use rotating phasors (no cos/sin per
//      sample), from the window's start (the phase does not matter);
//    - loops over whole seconds are capped at 100 at a time (a wild
//      timestamp cannot make them run for ever); NaN / infinite samples
//      count as silence and a filter state gone non-finite is restarted;
//    - WWV: a single minute frame is taken only when every bit of the time
//      was read clearly as well as all six markers (the browser took any
//      frame with six markers, and in noise took wrong days); otherwise two
//      frames in a row that agree, as before;
//    - the second's start is taken 2.5 ms later in the rising 5 ms frame
//      (see secondPhase): the browser's figure was 2.4 ms early on average;
//    - text lines are produced (the browser shows fields in its box).
//
//  How it fits in
//  --------------
//  See Time.h.
//
//  Contributing
//  ------------
//  Keep the function names parallel to the JavaScript; mark deviations.
// ============================================================================
#include "Time.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace dec {

namespace {

const double kPi = 3.14159265358979323846;
const int kSpecN = 16384;          // the carrier search's FFT (as the browser)
const std::size_t kFrames = 4096;  // 5 ms frames kept (~20 s; the browser keeps 2600-3000)
const std::size_t kSubs = 1024;    // 20 ms subcarrier frames kept (~20 s)
const int kMaxSeconds = 100;       // (Addition to the JS) whole seconds worked at most per call

bool finite(double v) { return v == v && v - v == 0; }   // not NaN, not +-inf
double jsRound(double v) { return std::floor(v + 0.5); }  // Math.round
double sgn(double v) { return v > 0 ? 1 : v < 0 ? -1 : 0; }
// Goertzel power from its two state values
double gpw(double a, double b, double w) { return a * a + b * b - w * a * b; }

// days since 1970-01-01 -> civil date (H. Hinnant's algorithm)
void civilFromDays(long long z, int& y, int& m, int& d) {
    z += 719468;
    const long long era = (z >= 0 ? z : z - 146096) / 146097;
    const long long doe = z - era * 146097;
    const long long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    long long yy = yoe + era * 400;
    const long long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const long long mp = (5 * doy + 2) / 153;
    d = static_cast<int>(doy - (153 * mp + 2) / 5 + 1);
    m = static_cast<int>(mp < 10 ? mp + 3 : mp - 9);
    if (m <= 2) ++yy;
    y = static_cast<int>(yy);
}
long long daysFromCivil(int y, int m, int d) {
    y -= m <= 2;
    const long long era = (y >= 0 ? y : y - 399) / 400;
    const long long yoe = y - era * 400;
    const long long doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const long long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

const TimeStationFreq kStations[] = {
    {TimeStation::Chu, 3330, "CHU, Canada"},      {TimeStation::Chu, 7850, "CHU, Canada"},
    {TimeStation::Chu, 14670, "CHU, Canada"},     {TimeStation::Wwv, 2500, "WWV / WWVH, USA"},
    {TimeStation::Wwv, 5000, "WWV / WWVH, USA"},  {TimeStation::Wwv, 10000, "WWV / WWVH, USA"},
    {TimeStation::Wwv, 15000, "WWV / WWVH, USA"}, {TimeStation::Wwv, 20000, "WWV, USA"},
    {TimeStation::Wwv, 25000, "WWV, USA"},        {TimeStation::Rwm, 4996, "RWM, Russia"},
    {TimeStation::Rwm, 9996, "RWM, Russia"},      {TimeStation::Rwm, 14996, "RWM, Russia"}};

}  // namespace

// ---------------------------------------------------------------------------
// names, the station list, UTC helpers

const char* timeStationName(TimeStation st) {
    switch (st) {
        case TimeStation::Chu: return "CHU";
        case TimeStation::Wwv: return "WWV";
        case TimeStation::Wwvh: return "WWVH";
        case TimeStation::Rwm: return "RWM";
    }
    return "?";
}

bool parseTimeStation(const std::string& name, TimeStation& st) {
    std::string u;
    for (std::size_t i = 0; i < name.size(); ++i) u += static_cast<char>(std::toupper(static_cast<unsigned char>(name[i])));
    if (u == "CHU") { st = TimeStation::Chu; return true; }
    if (u == "WWV" || u == "WWVH" || u == "WWV/WWVH") { st = TimeStation::Wwv; return true; }
    if (u == "RWM") { st = TimeStation::Rwm; return true; }
    return false;
}

const std::vector<TimeStationFreq>& timeStationList() {
    static const std::vector<TimeStationFreq> list(kStations, kStations + sizeof kStations / sizeof kStations[0]);
    return list;
}

void utcFields(double unixSeconds, int& year, int& month, int& day, int& doy, int& hour, int& minute, int& second) {
    const double fl = std::floor(unixSeconds);
    long long s = static_cast<long long>(fl);
    long long days = s >= 0 ? s / 86400 : -((-s + 86399) / 86400);
    long long sod = s - days * 86400;
    civilFromDays(days, year, month, day);
    doy = static_cast<int>(days - daysFromCivil(year, 1, 1)) + 1;
    hour = static_cast<int>(sod / 3600);
    minute = static_cast<int>((sod / 60) % 60);
    second = static_cast<int>(sod % 60);
}

double utcFromDayOfYear(int year, int doy, int hour, int minute, int second) {
    return static_cast<double>(daysFromCivil(year, 1, 1) + doy - 1) * 86400.0 + hour * 3600.0 + minute * 60.0 + second;
}

// ---------------------------------------------------------------------------
// TimeDecoder

TimeDecoder::TimeDecoder()
    : kind_(TimeStation::Chu), rate_(0), frN_(240), subN_(960), chunkN_(2400), fft_(kSpecN) {
    spec_.assign(kSpecN, 0.f);
    specAcc_.assign(kSpecN / 2, 0.0);
    fbuf_.resize(kSpecN);
    hann_.resize(kSpecN);
    for (int k = 0; k < kSpecN; ++k) hann_[k] = static_cast<float>(0.5 - 0.5 * std::cos(2 * kPi * k / kSpecN));
    sorted_.reserve(kSpecN / 2);
    frames_.resize(kFrames);
    sub_.resize(kSubs);
    reset();
}

void TimeDecoder::reset() {
    // (the browser's reset(kind) and timeFeed()'s retuning branch together)
    tsPending_ = false; ts_ = 0; streamN_ = 0; anchorSet_ = false; anchor_ = 0; histN_ = 0;
    jumping_ = false; jumpSince_ = 0; jumps_ = 0; tEnd_ = 0;
    fc_ = 1000; fcN_ = 0; specN_ = 0; specCount_ = 0;
    std::fill(specAcc_.begin(), specAcc_.end(), 0.0);
    carrierDb_ = -99; noiseDb_ = -99;
    grStarted_ = false; sbStarted_ = false; grN_ = 0; sbN_ = 0; grT_ = 0; sbT_ = 0;
    std::memset(g_, 0, sizeof g_); std::memset(sg_, 0, sizeof sg_);
    ringN_ = 0; anchorT_ = 0;
    std::fill(ring_.begin(), ring_.end(), 0.f);
    year_ = 0;
    result_ = Result(); results_ = 0;
    out_.clear();
    timeJump();
}

// the audio's timing jumped (samples lost): what was built up from before no longer lines up, so start it afresh
void TimeDecoder::timeJump() {
    clearProfile();
    zN_ = 0; minuteKnown_ = false; minuteStart_ = 0; minuteSeen_ = 0; minSecSet_ = false; minSec_ = 0;
    chuSecSet_ = false; chuSec_ = 0; lastSecSet_ = false; lastSec_ = 0;
    aN_ = 0; std::memset(bits_, 0, sizeof bits_); std::memset(margin_, 0, sizeof margin_); minuteAtSet_ = false; minuteAt_ = 0; wwvPrevSet_ = false;
    wwvPrevT_ = 0; wwvPrevSecStart_ = 0;
    frHead_ = 0; frCount_ = 0; subHead_ = 0; subCount_ = 0;
    tickAnnounced_ = false; minuteAnnounced_ = 0;
}

void TimeDecoder::clearProfile() {
    std::memset(prof_, 0, sizeof prof_); std::memset(prof2_, 0, sizeof prof2_);
    profN_ = 0; havePhase_ = false; tickPhase_ = 0; tickQ_ = 0; wwvh_ = false; zN_ = 0;
}

void TimeDecoder::setStation(TimeStation st) {
    kind_ = st == TimeStation::Wwvh ? TimeStation::Wwv : st;
    reset();
}

bool TimeDecoder::setStation(const std::string& name) {
    TimeStation st;
    if (!parseTimeStation(name, st)) return false;
    setStation(st);
    return true;
}

bool TimeDecoder::stationForFrequency(double hz, TimeStation& st, double* listedKHz) {
    // (timeStationAt(): within 500 Hz of a listed frequency)
    for (std::size_t i = 0; i < sizeof kStations / sizeof kStations[0]; ++i)
        if (std::fabs(hz - kStations[i].kHz * 1e3) <= 500) {
            st = kStations[i].station;
            if (listedKHz) *listedKHz = kStations[i].kHz;
            return true;
        }
    return false;
}

void TimeDecoder::setTimestamp(double unixSeconds) {
    if (!finite(unixSeconds)) return;
    ts_ = unixSeconds; tsPending_ = true;
}

void TimeDecoder::takeText(std::string& out) {
    out += out_;
    out_.clear();
}

void TimeDecoder::setRate(double rate) {
    rate_ = rate;
    frN_ = std::max(1, static_cast<int>(jsRound(rate * 0.005)));
    subN_ = std::max(1, static_cast<int>(jsRound(rate * 0.02)));
    chunkN_ = std::max<std::size_t>(1, static_cast<std::size_t>(rate * 0.05));
    ring_.assign(static_cast<std::size_t>(jsRound(rate * 2)), 0.f);
    ringN_ = 0;
    const std::size_t len = static_cast<std::size_t>(rate * 0.51) + 4;
    mc_.assign(len, 0.0); ms_.assign(len, 0.0); sc_.assign(len, 0.0); ss_.assign(len, 0.0);
    specN_ = 0; specCount_ = 0;
    std::fill(specAcc_.begin(), specAcc_.end(), 0.0);
    grStarted_ = false; sbStarted_ = false;
    // (the stream's timing is counted in samples at one rate: a new rate starts it again)
    streamN_ = 0; anchorSet_ = false; histN_ = 0; jumping_ = false;
}

void TimeDecoder::process(const float* audio, std::size_t n, double rate) {
    if (!(rate > 0) || !finite(rate) || !audio || n == 0) return;
    if (rate != rate_) { setRate(rate); timeJump(); }
    // ---- timeFeed(): when these samples were heard, on the computer's own clock. Blocks only ever arrive
    // late, so the earliest estimate is the right one; but over the last 3 seconds only, so that samples lost
    // along the way are allowed for within 3 s.
    const double nBefore = static_cast<double>(streamN_);
    streamN_ += static_cast<long long>(n);
    if (tsPending_ || !anchorSet_) {
        const double now = (tsPending_ ? ts_ : 0) + n / rate_;          // (the block's end)
        const double est = now - static_cast<double>(streamN_) / rate_;  // (= the timestamp - samples before it)
        // (Addition to the JS: entries within 0.1 s are merged, keeping the earliest, so the history has a
        //  fixed size whatever the block size)
        if (histN_ > 0 && now - hist_[histN_ - 1].now < 0.1 && now >= hist_[histN_ - 1].now) {
            if (est < hist_[histN_ - 1].est) hist_[histN_ - 1].est = est;
        } else {
            if (histN_ == 64) { std::memmove(hist_, hist_ + 1, sizeof(Hist) * 63); --histN_; }
            hist_[histN_].now = now; hist_[histN_].est = est; ++histN_;
        }
        int drop = 0;
        while (drop < histN_ - 1 && (hist_[drop].now < now - 3 || hist_[drop].now > now + 1)) ++drop;
        if (drop) { std::memmove(hist_, hist_ + drop, sizeof(Hist) * (histN_ - drop)); histN_ -= drop; }
        double a = hist_[0].est;
        for (int i = 1; i < histN_; ++i) a = std::min(a, hist_[i].est);
        // (a real jump is over 50 ms and still there a second later: block delays vary by less, and come and go)
        if (anchorSet_ && std::fabs(a - anchor_) > 0.05) {
            if (!jumping_) { jumping_ = true; jumpSince_ = now; }
            if (now - jumpSince_ >= 1) { timeJump(); anchor_ = a; jumping_ = false; ++jumps_; }
        } else {
            jumping_ = false;
            anchor_ = anchorSet_ ? anchor_ * 0.9 + a * 0.1 : a;
            anchorSet_ = true;
        }
    }
    tsPending_ = false;
    const double t0 = anchor_ + nBefore / rate_;
    // (Addition to the JS) in chunks of 50 ms, the per-second work after each
    for (std::size_t off = 0; off < n; off += chunkN_) {
        const std::size_t m = std::min(chunkN_, n - off);
        chunk(audio + off, m, t0 + off / rate_);
        tEnd_ = t0 + (off + m) / rate_;
        secondsWork();
    }
}

void TimeDecoder::chunk(const float* x, std::size_t n, double t0) {
    const double rate = rate_;
    findCarrier(x, n);
    const double fc = fc_, tick1 = fc + 1000, tick2 = fc + 1200;
    // (Addition to the JS: a fourth tone: CHU's FSK mark, to tell its code from a minute tone; otherwise the
    //  1500 Hz hour tone of WWV and WWVH)
    const double extra = kind_ == TimeStation::Chu ? fc + 2225 : fc + 1500;
    // ---- 5 ms frames: tick tones and the carrier (Goertzel)
    if (!grStarted_) { grStarted_ = true; grN_ = 0; grT_ = t0; std::memset(g_, 0, sizeof g_); }
    const double w1 = 2 * std::cos(2 * kPi * tick1 / rate), w2 = 2 * std::cos(2 * kPi * tick2 / rate);
    const double wc = 2 * std::cos(2 * kPi * fc / rate), wx = 2 * std::cos(2 * kPi * extra / rate);
    // ---- the WWV subcarrier sideband (fc + 100), 20 ms frames: rectangular, so the carrier 100 Hz away falls in a null
    if (!sbStarted_) { sbStarted_ = true; sbN_ = 0; sbT_ = t0; std::memset(sg_, 0, sizeof sg_); }
    const double ws = 2 * std::cos(2 * kPi * (fc + 100) / rate), wsc = wc;
    // ---- CHU: the raw audio kept (about 2 s), for its bytes to be read bit by bit once each second is over
    const bool chu = kind_ == TimeStation::Chu;
    const std::size_t L = ring_.size();
    if (chu) anchorT_ = t0 - static_cast<double>(ringN_) / rate;   // (sample n was heard at anchorT + n / rate)
    std::size_t ri = chu ? static_cast<std::size_t>(ringN_ % static_cast<long long>(L)) : 0;
    for (std::size_t i = 0; i < n; ++i) {
        const double v = finite(x[i]) ? x[i] : 0.0;
        if (chu) { ring_[ri] = static_cast<float>(v); if (++ri == L) ri = 0; }
        double s0 = v + w1 * g_[0] - g_[1]; g_[1] = g_[0]; g_[0] = s0;
        s0 = v + w2 * g_[2] - g_[3]; g_[3] = g_[2]; g_[2] = s0;
        s0 = v + wc * g_[4] - g_[5]; g_[5] = g_[4]; g_[4] = s0;
        s0 = v + wx * g_[6] - g_[7]; g_[7] = g_[6]; g_[6] = s0;
        if (++grN_ == frN_) {
            const double t = t0 + i / rate;
            Frame f;
            f.t = grT_ + 0.0025;
            f.tick = gpw(g_[0], g_[1], w1); f.tick2 = gpw(g_[2], g_[3], w2);
            f.car = gpw(g_[4], g_[5], wc); f.extra = gpw(g_[6], g_[7], wx);
            frame(f);
            grN_ = 0; grT_ = t + 1 / rate; std::memset(g_, 0, sizeof g_);
        }
        s0 = v + ws * sg_[0] - sg_[1]; sg_[1] = sg_[0]; sg_[0] = s0;
        s0 = v + wsc * sg_[2] - sg_[3]; sg_[3] = sg_[2]; sg_[2] = s0;
        if (++sbN_ == subN_) {
            const double t = t0 + i / rate;
            SubFrame s;
            s.t = sbT_ + 0.01; s.p = gpw(sg_[0], sg_[1], ws);
            sub_[subHead_] = s; subHead_ = (subHead_ + 1) % kSubs; if (subCount_ < kSubs) ++subCount_;
            sbN_ = 0; sbT_ = t + 1 / rate; std::memset(sg_, 0, sizeof sg_);
        }
    }
    if (chu) ringN_ += static_cast<long long>(n);
    // (Addition to the JS) a filter gone non-finite starts again
    bool ok = true;
    for (int k = 0; k < 8; ++k) ok = ok && finite(g_[k]);
    for (int k = 0; k < 4; ++k) ok = ok && finite(sg_[k]);
    if (!ok) { grStarted_ = false; sbStarted_ = false; }
}

// the carrier's exact pitch (700-1300 Hz), from a long FFT every so often (an off-frequency stick moves it)
void TimeDecoder::findCarrier(const float* x, std::size_t n) {
    const int N = kSpecN;
    for (std::size_t i = 0; i < n; ++i) {
        spec_[specN_++] = finite(x[i]) ? x[i] : 0.f;
        if (specN_ != static_cast<std::size_t>(N)) continue;
        for (int k = 0; k < N; ++k) fbuf_[k] = std::complex<float>(spec_[k] * hann_[k], 0.f);
        fft_.forward(&fbuf_[0]);
        for (int k = 0; k < N / 2; ++k) specAcc_[k] += std::norm(fbuf_[k]);
        specN_ = 0; ++specCount_;
        if (specCount_ < 3) continue;
        const double bw = rate_ / N;
        const int lo = static_cast<int>(std::floor(700 / bw)), hi = std::min(N / 2 - 2, static_cast<int>(std::ceil(1300 / bw)));
        int pk = lo;
        for (int k = lo; k <= hi; ++k) if (specAcc_[k] > specAcc_[pk]) pk = k;
        const double a = specAcc_[pk - 1], b = specAcc_[pk], c = specAcc_[pk + 1];
        double den = 2 * (a - 2 * b + c); if (den == 0) den = 1;
        const double d = (a - c) / den;
        sorted_.assign(specAcc_.begin() + lo, specAcc_.begin() + hi + 1);
        std::nth_element(sorted_.begin(), sorted_.begin() + (sorted_.size() >> 1), sorted_.end());
        noiseDb_ = 10 * std::log10(sorted_[sorted_.size() >> 1] + 1e-30);
        carrierDb_ = 10 * std::log10(b + 1e-30);
        if (!finite(noiseDb_) || !finite(carrierDb_)) { noiseDb_ = carrierDb_ = -99; }
        else if (carrierDb_ - noiseDb_ > 10) {
            const double f = (pk + d) * bw;
            const bool moved = !fcN_ ? std::fabs(f - fc_) > 5 : std::fabs(f - fc_) > 8;
            fc_ = fcN_ ? 0.7 * fc_ + 0.3 * f : f; ++fcN_;
            // (found somewhere new, as with a stick well off frequency: the ticks were being looked for at the wrong
            //  pitch until now, so their picture is started again here rather than left to fade)
            if (moved) clearProfile();
        }
        std::fill(specAcc_.begin(), specAcc_.end(), 0.0); specCount_ = 0;
    }
}

void TimeDecoder::frame(const Frame& f) {
    frames_[frHead_] = f; frHead_ = (frHead_ + 1) % kFrames; if (frCount_ < kFrames) ++frCount_;
    const double fr = f.t - std::floor(f.t);
    const int b = static_cast<int>(std::floor(fr * 200)) % 200;
    const double e = kind_ == TimeStation::Rwm ? f.car : f.tick;
    // a running profile (about the last 20 s): each second's frames fade it a little
    prof_[b] = prof_[b] * 0.95 + e; prof2_[b] = prof2_[b] * 0.95 + f.tick2;
    if (b == 199) ++profN_;
}

// where the second starts on the computer's clock (0..1), and how sure
bool TimeDecoder::secondPhase(const double* P, double& phase, double& q, double& peak) const {
    const int n = 200;
    if (profN_ < 6) return false;
    double best = -1, sum = 0, D[200];
    int bi = 0;
    for (int b = 0; b < n; ++b) { D[b] = P[b] - P[(b + n - 1) % n]; sum += std::fabs(D[b]); }
    for (int b = 0; b < n; ++b) if (D[b] > best) { best = D[b]; bi = b; }
    q = best / (sum / n + 1e-30);
    // (the rise falls in frame bi. The browser takes its start, less half a frame: bi / 200 - 0.0025. Addition
    //  to the JS: bi / 200. Measured on the generators over every alignment of the 5 ms frames to the second
    //  (tests), the browser's figure was 0-5 ms early, 2.4 ms on average, for all four stations; this one is
    //  within +-2.5 ms, about 0 on average: the tick starts in the first half of the frame with the biggest rise.)
    phase = bi / 200.0;
    peak = best;
    return finite(q);
}

std::string TimeDecoder::stationNow() const {
    if (kind_ == TimeStation::Wwv) return wwvh_ ? "WWVH" : "WWV";
    return timeStationName(kind_);
}

// the strength a minute tone is judged by
double TimeDecoder::sig(const Frame& f) const {
    if (kind_ == TimeStation::Rwm) return f.car;
    double s = std::max(f.tick, f.tick2);
    if (kind_ == TimeStation::Wwv) s = std::max(s, f.extra);   // (Addition to the JS: the 1500 Hz hour tone)
    return s;
}

// frames with a + 0 <= t < b: the mean (or the most) of sig() (which = 0) or of the extra tone (which = 1)
double TimeDecoder::frameWindow(double a, double b, bool takeMax, int& count) const {
    count = 0;
    double acc = 0;
    for (std::size_t j = 0; j < frCount_; ++j) {
        const Frame& f = frames_[(frHead_ + kFrames - 1 - j) % kFrames];
        if (f.t >= b) continue;
        if (f.t < a) break;
        const double v = sig(f);
        if (takeMax) acc = count ? std::max(acc, v) : v; else acc += v;
        ++count;
    }
    return count && !takeMax ? acc / count : acc;
}

double TimeDecoder::subWindow(double a, double b) const {
    double acc = 0;
    int count = 0;
    for (std::size_t j = 0; j < subCount_; ++j) {
        const SubFrame& s = sub_[(subHead_ + kSubs - 1 - j) % kSubs];
        if (s.t >= b) continue;
        if (s.t < a) break;
        acc += s.p; ++count;
    }
    return count ? acc / count : 0;
}

void TimeDecoder::minuteWork() {
    if (!havePhase_ || tickQ_ < 6 || !frCount_) return;
    const double ph = tickPhase_;
    const double lastT = frames_[(frHead_ + kFrames - 1) % kFrames].t;
    const long long kEnd = static_cast<long long>(std::floor(lastT - ph)) - 1;
    if (!minSecSet_) { minSec_ = kEnd; minSecSet_ = true; return; }
    if (kEnd - minSec_ > kMaxSeconds) minSec_ = kEnd - kMaxSeconds;
    for (long long k = minSec_ + 1; k <= kEnd; ++k) {
        const double s0 = static_cast<double>(k) + ph;
        int nm = 0, nz = 0;
        const double m = frameWindow(s0 + 0.36, s0 + 0.48, false, nm), z = frameWindow(s0 - 0.003, s0 + 0.012, true, nz);
        if (!nm || !nz) continue;
        // against a typical second tick (the median of recent ones), not this second's own start: seconds 29 and 59 have
        // no tick at all, and their quiet would look "strong" against nothing
        if (zN_ == 40) { std::memmove(zHist_, zHist_ + 1, sizeof(double) * 39); --zN_; }
        zHist_[zN_++] = z;
        std::copy(zHist_, zHist_ + zN_, zTmp_);
        std::nth_element(zTmp_, zTmp_ + (zN_ >> 1), zTmp_ + zN_);
        const double typ = zTmp_[zN_ >> 1];
        // (Addition to the JS) CHU's FSK seconds (31-39) are not minute tones: their mark tone is there
        bool notFsk = true;
        if (kind_ == TimeStation::Chu) {
            double mk = 0; int c = 0;
            for (std::size_t j = 0; j < frCount_; ++j) {
                const Frame& f = frames_[(frHead_ + kFrames - 1 - j) % kFrames];
                if (f.t >= s0 + 0.48) continue;
                if (f.t < s0 + 0.36) break;
                mk += f.extra; ++c;
            }
            if (c) mk /= c;
            notFsk = mk < 0.5 * m;
        }
        if (zN_ >= 8 && z > 0.4 * typ && m > 0.35 * typ && notFsk) {
            minuteStart_ = s0; minuteKnown_ = true; ++minuteSeen_;
            // (Addition to the JS) a line: RWM every minute (it is all it gives), the others the first time
            double mo;
            if ((kind_ == TimeStation::Rwm || !minuteAnnounced_) && minuteOffset(mo)) {
                char buf[160];
                std::snprintf(buf, sizeof buf, "%s minute marker: PC clock %+.3f s (to the second, if it is within 30 s)\n",
                              stationNow().c_str(), mo);
                out_ += buf;
                ++minuteAnnounced_;
            }
        }
    }
    minSec_ = kEnd;
}

// the clock to the second, from the minute tick alone (assuming the clock is within 30 s of right)
bool TimeDecoder::minuteOffset(double& off) const {
    double to;
    if (!minuteKnown_ || !tickOffset(to)) return false;
    off = std::fmod(std::fmod(minuteStart_, 60.0) + 60.0, 60.0);
    if (off > 30) off -= 60;
    return true;
}

// the clock correction from the ticks alone: the fraction of a second, assuming the clock is within 30 s
bool TimeDecoder::tickOffset(double& off) const {
    if (!havePhase_ || tickQ_ < 6) return false;
    off = tickPhase_;
    if (off > 0.5) off -= 1;   // the computer's clock is this much fast (+) or slow (-)
    return true;
}

// ---- once a second or so: tick phase, minute start, then CHU bursts / WWV bits
void TimeDecoder::secondsWork() {
    double ph, q, pk;
    if (secondPhase(prof_, ph, q, pk)) {
        // (Addition to the JS) WWVH: its 1200 Hz ticks, when they are clearly the stronger
        if (kind_ == TimeStation::Wwv) {
            double ph2, q2, pk2;
            bool h = false;
            if (secondPhase(prof2_, ph2, q2, pk2) && q2 >= 6) h = wwvh_ ? pk2 > pk / 1.5 : pk2 > pk * 1.5;
            if (h) { ph = ph2; q = q2; }
            wwvh_ = h;
        }
        tickPhase_ = ph; tickQ_ = q; havePhase_ = true;
        double to;
        if (!tickAnnounced_ && tickOffset(to)) {
            char buf[160];
            std::snprintf(buf, sizeof buf, "%s second ticks found (%s): PC clock %+.3f s past the second (whole seconds unknown)\n",
                          stationNow().c_str(), tickQ_ > 8 ? "clear" : "weak", to);
            out_ += buf;
            tickAnnounced_ = true;
        }
    }
    minuteWork();
    if (kind_ == TimeStation::Chu) chuBursts();
    if (kind_ == TimeStation::Wwv) wwvBits();
}

// ---- CHU's bytes, read from the raw audio once each second is over: the first start bit after the mark tone found,
// then the ten bytes back to back (11 bits each: start, 8 data least first, 2 stop), each bit measured over its
// whole length against both tones (200 Hz apart: a whole bit is needed to tell them apart reliably)
bool TimeDecoder::chuSecondBytes(double s0, int* bytes) {
    const double R = rate_, bitN = R / 300;
    const long long L = static_cast<long long>(ring_.size());
    const long long n0 = static_cast<long long>(jsRound((s0 + 0.012 - anchorT_) * R));
    const long long n1 = static_cast<long long>(jsRound((s0 + 0.52 - anchorT_) * R));
    if (n0 < ringN_ - L || n1 > ringN_ || n0 < 0) return false;
    const long long len = n1 - n0;
    if (len <= 0 || static_cast<std::size_t>(len + 1) > mc_.size()) return false;
    const double fc = fc_;
    // running sums of the audio mixed with each tone, so any stretch's correlation is quick
    // (Addition to the JS: rotating phasors from the window's start instead of cos/sin of the absolute sample)
    const double wm = 2 * kPi * (fc + 2225) / R, wsp = 2 * kPi * (fc + 2025) / R;
    const double rmr = std::cos(wm), rmi = std::sin(wm), rsr = std::cos(wsp), rsi = std::sin(wsp);
    double pmr = 1, pmi = 0, psr = 1, psi = 0;
    mc_[0] = ms_[0] = sc_[0] = ss_[0] = 0;
    for (long long i = 0; i < len; ++i) {
        const double v = ring_[static_cast<std::size_t>((n0 + i) % L)];
        mc_[i + 1] = mc_[i] + v * pmr; ms_[i + 1] = ms_[i] + v * pmi;
        sc_[i + 1] = sc_[i] + v * psr; ss_[i + 1] = ss_[i] + v * psi;
        double t = pmr * rmr - pmi * rmi; pmi = pmr * rmi + pmi * rmr; pmr = t;
        t = psr * rsr - psi * rsi; psi = psr * rsi + psi * rsr; psr = t;
        if ((i & 1023) == 1023) {
            double m = std::sqrt(pmr * pmr + pmi * pmi); pmr /= m; pmi /= m;
            m = std::sqrt(psr * psr + psi * psi); psr /= m; psi /= m;
        }
    }
    struct Sums {
        const std::vector<double> &mc, &ms, &sc, &ss;
        double len;
        double pw(const std::vector<double>& A, const std::vector<double>& B, double a, double b) const {
            const std::size_t ia = static_cast<std::size_t>(std::max(0.0, std::min(len, jsRound(a))));
            const std::size_t ib = static_cast<std::size_t>(std::max(0.0, std::min(len, jsRound(b))));
            const double c = A[ib] - A[ia], d = B[ib] - B[ia];
            return c * c + d * d;
        }
        double disc(double a, double b) const { return pw(mc, ms, a, b) - pw(sc, ss, a, b); }   // > 0 mark, < 0 space
        double frameScore(double p, double bitN) const {
            double sc0 = 0;
            for (int j = 0; j < 10; ++j) {
                const double st = p + j * 11 * bitN;
                sc0 += sgn(-disc(st, st + bitN)) + sgn(disc(st + 9 * bitN, st + 10 * bitN)) + sgn(disc(st + 10 * bitN, st + 11 * bitN));
            }
            return sc0;
        }
    } S = {mc_, ms_, sc_, ss_, static_cast<double>(len)};
    // where the burst starts: every one of its ten bytes has a space start bit and two mark stop bits (30 checks), so
    // each candidate start is scored on how well that framing fits, and the best kept
    double best = -1, bs = -1e300;
    for (double p = 0; p + 110 * bitN <= len; p += bitN / 4) {
        const double sc0 = S.frameScore(p, bitN);
        if (sc0 > bs) { bs = sc0; best = p; }
    }
    if (best < 0 || bs < 22) return false;   // (no burst here: framing fits nowhere well)
    // then the start bit's edge exactly, near there: a whole bit before reads most mark and a whole bit after most space
    double edge = best, bv = -1e300;
    for (double p = best - bitN / 2; p <= best + bitN / 2; p += 1) {
        const double v = S.disc(p - bitN, p) - S.disc(p, p + bitN);
        if (v > bv) { bv = v; edge = p; }
    }
    best = S.frameScore(edge, bitN) >= bs - 1 ? edge : best;
    for (int j = 0; j < 10; ++j) {
        const double st = best + j * 11 * bitN;
        int v = 0;
        for (int i = 0; i < 8; ++i) {
            const double a = st + (1 + i) * bitN + 0.03 * bitN, b = st + (2 + i) * bitN - 0.03 * bitN;   // (nearly the whole bit)
            if (S.disc(a, b) > 0) v |= 1 << i;
        }
        bytes[j] = v;
    }
    return true;
}

void TimeDecoder::chuBursts() {
    if (!havePhase_ || tickQ_ < 6 || ring_.empty()) return;
    const double ph = tickPhase_;
    const double nowT = anchorT_ + static_cast<double>(ringN_) / rate_;
    const long long kEnd = static_cast<long long>(std::floor(nowT - ph - 0.53));   // the last second whose data window is over
    if (!chuSecSet_) { chuSec_ = kEnd; chuSecSet_ = true; return; }
    if (kEnd - chuSec_ > kMaxSeconds) chuSec_ = kEnd - kMaxSeconds;
    for (long long k = chuSec_ + 1; k <= kEnd; ++k) {
        int by[10];
        if (!chuSecondBytes(static_cast<double>(k) + ph, by)) continue;
        // format B (second 31): the second half the first inverted, exactly (the year is only taken when perfect)
        bool inv = true;
        for (int i = 0; i < 5; ++i) inv = inv && by[5 + i] == (by[i] ^ 255);
        if (inv) {
            const int y = (by[1] & 15) * 1000 + (by[1] >> 4) * 100 + (by[2] & 15) * 10 + (by[2] >> 4);
            if (y >= 2000 && y < 2100) year_ = y;
            continue;
        }
        // format A (32-39): each half on its own; a time is taken once two halves anywhere in the minute agree on the
        // day, hour and minute, and on where the minute started (a stray bit in one copy costs nothing)
        const double secStart = static_cast<double>(k) + ph;
        int keep = 0;
        for (int i = 0; i < aN_; ++i) if (secStart - aCands_[i].secStart < 70) aCands_[keep++] = aCands_[i];
        aN_ = keep;
        for (int h = 0; h < 2; ++h) {
            int d[10];
            bool ok = true;
            for (int i = 0; i < 5; ++i) { d[2 * i] = by[5 * h + i] & 15; d[2 * i + 1] = by[5 * h + i] >> 4; }
            for (int i = 0; i < 10; ++i) ok = ok && d[i] <= 9;
            if (d[0] != 6 || !ok) continue;
            const int doy = d[1] * 100 + d[2] * 10 + d[3], hh = d[4] * 10 + d[5], mm = d[6] * 10 + d[7], ss = d[8] * 10 + d[9];
            if (doy < 1 || doy > 366 || hh > 23 || mm > 59 || ss < 32 || ss > 39) continue;
            if (aN_ == 64) { std::memmove(aCands_, aCands_ + 1, sizeof(ACand) * 63); --aN_; }
            ACand c; c.k = k; c.doy = doy; c.hh = hh; c.mm = mm; c.ss = ss; c.minStart = secStart - ss; c.secStart = secStart;
            aCands_[aN_++] = c;
        }
        // (from a different second: one burst's two copies can't confirm each other, in case a fault hit both alike)
        const ACand* ok = 0;
        for (int i = 0; i < aN_ && !ok; ++i) {
            if (aCands_[i].k != k) continue;
            for (int j = 0; j < aN_; ++j) {
                const ACand& o = aCands_[j];
                if (o.k != k && o.doy == aCands_[i].doy && o.hh == aCands_[i].hh && o.mm == aCands_[i].mm &&
                    std::fabs(o.minStart - aCands_[i].minStart) < 0.5) { ok = &aCands_[i]; break; }
            }
        }
        if (!ok) continue;
        int year = year_;
        if (!year) { int mo, da, dy, hh, mi, se; utcFields(tEnd_, year, mo, da, dy, hh, mi, se); }   // (the computer's year)
        const double T = utcFromDayOfYear(year, ok->doy, ok->hh, ok->mm, ok->ss);
        newResult("CHU", T, secStart, year_ != 0);
    }
    chuSec_ = kEnd;
}

// WWV: each whole second (from the tick phase), classified by how long the 100 Hz pulse lasted; the minute found
// from the 800 ms minute tick; a minute's bits decoded when the minute is over
void TimeDecoder::wwvBits() {
    if (!havePhase_ || tickQ_ < 6 || !subCount_) return;
    const double ph = tickPhase_;
    const double latest = sub_[(subHead_ + kSubs - 1) % kSubs].t;
    const long long kEnd = static_cast<long long>(std::floor(latest - ph)) - 1;   // the last second fully heard
    if (!lastSecSet_) { lastSec_ = kEnd; lastSecSet_ = true; return; }
    if (kEnd - lastSec_ > kMaxSeconds) lastSec_ = kEnd - kMaxSeconds;
    for (long long k = lastSec_ + 1; k <= kEnd; ++k) {
        const double s0 = static_cast<double>(k) + ph;
        const double A = subWindow(s0 + 0.06, s0 + 0.16), B = subWindow(s0 + 0.26, s0 + 0.44);
        const double C = subWindow(s0 + 0.56, s0 + 0.74), N = subWindow(s0 + 0.85, s0 + 0.97);
        const double span = A - N, v1 = (B - N) / (span ? span : 1), v2 = (C - N) / (span ? span : 1);
        char bit = '?';
        double margin = 0;
        if (span > N * 2 && span > 0) {
            bit = v1 < 0.5 ? '0' : v2 < 0.5 ? '1' : 'P';
            // (Addition to the JS) how far the reading was from the thresholds
            margin = bit == '0' ? 0.5 - v1 : bit == '1' ? std::min(v1 - 0.5, 0.5 - v2) : std::min(v1 - 0.5, v2 - 0.5);
        }
        if (minuteKnown_ && std::fabs(minuteStart_ - s0) < 0.5 && !(minuteAtSet_ && minuteAt_ == k)) {
            minuteAt_ = k; minuteAtSet_ = true; std::memset(bits_, 0, sizeof bits_); std::memset(margin_, 0, sizeof margin_);
        }
        if (minuteAtSet_) {
            const long long sec = k - minuteAt_;
            if (sec >= 0 && sec < 60) { bits_[sec] = bit; margin_[sec] = margin; }
            if (sec == 59) wwvFrame(minuteAt_);
            if (sec >= 60) minuteAt_ += 60 * (sec / 60);   // (keeps counting if a minute tick is missed)
        }
    }
    lastSec_ = kEnd;
}

unsigned TimeDecoder::bitsRead() const {
    unsigned c = 0;
    for (int i = 0; i < 60; ++i) if (bits_[i] == '0' || bits_[i] == '1' || bits_[i] == 'P') ++c;
    return c;
}

void TimeDecoder::wwvFrame(long long m0) {
    struct B {
        const char* bits;
        // a BCD field: NaN (here: -1) when any of its bits is not a clean 0 or 1
        int num(const int* secs, const int* w, int n) const {
            int v = 0;
            for (int i = 0; i < n; ++i) {
                const char x = bits[secs[i]];
                if (x != '0' && x != '1') return -1000;
                if (x == '1') v += w[i];
            }
            return v;
        }
    } b = {bits_};
    static const int w8[] = {1, 2, 4, 8}, w4[] = {1, 2, 4}, w2[] = {1, 2};
    static const int sMu[] = {10, 11, 12, 13}, sMt[] = {15, 16, 17}, sHu[] = {20, 21, 22, 23}, sHt[] = {25, 26};
    static const int sDu[] = {30, 31, 32, 33}, sDt[] = {35, 36, 37, 38}, sDh[] = {40, 41};
    static const int sYu[] = {4, 5, 6, 7}, sYt[] = {51, 52, 53, 54};
    int markers = 0;
    for (int s = 9; s <= 59; s += 10) if (bits_[s] == 'P') ++markers;
    const int mm = b.num(sMu, w8, 4) + 10 * b.num(sMt, w4, 3);
    const int hh = b.num(sHu, w8, 4) + 10 * b.num(sHt, w2, 2);
    const int doy = b.num(sDu, w8, 4) + 10 * b.num(sDt, w8, 4) + 100 * b.num(sDh, w2, 2);
    const int yy = b.num(sYu, w8, 4) + 10 * b.num(sYt, w8, 4);
    if (markers < 5 || !(mm >= 0 && mm <= 59 && hh >= 0 && hh <= 23 && doy >= 1 && doy <= 366 && yy >= 0 && yy <= 99)) return;
    const double T = utcFromDayOfYear(2000 + yy, doy, hh, mm, 0), secStart = static_cast<double>(m0) + tickPhase_;
    const bool hadPrev = wwvPrevSet_;
    const double prevT = wwvPrevT_, prevS = wwvPrevSecStart_;
    wwvPrevSet_ = true; wwvPrevT_ = T; wwvPrevSecStart_ = secStart;
    // one frame with every marker right is taken; otherwise two in a row that agree
    // (Addition to the JS: a single frame only when every bit of the time was also read clearly, at least 0.25 from
    //  the thresholds. In noise (carrier 12 dB over the noise in 3 kHz, the subcarrier at 18%) the browser's rule
    //  took a frame with all six markers right but a day bit wrong about one time in four: a time 200 days out.
    //  Two frames in a row that agree cannot be wrong alike.)
    bool clean = markers == 6;
    static const int* const fields[] = {sMu, sMt, sHu, sHt, sDu, sDt, sDh, sYu, sYt};
    static const int counts[] = {4, 3, 4, 2, 4, 4, 2, 4, 4};
    for (int f = 0; f < 9 && clean; ++f)
        for (int i = 0; i < counts[f]; ++i) clean = clean && margin_[fields[f][i]] >= 0.25;
    const bool agree = hadPrev && std::fabs((secStart - prevS) - (T - prevT)) < 0.5;
    if (!clean && !agree) return;
    newResult(stationNow(), T, secStart, true);
}

void TimeDecoder::newResult(const std::string& station, double T, double secStart, bool yearKnown) {
    const bool sameMinute = result_.valid && result_.station == station && std::floor(result_.T / 60) == std::floor(T / 60);
    result_.valid = true; result_.T = T; result_.offset = secStart - T; result_.at = tEnd_;
    result_.yearKnown = yearKnown; result_.station = station;
    int y, mo, d, doy, hh, mi, se;
    utcFields(T, y, mo, d, doy, hh, mi, se);
    char buf[200];
    std::snprintf(buf, sizeof buf, "%04d-%02d-%02d %02d:%02d:%02d UTC (day %d)", y, mo, d, hh, mi, se, doy);
    result_.text = buf;
    ++results_;
    // (Addition to the JS) a text line, one per minute per station (CHU confirms each second 33-39)
    if (!sameMinute) {
        std::snprintf(buf, sizeof buf, "%s %s, PC clock %+.3f s \xC2\xB1" "0.005%s\n", station.c_str(), result_.text.c_str(),
                      result_.offset, yearKnown ? "" : " (year from the PC clock)");
        out_ += buf;
    }
}

TimeReading TimeDecoder::lastFull() const {
    TimeReading r;
    if (!result_.valid) return r;
    r.level = TimeLevel::Full; r.offset = result_.offset; r.error = 0.005; r.stationTime = result_.T;
    r.at = result_.at; r.station = result_.station; r.text = result_.text;
    return r;
}

// what's best to set the clock from, now: the station's code (full time), else its minute tick (to the second,
// assuming the clock is within 30 s), else its second ticks (the fraction of a second only)
TimeReading TimeDecoder::best(double cur) const {
    if (result_.valid && tEnd_ - result_.at < 600) return lastFull();
    TimeReading r;
    double mo, to;
    // (the minute tick gives the seconds within the minute: whole minutes are kept from the correction in use; the
    //  second ticks give only the fraction, so whole seconds are kept)
    if (minuteSeen_ && minuteOffset(mo)) {
        r.level = TimeLevel::Minute; r.offset = mo + 60 * jsRound((cur - mo) / 60);
    } else if (tickOffset(to)) {
        r.level = TimeLevel::Ticks; r.offset = to + jsRound(cur - to);
    } else {
        return r;
    }
    r.error = 0.005; r.at = tEnd_; r.station = stationNow(); r.stationTime = tEnd_ - r.offset;
    return r;
}

Status TimeDecoder::status() const {
    Status s;
    const TimeReading r = best(0);
    const double snr = carrierDb_ - noiseDb_;
    double to;
    const bool ticks = tickOffset(to);
    s.signal = fcN_ > 0 && snr > 10;
    s.locked = ticks;
    s.snrDb = fcN_ ? snr : 0;
    s.quality = std::min(1.0, std::max(0.0, tickQ_ / 20));
    s.toneHz = fc_;
    s.characters = results_;
    s.station = stationNow();
    s.timeValid = r.level == TimeLevel::Full;
    if (r.level != TimeLevel::None) {
        s.clockOffset = -r.offset;               // (station - computer)
        s.clockError = r.error;
        s.stationTime = tEnd_ - r.offset;
    }
    const char* lv = r.level == TimeLevel::Full ? "time code" : r.level == TimeLevel::Minute ? "minute marker"
                   : r.level == TimeLevel::Ticks ? "ticks" : "listening";
    s.mode = s.station + " " + lv;
    char buf[200];
    std::string info;
    if (!fcN_) info = "listening";
    else if (snr < 10) info = "no carrier";
    else {
        std::snprintf(buf, sizeof buf, "carrier %.0f dB", snr);
        info = buf;
        if (ticks) { std::snprintf(buf, sizeof buf, " · ticks %s (q %.1f)", tickQ_ > 8 ? "clear" : "weak", tickQ_); info += buf; }
        else info += " · no ticks yet";
        if (minuteSeen_) info += " · minute found";
        if (kind_ == TimeStation::Wwv && minuteAtSet_) { std::snprintf(buf, sizeof buf, " · %u bits", bitsRead()); info += buf; }
    }
    if (r.level != TimeLevel::None) {
        std::snprintf(buf, sizeof buf, " · PC clock %+.3f s%s", r.offset,
                      r.level == TimeLevel::Ticks ? " (fraction only)" : r.level == TimeLevel::Minute ? " (to the second)" : "");
        info += buf;
    }
    s.info = info;
    return s;
}

}  // namespace dec
