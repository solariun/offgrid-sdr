// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/ax25/modem.cpp
//  Layer:  Library liboffgrid-decoders (internal to the AX.25 decoder)
//
//  Where this file comes from
//  --------------------------
//  Copied from AX25Toolkit by Gustavo Campos (kiss_modem/modem.cpp,
//  https://github.com/solariun/AX25Toolkit), used with the author's
//  permission; DSP derived from Dire Wolf by John Langner, WB2OSZ
//  (https://github.com/wb2osz/direwolf, GPLv2 or later).  Original header
//  comment kept below.
//
//  What this file does
//  -------------------
//  The AFSK / G3RUH demodulator of modem.h.  Per sample (AFSK):
//    1. band-pass pre-filter around the two tones;
//    2. mix with a mark and a space local oscillator, low-pass (RRC) each
//       I and Q: the strength of each tone over about a bit;
//    3. slicer 0: each strength through its own AGC, mark - space;
//       slicers 1..n-1: mark - gain * space (no AGC);
//    4. per slicer, a bit clock (a 32-bit phase that wraps once a bit) is
//       pulled towards each transition of the sliced signal (strongly while
//       searching, gently once DCD is on); when it wraps, the bit is sampled
//       and NRZI-decoded (no change = 1) and handed to the callback;
//    5. DCD: of the last 32 transitions, how many fell near where the clock
//       expected them (25 = on, under 10 = off; none for 4 bits = off).
//  9600: low-pass, AGC, the same clock, then G3RUH descrambling and NRZI.
//
//  Changes from AX25Toolkit: see modem.h; each is marked "OFFgrid:".
//
//  How it fits in
//  --------------
//  Implements modem.h for Ax25.cpp.  Standard library only.
//
//  Contributing
//  ------------
//  No allocation per sample.  Run tests/test_decoders_ax25.cpp.
// ============================================================================
//
// Modem demodulator & modulator — AFSK 1200/300, GMSK 9600
// DSP algorithms derived from Dire Wolf by John Langner, WB2OSZ (GPLv2)
// Original: https://github.com/wb2osz/direwolf
// Simplified for kiss_modem: single-decoder, C++ classes, no global state
#include "modem.h"
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// PLL: 32-bit phase accumulator wraps at 2^32
static const double TICKS_PER_PLL_CYCLE = 256.0 * 256.0 * 256.0 * 256.0;

namespace dec {
namespace modem {

// ===========================================================================
//  DSP primitives
// ===========================================================================

// OFFgrid: push_sample (a memmove per sample) is replaced by Delay::push in
// modem.h.

float Demodulator::convolve(const float* data, const float* filter, int taps) {
    float sum = 0.0f;
    for (int j = 0; j < taps; j++)
        sum += filter[j] * data[j];
    return sum;
}

float Demodulator::agc(float in, float fast_attack, float slow_decay, float* peak, float* valley) {
    if (in >= *peak)
        *peak = in * fast_attack + *peak * (1.0f - fast_attack);
    else
        *peak = in * slow_decay + *peak * (1.0f - slow_decay);

    if (in <= *valley)
        *valley = in * fast_attack + *valley * (1.0f - fast_attack);
    else
        *valley = in * slow_decay + *valley * (1.0f - slow_decay);

    float x = in;
    if (x > *peak) x = *peak;
    if (x < *valley) x = *valley;

    if (*peak > *valley)
        return (x - 0.5f * (*peak + *valley)) / (*peak - *valley);
    return 0.0f;
}

// OFFgrid: unsigned state, kept to the 17 bits the polynomial uses (the
// original shifted a signed int left without bound).
int Demodulator::descramble(int in, unsigned int* state) {
    int out = (int)((unsigned int)in ^ (*state >> 16) ^ (*state >> 11)) & 1;
    *state = ((*state << 1) | (unsigned int)(in & 1)) & 0x1FFFFu;
    return out;
}

// ===========================================================================
//  Filter generation (from Direwolf dsp.c)
// ===========================================================================

static float window_func(int j, int size) {
    // Hamming window
    return 0.54f - 0.46f * std::cos(2.0f * (float)M_PI * (float)j / (float)(size - 1));
}

void Demodulator::gen_lowpass(float fc, float* filter, int taps) {
    float center = 0.5f * (float)(taps - 1);
    for (int j = 0; j < taps; j++) {
        float t = (float)j - center;
        if (std::fabs(t) < 0.001f)
            filter[j] = 2.0f * fc;
        else
            filter[j] = std::sin(2.0f * (float)M_PI * fc * t) / ((float)M_PI * t);
        filter[j] *= window_func(j, taps);
    }
    // Normalize
    float sum = 0;
    for (int j = 0; j < taps; j++) sum += filter[j];
    if (std::fabs(sum) > 0.001f)
        for (int j = 0; j < taps; j++) filter[j] /= sum;
}

void Demodulator::gen_bandpass(float f1, float f2, float* filter, int taps) {
    float center = 0.5f * (float)(taps - 1);
    for (int j = 0; j < taps; j++) {
        float t = (float)j - center;
        float h;
        if (std::fabs(t) < 0.001f)
            h = 2.0f * (f2 - f1);
        else
            h = (std::sin(2.0f * (float)M_PI * f2 * t) - std::sin(2.0f * (float)M_PI * f1 * t)) / ((float)M_PI * t);
        filter[j] = h * window_func(j, taps);
    }
    // Normalize for unity gain at center frequency
    float w = 2.0f * (float)M_PI * (f1 + f2) / 2.0f;
    float sum = 0;
    for (int j = 0; j < taps; j++)
        sum += filter[j] * std::cos(w * ((float)j - 0.5f * (float)(taps - 1)));
    if (std::fabs(sum) > 0.001f)
        for (int j = 0; j < taps; j++) filter[j] /= sum;
}

void Demodulator::gen_rrc_lowpass(float* filter, int taps, float rolloff, float sps) {
    float center = 0.5f * (float)(taps - 1);
    for (int j = 0; j < taps; j++) {
        float t = ((float)j - center) / sps;  // in symbol periods
        float h;
        if (std::fabs(t) < 0.0001f) {
            h = 1.0f - rolloff + 4.0f * rolloff / (float)M_PI;
        } else if (std::fabs(std::fabs(t) - 1.0f / (4.0f * rolloff)) < 0.001f && rolloff > 0.001f) {
            h = (rolloff / std::sqrt(2.0f)) *
                ((1.0f + 2.0f / (float)M_PI) * std::sin((float)M_PI / (4.0f * rolloff)) +
                 (1.0f - 2.0f / (float)M_PI) * std::cos((float)M_PI / (4.0f * rolloff)));
        } else {
            float num = std::sin((float)M_PI * t * (1.0f - rolloff)) +
                        4.0f * rolloff * t * std::cos((float)M_PI * t * (1.0f + rolloff));
            float den = (float)M_PI * t * (1.0f - (4.0f * rolloff * t) * (4.0f * rolloff * t));
            h = (std::fabs(den) > 0.0001f) ? num / den : 1.0f;
        }
        filter[j] = h * window_func(j, taps);
    }
    // Normalize
    float sum = 0;
    for (int j = 0; j < taps; j++) sum += filter[j];
    if (std::fabs(sum) > 0.001f)
        for (int j = 0; j < taps; j++) filter[j] /= sum;
}

// ===========================================================================
//  Demodulator init
// ===========================================================================

void Demodulator::init(Type type, int sample_rate, int num_slicers) {
    type_ = type;
    sample_rate_ = sample_rate;
    // OFFgrid: per-slicer clock and DCD state.
    std::memset(sl_, 0, sizeof(sl_));
    num_slicers_ = std::max(1, std::min(MAX_SLICERS, num_slicers));
    if (type == GMSK_9600 || type == AIS) num_slicers_ = 1;
    // OFFgrid: space-tone gains for slicers 1..n-1, 0.5 .. 4 evenly spaced
    // in the logarithm (Dire Wolf's MIN_G / MAX_G); slicer 0 uses the AGC.
    space_gain_[0] = 1.0f;
    for (int k = 1; k < num_slicers_; ++k)
        space_gain_[k] = num_slicers_ == 2 ? 1.0f
                       : 0.5f * (float)std::pow(8.0, (double)(k - 1) / (double)(num_slicers_ - 2));

    // Init cosine table
    for (int j = 0; j < 256; j++)
        cos256_[j] = std::cos((float)j * 2.0f * (float)M_PI / 256.0f);

    switch (type) {
        case AFSK_1200: init_afsk(1200, 1200, 2200); break;
        case AFSK_300:  init_afsk(300, 1600, 1800);  break;
        case EAS:       init_afsk(521, 2083, 1563);   break;
        case GMSK_9600:
        case AIS:       init_9600(); break;
        default:        init_afsk(1200, 1200, 2200); break;
    }

    // No transitions for too long → DCD off (silence / no signal)
    // Threshold: 4 bit periods worth of samples with no transition
    // (OFFgrid: computed once here instead of every sample)
    silence_thresh_ = (sample_rate_ * 4) / (type_ == GMSK_9600 ? 9600 :
                      type_ == AFSK_300 ? 300 : 1200);
}

void Demodulator::set_type(Type type) {
    init(type, sample_rate_, num_slicers_);
}

bool Demodulator::dcd() const {
    for (int k = 0; k < num_slicers_; ++k)
        if (sl_[k].dcd_) return true;
    return false;
}

int Demodulator::dcd_count() const {
    int best = 0;
    for (int k = 0; k < num_slicers_; ++k) best = std::max(best, sl_[k].dcd_count_);
    return best;
}

void Demodulator::init_afsk(int baud, int mark, int space) {
    std::memset(&afsk_, 0, sizeof(afsk_));

    // PLL step
    pll_step_per_sample_ = (int)std::floor(TICKS_PER_PLL_CYCLE * (double)baud / (double)sample_rate_ + 0.5);
    pll_locked_inertia_ = 0.74f;
    pll_searching_inertia_ = 0.50f;

    // Profile A: Mark/Space local oscillators
    afsk_.m_osc_phase = 0;
    afsk_.m_osc_delta = (unsigned int)std::floor(std::pow(2.0, 32.0) * (double)mark / (double)sample_rate_ + 0.5);
    afsk_.s_osc_phase = 0;
    afsk_.s_osc_delta = (unsigned int)std::floor(std::pow(2.0, 32.0) * (double)space / (double)sample_rate_ + 0.5);

    // Pre-filter (bandpass around mark/space)
    afsk_.use_prefilter = true;
    float prebaud = (baud > 600) ? 0.155f : 0.87f;
    float prefilt_sym = (baud > 600) ? (383.0f * 1200.0f / 44100.0f) : 1.857f;

    afsk_.pre_filter_taps = ((int)(prefilt_sym * (float)sample_rate_ / (float)baud)) | 1;
    if (afsk_.pre_filter_taps > MAX_FILTER) afsk_.pre_filter_taps = (MAX_FILTER - 1) | 1;

    float f1 = ((float)std::min(mark, space) - prebaud * (float)baud) / (float)sample_rate_;
    float f2 = ((float)std::max(mark, space) + prebaud * (float)baud) / (float)sample_rate_;
    if (f1 < 0.001f) f1 = 0.001f;
    gen_bandpass(f1, f2, afsk_.pre_filter, afsk_.pre_filter_taps);

    // RRC low-pass filter (Profile A values)
    float rrc_width = 2.80f;
    float rrc_rolloff = 0.20f;
    afsk_.lp_filter_taps = ((int)(rrc_width * (float)sample_rate_ / (float)baud)) | 1;
    if (afsk_.lp_filter_taps > MAX_FILTER) afsk_.lp_filter_taps = (MAX_FILTER - 1) | 1;
    gen_rrc_lowpass(afsk_.lp_filter, afsk_.lp_filter_taps, rrc_rolloff, (float)sample_rate_ / (float)baud);

    // OFFgrid: delay line lengths.
    afsk_.raw_cb.taps = afsk_.pre_filter_taps;
    afsk_.m_I_raw.taps = afsk_.m_Q_raw.taps = afsk_.lp_filter_taps;
    afsk_.s_I_raw.taps = afsk_.s_Q_raw.taps = afsk_.lp_filter_taps;

    // AGC
    afsk_.agc_fast_attack = 0.70f;
    afsk_.agc_slow_decay = 0.000090f;
    afsk_.m_peak = afsk_.m_valley = 0;
    afsk_.s_peak = afsk_.s_valley = 0;
}

void Demodulator::init_9600() {
    std::memset(&bb_, 0, sizeof(bb_));

    pll_step_per_sample_ = (int)std::floor(TICKS_PER_PLL_CYCLE * 9600.0 / (double)sample_rate_ + 0.5);
    pll_locked_inertia_ = 0.74f;
    pll_searching_inertia_ = 0.50f;

    // Low-pass filter — wider cutoff for baseband (need to pass the data bandwidth)
    float fc = 9600.0f * 0.7f / (float)sample_rate_;
    bb_.lp_filter_taps = ((int)(0.8f * (float)sample_rate_ / 9600.0f)) | 1;
    if (bb_.lp_filter_taps < 3) bb_.lp_filter_taps = 3;
    if (bb_.lp_filter_taps > MAX_FILTER) bb_.lp_filter_taps = (MAX_FILTER - 1) | 1;
    gen_lowpass(fc, bb_.lp_filter, bb_.lp_filter_taps);
    bb_.audio_buf.taps = bb_.lp_filter_taps;   // OFFgrid

    bb_.agc_fast_attack = 0.70f;
    bb_.agc_slow_decay = 0.000090f;
    bb_.peak = bb_.valley = 0;
}

// ===========================================================================
//  Demodulator process sample
// ===========================================================================

void Demodulator::process_sample(float fsam) {
    // OFFgrid: float in (the original took int16 and divided by 16384).
    switch (type_) {
        case AFSK_1200:
        case AFSK_300:
        case EAS:
            process_afsk(fsam);
            break;
        case GMSK_9600:
        case AIS:
            process_9600(fsam);
            break;
        default:
            process_afsk(fsam);
            break;
    }
}

void Demodulator::process_afsk(float fsam) {
    // Optional bandpass pre-filter
    if (afsk_.use_prefilter) {
        afsk_.raw_cb.push(fsam);
        fsam = convolve(afsk_.raw_cb.data(), afsk_.pre_filter, afsk_.pre_filter_taps);
    }

    // ── Profile A: Mark/Space amplitude comparison ──
    afsk_.m_I_raw.push(fsam * fcos256(afsk_.m_osc_phase));
    afsk_.m_Q_raw.push(fsam * fsin256(afsk_.m_osc_phase));
    afsk_.m_osc_phase += afsk_.m_osc_delta;

    afsk_.s_I_raw.push(fsam * fcos256(afsk_.s_osc_phase));
    afsk_.s_Q_raw.push(fsam * fsin256(afsk_.s_osc_phase));
    afsk_.s_osc_phase += afsk_.s_osc_delta;

    float m_I = convolve(afsk_.m_I_raw.data(), afsk_.lp_filter, afsk_.lp_filter_taps);
    float m_Q = convolve(afsk_.m_Q_raw.data(), afsk_.lp_filter, afsk_.lp_filter_taps);
    float m_amp = std::sqrt(m_I * m_I + m_Q * m_Q);

    float s_I = convolve(afsk_.s_I_raw.data(), afsk_.lp_filter, afsk_.lp_filter_taps);
    float s_Q = convolve(afsk_.s_Q_raw.data(), afsk_.lp_filter, afsk_.lp_filter_taps);
    float s_amp = std::sqrt(s_I * s_I + s_Q * s_Q);

    float m_norm = agc(m_amp, afsk_.agc_fast_attack, afsk_.agc_slow_decay,
                       &afsk_.m_peak, &afsk_.m_valley);
    float s_norm = agc(s_amp, afsk_.agc_fast_attack, afsk_.agc_slow_decay,
                       &afsk_.s_peak, &afsk_.s_valley);

    nudge_pll(0, m_norm - s_norm);

    // OFFgrid: the other slicers (as Dire Wolf's demod_afsk.c with several
    // slicers): the raw strengths, the space tone scaled.
    for (int k = 1; k < num_slicers_; ++k)
        nudge_pll(k, m_amp - s_amp * space_gain_[k]);
}

void Demodulator::process_9600(float fsam) {
    bb_.audio_buf.push(fsam);
    float filtered = convolve(bb_.audio_buf.data(), bb_.lp_filter, bb_.lp_filter_taps);

    float demod_out = agc(filtered, bb_.agc_fast_attack, bb_.agc_slow_decay,
                          &bb_.peak, &bb_.valley);

    nudge_pll(0, demod_out);
}

void Demodulator::nudge_pll(int k, float demod_out) {
    Slicer& S = sl_[k];   // OFFgrid: per-slicer state
    S.prev_d_c_pll_ = S.data_clock_pll_;

    // Advance PLL (unsigned add to avoid signed overflow)
    S.data_clock_pll_ = (int)((unsigned int)S.data_clock_pll_ + (unsigned int)pll_step_per_sample_);

    // Count samples since last transition (for DCD silence detection)
    S.dcd_missing_++;

    // Overflow (positive → negative): sample a data bit
    if (S.data_clock_pll_ < 0 && S.prev_d_c_pll_ > 0) {
        int raw_bit = demod_out > 0 ? 1 : 0;

        int dbit;
        if (type_ == GMSK_9600 || type_ == AIS) {
            int descrambled = descramble(raw_bit, &S.scramble_state_);
            dbit = (descrambled == S.prev_demod_bit_) ? 1 : 0;
            S.prev_demod_bit_ = descrambled;
        } else {
            dbit = (raw_bit == S.prev_demod_bit_) ? 1 : 0;
            S.prev_demod_bit_ = raw_bit;
        }

        if (on_bit_) on_bit_(k, dbit);
    }

    // --- DCD tracking at every data transition ---
    int demod_data = demod_out > 0 ? 1 : 0;
    if (demod_data != S.prev_demod_data_) {
        // Transition detected — evaluate quality
        // A "good" transition has PLL phase near zero (mid-point between samples)
        // PLL is signed 32-bit: 0 = perfect mid-bit, ±2^31 = worst
        // Good = PLL magnitude < 25% of full cycle
        // (OFFgrid: magnitude in unsigned arithmetic: -INT_MIN is undefined)
        unsigned int pll_mag = S.data_clock_pll_ < 0 ? 0u - (unsigned int)S.data_clock_pll_
                                                     : (unsigned int)S.data_clock_pll_;
        bool good = (pll_mag < 0x40000000u);  // < 25% of 2^32

        // Shift into 32-bit quality register
        S.dcd_shreg_ <<= 1;
        if (good) S.dcd_shreg_ |= 1;

        // Popcount (count good transitions in last 32)
        unsigned int v = S.dcd_shreg_;
        int cnt = 0;
        while (v) { cnt += (int)(v & 1); v >>= 1; }
        S.dcd_count_ = cnt;

        // Hysteresis: DCD ON at >= 25/32, OFF at < 10/32
        if (!S.dcd_ && S.dcd_count_ >= DCD_THRESH_ON)
            S.dcd_ = true;
        else if (S.dcd_ && S.dcd_count_ < DCD_THRESH_OFF)
            S.dcd_ = false;

        S.dcd_missing_ = 0;

        // PLL nudge
        if (S.dcd_)
            S.data_clock_pll_ = (int)((float)S.data_clock_pll_ * pll_locked_inertia_);
        else
            S.data_clock_pll_ = (int)((float)S.data_clock_pll_ * pll_searching_inertia_);
    }

    // No transitions for too long → DCD off (silence / no signal)
    if (S.dcd_missing_ > silence_thresh_) {
        S.dcd_ = false;
        S.dcd_shreg_ = 0;
        S.dcd_count_ = 0;
        S.dcd_missing_ = silence_thresh_;   // OFFgrid: bounded (no overflow after hours of silence)
    }

    S.prev_demod_data_ = demod_data;
}

// OFFgrid: the Modulator is not copied (see modem.h).

}  // namespace modem
}  // namespace dec
