// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/ax25/modem.h
//  Layer:  Library liboffgrid-decoders (internal header of the AX.25 decoder)
//
//  Where this file comes from
//  --------------------------
//  Copied from AX25Toolkit by Gustavo Campos (kiss_modem/modem.h,
//  https://github.com/solariun/AX25Toolkit), used with the author's
//  permission.  Its DSP is derived from Dire Wolf by John Langner, WB2OSZ
//  (https://github.com/wb2osz/direwolf, GPLv2 or later); the original
//  header comment is kept below.
//
//  What this file does
//  -------------------
//  The demodulator: audio samples in, NRZI-decoded bits out (one callback per
//  bit), for AFSK 1200 (Bell 202, VHF APRS), AFSK 300 (HF packet) and G3RUH
//  9600 (baseband from the FM discriminator).  It does not know about HDLC
//  flags or frames: hdlc.h does that.
//
//  Changes from AX25Toolkit (all marked "OFFgrid:" in modem.cpp):
//    - namespace modem -> dec::modem (so a program can link both libraries);
//    - float input (nominal -1..1) instead of int16 / 16384;
//    - several slicers (Dire Wolf's "multiple slicers", the "+" of its MODEM setting):
//      slicer 0 is AX25Toolkit's AGC-normalised mark - space; slicers 1..n-1
//      compare the raw mark and space strengths with the space tone scaled
//      by 0.5 .. 4 (as Dire Wolf's demod_afsk.c does), so a radio whose
//      de-emphasis leaves 2200 Hz weaker (or a flat one) is still read.  Each
//      slicer has its own bit clock (PLL), DCD and callback index;
//    - the filter delay lines are circular (twice-written) buffers instead of
//      a memmove per sample: the same numbers, far less memory traffic;
//    - the G3RUH descrambler state is unsigned and masked (the original
//      shifted a signed int left forever: undefined behaviour in C++);
//    - the PLL phase magnitude is taken in unsigned arithmetic (the original
//      negated INT_MIN in a corner case);
//    - the Modulator class is not copied: Ax25Encoders.cpp synthesises the
//      audio with fractional bit timing (the original used an integer number
//      of samples per bit, 2% fast at 44100 Hz).
//
//  How it fits in
//  --------------
//  Used by Ax25.cpp (dec::Ax25Decoder) only; standard library only.
//
//  Contributing
//  ------------
//  Keep it close to AX25Toolkit's modem (so fixes can flow both ways) and to
//  Dire Wolf's algorithms; no allocation per sample; strict C++11.  Run
//  tests/test_decoders_ax25.cpp after any change.
// ============================================================================
//
// Modem demodulator — AFSK 1200/300, GMSK 9600, PSK 2400/4800
// DSP algorithms derived from Dire Wolf by John Langner, WB2OSZ (GPLv2)
// Original: https://github.com/wb2osz/direwolf
// Simplified for kiss_modem: single-decoder, C++ classes, no global state
#ifndef OFFGRID_DECODERS_AX25_MODEM_H
#define OFFGRID_DECODERS_AX25_MODEM_H

#include <cstdint>
#include <functional>

namespace dec {
namespace modem {

enum Type {
    // Implemented — selectable via -s SPEED
    AFSK_1200,   // Bell 202: mark=1200 Hz, space=2200 Hz, 1200 baud
    AFSK_300,    // HF: mark=1600 Hz, space=1800 Hz, 300 baud
    GMSK_9600,   // G3RUH scrambled baseband, 9600 baud

    // Planned — DSP init exists but CLI not yet wired
    PSK_2400,    // QPSK V.26B, 2400 bps (future)
    PSK_4800,    // 8PSK, 4800 bps (future)
    AIS,         // 9600 baud baseband, maritime (future)
    EAS,         // AFSK 521 baud, Emergency Alert (future)
};

// Maximum FIR filter taps
static const int MAX_FILTER = 480;

// OFFgrid: the most slicers one demodulator runs (Dire Wolf allows 9).
static const int MAX_SLICERS = 5;

// ---------------------------------------------------------------------------
//  Demodulator
// ---------------------------------------------------------------------------
class Demodulator {
public:
    // OFFgrid: the callback also says which slicer produced the bit.
    typedef std::function<void(int slicer, int bit)> BitCb;

    // OFFgrid: num_slicers 1..MAX_SLICERS (AFSK only; 9600 always uses 1).
    void init(Type type, int sample_rate, int num_slicers = 1);
    // OFFgrid: float input, nominal -1..1 (must be finite: the caller checks).
    void process_sample(float sample);
    void set_on_bit(BitCb cb) { on_bit_ = cb; }
    void set_type(Type type);   // runtime switch (re-initializes filters)
    bool dcd() const;                       // OFFgrid: any slicer has DCD
    bool dcd(int slicer) const { return sl_[slicer].dcd_; }
    int  dcd_count() const;                 // OFFgrid: best good-transition count (0..32)
    int  num_slicers() const { return num_slicers_; }
    int  sample_rate() const { return sample_rate_; }

private:
    // OFFgrid: a delay line written twice (at pos and pos + taps) so the last
    // `taps` samples are always contiguous: data()[j] is the sample j steps
    // ago, as the original memmove'd buffer had it.
    struct Delay {
        float buf[2 * MAX_FILTER];
        int   pos;
        int   taps;
        void push(float v) {
            pos = (pos == 0 ? taps : pos) - 1;
            buf[pos] = v;
            buf[pos + taps] = v;
        }
        const float* data() const { return buf + pos; }
    };

    // OFFgrid: the bit clock and DCD, one per slicer (were members).
    struct Slicer {
        int   data_clock_pll_;
        int   prev_d_c_pll_;
        int   prev_demod_data_;     // per-sample: for PLL transition detection
        int   prev_demod_bit_;      // per-bit: for NRZI decode at sample points
        unsigned int scramble_state_;
        bool  dcd_;
        unsigned int dcd_shreg_;     // 32-bit shift register (1=good, 0=bad transition)
        int          dcd_count_;     // popcount of dcd_shreg_ (cached)
        int          dcd_missing_;   // samples since last transition (detect silence)
    };

    BitCb on_bit_;
    Type  type_ = AFSK_1200;
    int   sample_rate_ = 44100;
    int   num_slicers_ = 1;
    float space_gain_[MAX_SLICERS];  // OFFgrid: per slicer (slicer 0 uses the AGC)
    int   silence_thresh_ = 0;

    // ----- AFSK state -----
    struct {
        // Local oscillators (DDS)
        unsigned int m_osc_phase, m_osc_delta;  // mark
        unsigned int s_osc_phase, s_osc_delta;  // space

        // I/Q sample buffers for mark and space
        Delay m_I_raw;
        Delay m_Q_raw;
        Delay s_I_raw;
        Delay s_Q_raw;

        // Pre-filter (bandpass)
        Delay raw_cb;
        float pre_filter[MAX_FILTER];
        int   pre_filter_taps;
        bool  use_prefilter;

        // Low-pass / RRC filter
        float lp_filter[MAX_FILTER];
        int   lp_filter_taps;

        // AGC
        float agc_fast_attack, agc_slow_decay;
        float m_peak, m_valley;
        float s_peak, s_valley;
    } afsk_;
    // OFFgrid: AX25Toolkit's "Profile B" (FM discriminator, disabled there)
    // is not copied.

    // ----- 9600 baud state -----
    struct {
        Delay audio_buf;
        float lp_filter[MAX_FILTER];
        int   lp_filter_taps;

        // AGC
        float agc_fast_attack, agc_slow_decay;
        float peak, valley;
    } bb_;

    // ----- PLL (shared settings) -----
    int   pll_step_per_sample_;
    float pll_locked_inertia_;
    float pll_searching_inertia_;

    // ----- DCD tracking (from Direwolf: transition quality over 32 symbols) -----
    // A good transition occurs when the PLL phase is near zero at a data transition.
    // We track the last 32 transitions: 1=good, 0=bad.
    // DCD ON when >= DCD_THRESH_ON good transitions out of 32.
    // DCD OFF when < DCD_THRESH_OFF.
    static const int DCD_THRESH_ON  = 25;  // need 25/32 good transitions to lock
    static const int DCD_THRESH_OFF = 10;  // drop below 10/32 to unlock

    Slicer sl_[MAX_SLICERS];

    // Cosine table
    float cos256_[256];

    // Helpers
    void init_afsk(int baud, int mark, int space);
    void init_9600();
    void process_afsk(float fsam);
    void process_9600(float fsam);
    void nudge_pll(int slicer, float demod_out);

    float fcos256(unsigned int phase) const { return cos256_[(phase >> 24) & 0xff]; }
    float fsin256(unsigned int phase) const { return cos256_[((phase >> 24) - 64) & 0xff]; }

    // DSP primitives
    static float convolve(const float* data, const float* filter, int taps);
    static float agc(float in, float fast_attack, float slow_decay, float* peak, float* valley);
    static int   descramble(int in, unsigned int* state);

    // Filter generation
    static void gen_bandpass(float f1, float f2, float* filter, int taps);
    static void gen_lowpass(float fc, float* filter, int taps);
    static void gen_rrc_lowpass(float* filter, int taps, float rolloff, float samples_per_symbol);
};

}  // namespace modem
}  // namespace dec

#endif
