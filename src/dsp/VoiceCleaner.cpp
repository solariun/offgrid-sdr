// ============================================================================
//  offgrid-dsp - liboffgrid-dsp, the signal processing library of OFFgrid-SDR TUI.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr) by diddy-boy.
//  SPDX-License-Identifier: GPL-3.0-or-later AND BSD-3-Clause
//  (dsp::rnnoise below is a C++11 port of RNNoise; its BSD notice is kept
//   at the end of this header block, as its terms require)
// ----------------------------------------------------------------------------
//  File:   src/dsp/VoiceCleaner.cpp
//  Layer:  Library liboffgrid-dsp
//
//  What this file does
//  -------------------
//  1. dsp::rnnoise::Denoiser: RNNoise (Jean-Marc Valin, Xiph.Org Foundation
//     and Mozilla; https://github.com/xiph/rnnoise), release v0.1.1, the last
//     "classic" one and the one whose weights the browser app's WASM carries,
//     ported from C to C++11.  The C files and what came from each:
//       kiss_fft.c / _kiss_fft_guts.h   KissFft (mixed-radix FFT, 960 points)
//       celt_lpc.c                      lpc(), autocorr()
//       pitch.c / pitch.h               pitchDownsample(), pitchSearch(),
//                                       removeDoubling(), xcorr kernels
//       rnn.c / tansig_table.h          tansig/sigmoid, dense and GRU layers
//       denoise.c                       band energies, features, pitch
//                                       filter, the frame pipeline
//     The float build of the C code is followed exactly (same operations in
//     the same order, the same float/double promotions), so the result
//     matches a C build of RNNoise to float precision.
//  2. dsp::VoiceCleaner: OFFgrid-SDR's AiVoiceCleaner (index.html, "AI voice
//     cleaner begin"): audio x 32768 into 480-sample frames, each frame
//     cleaned and mixed with the original by the strength, queued, and the
//     caller's block filled from the queue (silence while the first frame
//     fills).
//
//  How it fits in
//  --------------
//  Declared in VoiceCleaner.h, weights in RnnoiseModel.cpp, used by
//  Receiver.cpp (ReceiverConfig::voiceCleaner).  Resampling for rates other
//  than 48 kHz uses detail::Resampler (Filters.h).
//
//  Contributing
//  ------------
//  Keep the Denoiser a faithful port: tests/test_filters.cpp compares its
//  per-frame gains and output with the reference C code.  Comments marked
//  "(C: ...)" name the C function or macro a piece of code comes from.
//  Anything that is not in RNNoise or in the JS is marked "Addition to the JS".
// ----------------------------------------------------------------------------
//  RNNoise licence (BSD-3-Clause; the Opus-derived files kiss_fft.c,
//  pitch.c, celt_lpc.c and rnn.c carry the same terms without the third
//  clause), kept as its terms require:
//
//  Copyright (c) 2017, Mozilla
//  Copyright (c) 2018, Gregor Richards
//  Copyright (c) 2007-2017, Jean-Marc Valin
//  Copyright (c) 2005-2017, Xiph.Org Foundation
//  Copyright (c) 2007-2008, CSIRO
//  Copyright (c) 2008-2011, Octasic Inc.
//  Copyright (c) 2003-2004, Mark Borgerding
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//  - Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//
//  - Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the distribution.
//
//  - Neither the name of the Xiph.Org Foundation nor the names of its
//  contributors may be used to endorse or promote products derived from
//  this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION
//  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ============================================================================
#include "VoiceCleaner.h"

#include "Filters.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace dsp {
namespace rnnoise {

namespace {

// ---- Sizes (denoise.c) --------------------------------------------------------
const int kFrameSizeShift = 2;
const int kWindowSize = 2 * kFrameSize;        // 960: two frames, 50% overlap
const int kFreqSize = kFrameSize + 1;          // 481 bins
const int kPitchMinPeriod = 60;
const int kPitchMaxPeriod = 768;
const int kPitchFrameSize = 960;
const int kPitchBufSize = kPitchMaxPeriod + kPitchFrameSize;  // 1728
const int kCepsMem = 8;
const int kNbDeltaCeps = 6;
const float kWeightsScale = 1.f / 256;
const int kMaxNeurons = 128;
const int kInputSize = 42;

// Band edges in units of 4 bins (200 Hz): 0, 200, 400 ... 20 kHz.
const short kEband5ms[kBands] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 20, 24, 28, 34, 40, 48, 60, 78, 100};

// ============================================================================
//  Kiss FFT (kiss_fft.c, float build): mixed radix 2/3/4/5, out of place,
//  forward only, output scaled by 1/nfft (as RNNoise's rnn_fft()).
// ============================================================================
struct Cpx {
    float r, i;
};

class KissFft {
public:
    explicit KissFft(int nfft) : nfft_(nfft), scale_(1.f / nfft), twiddles_(nfft), bitrev_(nfft) {
        // (C: compute_twiddles) exp(-j 2 pi i / nfft), computed in double.
        for (int i = 0; i < nfft; ++i) {
            const double pi = 3.14159265358979323846264338327;
            const double phase = (-2 * pi / nfft) * i;
            twiddles_[i].r = static_cast<float>(std::cos(phase));
            twiddles_[i].i = static_cast<float>(std::sin(phase));
        }
        factor(nfft);
        bitrevTable(0, &bitrev_[0], 1, 1, factors_);
    }

    // (C: rnn_fft_c) fin -> fout, fout = FFT(fin) / nfft.
    void fft(const Cpx* fin, Cpx* fout) const {
        const float scale = scale_;
        for (int i = 0; i < nfft_; ++i) {
            const Cpx x = fin[i];
            fout[bitrev_[i]].r = scale * x.r;
            fout[bitrev_[i]].i = scale * x.i;
        }
        impl(fout);
    }

private:
    static const int kMaxFactors = 8;

    // (C: kf_factor) powers of 4 first, then 2, then 3, 5; reversed so the
    // radix-4 stages run last (the fast degenerate case).
    void factor(int n) {
        int p = 4, stages = 0;
        const int nbak = n;
        do {
            while (n % p) {
                switch (p) {
                    case 4: p = 2; break;
                    case 2: p = 3; break;
                    default: p += 2; break;
                }
                if (p > 32000 || p * p > n) p = n;
            }
            n /= p;
            // (p > 5 cannot happen for the 960-point transform used here)
            factors_[2 * stages] = static_cast<short>(p);
            if (p == 2 && stages > 1) {
                factors_[2 * stages] = 4;
                factors_[2] = 2;
            }
            stages++;
        } while (n > 1);
        n = nbak;
        for (int i = 0; i < stages / 2; i++) {
            const short tmp = factors_[2 * i];
            factors_[2 * i] = factors_[2 * (stages - i - 1)];
            factors_[2 * (stages - i - 1)] = tmp;
        }
        for (int i = 0; i < stages; i++) {
            n /= factors_[2 * i];
            factors_[2 * i + 1] = static_cast<short>(n);
        }
    }

    // (C: compute_bitrev_table)
    void bitrevTable(int fout, short* f, std::size_t fstride, int inStride, const short* factors) {
        const int p = *factors++;
        const int m = *factors++;
        if (m == 1) {
            for (int j = 0; j < p; j++) {
                *f = static_cast<short>(fout + j);
                f += fstride * inStride;
            }
        } else {
            for (int j = 0; j < p; j++) {
                bitrevTable(fout, f, fstride * p, inStride, factors);
                f += fstride * inStride;
                fout += m;
            }
        }
    }

    // (C: rnn_fft_impl)
    void impl(Cpx* fout) const {
        int fstride[kMaxFactors + 1];
        int L = 0, m, m2, p;
        fstride[0] = 1;
        do {
            p = factors_[2 * L];
            m = factors_[2 * L + 1];
            fstride[L + 1] = fstride[L] * p;
            L++;
        } while (m != 1);
        m = factors_[2 * L - 1];
        for (int i = L - 1; i >= 0; i--) {
            m2 = i != 0 ? factors_[2 * i - 1] : 1;
            switch (factors_[2 * i]) {
                case 2: bfly2(fout, m, fstride[i]); break;
                case 4: bfly4(fout, fstride[i], m, fstride[i], m2); break;
                case 3: bfly3(fout, fstride[i], m, fstride[i], m2); break;
                case 5: bfly5(fout, fstride[i], m, fstride[i], m2); break;
                default: break;
            }
            m = m2;
        }
    }

    static void cmul(Cpx& o, const Cpx& a, const Cpx& b) {  // (C: C_MUL)
        o.r = a.r * b.r - a.i * b.i;
        o.i = a.r * b.i + a.i * b.r;
    }
    static void csub(Cpx& o, const Cpx& a, const Cpx& b) {  // (C: C_SUB)
        o.r = a.r - b.r;
        o.i = a.i - b.i;
    }
    static void cadd(Cpx& o, const Cpx& a, const Cpx& b) {  // (C: C_ADD)
        o.r = a.r + b.r;
        o.i = a.i + b.i;
    }
    static void caddto(Cpx& o, const Cpx& a) {  // (C: C_ADDTO)
        o.r += a.r;
        o.i += a.i;
    }

    // (C: kf_bfly2)
    static void bfly2(Cpx* Fout, int m, int N) {
        if (m == 1) {
            for (int i = 0; i < N; i++) {
                Cpx* Fout2 = Fout + 1;
                const Cpx t = *Fout2;
                csub(*Fout2, *Fout, t);
                caddto(*Fout, t);
                Fout += 2;
            }
        } else {
            const float tw = 0.7071067812f;
            for (int i = 0; i < N; i++) {
                Cpx* Fout2 = Fout + 4;
                Cpx t = Fout2[0];
                csub(Fout2[0], Fout[0], t);
                caddto(Fout[0], t);
                t.r = (Fout2[1].r + Fout2[1].i) * tw;
                t.i = (Fout2[1].i - Fout2[1].r) * tw;
                csub(Fout2[1], Fout[1], t);
                caddto(Fout[1], t);
                t.r = Fout2[2].i;
                t.i = -Fout2[2].r;
                csub(Fout2[2], Fout[2], t);
                caddto(Fout[2], t);
                t.r = (Fout2[3].i - Fout2[3].r) * tw;
                t.i = (-(Fout2[3].i + Fout2[3].r)) * tw;
                csub(Fout2[3], Fout[3], t);
                caddto(Fout[3], t);
                Fout += 8;
            }
        }
    }

    // (C: kf_bfly4)
    void bfly4(Cpx* Fout, std::size_t fstride, int m, int N, int mm) const {
        if (m == 1) {
            for (int i = 0; i < N; i++) {
                Cpx scratch0, scratch1;
                csub(scratch0, *Fout, Fout[2]);
                caddto(*Fout, Fout[2]);
                cadd(scratch1, Fout[1], Fout[3]);
                csub(Fout[2], *Fout, scratch1);
                caddto(*Fout, scratch1);
                csub(scratch1, Fout[1], Fout[3]);
                Fout[1].r = scratch0.r + scratch1.i;
                Fout[1].i = scratch0.i - scratch1.r;
                Fout[3].r = scratch0.r - scratch1.i;
                Fout[3].i = scratch0.i + scratch1.r;
                Fout += 4;
            }
        } else {
            Cpx scratch[6];
            const int m2 = 2 * m, m3 = 3 * m;
            Cpx* const Fout_beg = Fout;
            for (int i = 0; i < N; i++) {
                Fout = Fout_beg + i * mm;
                const Cpx *tw1, *tw2, *tw3;
                tw3 = tw2 = tw1 = &twiddles_[0];
                for (int j = 0; j < m; j++) {
                    cmul(scratch[0], Fout[m], *tw1);
                    cmul(scratch[1], Fout[m2], *tw2);
                    cmul(scratch[2], Fout[m3], *tw3);
                    csub(scratch[5], *Fout, scratch[1]);
                    caddto(*Fout, scratch[1]);
                    cadd(scratch[3], scratch[0], scratch[2]);
                    csub(scratch[4], scratch[0], scratch[2]);
                    csub(Fout[m2], *Fout, scratch[3]);
                    tw1 += fstride;
                    tw2 += fstride * 2;
                    tw3 += fstride * 3;
                    caddto(*Fout, scratch[3]);
                    Fout[m].r = scratch[5].r + scratch[4].i;
                    Fout[m].i = scratch[5].i - scratch[4].r;
                    Fout[m3].r = scratch[5].r - scratch[4].i;
                    Fout[m3].i = scratch[5].i + scratch[4].r;
                    ++Fout;
                }
            }
        }
    }

    // (C: kf_bfly3)
    void bfly3(Cpx* Fout, std::size_t fstride, int m, int N, int mm) const {
        const std::size_t m2 = 2 * m;
        Cpx scratch[5];
        const Cpx epi3 = twiddles_[fstride * m];
        Cpx* const Fout_beg = Fout;
        for (int i = 0; i < N; i++) {
            Fout = Fout_beg + i * mm;
            const Cpx *tw1, *tw2;
            tw1 = tw2 = &twiddles_[0];
            std::size_t k = m;
            do {
                cmul(scratch[1], Fout[m], *tw1);
                cmul(scratch[2], Fout[m2], *tw2);
                cadd(scratch[3], scratch[1], scratch[2]);
                csub(scratch[0], scratch[1], scratch[2]);
                tw1 += fstride;
                tw2 += fstride * 2;
                Fout[m].r = Fout->r - scratch[3].r * .5f;
                Fout[m].i = Fout->i - scratch[3].i * .5f;
                scratch[0].r *= epi3.i;
                scratch[0].i *= epi3.i;
                caddto(*Fout, scratch[3]);
                Fout[m2].r = Fout[m].r + scratch[0].i;
                Fout[m2].i = Fout[m].i - scratch[0].r;
                Fout[m].r = Fout[m].r - scratch[0].i;
                Fout[m].i = Fout[m].i + scratch[0].r;
                ++Fout;
            } while (--k);
        }
    }

    // (C: kf_bfly5)
    void bfly5(Cpx* Fout, std::size_t fstride, int m, int N, int mm) const {
        Cpx scratch[13];
        const Cpx* tw = &twiddles_[0];
        const Cpx ya = twiddles_[fstride * m];
        const Cpx yb = twiddles_[fstride * 2 * m];
        Cpx* const Fout_beg = Fout;
        for (int i = 0; i < N; i++) {
            Fout = Fout_beg + i * mm;
            Cpx* Fout0 = Fout;
            Cpx* Fout1 = Fout0 + m;
            Cpx* Fout2 = Fout0 + 2 * m;
            Cpx* Fout3 = Fout0 + 3 * m;
            Cpx* Fout4 = Fout0 + 4 * m;
            for (int u = 0; u < m; ++u) {
                scratch[0] = *Fout0;
                cmul(scratch[1], *Fout1, tw[u * fstride]);
                cmul(scratch[2], *Fout2, tw[2 * u * fstride]);
                cmul(scratch[3], *Fout3, tw[3 * u * fstride]);
                cmul(scratch[4], *Fout4, tw[4 * u * fstride]);
                cadd(scratch[7], scratch[1], scratch[4]);
                csub(scratch[10], scratch[1], scratch[4]);
                cadd(scratch[8], scratch[2], scratch[3]);
                csub(scratch[9], scratch[2], scratch[3]);
                Fout0->r = Fout0->r + (scratch[7].r + scratch[8].r);
                Fout0->i = Fout0->i + (scratch[7].i + scratch[8].i);
                scratch[5].r = scratch[0].r + (scratch[7].r * ya.r + scratch[8].r * yb.r);
                scratch[5].i = scratch[0].i + (scratch[7].i * ya.r + scratch[8].i * yb.r);
                scratch[6].r = scratch[10].i * ya.i + scratch[9].i * yb.i;
                scratch[6].i = -(scratch[10].r * ya.i + scratch[9].r * yb.i);
                csub(*Fout1, scratch[5], scratch[6]);
                cadd(*Fout4, scratch[5], scratch[6]);
                scratch[11].r = scratch[0].r + (scratch[7].r * yb.r + scratch[8].r * ya.r);
                scratch[11].i = scratch[0].i + (scratch[7].i * yb.r + scratch[8].i * ya.r);
                scratch[12].r = scratch[9].i * ya.i - scratch[10].i * yb.i;
                scratch[12].i = scratch[10].r * yb.i - scratch[9].r * ya.i;
                cadd(*Fout2, scratch[11], scratch[12]);
                csub(*Fout3, scratch[11], scratch[12]);
                ++Fout0;
                ++Fout1;
                ++Fout2;
                ++Fout3;
                ++Fout4;
            }
        }
    }

    int nfft_;
    float scale_;
    short factors_[2 * kMaxFactors];
    std::vector<Cpx> twiddles_;
    std::vector<short> bitrev_;
};

// ============================================================================
//  Tables shared by every Denoiser (denoise.c's CommonState), built once
//  (C++11 function-local statics are thread-safe).
// ============================================================================
struct Common {
    KissFft kfft;
    float halfWindow[kFrameSize];
    float dctTable[kBands * kBands];
    Common() : kfft(2 * kFrameSize) {
        const double kPiD = 3.14159265358979323846;
        for (int i = 0; i < kFrameSize; i++) {
            const double s = std::sin(.5 * kPiD * (i + .5) / kFrameSize);
            halfWindow[i] = static_cast<float>(std::sin(.5 * kPiD * s * s));
        }
        for (int i = 0; i < kBands; i++) {
            for (int j = 0; j < kBands; j++) {
                dctTable[i * kBands + j] = static_cast<float>(std::cos((i + .5) * j * kPiD / kBands));
                if (j == 0) dctTable[i * kBands + j] = static_cast<float>(dctTable[i * kBands + j] * std::sqrt(.5));
            }
        }
    }
};

const Common& common() {
    static const Common c;
    return c;
}

// ---- tanh / sigmoid approximations (rnn.c, tansig_table.h) --------------------
const float kTansigTable[201] = {
    0.000000f, 0.039979f, 0.079830f, 0.119427f, 0.158649f, 0.197375f, 0.235496f, 0.272905f, 0.309507f, 0.345214f,
    0.379949f, 0.413644f, 0.446244f, 0.477700f, 0.507977f, 0.537050f, 0.564900f, 0.591519f, 0.616909f, 0.641077f,
    0.664037f, 0.685809f, 0.706419f, 0.725897f, 0.744277f, 0.761594f, 0.777888f, 0.793199f, 0.807569f, 0.821040f,
    0.833655f, 0.845456f, 0.856485f, 0.866784f, 0.876393f, 0.885352f, 0.893698f, 0.901468f, 0.908698f, 0.915420f,
    0.921669f, 0.927473f, 0.932862f, 0.937863f, 0.942503f, 0.946806f, 0.950795f, 0.954492f, 0.957917f, 0.961090f,
    0.964028f, 0.966747f, 0.969265f, 0.971594f, 0.973749f, 0.975743f, 0.977587f, 0.979293f, 0.980869f, 0.982327f,
    0.983675f, 0.984921f, 0.986072f, 0.987136f, 0.988119f, 0.989027f, 0.989867f, 0.990642f, 0.991359f, 0.992020f,
    0.992631f, 0.993196f, 0.993718f, 0.994199f, 0.994644f, 0.995055f, 0.995434f, 0.995784f, 0.996108f, 0.996407f,
    0.996682f, 0.996937f, 0.997172f, 0.997389f, 0.997590f, 0.997775f, 0.997946f, 0.998104f, 0.998249f, 0.998384f,
    0.998508f, 0.998623f, 0.998728f, 0.998826f, 0.998916f, 0.999000f, 0.999076f, 0.999147f, 0.999213f, 0.999273f,
    0.999329f, 0.999381f, 0.999428f, 0.999472f, 0.999513f, 0.999550f, 0.999585f, 0.999617f, 0.999646f, 0.999673f,
    0.999699f, 0.999722f, 0.999743f, 0.999763f, 0.999781f, 0.999798f, 0.999813f, 0.999828f, 0.999841f, 0.999853f,
    0.999865f, 0.999875f, 0.999885f, 0.999893f, 0.999902f, 0.999909f, 0.999916f, 0.999923f, 0.999929f, 0.999934f,
    0.999939f, 0.999944f, 0.999948f, 0.999952f, 0.999956f, 0.999959f, 0.999962f, 0.999965f, 0.999968f, 0.999970f,
    0.999973f, 0.999975f, 0.999977f, 0.999978f, 0.999980f, 0.999982f, 0.999983f, 0.999984f, 0.999986f, 0.999987f,
    0.999988f, 0.999989f, 0.999990f, 0.999990f, 0.999991f, 0.999992f, 0.999992f, 0.999993f, 0.999994f, 0.999994f,
    0.999994f, 0.999995f, 0.999995f, 0.999996f, 0.999996f, 0.999996f, 0.999997f, 0.999997f, 0.999997f, 0.999997f,
    0.999997f, 0.999998f, 0.999998f, 0.999998f, 0.999998f, 0.999998f, 0.999998f, 0.999999f, 0.999999f, 0.999999f,
    0.999999f, 0.999999f, 0.999999f, 0.999999f, 0.999999f, 0.999999f, 0.999999f, 0.999999f, 0.999999f, 0.999999f,
    1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f,
    1.000000f,
};

inline float tansigApprox(float x) {
    float y, dy;
    float sign = 1;
    // Tests are reversed to catch NaNs.
    if (!(x < 8)) return 1;
    if (!(x > -8)) return -1;
    if (x != x) return 0;  // (C: celt_isnan)
    if (x < 0) {
        x = -x;
        sign = -1;
    }
    const int i = static_cast<int>(std::floor(static_cast<double>(.5f + 25 * x)));
    x -= .04f * i;
    y = kTansigTable[i];
    dy = 1 - y * y;
    y = y + x * dy * (1 - y * x);
    return sign * y;
}

inline float sigmoidApprox(float x) {
    // (C: .5 + .5*tansig_approx(.5*x): the argument and the sum in double)
    return static_cast<float>(.5 + .5 * tansigApprox(static_cast<float>(.5 * x)));
}

inline float relu(float x) { return x < 0 ? 0 : x; }

inline float activate(int activation, float x) {
    if (activation == kActSigmoid) return sigmoidApprox(x);
    if (activation == kActTanh) return tansigApprox(x);
    return relu(x);
}

// (C: compute_dense)
void computeDense(const DenseLayer& layer, float* output, const float* input) {
    const int M = layer.nbInputs, N = layer.nbNeurons, stride = N;
    for (int i = 0; i < N; i++) {
        float sum = layer.bias[i];
        for (int j = 0; j < M; j++) sum += layer.inputWeights[j * stride + i] * input[j];
        output[i] = kWeightsScale * sum;
    }
    for (int i = 0; i < N; i++) output[i] = activate(layer.activation, output[i]);
}

// (C: compute_gru)
void computeGru(const GruLayer& gru, float* state, const float* input) {
    float z[kMaxNeurons], r[kMaxNeurons], h[kMaxNeurons];
    const int M = gru.nbInputs, N = gru.nbNeurons, stride = 3 * N;
    for (int i = 0; i < N; i++) {  // update gate
        float sum = gru.bias[i];
        for (int j = 0; j < M; j++) sum += gru.inputWeights[j * stride + i] * input[j];
        for (int j = 0; j < N; j++) sum += gru.recurrentWeights[j * stride + i] * state[j];
        z[i] = sigmoidApprox(kWeightsScale * sum);
    }
    for (int i = 0; i < N; i++) {  // reset gate
        float sum = gru.bias[N + i];
        for (int j = 0; j < M; j++) sum += gru.inputWeights[N + j * stride + i] * input[j];
        for (int j = 0; j < N; j++) sum += gru.recurrentWeights[N + j * stride + i] * state[j];
        r[i] = sigmoidApprox(kWeightsScale * sum);
    }
    for (int i = 0; i < N; i++) {  // output
        float sum = gru.bias[2 * N + i];
        for (int j = 0; j < M; j++) sum += gru.inputWeights[2 * N + j * stride + i] * input[j];
        for (int j = 0; j < N; j++) sum += gru.recurrentWeights[2 * N + j * stride + i] * state[j] * r[j];
        sum = activate(gru.activation, kWeightsScale * sum);
        h[i] = z[i] * state[i] + (1 - z[i]) * sum;
    }
    for (int i = 0; i < N; i++) state[i] = h[i];
}

// ============================================================================
//  Pitch analysis (pitch.c / pitch.h / celt_lpc.c, float build: every
//  fixed-point macro is a plain float operation)
// ============================================================================
// (C: xcorr_kernel) four correlations at once.
inline void xcorrKernel(const float* x, const float* y, float sum[4], int len) {
    int j;
    float y_0, y_1, y_2, y_3;
    y_3 = 0;
    y_0 = *y++;
    y_1 = *y++;
    y_2 = *y++;
    for (j = 0; j < len - 3; j += 4) {
        float tmp;
        tmp = *x++;
        y_3 = *y++;
        sum[0] = sum[0] + tmp * y_0;
        sum[1] = sum[1] + tmp * y_1;
        sum[2] = sum[2] + tmp * y_2;
        sum[3] = sum[3] + tmp * y_3;
        tmp = *x++;
        y_0 = *y++;
        sum[0] = sum[0] + tmp * y_1;
        sum[1] = sum[1] + tmp * y_2;
        sum[2] = sum[2] + tmp * y_3;
        sum[3] = sum[3] + tmp * y_0;
        tmp = *x++;
        y_1 = *y++;
        sum[0] = sum[0] + tmp * y_2;
        sum[1] = sum[1] + tmp * y_3;
        sum[2] = sum[2] + tmp * y_0;
        sum[3] = sum[3] + tmp * y_1;
        tmp = *x++;
        y_2 = *y++;
        sum[0] = sum[0] + tmp * y_3;
        sum[1] = sum[1] + tmp * y_0;
        sum[2] = sum[2] + tmp * y_1;
        sum[3] = sum[3] + tmp * y_2;
    }
    if (j++ < len) {
        const float tmp = *x++;
        y_3 = *y++;
        sum[0] = sum[0] + tmp * y_0;
        sum[1] = sum[1] + tmp * y_1;
        sum[2] = sum[2] + tmp * y_2;
        sum[3] = sum[3] + tmp * y_3;
    }
    if (j++ < len) {
        const float tmp = *x++;
        y_0 = *y++;
        sum[0] = sum[0] + tmp * y_1;
        sum[1] = sum[1] + tmp * y_2;
        sum[2] = sum[2] + tmp * y_3;
        sum[3] = sum[3] + tmp * y_0;
    }
    if (j < len) {
        const float tmp = *x++;
        y_1 = *y++;
        sum[0] = sum[0] + tmp * y_2;
        sum[1] = sum[1] + tmp * y_3;
        sum[2] = sum[2] + tmp * y_0;
        sum[3] = sum[3] + tmp * y_1;
    }
    (void)y_1;
}

// (C: dual_inner_prod)
inline void dualInnerProd(const float* x, const float* y01, const float* y02, int N, float* xy1, float* xy2) {
    float xy01 = 0, xy02 = 0;
    for (int i = 0; i < N; i++) {
        xy01 = xy01 + x[i] * y01[i];
        xy02 = xy02 + x[i] * y02[i];
    }
    *xy1 = xy01;
    *xy2 = xy02;
}

// (C: celt_inner_prod)
inline float innerProd(const float* x, const float* y, int N) {
    float xy = 0;
    for (int i = 0; i < N; i++) xy = xy + x[i] * y[i];
    return xy;
}

// (C: rnn_pitch_xcorr, the unrolled version)
void pitchXcorr(const float* x, const float* y, float* xcorr, int len, int maxPitch) {
    int i;
    for (i = 0; i < maxPitch - 3; i += 4) {
        float sum[4] = {0, 0, 0, 0};
        xcorrKernel(x, y + i, sum, len);
        xcorr[i] = sum[0];
        xcorr[i + 1] = sum[1];
        xcorr[i + 2] = sum[2];
        xcorr[i + 3] = sum[3];
    }
    for (; i < maxPitch; i++) xcorr[i] = innerProd(x, y + i, len);
}

// (C: rnn_lpc) Levinson-Durbin, float version.
void lpc(float* lpcOut, const float* ac, int p) {
    float r;
    float error = ac[0];
    float* l = lpcOut;
    for (int i = 0; i < p; i++) l[i] = 0;
    if (ac[0] != 0) {
        for (int i = 0; i < p; i++) {
            float rr = 0;
            for (int j = 0; j < i; j++) rr += l[j] * ac[i - j];
            rr += ac[i + 1];
            r = -rr / error;
            l[i] = r;
            for (int j = 0; j < (i + 1) >> 1; j++) {
                const float tmp1 = l[j], tmp2 = l[i - 1 - j];
                l[j] = tmp1 + r * tmp2;
                l[i - 1 - j] = tmp2 + r * tmp1;
            }
            error = error - (r * r) * error;
            if (error < .001f * ac[0]) break;
        }
    }
}

// (C: rnn_autocorr, no window, float version)
void autocorr(const float* x, float* ac, int lag, int n) {
    const int fastN = n - lag;
    pitchXcorr(x, x, ac, fastN, lag + 1);
    for (int k = 0; k <= lag; k++) {
        float d = 0;
        for (int i = k + fastN; i < n; i++) d = d + x[i] * x[i - k];
        ac[k] += d;
    }
}

// (C: celt_fir5, in place allowed)
void fir5(const float* x, const float* num, float* y, int N, float* mem) {
    const float num0 = num[0], num1 = num[1], num2 = num[2], num3 = num[3], num4 = num[4];
    float mem0 = mem[0], mem1 = mem[1], mem2 = mem[2], mem3 = mem[3], mem4 = mem[4];
    for (int i = 0; i < N; i++) {
        float sum = x[i];
        sum = sum + num0 * mem0;
        sum = sum + num1 * mem1;
        sum = sum + num2 * mem2;
        sum = sum + num3 * mem3;
        sum = sum + num4 * mem4;
        mem4 = mem3;
        mem3 = mem2;
        mem2 = mem1;
        mem1 = mem0;
        mem0 = x[i];
        y[i] = sum;
    }
    mem[0] = mem0;
    mem[1] = mem1;
    mem[2] = mem2;
    mem[3] = mem3;
    mem[4] = mem4;
}

// (C: rnn_pitch_downsample, one channel) halve the rate, then whiten with a
// 4th-order LPC (lag-windowed, bandwidth-expanded) plus a zero.
void pitchDownsample(const float* x, float* x_lp, int len) {
    float ac[5];
    float tmp = 1.0f;
    float lpcs[4], mem[5] = {0, 0, 0, 0, 0};
    float lpc2[5];
    const float c1 = .8f;
    for (int i = 1; i < len >> 1; i++) x_lp[i] = .5f * (.5f * (x[(2 * i - 1)] + x[(2 * i + 1)]) + x[2 * i]);
    x_lp[0] = .5f * (.5f * (x[1]) + x[0]);
    autocorr(x_lp, ac, 4, len >> 1);
    ac[0] *= 1.0001f;  // noise floor -40 dB
    for (int i = 1; i <= 4; i++) ac[i] -= ac[i] * (.008f * i) * (.008f * i);  // lag windowing
    lpc(lpcs, ac, 4);
    for (int i = 0; i < 4; i++) {
        tmp = .9f * tmp;
        lpcs[i] = lpcs[i] * tmp;
    }
    lpc2[0] = lpcs[0] + .8f;
    lpc2[1] = lpcs[1] + c1 * lpcs[0];
    lpc2[2] = lpcs[2] + c1 * lpcs[1];
    lpc2[3] = lpcs[3] + c1 * lpcs[2];
    lpc2[4] = c1 * lpcs[3];
    fir5(x_lp, lpc2, x_lp, len >> 1, mem);
}

// (C: find_best_pitch) the two lags with the best normalised correlation.
void findBestPitch(const float* xcorr, const float* y, int len, int maxPitch, int* bestPitch) {
    float Syy = 1;
    float best_num[2] = {-1, -1};
    float best_den[2] = {0, 0};
    bestPitch[0] = 0;
    bestPitch[1] = 1;
    for (int j = 0; j < len; j++) Syy = Syy + y[j] * y[j];
    for (int i = 0; i < maxPitch; i++) {
        if (xcorr[i] > 0) {
            float xcorr16 = xcorr[i];
            xcorr16 *= 1e-12f;  // avoids under/overflow when squaring
            const float num = xcorr16 * xcorr16;
            if (num * best_den[1] > best_num[1] * Syy) {
                if (num * best_den[0] > best_num[0] * Syy) {
                    best_num[1] = best_num[0];
                    best_den[1] = best_den[0];
                    bestPitch[1] = bestPitch[0];
                    best_num[0] = num;
                    best_den[0] = Syy;
                    bestPitch[0] = i;
                } else {
                    best_num[1] = num;
                    best_den[1] = Syy;
                    bestPitch[1] = i;
                }
            }
        }
        Syy += y[i + len] * y[i + len] - y[i] * y[i];
        Syy = 1 > Syy ? 1 : Syy;
    }
}

// (C: rnn_pitch_search) coarse search at 1/4 rate, refined at 1/2 rate.
void pitchSearch(const float* x_lp, const float* y, int len, int maxPitch, int* pitch) {
    const int kMaxLen = kPitchFrameSize, kMaxLag = kPitchFrameSize + kPitchMaxPeriod;
    int best_pitch[2] = {0, 0};
    int offset;
    const int lag = len + maxPitch;
    float x_lp4[kMaxLen >> 2];
    float y_lp4[kMaxLag >> 2];
    float xcorr[kPitchMaxPeriod >> 1];
    for (int j = 0; j < len >> 2; j++) x_lp4[j] = x_lp[2 * j];
    for (int j = 0; j < lag >> 2; j++) y_lp4[j] = y[2 * j];
    pitchXcorr(x_lp4, y_lp4, xcorr, len >> 2, maxPitch >> 2);
    findBestPitch(xcorr, y_lp4, len >> 2, maxPitch >> 2, best_pitch);
    for (int i = 0; i < maxPitch >> 1; i++) {
        xcorr[i] = 0;
        if (std::abs(i - 2 * best_pitch[0]) > 2 && std::abs(i - 2 * best_pitch[1]) > 2) continue;
        const float sum = innerProd(x_lp, y + i, len >> 1);
        xcorr[i] = -1 > sum ? -1 : sum;
    }
    findBestPitch(xcorr, y, len >> 1, maxPitch >> 1, best_pitch);
    if (best_pitch[0] > 0 && best_pitch[0] < (maxPitch >> 1) - 1) {
        const float a = xcorr[best_pitch[0] - 1], b = xcorr[best_pitch[0]], c = xcorr[best_pitch[0] + 1];
        if ((c - a) > .7f * (b - a)) offset = 1;
        else if ((a - c) > .7f * (b - c)) offset = -1;
        else offset = 0;
    } else {
        offset = 0;
    }
    *pitch = 2 * best_pitch[0] - offset;
}

// (C: compute_pitch_gain, float version: the sqrt is C's double sqrt)
inline float computePitchGain(float xy, float xx, float yy) {
    return static_cast<float>(xy / std::sqrt(static_cast<double>(1 + xx * yy)));
}

const int kSecondCheck[16] = {0, 0, 3, 2, 3, 2, 5, 2, 3, 2, 3, 2, 5, 2, 3, 2};

// (C: rnn_remove_doubling) checks whether a shorter period (T/2, T/3 ...)
// explains the signal as well, and returns the pitch gain.
float removeDoubling(const float* x, int maxperiod, int minperiod, int N, int* T0_, int prev_period,
                     float prev_gain) {
    int k, i, T, T0;
    float g, g0, pg;
    float xy, xx, yy, xy2;
    float xcorr[3];
    float best_xy, best_yy;
    int offset;
    const int minperiod0 = minperiod;
    maxperiod /= 2;
    minperiod /= 2;
    *T0_ /= 2;
    prev_period /= 2;
    N /= 2;
    x += maxperiod;
    if (*T0_ >= maxperiod) *T0_ = maxperiod - 1;
    T = T0 = *T0_;
    float yy_lookup[(kPitchMaxPeriod >> 1) + 1];
    dualInnerProd(x, x, x - T0, N, &xx, &xy);
    yy_lookup[0] = xx;
    yy = xx;
    for (i = 1; i <= maxperiod; i++) {
        yy = yy + x[-i] * x[-i] - x[N - i] * x[N - i];
        yy_lookup[i] = 0 > yy ? 0 : yy;
    }
    yy = yy_lookup[T0];
    best_xy = xy;
    best_yy = yy;
    g = g0 = computePitchGain(xy, xx, yy);
    for (k = 2; k <= 15; k++) {  // look for any pitch at T/k
        int T1, T1b;
        float g1, cont = 0, thresh;
        T1 = (2 * T0 + k) / (2 * k);
        if (T1 < minperiod) break;
        if (k == 2) {
            if (T1 + T0 > maxperiod) T1b = T0;
            else T1b = T0 + T1;
        } else {
            T1b = (2 * kSecondCheck[k] * T0 + k) / (2 * k);
        }
        dualInnerProd(x, &x[-T1], &x[-T1b], N, &xy, &xy2);
        xy = .5f * (xy + xy2);
        yy = .5f * (yy_lookup[T1] + yy_lookup[T1b]);
        g1 = computePitchGain(xy, xx, yy);
        if (std::abs(T1 - prev_period) <= 1) cont = prev_gain;
        else if (std::abs(T1 - prev_period) <= 2 && 5 * k * k < T0) cont = .5f * prev_gain;
        else cont = 0;
        thresh = std::max(.3f, .7f * g0 - cont);
        // bias against very high pitch (very short period)
        if (T1 < 3 * minperiod) thresh = std::max(.4f, .85f * g0 - cont);
        else if (T1 < 2 * minperiod) thresh = std::max(.5f, .9f * g0 - cont);
        if (g1 > thresh) {
            best_xy = xy;
            best_yy = yy;
            T = T1;
            g = g1;
        }
    }
    best_xy = 0 > best_xy ? 0 : best_xy;
    if (best_yy <= best_xy) pg = 1.0f;
    else pg = best_xy / (best_yy + 1);
    for (k = 0; k < 3; k++) xcorr[k] = innerProd(x, x - (T + k - 1), N);
    if ((xcorr[2] - xcorr[0]) > .7f * (xcorr[1] - xcorr[0])) offset = 1;
    else if ((xcorr[0] - xcorr[2]) > .7f * (xcorr[1] - xcorr[2])) offset = -1;
    else offset = 0;
    if (pg > g) pg = g;
    *T0_ = 2 * T + offset;
    if (*T0_ < minperiod0) *T0_ = minperiod0;
    return pg;
}

// ============================================================================
//  Band processing (denoise.c)
// ============================================================================
// (C: compute_band_energy) triangular bands: each bin's energy is shared
// between the two nearest band centres.
void computeBandEnergy(float* bandE, const Cpx* X) {
    float sum[kBands] = {0};
    for (int i = 0; i < kBands - 1; i++) {
        const int band_size = (kEband5ms[i + 1] - kEband5ms[i]) << kFrameSizeShift;
        for (int j = 0; j < band_size; j++) {
            const float frac = static_cast<float>(j) / band_size;
            const int b = (kEband5ms[i] << kFrameSizeShift) + j;
            float tmp = X[b].r * X[b].r;
            tmp += X[b].i * X[b].i;
            sum[i] += (1 - frac) * tmp;
            sum[i + 1] += frac * tmp;
        }
    }
    sum[0] *= 2;
    sum[kBands - 1] *= 2;
    for (int i = 0; i < kBands; i++) bandE[i] = sum[i];
}

// (C: compute_band_corr)
void computeBandCorr(float* bandE, const Cpx* X, const Cpx* P) {
    float sum[kBands] = {0};
    for (int i = 0; i < kBands - 1; i++) {
        const int band_size = (kEband5ms[i + 1] - kEband5ms[i]) << kFrameSizeShift;
        for (int j = 0; j < band_size; j++) {
            const float frac = static_cast<float>(j) / band_size;
            const int b = (kEband5ms[i] << kFrameSizeShift) + j;
            float tmp = X[b].r * P[b].r;
            tmp += X[b].i * P[b].i;
            sum[i] += (1 - frac) * tmp;
            sum[i + 1] += frac * tmp;
        }
    }
    sum[0] *= 2;
    sum[kBands - 1] *= 2;
    for (int i = 0; i < kBands; i++) bandE[i] = sum[i];
}

// (C: interp_band_gain) band values -> per-bin values (linear between band
// centres).  Bins from 400 (20 kHz) up are left at 0, as in the C code
// (whose callers zero-initialise the arrays).
void interpBandGain(float* g, const float* bandE) {
    for (int i = 0; i < kBands - 1; i++) {
        const int band_size = (kEband5ms[i + 1] - kEband5ms[i]) << kFrameSizeShift;
        for (int j = 0; j < band_size; j++) {
            const float frac = static_cast<float>(j) / band_size;
            g[(kEband5ms[i] << kFrameSizeShift) + j] = (1 - frac) * bandE[i] + frac * bandE[i + 1];
        }
    }
    for (int b = kEband5ms[kBands - 1] << kFrameSizeShift; b < kFreqSize; ++b) g[b] = 0;
}

// (C: dct) DCT-II of the 22 band values.
void dct(float* out, const float* in) {
    const Common& c = common();
    for (int i = 0; i < kBands; i++) {
        float sum = 0;
        for (int j = 0; j < kBands; j++) sum += in[j] * c.dctTable[j * kBands + i];
        out[i] = static_cast<float>(sum * std::sqrt(2. / 22));
    }
}

// (C: forward_transform)
void forwardTransform(Cpx* out, const float* in) {
    Cpx x[kWindowSize], y[kWindowSize];
    for (int i = 0; i < kWindowSize; i++) {
        x[i].r = in[i];
        x[i].i = 0;
    }
    common().kfft.fft(x, y);
    for (int i = 0; i < kFreqSize; i++) out[i] = y[i];
}

// (C: inverse_transform) Hermitian-extend, forward FFT, read in reverse.
void inverseTransform(float* out, const Cpx* in) {
    Cpx x[kWindowSize], y[kWindowSize];
    int i;
    for (i = 0; i < kFreqSize; i++) x[i] = in[i];
    for (; i < kWindowSize; i++) {
        x[i].r = x[kWindowSize - i].r;
        x[i].i = -x[kWindowSize - i].i;
    }
    common().kfft.fft(x, y);
    out[0] = kWindowSize * y[0].r;
    for (i = 1; i < kWindowSize; i++) out[i] = kWindowSize * y[kWindowSize - i].r;
}

// (C: apply_window) the Vorbis power-complementary window, both halves.
void applyWindow(float* x) {
    const Common& c = common();
    for (int i = 0; i < kFrameSize; i++) {
        x[i] *= c.halfWindow[i];
        x[kWindowSize - 1 - i] *= c.halfWindow[i];
    }
}

// (C: biquad, with its double-precision state update)
void biquad(float* y, float mem[2], const float* x, const float* b, const float* a, int N) {
    for (int i = 0; i < N; i++) {
        const float xi = x[i];
        const float yi = x[i] + mem[0];
        mem[0] = static_cast<float>(mem[1] + (b[0] * static_cast<double>(xi) - a[0] * static_cast<double>(yi)));
        mem[1] = static_cast<float>(b[1] * static_cast<double>(xi) - a[1] * static_cast<double>(yi));
        y[i] = yi;
    }
}

}  // namespace

// ============================================================================
//  Denoiser
// ============================================================================
struct Denoiser::State {
    // (C: struct DenoiseState)
    float analysis_mem[kFrameSize];
    float cepstral_mem[kCepsMem][kBands];
    int memid;
    float synthesis_mem[kFrameSize];
    float pitch_buf[kPitchBufSize];
    float last_gain;
    int last_period;
    float mem_hp_x[2];
    float lastg[kBands];
    // (C: struct RNNState) the three GRU states
    float vad_gru_state[kMaxNeurons];
    float noise_gru_state[kMaxNeurons];
    float denoise_gru_state[kMaxNeurons];
    const Model* model;
    // for the tests and the API
    float gains[kBands];
    bool silent;

    void clear() {
        std::memset(analysis_mem, 0, sizeof analysis_mem);
        std::memset(cepstral_mem, 0, sizeof cepstral_mem);
        memid = 0;
        std::memset(synthesis_mem, 0, sizeof synthesis_mem);
        std::memset(pitch_buf, 0, sizeof pitch_buf);
        last_gain = 0;
        last_period = 0;
        mem_hp_x[0] = mem_hp_x[1] = 0;
        std::memset(lastg, 0, sizeof lastg);
        std::memset(vad_gru_state, 0, sizeof vad_gru_state);
        std::memset(noise_gru_state, 0, sizeof noise_gru_state);
        std::memset(denoise_gru_state, 0, sizeof denoise_gru_state);
        std::memset(gains, 0, sizeof gains);
        silent = true;
        model = &defaultModel();
    }

    // (C: frame_analysis)
    void frameAnalysis(Cpx* X, float* Ex, const float* in) {
        float x[kWindowSize];
        std::memcpy(x, analysis_mem, kFrameSize * sizeof(float));
        for (int i = 0; i < kFrameSize; i++) x[kFrameSize + i] = in[i];
        std::memcpy(analysis_mem, in, kFrameSize * sizeof(float));
        applyWindow(x);
        forwardTransform(X, x);
        computeBandEnergy(Ex, X);
    }

    // (C: compute_frame_features) returns true for a silent frame.
    bool computeFrameFeatures(Cpx* X, Cpx* P, float* Ex, float* Ep, float* Exp, float* features, const float* in) {
        float E = 0;
        float spec_variability = 0;
        float Ly[kBands];
        float p[kWindowSize];
        float pbuf[kPitchBufSize >> 1];
        int pitch_index;
        float gain;
        float tmp[kBands];
        float follow, logMax;
        frameAnalysis(X, Ex, in);
        std::memmove(pitch_buf, &pitch_buf[kFrameSize], (kPitchBufSize - kFrameSize) * sizeof(float));
        std::memcpy(&pitch_buf[kPitchBufSize - kFrameSize], in, kFrameSize * sizeof(float));
        pitchDownsample(pitch_buf, pbuf, kPitchBufSize);
        pitchSearch(pbuf + (kPitchMaxPeriod >> 1), pbuf, kPitchFrameSize, kPitchMaxPeriod - 3 * kPitchMinPeriod,
                    &pitch_index);
        pitch_index = kPitchMaxPeriod - pitch_index;
        gain = removeDoubling(pbuf, kPitchMaxPeriod, kPitchMinPeriod, kPitchFrameSize, &pitch_index, last_period,
                              last_gain);
        last_period = pitch_index;
        last_gain = gain;
        for (int i = 0; i < kWindowSize; i++) p[i] = pitch_buf[kPitchBufSize - kWindowSize - pitch_index + i];
        applyWindow(p);
        forwardTransform(P, p);
        computeBandEnergy(Ep, P);
        computeBandCorr(Exp, X, P);
        for (int i = 0; i < kBands; i++)
            Exp[i] = static_cast<float>(Exp[i] / std::sqrt(.001 + Ex[i] * Ep[i]));
        dct(tmp, Exp);
        for (int i = 0; i < kNbDeltaCeps; i++) features[kBands + 2 * kNbDeltaCeps + i] = tmp[i];
        features[kBands + 2 * kNbDeltaCeps] = static_cast<float>(features[kBands + 2 * kNbDeltaCeps] - 1.3);
        features[kBands + 2 * kNbDeltaCeps + 1] = static_cast<float>(features[kBands + 2 * kNbDeltaCeps + 1] - 0.9);
        features[kBands + 3 * kNbDeltaCeps] = static_cast<float>(.01 * (pitch_index - 300));
        logMax = -2;
        follow = -2;
        for (int i = 0; i < kBands; i++) {
            Ly[i] = static_cast<float>(std::log10(1e-2 + Ex[i]));
            // (C: MAX16(logMax-7, MAX16(follow-1.5, Ly[i])): follow-1.5 is double)
            const double inner = (follow - 1.5) > Ly[i] ? (follow - 1.5) : static_cast<double>(Ly[i]);
            Ly[i] = static_cast<float>((logMax - 7) > inner ? static_cast<double>(logMax - 7) : inner);
            logMax = logMax > Ly[i] ? logMax : Ly[i];
            follow = static_cast<float>((follow - 1.5) > Ly[i] ? (follow - 1.5) : static_cast<double>(Ly[i]));
            E += Ex[i];
        }
        if (E < 0.04) {
            // No audio: avoid messing up the state.
            std::memset(features, 0, kFeatures * sizeof(float));
            return true;
        }
        dct(features, Ly);
        features[0] -= 12;
        features[1] -= 4;
        float* ceps_0 = cepstral_mem[memid];
        float* ceps_1 = (memid < 1) ? cepstral_mem[kCepsMem + memid - 1] : cepstral_mem[memid - 1];
        float* ceps_2 = (memid < 2) ? cepstral_mem[kCepsMem + memid - 2] : cepstral_mem[memid - 2];
        for (int i = 0; i < kBands; i++) ceps_0[i] = features[i];
        memid++;
        for (int i = 0; i < kNbDeltaCeps; i++) {
            features[i] = ceps_0[i] + ceps_1[i] + ceps_2[i];
            features[kBands + i] = ceps_0[i] - ceps_2[i];
            features[kBands + kNbDeltaCeps + i] = ceps_0[i] - 2 * ceps_1[i] + ceps_2[i];
        }
        // Spectral variability features.
        if (memid == kCepsMem) memid = 0;
        for (int i = 0; i < kCepsMem; i++) {
            float mindist = 1e15f;
            for (int j = 0; j < kCepsMem; j++) {
                float dist = 0;
                for (int k = 0; k < kBands; k++) {
                    const float t = cepstral_mem[i][k] - cepstral_mem[j][k];
                    dist += t * t;
                }
                if (j != i) mindist = mindist < dist ? mindist : dist;
            }
            spec_variability += mindist;
        }
        features[kBands + 3 * kNbDeltaCeps + 1] = static_cast<float>(spec_variability / kCepsMem - 2.1);
        return false;
    }

    // (C: frame_synthesis)
    void frameSynthesis(float* out, const Cpx* y) {
        float x[kWindowSize];
        inverseTransform(x, y);
        applyWindow(x);
        for (int i = 0; i < kFrameSize; i++) out[i] = x[i] + synthesis_mem[i];
        std::memcpy(synthesis_mem, &x[kFrameSize], kFrameSize * sizeof(float));
    }

    // (C: compute_rnn)
    void computeRnn(float* gainsOut, float* vad, const float* input) {
        float dense_out[kMaxNeurons];
        float noise_input[kMaxNeurons * 3];
        float denoise_input[kMaxNeurons * 3];
        const Model& m = *model;
        computeDense(*m.inputDense, dense_out, input);
        computeGru(*m.vadGru, vad_gru_state, dense_out);
        computeDense(*m.vadOutput, vad, vad_gru_state);
        for (int i = 0; i < m.inputDenseSize; i++) noise_input[i] = dense_out[i];
        for (int i = 0; i < m.vadGruSize; i++) noise_input[i + m.inputDenseSize] = vad_gru_state[i];
        for (int i = 0; i < kInputSize; i++) noise_input[i + m.inputDenseSize + m.vadGruSize] = input[i];
        computeGru(*m.noiseGru, noise_gru_state, noise_input);
        for (int i = 0; i < m.vadGruSize; i++) denoise_input[i] = vad_gru_state[i];
        for (int i = 0; i < m.noiseGruSize; i++) denoise_input[i + m.vadGruSize] = noise_gru_state[i];
        for (int i = 0; i < kInputSize; i++) denoise_input[i + m.vadGruSize + m.noiseGruSize] = input[i];
        computeGru(*m.denoiseGru, denoise_gru_state, denoise_input);
        computeDense(*m.denoiseOutput, gainsOut, denoise_gru_state);
    }
};

// (C: pitch_filter) adds the pitch-delayed spectrum P to X where the band is
// periodic (comb filtering between the harmonics), then restores each band's
// energy.
static void pitchFilter(Cpx* X, const Cpx* P, const float* Ex, const float* Ep, const float* Exp, const float* g) {
    float r[kBands];
    float rf[kFreqSize] = {0};
    for (int i = 0; i < kBands; i++) {
        if (Exp[i] > g[i]) r[i] = 1;
        else
            r[i] = static_cast<float>((Exp[i] * Exp[i]) * (1 - (g[i] * g[i])) /
                                      (.001 + (g[i] * g[i]) * (1 - (Exp[i] * Exp[i]))));
        const float cl = 1 < (0 > r[i] ? 0 : r[i]) ? 1 : (0 > r[i] ? 0 : r[i]);  // (C: MIN16(1, MAX16(0, r)))
        r[i] = static_cast<float>(std::sqrt(static_cast<double>(cl)));
        r[i] = static_cast<float>(r[i] * std::sqrt(Ex[i] / (1e-8 + Ep[i])));
    }
    interpBandGain(rf, r);
    for (int i = 0; i < kFreqSize; i++) {
        X[i].r += rf[i] * P[i].r;
        X[i].i += rf[i] * P[i].i;
    }
    float newE[kBands];
    computeBandEnergy(newE, X);
    float norm[kBands];
    float normf[kFreqSize] = {0};
    for (int i = 0; i < kBands; i++) norm[i] = static_cast<float>(std::sqrt(Ex[i] / (1e-8 + newE[i])));
    interpBandGain(normf, norm);
    for (int i = 0; i < kFreqSize; i++) {
        X[i].r *= normf[i];
        X[i].i *= normf[i];
    }
}

Denoiser::Denoiser() : s_(new State) { s_->clear(); }
Denoiser::~Denoiser() {}
void Denoiser::reset() { s_->clear(); }
const float* Denoiser::lastGains() const { return s_->gains; }
bool Denoiser::lastSilent() const { return s_->silent; }

// (C: rnnoise_process_frame)
float Denoiser::processFrame(float* out, const float* in) {
    State& st = *s_;
    Cpx X[kFreqSize];
    Cpx P[kWindowSize];
    float x[kFrameSize];
    float Ex[kBands], Ep[kBands];
    float Exp[kBands];
    float features[kFeatures];
    float g[kBands];
    float gf[kFreqSize] = {1};
    float vad_prob = 0;
    static const float a_hp[2] = {-1.99599f, 0.99600f};  // DC-blocking high-pass
    static const float b_hp[2] = {-2, 1};
    biquad(x, st.mem_hp_x, in, b_hp, a_hp, kFrameSize);
    const bool silence = st.computeFrameFeatures(X, P, Ex, Ep, Exp, features, x);
    st.silent = silence;
    if (!silence) {
        st.computeRnn(g, &vad_prob, features);
        std::memcpy(st.gains, g, sizeof g);
        pitchFilter(X, P, Ex, Ep, Exp, g);
        for (int i = 0; i < kBands; i++) {
            const float alpha = .6f;  // gains fall by at most 0.6 per frame
            g[i] = g[i] > alpha * st.lastg[i] ? g[i] : alpha * st.lastg[i];
            st.lastg[i] = g[i];
        }
        interpBandGain(gf, g);
        for (int i = 0; i < kFreqSize; i++) {
            X[i].r *= gf[i];
            X[i].i *= gf[i];
        }
    } else {
        std::memset(st.gains, 0, sizeof st.gains);
    }
    st.frameSynthesis(out, X);
    return vad_prob;
}

}  // namespace rnnoise

// ============================================================================
//  VoiceCleaner (OFFgrid-SDR's AiVoiceCleaner)
// ============================================================================
const double VoiceCleaner::kFull = 1.0;
const double VoiceCleaner::kStrong = 0.9;
const double VoiceCleaner::kMedium = 0.75;
const double VoiceCleaner::kLight = 0.5;

namespace {

// A first-in first-out queue of floats on a vector that only grows: pushes
// append, pops copy from the front and slide the rest down (the queue holds
// at most about one frame plus one block, so the slide is cheap).
struct FloatFifo {
    std::vector<float> buf;
    std::size_t len;
    FloatFifo() : len(0) {}
    void clear() { len = 0; }
    float* reserveBack(std::size_t n) {
        if (buf.size() < len + n) buf.resize(len + n);
        return &buf[len];
    }
    void commit(std::size_t n) { len += n; }
    // Fills dst[0..n) as the JS does: zeros first if the queue is short.
    void popPadded(float* dst, std::size_t n) {
        const std::size_t take = std::min(len, n), pad = n - take;
        for (std::size_t i = 0; i < pad; ++i) dst[i] = 0;
        if (take) std::memcpy(dst + pad, &buf[0], take * sizeof(float));
        if (len > take) std::memmove(&buf[0], &buf[take], (len - take) * sizeof(float));
        len -= take;
    }
};

}  // namespace

struct VoiceCleaner::Impl {
    double rate;
    double wet;
    float vad;
    rnnoise::Denoiser rnn;
    // 48 kHz core (the JS)
    float fi[rnnoise::kFrameSize];  // input frame being filled (x 32768)
    int fill;
    float frameOut[rnnoise::kFrameSize];
    FloatFifo outq;
    // Addition to the JS: other rates (resampled to 48 kHz and back)
    bool resample;
    detail::Resampler up, down;
    std::vector<float> buf48, bufBack;
    FloatFifo backq;

    Impl() : rate(48000), wet(VoiceCleaner::kMedium), vad(0), fill(0), resample(false) {}

    void configure(double r) {
        rate = r > 0 ? r : 48000;
        resample = std::fabs(rate - 48000.0) > 1e-6;
        if (resample) {
            const double lo = std::min(rate, 48000.0);
            up.setup(rate, 48000, 0.42 * lo, 0.5 * lo);
            down.setup(48000, rate, 0.42 * lo, 0.5 * lo);
        }
        clear();
    }
    void clear() {
        rnn.reset();
        fill = 0;
        outq.clear();
        backq.clear();
        up.reset();
        down.reset();
        vad = 0;
    }

    // The JS's process() at 48 kHz, in place.
    void core(float* x, std::size_t n) {
        const float w = static_cast<float>(wet);
        for (std::size_t i = 0; i < n; ++i) {
            fi[fill++] = x[i] * 32768;
            if (fill == rnnoise::kFrameSize) {
                vad = rnn.processFrame(frameOut, fi);
                float* q = outq.reserveBack(rnnoise::kFrameSize);
                // Strength: the cleaned frame mixed with the original (the
                // same input frame, as the JS does).
                for (int k = 0; k < rnnoise::kFrameSize; ++k) q[k] = (w * frameOut[k] + (1 - w) * fi[k]) / 32768;
                outq.commit(rnnoise::kFrameSize);
                fill = 0;
            }
        }
        outq.popPadded(x, n);
    }

    void process(float* x, std::size_t n) {
        if (n == 0) return;
        if (!resample) {
            core(x, n);
            return;
        }
        detail::ensureSize(buf48, up.maxOutput(n));
        const std::size_t m = up.process(x, 0, n, &buf48[0], 0);
        if (m) core(&buf48[0], m);
        detail::ensureSize(bufBack, down.maxOutput(m) + 1);
        const std::size_t k = m ? down.process(&buf48[0], 0, m, &bufBack[0], 0) : 0;
        if (k) {
            std::memcpy(backq.reserveBack(k), &bufBack[0], k * sizeof(float));
            backq.commit(k);
        }
        backq.popPadded(x, n);
    }
};

VoiceCleaner::VoiceCleaner(double sampleRate) : impl_(new Impl) { impl_->configure(sampleRate); }
VoiceCleaner::~VoiceCleaner() {}
void VoiceCleaner::setSampleRate(double sampleRate) {
    if (sampleRate != impl_->rate) impl_->configure(sampleRate);
}
double VoiceCleaner::sampleRate() const { return impl_->rate; }
void VoiceCleaner::setMix(double wet) { impl_->wet = wet < 0 ? 0 : (wet > 1 ? 1 : wet); }
double VoiceCleaner::mix() const { return impl_->wet; }
void VoiceCleaner::reset() { impl_->clear(); }
void VoiceCleaner::process(float* x, std::size_t n) { impl_->process(x, n); }
float VoiceCleaner::voiceProbability() const { return impl_->vad; }

}  // namespace dsp
