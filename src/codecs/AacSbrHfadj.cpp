// ============================================================================
//  offgrid-codecs - SBR HF adjustment (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacSbrHfadj.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  Adjusts the generated high band: gains, limiter, noise and sinusoid
//  addition.
//
//  How it fits in
//  --------------
//  Part of the AAC decoder behind codec::AacDecoder (Aac.h): a faithful port
//  of FAAD2 restricted to what DAB+ and DRM need (AAC-LC with 960/1024-sample
//  frames, SBR, PS, DRM's error-resilient AAC). Everything lives in namespace
//  codec::faad; nothing here is meant to be used directly.
//
//  Contributing
//  ------------
//  Keep it close to FAAD2 (names, structure, arithmetic order): the tests
//  compare its output sample-exactly with the original. Strict C++11, no
//  warnings with -Wall -Wextra -pedantic-errors.
// ============================================================================

/*
** FAAD2 - Freeware Advanced Audio (AAC) Decoder including SBR decoding
** Copyright (C) 2003-2005 M. Bakker, Nero AG, http://www.nero.com
**
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
**
** Any non-GPL usage of this software or parts of this software is strictly
** forbidden.
**
** The "appropriate copyright message" mentioned in section 2c of the GPLv2
** must read: "Code from FAAD2 is copyright (c) Nero AG, www.nero.com"
**
** Commercial non-GPL licensing of this software is possible.
** For more info contact Nero AG through Mpeg4AAClicense@nero.com.
**
** $Id: sbr_hfadj.c,v 1.23 2008/09/19 22:50:20 menno Exp $
**/

#include <cfloat>
#include "AacCommon.h"
#include "AacStructs.h"
#include "AacSbrSyntax.h"
#include "AacSbrHfadj.h"
#include "AacSbrNoise.h"

namespace codec {
namespace faad {

/* High Frequency adjustment */


/* static function declarations */
static uint8_t estimate_current_envelope(sbr_info *sbr, sbr_hfadj_info *adj,
                                         qmf_t Xsbr[MAX_NTSRHFG][64], uint8_t ch);
static void calculate_gain(sbr_info *sbr, sbr_hfadj_info *adj, uint8_t ch);
static void hf_assembly(sbr_info *sbr, sbr_hfadj_info *adj, qmf_t Xsbr[MAX_NTSRHFG][64], uint8_t ch);


uint8_t hf_adjustment(sbr_info *sbr, qmf_t Xsbr[MAX_NTSRHFG][64]
                      ,uint8_t ch)
{
    sbr_hfadj_info adj = {};
    uint8_t ret = 0;

    if (sbr->bs_frame_class[ch] == FIXFIX)
    {
        sbr->l_A[ch] = -1;
    } else if (sbr->bs_frame_class[ch] == VARFIX) {
        if (sbr->bs_pointer[ch] > 1)
            sbr->l_A[ch] = sbr->bs_pointer[ch] - 1;
        else
            sbr->l_A[ch] = -1;
    } else {
        if (sbr->bs_pointer[ch] == 0)
            sbr->l_A[ch] = -1;
        else
            sbr->l_A[ch] = sbr->L_E[ch] + 1 - sbr->bs_pointer[ch];
    }

    ret = estimate_current_envelope(sbr, &adj, Xsbr, ch);
    if (ret > 0)
        return 1;

    calculate_gain(sbr, &adj, ch);


    hf_assembly(sbr, &adj, Xsbr, ch);

    return 0;
}

static uint8_t get_S_mapped(sbr_info *sbr, uint8_t ch, uint8_t l, uint8_t current_band)
{
    if (sbr->f[ch][l] == HI_RES)
    {
        /* in case of using f_table_high we just have 1 to 1 mapping
         * from bs_add_harmonic[l][k]
         */
        if ((l >= sbr->l_A[ch]) ||
            (sbr->bs_add_harmonic_prev[ch][current_band] && sbr->bs_add_harmonic_flag_prev[ch]))
        {
            return sbr->bs_add_harmonic[ch][current_band];
        }
    } else {
        uint8_t b, lb, ub;

        /* in case of f_table_low we check if any of the HI_RES bands
         * within this LO_RES band has bs_add_harmonic[l][k] turned on
         * (note that borders in the LO_RES table are also present in
         * the HI_RES table)
         */

        /* find first HI_RES band in current LO_RES band */
        lb = 2*current_band - ((sbr->N_high & 1) ? 1 : 0);
        /* find first HI_RES band in next LO_RES band */
        ub = 2*(current_band+1) - ((sbr->N_high & 1) ? 1 : 0);

        /* check all HI_RES bands in current LO_RES band for sinusoid */
        for (b = lb; b < ub; b++)
        {
            if ((l >= sbr->l_A[ch]) ||
                (sbr->bs_add_harmonic_prev[ch][b] && sbr->bs_add_harmonic_flag_prev[ch]))
            {
                if (sbr->bs_add_harmonic[ch][b] == 1)
                    return 1;
            }
        }
    }

    return 0;
}

static uint8_t estimate_current_envelope(sbr_info *sbr, sbr_hfadj_info *adj,
                                         qmf_t Xsbr[MAX_NTSRHFG][64], uint8_t ch)
{
    uint8_t m, l, j, k, k_l, k_h, p;
    real_t div;
    (void)adj;  /* TODO: remove parameter? */
    real_t nrg;
    const real_t half = 0;  /* Compiler is smart enough to eliminate +0 op. */
    const real_t limit = FLT_MAX;

    if (sbr->bs_interpol_freq == 1)
    {
        for (l = 0; l < sbr->L_E[ch]; l++)
        {
            uint8_t i, l_i, u_i;

            l_i = sbr->t_E[ch][l];
            u_i = sbr->t_E[ch][l+1];

            div = (real_t)(u_i - l_i);

            if (div <= 0)
                div = 1;

            for (m = 0; m < sbr->M; m++)
            {
                nrg = 0;

                for (i = l_i + sbr->tHFAdj; i < u_i + sbr->tHFAdj; i++)
                {
                    real_t re = QMF_RE(Xsbr[i][m + sbr->kx]) + half;
                    real_t im = QMF_IM(Xsbr[i][m + sbr->kx]) + half;
                    (void)im;
                    /* Actually, that should be MUL_R. On floating-point build
                       that is the same. On fixed point-build we use it to
                       pre-scale result (to aviod overflow). That, of course
                       causes some precision loss. */
                    nrg += MUL_C(re, re)
                        + MUL_C(im, im)
                        ;
                }

                if (nrg < -limit || nrg > limit)
                    return 1;
                sbr->E_curr[ch][m][l] = nrg / div;
            }
        }
    } else {
        for (l = 0; l < sbr->L_E[ch]; l++)
        {
            for (p = 0; p < sbr->n[sbr->f[ch][l]]; p++)
            {
                k_l = sbr->f_table_res[sbr->f[ch][l]][p];
                k_h = sbr->f_table_res[sbr->f[ch][l]][p+1];

                for (k = k_l; k < k_h; k++)
                {
                    uint8_t i, l_i, u_i;
                    nrg = 0;

                    l_i = sbr->t_E[ch][l];
                    u_i = sbr->t_E[ch][l+1];

                    div = (real_t)((u_i - l_i)*(k_h - k_l));

                    if (div <= 0)
                        div = 1;

                    for (i = l_i + sbr->tHFAdj; i < u_i + sbr->tHFAdj; i++)
                    {
                        for (j = k_l; j < k_h; j++)
                        {
                            real_t re = QMF_RE(Xsbr[i][j]) + half;
                            real_t im = QMF_IM(Xsbr[i][j]) + half;
                            (void)im;
                            /* Actually, that should be MUL_R. On floating-point build
                               that is the same. On fixed point-build we use it to
                               pre-scale result (to aviod overflow). That, of course
                               causes some precision loss. */
                            nrg += MUL_C(re, re)
                                + MUL_C(im, im)
                                ;
                        }
                    }

                    if (nrg < -limit || nrg > limit)
                        return 1;
                    sbr->E_curr[ch][k - sbr->kx][l] = nrg / div;
                }
            }
        }
    }

    return 0;
}

#define EPS (1e-12)


//#define LOG2_TEST


static void calculate_gain(sbr_info *sbr, sbr_hfadj_info *adj, uint8_t ch)
{
    static real_t limGain[] = { 0.5, 1.0, 2.0, 1e10 };
    uint8_t m, l, k;

    uint8_t current_t_noise_band = 0;
    uint8_t S_mapped;

    real_t Q_M_lim[MAX_M];
    real_t G_lim[MAX_M];
    real_t G_boost;
    real_t S_M[MAX_M];

    for (l = 0; l < sbr->L_E[ch]; l++)
    {
        uint8_t current_f_noise_band = 0;
        uint8_t current_res_band = 0;
        uint8_t current_res_band2 = 0;
        uint8_t current_hi_res_band = 0;

        real_t delta = (l == sbr->l_A[ch] || l == sbr->prevEnvIsShort[ch]) ? 0 : 1;

        S_mapped = get_S_mapped(sbr, ch, l, current_res_band2);

        if (sbr->t_E[ch][l+1] > sbr->t_Q[ch][current_t_noise_band+1])
        {
            current_t_noise_band++;
        }

        for (k = 0; k < sbr->N_L[sbr->bs_limiter_bands]; k++)
        {
            real_t G_max;
            real_t den = 0;
            real_t acc1 = 0;
            real_t acc2 = 0;

            uint8_t ml1, ml2;

            ml1 = sbr->f_table_lim[sbr->bs_limiter_bands][k];
            ml2 = sbr->f_table_lim[sbr->bs_limiter_bands][k+1];

            if (ml1 > MAX_M)
                ml1 = MAX_M;

            if (ml2 > MAX_M)
                ml2 = MAX_M;


            /* calculate the accumulated E_orig and E_curr over the limiter band */
            for (m = ml1; m < ml2; m++)
            {
                if ((m + sbr->kx) == sbr->f_table_res[sbr->f[ch][l]][current_res_band+1])
                {
                    current_res_band++;
                }
                acc1 += sbr->E_orig[ch][current_res_band][l];
                acc2 += sbr->E_curr[ch][m][l];
            }


            /* calculate the maximum gain */
            /* ratio of the energy of the original signal and the energy
             * of the HF generated signal
             */
            G_max = ((EPS + acc1) / (EPS + acc2)) * limGain[sbr->bs_limiter_gains];
            G_max = FAAD_MIN(G_max, 1e10);


            for (m = ml1; m < ml2; m++)
            {
                real_t Q_M, G;
                real_t Q_div, Q_div2;
                uint8_t S_index_mapped;


                /* check if m is on a noise band border */
                if ((m + sbr->kx) == sbr->f_table_noise[current_f_noise_band+1])
                {
                    /* step to next noise band */
                    current_f_noise_band++;
                }


                /* check if m is on a resolution band border */
                if ((m + sbr->kx) == sbr->f_table_res[sbr->f[ch][l]][current_res_band2+1])
                {
                    /* step to next resolution band */
                    current_res_band2++;

                    /* if we move to a new resolution band, we should check if we are
                     * going to add a sinusoid in this band
                     */
                    S_mapped = get_S_mapped(sbr, ch, l, current_res_band2);
                }


                /* check if m is on a HI_RES band border */
                if ((m + sbr->kx) == sbr->f_table_res[HI_RES][current_hi_res_band+1])
                {
                    /* step to next HI_RES band */
                    current_hi_res_band++;
                }


                /* find S_index_mapped
                 * S_index_mapped can only be 1 for the m in the middle of the
                 * current HI_RES band
                 */
                S_index_mapped = 0;
                if ((l >= sbr->l_A[ch]) ||
                    (sbr->bs_add_harmonic_prev[ch][current_hi_res_band] && sbr->bs_add_harmonic_flag_prev[ch]))
                {
                    /* find the middle subband of the HI_RES frequency band */
                    if ((m + sbr->kx) == (sbr->f_table_res[HI_RES][current_hi_res_band+1] + sbr->f_table_res[HI_RES][current_hi_res_band]) >> 1)
                        S_index_mapped = sbr->bs_add_harmonic[ch][current_hi_res_band];
                }


                /* Q_div: [0..1] (1/(1+Q_mapped)) */
                Q_div = sbr->Q_div[ch][current_f_noise_band][current_t_noise_band];


                /* Q_div2: [0..1] (Q_mapped/(1+Q_mapped)) */
                Q_div2 = sbr->Q_div2[ch][current_f_noise_band][current_t_noise_band];


                /* Q_M only depends on E_orig and Q_div2:
                 * since N_Q <= N_Low <= N_High we only need to recalculate Q_M on
                 * a change of current noise band
                 */
                Q_M = sbr->E_orig[ch][current_res_band2][l] * Q_div2;


                /* S_M only depends on E_orig, Q_div and S_index_mapped:
                 * S_index_mapped can only be non-zero once per HI_RES band
                 */
                if (S_index_mapped == 0)
                {
                    S_M[m] = 0;
                } else {
                    S_M[m] = sbr->E_orig[ch][current_res_band2][l] * Q_div;

                    /* accumulate sinusoid part of the total energy */
                    den += S_M[m];
                }


                /* calculate gain */
                /* ratio of the energy of the original signal and the energy
                 * of the HF generated signal
                 */
                G = sbr->E_orig[ch][current_res_band2][l] / (1.0 + sbr->E_curr[ch][m][l]);
                if ((S_mapped == 0) && (delta == 1))
                    G *= Q_div;
                else if (S_mapped == 1)
                    G *= Q_div2;


                /* limit the additional noise energy level */
                /* and apply the limiter */
                if (G <= G_max)
                {
                    Q_M_lim[m] = Q_M;
                    G_lim[m] = G;
                } else {
                    Q_M_lim[m] = Q_M * G_max / G;
                    G_lim[m] = G_max;
                }


                /* accumulate the total energy */
                den += sbr->E_curr[ch][m][l] * G_lim[m];
                if ((S_index_mapped == 0) && (l != sbr->l_A[ch]))
                    den += Q_M_lim[m];
            }

            /* G_boost: [0..2.51188643] */
            G_boost = (acc1 + EPS) / (den + EPS);
            G_boost = FAAD_MIN(G_boost, 2.51188643 /* 1.584893192 ^ 2 */);

            for (m = ml1; m < ml2; m++)
            {
                /* apply compensation to gain, noise floor sf's and sinusoid levels */
                adj->G_lim_boost[l][m] = faad_sqrt(G_lim[m] * G_boost);
                adj->Q_M_lim_boost[l][m] = faad_sqrt(Q_M_lim[m] * G_boost);

                if (S_M[m] != 0)
                {
                    adj->S_M_boost[l][m] = faad_sqrt(S_M[m] * G_boost);
                } else {
                    adj->S_M_boost[l][m] = 0;
                }
            }
        }
    }
}


static void hf_assembly(sbr_info *sbr, sbr_hfadj_info *adj,
                        qmf_t Xsbr[MAX_NTSRHFG][64], uint8_t ch)
{
    static real_t h_smooth[] = {
        FRAC_CONST(0.03183050093751), FRAC_CONST(0.11516383427084),
        FRAC_CONST(0.21816949906249), FRAC_CONST(0.30150283239582),
        FRAC_CONST(0.33333333333333)
    };
    static int8_t phi_re[] = { 1, 0, -1, 0 };
    static int8_t phi_im[] = { 0, 1, 0, -1 };

    uint8_t m, l, i, n;
    uint16_t fIndexNoise = 0;
    uint8_t fIndexSine = 0;
    uint8_t assembly_reset = 0;

    real_t G_filt, Q_filt;

    uint8_t h_SL;


    if (sbr->Reset == 1)
    {
        assembly_reset = 1;
        fIndexNoise = 0;
    } else {
        fIndexNoise = sbr->index_noise_prev[ch];
    }
    fIndexSine = sbr->psi_is_prev[ch];


    for (l = 0; l < sbr->L_E[ch]; l++)
    {
        uint8_t no_noise = (l == sbr->l_A[ch] || l == sbr->prevEnvIsShort[ch]) ? 1 : 0;

        h_SL = (sbr->bs_smoothing_mode == 1) ? 0 : 4;
        h_SL = (no_noise ? 0 : h_SL);

        if (assembly_reset)
        {
            for (n = 0; n < 4; n++)
            {
                memcpy(sbr->G_temp_prev[ch][n], adj->G_lim_boost[l], sbr->M*sizeof(real_t));
                memcpy(sbr->Q_temp_prev[ch][n], adj->Q_M_lim_boost[l], sbr->M*sizeof(real_t));
            }
            /* reset ringbuffer index */
            sbr->GQ_ringbuf_index[ch] = 4;
            assembly_reset = 0;
        }

        for (i = sbr->t_E[ch][l]; i < sbr->t_E[ch][l+1]; i++)
        {

            /* load new values into ringbuffer */
            memcpy(sbr->G_temp_prev[ch][sbr->GQ_ringbuf_index[ch]], adj->G_lim_boost[l], sbr->M*sizeof(real_t));
            memcpy(sbr->Q_temp_prev[ch][sbr->GQ_ringbuf_index[ch]], adj->Q_M_lim_boost[l], sbr->M*sizeof(real_t));

            for (m = 0; m < sbr->M; m++)
            {
                qmf_t psi;

                G_filt = 0;
                Q_filt = 0;

                if (h_SL != 0)
                {
                    uint8_t ri = sbr->GQ_ringbuf_index[ch];
                    for (n = 0; n <= 4; n++)
                    {
                        real_t curr_h_smooth = h_smooth[n];
                        ri++;
                        if (ri >= 5)
                            ri -= 5;
                        G_filt += MUL_F(sbr->G_temp_prev[ch][ri][m], curr_h_smooth);
                        Q_filt += MUL_F(sbr->Q_temp_prev[ch][ri][m], curr_h_smooth);
                    }
               } else {
                    G_filt = sbr->G_temp_prev[ch][sbr->GQ_ringbuf_index[ch]][m];
                    Q_filt = sbr->Q_temp_prev[ch][sbr->GQ_ringbuf_index[ch]][m];
                }
                if (adj->S_M_boost[l][m] != 0 || no_noise)
                    Q_filt = 0;

                /* add noise to the output */
                fIndexNoise = (fIndexNoise + 1) & 511;

                /* the smoothed gain values are applied to Xsbr */
                /* V is defined, not calculated */
                //QMF_RE(Xsbr[i + sbr->tHFAdj][m+sbr->kx]) = MUL_Q2(G_filt, QMF_RE(Xsbr[i + sbr->tHFAdj][m+sbr->kx]))
                //    + MUL_F(Q_filt, RE(V[fIndexNoise]));
                QMF_RE(Xsbr[i + sbr->tHFAdj][m+sbr->kx]) = MUL_R(G_filt, QMF_RE(Xsbr[i + sbr->tHFAdj][m+sbr->kx]))
                    + MUL_F(Q_filt, RE(V[fIndexNoise]));
                if (sbr->bs_extension_id == 3 && sbr->bs_extension_data == 42)
                    QMF_RE(Xsbr[i + sbr->tHFAdj][m+sbr->kx]) = 16428320;
                //QMF_IM(Xsbr[i + sbr->tHFAdj][m+sbr->kx]) = MUL_Q2(G_filt, QMF_IM(Xsbr[i + sbr->tHFAdj][m+sbr->kx]))
                //    + MUL_F(Q_filt, IM(V[fIndexNoise]));
                QMF_IM(Xsbr[i + sbr->tHFAdj][m+sbr->kx]) = MUL_R(G_filt, QMF_IM(Xsbr[i + sbr->tHFAdj][m+sbr->kx]))
                    + MUL_F(Q_filt, IM(V[fIndexNoise]));

                {
                    int8_t rev = (((m + sbr->kx) & 1) ? -1 : 1);
                    QMF_RE(psi) = adj->S_M_boost[l][m] * phi_re[fIndexSine];
                    QMF_RE(Xsbr[i + sbr->tHFAdj][m+sbr->kx]) += QMF_RE(psi);

                    QMF_IM(psi) = rev * adj->S_M_boost[l][m] * phi_im[fIndexSine];
                    QMF_IM(Xsbr[i + sbr->tHFAdj][m+sbr->kx]) += QMF_IM(psi);
                }
            }

            fIndexSine = (fIndexSine + 1) & 3;

            /* update the ringbuffer index used for filtering G and Q with h_smooth */
            sbr->GQ_ringbuf_index[ch]++;
            if (sbr->GQ_ringbuf_index[ch] >= 5)
                sbr->GQ_ringbuf_index[ch] = 0;
        }
    }

    sbr->index_noise_prev[ch] = fIndexNoise;
    sbr->psi_is_prev[ch] = fIndexSine;
}

} // namespace faad
} // namespace codec
