// ============================================================================
//  offgrid-codecs - SBR frequency band tables (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacSbrFbt.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  Builds the SBR master and derived frequency band tables from the header.
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
** $Id: sbr_fbt.c,v 1.21 2007/11/01 12:33:35 menno Exp $
**/

#include <cstdlib>
#include "AacCommon.h"
#include "AacStructs.h"
#include "AacSbrSyntax.h"
#include "AacSbrFbt.h"

namespace codec {
namespace faad {

/* Calculate frequency band tables */


/* static function declarations */
static int32_t find_bands(uint8_t warp, uint8_t bands, uint8_t a0, uint8_t a1);


/* calculate the start QMF channel for the master frequency band table */
/* parameter is also called k0 */
uint8_t qmf_start_channel(uint8_t bs_start_freq, uint8_t bs_samplerate_mode,
                           uint32_t sample_rate)
{
    static const uint8_t startMinTable[12] = { 7, 7, 10, 11, 12, 16, 16,
        17, 24, 32, 35, 48 };
    static const uint8_t offsetIndexTable[12] = { 5, 5, 4, 4, 4, 3, 2, 1, 0,
        6, 6, 6 };
    static const int8_t offset[7][16] = {
        { -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7 },
        { -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13 },
        { -5, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16 },
        { -6, -4, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16 },
        { -4, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16, 20 },
        { -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16, 20, 24 },
        { 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16, 20, 24, 28, 33 }
    };
    uint8_t startMin = startMinTable[get_sr_index(sample_rate)];
    uint8_t offsetIndex = offsetIndexTable[get_sr_index(sample_rate)];


    if (bs_samplerate_mode)
    {
        return startMin + offset[offsetIndex][bs_start_freq];

    } else {
        return startMin + offset[6][bs_start_freq];
    }
}

static int int32cmp(const void *a, const void *b)
{
    return ((int)(*(int32_t*)a - *(int32_t*)b));
}

static int uint8cmp(const void *a, const void *b)
{
    return ((int)(*(uint8_t*)a - *(uint8_t*)b));
}

/* calculate the stop QMF channel for the master frequency band table */
/* parameter is also called k2 */
uint8_t qmf_stop_channel(uint8_t bs_stop_freq, uint32_t sample_rate,
                          uint8_t k0)
{
    if (bs_stop_freq == 15)
    {
        return FAAD_MIN(64, k0 * 3);
    } else if (bs_stop_freq == 14) {
        return FAAD_MIN(64, k0 * 2);
    } else {
        static const uint8_t stopMinTable[12] = { 13, 15, 20, 21, 23,
            32, 32, 35, 48, 64, 70, 96 };
        static const int8_t offset[12][14] = {
            { 0, 2, 4, 6, 8, 11, 14, 18, 22, 26, 31, 37, 44, 51 },
            { 0, 2, 4, 6, 8, 11, 14, 18, 22, 26, 31, 36, 42, 49 },
            { 0, 2, 4, 6, 8, 11, 14, 17, 21, 25, 29, 34, 39, 44 },
            { 0, 2, 4, 6, 8, 11, 14, 17, 20, 24, 28, 33, 38, 43 },
            { 0, 2, 4, 6, 8, 11, 14, 17, 20, 24, 28, 32, 36, 41 },
            { 0, 2, 4, 6, 8, 10, 12, 14, 17, 20, 23, 26, 29, 32 },
            { 0, 2, 4, 6, 8, 10, 12, 14, 17, 20, 23, 26, 29, 32 },
            { 0, 1, 3, 5, 7, 9, 11, 13, 15, 17, 20, 23, 26, 29 },
            { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16 },
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
            { 0, -1, -2, -3, -4, -5, -6, -6, -6, -6, -6, -6, -6, -6 },
            { 0, -3, -6, -9, -12, -15, -18, -20, -22, -24, -26, -28, -30, -32 }
        };
        uint8_t stopMin = stopMinTable[get_sr_index(sample_rate)];


        /* bs_stop_freq <= 13 */
        return FAAD_MIN(64, stopMin + offset[get_sr_index(sample_rate)][FAAD_MIN(bs_stop_freq, 13)]);
    }

    // return 0;
}

/* calculate the master frequency table from k0, k2, bs_freq_scale
   and bs_alter_scale

   version for bs_freq_scale = 0
*/
uint8_t master_frequency_table_fs0(sbr_info *sbr, uint8_t k0, uint8_t k2,
                                   uint8_t bs_alter_scale)
{
    int8_t incr;
    uint8_t k;
    uint8_t dk;
    int32_t nrBands, k2Achieved;
    int32_t k2Diff, vDk[64] = {0};

    /* mft only defined for k2 > k0 */
    if (k2 <= k0)
    {
        sbr->N_master = 0;
        return 1;
    }

    dk = bs_alter_scale ? 2 : 1;

    if (bs_alter_scale)
    {
        nrBands = (((k2-k0+2)>>2)<<1);
    } else {
        nrBands = (((k2-k0)>>1)<<1);
    }
    nrBands = FAAD_MIN(nrBands, 63);
    if (nrBands <= 0)
        return 1;

    k2Achieved = k0 + nrBands * dk;
    k2Diff = k2 - k2Achieved;
    for (k = 0; k < nrBands; k++)
        vDk[k] = dk;

    if (k2Diff)
    {
        incr = (k2Diff > 0) ? -1 : 1;
        k = (uint8_t) ((k2Diff > 0) ? (nrBands-1) : 0);

        while (k2Diff != 0)
        {
            vDk[k] -= incr;
            k += incr;
            k2Diff += incr;
        }
    }

    sbr->f_master[0] = k0;
    for (k = 1; k <= nrBands; k++)
        sbr->f_master[k] = (uint8_t)(sbr->f_master[k-1] + vDk[k-1]);

    sbr->N_master = (uint8_t)nrBands;
    sbr->N_master = (FAAD_MIN(sbr->N_master, 64));


    return 0;
}

/*
   This function finds the number of bands using this formula:
    bands * log(a1/a0)/log(2.0) + 0.5
*/
static int32_t find_bands(uint8_t warp, uint8_t bands, uint8_t a0, uint8_t a1)
{
    real_t div = (real_t)faad_log(2.0);
    if (warp) div *= (real_t)1.3;

    return (int32_t)(bands * faad_log((float)a1/(float)a0)/div + 0.5);
}

static real_t find_initial_power(uint8_t bands, uint8_t a0, uint8_t a1)
{
    return (real_t)faad_pow((real_t)a1/(real_t)a0, 1.0/(real_t)bands);
}

/*
   version for bs_freq_scale > 0
*/
uint8_t master_frequency_table(sbr_info *sbr, uint8_t k0, uint8_t k2,
                               uint8_t bs_freq_scale, uint8_t bs_alter_scale)
{
    uint8_t k, bands, twoRegions;
    uint8_t k1;
    uint8_t nrBand0, nrBand1;
    int32_t vDk0[64] = {0}, vDk1[64] = {0};
    int32_t vk0[64] = {0}, vk1[64] = {0};
    uint8_t temp1[] = { 6, 5, 4 };
    real_t q, qk;
    int32_t A_1;
    (void)bs_alter_scale;  /* TODO: remove parameter? */

    /* mft only defined for k2 > k0 */
    if (k2 <= k0)
    {
        sbr->N_master = 0;
        return 1;
    }

    bands = temp1[bs_freq_scale-1];

    if ((float)k2/(float)k0 > 2.2449)
    {
        twoRegions = 1;
        k1 = k0 << 1;
    } else {
        twoRegions = 0;
        k1 = k2;
    }

    nrBand0 = (uint8_t)(2 * find_bands(0, bands, k0, k1));
    nrBand0 = FAAD_MIN(nrBand0, 63);
    if (nrBand0 <= 0)
        return 1;

    q = find_initial_power(nrBand0, k0, k1);
    qk = REAL_CONST(k0);
    A_1 = (int32_t)(qk + .5);
    for (k = 0; k <= nrBand0; k++)
    {
        int32_t A_0 = A_1;
        qk *= q;
        A_1 = (int32_t)(qk + 0.5);
        vDk0[k] = A_1 - A_0;
    }

    /* needed? */
    qsort(vDk0, nrBand0, sizeof(vDk0[0]), int32cmp);

    vk0[0] = k0;
    for (k = 1; k <= nrBand0; k++)
    {
        vk0[k] = vk0[k-1] + vDk0[k-1];
        if (vDk0[k-1] == 0)
            return 1;
    }

    if (!twoRegions)
    {
        for (k = 0; k <= nrBand0; k++)
            sbr->f_master[k] = (uint8_t) vk0[k];

        sbr->N_master = nrBand0;
        sbr->N_master = FAAD_MIN(sbr->N_master, 64);
        return 0;
    }

    nrBand1 = (uint8_t)(2 * find_bands(1 /* warped */, bands, k1, k2));
    nrBand1 = FAAD_MIN(nrBand1, 63);

    q = find_initial_power(nrBand1, k1, k2);
    qk = REAL_CONST(k1);
    A_1 = (int32_t)(qk + .5);
    for (k = 0; k <= nrBand1 - 1; k++)
    {
        int32_t A_0 = A_1;
        qk *= q;
        A_1 = (int32_t)(qk + 0.5);
        vDk1[k] = A_1 - A_0;
    }

    if (vDk1[0] < vDk0[nrBand0 - 1])
    {
        int32_t change;

        /* needed? */
        qsort(vDk1, nrBand1 + 1, sizeof(vDk1[0]), int32cmp);
        change = vDk0[nrBand0 - 1] - vDk1[0];
        vDk1[0] = vDk0[nrBand0 - 1];
        vDk1[nrBand1 - 1] = vDk1[nrBand1 - 1] - change;
    }

    /* needed? */
    qsort(vDk1, nrBand1, sizeof(vDk1[0]), int32cmp);
    vk1[0] = k1;
    for (k = 1; k <= nrBand1; k++)
    {
        vk1[k] = vk1[k-1] + vDk1[k-1];
        if (vDk1[k-1] == 0)
            return 1;
    }

    sbr->N_master = nrBand0 + nrBand1;
    sbr->N_master = FAAD_MIN(sbr->N_master, 64);
    for (k = 0; k <= nrBand0; k++)
    {
        sbr->f_master[k] =  (uint8_t) vk0[k];
    }
    for (k = nrBand0 + 1; k <= sbr->N_master; k++)
    {
        sbr->f_master[k] = (uint8_t) vk1[k - nrBand0];
    }


    return 0;
}

/* calculate the derived frequency border tables from f_master */
uint8_t derived_frequency_table(sbr_info *sbr, uint8_t bs_xover_band,
                                uint8_t k2)
{
    uint8_t k, i;
    uint32_t minus;

    /* The following relation shall be satisfied: bs_xover_band < N_Master */
    if (sbr->N_master <= bs_xover_band)
        return 1;

    sbr->N_high = sbr->N_master - bs_xover_band;
    sbr->N_low = (sbr->N_high>>1) + (sbr->N_high - ((sbr->N_high>>1)<<1));

    sbr->n[0] = sbr->N_low;
    sbr->n[1] = sbr->N_high;

    for (k = 0; k <= sbr->N_high; k++)
    {
        sbr->f_table_res[HI_RES][k] = sbr->f_master[k + bs_xover_band];
    }

    sbr->M = sbr->f_table_res[HI_RES][sbr->N_high] - sbr->f_table_res[HI_RES][0];
    if (sbr->M > MAX_M)
        return 1;
    sbr->kx = sbr->f_table_res[HI_RES][0];
    if (sbr->kx > 32)
        return 1;
    if (sbr->kx + sbr->M > 64)
        return 1;

    minus = (sbr->N_high & 1) ? 1 : 0;

    i = 0;
    for (k = 0; k <= sbr->N_low; k++)
    {
        if (k != 0)
            i = (uint8_t)(2*k - minus);
        sbr->f_table_res[LO_RES][k] = sbr->f_table_res[HI_RES][i];
    }


    sbr->N_Q = 0;
    if (sbr->bs_noise_bands == 0)
    {
        sbr->N_Q = 1;
    } else {
        sbr->N_Q = (uint8_t)(FAAD_MAX(1, find_bands(0, sbr->bs_noise_bands, sbr->kx, k2)));
        sbr->N_Q = FAAD_MIN(5, sbr->N_Q);
    }

    i = 0;
    for (k = 0; k <= sbr->N_Q; k++)
    {
        if (k != 0)
            i = i + (sbr->N_low - i)/(sbr->N_Q + 1 - k);
        sbr->f_table_noise[k] = sbr->f_table_res[LO_RES][i];
    }

    /* build table for mapping k to g in hf patching */
    for (k = 0; k < 64; k++)
    {
        uint8_t g;
        for (g = 0; g < sbr->N_Q; g++)
        {
            if ((sbr->f_table_noise[g] <= k) &&
                (k < sbr->f_table_noise[g+1]))
            {
                sbr->table_map_k_to_g[k] = g;
                break;
            }
        }
    }


    return 0;
}

/* TODO: blegh, ugly */
/* Modified to calculate for all possible bs_limiter_bands always
 * This reduces the number calls to this functions needed (now only on
 * header reset)
 */
void limiter_frequency_table(sbr_info *sbr)
{
    static const real_t limiterBandsCompare[] = { REAL_CONST(1.327152),
        REAL_CONST(1.185093), REAL_CONST(1.119872) };
    uint8_t k, s;
    int8_t nrLim;

    sbr->f_table_lim[0][0] = sbr->f_table_res[LO_RES][0] - sbr->kx;
    sbr->f_table_lim[0][1] = sbr->f_table_res[LO_RES][sbr->N_low] - sbr->kx;
    sbr->N_L[0] = 1;


    for (s = 1; s < 4; s++)
    {
        uint8_t limTable[100 /*TODO*/] = {0};
        uint8_t patchBorders[64/*??*/] = {0};


        patchBorders[0] = sbr->kx;
        for (k = 1; k <= sbr->noPatches; k++)
        {
            patchBorders[k] = patchBorders[k-1] + sbr->patchNoSubbands[k-1];
        }

        for (k = 0; k <= sbr->N_low; k++)
        {
            limTable[k] = sbr->f_table_res[LO_RES][k];
        }
        for (k = 1; k < sbr->noPatches; k++)
        {
            limTable[k+sbr->N_low] = patchBorders[k];
        }

        /* needed */
        qsort(limTable, sbr->noPatches + sbr->N_low, sizeof(limTable[0]), uint8cmp);
        k = 1;
        nrLim = sbr->noPatches + sbr->N_low - 1;

        if (nrLim < 0) // TODO: BIG FAT PROBLEM
            return;

restart:
        if (k <= nrLim)
        {
            real_t nOctaves;

            if (limTable[k-1] != 0)
                nOctaves = (real_t)limTable[k]/(real_t)limTable[k-1];
            else
                nOctaves = 0;

            if (nOctaves < limiterBandsCompare[s - 1])
            {
                uint8_t i;
                if (limTable[k] != limTable[k-1])
                {
                    uint8_t found = 0, found2 = 0;
                    for (i = 0; i <= sbr->noPatches; i++)
                    {
                        if (limTable[k] == patchBorders[i])
                            found = 1;
                    }
                    if (found)
                    {
                        found2 = 0;
                        for (i = 0; i <= sbr->noPatches; i++)
                        {
                            if (limTable[k-1] == patchBorders[i])
                                found2 = 1;
                        }
                        if (found2)
                        {
                            k++;
                            goto restart;
                        } else {
                            /* remove (k-1)th element */
                            limTable[k-1] = sbr->f_table_res[LO_RES][sbr->N_low];
                            qsort(limTable, sbr->noPatches + sbr->N_low, sizeof(limTable[0]), uint8cmp);
                            nrLim--;
                            goto restart;
                        }
                    }
                }
                /* remove kth element */
                limTable[k] = sbr->f_table_res[LO_RES][sbr->N_low];
                qsort(limTable, nrLim, sizeof(limTable[0]), uint8cmp);
                nrLim--;
                goto restart;
            } else {
                k++;
                goto restart;
            }
        }

        sbr->N_L[s] = nrLim;
        for (k = 0; k <= nrLim; k++)
        {
            sbr->f_table_lim[s][k] = limTable[k] - sbr->kx;
        }

    }
}

} // namespace faad
} // namespace codec
