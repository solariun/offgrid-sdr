// ============================================================================
//  offgrid-codecs - spectral reconstruction (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacSpecrec.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  Inverse quantisation, scalefactor application, short-block
//  de-interleaving, then the tools (PNS, M/S, IS, TNS, DRC) and the filter
//  bank, per element.
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
** $Id: specrec.c,v 1.63 2010/06/04 20:47:56 menno Exp $
**/

#include <cstdlib>
#include "AacCommon.h"
#include "AacStructs.h"
#include "AacSpecrec.h"
#include "AacFiltbank.h"
#include "AacSyntax.h"
#include "AacIqTab.h"
#include "AacMs.h"
#include "AacIs.h"
#include "AacPns.h"
#include "AacTns.h"
#include "AacDrc.h"

namespace codec {
namespace faad {

/*
  Spectral reconstruction:
   - grouping/sectioning
   - inverse quantization
   - applying scalefactors
*/


/* static function declarations */
static uint8_t quant_to_spec(NeAACDecStruct *hDecoder,
                             ic_stream *ics, int16_t *quant_data,
                             real_t *spec_data, uint16_t frame_len);


static const uint8_t num_swb_960_window[] =
{
    40, 40, 45, 49, 49, 49, 46, 46, 42, 42, 42, 40
};

static const uint8_t num_swb_1024_window[] =
{
    41, 41, 47, 49, 49, 51, 47, 47, 43, 43, 43, 40
};

static const uint8_t num_swb_128_window[] =
{
    12, 12, 12, 14, 14, 14, 15, 15, 15, 15, 15, 15
};

static const uint16_t swb_offset_1024_96[] =
{
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56,
    64, 72, 80, 88, 96, 108, 120, 132, 144, 156, 172, 188, 212, 240,
    276, 320, 384, 448, 512, 576, 640, 704, 768, 832, 896, 960, 1024
};

static const uint16_t swb_offset_128_96[] =
{
    0, 4, 8, 12, 16, 20, 24, 32, 40, 48, 64, 92, 128
};

static const uint16_t swb_offset_1024_64[] =
{
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56,
    64, 72, 80, 88, 100, 112, 124, 140, 156, 172, 192, 216, 240, 268,
    304, 344, 384, 424, 464, 504, 544, 584, 624, 664, 704, 744, 784, 824,
    864, 904, 944, 984, 1024
};

static const uint16_t swb_offset_128_64[] =
{
    0, 4, 8, 12, 16, 20, 24, 32, 40, 48, 64, 92, 128
};

static const uint16_t swb_offset_1024_48[] =
{
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72,
    80, 88, 96, 108, 120, 132, 144, 160, 176, 196, 216, 240, 264, 292,
    320, 352, 384, 416, 448, 480, 512, 544, 576, 608, 640, 672, 704, 736,
    768, 800, 832, 864, 896, 928, 1024
};


static const uint16_t swb_offset_128_48[] =
{
    0, 4, 8, 12, 16, 20, 28, 36, 44, 56, 68, 80, 96, 112, 128
};

static const uint16_t swb_offset_1024_32[] =
{
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72,
    80, 88, 96, 108, 120, 132, 144, 160, 176, 196, 216, 240, 264, 292,
    320, 352, 384, 416, 448, 480, 512, 544, 576, 608, 640, 672, 704, 736,
    768, 800, 832, 864, 896, 928, 960, 992, 1024
};


static const uint16_t swb_offset_1024_24[] =
{
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 52, 60, 68,
    76, 84, 92, 100, 108, 116, 124, 136, 148, 160, 172, 188, 204, 220,
    240, 260, 284, 308, 336, 364, 396, 432, 468, 508, 552, 600, 652, 704,
    768, 832, 896, 960, 1024
};


static const uint16_t swb_offset_128_24[] =
{
    0, 4, 8, 12, 16, 20, 24, 28, 36, 44, 52, 64, 76, 92, 108, 128
};

static const uint16_t swb_offset_1024_16[] =
{
    0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 100, 112, 124,
    136, 148, 160, 172, 184, 196, 212, 228, 244, 260, 280, 300, 320, 344,
    368, 396, 424, 456, 492, 532, 572, 616, 664, 716, 772, 832, 896, 960, 1024
};

static const uint16_t swb_offset_128_16[] =
{
    0, 4, 8, 12, 16, 20, 24, 28, 32, 40, 48, 60, 72, 88, 108, 128
};

static const uint16_t swb_offset_1024_8[] =
{
    0, 12, 24, 36, 48, 60, 72, 84, 96, 108, 120, 132, 144, 156, 172,
    188, 204, 220, 236, 252, 268, 288, 308, 328, 348, 372, 396, 420, 448,
    476, 508, 544, 580, 620, 664, 712, 764, 820, 880, 944, 1024
};

static const uint16_t swb_offset_128_8[] =
{
    0, 4, 8, 12, 16, 20, 24, 28, 36, 44, 52, 60, 72, 88, 108, 128
};

static const uint16_t *swb_offset_1024_window[] =
{
    swb_offset_1024_96,      /* 96000 */
    swb_offset_1024_96,      /* 88200 */
    swb_offset_1024_64,      /* 64000 */
    swb_offset_1024_48,      /* 48000 */
    swb_offset_1024_48,      /* 44100 */
    swb_offset_1024_32,      /* 32000 */
    swb_offset_1024_24,      /* 24000 */
    swb_offset_1024_24,      /* 22050 */
    swb_offset_1024_16,      /* 16000 */
    swb_offset_1024_16,      /* 12000 */
    swb_offset_1024_16,      /* 11025 */
    swb_offset_1024_8        /* 8000  */
};


static const  uint16_t *swb_offset_128_window[] =
{
    swb_offset_128_96,       /* 96000 */
    swb_offset_128_96,       /* 88200 */
    swb_offset_128_64,       /* 64000 */
    swb_offset_128_48,       /* 48000 */
    swb_offset_128_48,       /* 44100 */
    swb_offset_128_48,       /* 32000 */
    swb_offset_128_24,       /* 24000 */
    swb_offset_128_24,       /* 22050 */
    swb_offset_128_16,       /* 16000 */
    swb_offset_128_16,       /* 12000 */
    swb_offset_128_16,       /* 11025 */
    swb_offset_128_8         /* 8000  */
};

#define bit_set(A, B) ((A) & (1<<(B)))

/* 4.5.2.3.4 */
/*
  - determine the number of windows in a window_sequence named num_windows
  - determine the number of window_groups named num_window_groups
  - determine the number of windows in each group named window_group_length[g]
  - determine the total number of scalefactor window bands named num_swb for
    the actual window type
  - determine swb_offset[swb], the offset of the first coefficient in
    scalefactor window band named swb of the window actually used
  - determine sect_sfb_offset[g][section],the offset of the first coefficient
    in section named section. This offset depends on window_sequence and
    scale_factor_grouping and is needed to decode the spectral_data().
*/
uint8_t window_grouping_info(NeAACDecStruct *hDecoder, ic_stream *ics)
{
    uint8_t i, g;

    uint8_t sf_index = hDecoder->sf_index;

    if (sf_index >= 12)
        return 32;

    switch (ics->window_sequence) {
    case ONLY_LONG_SEQUENCE:
    case LONG_START_SEQUENCE:
    case LONG_STOP_SEQUENCE:
        ics->num_windows = 1;
        ics->num_window_groups = 1;
        ics->window_group_length[ics->num_window_groups-1] = 1;
            if (hDecoder->frameLength == 1024)
                ics->num_swb = num_swb_1024_window[sf_index];
            else /* if (hDecoder->frameLength == 960) */
                ics->num_swb = num_swb_960_window[sf_index];

        if (ics->max_sfb > ics->num_swb)
        {
            return 32;
        }

        /* preparation of sect_sfb_offset for long blocks */
        /* also copy the last value! */
            for (i = 0; i < ics->num_swb; i++)
            {
                ics->sect_sfb_offset[0][i] = swb_offset_1024_window[sf_index][i];
                ics->swb_offset[i] = swb_offset_1024_window[sf_index][i];
            }
            ics->sect_sfb_offset[0][ics->num_swb] = hDecoder->frameLength;
            ics->swb_offset[ics->num_swb] = hDecoder->frameLength;
            ics->swb_offset_max = hDecoder->frameLength;
        if (ics->num_swb > 0 && ics->swb_offset[ics->num_swb] < ics->swb_offset[ics->num_swb-1]) {
            return 32;
        }
        return 0;
    case EIGHT_SHORT_SEQUENCE:
        ics->num_windows = 8;
        ics->num_window_groups = 1;
        ics->window_group_length[ics->num_window_groups-1] = 1;
        ics->num_swb = num_swb_128_window[sf_index];

        if (ics->max_sfb > ics->num_swb)
        {
            return 32;
        }

        for (i = 0; i < ics->num_swb; i++)
            ics->swb_offset[i] = swb_offset_128_window[sf_index][i];
        ics->swb_offset[ics->num_swb] = hDecoder->frameLength/8;
        ics->swb_offset_max = hDecoder->frameLength/8;

        for (i = 0; i < ics->num_windows-1; i++) {
            if (bit_set(ics->scale_factor_grouping, 6-i) == 0)
            {
                ics->num_window_groups += 1;
                ics->window_group_length[ics->num_window_groups-1] = 1;
            } else {
                ics->window_group_length[ics->num_window_groups-1] += 1;
            }
        }

        /* preparation of sect_sfb_offset for short blocks */
        for (g = 0; g < ics->num_window_groups; g++)
        {
            uint16_t width;
            uint8_t sect_sfb = 0;
            uint16_t offset = 0;

            for (i = 0; i < ics->num_swb; i++)
            {
                if (i+1 == ics->num_swb)
                {
                    width = (hDecoder->frameLength/8) - swb_offset_128_window[sf_index][i];
                } else {
                    width = swb_offset_128_window[sf_index][i+1] -
                        swb_offset_128_window[sf_index][i];
                }
                width *= ics->window_group_length[g];
                ics->sect_sfb_offset[g][sect_sfb++] = offset;
                offset += width;
            }
            ics->sect_sfb_offset[g][sect_sfb] = offset;
        }
        return 0;
    default:
        return 32;
    }
}

/* iquant() * output = sign(input)*abs(input)^(4/3) */
static inline real_t iquant(int16_t q, const real_t *tab, uint8_t *error)
{
    if (q < 0)
    {
        /* tab contains a value for all possible q [0,8192] */
        if (-q < IQ_TABLE_SIZE)
            return -tab[-q];

        *error = 17;
        return 0;
    } else {
        /* tab contains a value for all possible q [0,8192] */
        if (q < IQ_TABLE_SIZE)
            return tab[q];

        *error = 17;
        return 0;
    }
}

static const real_t pow2sf_tab[] = {
    2.9802322387695313E-008, 5.9604644775390625E-008, 1.1920928955078125E-007,
    2.384185791015625E-007, 4.76837158203125E-007, 9.5367431640625E-007,
    1.9073486328125E-006, 3.814697265625E-006, 7.62939453125E-006,
    1.52587890625E-005, 3.0517578125E-005, 6.103515625E-005,
    0.0001220703125, 0.000244140625, 0.00048828125,
    0.0009765625, 0.001953125, 0.00390625,
    0.0078125, 0.015625, 0.03125,
    0.0625, 0.125, 0.25,
    0.5, 1.0, 2.0,
    4.0, 8.0, 16.0, 32.0,
    64.0, 128.0, 256.0,
    512.0, 1024.0, 2048.0,
    4096.0, 8192.0, 16384.0,
    32768.0, 65536.0, 131072.0,
    262144.0, 524288.0, 1048576.0,
    2097152.0, 4194304.0, 8388608.0,
    16777216.0, 33554432.0, 67108864.0,
    134217728.0, 268435456.0, 536870912.0,
    1073741824.0, 2147483648.0, 4294967296.0,
    8589934592.0, 17179869184.0, 34359738368.0,
    68719476736.0, 137438953472.0, 274877906944.0
};

/* quant_to_spec: perform dequantisation and scaling
 * and in case of short block it also does the deinterleaving
 */
/*
  For ONLY_LONG_SEQUENCE windows (num_window_groups = 1,
  window_group_length[0] = 1) the spectral data is in ascending spectral
  order.
  For the EIGHT_SHORT_SEQUENCE window, the spectral order depends on the
  grouping in the following manner:
  - Groups are ordered sequentially
  - Within a group, a scalefactor band consists of the spectral data of all
    grouped SHORT_WINDOWs for the associated scalefactor window band. To
    clarify via example, the length of a group is in the range of one to eight
    SHORT_WINDOWs.
  - If there are eight groups each with length one (num_window_groups = 8,
    window_group_length[0..7] = 1), the result is a sequence of eight spectra,
    each in ascending spectral order.
  - If there is only one group with length eight (num_window_groups = 1,
    window_group_length[0] = 8), the result is that spectral data of all eight
    SHORT_WINDOWs is interleaved by scalefactor window bands.
  - Within a scalefactor window band, the coefficients are in ascending
    spectral order.
*/
static uint8_t quant_to_spec(NeAACDecStruct *hDecoder,
                             ic_stream *ics, int16_t *quant_data,
                             real_t *spec_data, uint16_t frame_len)
{
    static const real_t pow2_table[] =
    {
        COEF_CONST(1.0),
        COEF_CONST(1.1892071150027210667174999705605), /* 2^0.25 */
        COEF_CONST(1.4142135623730950488016887242097), /* 2^0.5 */
        COEF_CONST(1.6817928305074290860622509524664) /* 2^0.75 */
    };
    const real_t *tab = iq_table;

    uint8_t g, sfb, win;
    uint16_t width, bin, k, gindex;
    uint8_t error = 0; /* Init error flag */
    real_t scf;

    k = 0;
    gindex = 0;

    /* In this case quant_to_spec is no-op and spec_data remains undefined.
     * Without peeking into AAC specification, there is no strong evidence if
     * such streams are invalid -> just calm down MSAN. */
    if (ics->num_swb == 0)
        memset(spec_data, 0, frame_len * sizeof(real_t));

    for (g = 0; g < ics->num_window_groups; g++)
    {
        uint16_t j = 0;
        uint16_t gincrease = 0;
        uint16_t win_inc = ics->swb_offset[ics->num_swb];

        for (sfb = 0; sfb < ics->num_swb; sfb++)
        {
            int32_t exp, frac;
            uint16_t wa = gindex + j;
            int16_t scale_factor = ics->scale_factors[g][sfb];

            width = ics->swb_offset[sfb+1] - ics->swb_offset[sfb];
            if (width + 3 >= 1024) 
            {
                // quant_data contains 1024 uint16_t, the k iterator + 3
                // should never reach more 1024
                error = 17;
                continue;
            }

            (void)hDecoder;

            /* scale_factor for IS or PNS, has different meaning; fill with almost zeroes */
            if (is_intensity(ics, g, sfb) || is_noise(ics, g, sfb))
            {
                scale_factor = 0;
            }

            /* scale_factor must be between 0 and 255 */
            exp = (scale_factor /* - 100 */) >> 2;
            /* frac must always be > 0 */
            frac = (scale_factor /* - 100 */) & 3;

            scf = pow2sf_tab[exp/*+25*/] * pow2_table[frac];

            for (win = 0; win < ics->window_group_length[g]; win++)
            {
                for (bin = 0; bin < width; bin += 4)
                {
                    uint16_t wb = wa + bin;
                    spec_data[wb+0] = iquant(quant_data[k+0], tab, &error) * scf;
                    spec_data[wb+1] = iquant(quant_data[k+1], tab, &error) * scf;
                    spec_data[wb+2] = iquant(quant_data[k+2], tab, &error) * scf;
                    spec_data[wb+3] = iquant(quant_data[k+3], tab, &error) * scf;

                    gincrease += 4;
                    k += 4;
                }
                wa += win_inc;
            }
            j += width;
        }
        gindex += gincrease;
    }

    return error;
}

static uint8_t allocate_single_channel(NeAACDecStruct *hDecoder, uint8_t channel,
                                       uint8_t output_channels)
{
    int mul = 1;


    if (hDecoder->time_out[channel] != NULL)
    {
        faad_free(hDecoder->time_out[channel]);
        hDecoder->time_out[channel] = NULL;
    }

    {
        mul = 1;
        hDecoder->sbr_alloced[hDecoder->fr_ch_ele] = 0;
        if ((hDecoder->sbr_present_flag == 1) || (hDecoder->forceUpSampling == 1))
        {
            /* SBR requires 2 times as much output data */
            mul = 2;
            hDecoder->sbr_alloced[hDecoder->fr_ch_ele] = 1;
        }
        hDecoder->time_out[channel] = (real_t*)faad_malloc(mul*hDecoder->frameLength*sizeof(real_t));
        memset(hDecoder->time_out[channel], 0, mul*hDecoder->frameLength*sizeof(real_t));
    }

    if (output_channels == 2)
    {
        if (hDecoder->time_out[channel+1] != NULL)
        {
            faad_free(hDecoder->time_out[channel+1]);
            hDecoder->time_out[channel+1] = NULL;
        }

        hDecoder->time_out[channel+1] = (real_t*)faad_malloc(mul*hDecoder->frameLength*sizeof(real_t));
        memset(hDecoder->time_out[channel+1], 0, mul*hDecoder->frameLength*sizeof(real_t));
    }

    if (hDecoder->fb_intermed[channel] != NULL)
    {
        faad_free(hDecoder->fb_intermed[channel]);
        hDecoder->fb_intermed[channel] = NULL;
    }

    hDecoder->fb_intermed[channel] = (real_t*)faad_malloc(hDecoder->frameLength*sizeof(real_t));
    memset(hDecoder->fb_intermed[channel], 0, hDecoder->frameLength*sizeof(real_t));


    return 0;
}

static uint8_t allocate_channel_pair(NeAACDecStruct *hDecoder,
                                     uint8_t channel, uint8_t paired_channel)
{
    int mul = 1;


    {
        mul = 1;
        hDecoder->sbr_alloced[hDecoder->fr_ch_ele] = 0;
        if ((hDecoder->sbr_present_flag == 1) || (hDecoder->forceUpSampling == 1))
        {
            /* SBR requires 2 times as much output data */
            mul = 2;
            hDecoder->sbr_alloced[hDecoder->fr_ch_ele] = 1;
        }
    }
    if (hDecoder->time_out[channel] != NULL)
    {
        faad_free(hDecoder->time_out[channel]);
        hDecoder->time_out[channel] = NULL;
    }
    if (hDecoder->time_out[paired_channel] != NULL)
    {
        faad_free(hDecoder->time_out[paired_channel]);
        hDecoder->time_out[paired_channel] = NULL;
    }
    hDecoder->time_out[channel] = (real_t*)faad_malloc(mul*hDecoder->frameLength*sizeof(real_t));
    memset(hDecoder->time_out[channel], 0, mul*hDecoder->frameLength*sizeof(real_t));
    hDecoder->time_out[paired_channel] = (real_t*)faad_malloc(mul*hDecoder->frameLength*sizeof(real_t));
    memset(hDecoder->time_out[paired_channel], 0, mul*hDecoder->frameLength*sizeof(real_t));

    if (hDecoder->fb_intermed[channel] != NULL)
    {
        faad_free(hDecoder->fb_intermed[channel]);
        hDecoder->fb_intermed[channel] = NULL;
    }
    if (hDecoder->fb_intermed[paired_channel] != NULL)
    {
        faad_free(hDecoder->fb_intermed[paired_channel]);
        hDecoder->fb_intermed[paired_channel] = NULL;
    }
    hDecoder->fb_intermed[channel] = (real_t*)faad_malloc(hDecoder->frameLength*sizeof(real_t));
    memset(hDecoder->fb_intermed[channel], 0, hDecoder->frameLength*sizeof(real_t));
    hDecoder->fb_intermed[paired_channel] = (real_t*)faad_malloc(hDecoder->frameLength*sizeof(real_t));
    memset(hDecoder->fb_intermed[paired_channel], 0, hDecoder->frameLength*sizeof(real_t));


    return 0;
}

uint8_t reconstruct_single_channel(NeAACDecStruct *hDecoder, ic_stream *ics,
                                   element *sce, int16_t *spec_data)
{
    uint8_t retval;
    uint8_t output_channels;
    real_t spec_coef[1024];


    /* always allocate 2 channels, PS can always "suddenly" turn up */
    output_channels = 2;

    if (hDecoder->element_output_channels[hDecoder->fr_ch_ele] == 0)
    {
        /* element_output_channels not set yet */
        hDecoder->element_output_channels[hDecoder->fr_ch_ele] = output_channels;
    } else if (hDecoder->element_output_channels[hDecoder->fr_ch_ele] != output_channels) {
        /* element inconsistency */

        /* this only happens if PS is actually found but not in the first frame
         * this means that there is only 1 bitstream element!
         */

        /* The simplest way to fix the accounting,
         * is to reallocate this and all the following channels.
         */
        memset(&hDecoder->element_alloced[hDecoder->fr_ch_ele], 0,
            sizeof(uint8_t) * (MAX_SYNTAX_ELEMENTS - hDecoder->fr_ch_ele));

        hDecoder->element_output_channels[hDecoder->fr_ch_ele] = output_channels;

        //return 21;
    }

    if (hDecoder->element_alloced[hDecoder->fr_ch_ele] == 0)
    {
        retval = allocate_single_channel(hDecoder, sce->channel, output_channels);
        if (retval > 0)
            return retval;

        hDecoder->element_alloced[hDecoder->fr_ch_ele] = 1;
    }

    /* sanity check, CVE-2018-20199, CVE-2018-20360 */
    if(!hDecoder->time_out[sce->channel])
        return 15;
    if(output_channels > 1 && !hDecoder->time_out[sce->channel+1])
        return 15;
    if(!hDecoder->fb_intermed[sce->channel])
        return 15;

    /* dequantisation and scaling */
    retval = quant_to_spec(hDecoder, ics, spec_data, spec_coef, hDecoder->frameLength);
    if (retval > 0)
        return retval;


    /* pns decoding */
    pns_decode(ics, NULL, spec_coef, NULL, hDecoder->frameLength, 0, hDecoder->object_type,
        &(hDecoder->rng1), &(hDecoder->rng2));


    /* tns decoding */
    tns_decode_frame(ics, &(ics->tns), hDecoder->sf_index, hDecoder->object_type,
        spec_coef, hDecoder->frameLength);

    /* drc decoding */
    if (hDecoder->drc->present)
    {
        if (!hDecoder->drc->exclude_mask[sce->channel] || !hDecoder->drc->excluded_chns_present)
            drc_decode(hDecoder->drc, spec_coef);
    }
    /* filter bank */
        ifilter_bank(hDecoder->fb, ics->window_sequence, ics->window_shape,
            hDecoder->window_shape_prev[sce->channel], spec_coef,
            hDecoder->time_out[sce->channel], hDecoder->fb_intermed[sce->channel],
            hDecoder->object_type, hDecoder->frameLength);

    /* save window shape for next frame */
    hDecoder->window_shape_prev[sce->channel] = ics->window_shape;


    if (((hDecoder->sbr_present_flag == 1) || (hDecoder->forceUpSampling == 1))
        && hDecoder->sbr_alloced[hDecoder->fr_ch_ele])
    {
        int ele = hDecoder->fr_ch_ele;
        int ch = sce->channel;

        /* following case can happen when forceUpSampling == 1 */
        if (hDecoder->sbr[ele] == NULL)
        {
            hDecoder->sbr[ele] = sbrDecodeInit(hDecoder->frameLength,
                hDecoder->element_id[ele], 2*get_sample_rate(hDecoder->sf_index),
                hDecoder->downSampledSBR
                , 0
                );
        }
        if (!hDecoder->sbr[ele])
            return 19;

        if (sce->ics1.window_sequence == EIGHT_SHORT_SEQUENCE)
            hDecoder->sbr[ele]->maxAACLine = 8*FAAD_MIN(sce->ics1.swb_offset[FAAD_MAX(sce->ics1.max_sfb-1, 0)], sce->ics1.swb_offset_max);
        else
            hDecoder->sbr[ele]->maxAACLine = FAAD_MIN(sce->ics1.swb_offset[FAAD_MAX(sce->ics1.max_sfb-1, 0)], sce->ics1.swb_offset_max);

        /* check if any of the PS tools is used */
        if (hDecoder->ps_used[ele] == 0)
        {
            retval = sbrDecodeSingleFrame(hDecoder->sbr[ele], hDecoder->time_out[ch],
                hDecoder->postSeekResetFlag, hDecoder->downSampledSBR);
        } else {
            retval = sbrDecodeSingleFramePS(hDecoder->sbr[ele], hDecoder->time_out[ch],
                hDecoder->time_out[ch+1], hDecoder->postSeekResetFlag,
                hDecoder->downSampledSBR);
        }
        if (retval > 0)
            return retval;
    } else if (((hDecoder->sbr_present_flag == 1) || (hDecoder->forceUpSampling == 1))
        && !hDecoder->sbr_alloced[hDecoder->fr_ch_ele])
    {
        return 23;
    }

    /* copy L to R when no PS is used */
    if ((hDecoder->ps_used[hDecoder->fr_ch_ele] == 0) &&
        (hDecoder->element_output_channels[hDecoder->fr_ch_ele] == 2))
    {
        int ele = hDecoder->fr_ch_ele;
        int ch = sce->channel;
        int frame_size = 1;
        if (hDecoder->sbr_alloced[ele])
            frame_size = 2;
        frame_size *= hDecoder->frameLength*sizeof(real_t);

        memcpy(hDecoder->time_out[ch+1], hDecoder->time_out[ch], frame_size);
    }

    return 0;
}

uint8_t reconstruct_channel_pair(NeAACDecStruct *hDecoder, ic_stream *ics1, ic_stream *ics2,
                                 element *cpe, int16_t *spec_data1, int16_t *spec_data2)
{
    uint8_t retval;
    real_t spec_coef1[1024];
    real_t spec_coef2[1024];

    if (hDecoder->element_alloced[hDecoder->fr_ch_ele] != 2)
    {
        retval = allocate_channel_pair(hDecoder, cpe->channel, (uint8_t)cpe->paired_channel);
        if (retval > 0)
            return retval;

        hDecoder->element_alloced[hDecoder->fr_ch_ele] = 2;
    }

    /* sanity check, CVE-2018-20199, CVE-2018-20360 */
    if(!hDecoder->time_out[cpe->channel] || !hDecoder->time_out[cpe->paired_channel])
        return 15;
    if(!hDecoder->fb_intermed[cpe->channel] || !hDecoder->fb_intermed[cpe->paired_channel])
        return 15;

    /* dequantisation and scaling */
    retval = quant_to_spec(hDecoder, ics1, spec_data1, spec_coef1, hDecoder->frameLength);
    if (retval > 0)
        return retval;
    retval = quant_to_spec(hDecoder, ics2, spec_data2, spec_coef2, hDecoder->frameLength);
    if (retval > 0)
        return retval;


    /* pns decoding */
    if (ics1->ms_mask_present)
    {
        pns_decode(ics1, ics2, spec_coef1, spec_coef2, hDecoder->frameLength, 1, hDecoder->object_type,
            &(hDecoder->rng1), &(hDecoder->rng2));
    } else {
        pns_decode(ics1, ics2, spec_coef1, spec_coef2, hDecoder->frameLength, 0, hDecoder->object_type,
            &(hDecoder->rng1), &(hDecoder->rng2));
    }

    /* mid/side decoding */
    ms_decode(ics1, ics2, spec_coef1, spec_coef2, hDecoder->frameLength);


    /* intensity stereo decoding */
    is_decode(ics1, ics2, spec_coef1, spec_coef2, hDecoder->frameLength);


    /* tns decoding */
    tns_decode_frame(ics1, &(ics1->tns), hDecoder->sf_index, hDecoder->object_type,
        spec_coef1, hDecoder->frameLength);
    tns_decode_frame(ics2, &(ics2->tns), hDecoder->sf_index, hDecoder->object_type,
        spec_coef2, hDecoder->frameLength);

    /* drc decoding */
    if (hDecoder->drc->present)
    {
        if (!hDecoder->drc->exclude_mask[cpe->channel] || !hDecoder->drc->excluded_chns_present)
            drc_decode(hDecoder->drc, spec_coef1);
        if (!hDecoder->drc->exclude_mask[cpe->paired_channel] || !hDecoder->drc->excluded_chns_present)
            drc_decode(hDecoder->drc, spec_coef2);
    }
    /* filter bank */
        ifilter_bank(hDecoder->fb, ics1->window_sequence, ics1->window_shape,
            hDecoder->window_shape_prev[cpe->channel], spec_coef1,
            hDecoder->time_out[cpe->channel], hDecoder->fb_intermed[cpe->channel],
            hDecoder->object_type, hDecoder->frameLength);
        ifilter_bank(hDecoder->fb, ics2->window_sequence, ics2->window_shape,
            hDecoder->window_shape_prev[cpe->paired_channel], spec_coef2,
            hDecoder->time_out[cpe->paired_channel], hDecoder->fb_intermed[cpe->paired_channel],
            hDecoder->object_type, hDecoder->frameLength);

    /* save window shape for next frame */
    hDecoder->window_shape_prev[cpe->channel] = ics1->window_shape;
    hDecoder->window_shape_prev[cpe->paired_channel] = ics2->window_shape;


    if (((hDecoder->sbr_present_flag == 1) || (hDecoder->forceUpSampling == 1))
        && hDecoder->sbr_alloced[hDecoder->fr_ch_ele])
    {
        int ele = hDecoder->fr_ch_ele;
        int ch0 = cpe->channel;
        int ch1 = cpe->paired_channel;

        /* following case can happen when forceUpSampling == 1 */
        if (hDecoder->sbr[ele] == NULL)
        {
            hDecoder->sbr[ele] = sbrDecodeInit(hDecoder->frameLength,
                hDecoder->element_id[ele], 2*get_sample_rate(hDecoder->sf_index),
                hDecoder->downSampledSBR
                , 0
                );
        }
        if (!hDecoder->sbr[ele])
            return 19;

        if (cpe->ics1.window_sequence == EIGHT_SHORT_SEQUENCE)
            hDecoder->sbr[ele]->maxAACLine = 8*FAAD_MIN(cpe->ics1.swb_offset[FAAD_MAX(cpe->ics1.max_sfb-1, 0)], cpe->ics1.swb_offset_max);
        else
            hDecoder->sbr[ele]->maxAACLine = FAAD_MIN(cpe->ics1.swb_offset[FAAD_MAX(cpe->ics1.max_sfb-1, 0)], cpe->ics1.swb_offset_max);

        retval = sbrDecodeCoupleFrame(hDecoder->sbr[ele],
            hDecoder->time_out[ch0], hDecoder->time_out[ch1],
            hDecoder->postSeekResetFlag, hDecoder->downSampledSBR);
        if (retval > 0)
            return retval;
    } else if (((hDecoder->sbr_present_flag == 1) || (hDecoder->forceUpSampling == 1))
        && !hDecoder->sbr_alloced[hDecoder->fr_ch_ele])
    {
        return 23;
    }

    return 0;
}

} // namespace faad
} // namespace codec
