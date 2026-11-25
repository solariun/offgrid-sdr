// ============================================================================
//  offgrid-codecs - PCM output (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacOutput.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  Turns the decoded channels into interleaved float PCM (scaled to -1..1),
//  with mono duplicated to both channels as faad2 does.
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
** $Id: output.c,v 1.47 2009/01/26 23:51:15 menno Exp $
**/

#include "AacCommon.h"
#include "AacStructs.h"
#include "AacOutput.h"

namespace codec {
namespace faad {

#define FLOAT_SCALE (1.0f/(1<<15))

#define DM_MUL REAL_CONST(0.3203772410170407) // 1/(1+sqrt(2) + 1/sqrt(2))
#define RSQRT2 REAL_CONST(0.7071067811865475244) // 1/sqrt(2)


static inline real_t get_sample(real_t **input, uint8_t channel, uint16_t sample,
                                uint8_t down_matrix, uint8_t *internal_channel)
{
    if (!down_matrix)
        return input[internal_channel[channel]][sample];

    if (channel == 0)
    {
        return DM_MUL * (input[internal_channel[1]][sample] +
            input[internal_channel[0]][sample] * RSQRT2 +
            input[internal_channel[3]][sample] * RSQRT2);
    } else {
        return DM_MUL * (input[internal_channel[2]][sample] +
            input[internal_channel[0]][sample] * RSQRT2 +
            input[internal_channel[4]][sample] * RSQRT2);
    }
}

#define CLIP(sample, max, min) \
if (sample >= 0.0f)            \
{                              \
    if (sample >= max)         \
        sample = max;          \
} else {                       \
    if (sample <= min)         \
        sample = min;          \
}

#define CONV(a,b) ((a<<1)|(b&0x1))

static void to_PCM_16bit(NeAACDecStruct *hDecoder, real_t **input,
                         uint8_t channels, uint16_t frame_len,
                         int16_t **sample_buffer)
{
    uint8_t ch, ch1;
    uint16_t i;

    switch (CONV(channels,hDecoder->downMatrix))
    {
    case CONV(1,0):
    case CONV(1,1):
        for(i = 0; i < frame_len; i++)
        {
            real_t inp = input[hDecoder->internal_channel[0]][i];

            CLIP(inp, 32767.0f, -32768.0f);

            (*sample_buffer)[i] = (int16_t)lrintf(inp);
        }
        break;
    case CONV(2,0):
        if (hDecoder->upMatrix)
        {
            ch  = hDecoder->internal_channel[0];
            for(i = 0; i < frame_len; i++)
            {
                real_t inp0 = input[ch][i];

                CLIP(inp0, 32767.0f, -32768.0f);

                (*sample_buffer)[(i*2)+0] = (int16_t)lrintf(inp0);
                (*sample_buffer)[(i*2)+1] = (int16_t)lrintf(inp0);
            }
        } else {
            ch  = hDecoder->internal_channel[0];
            ch1 = hDecoder->internal_channel[1];
            for(i = 0; i < frame_len; i++)
            {
                real_t inp0 = input[ch ][i];
                real_t inp1 = input[ch1][i];

                CLIP(inp0, 32767.0f, -32768.0f);
                CLIP(inp1, 32767.0f, -32768.0f);

                (*sample_buffer)[(i*2)+0] = (int16_t)lrintf(inp0);
                (*sample_buffer)[(i*2)+1] = (int16_t)lrintf(inp1);
            }
        }
        break;
    default:
        for (ch = 0; ch < channels; ch++)
        {
            for(i = 0; i < frame_len; i++)
            {
                real_t inp = get_sample(input, ch, i, hDecoder->downMatrix, hDecoder->internal_channel);

                CLIP(inp, 32767.0f, -32768.0f);

                (*sample_buffer)[(i*channels)+ch] = (int16_t)lrintf(inp);
            }
        }
        break;
    }
}

static void to_PCM_24bit(NeAACDecStruct *hDecoder, real_t **input,
                         uint8_t channels, uint16_t frame_len,
                         int32_t **sample_buffer)
{
    uint8_t ch, ch1;
    uint16_t i;

    switch (CONV(channels,hDecoder->downMatrix))
    {
    case CONV(1,0):
    case CONV(1,1):
        for(i = 0; i < frame_len; i++)
        {
            real_t inp = input[hDecoder->internal_channel[0]][i];

            inp *= 256.0f;
            CLIP(inp, 8388607.0f, -8388608.0f);

            (*sample_buffer)[i] = (int32_t)lrintf(inp);
        }
        break;
    case CONV(2,0):
        if (hDecoder->upMatrix)
        {
            ch = hDecoder->internal_channel[0];
            for(i = 0; i < frame_len; i++)
            {
                real_t inp0 = input[ch][i];

                inp0 *= 256.0f;
                CLIP(inp0, 8388607.0f, -8388608.0f);

                (*sample_buffer)[(i*2)+0] = (int32_t)lrintf(inp0);
                (*sample_buffer)[(i*2)+1] = (int32_t)lrintf(inp0);
            }
        } else {
            ch  = hDecoder->internal_channel[0];
            ch1 = hDecoder->internal_channel[1];
            for(i = 0; i < frame_len; i++)
            {
                real_t inp0 = input[ch ][i];
                real_t inp1 = input[ch1][i];

                inp0 *= 256.0f;
                inp1 *= 256.0f;
                CLIP(inp0, 8388607.0f, -8388608.0f);
                CLIP(inp1, 8388607.0f, -8388608.0f);

                (*sample_buffer)[(i*2)+0] = (int32_t)lrintf(inp0);
                (*sample_buffer)[(i*2)+1] = (int32_t)lrintf(inp1);
            }
        }
        break;
    default:
        for (ch = 0; ch < channels; ch++)
        {
            for(i = 0; i < frame_len; i++)
            {
                real_t inp = get_sample(input, ch, i, hDecoder->downMatrix, hDecoder->internal_channel);

                inp *= 256.0f;
                CLIP(inp, 8388607.0f, -8388608.0f);

                (*sample_buffer)[(i*channels)+ch] = (int32_t)lrintf(inp);
            }
        }
        break;
    }
}

static void to_PCM_32bit(NeAACDecStruct *hDecoder, real_t **input,
                         uint8_t channels, uint16_t frame_len,
                         int32_t **sample_buffer)
{
    uint8_t ch, ch1;
    uint16_t i;

    switch (CONV(channels,hDecoder->downMatrix))
    {
    case CONV(1,0):
    case CONV(1,1):
        for(i = 0; i < frame_len; i++)
        {
            real_t inp = input[hDecoder->internal_channel[0]][i];

            inp *= 65536.0f;
            CLIP(inp, 2147483647.0f, -2147483648.0f);

            (*sample_buffer)[i] = (int32_t)lrintf(inp);
        }
        break;
    case CONV(2,0):
        if (hDecoder->upMatrix)
        {
            ch = hDecoder->internal_channel[0];
            for(i = 0; i < frame_len; i++)
            {
                real_t inp0 = input[ch][i];

                inp0 *= 65536.0f;
                CLIP(inp0, 2147483647.0f, -2147483648.0f);

                (*sample_buffer)[(i*2)+0] = (int32_t)lrintf(inp0);
                (*sample_buffer)[(i*2)+1] = (int32_t)lrintf(inp0);
            }
        } else {
            ch  = hDecoder->internal_channel[0];
            ch1 = hDecoder->internal_channel[1];
            for(i = 0; i < frame_len; i++)
            {
                real_t inp0 = input[ch ][i];
                real_t inp1 = input[ch1][i];

                inp0 *= 65536.0f;
                inp1 *= 65536.0f;
                CLIP(inp0, 2147483647.0f, -2147483648.0f);
                CLIP(inp1, 2147483647.0f, -2147483648.0f);

                (*sample_buffer)[(i*2)+0] = (int32_t)lrintf(inp0);
                (*sample_buffer)[(i*2)+1] = (int32_t)lrintf(inp1);
            }
        }
        break;
    default:
        for (ch = 0; ch < channels; ch++)
        {
            for(i = 0; i < frame_len; i++)
            {
                real_t inp = get_sample(input, ch, i, hDecoder->downMatrix, hDecoder->internal_channel);

                inp *= 65536.0f;
                CLIP(inp, 2147483647.0f, -2147483648.0f);

                (*sample_buffer)[(i*channels)+ch] = (int32_t)lrintf(inp);
            }
        }
        break;
    }
}

static void to_PCM_float(NeAACDecStruct *hDecoder, real_t **input,
                         uint8_t channels, uint16_t frame_len,
                         float32_t **sample_buffer)
{
    uint8_t ch, ch1;
    uint16_t i;

    switch (CONV(channels,hDecoder->downMatrix))
    {
    case CONV(1,0):
    case CONV(1,1):
        for(i = 0; i < frame_len; i++)
        {
            real_t inp = input[hDecoder->internal_channel[0]][i];
            (*sample_buffer)[i] = inp*FLOAT_SCALE;
        }
        break;
    case CONV(2,0):
        if (hDecoder->upMatrix)
        {
            ch = hDecoder->internal_channel[0];
            for(i = 0; i < frame_len; i++)
            {
                real_t inp0 = input[ch][i];
                (*sample_buffer)[(i*2)+0] = inp0*FLOAT_SCALE;
                (*sample_buffer)[(i*2)+1] = inp0*FLOAT_SCALE;
            }
        } else {
            ch  = hDecoder->internal_channel[0];
            ch1 = hDecoder->internal_channel[1];
            for(i = 0; i < frame_len; i++)
            {
                real_t inp0 = input[ch ][i];
                real_t inp1 = input[ch1][i];
                (*sample_buffer)[(i*2)+0] = inp0*FLOAT_SCALE;
                (*sample_buffer)[(i*2)+1] = inp1*FLOAT_SCALE;
            }
        }
        break;
    default:
        for (ch = 0; ch < channels; ch++)
        {
            for(i = 0; i < frame_len; i++)
            {
                real_t inp = get_sample(input, ch, i, hDecoder->downMatrix, hDecoder->internal_channel);
                (*sample_buffer)[(i*channels)+ch] = inp*FLOAT_SCALE;
            }
        }
        break;
    }
}

static void to_PCM_double(NeAACDecStruct *hDecoder, real_t **input,
                          uint8_t channels, uint16_t frame_len,
                          double **sample_buffer)
{
    uint8_t ch, ch1;
    uint16_t i;

    switch (CONV(channels,hDecoder->downMatrix))
    {
    case CONV(1,0):
    case CONV(1,1):
        for(i = 0; i < frame_len; i++)
        {
            real_t inp = input[hDecoder->internal_channel[0]][i];
            (*sample_buffer)[i] = (double)inp*FLOAT_SCALE;
        }
        break;
    case CONV(2,0):
        if (hDecoder->upMatrix)
        {
            ch = hDecoder->internal_channel[0];
            for(i = 0; i < frame_len; i++)
            {
                real_t inp0 = input[ch][i];
                (*sample_buffer)[(i*2)+0] = (double)inp0*FLOAT_SCALE;
                (*sample_buffer)[(i*2)+1] = (double)inp0*FLOAT_SCALE;
            }
        } else {
            ch  = hDecoder->internal_channel[0];
            ch1 = hDecoder->internal_channel[1];
            for(i = 0; i < frame_len; i++)
            {
                real_t inp0 = input[ch ][i];
                real_t inp1 = input[ch1][i];
                (*sample_buffer)[(i*2)+0] = (double)inp0*FLOAT_SCALE;
                (*sample_buffer)[(i*2)+1] = (double)inp1*FLOAT_SCALE;
            }
        }
        break;
    default:
        for (ch = 0; ch < channels; ch++)
        {
            for(i = 0; i < frame_len; i++)
            {
                real_t inp = get_sample(input, ch, i, hDecoder->downMatrix, hDecoder->internal_channel);
                (*sample_buffer)[(i*channels)+ch] = (double)inp*FLOAT_SCALE;
            }
        }
        break;
    }
}

void *output_to_PCM(NeAACDecStruct *hDecoder,
                    real_t **input, void *sample_buffer, uint8_t channels,
                    uint16_t frame_len, uint8_t format)
{
    int16_t   *short_sample_buffer = (int16_t*)sample_buffer;
    int32_t   *int_sample_buffer = (int32_t*)sample_buffer;
    float32_t *float_sample_buffer = (float32_t*)sample_buffer;
    double    *double_sample_buffer = (double*)sample_buffer;


    /* Copy output to a standard PCM buffer */
    switch (format)
    {
    case FAAD_FMT_16BIT:
        to_PCM_16bit(hDecoder, input, channels, frame_len, &short_sample_buffer);
        break;
    case FAAD_FMT_24BIT:
        to_PCM_24bit(hDecoder, input, channels, frame_len, &int_sample_buffer);
        break;
    case FAAD_FMT_32BIT:
        to_PCM_32bit(hDecoder, input, channels, frame_len, &int_sample_buffer);
        break;
    case FAAD_FMT_FLOAT:
        to_PCM_float(hDecoder, input, channels, frame_len, &float_sample_buffer);
        break;
    case FAAD_FMT_DOUBLE:
        to_PCM_double(hDecoder, input, channels, frame_len, &double_sample_buffer);
        break;
    }


    return sample_buffer;
}

} // namespace faad
} // namespace codec
