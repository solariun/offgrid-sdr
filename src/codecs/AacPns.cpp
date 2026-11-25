// ============================================================================
//  offgrid-codecs - perceptual noise substitution (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacPns.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  PNS: fills bands coded as noise with scaled random values (and correlated
//  noise for pairs).
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
** $Id: pns.c,v 1.39 2010/06/04 20:47:56 menno Exp $
**/

#include "AacCommon.h"
#include "AacStructs.h"
#include "AacPns.h"

namespace codec {
namespace faad {

/* static function declarations */
static void gen_rand_vector(real_t *spec, int16_t scale_factor, uint16_t size,
                            uint8_t sub,
                            /* RNG states */ uint32_t *rng1, uint32_t *rng2);


/* The function gen_rand_vector(addr, size) generates a vector of length
   <size> with signed random values of average energy MEAN_NRG per random
   value. A suitable random number generator can be realized using one
   multiplication/accumulation per random value.
*/
static inline void gen_rand_vector(real_t *spec, int16_t scale_factor, uint16_t size,
                                   uint8_t sub,
                                   /* RNG states */ uint32_t *rng1, uint32_t *rng2)
{
    uint16_t i;
    real_t energy = 0.0;
    (void)sub;

    scale_factor = FAAD_MIN(FAAD_MAX(scale_factor, -120), 120);

    for (i = 0; i < size; i++)
    {
        real_t tmp = (real_t)(int32_t)ne_rng(rng1, rng2);
        spec[i] = tmp;
        energy += tmp*tmp;
    }

    if (energy > 0)
    {
        real_t scale = (real_t)1.0/(real_t)faad_sqrt(energy);
        scale *= (real_t)faad_pow(2.0, 0.25 * scale_factor);
        for (i = 0; i < size; i++)
        {
            spec[i] *= scale;
        }
    }
}

void pns_decode(ic_stream *ics_left, ic_stream *ics_right,
                real_t *spec_left, real_t *spec_right, uint16_t frame_len,
                uint8_t channel_pair, uint8_t object_type,
                /* RNG states */ uint32_t *rng1, uint32_t *rng2)
{
    uint8_t g, sfb, b;
    uint16_t begin, end;

    uint8_t group = 0;
    uint16_t nshort = frame_len >> 3;

    uint8_t sub = 0;

    (void)object_type;

    for (g = 0; g < ics_left->num_window_groups; g++)
    {
        /* Do perceptual noise substitution decoding */
        for (b = 0; b < ics_left->window_group_length[g]; b++)
        {
            uint16_t base = group * nshort;
            for (sfb = 0; sfb < ics_left->max_sfb; sfb++)
            {
                uint32_t r1_dep = 0, r2_dep = 0;

                if (is_noise(ics_left, g, sfb))
                {

                    /* swb_offset_max is per-window, not per-frame; clamping
                       group>0 offsets against it zeroed out begin/end. */
                    begin = FAAD_MIN(base + ics_left->swb_offset[sfb], frame_len);
                    end = FAAD_MIN(base + ics_left->swb_offset[sfb+1], frame_len);

                    r1_dep = *rng1;
                    r2_dep = *rng2;

                    /* Generate random vector */
                    gen_rand_vector(&spec_left[begin],
                        ics_left->scale_factors[g][sfb], end - begin, sub, rng1, rng2);
                }

/* From the spec:
   If the same scalefactor band and group is coded by perceptual noise
   substitution in both channels of a channel pair, the correlation of
   the noise signal can be controlled by means of the ms_used field: While
   the default noise generation process works independently for each channel
   (separate generation of random vectors), the same random vector is used
   for both channels if ms_used[] is set for a particular scalefactor band
   and group. In this case, no M/S stereo coding is carried out (because M/S
   stereo coding and noise substitution coding are mutually exclusive).
   If the same scalefactor band and group is coded by perceptual noise
   substitution in only one channel of a channel pair the setting of ms_used[]
   is not evaluated.
*/
                if ((ics_right != NULL)
                    && is_noise(ics_right, g, sfb))
                {

                    if (channel_pair && is_noise(ics_left, g, sfb) &&
                        (((ics_left->ms_mask_present == 1) &&
                        (ics_left->ms_used[g][sfb])) ||
                        (ics_left->ms_mask_present == 2)))
                    {
                        /*uint16_t c;*/

                        begin = FAAD_MIN(base + ics_right->swb_offset[sfb], frame_len);
                        end = FAAD_MIN(base + ics_right->swb_offset[sfb+1], frame_len);

                        /* Generate random vector dependent on left channel*/
                        gen_rand_vector(&spec_right[begin],
                            ics_right->scale_factors[g][sfb], end - begin, sub, &r1_dep, &r2_dep);

                    } else /*if (ics_left->ms_mask_present == 0)*/ {
                        begin = FAAD_MIN(base + ics_right->swb_offset[sfb], frame_len);
                        end = FAAD_MIN(base + ics_right->swb_offset[sfb+1], frame_len);

                        /* Generate random vector */
                        gen_rand_vector(&spec_right[begin],
                            ics_right->scale_factors[g][sfb], end - begin, sub, rng1, rng2);
                    }
                }
            } /* sfb */
            group++;
        } /* b */
    } /* g */
}

} // namespace faad
} // namespace codec
