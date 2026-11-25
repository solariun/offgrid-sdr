// ============================================================================
//  offgrid-codecs - intensity stereo (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacIs.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  Intensity stereo: rebuilds the right channel of a pair from the left for
//  bands coded as intensity.
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
** $Id: is.c,v 1.28 2007/11/01 12:33:31 menno Exp $
**/

#include "AacCommon.h"
#include "AacStructs.h"
#include "AacSyntax.h"
#include "AacIs.h"

namespace codec {
namespace faad {

void is_decode(ic_stream *ics, ic_stream *icsr, real_t *l_spec, real_t *r_spec,
               uint16_t frame_len)
{
    uint8_t g, sfb, b;
    uint16_t i;
    real_t scale;

    uint16_t nshort = frame_len/8;
    uint8_t group = 0;

    for (g = 0; g < icsr->num_window_groups; g++)
    {
        /* Do intensity stereo decoding */
        for (b = 0; b < icsr->window_group_length[g]; b++)
        {
            for (sfb = 0; sfb < icsr->max_sfb; sfb++)
            {
                if (is_intensity(icsr, g, sfb))
                {
                    int16_t scale_factor = icsr->scale_factors[g][sfb];

                    scale_factor = FAAD_MIN(FAAD_MAX(scale_factor, -120), 120);
                    scale = (real_t)faad_pow(0.5, (0.25*scale_factor));

                    /* Scale from left to right channel,
                       do not touch left channel */
                    for (i = icsr->swb_offset[sfb]; i < FAAD_MIN(icsr->swb_offset[sfb+1], ics->swb_offset_max); i++)
                    {
                        r_spec[(group*nshort)+i] = MUL_R(l_spec[(group*nshort)+i], scale);
                        if (is_intensity(icsr, g, sfb) != invert_intensity(ics, g, sfb))
                            r_spec[(group*nshort)+i] = -r_spec[(group*nshort)+i];
                    }
                }
            }
            group++;
        }
    }
}

} // namespace faad
} // namespace codec
