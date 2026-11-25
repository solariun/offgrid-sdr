// ============================================================================
//  offgrid-codecs - mid/side stereo (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacMs.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  Mid/side stereo: rebuilds left/right from mid/side for the bands that use
//  it.
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
** $Id: ms.c,v 1.21 2007/11/01 12:33:32 menno Exp $
**/

#include "AacCommon.h"
#include "AacStructs.h"
#include "AacSyntax.h"
#include "AacMs.h"
#include "AacIs.h"
#include "AacPns.h"

namespace codec {
namespace faad {

void ms_decode(ic_stream *ics, ic_stream *icsr, real_t *l_spec, real_t *r_spec,
               uint16_t frame_len)
{
    uint8_t g, b, sfb;
    uint8_t group = 0;
    uint16_t nshort = frame_len/8;

    uint16_t i, k;
    real_t tmp;

    if (ics->ms_mask_present >= 1)
    {
        for (g = 0; g < ics->num_window_groups; g++)
        {
            for (b = 0; b < ics->window_group_length[g]; b++)
            {
                for (sfb = 0; sfb < ics->max_sfb; sfb++)
                {
                    /* If intensity stereo coding or noise substitution is on
                       for a particular scalefactor band, no M/S stereo decoding
                       is carried out.
                     */
                    if ((ics->ms_used[g][sfb] || ics->ms_mask_present == 2) &&
                        !is_intensity(icsr, g, sfb) && !is_noise(ics, g, sfb))
                    {
                        for (i = ics->swb_offset[sfb]; i < FAAD_MIN(ics->swb_offset[sfb+1], ics->swb_offset_max); i++)
                        {
                            k = (group*nshort) + i;
                            tmp = l_spec[k] - r_spec[k];
                            l_spec[k] = l_spec[k] + r_spec[k];
                            r_spec[k] = tmp;
                        }
                    }
                }
                group++;
            }
        }
    }
}

} // namespace faad
} // namespace codec
