// ============================================================================
//  offgrid-codecs - dynamic range control (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacDrc.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  Applies the DRC gains a stream sends in its fill elements to the spectral
//  data (as faad2 does with APPLY_DRC).
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
** $Id: drc.c,v 1.29 2015/02/22 10:09:29 knik Exp $
**/

#include <cstdlib>
#include "AacCommon.h"
#include "AacStructs.h"
#include "AacSyntax.h"
#include "AacDrc.h"

namespace codec {
namespace faad {

drc_info *drc_init(real_t cut, real_t boost)
{
    drc_info *drc = (drc_info*)faad_malloc(sizeof(drc_info));
    memset(drc, 0, sizeof(drc_info));

    drc->ctrl1 = cut;
    drc->ctrl2 = boost;

    drc->num_bands = 1;
    drc->band_top[0] = 1024/4 - 1;
    drc->dyn_rng_sgn[0] = 1;
    drc->dyn_rng_ctl[0] = 0;

    return drc;
}

void drc_end(drc_info *drc)
{
    if (drc) faad_free(drc);
}


void drc_decode(drc_info *drc, real_t *spec)
{
    uint16_t i, bd, top;
    real_t factor, exp;
    uint16_t bottom = 0;

    if (drc->num_bands == 1)
        drc->band_top[0] = 1024/4 - 1;

    for (bd = 0; bd < drc->num_bands; bd++)
    {
        top = 4 * (drc->band_top[bd] + 1);

        /* Decode DRC gain factor */
        if (drc->dyn_rng_sgn[bd])  /* compress */
            exp = ((-drc->ctrl1 * drc->dyn_rng_ctl[bd]) - (DRC_REF_LEVEL - drc->prog_ref_level))/REAL_CONST(24.0);
        else /* boost */
            exp = ((drc->ctrl2 * drc->dyn_rng_ctl[bd]) - (DRC_REF_LEVEL - drc->prog_ref_level))/REAL_CONST(24.0);
        factor = (real_t)faad_pow(2.0, exp);

        /* Apply gain factor */
        for (i = bottom; i < top; i++)
            spec[i] *= factor;

        bottom = top;
    }
}

} // namespace faad
} // namespace codec
