// ============================================================================
//  offgrid-codecs - pulse data (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacPulse.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  Adds the pulse data (a few large spectral values) back into the quantised
//  spectrum.
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
** $Id: pulse.c,v 1.21 2007/11/01 12:33:34 menno Exp $
**/

#include "AacCommon.h"
#include "AacStructs.h"
#include "AacSyntax.h"
#include "AacPulse.h"

namespace codec {
namespace faad {

uint8_t pulse_decode(ic_stream *ics, int16_t *spec_data, uint16_t framelen)
{
    uint8_t i;
    uint16_t k;
    pulse_info *pul = &(ics->pul);

    k = FAAD_MIN(ics->swb_offset[pul->pulse_start_sfb], ics->swb_offset_max);

    for (i = 0; i <= pul->number_pulse; i++)
    {
        k += pul->pulse_offset[i];

        if (k >= framelen)
            return 15; /* should not be possible */

        if (spec_data[k] > 0)
            spec_data[k] += pul->pulse_amp[i];
        else
            spec_data[k] -= pul->pulse_amp[i];
    }

    return 0;
}

} // namespace faad
} // namespace codec
