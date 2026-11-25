// ============================================================================
//  offgrid-codecs - AAC bit reader (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacBits.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  The bit reader every AAC/SBR/PS parser uses: big-endian words,
//  show/flush/get bits, byte alignment, and the reversed reader HCR and RVLC
//  need.
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
** $Id: bits.c,v 1.44 2007/11/01 12:33:29 menno Exp $
**/

#include <cstdlib>
#include "AacCommon.h"
#include "AacStructs.h"
#include "AacBits.h"

namespace codec {
namespace faad {

/* reads only n bytes from the stream instead of the standard 4 */
static /*inline*/ uint32_t getdword_n(void *mem, int n)
{
    uint8_t* m8 = (uint8_t*)mem;
    switch (n)
    {
    case 3:
        return ((uint32_t)m8[2] << 8) | ((uint32_t)m8[1] << 16) | ((uint32_t)m8[0] << 24);
    case 2:
        return ((uint32_t)m8[1] << 16) | ((uint32_t)m8[0] << 24);
    case 1:
        return (uint32_t)m8[0] << 24;
    default:
        return 0;
    }
}

/* initialize buffer, call once before first getbits or showbits */
void faad_initbits(bitfile *ld, const void *_buffer, const uint32_t buffer_size)
{
    uint32_t tmp;

    if (ld == NULL)
        return;

    if (buffer_size == 0 || _buffer == NULL)
    {
        ld->error = 1;
        return;
    }

    ld->buffer = _buffer;

    ld->buffer_size = buffer_size;
    ld->bytes_left  = buffer_size;

    if (ld->bytes_left >= 4)
    {
        tmp = getdword((uint32_t*)ld->buffer);
        ld->bytes_left -= 4;
    } else {
        tmp = getdword_n((uint32_t*)ld->buffer, ld->bytes_left);
        ld->bytes_left = 0;
    }
    ld->bufa = tmp;

    if (ld->bytes_left >= 4)
    {
        tmp = getdword((uint32_t*)ld->buffer + 1);
        ld->bytes_left -= 4;
    } else {
        tmp = getdword_n((uint32_t*)ld->buffer + 1, ld->bytes_left);
        ld->bytes_left = 0;
    }
    ld->bufb = tmp;

    ld->start = (uint32_t*)ld->buffer;
    ld->tail = ((uint32_t*)ld->buffer + 2);

    ld->bits_left = 32;

    ld->error = 0;
}

void faad_endbits(bitfile *ld)
{
    (void)ld;
}

uint32_t faad_get_processed_bits(bitfile *ld)
{
    return (uint32_t)(8 * (4*(ld->tail - ld->start) - 4) - (ld->bits_left));
}

uint8_t faad_byte_align(bitfile *ld)
{
    int remainder = (32 - ld->bits_left) & 0x7;

    if (remainder)
    {
        faad_flushbits(ld, 8 - remainder);
        return (uint8_t)(8 - remainder);
    }
    return 0;
}

void faad_flushbits_ex(bitfile *ld, uint32_t bits)
{
    uint32_t tmp;

    ld->bufa = ld->bufb;
    if (ld->bytes_left >= 4)
    {
        tmp = getdword(ld->tail);
        ld->bytes_left -= 4;
    } else {
        tmp = getdword_n(ld->tail, ld->bytes_left);
        ld->bytes_left = 0;
    }
    ld->bufb = tmp;
    ld->tail++;
    ld->bits_left += (32 - bits);
    //ld->bytes_left -= 4;
//    if (ld->bytes_left == 0)
//        ld->no_more_reading = 1;
//    if (ld->bytes_left < 0)
//        ld->error = 1;
}

/* rewind to beginning */
void faad_rewindbits(bitfile *ld)
{
    uint32_t tmp;

    ld->bytes_left = ld->buffer_size;

    if (ld->bytes_left >= 4)
    {
        tmp = getdword((uint32_t*)&ld->start[0]);
        ld->bytes_left -= 4;
    } else {
        tmp = getdword_n((uint32_t*)&ld->start[0], ld->bytes_left);
        ld->bytes_left = 0;
    }
    ld->bufa = tmp;

    if (ld->bytes_left >= 4)
    {
        tmp = getdword((uint32_t*)&ld->start[1]);
        ld->bytes_left -= 4;
    } else {
        tmp = getdword_n((uint32_t*)&ld->start[1], ld->bytes_left);
        ld->bytes_left = 0;
    }
    ld->bufb = tmp;

    ld->bits_left = 32;
    ld->tail = &ld->start[2];
}

/* reset to a certain point */
void faad_resetbits(bitfile *ld, uint32_t bits)
{
    uint32_t tmp;
    uint32_t words = bits >> 5;
    uint32_t remainder = bits & 0x1F;

    if (ld->buffer_size < words * 4)
        ld->bytes_left = 0;
    else
        ld->bytes_left = ld->buffer_size - words*4;

    if (ld->bytes_left >= 4)
    {
        tmp = getdword(&ld->start[words]);
        ld->bytes_left -= 4;
    } else {
        tmp = getdword_n(&ld->start[words], ld->bytes_left);
        ld->bytes_left = 0;
    }
    ld->bufa = tmp;

    if (ld->bytes_left >= 4)
    {
        tmp = getdword(&ld->start[words+1]);
        ld->bytes_left -= 4;
    } else {
        tmp = getdword_n(&ld->start[words+1], ld->bytes_left);
        ld->bytes_left = 0;
    }
    ld->bufb = tmp;

    ld->bits_left = 32 - remainder;
    ld->tail = &ld->start[words+2];

    /* recheck for reading too many bytes */
    ld->error = 0;
//    if (ld->bytes_left == 0)
//        ld->no_more_reading = 1;
//    if (ld->bytes_left < 0)
//        ld->error = 1;
}

uint8_t *faad_getbitbuffer(bitfile *ld, uint32_t bits)
{
    int i;
    unsigned int temp;
    int bytes = bits >> 3;
    int remainder = bits & 0x7;

    uint8_t *buffer = (uint8_t*)faad_malloc((bytes+1)*sizeof(uint8_t));

    for (i = 0; i < bytes; i++)
    {
        buffer[i] = (uint8_t)faad_getbits(ld, 8);
    }

    if (remainder)
    {
        temp = faad_getbits(ld, remainder) << (8-remainder);

        buffer[bytes] = (uint8_t)temp;
    }

    return buffer;
}

/* return the original data buffer */
void *faad_origbitbuffer(bitfile *ld)
{
    return (void*)ld->start;
}

/* return the original data buffer size */
uint32_t faad_origbitbuffer_size(bitfile *ld)
{
    return ld->buffer_size;
}


/* EOF */

} // namespace faad
} // namespace codec
