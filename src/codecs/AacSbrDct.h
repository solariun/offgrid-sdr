// ============================================================================
//  offgrid-codecs - SBR DCT (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacSbrDct.h
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  The DCT-IV/DCT-II kernels the SBR QMF banks use.
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
** $Id: sbr_dct.h,v 1.19 2007/11/01 12:33:34 menno Exp $
**/

#ifndef OFFGRID_CODECS_AACSBRDCT_H
#define OFFGRID_CODECS_AACSBRDCT_H

#include "AacCommon.h"

namespace codec {
namespace faad {

void dct4_kernel(real_t * in_real, real_t * in_imag, real_t * out_real, real_t * out_imag);

void DCT3_32_unscaled(real_t *y, real_t *x);
void DCT4_32(real_t *y, real_t *x);
void DST4_32(real_t *y, real_t *x);
void DCT2_32_unscaled(real_t *y, real_t *x);
void DCT4_16(real_t *y, real_t *x);
void DCT2_16_unscaled(real_t *y, real_t *x);

} // namespace faad
} // namespace codec

#endif // OFFGRID_CODECS_AACSBRDCT_H
