// ============================================================================
//  offgrid-codecs - SBR syntax (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacSbrSyntax.h
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  Reads SBR extension data: header, single/channel-pair element data,
//  extended data (PS, DRM PS).
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
** $Id: sbr_syntax.h,v 1.23 2007/11/01 12:33:36 menno Exp $
**/

#ifndef OFFGRID_CODECS_AACSBRSYNTAX_H
#define OFFGRID_CODECS_AACSBRSYNTAX_H

#include "AacCommon.h"
#include "AacBits.h"

namespace codec {
namespace faad {

#define T_HFGEN 8
#define T_HFADJ 2

#define EXT_SBR_DATA     13
#define EXT_SBR_DATA_CRC 14

#define FIXFIX 0
#define FIXVAR 1
#define VARFIX 2
#define VARVAR 3

#define LO_RES 0
#define HI_RES 1

#define NO_TIME_SLOTS_960 15
#define NO_TIME_SLOTS     16
#define RATE              2

#define NOISE_FLOOR_OFFSET 6


uint8_t sbr_extension_data(bitfile *ld, sbr_info *sbr, uint16_t cnt,
                           uint8_t resetFlag);

} // namespace faad
} // namespace codec

#endif // OFFGRID_CODECS_AACSBRSYNTAX_H
