// ============================================================================
//  offgrid-codecs - AAC bitstream syntax (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacSyntax.h
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  Raw data block parsing: SCE/CPE/LFE/fill (incl. OFFgrid-SDR's DAB+
//  fill-element change)/PCE/DSE, ICS info, section, scalefactor and spectral
//  data, TNS, pulse, the DRM error-resilient layout and ADTS headers.
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
** $Id: syntax.h,v 1.60 2009/01/26 23:51:17 menno Exp $
**/

#ifndef OFFGRID_CODECS_AACSYNTAX_H
#define OFFGRID_CODECS_AACSYNTAX_H

#include "AacCommon.h"
#include "AacBits.h"

namespace codec {
namespace faad {

#define MAIN       1
#define LC         2
#define SSR        3
#define LTP        4
#define HE_AAC     5
#define LD        23
#define ER_LC     17
#define ER_LTP    19
#define DRM_ER_LC 27 /* special object type for DRM */

/* header types */
#define RAW        0
#define ADIF       1
#define ADTS       2
#define LATM       3

/* SBR signalling */
#define NO_SBR           0
#define SBR_UPSAMPLED    1
#define SBR_DOWNSAMPLED  2
#define NO_SBR_UPSAMPLED 3

/* DRM channel definitions */
#define DRMCH_MONO          1
#define DRMCH_STEREO        2
#define DRMCH_SBR_MONO      3
#define DRMCH_SBR_STEREO    4
#define DRMCH_SBR_PS_STEREO 5


/* First object type that has ER */
#define ER_OBJECT_START 17


/* Bitstream */
#define LEN_SE_ID 3
#define LEN_TAG   4
#define LEN_BYTE  8

#define EXT_FIL            0
#define EXT_FILL_DATA      1
#define EXT_DATA_ELEMENT   2
#define EXT_DYNAMIC_RANGE 11
#define ANC_DATA           0

/* Syntax elements */
#define ID_SCE 0x0
#define ID_CPE 0x1
#define ID_CCE 0x2
#define ID_LFE 0x3
#define ID_DSE 0x4
#define ID_PCE 0x5
#define ID_FIL 0x6
#define ID_END 0x7
#define INVALID_ELEMENT_ID 255

#define ONLY_LONG_SEQUENCE   0x0
#define LONG_START_SEQUENCE  0x1
#define EIGHT_SHORT_SEQUENCE 0x2
#define LONG_STOP_SEQUENCE   0x3

#define ZERO_HCB       0
#define FIRST_PAIR_HCB 5
#define ESC_HCB        11
#define QUAD_LEN       4
#define PAIR_LEN       2
#define NOISE_HCB      13
#define INTENSITY_HCB2 14
#define INTENSITY_HCB  15

#define INVALID_SBR_ELEMENT 255

int8_t GASpecificConfig(bitfile *ld, mp4AudioSpecificConfig *mp4ASC,
                        program_config *pce);

uint8_t adts_frame(adts_header *adts, bitfile *ld);
void get_adif_header(adif_header *adif, bitfile *ld);
void raw_data_block(NeAACDecStruct *hDecoder, NeAACDecFrameInfo *hInfo,
                    bitfile *ld, program_config *pce, drc_info *drc);
uint8_t reordered_spectral_data(NeAACDecStruct *hDecoder, ic_stream *ics, bitfile *ld,
                                int16_t *spectral_data);
void DRM_aac_scalable_main_element(NeAACDecStruct *hDecoder, NeAACDecFrameInfo *hInfo,
                                   bitfile *ld, program_config *pce, drc_info *drc);

} // namespace faad
} // namespace codec

#endif // OFFGRID_CODECS_AACSYNTAX_H
