// ============================================================================
//  offgrid-codecs - error messages (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacError.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  The text for each numeric decoder error code.
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
** $Id: error.c,v 1.33 2008/09/19 23:31:39 menno Exp $
**/

#include "AacCommon.h"
#include "AacError.h"

namespace codec {
namespace faad {

const char *err_msg[] = {
    "No error",
    "Gain control not yet implemented",
    "Pulse coding not allowed in short blocks",
    "Invalid huffman codebook",
    "Scalefactor out of range",
    "Unable to find ADTS syncword",
    "Channel coupling not yet implemented",
    "Channel configuration not allowed in error resilient frame",
    "Bit error in error resilient scalefactor decoding",
    "Error decoding huffman scalefactor (bitstream error)",
    "Error decoding huffman codeword (bitstream error)",
    "Non existent huffman codebook number found",
    "Invalid number of channels",
    "Maximum number of bitstream elements exceeded",
    "Input data buffer too small",
    "Array index out of range",
    "Maximum number of scalefactor bands exceeded",
    "Quantised value out of range",
    "LTP lag out of range",
    "Invalid SBR parameter decoded",
    "SBR called without being initialised",
    "Unexpected channel configuration change",
    "Error in program_config_element",
    "First SBR frame is not the same as first AAC frame",
    "Unexpected fill element with SBR data",
    "Not all elements were provided with SBR data",
    "LTP decoding not available",
    "Output data buffer too small",
    "CRC error in DRM data",
    "PNS not allowed in DRM data stream",
    "No standard extension payload allowed in DRM",
    "PCE shall be the first element in a frame",
    "Bitstream value not allowed by specification",
    "MAIN prediction not initialised",
    "Long term prediction not initialised"
};

} // namespace faad
} // namespace codec
