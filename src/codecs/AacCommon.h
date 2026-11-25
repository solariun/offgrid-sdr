// ============================================================================
//  offgrid-codecs - shared definitions (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacCommon.h
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  What FAAD2's common.h gave every file: the number types (real_t, complex_t),
//  the arithmetic macros of its floating-point build, the feature choices
//  (fixed here once: see below) and the shared helpers of AacCommon.cpp.
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
** $Id: common.h,v 1.79 2015/01/26 17:48:53 knik Exp $
**/

#ifndef OFFGRID_CODECS_AACCOMMON_H
#define OFFGRID_CODECS_AACCOMMON_H

#include <cstdio>
#include <cstdlib>
#include <cstddef>
#include <cstring>
#include <cmath>
#include <stdint.h>

#include "AacFaadApi.h"

/* ---- OFFgrid-SDR: the compile-time choices of FAAD2's common.h, resolved once ----
   FAAD2 is configured by preprocessor symbols. OFFgrid-SDR builds it with DRM_SUPPORT and APPLY_DRC, in floating
   point, so these were active and are what this port keeps (the code for the others was removed with unifdef):
     ERROR_RESILIENCE, DRM, DRM_PS, ALLOW_SMALL_FRAMELENGTH (960-sample frames), SBR_DEC, PS_DEC, APPLY_DRC
   and these were not (Main, LTP, LD, SSR profiles; fixed point; low-power SBR). */

/* FAAD2's min/max macros, renamed so they cannot clash with std::min/std::max */
#define FAAD_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define FAAD_MIN(a, b) (((a) < (b)) ? (a) : (b))

#define DIV_R(A, B) ((A)/(B))
#define DIV_C(A, B) ((A)/(B))
#define DIV_F(A, B) ((A)/(B))

#define qmf_t complex_t
#define QMF_RE(A) RE(A)
#define QMF_IM(A) IM(A)

#define MUL_R(A,B) ((A)*(B))
#define MUL_C(A,B) ((A)*(B))
#define MUL_F(A,B) ((A)*(B))

#define REAL_CONST(A) ((real_t)(A))
#define COEF_CONST(A) ((real_t)(A))
#define Q2_CONST(A) ((real_t)(A))
#define FRAC_CONST(A) ((real_t)(A)) /* pure fractional part */

#define RE(A) (A)[0]
#define IM(A) (A)[1]

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2 /* PI/2 */
#define M_PI_2 1.57079632679489661923
#endif

namespace codec {
namespace faad {

typedef float float32_t;
typedef float real_t;
typedef real_t complex_t[2];

/* Complex multiplication */
static inline void ComplexMult(real_t *y1, real_t *y2,
    real_t x1, real_t x2, real_t c1, real_t c2)
{
    *y1 = MUL_F(x1, c1) + MUL_F(x2, c2);
    *y2 = MUL_F(x2, c1) - MUL_F(x1, c2);
}

/* OFFgrid-SDR: C's <math.h> functions take and return double. In C++, <cmath> also has float overloads, which a
   float argument would pick, rounding differently from FAAD2 built as C. The port calls these instead, so every
   such call is computed in double exactly as FAAD2's C does (and the output stays sample-exact with it). */
static inline double faad_sqrt(double x) { return std::sqrt(x); }
static inline double faad_pow(double x, double y) { return std::pow(x, y); }
static inline double faad_exp(double x) { return std::exp(x); }
static inline double faad_log(double x) { return std::log(x); }
static inline double faad_cos(double x) { return std::cos(x); }
static inline double faad_sin(double x) { return std::sin(x); }
static inline double faad_atan(double x) { return std::atan(x); }
static inline double faad_atan2(double y, double x) { return std::atan2(y, x); }

/* common functions */
uint32_t ne_rng(uint32_t *rng1, uint32_t *rng2);
uint8_t get_sr_index(const uint32_t samplerate);
uint8_t max_pred_sfb(const uint8_t sr_index);
uint8_t max_tns_sfb(const uint8_t sr_index, const uint8_t object_type,
                    const uint8_t is_short);
uint32_t get_sample_rate(const uint8_t sr_index);
int8_t can_decode_ot(const uint8_t object_type);

void *faad_malloc(size_t size);
void faad_free(void *b);

} // namespace faad
} // namespace codec

#endif // OFFGRID_CODECS_AACCOMMON_H
