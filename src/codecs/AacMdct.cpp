// ============================================================================
//  offgrid-codecs - IMDCT (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacMdct.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  The inverse MDCT (via the complex FFT) for block sizes 256/2048 and
//  240/1920.
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
** $Id: mdct.c,v 1.47 2007/11/01 12:33:31 menno Exp $
**/

#include <cstdlib>
#include <cassert>
#include "AacCommon.h"
#include "AacStructs.h"
#include "AacCfft.h"
#include "AacMdct.h"
#include "AacMdctTab.h"

namespace codec {
namespace faad {

/*
 * Fast (I)MDCT Implementation using (I)FFT ((Inverse) Fast Fourier Transform)
 * and consists of three steps: pre-(I)FFT complex multiplication, complex
 * (I)FFT, post-(I)FFT complex multiplication,
 *
 * As described in:
 *  P. Duhamel, Y. Mahieux, and J.P. Petit, "A Fast Algorithm for the
 *  Implementation of Filter Banks Based on 'Time Domain Aliasing
 *  Cancellation'," IEEE Proc. on ICASSP'91, 1991, pp. 2209-2212.
 *
 *
 * As of April 6th 2002 completely rewritten.
 * This (I)MDCT can now be used for any data size n, where n is divisible by 8.
 *
 */


mdct_info *faad_mdct_init(uint16_t N)
{
    mdct_info *mdct = (mdct_info*)faad_malloc(sizeof(mdct_info));

    assert(N % 8 == 0);

    mdct->N = N;

    /* NOTE: For "small framelengths" in FIXED_POINT the coefficients need to be
     * scaled by sqrt("(nearest power of 2) > N" / N) */

    /* RE(mdct->sincos[k]) = scale*(real_t)(cos(2.0*M_PI*(k+1./8.) / (real_t)N));
     * IM(mdct->sincos[k]) = scale*(real_t)(sin(2.0*M_PI*(k+1./8.) / (real_t)N)); */
    /* scale is 1 for fixed point, sqrt(N) for floating point */
    switch (N)
    {
    case 2048: mdct->sincos = (complex_t*)mdct_tab_2048; break;
    case 256:  mdct->sincos = (complex_t*)mdct_tab_256;  break;
    case 1920: mdct->sincos = (complex_t*)mdct_tab_1920; break;
    case 240:  mdct->sincos = (complex_t*)mdct_tab_240;  break;
    }

    /* initialise fft */
    mdct->cfft = cffti(N/4);


    return mdct;
}

void faad_mdct_end(mdct_info *mdct)
{
    if (mdct != NULL)
    {

        cfftu(mdct->cfft);

        faad_free(mdct);
    }
}

void faad_imdct(mdct_info *mdct, real_t *X_in, real_t *X_out)
{
    uint16_t k;

    complex_t x;
    complex_t Z1[512];
    complex_t *sincos = mdct->sincos;

    uint16_t N  = mdct->N;
    uint16_t N2 = N >> 1;
    uint16_t N4 = N >> 2;
    uint16_t N8 = N >> 3;


    /* pre-IFFT complex multiplication */
    for (k = 0; k < N4; k++)
    {
        ComplexMult(&IM(Z1[k]), &RE(Z1[k]),
            X_in[2*k], X_in[N2 - 1 - 2*k], RE(sincos[k]), IM(sincos[k]));
    }


    /* complex IFFT, any non-scaling FFT can be used here */
    cfftb(mdct->cfft, Z1);


    /* post-IFFT complex multiplication */
    for (k = 0; k < N4; k++)
    {
        RE(x) = RE(Z1[k]);
        IM(x) = IM(Z1[k]);
        ComplexMult(&IM(Z1[k]), &RE(Z1[k]),
            IM(x), RE(x), RE(sincos[k]), IM(sincos[k]));

    }

    /* reordering */
    for (k = 0; k < N8; k+=2)
    {
        X_out[              2*k] =  IM(Z1[N8 +     k]);
        X_out[          2 + 2*k] =  IM(Z1[N8 + 1 + k]);

        X_out[          1 + 2*k] = -RE(Z1[N8 - 1 - k]);
        X_out[          3 + 2*k] = -RE(Z1[N8 - 2 - k]);

        X_out[N4 +          2*k] =  RE(Z1[         k]);
        X_out[N4 +    + 2 + 2*k] =  RE(Z1[     1 + k]);

        X_out[N4 +      1 + 2*k] = -IM(Z1[N4 - 1 - k]);
        X_out[N4 +      3 + 2*k] = -IM(Z1[N4 - 2 - k]);

        X_out[N2 +          2*k] =  RE(Z1[N8 +     k]);
        X_out[N2 +    + 2 + 2*k] =  RE(Z1[N8 + 1 + k]);

        X_out[N2 +      1 + 2*k] = -IM(Z1[N8 - 1 - k]);
        X_out[N2 +      3 + 2*k] = -IM(Z1[N8 - 2 - k]);

        X_out[N2 + N4 +     2*k] = -IM(Z1[         k]);
        X_out[N2 + N4 + 2 + 2*k] = -IM(Z1[     1 + k]);

        X_out[N2 + N4 + 1 + 2*k] =  RE(Z1[N4 - 1 - k]);
        X_out[N2 + N4 + 3 + 2*k] =  RE(Z1[N4 - 2 - k]);
    }

}

} // namespace faad
} // namespace codec
