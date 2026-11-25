// ============================================================================
//  offgrid-codecs - filter bank (IMDCT + windowing) (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacFiltbank.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  The synthesis filter bank: IMDCT of long/short blocks, sine/KBD windows
//  and overlap-add, for 1024- and 960-sample frames.
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
** $Id: filtbank.c,v 1.46 2009/01/26 23:51:15 menno Exp $
**/

#include <cstdlib>
#include <cassert>
#include "AacCommon.h"
#include "AacStructs.h"
#include "AacFiltbank.h"
#include "AacSyntax.h"
#include "AacKbdWin.h"
#include "AacSineWin.h"
#include "AacMdct.h"

namespace codec {
namespace faad {

fb_info *filter_bank_init(uint16_t frame_len)
{
    uint16_t nshort = frame_len/8;

    fb_info *fb = (fb_info*)faad_malloc(sizeof(fb_info));
    memset(fb, 0, sizeof(fb_info));

    /* normal */
    fb->mdct256 = faad_mdct_init(2*nshort);
    fb->mdct2048 = faad_mdct_init(2*frame_len);

    if (frame_len == 1024)
    {
        fb->long_window[0]  = sine_long_1024;
        fb->short_window[0] = sine_short_128;
        fb->long_window[1]  = kbd_long_1024;
        fb->short_window[1] = kbd_short_128;
    } else /* (frame_len == 960) */ {
        fb->long_window[0]  = sine_long_960;
        fb->short_window[0] = sine_short_120;
        fb->long_window[1]  = kbd_long_960;
        fb->short_window[1] = kbd_short_120;
    }

    return fb;
}

void filter_bank_end(fb_info *fb)
{
    if (fb != NULL)
    {

        faad_mdct_end(fb->mdct256);
        faad_mdct_end(fb->mdct2048);

        faad_free(fb);
    }
}

static inline void imdct_long(fb_info *fb, real_t *in_data, real_t *out_data, uint16_t len)
{
    (void)len;
    faad_imdct(fb->mdct2048, in_data, out_data);
}


void ifilter_bank(fb_info *fb, uint8_t window_sequence, uint8_t window_shape,
                  uint8_t window_shape_prev, real_t *freq_in,
                  real_t *time_out, real_t *overlap,
                  uint8_t object_type, uint16_t frame_len)
{
    int16_t i;
    real_t transf_buf[2*1024] = {0};

    const real_t *window_long = NULL;
    const real_t *window_long_prev = NULL;
    const real_t *window_short = NULL;
    const real_t *window_short_prev = NULL;

    uint16_t nlong = frame_len;
    uint16_t nshort = frame_len/8;
    uint16_t trans = nshort/2;

    uint16_t nflat_ls = (nlong-nshort)/2;


    /* select windows of current frame and previous frame (Sine or KBD) */
        (void)object_type;
        window_long       = fb->long_window[window_shape];
        window_long_prev  = fb->long_window[window_shape_prev];
        window_short      = fb->short_window[window_shape];
        window_short_prev = fb->short_window[window_shape_prev];


    switch (window_sequence)
    {
    case ONLY_LONG_SEQUENCE:
        /* perform iMDCT */
        imdct_long(fb, freq_in, transf_buf, 2*nlong);

        /* add second half output of previous frame to windowed output of current frame */
        for (i = 0; i < nlong; i+=4)
        {
            time_out[i]   = overlap[i]   + MUL_F(transf_buf[i],window_long_prev[i]);
            time_out[i+1] = overlap[i+1] + MUL_F(transf_buf[i+1],window_long_prev[i+1]);
            time_out[i+2] = overlap[i+2] + MUL_F(transf_buf[i+2],window_long_prev[i+2]);
            time_out[i+3] = overlap[i+3] + MUL_F(transf_buf[i+3],window_long_prev[i+3]);
        }

        /* window the second half and save as overlap for next frame */
        for (i = 0; i < nlong; i+=4)
        {
            overlap[i]   = MUL_F(transf_buf[nlong+i],window_long[nlong-1-i]);
            overlap[i+1] = MUL_F(transf_buf[nlong+i+1],window_long[nlong-2-i]);
            overlap[i+2] = MUL_F(transf_buf[nlong+i+2],window_long[nlong-3-i]);
            overlap[i+3] = MUL_F(transf_buf[nlong+i+3],window_long[nlong-4-i]);
        }
        break;

    case LONG_START_SEQUENCE:
        /* perform iMDCT */
        imdct_long(fb, freq_in, transf_buf, 2*nlong);

        /* add second half output of previous frame to windowed output of current frame */
        for (i = 0; i < nlong; i+=4)
        {
            time_out[i]   = overlap[i]   + MUL_F(transf_buf[i],window_long_prev[i]);
            time_out[i+1] = overlap[i+1] + MUL_F(transf_buf[i+1],window_long_prev[i+1]);
            time_out[i+2] = overlap[i+2] + MUL_F(transf_buf[i+2],window_long_prev[i+2]);
            time_out[i+3] = overlap[i+3] + MUL_F(transf_buf[i+3],window_long_prev[i+3]);
        }

        /* window the second half and save as overlap for next frame */
        /* construct second half window using padding with 1's and 0's */
        for (i = 0; i < nflat_ls; i++)
            overlap[i] = transf_buf[nlong+i];
        for (i = 0; i < nshort; i++)
            overlap[nflat_ls+i] = MUL_F(transf_buf[nlong+nflat_ls+i],window_short[nshort-i-1]);
        for (i = 0; i < nflat_ls; i++)
            overlap[nflat_ls+nshort+i] = 0;
        break;

    case EIGHT_SHORT_SEQUENCE:
        /* perform iMDCT for each short block */
        faad_imdct(fb->mdct256, freq_in+0*nshort, transf_buf+2*nshort*0);
        faad_imdct(fb->mdct256, freq_in+1*nshort, transf_buf+2*nshort*1);
        faad_imdct(fb->mdct256, freq_in+2*nshort, transf_buf+2*nshort*2);
        faad_imdct(fb->mdct256, freq_in+3*nshort, transf_buf+2*nshort*3);
        faad_imdct(fb->mdct256, freq_in+4*nshort, transf_buf+2*nshort*4);
        faad_imdct(fb->mdct256, freq_in+5*nshort, transf_buf+2*nshort*5);
        faad_imdct(fb->mdct256, freq_in+6*nshort, transf_buf+2*nshort*6);
        faad_imdct(fb->mdct256, freq_in+7*nshort, transf_buf+2*nshort*7);

        /* add second half output of previous frame to windowed output of current frame */
        for (i = 0; i < nflat_ls; i++)
            time_out[i] = overlap[i];
        for(i = 0; i < nshort; i++)
        {
            time_out[nflat_ls+         i] = overlap[nflat_ls+         i] + MUL_F(transf_buf[nshort*0+i],window_short_prev[i]);
            time_out[nflat_ls+1*nshort+i] = overlap[nflat_ls+nshort*1+i] + MUL_F(transf_buf[nshort*1+i],window_short[nshort-1-i]) + MUL_F(transf_buf[nshort*2+i],window_short[i]);
            time_out[nflat_ls+2*nshort+i] = overlap[nflat_ls+nshort*2+i] + MUL_F(transf_buf[nshort*3+i],window_short[nshort-1-i]) + MUL_F(transf_buf[nshort*4+i],window_short[i]);
            time_out[nflat_ls+3*nshort+i] = overlap[nflat_ls+nshort*3+i] + MUL_F(transf_buf[nshort*5+i],window_short[nshort-1-i]) + MUL_F(transf_buf[nshort*6+i],window_short[i]);
            if (i < trans)
                time_out[nflat_ls+4*nshort+i] = overlap[nflat_ls+nshort*4+i] + MUL_F(transf_buf[nshort*7+i],window_short[nshort-1-i]) + MUL_F(transf_buf[nshort*8+i],window_short[i]);
        }

        /* window the second half and save as overlap for next frame */
        for(i = 0; i < nshort; i++)
        {
            if (i >= trans)
                overlap[nflat_ls+4*nshort+i-nlong] = MUL_F(transf_buf[nshort*7+i],window_short[nshort-1-i]) + MUL_F(transf_buf[nshort*8+i],window_short[i]);
            overlap[nflat_ls+5*nshort+i-nlong] = MUL_F(transf_buf[nshort*9+i],window_short[nshort-1-i]) + MUL_F(transf_buf[nshort*10+i],window_short[i]);
            overlap[nflat_ls+6*nshort+i-nlong] = MUL_F(transf_buf[nshort*11+i],window_short[nshort-1-i]) + MUL_F(transf_buf[nshort*12+i],window_short[i]);
            overlap[nflat_ls+7*nshort+i-nlong] = MUL_F(transf_buf[nshort*13+i],window_short[nshort-1-i]) + MUL_F(transf_buf[nshort*14+i],window_short[i]);
            overlap[nflat_ls+8*nshort+i-nlong] = MUL_F(transf_buf[nshort*15+i],window_short[nshort-1-i]);
        }
        for (i = 0; i < nflat_ls; i++)
            overlap[nflat_ls+nshort+i] = 0;
        break;

    case LONG_STOP_SEQUENCE:
        /* perform iMDCT */
        imdct_long(fb, freq_in, transf_buf, 2*nlong);

        /* add second half output of previous frame to windowed output of current frame */
        /* construct first half window using padding with 1's and 0's */
        for (i = 0; i < nflat_ls; i++)
            time_out[i] = overlap[i];
        for (i = 0; i < nshort; i++)
            time_out[nflat_ls+i] = overlap[nflat_ls+i] + MUL_F(transf_buf[nflat_ls+i],window_short_prev[i]);
        for (i = 0; i < nflat_ls; i++)
            time_out[nflat_ls+nshort+i] = overlap[nflat_ls+nshort+i] + transf_buf[nflat_ls+nshort+i];

        /* window the second half and save as overlap for next frame */
        for (i = 0; i < nlong; i++)
            overlap[i] = MUL_F(transf_buf[nlong+i],window_long[nlong-1-i]);
		break;
    }


}

} // namespace faad
} // namespace codec
