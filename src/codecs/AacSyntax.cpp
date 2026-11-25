// ============================================================================
//  offgrid-codecs - AAC bitstream syntax (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacSyntax.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  Raw data block parsing: SCE/CPE/LFE/fill (incl. OFFgrid-SDR's DAB+
//  fill-element change)/PCE/DSE, ICS info, section, scalefactor and spectral
//  data, TNS, pulse, the DRM error-resilient layout and ADTS headers.
//
//  OFFgrid-SDR's changes here (each marked "OFFgrid-SDR" below): fill
//  elements parsed in DRM-enabled builds for DAB+ (as the browser version's
//  FAAD2 patch); SBR fill data skipped when the configuration says there is
//  no SBR (sbr_signalled_absent); dynamic range gains kept only while frames
//  keep carrying them (drc->seen).  On undamaged streams only the last can
//  matter, and only for one that sends DRC now and then rather than in
//  every frame (each gain then acts on one frame, not until the next).
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
** $Id: syntax.c,v 1.93 2009/01/26 23:51:15 menno Exp $
**/

#include <cstdlib>
#include <cstdio>
#include "AacCommon.h"
#include "AacStructs.h"
#include "AacSyntax.h"
#include "AacSpecrec.h"
#include "AacHuffman.h"
#include "AacBits.h"
#include "AacPulse.h"
#include "AacDrc.h"
#include "AacRvlc.h"
#include "AacSbrSyntax.h"
#include "AacMp4.h"

namespace codec {
namespace faad {

/*
   Reads the AAC bitstream as defined in 14496-3 (MPEG-4 Audio)
*/


/* static function declarations */
static void decode_sce_lfe(NeAACDecStruct *hDecoder, NeAACDecFrameInfo *hInfo, bitfile *ld,
                           uint8_t id_syn_ele);
static void decode_cpe(NeAACDecStruct *hDecoder, NeAACDecFrameInfo *hInfo, bitfile *ld,
                       uint8_t id_syn_ele);
static uint8_t single_lfe_channel_element(NeAACDecStruct *hDecoder, bitfile *ld,
                                          uint8_t channel, uint8_t *tag);
static uint8_t channel_pair_element(NeAACDecStruct *hDecoder, bitfile *ld,
                                    uint8_t channel, uint8_t *tag);
static uint16_t data_stream_element(NeAACDecStruct *hDecoder, bitfile *ld);
static uint8_t program_config_element(program_config *pce, bitfile *ld);
static uint8_t fill_element(NeAACDecStruct *hDecoder, bitfile *ld, drc_info *drc
                            ,uint8_t sbr_ele
                            );
static uint8_t individual_channel_stream(NeAACDecStruct *hDecoder, element *ele,
                                         bitfile *ld, ic_stream *ics, uint8_t scal_flag,
                                         int16_t *spec_data);
static uint8_t ics_info(NeAACDecStruct *hDecoder, ic_stream *ics, bitfile *ld,
                        uint8_t common_window);
static uint8_t section_data(NeAACDecStruct *hDecoder, ic_stream *ics, bitfile *ld);
static uint8_t scale_factor_data(NeAACDecStruct *hDecoder, ic_stream *ics, bitfile *ld);
static uint8_t spectral_data(NeAACDecStruct *hDecoder, ic_stream *ics, bitfile *ld,
                             int16_t *spectral_data);
static uint8_t pulse_data(ic_stream *ics, pulse_info *pul, bitfile *ld);
static void tns_data(ic_stream *ics, tns_info *tns, bitfile *ld);
static uint8_t adts_fixed_header(adts_header *adts, bitfile *ld);
static void adts_variable_header(adts_header *adts, bitfile *ld);
static void adts_error_check(adts_header *adts, bitfile *ld);
static uint8_t side_info(NeAACDecStruct *hDecoder, element *ele,
                         bitfile *ld, ic_stream *ics, uint8_t scal_flag);
static int8_t DRM_aac_scalable_main_header(NeAACDecStruct *hDecoder, ic_stream *ics1, ic_stream *ics2,
                                           bitfile *ld, uint8_t this_layer_stereo);
/* OFFgrid-SDR: also declared in DRM builds, which decode DAB+ (plain AAC with fill elements) as well */
static uint16_t extension_payload(bitfile *ld, drc_info *drc, uint16_t count);
static uint8_t dynamic_range_info(bitfile *ld, drc_info *drc);
static uint8_t excluded_channels(bitfile *ld, drc_info *drc);


/* Table 4.4.1 */
int8_t GASpecificConfig(bitfile *ld, mp4AudioSpecificConfig *mp4ASC,
                        program_config *pce_out)
{
    program_config pce;

    /* 1024 or 960 */
    mp4ASC->frameLengthFlag = faad_get1bit(ld);

    mp4ASC->dependsOnCoreCoder = faad_get1bit(ld);
    if (mp4ASC->dependsOnCoreCoder == 1)
    {
        mp4ASC->coreCoderDelay = (uint16_t)faad_getbits(ld, 14);
    }

    mp4ASC->extensionFlag = faad_get1bit(ld);
    if (mp4ASC->channelsConfiguration == 0)
    {
        if (program_config_element(&pce, ld))
            return -3;
        //mp4ASC->channelsConfiguration = pce.channels;

        if (pce_out != NULL)
            memcpy(pce_out, &pce, sizeof(program_config));

        /*
        if (pce.num_valid_cc_elements)
            return -3;
        */
    }

    if (mp4ASC->extensionFlag == 1)
    {
        /* Error resilience not supported yet */
        if (mp4ASC->objectTypeIndex >= ER_OBJECT_START)
        {
            mp4ASC->aacSectionDataResilienceFlag = faad_get1bit(ld);
            mp4ASC->aacScalefactorDataResilienceFlag = faad_get1bit(ld);
            mp4ASC->aacSpectralDataResilienceFlag = faad_get1bit(ld);
        }
        /* 1 bit: extensionFlag3 */
        faad_getbits(ld, 1);
	}

    return 0;
}

/* Table 4.4.2 */
/* An MPEG-4 Audio decoder is only required to follow the Program
   Configuration Element in GASpecificConfig(). The decoder shall ignore
   any Program Configuration Elements that may occur in raw data blocks.
   PCEs transmitted in raw data blocks cannot be used to convey decoder
   configuration information.
*/
static uint8_t program_config_element(program_config *pce, bitfile *ld)
{
    uint8_t i;

    memset(pce, 0, sizeof(program_config));

    pce->channels = 0;

    pce->element_instance_tag = (uint8_t)faad_getbits(ld, 4);

    pce->object_type = (uint8_t)faad_getbits(ld, 2);
    pce->sf_index = (uint8_t)faad_getbits(ld, 4);
    pce->num_front_channel_elements = (uint8_t)faad_getbits(ld, 4);
    pce->num_side_channel_elements = (uint8_t)faad_getbits(ld, 4);
    pce->num_back_channel_elements = (uint8_t)faad_getbits(ld, 4);
    pce->num_lfe_channel_elements = (uint8_t)faad_getbits(ld, 2);
    pce->num_assoc_data_elements = (uint8_t)faad_getbits(ld, 3);
    pce->num_valid_cc_elements = (uint8_t)faad_getbits(ld, 4);

    pce->mono_mixdown_present = faad_get1bit(ld);
    if (pce->mono_mixdown_present == 1)
    {
        pce->mono_mixdown_element_number = (uint8_t)faad_getbits(ld, 4);
    }

    pce->stereo_mixdown_present = faad_get1bit(ld);
    if (pce->stereo_mixdown_present == 1)
    {
        pce->stereo_mixdown_element_number = (uint8_t)faad_getbits(ld, 4);
    }

    pce->matrix_mixdown_idx_present = faad_get1bit(ld);
    if (pce->matrix_mixdown_idx_present == 1)
    {
        pce->matrix_mixdown_idx = (uint8_t)faad_getbits(ld, 2);
        pce->pseudo_surround_enable = faad_get1bit(ld);
    }

    for (i = 0; i < pce->num_front_channel_elements; i++)
    {
        pce->front_element_is_cpe[i] = faad_get1bit(ld);
        pce->front_element_tag_select[i] = (uint8_t)faad_getbits(ld, 4);

        if (pce->front_element_is_cpe[i] & 1)
        {
            pce->cpe_channel[pce->front_element_tag_select[i]] = pce->channels;
            pce->num_front_channels += 2;
            pce->channels += 2;
        } else {
            pce->sce_channel[pce->front_element_tag_select[i]] = pce->channels;
            pce->num_front_channels++;
            pce->channels++;
        }
    }

    for (i = 0; i < pce->num_side_channel_elements; i++)
    {
        pce->side_element_is_cpe[i] = faad_get1bit(ld);
        pce->side_element_tag_select[i] = (uint8_t)faad_getbits(ld, 4);

        if (pce->side_element_is_cpe[i] & 1)
        {
            pce->cpe_channel[pce->side_element_tag_select[i]] = pce->channels;
            pce->num_side_channels += 2;
            pce->channels += 2;
        } else {
            pce->sce_channel[pce->side_element_tag_select[i]] = pce->channels;
            pce->num_side_channels++;
            pce->channels++;
        }
    }

    for (i = 0; i < pce->num_back_channel_elements; i++)
    {
        pce->back_element_is_cpe[i] = faad_get1bit(ld);
        pce->back_element_tag_select[i] = (uint8_t)faad_getbits(ld, 4);

        if (pce->back_element_is_cpe[i] & 1)
        {
            pce->cpe_channel[pce->back_element_tag_select[i]] = pce->channels;
            pce->channels += 2;
            pce->num_back_channels += 2;
        } else {
            pce->sce_channel[pce->back_element_tag_select[i]] = pce->channels;
            pce->num_back_channels++;
            pce->channels++;
        }
    }

    for (i = 0; i < pce->num_lfe_channel_elements; i++)
    {
        pce->lfe_element_tag_select[i] = (uint8_t)faad_getbits(ld, 4);

        pce->sce_channel[pce->lfe_element_tag_select[i]] = pce->channels;
        pce->num_lfe_channels++;
        pce->channels++;
    }

    for (i = 0; i < pce->num_assoc_data_elements; i++)
        pce->assoc_data_element_tag_select[i] = (uint8_t)faad_getbits(ld, 4);

    for (i = 0; i < pce->num_valid_cc_elements; i++)
    {
        pce->cc_element_is_ind_sw[i] = faad_get1bit(ld);
        pce->valid_cc_element_tag_select[i] = (uint8_t)faad_getbits(ld, 4);
    }

    faad_byte_align(ld);

    pce->comment_field_bytes = (uint8_t)faad_getbits(ld, 8);

    for (i = 0; i < pce->comment_field_bytes; i++)
    {
        pce->comment_field_data[i] = (uint8_t)faad_getbits(ld, 8);
    }
    pce->comment_field_data[i] = 0;

    if (pce->channels > MAX_CHANNELS)
        return 22;

    return 0;
}

static void decode_sce_lfe(NeAACDecStruct *hDecoder,
                           NeAACDecFrameInfo *hInfo, bitfile *ld,
                           uint8_t id_syn_ele)
{
    uint8_t channels = hDecoder->fr_channels;
    uint8_t tag = 0;

    /* One or two channels are used;
       exact number will be known after single_lfe_channel_element
    */
    if (channels+2 > MAX_CHANNELS)
    {
        hInfo->error = 12;
        return;
    }
    if (hDecoder->fr_ch_ele+1 > MAX_SYNTAX_ELEMENTS)
    {
        hInfo->error = 13;
        return;
    }

    /* for SCE hDecoder->element_output_channels[] is not set here because this
       can become 2 when some form of Parametric Stereo coding is used
    */

    if (hDecoder->element_id[hDecoder->fr_ch_ele] != INVALID_ELEMENT_ID &&
        hDecoder->element_id[hDecoder->fr_ch_ele] != id_syn_ele)
    {
        /* element inconsistency */
        hInfo->error = 21;
        return;
    }

    /* save the syntax element id */
    hDecoder->element_id[hDecoder->fr_ch_ele] = id_syn_ele;

    /* decode the element */
    hInfo->error = single_lfe_channel_element(hDecoder, ld, channels, &tag);

    /* map output channels position to internal data channels */
    if (hDecoder->element_output_channels[hDecoder->fr_ch_ele] == 2)
    {
        /* this might be faulty when pce_set is true */
        hDecoder->internal_channel[channels] = channels;
        hDecoder->internal_channel[channels+1] = channels+1;
    } else {
        if (hDecoder->pce_set)
        {
            if (hDecoder->pce.channels > MAX_CHANNELS)
            {
                hInfo->error = 22;
                return;
            }
            hDecoder->internal_channel[hDecoder->pce.sce_channel[tag]] = channels;
        } else {
            hDecoder->internal_channel[channels] = channels;
        }
    }

    hDecoder->fr_channels += hDecoder->element_output_channels[hDecoder->fr_ch_ele];
    hDecoder->fr_ch_ele++;
}

static void decode_cpe(NeAACDecStruct *hDecoder, NeAACDecFrameInfo *hInfo, bitfile *ld,
                       uint8_t id_syn_ele)
{
    uint8_t channels = hDecoder->fr_channels;
    uint8_t tag = 0;

    if (channels+2 > MAX_CHANNELS)
    {
        hInfo->error = 12;
        return;
    }
    if (hDecoder->fr_ch_ele+1 > MAX_SYNTAX_ELEMENTS)
    {
        hInfo->error = 13;
        return;
    }
    if (hDecoder->pce_set && (hDecoder->pce.channels > MAX_CHANNELS))
    {
        hInfo->error = 22;
        return;
    }

    /* for CPE the number of output channels is always 2 */
    if (hDecoder->element_output_channels[hDecoder->fr_ch_ele] == 0)
    {
        /* element_output_channels not set yet */
        hDecoder->element_output_channels[hDecoder->fr_ch_ele] = 2;
    } else if (hDecoder->element_output_channels[hDecoder->fr_ch_ele] != 2) {
        /* element inconsistency */
        hInfo->error = 21;
        return;
    }

    if (hDecoder->element_id[hDecoder->fr_ch_ele] != INVALID_ELEMENT_ID &&
        hDecoder->element_id[hDecoder->fr_ch_ele] != id_syn_ele)
    {
        /* element inconsistency */
        hInfo->error = 21;
        return;
    }

    /* save the syntax element id */
    hDecoder->element_id[hDecoder->fr_ch_ele] = id_syn_ele;

    /* decode the element */
    hInfo->error = channel_pair_element(hDecoder, ld, channels, &tag);

    /* map output channel position to internal data channels */
    if (hDecoder->pce_set)
    {
        hDecoder->internal_channel[hDecoder->pce.cpe_channel[tag]] = channels;
        hDecoder->internal_channel[hDecoder->pce.cpe_channel[tag]+1] = channels+1;
    } else {
        hDecoder->internal_channel[channels] = channels;
        hDecoder->internal_channel[channels+1] = channels+1;
    }

    hDecoder->fr_channels += 2;
    hDecoder->fr_ch_ele++;
}

void raw_data_block(NeAACDecStruct *hDecoder, NeAACDecFrameInfo *hInfo,
                    bitfile *ld, program_config *pce, drc_info *drc)
{
    uint8_t id_syn_ele;
    uint8_t ele_this_frame = 0;

    hDecoder->fr_channels = 0;
    hDecoder->fr_ch_ele = 0;
    hDecoder->first_syn_ele = 25;
    hDecoder->has_lfe = 0;
    drc->seen = 0;   /* OFFgrid-SDR: see the end of this function */

    if (hDecoder->object_type < ER_OBJECT_START)
    {
        /* Table 4.4.3: raw_data_block() */
        while ((id_syn_ele = (uint8_t)faad_getbits(ld, LEN_SE_ID)) != ID_END)
        {
            switch (id_syn_ele) {
            case ID_SCE:
                ele_this_frame++;
                if (hDecoder->first_syn_ele == 25) hDecoder->first_syn_ele = id_syn_ele;
                decode_sce_lfe(hDecoder, hInfo, ld, id_syn_ele);
                if (hInfo->error > 0)
                    return;
                break;
            case ID_CPE:
                ele_this_frame++;
                if (hDecoder->first_syn_ele == 25) hDecoder->first_syn_ele = id_syn_ele;
                decode_cpe(hDecoder, hInfo, ld, id_syn_ele);
                if (hInfo->error > 0)
                    return;
                break;
            case ID_LFE:
                hInfo->error = 32;
                if (hInfo->error > 0)
                    return;
                break;
            case ID_CCE: /* not implemented yet, but skip the bits */
                hInfo->error = 32;
                if (hInfo->error > 0)
                    return;
                break;
            case ID_DSE:
                ele_this_frame++;
                data_stream_element(hDecoder, ld);
                break;
            case ID_PCE:
                if (ele_this_frame != 0)
                {
                    hInfo->error = 31;
                    return;
                }
                ele_this_frame++;
                /* 14496-4: 5.6.4.1.2.1.3: */
                /* program_configuration_element()'s in access units shall be ignored */
                program_config_element(pce, ld);
                //if ((hInfo->error = program_config_element(pce, ld)) > 0)
                //    return;
                //hDecoder->pce_set = 1;
                break;
            case ID_FIL:
                ele_this_frame++;
                /* one sbr_info describes a channel_element not a channel! */
                /* if we encounter SBR data here: error */
                /* SBR data will be read directly in the SCE/LFE/CPE element */
                if ((hInfo->error = fill_element(hDecoder, ld, drc
                    , INVALID_SBR_ELEMENT
                    )) > 0)
                    return;
                break;
            }
            if (ld->error != 0)
            {
                hInfo->error = 32;
                return;
            }
        }
    } else {
        /* Table 262: er_raw_data_block() */
        switch (hDecoder->channelConfiguration)
        {
        case 1:
            decode_sce_lfe(hDecoder, hInfo, ld, ID_SCE);
            if (hInfo->error > 0)
                return;
            break;
        case 2:
            decode_cpe(hDecoder, hInfo, ld, ID_CPE);
            if (hInfo->error > 0)
                return;
            break;
        case 3:
            decode_sce_lfe(hDecoder, hInfo, ld, ID_SCE);
            if (hInfo->error > 0)
                return;
            decode_cpe(hDecoder, hInfo, ld, ID_CPE);
            if (hInfo->error > 0)
                return;
            break;
        case 4:
            decode_sce_lfe(hDecoder, hInfo, ld, ID_SCE);
            if (hInfo->error > 0)
                return;
            decode_cpe(hDecoder, hInfo, ld, ID_CPE);
            if (hInfo->error > 0)
                return;
            decode_sce_lfe(hDecoder, hInfo, ld, ID_SCE);
            if (hInfo->error > 0)
                return;
            break;
        case 5:
            decode_sce_lfe(hDecoder, hInfo, ld, ID_SCE);
            if (hInfo->error > 0)
                return;
            decode_cpe(hDecoder, hInfo, ld, ID_CPE);
            if (hInfo->error > 0)
                return;
            decode_cpe(hDecoder, hInfo, ld, ID_CPE);
            if (hInfo->error > 0)
                return;
            break;
        case 6:
            decode_sce_lfe(hDecoder, hInfo, ld, ID_SCE);
            if (hInfo->error > 0)
                return;
            decode_cpe(hDecoder, hInfo, ld, ID_CPE);
            if (hInfo->error > 0)
                return;
            decode_cpe(hDecoder, hInfo, ld, ID_CPE);
            if (hInfo->error > 0)
                return;
            decode_sce_lfe(hDecoder, hInfo, ld, ID_LFE);
            if (hInfo->error > 0)
                return;
            break;
        case 7: /* 8 channels */
            decode_sce_lfe(hDecoder, hInfo, ld, ID_SCE);
            if (hInfo->error > 0)
                return;
            decode_cpe(hDecoder, hInfo, ld, ID_CPE);
            if (hInfo->error > 0)
                return;
            decode_cpe(hDecoder, hInfo, ld, ID_CPE);
            if (hInfo->error > 0)
                return;
            decode_cpe(hDecoder, hInfo, ld, ID_CPE);
            if (hInfo->error > 0)
                return;
            decode_sce_lfe(hDecoder, hInfo, ld, ID_LFE);
            if (hInfo->error > 0)
                return;
            break;
        default:
            hInfo->error = 7;
            return;
        }
    }

    /* OFFgrid-SDR: FAAD2 applies the last dynamic range info it read to every later frame, for ever. A stream with
       DRC sends it in every frame (in a fill element after the audio, so it acts from the next frame on) and nothing
       changes for it here; but one damaged AU whose bits look like DRC data left a fixed gain (seen: +9 dB, -6 dB)
       on all the audio after it. So the gains last only while frames keep bringing them. */
    if (!drc->seen)
        drc->present = 0;

    /* new in corrigendum 14496-3:2002 */
    if (hDecoder->object_type != DRM_ER_LC
        )
    {
        faad_byte_align(ld);
    }

    return;
}

/* Table 4.4.4 and */
/* Table 4.4.9 */
static uint8_t single_lfe_channel_element(NeAACDecStruct *hDecoder, bitfile *ld,
                                          uint8_t channel, uint8_t *tag)
{
    uint8_t retval = 0;
    element sce = {};
    ic_stream *ics = &(sce.ics1);
    int16_t spec_data[1024] = {0};

    sce.element_instance_tag = (uint8_t)faad_getbits(ld, LEN_TAG);

    *tag = sce.element_instance_tag;
    sce.channel = channel;
    sce.paired_channel = -1;

    retval = individual_channel_stream(hDecoder, &sce, ld, ics, 0, spec_data);
    if (retval > 0)
        return retval;

    /* IS not allowed in single channel */
    if (ics->is_used)
        return 32;

    /* check if next bitstream element is a fill element */
    /* if so, read it now so SBR decoding can be done in case of a file with SBR */
    if (faad_showbits(ld, LEN_SE_ID) == ID_FIL)
    {
        faad_flushbits(ld, LEN_SE_ID);

        /* one sbr_info describes a channel_element not a channel! */
        if ((retval = fill_element(hDecoder, ld, hDecoder->drc, hDecoder->fr_ch_ele)) > 0)
        {
            return retval;
        }
    }

    /* noiseless coding is done, spectral reconstruction is done now */
    retval = reconstruct_single_channel(hDecoder, ics, &sce, spec_data);
    if (retval > 0)
        return retval;

    return 0;
}

/* Table 4.4.5 */
static uint8_t channel_pair_element(NeAACDecStruct *hDecoder, bitfile *ld,
                                    uint8_t channels, uint8_t *tag)
{
    int16_t spec_data1[1024] = {0};
    int16_t spec_data2[1024] = {0};
    element cpe = {};
    ic_stream *ics1 = &(cpe.ics1);
    ic_stream *ics2 = &(cpe.ics2);
    uint8_t result;

    cpe.channel        = channels;
    cpe.paired_channel = channels+1;

    cpe.element_instance_tag = (uint8_t)faad_getbits(ld, LEN_TAG);
    *tag = cpe.element_instance_tag;

    if ((cpe.common_window = faad_get1bit(ld)) & 1)
    {
        /* both channels have common ics information */
        if ((result = ics_info(hDecoder, ics1, ld, cpe.common_window)) > 0)
            return result;

        ics1->ms_mask_present = (uint8_t)faad_getbits(ld, 2);
        if (ics1->ms_mask_present == 3)
        {
            /* bitstream error */
            return 32;
        }
        if (ics1->ms_mask_present == 1)
        {
            uint8_t g, sfb;
            for (g = 0; g < ics1->num_window_groups; g++)
            {
                for (sfb = 0; sfb < ics1->max_sfb; sfb++)
                {
                    ics1->ms_used[g][sfb] = faad_get1bit(ld);
                }
            }
        }

        if ((hDecoder->object_type >= ER_OBJECT_START) && (ics1->predictor_data_present))
        {
            if ((
                faad_get1bit(ld)) & 1)
            {
                return 26;
            }
        }

        memcpy(ics2, ics1, sizeof(ic_stream));
    } else {
        ics1->ms_mask_present = 0;
    }

    if ((result = individual_channel_stream(hDecoder, &cpe, ld, ics1,
        0, spec_data1)) > 0)
    {
        return result;
    }

    if (cpe.common_window && (hDecoder->object_type >= ER_OBJECT_START) &&
        (ics1->predictor_data_present))
    {
        if ((
            faad_get1bit(ld)) & 1)
        {
            return 26;
        }
    }

    if ((result = individual_channel_stream(hDecoder, &cpe, ld, ics2,
        0, spec_data2)) > 0)
    {
        return result;
    }

    /* check if next bitstream element is a fill element */
    /* if so, read it now so SBR decoding can be done in case of a file with SBR */
    if (faad_showbits(ld, LEN_SE_ID) == ID_FIL)
    {
        faad_flushbits(ld, LEN_SE_ID);

        /* one sbr_info describes a channel_element not a channel! */
        if ((result = fill_element(hDecoder, ld, hDecoder->drc, hDecoder->fr_ch_ele)) > 0)
        {
            return result;
        }
    }

    /* noiseless coding is done, spectral reconstruction is done now */
    if ((result = reconstruct_channel_pair(hDecoder, ics1, ics2, &cpe,
        spec_data1, spec_data2)) > 0)
    {
        return result;
    }

    return 0;
}

/* Table 4.4.6 */
static uint8_t ics_info(NeAACDecStruct *hDecoder, ic_stream *ics, bitfile *ld,
                        uint8_t common_window)
{
    uint8_t retval = 0;
    uint8_t ics_reserved_bit;

    ics_reserved_bit = faad_get1bit(ld);
    if (ics_reserved_bit != 0)
        return 32;
    ics->window_sequence = (uint8_t)faad_getbits(ld, 2);
    ics->window_shape = faad_get1bit(ld);


    if (ics->window_sequence == EIGHT_SHORT_SEQUENCE)
    {
        ics->max_sfb = (uint8_t)faad_getbits(ld, 4);
        ics->scale_factor_grouping = (uint8_t)faad_getbits(ld, 7);
    } else {
        ics->max_sfb = (uint8_t)faad_getbits(ld, 6);
    }

    /* get the grouping information */
    if ((retval = window_grouping_info(hDecoder, ics)) > 0)
        return retval;


    /* should be an error */
    /* check the range of max_sfb */
    if (ics->max_sfb > ics->num_swb)
        return 16;

    if (ics->window_sequence != EIGHT_SHORT_SEQUENCE)
    {
        if ((ics->predictor_data_present = faad_get1bit(ld)) & 1)
        {
            if (hDecoder->object_type == MAIN) /* MPEG2 style AAC predictor */
            {
                uint8_t sfb;
                uint8_t predictor_reset, predictor_reset_group_number, prediction_used;
                uint8_t limit = FAAD_MIN(ics->max_sfb, max_pred_sfb(hDecoder->sf_index));

                predictor_reset = faad_get1bit(ld);
                if (predictor_reset)
                {
                    predictor_reset_group_number =
                        (uint8_t)faad_getbits(ld, 5);
                } else {
                    predictor_reset_group_number = 0;
                }

                for (sfb = 0; sfb < limit; sfb++)
                {
                    prediction_used = faad_get1bit(ld);
                }
                (void)predictor_reset_group_number;
                (void)prediction_used;
            }
            (void)common_window;
        }
    }

    return retval;
}

/* Table 4.4.7 */
static uint8_t pulse_data(ic_stream *ics, pulse_info *pul, bitfile *ld)
{
    uint8_t i;

    pul->number_pulse = (uint8_t)faad_getbits(ld, 2);
    pul->pulse_start_sfb = (uint8_t)faad_getbits(ld, 6);

    /* check the range of pulse_start_sfb */
    if (pul->pulse_start_sfb > ics->num_swb)
        return 16;

    for (i = 0; i < pul->number_pulse+1; i++)
    {
        pul->pulse_offset[i] = (uint8_t)faad_getbits(ld, 5);
        pul->pulse_amp[i] = (uint8_t)faad_getbits(ld, 4);
    }

    return 0;
}


/* Table 4.4.10 */
static uint16_t data_stream_element(NeAACDecStruct *hDecoder, bitfile *ld)
{
    uint8_t byte_aligned;
    uint16_t i, count;
    (void)hDecoder;  /* TODO: remove parameter; rename method; why result is unused? */

    /* element_instance_tag = */ faad_getbits(ld, LEN_TAG);
    byte_aligned = faad_get1bit(ld);
    count = (uint16_t)faad_getbits(ld, 8);
    if (count == 255)
    {
        count += (uint16_t)faad_getbits(ld, 8);
    }
    if (byte_aligned)
        faad_byte_align(ld);

    for (i = 0; i < count; i++)
    {
        faad_getbits(ld, LEN_BYTE);
    }

    return count;
}

/* Table 4.4.11 */
static uint8_t fill_element(NeAACDecStruct *hDecoder, bitfile *ld, drc_info *drc
                            ,uint8_t sbr_ele
                            )
{
    uint16_t count;
    uint8_t bs_extension_type;

    count = (uint16_t)faad_getbits(ld, 4);
    if (count == 15)
    {
        count += (uint16_t)faad_getbits(ld, 8) - 1;
    }

    if (count > 0)
    {
        bs_extension_type = (uint8_t)faad_showbits(ld, 4);

        if ((bs_extension_type == EXT_SBR_DATA) ||
            (bs_extension_type == EXT_SBR_DATA_CRC))
        {
            /* OFFgrid-SDR: a stream whose configuration says it has no SBR (DAB+'s sbr_flag 0) has none: SBR-looking
               fill data there can only be damage, and FAAD2 would switch implicit SBR on for the rest of the stream
               (then every AU fails with error 23, or decodes through an SBR stage with no SBR data). It is skipped,
               as an unknown payload would be. */
            if (hDecoder->sbr_signalled_absent)
            {
                uint16_t i;
                for (i = 0; i < count; i++)
                    faad_getbits(ld, 8);
                return 0;
            }

            if (sbr_ele == INVALID_SBR_ELEMENT)
                return 24;

            if (!hDecoder->sbr[sbr_ele])
            {
                hDecoder->sbr[sbr_ele] = sbrDecodeInit(hDecoder->frameLength,
                    hDecoder->element_id[sbr_ele], 2*get_sample_rate(hDecoder->sf_index),
                    hDecoder->downSampledSBR
                    , 0
                    );
            }
            if (!hDecoder->sbr[sbr_ele])
                return 19;

            hDecoder->sbr_present_flag = 1;

            /* parse the SBR data */
            hDecoder->sbr[sbr_ele]->ret = sbr_extension_data(ld, hDecoder->sbr[sbr_ele], count,
                hDecoder->postSeekResetFlag);


            if (hDecoder->sbr[sbr_ele]->ps_used)
            {
                if (hDecoder->ps_present_flag != 0)
                {
                    hDecoder->ps_used[sbr_ele] = 1;

                    /* set element independent flag to 1 as well */
                    hDecoder->ps_used_global = 1;
                } else {
                    hDecoder->sbr[sbr_ele]->ps_used = 0;
                }
            }
        } else {
            /* OFFgrid-SDR: DRM forbids standard extension payloads, but only DRM streams should be refused them:
               a DRM-enabled build also decodes DAB+ and other plain AAC, whose fill elements (padding, DRC) are normal */
            if (hDecoder->object_type == DRM_ER_LC)
            {
                (void)drc;
                return 30;
            }
            while (count > 0)
            {
                uint16_t payload_bytes = extension_payload(ld, drc, count);
                if (payload_bytes <= count) {
                    count -= payload_bytes;
                } else {
                    count = 0;
                }
            }
        }
    }

    return 0;
}

/* Table 4.4.12 */

/* Table 4.4.13 ASME */
void DRM_aac_scalable_main_element(NeAACDecStruct *hDecoder, NeAACDecFrameInfo *hInfo,
                                   bitfile *ld, program_config *pce, drc_info *drc)
{
    uint8_t channels = hDecoder->fr_channels = 0;
    uint8_t this_layer_stereo = (hDecoder->channelConfiguration > 1) ? 1 : 0;
    element cpe = {};
    ic_stream *ics1 = &(cpe.ics1);
    ic_stream *ics2 = &(cpe.ics2);
    int16_t spec_data1[1024] = {0};
    int16_t spec_data2[1024] = {0};

    (void)drc;  /* TODO: remove unused parameter? */
    (void)pce;  /* TODO: remove unused parameter? */

    hDecoder->fr_ch_ele = 0;

    hInfo->error = DRM_aac_scalable_main_header(hDecoder, ics1, ics2, ld, this_layer_stereo);
    if (hInfo->error > 0)
        return;

    cpe.common_window = 1;
    if (this_layer_stereo)
    {
        hDecoder->element_id[0] = ID_CPE;
        if (hDecoder->element_output_channels[hDecoder->fr_ch_ele] == 0)
            hDecoder->element_output_channels[hDecoder->fr_ch_ele] = 2;
    } else {
        hDecoder->element_id[0] = ID_SCE;
    }

    if (this_layer_stereo)
    {
        cpe.channel        = 0;
        cpe.paired_channel = 1;
    }


    /* Stereo2 / Mono1 */
    ics1->tns_data_present = faad_get1bit(ld);

    if(faad_get1bit(ld)) {
         hInfo->error = 26;
         return;
    }

    hInfo->error = side_info(hDecoder, &cpe, ld, ics1, 1);
    if (hInfo->error > 0)
        return;
    if (this_layer_stereo)
    {
        /* Stereo3 */
        ics2->tns_data_present = faad_get1bit(ld);
            faad_get1bit(ld);
        hInfo->error = side_info(hDecoder, &cpe, ld, ics2, 1);
        if (hInfo->error > 0)
            return;
    }
    /* Stereo4 / Mono2 */
    if (ics1->tns_data_present)
        tns_data(ics1, &(ics1->tns), ld);
    if (this_layer_stereo)
    {
        /* Stereo5 */
        if (ics2->tns_data_present)
            tns_data(ics2, &(ics2->tns), ld);
    }

    /* CRC check */
    if (hDecoder->object_type == DRM_ER_LC)
    {
        if ((hInfo->error = (uint8_t)faad_check_CRC(ld, (uint16_t)faad_get_processed_bits(ld) - 8)) > 0)
            return;
    }

    /* Stereo6 / Mono3 */
    /* error resilient spectral data decoding */
    if ((hInfo->error = reordered_spectral_data(hDecoder, ics1, ld, spec_data1)) > 0)
    {
        return;
    }
    if (this_layer_stereo)
    {
        /* Stereo7 */
        /* error resilient spectral data decoding */
        if ((hInfo->error = reordered_spectral_data(hDecoder, ics2, ld, spec_data2)) > 0)
        {
            return;
        }
    }


    /* In case of DRM we need to read the SBR info before channel reconstruction */
    if ((hDecoder->sbr_present_flag == 1) && (hDecoder->object_type == DRM_ER_LC))
    {
        bitfile ld_sbr = {};
        uint32_t i;
        uint16_t count = 0;
        uint8_t *revbuffer;
        uint8_t *prevbufstart;
        uint8_t *pbufend;

        /* all forward bitreading should be finished at this point */
        uint32_t bitsconsumed = faad_get_processed_bits(ld);
        uint32_t buffer_size = faad_origbitbuffer_size(ld);
        uint8_t *buffer = (uint8_t*)faad_origbitbuffer(ld);

        if (bitsconsumed + 8 > buffer_size*8)
        {
            hInfo->error = 14;
            return;
        }

        if (!hDecoder->sbr[0])
        {
            hDecoder->sbr[0] = sbrDecodeInit(hDecoder->frameLength, hDecoder->element_id[0],
                2*get_sample_rate(hDecoder->sf_index), 0 /* ds SBR */, 1);
        }
        if (!hDecoder->sbr[0])
        {
            hInfo->error = 19;
            return;
        }

        /* Reverse bit reading of SBR data in DRM audio frame */
        revbuffer = (uint8_t*)faad_malloc(buffer_size*sizeof(uint8_t));
        prevbufstart = revbuffer;
        pbufend = &buffer[buffer_size - 1];
        for (i = 0; i < buffer_size; i++)
            *prevbufstart++ = reverse_byte(*pbufend--);

        /* Set SBR data */
        /* consider 8 bits from AAC-CRC */
        /* SBR buffer size is original buffer size minus AAC buffer size */
        count = (uint16_t)bit2byte(buffer_size*8 - bitsconsumed);
        faad_initbits(&ld_sbr, revbuffer, count);

        hDecoder->sbr[0]->sample_rate = get_sample_rate(hDecoder->sf_index);
        hDecoder->sbr[0]->sample_rate *= 2;

        faad_getbits(&ld_sbr, 8); /* Skip 8-bit CRC */

        hDecoder->sbr[0]->ret = sbr_extension_data(&ld_sbr, hDecoder->sbr[0], count, hDecoder->postSeekResetFlag);
        if (hDecoder->sbr[0]->ps_used)
        {
            hDecoder->ps_used[0] = 1;
            hDecoder->ps_used_global = 1;
        }

        if (ld_sbr.error)
        {
            hDecoder->sbr[0]->ret = 1;
        }

        /* check CRC */
        /* no need to check it if there was already an error */
        if (hDecoder->sbr[0]->ret == 0)
            hDecoder->sbr[0]->ret = (uint8_t)faad_check_CRC(&ld_sbr, (uint16_t)faad_get_processed_bits(&ld_sbr) - 8);

        /* SBR data was corrupted, disable it until the next header */
        if (hDecoder->sbr[0]->ret != 0)
        {
            hDecoder->sbr[0]->header_count = 0;
        }

        faad_endbits(&ld_sbr);

        if (revbuffer)
            faad_free(revbuffer);
    }

    if (this_layer_stereo)
    {
        hInfo->error = reconstruct_channel_pair(hDecoder, ics1, ics2, &cpe, spec_data1, spec_data2);
        if (hInfo->error > 0)
            return;
    } else {
        hInfo->error = reconstruct_single_channel(hDecoder, ics1, &cpe, spec_data1);
        if (hInfo->error > 0)
            return;
    }

    /* map output channels position to internal data channels */
    if (hDecoder->element_output_channels[hDecoder->fr_ch_ele] == 2)
    {
        /* this might be faulty when pce_set is true */
        hDecoder->internal_channel[channels] = channels;
        hDecoder->internal_channel[channels+1] = channels+1;
    } else {
        hDecoder->internal_channel[channels] = channels;
    }

    hDecoder->fr_channels += hDecoder->element_output_channels[hDecoder->fr_ch_ele];
    hDecoder->fr_ch_ele++;

    return;
}

/* Table 4.4.15 */
static int8_t DRM_aac_scalable_main_header(NeAACDecStruct *hDecoder, ic_stream *ics1, ic_stream *ics2,
                                           bitfile *ld, uint8_t this_layer_stereo)
{
    uint8_t retval = 0;
    uint8_t ics_reserved_bit;

    ics_reserved_bit = faad_get1bit(ld);
    if (ics_reserved_bit != 0)
        return 32;
    ics1->window_sequence = (uint8_t)faad_getbits(ld, 2);
    ics1->window_shape = faad_get1bit(ld);

    if (ics1->window_sequence == EIGHT_SHORT_SEQUENCE)
    {
        ics1->max_sfb = (uint8_t)faad_getbits(ld, 4);
        ics1->scale_factor_grouping = (uint8_t)faad_getbits(ld, 7);
    } else {
        ics1->max_sfb = (uint8_t)faad_getbits(ld, 6);
    }

    /* get the grouping information */
    if ((retval = window_grouping_info(hDecoder, ics1)) > 0)
        return retval;

    /* should be an error */
    /* check the range of max_sfb */
    if (ics1->max_sfb > ics1->num_swb)
        return 16;

    if (this_layer_stereo)
    {
        ics1->ms_mask_present = (uint8_t)faad_getbits(ld, 2);
        if (ics1->ms_mask_present == 3)
        {
            /* bitstream error */
            return 32;
        }
        if (ics1->ms_mask_present == 1)
        {
            uint8_t g, sfb;
            for (g = 0; g < ics1->num_window_groups; g++)
            {
                for (sfb = 0; sfb < ics1->max_sfb; sfb++)
                {
                    ics1->ms_used[g][sfb] = faad_get1bit(ld);
                }
            }
        }

        memcpy(ics2, ics1, sizeof(ic_stream));
    } else {
        ics1->ms_mask_present = 0;
    }

    return 0;
}

static uint8_t side_info(NeAACDecStruct *hDecoder, element *ele,
                         bitfile *ld, ic_stream *ics, uint8_t scal_flag)
{
    uint8_t result;

    ics->global_gain = (uint8_t)faad_getbits(ld, 8);

    if (!ele->common_window && !scal_flag)
    {
        if ((result = ics_info(hDecoder, ics, ld, ele->common_window)) > 0)
            return result;
    }

    if ((result = section_data(hDecoder, ics, ld)) > 0)
        return result;

    if ((result = scale_factor_data(hDecoder, ics, ld)) > 0)
        return result;

    if (!scal_flag)
    {
        /**
         **  NOTE: It could be that pulse data is available in scalable AAC too,
         **        as said in Amendment 1, this could be only the case for ER AAC,
         **        though. (have to check this out later)
         **/
        /* get pulse data */
        if ((ics->pulse_data_present = faad_get1bit(ld)) & 1)
        {
            if ((result = pulse_data(ics, &(ics->pul), ld)) > 0)
                return result;
        }

        /* get tns data */
        if ((ics->tns_data_present = faad_get1bit(ld)) & 1)
        {
            if (hDecoder->object_type < ER_OBJECT_START)
                tns_data(ics, &(ics->tns), ld);
        }

        /* get gain control data */
        if ((ics->gain_control_data_present = faad_get1bit(ld)) & 1)
        {
            return 1;
        }
    }

    if (hDecoder->aacSpectralDataResilienceFlag)
    {
        ics->length_of_reordered_spectral_data = (uint16_t)faad_getbits(ld, 14);

        if (hDecoder->channelConfiguration == 2)
        {
            if (ics->length_of_reordered_spectral_data > 6144)
                ics->length_of_reordered_spectral_data = 6144;
        } else {
            if (ics->length_of_reordered_spectral_data > 12288)
                ics->length_of_reordered_spectral_data = 12288;
        }

        ics->length_of_longest_codeword = (uint8_t)faad_getbits(ld, 6);
        if (ics->length_of_longest_codeword >= 49)
            ics->length_of_longest_codeword = 49;
    }

    /* RVLC spectral data is put here */
    if (hDecoder->aacScalefactorDataResilienceFlag)
    {
        if ((result = rvlc_decode_scale_factors(ics, ld)) > 0)
            return result;
    }

    return 0;
}

/* Table 4.4.24 */
static uint8_t individual_channel_stream(NeAACDecStruct *hDecoder, element *ele,
                                         bitfile *ld, ic_stream *ics, uint8_t scal_flag,
                                         int16_t *spec_data)
{
    uint8_t result;

    result = side_info(hDecoder, ele, ld, ics, scal_flag);
    if (result > 0)
        return result;

    if (hDecoder->object_type >= ER_OBJECT_START)
    {
        if (ics->tns_data_present)
            tns_data(ics, &(ics->tns), ld);
    }

    /* CRC check */
    if (hDecoder->object_type == DRM_ER_LC)
    {
        if ((result = (uint8_t)faad_check_CRC(ld, (uint16_t)faad_get_processed_bits(ld) - 8)) > 0)
            return result;
    }

    if (hDecoder->aacSpectralDataResilienceFlag)
    {
        /* error resilient spectral data decoding */
        if ((result = reordered_spectral_data(hDecoder, ics, ld, spec_data)) > 0)
        {
            return result;
        }
    } else {
        /* decode the spectral data */
        if ((result = spectral_data(hDecoder, ics, ld, spec_data)) > 0)
        {
            return result;
        }
    }

    /* pulse coding reconstruction */
    if (ics->pulse_data_present)
    {
        if (ics->window_sequence != EIGHT_SHORT_SEQUENCE)
        {
            if ((result = pulse_decode(ics, spec_data, hDecoder->frameLength)) > 0)
                return result;
        } else {
            return 2; /* pulse coding not allowed for short blocks */
        }
    }

    return 0;
}

/* Table 4.4.25 */
static uint8_t section_data(NeAACDecStruct *hDecoder, ic_stream *ics, bitfile *ld)
{
    uint8_t g;
    uint8_t sect_esc_val, sect_bits;
    uint8_t sect_lim; /* 51 or 120, anyways less than 127. */

    if (ics->window_sequence == EIGHT_SHORT_SEQUENCE) {
        sect_bits = 3;
        sect_lim = 8 * 15;
    } else {
        sect_bits = 5;
        sect_lim = MAX_SFB;
    }
    sect_esc_val = (1u<<sect_bits) - 1;


    for (g = 0; g < ics->num_window_groups; g++)
    {
        uint8_t k = 0;
        uint8_t i = 0;

        while (k < ics->max_sfb)
        {
            uint8_t vcb11 = 0;
            uint8_t sfb;
            uint8_t sect_len_incr;
            uint8_t sect_len = 0;
            uint8_t sect_cb_bits = 4;

            /* if "faad_getbits" detects error and returns "0", "k" is never
               incremented and we cannot leave the while loop */
            if (ld->error != 0)
                return 14;
            if (i >= sect_lim)
                return 15;

            if (hDecoder->aacSectionDataResilienceFlag)
                sect_cb_bits = 5;

            ics->sect_cb[g][i] = (uint8_t)faad_getbits(ld, sect_cb_bits);

            if (ics->sect_cb[g][i] == 12)
                return 32;


            /* PNS not allowed in DRM */
            if (ics->sect_cb[g][i] == NOISE_HCB)
                return 29;
            if (ics->sect_cb[g][i] == INTENSITY_HCB2 || ics->sect_cb[g][i] == INTENSITY_HCB)
                ics->is_used = 1;

            if (hDecoder->aacSectionDataResilienceFlag)
            {
                if ((ics->sect_cb[g][i] == 11) ||
                    ((ics->sect_cb[g][i] >= 16) && (ics->sect_cb[g][i] <= 32)))
                {
                    vcb11 = 1;
                }
            }
            if (vcb11)
            {
                sect_len_incr = 1;
            } else {
                sect_len_incr = (uint8_t)faad_getbits(ld, sect_bits);
            }
            while (sect_len_incr == sect_esc_val /* &&
                (k+sect_len < ics->max_sfb)*/)
            {
                sect_len += sect_len_incr;
                if (sect_len > sect_lim)
                    return 15;
                sect_len_incr = (uint8_t)faad_getbits(ld, sect_bits);
            }

            sect_len += sect_len_incr;

            ics->sect_start[g][i] = k;
            ics->sect_end[g][i] = k + sect_len;


            if (sect_len > sect_lim)
                return 15;
            if (k + sect_len > sect_lim)
                return 15;

            for (sfb = k; sfb < k + sect_len; sfb++)
            {
                ics->sfb_cb[g][sfb] = ics->sect_cb[g][i];
            }


            k += sect_len; /* k <= sect_lim */
            i++;
        }
        ics->num_sec[g] = i;

        /* the sum of all sect_len_incr elements for a given window
         * group shall equal max_sfb */
        if (k != ics->max_sfb)
        {
            return 32;
        }
    }


    return 0;
}

/*
 *  decode_scale_factors()
 *   decodes the scalefactors from the bitstream
 */
/*
 * All scalefactors (and also the stereo positions and pns energies) are
 * transmitted using Huffman coded DPCM relative to the previous active
 * scalefactor (respectively previous stereo position or previous pns energy,
 * see subclause 4.6.2 and 4.6.3). The first active scalefactor is
 * differentially coded relative to the global gain.
 */
static uint8_t decode_scale_factors(ic_stream *ics, bitfile *ld)
{
    uint8_t g, sfb;
    int16_t t;

    int16_t scale_factor = ics->global_gain;
    int16_t is_position = 0;
    int16_t scale_factor_max = 255;

    for (g = 0; g < ics->num_window_groups; g++)
    {
        for (sfb = 0; sfb < ics->max_sfb; sfb++)
        {
            switch (ics->sfb_cb[g][sfb])
            {
            case ZERO_HCB: /* zero book */
                ics->scale_factors[g][sfb] = 0;
//#define SF_PRINT
                break;
            case INTENSITY_HCB: /* intensity books */
            case INTENSITY_HCB2:

                /* decode intensity position */
                t = huffman_scale_factor(ld);
                is_position += (t - 60);
                ics->scale_factors[g][sfb] = is_position;

                break;
            case NOISE_HCB: /* noise books */

                /* PNS not allowed in DRM */
                return 29;

                break;
            default: /* spectral books */

                /* ics->scale_factors[g][sfb] must be between 0 and 255 */

                ics->scale_factors[g][sfb] = 0;

                /* decode scale factor */
                t = huffman_scale_factor(ld);
                scale_factor += (t - 60);
                if (scale_factor < 0 || scale_factor > 255)
                    return 4;
                ics->scale_factors[g][sfb] = FAAD_MIN(scale_factor, scale_factor_max);

                break;
            }
        }
    }

    return 0;
}

/* Table 4.4.26 */
static uint8_t scale_factor_data(NeAACDecStruct *hDecoder, ic_stream *ics, bitfile *ld)
{
    uint8_t ret = 0;

    if (!hDecoder->aacScalefactorDataResilienceFlag)
    {
        ret = decode_scale_factors(ics, ld);
    } else {
        /* In ER AAC the parameters for RVLC are seperated from the actual
           data that holds the scale_factors.
           Strangely enough, 2 parameters for HCR are put inbetween them.
        */
        ret = rvlc_scale_factor_data(ics, ld);
    }


    return ret;
}

/* Table 4.4.27 */
static void tns_data(ic_stream *ics, tns_info *tns, bitfile *ld)
{
    uint8_t w, filt, i, coef_bits;
    uint8_t n_filt_bits = 2;
    uint8_t length_bits = 6;
    uint8_t order_bits = 5;

    if (ics->window_sequence == EIGHT_SHORT_SEQUENCE)
    {
        n_filt_bits = 1;
        length_bits = 4;
        order_bits = 3;
    }

    for (w = 0; w < ics->num_windows; w++)
    {
        uint8_t start_coef_bits = 3;
        tns->n_filt[w] = (uint8_t)faad_getbits(ld, n_filt_bits);

        if (tns->n_filt[w])
        {
            if ((tns->coef_res[w] = faad_get1bit(ld)) & 1)
                start_coef_bits = 4;
        }

        for (filt = 0; filt < tns->n_filt[w]; filt++)
        {
            tns->length[w][filt] = (uint8_t)faad_getbits(ld, length_bits);
            tns->order[w][filt]  = (uint8_t)faad_getbits(ld, order_bits);
            if (tns->order[w][filt])
            {
                tns->direction[w][filt] = faad_get1bit(ld);
                tns->coef_compress[w][filt] = faad_get1bit(ld);

                coef_bits = start_coef_bits - tns->coef_compress[w][filt];
                for (i = 0; i < tns->order[w][filt]; i++)
                {
                    tns->coef[w][filt][i] = (uint8_t)faad_getbits(ld, coef_bits);
                }
            }
        }
    }
}


/* Table 4.4.29 */
static uint8_t spectral_data(NeAACDecStruct *hDecoder, ic_stream *ics, bitfile *ld,
                             int16_t *spectral_data)
{
    int8_t i;
    uint8_t g;
    uint16_t inc, k, p = 0;
    uint8_t groups = 0;
    uint8_t sect_cb;
    uint8_t result;
    uint16_t nshort = hDecoder->frameLength/8;


    for(g = 0; g < ics->num_window_groups; g++)
    {
        p = groups*nshort;

        for (i = 0; i < ics->num_sec[g]; i++)
        {
            sect_cb = ics->sect_cb[g][i];

            inc = (sect_cb >= FIRST_PAIR_HCB) ? 2 : 4;

            switch (sect_cb)
            {
            case ZERO_HCB:
            case NOISE_HCB:
            case INTENSITY_HCB:
            case INTENSITY_HCB2:
//#define SD_PRINT
//#define SFBO_PRINT
                p += (ics->sect_sfb_offset[g][ics->sect_end[g][i]] -
                    ics->sect_sfb_offset[g][ics->sect_start[g][i]]);
                break;
            default:
                for (k = ics->sect_sfb_offset[g][ics->sect_start[g][i]];
                     k < ics->sect_sfb_offset[g][ics->sect_end[g][i]]; k += inc)
                {
                    if ((result = huffman_spectral_data(sect_cb, ld, &spectral_data[p])) > 0)
                        return result;
                    p += inc;
                }
                break;
            }
        }
        groups += ics->window_group_length[g];
    }


    return 0;
}

/* OFFgrid-SDR: compiled in DRM builds too (see fill_element) */
/* Table 4.4.30 */
static uint16_t extension_payload(bitfile *ld, drc_info *drc, uint16_t count)
{
    uint16_t i, n, dataElementLength;
    uint8_t dataElementLengthPart;
    uint8_t align = 4, data_element_version, loopCounter;

    uint8_t extension_type = (uint8_t)faad_getbits(ld, 4);

    switch (extension_type)
    {
    case EXT_DYNAMIC_RANGE:
        drc->present = 1;
        drc->seen = 1;   /* OFFgrid-SDR: see raw_data_block */
        n = dynamic_range_info(ld, drc);
        return n;
    case EXT_FILL_DATA:
        /* fill_nibble = */ faad_getbits(ld, 4); /* must be '0000' */
        for (i = 0; i < count-1; i++)
        {
            /* fill_byte[i] = */ faad_getbits(ld, 8); /* must be '10100101' */
        }
        return count;
    case EXT_DATA_ELEMENT:
        data_element_version = (uint8_t)faad_getbits(ld, 4);
        switch (data_element_version)
        {
        case ANC_DATA:
            loopCounter = 0;
            dataElementLength = 0;
            do {
                dataElementLengthPart = (uint8_t)faad_getbits(ld, 8);
                dataElementLength += dataElementLengthPart;
                loopCounter++;
            } while (dataElementLengthPart == 255);

            for (i = 0; i < dataElementLength; i++)
            {
                /* data_element_byte[i] = */ faad_getbits(ld, 8);
                return (dataElementLength+loopCounter+1);
            }
            /* fall through */
        default:
            align = 0;
        }
        /* fall through */
    case EXT_FIL:
    default:
        faad_getbits(ld, align);
        for (i = 0; i < count-1; i++)
        {
            /* other_bits[i] = */ faad_getbits(ld, 8);
        }
        return count;
    }
}

/* Table 4.4.31 */
static uint8_t dynamic_range_info(bitfile *ld, drc_info *drc)
{
    uint8_t i, n = 1;
    uint8_t band_incr;

    drc->num_bands = 1;

    if (faad_get1bit(ld) & 1)
    {
        drc->pce_instance_tag = (uint8_t)faad_getbits(ld, 4);
        /* drc->drc_tag_reserved_bits = */ faad_getbits(ld, 4);
        n++;
    }

    drc->excluded_chns_present = faad_get1bit(ld);
    if (drc->excluded_chns_present == 1)
    {
        n += excluded_channels(ld, drc);
    }

    if (faad_get1bit(ld) & 1)
    {
        band_incr = (uint8_t)faad_getbits(ld, 4);
        /* drc->drc_bands_reserved_bits = */ faad_getbits(ld, 4);
        n++;
        drc->num_bands += band_incr;

        for (i = 0; i < drc->num_bands; i++)
        {
            drc->band_top[i] = (uint8_t)faad_getbits(ld, 8);
            n++;
        }
    }

    if (faad_get1bit(ld) & 1)
    {
        drc->prog_ref_level = (uint8_t)faad_getbits(ld, 7);
        /* drc->prog_ref_level_reserved_bits = */ faad_get1bit(ld);
        n++;
    }

    for (i = 0; i < drc->num_bands; i++)
    {
        drc->dyn_rng_sgn[i] = faad_get1bit(ld);
        drc->dyn_rng_ctl[i] = (uint8_t)faad_getbits(ld, 7);
        n++;
    }

    return n;
}

/* Table 4.4.32 */
static uint8_t excluded_channels(bitfile *ld, drc_info *drc)
{
    uint8_t i, n = 0;
    uint8_t num_excl_chan = 7;

    for (i = 0; i < 7; i++)
    {
        drc->exclude_mask[i] = faad_get1bit(ld);
    }
    n++;

    while ((drc->additional_excluded_chns[n-1] = faad_get1bit(ld)) == 1)
    {
        if (i >= MAX_CHANNELS - num_excl_chan - 7)
            return n;
        for (i = num_excl_chan; i < num_excl_chan+7; i++)
        {
            drc->exclude_mask[i] = faad_get1bit(ld);
        }
        n++;
        num_excl_chan += 7;
    }

    return n;
}

/* Annex A: Audio Interchange Formats */

/* Table 1.A.2 */
void get_adif_header(adif_header *adif, bitfile *ld)
{
    uint8_t i;

    /* adif_id[0] = */ faad_getbits(ld, 8);
    /* adif_id[1] = */ faad_getbits(ld, 8);
    /* adif_id[2] = */ faad_getbits(ld, 8);
    /* adif_id[3] = */ faad_getbits(ld, 8);
    adif->copyright_id_present = faad_get1bit(ld);
    if(adif->copyright_id_present)
    {
        for (i = 0; i < 72/8; i++)
        {
            adif->copyright_id[i] = (int8_t)faad_getbits(ld, 8);
        }
        adif->copyright_id[i] = 0;
    }
    adif->original_copy  = faad_get1bit(ld);
    adif->home = faad_get1bit(ld);
    adif->bitstream_type = faad_get1bit(ld);
    adif->bitrate = faad_getbits(ld, 23);
    adif->num_program_config_elements = (uint8_t)faad_getbits(ld, 4);

    for (i = 0; i < adif->num_program_config_elements + 1; i++)
    {
        if(adif->bitstream_type == 0)
        {
            adif->adif_buffer_fullness = faad_getbits(ld, 20);
        } else {
            adif->adif_buffer_fullness = 0;
        }

        program_config_element(&adif->pce[i], ld);
    }
}

/* Table 1.A.5 */
uint8_t adts_frame(adts_header *adts, bitfile *ld)
{
    /* faad_byte_align(ld); */
    if (adts_fixed_header(adts, ld))
        return 5;
    adts_variable_header(adts, ld);
    adts_error_check(adts, ld);

    return 0;
}

/* Table 1.A.6 */
static uint8_t adts_fixed_header(adts_header *adts, bitfile *ld)
{
    uint16_t i;
    uint8_t sync_err = 1;

    /* try to recover from sync errors */
    for (i = 0; i < 768; i++)
    {
        adts->syncword = (uint16_t)faad_showbits(ld, 12);
        if (adts->syncword != 0xFFF)
        {
            faad_getbits(ld, 8);
        } else {
            sync_err = 0;
            faad_getbits(ld, 12);
            break;
        }
    }
    if (sync_err)
        return 5;

    adts->id = faad_get1bit(ld);
    adts->layer = (uint8_t)faad_getbits(ld, 2);
    adts->protection_absent = faad_get1bit(ld);
    adts->profile = (uint8_t)faad_getbits(ld, 2);
    adts->sf_index = (uint8_t)faad_getbits(ld, 4);
    adts->private_bit = faad_get1bit(ld);
    adts->channel_configuration = (uint8_t)faad_getbits(ld, 3);
    adts->original = faad_get1bit(ld);
    adts->home = faad_get1bit(ld);

    if (adts->old_format == 1)
    {
        /* Removed in corrigendum 14496-3:2002 */
        if (adts->id == 0)
        {
            adts->emphasis = (uint8_t)faad_getbits(ld, 2);
        }
    }

    return 0;
}

/* Table 1.A.7 */
static void adts_variable_header(adts_header *adts, bitfile *ld)
{
    adts->copyright_identification_bit = faad_get1bit(ld);
    adts->copyright_identification_start = faad_get1bit(ld);
    adts->aac_frame_length = (uint16_t)faad_getbits(ld, 13);
    adts->adts_buffer_fullness = (uint16_t)faad_getbits(ld, 11);
    adts->no_raw_data_blocks_in_frame = (uint8_t)faad_getbits(ld, 2);
}

/* Table 1.A.8 */
static void adts_error_check(adts_header *adts, bitfile *ld)
{
    if (adts->protection_absent == 0)
    {
        adts->crc_check = (uint16_t)faad_getbits(ld, 16);
    }
}

/* LATM parsing functions */

} // namespace faad
} // namespace codec
