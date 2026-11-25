// ============================================================================
//  offgrid-codecs - the decoder object (faad2 decoder.c) (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
//  Ported to C++11 from FAAD2 (GPL-2.0-or-later, commit a335040, as built by
//  OFFgrid-SDR: see licenses/FAAD2). FAAD2's own notice, kept below, applies
//  to the code taken from it; "Code from FAAD2 is copyright (c) Nero AG".
// ----------------------------------------------------------------------------
//  File:   src/codecs/AacCore.cpp
//  Layer:  Library liboffgrid-codecs (internal: the FAAD2 port)
//
//  What this file does
//  -------------------
//  Opening, configuring (AudioSpecificConfig, DRM), decoding one access unit
//  (raw, ADTS or DRM framing) into samples, and closing the decoder state.
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
** $Id: decoder.c,v 1.119 2015/01/24 14:21:05 knik Exp $
**/

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include "AacCommon.h"
#include "AacStructs.h"
#include "AacMp4.h"
#include "AacSyntax.h"
#include "AacError.h"
#include "AacOutput.h"
#include "AacFiltbank.h"
#include "AacDrc.h"
#include "AacSbrDec.h"
#include "AacSbrSyntax.h"

namespace codec {
namespace faad {


/* static function declarations */
static void* aac_frame_decode(NeAACDecStruct *hDecoder,
                              NeAACDecFrameInfo *hInfo,
                              unsigned char *buffer,
                              unsigned long buffer_size,
                              void **sample_buffer2,
                              unsigned long sample_buffer_size);
static void create_channel_config(NeAACDecStruct *hDecoder,
                                  NeAACDecFrameInfo *hInfo);



const char* NeAACDecGetErrorMessage(unsigned char errcode)
{
    if (errcode >= NUM_ERROR_MESSAGES)
        return NULL;
    return err_msg[errcode];
}

unsigned long NeAACDecGetCapabilities(void)
{
    uint32_t cap = 0;

    /* can't do without it */
    cap += LC_DEC_CAP;

    cap += ERROR_RESILIENCE_CAP;

    return cap;
}

const unsigned char mes[] = { 0x67,0x20,0x61,0x20,0x20,0x20,0x6f,0x20,0x72,0x20,0x65,0x20,0x6e,0x20,0x20,0x20,0x74,0x20,0x68,0x20,0x67,0x20,0x69,0x20,0x72,0x20,0x79,0x20,0x70,0x20,0x6f,0x20,0x63 };
NeAACDecHandle NeAACDecOpen(void)
{
    uint8_t i;
    NeAACDecStruct *hDecoder = NULL;

    if ((hDecoder = (NeAACDecStruct*)faad_malloc(sizeof(NeAACDecStruct))) == NULL)
        return NULL;

    memset(hDecoder, 0, sizeof(NeAACDecStruct));

    hDecoder->cmes = mes;
    hDecoder->config.outputFormat  = FAAD_FMT_16BIT;
    hDecoder->config.defObjectType = MAIN;
    hDecoder->config.defSampleRate = 44100; /* Default: 44.1kHz */
    hDecoder->config.downMatrix = 0;
    hDecoder->adts_header_present = 0;
    hDecoder->adif_header_present = 0;
    hDecoder->latm_header_present = 0;
    hDecoder->aacSectionDataResilienceFlag = 0;
    hDecoder->aacScalefactorDataResilienceFlag = 0;
    hDecoder->aacSpectralDataResilienceFlag = 0;
    hDecoder->frameLength = 1024;

    hDecoder->frame = 0;
    hDecoder->sample_buffer = NULL;

    // Same as (1, 1) after 1024 iterations; otherwise first values does not look random at all.
    hDecoder->rng1 = 0x2bb431ea;
    hDecoder->rng2 = 0x206155b7;

    for (i = 0; i < MAX_CHANNELS; i++)
    {
        hDecoder->element_id[i] = INVALID_ELEMENT_ID;
        hDecoder->window_shape_prev[i] = 0;
        hDecoder->time_out[i] = NULL;
        hDecoder->fb_intermed[i] = NULL;
    }

    for (i = 0; i < MAX_SYNTAX_ELEMENTS; i++)
    {
        hDecoder->sbr[i] = NULL;
    }

    hDecoder->drc = drc_init(REAL_CONST(1.0), REAL_CONST(1.0));
    hDecoder->ps_present_flag = -1;

    return hDecoder;
}

NeAACDecConfigurationPtr NeAACDecGetCurrentConfiguration(NeAACDecHandle hpDecoder)
{
    NeAACDecStruct* hDecoder = (NeAACDecStruct*)hpDecoder;
    if (hDecoder)
    {
        NeAACDecConfigurationPtr config = &(hDecoder->config);

        return config;
    }

    return NULL;
}

unsigned char NeAACDecSetConfiguration(NeAACDecHandle hpDecoder,
                                                   NeAACDecConfigurationPtr config)
{
    NeAACDecStruct* hDecoder = (NeAACDecStruct*)hpDecoder;
    if (hDecoder && config)
    {
        /* check if we can decode this object type */
        if (can_decode_ot(config->defObjectType) < 0)
            return 0;
        hDecoder->config.defObjectType = config->defObjectType;

        /* samplerate: anything but 0 should be possible */
        if (config->defSampleRate == 0)
            return 0;
        hDecoder->config.defSampleRate = config->defSampleRate;

        /* check output format */
        if ((config->outputFormat < 1) || (config->outputFormat > 5))
            return 0;
        hDecoder->config.outputFormat = config->outputFormat;

        if (config->downMatrix > 1)
            return 0;
        hDecoder->config.downMatrix = config->downMatrix;

        /* OK */
        return 1;
    }

    return 0;
}


long NeAACDecInit(NeAACDecHandle hpDecoder,
                              unsigned char *buffer,
                              unsigned long buffer_size,
                              unsigned long *samplerate,
                              unsigned char *channels)
{
    uint32_t bits = 0;
    bitfile ld;
    adif_header adif;
    adts_header adts;
    NeAACDecStruct* hDecoder = (NeAACDecStruct*)hpDecoder;


    if ((hDecoder == NULL) || (samplerate == NULL) || (channels == NULL) || (buffer_size == 0))
        return -1;

    adts.old_format = hDecoder->config.useOldADTSFormat;
    hDecoder->sf_index = get_sr_index(hDecoder->config.defSampleRate);
    hDecoder->object_type = hDecoder->config.defObjectType;
    *samplerate = get_sample_rate(hDecoder->sf_index);
    *channels = 1;

    if (buffer != NULL)
    {

        faad_initbits(&ld, buffer, buffer_size);

        /* Check if an ADIF header is present */
        if ((buffer_size >= 8) && (buffer[0] == 'A') && (buffer[1] == 'D') &&
            (buffer[2] == 'I') && (buffer[3] == 'F'))
        {
            hDecoder->adif_header_present = 1;

            get_adif_header(&adif, &ld);
            faad_byte_align(&ld);

            hDecoder->sf_index = adif.pce[0].sf_index;
            hDecoder->object_type = adif.pce[0].object_type + 1;

            *samplerate = get_sample_rate(hDecoder->sf_index);
            *channels = adif.pce[0].channels;

            memcpy(&(hDecoder->pce), &(adif.pce[0]), sizeof(program_config));
            hDecoder->pce_set = 1;

            bits = bit2byte(faad_get_processed_bits(&ld));

        /* Check if an ADTS header is present */
        } else if (adts_frame(&adts, &ld) == 0) {
            hDecoder->adts_header_present = 1;

            hDecoder->sf_index = adts.sf_index;
            hDecoder->object_type = adts.profile + 1;

            *samplerate = get_sample_rate(hDecoder->sf_index);
            *channels = (adts.channel_configuration > 6) ?
                2 : adts.channel_configuration;
        }

        if (ld.error)
        {
            faad_endbits(&ld);
            return -1;
        }
        faad_endbits(&ld);
    }

    if (!*samplerate)
        return -1;

    hDecoder->channelConfiguration = *channels;

    /* implicit signalling */
    if (*samplerate <= 24000 && (hDecoder->config.dontUpSampleImplicitSBR == 0))
    {
        *samplerate *= 2;
        hDecoder->forceUpSampling = 1;
    } else if (*samplerate > 24000 && (hDecoder->config.dontUpSampleImplicitSBR == 0)) {
        hDecoder->downSampledSBR = 1;
    }

    if (can_decode_ot(hDecoder->object_type) < 0)
        return -1;

    hDecoder->fb = filter_bank_init(hDecoder->frameLength);


    return bits;
}

/* Init the library using a DecoderSpecificInfo */
char NeAACDecInit2(NeAACDecHandle hpDecoder,
                               unsigned char *pBuffer,
                               unsigned long SizeOfDecoderSpecificInfo,
                               unsigned long *samplerate,
                               unsigned char *channels)
{
    NeAACDecStruct* hDecoder = (NeAACDecStruct*)hpDecoder;
    int8_t rc;
    mp4AudioSpecificConfig mp4ASC;

    if((hDecoder == NULL)
        || (pBuffer == NULL)
        || (SizeOfDecoderSpecificInfo < 2)
        || (samplerate == NULL)
        || (channels == NULL))
    {
        return -1;
    }

    hDecoder->adif_header_present = 0;
    hDecoder->adts_header_present = 0;

    /* decode the audio specific config */
    rc = AudioSpecificConfig2(pBuffer, SizeOfDecoderSpecificInfo, &mp4ASC,
        &(hDecoder->pce), hDecoder->latm_header_present);

    /* copy the relevant info to the decoder handle */
    *samplerate = mp4ASC.samplingFrequency;
    if (mp4ASC.channelsConfiguration)
    {
        *channels = mp4ASC.channelsConfiguration;
    } else {
        *channels = hDecoder->pce.channels;
        hDecoder->pce_set = 1;
    }
    hDecoder->ps_present_flag = mp4ASC.ps_present_flag;
    hDecoder->sf_index = mp4ASC.samplingFrequencyIndex;
    hDecoder->object_type = mp4ASC.objectTypeIndex;
    hDecoder->aacSectionDataResilienceFlag = mp4ASC.aacSectionDataResilienceFlag;
    hDecoder->aacScalefactorDataResilienceFlag = mp4ASC.aacScalefactorDataResilienceFlag;
    hDecoder->aacSpectralDataResilienceFlag = mp4ASC.aacSpectralDataResilienceFlag;
    hDecoder->sbr_present_flag = mp4ASC.sbr_present_flag;
    hDecoder->downSampledSBR = mp4ASC.downSampledSBR;
    if (hDecoder->config.dontUpSampleImplicitSBR == 0)
        hDecoder->forceUpSampling = mp4ASC.forceUpSampling;
    else
        hDecoder->forceUpSampling = 0;

    /* AAC core decoder samplerate is 2 times as low */
    if (((hDecoder->sbr_present_flag == 1)&&(!hDecoder->downSampledSBR)) || hDecoder->forceUpSampling == 1)
    {
        hDecoder->sf_index = get_sr_index(mp4ASC.samplingFrequency / 2);
    }

    if (rc != 0)
    {
        return rc;
    }
    hDecoder->channelConfiguration = mp4ASC.channelsConfiguration;
    if (mp4ASC.frameLengthFlag)
        hDecoder->frameLength = 960;

    hDecoder->fb = filter_bank_init(hDecoder->frameLength);


    return 0;
}

char NeAACDecInitDRM(NeAACDecHandle *hpDecoder,
                                 unsigned long samplerate,
                                 unsigned char channels)
{
    NeAACDecStruct** hDecoder = (NeAACDecStruct**)hpDecoder;
    if (hDecoder == NULL)
        return 1; /* error */

    NeAACDecClose(*hDecoder);

    *hDecoder = (NeAACDecStruct*)NeAACDecOpen();

    /* Special object type defined for DRM */
    (*hDecoder)->config.defObjectType = DRM_ER_LC;

    (*hDecoder)->config.defSampleRate = samplerate;
    (*hDecoder)->aacSectionDataResilienceFlag = 1; /* VCB11 */
    (*hDecoder)->aacScalefactorDataResilienceFlag = 0; /* no RVLC */
    (*hDecoder)->aacSpectralDataResilienceFlag = 1; /* HCR */
    (*hDecoder)->frameLength = 960;
    (*hDecoder)->sf_index = get_sr_index((*hDecoder)->config.defSampleRate);
    (*hDecoder)->object_type = (*hDecoder)->config.defObjectType;

    if ((channels == DRMCH_STEREO) || (channels == DRMCH_SBR_STEREO))
        (*hDecoder)->channelConfiguration = 2;
    else
        (*hDecoder)->channelConfiguration = 1;

    if ((channels == DRMCH_MONO) || (channels == DRMCH_STEREO))
        (*hDecoder)->sbr_present_flag = 0;
    else
        (*hDecoder)->sbr_present_flag = 1;

    (*hDecoder)->fb = filter_bank_init((*hDecoder)->frameLength);

    return 0;
}

void NeAACDecClose(NeAACDecHandle hpDecoder)
{
    uint8_t i;
    NeAACDecStruct* hDecoder = (NeAACDecStruct*)hpDecoder;

    if (hDecoder == NULL)
        return;


    for (i = 0; i < MAX_CHANNELS; i++)
    {
        if (hDecoder->time_out[i]) faad_free(hDecoder->time_out[i]);
        if (hDecoder->fb_intermed[i]) faad_free(hDecoder->fb_intermed[i]);
    }

        filter_bank_end(hDecoder->fb);

    drc_end(hDecoder->drc);

    if (hDecoder->sample_buffer) faad_free(hDecoder->sample_buffer);

    for (i = 0; i < MAX_SYNTAX_ELEMENTS; i++)
    {
        if (hDecoder->sbr[i])
            sbrDecodeEnd(hDecoder->sbr[i]);
    }

    if (hDecoder) faad_free(hDecoder);
}

void NeAACDecPostSeekReset(NeAACDecHandle hpDecoder, long frame)
{
    NeAACDecStruct* hDecoder = (NeAACDecStruct*)hpDecoder;
    if (hDecoder)
    {
        hDecoder->postSeekResetFlag = 1;

        if (frame != -1)
            hDecoder->frame = frame;
    }
}

static void create_channel_config(NeAACDecStruct *hDecoder, NeAACDecFrameInfo *hInfo)
{
    hInfo->num_front_channels = 0;
    hInfo->num_side_channels = 0;
    hInfo->num_back_channels = 0;
    hInfo->num_lfe_channels = 0;
    memset(hInfo->channel_position, 0, MAX_CHANNELS*sizeof(uint8_t));

    if (hDecoder->downMatrix)
    {
        hInfo->num_front_channels = 2;
        hInfo->channel_position[0] = FRONT_CHANNEL_LEFT;
        hInfo->channel_position[1] = FRONT_CHANNEL_RIGHT;
        return;
    }

    /* check if there is a PCE */
    /* TODO: why CPE flag is ignored? */
    if (hDecoder->pce_set)
    {
        uint8_t i, chpos = 0;
        uint8_t chdir, back_center = 0;

        hInfo->num_front_channels = hDecoder->pce.num_front_channels;
        hInfo->num_side_channels = hDecoder->pce.num_side_channels;
        hInfo->num_back_channels = hDecoder->pce.num_back_channels;
        hInfo->num_lfe_channels = hDecoder->pce.num_lfe_channels;
        chdir = hInfo->num_front_channels;
        if (chdir & 1)
        {
            if (hInfo->num_front_channels == 1 &&
                hInfo->num_side_channels == 0 &&
                hInfo->num_back_channels == 0 &&
                hInfo->num_lfe_channels == 0 &&
                hDecoder->ps_used_global)
            {
                /* When PS is enabled output is always stereo */
                hInfo->channel_position[chpos++] = FRONT_CHANNEL_LEFT;
                hInfo->channel_position[chpos++] = FRONT_CHANNEL_RIGHT;
            } else
            hInfo->channel_position[chpos++] = FRONT_CHANNEL_CENTER;
            chdir--;
        }
        for (i = 0; i < chdir; i++)
        {
            hInfo->channel_position[chpos++] =
                (i & 1) ? FRONT_CHANNEL_RIGHT : FRONT_CHANNEL_LEFT;
        }

        for (i = 0; i < hInfo->num_side_channels; i++)
        {
            hInfo->channel_position[chpos++] =
                (i & 1) ? SIDE_CHANNEL_RIGHT : SIDE_CHANNEL_LEFT;
        }

        chdir = hInfo->num_back_channels;
        if (chdir & 1)
        {
            back_center = 1;
            chdir--;
        }
        for (i = 0; i < chdir; i++)
        {
            hInfo->channel_position[chpos++] =
                (i & 1) ? BACK_CHANNEL_RIGHT : BACK_CHANNEL_LEFT;
        }
        if (back_center)
        {
            hInfo->channel_position[chpos++] = BACK_CHANNEL_CENTER;
        }

        for (i = 0; i < hInfo->num_lfe_channels; i++)
        {
            hInfo->channel_position[chpos++] = LFE_CHANNEL;
        }

    } else {
        switch (hDecoder->channelConfiguration)
        {
        case 1:
            if (hDecoder->ps_used_global)
            {
                /* When PS is enabled output is always stereo */
                hInfo->num_front_channels = 2;
                hInfo->channel_position[0] = FRONT_CHANNEL_LEFT;
                hInfo->channel_position[1] = FRONT_CHANNEL_RIGHT;
            } else {
                hInfo->num_front_channels = 1;
                hInfo->channel_position[0] = FRONT_CHANNEL_CENTER;
            }
            break;
        case 2:
            hInfo->num_front_channels = 2;
            hInfo->channel_position[0] = FRONT_CHANNEL_LEFT;
            hInfo->channel_position[1] = FRONT_CHANNEL_RIGHT;
            break;
        case 3:
            hInfo->num_front_channels = 3;
            hInfo->channel_position[0] = FRONT_CHANNEL_CENTER;
            hInfo->channel_position[1] = FRONT_CHANNEL_LEFT;
            hInfo->channel_position[2] = FRONT_CHANNEL_RIGHT;
            break;
        case 4:
            hInfo->num_front_channels = 3;
            hInfo->num_back_channels = 1;
            hInfo->channel_position[0] = FRONT_CHANNEL_CENTER;
            hInfo->channel_position[1] = FRONT_CHANNEL_LEFT;
            hInfo->channel_position[2] = FRONT_CHANNEL_RIGHT;
            hInfo->channel_position[3] = BACK_CHANNEL_CENTER;
            break;
        case 5:
            hInfo->num_front_channels = 3;
            hInfo->num_back_channels = 2;
            hInfo->channel_position[0] = FRONT_CHANNEL_CENTER;
            hInfo->channel_position[1] = FRONT_CHANNEL_LEFT;
            hInfo->channel_position[2] = FRONT_CHANNEL_RIGHT;
            hInfo->channel_position[3] = BACK_CHANNEL_LEFT;
            hInfo->channel_position[4] = BACK_CHANNEL_RIGHT;
            break;
        case 6:
            hInfo->num_front_channels = 3;
            hInfo->num_back_channels = 2;
            hInfo->num_lfe_channels = 1;
            hInfo->channel_position[0] = FRONT_CHANNEL_CENTER;
            hInfo->channel_position[1] = FRONT_CHANNEL_LEFT;
            hInfo->channel_position[2] = FRONT_CHANNEL_RIGHT;
            hInfo->channel_position[3] = BACK_CHANNEL_LEFT;
            hInfo->channel_position[4] = BACK_CHANNEL_RIGHT;
            hInfo->channel_position[5] = LFE_CHANNEL;
            break;
        case 7:
            hInfo->num_front_channels = 3;
            hInfo->num_side_channels = 2;
            hInfo->num_back_channels = 2;
            hInfo->num_lfe_channels = 1;
            hInfo->channel_position[0] = FRONT_CHANNEL_CENTER;
            hInfo->channel_position[1] = FRONT_CHANNEL_LEFT;
            hInfo->channel_position[2] = FRONT_CHANNEL_RIGHT;
            hInfo->channel_position[3] = SIDE_CHANNEL_LEFT;
            hInfo->channel_position[4] = SIDE_CHANNEL_RIGHT;
            hInfo->channel_position[5] = BACK_CHANNEL_LEFT;
            hInfo->channel_position[6] = BACK_CHANNEL_RIGHT;
            hInfo->channel_position[7] = LFE_CHANNEL;
            break;
        default: /* channelConfiguration == 0 || channelConfiguration > 7 */
            {
                uint8_t i;
                uint8_t ch = hDecoder->fr_channels - hDecoder->has_lfe;
                if (ch & 1) /* there's either a center front or a center back channel */
                {
                    uint8_t ch1 = (ch-1)/2;
                    if (hDecoder->first_syn_ele == ID_SCE)
                    {
                        hInfo->num_front_channels = ch1 + 1;
                        hInfo->num_back_channels = ch1;
                        hInfo->channel_position[0] = FRONT_CHANNEL_CENTER;
                        for (i = 1; i <= ch1; i+=2)
                        {
                            hInfo->channel_position[i] = FRONT_CHANNEL_LEFT;
                            hInfo->channel_position[i+1] = FRONT_CHANNEL_RIGHT;
                        }
                        for (i = ch1+1; i < ch; i+=2)
                        {
                            hInfo->channel_position[i] = BACK_CHANNEL_LEFT;
                            hInfo->channel_position[i+1] = BACK_CHANNEL_RIGHT;
                        }
                    } else {
                        hInfo->num_front_channels = ch1;
                        hInfo->num_back_channels = ch1 + 1;
                        for (i = 0; i < ch1; i+=2)
                        {
                            hInfo->channel_position[i] = FRONT_CHANNEL_LEFT;
                            hInfo->channel_position[i+1] = FRONT_CHANNEL_RIGHT;
                        }
                        for (i = ch1; i < ch-1; i+=2)
                        {
                            hInfo->channel_position[i] = BACK_CHANNEL_LEFT;
                            hInfo->channel_position[i+1] = BACK_CHANNEL_RIGHT;
                        }
                        hInfo->channel_position[ch-1] = BACK_CHANNEL_CENTER;
                    }
                } else {
                    uint8_t ch1 = (ch)/2;
                    hInfo->num_front_channels = ch1;
                    hInfo->num_back_channels = ch1;
                    if (ch1 & 1)
                    {
                        hInfo->channel_position[0] = FRONT_CHANNEL_CENTER;
                        for (i = 1; i <= ch1; i+=2)
                        {
                            hInfo->channel_position[i] = FRONT_CHANNEL_LEFT;
                            hInfo->channel_position[i+1] = FRONT_CHANNEL_RIGHT;
                        }
                        for (i = ch1+1; i < ch-1; i+=2)
                        {
                            hInfo->channel_position[i] = BACK_CHANNEL_LEFT;
                            hInfo->channel_position[i+1] = BACK_CHANNEL_RIGHT;
                        }
                        hInfo->channel_position[ch-1] = BACK_CHANNEL_CENTER;
                    } else {
                        for (i = 0; i < ch1; i+=2)
                        {
                            hInfo->channel_position[i] = FRONT_CHANNEL_LEFT;
                            hInfo->channel_position[i+1] = FRONT_CHANNEL_RIGHT;
                        }
                        for (i = ch1; i < ch; i+=2)
                        {
                            hInfo->channel_position[i] = BACK_CHANNEL_LEFT;
                            hInfo->channel_position[i+1] = BACK_CHANNEL_RIGHT;
                        }
                    }
                }
                hInfo->num_lfe_channels = hDecoder->has_lfe;
                for (i = ch; i < hDecoder->fr_channels; i++)
                {
                    hInfo->channel_position[i] = LFE_CHANNEL;
                }
            }
            break;
        }
    }
}

void* NeAACDecDecode(NeAACDecHandle hpDecoder,
                                 NeAACDecFrameInfo *hInfo,
                                 unsigned char *buffer,
                                 unsigned long buffer_size)
{
    NeAACDecStruct* hDecoder = (NeAACDecStruct*)hpDecoder;
    return aac_frame_decode(hDecoder, hInfo, buffer, buffer_size, NULL, 0);
}

void* NeAACDecDecode2(NeAACDecHandle hpDecoder,
                                  NeAACDecFrameInfo *hInfo,
                                  unsigned char *buffer,
                                  unsigned long buffer_size,
                                  void **sample_buffer,
                                  unsigned long sample_buffer_size)
{
    NeAACDecStruct* hDecoder = (NeAACDecStruct*)hpDecoder;
    if ((sample_buffer_size == 0) || (sample_buffer == NULL) || (*sample_buffer == NULL))
    {
        hInfo->error = 27;
        return NULL;
    }

    return aac_frame_decode(hDecoder, hInfo, buffer, buffer_size,
        sample_buffer, sample_buffer_size);
}

static void* aac_frame_decode(NeAACDecStruct *hDecoder,
                              NeAACDecFrameInfo *hInfo,
                              unsigned char *buffer,
                              unsigned long buffer_size,
                              void **sample_buffer2,
                              unsigned long sample_buffer_size)
{
    uint16_t i;
    uint8_t channels = 0;
    uint8_t output_channels = 0;
    bitfile ld;
    uint32_t bitsconsumed;
    uint16_t frame_len;
    void *sample_buffer;
    uint32_t required_buffer_size=0;


    /* safety checks */
    if ((hDecoder == NULL) || (hInfo == NULL) || (buffer == NULL))
    {
        return NULL;
    }


    frame_len = hDecoder->frameLength;


    memset(hInfo, 0, sizeof(NeAACDecFrameInfo));
    memset(hDecoder->internal_channel, 0, MAX_CHANNELS*sizeof(hDecoder->internal_channel[0]));


    /* check for some common metadata tag types in the bitstream
     * No need to return an error
     */
    /* ID3 */
    if (buffer_size >= 128)
    {
        if (memcmp(buffer, "TAG", 3) == 0)
        {
            /* found it */
            hInfo->bytesconsumed = 128; /* 128 bytes fixed size */
            /* no error, but no output either */
            return NULL;
        }
    }


    /* initialize the bitstream */
    faad_initbits(&ld, buffer, buffer_size);
    if (ld.error != 0)
        return NULL;


    if (hDecoder->object_type == DRM_ER_LC)
    {
        /* We do not support stereo right now */
        if (0) //(hDecoder->channelConfiguration == 2)
        {
            hInfo->error = 28; // Throw CRC error
            goto error;
        }

        faad_getbits(&ld, 8);
    }

    if (hDecoder->adts_header_present)
    {
        adts_header adts;

        adts.old_format = hDecoder->config.useOldADTSFormat;
        if ((hInfo->error = adts_frame(&adts, &ld)) > 0)
            goto error;

        /* MPEG2 does byte_alignment() here,
         * but ADTS header is always multiple of 8 bits in MPEG2
         * so not needed to actually do it.
         */
    }


    /* decode the complete bitstream */
    if (/*(hDecoder->object_type == 6) ||*/ hDecoder->object_type == DRM_ER_LC)
    {
        DRM_aac_scalable_main_element(hDecoder, hInfo, &ld, &hDecoder->pce, hDecoder->drc);
    } else {
        raw_data_block(hDecoder, hInfo, &ld, &hDecoder->pce, hDecoder->drc);
    }


    channels = hDecoder->fr_channels;

    if (hInfo->error > 0)
        goto error;

    /* safety check */
    if (channels == 0 || channels > MAX_CHANNELS)
    {
        /* invalid number of channels */
        hInfo->error = 12;
        goto error;
    }

    /* no more bit reading after this */
    bitsconsumed = faad_get_processed_bits(&ld);
    hInfo->bytesconsumed = bit2byte(bitsconsumed);
    if (ld.error)
    {
        hInfo->error = 14;
        goto error;
    }
    faad_endbits(&ld);


    if (!hDecoder->adts_header_present && !hDecoder->adif_header_present
        )
    {
        if (hDecoder->channelConfiguration == 0)
            hDecoder->channelConfiguration = channels;

        if (channels == 8) /* 7.1 */
            hDecoder->channelConfiguration = 7;
        if (channels == 7) /* not a standard channelConfiguration */
            hDecoder->channelConfiguration = 0;
    }

    if ((channels == 5 || channels == 6) && hDecoder->config.downMatrix)
    {
        hDecoder->downMatrix = 1;
        output_channels = 2;
    } else {
        output_channels = channels;
    }

    hDecoder->upMatrix = 0;

    /* Make a channel configuration based on either a PCE or a channelConfiguration */
    if (!hDecoder->downMatrix && hDecoder->pce_set)
    {
        /* In some codepath program_config_element result is ignored. */
        if (hDecoder->pce.channels > MAX_CHANNELS)
        {
            hInfo->error = 22;
            return NULL;
        }
    }
    create_channel_config(hDecoder, hInfo);

    /* number of samples in this frame */
    hInfo->samples = frame_len*output_channels;
    /* number of channels in this frame */
    hInfo->channels = output_channels;
    /* samplerate */
    hInfo->samplerate = get_sample_rate(hDecoder->sf_index);
    /* object type */
    hInfo->object_type = hDecoder->object_type;
    /* sbr */
    hInfo->sbr = NO_SBR;
    /* header type */
    hInfo->header_type = RAW;
    if (hDecoder->adif_header_present)
        hInfo->header_type = ADIF;
    if (hDecoder->adts_header_present)
        hInfo->header_type = ADTS;
    hInfo->ps = hDecoder->ps_used_global;

    /* check if frame has channel elements */
    if (channels == 0)
    {
        hDecoder->frame++;
        return NULL;
    }

    {
        static const uint8_t str[] = { sizeof(int16_t), sizeof(int32_t), sizeof(int32_t),
            sizeof(float32_t), sizeof(double), sizeof(int16_t), sizeof(int16_t),
            sizeof(int16_t), sizeof(int16_t), 0, 0, 0
        };
        uint8_t stride = str[hDecoder->config.outputFormat-1];
        if (((hDecoder->sbr_present_flag == 1)&&(!hDecoder->downSampledSBR)) || (hDecoder->forceUpSampling == 1))
        {
            stride = 2 * stride;
        }
        required_buffer_size = frame_len*output_channels*stride;
    }

    /* check if we want to use internal sample_buffer */
    if (sample_buffer_size == 0)
    {
        /* allocate the buffer for the final samples */
        if (hDecoder->sample_buffer_size != required_buffer_size) 
        {
            if (hDecoder->sample_buffer)
                faad_free(hDecoder->sample_buffer);
            hDecoder->sample_buffer = NULL;
            hDecoder->sample_buffer = faad_malloc(required_buffer_size);
            hDecoder->sample_buffer_size = required_buffer_size;
        }
    } else if (sample_buffer_size < required_buffer_size) {
        /* provided sample buffer is not big enough */
        hInfo->error = 27;
        return NULL;
    }

    if (sample_buffer_size == 0)
    {
        sample_buffer = hDecoder->sample_buffer;
    } else {
        sample_buffer = *sample_buffer2;
    }

    if ((hDecoder->sbr_present_flag == 1) || (hDecoder->forceUpSampling == 1))
    {
        uint8_t ele;

        /* this data is different when SBR is used or when the data is upsampled */
        if (!hDecoder->downSampledSBR)
        {
            frame_len *= 2;
            hInfo->samples *= 2;
            hInfo->samplerate *= 2;
        }

        /* check if every element was provided with SBR data */
        for (ele = 0; ele < hDecoder->fr_ch_ele; ele++)
        {
            if (hDecoder->sbr[ele] == NULL)
            {
                hInfo->error = 25;
                goto error;
            }
        }

        /* sbr */
        if (hDecoder->sbr_present_flag == 1)
        {
            hInfo->object_type = HE_AAC;
            hInfo->sbr = SBR_UPSAMPLED;
        } else {
            hInfo->sbr = NO_SBR_UPSAMPLED;
        }
        if (hDecoder->downSampledSBR)
        {
            hInfo->sbr = SBR_DOWNSAMPLED;
        }
    }


    sample_buffer = output_to_PCM(hDecoder, hDecoder->time_out, sample_buffer,
        output_channels, frame_len, hDecoder->config.outputFormat);


    //conceal_output(hDecoder, frame_len, output_channels, sample_buffer);


    hDecoder->postSeekResetFlag = 0;

    hDecoder->frame++;
        if (hDecoder->frame <= 1)
            hInfo->samples = 0;

    /* cleanup */


    return sample_buffer;

error:

    /* reset filterbank state */
    for (i = 0; i < MAX_CHANNELS; i++)
    {
        if (hDecoder->fb_intermed[i] != NULL)
        {
            memset(hDecoder->fb_intermed[i], 0, hDecoder->frameLength*sizeof(real_t));
        }
    }
    for (i = 0; i < MAX_SYNTAX_ELEMENTS; i++)
    {
        if (hDecoder->sbr[i] != NULL)
        {
            sbrReset(hDecoder->sbr[i]);
        }
    }


    faad_endbits(&ld);

    /* cleanup */

    return NULL;
}

} // namespace faad
} // namespace codec
