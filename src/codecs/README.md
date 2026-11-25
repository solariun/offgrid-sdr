# liboffgrid-codecs — the audio decoders of digital radio, in C++11

The sound of DAB, DAB+ and DRM: an AAC decoder (AAC-LC, HE-AAC with SBR, HE-AAC v2 with
parametric stereo, 960- and 1024-sample frames, DRM's error-resilient AAC) and an MPEG Audio
Layer II decoder (classic DAB). Compressed units in, interleaved float PCM out. No threads, no
devices, no external libraries, no global state: the C++11 standard library only.

| Decoder | Header | Class | What it decodes |
|---|---|---|---|
| AAC | `Aac.h` | `codec::AacDecoder` | DAB+ AUs, DRM audio frames, raw AUs with an AudioSpecificConfig, ADTS frames |
| MP2 | `Mp2.h` | `codec::Mp2Decoder` | MPEG-1 Layer II (48/44.1/32 kHz) and MPEG-2 half rates (24/22.05/16 kHz), mono, stereo, joint stereo, dual channel; plus each frame's ancillary bytes (DAB's X-PAD and F-PAD) |

The AAC decoder is a port of [FAAD2](https://github.com/knik0/faad2) (GPL-2.0-or-later; M. Bakker,
Nero AG, Ahead Software AG and contributors) at the commit OFFgrid-SDR's browser version embeds
(`licenses/FAAD2`), configured the same way (`DRM_SUPPORT`, `APPLY_DRC`, floating point) and with
OFFgrid-SDR's DAB+ fill-element change. The MP2 decoder is a port of
[pl_mpeg](https://github.com/phoboslab/pl_mpeg)'s (MIT; Dominic Szablewski, after kjmp2 by
Martin J. Fiedler) with OFFgrid-SDR's DAB changes (`licenses/pi_mpeg`). Both produce
**sample-for-sample the same output as the originals** compiled as C (see the tests).

## Use

AAC, as DAB+ signals it (the super frame header: `dac_rate`, `sbr_flag`, `aac_channel_mode`, `ps_flag`):

```cpp
#include "codecs/Aac.h"

codec::AacDecoder aac;
aac.init(codec::AacParams::dabPlus(48000, /*sbr*/ true, /*stereo*/ false, /*ps*/ true));
std::vector<float> pcm;
if (aac.decode(au, auLen, pcm))          // one AU, without its CRC
    play(pcm.data(), pcm.size() / aac.channels(), aac.sampleRate());
else
    std::printf("bad AU: %s\n", aac.lastErrorMessage());
```

DRM (from the SDC's audio information; each audio frame with its CRC byte in front):

```cpp
aac.init(codec::AacParams::drmAudio(24000, /*sbr*/ true, /*stereo*/ false, /*ps*/ true));
aac.decode(crcAndFrame, len, pcm);
```

Or from an MPEG-4 AudioSpecificConfig (`aac.init(ascBytes, ascLen)`), an ADTS stream
(`aac.initAdts(firstFrame, len)`, then whole ADTS frames to `decode`), or explicit parameters
(`codec::AacParams` fields: AAC sampling rate *without* SBR's doubling, coded channels, SBR, PS,
frame length 960/1024, DRM).

MP2, one whole frame per call (DAB: a 48 kHz frame is one CIF's sub-channel bytes; a 24 kHz frame
is two CIFs joined):

```cpp
#include "codecs/Mp2.h"

codec::Mp2Decoder mp2;
if (mp2.decode(frame, len, pcm)) {                      // 1152 stereo pairs
    play(pcm.data(), 1152, mp2.sampleRate());
    const codec::Mp2Decoder::DabPad &pad = mp2.dabPad(); // F-PAD, and where X-PAD ends
    readPad(frame, pad.xpadEnd, pad.fpad[0], pad.fpad[1]);
}
```

## What comes out

- **AAC**: interleaved float, 16-bit full scale = 1.0 (FAAD2's float output), always **2 channels**:
  a mono stream comes out as two identical channels, as FAAD2 with parametric stereo does (PS can
  appear at any frame). The rate is the AAC rate, doubled with SBR. The first AU after `init` (and
  after a rebuild, below) gives no samples: FAAD2's start-up delay. 960-sample frames give 960
  (1920 with SBR) samples per channel, 1024-sample frames 1024 (2048).
- **MP2**: 1152 interleaved stereo pairs per frame (mono duplicated), about -1..1.

## Damaged input

Both decoders take whatever bytes come off the air: they never read outside what they are given,
refuse what they cannot decode (`decode` returns false, `pcm` empty) and stay usable. FAAD2 keeps
a few states for ever that one damaged AU can set wrongly, and this port deals with them (each
marked "OFFgrid-SDR" in the source):

- a damaged AU that changes the stream's element layout (FAAD2 error 21) or switches implicit SBR
  on (error 23) made FAAD2 refuse every AU after it: the decoder rebuilds itself from its
  configuration on those errors and after 8 failures in a row (`restarts()` counts it);
- with explicit parameters saying "no SBR" (DAB+'s `sbr_flag` 0), SBR-looking fill data is skipped
  instead of turning SBR on;
- dynamic range control gains last while the stream keeps sending them, instead of for ever: a
  damaged AU's fake DRC data had left a fixed +9 dB or -6 dB on all the audio after it.

On undamaged streams none of this changes a sample, with one exception: a stream that sends DRC
data only now and then (rather than in every frame, as encoders do) gets each gain for one frame,
not until the next one arrives.

## Files

| File | What it is |
|---|---|
| `Aac.h`, `Aac.cpp` | `codec::AacDecoder`, the public face of the AAC decoder |
| `AacFaadApi.h`, `AacCore.cpp`, `AacCommon.*`, `AacStructs.h`, `AacError.*` | FAAD2's decoder object, configuration and shared definitions (`decoder.c`, `common.c`, `neaacdec.h`, ...) |
| `AacMp4.*` | the AudioSpecificConfig parser |
| `AacSyntax.*`, `AacBits.*`, `AacHuffman.*`, `AacHcb.h`, `AacPulse.*` | the AAC bitstream: elements, ICS, sections, scale factors, spectral Huffman codebooks, ADTS |
| `AacHcr.*`, `AacRvlc.*` | error-resilient AAC (DRM): Huffman codeword reordering, reversible scale factors |
| `AacSpecrec.*`, `AacIqTab.h`, `AacPns.*`, `AacMs.*`, `AacIs.*`, `AacTns.*`, `AacDrc.*` | spectral reconstruction and the coding tools |
| `AacFiltbank.*`, `AacMdct.*`, `AacCfft.*`, `AacSineWin.h`, `AacKbdWin.h`, `AacMdctTab.h` | the filter bank (IMDCT 2048/256 and 1920/240, windows) |
| `AacSbr*.{h,cpp}` | spectral band replication |
| `AacPsDec.*`, `AacPsSyntax.cpp`, `AacPsTab.h` | MPEG-4 parametric stereo |
| `AacDrmPs.*` | DRM's parametric stereo |
| `AacOutput.*` | PCM output |
| `Mp2.h`, `Mp2.cpp` | `codec::Mp2Decoder` |

### What of FAAD2 is here, and what isn't

Ported: everything a `DRM_SUPPORT` floating-point build of FAAD2 compiles, with the preprocessor
choices resolved (unifdef): AAC-LC and ER AAC-LC syntax (SCE, CPE, LFE, DSE, PCE; a CCE makes the frame fail, error 32, as in FAAD2;
fill elements with OFFgrid-SDR's DAB+ change, ADTS/ADIF headers), section/scale factor/spectral
Huffman, pulse data, TNS, M/S, intensity stereo, PNS, DRC, the filter bank for 1024- and
960-sample frames, SBR (high quality), PS, DRM (object type 27, VCB11, HCR; RVLC parsed), DRM
parametric stereo. Left out (as in OFFgrid-SDR's build): Main-profile prediction, LTP, SSR, ER-AAC LD,
fixed point, low-power SBR, LATM, the 16/24/32-bit and double output formats' callers (the code is
kept, the library asks for float), `NeAACDecGetVersion`.

Each port file starts with our header block, then FAAD2's own notice, unchanged. The code is
FAAD2's, mechanically converted: C++11 casts for `malloc`, `const` error strings, `{}` for `{0}`,
min/max as `FAAD_MIN`/`FAAD_MAX`, everything in namespace `codec::faad`, and the `<math.h>` calls
made through double-precision wrappers (`faad_sqrt`, `faad_pow`, ...), because C++'s `<cmath>`
would pick float overloads for float arguments and round differently from FAAD2 built as C.

### What of pl_mpeg is here

Its MP2 decoder only (header, allocation, scale factors, dequantisation, the 32-band synthesis),
with OFFgrid-SDR's three DAB changes, reading one whole frame from a bounded bit reader instead of
pl_mpeg's byte stream (so no sync search, and no check that a header matches the previous one).
A free-format bit rate (index 0), which read pl_mpeg's table at -1, is refused, as is a frame whose
audio runs past its own end.

## Test

```sh
c++ -std=c++11 -O2 -Isrc src/codecs/*.cpp tests/test_codecs.cpp -o test_codecs
./test_codecs            # data in tests/data; optional: data directory, fuzz rounds
```

It decodes OFFgrid-SDR's DAB+, DAB (MP2) and DRM demo stations' audio and HE-AAC, HE-AAC v2,
AAC-LC stereo and MP2 (48/24 kHz, mono/stereo) test streams (`tests/data/codec_*`), compares each
frame with the original FAAD2's and pl_mpeg's output (`tests/data/codec_reference.txt`, made by
the original C code), feeds thousands of damaged units through every configuration, and prints the
real-time factor. Build it with `-fsanitize=address,undefined` for a memory-safety run.

## Speed

On an Apple M-series core, -O2: DAB+ AAC-LC 32 kHz mono about 3000x real time, DRM AAC 24 kHz
about 500x, HE-AAC v2 48 kHz about 600x, MP2 1500-3500x. Peak stack use about 110 kB (HE-AAC v2),
16 kB (MP2): fine for any thread.
