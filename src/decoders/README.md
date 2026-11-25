# liboffgrid-decoders — radio text and picture decoders in C++11

Eighteen decoders behind one interface, each also usable alone through its own
header. Input: mono demodulated audio (any rate from 8 kHz up), or the stick's IQ
for VDL2, AIS, DAB and DRM. Output: text (pictures for fax and SSTV, columns for
Hellschreiber, the time and the PC clock's error for time signals; for DAB and
DRM also sound, a station list and slideshow pictures). No threads, no devices,
no terminal: the standard library, liboffgrid-dsp's FFT and, for DAB and DRM's
sound, liboffgrid-codecs.

| Decoder | Header | Class | What it reads |
|---|---|---|---|
| CW | `Cw.h` | `dec::CwDecoder` | Morse; learns the speed (WPM) and finds the tone itself |
| RTTY | `Rtty.h` | `dec::RttyDecoder` | Baudot/ITA2; finds the tone pair, shift (170/200/425/450/850 Hz), speed (45.45/50/75 Bd) and polarity |
| PSK | `Psk.h` | `dec::PskAutoDecoder` (`dec::PskDecoder` for one speed) | PSK31, PSK63, PSK125 (Varicode); picks the speed, follows the tone ±50 Hz |
| NAVTEX | `Navtex.h` | `dec::NavtexDecoder` | SITOR-B (FEC) ships' broadcasts, whole messages with station, subject and number |
| DSC | `Navtex.h` | `dec::DscDecoder` | HF Digital Selective Calling: format, category, nature of distress, MMSIs |
| Hellschreiber | `Hell.h` | `dec::HellDecoder` | Feld Hell: 14-dot columns at 245 dots/s, as a picture |
| Weather fax | `Fax.h` | `dec::FaxDecoder` | HF radiofax (IOC 576), 60/90/120/240 LPM, 1809-pixel lines; start/phasing/stop tones, automatic slant correction, manual start and alignment |
| SSTV | `Sstv.h` | `dec::SstvDecoder` | slow-scan TV by VIS code: Martin M1/M2, Scottie S1/S2, PD90, PD120, with drift correction |
| Time signals | `Time.h` | `dec::TimeDecoder` | CHU (FSK code), WWV and WWVH (100 Hz BCD code, told apart by their ticks), RWM (minute marker): the time and the PC clock's error to ±2.5 ms |
| Clock | `Clock.h` | `dec::ClockCorrection` | the corrected time: offset, source, history, the PC clock's drift from two readings, save/load |
| ACARS | `Acars.h` | `dec::AcarsDecoder` | aircraft messages: 2400 bd MSK on AM audio; registration, flight, label (with its meaning), text; the parser is shared with VDL2 |
| VDL2 | `Vdl2.h` | `dec::Vdl2Decoder` | VHF Data Link Mode 2 from IQ: every channel 136.6–137 MHz at once; D8PSK, Reed-Solomon, AVLC; aircraft and ground addresses with countries, ACARS over AVLC |
| AIS | `Ais.h` | `dec::AisDecoder` | ships from IQ, both channels at once: positions, static/voyage data, class B, aids to navigation, base stations; a vessel table and `!AIVDM` sentences |
| FT8 / FT4 | `Ft8.h` (`Ftx.h`) | `dec::Ft8Decoder`, `dec::Ft4Decoder` | 15 s / 7.5 s slots by the corrected clock, every 77-bit message type, LDPC, up to three subtract-and-decode passes; country, distance and bearing of the caller; slot timings for the clock |
| WSPR | `Wspr.h` | `dec::WsprDecoder` | 2-minute slots, Fano decoder, about −28 dB; finds stations with the clock a minute out; measures the clock and the stick's frequency error |
| Callsigns, locators | `Callsign.h` | `dec::countryOf`, `dec::Locator` | offline callsign → country (ITU prefixes and areas), Maidenhead ↔ lat/lon, distance and bearing, 620 cities |
| Heard | `Heard.h` | `dec::HeardList` | every FT8/FT4/WSPR decode, band by band: stations, countries, farthest, open directions, trend |
| DAB / DAB+ | `Dab.h` | `dec::DabDecoder` | digital radio from IQ at 2.048 MS/s (Band III, 5A–13F): lock, FIC, the ensemble and its stations, DAB+ (AAC) and classic DAB (MP2) sound through liboffgrid-codecs, dynamic label text, MOT slideshow and SPI logos; choose a station with `selectService` |
| DRM | `Drm.h` | `dec::DrmDecoder` | Digital Radio Mondiale from IQ (48 kS/s up): robustness modes A–D, every occupancy, 4/16/64-QAM, FAC and SDC, up to four services, AAC sound (with SBR/PS) through liboffgrid-codecs, text messages, MOT slideshow, Journaline pages; xHE-AAC is recognised but not played |
| AX.25 / APRS | `Ax25.h` | `dec::Ax25Decoder` | packet radio: AFSK 1200 (VHF), AFSK 300 (HF), G3RUH 9600; frames with digipeater paths; APRS positions (plain, compressed, Mic-E), weather, messages, status, objects, items, telemetry |

Signal generators for each (tests, demos, training): `FskEncoders.h` (namespace
`dec::enc`: CW, RTTY, NAVTEX, DSC, noise), `PskEncoders.h` (`synthPsk`, `synthHell`),
`PictureEncoders.h` (fax and SSTV transmissions, test pictures), `TimeEncoders.h`
(CHU, WWV, WWVH, RWM at any UTC time), `Ax25Encoders.h` (AX.25 frames, AFSK/G3RUH
audio, the demo APRS packets), `AviationEncoders.h` (ACARS audio, VDL2 bursts),
`AisEncoders.h` (AIS messages and GMSK bursts), `Ftx.h` (FT8/FT4 synthesis),
`WsprEncoders.h` (WSPR transmissions), `DabEncoders.h` (the browser version's demo
multiplex, `dec::DabDemoMultiplex`) and `DrmEncoders.h` (a DRM transmitter for every
mode and occupancy, and the two demo stations).

FT8/FT4 follow [ft8_lib](https://github.com/kgoba/ft8_lib) by Kārlis Goba (MIT); WSPR
follows WSJT-X's wsprd (K1JT) and KA9Q's Fano decoder; ACARS follows acarsdec and
VDL2 dumpvdl2, as the browser version credits. DAB follows the browser version's
receiver (after welle.io / ETSI EN 300 401); DRM follows it and
[Dream](https://sourceforge.net/projects/drm/) (GPL-2.0-or-later, `licenses/dream-COPYING.txt`). Shared code tables: `Codes.h`.

The AX.25 modem, HDLC and frame code in `ax25/` come from
[AX25Toolkit](https://github.com/solariun/AX25Toolkit) (Gustavo Campos), whose DSP
derives from [Dire Wolf](https://github.com/wb2osz/direwolf) by John Langner,
WB2OSZ (GPL-2.0-or-later).

## Use

```cpp
#include "decoders/Decoder.h"

std::unique_ptr<dec::Decoder> d = dec::makeDecoder(dec::Kind::Rtty);
d->process(audio, n, 48000);          // feed as it arrives, any block size
std::string text;
d->takeText(text);                    // what's new since last time
dec::Status s = d->status();          // locked, snrDb, toneHz, shiftHz, baud, wpm, mode, info
```

Or one decoder with its own settings:

```cpp
#include "decoders/Rtty.h"
dec::RttyDecoder rtty;
rtty.setFixedShift(450);              // 0 = find it
rtty.setFixedBaud(50);
rtty.setAutoReverse(false);
```

## Where the signal must be in the audio

Demodulate in USB with the dial below the signal by the decoder's
`audioCenterHz()`, as OFFgrid-SDR does:

| Decoder | Audio centre | Dial for a listed frequency F |
|---|---|---|
| RTTY, NAVTEX, DSC | 1500 Hz (tones e.g. 1415/1585) | F − 1.5 kHz |
| PSK, HELL | 1000 Hz | F − 1 kHz |
| FAX | 1900 Hz (1500 black … 2300 white) | F − 1.9 kHz |
| SSTV | 1500–2300 Hz video, 1200 Hz sync | F in USB (LSB below 10 MHz, NFM above 30 MHz) |
| TIME | the carrier at 1000 Hz | F − 1 kHz |
| AX.25 1200 | 1200/2200 Hz | F in NFM (APRS 144.800 / 144.390) |
| AX.25 300 | 1600/1800 Hz | F (the listed dial) in USB |
| CW | the tone anywhere 300–1200 Hz | the CW mode's pitch |

Time and slot decoders (TIME, FT8, FT4, WSPR) also need `setTimestamp(unix seconds)`
before each `process()`: the computer's clock at the block's first sample; slot
decoders also `setClockOffset()`. IQ decoders (AIS, VDL2, DAB, DRM: `wantsIq()`) take
`processIq(iq, n, rate, centerHz, dialHz)` instead of audio. DAB needs exactly 2.048 MS/s
with the stick within about 200 kHz of the channel's centre (the dial names the channel);
DRM any rate from 48 kS/s, the dial within about 1.6 kHz of the station (DRM's carrier 0:
the centre of a 10/20 kHz signal). Both give sound (`hasAudio`, `audioRate`, `takeAudio`:
interleaved stereo), stations (`services`, `selectService`) and pictures (`takeImages`:
JPEG/PNG bytes). Slot decoders do about
0.2–0.8 s of work at each slot's end: call them from a thread of their own. Picture decoders give
`dec::PictureLine`s through `takeLines()` (a new picture number = a new picture).

## Tests

`make test` runs `tests/test_decoders_fsk.cpp` (CW, RTTY, NAVTEX, DSC),
`tests/test_decoders_psk.cpp` (PSK, Hell), `tests/test_decoders_picture.cpp` (fax,
SSTV), `tests/test_decoders_time.cpp` (time signals, clock) and
`tests/test_decoders_ax25.cpp` (AX.25, APRS), `tests/test_decoders_aviation.cpp`
(ACARS, VDL2), `tests/test_decoders_ais.cpp`, `tests/test_decoders_ftx.cpp` (FT8, FT4,
callsigns, Heard), `tests/test_decoders_wspr.cpp`, `tests/test_decoders_dab.cpp`
(the demo multiplex: stations, byte-exact audio units, text, slides, logos, sound;
noise, frequency error, an echo, lost samples) and `tests/test_decoders_drm.cpp`
(every mode and occupancy, byte-exact audio units, sound, text, slides, Journaline;
echo, mirrored spectrum, noise): exact text from the encoders at 48
and 12 kHz in noise, no false text on noise alone, reset, NaN recovery, speed
and tone search, and the CPU cost (every decoder runs hundreds of times faster
than real time).

`examples/decode.cpp` decodes a WAV file: `bin/example-decode rtty file.wav`.

## Credits

Ported from OFFgrid-SDR (diddy-boy, GPL-3.0-or-later). Changes from the browser
version are marked "Addition to the JS" in the code; several are fixes worth
carrying back (PSK's lost first character and stray character after an over,
NAVTEX/DSC decoding a single transmission, a signal detector for Hell).
