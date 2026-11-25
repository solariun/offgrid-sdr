# liboffgrid-decoders — radio text and picture decoders in C++11

Ten decoders behind one interface, each also usable alone through its own
header. Input: mono demodulated audio (any rate from 8 kHz up). Output: text
(pictures for fax and SSTV, columns for Hellschreiber, the time and the PC clock's error for time signals). No threads, no devices, no terminal:
the standard library and liboffgrid-dsp's FFT only.

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
| AX.25 / APRS | `Ax25.h` | `dec::Ax25Decoder` | packet radio: AFSK 1200 (VHF), AFSK 300 (HF), G3RUH 9600; frames with digipeater paths; APRS positions (plain, compressed, Mic-E), weather, messages, status, objects, items, telemetry |

Signal generators for each (tests, demos, training): `FskEncoders.h` (namespace
`dec::enc`: CW, RTTY, NAVTEX, DSC, noise), `PskEncoders.h` (`synthPsk`, `synthHell`),
`PictureEncoders.h` (fax and SSTV transmissions, test pictures), `TimeEncoders.h`
(CHU, WWV, WWVH, RWM at any UTC time) and `Ax25Encoders.h` (AX.25 frames, AFSK/G3RUH
audio, the demo APRS packets). Shared code tables: `Codes.h`.

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

Time decoders also need `setTimestamp(unix seconds)` before each `process()`: the
computer's clock at the block's first sample. Picture decoders give
`dec::PictureLine`s through `takeLines()` (a new picture number = a new picture).

## Tests

`make test` runs `tests/test_decoders_fsk.cpp` (CW, RTTY, NAVTEX, DSC),
`tests/test_decoders_psk.cpp` (PSK, Hell), `tests/test_decoders_picture.cpp` (fax,
SSTV), `tests/test_decoders_time.cpp` (time signals, clock) and
`tests/test_decoders_ax25.cpp` (AX.25, APRS): exact text from the encoders at 48
and 12 kHz in noise, no false text on noise alone, reset, NaN recovery, speed
and tone search, and the CPU cost (every decoder runs hundreds of times faster
than real time).

`examples/decode.cpp` decodes a WAV file: `bin/example-decode rtty file.wav`.

## Credits

Ported from OFFgrid-SDR (diddy-boy, GPL-3.0-or-later). Changes from the browser
version are marked "Addition to the JS" in the code; several are fixes worth
carrying back (PSK's lost first character and stray character after an over,
NAVTEX/DSC decoding a single transmission, a signal detector for Hell).
