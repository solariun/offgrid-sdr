# liboffgrid-decoders — radio text and picture decoders in C++11

Six decoders behind one interface, each also usable alone through its own
header. Input: mono demodulated audio (any rate from 8 kHz up). Output: text
(or picture columns for Hellschreiber). No threads, no devices, no terminal:
the standard library and liboffgrid-dsp's FFT only.

| Decoder | Header | Class | What it reads |
|---|---|---|---|
| CW | `Cw.h` | `dec::CwDecoder` | Morse; learns the speed (WPM) and finds the tone itself |
| RTTY | `Rtty.h` | `dec::RttyDecoder` | Baudot/ITA2; finds the tone pair, shift (170/200/425/450/850 Hz), speed (45.45/50/75 Bd) and polarity |
| PSK | `Psk.h` | `dec::PskAutoDecoder` (`dec::PskDecoder` for one speed) | PSK31, PSK63, PSK125 (Varicode); picks the speed, follows the tone ±50 Hz |
| NAVTEX | `Navtex.h` | `dec::NavtexDecoder` | SITOR-B (FEC) ships' broadcasts, whole messages with station, subject and number |
| DSC | `Navtex.h` | `dec::DscDecoder` | HF Digital Selective Calling: format, category, nature of distress, MMSIs |
| Hellschreiber | `Hell.h` | `dec::HellDecoder` | Feld Hell: 14-dot columns at 245 dots/s, as a picture |

Signal generators for each (tests, demos, training): `FskEncoders.h` (namespace
`dec::enc`: CW, RTTY, NAVTEX, DSC, noise) and `PskEncoders.h` (`synthPsk`,
`synthHell`). Shared code tables: `Codes.h`.

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
| CW | the tone anywhere 300–1200 Hz | the CW mode's pitch |

## Tests

`make test` runs `tests/test_decoders_fsk.cpp` (CW, RTTY, NAVTEX, DSC) and
`tests/test_decoders_psk.cpp` (PSK, Hell): exact text from the encoders at 48
and 12 kHz in noise, no false text on noise alone, reset, NaN recovery, speed
and tone search, and the CPU cost (every decoder runs hundreds of times faster
than real time).

`examples/decode.cpp` decodes a WAV file: `bin/example-decode rtty file.wav`.

## Credits

Ported from OFFgrid-SDR (diddy-boy, GPL-3.0-or-later). Changes from the browser
version are marked "Addition to the JS" in the code; several are fixes worth
carrying back (PSK's lost first character and stray character after an over,
NAVTEX/DSC decoding a single transmission, a signal detector for Hell).
