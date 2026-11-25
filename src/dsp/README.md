# liboffgrid-dsp — radio signal processing in C++11

IQ samples in, sound out, for one tuned signal; plus a spectrum analyser and a
simulated band. No dependencies beyond the C++11 standard library.

| | |
|---|---|
| `dsp::Receiver` | WFM (stereo, de-emphasis, **RDS**), NFM, AM, SAM, USB, LSB, CW; channel filter, squelch, AGC, volume; level, noise floor, carrier offset (calibration), a mono tap for decoders |
| `dsp::SpectrumAnalyzer` | windowed FFT (Blackman-Harris), dBFS, averaging, frame-rate limit |
| `dsp::DemoSource` | a band of simulated stations as u8 IQ (FM with RDS, AM, SSB, CW, NFM), plus `addAudioStation` for any audio as an SSB signal |
| `dsp::convertU8` | the stick's unsigned bytes to complex samples |

Everything public is in **`dsp/Receiver.h`**; the other headers are internals
(`Fft.h` is shared with liboffgrid-decoders).

## Use

```cpp
#include "dsp/Receiver.h"

dsp::Receiver rx(48000);                 // audio rate
dsp::ReceiverConfig c;
c.sampleRate = 2048000;
c.offsetHz = 250000;                     // the station minus the stick's centre
c.mode = dsp::Mode::WFM;
rx.configure(c);                         // any time; cheap changes are instant

std::vector<dsp::Complex> iq;
std::vector<float> stereo;               // L, R, L, R ... at 48 kHz
dsp::convertU8(bytes, byteCount, iq);    // byte count, not samples
rx.process(&iq[0], iq.size(), stereo);
dsp::RdsInfo rds = rx.rds();             // PI, PS, RadioText, PTY, clock
double level = rx.channelDbfs(), floor = rx.noiseDbfs();
```

Cost on an Apple M2 at 2.4 MS/s: WFM stereo + RDS 5.3 % of one core, the other
modes about 2 %. `tests/test_dsp.cpp` (82 checks) documents the behaviour.

Ports OFFgrid-SDR's RDS decoder, SAM demodulator and SSB high-pass (diddy-boy),
and follows the chain design of [signals](https://github.com/jtarrio/signals)
(Apache-2.0).
