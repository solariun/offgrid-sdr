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

## Audio clean-up filters

OFFgrid-SDR's clean-up filters, ported from its JavaScript, in
**`dsp/AudioFilters.h`** and **`dsp/VoiceCleaner.h`** (public: usable on their
own, and wired into `dsp::Receiver`). All are off by default; with all of them
off the receiver's output is bit for bit what it was before them.

| `ReceiverConfig` | Class | What it does | Modes |
|---|---|---|---|
| `noiseBlanker`, `noiseBlankerThresholdDb` | `NoiseBlanker` (`WideBlanker` + `ChannelBlanker`) | impulse noise, on the raw IQ: median-referenced spikes zeroed wideband, then the ±128 kHz slice around the station subtracted during spikes | AM SAM USB LSB CW |
| `humFilterHz` (0/50/60) | `HumFilter` | notches at the mains frequency and 7 harmonics (6-20 Hz wide) | AM SAM USB LSB CW |
| `autoNotch` | `AutoNotch` | finds steady whistles every 0.1 s, notches up to 3 and follows them | AM SAM USB LSB |
| `smartNotch` | `SmartNotch` | the whine finder: a list of notch frequencies to show, add, drag, remove | AM SAM USB LSB |
| `voiceCleaner`, `voiceCleanerMix` | `VoiceCleaner` | the AI voice cleaner: **RNNoise** (C++11 port, same weights as the app's WASM) mixed Full/Strong/Medium/Light | AM SAM USB LSB |
| `noiseReduction`, `noiseReductionLevel` (1-3) | `SpectralNR` | hiss: STFT, minimum-statistics floor, Wiener-style gain | AM SAM USB LSB |
| `limiter`, `retuneMuteSeconds` | `Limiter` | look-ahead, volume-aware peak limiter; mute then 100 ms fade-in after a retune | all |
| (application) | `DcBlocker` | the stick's DC spike, on the IQ before the spectrum and the receiver | — |

The audio filters run on the mono audio at the audio rate, after the decoder
tap, in the app's order: hum → auto notch → smart notch → voice cleaner →
noise reduction → retune mute → limiter → squelch and volume.

```cpp
c.mode = dsp::Mode::USB;
c.noiseBlanker = true;
c.voiceCleaner = true;                   // RNNoise, Medium
rx.configure(c);
...
int hum = 0;
double hz = rx.smartNotchFind(&hum);     // the strongest steady whine, notched (0: none; hum = 50/60: mains hum)
std::vector<double> lines = rx.smartNotches();
rx.smartNotchMove(0, hz + 5);            // dragged; smartNotchAdd / Remove / Clear, smartNotchSpectrum for the strip
```

The RNNoise port (`dsp::rnnoise::Denoiser`, release v0.1.1, BSD-3-Clause,
notice kept in `VoiceCleaner.cpp` and `RnnoiseModel.cpp`) matches the
reference C code bit for bit on the test input (gains, voice probability and
output of every frame). Cost on an Apple M-series core at 48 kHz: voice
cleaner 1.7 %, the others 0.05-0.5 % each, the blanker 1.4 % at 2 MS/s.
`tests/test_filters.cpp` documents the behaviour. Credits: the filters and
their tuning are OFFgrid-SDR's (diddy-boy); RNNoise is Jean-Marc Valin's
(Xiph.Org Foundation, Mozilla).

## Signal analysis: AUTO, the Signal Monitor, propagation, alerts

Ported from OFFgrid-SDR's JavaScript (diddy-boy); pure engines on numbers (no
UI, no decoder dependency: decoders are named by `dec::Kind` strings).

| Header | Class / functions | What it does |
|---|---|---|
| `dsp/AutoMode.h` | `AutoDetector` | **AUTO**: listens ~2 s near the dial and says what the signal is: a receiver mode (`"WFM"`, `"NFM"`, `"AM"`, `"USB"`, `"LSB"`, `"CW"`), a decoder (`"RTTY"`, `"NAVTEX"`, `"DSC"`, `"PSK"`, `"HELL"`, `"CW"`, `"FAX"`, `"SSTV"`, `"FT8"`, `"FT4"`, `"WSPR"`, `"TIME"`, `"DRM"`, `"DAB"`, `"ACARS"`, `"VDL2"`, `"AIS"`, `"AX25"`) and where to tune (SSB carriers to a few Hz from the voice's harmonics). By frequency alone: DAB, airband / ACARS / VDL2, AIS, APRS, PMR446, UK CB, marine VHF, FM broadcast. Listens again once, then falls back |
| `dsp/SignalMonitor.h` | `SignalMonitor` | up to 8 signals measured 10× a second from a ~100 Hz-bin spectrum of its own: level (average, peak), noise, S/N, occupied bandwidth, transmissions (count, average / longest, active / silent, occupancy, "every N s"), fades, drift (Hz/s now, Hz/min over 10 min, total), **Follow**, CSV log |
| | `SignalAnalyzer` | the deep analysis (~6 s of one band): FSK shift and baud (RTTY, NAVTEX/DSC, AFSK), PSK31/63/125, Morse WPM, OOK, FM, AM, carrier, SSB |
| | `ChannelExtractor` | a band mixed to 0 Hz and decimated (8-sample average + windowed sinc), streaming: the per-signal speakers and recordings feed a `Receiver` of their own from it |
| `dsp/Propagation.h` | `propagationGuide()` … | **Likely bands open**: from UTC time and lat/lon (or a Maidenhead locator), the MUF/LUF, each HF band open / possible / unlikely, the next dusk or dawn, the summary line |
| `dsp/SpectrumTools.h` | `SpectrumExtras` | peak hold (falls 3 dB/s), averaging, **new-signal alerts** (a signal where there was nothing for 2 minutes; learns the band for 10 s) |

```cpp
#include "dsp/AutoMode.h"

dsp::AutoDetector ad;
if (!ad.begin(dialHz))                        // false: it must listen
    while (!ad.feedIq(&iq[0], iq.size(), rate, centreHz)) next_block(iq);
const dsp::AutoResult& r = ad.result();       // r.mode, r.decoder, r.tuneHz, r.label, r.sure, r.reason

dsp::SignalMonitor mon;
int i = mon.add(14.0956e6, 400, "USB", "beacon", /*follow*/ true);
mon.feedIq(&iq[0], iq.size(), rate, centreHz);   // every block
dsp::MonitorStats s = mon.stats(i);           // s.snr, s.nTx, s.duty, s.interval, s.driftNow ...
std::string log = mon.csv(unixTimeAtZero);

dsp::PropGuide g = dsp::propagationGuide(std::time(0), 51.5, -0.1, /*ham*/ true);
```

Cost on an Apple M-series core: the monitor with 8 signals at 2.048 MS/s
about 1 %; a `ChannelExtractor` 2 MS/s → 200 kHz about 2 %; AUTO's analysis
some tens of ms once per detection. `tests/test_analysis.cpp` (61 checks)
documents the behaviour: AUTO on signals from the repo's encoders and
`DemoSource`, the monitor's figures on keyed and drifting carriers, the
analyser, the propagation guide and the alerts.
