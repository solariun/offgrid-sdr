# liboffgrid-sdrconnect: an SDRplay RSP through SDRconnect, in C++11

Uses an **SDRplay RSP** (RSP1A, RSP1B, RSPdx, RSPduo and others) through SDRplay's
**SDRconnect** and its **WebSocket API** (SDRplay's spec 1.0.3; SDRconnect 1.0.6 or
later, and 1.0.10 still reports API 1.0.3). SDRconnect can run on this computer or on
another one on the network, on port 5454. It needs no external libraries: no
libwebsockets, no OpenSSL and no SDRplay driver, only POSIX sockets. It builds on Linux
and macOS.

It's ported from the SDRconnect support in OFFgrid-SDR's browser version (index.html,
by diddy-boy): *SDRconnect (from 2.3)*, *SDRconnect Lite mode (from 2.4)* and *the
speed test (from 2.4)*. See [SDRconnect.md](../../SDRconnect.md) for setting up
SDRconnect itself (headless server, service, firewall).

## Files

| File | |
|---|---|
| `sdrconnect/SdrConnect.h` | **the public class**, `sdrc::Client` |
| `sdrconnect/WebSocket.h` | a minimal RFC 6455 client (`sdrc::WebSocket`), with SHA-1, base64 and framing (also usable alone) |
| `sdrconnect/*.cpp` | implementation |

## Use

```cpp
#include "sdrconnect/SdrConnect.h"

sdrc::Client rsp;
std::string err;
if (!rsp.connect("shack-pc", err)) { /* err says why, in OFFgrid-SDR's words */ }
rsp.setSampleRate(2048000);          // 2.048, 6, 8 or 10 MHz views
rsp.setFrequency(100e6);             // Hz
rsp.setGainAuto();                   // or rsp.setGainFraction(dB / 49.6)
rsp.setStateCallback([](sdrc::LinkState s, const std::string& msg) { /* NET badge, status line */ });
rsp.startIq([](const std::complex<float>* iq, std::size_t n) {
    // complex float, full scale +-1, at rsp.sampleRate(), centred on rsp.frequency(),
    // on the client's thread: copy it somewhere and return quickly.
    // sdrc::Client::toU8(iq, n, out) gives an RTL-SDR's u8 I,Q if you need it.
}, err);
...
rsp.setLite(true);                   // swap to Lite mode while running (or startLite(...))
rsp.stop();                          // streams off, RSP stopped; still connected
rsp.disconnect();                    // or let the destructor do it
```

**Lite mode**, for slow networks. SDRconnect demodulates and sends its own sound and
spectrum, about 0.2 MB/s instead of 8 MB/s:

```cpp
rsp.startLite([](const float* lr, std::size_t frames) { /* 48 kHz, L R L R */ },
              [](const float* dB, std::size_t bins) { /* low frequency first */ }, err);
rsp.setLiteTuning(sdrc::Client::liteDemodulator("WBFM"), dialHz - modeShift, filterHz);
```

`liteSupports(mode)` returns false for DAB, DRM, AIS and VDL2, which need the full stream.
`liteStats()` reports what's arriving. The state callback says so if no sound is arriving,
or only silence (SDRconnect's own mute or volume).

**Before connecting the radio:** `speedTest(rate)` checks whether the network can carry the
full stream. It returns MB/s against `rate * 4 / 1e6`, with a verdict of **keeps up** (97% or
more), **only just** (90 to 97%) or **too slow**, as text ready to show. `probeAntennas()` gets
the full antenna list: above 30 MHz it tunes to 1 MHz for a moment, because SDRconnect only
lists the antennas usable at the present frequency. After that, `antennas()` returns the list.

The rest of the API:

- **Antennas:** `setAntennaRules()`, `antennaFor(hz)`, `setAntenna(name)`, `setAutoAntenna(bool)`.
  By default an RSPdx uses A from 30 MHz up and B below, and never C above 200 MHz.
- **Profiles:** `setStartProfile(name)` and `applyProfile(name)` use SDRconnect device profiles,
  which is how to switch on the bias-tee, since the API has no switch for it.
- **Gain and status:** `lnaRange()`, `lnaState()`, `overload()`, `canControl()`, `stats()`.
- **Raw access:** `property(name)`, `requestProperty()`, `setProperty()`.

Calls return `false`, or an error string, on failure and never throw.

### Threads

Each `Client` has one thread. It reads the socket, keeps the properties, runs the timers,
reconnects, and calls **every** callback. All other calls are thread-safe. Some calls block
and wait for SDRconnect: `connect`, `startDevice`, `startIq`, `startLite`, `stop`, `speedTest`,
`probeAntennas` and `applyProfile`. Don't call those from inside a callback. If you do, they
refuse with an error.

### Behaviour kept from the JS

- **Starting the RSP:** the client waits until SDRconnect says it's `started` (15 s at most)
  before asking for IQ. In the field, asking too early crashed SDRconnect's headless server.
  It doesn't select a device that's already active.
- **Retuning:** samples are dropped until SDRconnect confirms the new frequency, to within
  5 kHz because it rounds to its tuning step. Then they're dropped until the new band's LNA
  range has arrived (0.8 s at most). If no confirmation comes within 1.5 s, the client carries
  on anyway. When SDRconnect has rounded the centre, the samples are shifted so stations
  appear at their true frequency.
- **Gain:** after a gain change, the samples for the next 250 ms are dropped while the RSP
  settles.
- **Sample rate:** if SDRconnect keeps a different rate from the one asked for (an RSPdx kept
  2.048 MHz when asked for 1.024), the samples are resampled to the rate asked for.
- **Gain slider:** the first 90% of the slider covers the safe 70% of the LNA range. Auto uses
  the band's middle LNA state.
- **Reconnecting:** if the link drops while streaming, the state becomes `Lost` and the client
  retries every 3 s. Once reconnected, it starts the RSP again, re-sends the stream, tuning,
  gain and antenna settings, and reports "Reconnected". A link that drops while idle doesn't
  retry.

The app keeps these parts of the JS, because they belong to its radio rather than to the link:

- the 0.1 s jitter reserve before samples are handed over;
- Lite mode's loudness levelling and its spectrum alignment with the full stream;
- the diagnostics log.

## The wire format (as the JS uses it)

**Commands** are JSON text with three string fields, for example
`{"event_type":"set_property","property":"device_center_frequency","value":"100000000"}`.

The `event_type` values used are:

- `get_property` and `set_property`
- `iq_stream_enable`, `audio_stream_enable` and `spectrum_enable`
- `device_stream_enable`
- `selected_device_name`
- `apply_device_profile`

**Answers** have the event type `get_property_response` or `property_changed`, with the
`property` and its `value`. An optional `device` field is accepted only when it's `primary`.

**Binary messages** start with a 16-bit little-endian type:

| Type | Payload | Rate |
|---|---|---|
| 2 | IQ: int16 LE I, Q, I, Q… (÷ 32768) | `device_sample_rate`: 4 bytes a sample, 8.2 MB/s at 2.048 MHz, 40 MB/s at 10 MHz |
| 1 | Lite sound: int16 LE stereo L, R… (÷ 32768) | 48 kHz |
| 3 | Lite spectrum: a byte a bin, lowest first | dB = `spectrum_base` + byte/255 × (`spectrum_ref_level` − `spectrum_base`) |

## Build and test

```
c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors -Isrc -pthread \
    src/sdrconnect/*.cpp tests/test_sdrconnect.cpp -o test_sdrconnect && ./test_sdrconnect
```

The test needs no RSP. It checks:

- SHA-1 and base64 against their published test vectors, and the RFC 6455 handshake example;
- the framing, against the RFC's own example frames;
- the client, against a **mock SDRconnect** running on localhost.

In the static library build, `src/sdrconnect/*.cpp` becomes `liboffgrid-sdrconnect.a`.
Linking it needs only `-pthread`.
