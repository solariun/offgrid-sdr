# OFFgrid-SDR TUI — Specification (SDD), v0.4.0

**What it is.** `offgrid` is a C++11 port of OFFgrid-SDR, the single-file browser receiver by diddy-boy, to a
**text terminal on Linux and macOS**. It drives an RTL-SDR stick through **its own driver** (no librtlsdr, no
libusb), demodulates in software, plays the sound, and shows a full keyboard-and-mouse interface (big frequency,
mode buttons, lights, S-meter, spectrum, waterfall, band list, controls, RDS, status) drawn with the
**Graphicx** library (github.com/solariun/Graphicx), which was extended for this port with a pixel canvas, a cell
painter and a widget toolkit (§9). It builds into **one binary with no third-party libraries**: fully static on
Linux, OS libraries only on macOS (§0.2 D6).

**Status (2026-11-25): phases 1 (`46da55f`), 2 (`254ddd6`) and 3 (`0f92db6`) committed; phase 4 built, measured and committed.** Phase 4 adds ACARS, VDL2, AIS, FT8, FT4, WSPR, offline callsign/locator tools, the Heard list, a decoder thread (D18) and `--locator`. **First live data decode:** VDL2 from a real aircraft (E33).
Phase 3 adds weather fax, SSTV, time signals with the clock correction, AX.25/APRS (on Gustavo's AX25Toolkit, brought
forward from phase 4 at his request), PNG pictures, and full mouse use (D14). Phase 2 adds
six decoders (CW, RTTY, PSK31/63/125, NAVTEX, DSC, Hellschreiber), memories, frequency labels, a logbook, a settings
file, scanning and frequency calibration, and splits the code into four reusable static libraries (D11). Built: the USB layer (Linux
usbfs, macOS IOKit), the RTL-SDR driver (RTL2832U + R820T/R820T2/R828D, Blog V4 inputs), the DSP chain (WFM stereo
+ RDS, NFM, AM, SAM, USB, LSB, CW; spectrum; demo band), sound out (macOS AudioQueue, Linux ALSA ioctls or a
pipe), the sources (stick, rtl_tcp, IQ file, demo), the engine, the terminal interface, WAV recording and the
headless mode. Measured on macOS with a **Nooelec NESDR SMArt v5 (R820T)**: 2.048 MS/s sustained, 0 drops,
0 sound gaps in 20 s, two live FM stations decoded in stereo with RDS (§10). **Not verified:** any Linux build
(no Linux machine or cross-compiler was available; the Linux files were only syntax-checked against stub
headers, P1), the R828D / Blog V4 tuner on hardware (P3), rtl_tcp against a real server (P5).

This file is normative: every change to behaviour, API, file layout or tests is recorded here first. Where this
file and the code disagree, the disagreement is a defect to be resolved here first.

**Repository:** github.com/solariun/offgrid-sdr (the C++ port lives beside the browser version's `index.html`).
**Author of the port:** Gustavo Campos. **License:** GPL-3.0-or-later (D3).

### Changelog

| Date | Change |
|---|---|
| 2026-11-25 | **Phase 1 written**: `src/usb`, `src/rtlsdr`, `src/dsp`, `src/audio`, `src/app`, `tools/rtl-probe.cpp`, `tests/test_dsp.cpp`, `Makefile`, `TUI.md`, this file; Graphicx added as the submodule `external/Graphicx`; README.MD points to TUI.md. |
| 2026-11-25 | **Graphicx extended** (Graphicx `c06f4d5`, the submodule's commit): `color.h` (colormaps, themes), `canvas.h` (pixel drawing, two fonts), `ui.h` (Surface, Painter, DotCanvas, Display, layout), `widgets.h` (App and ~25 widgets), events and mouse in `terminal.h`; `tests/test_ui.cpp` (124 checks), `examples/canvas.cpp`, `examples/dashboard.cpp`. The existing picture API is unchanged (its 62 checks still pass). |
| 2026-11-25 | **Phase 4**: `src/decoders/Acars.*`, `Vdl2.*`, `AviationEncoders.*`, `Ais.*`, `AisEncoders.*`, `Ftx.*`, `Ft8.*`, `Callsign.*`, `Heard.*`, `Wspr.*`, `WsprEncoders.*`; tests `test_decoders_aviation.cpp`, `test_decoders_ais.cpp`, `test_decoders_ftx.cpp`, `test_decoders_wspr.cpp`; Decoder.h gains `Kind::Acars/Vdl2/Ais/Ft8/Ft4/Wspr`, `wantsIq`/`processIq` and `setClockOffset`; Radio runs decoders on a worker thread with a bounded job queue (D18), keeps the Heard list, hands FT8/FT4/WSPR slot timings to the clock, passes the dial and the locator; DemoSource gains AM-envelope and burst stations; the demo band gains ACARS, VDL2, AIS, FT8, FT4 and WSPR stations; the Ui gains the Heard tab; options `--locator`. |
| 2026-11-25 | **Phase 3** (Graphicx `b03690b`): `src/decoders/Fax.*`, `Sstv.*`, `PictureEncoders.*`, `Time.*`, `Clock.*`, `TimeEncoders.*`, `Ax25.*`, `Ax25Aprs.cpp`, `Ax25Encoders.*`, `ax25/` (modem, HDLC, frames from AX25Toolkit); tests `test_decoders_picture.cpp`, `test_decoders_time.cpp`, `test_decoders_ax25.cpp`; `src/app/DecoderPlan.*`; Decoder.h gains `Kind::Fax/Sstv/Time/Ax25`, `PictureLine`, `takeLines`, `setTimestamp` and the picture/time fields of `Status`; SampleQueue stamps each block's arrival time; Radio gains a decoder setup hook and picture lines; the demo band gains fax, SSTV, CHU, WWV and APRS (NFM) stations; the Ui gains the toolbar, prompt buttons, frequency-display mouse actions, the picture panel with PNG saving and the corrected clock; headless saves pictures. Graphicx gains `png.h` (PNG writer with its own deflate). |
| 2026-11-25 | Defects found in phase 3 and fixed the same day: the toolbar built in `stopDecoder()` instead of `build()` (an edit landed at the wrong match); the prompt's button row on its border (5 rows, now 6); time decoders saw the computer's clock +0.13–0.15 s late (timestamped when processed and the demo's code made before it started: now stamped on arrival and built at start, E28); the stick overloaded on 144.8 MHz with automatic gain (E30). |
| 2026-11-25 | **Phase 2**: `src/decoders` (liboffgrid-decoders: `Decoder.h` interface + factory, `Cw`, `Rtty`, `Psk`, `Navtex` (+DSC), `Hell`, `Codes`, `FskEncoders`, `PskEncoders`), `tests/test_decoders_fsk.cpp`, `tests/test_decoders_psk.cpp`; `src/app/Store.*`; `examples/fm-radio.cpp`, `examples/decode.cpp`; a README per library; Receiver gains `decoderAudio()` and `carrierOffsetHz()`; DemoSource gains `addAudioStation()` and the demo band carries decoder stations; Radio gains the decoder and the decoder shift (D12); Ui gains the decoder panel, list tabs, prompts, scan, calibration, labels; Makefile builds the libraries. |
| 2026-11-25 | Defects found in phase 2 and fixed the same day: the demo's Hilbert transformer gave LSB (sign) and then no sideband at all (wrong taps visited) — now USB 37 dB over LSB (E19); the carrier offset wandered ±100 Hz with a 1 s average, then started from a transient with an 8 s one — now a cumulative mean after 0.5 s of settling (E21); Hell's signal flag never rose on a real signal (E20); RTTY re-locked at 75 Bd after noise gaps (P13); `Label` name clash in Ui; tab keys tuned instead of switching tabs. |
| 2026-11-25 | Defects found while measuring and fixed the same day: the IQ byte count passed as a sample count (half rate, §10 E5); sound "gaps" counted before the first audio arrived (E7); PI printed through a `double` format (E9); overlapping S-meter labels, list scroll bar over text, clipped clock (E10); 8.4 MB/5.6 s of terminal output cut to 2.9 MB (E11). |

### How to read this document

§0 holds the decisions, §1 the files and build, §2 the requirements (numbered **R**), §3 the architecture, §4–§8
each part as built, §9 the Graphicx contract the program relies on, §10 the evidence (numbered **E**), §11 the open
problems (**P**), §12 the plan for the next phases, §13 the glossary. "As built" means read from the code on
2026-11-25.

## 0. Decisions

### 0.1 The idea in one picture

```
 RTL-SDR stick ──USB──> usb::Device ──> rtlsdr::RtlSdr ─┐
 rtl_tcp server ──TCP────────────────────────────────────┤  u8 IQ blocks
 IQ recording ──file─────────────────────────────────────┼──> app::Source ──> SampleQueue
 dsp::DemoSource ──generated─────────────────────────────┘                        │
                                                                                   v  DSP thread
                                         ┌──────────── app::Radio::loop ──────────────┐
                                         │ convertU8 → SpectrumAnalyzer → spectrum    │
                                         │           → Receiver → audio → AudioOutput │──> speakers
                                         │                              → WAV file    │
                                         └───────────── Snapshot (mutex) ─────────────┘
                                                              │  30 times a second
                                                              v
                                    app::Ui (Graphicx App + widgets) ──> terminal
```

### 0.2 Decisions

| # | Decision | Why |
|---|---|---|
| D1 | **Phased port.** Phase 1 = driver, DSP core (WFM stereo + RDS, NFM, AM, SAM, SSB, CW), spectrum, waterfall, interface, sound, recording. Decoders, DAB/DRM, AUTO and the rest follow in phases 2–5 (§12). | Decided by Gustavo (2026-11-25): the browser version is ~14,000 lines of JavaScript plus DAB/DRM/FT8/AAC engines; one pass would leave most of it unverified. |
| D2 | **In the offgrid-sdr repository, beside `index.html`.** The browser version stays untouched; README.MD gains one pointer paragraph. | Decided by Gustavo. |
| D3 | **GPL-3.0-or-later** for the port; every source file carries `SPDX-License-Identifier: GPL-3.0-or-later`. Graphicx stays MIT. | Decided by Gustavo; required because the port reuses OFFgrid-SDR's code (GPL-3.0-or-later). |
| D4 | **Strict C++11**: every file compiles with `-std=c++11 -pedantic-errors -Werror=c++14-extensions -Werror=c++17-extensions -Wall -Wextra` with no warning. | Gustavo's rule for all his projects ("EVERYTHING must be C++11"). |
| D5 | **No third-party libraries.** USB through the kernel (usbfs) or IOKit; sound through ALSA ioctls or AudioQueue; FFT, filters, RDS written here. | Makes D6 possible and the build a single `make`. |
| D6 | **Fully static on Linux** (`-static`); **OS libraries only on macOS** (libSystem, libc++, IOKit, CoreFoundation, AudioToolbox). | Gustavo: "the build for linux and mac must be fully static". macOS has no static libSystem (Apple does not ship `crt0.o`/static libc), so the Mac binary is as static as macOS allows; `make check-static` shows it. |
| D7 | **The interface is Graphicx.** Everything drawn goes through Graphicx widgets; missing general features were added to Graphicx itself (§9), not to this program. | Gustavo: "extend graphicx to have all the graphics element for design … complete and powerful for all needs". |
| D8 | **Linux sound falls back to a pipe.** If the ALSA device can't be opened (a sound server owns it), raw S16 is piped to `pw-cat`, `pacat` or `aplay`. | Opening `/dev/snd` directly fails on most desktops (PipeWire/PulseAudio); a child process keeps the binary static. |
| D9 | **The RTL-SDR driver ports webrtlsdr**, the driver the browser version uses (Apache-2.0, which follows librtlsdr), not librtlsdr itself. | Same behaviour as the browser version, including its Blog V4 handling and gain model. |
| D11 | **Reusable libraries.** `liboffgrid-rtlsdr` (usb + rtlsdr), `liboffgrid-dsp`, `liboffgrid-decoders`, `liboffgrid-audio`, each with public headers, a README and no dependency on the program; the decoders depend only on the standard library and the DSP library's FFT; every decoder also has its own header. | Gustavo (2026-11-25): "make every decoder and appliable a lib so it can be reused in the future". |
| D12 | **In decoder modes the dial shows the signal's frequency**; the receiver is tuned below it by the decoder's audio centre (RTTY/NAVTEX/DSC 1500 Hz, PSK/HELL 1000 Hz; CW none). | As the browser version (`modeShift`), so listed frequencies are typed as they are. |
| D13 | **Settings in `$XDG_CONFIG_HOME/offgrid/settings.ini` (else `~/.config/offgrid`), the log in `logbook.csv` there**; plain text, saved on exit by a write-then-rename. The command line overrides saved settings; `--fresh` ignores them. | Human-readable, editable, no library. |
| D14 | **Everything by mouse too.** A toolbar for every keyboard-only action; OK/Cancel in prompts; click the frequency to type one, wheel over it to tune; click the decoder panel title for the decoder menu. | Gustavo (2026-11-25): "also make sure I can use mouse". |
| D15 | **AX.25 from AX25Toolkit.** The modem (Dire Wolf-derived), HDLC and frame code are copied into `src/decoders/ax25/` with their headers, adapted minimally (namespaces, float input, multi-slicer, two undefined-behaviour fixes); APRS interpretation ported from the browser version. | Gustavo: "also add the ax25 decoding you can also use the AX25ToolKit from my repos". AX25Toolkit has no LICENSE file (P16); Dire Wolf is GPL-2.0-or-later, compatible with D3. |
| D16 | **Pictures are saved as PNG** where offgrid was started (`offgrid-FAX-…png`, `offgrid-SSTV-…png`), when the next picture starts, the decoder stops or the program ends; also headless. | The browser version archives fax charts; PNG via Graphicx's own encoder keeps D5. |
| D17 | **The clock correction is kept in `clock.txt`** (`dec::ClockCorrection::save()`), with the JS sign (offset = PC − true); **T** sets it from a full time code. | Same rules as the browser version (drift from two readings ≥ 0.5 day apart). |
| D18 | **Decoders run on their own thread.** The DSP thread queues each block's decoder input (audio tap or IQ, timestamp, clock offset, dial, centre); the worker creates/resets the decoder and publishes text, pictures, status, Heard notes and slot timings under the Radio mutex. The queue holds about 2 s; older jobs are dropped and counted. | FT8/WSPR slot decodes take 0.2–0.8 s of CPU; on the DSP thread they would stall the sound and overflow the 0.5 s sample queue. |
| D19 | **FT8/FT4/WSPR slot timings correct the clock automatically** (ClockCorrection::applyFtxTimings), as the browser version does. | Same behaviour as the browser version. |
| D10 | **No Linux cross-compiler was installed** (`brew install zig` declined by Gustavo, 2026-11-25). | Linux code is syntax-checked against stub headers only until a Linux build is run (P1). |

### 0.3 Development standards: the definition of done

A change is done when: (1) this file describes it; (2) every source compiles under D4 with no warning, on clang
(macOS) and — once P1 is closed — GCC (Linux); (3) `make test` passes (`tests/test_dsp.cpp`, and Graphicx's
`make test`); (4) it was run: on the stick when it touches the driver or the DSP, in a pseudo-terminal capture
when it touches the interface (§10 E8); (5) every new file starts with the header block (title lines, SPDX, File,
Layer, What this file does, How it fits in, Contributing). Commits carry no co-author or "generated by" lines.

## 1. Files and build (normative)

### 1.1 Files

| Path | Contents |
|---|---|
| `src/usb/UsbDevice.h` | `usb::Device`: find, open + claim, control, sync bulk read, async bulk stream |
| `src/usb/UsbLinux.cpp` | Linux usbfs: `/sys/bus/usb/devices` scan, `USBDEVFS_*` ioctls, URB stream thread (`#ifdef __linux__`) |
| `src/usb/UsbMac.cpp` | macOS IOKit: `IOUSBDeviceInterface500` / `IOUSBInterfaceInterface500`, `ReadPipeAsync` on a CFRunLoop thread (`#ifdef __APPLE__`) |
| `src/rtlsdr/RtlSdr.h/.cpp` | `rtlsdr::RtlSdr`, the public driver class; 42 known USB ids |
| `src/rtlsdr/Rtl2832u.h/.cpp` | the RTL2832U demodulator: registers, I2C repeater, GPIO, resampler, IF, FIFO |
| `src/rtlsdr/R82xx.h/.cpp` | R820T / R820T2 / R828D tuners, Blog V4 band inputs and upconverter |
| `src/dsp/Receiver.h` | public DSP API: `convertU8`, `Mode`, `ReceiverConfig`, `RdsInfo`, `Receiver`, `SpectrumAnalyzer`, `DemoSource` |
| `src/dsp/Receiver.cpp`, `Filters.*`, `Fft.*`, `Rds.*`, `Spectrum.cpp`, `DemoSource.cpp`, `DspCommon.h` | the DSP (§6) |
| `src/audio/AudioOutput.h/.cpp` | `audio::AudioOutput`, `FrameRing`, the null output |
| `src/audio/AudioMac.cpp`, `AudioLinux.cpp` | AudioQueue; ALSA ioctls + pipe fallback |
| `src/app/Source.*` | `SampleQueue`; RTL, rtl_tcp, file and demo sources |
| `src/app/Radio.*` | the engine: DSP thread, tuning, snapshot, WAV recording |
| `src/app/Ui.*` | the terminal interface |
| `src/app/Bands.*`, `Options.*`, `main.cpp` | band plan and frequency text, command line, entry point, headless mode |
| `src/decoders/Decoder.h/.cpp` | the decoder interface, `Kind`, `Status`, `makeDecoder` |
| `src/decoders/Cw.*`, `Rtty.*`, `Psk.*`, `Navtex.*`, `Hell.*`, `Codes.*` | the decoders and their tables (§6b) |
| `src/decoders/FskEncoders.*`, `PskEncoders.*` | signal generators for tests and the demo band |
| `src/app/Store.*` | settings file, memories, labels, logbook |
| `src/*/README.md` | one per library |
| `examples/fm-radio.cpp`, `examples/decode.cpp` | the libraries used on their own |
| `tools/rtl-probe.cpp` | driver smoke test |
| `tests/test_dsp.cpp`, `tests/test_decoders_fsk.cpp`, `tests/test_decoders_psk.cpp` | self-tests (PASS/FAIL lines, exit code) |
| `external/Graphicx` | git submodule, MIT |
| `Makefile`, `TUI.md`, `spec.md`, `.gitignore` | build, user guide, this file |

### 1.2 Make targets (as built)

| Target | Result |
|---|---|
| `make libs` | `build/lib/liboffgrid-{rtlsdr,dsp,decoders,audio}.a` |
| `make examples` | `bin/example-fm-radio`, `bin/example-decode` |
| `make install` | the four libraries into `$(PREFIX)/lib`, headers into `$(PREFIX)/include/offgrid/{usb,rtlsdr,dsp,decoders,audio}` |
| `make` | `bin/offgrid` (links the four libraries and Graphicx). Linux: `-static -pthread` (R1.1); macOS: `-framework IOKit -framework CoreFoundation -framework AudioToolbox`. Stops with a message when `$(GRAPHICX)/include/graphicx/widgets.h` is missing. |
| `make tools` | `bin/rtl-probe` |
| `make test` | builds and runs every `tests/test_*.cpp` (`bin/test_dsp`, `bin/test_decoders_fsk`, `bin/test_decoders_psk`) |
| `make check-static` | macOS: `otool -L bin/offgrid`; Linux: `file` and `ldd` |
| `make clean` | removes `build/` and `bin/` |
| Variables | `GRAPHICX=path` (default `external/Graphicx`), `STATIC=0` (Linux dynamic), `CXX` |

## 2. Requirements

**R1 Build.** R1.1 Linux: one static executable, no shared library at run time. R1.2 macOS: only OS libraries
(D6). R1.3 Strict C++11 (D4). R1.4 No library beyond the C++ standard library and the OS (D5).

**R2 Sources.** R2.1 RTL-SDR sticks by index (`--device N`, `--list`). R2.2 rtl_tcp (`--tcp host[:port]`,
default port 1234, commands 0x01–0x05, 0x09, 0x0e). R2.3 Raw u8 IQ files (`--file`, `--center`, `--rate`),
played in real time, looped, not tunable outside their span. R2.4 The demo band (`--demo`), in real time.
R2.5 A source never blocks on the DSP: a bounded queue (64 blocks) drops the oldest block and counts it.

**R3 Driver.** R3.1 RTL2832U with R820T, R820T2, R828D; Blog V4 recognised by its USB strings. R3.2 Sample rates
225,001–300,000 and 900,001–3,200,000 Hz. R3.3 Frequency, ppm, gain steps or AGC, direct sampling, bias-tee.
R3.4 Streaming ≥ 2.4 MS/s without loss. R3.5 Linux: detach `dvb_usb_rtl28xxu`, reattach on close; clear
messages for permission and busy errors.

**R4 Demodulation.** R4.1 WFM: stereo from the 19 kHz pilot with mono fallback, de-emphasis 50/75 µs, RDS (PI,
PS, RadioText, PTY, TP/TA/MS, clock). R4.2 NFM, AM, SAM, USB, LSB, CW (pitch 700 Hz). R4.3 Filter width per
mode (defaults: WFM 150 k, NFM 12.5 k, AM/SAM 10 k, SSB 2.8 k, CW 500 Hz), squelch, volume, AGC.
R4.4 Audio at the output's rate, exact on average. R4.5 Real time with headroom: ≤ 25 % of one core at
2.4 MS/s for WFM stereo + RDS.

**R5 Spectrum.** Windowed FFT (Blackman-Harris), 256–65,536 bins, dBFS (full-scale tone = 0 dB), averaged,
≤ 30 frames/s.

**R6 Sound.** R6.1 Never blocks the DSP. R6.2 Starts silent until ~40 ms is queued; counts real dropouts.
R6.3 Delay bounded to ≤ 250 ms (clock drift). R6.4 WAV recording, 16-bit stereo.

**R7 Interface.** R7.1 Everything usable by keyboard *and* mouse. R7.2 Works from 90 columns (no side panel
below 90, a 28-column panel from 90, 34 from 110). R7.3 Truecolor default, `--256` option. R7.4 Only changed
cells are sent; waterfall at 15 rows/s. R7.5 Every theme from Graphicx. R7.6 The keys of TUI.md.

**R9 Decoders.** R9.1 One interface (`process` mono audio at any rate ≥ 8 kHz, `takeText`, `status`, `reset`,
`audioCenterHz`; pictures through `takeColumns`). R9.2 CW (speed learned), RTTY (shift 170/200/425/450/850, 45.45/50/75
Bd, polarity found), PSK31/63/125 (speed picked, AFC ±50 Hz), NAVTEX (messages with station/subject/number), DSC
(format, category, nature, MMSIs), Feld Hell (columns). R9.3 No text from noise alone. R9.4 Decoders hear the sound
before squelch and volume. R9.5 Each decoder usable alone (D11).

**R10 Keeping things.** Memories (name, frequency, mode, filter), labels (shown on the spectrum), the log (CSV, UTC),
the settings and each band's last frequency (D13).

**R11 Scan and calibration.** Band scan in the band's step, memory scan; 0.2 s per channel; stop on squelch (or 12
dB over the noise when the squelch is off), resume 2 s after. Calibration from the carrier of a known station:
error = −offset / frequency, added to the ppm correction after confirmation.

**R12 Pictures.** Fax 60/90/120/240 LPM, 1809 px, start/phasing/stop, automatic slant; SSTV Martin M1/M2, Scottie
S1/S2, PD90, PD120 by VIS; drawn as they arrive; saved as PNG (D16).

**R13 Time.** CHU, WWV/WWVH, RWM: the PC clock's error to ±10 ms (measured ±2.5 ms); blocks time-stamped on arrival;
the corrected clock shown and kept (D17).

**R14 AX.25.** 1200 Bd (NFM, VHF), 300 Bd (USB, HF), 9600 Bd (flat audio ≥ 38.4 kHz); FCS checked; APRS interpreted.

**R15 Mouse.** Every action reachable with the mouse (D14).

**R16 Phase 4 decoders.** ACARS (AM), VDL2 (IQ, all channels), AIS (IQ, both channels), FT8, FT4, WSPR (slots by
the corrected clock); countries, distance and bearing from `--locator`; the Heard summary.

**R8 Headless.** `--headless N`: one status line per second for N seconds (level, noise, measured rate, DSP load,
sound queue and gaps, drops, stereo, overload, RDS, strongest spectrum peaks).

## 3. Architecture (as built)

**Threads.** (1) The source thread: the USB stream thread (RTL), the socket reader (rtl_tcp), or the pacing thread
(file, demo: one 16,384-sample block whenever ≥ 8,192 samples are due by the wall clock; after a stall of more
than 1 s it skips ahead instead of catching up). (2) The DSP thread (`Radio::loop`): pops a block (50 ms
timeout), applies pending settings, converts, feeds the spectrum and the receiver, writes sound (only while
less than 250 ms is queued), writes the WAV, updates the snapshot; once a second it computes the DSP load,
measured sample rate and overload. (3) The sound thread (Linux) or the AudioQueue callback (macOS). (4) The main
thread: the interface at 30 frames/s, reading `Radio::snapshot` and calling the setters.

**Locking.** One mutex in `Radio` guards the settings, the snapshot and the WAV file; the DSP thread holds it only
to apply changes and to copy results, never while demodulating. `SampleQueue` and `FrameRing` have their own.

**Tuning (as built, `Radio::tune`).** The dial is clamped to 24 kHz–1,766 MHz. While the dial stays within ±40 %
of the span from the stick's centre, only the receiver's offset changes (instant, no USB traffic). Otherwise the
centre moves to dial − span/4 (or + span/4 near 0 Hz), keeping the DC spike off the signal, the queue is cleared
and the sound flushed. A file source never moves; the dial is limited to ±47.5 % of its span.

## 4. USB layer (as built)

`usb::Device::find(ids)` returns `DeviceInfo` (ids, strings, OS path); `open(info, interface, error)` claims
the interface. Linux: `/sys/bus/usb/devices/*` attributes, `/dev/bus/usb/BBB/DDD`, `USBDEVFS_GETDRIVER` /
`USBDEVFS_DISCONNECT` / `USBDEVFS_CLAIMINTERFACE`, `USBDEVFS_CONTROL`, `USBDEVFS_BULK`, a stream of N URBs reaped
with `USBDEVFS_REAPURBNDELAY` after `poll`, `USBDEVFS_CLEAR_HALT` on a stall, `ENODEV`/`POLLHUP` = unplugged;
on close it releases and reconnects the kernel driver. macOS: `IOUSBHostDevice` (fallback `IOUSBDevice`)
matched by vendor/product, `DeviceRequestTO` (fallback `ControlRequestTO`), `ReadPipeTO`, and N `ReadPipeAsync`
in flight on a CFRunLoop thread, resubmitted from each callback; stop = `AbortPipe` and wait for every callback.
Internally failures throw `rtlsdr::detail::IoError`; every public driver call catches it and returns `false`
with `error()` (do not build with `-fno-exceptions`).

## 5. RTL-SDR driver (as built)

Follows webrtlsdr 3.0.7 (D9). Control transfers are retried up to 5 times, 5–25 ms apart, on any failure
(P4). Tuner probing: R820T/R828D at I2C 0x34, then 0x74. Gain: webrtlsdr's model, steps of 3.5·k and 3.5·k+2.3
dB from 0 to 52.5 dB (`gains()`), not librtlsdr's 29-entry table. Direct sampling (`setDirectSampling`) takes
effect only below 28.8 MHz and never on a Blog V4, which has its own HF input: below 28.8 MHz the V4 selects its
HF input and upconverter (GPIO 5), 28.8–250 MHz the VHF input, above that UHF. The tuner bandwidth is the
initialisation value, not changed with the sample rate (P6). Control calls are serialised; the sample callback
must not call `stop()` or `close()`.

## 6. DSP (as built)

**Receiver chain** (`src/dsp/Receiver.cpp`): mixer (NCO) to 0 Hz → stage 1 FIR decimation (WFM to ~320–400
kHz, others to ~48 kHz) → stage 2 for SSB/CW (to ~12 kHz) → channel filter (real low-pass; SSB: complex
one-sided band-pass) → channel power and squelch → demodulator → fractional resampler to the audio rate →
squelch ramp, volume, clip, interleave. WFM: discriminator → RDS decoder on the multiplex; 19 kHz pilot PLL;
L−R from 38 kHz; 15 kHz low-pass with pilot removal; matrix (mono fallback); de-emphasis. NFM: discriminator +
DC removal. AM: envelope / carrier − 1. SAM: carrier PLL (OFFgrid-SDR's DemodSAM). USB/LSB: one-sided signal +
SSB high-pass (OFFgrid-SDR's SsbHighPass) + AGC. CW: narrow filter + BFO at `cwPitchHz` + AGC. Buffers only
grow: no allocation per block once sized.

**RDS** (`Rds.*`): a port of OFFgrid-SDR's RdsDecoder (names kept for cross-porting fixes) with a sturdier sync
rule (P7), RDS and RBDS programme-type tables (RBDS when de-emphasis is 75 µs), the RDS character set as UTF-8,
clock in local time; and an encoder for the demo (groups 0A, 2A, 4A). Changing the offset resets RDS, the pilot
PLL, SAM and the AGC.

**Details (as built):** filter widths are the flat passband (stop band 15 % of the width further out); SSB passes
100 Hz to the width; AGC attack 2 ms, hang 0.3 s (CW 0.5 s), decay 0.3 s, fixed gain ×20 with AGC off; WFM
stereo/mono cross-fade 50 ms; squelch ramp 5 ms; the noise floor is the 25th percentile of an FFT of the bins
outside the channel; SSB/CW audio may be up to ±4 frames off at an instant (exact over time); `process()`
allocates nothing once sized, except the caller's output vector and the RDS strings (~11 times a second).

**Spectrum** (`Spectrum.cpp`): ring of the latest N samples; 4-term Blackman-Harris; |X|²/(Σw)²; exponential
average in linear power; dB; fftshifted. **FFT** (`Fft.*`): radix-2, own code.

**Demo band** (`DemoSource.cpp`): deterministic. In 87.5–108 MHz: WFM stations at every x.3 and x.7 MHz with
pilot, stereo tones or melodies, RDS (`OFFGRID `, `DEMO FM `, `RADIO 1 `, `CLASSIC ` (mono), `JAZZ 24 `,
`NEWS 24 `). Elsewhere, every 2 MHz: AM music (+0.15), USB voice (+0.45), CW "CQ CQ DE OFFGRID" (+0.75), LSB
voice (+1.05), NFM tone (+1.35), AM tone (+1.65 MHz); noise at −42 dBFS (less lets the u8 quantiser's intermodulation fall on the regular station grid and garble RDS); output quantised like a stick. Each FM station is a full multiplex with pre-emphasis; the FM deviation is interpolated between programme samples (stepping it made images that broke RDS).

**Interface contract:** `convertU8(bytes, count, out)` takes the **byte** count (E5).

## 7. Sound (as built)

`FrameRing` (1 s at 48 kHz; when full the oldest frames go and an overflow is counted). macOS: AudioQueue,
float32 stereo, 4 × 1,024 frames, primed at 2 buffers (R6.2). Linux: `/dev/snd/pcmC0D0p` (or `--audio hw:C,D`)
with `HW_PARAMS` (RW interleaved, S16_LE, 2 ch, period ≥ 1,024, 4 periods, rate tried: asked, 48,000, 44,100),
`SW_PARAMS` (start at half the buffer, alsa-lib's boundary rule), `PREPARE`, `WRITEI_FRAMES` from a thread,
`PREPARE` again on `EPIPE`; else a pipe (D8). The null output (`--no-audio`, or when no output opens) drains
on the wall clock so timing is unchanged.

## 8. The program (as built)

**Options** (`Options.cpp`): as listed by `offgrid --help` and TUI.md. Frequencies parse `94.0` (below 2000 =
MHz), `7074k`, `145.8M`, `1090 MHz`, `145.800.000` (dots as group marks), plain Hz.

**Band plan** (`Bands.cpp`): 41 bands in 4 groups (Broadcast: FM, LW, MW, 120–11 m, DAB III; Amateur: 160 m–70
cm; Air & sea: Airband 8.33 kHz, Marine VHF, AIS, NOAA, NAVTEX; Other: PMR446, CB, WWV, The Buzzer, ISM 433),
each with range, start frequency, mode and step; the narrowest band containing a frequency names it; each
band remembers its last frequency for the session.

**Interface** (`Ui.cpp`): rows 0–3 header (BigText frequency in the 3x5 font with quadrants; mode RadioGroup;
band / step / filter line; six Leds; Meter −120…−10 dBFS with peak hold and labels; Sparkline of the level;
UTC clock); spectrum Plot (40 % of the main area) with the passband as a shaded band and the dial as a
marker; Waterfall under the plot area; side panel (≥ 90 columns): Bands ListBox, Controls (Volume, Squelch, RF
gain, Auto gain, Filter, Zoom x1–x32, WF floor, WF range, Stereo, Bias-T); RDS line; StatusBar. Spectrum window:
span/zoom, centred on the dial in quarter-window jumps; the DC bin is replaced by its neighbours' mean;
waterfall rows = peak of two frames (15/s); the waterfall floor follows the 20th percentile − 3 dB until the
user moves it. With no widget focused, keys tune (App with `autoFocus = false`, `clearFocus()` on Esc).

**Headless** (`main.cpp`): R8; peaks = local maxima 15 dB over the median, DC excluded, top 5; the carrier offset
and its ppm; with `--decoder`, the decoder's status and each new piece of text (`decoded: …`).

**Phase 2 in the interface (as built).** The side list has three tabs (Bands, Memories, Log; ←/→ switch them while
the list is focused). A decoder (menu on `d`, or `--decoder`) sets the mode (CW for CW, else USB with a 3 kHz
filter and the decoder shift, D12), opens a panel under the waterfall (a third of the main area: wrapped text,
newest at the bottom; Hell: the columns as a half-block picture) titled with the decoder's mode and status, and
adds a green spectrum marker where the decoder expects the tone. The decoder is reset on every retune. Memories
(`M`) are named after the RDS station when there is one; `Del` deletes the selected one. Labels (`l`) within 500 Hz
replace each other; empty text removes. The log (`L`) records UTC time, dial, mode (or the decoder), level, SNR,
RDS station and notes. Scan state: SCAN light amber while moving, green while held on a signal. Calibration (`K`)
needs WFM, NFM or SAM and a signal ≥ 10 dB over the noise, and shows the measured offset, the error and the new
correction before applying it; the ppm is saved.

**Persistence** (`Store.cpp`, D13): `[settings]` key=value; `[bands]` name=Hz; `[memories]` name|Hz|mode|filter;
`[labels]` Hz|text (sorted). Text fields lose `| , " tab CR LF`. Saved on exit: frequency, mode, filter, volume,
squelch, gain, ppm, stereo, de-emphasis, theme, colormap, step, zoom, waterfall range, decoder.

## 6b. Decoders (as built)

Common: every decoder implements `dec::Decoder` (D11, R9). Audio centres and dial offsets: RTTY/NAVTEX/DSC 1500 Hz
(tones e.g. 1415/1585 Hz; NAVTEX mark = higher tone, DSC a 1 bit = lower tone), PSK/HELL 1000 Hz, CW the tone
anywhere 300–1200 Hz. Ported from OFFgrid-SDR's JavaScript with these documented changes ("Addition to the JS" in the
code): **PSK** ignores a lone 0 before the first character (the JS lost the first character about half the time),
closes its squelch after 2 weak bit centres (the JS printed a stray character after each over), opens only on a
strong idle with a small AFC error, pulls in faster while shut; NaN recovery on every running value. **Hell** has a
tone-over-neighbours detector (tone at 1000 Hz against 500 and 1500 Hz, ≥ 6 dB, 1 s hold) for its signal flag.
**NAVTEX/DSC** find the 170 Hz pair by band energy, keep reading bits once the pair is known, break sync ties by
the phasing symbols, learn DX/RX parity from the phasing, so a single transmission decodes (the JS relied on its
looping demo); `navtexAudio` sends 144 phasing slots (≈10 s, as real stations). **CW** filters ±125 Hz around the
tone it finds by FFT before measuring the level (the JS relied on the receiver's narrow filter). **RTTY** runs a
twin decoder of the opposite polarity and switches when the twin reads better text; text already handed out is
not taken back. The first characters after a pause are lost while the speed is learned (CW, RTTY), as in the JS.

## 6c. Phase 3 decoders (as built)

**Fax** (`Fax.*`): instantaneous frequency from a 1900 Hz mix; states waiting → start tone (300 Hz) → phasing →
receiving → stop (450 Hz); lines assembled from two consecutive rows with an absolute start count (the JS rotated
one row); automatic slant from a line fit of the phasing pulses (≤ 500 ppm, outliers dropped); lines handed out
1.25 s late so the stop-tone trim can still drop them; `startNow()`, `stop()`, `shift()` for manual use.
**SSTV** (`Sstv.*`): pitch by a 1750 Hz mix, four 900 Hz low-pass stages, phase step; VIS → mode; line sync and
drift; the JS's six modes; a fixed 64k pitch ring. **Time** (`Time.*`): carrier found 700–1300 Hz; 5 ms frames;
tick profile per 5 ms of the second (1000 and 1200 Hz: WWV vs WWVH, an addition — the JS never used the 1200 Hz
profile); minute marker (CHU candidates refused while its FSK mark is on); CHU's Bell 103 bytes (seconds 31–39),
WWV's 100 Hz BCD; a single WWV frame accepted only when every bit is clear, else two agreeing frames; the second's
start at the rising frame's bin start (the JS's was 0–5 ms early); lost samples → `timeJump()`. **Clock**
(`Clock.*`): the JS clock rules (offset, source, ≤ 12 measurements, drift from two ≥ 0.5 day apart, 3 days for
hand settings, FT8 nudging). **AX.25** (`Ax25.*`, `ax25/`): up to 5 slicers (Dire Wolf style), DCD, HDLC with FCS
and byte-boundary closing flag, AX.25 address/control/PID, APRS (positions plain/compressed/Mic-E, weather,
messages/acks/bulletins, status, objects, items, telemetry, third-party), distance/bearing from a home.

## 9. Graphicx contract (as built)

The program relies on these Graphicx features, added for it and documented in Graphicx's README (MIT):
`terminal::Session::enableMouse/readEvent`, `terminal::parseEvent` (SGR 1006 mouse, modifiers, F-keys,
kitty `u` keys, UTF-8); `ui::Surface/Painter/Display` (only changed cells; since E11 only changed colors are
sent; synchronized output); `ui::DotCanvas`; `PixelMode` HalfBlocks/Quadrants/Braille/Ascii; `ui::App` (focus,
mouse capture, popups, `bindKey`, `onUnhandled`, `onLayout`, `autoFocus`, `clearFocus`); widgets `BigText`,
`RadioGroup`, `Led`, `Meter`, `Sparkline`, `Plot` (bands, markers, max decimation, `onMouse`), `Waterfall`
(`addRow`, markers, colormap), `ListBox`, `Slider`, `Toggle`, `Panel`, `Label`, `StatusBar`, `Dialog`, and
`color.h` themes/colormaps. A Graphicx change that alters these is a change to this program.

## 10. Validation evidence

All on 2026-11-25, macOS (Apple clang 21), unless stated.

| # | What | Result |
|---|---|---|
| E1 | Strict C++11 (D4) over the 22 sources of `src/`, `tools/`, `tests/`; `UsbLinux.cpp` and `AudioLinux.cpp` with `-D__linux__` against stub kernel headers | no error, no warning |
| E2 | `make check-static` | IOKit, CoreFoundation, AudioToolbox, libc++, libSystem only |
| E3 | `rtl-probe` on a Nooelec NESDR SMArt v5 | found `0bda:2838`, tuner R820T, 8,192,000 bytes in 2.02 s = 4.064 MB/s (expected 4.096), loss estimate 1 transfer (start-up timing) |
| E4 | Driver against a mock USB device (agent test) | tuner at 0x34; 2.048 MS/s exact; 100 MHz → 99,999,983 Hz, PLL locked; IF 3,569,993 Hz |
| E5 | Demo, headless | before the fix 1.02 MS/s (bytes passed as samples); after 2.046–2.053 MS/s |
| E6 | Demo 100.3 MHz, headless 6 s | −28 dBFS, SNR 31 dB, STEREO, RDS PI C201 "RADIO 1 " Pop Music, RadioText complete by t=3 s, DSP 5 % |
| E7 | Stick, 94.0 / 99.4 MHz, headless | GLOW FM (PI 8441, RT "Het ritme van de Brainport") and VERONICA (PI 83E1), STEREO, RDS groups ~11/s; with sound 20 s: 2.048 MS/s every second, 0 gaps (after the priming fix; 25 at start-up before it), 0 drops, sound delay 75–91 ms |
| E8 | Interface in a 160x46 pseudo-terminal, demo and stick, captured and rendered (tty2html) | every element of §8 present; zoom, tuning and help dialog respond; layout checked at cell level |
| E9 | RDS PI in the interface | was "PI 0000" (an integer through a `double` format), now "PI C201" |
| E10 | Layout | S-meter labels no longer overlap; list text clear of the scroll bar; clock fits |
| E11 | Terminal output, 5.6 s demo at 160x46 | 8.4 MB before; 2.9 MB truecolor / 2.6 MB with `--256` after (changed-colors-only SGR, waterfall 15 rows/s) |
| E12 | WAV of 6 s from VERONICA | 289,537 frames at 48 kHz (6.03 s), RMS L 0.103 R 0.103, L−R 0.074 (real stereo) |
| E13 | DSP self-test (`make test`, 82 checks, ~5 s) | **all pass** (also clean under ASan+UBSan): FFT tone at −0.00 dBFS, leakage < −147 dB; resampler 480,000–480,001 frames in 10 s at 2.048 and 2.4 MS/s; blocks of 0–13 samples equal one big block; FM demo: levels within 0.1 dB, pilot on the stereo stations only, RDS PS in 0.45–0.70 s, tones 1000.0/1500.0 Hz, stereo separation ~25.7 dB, PI/RT/PTY/clock exact; HF demo: AM/SAM/NFM tones 1000.0 Hz, CW 700.0 Hz, ≥ 99.7 % of SSB energy in 100–3000 Hz, opposite sideband rejected by 79.7 dB, squelched empty channel silent. Cost at 2.4 MS/s (Apple M2, `-O2`): WFM stereo+RDS 5.3 % of a core, NFM 2.2 %, AM 2.2 %, SAM 2.4 %, USB/LSB 2.0 %, CW 2.1 %; demo generator 5.0 % |
| E15 | Decoder self-tests | `test_decoders_fsk` 62/62 (overs separated by loud noise gaps decode one by one at 45.45/170 and 50/450, speed kept; CW 20/35 WPM within 2 %, tone ±0.4 Hz; RTTY 45.45/170 and 50/450 normal and reversed; NAVTEX exact with header; DSC 4/4 calls; reset; 20 s of noise gives no text; 48 and 12 kHz at 15 dB SNR in 3 kHz). `test_decoders_psk` 62/62 (PSK31/63/125 exact at 10 dB SNR on tone and 30 Hz off, ±50 Hz AFC, speed picked, mid-over start, speed change, NaN/Inf, 60 s of noise gives no text; Hell 59/59 letters, signal flag up in every over and never on noise) |
| E16 | Decoder CPU (Apple, 48 kHz audio) | CW ~1530×, RTTY ~800×, NAVTEX ~1670×, DSC ~1660×, PSK (3 speeds) ~768×, Hell ~667× real time |
| E17 | Demo band through the whole radio (IQ → receiver → decoder), headless 25–40 s | RTTY 50/450 at 7.355 MHz ✓; PSK31 14.071 MHz and PSK63 7.0425 MHz exact; NAVTEX 518 kHz the whole message with header; DSC 8414.5 kHz both calls (safety, distress "sinking", MMSIs); CW 7.275 MHz 20 WPM ("RQ CQ DE OFFGRID OFFGRID K": the first letter after each pause is lost, as in the JS); Hell the picture readable; RTTY 45.45/170 at 7.312 MHz: every over after the first exact ("UUQ DE …" on the first while the speed is learned, P14) after the P13 fix |
| E18 | Interface (pty capture, temporary config dir) | PSK decoder panel shows the text; `M` wrote `PSK31 demo\|14071000\|USB\|3000`, `l` wrote the label, `L` appended `…,14071000,PSK,-46.5,23.2,,nice signal`; PSK light on |
| E19 | Demo audio stations: a 1 kHz tone at 10.5 MHz received in USB and LSB | USB −32.0 dBFS, audio RMS 0.357; LSB −69.2 dBFS, RMS 0.0045 (37 dB) |
| E20 | Hell signal flag on the demo station through the radio | was "searching" (contrast 44 % < 0.45); now up 78 of 80 quarter-seconds (down only in the gaps' tails) |
| E21 | Calibration on the stick, 24 s per station | GLOW FM 94.0 MHz: +43.8 Hz → −0.47 ppm (steady from 4 s: −0.45…−0.49); VERONICA 99.4 MHz: +31.8 Hz → −0.32 ppm; agreement 0.15 ppm (station tolerance, multipath); the NESDR SMArt v5 has a 0.5 ppm TCXO |
| E22 | `example-fm-radio` (driver + DSP + audio libraries only) on the stick | GLOW FM in stereo with RDS text, through the speakers |
| E23 | Phase 3 self-tests | `test_decoders_picture` 38/38 (fax 60/120/240 LPM and SSTV ×6 at 48/12 kHz, ±100 ppm sender clock, 20 dB: MAE fax 10–13, SSTV 5–9.5 of 255, aligned; no picture from noise); `test_decoders_time` 128/128 (CHU, WWV, WWVH, RWM at 48/12 kHz with the PC clock −0.4/+2.3 s: offset error 0.0 ms frame-aligned, ≤ 2.75 ms over all alignments; WWV vs WWVH named; lost samples; no time from noise; ClockCorrection drift, save/load); `test_decoders_ax25` 60/60 (7 packets byte-exact at 48/44.1/22.05/12 kHz, ±2 % tone/clock, de-emphasis, 300 and 9600 Bd, bad FCS rejected, APRS fields, no frames from 60 s of noise; ASan/UBSan clean) |
| E24 | Phase 3 CPU (48 kHz) | fax ~1240×, SSTV ~2430×, time ~1050–1150×, AX.25 1200 Bd with 5 slicers ~107× real time |
| E25 | APRS demo station through the radio (NFM, 1200 Bd), 40 s | all 7 packets, fully interpreted (Mic-E car 61 km/h 135°, weather 12.2 °C 270° 19 km/h 1014.2 hPa 78 %, message, status, object, walker, balloon 9,510 m) |
| E26 | Time demo stations through the radio | CHU 7850 kHz: ticks, minute, full code "2026-11-25 13:02:37 UTC"; WWV 10 MHz: ticks, minute, 49 bits, full code "13:03:00 UTC" |
| E27 | Picture demo stations through the radio | fax 7880 kHz: a chart, then phasing of the next; SSTV 14.230 MHz Martin M1: picture 1 complete, picture 2 started |
| E28 | Time-stamping | before: the demo's time stations read the PC clock +0.13–0.15 s off (should be ~0). After stamping blocks on arrival and locking the demo stations to the clock: CHU +0.005 and +0.010 s over two minutes, WWV +0.005 s (full codes); the decoder alone, fed the encoders' audio with exact stamps at fractions 0/0.37/0.73: +0.000 s |
| E29 | Pictures saved headless from the demo band | `offgrid-SSTV-…-1.png` 320×256 (colour test card, no slant), `offgrid-FAX-…-1.png` 1809×198 (chart, straight); viewed |
| E30 | Live APRS on the stick, 144.800 MHz NFM | automatic gain: OVERLOAD at −30 dBFS, 0 frames in 3 min; fixed 28 dB: no overload, channel at the noise floor (−50 dBFS) all 5 min, 0 frames — nothing received (antenna / local traffic), so not a decoder result (P19) |
| E30b | Phase 4 self-tests | aviation 38/38 (ACARS 4 demo messages exact at 48/12 kHz, bad CRC rejected; VDL2 5 frames on 3 channels at once at 2.048/2.4 MS/s +250 Hz; 7 % of a core for 10 channels); AIS 50/50 (both channels at once, ±1 kHz, 98 % at 16 dB SNR, gpsd's AIVDM re-encoded byte-exact, 2 % of a core); FT8/FT4 156/156 (encoder identical to ft8_lib on 7 references, every message type, FT8 15/15 to −18 dB and FT4 8/8 to −12 dB with the PC clock ±2 s corrected, dt ≤ 10 ms; ~0.5 s a slot); WSPR 39/39 (K1ABC FN42 37 reference symbols identical, 4 stations to −26 dB, clock 40 s out found and measured to 0.02 s, stick error measured; ~0.7 s a slot). All suites: 715 checks pass |
| E31 | Phase 4 demo band through the radio | ACARS all messages; VDL2 3 channels with ACARS-over-AVLC, acks, ATN; AIS both channels, 6 vessels with every field; WSPR G4OGR and K1OGS at −22/−26 dB, dt +0.0 s, RF 14.097062/14.097131 MHz (after placing the demo 1 s into the slot and passing the dial); FT8 8–10 decodes a slot with countries and distances from JO21, dt +0.0 s; FT4 contest exchange; DSP thread 2–3 %, 0 gaps, 0 drops |
| E32 | Heard tab (pty capture) | "20 m: 7 stations, 7 countries" after a minute of FT8 demo |
| E33 | **Live on the stick** | **VDL2 136.975 MHz: a real frame, 486649 (Netherlands) → ground, PH-CDQ, flight CD040P, label H1** (3 min, gain 28 dB); ACARS 131.725 MHz: none in 3 min; AIS 162.000 MHz: none in 4 min (inland, indoor antenna) |
| E14 | Graphicx `make test` | 62 picture checks + 124 interface checks, 0 failures; `example-dashboard` driven in a pty (keys, mouse, theme change, menu, quit) |

## 11. Open problems

| # | Problem | Evidence / next step |
|---|---|---|
| P1 | **No Linux build has been run.** `UsbLinux.cpp` and `AudioLinux.cpp` were only syntax-checked against stub headers written from the kernel's definitions. | Build with `make` on Linux (glibc and musl), then `make check-static`, `bin/rtl-probe`, `bin/offgrid --headless 10`. |
| P2 | Static glibc and `getaddrinfo`: a host *name* for `--tcp` needs glibc's NSS libraries at run time. | Use an IP address, or build with musl. |
| P3 | R828D crystal: webrtlsdr uses 28.8 MHz for every R828D; librtlsdr uses 16 MHz on non-V4 R828D boards (e.g. Astrometa), which would mistune. | Test a Blog V4 and an R828D board; port librtlsdr's crystal choice if needed. |
| P4 | Control transfers are retried on every failure (webrtlsdr only on stalls); probing 0x34 on an R828D costs ~150 ms at open. | Cosmetic. |
| P5 | rtl_tcp not tested against a server; its gain list is the R820T table. | Test with `rtl_tcp`. |
| P6 | Tuner IF bandwidth not set from the sample rate (as in webrtlsdr). | Port librtlsdr's `r82xx_set_bandwidth`. |
| P7 | RDS sync in the browser version: its RdsDecoder counts repaired blocks as good when deciding to drop sync, so after a false sync (about a third of random words look repairable) it can stay on the wrong bit. Fixed here (clean = 2, repaired = 1, bad = 0; drop below an average of 0.7). | Report the fix upstream. |
| P11 | `noiseDbfs()` in WFM reads a few dB high on strong stations (its own outer sidebands fall in the measured bins). | Fine for the SNR display; refine with a wider exclusion. |
| P12 | No GCC build yet of the DSP (clang only). | Part of P1. |
| P8 | The terminal sends ~0.5 MB/s at 160x46 truecolor. | Fine locally and on a LAN; for slow links `--256`, a smaller window; a "low bandwidth" mode (slower waterfall, 15 fps) is a candidate. |
| P9 | `Plot` y labels are a fixed 7 columns; the waterfall leaves that gutter empty. | Cosmetic. |
| P10 | ~~No settings file~~ | Fixed in phase 2 (D13). |
| P13 | ~~RTTY re-locked at 75 Bd after a gap of AGC-amplified noise~~ (the JS kept measuring pulse lengths on noise). Fixed: the pair counts only while the two tone filters hold ≥ 25 % of the power; history cleared when it goes; speed changes need two agreeing measurements; characters read from the start of an over on the known tones; start bits need 0.8 bit of mark before; tone moves smoothed. | Fixed 2026-11-25 (E15, E17). |
| P14 | CW and RTTY lose the first characters after a pause (speed learning), as the JS does. | By design for now; a look-back re-decode is possible. |
| P19 | No live APRS frame yet: none heard in 5 min on 144.800 MHz with the stick's antenna; automatic gain overloads there. | Outdoor VHF antenna; fixed gain (`-g 28`). |
| P20 | The time decoder's first "second ticks found" estimate through the radio can be off by up to ~0.4 s (AGC settling), and once WWV's minute marker came out 21 s off; the full time codes were right (E28). T only accepts a full code. | Hold the tick report until the tick profile is steady. |
| P21 | ACARS and AIS not yet received live (none in range in 3–4 min). | Outdoor antenna; a recording. |
| P22 | The Heard tab's columns are cut in the 34-column side panel. | A wider popup view. |
| P23 | FT4 reaches about −14 dB against WSJT-X's −17.5 dB; WSPR misses drifts beyond ±2 Hz (as the JS). | Tune the FT4 sync; a drift search. |
| P16 | AX25Toolkit has no LICENSE file; its code is used with its author's permission (D15). | Add a license to AX25Toolkit (GPL-2.0-or-later or compatible). |
| P17 | AX.25 9600 Bd needs flat discriminator audio at ≥ 38.4 kHz; the receiver's NFM audio is de-emphasised and 3 kHz wide. | A flat FM tap in the Receiver. |
| P18 | Time decoding was not tried on real HF signals (the stick has no HF input). | Test with direct sampling or an upconverter. |
| P15 | Calibration needs ≥ 10 dB SNR and several seconds; FM stations can be a few tens of Hz off (0.3 ppm at 100 MHz). | Average two stations; a time signal (WWV 10 MHz) needs HF. |

## 12. Plan: the next phases

| Phase | Content |
|---|---|
| 2 | **Built (2026-11-25):** text decoders on the audio: CW, RTTY, PSK31/63/125, NAVTEX, DSC, Hellschreiber; memories, logbook, labels, settings file; scan; frequency calibration; the code split into reusable libraries |
| 3 | **Built (2026-11-25):** weather fax, SSTV, time signals (CHU, WWV, RWM) and the clock; AX.25/APRS (from phase 4); full mouse use |
| 4 | **Built (2026-11-25):** ACARS, VDL2, AIS, FT8/FT4/WSPR with the Heard list (the band survey is left for phase 5) |
| 5 | Digital broadcast: DAB/DAB+ and DRM (AAC/xHE-AAC in C++11), AUTO mode, Signal Monitor, waterfall replay, SDRconnect |

## 13. Glossary

**IQ**: complex baseband samples (in-phase, quadrature). **u8 IQ**: the stick's format, unsigned bytes I, Q
around 127.5. **dBFS**: decibels relative to full scale. **Span**: the bandwidth seen at once, equal to the sample
rate. **Dial**: the tuned frequency. **Centre**: the stick's tuned frequency (middle of the span). **DC spike**:
the stick's artefact at the centre. **WFM/NFM**: wide/narrow FM. **SAM**: synchronous AM. **SSB**: USB/LSB.
**RDS**: Radio Data System (PI programme id, PS station name, RT radio text, PTY programme type). **Pilot**: the
19 kHz tone that marks FM stereo. **usbfs**: Linux's user-space USB interface. **URB**: a USB request block.
**ALSA**: Linux's sound interface. **Cell**: one character position in the terminal. **Braille / quadrant / half
block**: Unicode characters used to draw 2x4, 2x2 and 1x2 pixels per cell.
