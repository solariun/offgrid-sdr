# OFFgrid-SDR in the terminal (`offgrid`)

A C++11 port of OFFgrid-SDR that runs in a text terminal on **Linux and macOS**:
one static binary, its own RTL-SDR driver (no librtlsdr, no libusb), and a full
mouse-and-keyboard interface drawn with [Graphicx](https://github.com/solariun/Graphicx).

```
 █▀█ █▀█ █▀█ ▀▀█   WFM  NFM  AM  SAM  USB  LSB  CW           S ▮▮▮▮▮▮▮▮▮▮▮▮▮▮┃▯▯▯ -16.0 dB SNR 13
 █▄█ █▄█ █▄█ ▄█    FM broadcast   step 100 kHz   filter 150 kHz       -100 -80  -60 -40
              MHz  ● ST  ● RDS  ○ SQL  ○ OVL  ○ REC  ○ MUTE                         11:42:59 UTC
 -32 ┊        ⣰⣶⣦⡀          ┊          ⢀⣴⣦            ┊ ╭ Bands ─────────────╮
     ┊      ⣴⣿⣿⣿⣿⣦          ┊         ⣰⣿⣿⣿⡄           ┊ │ FM broadcast  Broad│
 -72 ⣀⣀⣀⣀⣠⣴⣿⣿⣿⣿⣿⣿⣷⣄⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣠⣾⣿⣿⣿⣿⣿⣄⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀ │ LW            Broad│
     98.76      99.10      99.45      99.79      100.13 │ ...                 │
     ▀▀▀▀▀▀▀▀▀▀▀▀ waterfall (colormap, 15 rows/s) ▀▀▀▀▀▀ ╰─────────────────────╯
 RDS  VERONICA   PI 83E1   Pop Music   "VERONICA"
 NESDR SMArt v5 (R820T)  2.048 MS/s  DSP 5%  audio CoreAudio 80 ms        F1 help  q quit
```

## What it does (phases 1 to 4)

| | |
|---|---|
| **Sources** | RTL-SDR sticks (R820T, R820T2, R828D, RTL-SDR Blog V4) through our own driver; `rtl_tcp` servers; raw IQ recordings (`.cu8`, rtl_sdr's output); a demo band with simulated stations |
| **Modes** | WFM with stereo and **RDS** (station name, programme type, radio text, clock), NFM, AM, SAM (synchronous AM), USB, LSB, CW |
| **Display** | big frequency, mode buttons, status lights (stereo, RDS, squelch, overload, recording, mute), S-meter with peak hold and level history, spectrum (braille plot with the passband shaded), waterfall (half blocks, 9 colormaps), band list, controls, RDS line, status line |
| **Controls** | tuning by step / typed frequency / mouse click, step sizes 1 Hz to 1 MHz, 41 bands, volume, mute, squelch, RF gain or auto, filter width, zoom x1-x32, waterfall floor and range (auto floor), stereo/mono, bias-tee, direct sampling, ppm correction |
| **Decoders** | CW (speed learned), RTTY (shift, speed and polarity found), PSK31/63/125 (speed picked, AFC), NAVTEX (whole messages), DSC (ships' calls), Hellschreiber (picture); in a panel under the waterfall, or printed with `--headless` |
| **Pictures** | weather fax (60–240 LPM, automatic slant) and SSTV (Martin M1/M2, Scottie S1/S2, PD90, PD120) drawn in the panel as they arrive and saved as PNG (`offgrid-FAX-…png`, `offgrid-SSTV-…png`) when each one ends |
| **Time and clock** | CHU, WWV/WWVH and RWM: the time and the computer clock's error (±2.5 ms); **T** (or *Set clock*) corrects the clock shown in the header (marked •), kept with its learned drift in `clock.txt` |
| **AX.25 / APRS** | packet radio at 1200 Bd (VHF, e.g. APRS 144.800 / 144.390 MHz in NFM), 300 Bd (HF) and 9600 Bd: every frame with its path, and APRS positions (also Mic-E and compressed), weather, messages, status, objects, items and telemetry in plain words; built on [AX25Toolkit](https://github.com/solariun/AX25Toolkit) |
| **Aviation** | ACARS (AM, 131.550 / 131.725 MHz …) and VDL2 (every channel 136.6–137 MHz at once): registrations, flights, labels, message text, aircraft countries |
| **Ships** | AIS on both channels at once (dial 162.000 MHz): positions, names, call signs, destinations, ETAs, sizes; `!AIVDM` sentences |
| **FT8, FT4, WSPR** | the weak-signal modes, by the corrected clock (their timings also keep the clock right): every message, caller's country, distance and bearing (`--locator`), WSPR power and the stick's frequency error; the **Heard** tab sums up who you hear, band by band |
| **Mouse** | everything: a toolbar under the header for every action (Decoder ▾, Memory +, Label, Log, Scan, Scan mem, Calibrate, Record, Save picture, Set clock, Theme, Help, Quit), OK / Cancel in prompts, the tabs, lists, sliders and check boxes; click the frequency to type one, the wheel over it tunes; click on the spectrum or waterfall to tune; click the decoder panel's title to change decoder |
| **Memories, labels, log** | named memories (frequency, mode, filter); your own labels drawn on the spectrum; a logbook (`logbook.csv`: UTC time, frequency, mode, level, SNR, station, notes) |
| **Scan** | the current band in its channel steps, or the memories; stops on a signal, goes on 2 s after it ends |
| **Calibration** | K on a station of known frequency (FM broadcast, a time signal): the stick's error in ppm is measured and corrected |
| **Settings** | kept between runs in `~/.config/offgrid/settings.ini` (`--fresh` ignores them) |
| **Recording** | the sound to a 16-bit stereo WAV (R key or `--wav`) |
| **Themes** | amber (the browser version's default), orange, lime, green, cyan, vfd, blue, violet, rose, mono, light |

Not yet ported (next phase): DAB/DAB+, DRM, AUTO mode, the waterfall
replay, the Signal Monitor and SDRconnect (phase 5).

### Decoders

Press **d** and pick one (or `--decoder NAME`). As in the browser version, the dial
then shows the **signal's own frequency**: type the listed frequency (14.071 for
PSK31, 518k for NAVTEX, 8414.5k for DSC) and the receiver sits below it in USB so the
tones land where the decoder listens (1.5 kHz for RTTY/NAVTEX/DSC, 1 kHz for PSK and
Hell; CW uses the CW mode). A green marker on the spectrum shows where the decoder
expects the signal. **D** switches the decoder off.

Try them with no hardware: `bin/offgrid --demo -f 7.355M --decoder rtty` (also
7.312M RTTY, 7.275M CW, 14.071M PSK31, 7.0425M PSK63, 518k NAVTEX, 8414.5k DSC,
14.0635M Hell, 7.880M fax, 14.230M SSTV, 7.850M CHU and 10M WWV time signals,
144.800M APRS, 131.55M ACARS, 136.8M VDL2, 162M AIS, 14.074M FT8, 14.080M FT4,
14.0956M WSPR).

Decoders run on a thread of their own, so a heavy decode (an FT8 or WSPR slot,
up to about a second of work) never interrupts the sound or the display.

Each decoder sets the radio the way the browser version does (`src/app/DecoderPlan.h`):
FAX USB 1.9 kHz below; SSTV on the frequency in USB (LSB below 10 MHz, NFM above
30 MHz); TIME USB 1 kHz below, the station picked from the frequency; AX.25 in NFM
at 1200 Bd above 30 MHz, in USB at 300 Bd below.

### Files

| File | Contents |
|---|---|
| `~/.config/offgrid/settings.ini` | the last frequency, mode, volume, theme, ppm …, the memories, the labels, each band's last frequency (plain text; written on exit) |
| `~/.config/offgrid/logbook.csv` | the log, one line per entry, for any spreadsheet |
| `~/.config/offgrid/clock.txt` | the clock correction, its source and history (the PC clock's drift) |
| `offgrid-FAX-…png`, `offgrid-SSTV-…png`, `offgrid-…wav` | pictures and recordings, in the directory offgrid was started from |

`$XDG_CONFIG_HOME` moves them, and `OFFGRID_CONFIG=dir` overrides the directory.

## Build

```bash
git clone --recursive https://github.com/solariun/offgrid-sdr.git
cd offgrid-sdr
make
```

Only a C++11 compiler and `make` are needed: no libraries.

- **Linux:** the binary is **fully static** (`-static`): copy `bin/offgrid` to any
  Linux of the same CPU architecture. USB uses the kernel's usbfs and sound the
  kernel's ALSA interface directly. Needs the kernel headers that come with every
  compiler setup (`linux-libc-dev` on Debian/Ubuntu, `linux-headers` on Alpine).
  `make STATIC=0` links dynamically. A glibc static build warns that `getaddrinfo`
  needs glibc's NSS at run time: that only affects `--tcp` with a host *name*
  (an IP address always works); building with musl (Alpine) avoids it.
- **macOS:** Apple does not ship a static C library, so no Mac program can be
  100% static. `bin/offgrid` links only libraries that are part of macOS itself
  (libSystem, libc++, IOKit, CoreFoundation, AudioToolbox); nothing to install.
  `make check-static` shows the list.

`make tools` builds `bin/rtl-probe` (finds a stick, streams 2 s, reports the
rate); `make test` runs every test (DSP and decoders).

### Linux: give your user the stick

```bash
echo 'blacklist dvb_usb_rtl28xxu' | sudo tee /etc/modprobe.d/rtl-sdr-blacklist.conf
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="2838", MODE="0666"' | sudo tee /etc/udev/rules.d/20-rtlsdr.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then unplug and replug the stick. (`offgrid` also detaches the TV driver by itself
when it can.) Sound: `offgrid` opens the ALSA device directly; when PipeWire or
PulseAudio already owns it (the usual desktop), it plays through `pw-cat`,
`pacat` or `aplay` instead. `--audio hw:1,0` or `--audio pipe:COMMAND` chooses.

## Use

```bash
bin/offgrid                          # first stick, 100 MHz WFM
bin/offgrid -f 94.0 --theme vfd      # a station, another look
bin/offgrid -f 7.1M -m lsb --direct 2   # 40 m on a stick without the V4's HF input
bin/offgrid --tcp 192.168.1.20:1234  # an rtl_tcp server
bin/offgrid --file capture.cu8 --center 145.8M --rate 2.048
bin/offgrid --demo                   # no hardware: simulated stations
bin/offgrid --headless 10 -f 94.0    # no interface: one status line per second
```

`offgrid --help` lists every option.

### Keys

| Key | Does |
|-----|------|
| ← / → | tune one step (Shift: ten) |
| ↑ / ↓ | larger / smaller step |
| PgUp / PgDn | ±1 MHz |
| Enter or f | type a frequency (`94.0`, `7074k`, `145.8M`, `1090 MHz`) |
| 1 … 7, m | WFM, NFM, AM, SAM, USB, LSB, CW; next mode |
| b | the side list: Bands, Memories, Log tabs (← → change tab, ↑ ↓ Enter) |
| Tab / Esc | move through the side controls / give the keys back to tuning |
| + / -, space | volume, mute |
| g / G, a | RF gain down / up, automatic gain |
| s / S | squelch down / up |
| w / W | filter narrower / wider |
| z / Z | zoom the spectrum and waterfall in / out |
| [ ] { } | waterfall floor, waterfall range |
| t, c | next theme, next waterfall colors |
| R | record the sound to `offgrid-DATE-TIME.wav` |
| d / D | decoder menu / decoder off |
| M, Del | save a memory here (named after the RDS station when there is one) / delete the selected memory |
| l | label this frequency on the spectrum (empty text removes it) |
| L | log what you hear (with your notes) |
| n / N | scan this band / the memories (Esc or any tuning key stops) |
| K | calibrate on a station of known frequency |
| T | set the clock from the time decoder's last full time code |
| F1, h, ? | help |
| q | quit |

**Mouse:** click or drag on the spectrum or waterfall to tune; the wheel steps;
every control on the right (lists, sliders, check boxes) works with the mouse.
`--no-mouse` leaves the mouse to the terminal (text selection).

### Over SSH

Everything is text, so it works over SSH. With a slow link use `--256` (shorter
color codes) and a smaller terminal window; the waterfall scrolls at 15 rows a
second and only changed cells are sent (about 0.5 MB/s at 160x46 in truecolor).

## Reusable libraries

The program is four static libraries plus its interface; each library works on its
own (no dependency on the program, the terminal or each other, except that the
decoders use the DSP library's FFT). `make libs` builds them, `make install
PREFIX=…` installs them with their headers, and each directory has a README:

| Library | What it is |
|---|---|
| [`liboffgrid-rtlsdr`](src/rtlsdr/README.md) | the RTL-SDR driver (no libusb) |
| [`liboffgrid-dsp`](src/dsp/README.md) | demodulators, RDS, spectrum, demo band |
| [`liboffgrid-decoders`](src/decoders/README.md) | CW, RTTY, PSK, NAVTEX, DSC, Hell, weather fax, SSTV, time signals (+ clock correction), AX.25/APRS, ACARS, VDL2, AIS, FT8, FT4, WSPR, callsign/locator tools, the Heard list, and their signal generators |
| [`liboffgrid-audio`](src/audio/README.md) | sound output (no sound libraries) |

`make examples` builds two small programs that use them: `example-fm-radio` (an FM
radio with RDS in ~100 lines: driver + DSP + audio) and `example-decode` (decode a
WAV file with any decoder).

## Layout of the code

```
src/usb/      USB without libusb: UsbLinux.cpp (usbfs ioctls), UsbMac.cpp (IOKit)
src/rtlsdr/   the RTL-SDR driver: Rtl2832u (demodulator chip), R82xx (tuners), RtlSdr (public class)
src/dsp/      signal processing: FFT, filters, demodulators, RDS, spectrum, demo band
src/decoders/ decoders: Cw, Rtty, Psk, Navtex (+DSC), Hell, Fax, Sstv, Time (+Clock), Ax25 (+APRS;
              ax25/ = AX25Toolkit's modem, HDLC, frames); encoders for tests and the demo
src/audio/    sound out: AudioMac.cpp (AudioQueue), AudioLinux.cpp (ALSA ioctls / pipe)
src/app/      the program: Source (stick, rtl_tcp, file, demo), Radio (engine), Ui, Store (settings, memories,
              labels, log), Bands, Options, main
tools/        rtl-probe
examples/     fm-radio.cpp, decode.cpp (the libraries on their own)
tests/        test_dsp.cpp, test_decoders_{fsk,psk,picture,time,ax25}.cpp
external/Graphicx   the terminal graphics and widget library (git submodule)
spec.md       the specification: requirements, design, decisions, evidence, open problems
```

## Credits and license

The terminal port is free software under the **GNU GPL, version 3 or later**,
like OFFgrid-SDR itself. It ports OFFgrid-SDR by diddy-boy (the RDS decoder, the
SAM demodulator, the band plan, the look); the RTL-SDR driver follows
[webrtlsdr](https://github.com/jtarrio/webrtlsdr) and
[signals](https://github.com/jtarrio/signals) by Jacobo Tarrio (Apache-2.0),
which follow librtlsdr (GPL-2.0-or-later). Graphicx is MIT.
