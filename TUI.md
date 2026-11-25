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

## What it does (phase 1)

| | |
|---|---|
| **Sources** | RTL-SDR sticks (R820T, R820T2, R828D, RTL-SDR Blog V4) through our own driver; `rtl_tcp` servers; raw IQ recordings (`.cu8`, rtl_sdr's output); a demo band with simulated stations |
| **Modes** | WFM with stereo and **RDS** (station name, programme type, radio text, clock), NFM, AM, SAM (synchronous AM), USB, LSB, CW |
| **Display** | big frequency, mode buttons, status lights (stereo, RDS, squelch, overload, recording, mute), S-meter with peak hold and level history, spectrum (braille plot with the passband shaded), waterfall (half blocks, 9 colormaps), band list, controls, RDS line, status line |
| **Controls** | tuning by step / typed frequency / mouse click, step sizes 1 Hz to 1 MHz, 41 bands, volume, mute, squelch, RF gain or auto, filter width, zoom x1-x32, waterfall floor and range (auto floor), stereo/mono, bias-tee, direct sampling, ppm correction |
| **Recording** | the sound to a 16-bit stereo WAV (R key or `--wav`) |
| **Themes** | amber (the browser version's default), orange, lime, green, cyan, vfd, blue, violet, rose, mono, light |

Not yet ported (next phases): the decoders (CW, RTTY, PSK, NAVTEX, DSC, weather fax,
SSTV, ACARS, APRS, AIS, FT8/FT4/WSPR, time signals), DAB/DAB+ and DRM, AUTO mode,
the waterfall replay, the Signal Monitor, memories and the logbook, SDRconnect.

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
rate); `make test` runs the DSP tests.

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
| b | the band list (↑ ↓ Enter) |
| Tab / Esc | move through the side controls / give the keys back to tuning |
| + / -, space | volume, mute |
| g / G, a | RF gain down / up, automatic gain |
| s / S | squelch down / up |
| w / W | filter narrower / wider |
| z / Z | zoom the spectrum and waterfall in / out |
| [ ] { } | waterfall floor, waterfall range |
| t, c | next theme, next waterfall colors |
| R | record the sound to `offgrid-DATE-TIME.wav` |
| F1, h, ? | help |
| q | quit |

**Mouse:** click or drag on the spectrum or waterfall to tune; the wheel steps;
every control on the right (lists, sliders, check boxes) works with the mouse.
`--no-mouse` leaves the mouse to the terminal (text selection).

### Over SSH

Everything is text, so it works over SSH. With a slow link use `--256` (shorter
color codes) and a smaller terminal window; the waterfall scrolls at 15 rows a
second and only changed cells are sent (about 0.5 MB/s at 160x46 in truecolor).

## Layout of the code

```
src/usb/      USB without libusb: UsbLinux.cpp (usbfs ioctls), UsbMac.cpp (IOKit)
src/rtlsdr/   the RTL-SDR driver: Rtl2832u (demodulator chip), R82xx (tuners), RtlSdr (public class)
src/dsp/      signal processing: FFT, filters, demodulators, RDS, spectrum, demo band
src/audio/    sound out: AudioMac.cpp (AudioQueue), AudioLinux.cpp (ALSA ioctls / pipe)
src/app/      the program: Source (stick, rtl_tcp, file, demo), Radio (engine), Ui, Bands, Options, main
tools/        rtl-probe
tests/        test_dsp.cpp
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
