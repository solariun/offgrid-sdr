# liboffgrid-rtlsdr — an RTL-SDR driver in C++11, without libusb

Drives RTL2832U sticks with R820T, R820T2 and R828D tuners (RTL-SDR Blog V3/V4,
Nooelec NESDR and the like) straight through the operating system:

- **Linux:** the kernel's usbfs (`/dev/bus/usb`), so it works in a fully static
  binary; detaches the `dvb_usb_rtl28xxu` TV driver and gives it back on close.
- **macOS:** IOKit (part of the OS).

Ported from [webrtlsdr](https://github.com/jtarrio/webrtlsdr) (Apache-2.0), the
driver of OFFgrid-SDR's browser version, which follows librtlsdr.

## Files

| File | |
|---|---|
| `rtlsdr/RtlSdr.h` | **the public class** |
| `usb/UsbDevice.h` | the small USB interface it needs (also usable alone) |
| `rtlsdr/Rtl2832u.*`, `rtlsdr/R82xx.*`, `usb/UsbLinux.cpp`, `usb/UsbMac.cpp` | internals |

## Use

```cpp
#include "rtlsdr/RtlSdr.h"

std::vector<usb::DeviceInfo> sticks = rtlsdr::RtlSdr::find();
rtlsdr::RtlSdr stick;
if (sticks.empty() || !stick.open(sticks[0])) { /* stick.error() says why */ }
stick.setSampleRate(2048000);
stick.setCenterFrequency(100000000);   // Hz
stick.setAutoGain();                   // or stick.setGain(30.0) (nearest of stick.gains())
stick.start([](const uint8_t* iq, size_t bytes) {
    // unsigned 8-bit I, Q, I, Q ... (127.5 = zero), on the USB thread:
    // copy it somewhere and return quickly.
});
...
stick.stop();
```

Also: `setFrequencyCorrection(ppm)`, `setDirectSampling(0|1|2)` (HF below 28.8 MHz
on sticks without an HF input), `setBiasTee(bool)`, `tunerName()`, `isBlogV4()`.
Every call returns `false` with `error()` on failure; it never throws.

## Build and link

`make libs` builds `build/lib/liboffgrid-rtlsdr.a`. Link it with
`-framework IOKit -framework CoreFoundation` on macOS, `-pthread` on Linux.
`examples/fm-radio.cpp` and `tools/rtl-probe.cpp` use it. On Linux, give your user
the stick with a udev rule (see TUI.md).
