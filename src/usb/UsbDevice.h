// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/usb/UsbDevice.h
//  Layer:  USB access (no libusb: talks to the operating system directly)
//
//  What this file does
//  -------------------
//  Declares usb::Device, the small piece of USB the RTL-SDR driver needs:
//  finding a device by vendor/product id, vendor control transfers, and a
//  continuous stream of bulk-IN transfers delivered to a callback.
//
//  How it fits in
//  --------------
//  Two implementations, chosen at compile time:
//    UsbLinux.cpp   Linux usbfs (/dev/bus/usb/BBB/DDD + ioctl), the same
//                   kernel interface libusb uses; works in a fully static
//                   binary.  Detaches the dvb_usb_rtl28xxu TV driver.
//    UsbMac.cpp     macOS IOKit (IOUSBDeviceInterface / IOUSBInterface).
//  rtlsdr::RtlSdr (src/rtlsdr) is the only user.
//
//  Contributing
//  ------------
//  Keep this interface minimal and blocking-free for the stream: the
//  callback runs on the stream's own thread and must return quickly.
// ============================================================================
#ifndef OFFGRID_USB_USBDEVICE_H
#define OFFGRID_USB_USBDEVICE_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace usb {

// One USB device found on the bus.
struct DeviceInfo {
    std::uint16_t vendorId;
    std::uint16_t productId;
    std::string manufacturer;  // string descriptors (may be empty)
    std::string product;
    std::string serial;
    std::string path;          // OS-specific address used by open()
};

// Called with each completed bulk transfer (on the stream thread).
typedef std::function<void(const std::uint8_t* data, std::size_t length)> BulkCallback;

class Device {
public:
    virtual ~Device() {}

    // Every device whose (vendor, product) pair is in `ids`.
    static std::vector<DeviceInfo> find(const std::vector<std::pair<std::uint16_t, std::uint16_t> >& ids);
    // Opens the device and claims `interfaceNumber` (detaching a kernel
    // driver first where the OS needs it).  nullptr + error text on failure.
    static std::unique_ptr<Device> open(const DeviceInfo& info, int interfaceNumber, std::string& error);

    // Control transfer.  requestType bit 7 set = device-to-host (IN).
    // Returns the number of bytes transferred, or -1 on error.
    virtual int control(std::uint8_t requestType, std::uint8_t request, std::uint16_t value, std::uint16_t index,
                        std::uint8_t* data, std::uint16_t length, unsigned timeoutMs) = 0;

    // Synchronous bulk IN read (used to flush the endpoint).  Returns bytes
    // read or -1.
    virtual int bulkRead(std::uint8_t endpoint, std::uint8_t* data, std::size_t length, unsigned timeoutMs) = 0;

    // Starts `transfers` concurrent bulk-IN transfers of `transferSize`
    // bytes each, resubmitted as they complete, and calls `callback` for
    // every completed one, in order, from a thread of its own.
    virtual bool startStream(std::uint8_t endpoint, std::size_t transferSize, int transfers, BulkCallback callback) = 0;
    // Stops the stream and waits for its thread.  Safe to call twice.
    virtual void stopStream() = 0;

    // Last error text (for messages).
    virtual std::string lastError() const = 0;
};

}  // namespace usb

#endif
