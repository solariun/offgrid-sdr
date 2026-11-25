// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/usb/UsbMac.cpp
//  Layer:  USB access (no libusb: talks to the operating system directly)
//
//  What this file does
//  -------------------
//  Implements usb::Device on macOS with the IOKit USB "device interface"
//  and "interface interface" COM-style plug-ins (IOUSBLib.h), which is what
//  libusb's darwin backend uses underneath.
//
//    find()         IOServiceGetMatchingServices with a matching dictionary
//                   per (idVendor, idProduct); reads the string properties
//                   and the "locationID" (used as DeviceInfo::path).
//    open()         IOCreatePlugInInterfaceForService + QueryInterface for
//                   IOUSBDeviceInterface500, USBDeviceOpen, SetConfiguration
//                   if the device is unconfigured, finds the interface by
//                   number, USBInterfaceOpen, maps endpoint -> pipeRef.
//    control()      DeviceRequestTO on the default pipe.
//    bulkRead()     ReadPipeTO (synchronous).
//    startStream()  A thread with its own CFRunLoop holding the interface's
//                   async event source; N ReadPipeAsync requests in flight,
//                   each resubmitted from its completion callback.  With
//                   16 x 64 KiB queued the host controller always has a
//                   buffer ready, so 2.4 MS/s (4.8 MB/s) does not drop data.
//
//  How it fits in
//  --------------
//  Selected at compile time (#ifdef __APPLE__); UsbLinux.cpp is the Linux
//  twin.  Links only the system frameworks IOKit and CoreFoundation.
//  macOS has no kernel driver for RTL2832U sticks, so nothing needs to be
//  detached and no permission setup is needed.
//
//  Contributing
//  ------------
//  * All async completions run on the stream thread's run loop, so the
//    callback, resubmission and the in-flight counter need no locking.
//  * The pipe is aborted from the stream thread itself (AbortPipe), then the
//    loop keeps running until every aborted request has called back; only
//    then may the buffers be freed.
// ============================================================================
#ifdef __APPLE__

#include "UsbDevice.h"

#include <atomic>
#include <cstdio>
#include <mutex>
#include <thread>

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/usb/IOUSBLib.h>

namespace usb {

namespace {

typedef IOUSBDeviceInterface500 UsbDeviceIf;      // DeviceRequestTO, USBDeviceOpen...
typedef IOUSBInterfaceInterface500 UsbInterfaceIf;  // ReadPipeTO, ReadPipeAsync...

// IOReturn -> readable text with the hex code.
std::string ioText(IOReturn code) {
    const char* name = "error";
    switch (code) {
        case kIOReturnExclusiveAccess: name = "exclusive access (device in use by another program)"; break;
        case kIOReturnNoDevice: name = "no such device (unplugged?)"; break;
        case kIOReturnNotResponding: name = "device not responding"; break;
        case kIOReturnNotOpen: name = "device not open"; break;
        case kIOReturnAborted: name = "aborted"; break;
        case kIOReturnNotPrivileged: name = "not privileged"; break;
        case kIOUSBPipeStalled: name = "pipe stalled"; break;
        case kIOUSBTransactionTimeout: name = "timed out"; break;
        default: break;
    }
    char buffer[96];
    std::snprintf(buffer, sizeof buffer, "%s (0x%08x)", name, static_cast<unsigned>(code));
    return buffer;
}

// Reads a CFString registry property of `service` as UTF-8 ("" if absent).
std::string stringProperty(io_service_t service, const char* key) {
    std::string result;
    CFStringRef cfKey = CFStringCreateWithCString(kCFAllocatorDefault, key, kCFStringEncodingUTF8);
    CFTypeRef value = IORegistryEntryCreateCFProperty(service, cfKey, kCFAllocatorDefault, 0);
    CFRelease(cfKey);
    if (value) {
        if (CFGetTypeID(value) == CFStringGetTypeID()) {
            char buffer[256];
            if (CFStringGetCString(static_cast<CFStringRef>(value), buffer, sizeof buffer, kCFStringEncodingUTF8)) {
                result = buffer;
            }
        }
        CFRelease(value);
    }
    return result;
}

// Reads a CFNumber registry property of `service` (false if absent).
bool numberProperty(io_service_t service, const char* key, long long& out) {
    bool ok = false;
    CFStringRef cfKey = CFStringCreateWithCString(kCFAllocatorDefault, key, kCFStringEncodingUTF8);
    CFTypeRef value = IORegistryEntryCreateCFProperty(service, cfKey, kCFAllocatorDefault, 0);
    CFRelease(cfKey);
    if (value) {
        if (CFGetTypeID(value) == CFNumberGetTypeID()) {
            ok = CFNumberGetValue(static_cast<CFNumberRef>(value), kCFNumberLongLongType, &out);
        }
        CFRelease(value);
    }
    return ok;
}

// Puts an integer into a matching dictionary.
void setDictNumber(CFMutableDictionaryRef dict, const char* key, int value) {
    CFStringRef cfKey = CFStringCreateWithCString(kCFAllocatorDefault, key, kCFStringEncodingUTF8);
    CFNumberRef number = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &value);
    CFDictionarySetValue(dict, cfKey, number);
    CFRelease(number);
    CFRelease(cfKey);
}

// Every IOKit service of class `className` with this vendor/product id.
// The caller must IOObjectRelease() each returned service.
std::vector<io_service_t> matchServices(const char* className, std::uint16_t vid, std::uint16_t pid) {
    std::vector<io_service_t> services;
    CFMutableDictionaryRef match = IOServiceMatching(className);
    if (!match) return services;
    setDictNumber(match, kUSBVendorID, vid);
    setDictNumber(match, kUSBProductID, pid);
    io_iterator_t iterator = 0;
    // MACH_PORT_NULL = "the default main port" (avoids the deprecated
    // kIOMasterPortDefault / the 12.0+-only kIOMainPortDefault).
    // IOServiceGetMatchingServices consumes the dictionary reference.
    if (IOServiceGetMatchingServices(MACH_PORT_NULL, match, &iterator) != KERN_SUCCESS) return services;
    while (io_service_t service = IOIteratorNext(iterator)) services.push_back(service);
    IOObjectRelease(iterator);
    return services;
}

// Modern macOS (10.11+) publishes devices as IOUSBHostDevice; older ones
// as IOUSBDevice.  Try the new class first.
std::vector<io_service_t> matchDevices(std::uint16_t vid, std::uint16_t pid) {
    std::vector<io_service_t> services = matchServices("IOUSBHostDevice", vid, pid);
    if (services.empty()) services = matchServices(kIOUSBDeviceClassName, vid, pid);
    return services;
}

std::string locationText(io_service_t service) {
    long long location = 0;
    if (!numberProperty(service, kUSBDevicePropertyLocationID, location)) return std::string();
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "0x%08llx", static_cast<unsigned long long>(location));
    return buffer;
}

// ---------------------------------------------------------------------------
//  MacDevice: an opened device + one opened interface.
// ---------------------------------------------------------------------------
class MacDevice : public Device {
public:
    MacDevice(UsbDeviceIf** dev, bool deviceOpen, UsbInterfaceIf** intf, CFRunLoopSourceRef source)
        : dev_(dev), deviceOpen_(deviceOpen), intf_(intf), source_(source), running_(false), inFlight_(0),
          pipe_(0), consecutiveErrors_(0) {}

    ~MacDevice() {
        stopStream();
        if (source_) CFRelease(source_);
        (*intf_)->USBInterfaceClose(intf_);
        (*intf_)->Release(intf_);
        if (deviceOpen_) (*dev_)->USBDeviceClose(dev_);
        (*dev_)->Release(dev_);
    }

    int control(std::uint8_t requestType, std::uint8_t request, std::uint16_t value, std::uint16_t index,
                std::uint8_t* data, std::uint16_t length, unsigned timeoutMs) {
        IOUSBDevRequestTO req;
        req.bmRequestType = requestType;
        req.bRequest = request;
        req.wValue = value;
        req.wIndex = index;
        req.wLength = length;
        req.pData = data;
        req.wLenDone = 0;
        req.noDataTimeout = timeoutMs;
        req.completionTimeout = timeoutMs;
        // DeviceRequestTO works without USBDeviceOpen for vendor requests;
        // fall back to the interface's control pipe if the system disagrees.
        IOReturn rc = (*dev_)->DeviceRequestTO(dev_, &req);
        if (rc == kIOReturnNotOpen) {
            req.wLenDone = 0;
            rc = (*intf_)->ControlRequestTO(intf_, 0, &req);
        }
        if (rc != kIOReturnSuccess) {
            setError("USB control transfer failed: " + ioText(rc));
            return -1;
        }
        return static_cast<int>(req.wLenDone);
    }

    int bulkRead(std::uint8_t endpoint, std::uint8_t* data, std::size_t length, unsigned timeoutMs) {
        UInt8 pipe = pipeFor(endpoint | 0x80u);
        if (pipe == 0) {
            setError("USB bulk read: no such IN endpoint");
            return -1;
        }
        UInt32 size = static_cast<UInt32>(length);
        IOReturn rc = (*intf_)->ReadPipeTO(intf_, pipe, data, &size, timeoutMs, timeoutMs);
        if (rc == kIOUSBTransactionTimeout || rc == kIOUSBPipeStalled) {
            // After a timeout or stall IOKit wants the data toggle reset on
            // both ends before the pipe is used again.
            (*intf_)->ClearPipeStallBothEnds(intf_, pipe);
        }
        if (rc != kIOReturnSuccess) {
            setError("USB bulk read failed: " + ioText(rc));
            return -1;
        }
        return static_cast<int>(size);
    }

    bool startStream(std::uint8_t endpoint, std::size_t transferSize, int transfers, BulkCallback callback) {
        stopStream();
        if (transfers < 1 || transferSize == 0) {
            setError("startStream: bad transfer count or size");
            return false;
        }
        pipe_ = pipeFor(endpoint | 0x80u);
        if (pipe_ == 0) {
            setError("USB stream: no such IN endpoint");
            return false;
        }
        callback_ = callback;
        buffers_.assign(static_cast<std::size_t>(transfers), std::vector<std::uint8_t>(transferSize));
        slots_.assign(static_cast<std::size_t>(transfers), Slot());
        for (std::size_t i = 0; i < slots_.size(); ++i) {
            slots_[i].self = this;
            slots_[i].index = i;
        }
        consecutiveErrors_ = 0;
        inFlight_ = 0;
        running_.store(true);
        // Start from a clean data toggle on both ends (harmless if fine).
        (*intf_)->ClearPipeStallBothEnds(intf_, pipe_);

        // The thread reports (within microseconds) whether its first
        // submissions worked: -1 = not yet, 0 = failed, 1 = streaming.
        startOk_.store(-1);
        thread_ = std::thread(&MacDevice::streamLoop, this);
        while (startOk_.load() < 0) std::this_thread::yield();
        if (startOk_.load() == 0) {
            thread_.join();
            running_.store(false);
            return false;
        }
        return true;
    }

    void stopStream() {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
    }

    std::string lastError() const {
        std::lock_guard<std::mutex> lock(errorMutex_);
        return error_;
    }

private:
    // refcon for each async read: which device and which buffer.
    struct Slot {
        MacDevice* self;
        std::size_t index;
        Slot() : self(0), index(0) {}
    };

    void setError(const std::string& text) {
        std::lock_guard<std::mutex> lock(errorMutex_);
        error_ = text;
    }

    // Finds the pipeRef (1..n) of an endpoint address such as 0x81.
    UInt8 pipeFor(unsigned endpointAddress) {
        UInt8 count = 0;
        if ((*intf_)->GetNumEndpoints(intf_, &count) != kIOReturnSuccess) return 0;
        for (UInt8 pipe = 1; pipe <= count; ++pipe) {
            UInt8 direction = 0, number = 0, type = 0, interval = 0;
            UInt16 maxPacket = 0;
            if ((*intf_)->GetPipeProperties(intf_, pipe, &direction, &number, &type, &maxPacket, &interval) !=
                kIOReturnSuccess) {
                continue;
            }
            unsigned address = number | (direction == kUSBIn ? 0x80u : 0u);
            if (address == endpointAddress) return pipe;
        }
        return 0;
    }

    bool submit(std::size_t i) {
        IOReturn rc = (*intf_)->ReadPipeAsync(intf_, pipe_, &buffers_[i][0], static_cast<UInt32>(buffers_[i].size()),
                                              &MacDevice::readDone, &slots_[i]);
        if (rc != kIOReturnSuccess) {
            setError("USB stream: ReadPipeAsync failed: " + ioText(rc));
            return false;
        }
        ++inFlight_;
        return true;
    }

    // IOAsyncCallback1: runs on the stream thread's run loop.
    static void readDone(void* refcon, IOReturn result, void* arg0) {
        Slot* slot = static_cast<Slot*>(refcon);
        slot->self->completed(slot->index, result, static_cast<std::size_t>(reinterpret_cast<std::uintptr_t>(arg0)));
    }

    void completed(std::size_t i, IOReturn result, std::size_t bytes) {
        --inFlight_;
        bool keep = running_.load();
        if (result == kIOReturnSuccess) {
            consecutiveErrors_ = 0;
            if (keep && bytes > 0 && callback_) callback_(&buffers_[i][0], bytes);
        } else if (result == kIOReturnAborted) {
            keep = false;  // we are stopping
        } else if (result == kIOReturnNoDevice || result == kIOReturnNotResponding) {
            setError("USB device was unplugged: " + ioText(result));
            running_.store(false);
            keep = false;
        } else {
            if (result == kIOUSBPipeStalled) (*intf_)->ClearPipeStallBothEnds(intf_, pipe_);
            if (++consecutiveErrors_ > 50) {
                setError("USB stream: too many errors, last " + ioText(result));
                running_.store(false);
                keep = false;
            }
        }
        if (keep && running_.load()) submit(i);
    }

    void streamLoop() {
        CFRunLoopRef loop = CFRunLoopGetCurrent();
        CFRunLoopAddSource(loop, source_, kCFRunLoopDefaultMode);
        int submitted = 0;
        for (std::size_t i = 0; i < buffers_.size(); ++i) {
            if (submit(i)) ++submitted;
        }
        startOk_.store(submitted > 0 ? 1 : 0);
        if (submitted > 0) {
            // Stream until asked to stop (or every request died).
            while (running_.load() && inFlight_ > 0) {
                CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.1, true);
            }
            // Abort what is still queued and let the aborted requests call
            // back (each must, before its buffer may be freed).  Bounded
            // wait so an unplugged stick cannot hang the program.
            if (inFlight_ > 0) (*intf_)->AbortPipe(intf_, pipe_);
            for (int i = 0; i < 40 && inFlight_ > 0; ++i) {
                CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.05, true);
            }
        }
        CFRunLoopRemoveSource(loop, source_, kCFRunLoopDefaultMode);
        running_.store(false);
    }

    UsbDeviceIf** dev_;
    bool deviceOpen_;
    UsbInterfaceIf** intf_;
    CFRunLoopSourceRef source_;

    std::thread thread_;
    std::atomic<bool> running_;
    std::atomic<int> startOk_;
    int inFlight_;  // only touched on the stream thread
    UInt8 pipe_;
    int consecutiveErrors_;
    BulkCallback callback_;
    std::vector<std::vector<std::uint8_t> > buffers_;
    std::vector<Slot> slots_;

    mutable std::mutex errorMutex_;
    std::string error_;
};

// Creates a COM-style plug-in of `pluginType` for `service` and queries it
// for `interfaceId`.  Returns 0 on failure.
void** queryPlugin(io_service_t service, CFUUIDRef pluginType, CFUUIDRef interfaceId, std::string& error) {
    IOCFPlugInInterface** plugin = 0;
    SInt32 score = 0;
    kern_return_t kr = IOCreatePlugInInterfaceForService(service, pluginType, kIOCFPlugInInterfaceID, &plugin, &score);
    if (kr != KERN_SUCCESS || !plugin) {
        error = "IOCreatePlugInInterfaceForService failed: " + ioText(kr);
        return 0;
    }
    void** result = 0;
    HRESULT hr = (*plugin)->QueryInterface(plugin, CFUUIDGetUUIDBytes(interfaceId), reinterpret_cast<LPVOID*>(&result));
    (*plugin)->Release(plugin);
    if (hr != S_OK || !result) {
        error = "QueryInterface for the USB interface failed";
        return 0;
    }
    return result;
}

}  // namespace

// ---------------------------------------------------------------------------
//  Device::find
// ---------------------------------------------------------------------------
std::vector<DeviceInfo> Device::find(const std::vector<std::pair<std::uint16_t, std::uint16_t> >& ids) {
    std::vector<DeviceInfo> found;
    for (std::size_t k = 0; k < ids.size(); ++k) {
        std::vector<io_service_t> services = matchDevices(ids[k].first, ids[k].second);
        for (std::size_t i = 0; i < services.size(); ++i) {
            DeviceInfo info;
            info.vendorId = ids[k].first;
            info.productId = ids[k].second;
            info.manufacturer = stringProperty(services[i], kUSBVendorString);
            info.product = stringProperty(services[i], kUSBProductString);
            info.serial = stringProperty(services[i], kUSBSerialNumberString);
            info.path = locationText(services[i]);
            found.push_back(info);
            IOObjectRelease(services[i]);
        }
    }
    return found;
}

// ---------------------------------------------------------------------------
//  Device::open
// ---------------------------------------------------------------------------
std::unique_ptr<Device> Device::open(const DeviceInfo& info, int interfaceNumber, std::string& error) {
    // Find the service again by vendor/product + location.
    io_service_t service = 0;
    std::vector<io_service_t> services = matchDevices(info.vendorId, info.productId);
    for (std::size_t i = 0; i < services.size(); ++i) {
        if (!service && (info.path.empty() || locationText(services[i]) == info.path)) {
            service = services[i];
        } else {
            IOObjectRelease(services[i]);
        }
    }
    if (!service) {
        error = "USB device " + info.path + " is no longer present";
        return std::unique_ptr<Device>();
    }

    UsbDeviceIf** dev = reinterpret_cast<UsbDeviceIf**>(
        queryPlugin(service, kIOUSBDeviceUserClientTypeID, kIOUSBDeviceInterfaceID500, error));
    IOObjectRelease(service);
    if (!dev) return std::unique_ptr<Device>();

    // Opening the device is only needed to change its configuration; if
    // something else holds it we can still use the interface.
    bool deviceOpen = (*dev)->USBDeviceOpen(dev) == kIOReturnSuccess;
    UInt8 config = 0;
    if ((*dev)->GetConfiguration(dev, &config) == kIOReturnSuccess && config == 0) {
        IOUSBConfigurationDescriptorPtr descriptor = 0;
        if (deviceOpen && (*dev)->GetConfigurationDescriptorPtr(dev, 0, &descriptor) == kIOReturnSuccess) {
            (*dev)->SetConfiguration(dev, descriptor->bConfigurationValue);
        }
    }

    // Walk the interfaces of the active configuration for ours.
    IOUSBFindInterfaceRequest request;
    request.bInterfaceClass = kIOUSBFindInterfaceDontCare;
    request.bInterfaceSubClass = kIOUSBFindInterfaceDontCare;
    request.bInterfaceProtocol = kIOUSBFindInterfaceDontCare;
    request.bAlternateSetting = kIOUSBFindInterfaceDontCare;
    io_iterator_t iterator = 0;
    UsbInterfaceIf** intf = 0;
    if ((*dev)->CreateInterfaceIterator(dev, &request, &iterator) == kIOReturnSuccess) {
        while (io_service_t candidate = IOIteratorNext(iterator)) {
            if (!intf) {
                UsbInterfaceIf** maybe = reinterpret_cast<UsbInterfaceIf**>(
                    queryPlugin(candidate, kIOUSBInterfaceUserClientTypeID, kIOUSBInterfaceInterfaceID500, error));
                if (maybe) {
                    UInt8 number = 0;
                    if ((*maybe)->GetInterfaceNumber(maybe, &number) == kIOReturnSuccess &&
                        number == interfaceNumber) {
                        intf = maybe;
                    } else {
                        (*maybe)->Release(maybe);
                    }
                }
            }
            IOObjectRelease(candidate);
        }
        IOObjectRelease(iterator);
    }
    if (!intf) {
        if (error.empty()) error = "USB interface not found";
        if (deviceOpen) (*dev)->USBDeviceClose(dev);
        (*dev)->Release(dev);
        return std::unique_ptr<Device>();
    }

    IOReturn rc = (*intf)->USBInterfaceOpen(intf);
    if (rc != kIOReturnSuccess) {
        error = "Cannot open the USB interface: " + ioText(rc);
        if (rc == kIOReturnExclusiveAccess) error += ".  Is another program (gqrx, rtl_sdr, SDR++...) using the stick?";
        (*intf)->Release(intf);
        if (deviceOpen) (*dev)->USBDeviceClose(dev);
        (*dev)->Release(dev);
        return std::unique_ptr<Device>();
    }

    // The run-loop source that delivers ReadPipeAsync completions.
    CFRunLoopSourceRef source = 0;
    rc = (*intf)->CreateInterfaceAsyncEventSource(intf, &source);
    if (rc != kIOReturnSuccess || !source) {
        error = "Cannot create the USB async event source: " + ioText(rc);
        (*intf)->USBInterfaceClose(intf);
        (*intf)->Release(intf);
        if (deviceOpen) (*dev)->USBDeviceClose(dev);
        (*dev)->Release(dev);
        return std::unique_ptr<Device>();
    }

    error.clear();
    return std::unique_ptr<Device>(new MacDevice(dev, deviceOpen, intf, source));
}

}  // namespace usb

#endif  // __APPLE__
