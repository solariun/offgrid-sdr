// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/usb/UsbLinux.cpp
//  Layer:  USB access (no libusb: talks to the operating system directly)
//
//  What this file does
//  -------------------
//  Implements usb::Device on Linux with "usbfs", the kernel's raw USB
//  character devices under /dev/bus/usb/BBB/DDD.  This is exactly what
//  libusb does internally on Linux, minus the library: a handful of ioctl()
//  calls from <linux/usbdevice_fs.h>.
//
//    find()         walks /sys/bus/usb/devices/* (sysfs) and reads the
//                   idVendor / idProduct / busnum / devnum / string files.
//    open()         opens /dev/bus/usb/BBB/DDD, detaches the DVB-T kernel
//                   driver (dvb_usb_rtl28xxu) from the interface with
//                   USBDEVFS_DISCONNECT, then USBDEVFS_CLAIMINTERFACE.
//    control()      USBDEVFS_CONTROL.
//    bulkRead()     USBDEVFS_BULK (synchronous).
//    startStream()  N asynchronous URBs (USBDEVFS_SUBMITURB), a thread that
//                   poll()s the fd and reaps completed URBs with
//                   USBDEVFS_REAPURBNDELAY, hands the data to the callback
//                   and resubmits.  stopStream() cancels them with
//                   USBDEVFS_DISCARDURB and waits until all are reaped.
//
//  How it fits in
//  --------------
//  Selected at compile time (#ifdef __linux__); UsbMac.cpp is the macOS
//  twin.  Nothing here needs a shared library, so the program can be linked
//  with -static and copied to any Linux machine (Raspberry Pi included).
//
//  Contributing
//  ------------
//  * Permissions: /dev/bus/usb nodes are root-only by default.  Users need a
//    udev rule (see the error text in openNode()).  Do not "fix" this by
//    asking people to run as root.
//  * usbfs semantics worth knowing: a completed URB makes the fd *writable*
//    (POLLOUT), not readable; DISCARDURB on an URB that already completed
//    fails with EINVAL, which is fine; every submitted URB must be reaped
//    exactly once before its memory is freed.
// ============================================================================
#ifdef __linux__

#include "UsbDevice.h"

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <thread>

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <linux/usbdevice_fs.h>

namespace usb {

namespace {

// Where the kernel publishes one directory per USB device (and one per
// interface, whose names contain a ':' and are skipped).
const char* const kSysfsRoot = "/sys/bus/usb/devices";

// Reads the first line of a sysfs attribute file ("" if missing).
std::string readSysfsLine(const std::string& path) {
    std::ifstream in(path.c_str());
    std::string line;
    if (in) std::getline(in, line);
    // sysfs values end with '\n', which getline already removed; strip a
    // stray '\r' or trailing spaces just in case.
    while (!line.empty() && (line[line.size() - 1] == '\r' || line[line.size() - 1] == ' ')) {
        line.erase(line.size() - 1);
    }
    return line;
}

// Parses a sysfs number ("0bda" in hex for ids, "3" in decimal for bus/dev).
bool parseNumber(const std::string& text, int base, unsigned long& out) {
    if (text.empty()) return false;
    char* end = 0;
    errno = 0;
    out = std::strtoul(text.c_str(), &end, base);
    return errno == 0 && end != text.c_str();
}

// errno -> "Permission denied (EACCES)" style text.
std::string errnoText(int err) {
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, " (errno %d)", err);
    return std::string(std::strerror(err)) + buffer;
}

// ---------------------------------------------------------------------------
//  LinuxDevice: one opened /dev/bus/usb node with one claimed interface.
// ---------------------------------------------------------------------------
class LinuxDevice : public Device {
public:
    LinuxDevice(int fd, int interfaceNumber, bool detachedKernelDriver)
        : fd_(fd), interface_(interfaceNumber), detachedKernelDriver_(detachedKernelDriver), running_(false) {}

    ~LinuxDevice() {
        stopStream();
        // Release our claim and, if we kicked the DVB-T driver off the
        // interface, give it back (librtlsdr does the same on close).
        unsigned int ifno = static_cast<unsigned int>(interface_);
        ioctl(fd_, USBDEVFS_RELEASEINTERFACE, &ifno);
        if (detachedKernelDriver_) {
            struct usbdevfs_ioctl command;
            std::memset(&command, 0, sizeof command);
            command.ifno = interface_;
            command.ioctl_code = USBDEVFS_CONNECT;
            command.data = 0;
            ioctl(fd_, USBDEVFS_IOCTL, &command);
        }
        close(fd_);
    }

    int control(std::uint8_t requestType, std::uint8_t request, std::uint16_t value, std::uint16_t index,
                std::uint8_t* data, std::uint16_t length, unsigned timeoutMs) {
        struct usbdevfs_ctrltransfer transfer;
        std::memset(&transfer, 0, sizeof transfer);
        transfer.bRequestType = requestType;
        transfer.bRequest = request;
        transfer.wValue = value;
        transfer.wIndex = index;
        transfer.wLength = length;
        transfer.timeout = timeoutMs;
        transfer.data = data;
        int result;
        do {
            result = ioctl(fd_, USBDEVFS_CONTROL, &transfer);
        } while (result < 0 && errno == EINTR);
        if (result < 0) {
            setError("USB control transfer failed: " + errnoText(errno));
            return -1;
        }
        return result;
    }

    int bulkRead(std::uint8_t endpoint, std::uint8_t* data, std::size_t length, unsigned timeoutMs) {
        struct usbdevfs_bulktransfer transfer;
        std::memset(&transfer, 0, sizeof transfer);
        transfer.ep = endpoint | 0x80u;  // IN endpoint
        transfer.len = static_cast<unsigned int>(length);
        transfer.timeout = timeoutMs;
        transfer.data = data;
        int result;
        do {
            result = ioctl(fd_, USBDEVFS_BULK, &transfer);
        } while (result < 0 && errno == EINTR);
        if (result < 0) {
            setError("USB bulk read failed: " + errnoText(errno));
            return -1;
        }
        return result;
    }

    bool startStream(std::uint8_t endpoint, std::size_t transferSize, int transfers, BulkCallback callback) {
        stopStream();
        if (transfers < 1 || transferSize == 0) {
            setError("startStream: bad transfer count or size");
            return false;
        }
        callback_ = callback;
        endpoint_ = endpoint | 0x80u;
        // One buffer and one URB per in-flight transfer.  The URB structs
        // must stay at a fixed address while the kernel owns them, so the
        // vectors are sized once here and never resized while streaming.
        buffers_.assign(static_cast<std::size_t>(transfers), std::vector<std::uint8_t>(transferSize));
        urbs_.assign(static_cast<std::size_t>(transfers), usbdevfs_urb());
        inFlight_.assign(static_cast<std::size_t>(transfers), false);

        running_.store(true);
        int submitted = 0;
        for (int i = 0; i < transfers; ++i) {
            if (submit(static_cast<std::size_t>(i))) ++submitted;
        }
        if (submitted == 0) {
            running_.store(false);
            setError("USB stream: could not submit any transfer: " + errnoText(errno));
            return false;
        }
        thread_ = std::thread(&LinuxDevice::streamLoop, this);
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
    // Fills in and submits URB number `i`.
    bool submit(std::size_t i) {
        usbdevfs_urb& urb = urbs_[i];
        std::memset(&urb, 0, sizeof urb);
        urb.type = USBDEVFS_URB_TYPE_BULK;
        urb.endpoint = endpoint_;
        urb.buffer = &buffers_[i][0];
        urb.buffer_length = static_cast<int>(buffers_[i].size());
        urb.usercontext = reinterpret_cast<void*>(i);  // our index, returned on reap
        if (ioctl(fd_, USBDEVFS_SUBMITURB, &urb) < 0) {
            inFlight_[i] = false;
            return false;
        }
        inFlight_[i] = true;
        return true;
    }

    std::size_t inFlightCount() const {
        std::size_t n = 0;
        for (std::size_t i = 0; i < inFlight_.size(); ++i) n += inFlight_[i] ? 1 : 0;
        return n;
    }

    // Reaps every URB that has completed, without blocking.  Calls the
    // callback for good ones and resubmits while running.  Returns false
    // if the device has gone away.
    bool reapCompleted() {
        for (;;) {
            usbdevfs_urb* done = 0;
            if (ioctl(fd_, USBDEVFS_REAPURBNDELAY, &done) < 0) {
                if (errno == EAGAIN || errno == EINTR) return true;  // nothing (more) completed
                if (errno == ENODEV) {
                    setError("USB device was unplugged");
                    for (std::size_t i = 0; i < inFlight_.size(); ++i) inFlight_[i] = false;
                    return false;
                }
                return true;
            }
            std::size_t i = reinterpret_cast<std::size_t>(done->usercontext);
            if (i >= urbs_.size()) continue;  // cannot happen; be defensive
            inFlight_[i] = false;
            bool keep = running_.load();
            if (done->status == 0 && done->actual_length > 0 && keep && callback_) {
                callback_(&buffers_[i][0], static_cast<std::size_t>(done->actual_length));
            } else if (done->status == -EPIPE && keep) {
                // Endpoint stalled: clear the halt and carry on.
                unsigned int ep = endpoint_;
                ioctl(fd_, USBDEVFS_CLEAR_HALT, &ep);
            } else if (done->status == -ENODEV || done->status == -ESHUTDOWN) {
                setError("USB device was unplugged");
                running_.store(false);
                keep = false;
            }
            if (keep && running_.load()) submit(i);
        }
    }

    // The stream thread: wait for completions, reap, repeat; on stop,
    // cancel what is still queued and reap it all before returning.
    void streamLoop() {
        while (running_.load()) {
            struct pollfd p;
            p.fd = fd_;
            p.events = POLLOUT;  // usbfs signals "URB completed" as writable
            p.revents = 0;
            int r = poll(&p, 1, 100);
            if (r < 0 && errno != EINTR) break;
            if (r > 0) {
                if (p.revents & (POLLERR | POLLHUP)) {
                    setError("USB device was unplugged");
                    break;
                }
                if (!reapCompleted()) break;
            }
            if (inFlightCount() == 0) break;  // everything failed to resubmit
        }
        running_.store(false);
        // Cancel whatever is still with the kernel...
        for (std::size_t i = 0; i < urbs_.size(); ++i) {
            if (inFlight_[i]) ioctl(fd_, USBDEVFS_DISCARDURB, &urbs_[i]);
        }
        // ...and reap it (discarded URBs still have to be reaped).  Give up
        // after ~2 s so an unplugged stick cannot hang the program.
        for (int attempt = 0; attempt < 200 && inFlightCount() > 0; ++attempt) {
            struct pollfd p;
            p.fd = fd_;
            p.events = POLLOUT;
            p.revents = 0;
            poll(&p, 1, 10);
            if (!reapCompleted()) break;
        }
    }

    // error_ is written by both the caller's thread and the stream thread.
    void setError(const std::string& text) {
        std::lock_guard<std::mutex> lock(errorMutex_);
        error_ = text;
    }

    int fd_;
    int interface_;
    bool detachedKernelDriver_;
    std::atomic<bool> running_;
    std::thread thread_;
    BulkCallback callback_;
    unsigned char endpoint_;
    std::vector<std::vector<std::uint8_t> > buffers_;
    std::vector<usbdevfs_urb> urbs_;
    std::vector<bool> inFlight_;  // touched only by the stream thread while it runs
    mutable std::mutex errorMutex_;
    std::string error_;
};

}  // namespace

// ---------------------------------------------------------------------------
//  Device::find
// ---------------------------------------------------------------------------
std::vector<DeviceInfo> Device::find(const std::vector<std::pair<std::uint16_t, std::uint16_t> >& ids) {
    std::vector<DeviceInfo> found;
    DIR* dir = opendir(kSysfsRoot);
    if (!dir) return found;
    while (struct dirent* entry = readdir(dir)) {
        std::string name = entry->d_name;
        // Skip ".", "..", interface nodes ("1-1:1.0") and root hubs ("usb1"
        // are fine to read but never match).
        if (name.empty() || name[0] == '.' || name.find(':') != std::string::npos) continue;
        std::string base = std::string(kSysfsRoot) + "/" + name + "/";
        unsigned long vid, pid, bus, dev;
        if (!parseNumber(readSysfsLine(base + "idVendor"), 16, vid)) continue;
        if (!parseNumber(readSysfsLine(base + "idProduct"), 16, pid)) continue;
        bool wanted = false;
        for (std::size_t i = 0; i < ids.size(); ++i) {
            if (ids[i].first == vid && ids[i].second == pid) wanted = true;
        }
        if (!wanted) continue;
        if (!parseNumber(readSysfsLine(base + "busnum"), 10, bus)) continue;
        if (!parseNumber(readSysfsLine(base + "devnum"), 10, dev)) continue;
        DeviceInfo info;
        info.vendorId = static_cast<std::uint16_t>(vid);
        info.productId = static_cast<std::uint16_t>(pid);
        info.manufacturer = readSysfsLine(base + "manufacturer");
        info.product = readSysfsLine(base + "product");
        info.serial = readSysfsLine(base + "serial");
        char path[64];
        std::snprintf(path, sizeof path, "/dev/bus/usb/%03lu/%03lu", bus, dev);
        info.path = path;
        found.push_back(info);
    }
    closedir(dir);
    return found;
}

// ---------------------------------------------------------------------------
//  Device::open
// ---------------------------------------------------------------------------
std::unique_ptr<Device> Device::open(const DeviceInfo& info, int interfaceNumber, std::string& error) {
    int fd = ::open(info.path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        int err = errno;
        error = "Cannot open " + info.path + ": " + errnoText(err);
        if (err == EACCES || err == EPERM) {
            error +=
                ".\nYour user may not access this USB device.  Install a udev rule, e.g. "
                "/etc/udev/rules.d/20-rtlsdr.rules containing\n"
                "  SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0bda\", ATTRS{idProduct}==\"2838\", "
                "MODE=\"0666\"\n"
                "then run 'sudo udevadm control --reload-rules && sudo udevadm trigger' and "
                "replug the stick.  Also blacklist the DVB-T driver: echo 'blacklist "
                "dvb_usb_rtl28xxu' | sudo tee /etc/modprobe.d/blacklist-rtlsdr.conf";
        }
        return std::unique_ptr<Device>();
    }

    // Detach the kernel's DVB-T driver (dvb_usb_rtl28xxu) if it is bound to
    // the interface.  ENODATA means no driver was bound: nothing to do.
    bool detached = false;
    {
        struct usbdevfs_getdriver driver;
        std::memset(&driver, 0, sizeof driver);
        driver.interface = static_cast<unsigned int>(interfaceNumber);
        bool bound = ioctl(fd, USBDEVFS_GETDRIVER, &driver) == 0;
        // "usbfs" means another program holds it through usbfs: do not kick
        // it off, the claim below will report EBUSY instead.
        if (bound && std::strcmp(driver.driver, "usbfs") != 0) {
            struct usbdevfs_ioctl command;
            std::memset(&command, 0, sizeof command);
            command.ifno = interfaceNumber;
            command.ioctl_code = USBDEVFS_DISCONNECT;
            command.data = 0;
            if (ioctl(fd, USBDEVFS_IOCTL, &command) == 0) {
                detached = true;
            } else if (errno != ENODATA) {
                int err = errno;
                error = std::string("Cannot detach kernel driver '") + driver.driver + "' from " + info.path + ": " +
                        errnoText(err) +
                        ".  Unload it with 'sudo rmmod dvb_usb_rtl28xxu' and blacklist it in "
                        "/etc/modprobe.d/.";
                close(fd);
                return std::unique_ptr<Device>();
            }
        }
    }

    unsigned int ifno = static_cast<unsigned int>(interfaceNumber);
    if (ioctl(fd, USBDEVFS_CLAIMINTERFACE, &ifno) < 0) {
        int err = errno;
        error = "Cannot claim USB interface of " + info.path + ": " + errnoText(err);
        if (err == EBUSY) error += ".  Is another program (rtl_sdr, rtl_tcp, gqrx...) using the stick?";
        if (detached) {
            struct usbdevfs_ioctl command;
            std::memset(&command, 0, sizeof command);
            command.ifno = interfaceNumber;
            command.ioctl_code = USBDEVFS_CONNECT;
            command.data = 0;
            ioctl(fd, USBDEVFS_IOCTL, &command);
        }
        close(fd);
        return std::unique_ptr<Device>();
    }

    return std::unique_ptr<Device>(new LinuxDevice(fd, interfaceNumber, detached));
}

}  // namespace usb

#endif  // __linux__
