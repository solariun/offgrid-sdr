// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/ax25/ax25frame.h
//  Layer:  Library liboffgrid-decoders (internal header of the AX.25 decoder)
//
//  Where this file comes from
//  --------------------------
//  The "AX.25 address" and "AX.25 raw frame" parts of AX25Toolkit's
//  lib/ax25lib.hpp by Gustavo Campos (https://github.com/solariun/AX25Toolkit),
//  used with the author's permission.  The rest of ax25lib (serial ports,
//  KISS, the connected-mode router, the BBS, its APRS helpers) is not needed
//  to decode and is not copied.  Original header comment kept below.
//
//  What this file does
//  -------------------
//  ax25::Addr   a callsign, SSID and "has been repeated" (H) bit; the 7-byte
//               on-air form (each character shifted left by one).
//  ax25::Frame  destination, source, digipeaters, control, PID, information;
//               decode() from the bytes between the flags (FCS removed),
//               encode() back, type() from the control byte (UI, I, RR...).
//
//  Changes from AX25Toolkit (marked "OFFgrid:"):
//    - namespace ax25 -> dec::ax25;
//    - Frame::format() (an iostream-based dump) is not copied: Ax25.cpp
//      prints frames its own way;
//    - Frame::decode() also rejects a frame whose address field never ends
//      (no "last address" bit) or holds more than 8 digipeaters, as the
//      AX.25 2.2 spec and OFFgrid-SDR's JS parseAx25 do.
//
//  How it fits in
//  --------------
//  Used by Ax25.cpp and Ax25Encoders.cpp.  Standard library only.
//
//  Contributing
//  ------------
//  Keep in step with AX25Toolkit's ax25lib; strict C++11.
// ============================================================================
//
// =============================================================================
// ax25lib.hpp — AX.25 / KISS library  (C++11, POSIX, Linux + macOS)
//
#ifndef OFFGRID_DECODERS_AX25_AX25FRAME_H
#define OFFGRID_DECODERS_AX25_AX25FRAME_H

#include <cstdint>
#include <string>
#include <vector>

namespace dec {
namespace ax25 {


// ─────────────────────────────────────────────────────────────────────────────
// AX.25 address  (max 6-char callsign + 0-15 SSID)
// ─────────────────────────────────────────────────────────────────────────────
struct Addr {
    char call[7]{};      // NUL-terminated, uppercase
    int  ssid     = 0;
    bool repeated = false;  // H bit (digipeater "has repeated")

    static Addr         make(const std::string& callssid);
    static Addr         decode(const uint8_t* p);      // 7 raw bytes
    std::vector<uint8_t> encode(bool last_addr) const; // 7 raw bytes

    std::string str() const;
    bool operator==(const Addr& o) const;
    bool operator!=(const Addr& o) const { return !(*this == o); }
    bool empty()                   const { return call[0] == '\0'; }
};

// ─────────────────────────────────────────────────────────────────────────────
// AX.25 raw frame
// ─────────────────────────────────────────────────────────────────────────────
struct Frame {
    Addr  dest, src;
    std::vector<Addr>    digis;
    uint8_t              ctrl    = 0x03;
    uint8_t              pid     = 0xF0;
    bool                 has_pid = true;
    std::vector<uint8_t> info;

    enum class Type { Unknown, IFrame, SFrame, UI, SABM, DISC, UA, DM, FRMR, RR, RNR, REJ };

    Type type()     const;
    int  get_ns()   const { return (ctrl >> 1) & 7; }
    int  get_nr()   const { return (ctrl >> 5) & 7; }
    bool get_pf()   const { return (ctrl & 0x10) != 0; }

    // Returns true and fills 'out' on success; returns false if raw is malformed.
    static bool decode(const std::vector<uint8_t>& raw, Frame& out);
    std::vector<uint8_t>        encode() const;
    // (OFFgrid: format() not copied)
};

}  // namespace ax25
}  // namespace dec

#endif
