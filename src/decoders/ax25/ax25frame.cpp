// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/ax25/ax25frame.cpp
//  Layer:  Library liboffgrid-decoders (internal to the AX.25 decoder)
//
//  Where this file comes from
//  --------------------------
//  The Addr and Frame parts of AX25Toolkit's lib/ax25lib.cpp by Gustavo
//  Campos (https://github.com/solariun/AX25Toolkit), used with the author's
//  permission.  Original header comment kept below.
//
//  What this file does
//  -------------------
//  Implements ax25frame.h: address and frame (de)coding.
//
//  How it fits in
//  --------------
//  Standard library only.
//
//  Contributing
//  ------------
//  Changes from AX25Toolkit are marked "OFFgrid:".
// ============================================================================
//
// =============================================================================
// ax25lib.cpp — Implementation
// =============================================================================
#include "ax25frame.h"

#include <cctype>
#include <cstring>

namespace dec {
namespace ax25 {

// =============================================================================
// Addr
// =============================================================================
Addr Addr::make(const std::string& callssid) {
    Addr a;
    std::string s = callssid;
    for (auto& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    auto dash = s.find('-');
    std::string cs = (dash != std::string::npos) ? s.substr(0, dash) : s;
    if (cs.size() > 6) cs.resize(6);
    std::memcpy(a.call, cs.c_str(), cs.size() + 1);  // includes NUL
    if (dash != std::string::npos) {
        try { a.ssid = std::stoi(s.substr(dash + 1)); } catch (...) {}
    }
    return a;
}

Addr Addr::decode(const uint8_t* p) {
    Addr a;
    char cs[7]{};
    for (int i = 0; i < 6; ++i) cs[i] = static_cast<char>((p[i] >> 1) & 0x7F);
    // trim trailing spaces
    int end = 5;
    while (end >= 0 && cs[end] == ' ') --end;
    cs[end + 1] = '\0';
    std::memcpy(a.call, cs, 7);
    a.ssid     = (p[6] >> 1) & 0x0F;
    a.repeated = (p[6] & 0x80) != 0;
    return a;
}

std::vector<uint8_t> Addr::encode(bool last_addr) const {
    std::vector<uint8_t> out(7, static_cast<uint8_t>(' ' << 1));
    for (int i = 0; call[i] != '\0' && i < 6; ++i)
        out[i] = static_cast<uint8_t>((call[i] & 0x7F) << 1);
    uint8_t sb = 0x60;                      // reserved bits per spec
    sb = static_cast<uint8_t>(sb | (ssid & 0x0F) << 1);
    if (repeated)  sb |= 0x80;
    if (last_addr) sb |= 0x01;
    out[6] = sb;
    return out;
}

std::string Addr::str() const {
    std::string s(call);
    if (ssid > 0) { s += '-'; s += std::to_string(ssid); }
    if (repeated)   s += '*';
    return s;
}

bool Addr::operator==(const Addr& o) const {
    return std::strcmp(call, o.call) == 0 && ssid == o.ssid;
}

// =============================================================================
// Frame helpers (local)
// =============================================================================
namespace {
constexpr uint8_t PF_BIT    = 0x10;
constexpr uint8_t SABM_BASE = 0x2F;   // P/F cleared
constexpr uint8_t DISC_BASE = 0x43;
constexpr uint8_t UA_BASE   = 0x63;
constexpr uint8_t DM_BASE   = 0x0F;
constexpr uint8_t UI_BASE   = 0x03;
constexpr uint8_t FRMR_BASE = 0x87;

// (OFFgrid: the I/RR/REJ control-byte builders mk_i, mk_rr, mk_rej, used by
//  ax25lib's connected mode, are not copied: unused here)
} // anon

// =============================================================================
// Frame
// =============================================================================
Frame::Type Frame::type() const {
    if ((ctrl & 0x01) == 0x00) return Type::IFrame;
    if ((ctrl & 0x03) == 0x01) {
        // SS bits (3-2): 00=RR, 01=RNR, 10=REJ  (AX.25 v2.2 §4.3.2)
        switch ((ctrl >> 2) & 3) {
            case 0: return Type::RR;
            case 1: return Type::RNR;
            case 2: return Type::REJ;
            default: return Type::SFrame;
        }
    }
    // U-frame: ignore P/F bit for matching
    switch (static_cast<uint8_t>(ctrl & ~PF_BIT)) {
        case UI_BASE:   return Type::UI;
        case SABM_BASE: return Type::SABM;
        case DISC_BASE: return Type::DISC;
        case UA_BASE:   return Type::UA;
        case DM_BASE:   return Type::DM;
        case FRMR_BASE: return Type::FRMR;
        default:        return Type::Unknown;
    }
}

bool Frame::decode(const std::vector<uint8_t>& raw, Frame& f) {
    if (raw.size() < 14) return false;
    std::size_t pos = 0;

    f.dest = Addr::decode(&raw[pos]); pos += 7;
    f.src  = Addr::decode(&raw[pos]);
    bool last = (raw[pos + 6] & 0x01) != 0;
    pos += 7;

    f.digis.clear();   // OFFgrid: `f` may be reused
    while (!last && pos + 7 <= raw.size()) {
        Addr d = Addr::decode(&raw[pos]);
        last = (raw[pos + 6] & 0x01) != 0;
        f.digis.push_back(d);
        pos += 7;
    }
    // OFFgrid: the address field must end, after at most 8 digipeaters.
    if (!last || f.digis.size() > 8) return false;

    if (pos >= raw.size()) return false;
    f.ctrl = raw[pos++];

    Type t = f.type();
    if (t == Type::IFrame || t == Type::UI) {
        if (pos >= raw.size()) return false;
        f.pid = raw[pos++]; f.has_pid = true;
    } else {
        f.has_pid = false;
    }

    f.info = std::vector<uint8_t>(raw.begin() + pos, raw.end());
    return true;
}

std::vector<uint8_t> Frame::encode() const {
    std::vector<uint8_t> out;
    auto app = [&](const std::vector<uint8_t>& v){ out.insert(out.end(),v.begin(),v.end()); };
    app(dest.encode(false));
    app(src.encode(digis.empty()));
    for (std::size_t i = 0; i < digis.size(); ++i)
        app(digis[i].encode(i == digis.size() - 1));
    out.push_back(ctrl);
    if (has_pid) out.push_back(pid);
    out.insert(out.end(), info.begin(), info.end());
    return out;
}

// (OFFgrid: Frame::format() not copied)

}  // namespace ax25
}  // namespace dec
