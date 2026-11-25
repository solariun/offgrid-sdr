// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Decoder.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The factory and names of Decoder.h: makeDecoder(kind) returns the
//  decoder of that kind with its default settings.
//
//  Contributing
//  ------------
//  A new decoder: a Kind value in Decoder.h, its name in kNames below (same
//  order), a line in makeDecoder(), and its own header + source file.
// ============================================================================
#include "Decoder.h"

#include <cctype>

#include "Ax25.h"
#include "Cw.h"
#include "Fax.h"
#include "Hell.h"
#include "Navtex.h"
#include "Psk.h"
#include "Rtty.h"
#include "Sstv.h"
#include "Time.h"

namespace dec {

namespace {
const char* const kNames[] = {"CW", "RTTY", "PSK", "NAVTEX", "DSC", "HELL", "FAX", "SSTV", "TIME", "AX25"};
const int kCount = 10;
}  // namespace

const char* kindName(Kind kind) {
    int i = static_cast<int>(kind);
    return i >= 0 && i < kCount ? kNames[i] : "?";
}

bool parseKind(const std::string& name, Kind& kind) {
    std::string up;
    for (size_t i = 0; i < name.size(); ++i) up += static_cast<char>(std::toupper(static_cast<unsigned char>(name[i])));
    if (up == "HELLSCHREIBER" || up == "FELDHELL") up = "HELL";
    if (up == "PSK31" || up == "PSK63" || up == "PSK125") up = "PSK";
    if (up == "AX.25" || up == "APRS" || up == "PACKET") up = "AX25";
    if (up == "WEFAX" || up == "RADIOFAX") up = "FAX";
    for (int i = 0; i < kCount; ++i) {
        if (up == kNames[i]) {
            kind = static_cast<Kind>(i);
            return true;
        }
    }
    return false;
}

std::vector<Kind> allKinds() {
    std::vector<Kind> k;
    for (int i = 0; i < kCount; ++i) k.push_back(static_cast<Kind>(i));
    return k;
}

std::unique_ptr<Decoder> makeDecoder(Kind kind) {
    switch (kind) {
        case Kind::Cw: return std::unique_ptr<Decoder>(new CwDecoder());
        case Kind::Rtty: return std::unique_ptr<Decoder>(new RttyDecoder());
        case Kind::Psk: return std::unique_ptr<Decoder>(new PskAutoDecoder());
        case Kind::Navtex: return std::unique_ptr<Decoder>(new NavtexDecoder());
        case Kind::Dsc: return std::unique_ptr<Decoder>(new DscDecoder());
        case Kind::Hell: return std::unique_ptr<Decoder>(new HellDecoder());
        case Kind::Fax: return std::unique_ptr<Decoder>(new FaxDecoder());
        case Kind::Sstv: return std::unique_ptr<Decoder>(new SstvDecoder());
        case Kind::Ax25: return std::unique_ptr<Decoder>(new Ax25Decoder());
        case Kind::Time: return std::unique_ptr<Decoder>(new TimeDecoder());
    }
    return std::unique_ptr<Decoder>();
}

}  // namespace dec
