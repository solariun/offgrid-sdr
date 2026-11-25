// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Vdl2.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  Vdl2Decoder: the shared first stage (mix + decimate by 4), one channel
//  stage and burst receiver per VDL2 channel, the burst decoding (preamble,
//  D8PSK, descrambling, header, Reed-Solomon, HDLC, CRC) and the AVLC frame
//  parser; see Vdl2.h for the method.  Port of OFFgrid-SDR's JS vdllib
//  worker (diddy-boy, https://github.com/solariun/offgrid-sdr; written from
//  the standard and checked against dumpvdl2 by Tomasz Lemiech), with the
//  JS comments kept (reworded) next to the code they explain.
//
//  Differences from the JS:
//    - Addition to the JS: the shared stage's second-order CIC is computed
//      as the equivalent 7-tap triangle (1 2 3 4 3 2 1)/16 every 4th sample
//      instead of with running integrators: the same output, but it cannot
//      lose precision over hours (the JS integrators grow without bound when
//      the stick's DC lands at the mixing frequency).
//    - Addition to the JS: the first noise floor is the median power of the
//      first 1024 samples of a channel however they arrive (the JS needed a
//      single block of 256+ samples, else started from 1e-6); no burst is
//      looked for until then (20 ms at 2.048 MS/s).
//    - Addition to the JS: non-finite IQ samples are replaced by 0, and a
//      non-finite running state is reset, so one bad sample cannot stop it.
//    - Addition to the JS: the default channel list is every channel the JS
//      knows (it used the locator, else 136.975 alone); setLocation() gives
//      the JS choice.
//    - Addition to the JS: ACARS over AVLC is parsed with the ACARS
//      decoder's own parser (acarsParse, Acars.h): the registration loses
//      all its dots (the JS: leading ones), a DEL label character reads "d",
//      the text is tidied the same way; the ETX is taken just before the
//      CRC and DEL when it is there (the JS took the last ETX-valued byte,
//      which a CRC byte can be).
//    - Addition to the JS: a U frame other than XID reads "Link control"
//      (the JS printed "Acknowledgement (link control)").
//    - buffers are kept and reused (the JS allocated per block and burst).
//
//  How it fits in
//  --------------
//  Implements Vdl2.h.  Uses Acars.h.  Standard library only.
//
//  Contributing
//  ------------
//  No allocation per sample.  Keep in step with the JS; run
//  tests/test_decoders_aviation.cpp (it checks real-time CPU use too).
// ============================================================================
#include "Vdl2.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace dec {

namespace {

const double kPi = 3.141592653589793;
const double kTwoPi = 6.283185307179586;

bool finite(double v) { return v == v && v - v == 0; }   // not NaN, not +-inf

// ---------------------------------------------------------------------------
// GF(256), field polynomial 0x187 (the JS VG / VDL_GF), and the RS(255,249)
// generator, highest degree first (VDL_RS_GEN)
// ---------------------------------------------------------------------------
struct Gf {
    unsigned char exp[512], log[256];
    unsigned char gen[7];
    Gf() {
        unsigned x = 1;
        log[0] = 0;
        for (int i = 0; i < 255; ++i) {
            exp[i] = static_cast<unsigned char>(x);
            log[x] = static_cast<unsigned char>(i);
            x <<= 1;
            if (x & 256) x ^= 0x187;
        }
        for (int i = 255; i < 512; ++i) exp[i] = exp[i - 255];
        // g(x) = prod over i = 0..5 of (x - a^(120 + i))
        unsigned char g[7] = {1, 0, 0, 0, 0, 0, 0};
        int len = 1;
        for (int i = 0; i < 6; ++i) {
            const unsigned char r = exp[(120 + i) % 255];
            unsigned char n[7] = {0, 0, 0, 0, 0, 0, 0};
            for (int j = 0; j < len; ++j) {
                n[j] ^= g[j];
                n[j + 1] ^= mul(g[j], r);
            }
            ++len;
            for (int j = 0; j < len; ++j) g[j] = n[j];
        }
        for (int j = 0; j < 7; ++j) gen[j] = g[j];
    }
    unsigned char mul(unsigned a, unsigned b) const { return a && b ? exp[log[a] + log[b]] : 0; }
    unsigned char div(unsigned a, unsigned b) const { return a ? exp[(log[a] + 255 - log[b]) % 255] : 0; }
    unsigned char pw(int e) const { return exp[((e % 255) + 255) % 255]; }
};
const Gf& gf() {
    static const Gf g;
    return g;
}

// The header's parity-check matrix (VDL_HM: 0b0000000011111111111110000, ...).
const unsigned kHm[5] = {0x1fff0u, 0x7e1fe8u, 0x18e61e4u, 0x1b6a662u, 0xd3caa1u};

// Syndrome -> the single-bit error that gives it (VDL_SYNTAB: the first bit
// found wins; 0 -> no error), -1 when no single-bit error does.
struct SynTab {
    int t[32];
    SynTab() {
        for (int i = 0; i < 32; ++i) t[i] = -1;
        t[0] = 0;
        for (int i = 0; i < 25; ++i) {
            const unsigned s = vdl2HeaderSyndrome(1u << i);
            if (t[s] < 0) t[s] = 1 << i;
        }
    }
};
const SynTab& synTab() {
    static const SynTab s;
    return s;
}

// FEC bytes of the last RS block for its data length (the JS fecOf).
int fecOf(int L) { return L < 3 ? 0 : L < 31 ? 2 : L < 68 ? 4 : 6; }

// AVLC address (4 bytes, 7 bits each in bits 1-7, reversed): addr, type, status.
void addrOf(const unsigned char* b, unsigned& addr, int& type) {
    const unsigned raw = (b[0] >> 1) | (static_cast<unsigned>(b[1] >> 1) << 7) | (static_cast<unsigned>(b[2] >> 1) << 14) |
                         (static_cast<unsigned>(b[3] >> 1) << 21);
    unsigned v = 0;
    for (int i = 0; i < 28; ++i)
        if ((raw >> i) & 1) v |= 1u << (27 - i);
    addr = v & 0xffffffu;
    type = static_cast<int>((v >> 24) & 7);
}

std::string hex6(unsigned a) {
    char b[16];
    std::snprintf(b, sizeof b, "%06X", a & 0xffffffu);
    return b;
}

std::string mhz(double hz) {
    char b[32];
    std::snprintf(b, sizeof b, "%.3f", hz / 1e6);
    return b;
}

const char* const kTypes[8] = {"", "aircraft", "", "", "ground", "ground", "", "all"};

struct IcaoBlock {
    unsigned lo, hi;
    const char* name;
};
// ICAO's 24-bit address allocation (Annex 10 vol. III), the blocks as
// tar1090 lists them; in the JS ICAO_BLOCKS order (sorted by start; some
// territories sit inside a country's block, as there).
const IcaoBlock kIcao[] = {
    {0x004000, 0x0047FF, "Zimbabwe"},
    {0x006000, 0x006FFF, "Mozambique"},
    {0x008000, 0x00FFFF, "South Africa"},
    {0x010000, 0x017FFF, "Egypt"},
    {0x018000, 0x01FFFF, "Libya"},
    {0x020000, 0x027FFF, "Morocco"},
    {0x028000, 0x02FFFF, "Tunisia"},
    {0x030000, 0x0307FF, "Botswana"},
    {0x032000, 0x032FFF, "Burundi"},
    {0x034000, 0x034FFF, "Cameroon"},
    {0x035000, 0x0357FF, "Comoros"},
    {0x036000, 0x036FFF, "Republic of the Congo"},
    {0x038000, 0x038FFF, "C\xC3\xB4te d\xE2\x80\x99Ivoire"},
    {0x03E000, 0x03EFFF, "Gabon"},
    {0x040000, 0x040FFF, "Ethiopia"},
    {0x042000, 0x042FFF, "Equatorial Guinea"},
    {0x044000, 0x044FFF, "Ghana"},
    {0x046000, 0x046FFF, "Guinea"},
    {0x048000, 0x0487FF, "Guinea-Bissau"},
    {0x04A000, 0x04A7FF, "Lesotho"},
    {0x04C000, 0x04CFFF, "Kenya"},
    {0x050000, 0x050FFF, "Liberia"},
    {0x054000, 0x054FFF, "Madagascar"},
    {0x058000, 0x058FFF, "Malawi"},
    {0x05A000, 0x05A7FF, "Maldives"},
    {0x05C000, 0x05CFFF, "Mali"},
    {0x05E000, 0x05E7FF, "Mauritania"},
    {0x060000, 0x0607FF, "Mauritius"},
    {0x062000, 0x062FFF, "Niger"},
    {0x064000, 0x064FFF, "Nigeria"},
    {0x068000, 0x068FFF, "Uganda"},
    {0x06A000, 0x06AFFF, "Qatar"},
    {0x06C000, 0x06CFFF, "Central African Republic"},
    {0x06E000, 0x06EFFF, "Rwanda"},
    {0x070000, 0x070FFF, "Senegal"},
    {0x074000, 0x0747FF, "Seychelles"},
    {0x076000, 0x0767FF, "Sierra Leone"},
    {0x078000, 0x078FFF, "Somalia"},
    {0x07A000, 0x07A7FF, "Eswatini"},
    {0x07C000, 0x07CFFF, "Sudan"},
    {0x080000, 0x080FFF, "Tanzania"},
    {0x084000, 0x084FFF, "Chad"},
    {0x088000, 0x088FFF, "Togo"},
    {0x08A000, 0x08AFFF, "Zambia"},
    {0x08C000, 0x08CFFF, "DR Congo"},
    {0x090000, 0x090FFF, "Angola"},
    {0x094000, 0x0947FF, "Benin"},
    {0x096000, 0x0967FF, "Cabo Verde"},
    {0x098000, 0x0987FF, "Djibouti"},
    {0x09A000, 0x09AFFF, "Gambia"},
    {0x09C000, 0x09CFFF, "Burkina Faso"},
    {0x09E000, 0x09E7FF, "S\xC3\xA3o Tom\xC3\xA9 and Pr\xC3\xADncipe"},
    {0x0A0000, 0x0A7FFF, "Algeria"},
    {0x0A8000, 0x0A8FFF, "Bahamas"},
    {0x0AA000, 0x0AA7FF, "Barbados"},
    {0x0AB000, 0x0AB7FF, "Belize"},
    {0x0AC000, 0x0ADFFF, "Colombia"},
    {0x0AE000, 0x0AEFFF, "Costa Rica"},
    {0x0B0000, 0x0B0FFF, "Cuba"},
    {0x0B2000, 0x0B2FFF, "El Salvador"},
    {0x0B4000, 0x0B4FFF, "Guatemala"},
    {0x0B6000, 0x0B6FFF, "Guyana"},
    {0x0B8000, 0x0B8FFF, "Haiti"},
    {0x0BA000, 0x0BAFFF, "Honduras"},
    {0x0BC000, 0x0BC7FF, "Saint Vincent and the Grenadines"},
    {0x0BE000, 0x0BEFFF, "Jamaica"},
    {0x0C0000, 0x0C0FFF, "Nicaragua"},
    {0x0C2000, 0x0C2FFF, "Panama"},
    {0x0C4000, 0x0C4FFF, "Dominican Republic"},
    {0x0C6000, 0x0C6FFF, "Trinidad and Tobago"},
    {0x0C8000, 0x0C8FFF, "Suriname"},
    {0x0CA000, 0x0CA7FF, "Antigua and Barbuda"},
    {0x0CC000, 0x0CC7FF, "Grenada"},
    {0x0D0000, 0x0D7FFF, "Mexico"},
    {0x0D8000, 0x0DFFFF, "Venezuela"},
    {0x100000, 0x1FFFFF, "Russia"},
    {0x201000, 0x2017FF, "Namibia"},
    {0x202000, 0x2027FF, "Eritrea"},
    {0x300000, 0x33FFFF, "Italy"},
    {0x340000, 0x37FFFF, "Spain"},
    {0x380000, 0x3BFFFF, "France"},
    {0x3C0000, 0x3FFFFF, "Germany"},
    {0x400000, 0x4001BF, "Bermuda"},
    {0x400000, 0x43FFFF, "UK"},
    {0x4001C0, 0x4001FF, "Cayman Islands"},
    {0x400300, 0x4003FF, "Turks and Caicos Islands"},
    {0x424135, 0x4241F2, "Cayman Islands"},
    {0x424200, 0x4246FF, "Bermuda"},
    {0x424700, 0x424899, "Cayman Islands"},
    {0x424B00, 0x424BFF, "Isle of Man"},
    {0x43BE00, 0x43BEFF, "Bermuda"},
    {0x43E700, 0x43EAFD, "Isle of Man"},
    {0x43EAFE, 0x43EEFF, "Guernsey"},
    {0x440000, 0x447FFF, "Austria"},
    {0x448000, 0x44FFFF, "Belgium"},
    {0x450000, 0x457FFF, "Bulgaria"},
    {0x458000, 0x45FFFF, "Denmark"},
    {0x460000, 0x467FFF, "Finland"},
    {0x468000, 0x46FFFF, "Greece"},
    {0x470000, 0x477FFF, "Hungary"},
    {0x478000, 0x47FFFF, "Norway"},
    {0x480000, 0x487FFF, "Netherlands"},
    {0x488000, 0x48FFFF, "Poland"},
    {0x490000, 0x497FFF, "Portugal"},
    {0x498000, 0x49FFFF, "Czechia"},
    {0x4A0000, 0x4A7FFF, "Romania"},
    {0x4A8000, 0x4AFFFF, "Sweden"},
    {0x4B0000, 0x4B7FFF, "Switzerland"},
    {0x4B8000, 0x4BFFFF, "Turkey"},
    {0x4C0000, 0x4C7FFF, "Serbia"},
    {0x4C8000, 0x4C87FF, "Cyprus"},
    {0x4CA000, 0x4CAFFF, "Ireland"},
    {0x4CC000, 0x4CCFFF, "Iceland"},
    {0x4D0000, 0x4D07FF, "Luxembourg"},
    {0x4D2000, 0x4D27FF, "Malta"},
    {0x4D4000, 0x4D47FF, "Monaco"},
    {0x500000, 0x5007FF, "San Marino"},
    {0x501000, 0x5017FF, "Albania"},
    {0x501800, 0x501FFF, "Croatia"},
    {0x502800, 0x502FFF, "Latvia"},
    {0x503800, 0x503FFF, "Lithuania"},
    {0x504800, 0x504FFF, "Moldova"},
    {0x505800, 0x505FFF, "Slovakia"},
    {0x506800, 0x506FFF, "Slovenia"},
    {0x507800, 0x507FFF, "Uzbekistan"},
    {0x508000, 0x50FFFF, "Ukraine"},
    {0x510000, 0x5107FF, "Belarus"},
    {0x511000, 0x5117FF, "Estonia"},
    {0x512000, 0x5127FF, "North Macedonia"},
    {0x513000, 0x5137FF, "Bosnia and Herzegovina"},
    {0x514000, 0x5147FF, "Georgia"},
    {0x515000, 0x5157FF, "Tajikistan"},
    {0x516000, 0x5167FF, "Montenegro"},
    {0x600000, 0x6007FF, "Armenia"},
    {0x600800, 0x600FFF, "Azerbaijan"},
    {0x601000, 0x6017FF, "Kyrgyzstan"},
    {0x601800, 0x601FFF, "Turkmenistan"},
    {0x680000, 0x6807FF, "Bhutan"},
    {0x681000, 0x6817FF, "Micronesia, Federated States of"},
    {0x682000, 0x6827FF, "Mongolia"},
    {0x683000, 0x6837FF, "Kazakhstan"},
    {0x684000, 0x6847FF, "Palau"},
    {0x700000, 0x700FFF, "Afghanistan"},
    {0x702000, 0x702FFF, "Bangladesh"},
    {0x704000, 0x704FFF, "Myanmar"},
    {0x706000, 0x706FFF, "Kuwait"},
    {0x708000, 0x708FFF, "Laos"},
    {0x70A000, 0x70AFFF, "Nepal"},
    {0x70C000, 0x70C7FF, "Oman"},
    {0x70E000, 0x70EFFF, "Cambodia"},
    {0x710000, 0x717FFF, "Saudi Arabia"},
    {0x718000, 0x71FFFF, "South Korea"},
    {0x720000, 0x727FFF, "North Korea"},
    {0x728000, 0x72FFFF, "Iraq"},
    {0x730000, 0x737FFF, "Iran"},
    {0x738000, 0x73FFFF, "Israel"},
    {0x740000, 0x747FFF, "Jordan"},
    {0x748000, 0x74FFFF, "Lebanon"},
    {0x750000, 0x757FFF, "Malaysia"},
    {0x758000, 0x75FFFF, "Philippines"},
    {0x760000, 0x767FFF, "Pakistan"},
    {0x768000, 0x76FFFF, "Singapore"},
    {0x770000, 0x777FFF, "Sri Lanka"},
    {0x778000, 0x77FFFF, "Syria"},
    {0x780000, 0x7BFFFF, "China"},
    {0x789000, 0x789FFF, "Hong Kong"},
    {0x7C0000, 0x7FFFFF, "Australia"},
    {0x800000, 0x83FFFF, "India"},
    {0x840000, 0x87FFFF, "Japan"},
    {0x880000, 0x887FFF, "Thailand"},
    {0x888000, 0x88FFFF, "Viet Nam"},
    {0x890000, 0x890FFF, "Yemen"},
    {0x894000, 0x894FFF, "Bahrain"},
    {0x895000, 0x8957FF, "Brunei"},
    {0x896000, 0x896FFF, "UAE"},
    {0x897000, 0x8977FF, "Solomon Islands"},
    {0x898000, 0x898FFF, "Papua New Guinea"},
    {0x899000, 0x8997FF, "Taiwan"},
    {0x8A0000, 0x8A7FFF, "Indonesia"},
    {0x900000, 0x9007FF, "Marshall Islands"},
    {0x901000, 0x9017FF, "Cook Islands"},
    {0x902000, 0x9027FF, "Samoa"},
    {0xA00000, 0xAFFFFF, "USA"},
    {0xC00000, 0xC3FFFF, "Canada"},
    {0xC80000, 0xC87FFF, "New Zealand"},
    {0xC88000, 0xC88FFF, "Fiji"},
    {0xC8A000, 0xC8A7FF, "Nauru"},
    {0xC8C000, 0xC8C7FF, "Saint Lucia"},
    {0xC8D000, 0xC8D7FF, "Tonga"},
    {0xC8E000, 0xC8E7FF, "Kiribati"},
    {0xC90000, 0xC907FF, "Vanuatu"},
    {0xC91000, 0xC917FF, "Andorra"},
    {0xC92000, 0xC927FF, "Dominica"},
    {0xC93000, 0xC937FF, "Saint Kitts and Nevis"},
    {0xC94000, 0xC947FF, "South Sudan"},
    {0xC95000, 0xC957FF, "Timor-Leste"},
    {0xC97000, 0xC977FF, "Tuvalu"},
    {0xE00000, 0xE3FFFF, "Argentina"},
    {0xE40000, 0xE7FFFF, "Brazil"},
    {0xE80000, 0xE80FFF, "Chile"},
    {0xE84000, 0xE84FFF, "Ecuador"},
    {0xE88000, 0xE88FFF, "Paraguay"},
    {0xE8C000, 0xE8CFFF, "Peru"},
    {0xE90000, 0xE90FFF, "Uruguay"},
    {0xE94000, 0xE94FFF, "Bolivia"},
    {0xF00000, 0xF07FFF, "ICAO (temporary)"},
    {0xF09000, 0xF097FF, "ICAO (special use)"},
};

}  // namespace

// ===========================================================================
// Channels
// ===========================================================================
const std::vector<double>& vdl2AllChannels() {
    static const double f[] = {136.650e6, 136.675e6, 136.700e6, 136.725e6, 136.775e6,
                               136.800e6, 136.825e6, 136.875e6, 136.900e6, 136.975e6};
    static const std::vector<double> v(f, f + sizeof f / sizeof f[0]);
    return v;
}

std::vector<double> vdl2ChannelsFor(double lat, double lon) {
    std::vector<double> v;
    if (lat > 30 && lat < 72 && lon > -30 && lon < 45) {           // (Europe)
        const double e[] = {136.675e6, 136.725e6, 136.775e6, 136.825e6, 136.875e6, 136.975e6};
        v.assign(e, e + 6);
    } else if (lat > 12 && lat < 75 && lon > -170 && lon < -50) {  // (North America)
        const double a[] = {136.650e6, 136.700e6, 136.800e6, 136.975e6};
        v.assign(a, a + 4);
    } else {
        v.push_back(vdl2CommonChannelHz);                             // (the common channel, worldwide)
    }
    return v;
}

// ===========================================================================
// Coding pieces
// ===========================================================================
const int vdl2Preamble[16] = {0, 3, -3, 1, 1, 2, 0, 4, -3, 4, -2, 3, 1, -2, -3, 0};
const int vdl2Gray[8] = {0, 1, 3, 2, 6, 7, 5, 4};

void vdl2RsEncode(const unsigned char data[249], unsigned char p[6]) {
    const Gf& G = gf();
    for (int j = 0; j < 6; ++j) p[j] = 0;
    for (int i = 0; i < 249; ++i) {
        const unsigned char fb = data[i] ^ p[0];
        for (int j = 0; j < 5; ++j) p[j] = p[j + 1] ^ G.mul(fb, G.gen[j + 1]);
        p[5] = G.mul(fb, G.gen[6]);
    }
}

// RS(255,249) decoding with erasures (first root 120, primitive 1; position
// i = degree 254 - i): Berlekamp-Massey from the erasure locator, Chien
// search, Forney.  Returns the number corrected, or -1.
int vdl2RsDecode(unsigned char c[255], const int* erasures, int ne) {
    const Gf& G = gf();
    if (ne < 0 || ne > 6) return -1;
    unsigned char S[6];
    bool any = false;
    for (int j = 0; j < 6; ++j) {
        unsigned char s = 0;
        const unsigned char r = G.pw(120 + j);
        for (int i = 0; i < 255; ++i) s = G.mul(s, r) ^ c[i];
        S[j] = s;
        if (s) any = true;
    }
    if (!any) return 0;
    // (the lengths are tracked as the JS arrays' lengths, trailing zeros too)
    unsigned char lam[32] = {0}, B[32] = {0}, T[32] = {0};
    int lamLen = 1, BLen;
    lam[0] = 1;
    for (int k = 0; k < ne; ++k) {
        const unsigned char X = G.pw(254 - erasures[k]);
        unsigned char n[32] = {0};
        for (int i = 0; i < lamLen; ++i) n[i] = lam[i];
        n[lamLen] = 0;
        for (int i = lamLen; i > 0; --i) n[i] ^= G.mul(lam[i - 1], X);
        ++lamLen;
        for (int i = 0; i < lamLen; ++i) lam[i] = n[i];
    }
    for (int i = 0; i < lamLen; ++i) B[i] = lam[i];
    BLen = lamLen;
    int L = ne;
    for (int r = ne; r < 6; ++r) {
        unsigned char d = 0;
        for (int i = 0; i < lamLen; ++i)
            if (r - i >= 0) d ^= G.mul(lam[i], S[r - i]);
        // B = [0].concat(B)
        for (int i = BLen; i > 0; --i) B[i] = B[i - 1];
        B[0] = 0;
        ++BLen;
        if (BLen > 30) return -1;
        if (d) {
            int TLen = lamLen;
            for (int i = 0; i < lamLen; ++i) T[i] = lam[i];
            while (TLen < BLen) T[TLen++] = 0;
            for (int i = 0; i < BLen; ++i) T[i] ^= G.mul(d, B[i]);
            if (2 * L <= r + ne) {
                L = r + 1 + ne - L;
                for (int i = 0; i < lamLen; ++i) B[i] = G.div(lam[i], d);
                BLen = lamLen;
            }
            for (int i = 0; i < TLen; ++i) lam[i] = T[i];
            lamLen = TLen;
        }
    }
    while (lamLen > 1 && !lam[lamLen - 1]) --lamLen;
    const int deg = lamLen - 1;
    if (deg > 6) return -1;
    unsigned char om[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j <= std::min(i, deg); ++j) om[i] ^= G.mul(lam[j], S[i - j]);
    int n = 0;
    for (int pos = 0; pos < 255; ++pos) {
        const unsigned char Xinv = G.pw(-(254 - pos));
        unsigned char v = 0;
        for (int i = deg; i >= 0; --i) v = G.mul(v, Xinv) ^ lam[i];
        if (v) continue;
        unsigned char num = 0;
        for (int i = 5; i >= 0; --i) num = G.mul(num, Xinv) ^ om[i];
        unsigned char den = 0;
        for (int i = 1; i <= deg; i += 2) den ^= G.mul(lam[i], G.pw(-(254 - pos) * (i - 1)));
        if (!den) return -1;
        // (Forney: X^(1 - 120) Omega(1/X) / Lambda'(1/X))
        c[pos] ^= G.div(G.mul(num, G.pw((254 - pos) * (1 - 120))), den);
        ++n;
    }
    return n == deg ? n : -1;
}

unsigned vdl2Crc(const unsigned char* b, std::size_t n) {
    unsigned c = 0xffff;
    for (std::size_t i = 0; i < n; ++i) {
        c ^= b[i];
        for (int k = 0; k < 8; ++k) c = (c & 1) ? (c >> 1) ^ 0x8408u : c >> 1;
    }
    return (c ^ 0xffffu) & 0xffffu;
}

unsigned vdl2HeaderSyndrome(unsigned h) {
    unsigned s = 0;
    for (int i = 0; i < 5; ++i) {
        unsigned p = h & kHm[i], q = 0;
        while (p) {
            q ^= p & 1;
            p >>= 1;
        }
        s |= q << (4 - i);
    }
    return s;
}

const char* icaoCountry(unsigned a) {
    int lo = 0, hi = static_cast<int>(sizeof kIcao / sizeof kIcao[0]) - 1;
    while (lo <= hi) {
        const int m = (lo + hi) >> 1;
        if (a < kIcao[m].lo) hi = m - 1;
        else if (a > kIcao[m].hi) lo = m + 1;
        else return kIcao[m].name;
    }
    return "";
}

// ===========================================================================
// AVLC frames
// ===========================================================================
bool vdl2ParseFrame(const unsigned char* f, std::size_t n, Vdl2Frame& out) {
    if (n < 11) return false;
    addrOf(f, out.dst, out.dstType);
    addrOf(f + 4, out.src, out.srcType);
    const unsigned ctl = f[8];
    out.control = static_cast<unsigned char>(ctl);
    out.bytes.assign(f, f + n);
    const unsigned char* info = f + 9;
    const std::size_t ni = n - 11;
    out.kind = (ctl & 1) == 0 ? 'I' : (ctl & 3) == 1 ? 'S' : 'U';
    out.isAcars = out.atn = false;
    out.what.clear();
    if (out.kind == 'I' && ni > 3 && info[0] == 0xff && info[1] == 0xff && info[2] == 0x01) {
        // the ACARS block, parity bits off, up to the ETX (or ETB)
        const unsigned char* a = info + 3;
        const std::size_t na = ni - 3;
        long e = -1;
        // Addition to the JS: the ETX where it belongs, before the CRC and DEL
        if (na >= 4 && ((a[na - 4] & 0x7f) == 0x03 || (a[na - 4] & 0x7f) == 0x17)) e = static_cast<long>(na) - 4;
        for (long i = static_cast<long>(na) - 1; e < 0 && i >= 0; --i)
            if ((a[i] & 0x7f) == 0x03) e = i;
        for (long i = static_cast<long>(na) - 1; e < 0 && i >= 0; --i)
            if ((a[i] & 0x7f) == 0x17) e = i;
        const std::size_t len = e > 0 ? static_cast<std::size_t>(e) : na;
        if (acarsParse(a, len, out.acars)) out.isAcars = true;
        else out.atn = true;   // (too short to be ACARS)
    } else if (out.kind == 'I') {
        out.atn = true;
    } else if (out.kind == 'S') {
        static const char* const kS[4] = {"Receive Ready", "Reject", "Receive Not Ready", "Selective Reject"};
        out.what = kS[(ctl >> 2) & 3];
    } else {
        out.what = (ctl & 0xef) == 0xaf ? "XID" : "link control";
    }
    return true;
}

namespace {
std::string tagOf(unsigned a, int t) {
    if (t == 1) {
        const char* c = icaoCountry(a);
        return *c ? std::string(" (") + c + ")" : std::string();
    }
    const char* n = t >= 0 && t < 8 ? kTypes[t] : "";
    return std::string(" (") + (*n ? n : "?") + ")";
}
}  // namespace

std::string Vdl2Frame::headLine() const {
    std::string s = mhz(freqHz) + " \xC2\xB7 " + hex6(src) + tagOf(src, srcType) + " \xE2\x86\x92 " + hex6(dst) +
                    tagOf(dst, dstType);
    if (isAcars) {
        s += " \xC2\xB7 " + (acars.reg.empty() ? std::string("?") : acars.reg);
        if (!acars.flight.empty()) s += " \xC2\xB7 " + acars.flight;
        s += " \xC2\xB7 label " + acars.label;
    }
    return s;
}

std::string Vdl2Frame::body() const {
    if (isAcars) return acars.text.empty() ? std::string("(no text)") : acars.text;
    if (atn) return "ATN data (controller\xE2\x80\x93pilot datalink and other network messages)";
    if (what == "XID") return "Link management (XID)";
    if (what == "link control") return "Link control";   // Addition to the JS (see the file header)
    if (!what.empty()) return "Acknowledgement (" + what + ")";
    return "Link control";
}

// ===========================================================================
// One channel: its own mixer and decimator (the JS VdlChan), and the burst
// receiver (the JS VdlRx)
// ===========================================================================
struct Vdl2Decoder::Chan {
    explicit Chan(double f) : f(f), rate(0) { clear(); }

    double f;           // the channel, Hz
    // --- VdlChan ---
    double rate;        // input rate (the shared stage's)
    int D1, D2;
    double fs1, fs2;
    double h[31];
    double zr2[62], zi2[62];
    int zk, k1, k2;
    double ar, ai, mr, mi;
    std::vector<float> outr, outi;
    // --- VdlRx ---
    double sps;
    std::vector<double> taps, fr2, fi2;
    int nt, fp;
    double noise;
    bool noiseSet;
    std::vector<double> initP;   // the first powers, for the first noise floor
    double hr[64], hi[64];
    unsigned hn;
    bool inBurst;
    std::vector<float> br, bi;   // the burst being gathered
    int lowRun;
    std::size_t maxB;
    unsigned bursts, good, rsFail, crcFail;
    // burst decoding work space
    std::vector<unsigned char> bits, blocks, data, bl, frame;

    void clear() {
        rate = 0;
        zk = k1 = k2 = 0;
        ar = ai = 0;
        mr = 1;
        mi = 0;
        fp = 0;
        noise = 1e-6;
        noiseSet = false;
        initP.clear();
        hn = 0;
        inBurst = false;
        br.clear();
        bi.clear();
        lowRun = 0;
        bursts = good = rsFail = crcFail = 0;
    }

    void setup(double r) {
        rate = r;
        D1 = std::max(1, static_cast<int>(std::floor(r / 256000 + 0.5)));
        fs1 = r / D1;
        D2 = std::max(1, static_cast<int>(std::floor(fs1 / 51200 + 0.5)));
        fs2 = fs1 / D2;
        // a 31-tap low-pass (Hamming-windowed sinc, 20 kHz)
        const int N = 31;
        const double fc = 20000 / fs1;
        double sum = 0;
        for (int i = 0; i < N; ++i) {
            const double x = i - (N - 1) / 2.0, w = 0.54 - 0.46 * std::cos(kTwoPi * i / (N - 1));
            h[i] = (x != 0 ? std::sin(kTwoPi * fc * x) / (kPi * x) : 2 * fc) * w;
            sum += h[i];
        }
        for (int i = 0; i < N; ++i) h[i] /= sum;
        for (int i = 0; i < 62; ++i) zr2[i] = zi2[i] = 0;
        zk = k1 = k2 = 0;
        ar = ai = 0;
        mr = 1;
        mi = 0;
        setupRx(fs2);
    }

    // VdlRx's constructor: a low-pass filter to the signal's own width, about
    // +-8.4 kHz (windowed sinc, Blackman, cut-off 9 kHz)
    void setupRx(double r) {
        sps = r / 10500;
        nt = 2 * static_cast<int>(std::floor(r / 10500 * 3 + 0.5)) + 1;
        const double fc = 9000 / r;
        taps.assign(static_cast<std::size_t>(nt), 0.0);
        double sum = 0;
        for (int i = 0; i < nt; ++i) {
            const double x = i - (nt - 1) / 2.0;
            const double w = 0.42 - 0.5 * std::cos(kTwoPi * i / (nt - 1)) + 0.08 * std::cos(2 * kTwoPi * i / (nt - 1));
            taps[static_cast<std::size_t>(i)] = (x != 0 ? std::sin(kTwoPi * fc * x) / (kPi * x) : 2 * fc) * w;
            sum += taps[static_cast<std::size_t>(i)];
        }
        for (int i = 0; i < nt; ++i) taps[static_cast<std::size_t>(i)] /= sum;
        fr2.assign(static_cast<std::size_t>(2 * nt), 0.0);
        fi2.assign(static_cast<std::size_t>(2 * nt), 0.0);
        fp = 0;
        noise = 1e-6;
        noiseSet = false;
        initP.clear();
        initP.reserve(1024);
        hn = 0;
        inBurst = false;
        lowRun = 0;
        maxB = static_cast<std::size_t>(12000 * sps);
        br.clear();
        bi.clear();
        br.reserve(maxB + 64);
        bi.reserve(maxB + 64);
    }

    // The channel's samples at the shared stage's rate `r`, `off` Hz from its
    // mixing frequency: mixed to 0 Hz, a boxcar of D1, the FIR every D2.
    void process(const float* I, const float* Q, std::size_t n, double r, double off, Vdl2Decoder& owner) {
        if (r != rate) setup(r);
        const double w = -kTwoPi * off / r, cw = std::cos(w), sw = std::sin(w);
        const int N = 31;
        const std::size_t cap = n / static_cast<std::size_t>(D1 * D2) + 2;
        if (outr.size() < cap) {
            outr.resize(cap);
            outi.resize(cap);
        }
        std::size_t no = 0;
        for (std::size_t j = 0; j < n; ++j) {
            const double x = I[j], y = Q[j];
            ar += x * mr - y * mi;
            ai += x * mi + y * mr;
            const double t = mr * cw - mi * sw;
            mi = mr * sw + mi * cw;
            mr = t;
            if (++k1 == D1) {
                k1 = 0;
                const double g = 1.5 - 0.5 * (mr * mr + mi * mi);
                mr *= g;
                mi *= g;
                zr2[zk] = zr2[zk + N] = ar / D1;
                zi2[zk] = zi2[zk + N] = ai / D1;
                zk = zk + 1 == N ? 0 : zk + 1;
                ar = ai = 0;
                if (++k2 == D2) {
                    k2 = 0;
                    double yr = 0, yi = 0;
                    for (int k = 0, q = zk; k < N; ++k, ++q) {
                        yr += h[k] * zr2[q];
                        yi += h[k] * zi2[q];
                    }
                    outr[no] = static_cast<float>(yr);
                    outi[no] = static_cast<float>(yi);
                    ++no;
                }
            }
        }
        if (!finite(mr) || !finite(mi) || !finite(ar) || !finite(ai)) {   // Addition to the JS
            const double keep = rate;
            clear();
            setup(keep);
            return;
        }
        if (no) push(&outr[0], &outi[0], no, owner);
    }

    // VdlRx.push: bursts found by their power, each decoded whole
    void push(const float* re, const float* im, std::size_t n, Vdl2Decoder& owner) {
        const double* T = &taps[0];
        double* FR = &fr2[0];
        double* FI = &fi2[0];
        const double lim4 = 4, lim16 = 16, quiet = 3 * sps;
        for (std::size_t i0 = 0; i0 < n; ++i0) {
            // (the filter reads a doubled buffer straight through, with no
            //  wrapping; the history before a burst is a ring)
            int p0 = fp;
            FR[p0] = FR[p0 + nt] = re[i0];
            FI[p0] = FI[p0 + nt] = im[i0];
            p0 = p0 + 1 == nt ? 0 : p0 + 1;
            fp = p0;
            double yr = 0, yi = 0;
            for (int k = 0, j = p0 + nt - 1; k < nt; ++k, --j) {
                yr += T[k] * FR[j];
                yi += T[k] * FI[j];
            }
            const double p = yr * yr + yi * yi;
            if (!noiseSet) {
                // Addition to the JS: the first noise floor from the median
                // power of the first 1024 samples, however they arrive (the
                // JS: of the first block, from the raw samples).
                const double raw = static_cast<double>(re[i0]) * re[i0] + static_cast<double>(im[i0]) * im[i0];
                initP.push_back(raw);
                if (initP.size() >= 1024) {
                    std::sort(initP.begin(), initP.end());
                    noise = initP[initP.size() >> 1];
                    if (!(noise > 0)) noise = 1e-9;
                    noiseSet = true;
                    initP.clear();
                }
                continue;
            }
            noise += (std::min(p, noise * lim4) - noise) * 0.0005;
            if (inBurst) {
                br.push_back(static_cast<float>(yr));
                bi.push_back(static_cast<float>(yi));
                lowRun = p < noise * lim4 ? lowRun + 1 : 0;
                if (lowRun > quiet || br.size() > maxB) {
                    burst(owner);
                    inBurst = false;
                    br.clear();
                    bi.clear();
                }
            } else {
                const unsigned hh = hn & 63;
                hr[hh] = yr;
                hi[hh] = yi;
                ++hn;
                if (p > noise * lim16) {
                    const unsigned m = std::min(40u, hn);
                    br.clear();
                    bi.clear();
                    for (unsigned k = hn - m; k < hn; ++k) {
                        br.push_back(static_cast<float>(hr[k & 63]));
                        bi.push_back(static_cast<float>(hi[k & 63]));
                    }
                    inBurst = true;
                    lowRun = 0;
                    hn = 0;
                }
            }
        }
        if (!finite(noise)) noise = 1e-6;   // Addition to the JS
    }

    // cubic interpolation (under 5 samples a symbol)
    static double cub(const float* a, int i, double f) {
        const double p0 = a[i - 1], p1 = a[i], p2 = a[i + 1], p3 = a[i + 2];
        return p1 + 0.5 * f * (p2 - p0 + f * (2 * p0 - 5 * p1 + 4 * p2 - p3 + f * (3 * (p1 - p2) + p3 - p0)));
    }
    void at(double t, double* out) const {
        const int n = static_cast<int>(br.size());
        const int i = static_cast<int>(std::floor(t));
        const double fr = t - i;
        if (i < 1 || i + 2 >= n) {
            out[0] = out[1] = 0;
            return;
        }
        out[0] = cub(&br[0], i, fr);
        out[1] = cub(&bi[0], i, fr);
    }

    struct Fit {
        double t0, s, off;
    };
    // the preamble: each symbol times the previous one's conjugate and the
    // expected step's inverse, normalised and summed: 15 for a perfect fit,
    // its angle the frequency offset per symbol
    Fit fit(double t0, const double* er, const double* ei) const {
        double u[2], v[2];
        at(t0, u);
        double sr = 0, si = 0;
        for (int k = 1; k < 16; ++k) {
            at(t0 + k * sps, v);
            const double pr = v[0] * u[0] + v[1] * u[1], pi = v[1] * u[0] - v[0] * u[1];
            const double qr = pr * er[k - 1] - pi * ei[k - 1], qi = pr * ei[k - 1] + pi * er[k - 1];
            double m = std::sqrt(qr * qr + qi * qi);
            if (!(m > 0)) m = 1;
            sr += qr / m;
            si += qi / m;
            u[0] = v[0];
            u[1] = v[1];
        }
        Fit r;
        r.t0 = t0;
        r.s = std::sqrt(sr * sr + si * si);
        r.off = std::atan2(si, sr);
        return r;
    }

    void burst(Vdl2Decoder& owner) {
        ++bursts;
        const int n = static_cast<int>(br.size());
        if (n < 40 * sps) return;
        double er[15], ei[15];
        for (int k = 1; k < 16; ++k) {
            const double a = (vdl2Preamble[k] - vdl2Preamble[k - 1]) * kPi / 4;
            er[k - 1] = std::cos(a);
            ei[k - 1] = -std::sin(a);
        }
        // half-sample steps over the first 25 symbols, then sixteenths around
        // the best (the burst detector has the start)
        const double tmax = std::min(n - 20 * sps, 25 * sps);
        bool have = false;
        Fit best;
        best.t0 = best.s = best.off = 0;
        for (double t0 = 1; t0 < tmax; t0 += 0.5) {
            const Fit r = fit(t0, er, ei);
            if (!have || r.s > best.s) {
                best = r;
                have = true;
            }
        }
        if (!have) return;
        // (as the JS: the loop's bound follows the best as it moves)
        for (double t0 = best.t0 - 0.5; t0 <= best.t0 + 0.5; t0 += 0.0625) {
            if (t0 < 1) continue;
            const Fit r = fit(t0, er, ei);
            if (r.s > best.s) best = r;
        }
        double v[2];
        at(best.t0 + 15 * sps, v);
        double prev = std::atan2(v[1], v[0]);
        if (2 * (15 - best.s) > 1.5) return;
        // the symbols: each phase step, less the offset, to the nearest eighth
        // of a turn; the offset tracked as it goes
        bits.clear();
        double off = best.off;
        for (int k = 16;; ++k) {
            const double t = best.t0 + k * sps;
            if (t + 1 >= n) break;
            at(t, v);
            const double ph = std::atan2(v[1], v[0]);
            double d = ph - prev - off;
            d = std::fmod(std::fmod(d, kTwoPi) + kTwoPi, kTwoPi);
            const int q = static_cast<int>(std::floor(d / (kPi / 4) + 0.5)) % 8;
            double e = d - q * kPi / 4;
            if (e > kPi) e -= kTwoPi;
            off += e * 0.05;
            prev = ph;
            const int g = vdl2Gray[q];
            bits.push_back(static_cast<unsigned char>((g >> 2) & 1));
            bits.push_back(static_cast<unsigned char>((g >> 1) & 1));
            bits.push_back(static_cast<unsigned char>(g & 1));
        }
        unsigned lfsr = 0x6959;
        for (std::size_t i = 0; i < bits.size(); ++i) {
            const unsigned b = (lfsr ^ (lfsr >> 14)) & 1;
            lfsr = (lfsr >> 1) | (b << 14);
            bits[i] ^= static_cast<unsigned char>(b);
        }
        if (bits.size() < 25) return;
        unsigned hd = 0;
        for (int i = 0; i < 25; ++i) hd = (hd << 1) | bits[static_cast<std::size_t>(i)];
        hd &= (1u << 22) - 1;   // (the 3 reserved bits forced to 0)
        const int fix = synTab().t[vdl2HeaderSyndrome(hd)];
        if (fix < 0) { ++rsFail; return; }
        hd ^= static_cast<unsigned>(fix);
        if (hd >> 22) { ++rsFail; return; }
        int datalen = 0;
        const unsigned lenRev = (hd >> 5) & 0x1ffffu;
        for (int i = 0; i < 17; ++i)
            if ((lenRev >> i) & 1) datalen |= 1 << (16 - i);
        if (datalen < 16 || datalen > 8 * 249 * 8) { ++rsFail; return; }
        const int octets = (datalen + 7) / 8, nb = (octets + 248) / 249, lastLen = octets - (nb - 1) * 249;
        const int fecTotal = (nb - 1) * 6 + fecOf(lastLen);
        const std::size_t need = static_cast<std::size_t>(25 + 8 * (octets + fecTotal));
        if (bits.size() < need) { ++rsFail; return; }
        blocks.assign(static_cast<std::size_t>(nb) * 255, 0);
        int row = 0, col = 0;
        for (int i = 0; i < octets; ++i) {
            if (row == nb - 1 && col >= lastLen) {
                row = 0;
                ++col;
            }
            blocks[static_cast<std::size_t>(row++) * 255 + col] = byteAt(i);
            if (row == nb) {
                row = 0;
                ++col;
            }
        }
        const int fecRows = fecOf(lastLen) ? nb : nb - 1;
        int fi = octets;
        for (int c = 0; c < 6; ++c)
            for (int r = 0; r < fecRows; ++r)
                if (c < (r == nb - 1 ? fecOf(lastLen) : 6)) blocks[static_cast<std::size_t>(r) * 255 + 249 + c] = byteAt(fi++);
        for (int r = 0; r < nb; ++r) {
            const int nf = r == nb - 1 ? fecOf(lastLen) : 6;
            if (!nf) continue;
            int era[6], ne = 0;
            for (int c = nf; c < 6; ++c) era[ne++] = 249 + c;
            // (a short block: its unused data columns are known zeros; the code
            //  is shortened, so they stay as they are)
            if (vdl2RsDecode(&blocks[static_cast<std::size_t>(r) * 255], era, ne) < 0) { ++rsFail; return; }
        }
        data.clear();
        row = 0;
        col = 0;
        for (int i = 0; i < octets; ++i) {
            if (row == nb - 1 && col >= lastLen) {
                row = 0;
                ++col;
            }
            data.push_back(blocks[static_cast<std::size_t>(row++) * 255 + col]);
            if (row == nb) {
                row = 0;
                ++col;
            }
        }
        // the frames: HDLC flags and unstuffing, then each frame's CRC (a
        // flag's own leading 0 and five 1s, already gathered when its sixth 1
        // shows it's a flag, are taken back off before the frame is cut)
        bool haveBl = false;
        bl.clear();
        int ones = 0;
        for (int i = 0; i < datalen; ++i) {
            const int bit = (data[static_cast<std::size_t>(i >> 3)] >> (i & 7)) & 1;
            if (bit) {
                if (++ones > 6) {
                    haveBl = false;
                    continue;
                }
                if (ones < 6 && haveBl) bl.push_back(1);
                continue;
            }
            if (ones == 6) {
                if (haveBl) {
                    std::size_t len = bl.size() >= 6 ? bl.size() - 6 : 0;
                    if (len >= 88 && len % 8 == 0) {
                        frame.assign(len / 8, 0);
                        for (std::size_t j = 0; j < len; ++j) frame[j >> 3] |= static_cast<unsigned char>(bl[j] << (j & 7));
                        gotFrame(owner);
                    }
                }
                bl.clear();
                haveBl = true;
                ones = 0;
                continue;
            }
            if (ones == 5) {   // (a stuffed 0)
                ones = 0;
                continue;
            }
            ones = 0;
            if (haveBl) bl.push_back(0);
        }
    }

    unsigned char byteAt(int o) const {
        unsigned x = 0;
        for (int k = 0; k < 8; ++k) x |= static_cast<unsigned>(bits[static_cast<std::size_t>(25 + 8 * o + k)]) << k;
        return static_cast<unsigned char>(x);
    }

    void gotFrame(Vdl2Decoder& owner) {
        const std::size_t n = frame.size();
        const unsigned c = vdl2Crc(&frame[0], n - 2);
        if ((c & 255) != frame[n - 2] || (c >> 8) != frame[n - 1]) {
            ++crcFail;
            return;
        }
        ++good;
        owner.gotFrame(f, &frame[0], n);
    }
};

// ===========================================================================
// Vdl2Decoder
// ===========================================================================
Vdl2Decoder::Vdl2Decoder()
    : messagesOnly_(false), maxFrames_(300), mid_(0), rate_(0), mr_(1), mi_(0), pr_(0), pi_(0), k_(0), t_(0),
      lastFrameT_(-1e9), count_(0) {
    cr_[0] = cr_[1] = ci_[0] = ci_[1] = 0;
    setChannels(vdl2AllChannels());
}

Vdl2Decoder::~Vdl2Decoder() {
    for (std::size_t i = 0; i < rx_.size(); ++i) delete rx_[i];
}

void Vdl2Decoder::setChannels(const std::vector<double>& hz) {
    for (std::size_t i = 0; i < rx_.size(); ++i) delete rx_[i];
    rx_.clear();
    chans_.clear();
    for (std::size_t i = 0; i < hz.size(); ++i) {
        if (!finite(hz[i]) || !(hz[i] > 0)) continue;
        if (std::find(chans_.begin(), chans_.end(), hz[i]) != chans_.end()) continue;
        chans_.push_back(hz[i]);
    }
    std::sort(chans_.begin(), chans_.end());
    for (std::size_t i = 0; i < chans_.size(); ++i) rx_.push_back(new Chan(chans_[i]));
    inView_.clear();
    resetShared();
}

void Vdl2Decoder::setLocation(double lat, double lon) { setChannels(vdl2ChannelsFor(lat, lon)); }

void Vdl2Decoder::setMaxFrames(std::size_t n) {
    maxFrames_ = n ? n : 1;
    while (frames_.size() > maxFrames_) frames_.pop_front();
}

void Vdl2Decoder::resetShared() {
    mid_ = 0;
    rate_ = 0;
    mr_ = 1;
    mi_ = 0;
    cr_[0] = cr_[1] = ci_[0] = ci_[1] = 0;
    pr_ = pi_ = 0;
    k_ = 0;
    for (std::size_t i = 0; i < rx_.size(); ++i) rx_[i]->clear();
}

void Vdl2Decoder::reset() {
    resetShared();
    inView_.clear();
    t_ = 0;
    lastFrameT_ = -1e9;
    count_ = 0;
    frames_.clear();
    fresh_.clear();
    text_.clear();
    countF_.clear();
    countN_.clear();
}

void Vdl2Decoder::process(const float* audio, std::size_t n, double rate) {
    (void)audio;
    (void)n;
    (void)rate;
}

void Vdl2Decoder::processIq(const std::complex<float>* iq, std::size_t n, double rate, double centerHz, double dialHz) {
    (void)dialHz;
    if (!iq || !n || !(rate > 0) || !finite(rate) || !finite(centerHz)) return;   // Addition to the JS
    t_ += n / rate;
    // the channels in view, and their middle
    inView_.clear();
    double lo = 0, hi = 0;
    for (std::size_t i = 0; i < chans_.size(); ++i)
        if (std::fabs(chans_[i] - centerHz) < rate * 0.45) {
            if (inView_.empty() || chans_[i] < lo) lo = chans_[i];
            if (inView_.empty() || chans_[i] > hi) hi = chans_[i];
            inView_.push_back(i);
        }
    if (inView_.empty()) return;
    const double mid = (lo + hi) / 2;
    if (mid != mid_ || rate != rate_) {
        resetShared();
        mid_ = mid;
        rate_ = rate;
    }
    // (a shared first stage: the whole block of channels mixed down ONCE,
    //  around their middle, and reduced by 4 with a second-order CIC (two
    //  cascaded boxcars: aliases well down); each channel then works at a
    //  quarter of the stick's rate)
    const int R = 4;
    const double w = -kTwoPi * (mid - centerHz) / rate, cw = std::cos(w), sw = std::sin(w);
    const std::size_t cap = n / R + 2;
    if (sr_.size() < cap) {
        sr_.resize(cap);
        si_.resize(cap);
    }
    std::size_t no = 0;
    double mr = mr_, mi = mi_, ar = cr_[0], ai = ci_[0], br = cr_[1], bi = ci_[1], pr = pr_, pi = pi_;
    int k = k_;
    for (std::size_t j = 0; j < n; ++j) {
        double x = iq[j].real(), y = iq[j].imag();
        if (!finite(x) || !finite(y)) x = y = 0;   // Addition to the JS
        const double xr = x * mr - y * mi, xi = x * mi + y * mr;
        const double t = mr * cw - mi * sw;
        mi = mr * sw + mi * cw;
        mr = t;
        // Addition to the JS: the CIC as its 7-tap triangle (1 2 3 4 3 2 1):
        // sample j of a block of 4 weighs 4 - j now and j in the next output
        ar += (R - k) * xr;
        ai += (R - k) * xi;
        br += k * xr;
        bi += k * xi;
        if (++k == R) {
            k = 0;
            const double g = 1.5 - 0.5 * (mr * mr + mi * mi);
            mr *= g;
            mi *= g;
            sr_[no] = static_cast<float>((ar + pr) / (R * R));
            si_[no] = static_cast<float>((ai + pi) / (R * R));
            ++no;
            pr = br;
            pi = bi;
            ar = ai = br = bi = 0;
        }
    }
    mr_ = mr;
    mi_ = mi;
    cr_[0] = ar;
    ci_[0] = ai;
    cr_[1] = br;
    ci_[1] = bi;
    pr_ = pr;
    pi_ = pi;
    k_ = k;
    if (!finite(mr_) || !finite(mi_)) {   // Addition to the JS (cannot happen from finite input)
        resetShared();
        return;
    }
    if (!no) return;
    const double r2 = rate / R;
    for (std::size_t i = 0; i < inView_.size(); ++i) {
        Chan& c = *rx_[inView_[i]];
        c.process(&sr_[0], &si_[0], no, r2, c.f - mid, *this);
    }
}

void Vdl2Decoder::gotFrame(double freq, const unsigned char* f, std::size_t n) {
    Vdl2Frame fr;
    if (!vdl2ParseFrame(f, n, fr)) return;
    fr.freqHz = freq;
    fr.time = t_;
    fr.acars.freqHz = freq;
    fr.acars.time = t_;
    ++count_;
    lastFrameT_ = t_;
    std::size_t i = 0;
    while (i < countF_.size() && countF_[i] != freq) ++i;
    if (i == countF_.size()) {
        countF_.push_back(freq);
        countN_.push_back(0);
    }
    ++countN_[i];
    frames_.push_back(fr);
    while (frames_.size() > maxFrames_) frames_.pop_front();
    fresh_.push_back(fr);
    if (fresh_.size() > 1000) fresh_.erase(fresh_.begin());
    if (messagesOnly_ && fr.kind != 'I') return;
    text_ += "VDL2 " + fr.headLine() + "\n  ";
    const std::string body = fr.body();
    for (std::size_t j = 0; j < body.size(); ++j) {
        text_ += body[j];
        if (body[j] == '\n') text_ += "  ";
    }
    text_ += '\n';
}

void Vdl2Decoder::takeFrames(std::vector<Vdl2Frame>& out) {
    out.insert(out.end(), fresh_.begin(), fresh_.end());
    fresh_.clear();
}

void Vdl2Decoder::takeText(std::string& out) {
    out += text_;
    text_.clear();
}

unsigned Vdl2Decoder::countOn(double hz) const {
    for (std::size_t i = 0; i < countF_.size(); ++i)
        if (std::fabs(countF_[i] - hz) < 1) return countN_[i];
    return 0;
}

unsigned Vdl2Decoder::bursts() const {
    unsigned s = 0;
    for (std::size_t i = 0; i < rx_.size(); ++i) s += rx_[i]->bursts;
    return s;
}
unsigned Vdl2Decoder::rsFailures() const {
    unsigned s = 0;
    for (std::size_t i = 0; i < rx_.size(); ++i) s += rx_[i]->rsFail;
    return s;
}
unsigned Vdl2Decoder::crcFailures() const {
    unsigned s = 0;
    for (std::size_t i = 0; i < rx_.size(); ++i) s += rx_[i]->crcFail;
    return s;
}

Status Vdl2Decoder::status() const {
    Status s;
    s.signal = t_ - lastFrameT_ < 2;
    s.locked = s.signal;
    s.baud = 10500;
    s.characters = count_;
    s.mode = "VDL2";
    char buf[128];
    if (count_)
        std::snprintf(buf, sizeof buf, "%u message%s \xC2\xB7 %u channel%s", count_, count_ == 1 ? "" : "s",
                      static_cast<unsigned>(inView_.size()), inView_.size() == 1 ? "" : "s");
    else
        std::snprintf(buf, sizeof buf, "listening on %u channel%s: bursts are short, now and then",
                      static_cast<unsigned>(inView_.size()), inView_.size() == 1 ? "" : "s");
    s.info = buf;
    return s;
}

}  // namespace dec
