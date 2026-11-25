// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DabInternal.h
//  Layer:  Library liboffgrid-decoders (internal header, not installed)
//
//  What this file does
//  -------------------
//  The pieces the DAB / DAB+ receiver (Dab.cpp) and the DAB multiplex
//  generator (DabEncoders.cpp) share, in namespace dec::dab:
//    - the mode I numbers (2048-point FFT, 1536 carriers, 76 symbols, 96 ms
//      frames at 2.048 MS/s) and the tables built from ETSI EN 300 401 (the
//      phase reference symbol, the frequency interleaver, the puncturing
//      vectors, the convolutional code, the UEP table and its profiles);
//    - the error correction: Viterbi (64 states, soft input), depuncturing,
//      energy dispersal (PRBS), CRC-16, the DAB+ fire code and Reed-Solomon
//      RS(120,110) (DabFec.cpp);
//    - sub-channel protection (EEP / UEP) to puncturing sections;
//    - PAD: dynamic label (DLS), MOT SlideShow reassembly, the SPI service's
//      packets and directory (station logos), DAB's character sets
//      (DabPad.cpp);
//    - a small interface to the audio codecs (liboffgrid-codecs' MP2 and
//      HE-AAC v2 decoders), so the receiver itself never depends on them
//      (DabAudio.cpp wires them).
//
//  How it fits in
//  --------------
//  A port of OFFgrid-SDR's JS DAB library (diddy-boy, https://github.com/
//  solariun/offgrid-sdr, index.html, the "dablib" block: "DAB / DAB+
//  receiver (mode I, 2.048 MHz) ... Tables from ETSI EN 300 401, as in
//  welle.io").  The table values are those of the JS, which took them from
//  the standard as welle.io (GPL-2.0-or-later) uses them.
//
//  Contributing
//  ------------
//  Keep each function in step with its JS counterpart (named in its
//  comment); mark changes "Addition to the JS".  Strict C++11, no
//  allocation per sample.  Run tests/test_decoders_dab.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_DAB_INTERNAL_H
#define OFFGRID_DECODERS_DAB_INTERNAL_H

#include <complex>
#include <cstddef>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace dec {
namespace dab {

typedef std::complex<float> Cf;

// ---- mode I (JS: DAB_Tu, DAB_Tg, DAB_Ts, DAB_Tn, DAB_K, DAB_L, DAB_FR, DAB_FS)
const int Tu = 2048;       // useful symbol length (samples), also the FFT size
const int Tg = 504;        // guard interval
const int Ts = 2552;       // a whole symbol
const int Tn = 2656;       // the null symbol
const int K = 1536;        // carriers
const int L = 76;          // symbols a frame (phase reference + 75)
const int FR = 196608;     // samples a frame (96 ms)
const double FS = 2048000.0;
const int kSymBits = 2 * K;     // soft bits a symbol (3072)
const int kCifBits = 55296;     // bits a CIF (864 CUs of 64)
const int kFicBits = 9216;      // the FIC: 3 symbols, 4 blocks of 2304 coded bits

// The tables, built once (thread-safe in C++11).
struct Tables {
    int carriers[K];            // carrier numbers -768..768 without 0 (JS DAB_CARRIERS)
    Cf prs[K];                  // the phase reference symbol (JS DAB_PRS)
    int bin[K];                 // FFT bin of each carrier (JS DAB_BIN)
    Cf win[K];                  // each carrier turned back 32 samples (JS DAB_WIN)
    int perm[K];                // frequency interleaver: bit position -> carrier index (JS DAB_PERM)
    unsigned char pcodes[24][32];   // puncturing vectors PI = 1..24 (JS DAB_PCODES)
    unsigned char pix[24];          // the tail's puncturing (JS DAB_PIX)
    signed char out[128 * 4];       // convolutional code outputs, +1 / -1 (JS DAB_OUT)
    unsigned char vcode[128];       // the same, four bits a register (JS DAB_VCODE)
    unsigned char prbs768[768];     // the FIC's energy dispersal (JS DAB_PRBS768)
    unsigned char gfExp[512];       // GF(256), 0x11D
    unsigned char gfLog[256];
};
const Tables& tables();

// time interleaving delays (JS DAB_IMAP)
extern const int kImap[16];

// ---- error correction (DabFec.cpp)
// Viterbi over the 64-state code (0133, 0171, 0145, 0133); soft input,
// positive = a 1, four values a bit; nbits + 6 (the tail) steps.  The
// decision memory is kept between calls (JS dabViterbi).
class Viterbi {
public:
    void decode(const float* soft, int nbits, unsigned char* out);

private:
    std::vector<unsigned char> dec_;
};

struct Section {
    int pattern;   // puncturing vector 1..24
    int blocks;    // blocks of 128 coded bits
};

// Depuncturing (JS dabDepunct): `soft` holds the punctured bits; `vb` gets
// nout * 4 + 24 values, zeros where a bit was punctured.
void depuncture(const float* soft, const Section* sections, int nSections, int nout, std::vector<float>& vb);
// How many punctured bits the sections and the tail give.
int puncturedLength(const Section* sections, int nSections);
// Energy dispersal sequence (JS dabPrbs).
void prbs(int n, std::vector<unsigned char>& out);
// CRC-16 CCITT, inverted (JS dabCrc16).
unsigned crc16(const unsigned char* by, std::size_t n);
bool crcOk(const unsigned char* by, std::size_t n);   // last two bytes the CRC of the rest
// Reed-Solomon RS(120,110) (shortened RS(255,245)), corrected in place:
// the number of bytes corrected, or -1 (JS dabRsDecode).
int rsDecode(unsigned char* cw, int n);
// RS parity for 110 data bytes (encoder side; Addition to the JS, which
// has its parity computed ahead).
void rsEncode(const unsigned char* data110, unsigned char* parity10);
// DAB+ super frame header: fire code and plausible AU starts (JS dabFire,
// dabSfOk).
bool fireOk(const unsigned char* sf);
bool superframeOk(const unsigned char* sf, int s);

// ---- sub-channels (FIG 0/1) and their protection (JS dabProtection)
struct SubChannel {
    SubChannel() : id(0), start(0), size(0), uep(false), uepIndex(0), opt(0), lev(0) {}
    int id, start, size;   // CUs
    bool uep;
    int uepIndex;          // short form: table 8 index
    int opt, lev;          // long form: option (A = 0, B = 1), level 0..3
};
struct Protection {
    Protection() : kbps(0), nSections(0) {}
    int kbps;
    Section sections[4];
    int nSections;
    std::string name() const;    // e.g. "EEP 3-A", "UEP 3"
    std::string level;
};
bool protectionOf(const SubChannel& sc, Protection& p);
// The UEP table (EN 300 401 table 8): index -> CUs, level, kbit/s.
struct UepRow {
    int cus, lev, kbps;
};
extern const UepRow kUep[64];
// Its puncturing profiles, as welle.io / the JS (DAB_UEP_PROF): L1..L4
// blocks and PI1..PI4 for a bit rate and level; false if not listed.
bool uepProfile(int kbps, int lev, int prof[8]);

// ---- characters (JS dabText): EBU Latin (0), UCS-2 (6), UTF-8 (15) -> UTF-8, trimmed
std::string text(const unsigned char* b, std::size_t n, int charset);
void appendUtf8(std::string& s, unsigned cp);

// ---- PAD (DabPad.cpp)
struct Picture {
    std::string name, mime;
    int w, h;
    std::vector<unsigned char> bytes;
};

// MOT SlideShow reassembly, header mode (JS DabSlides).
class Slides {
public:
    void group(const unsigned char* g, std::size_t n);
    std::vector<Picture> ready;

private:
    struct Obj {
        Obj() : tid(-1), hl(-1), bl(-1) {}
        int tid;
        std::map<int, std::vector<unsigned char> > hdr, body;
        int hl, bl;
    };
    std::vector<Obj> objs_;   // in arrival order (a few at a time)
};

// Programme-associated data: X-PAD with dynamic label and SlideShow (JS DabPad).
class Pad {
public:
    Pad();
    // xd: the bytes before the two F-PAD bytes; xlen how many hold X-PAD
    // (read backwards from xd[xlen - 1]); exact: DAB+ (announced length
    // must match); f0, f1: the F-PAD bytes.
    void process(const unsigned char* xd, int xlen, bool exact, int f0, int f1);
    bool changed;          // the label changed (cleared by the reader)
    std::string label;     // the dynamic label (UTF-8)
    Slides slides;

private:
    void dlSubfield(bool start, const unsigned char* d, int n);
    void motSubfield(bool start, const unsigned char* d, int n);
    int lastType_, lastLen_;
    bool dgOn_;
    std::vector<unsigned char> dg_;
    std::size_t dgNeed_;
    struct Seg {
        bool last;
        std::vector<unsigned char> chars;
    };
    std::map<int, Seg> segs_;
    int toggle_, charset_;
    int motLen_;
    bool motOn_;
    std::vector<unsigned char> mot_;
    std::size_t motNeed_;
    unsigned char x_[196];
};

// The SPI service (station logos): packets, data groups, MOT directory
// (JS DabSpi).
class Spi {
public:
    explicit Spi(int addr);
    void cif(const unsigned char* by, std::size_t n);
    std::vector<Picture> ready;
    unsigned packets, groups;

private:
    struct Obj {
        Obj() : type(0), tid(0), last(-1) {}
        int type, tid, last;
        std::map<int, std::vector<unsigned char> > segs;
    };
    struct DirEntry {
        int ctype, csub;
        std::string name;
    };
    void group(const unsigned char* g, std::size_t n);
    void complete(Obj& o);
    void directory(const std::vector<unsigned char>& d);
    void file(int tid, const DirEntry& e, const std::vector<unsigned char>& b);
    int addr_;
    bool curOn_;
    std::vector<unsigned char> cur_;
    std::map<std::pair<int, int>, Obj> objs_;
    bool haveDir_;
    std::map<int, DirEntry> dir_;
    std::map<int, bool> done_;
    std::map<int, std::vector<unsigned char> > waiting_;
};

// ---- the audio codecs (DabAudio.cpp): liboffgrid-codecs behind a small
// interface (Addition to the JS, whose worker loads FAAD2 and pl_mpeg's MP2
// as WebAssembly).
struct AacFormat {
    AacFormat() : rate(0), sbr(false), stereo(false), ps(false) {}
    int rate;        // the output rate: 48000 or 32000 (dac), with SBR the core runs at half
    bool sbr, stereo, ps;
    bool operator==(const AacFormat& o) const { return rate == o.rate && sbr == o.sbr && stereo == o.stereo && ps == o.ps; }
    bool operator!=(const AacFormat& o) const { return !(*this == o); }
};
class AudioCodecs {
public:
    virtual ~AudioCodecs() {}
    virtual bool available() const = 0;
    // One DAB+ access unit; appends interleaved stereo; false: not decoded.
    virtual bool aac(const AacFormat& f, const unsigned char* au, std::size_t n, std::vector<float>& stereo, int& rate) = 0;
    // One MPEG-1/2 Layer II frame (with header); appends stereo; also its
    // ancillary data is read by the receiver itself.
    virtual bool mp2(const unsigned char* fr, std::size_t n, std::vector<float>& stereo, int& rate) = 0;
    virtual void reset() = 0;
};
AudioCodecs* newAudioCodecs();
// The AudioSpecificConfig a DAB+ station's AAC needs (JS dabPlusAsc):
// AAC-LC, 960-sample frames, explicit SBR (0x2B7) and PS (0x548) signalling.
std::vector<unsigned char> plusAsc(const AacFormat& f);

// ---- the demo's data (DabDemoData.cpp): base64 of the JS's embedded blocks
// (kept as arrays of string pieces, each well under any compiler's literal
// limit, ended by a null pointer)
std::vector<unsigned char> demoPlus();   // id="dabdemoplus": 480 CIFs x 144 bytes, DAB+ super frames with RS parity
std::vector<unsigned char> demoMp2();    // id="dabdemomp2":  480 CIFs x 144 bytes, MP2 frames (48 kbit/s)
std::vector<unsigned char> demoSpi();    // id="dabdemospi":  480 CIFs x 48 bytes, SPI packets (logos)
void base64Append(const char* s, std::vector<unsigned char>& out, unsigned& acc, int& nbits);

}  // namespace dab
}  // namespace dec

#endif
