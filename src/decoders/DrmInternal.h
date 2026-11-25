// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DrmInternal.h
//  Layer:  Library liboffgrid-decoders (internal header, not installed)
//
//  What this file does
//  -------------------
//  The pieces the DRM receiver (Drm.h) is built from, shared by its .cpp
//  files and by the DRM signal generator (DrmEncoders.h):
//    - the tables (DrmTables.cpp): robustness modes A-D (useful part Tu,
//      guard Tg, symbols a frame), the pilots (frequency, time and gain
//      references), the FAC cells, the spectrum occupancies' carrier ranges,
//      boosted pilots, puncturing patterns and tails;
//    - the coding (DrmCoding.cpp): bit interleaver tables, the punctured
//      mother code (rate 1/4, 64 states), its Viterbi decoder, the energy
//      dispersal (PRBS), Dream's CRC-8/16, the CCITT CRC-16 of text and
//      packets, the gain references' phases, a DFT of any length;
//    - the receiver (DrmRx.cpp): Rx::analyse() reads one window of 48 kHz
//      complex samples from scratch (mode, timing, frequency, frame start,
//      channel, FAC, SDC) and Rx::mscFrames() decodes its main channel;
//    - the data (DrmData.cpp): audio super frames (AAC and xHE-AAC), the
//      text message, MOT SlideShow, Journaline (with a raw inflate).
//
//  How it fits in
//  --------------
//  A port of OFFgrid-SDR's JS DRM receiver (diddy-boy,
//  https://github.com/solariun/offgrid-sdr, index.html: the "drmlib" block),
//  which was written from ETSI ES 201 980 following Dream (TU Darmstadt,
//  Volker Fischer, Alexander Kurpiers and others; GPL-2.0-or-later, see
//  licenses/dream-COPYING.txt) for its tables and the details of its cell
//  mapping and coding.  Nothing here is public API: use Drm.h.
//
//  Contributing
//  ------------
//  Keep the names close to the JS's (drmIlTable -> ilTable, drmPattern ->
//  makePattern, ...) so the two can be read side by side.  Strict C++11.
// ============================================================================
#ifndef OFFGRID_DECODERS_DRM_INTERNAL_H
#define OFFGRID_DECODERS_DRM_INTERNAL_H

#include <complex>
#include <cstddef>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "dsp/Fft.h"

namespace dec {
namespace drm {

typedef std::vector<unsigned char> Bytes;
typedef std::pair<int, int> IntPair;

// ---- the tables (DrmTables.cpp) ---------------------------------------------
// Robustness modes, indexed 0..3 = A..D (the JS's DRM_MODES, DRM_MI).
extern const int kTu[4];       // useful part of a symbol, samples at 48 kHz
extern const int kTg[4];       // guard interval, samples
extern const int kSymbols[4];  // symbols a transmission frame (400 ms)
extern const char kModeLetter[4];
extern const double kOccupancyKhz[6];  // spectrum occupancy 0..5
const int kRate = 48000;               // DRM's timing is defined at 48 kHz
const int kSuperframeSamples = 57600;  // 1.2 s: every mode's superframe

// One mode's pilot tables (Dream's CellMappingTable, as the JS's DRM_T).
struct ModeTable {
    const int* freq;  // frequency pilots: [carrier, phase] pairs
    int nFreq;
    const int* time;  // time references (symbol 0): [carrier, phase] pairs
    int nTime;
    int scat[3];      // gain references: x, y, k0
    const int* W;
    int nW;
    const int* Z;
    int nZ;
    int Q;
    int boost[24];    // boosted pilots, 4 a spectrum occupancy
    const int* fac;   // the FAC cells: [symbol, carrier] pairs
    int nFac;
};
const ModeTable& modeTable(int mi);
int kMin(int so, int mi);  // carriers of occupancy so in mode mi (0, 0:
int kMax(int so, int mi);  //  not allowed, e.g. 4.5 kHz in modes C and D)
const int* punct(int rate);  // [group, ones, generators per step...]
const int* tailPattern(int tp);

const char* languageName(int code);     // FAC language (0..15)
const char* programmeTypeName(int code);  // FAC programme type (0..31)
const char* aacRateName(int code);      // SDC audio sampling rate, AAC
const char* xheRateName(int code);      // ... xHE-AAC (DRM's own table)

// ---- the coding (DrmCoding.cpp) ---------------------------------------------
// drmIlTable: the bit/cell interleaver's permutation of N.
void ilTable(int N, int t0, std::vector<int>& t);
// drmPattern: the generator set sent at each step (a code 1..5 into
// DRM_GENS), the rate's pattern for M steps, then the 6 tail steps.
typedef std::vector<unsigned char> Pattern;
void makePattern(int rate, int M, int N2, bool fac, Pattern& out);
int patternBits(const Pattern& pat);  // coded bits the pattern sends
// drmEncode: the mother code, punctured by pat (bits past nbits are 0).
void convEncode(const unsigned char* bits, std::size_t nbits, const Pattern& pat, Bytes& out);
// drmViterbi: soft values (positive means 0) -> the pattern's information
// bits (steps - 6).  Buffers kept between calls.
class Viterbi {
public:
    void decode(const double* soft, const Pattern& pat, Bytes& bits);

private:
    std::vector<unsigned char> prv_, bit_;
};
// drmPrbs: energy dispersal, x^9 + x^5 + 1, all ones to start.
void prbs(std::size_t n, Bytes& out);
// drmCrc: Dream's CRC (all ones to start, inverted at the end); deg 8:
// x^8+x^4+x^3+x^2+1, deg 16: CCITT.
unsigned crcDream(const unsigned char* bytes, std::size_t n, int deg);
// drmTextCrc: CRC-16 CCITT (0xFFFF start, inverted): text, packets, groups.
unsigned crc16(const unsigned char* b, std::size_t n);
// drmBitsNum: n bits (one a byte, MSB first) from position at.
unsigned bitsNum(const unsigned char* b, std::size_t at, int n);
// drmGainRefs: the gain references of symbol s: [carrier, phase 0..1023].
void gainRefs(int mi, int s, int kmin, int kmax, std::vector<IntPair>& out);

// A DFT of any length (the useful parts are 1152, 1024, 704 and 448
// samples): Bluestein's method on liboffgrid-dsp's radix-2 FFT, or the FFT
// directly for a power of two.  X[k] = sum x[n] e^(-2 pi i k n / N).
class Dft {
public:
    Dft() : n_(0), m_(0) {}
    void init(int n);
    int size() const { return n_; }
    void forward(const std::complex<double>* x, std::complex<double>* out);
    void inverse(const std::complex<double>* x, std::complex<double>* out);  // unscaled

private:
    int n_, m_;
    bool pow2_;
    dsp::detail::Fft fft_;
    std::vector<std::complex<double> > w_;  // the chirp
    std::vector<std::complex<float> > b_;   // FFT of the conjugate chirp
    std::vector<std::complex<float> > a_;
};

// ---- what a window reveals (DrmRx.cpp) ------------------------------------
struct FacFrame {
    int fr;          // the frame's index in the window
    int frame;       // identity 0..2 (3: the SDC frame of a reconfiguration)
    int so;          // spectrum occupancy 0..5
    bool longIl;     // long (2 s) interleaving
    int mscCode;     // 0 64-QAM, 1/2 64-QAM hierarchical, 3 16-QAM
    bool sdc16;      // SDC in 16-QAM (else 4-QAM)
    int services;    // the services field (audio/data counts)
    int serviceId;   // 24 bits
    int shortId;     // 0..3: which service this frame describes
    int lang;        // 0..15
    bool data;       // a data service
    int ptype;       // programme type (audio) or application (data)
};
struct AudioParams {
    AudioParams() : present(false), streamId(0), codec(0), sbr(false), mode(0), rateCode(0), text(false) {}
    bool present;
    int streamId;
    int codec;     // 0 AAC, 1 CELP, 2 HVXC, 3 xHE-AAC
    bool sbr;
    int mode;      // 0 mono, 1 parametric stereo, 2 stereo
    int rateCode;  // sampling rate code
    bool text;     // the last 4 bytes of each super frame: the text message
    Bytes xheConfig;  // xHE-AAC: DRM's compact configuration
};
struct DataApp {
    DataApp() : shortId(0), stream(0), packet(false), dataUnits(false), packetId(0), domain(0), packetLen(0), app(-1) {}
    int shortId, stream;
    bool packet, dataUnits;
    int packetId, domain, packetLen, app;  // app: DAB user application (2 SlideShow, 0x44A Journaline)
};
struct Mux {
    Mux() : present(false), protA(0), protB(0) {}
    bool present;
    int protA, protB;
    std::vector<IntPair> streams;  // [bytes in part A, bytes in part B] a multiplex frame
};
struct RxResult {
    RxResult() { clear(); }
    void clear();
    double metric;        // the guard-interval peak above its median
    int mode;             // 0..3, -1 none
    std::string state;
    int facOk, facTried, sdcOk, sdcTried;
    bool hasMer;
    double mer;           // dB, from the FAC cells
    double freqOffset;    // Hz
    int sh;               // whole-carrier offset
    double drift;         // Hz a second
    bool mirrored;
    std::vector<FacFrame> frames;
    bool settings;        // a FAC passed its CRC
    int so, mscCode, services;
    bool longIl, sdc16;
    // the services, by short ID (from the FAC frames and the SDC)
    bool svcSeen[4];
    FacFrame svc[4];
    bool haveLabel[4];
    std::string labels[4];
    AudioParams audio[4];
    std::string label;    // the last label read (the JS's out.label)
    Mux mux;
    std::vector<DataApp> apps;
};

// One multiplex frame (400 ms) of the main channel, numbered by its place
// in the stream (so overlapping windows join without repeats).
struct MuxFrame {
    long long id;
    Bytes all;  // every byte of the frame: streams' parts A, then parts B
};

class Rx {
public:
    struct Hint {
        Hint() : valid(false), mode(0), mirrored(false), sh(0) {}
        bool valid;
        int mode;
        bool mirrored;
        int sh;
    };
    Rx();
    Hint hint;
    bool keep;          // keep the cells for mscFrames()
    bool iterate;       // the extra decoding passes for a frame in trouble
    // x: complex 48 kHz samples (at least ~3.6 s).  A clear DRM signal whose
    // settings can't be read is tried again mirrored.
    void analyse(const std::complex<float>* x, std::size_t n, RxResult& out);
    // The main channel of the last analysed window (keep = true): the
    // multiplex frames after `after`.  winStart: the window's first sample's
    // place in the stream.
    void mscFrames(const RxResult& r, long long winStart, long long after, std::vector<MuxFrame>& out);

private:
    void analyseOnce(const std::complex<float>* x, std::size_t n, bool mirror, RxResult& out);
    void chan(int kmin, int kmax, const int* boost, int nboost, std::vector<std::complex<float> >& H);
    std::complex<double> C(int s, int k) const {  // carrier k of symbol s (counted from S0)
        return std::complex<double>(S_[static_cast<std::size_t>(S0_ + s) * Tu_ + c0_ + sh_ + k]);
    }

    std::vector<std::complex<double> > xin_;  // the window, mirrored or not
    std::vector<std::complex<float> > S_;     // nsym x Tu spectra
    std::vector<std::complex<float> > H1_, H2_;
    std::vector<unsigned char> mark_;
    std::vector<std::complex<double> > tx_, tX_;
    std::vector<double> steady_;
    std::map<int, Dft> dft_;
    Viterbi vit_;
    // the window as kept for the main channel
    int mi_, Tu_, Tg_, L_, c0_, sh_, S0_, nsym_, nframes_, start_, kmin_, kmax_, W2_;
    std::set<long long> facCells_, sdcCells_;  // symbol * 65536 + carrier + 32768
    std::set<int> fp_, tp_;
};

// ---- the data (DrmData.cpp) ---------------------------------------------------
// drmAacFrames: the AAC access units of an audio super frame (each with its
// CRC byte).  Returns how many (a bad header gives fewer).
struct AacAu {
    unsigned char crc;
    const unsigned char* p;
    int n;
};
int aacFrames(const unsigned char* by, const AudioParams& a, int lenBytes, std::vector<AacAu>& out);
// drmXheFrames: the xHE-AAC frames of an audio super frame; frames run on
// across super frames (the leftover kept here).
class XheFrames {
public:
    XheFrames() : headerBad(0), started_(false) {}
    void reset() {
        payload_.clear();
        started_ = false;
    }
    void feed(const unsigned char* by, int superFrameSize, std::vector<Bytes>& out);
    int headerBad;  // super frames whose header CRC failed (Addition to the JS)

private:
    Bytes payload_;
    bool started_;
};
// drmXheAsc: DRM's compact xHE-AAC configuration -> an MPEG-4
// AudioSpecificConfig with a UsacConfig (empty if there is none).
Bytes xheAsc(const AudioParams& a);
// drmTextFeed: the text message, 4 bytes a super frame.
class TextMessage {
public:
    TextMessage() { reset(); }
    void reset();
    // returns true when a message is complete (new or repeated)
    bool feed(const unsigned char* four);
    std::string text;
    bool have;

private:
    Bytes buf_;
    std::map<int, std::pair<bool, Bytes> > segs_;
    int toggle_;
};
// DrmSlides: packet mode -> MSC data groups -> MOT (header mode).
struct Picture {
    std::string name, mime;
    Bytes bytes;
};
// Packets of a packet-mode stream, put together into MSC data groups.
class PacketAssembler {
public:
    PacketAssembler() : packetLen(0), packetId(0), packets(0), maxGroup(20000), resetOnBad(false) {}
    int packetLen, packetId;
    int packets;  // packets whose CRC was right
    std::size_t maxGroup;
    bool resetOnBad;
    void frame(const unsigned char* st, std::size_t n, std::vector<Bytes>& groups);

private:
    Bytes cur_;
    bool have_ = false;
};
class MotSlides {
public:
    void reset();
    void group(const Bytes& g, std::vector<Picture>& ready);

private:
    struct Obj {
        Obj() : hl(-1), bl(-1) {}
        std::map<int, Bytes> hdr, body;
        int hl, bl;
    };
    std::map<int, Obj> objs_;
    std::set<int> done_;
};
// Journaline (ETSI TS 102 979): NML objects.
struct JournalinePage {
    JournalinePage() : id(0), type(0) {}
    unsigned id;
    int type;  // 1 menu, 2 text, 3 title only, 4 list
    std::string title, text;
    std::vector<std::pair<unsigned, std::string> > links;  // a menu's items
    std::vector<std::vector<std::string> > rows;           // a list's rows
};
class Journaline {
public:
    void reset() {
        seen_.clear();
        pages.clear();
        newPages = 0;
    }
    void group(const Bytes& g);
    std::map<unsigned, JournalinePage> pages;
    int newPages;  // pages new or revised (counted up)
    Journaline() : newPages(0) {}

private:
    std::map<unsigned, std::string> seen_;
};
bool nmlParse(unsigned id, int ot, const unsigned char* p, std::size_t n, JournalinePage& o);
// Raw deflate (RFC 1951), for compressed Journaline objects.  Addition to
// the JS (which used the browser's DecompressionStream).
bool inflateRaw(const unsigned char* in, std::size_t n, Bytes& out, std::size_t maxOut);

// ---- the sound (DrmAac.cpp) ----------------------------------------------------
// The small interface the receiver codes against, so everything up to the
// access units works (and is tested) without an AAC decoder; DrmAac.cpp
// fills it from liboffgrid-codecs (codec::AacDecoder) when that is built in.
class AudioCodec {
public:
    virtual ~AudioCodec() {}
    // (Re)opened for these parameters (AAC only); false if it can't decode them.
    virtual bool configure(const AudioParams& a) = 0;
    // One access unit: the DRM CRC byte, then the AU (as faad2's DRM mode
    // takes it).  Appends interleaved stereo float; false for a lost frame
    // (a bad CRC or a decoding error).
    virtual bool decode(const unsigned char* crcAndAu, std::size_t n, std::vector<float>& stereo, int& rate) = 0;
};
// null when the library was built without liboffgrid-codecs
AudioCodec* makeAacCodec();

}  // namespace drm
}  // namespace dec

#endif
