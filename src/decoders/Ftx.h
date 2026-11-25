// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Ftx.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  What FT8 and FT4 have in common (namespace dec::ftx): the 77-bit message,
//  its CRC-14 and LDPC(174,91) code, the tones, the synthesiser, and the
//  slot decoder that finds and decodes every signal in a stretch of audio.
//  A port of OFFgrid-SDR's `ft8lib` script (diddy-boy, index.html), whose
//  protocol constants (Costas patterns, Gray maps, the LDPC tables), CRC,
//  message formats and decoding method follow ft8_lib by Kārlis Goba
//  (https://github.com/kgoba/ft8_lib, MIT licence:
//
//    Copyright (c) 2018 Kārlis Goba.  Permission is hereby granted, free of
//    charge, to any person obtaining a copy of this software and associated
//    documentation files (the "Software"), to deal in the Software without
//    restriction, including without limitation the rights to use, copy,
//    modify, merge, publish, distribute, sublicense, and/or sell copies of
//    the Software, and to permit persons to whom the Software is furnished
//    to do so, subject to the following conditions: The above copyright
//    notice and this permission notice shall be included in all copies or
//    substantial portions of the Software.  THE SOFTWARE IS PROVIDED "AS
//    IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.)
//
//  FT8 and FT4 themselves are by Joe Taylor K1JT and Steve Franke K9AN
//  (WSJT-X).
//
//    FT8  79 symbols of 0.16 s (12.64 s), 8 tones 6.25 Hz apart, three
//         7-symbol Costas sync blocks (symbols 0, 36, 72), 58 data symbols
//         of 3 bits (Gray coded); GFSK with BT 2; 15 s slots, starting
//         0.5 s into the slot.  Decoded at 12800 Hz (a symbol = 2048
//         samples; a 4096-point FFT gives 3.125 Hz bins).
//    FT4  105 symbols of 48 ms (5.04 s), 4 tones 20.83 Hz apart, a ramp
//         symbol at each end, four 4-symbol Costas groups (symbols 1, 34,
//         67, 100), 87 data symbols of 2 bits; the 77-bit message XORed
//         with a fixed sequence first; GFSK with BT 1; 7.5 s slots,
//         starting 0.5 s in.  Decoded at 32000/3 Hz (a symbol = 512
//         samples; 1024-point FFTs, 10.4 Hz bins).
//
//  The message (pack77 / unpack77): every type the JS reads, and (Addition
//  to the JS) the contest types WSJT-X defines, packed and unpacked:
//      i3.n3  example                                   kind
//      0.0    TNX BOB 73 GL                             free text (13 chars)
//      0.1    K1ABC RR73; W9XYZ <KH1/KH7Z> -08          DXpedition
//      0.3/4  WA9XYZ KA1ABC R 16A EMA                   ARRL Field Day
//      0.5    telemetry 0123456789ABCDEF01              telemetry (18 hex)
//      1      CQ G4OGR IO81, G4OGR EA4XYZ R-09 ...      standard (/R)
//      2      PA3XYZ/P GM4ABC/P R JO22                  standard (/P)
//      3      TU; W9XYZ K1ABC R 579 MA                  ARRL RTTY Roundup
//      4      <W9XYZ> PJ4/K1ABC RR73, CQ YW18FIFA       nonstandard calls
//      5      <PA3XYZ> <G4ABC/P> R 590003 IO91NP        EU VHF contest
//  Calls shown in <...> are hashes: filled in from a CallHashTable of
//  calls already heard ("<...>" when not known).
//
//  The slot decoder (Engine), as the JS decode()/decode4(): a spectrogram
//  of half-symbol frames, every start time and base frequency scored on
//  the Costas sync tones against their neighbours, the best candidates'
//  symbols read into bit likelihoods, belief-propagation LDPC decoding
//  (no OSD: the JS has none), the CRC checked; a failed candidate gets a
//  second, exact measurement of its symbols (the JS: FT8 only); then each decoded
//  signal is rebuilt (GFSK, timing and frequency fine-tuned, strength
//  followed symbol by symbol) and subtracted, and the audio decoded again
//  (up to 3 passes) for the weaker stations it covered.
//
//  How it fits in
//  --------------
//  Ft8.h's Ft8Decoder / Ft4Decoder drive the Engine from live audio in
//  real-time slots; this header is also usable alone (tests, an encoder
//  for a demo band, a WAV-file decoder).  Depends on src/dsp/Fft.h and the
//  C++11 standard library.
//
//  Contributing
//  ------------
//  Keep it in step with the browser's ft8lib block; mark deviations
//  "Addition to the JS"; keep crediting ft8_lib.  Tests:
//  tests/test_decoders_ftx.cpp.
// ============================================================================
#ifndef OFFGRID_DECODERS_FTX_H
#define OFFGRID_DECODERS_FTX_H

#include "../dsp/Fft.h"

#include <complex>
#include <cstddef>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace dec {
namespace ftx {

typedef unsigned char u8;

// ---------------------------------------------------------------- constants
const int kPayloadBits = 77;   // the message
const int kPayloadBytes = 10;  // 77 bits, MSB first (bit 0 = 0x80 of byte 0)
const int kK = 91;             // message + CRC-14
const int kN = 174;            // LDPC codeword
const int kM = 83;             // LDPC checks
const int kFt8Symbols = 79;
const int kFt4Symbols = 105;

extern const u8 kFt8Costas[7];
extern const u8 kFt8Gray[8];
extern const u8 kFt4Costas[4][4];
extern const u8 kFt4Gray[4];
extern const u8 kFt4Xor[10];

// ---------------------------------------------------------------- callsign hashes
// Hashed calls ("<...>"): 22, 12 or 10 bits of a call, as WSJT-X computes
// them; the table remembers calls already sent or received in full so a
// hash can be shown as the call (port of the JS `hashes` map, saveCall,
// lookupCall).  (Addition to the JS: bounded, the oldest forgotten.)
class CallHashTable {
public:
    explicit CallHashTable(std::size_t capacity = 4000);
    // The 22-bit hash of a call (up to 11 characters of " 0-9A-Z/");
    // false if it has other characters.  The 12- and 10-bit hashes are
    // its top bits: h22 >> 10, h22 >> 12.
    static bool hash22(const std::string& call, unsigned& h22);
    void save(const std::string& call);
    // bits: 22, 12 or 10.  True and the call if known.
    bool lookup(int bits, unsigned hash, std::string& call) const;
    void clear();
    std::size_t size() const { return order_.size(); }

private:
    std::size_t cap_;
    std::map<unsigned, std::string> h22_, h12_, h10_;
    std::deque<std::string> order_;
    std::set<std::string> known_;
};

// ---------------------------------------------------------------- the 77-bit message
enum class MessageType {
    FreeText,      // 0.0
    Dxpedition,    // 0.1
    FieldDay,      // 0.3, 0.4
    Telemetry,     // 0.5
    Standard,      // 1 (and 2: /P calls)
    RttyRoundup,   // 3
    NonStandard,   // 4
    EuVhf,         // 5
    Unknown        // others (0.2, 0.6, 0.7, 6, 7): shown as "(message type i3.n3)"
};
const char* messageTypeName(MessageType t);  // "free text", "DXpedition", ...

// Upper case, single spaces, trimmed.
std::string normalizeMessage(const std::string& text);
// A message's text -> 77 bits.  Tries, in turn: telemetry ("telemetry"
// and up to 18 hex digits), DXpedition (contains "RR73;"), standard,
// RTTY Roundup, Field Day, EU VHF, nonstandard calls, free text (13
// characters of " 0-9A-Z+-./?").  Calls it sends in full go into `ht` (if
// given), as the JS does.  False if none fits.
bool pack77(const std::string& text, u8 a77[kPayloadBytes], CallHashTable* ht = 0, MessageType* type = 0);
// 77 bits -> text (port of unpack77; calls read in full go into `ht`).
// False for an invalid message (a callsign field out of range, empty free
// text).
bool unpack77(const u8 a77[kPayloadBytes], std::string& text, CallHashTable* ht = 0, MessageType* type = 0);
int messageI3(const u8 a77[kPayloadBytes]);
int messageN3(const u8 a77[kPayloadBytes]);

// ---------------------------------------------------------------- CRC, LDPC
// CRC-14 (polynomial 0x2757) of the first `nbits` bits of `bytes` (MSB
// first).  FT8 / FT4 take it over the 77 bits and 5 zero bits (82).
unsigned crc14(const u8* bytes, int nbits);
// 77 bits -> 91 bits (the CRC appended), 12 bytes.
void addCrc(const u8 a77[kPayloadBytes], u8 a91[12]);
bool checkCrc(const u8 a91[12]);
// 91 bits -> the 174-bit codeword, one bit per byte (message first).
void ldpcEncode(const u8 a91[12], u8 codeword[kN]);
// Parity checks that fail (0 = a codeword).
int ldpcCheck(const u8 codeword[kN]);
// Belief propagation (port of bpDecode): llr > 0 means 1.  True and the
// codeword when all checks pass within `iters` iterations.
bool bpDecode(const float llr[kN], int iters, u8 codeword[kN]);
// Packs bits (one per byte) into bytes, MSB first.
void packBits(const u8* bits, int n, u8* bytes);

// ---------------------------------------------------------------- tones and audio
// The channel symbols: FT8 79 tones 0..7, FT4 105 tones 0..3.
void ft8TonesFromPayload(const u8 a77[kPayloadBytes], u8 tones[kFt8Symbols]);
void ft4TonesFromPayload(const u8 a77[kPayloadBytes], u8 tones[kFt4Symbols]);
bool ft8Tones(const std::string& text, u8 tones[kFt8Symbols], CallHashTable* ht = 0);   // port of tones()
bool ft4Tones(const std::string& text, u8 tones[kFt4Symbols], CallHashTable* ht = 0);   // port of tones4()

// The GFSK frequency pulse (the JS gfskPhaseSteps' pulse): the share of a
// symbol's tone at time t symbols from its centre, for bandwidth-time
// product bt (FT8 2, FT4 1).  Exact erf (the JS: a 1.5e-7 approximation).
double gfskPulse(double t, double bt);
// Phase-continuous GFSK audio of a tone sequence (as the JS synthesiser
// and the subtraction rebuild signals): tone k at f0 + k * spacingHz, the
// tone changes smoothed by the Gaussian pulse (bt; 0 = plain FSK), the
// first and last tones held before and after.  Appends
// round(n * symbolSeconds * rate) samples of amplitude * cos(phase) (any
// rate; fractional samples per symbol are fine), starting at phase 0.
void synthesize(const u8* tones, int n, double symbolSeconds, double spacingHz, double bt, double f0, double rate,
                double amplitude, std::vector<float>& out);
// A whole FT8 / FT4 transmission of a message (12.64 s / 5.04 s): false if
// the message does not pack.  The signal's power is amplitude^2 / 2.
bool synthesizeFt8(const std::string& text, double f0, double rate, double amplitude, std::vector<float>& out,
                   CallHashTable* ht = 0);
bool synthesizeFt4(const std::string& text, double f0, double rate, double amplitude, std::vector<float>& out,
                   CallHashTable* ht = 0);

// ---------------------------------------------------------------- the slot decoder
enum class Protocol { Ft8, Ft4 };

// The two protocols' numbers (the JS FTX table and ft8lib constants).
struct ProtocolInfo {
    const char* name;      // "FT8", "FT4"
    double sampleRate;     // the decoder's rate: 12800, 32000/3
    int symbolSamples;     // at that rate: 2048, 512
    double symbolSeconds;  // 0.16, 0.048
    double toneSpacing;    // Hz: 6.25, 20.833
    int symbols;           // 79, 105
    int tones;             // 8, 4
    double bt;             // GFSK: 2, 1
    double slotSeconds;    // 15, 7.5
    double txStart;        // 0.5 s into the slot
    double window;         // audio decoded at each slot's end: 27.6 s, 13 s
    double minLength;      // the least audio worth decoding: 13.5 s, 6 s
};
const ProtocolInfo& protocolInfo(Protocol p);

struct DecodeOptions {
    DecodeOptions()
        : fLo(200), fHi(3000), passes(3), maxCandidates(0), minScore(2.2), iters(30), precise(40), preciseRank(120),
          snrCal(0), refine(true) {}
    double fLo, fHi;     // base (lowest tone) frequencies searched, Hz
    int passes;          // decode, subtract, decode again: 1..3 (the JS default 3)
    int maxCandidates;   // per pass; 0 = the JS's (FT8 300, FT4 220)
    double minScore;     // sync score (dB) for a candidate; later passes 0.4 lower
    int iters;           // belief-propagation iterations
    int precise;         // second (exact) tries per pass (later passes 20); 0 = none (JS: FT8 only)
    int preciseRank;     // ... among the best this many candidates (later passes 60)
    double snrCal;       // dB added to the SNR (the JS opt.snrCal)
    bool refine;         // (Addition to the JS) report every decode's start and frequency fine-tuned
};

// One decoded message from Engine::decode.
struct RawDecode {
    RawDecode() : start(0), freq(0), snr(0), score(0), pass(1) {
        for (int i = 0; i < kPayloadBytes; ++i) payload[i] = 0;
    }
    std::string text;
    u8 payload[kPayloadBytes];
    double start;   // the first symbol's start, samples at the protocol's rate from the audio's start
    double freq;    // tone 0, Hz
    int snr;        // dB in 2500 Hz (as WSJT-X reports it)
    double score;   // the sync score that found it
    int pass;       // 1, or 2-3 when found after subtracting stronger signals
};

// Decodes every signal in a stretch of audio at protocolInfo(p).sampleRate.
// Keeps its work buffers between calls (sized on first use).  One object per
// thread.
class Engine {
public:
    Engine();
    void decode(Protocol p, const float* x, std::size_t n, const DecodeOptions& opt, CallHashTable* ht,
                std::vector<RawDecode>& out);

private:
    struct Cand { float score; int m, b; };
    struct Found { RawDecode r; u8 tones[kFt4Symbols]; int ntones; };
    void decodeOnce(Protocol p, const DecodeOptions& opt, int pass, CallHashTable* ht, std::set<std::string>& seen,
                    std::vector<Found>& found);
    void spectrogram(int nfft, int hop, int b0, int nb, int nFrames);
    bool llrDecode(int iters, u8 cw[kN]);
    void precise(Protocol p, double start, double f0);
    double matchPower(double start, double f0, double fs, const std::vector<double>& phi, int sps, int step) const;
    void refine(double start, double f0, double fs, const std::vector<double>& phi, int sps, double binHz,
                double& bs, double& bf) const;
    void subtract(const Found& f, Protocol p, double& rs, double& rf);
    void phaseSteps(const u8* tones, int ntones, int sps, double bt, std::vector<double>& phi);

    std::vector<float> y_;              // the audio being worked on (signals subtracted)
    std::vector<float> db_, lin_;       // spectrogram: dB and power, frame x bin
    std::vector<float> win_;            // Hann window
    std::vector<std::complex<float> > buf_;
    dsp::detail::Fft fft_;
    std::vector<Cand> cands_, kept_;
    std::vector<float> prec_;           // exact tone powers (symbols x tones, dB)
    float llr_[kN];
    std::vector<double> phi_;           // a rebuilt signal's GFSK phase, and its cosine and sine
    std::vector<float> cphi_, sphi_;
    std::vector<double> pulse_;
    std::vector<float> cosW_, sinW_;
    std::vector<double> ar_, ai_, sr_, si_;
    double noise_;
};

}  // namespace ftx
}  // namespace dec

#endif
