// ============================================================================
//  offgrid-codecs - AAC decoder for DAB+ and DRM (part of OFFgrid-SDR TUI,
//  https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/codecs/Aac.cpp
//  Layer:  Library liboffgrid-codecs
//
//  What this file does
//  -------------------
//  codec::AacDecoder (see Aac.h): a small C++ face on the FAAD2 port.  It
//  opens FAAD2's decoder state, sets it up the way each system signals its
//  audio (AudioSpecificConfig, DAB+'s super frame header, DRM's SDC, ADTS),
//  asks for float output, and turns each NeAACDecDecode() call into a
//  std::vector<float> plus the rate, channels and error of that AU.
//
//  How it fits in
//  --------------
//  The only file of the AAC decoder that knows both worlds: the public API
//  (Aac.h, namespace codec) and FAAD2's internal one (AacFaadApi.h,
//  namespace codec::faad).  Everything FAAD2 does - parsing, SBR, PS, the
//  filter bank - is in the Aac*.cpp port files.
//
//  The DAB+ configuration is the one OFFgrid-SDR's browser version builds
//  (dabPlusAsc in index.html, as DAB+ receivers do): AAC-LC at the core rate
//  with 960-sample frames, then explicit SBR (sync 0x2B7) and PS (0x548)
//  signalling.  DRM uses FAAD2's own DRM set-up (NeAACDecInitDRM): object
//  type 27, VCB11 + HCR error resilience, 960-sample frames, its channel
//  codes 1..5 (mono, stereo, SBR mono, SBR stereo, SBR + PS).
//
//  Contributing
//  ------------
//  Keep the decoder state valid whatever comes in: every failure path leaves
//  h_ either a working decoder or 0.  Strict C++11, no warnings with -Wall
//  -Wextra -pedantic-errors.
// ============================================================================
#include "Aac.h"

#include <cstring>

#include "AacCommon.h"
#include "AacStructs.h"

namespace codec {

namespace {
// The sampling-frequency indexes of ISO/IEC 14496-3 (table 1.18).
const unsigned kSrTable[12] = {96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000};
int srIndex(unsigned rate) {
    for (int i = 0; i < 12; i++)
        if (kSrTable[i] == rate) return i;
    return -1;
}
// The zero bytes kept behind each AU: FAAD2's bit reader is careful about the
// buffer end, this is one more guard against a read just past it.
const std::size_t kPad = 64;
// Failures in a row after which the decoder is rebuilt (see decode()).
const unsigned kMaxFailures = 8;

// A bit writer for makeAsc().
struct BitOut {
    std::vector<std::uint8_t> bytes;
    unsigned nbits;
    BitOut() : nbits(0) {}
    void put(unsigned v, unsigned n) {
        for (unsigned i = n; i-- > 0;) {
            if ((nbits & 7) == 0) bytes.push_back(0);
            if ((v >> i) & 1) bytes.back() |= (std::uint8_t)(0x80 >> (nbits & 7));
            nbits++;
        }
    }
};

// Float output (FAAD2's FAAD_FMT_FLOAT: 16-bit full scale = 1.0).
void setFloat(void *h) {
    faad::NeAACDecConfigurationPtr c = faad::NeAACDecGetCurrentConfiguration(h);
    c->outputFormat = FAAD_FMT_FLOAT;
    faad::NeAACDecSetConfiguration(h, c);
}
} // namespace

AacParams AacParams::dabPlus(unsigned dacRate, bool sbr, bool stereo, bool ps) {
    AacParams p;
    p.sbr = sbr;
    p.ps = sbr && ps;
    p.sampleRate = sbr ? dacRate / 2 : dacRate;
    p.channels = (stereo && !p.ps) ? 2 : 1;   // with PS the AAC core is mono
    p.frameLength = 960;
    p.drm = false;
    return p;
}

AacParams AacParams::drmAudio(unsigned audioRate, bool sbr, bool stereo, bool ps) {
    AacParams p;
    p.sbr = sbr;
    p.ps = sbr && ps;
    p.sampleRate = audioRate;
    p.channels = (stereo && !p.ps) ? 2 : 1;
    p.frameLength = 960;
    p.drm = true;
    return p;
}

AacDecoder::AacDecoder()
    : h_(0), kind_(None), noSbr_(false), drmRate_(0), drmCode_(0), rate_(0), ch_(0), frame_(0), sbr_(false), ps_(false), used_(0), err_(-1),
      restarts_(0), failures_(0) {}

AacDecoder::~AacDecoder() { close(); }

void AacDecoder::close() {
    if (h_) faad::NeAACDecClose(h_);
    h_ = 0;
    kind_ = None;
    noSbr_ = false;
    cfg_.clear();
    rate_ = ch_ = frame_ = 0;
    sbr_ = ps_ = false;
    used_ = 0;
    err_ = -1;
    restarts_ = 0;
    failures_ = 0;
}

// (Re)creates FAAD2's state from the saved configuration (cfg_ / drm*):
// init() saves it and calls this; decode() calls it again to recover.
bool AacDecoder::open() {
    if (h_) faad::NeAACDecClose(h_);
    h_ = 0;
    unsigned long rate = 0;
    unsigned char ch = 0;
    if (kind_ == Drm) {
        faad::NeAACDecHandle h = faad::NeAACDecOpen();
        if (!h) return false;
        if (faad::NeAACDecInitDRM(&h, drmRate_, drmCode_) != 0 || !h) {
            if (h) faad::NeAACDecClose(h);
            return false;
        }
        h_ = h;
        setFloat(h_);   // (InitDRM opens a fresh state, so the format is set after it)
        rate = drmCode_ >= DRMCH_SBR_MONO ? drmRate_ * 2 : drmRate_;
        ch = 2;
    } else if (kind_ == Asc || kind_ == Adts) {
        h_ = faad::NeAACDecOpen();
        if (!h_) return false;
        setFloat(h_);
        if (noSbr_) {
            // Explicitly no SBR: no "implicit SBR" up-sampling of a core rate
            // <= 24 kHz either (FAAD2's assumption for configurations that
            // don't say).
            faad::NeAACDecConfigurationPtr c = faad::NeAACDecGetCurrentConfiguration(h_);
            c->dontUpSampleImplicitSBR = 1;
            faad::NeAACDecSetConfiguration(h_, c);
        }
        std::vector<std::uint8_t> copy(cfg_);   // (FAAD2 wants a writable buffer)
        copy.resize(cfg_.size() + kPad, 0);
        long r = kind_ == Asc ? faad::NeAACDecInit2(h_, &copy[0], (unsigned long)cfg_.size(), &rate, &ch)
                              : faad::NeAACDecInit(h_, &copy[0], (unsigned long)cfg_.size(), &rate, &ch);
        if (r < 0 || rate == 0) {
            faad::NeAACDecClose(h_);
            h_ = 0;
            return false;
        }
        if (noSbr_) {
            // (OFFgrid-SDR's change in the port: SBR fill data is skipped, so
            // a damaged AU cannot switch SBR on; see fill_element)
            static_cast<faad::NeAACDecStruct *>(h_)->sbr_signalled_absent = 1;
            // the output rate is the AAC rate itself (makeAsc's sampling
            // frequency index: bits 5..8 of the AudioSpecificConfig)
            unsigned idx = ((cfg_[0] & 7u) << 1) | (cfg_[1] >> 7);
            if (idx < 12) rate = kSrTable[idx];
        }
    } else {
        return false;
    }
    rate_ = (unsigned)rate;
    ch_ = ch;
    err_ = 0;
    return true;
}

std::vector<std::uint8_t> AacDecoder::makeAsc(const AacParams &p) {
    // As dabPlusAsc() in OFFgrid-SDR's index.html: AudioSpecificConfig with
    // object type 2 (AAC-LC) at the core rate, GASpecificConfig (frame length
    // flag, no core coder, no extension), then the explicit backward-compatible
    // SBR extension (sync 0x2B7, object type 5, SBR flag, output rate index)
    // and, for HE-AAC v2, the PS extension (sync 0x548, PS flag).
    std::vector<std::uint8_t> none;
    int core = srIndex(p.sampleRate);
    if (core < 0 || p.channels < 1 || p.channels > 2) return none;
    if (p.frameLength != 960 && p.frameLength != 1024) return none;
    BitOut b;
    b.put(2, 5);                                   // audioObjectType: AAC-LC
    b.put((unsigned)core, 4);                      // samplingFrequencyIndex
    b.put(p.channels, 4);                          // channelConfiguration
    b.put(p.frameLength == 960 ? 1 : 0, 1);        // frameLengthFlag
    b.put(0, 1);                                   // dependsOnCoreCoder
    b.put(0, 1);                                   // extensionFlag
    if (p.sbr) {
        int ext = srIndex(p.sampleRate * 2);
        if (ext < 0) return none;
        b.put(0x2b7, 11);                          // syncExtensionType
        b.put(5, 5);                               // extensionAudioObjectType: SBR
        b.put(1, 1);                               // sbrPresentFlag
        b.put((unsigned)ext, 4);                   // extensionSamplingFrequencyIndex
        if (p.ps) {
            b.put(0x548, 11);                      // syncExtensionType: PS
            b.put(1, 1);                           // psPresentFlag
        }
    }
    return b.bytes;
}

bool AacDecoder::init(const std::uint8_t *asc, std::size_t len) {
    close();
    if (!asc || len < 2 || len > 1024) return false;
    kind_ = Asc;
    cfg_.assign(asc, asc + len);
    if (open()) return true;
    close();
    return false;
}

bool AacDecoder::init(const std::vector<std::uint8_t> &asc) {
    return init(asc.empty() ? 0 : &asc[0], asc.size());
}

bool AacDecoder::init(const AacParams &p) {
    close();
    if (p.frameLength != 960 && p.frameLength != 1024) return false;
    if (p.drm) {
        // FAAD2's DRM mode (NeAACDecInitDRM): 960-sample frames only, and only
        // the rates DRM uses; its channel codes 1..5.
        if (p.frameLength != 960) return false;
        if (p.sampleRate != 12000 && p.sampleRate != 24000 && p.sampleRate != 48000) return false;
        if (!p.sbr) drmCode_ = p.channels == 2 ? DRMCH_STEREO : DRMCH_MONO;
        else if (p.ps) drmCode_ = DRMCH_SBR_PS_STEREO;
        else drmCode_ = p.channels == 2 ? DRMCH_SBR_STEREO : DRMCH_SBR_MONO;
        drmRate_ = p.sampleRate;
        kind_ = Drm;
        if (open()) return true;
        close();
        return false;
    }
    std::vector<std::uint8_t> asc = makeAsc(p);
    if (asc.empty()) return false;
    kind_ = Asc;
    noSbr_ = !p.sbr;   // DAB+ (and explicit parameters) say whether there is SBR: held to it
    cfg_ = asc;
    if (open()) return true;
    close();
    return false;
}

bool AacDecoder::initAdts(const std::uint8_t *data, std::size_t len) {
    close();
    if (!data || len < 7 || data[0] != 0xff || (data[1] & 0xf6) != 0xf0) return false;
    kind_ = Adts;
    cfg_.assign(data, data + 7);   // (the fixed header is all NeAACDecInit reads)
    if (open()) return true;
    close();
    return false;
}

bool AacDecoder::decode(const std::uint8_t *au, std::size_t len, std::vector<float> &pcm) {
    pcm.clear();
    used_ = 0;
    if (!h_) {
        err_ = -1;
        return false;
    }
    if (!au || len == 0 || len > 65536) {
        err_ = -1;
        return false;
    }
    // A private copy with zero padding behind it (FAAD2 wants a non-const
    // buffer and reads whole 32-bit words).
    if (buf_.size() < len + kPad) buf_.resize(len + kPad);
    std::memcpy(&buf_[0], au, len);
    std::memset(&buf_[len], 0, kPad);
    faad::NeAACDecFrameInfo info;
    std::memset(&info, 0, sizeof info);
    void *out = faad::NeAACDecDecode(h_, &info, &buf_[0], (unsigned long)len);
    used_ = info.bytesconsumed;
    err_ = info.error;
    if (info.error) {
        // OFFgrid-SDR: some errors are states FAAD2 never leaves.  It records
        // each frame's element layout (and whether SBR is on) and refuses any
        // later frame that differs, so one damaged AU can stop all that follow:
        //   21 "Unexpected channel configuration change": the damaged AU's
        //      bits named a channel pair where the stream has a single channel
        //      (or the reverse), and the layout kept that;
        //   23 the same for SBR: SBR-looking fill data switched implicit SBR
        //      on after the channels were set up without it.
        // So on those, and after kMaxFailures failures in a row (whatever the
        // cause), the decoder is rebuilt from its configuration; the next AU
        // starts it afresh (and, like any first AU, gives no samples).
        failures_++;
        if (info.error == 21 || info.error == 23 || failures_ >= kMaxFailures) {
            int e = err_;
            restarts_++;
            failures_ = 0;
            if (!open()) close();
            err_ = e;
        }
        return false;
    }
    failures_ = 0;
    sbr_ = info.sbr == SBR_UPSAMPLED || info.sbr == SBR_DOWNSAMPLED;
    ps_ = info.ps != 0;
    if (info.samplerate) rate_ = (unsigned)info.samplerate;
    if (info.channels) ch_ = info.channels;
    if (out && info.samples && info.channels) {
        const float *f = static_cast<const float *>(out);
        pcm.assign(f, f + info.samples);
        frame_ = (unsigned)(info.samples / info.channels);
    }
    return true;
}

bool AacDecoder::decode(const std::vector<std::uint8_t> &au, std::vector<float> &pcm) {
    return decode(au.empty() ? 0 : &au[0], au.size(), pcm);
}

const char *AacDecoder::lastErrorMessage() const {
    if (err_ < 0) return h_ ? "bad input" : "decoder not set up";
    const char *m = faad::NeAACDecGetErrorMessage((unsigned char)err_);
    return m ? m : "unknown error";
}

} // namespace codec
