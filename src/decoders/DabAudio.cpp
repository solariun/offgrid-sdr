// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DabAudio.cpp
//  Layer:  Library liboffgrid-decoders (internal)
//
//  What this file does
//  -------------------
//  The DAB receiver's sound: dab::AudioCodecs (DabInternal.h) over
//  liboffgrid-codecs' decoders, codec::Mp2Decoder (MPEG-1/2 Layer II
//  frames, DAB) and codec::AacDecoder (HE-AAC v2 access units, DAB+: set
//  up by explicit parameters, AacParams::dabPlus: 960-sample frames, the
//  core at half the output rate with SBR, PS).  Both give interleaved
//  stereo floats (a mono stream as two equal channels); anything else is
//  made stereo here.  The AAC decoder is set up again whenever the super
//  frame header's format changes.
//
//  How it fits in
//  --------------
//  Addition to the JS, whose worker runs FAAD2 and pl_mpeg's MP2 decoder as
//  WebAssembly (dabAac, dabMp2).  Dab.cpp only sees the small interface, so
//  the receiver and its tests stand alone; this file is the one place that
//  knows the codecs.  Build with src/codecs/*.cpp; to build the receiver
//  without liboffgrid-codecs define OFFGRID_DAB_NO_CODECS (available() is
//  then false: everything up to the audio units is still decoded).
//
//  Contributing
//  ------------
//  Follow the codecs' headers (src/codecs/Mp2.h, Aac.h); do not edit them
//  from here.  Run tests/test_decoders_dab.cpp (it checks PCM comes out).
// ============================================================================
#include "DabInternal.h"

#ifndef OFFGRID_DAB_NO_CODECS
#include "codecs/Aac.h"
#include "codecs/Mp2.h"
#endif

namespace dec {
namespace dab {

namespace {

#ifdef OFFGRID_DAB_NO_CODECS

class Codecs : public AudioCodecs {
public:
    bool available() const { return false; }
    bool aac(const AacFormat&, const unsigned char*, std::size_t, std::vector<float>&, int&) { return false; }
    bool mp2(const unsigned char*, std::size_t, std::vector<float>&, int&) { return false; }
    void reset() {}
};

#else

class Codecs : public AudioCodecs {
public:
    Codecs() : aacSet_(false), aacOk_(false) {}
    bool available() const { return true; }

    bool aac(const AacFormat& f, const unsigned char* au, std::size_t n, std::vector<float>& stereo, int& rate) {
        if (!aacSet_ || f != fmt_) {   // (FAAD2, (re)configured when the station's format changes: JS dabAac)
            fmt_ = f;
            aacSet_ = true;
            aacOk_ = aacDec_.init(codec::AacParams::dabPlus(static_cast<unsigned>(f.rate), f.sbr, f.stereo, f.ps));
        }
        if (!aacOk_) return false;
        if (!aacDec_.decode(au, n, pcm_)) return false;
        rate = static_cast<int>(aacDec_.sampleRate());
        append(pcm_, aacDec_.channels(), stereo);
        return true;
    }

    bool mp2(const unsigned char* fr, std::size_t n, std::vector<float>& stereo, int& rate) {
        if (!mp2Dec_.decode(fr, n, pcm_)) return false;
        rate = static_cast<int>(mp2Dec_.sampleRate());
        append(pcm_, mp2Dec_.channels(), stereo);
        return true;
    }

    void reset() {
        aacSet_ = false;
        aacOk_ = false;
        aacDec_.close();
        mp2Dec_.reset();
    }

private:
    static void append(const std::vector<float>& pcm, unsigned ch, std::vector<float>& stereo) {
        if (ch == 2) {
            stereo.insert(stereo.end(), pcm.begin(), pcm.end());
            return;
        }
        if (ch < 1) return;
        for (std::size_t i = 0; i + ch <= pcm.size(); i += ch) {   // (mono doubled; more than 2: the first two)
            stereo.push_back(pcm[i]);
            stereo.push_back(ch > 1 ? pcm[i + 1] : pcm[i]);
        }
    }

    codec::AacDecoder aacDec_;
    codec::Mp2Decoder mp2Dec_;
    AacFormat fmt_;
    bool aacSet_, aacOk_;
    std::vector<float> pcm_;
};

#endif

}  // namespace

AudioCodecs* newAudioCodecs() { return new Codecs; }

}  // namespace dab
}  // namespace dec
