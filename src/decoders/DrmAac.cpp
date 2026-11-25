// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DrmAac.cpp
//  Layer:  Library liboffgrid-decoders
//
//  What this file does
//  -------------------
//  The DRM receiver's sound: drm::makeAacCodec() wraps liboffgrid-codecs'
//  AAC decoder (src/codecs/Aac.h, codec::AacDecoder: HE-AAC v2 with faad2's
//  DRM mode) behind the small drm::AudioCodec interface the receiver codes
//  against.
//
//  As the JS's faadFor (OFFgrid-SDR, diddy-boy, index.html "drmlib"):
//  opened for DRM with the core sampling rate (24 kHz for rate code 3, else
//  12 kHz) and the channel configuration (SBR or not; mono, parametric
//  stereo or stereo); each access unit goes in with its DRM CRC byte first,
//  and the decoder checks it.  Output: always stereo pairs.  (The JS's DRM
//  receiver follows Dream, TU Darmstadt, GPL-2.0-or-later,
//  licenses/dream-COPYING.txt, which drives faad2 the same way.)
//
//  How it fits in
//  --------------
//  Declared in DrmInternal.h; used by Drm.cpp.  liboffgrid-codecs is a
//  required library of liboffgrid-decoders (link liboffgrid-codecs.a).
//
//  Contributing
//  ------------
//  Only this file knows the codec's API.
// ============================================================================
#include "decoders/DrmInternal.h"

#include "codecs/Aac.h"

namespace dec {
namespace drm {

namespace {

class AacCodec : public AudioCodec {
public:
    bool configure(const AudioParams& a) {
        if (a.codec != 0) return false;
        // the AAC (core) rate from the SDC's rate code (as the JS's faadFor:
        // 24 kHz for code 3, 12 kHz for code 1; 48 kHz for code 5, DRM+)
        const unsigned rate = a.rateCode == 3 ? 24000 : a.rateCode == 1 ? 12000 : a.rateCode == 5 ? 48000 : 0;
        if (!rate) return false;
        const bool ps = a.mode == 1 && a.sbr, stereo = a.mode == 2;
        ok_ = dec_.init(codec::AacParams::drmAudio(rate, a.sbr, stereo, ps));
        return ok_;
    }
    bool decode(const unsigned char* crcAndAu, std::size_t n, std::vector<float>& stereo, int& rate) {
        if (!ok_) return false;
        if (!dec_.decode(crcAndAu, n, pcm_)) return false;
        // (always stereo pairs: the codec gives 2 channels, mono doubled)
        if (dec_.channels() == 2)
            stereo.insert(stereo.end(), pcm_.begin(), pcm_.end());
        else if (dec_.channels() == 1)
            for (std::size_t i = 0; i < pcm_.size(); ++i) {
                stereo.push_back(pcm_[i]);
                stereo.push_back(pcm_[i]);
            }
        else
            for (std::size_t i = 0; i + dec_.channels() <= pcm_.size(); i += dec_.channels()) {
                stereo.push_back(pcm_[i]);
                stereo.push_back(pcm_[i + 1]);
            }
        if (dec_.sampleRate()) rate = static_cast<int>(dec_.sampleRate());
        return true;
    }
    AacCodec() : ok_(false) {}

private:
    codec::AacDecoder dec_;
    std::vector<float> pcm_;
    bool ok_;
};

}  // namespace

AudioCodec* makeAacCodec() { return new AacCodec; }

}  // namespace drm
}  // namespace dec
