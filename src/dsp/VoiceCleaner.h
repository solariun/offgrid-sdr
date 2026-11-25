// ============================================================================
//  offgrid-dsp - liboffgrid-dsp, the signal processing library of OFFgrid-SDR TUI.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr) by diddy-boy.
//  SPDX-License-Identifier: GPL-3.0-or-later AND BSD-3-Clause
//  (dsp::rnnoise is a C++11 port of RNNoise; RNNoise's BSD notice is kept in
//   VoiceCleaner.cpp and RnnoiseModel.cpp)
// ----------------------------------------------------------------------------
//  File:   src/dsp/VoiceCleaner.h
//  Layer:  Library liboffgrid-dsp
//
//  What this file does
//  -------------------
//  The "AI voice cleaner" of OFFgrid-SDR: RNNoise, a small recurrent neural
//  network trained on speech and noise by Jean-Marc Valin (Xiph.Org
//  Foundation / Mozilla, BSD-3-Clause, https://github.com/xiph/rnnoise),
//  which removes noise from speech 10 ms at a time.
//
//    dsp::rnnoise::Denoiser   an exact C++11 port of RNNoise's "classic"
//                             frame pipeline (release v0.1.1): 480-sample
//                             frames at 48 kHz, in int16 scale (+-32768).
//                             Per frame: windowed 960-point FFT, 22 band
//                             energies, pitch search and pitch filter,
//                             42 features (cepstrum, deltas, pitch
//                             correlation, spectral variability), the GRU
//                             network -> 22 band gains + voice probability,
//                             gains applied, inverse FFT, overlap-add.
//    dsp::rnnoise::Model      the network's layer descriptions; the weights
//                             (the same ones the browser app runs) are in
//                             RnnoiseModel.cpp, defaultModel().
//    dsp::VoiceCleaner        what the receiver uses: audio in -1..1 at any
//                             rate, cleaned in place, mixed with the original
//                             by a strength (Full / Strong / Medium / Light),
//                             exactly as OFFgrid-SDR's AiVoiceCleaner.
//
//  How it fits in
//  --------------
//  dsp::Receiver (Receiver.cpp) runs a VoiceCleaner on the mono audio at the
//  audio rate, after the notches and before noise reduction, when
//  ReceiverConfig::voiceCleaner is on and the mode is a speech mode (AM, SAM,
//  USB, LSB), as the JS does.  The browser app runs RNNoise compiled to
//  WebAssembly (@jitsi/rnnoise-wasm 0.2.1, Apache-2.0); this is the same C
//  code, ported, with the same weights (checked byte for byte against the
//  WASM's data section), so its output matches the reference C build to
//  float precision (tests/test_filters.cpp compares them frame by frame).
//
//  Contributing
//  ------------
//  The Denoiser follows the C code statement by statement, including where C
//  computes in double (sqrt/log10 of floats, double constants): keep it that
//  way, or the frame-by-frame comparison with the reference drifts.  In C++
//  std::sqrt(float) is a float function while C's sqrt() is double, so every
//  such call is written with an explicit static_cast<double>.  No allocation
//  per frame: all state is fixed arrays; VoiceCleaner's queues only grow.
// ============================================================================
#ifndef OFFGRID_DSP_VOICECLEANER_H
#define OFFGRID_DSP_VOICECLEANER_H

#include <cstddef>
#include <memory>
#include <vector>

namespace dsp {
namespace rnnoise {

// ---- The network's description (rnn.h / rnn_data.h of RNNoise) -------------
enum Activation { kActTanh = 0, kActSigmoid = 1, kActRelu = 2 };

// A fully connected layer: output[i] = act(scale * (bias[i] + sum_j w[j*N+i] in[j])).
struct DenseLayer {
    const signed char* bias;
    const signed char* inputWeights;
    int nbInputs;
    int nbNeurons;
    int activation;
};

// A GRU layer: three gates (update, reset, output) stored side by side, so
// the weights of input j for neuron i of gate g are at [j * 3N + g * N + i].
struct GruLayer {
    const signed char* bias;
    const signed char* inputWeights;
    const signed char* recurrentWeights;
    int nbInputs;
    int nbNeurons;
    int activation;
};

struct Model {
    int inputDenseSize;
    const DenseLayer* inputDense;
    int vadGruSize;
    const GruLayer* vadGru;
    int noiseGruSize;
    const GruLayer* noiseGru;
    int denoiseGruSize;
    const GruLayer* denoiseGru;
    int denoiseOutputSize;
    const DenseLayer* denoiseOutput;
    int vadOutputSize;
    const DenseLayer* vadOutput;
};

// RNNoise's built-in model (rnnoise_model_orig of v0.1.1), RnnoiseModel.cpp.
const Model& defaultModel();

const int kFrameSize = 480;     // samples per frame (10 ms at 48 kHz)
const int kBands = 22;          // gain bands (ERB-like, 0 .. 20 kHz)
const int kFeatures = 42;       // network inputs per frame

// ---- One RNNoise denoiser (DenoiseState + rnnoise_process_frame) -----------
class Denoiser {
public:
    Denoiser();
    ~Denoiser();
    void reset();  // as rnnoise_init(): all state to zero

    // One frame: kFrameSize samples in int16 scale (a full-scale sine is
    // +-32767) in, the same number out (10 ms later: the overlap-add delay).
    // Returns the voice activity probability (0..1; 0 for a silent frame).
    // `in` and `out` may be the same array.
    float processFrame(float* out, const float* in);

    // The network's 22 band gains for the last frame (before the 0.6
    // release smoothing), all 0 when that frame was silent (no network run).
    const float* lastGains() const;
    bool lastSilent() const;

private:
    Denoiser(const Denoiser&);
    Denoiser& operator=(const Denoiser&);
    struct State;
    std::unique_ptr<State> s_;
};

}  // namespace rnnoise

// ---- The AI voice cleaner (OFFgrid-SDR's AiVoiceCleaner) ----------------------
// Strength = the share of cleaned audio in the output ("wet"): Full 1.0,
// Strong 0.9, Medium 0.75 (OFFgrid-SDR's default), Light 0.5.  The rest is
// the original audio, so music (which RNNoise silences) stays audible, more
// quietly, at the cost of some background noise.
class VoiceCleaner {
public:
    static const double kFull, kStrong, kMedium, kLight;

    explicit VoiceCleaner(double sampleRate = 48000);
    ~VoiceCleaner();

    // Addition to the JS: the JS runs only at 48 kHz (and does nothing at
    // other rates).  Here any rate works: the audio is resampled to 48 kHz
    // and back internally.  Changing the rate resets the cleaner.
    void setSampleRate(double sampleRate);
    double sampleRate() const;
    void setMix(double wet);  // 0..1, default kMedium
    double mix() const;
    void reset();             // forget everything (new station)

    // Cleans `n` mono samples (-1..1) in place.  The output is delayed:
    // the very first 10 ms (one frame) come out as silence while the first
    // frame fills, then everything flows with a constant delay (as the JS).
    void process(float* x, std::size_t n);

    // The voice activity probability of the last frame (0..1).
    float voiceProbability() const;

private:
    VoiceCleaner(const VoiceCleaner&);
    VoiceCleaner& operator=(const VoiceCleaner&);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dsp

#endif
