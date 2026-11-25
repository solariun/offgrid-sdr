// ============================================================================
//  OFFgrid-SDR TUI - example: decode a sound file with any decoder.
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   examples/decode.cpp
//
//  What this file does
//  -------------------
//  Shows liboffgrid-decoders on its own: reads a WAV file (16-bit PCM or
//  32-bit float, mono or stereo, any rate), runs one decoder over it and
//  prints the text (Hellschreiber: the picture as text art).
//
//      make examples
//      bin/example-decode rtty recording.wav
//      bin/example-decode psk  psk31.wav
//
//  Record a signal with offgrid (USB mode, R key or --wav), or with any
//  program that saves the receiver's sound.
// ============================================================================
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "decoders/Decoder.h"

namespace {

unsigned le(const unsigned char* p, int n) {
    unsigned v = 0;
    for (int i = n - 1; i >= 0; --i) v = (v << 8) | p[i];
    return v;
}

// Mono float samples of a WAV file; false + message on failure.
bool readWav(const char* path, std::vector<float>& mono, double& rate, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    std::vector<unsigned char> d((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (d.size() < 44 || std::memcmp(&d[0], "RIFF", 4) || std::memcmp(&d[8], "WAVE", 4)) {
        error = "not a WAV file";
        return false;
    }
    unsigned format = 0, channels = 0, bits = 0;
    size_t pos = 12;
    while (pos + 8 <= d.size()) {
        unsigned size = le(&d[pos + 4], 4);
        if (!std::memcmp(&d[pos], "fmt ", 4)) {
            format = le(&d[pos + 8], 2);
            channels = le(&d[pos + 10], 2);
            rate = le(&d[pos + 12], 4);
            bits = le(&d[pos + 22], 2);
        } else if (!std::memcmp(&d[pos], "data", 4)) {
            size_t end = std::min(d.size(), pos + 8 + size);
            size_t bytes = bits / 8 * channels;
            if (!channels || !bytes || !((format == 1 && bits == 16) || (format == 3 && bits == 32))) {
                error = "only 16-bit PCM or 32-bit float WAV files";
                return false;
            }
            for (size_t p = pos + 8; p + bytes <= end; p += bytes) {
                float sum = 0;
                for (unsigned c = 0; c < channels; ++c) {
                    const unsigned char* s = &d[p + c * bits / 8];
                    if (format == 1) sum += static_cast<short>(le(s, 2)) / 32768.0f;
                    else {
                        unsigned u = le(s, 4);
                        float f;
                        std::memcpy(&f, &u, 4);
                        sum += f;
                    }
                }
                mono.push_back(sum / channels);
            }
            return true;
        }
        pos += 8 + size + (size & 1);
    }
    error = "no sound data";
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    dec::Kind kind;
    if (argc < 3 || !dec::parseKind(argv[1], kind)) {
        std::fprintf(stderr, "usage: %s cw|rtty|psk|navtex|dsc|hell file.wav\n", argv[0]);
        return 2;
    }
    std::vector<float> audio;
    double rate = 0;
    std::string error;
    if (!readWav(argv[2], audio, rate, error)) {
        std::fprintf(stderr, "%s: %s\n", argv[2], error.c_str());
        return 1;
    }
    std::unique_ptr<dec::Decoder> decoder = dec::makeDecoder(kind);
    std::vector<std::vector<float> > columns;
    const size_t block = 4096;
    for (size_t i = 0; i < audio.size(); i += block) {
        decoder->process(&audio[i], std::min(block, audio.size() - i), rate);
        std::string text;
        decoder->takeText(text);
        std::fputs(text.c_str(), stdout);
        if (decoder->hasImage()) decoder->takeColumns(columns);
    }
    if (!columns.empty()) {
        // Hellschreiber: columns become rows of text, top dot first.
        const size_t h = columns[0].size();
        for (size_t row = 0; row < h; ++row) {
            for (size_t c = 0; c < columns.size(); ++c) std::fputc(columns[c][row] > 0.5f ? '#' : ' ', stdout);
            std::fputc('\n', stdout);
        }
    }
    dec::Status s = decoder->status();
    std::printf("\n-- %s: %u characters, %s\n", dec::kindName(kind), s.characters, s.info.c_str());
    return 0;
}
