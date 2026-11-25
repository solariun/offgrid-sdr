// ============================================================================
//  offgrid-dsp - reusable C++11 radio signal processing (part of OFFgrid-SDR
//  TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/AutoMode.h
//  Layer:  Library liboffgrid-dsp (public header)
//
//  What this file does
//  -------------------
//  dsp::AutoDetector: OFFgrid-SDR's AUTO ("work out what the signal is, and
//  switch to it", diddy-boy, index.html), without the page.  Given where the
//  user tuned (the dial), it listens about 2 seconds to the stick's IQ near
//  the dial and says what the signal is and how to receive it:
//    - by frequency alone, without listening: a DAB channel (Band III), the
//      fixed-mode bands (airband AM, ACARS and VDL2 channels, AIS, APRS,
//      108-118 MHz navigation, PMR446, UK CB, marine VHF), the FM broadcast
//      band (wideband FM);
//    - by listening: narrowband FM (VHF/UHF), DRM (a flat block 4.5-20 kHz
//      wide), weather fax, FT8 / FT4 (on their dials), time stations and WSPR
//      (by their frequencies), RTTY / NAVTEX / DSC (two tones a standard shift
//      apart that take turns), PSK31/63/125, Hellschreiber, CW, AM (steady or
//      fading), SSTV, SSB voice (USB or LSB judged from the voice, the
//      carrier found to within a few hertz from the speaker's harmonics);
//    - nothing clear twice: a fall-back (AM, the ham band's sideband, or
//      narrowband FM on VHF) and found = false.
//  The answer is a receiver mode (dsp::Mode names), a decoder kind
//  (dec::Kind names) and the frequency to tune to, as the JS tunes.
//
//  How it fits in
//  --------------
//  Pure engine: no UI, no decoder dependency (strings).  The application:
//    1. begin(dial) when AUTO is pressed, or (with AUTO on) about 1.2 s after
//       the tuning settles somewhere new (not within 300 Hz of AUTO's own
//       last answer);  if it returns true the answer is ready at once;
//    2. otherwise feeds every IQ block with feedIq(iq, n, rate, centre) (the
//       raw samples, before any demodulator shift; the first 2 blocks are
//       skipped as the JS's, being possibly from before the retune) until
//       ready() (about 2 s of samples; 4 s when it listens again);
//    3. reads result(): selects result().mode, starts result().decoder, and
//       tunes the dial to result().tuneHz (for the decoder modes the JS's
//       dial: the signal's listed or centre frequency; the receiver's USB
//       carrier is then tuneHz - audioShiftHz).  For DAB, confirm with the
//       DAB receiver (a lock within 6 s, else narrowband FM, as the JS).
//  A retune while listening: call begin() again.
//
//  Contributing
//  ------------
//  Each rule carries the JS's reason in brackets: they were tuned on real
//  signals and the JS's test bench; change a threshold only with a test.
//  Frequency lists (FT8 dials, fax stations, time stations ...) are copied
//  from index.html.  Strict C++11; allocation only at begin() and in the
//  one-off analysis.  Ported from OFFgrid-SDR by diddy-boy.
// ============================================================================
#ifndef OFFGRID_DSP_AUTOMODE_H
#define OFFGRID_DSP_AUTOMODE_H

#include "SignalMonitor.h"

#include <cstddef>
#include <string>
#include <vector>

namespace dsp {

struct AutoResult {
    AutoResult();
    bool ready;          // a verdict (found or the fall-back) is available
    bool found;          // AUTO recognised something (false: the fall-back)
    bool listened;       // false: decided by frequency alone
    bool needsConfirm;   // DAB: the DAB receiver must lock within 6 s (else NFM)
    bool exact;          // tuned on the station's own carrier (nothing to refine)
    std::string jsMode;  // the JS's mode: "NBFM", "WBFM", "AM", "USB", "LSB", "CW", "RTTY",
                         // "NAVTEX", "DSC", "FAX", "SSTV", "PSK31", "HELL", "FT8", "FT4",
                         // "WSPR", "TIME", "DRM", "DAB", "VDL2", "ACARS", "AIS", "APRS"
    std::string mode;    // the receiver's mode (dsp::modeName): "WFM", "NFM", "AM", "USB", "LSB", "CW"
    std::string decoder; // dec::kindName, or "" (none): "RTTY", "PSK", "NAVTEX", "DSC", "FAX",
                         // "SSTV", "HELL", "CW", "FT8", "FT4", "WSPR", "TIME", "ACARS", "AX25",
                         // "AIS", "VDL2", "DRM", "DAB"
    std::string sideband;  // AM: "both" (the JS's selectMode argument), else ""
    double dialHz;       // where the user was tuned
    double tuneHz;       // where to tune (the JS's tuneTo)
    double retuneHz;     // tuneHz - dialHz
    double audioShiftHz; // decoder modes: the tone's place in the USB audio (RTTY 1500, PSK 1000,
                         // FAX 1900, FT8/FT4/WSPR 1500, TIME 1000; else 0)
    std::string label;   // "RTTY · 170 Hz shift", "SSB voice (LSB)" ...
    std::string sure;    // "likely", "possibly" ("" for the fall-back)
    double confidence;   // 0..1: 1 by frequency, 0.8 likely, 0.5 possibly, 0 not found
    std::string reason;  // one line for the status bar
};

class AutoDetector {
public:
    AutoDetector();

    // Demo band: 7425 kHz counts as a fax frequency (the JS's window.__sdrDemo).
    void setDemo(bool on) { demo_ = on; }

    // Starts a detection at the dial.  Returns true when the frequency alone
    // decides (result() is ready, nothing to feed).
    bool begin(double dialHz);
    void reset();  // forgets everything (no detection under way)

    // IQ from the stick (centreHz: its centre).  Mixed down to the dial and
    // decimated as it arrives (to ~32 kHz on HF, ~64 kHz on VHF/UHF); about
    // 2 s are kept, then the analysis runs here, in this call (some tens of
    // ms).  Returns true when result() is ready.
    bool feedIq(const Complex* iq, std::size_t n, double sampleRate, double centreHz);
    // Addition to the JS: the receiver's USB audio instead (the dial is the
    // suppressed carrier at 0 Hz; at least 16 kHz): made analytic (a Hilbert
    // FIR) and analysed as the ±4 kHz view.  Only what lies above the dial is
    // seen (no LSB, no DRM, no VHF FM width): for HF data and CW signals when
    // no IQ is available.  Use one of feedIq / feedAudio per detection.
    bool feedAudio(const float* audio, std::size_t n, double audioRate);

    bool listening() const { return active_ && !res_.ready; }
    bool ready() const { return res_.ready; }
    double progress() const;  // 0..1 of the current listen
    const AutoResult& result() const { return res_; }

    // The frequency rules alone (DAB, fixed-mode bands, FM broadcast): true
    // and r filled when they decide.
    static bool byFrequency(double dialHz, AutoResult& r);

    // The rules once listened, on a capture already mixed down (the dial at
    // 0 Hz): zr/zi at fs.  view: 0 = the ±16 kHz HF view (fs ~32 kHz),
    // 1 = the ±32 kHz VHF view (fs ~64 kHz), 2 = the ±4 kHz view (fs ~8 kHz,
    // no DRM or FM-width test).  dcRelHz: the stick's DC spike relative to the
    // dial (NaN: none); rawRate: the stick's rate (for the folded spike).
    // Returns found = false when nothing is clear (no fall-back applied).
    static AutoResult analyse(const std::vector<float>& zr, const std::vector<float>& zi, double fs, int view,
                              double dialHz, double dcRelHz, double rawRate, bool demo = false);

private:
    void startListen();
    void finishListen();
    bool demo_, active_, retry_, audio_;
    double dial_, centre_, rate_, want_, got_;
    int skip_, view_;
    ChannelExtractor ext_;
    std::vector<Complex> buf_;
    // feedAudio's Hilbert transformer
    std::vector<float> hil_, hring_;
    std::vector<Complex> hout_;
    std::size_t hpos_;
    AutoResult res_;
};

}  // namespace dsp

#endif
