// ============================================================================
//  offgrid-dsp - reusable C++11 radio signal processing (part of OFFgrid-SDR
//  TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/dsp/SpectrumTools.h
//  Layer:  Library liboffgrid-dsp (public header)
//
//  What this file does
//  -------------------
//  dsp::SpectrumExtras: peak hold, averaging and new-signal alerts over the
//  displayed spectrum, as OFFgrid-SDR's specExtras (diddy-boy, index.html:
//  "peak hold, averaging and new-signal alerts"):
//    - averaging: each pixel 0.8 x before + 0.2 x now;
//    - peak hold: the highest level, falling about 3 dB a second;
//    - new-signal alerts: "new" means a signal where there was NOTHING
//      before, not a louder one (broadcast stations fade and swell all the
//      time).  Each spot remembers when it last had a signal (the smoothed
//      level 9 dB over the median); an alert needs the spot and its
//      neighbours to have been empty for 2 minutes and the signal there for
//      3 frames.  For the first 10 s of a view it learns: spots occupied
//      at least half the time are known (broadcasters, beacons); others
//      (intermittent CW) flash once at their next burst.  One alert per
//      signal (neighbouring pixels grouped, at the strongest); spurs more
//      than 20 dB under the strongest new signal of the same frame are its
//      offspring, not stations; nothing inside the tuned passband.
//
//  How it fits in
//  --------------
//  Pure, over spectrum frames (dB per pixel, as drawn).  The application
//  calls frame() for each drawn spectrum line with the view's edges, the
//  passband and the time; it draws average()/peak() and a pulse at each of
//  lastAlerts().  It should not call it (or pass alerts = false) while
//  scanning, in Panorama, while the waterfall is paused or in Low-power
//  mode, as the JS.  Any change of view (edges or width) starts afresh.
//
//  Contributing
//  ------------
//  Strict C++11; no allocation per frame once the width is stable.  Ported
//  from OFFgrid-SDR by diddy-boy.
// ============================================================================
#ifndef OFFGRID_DSP_SPECTRUMTOOLS_H
#define OFFGRID_DSP_SPECTRUMTOOLS_H

#include <vector>

namespace dsp {

struct SpectrumAlert {
    double hz;       // the new signal's frequency (its strongest pixel's centre)
    double t;        // when (seconds, the caller's clock)
    float level;     // its smoothed level, dB
};

class SpectrumExtras {
public:
    SpectrumExtras();

    void setAveraging(bool on) { avgOn_ = on; }
    void setPeakHold(bool on) { peakOn_ = on; }
    void setAlerts(bool on) { alertsOn_ = on; }
    bool averaging() const { return avgOn_; }
    bool peakHold() const { return peakOn_; }
    bool alerts() const { return alertsOn_; }

    // One frame: W pixels of dB (non-finite = no data), the view loHz..hiHz,
    // now in seconds, and the tuned passband (no alerts inside it, ±2 px).
    void frame(const float* pix, int W, double loHz, double hiHz, double now, double passLoHz, double passHiHz);
    // Starts the view afresh (learning again for 10 s).
    void restart();

    const std::vector<float>& average() const { return avg_; }
    const std::vector<float>& peak() const { return peak_; }
    bool learning(double now) const { return now < learnUntil_; }
    // The alerts raised by the last frame() call.
    const std::vector<SpectrumAlert>& lastAlerts() const { return fresh_; }
    // All alerts of the last 10 minutes, and the total since the start.
    const std::vector<SpectrumAlert>& recentAlerts() const { return seen_; }
    long alertCount() const { return count_; }

private:
    bool avgOn_, peakOn_, alertsOn_, haveKey_, haveLast_, haveFrame_;
    double keyLo_, keyHi_;
    int keyW_;
    double learnUntil_, lastFrame_;
    long frames_, count_;
    std::vector<float> avg_, peak_, sm_, pres_, srt_;
    std::vector<unsigned char> hit_;
    std::vector<double> last_;
    std::vector<int> cands_;
    std::vector<SpectrumAlert> fresh_, seen_;
};

}  // namespace dsp

#endif
