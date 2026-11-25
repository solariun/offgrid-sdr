// ============================================================================
//  offgrid-sdrconnect - an SDRplay RSP through SDRplay's SDRconnect WebSocket
//  API, in C++11 (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/sdrconnect/SdrConnect.h
//  Layer:  Library liboffgrid-sdrconnect
//
//  What this file does
//  -------------------
//  Declares sdrc::Client: an SDRplay RSP (RSP1A, RSP1B, RSPdx, RSPduo...)
//  reached through SDRplay's SDRconnect program and its WebSocket API
//  (SDRplay's published specification 1.0.3; SDRconnect 1.0.6 or later,
//  1.0.10 still reports API 1.0.3), on this computer or another on the
//  network, port 5454.  Ported from OFFgrid-SDR's browser version (by
//  diddy-boy), sections "SDRconnect (from 2.3)", "SDRconnect Lite mode
//  (from 2.4)" and "the speed test (from 2.4)" of index.html.
//
//  The protocol, as OFFgrid-SDR uses it
//  ------------------------------------
//  Commands are JSON text messages, always three string fields:
//      {"event_type":"get_property","property":"device_center_frequency","value":""}
//      {"event_type":"set_property","property":"lna_state","value":"4"}
//      {"event_type":"iq_stream_enable","property":"","value":"true"}
//  event_type: get_property, set_property, iq_stream_enable,
//  audio_stream_enable, spectrum_enable, device_stream_enable,
//  selected_device_name (value = the device), apply_device_profile (value =
//  the profile's name).  SDRconnect answers with text messages
//      {"event_type":"get_property_response","property":"...","value":"..."}
//      {"event_type":"property_changed","property":"...","value":"..."}
//  (an optional "device" field: only "primary", or none, is taken).
//  Properties used: api_version, valid_devices, active_device,
//  valid_antennas, active_antenna, lna_state_min, lna_state_max, lna_state,
//  device_sample_rate, device_center_frequency, overload, can_control,
//  started, agc_enable, demodulator, device_vfo_frequency,
//  filter_bandwidth, spectrum_ref_level, spectrum_base, audio_mute,
//  audio_volume_percent.  Lists (valid_devices, valid_antennas) are
//  comma-separated; booleans are "true"/"false".
//
//  Binary messages: a 16-bit little-endian type, then the payload:
//    type 2  IQ: signed 16-bit little-endian I, Q, I, Q ... at
//            device_sample_rate; a sample is value / 32768 (full scale
//            +-1).  4 bytes a sample: 8.2 MB/s at 2.048 MHz, 40 MB/s at
//            10 MHz.
//    type 1  Lite sound: signed 16-bit little-endian stereo L, R, L, R ...
//            at 48 kHz (value / 32768), after SDRconnect's own demodulator,
//            filter and volume (about 0.19 MB/s).
//    type 3  Lite spectrum: one unsigned byte a bin, lowest frequency
//            first; a bin's level in dB is
//              spectrum_base + byte / 255 * (spectrum_ref_level - spectrum_base)
//            (defaults 0 and -120 dB when SDRconnect hasn't said).
//  Messages shorter than 6 bytes, and other types, are ignored (as the JS).
//
//  How it fits in
//  --------------
//  A terminal OFFgrid-SDR's Source adapter (src/app) makes one Client:
//      sdrc::Client rsp;
//      std::string err;
//      if (!rsp.connect("shack-pc", err)) ...           // "host[:port]", 5454 by default
//      rsp.setSampleRate(2048000);
//      rsp.setFrequency(100e6);
//      rsp.setGainAuto();                                // or setGainFraction(0.6)
//      rsp.startIq([](const std::complex<float>* iq, std::size_t n) { ... }, err);
//      ... rsp.setLite(true) for Lite mode, rsp.stop(), rsp.disconnect()
//  All callbacks come on the client's own thread (one thread, which also
//  reads the socket): copy the samples and return quickly.  Every other
//  call is thread-safe; the blocking ones (connect, startDevice, startIq,
//  startLite, stop, speedTest, probeAntennas, applyProfile) must not be
//  called from a callback (they return false with an error if they are).
//
//  Contributing
//  ------------
//  Keep the behaviour as OFFgrid-SDR's JS has it (the comments say which
//  JS function each part follows); anything new is marked "Addition to the
//  JS".  Never set a property beyond SDRplay's specification 1.0.3: newer
//  SDRconnect versions may have more, but only asking about them is safe.
// ============================================================================
#ifndef OFFGRID_SDRCONNECT_SDRCONNECT_H
#define OFFGRID_SDRCONNECT_SDRCONNECT_H

#include <complex>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace sdrc {

// The link, as OFFgrid-SDR's NET badge shows it.
enum class LinkState {
    Off,         // not connected (or disconnected on purpose)
    Connecting,  // connecting, or reconnecting
    Connected,   // connected (api_version answered)
    Lost         // dropped while streaming: retrying every few seconds
};

// The speed test's verdict (sdrcSpeedSay): >= 97% of the stream's rate
// keeps up; 90-97% only just; under 90% too slow.
enum class SpeedVerdict { KeepsUp, OnlyJust, TooSlow, Failed };

struct SpeedResult {
    SpeedResult() : ok(false), gotMBs(0), needMBs(0), rate(0), percent(0), verdict(SpeedVerdict::Failed) {}
    bool ok;               // false: error says why (no figures)
    double gotMBs;         // measured, MB/s (10^6 bytes)
    double needMBs;        // the full stream at rate: rate * 4 / 1e6
    double rate;           // the sample rate tested, Hz
    int percent;           // round(100 * got / need)
    SpeedVerdict verdict;
    std::string text;      // the verdict in words, as OFFgrid-SDR says it
    std::string error;
};

// What Lite mode is receiving (the JS's sdrc.liteStat, each second).
struct LiteStats {
    LiteStats() : soundFramesPerSec(0), soundDb(-200), spectrumFramesPerSec(0), bins(0), stereoSeen(false) {}
    double soundFramesPerSec;     // audio messages a second
    double soundDb;               // the sound's mean power, dB full scale (-200: none)
    double spectrumFramesPerSec;  // spectrum messages a second
    std::size_t bins;             // bins in the last spectrum message
    bool stereoSeen;              // L != R seen at least once
};

// The link's figures (the NET badge's details).
struct LinkStats {
    LinkStats() : bytes(0), MBs(0), iqSamples(0), droppedMessages(0), reconnects(0) {}
    std::uint64_t bytes;            // binary payload bytes received, all types
    double MBs;                     // over the last second
    std::uint64_t iqSamples;        // IQ samples delivered to the callback
    std::uint64_t droppedMessages;  // IQ messages discarded (retuning, settling, speed test)
    unsigned reconnects;
};

// Which antenna for which frequency (sdrcAntRules), per device.
struct AntennaRule {
    enum Kind { None, Below, Above, Range };
    AntennaRule() : kind(None), loMHz(0), hiMHz(0) {}
    AntennaRule(Kind k, double lo = 0, double hi = 0) : kind(k), loMHz(lo), hiMHz(hi) {}
    Kind kind;            // below the crossover, from it up, a custom range (wins), or not used
    double loMHz, hiMHz;  // Range: [lo, hi)
};
struct AntennaRules {
    AntennaRules() : crossMHz(30), user(false) {}
    double crossMHz;
    std::map<std::string, AntennaRule> map;
    bool user;  // changed by the user: kept as they are
};

// IQ at sampleRate(), complex float, full scale +-1 (centred on frequency()).
typedef std::function<void(const std::complex<float>* iq, std::size_t n)> IqCallback;
// Lite sound: 48 kHz, interleaved L, R, full scale +-1.
typedef std::function<void(const float* lr, std::size_t frames)> AudioCallback;
// Lite spectrum: dB per bin, lowest frequency first, across SDRconnect's span.
typedef std::function<void(const float* dB, std::size_t bins)> SpectrumCallback;
typedef std::function<void(LinkState state, const std::string& message)> StateCallback;
typedef std::function<void(const std::string& property, const std::string& value)> PropertyCallback;

class Client {
public:
    static const int kDefaultPort = 5454;
    static const int kLiteAudioRate = 48000;

    Client();
    ~Client();  // stops and disconnects politely (as the JS's pagehide)

    // ---- connection --------------------------------------------------------
    // "host", "host:port", "[v6]:port", "ws://host:port/path"; 5454 by default
    // (sdrcUrl).  wss:// is refused (SDRconnect's API is plain ws).
    static bool parseAddress(const std::string& address, std::string& host, int& port,
                             std::string& path, std::string& error);
    // Connects and waits for SDRconnect's api_version (then a quarter of a
    // second for the other first replies), within timeoutMs.  The messages
    // are OFFgrid-SDR's own (port 50000 explained, no answer, refused).
    bool connect(const std::string& address, std::string& error, int timeoutMs = 5000);
    // stop() if streaming, then a clean close (code 1000).
    void disconnect();
    bool isConnected() const;
    LinkState state() const;
    std::string url() const;  // ws://host:port/path

    // Retry a lost link (while streaming) every retryMs, the first after
    // firstMs (the JS: 3000 and 1000).  On by default.
    void setAutoReconnect(bool on, int retryMs = 3000, int firstMs = 1000);

    void setStateCallback(StateCallback cb);
    void setPropertyCallback(PropertyCallback cb);  // every property answer/change

    // ---- what SDRconnect says ----------------------------------------------
    std::string apiVersion() const;
    std::string device() const;                 // active_device, e.g. "RSPdx"
    std::vector<std::string> devices() const;   // valid_devices
    bool canControl() const;                    // false: another program controls the RSP
    bool overload() const;
    bool deviceStarted() const;                 // "started"
    std::string property(const std::string& name) const;  // last value ("" if none)
    bool hasProperty(const std::string& name) const;

    // ---- running -----------------------------------------------------------
    // The RSP started carefully (sdrcStart): if SDRconnect isn't running it,
    // the device is selected (only if not already) and started, waiting
    // (15 s at most) until SDRconnect says it runs; then the start profile,
    // if any, is applied.
    bool startDevice(std::string& error);
    // The full IQ stream (startDevice first if needed); from Lite mode, the
    // streams are swapped over.
    bool startIq(IqCallback cb, std::string& error);
    // Lite mode: SDRconnect's own sound and spectrum (startDevice first if
    // needed); from the full stream, swapped over.
    bool startLite(AudioCallback audio, SpectrumCallback spectrum, std::string& error);
    // Switches between the two while running (callbacks as last given).
    bool setLite(bool on);
    bool lite() const;
    bool streaming() const;
    // The streams off, the RSP stopped (waiting until SDRconnect says so,
    // 3 s at most); the connection stays open (sdrcStop, less the close).
    void stop();

    // ---- tuning ------------------------------------------------------------
    // The RSP's centre.  Samples are dropped until SDRconnect confirms the
    // retune (within 5 kHz: it rounds to its tuning step) and the new band's
    // LNA range has come (0.8 s at most); unconfirmed after 1.5 s, carried
    // on regardless.  A centre a little off (rounded) is corrected in the
    // samples, so a station shows at its true frequency.  The antenna for
    // the frequency is chosen by the rules (automatic antenna).
    bool setFrequency(double hz);
    double frequency() const;          // asked for (0 if never set)
    double reportedFrequency() const;  // device_center_frequency
    // The sample rate the IQ callback gets.  SDRconnect may keep another
    // (an RSPdx kept 2.048 MHz when asked for 1.024): the samples are then
    // resampled to the rate asked for (linear, with a [1,2,1]/4 low-pass
    // when decimating), as the JS's generator does.
    bool setSampleRate(double hz);
    double sampleRate() const;        // what the callback gets
    double deviceSampleRate() const;  // what SDRconnect reports

    // ---- gain (the LNA state; the IF gain is left to SDRconnect's AGC) -----
    // Auto: the middle LNA state of the band's range (sdrcGain).
    bool setGainAuto();
    // 0..1, higher = more gain: the first 90% covers the safe 70% of the
    // range, the last 10% the rest to full gain (sdrcLnaFor).  The app's
    // slider of 0..49.6 dB maps as dB / 49.6.
    bool setGainFraction(double fraction);
    // A raw LNA state (0 = the most gain), kept within the band's range.
    bool setLnaState(int state);
    bool lnaRange(int& minState, int& maxState) const;
    int lnaState() const;  // -1 if unknown
    static int lnaForFraction(double fraction, int lo, int hi);

    // ---- antennas ----------------------------------------------------------
    // Every antenna SDRconnect has listed for this device (it lists only those
    // usable at the present frequency), plus an RSPdx's Antenna C, sorted
    // (sdrcAntList).
    std::vector<std::string> antennas();
    // Above 30 MHz, tunes to 1 MHz for a moment so SDRconnect lists them
    // all, then back (sdrcAntProbe).  Blocking (up to about 5 s).
    bool probeAntennas();
    // Antennas remembered from an earlier session (to keep in the app's settings).
    void rememberAntennas(const std::vector<std::string>& seen);
    std::string activeAntenna() const;
    bool setAntenna(const std::string& name);  // also turns the automatic choice off
    void setAutoAntenna(bool on);              // on by default
    AntennaRules antennaRules();               // defaults made on first use
    void setAntennaRules(const AntennaRules& rules);
    std::string antennaFor(double hz);         // "" when the device has one antenna
    // Antenna C of an RSPdx works up to 200 MHz; others: no limit.
    static double antennaLimitHz(const std::string& device, const std::string& antenna);

    // ---- Lite mode tuning (sdrcLiteSync) -----------------------------------
    // SDRconnect's demodulator ("AM", "SAM", "USB", "LSB", "CW", "NFM",
    // "WFM"), its VFO (the dial less the mode's shift) and filter width.
    // Sent only when changed; the VFO checked each second and sent again if
    // SDRconnect's is more than 50 Hz off.
    bool setLiteTuning(const std::string& demodulator, double vfoHz, double bandwidthHz);
    // OFFgrid-SDR's mode name -> SDRconnect's demodulator (SDRC_DEMOD; "USB"
    // for the data modes).
    static std::string liteDemodulator(const std::string& offgridMode);
    // false for DAB, DRM, AIS, VDL2: they need the full stream.
    static bool liteSupports(const std::string& offgridMode);
    LiteStats liteStats() const;
    // Level of a spectrum byte, in dB (spectrum_base .. spectrum_ref_level).
    float spectrumDb(std::uint8_t value) const;

    // ---- profiles ----------------------------------------------------------
    // An SDRconnect device profile (for example with the bias-tee on, which
    // the API can't switch) applied on every startDevice.
    void setStartProfile(const std::string& name);
    // Applied now; then the tuning, gain, antenna and rate sent again so the
    // profile can't undo them.
    bool applyProfile(const std::string& name);

    // ---- the speed test (sdrcSpeedTest) ------------------------------------
    // With no stream running: the full IQ stream taken at rate (0 = the rate
    // set, else 2.048 MHz) for half a second plus measureMs, the first half
    // second not counted; the RSP started for it and stopped again if it
    // wasn't running.  Blocking (about measureMs + 1 s).
    SpeedResult speedTest(double rate = 0, int measureMs = 3000);
    static SpeedResult judgeSpeed(double gotMBs, double rate);

    // ---- raw API -----------------------------------------------------------
    bool sendEvent(const std::string& eventType, const std::string& property, const std::string& value);
    bool requestProperty(const std::string& name);
    bool setProperty(const std::string& name, const std::string& value);

    LinkStats stats() const;

    // ---- conversions (Addition to the JS) ----------------------------------
    // Complex float (+-1) to the stick's unsigned 8-bit I, Q (127.5 = zero),
    // 2n bytes, for code written for an RTL-SDR's samples.
    static void toU8(const std::complex<float>* iq, std::size_t n, std::uint8_t* out);
    // SDRconnect's raw int16 IQ payload (after the 2-byte type) to complex
    // float: n = bytes / 4 samples.
    static std::size_t s16ToComplex(const unsigned char* bytes, std::size_t len, std::complex<float>* out);

private:
    Client(const Client&);
    Client& operator=(const Client&);
    struct Impl;
    std::unique_ptr<Impl> d_;
};

// Small JSON helpers (exposed for the tests' mock server).  The API's
// messages are flat objects; values come back as text (numbers and
// booleans as written; nested values skipped).
namespace json {
bool parseFlat(const std::string& text, std::map<std::string, std::string>& out);
std::string quote(const std::string& s);
std::string event(const std::string& eventType, const std::string& property, const std::string& value);
}  // namespace json

}  // namespace sdrc

#endif
