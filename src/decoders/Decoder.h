// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/Decoder.h
//  Layer:  Library liboffgrid-decoders (public header)
//
//  What this file does
//  -------------------
//  The one interface every decoder implements, so a program can drive any
//  of them the same way:
//
//      std::unique_ptr<dec::Decoder> d = dec::makeDecoder(dec::Kind::Rtty);
//      for (;;) {
//          d->process(audio, n, 48000);      // mono demodulated audio, any rate
//          std::string text;
//          d->takeText(text);                 // new characters since last time
//          std::fputs(text.c_str(), stdout);
//          dec::Status s = d->status();       // locked? tones, speed, quality
//      }
//
//  Each decoder also has its own header (Cw.h, Rtty.h, Navtex.h (with DSC),
//  Psk.h, Hell.h) with its own settings; a program that needs one decoder
//  only includes that header.
//
//  Audio conventions (what the radio must deliver)
//  -----------------------------------------------
//  Mono float samples in about -1..1 at any rate from 8 kHz up, demodulated
//  in SSB (USB) or, for CW, with the beat note: the decoders look for their
//  tones in the audio (see audioCenterHz() and each header for where):
//    CW      the tone anywhere 250-1500 Hz (found by the decoder; the TUI
//            receiver's CW pitch is 700 Hz, the browser version's 600 Hz)
//    RTTY    a tone pair 170/200/425/450/850 Hz apart in 250-3000 Hz;
//            OFFgrid-SDR centres it on 1500 Hz (USB dial 1.5 kHz below the
//            listed frequency): 170 Hz shift = 1415/1585 Hz, mark the higher
//    NAVTEX  a 170 Hz pair around 1500 Hz, 1415/1585 Hz (dial 1.5 kHz below
//            518/490/4209.5 kHz)
//    DSC     the same signal as NAVTEX: 1415/1585 Hz, dial 1.5 kHz below the
//            DSC frequency (a 1 bit, "B", is the lower tone)
//    PSK     the tone at 1000 Hz ±50 Hz (dial 1 kHz below the PSK frequency)
//    HELL    the tone at 1000 Hz
//    FAX     1500 Hz (black) .. 2300 Hz (white), centre 1900 (USB dial 1.9 kHz
//            below the listed frequency; start/stop tones swing at 300/450 Hz)
//    SSTV    1500 .. 2300 Hz video, 1200 Hz sync, VIS 1100/1300 Hz (the dial ON
//            the listed frequency: USB, but LSB on the 40/80/160 m ham bands
//            (7.171, 3.845 MHz), narrowband FM above 30 MHz (ISS 145.800))
//    ACARS   AM audio (2400 bd MSK, 1200/2400 Hz) on 129-137 MHz channels
//    VDL2    IQ: every 25 kHz channel 136.6-137 MHz in view (processIq)
//    AIS     IQ: both channels, 161.975 and 162.025 MHz (processIq)
//    FT8/FT4 USB audio 200-3000 Hz, dial on the listed FT8/FT4 frequency
//    WSPR    USB audio 1400-1600 Hz, dial on the listed WSPR dial frequency
//    DAB     IQ at exactly 2.048 MS/s, the stick on the channel's centre (Dab.h)
//    DRM     IQ (processIq), 48 kS/s up: the dial on the station's frequency (Drm.h)
//    TIME    the carrier at 1000 Hz, found anywhere 700-1300 Hz (dial 1 kHz
//            below the station; CHU's code needs audio to ~3.3 kHz; Time.h)
//    AX25    packet/APRS: 1200 Bd AFSK 1200/2200 Hz in NFM audio, dial on the
//            channel; 300 Bd 1600/1800 Hz in USB, dial on the listed dial
//            frequency; 9600 Bd G3RUH needs flat FM discriminator audio (Ax25.h)
//
//  How it fits in
//  --------------
//  The library depends only on the C++11 standard library and
//  liboffgrid-dsp's FFT (src/dsp/Fft.h).  It never touches audio devices,
//  threads or the terminal: call process() from any one thread.
// ============================================================================
#ifndef OFFGRID_DECODERS_DECODER_H
#define OFFGRID_DECODERS_DECODER_H

#include <complex>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace dec {

enum class Kind { Cw, Rtty, Psk, Navtex, Dsc, Hell, Fax, Sstv, Time, Ax25, Acars, Vdl2, Ais, Ft8, Ft4, Wspr, Dab, Drm };
const char* kindName(Kind kind);  // "CW", "RTTY", "PSK", "NAVTEX", "DSC", "HELL", "FAX", "SSTV", "TIME"
bool parseKind(const std::string& name, Kind& kind);  // case-insensitive

// What a decoder reports about the signal it is following.
struct Status {
    Status()
        : locked(false), signal(false), snrDb(0), quality(0), toneHz(0), shiftHz(0), baud(0), wpm(0), characters(0),
          pictureWidth(0), pictureHeight(0), pictureRow(0), timeValid(false), clockOffset(0), clockError(0), stationTime(0) {}
    bool locked;          // decoding (in sync with a signal)
    bool signal;          // a signal of the right kind is present
    double snrDb;         // signal to noise, dB (0 when unknown)
    double quality;       // 0..1: how clean the decoding is
    double toneHz;        // the tone followed (CW, PSK, HELL) or the mark tone (FSK)
    double shiftHz;       // FSK: mark - space (Hz)
    double baud;          // symbols per second (RTTY, NAVTEX, DSC, PSK)
    double wpm;           // CW words per minute
    unsigned characters;  // characters decoded since reset()
    std::string mode;     // detail, e.g. "PSK63", "RTTY 45.45/170", "NAVTEX"
    std::string info;     // one line for a status bar, e.g. "24 WPM  SNR 18 dB"

    // Picture decoders (FAX, SSTV): the picture being received.
    int pictureWidth;     // pixels per line (0 = no picture yet)
    int pictureHeight;    // lines expected (0 = open-ended, e.g. a fax chart)
    int pictureRow;       // lines received so far of the current picture

    // Time decoder: how far the computer's clock is from the station's.
    bool timeValid;       // a full time code has been read
    double clockOffset;   // station time - computer time, seconds
    double clockError;    // its uncertainty, seconds (e.g. 0.005)
    double stationTime;   // the station's time now, Unix seconds (UTC)
    std::string station;  // "CHU", "WWV", "WWVH", "RWM"
};

// A station / programme a multi-service decoder offers (DAB services in an
// ensemble, DRM's up to four services).
struct Service {
    Service() : id(0), audio(true), bitrateKbps(0), selected(false) {}
    unsigned id;            // service id (DAB SId, DRM service index)
    std::string label;      // e.g. "NPO Radio 1"
    std::string detail;     // e.g. "DAB+ 96 kbit/s HE-AAC v2 stereo", "Pop music"
    bool audio;             // an audio service (else data)
    int bitrateKbps;
    bool selected;
};

// One line of a picture (FAX, SSTV).  `picture` counts the pictures since
// reset() (a new number = a new picture; the program saves the old one);
// `row` is the line's index in it; `rgb` holds width x 3 bytes.
struct PictureLine {
    unsigned picture;
    int row;
    std::vector<unsigned char> rgb;
};

class Decoder {
public:
    virtual ~Decoder() {}

    virtual Kind kind() const = 0;
    // Forget everything (call after retuning).  Settings are kept.
    virtual void reset() = 0;
    // Feeds n mono samples at `rate` Hz (the rate may change between calls;
    // the decoder adapts).
    virtual void process(const float* audio, std::size_t n, double rate) = 0;
    // Appends the text decoded since the last call (UTF-8; '\n' starts a
    // line).  Complete messages (NAVTEX, DSC) arrive as formatted lines.
    virtual void takeText(std::string& out) = 0;
    virtual Status status() const = 0;
    // Where the decoder expects the signal in the audio, Hz (for tuning aids).
    virtual double audioCenterHz() const = 0;

    // Picture decoders (Hellschreiber): new columns of dots since the last
    // call, each a vector of brightness values 0..1 from top to bottom.
    virtual bool hasImage() const { return false; }
    virtual void takeColumns(std::vector<std::vector<float> >& columns) { (void)columns; }

    // Picture decoders (FAX, SSTV): the lines received since the last call.
    virtual bool hasPicture() const { return false; }
    virtual void takeLines(std::vector<PictureLine>& lines) { (void)lines; }

    // Audio decoders (DAB, DRM): the programme's sound, decoded, to be played
    // instead of the receiver's own demodulated audio.  takeAudio appends
    // interleaved stereo float frames at audioRate() (which may change).
    virtual bool hasAudio() const { return false; }
    virtual int audioRate() const { return 48000; }
    virtual void takeAudio(std::vector<float>& stereo) { (void)stereo; }

    // Multi-service decoders (DAB, DRM): the services on offer and which is
    // played.  selectService() takes a Service::id.
    virtual std::vector<Service> services() const { return std::vector<Service>(); }
    virtual bool selectService(unsigned id) {
        (void)id;
        return false;
    }

    // Still pictures that arrive whole (DAB/DRM slideshow, station logos):
    // encoded image files (JPEG / PNG) with a name, for the program to
    // decode and show.  takeImages appends them.
    struct EncodedImage {
        std::string name;                 // e.g. "slide 12", "logo"
        std::string mime;                 // "image/jpeg", "image/png"
        std::vector<unsigned char> bytes;
    };
    virtual void takeImages(std::vector<EncodedImage>& images) { (void)images; }

    // Time decoders: the computer's clock (Unix seconds, UTC) at the first
    // sample of the next process() call.  Programs call it before every
    // process(); other decoders ignore it.
    virtual void setTimestamp(double unixSeconds) { (void)unixSeconds; }
    // Slot-based decoders (FT8, FT4, WSPR): the clock correction in use,
    // seconds (PC time - true time, the sign of dec::ClockCorrection), so
    // their slots follow the true time.
    virtual void setClockOffset(double pcMinusTrue) { (void)pcMinusTrue; }

    // IQ decoders (AIS, VDL2): they take the stick's complex samples instead
    // of audio (several channels at once).  Programs call processIq() for
    // these (process() is then not used): iq at `rate`, the stick's centre
    // frequency `centerHz`, and where the user is tuned, `dialHz`.
    virtual bool wantsIq() const { return false; }
    virtual void processIq(const std::complex<float>* iq, std::size_t n, double rate, double centerHz, double dialHz) {
        (void)iq;
        (void)n;
        (void)rate;
        (void)centerHz;
        (void)dialHz;
    }
};

// A decoder of the given kind with its default settings.
std::unique_ptr<Decoder> makeDecoder(Kind kind);
// All kinds, in menu order.
std::vector<Kind> allKinds();

}  // namespace dec

#endif
