// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/Source.cpp
//  Layer:  Application (where the IQ samples come from)
//
//  What this file does
//  -------------------
//  The four sources of Source.h and the SampleQueue.
//
//  Protocol notes
//  --------------
//  rtl_tcp: the server first sends 12 bytes ("RTL0", tuner type and gain
//  count, big-endian), then raw u8 IQ forever.  Commands are 5 bytes: a
//  command byte and a big-endian 32-bit value: 0x01 frequency (Hz), 0x02
//  sample rate, 0x03 gain mode (0 auto, 1 manual), 0x04 gain (tenths of a
//  dB), 0x05 frequency correction (ppm), 0x09 direct sampling, 0x0e
//  bias-tee.
//
//  Contributing
//  ------------
//  A source thread must never block on the DSP: it pushes into the queue
//  (which drops the oldest block when full) and goes on.
// ============================================================================
#include "Source.h"

#include <netdb.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "../decoders/Ax25Encoders.h"
#include "../decoders/FskEncoders.h"
#include "../decoders/PictureEncoders.h"
#include "../decoders/TimeEncoders.h"
#include "../decoders/PskEncoders.h"
#include "../dsp/Receiver.h"
#include "../rtlsdr/RtlSdr.h"

namespace app {

// ------------------------------------------------------------- SampleQueue --

SampleQueue::SampleQueue(std::size_t maxBlocks) : maxBlocks_(maxBlocks), dropped_(0) {}

void SampleQueue::push(const std::uint8_t* data, std::size_t length) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::uint8_t> block;
    if (!spare_.empty()) {
        block.swap(spare_.back());
        spare_.pop_back();
    }
    block.assign(data, data + length);
    if (blocks_.size() >= maxBlocks_) {
        spare_.push_back(std::vector<std::uint8_t>());
        spare_.back().swap(blocks_.front());
        blocks_.pop_front();
        times_.pop_front();
        ++dropped_;
    }
    blocks_.push_back(std::vector<std::uint8_t>());
    blocks_.back().swap(block);
    timeval tv;
    gettimeofday(&tv, 0);
    times_.push_back(tv.tv_sec + tv.tv_usec * 1e-6);
    ready_.notify_one();
}

bool SampleQueue::pop(std::vector<std::uint8_t>& block, int timeoutMs, double* arrived) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (blocks_.empty()) ready_.wait_for(lock, std::chrono::milliseconds(timeoutMs));
    if (blocks_.empty()) return false;
    if (spare_.size() < 8) {  // give the caller's old buffer back for reuse
        spare_.push_back(std::vector<std::uint8_t>());
        spare_.back().swap(block);
    }
    block.swap(blocks_.front());
    blocks_.pop_front();
    if (arrived) *arrived = times_.front();
    times_.pop_front();
    return true;
}

void SampleQueue::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    blocks_.clear();
    times_.clear();
}

std::size_t SampleQueue::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return blocks_.size();
}

namespace {

// --------------------------------------------------------------- RtlSource --

class RtlSource : public Source {
public:
    explicit RtlSource(int index) : index_(index), rate_(2048000), center_(100e6) {}
    ~RtlSource() { stop(); }

    bool start(std::string& error) {
        std::vector<usb::DeviceInfo> sticks = rtlsdr::RtlSdr::find();
        if (sticks.empty()) {
            error = "no RTL-SDR stick found (plug one in, or use --demo)";
            return false;
        }
        if (index_ < 0 || index_ >= static_cast<int>(sticks.size())) {
            error = "no stick number " + std::to_string(index_) + " (" + std::to_string(sticks.size()) + " found; see --list)";
            return false;
        }
        info_ = sticks[static_cast<size_t>(index_)];
        if (!rtl_.open(info_)) {
            error = rtl_.error();
            return false;
        }
        rtl_.setSampleRate(static_cast<std::uint32_t>(rate_));
        rtl_.setCenterFrequency(static_cast<std::uint64_t>(center_));
        rate_ = rtl_.sampleRate();
        SampleQueue* q = &queue_;
        if (!rtl_.start([q](const std::uint8_t* data, std::size_t n) { q->push(data, n); }, 32768, 16)) {
            error = rtl_.error();
            rtl_.close();
            return false;
        }
        return true;
    }

    void stop() {
        if (!rtl_.isOpen()) return;
        rtl_.stop();
        rtl_.close();
    }

    std::string name() const {
        std::string n = info_.product.empty() ? "RTL-SDR" : info_.product;
        return n + " (" + rtl_.tunerName() + (rtl_.isBlogV4() ? ", Blog V4" : "") + ")";
    }
    double sampleRate() const { return rate_; }
    bool setSampleRate(double hz) {
        rate_ = hz;
        if (!rtl_.isOpen()) return true;
        bool ok = rtl_.setSampleRate(static_cast<std::uint32_t>(hz));
        rate_ = rtl_.sampleRate();
        queue_.clear();
        return ok;
    }
    double centerFrequency() const { return rtl_.isOpen() ? static_cast<double>(rtl_.centerFrequency()) : center_; }
    bool setCenterFrequency(double hz) {
        center_ = hz;
        if (!rtl_.isOpen()) return true;
        bool ok = rtl_.setCenterFrequency(static_cast<std::uint64_t>(hz + 0.5));
        queue_.clear();  // samples from the old frequency are useless now
        return ok;
    }
    bool setGain(double dB) { return dB < 0 ? rtl_.setAutoGain() : rtl_.setGain(dB); }
    std::vector<double> gains() const { return rtl_.gains(); }
    bool setPpm(double ppm) { return rtl_.setFrequencyCorrection(ppm); }
    bool setBiasTee(bool on) { return rtl_.setBiasTee(on); }
    bool setDirectSampling(int mode) { return rtl_.setDirectSampling(mode); }
    std::string lastError() const { return rtl_.error(); }

private:
    int index_;
    usb::DeviceInfo info_;
    rtlsdr::RtlSdr rtl_;
    double rate_, center_;
};

// --------------------------------------------------------------- TcpSource --

class TcpSource : public Source {
public:
    explicit TcpSource(const std::string& hostPort) : hostPort_(hostPort), fd_(-1), running_(false), rate_(2048000), center_(100e6), gains_(0) {}
    ~TcpSource() { stop(); }

    bool start(std::string& error) {
        std::string host = hostPort_, port = "1234";
        size_t colon = hostPort_.rfind(':');
        if (colon != std::string::npos) {
            host = hostPort_.substr(0, colon);
            port = hostPort_.substr(colon + 1);
        }
        addrinfo hints;
        std::memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo* res = 0;
        if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res) != 0 || !res) {
            error = "rtl_tcp: cannot resolve " + hostPort_;
            return false;
        }
        for (addrinfo* a = res; a; a = a->ai_next) {
            fd_ = socket(a->ai_family, a->ai_socktype, a->ai_protocol);
            if (fd_ < 0) continue;
            if (connect(fd_, a->ai_addr, a->ai_addrlen) == 0) break;
            ::close(fd_);
            fd_ = -1;
        }
        freeaddrinfo(res);
        if (fd_ < 0) {
            error = "rtl_tcp: cannot connect to " + hostPort_;
            return false;
        }
        int one = 1;
        setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        unsigned char header[12];
        if (!readAll(header, 12) || std::memcmp(header, "RTL0", 4) != 0) {
            error = "rtl_tcp: " + hostPort_ + " is not an rtl_tcp server";
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        tuner_ = (header[4] << 24) | (header[5] << 16) | (header[6] << 8) | header[7];
        gains_ = (header[8] << 24) | (header[9] << 16) | (header[10] << 8) | header[11];
        command(0x02, static_cast<std::uint32_t>(rate_));
        command(0x01, static_cast<std::uint32_t>(center_));
        running_ = true;
        thread_ = std::thread(&TcpSource::loop, this);
        return true;
    }

    void stop() {
        running_ = false;
        if (fd_ >= 0) shutdown(fd_, SHUT_RDWR);
        if (thread_.joinable()) thread_.join();
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }

    std::string name() const { return "rtl_tcp " + hostPort_; }
    double sampleRate() const { return rate_; }
    bool setSampleRate(double hz) {
        rate_ = hz;
        queue_.clear();
        return fd_ < 0 || command(0x02, static_cast<std::uint32_t>(hz));
    }
    double centerFrequency() const { return center_; }
    bool setCenterFrequency(double hz) {
        center_ = hz;
        queue_.clear();
        return fd_ < 0 || command(0x01, static_cast<std::uint32_t>(hz + 0.5));
    }
    bool setGain(double dB) {
        if (dB < 0) return command(0x03, 0);
        return command(0x03, 1) && command(0x04, static_cast<std::uint32_t>(dB * 10 + 0.5));
    }
    std::vector<double> gains() const {
        // The R820T steps (rtl_tcp does not send them).
        static const double steps[] = {0, 0.9, 1.4, 2.7, 3.7, 7.7, 8.7, 12.5, 14.4, 15.7, 16.6, 19.7, 20.7, 22.9, 25.4,
                                       28.0, 29.7, 32.8, 33.8, 36.4, 37.2, 38.6, 40.2, 42.1, 43.4, 43.9, 44.5, 48.0, 49.6};
        return std::vector<double>(steps, steps + 29);
    }
    bool setPpm(double ppm) { return command(0x05, static_cast<std::uint32_t>(static_cast<std::int32_t>(std::floor(ppm + 0.5)))); }
    bool setBiasTee(bool on) { return command(0x0e, on ? 1 : 0); }
    bool setDirectSampling(int mode) { return command(0x09, static_cast<std::uint32_t>(mode)); }

private:
    bool readAll(unsigned char* p, size_t n) {
        while (n > 0) {
            ssize_t r = recv(fd_, p, n, 0);
            if (r <= 0) return false;
            p += r;
            n -= static_cast<size_t>(r);
        }
        return true;
    }
    bool command(unsigned char cmd, std::uint32_t value) {
        if (fd_ < 0) return false;
        unsigned char b[5] = {cmd, static_cast<unsigned char>(value >> 24), static_cast<unsigned char>(value >> 16),
                              static_cast<unsigned char>(value >> 8), static_cast<unsigned char>(value)};
        std::lock_guard<std::mutex> lock(sendMutex_);
        return send(fd_, b, 5, 0) == 5;
    }
    void loop() {
        std::vector<unsigned char> buf(32768);
        size_t have = 0;
        while (running_) {
            ssize_t r = recv(fd_, &buf[have], buf.size() - have, 0);
            if (r <= 0) break;
            have += static_cast<size_t>(r);
            if (have == buf.size()) {
                queue_.push(&buf[0], have);
                have = 0;
            }
        }
        running_ = false;
    }

    std::string hostPort_;
    int fd_;
    std::atomic<bool> running_;
    std::thread thread_;
    std::mutex sendMutex_;
    double rate_, center_;
    unsigned tuner_ = 0;
    unsigned gains_;
};

// ---------------------------------------------- paced sources (file, demo) --

// Base for sources that make their own samples: a thread that delivers
// rate * 2 bytes per second of wall-clock time, in ~8 ms blocks.
class PacedSource : public Source {
public:
    PacedSource(double rate, double center) : rate_(rate), center_(center), running_(false) {}
    ~PacedSource() { stopThread(); }

    void stop() { stopThread(); }
    double sampleRate() const { return rate_; }
    bool setSampleRate(double hz) {
        rate_ = hz;
        queue_.clear();
        return true;
    }
    double centerFrequency() const { return center_; }
    bool setCenterFrequency(double hz) {
        center_ = hz;
        queue_.clear();
        centerChanged(hz);
        return true;
    }
    bool setGain(double) { return true; }
    std::vector<double> gains() const { return std::vector<double>(); }

protected:
    virtual void produce(std::size_t samples, std::vector<std::uint8_t>& bytes) = 0;
    virtual void centerChanged(double) {}

    bool startThread() {
        running_ = true;
        thread_ = std::thread(&PacedSource::loop, this);
        return true;
    }
    void stopThread() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    double rate_, center_;

private:
    void loop() {
        typedef std::chrono::steady_clock Clock;
        Clock::time_point begin = Clock::now();
        double sent = 0;  // samples delivered since begin
        std::vector<std::uint8_t> bytes;
        while (running_) {
            double elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
            double due = elapsed * rate_ - sent;
            if (due < 8192) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            if (due > rate_) {  // fell far behind (suspend, debugger): skip ahead
                sent = elapsed * rate_ - 16384;
                continue;
            }
            std::size_t n = 16384;
            produce(n, bytes);
            queue_.push(&bytes[0], bytes.size());
            sent += n;
        }
    }

    std::atomic<bool> running_;
    std::thread thread_;
};

class FileSource : public PacedSource {
public:
    FileSource(const std::string& path, double rate, double center) : PacedSource(rate, center), path_(path), file_(0) {}
    ~FileSource() {
        stopThread();
        if (file_) std::fclose(file_);
    }
    bool start(std::string& error) {
        file_ = std::fopen(path_.c_str(), "rb");
        if (!file_) {
            error = "cannot open " + path_;
            return false;
        }
        return startThread();
    }
    std::string name() const { return "file " + path_.substr(path_.find_last_of('/') == std::string::npos ? 0 : path_.find_last_of('/') + 1); }
    bool tunable() const { return false; }
    bool setCenterFrequency(double) { return false; }  // a recording stays where it was made

protected:
    void produce(std::size_t samples, std::vector<std::uint8_t>& bytes) {
        bytes.resize(samples * 2);
        size_t got = std::fread(&bytes[0], 1, bytes.size(), file_);
        if (got < bytes.size()) {  // loop
            std::rewind(file_);
            got += std::fread(&bytes[got], 1, bytes.size() - got, file_);
            if (got < bytes.size()) std::fill(bytes.begin() + static_cast<long>(got), bytes.end(), 127);
        }
    }

private:
    std::string path_;
    FILE* file_;
};

// The decoder stations of the demo band, where the browser version's demo
// puts them (signal frequencies; each is sent as USB audio with its tones
// at the usual audio centre, so the decoder modes find them on the dial).
void addDecoderStations(dsp::DemoSource& demo) {
    const double r = 12000;  // audio rate of the generated programmes
    using namespace dec::enc;
    // RTTY: weather-style 50 Bd / 450 Hz and amateur 45.45 Bd / 170 Hz.
    std::vector<float> a = rttyAudio("\nWEATHER BULLETIN OFFGRID MET 1200 UTC WIND SW 4 TO 5 SEA MODERATE VIS GOOD\n", 50, 450, 1500, r);
    append(a, silence(2, r));
    demo.addAudioStation(7355000 - 1500, a, r, -38, dsp::Mode::USB, "RTTY 50/450");
    a = rttyAudio("\nCQ CQ CQ DE OFFGRID OFFGRID PSE K\n", 45.45, 170, 1500, r);
    append(a, silence(3, r));
    demo.addAudioStation(7312000 - 1500, a, r, -40, dsp::Mode::USB, "RTTY 45/170");
    // CW 20 WPM (tone 700 Hz above the carrier).
    a = cwAudio("CQ CQ DE OFFGRID OFFGRID K", 20, 700, r, 0.5, 0.5, 3.0);
    demo.addAudioStation(7275000 - 700, a, r, -40, dsp::Mode::CW, "CW 20 WPM");
    // NAVTEX 518 kHz (real phasing length, so the decoder catches ZCZC).
    a = navtexAudio("ZCZC SA23\nNITON NAVTEX DEMO\nGALE WARNING SOLE FITZROY SOUTHWEST GALE 8 EXPECTED\nNNNN\n", r, 1500);
    append(a, silence(5, r));
    demo.addAudioStation(518000 - 1500, a, r, -36, dsp::Mode::USB, "NAVTEX 518");
    // DSC 8414.5 kHz: a safety call, then a distress call.
    std::vector<int> bits = dscBits(dscAllShipsFields(108, "232123456"));
    for (int i = 0; i < 300; ++i) bits.push_back(i & 1);  // dot pattern between calls
    std::vector<int> distress = dscBits(dscDistressFields("244567890", 105));
    bits.insert(bits.end(), distress.begin(), distress.end());
    a = dscAudio(bits, r, 1500);
    append(a, silence(6, r));
    demo.addAudioStation(8414500 - 1500, a, r, -38, dsp::Mode::USB, "DSC 8414.5");
    // PSK31, PSK63 and Feld Hell (tone at 1000 Hz).
    a.clear();
    dec::synthPsk("CQ CQ CQ de OFFGRID OFFGRID pse k\n", 31.25, r, a);
    append(a, silence(3, r));
    demo.addAudioStation(14071000 - 1000, a, r, -40, dsp::Mode::USB, "PSK31");
    a.clear();
    dec::synthPsk("OFFGRID PSK63 test: the quick brown fox jumps over the lazy dog\n", 62.5, r, a);
    append(a, silence(3, r));
    demo.addAudioStation(7042500 - 1000, a, r, -40, dsp::Mode::USB, "PSK63");
    a.clear();
    dec::synthHell("OFFGRID HELL 73", r, a);
    append(a, silence(2, r));
    demo.addAudioStation(14063500 - 1000, a, r, -38, dsp::Mode::USB, "HELL");

    // Phase 3. Weather fax on 7880 kHz (dial 1.9 kHz below): a test chart.
    {
        std::vector<unsigned char> grey;
        dec::testChartGrey(dec::faxWidth, 200, grey);
        a.clear();
        dec::synthFax(grey, dec::faxWidth, 200, r, a);
        append(a, silence(10, r));
        demo.addAudioStation(7880000 - 1900, a, r, -36, dsp::Mode::USB, "FAX 120 LPM");
    }
    // SSTV on 14.230 MHz (USB, dial on the frequency): Martin M1 test card.
    {
        std::vector<unsigned char> rgb;
        dec::testCardRgb(320, 256, rgb);
        a.clear();
        dec::synthSstv(44, rgb, r, a);
        append(a, silence(8, r));
        demo.addAudioStation(14230000, a, r, -38, dsp::Mode::USB, "SSTV Martin M1");
    }
    // APRS on 144.800 MHz (NFM, 1200 Bd): the browser demo's packets.
    {
        std::vector<dec::AprsSample> packets = dec::aprsDemoPackets();
        a.clear();
        for (size_t i = 0; i < packets.size(); ++i) {
            dec::synthAx25(dec::ax25UiFrame(packets[i].source, packets[i].destination, packets[i].path, packets[i].info), r, a);
            append(a, silence(1.5, r));
        }
        demo.addAudioStation(144800000, a, r, -40, dsp::Mode::NFM, "APRS");
    }
    // Time stations, locked to the computer's clock: their code starts at
    // `now` and every (re)build plays the sample for the present moment
    // (5 minutes, then the code repeats and no longer matches the clock).
    {
        timeval tv;
        gettimeofday(&tv, 0);
        const double now = tv.tv_sec + tv.tv_usec * 1e-6, tr = 8000;
        demo.addAudioStation(7850000 - 1000, dec::enc::timeSignalAudio(dec::TimeStation::Chu, now, 300, tr), tr, -34,
                             dsp::Mode::USB, "CHU", now);
        demo.addAudioStation(10000000 - 1000, dec::enc::timeSignalAudio(dec::TimeStation::Wwv, now, 300, tr), tr, -34,
                             dsp::Mode::USB, "WWV", now);
    }
}

class DemoSourceImpl : public PacedSource {
public:
    DemoSourceImpl(double rate, double center) : PacedSource(rate, center), demo_(rate, center) {}
    ~DemoSourceImpl() { stopThread(); }
    bool start(std::string&) {
        // Built now, just before the first sample: the time stations' code
        // then starts at the computer's present time.
        addDecoderStations(demo_);
        return startThread();
    }
    std::string name() const { return "demo (simulated stations)"; }
    bool setSampleRate(double) { return false; }  // the demo runs at one rate

protected:
    void produce(std::size_t samples, std::vector<std::uint8_t>& bytes) {
        std::lock_guard<std::mutex> lock(mutex_);
        demo_.generate(samples, bytes);
    }
    void centerChanged(double hz) {
        std::lock_guard<std::mutex> lock(mutex_);
        demo_.setCenter(hz);
    }

private:
    std::mutex mutex_;
    dsp::DemoSource demo_;
};

}  // namespace

std::unique_ptr<Source> makeRtlSource(int index) { return std::unique_ptr<Source>(new RtlSource(index)); }
std::unique_ptr<Source> makeTcpSource(const std::string& hostPort) { return std::unique_ptr<Source>(new TcpSource(hostPort)); }
std::unique_ptr<Source> makeFileSource(const std::string& path, double rate, double center) {
    return std::unique_ptr<Source>(new FileSource(path, rate, center));
}
std::unique_ptr<Source> makeDemoSource(double rate, double center) {
    return std::unique_ptr<Source>(new DemoSourceImpl(rate, center));
}

}  // namespace app
