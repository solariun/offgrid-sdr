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

#include <complex>
#include <map>
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

#include "../decoders/Ais.h"
#include "../decoders/AisEncoders.h"
#include "../decoders/AviationEncoders.h"
#include "../decoders/DabEncoders.h"
#include "../decoders/DrmEncoders.h"
#include "../decoders/Ax25Encoders.h"
#include "../decoders/WsprEncoders.h"
#include "../decoders/FskEncoders.h"
#include "../decoders/Ftx.h"
#include "../decoders/PictureEncoders.h"
#include "../decoders/TimeEncoders.h"
#include "../decoders/PskEncoders.h"
#include "../dsp/Receiver.h"
#include "../rtlsdr/RtlSdr.h"
#include "../sdrconnect/SdrConnect.h"

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

// ---------------------------------------------------------- SdrConnectSource --

// An SDRplay RSP through SDRconnect (liboffgrid-sdrconnect): the full IQ
// stream, converted to the u8 samples every other source gives.
class SdrConnectSource : public Source {
public:
    explicit SdrConnectSource(const std::string& address) : address_(address), rate_(2048000), center_(100e6) {
        client_.setAutoReconnect(true);
    }
    ~SdrConnectSource() { stop(); }
    bool start(std::string& error) {
        if (!client_.connect(address_, error)) {
            error = "SDRconnect: " + error + " (is SDRconnect running there with its WebSocket API on?)";
            return false;
        }
        client_.setSampleRate(rate_);
        client_.setFrequency(center_);
        if (!client_.startDevice(error)) return false;
        SampleQueue* q = &queue_;
        std::vector<std::uint8_t>* buf = &u8_;
        return client_.startIq([q, buf](const std::complex<float>* iq, std::size_t n) {
            buf->resize(n * 2);
            sdrc::Client::toU8(iq, n, &(*buf)[0]);
            q->push(&(*buf)[0], buf->size());
        }, error);
    }
    void stop() {
        client_.stop();
        client_.disconnect();
    }
    std::string name() const { return "SDRconnect " + (client_.device().empty() ? address_ : client_.device()); }
    double sampleRate() const { return rate_; }
    bool setSampleRate(double hz) {
        rate_ = hz;
        queue_.clear();
        return client_.setSampleRate(hz);
    }
    double centerFrequency() const { return center_; }
    bool setCenterFrequency(double hz) {
        center_ = hz;
        queue_.clear();
        return client_.setFrequency(hz);
    }
    bool setGain(double dB) { return dB < 0 ? client_.setGainAuto() : client_.setGainFraction(dB / 49.6); }
    std::vector<double> gains() const {
        std::vector<double> g;
        for (int i = 0; i <= 31; ++i) g.push_back(i * 49.6 / 31);
        return g;
    }

private:
    std::string address_;
    sdrc::Client client_;
    double rate_, center_;
    std::vector<std::uint8_t> u8_;  // conversion buffer (client thread only)
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
    // Phase 4. ACARS on 131.550 MHz (AM, bursts under a carrier gate).
    {
        std::vector<float> audio, gate;
        dec::acarsDemoLoop(r, audio, gate);
        std::vector<float> env(audio.size());
        for (size_t i = 0; i < env.size(); ++i) env[i] = (i < gate.size() ? gate[i] : 0.0f) * (1.0f + 0.8f * audio[i]);
        demo.addAudioStation(131550000, env, r, -36, dsp::Mode::AM, "ACARS");
    }
    // VDL2 bursts on 136.975, 136.675 and 136.775 MHz (30 s loop).
    {
        const double br = 105000;  // 10 samples a symbol
        std::vector<dec::Vdl2DemoBurst> plan = dec::vdl2DemoBursts();
        std::map<double, std::vector<dsp::DemoSource::Burst> > byChannel;
        for (size_t i = 0; i < plan.size(); ++i) {
            dsp::DemoSource::Burst b;
            b.at = plan[i].at;
            std::vector<std::complex<float> > iq;
            dec::vdl2Modulate(dec::vdl2BurstSymbols(plan[i].frames), br, 0, 1.0, iq);
            b.iq.assign(iq.begin(), iq.end());
            byChannel[plan[i].freqHz].push_back(b);
        }
        for (std::map<double, std::vector<dsp::DemoSource::Burst> >::iterator c = byChannel.begin(); c != byChannel.end(); ++c)
            demo.addBurstStation(c->first, c->second, br, 30, -36, "VDL2");
    }
    // AIS on both channels (161.975 / 162.025 MHz): the Liverpool Bay
    // cycle, 30 s, as GMSK bursts.
    {
        const double br = 96000;
        std::vector<dec::AisDemoBurst> plan = dec::aisDemoPlan();
        std::vector<dsp::DemoSource::Burst> a, b;
        for (size_t i = 0; i < plan.size(); ++i) {
            std::vector<std::complex<float> > iq(static_cast<size_t>(plan[i].burst.seconds() * br) + 2);
            dec::aisAddBurst(&iq[0], iq.size(), br, 0.0, 0.0, 1.0, plan[i].burst);
            dsp::DemoSource::Burst burst;
            burst.at = plan[i].t;
            burst.iq.assign(iq.begin(), iq.end());
            (plan[i].channel == 'A' ? a : b).push_back(burst);
        }
        demo.addBurstStation(dec::kAisChannelA, a, br, 30, -40, "AIS A");
        demo.addBurstStation(dec::kAisChannelB, b, br, 30, -40, "AIS B");
    }
    // WSPR on the 20 m dial (14.0956 MHz): the demo stations every even
    // minute, locked to the clock (the 4-minute buffer starts on an even
    // minute, so its loop stays in step with the slots).
    {
        timeval tv;
        gettimeofday(&tv, 0);
        const double t0 = std::floor(tv.tv_sec / 120.0) * 120.0, wr = 12000;
        std::vector<float> air(static_cast<size_t>(240 * wr), 0.0f);
        std::vector<dec::enc::WsprStation> st = dec::enc::demoWsprStations();
        for (size_t k = 0; k < st.size(); ++k) {
            std::vector<float> tx = dec::enc::wsprTransmission(st[k], wr);
            // Transmissions start 1 s into the slot.
            for (int slot = 0; slot < 2; ++slot) {
                const size_t at = static_cast<size_t>((slot * 120 + 1) * wr);
                for (size_t i = 0; i < tx.size() && at + i < air.size(); ++i) air[at + i] += tx[i];
            }
        }
        demo.addAudioStation(14095600, air, wr, -50, dsp::Mode::USB, "WSPR", t0);
    }
    // FT8 on the 20 m dial (14.074 MHz) and FT4 (14.080 MHz): a QSO or two
    // and some CQs, slot by slot, locked to the clock (the loops start on a
    // whole minute, so they stay in step with the slots).
    {
        timeval tv;
        gettimeofday(&tv, 0);
        const double t0 = std::floor(tv.tv_sec / 60.0) * 60.0, fr = 12000;
        struct Tx {
            int slot;
            const char* text;
            double f0, amp;
        };
        const Tx ft8[] = {
            {0, "CQ G4OGR IO91", 820, 0.5},  {0, "CQ DL1OGS JO62", 1410, 0.25}, {0, "CQ EA8OGR IL18", 2150, 0.1},
            {1, "G4OGR K1OGS FN42", 860, 0.15}, {1, "CQ VK2OGS QF56", 1720, 0.06},
            {2, "K1OGS G4OGR -12", 820, 0.5},  {2, "DL1OGS PA3OGR JO21", 1410, 0.2},
            {3, "G4OGR K1OGS R-08", 860, 0.15}, {3, "PA3OGR DL1OGS -05", 1450, 0.25}, {3, "CQ JA1OGR PM95", 2400, 0.04},
        };
        std::vector<float> air(static_cast<size_t>(60 * fr), 0.0f);
        for (size_t i = 0; i < sizeof(ft8) / sizeof(ft8[0]); ++i) {
            std::vector<float> tx;
            if (!dec::ftx::synthesizeFt8(ft8[i].text, ft8[i].f0, fr, ft8[i].amp, tx)) continue;
            const size_t at = static_cast<size_t>((ft8[i].slot * 15 + 0.5) * fr);  // 0.5 s into the slot
            for (size_t k = 0; k < tx.size() && at + k < air.size(); ++k) air[at + k] += tx[k];
        }
        demo.addAudioStation(14074000, air, fr, -46, dsp::Mode::USB, "FT8", t0);

        const Tx ft4[] = {
            {0, "CQ TEST G4OGR IO91", 900, 0.4}, {1, "G4OGR SP9OGS 599 0012", 1300, 0.2},
            {2, "SP9OGS G4OGR R 599 0451", 900, 0.4}, {3, "CQ TEST OH2OGR KP20", 1900, 0.15},
        };
        std::vector<float> air4(static_cast<size_t>(30 * fr), 0.0f);
        for (size_t i = 0; i < sizeof(ft4) / sizeof(ft4[0]); ++i) {
            std::vector<float> tx;
            if (!dec::ftx::synthesizeFt4(ft4[i].text, ft4[i].f0, fr, ft4[i].amp, tx)) continue;
            const size_t at = static_cast<size_t>((ft4[i].slot * 7.5 + 0.5) * fr);
            for (size_t k = 0; k < tx.size() && at + k < air4.size(); ++k) air4[at + k] += tx[k];
        }
        demo.addAudioStation(14080000, air4, fr, -46, dsp::Mode::USB, "FT4", t0);
    }
    // DRM on 25 m (11.700 MHz "OFFgrid DRM", AAC with text, a slide and
    // Journaline; 11.750 MHz "OFFgrid xHE", xHE-AAC: no sound yet): the
    // browser version's demo loops, 48 kS/s IQ with DRM's k=0 at 0 Hz.
    for (int which = 0; which < 2; ++which) {
        std::vector<std::complex<float> > iq;
        if (!dec::drmSignal(dec::drmDemoStation(which), iq) || iq.empty()) continue;
        dsp::DemoSource::Burst b;
        b.at = 0;
        b.iq.assign(iq.begin(), iq.end());
        demo.addBurstStation(dec::drmDemoStationHz(which), std::vector<dsp::DemoSource::Burst>(1, b), 48000,
                             iq.size() / 48000.0, -30, which ? "DRM xHE" : "DRM");
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
    DemoSourceImpl(double rate, double center) : PacedSource(rate, center), demo_(rate, center), stickHz_(center), dabPhase_(0) {}
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
        addDab(samples, bytes);
    }
    void centerChanged(double hz) {
        std::lock_guard<std::mutex> lock(mutex_);
        demo_.setCenter(hz);
        stickHz_ = hz;
    }

private:
    // DAB: the browser version's demo multiplex ("OFFgrid DAB": a DAB+ and
    // an MP2 station, text, slides, logos) on 12A, 223.936 MHz.  Made only
    // while the stick is near it (it is 1.5 MHz wide, made on the fly at
    // 2.048 MS/s, about 2 % of a core), mixed in at the stick's offset.
    static double dabHz() { return 223936e3; }
    void addDab(std::size_t samples, std::vector<std::uint8_t>& bytes) {
        const double off = dabHz() - stickHz_;
        if (sampleRate() != dec::DabDemoMultiplex::sampleRate() || std::fabs(off) > 300e3) {
            dabPhase_ = 0;
            return;
        }
        if (!dab_) dab_.reset(new dec::DabDemoMultiplex());
        dabBuf_.resize(samples);
        dab_->generate(&dabBuf_[0], samples);
        const double step = 2 * 3.14159265358979 * off / sampleRate(), amp = 0.12 * 127.5;
        for (std::size_t i = 0; i < samples && 2 * i + 1 < bytes.size(); ++i) {
            const std::complex<float> v = dabBuf_[i] * std::complex<float>(static_cast<float>(std::cos(dabPhase_)),
                                                                            static_cast<float>(std::sin(dabPhase_)));
            dabPhase_ += step;
            if (dabPhase_ > 3.14159265358979) dabPhase_ -= 2 * 3.14159265358979;
            if (dabPhase_ < -3.14159265358979) dabPhase_ += 2 * 3.14159265358979;
            const double re = bytes[2 * i] + amp * v.real(), im = bytes[2 * i + 1] + amp * v.imag();
            bytes[2 * i] = static_cast<std::uint8_t>(std::max(0.0, std::min(255.0, re + 0.5)));
            bytes[2 * i + 1] = static_cast<std::uint8_t>(std::max(0.0, std::min(255.0, im + 0.5)));
        }
    }

    std::mutex mutex_;
    dsp::DemoSource demo_;
    double stickHz_;
    std::unique_ptr<dec::DabDemoMultiplex> dab_;
    std::vector<std::complex<float> > dabBuf_;
    double dabPhase_;
};

}  // namespace

std::unique_ptr<Source> makeRtlSource(int index) { return std::unique_ptr<Source>(new RtlSource(index)); }
std::unique_ptr<Source> makeTcpSource(const std::string& hostPort) { return std::unique_ptr<Source>(new TcpSource(hostPort)); }
std::unique_ptr<Source> makeSdrConnectSource(const std::string& hostPort) {
    return std::unique_ptr<Source>(new SdrConnectSource(hostPort));
}
std::unique_ptr<Source> makeFileSource(const std::string& path, double rate, double center) {
    return std::unique_ptr<Source>(new FileSource(path, rate, center));
}
std::unique_ptr<Source> makeDemoSource(double rate, double center) {
    return std::unique_ptr<Source>(new DemoSourceImpl(rate, center));
}

}  // namespace app
