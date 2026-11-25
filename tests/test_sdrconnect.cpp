// ============================================================================
//  offgrid-sdrconnect - an SDRplay RSP through SDRplay's SDRconnect WebSocket
//  API, in C++11 (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   tests/test_sdrconnect.cpp
//  Layer:  Tests (Library liboffgrid-sdrconnect)
//
//  What this file does
//  -------------------
//  Self-checking test of liboffgrid-sdrconnect.  Prints PASS/FAIL per check,
//  returns nonzero on any failure:
//    - SHA-1 (FIPS 180 / RFC 3174 vectors), base64 (RFC 4648 section 10),
//      the RFC 6455 handshake example (Sec-WebSocket-Accept);
//    - framing: the RFC 6455 section 5.7 examples byte for byte (masked and
//      unmasked, fragmented, ping, pong, 16- and 64-bit lengths), encode/
//      decode round trips fed whole and byte by byte, fragment assembly
//      with a control frame in between, protocol errors rejected;
//    - the JSON helpers, address parsing, the LNA mapping, the speed
//      verdicts, the antenna letters and limits;
//    - a MOCK SDRconnect: a small ws server on 127.0.0.1, in threads,
//      answering the API messages OFFgrid-SDR's JS uses (get/set_property,
//      device_stream_enable, iq/audio/spectrum streams, selected_device_name,
//      apply_device_profile), as an RSPdx would (antenna C listed only below
//      200 MHz, an LNA range by band, the centre rounded to a 2.5 kHz step,
//      2.048 MHz kept when asked for less), streaming a synthetic IQ tone,
//      48 kHz sound and 1024-bin spectra.  The client connects (through a
//      fragmented answer with a ping in it), starts the RSP, sets frequency,
//      rate and gain, receives IQ at the right rate with the tone at the right
//      frequency (the rounding corrected; resampled from 2.048 to 1.024 MHz),
//      switches antennas by frequency, probes the antenna list, applies a
//      profile, runs Lite mode (sound, spectrum, tuning), runs the speed test,
//      reconnects after the server drops it, and closes cleanly (code 1000).
//
//  How it fits in
//  --------------
//  Build (from the repository root):
//    c++ -std=c++11 -O2 -Wall -Wextra -pedantic-errors -Werror=c++14-extensions \
//        -Werror=c++17-extensions -Isrc -pthread src/sdrconnect/*.cpp \
//        tests/test_sdrconnect.cpp -o test_sdrconnect
//  Takes about 20 s (real time: the streams are paced like SDRconnect's).
//
//  Contributing
//  ------------
//  The mock follows what OFFgrid-SDR's JS (diddy-boy) expects of SDRconnect;
//  where the real SDRconnect is known to differ, change the mock and say so.
//  No real RSP or SDRconnect is needed.
// ============================================================================
#include "sdrconnect/SdrConnect.h"
#include "sdrconnect/WebSocket.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace {

int g_fail = 0, g_pass = 0;
void check(bool ok, const std::string& what) {
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
    if (ok) ++g_pass; else ++g_fail;
}
std::string bytes(const unsigned char* p, std::size_t n) { return std::string(reinterpret_cast<const char*>(p), n); }
void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
template <class P>
bool waitFor(P pred, int ms) {
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
        if (pred()) return true;
        sleepMs(10);
    }
    return pred();
}
std::string fmt(const char* f, double a) { char b[256]; std::snprintf(b, sizeof b, f, a); return b; }
std::string fmt2(const char* f, double a, double c) { char b[256]; std::snprintf(b, sizeof b, f, a, c); return b; }
const double kPi = 3.14159265358979323846;

// =========================================================== mock server ====
// A tiny SDRconnect: the WebSocket API's messages as OFFgrid-SDR's JS uses
// them, for one RSPdx.  Device state survives the connections (as the real
// SDRconnect's does: dropping the link doesn't stop the RSP).
class MockSdrConnect {
public:
    MockSdrConnect()
        : listenFd_(-1), port_(0), stop_(false), started_(false), center_(100e6), rate_(2048000), lna_(4),
          antenna_("Antenna A"), device_("RSPdx"), demod_("AM"), vfo_(0), bw_(0), keepRate_(true),
          fragmentFirst_(true), connections_(0), pongs_(0), lastClose_(-1), unmasked_(0), startAt_(0) {}
    ~MockSdrConnect() { shutdown(); }

    bool start() {
        listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listenFd_ < 0) return false;
        int one = 1;
        ::setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        sockaddr_in a;
        std::memset(&a, 0, sizeof a);
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        a.sin_port = 0;
        if (::bind(listenFd_, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0) return false;
        socklen_t l = sizeof a;
        ::getsockname(listenFd_, reinterpret_cast<sockaddr*>(&a), &l);
        port_ = ntohs(a.sin_port);
        if (::listen(listenFd_, 8) != 0) return false;
        acceptThread_ = std::thread(&MockSdrConnect::acceptLoop, this);
        return true;
    }
    void shutdown() {
        if (stop_.exchange(true)) return;
        if (acceptThread_.joinable()) acceptThread_.join();
        if (listenFd_ >= 0) ::close(listenFd_);
        dropAll();
        joinAll();
    }
    // the network goes: every connection cut, without a close frame
    void dropAll() {
        std::lock_guard<std::mutex> lk(connsM_);
        for (std::size_t i = 0; i < conns_.size(); ++i) {
            conns_[i]->alive = false;
            ::shutdown(conns_[i]->fd, SHUT_RDWR);
        }
    }
    int port() const { return port_; }

    // ---- what the tests look at
    std::string get(const std::string& p) { std::lock_guard<std::mutex> lk(m_); return propLocked(p); }
    bool started() { std::lock_guard<std::mutex> lk(m_); return started_; }
    double center() { std::lock_guard<std::mutex> lk(m_); return center_; }
    int lna() { std::lock_guard<std::mutex> lk(m_); return lna_; }
    std::string antenna() { std::lock_guard<std::mutex> lk(m_); return antenna_; }
    std::string profile() { std::lock_guard<std::mutex> lk(m_); return profile_; }
    int connections() { std::lock_guard<std::mutex> lk(m_); return connections_; }
    int pongs() { std::lock_guard<std::mutex> lk(m_); return pongs_; }
    int lastClose() { std::lock_guard<std::mutex> lk(m_); return lastClose_; }
    int unmasked() { std::lock_guard<std::mutex> lk(m_); return unmasked_; }
    bool logged(const std::string& entry) {
        std::lock_guard<std::mutex> lk(m_);
        return std::find(log_.begin(), log_.end(), entry) != log_.end();
    }
    bool anyStream() {
        std::lock_guard<std::mutex> lk(connsM_);
        for (std::size_t i = 0; i < conns_.size(); ++i)
            if (conns_[i]->alive && (conns_[i]->iq || conns_[i]->audio || conns_[i]->spec)) return true;
        return false;
    }
    bool iqOn() {
        std::lock_guard<std::mutex> lk(connsM_);
        for (std::size_t i = 0; i < conns_.size(); ++i)
            if (conns_[i]->alive && conns_[i]->iq) return true;
        return false;
    }

    // the tone's absolute frequency for the band the RSP is on
    static double toneFor(double center) {
        const double tones[] = {100.05e6, 10.02e6, 225.03e6, 1.01e6, 145.51e6};
        for (int i = 0; i < 5; ++i)
            if (std::fabs(tones[i] - center) < 400e3) return tones[i];
        return center + 100e3;
    }

private:
    struct Conn {
        Conn() : fd(-1), alive(true), iq(false), audio(false), spec(false) {}
        int fd;
        std::mutex sendM;
        std::atomic<bool> alive, iq, audio, spec;
        std::thread reader, streamer;
    };

    std::string propLocked(const std::string& p) {
        char b[64];
        if (p == "api_version") return "1.0.3";
        if (p == "valid_devices") return "RSPdx";
        if (p == "active_device") return device_;
        if (p == "valid_antennas") return center_ < 200e6 ? "Antenna A,Antenna B,Antenna C" : "Antenna A,Antenna B";
        if (p == "active_antenna") return antenna_;
        if (p == "lna_state_min") return "0";
        if (p == "lna_state_max") return center_ < 60e6 ? "20" : "26";
        if (p == "lna_state") { std::snprintf(b, sizeof b, "%d", lna_); return b; }
        if (p == "device_sample_rate") { std::snprintf(b, sizeof b, "%.0f", rate_); return b; }
        if (p == "device_center_frequency") { std::snprintf(b, sizeof b, "%.0f", center_); return b; }
        if (p == "overload") return "false";
        if (p == "can_control") return "true";
        if (p == "started") return started_ ? "true" : "false";
        if (p == "spectrum_ref_level") return "-20";
        if (p == "spectrum_base") return "-130";
        if (p == "audio_mute") return "false";
        if (p == "audio_volume_percent") return "50";
        if (p == "demodulator") return demod_;
        if (p == "device_vfo_frequency") { std::snprintf(b, sizeof b, "%.0f", vfo_); return b; }
        if (p == "filter_bandwidth") { std::snprintf(b, sizeof b, "%.0f", bw_); return b; }
        if (p == "agc_enable") return "true";
        return std::string();  // unknown: no answer (as SDRconnect)
    }

    void acceptLoop() {
        while (!stop_) {
            pollfd p;
            p.fd = listenFd_; p.events = POLLIN; p.revents = 0;
            if (::poll(&p, 1, 50) <= 0) continue;
            int fd = ::accept(listenFd_, 0, 0);
            if (fd < 0) continue;
#ifdef SO_NOSIGPIPE
            { int one = 1; ::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one); }
#endif
            std::shared_ptr<Conn> c(new Conn());
            c->fd = fd;
            {
                std::lock_guard<std::mutex> lk(connsM_);
                conns_.push_back(c);
            }
            c->reader = std::thread(&MockSdrConnect::readerLoop, this, c);
        }
    }
    void joinAll() {
        std::vector<std::shared_ptr<Conn> > cs;
        {
            std::lock_guard<std::mutex> lk(connsM_);
            cs = conns_;
        }
        for (std::size_t i = 0; i < cs.size(); ++i) {
            if (cs[i]->reader.joinable()) cs[i]->reader.join();
            if (cs[i]->streamer.joinable()) cs[i]->streamer.join();
            ::close(cs[i]->fd);
        }
    }

    bool sendRaw(const std::shared_ptr<Conn>& c, const std::string& f) {
        std::lock_guard<std::mutex> lk(c->sendM);
        const char* p = f.data();
        std::size_t n = f.size();
        while (n) {
            ssize_t w = ::send(c->fd, p, n, MSG_NOSIGNAL);
            if (w <= 0) {
                if (w < 0 && errno == EINTR) continue;
                c->alive = false;
                return false;
            }
            p += w;
            n -= static_cast<std::size_t>(w);
        }
        return true;
    }
    bool sendText(const std::shared_ptr<Conn>& c, const std::string& t) {
        return sendRaw(c, sdrc::ws::encodeFrame(sdrc::ws::OpText, t));
    }
    void reply(const std::shared_ptr<Conn>& c, const char* type, const std::string& p, const std::string& v) {
        sendText(c, "{\"event_type\":\"" + std::string(type) + "\",\"property\":\"" + p + "\",\"value\":" + sdrc::json::quote(v) + "}");
    }

    void readerLoop(std::shared_ptr<Conn> c) {
        // ---- the handshake
        std::string req;
        while (req.find("\r\n\r\n") == std::string::npos && c->alive && !stop_) {
            pollfd p;
            p.fd = c->fd; p.events = POLLIN; p.revents = 0;
            if (::poll(&p, 1, 50) <= 0) continue;
            char b[2048];
            ssize_t g = ::recv(c->fd, b, sizeof b, 0);
            if (g <= 0) { c->alive = false; return; }
            req.append(b, static_cast<std::size_t>(g));
        }
        std::string key;
        std::size_t k = req.find("Sec-WebSocket-Key:");
        if (k != std::string::npos) {
            std::size_t e = req.find("\r\n", k);
            key = req.substr(k + 18, e - k - 18);
            key.erase(0, key.find_first_not_of(' '));
        }
        sendRaw(c, "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: " +
                       sdrc::ws::acceptKey(key) + "\r\n\r\n");
        bool fragment;
        {
            std::lock_guard<std::mutex> lk(m_);
            ++connections_;
            fragment = fragmentFirst_;
            fragmentFirst_ = false;
        }
        c->streamer = std::thread(&MockSdrConnect::streamLoop, this, c);
        // ---- frames
        sdrc::ws::FrameDecoder dec;
        sdrc::ws::MessageAssembler asmb;
        while (c->alive && !stop_) {
            pollfd p;
            p.fd = c->fd; p.events = POLLIN; p.revents = 0;
            if (::poll(&p, 1, 50) <= 0) continue;
            char b[65536];
            ssize_t g = ::recv(c->fd, b, sizeof b, 0);
            if (g <= 0) break;
            dec.feed(b, static_cast<std::size_t>(g));
            sdrc::ws::Frame f;
            std::string err;
            int r;
            while ((r = dec.next(f, err)) > 0) {
                if (!f.masked) { std::lock_guard<std::mutex> lk(m_); ++unmasked_; }
                if (f.opcode == sdrc::ws::OpPong) { std::lock_guard<std::mutex> lk(m_); ++pongs_; continue; }
                if (f.opcode == sdrc::ws::OpPing) { sendRaw(c, sdrc::ws::encodeFrame(sdrc::ws::OpPong, f.payload)); continue; }
                if (f.opcode == sdrc::ws::OpClose) {
                    int code = f.payload.size() >= 2 ? ((static_cast<unsigned char>(f.payload[0]) << 8) | static_cast<unsigned char>(f.payload[1])) : 1005;
                    { std::lock_guard<std::mutex> lk(m_); lastClose_ = code; }
                    sendRaw(c, sdrc::ws::encodeFrame(sdrc::ws::OpClose, f.payload.substr(0, 2)));
                    c->alive = false;
                    break;
                }
                int op;
                std::string data;
                if (asmb.add(f, op, data, err) == 1 && op == sdrc::ws::OpText) command(c, data, fragment);
            }
            if (r < 0) break;
        }
        c->alive = false;
        ::shutdown(c->fd, SHUT_RDWR);
    }

    void command(const std::shared_ptr<Conn>& c, const std::string& text, bool& fragment) {
        std::map<std::string, std::string> m;
        if (!sdrc::json::parseFlat(text, m)) return;
        const std::string t = m["event_type"], p = m["property"], v = m["value"];
        if (t == "get_property") {
            std::string val;
            {
                std::lock_guard<std::mutex> lk(m_);
                val = propLocked(p);
            }
            if (val.empty()) return;
            if (p == "api_version" && fragment) {
                // the answer in three fragments, a ping between the first two (RFC 6455 5.4: allowed)
                fragment = false;
                std::string js = "{\"event_type\":\"get_property_response\",\"property\":\"api_version\",\"value\":\"" + val + "\",\"device\":\"primary\"}";
                std::string f = sdrc::ws::encodeFrame(sdrc::ws::OpText, js.substr(0, 10), false) +
                                sdrc::ws::encodeFrame(sdrc::ws::OpPing, "are you there") +
                                sdrc::ws::encodeFrame(sdrc::ws::OpContinuation, js.substr(10, 30), false) +
                                sdrc::ws::encodeFrame(sdrc::ws::OpContinuation, js.substr(40), true);
                sendRaw(c, f);
                return;
            }
            reply(c, "get_property_response", p, val);
            return;
        }
        if (t == "set_property") {
            std::vector<std::pair<std::string, std::string> > changed;
            {
                std::lock_guard<std::mutex> lk(m_);
                log_.push_back(p + "=" + v);
                if (p == "device_center_frequency") {
                    double was = center_;
                    center_ = std::floor(std::atof(v.c_str()) / 2500.0 + 0.5) * 2500.0;  // its tuning step
                    changed.push_back(std::make_pair(p, propLocked(p)));
                    if ((was < 200e6) != (center_ < 200e6)) changed.push_back(std::make_pair(std::string("valid_antennas"), propLocked("valid_antennas")));
                    // (Antenna C above 200 MHz: SDRconnect moves off it)
                    if (center_ >= 200e6 && antenna_ == "Antenna C") { antenna_ = "Antenna A"; changed.push_back(std::make_pair(std::string("active_antenna"), antenna_)); }
                } else if (p == "device_sample_rate") {
                    double r = std::atof(v.c_str());
                    rate_ = keepRate_ && r < 2048000 ? 2048000 : r;  // as an RSPdx was seen to do
                    changed.push_back(std::make_pair(p, propLocked(p)));
                } else if (p == "lna_state") {
                    lna_ = std::atoi(v.c_str());
                    changed.push_back(std::make_pair(p, propLocked(p)));
                } else if (p == "active_antenna") {
                    antenna_ = v;
                    changed.push_back(std::make_pair(p, v));
                } else if (p == "demodulator") {
                    demod_ = v;
                    changed.push_back(std::make_pair(p, v));
                } else if (p == "device_vfo_frequency") {
                    vfo_ = std::atof(v.c_str());
                    changed.push_back(std::make_pair(p, propLocked(p)));
                } else if (p == "filter_bandwidth") {
                    bw_ = std::atof(v.c_str());
                    changed.push_back(std::make_pair(p, propLocked(p)));
                }
            }
            for (std::size_t i = 0; i < changed.size(); ++i) reply(c, "property_changed", changed[i].first, changed[i].second);
            return;
        }
        {
            std::lock_guard<std::mutex> lk(m_);
            log_.push_back(t + "=" + v);
        }
        if (t == "device_stream_enable") {
            std::lock_guard<std::mutex> lk(m_);
            if (v == "true" && !started_) startAt_ = nowS() + 0.2;  // the RSP takes a moment to open
            if (v == "false") { started_ = false; startAt_ = 0; stopNotice_ = true; }
        } else if (t == "selected_device_name") {
            std::lock_guard<std::mutex> lk(m_);
            device_ = v;
        } else if (t == "apply_device_profile") {
            std::lock_guard<std::mutex> lk(m_);
            profile_ = v;
        } else if (t == "iq_stream_enable") c->iq = v == "true";
        else if (t == "audio_stream_enable") c->audio = v == "true";
        else if (t == "spectrum_enable") c->spec = v == "true";
    }

    static double nowS() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    // IQ, sound and spectrum, paced in real time
    void streamLoop(std::shared_ptr<Conn> c) {
        double t0 = nowS(), iqSent = 0, auSent = 0, auT0 = 0, lastSpec = 0, phase = 0, auPhase = 0, lastRate = 0;
        bool wasIq = false, wasAu = false;
        std::string buf;
        while (c->alive && !stop_) {
            sleepMs(4);
            double now = nowS(), center, rate;
            bool on;
            {
                std::lock_guard<std::mutex> lk(m_);
                if (startAt_ > 0 && now >= startAt_) { started_ = true; startAt_ = 0; startNotice_ = true; }
                on = started_;
                center = center_;
                rate = rate_;
            }
            // property_changed for "started", to every connection
            bool sn, stn;
            {
                std::lock_guard<std::mutex> lk(m_);
                sn = startNotice_; stn = stopNotice_;
                startNotice_ = stopNotice_ = false;
            }
            if (sn) reply(c, "property_changed", "started", "true");
            if (stn) reply(c, "property_changed", "started", "false");
            bool iq = on && c->iq, au = on && c->audio, sp = on && c->spec;
            if (iq && (!wasIq || rate != lastRate)) { t0 = now; iqSent = 0; lastRate = rate; }
            wasIq = iq;
            if (iq) {
                double due = std::floor((now - t0) * rate) - iqSent;
                double w = 2 * kPi * (toneFor(center) - center) / rate;
                while (due > 0) {
                    std::size_t n = static_cast<std::size_t>(std::min(due, 32768.0));
                    buf.assign(2 + 4 * n, '\0');
                    buf[0] = 2;  // type 2, little-endian
                    for (std::size_t i = 0; i < n; ++i) {
                        std::int16_t I = static_cast<std::int16_t>(std::floor(16384 * std::cos(phase) + 0.5));
                        std::int16_t Q = static_cast<std::int16_t>(std::floor(16384 * std::sin(phase) + 0.5));
                        phase += w;
                        if (phase > kPi) phase -= 2 * kPi;
                        if (phase < -kPi) phase += 2 * kPi;
                        buf[2 + 4 * i] = static_cast<char>(I & 0xFF);
                        buf[3 + 4 * i] = static_cast<char>((I >> 8) & 0xFF);
                        buf[4 + 4 * i] = static_cast<char>(Q & 0xFF);
                        buf[5 + 4 * i] = static_cast<char>((Q >> 8) & 0xFF);
                    }
                    if (!sendRaw(c, sdrc::ws::encodeFrame(sdrc::ws::OpBinary, buf))) return;
                    iqSent += n;
                    due -= n;
                }
            }
            if (au && !wasAu) { auSent = 0; auT0 = now; }
            wasAu = au;
            if (au) {
                double due = std::floor((now - auT0) * 48000) - auSent;
                while (due >= 960) {  // 20 ms frames
                    buf.assign(2 + 4 * 960, '\0');
                    buf[0] = 1;
                    for (int i = 0; i < 960; ++i) {
                        std::int16_t s = static_cast<std::int16_t>(std::floor(9830 * std::sin(auPhase) + 0.5));
                        auPhase += 2 * kPi * 1000 / 48000;
                        buf[2 + 4 * i] = static_cast<char>(s & 0xFF);
                        buf[3 + 4 * i] = static_cast<char>((s >> 8) & 0xFF);
                        buf[4 + 4 * i] = static_cast<char>(s & 0xFF);
                        buf[5 + 4 * i] = static_cast<char>((s >> 8) & 0xFF);
                    }
                    if (!sendRaw(c, sdrc::ws::encodeFrame(sdrc::ws::OpBinary, buf))) return;
                    auSent += 960;
                    due -= 960;
                }
            }
            if (sp && now - lastSpec >= 0.05) {  // 20 a second
                lastSpec = now;
                buf.assign(2 + 1024, '\0');
                buf[0] = 3;
                for (int i = 0; i < 1024; ++i) buf[2 + i] = static_cast<char>(i == 700 ? 230 : 40 + (i * 7) % 11);
                if (!sendRaw(c, sdrc::ws::encodeFrame(sdrc::ws::OpBinary, buf))) return;
            }
        }
    }

    int listenFd_, port_;
    std::atomic<bool> stop_;
    std::thread acceptThread_;
    std::mutex connsM_;
    std::vector<std::shared_ptr<Conn> > conns_;
    std::mutex m_;
    bool started_;
    double center_, rate_;
    int lna_;
    std::string antenna_, device_, demod_, profile_;
    double vfo_, bw_;
    bool keepRate_, fragmentFirst_;
    int connections_, pongs_, lastClose_, unmasked_;
    double startAt_;
    bool startNotice_ = false, stopNotice_ = false;
    std::vector<std::string> log_;
};

// ============================================================ IQ capture ====
struct Capture {
    std::mutex m;
    std::vector<std::complex<float> > iq;
    std::uint64_t total = 0;
    std::set<std::thread::id> threads;
    void reset() { std::lock_guard<std::mutex> lk(m); iq.clear(); }
    void add(const std::complex<float>* p, std::size_t n) {
        std::lock_guard<std::mutex> lk(m);
        threads.insert(std::this_thread::get_id());
        total += n;
        if (iq.size() < 400000) iq.insert(iq.end(), p, p + n);
    }
    std::uint64_t count() { std::lock_guard<std::mutex> lk(m); return total; }
    // the tone's frequency: the mean phase step, in Hz at rate
    double toneHz(double rate) {
        std::lock_guard<std::mutex> lk(m);
        if (iq.size() < 2000) return NAN;
        std::complex<double> acc(0, 0);
        std::size_t a = iq.size() / 4;  // past the first quarter
        for (std::size_t i = a + 1; i < iq.size(); ++i)
            acc += std::complex<double>(iq[i]) * std::conj(std::complex<double>(iq[i - 1]));
        return std::arg(acc) * rate / (2 * kPi);
    }
    double meanPower() {
        std::lock_guard<std::mutex> lk(m);
        double s = 0;
        for (std::size_t i = 0; i < iq.size(); ++i) s += std::norm(iq[i]);
        return iq.empty() ? 0 : s / iq.size();
    }
};

double measureRate(Capture& cap, int ms) {
    std::uint64_t c0 = cap.count();
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    sleepMs(ms);
    double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return (cap.count() - c0) / s;
}

// =================================================================== tests ===
void testHashes() {
    using sdrc::ws::sha1;
    using sdrc::ws::toHex;
    check(toHex(sha1("abc")) == "a9993e364706816aba3e25717850c26c9cd0d89d", "SHA-1 \"abc\" (FIPS 180-2 A.1)");
    check(toHex(sha1("")) == "da39a3ee5e6b4b0d3255bfef95601890afd80709", "SHA-1 of the empty string");
    check(toHex(sha1("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq")) == "84983e441c3bd26ebaae4aa1f95129e5e54670f1",
          "SHA-1 448-bit message (FIPS 180-2 A.2: two padding blocks)");
    check(toHex(sha1(std::string(1000000, 'a'))) == "34aa973cd4c4daa4f61eeb2bdbad27316534016f", "SHA-1 one million 'a' (FIPS 180-2 A.3)");
    check(toHex(sha1("The quick brown fox jumps over the lazy dog")) == "2fd4e1c67a2d28fced849ee1bb76e7391b93eb12", "SHA-1 quick brown fox");
    const char* in[] = {"", "f", "fo", "foo", "foob", "fooba", "foobar"};
    const char* out[] = {"", "Zg==", "Zm8=", "Zm9v", "Zm9vYg==", "Zm9vYmE=", "Zm9vYmFy"};
    bool b64 = true;
    for (int i = 0; i < 7; ++i) b64 = b64 && sdrc::ws::base64Encode(in[i]) == out[i];
    check(b64, "base64 RFC 4648 section 10 vectors");
    check(sdrc::ws::acceptKey("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=",
          "RFC 6455 handshake: key dGhlIHNhbXBsZSBub25jZQ== -> accept s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
}

void testFrames() {
    using namespace sdrc::ws;
    const unsigned char unmaskedHello[] = {0x81, 0x05, 0x48, 0x65, 0x6c, 0x6c, 0x6f};
    const unsigned char maskedHello[] = {0x81, 0x85, 0x37, 0xfa, 0x21, 0x3d, 0x7f, 0x9f, 0x4d, 0x51, 0x58};
    const unsigned char frag1[] = {0x01, 0x03, 0x48, 0x65, 0x6c};
    const unsigned char frag2[] = {0x80, 0x02, 0x6c, 0x6f};
    const unsigned char ping[] = {0x89, 0x05, 0x48, 0x65, 0x6c, 0x6c, 0x6f};
    const unsigned char maskedPong[] = {0x8a, 0x85, 0x37, 0xfa, 0x21, 0x3d, 0x7f, 0x9f, 0x4d, 0x51, 0x58};
    check(encodeFrame(OpText, "Hello") == bytes(unmaskedHello, 7), "RFC 6455 5.7: unmasked text \"Hello\"");
    check(encodeFrame(OpText, "Hello", true, true, 0x37fa213du) == bytes(maskedHello, 11), "RFC 6455 5.7: masked text \"Hello\"");
    check(encodeFrame(OpText, "Hel", false) == bytes(frag1, 5) && encodeFrame(OpContinuation, "lo") == bytes(frag2, 4),
          "RFC 6455 5.7: fragmented unmasked text");
    check(encodeFrame(OpPing, "Hello") == bytes(ping, 7), "RFC 6455 5.7: unmasked ping");
    check(encodeFrame(OpPong, "Hello", true, true, 0x37fa213du) == bytes(maskedPong, 11), "RFC 6455 5.7: masked pong");
    std::string f256 = encodeFrame(OpBinary, std::string(256, 'x'));
    const unsigned char h256[] = {0x82, 0x7E, 0x01, 0x00};
    check(f256.size() == 260 && f256.substr(0, 4) == bytes(h256, 4), "RFC 6455 5.7: 256-byte binary (16-bit length)");
    std::string f64k = encodeFrame(OpBinary, std::string(65536, 'x'));
    const unsigned char h64k[] = {0x82, 0x7F, 0, 0, 0, 0, 0, 1, 0, 0};
    check(f64k.size() == 65546 && f64k.substr(0, 10) == bytes(h64k, 10), "RFC 6455 5.7: 64 KiB binary (64-bit length)");

    // decoding the RFC's masked frame
    {
        FrameDecoder d;
        d.feed(reinterpret_cast<const char*>(maskedHello), 11);
        Frame f;
        std::string e;
        check(d.next(f, e) == 1 && f.masked && f.fin && f.opcode == OpText && f.payload == "Hello", "decode RFC masked \"Hello\"");
    }
    // round trips: lengths around the 7/16/64-bit boundaries, masked and not, whole and byte by byte
    bool allOk = true, byteOk = true;
    const std::size_t sizes[] = {0, 1, 125, 126, 127, 65535, 65536, 100000};
    for (int mi = 0; mi < 2; ++mi)
        for (std::size_t si = 0; si < sizeof sizes / sizeof sizes[0]; ++si) {
            std::string pl(sizes[si], '\0');
            for (std::size_t i = 0; i < pl.size(); ++i) pl[i] = static_cast<char>((i * 31 + 7) & 0xFF);
            std::string fr = encodeFrame(OpBinary, pl, true, mi == 1, 0xA1B2C3D4u);
            FrameDecoder d;
            d.feed(fr.data(), fr.size());
            Frame f;
            std::string e;
            allOk = allOk && d.next(f, e) == 1 && f.payload == pl && f.masked == (mi == 1) && d.buffered() == 0;
            if (sizes[si] <= 65536) {
                FrameDecoder d2;
                int r = 0;
                for (std::size_t i = 0; i < fr.size() && r == 0; ++i) {
                    d2.feed(fr.data() + i, 1);
                    r = d2.next(f, e);
                    if (i + 1 < fr.size() && r != 0) byteOk = false;
                }
                byteOk = byteOk && r == 1 && f.payload == pl;
            }
        }
    check(allOk, "frame round trips: 0..100000 bytes, masked and unmasked");
    check(byteOk, "frame decode fed one byte at a time");

    // fragmentation with a ping in between, as a server may send
    {
        std::string wire = encodeFrame(OpText, "Hel", false, true, 1) + encodeFrame(OpPing, "p", true, true, 2) +
                           encodeFrame(OpContinuation, "l", false, true, 3) + encodeFrame(OpContinuation, "o", true, true, 4);
        FrameDecoder d;
        d.feed(wire.data(), wire.size());
        MessageAssembler a;
        Frame f;
        std::string e, data;
        int op = -1, got = 0, pings = 0;
        while (d.next(f, e) == 1) {
            if (f.opcode == OpPing) { ++pings; continue; }
            if (a.add(f, op, data, e) == 1) ++got;
        }
        check(got == 1 && pings == 1 && op == OpText && data == "Hello", "fragments joined around an interleaved ping");
    }
    // protocol errors
    {
        Frame f;
        std::string e;
        FrameDecoder d1;
        std::string big = encodeFrame(OpPing, std::string(126, 'x'));
        d1.feed(big.data(), big.size());
        bool e1 = d1.next(f, e) < 0;
        FrameDecoder d2;
        std::string fragPing = encodeFrame(OpPing, "x", false);
        d2.feed(fragPing.data(), fragPing.size());
        bool e2 = d2.next(f, e) < 0;
        FrameDecoder d3;
        const unsigned char rsv[] = {0xC1, 0x00};
        d3.feed(reinterpret_cast<const char*>(rsv), 2);
        bool e3 = d3.next(f, e) < 0;
        FrameDecoder d4;
        const unsigned char reserved[] = {0x83, 0x00};
        d4.feed(reinterpret_cast<const char*>(reserved), 2);
        bool e4 = d4.next(f, e) < 0;
        MessageAssembler a;
        Frame c;
        c.opcode = OpContinuation;
        int op;
        std::string data;
        bool e5 = a.add(c, op, data, e) < 0;
        Frame t1;
        t1.opcode = OpText; t1.fin = false; t1.payload = "a";
        a.add(t1, op, data, e);
        bool e6 = a.add(t1, op, data, e) < 0;
        FrameDecoder d7(1000);
        std::string tooBig = encodeFrame(OpBinary, std::string(1001, 'x'));
        d7.feed(tooBig.data(), tooBig.size());
        bool e7 = d7.next(f, e) < 0;
        check(e1 && e2 && e3 && e4 && e5 && e6 && e7,
              "rejected: control > 125 bytes, fragmented control, RSV bits, reserved opcode, stray continuation, interleaved messages, over the limit");
    }
}

void testHelpers() {
    std::map<std::string, std::string> m;
    bool ok = sdrc::json::parseFlat("{\"event_type\":\"property_changed\", \"property\":\"lna_state\",\"value\":\"4\",\"n\":12.5,\"b\":true,"
                                    "\"o\":{\"x\":[1,\"}\"]},\"s\":\"a\\\"b\\u00e9\\n\"}", m);
    check(ok && m["event_type"] == "property_changed" && m["value"] == "4" && m["n"] == "12.5" && m["b"] == "true" &&
              m["s"] == "a\"b\xc3\xa9\n" && m.count("o"),
          "JSON: flat object, escapes, numbers, booleans, nested values skipped");
    check(sdrc::json::event("set_property", "device_center_frequency", "100000000") ==
              "{\"event_type\":\"set_property\",\"property\":\"device_center_frequency\",\"value\":\"100000000\"}",
          "JSON: the command format sdrcSend writes");

    std::string h, p, e;
    int port = 0;
    bool a1 = sdrc::Client::parseAddress("192.168.1.20", h, port, p, e) && h == "192.168.1.20" && port == 5454 && p == "/";
    bool a2 = sdrc::Client::parseAddress("ws://shack-pc.local:5455/api", h, port, p, e) && h == "shack-pc.local" && port == 5455 && p == "/api";
    bool a3 = sdrc::Client::parseAddress("[::1]:6000", h, port, p, e) && h == "::1" && port == 6000;
    bool a4 = sdrc::Client::parseAddress("fe80::1", h, port, p, e) && h == "fe80::1" && port == 5454;
    bool a5 = !sdrc::Client::parseAddress("", h, port, p, e) && e.find("No address") == 0;
    bool a6 = !sdrc::Client::parseAddress("host:70000", h, port, p, e) && !sdrc::Client::parseAddress("wss://host", h, port, p, e);
    check(a1 && a2 && a3 && a4 && a5 && a6, "addresses: default port 5454, ws://, paths, IPv6, errors (sdrcUrl)");

    using sdrc::Client;
    check(Client::lnaForFraction(0, 0, 27) == 27 && Client::lnaForFraction(1, 0, 27) == 0 && Client::lnaForFraction(0.9, 0, 27) == 8 &&
              Client::lnaForFraction(0.45, 0, 27) == 18,
          "LNA mapping: 0 -> least gain, 90% -> the safe limit (70% of the range), 100% -> full gain");
    check(Client::judgeSpeed(8.2, 2048000).verdict == sdrc::SpeedVerdict::KeepsUp &&
              Client::judgeSpeed(7.6, 2048000).verdict == sdrc::SpeedVerdict::OnlyJust &&
              Client::judgeSpeed(5.0, 2048000).verdict == sdrc::SpeedVerdict::TooSlow &&
              std::fabs(Client::judgeSpeed(5.0, 2048000).needMBs - 8.192) < 1e-9 && Client::judgeSpeed(5.0, 2048000).percent == 61,
          "speed verdicts: >= 97% keeps up, 90-97% only just, below too slow; 4 bytes a sample");
    check(Client::antennaLimitHz("RSPdx", "Antenna C") == 200e6 && Client::antennaLimitHz("RSPdx-R2", "Antenna C") == 200e6 &&
              std::isinf(Client::antennaLimitHz("RSPdx", "Antenna A")) && std::isinf(Client::antennaLimitHz("RSP1A", "Antenna C")),
          "Antenna C of an RSPdx limited to 200 MHz");
    check(Client::liteDemodulator("WBFM") == "WFM" && Client::liteDemodulator("NBFM") == "NFM" && Client::liteDemodulator("RTTY") == "USB" &&
              !Client::liteSupports("DAB") && Client::liteSupports("AM"),
          "Lite: OFFgrid-SDR modes to SDRconnect demodulators; DAB/DRM/AIS/VDL2 need the full stream");
    std::complex<float> c[3] = {std::complex<float>(0, 0), std::complex<float>(1, -1), std::complex<float>(-0.5f, 0.5f)};
    std::uint8_t u8[6];
    Client::toU8(c, 3, u8);
    const unsigned char raw[] = {0x00, 0x40, 0x00, 0xC0};  // I = 16384, Q = -16384
    std::complex<float> back;
    Client::s16ToComplex(raw, 4, &back);
    check(u8[0] == 128 && u8[2] == 255 && u8[3] == 0 && u8[4] == 64 && back == std::complex<float>(0.5f, -0.5f),
          "conversions: complex float -> u8 IQ (127.5 = zero); SDRconnect's int16 LE / 32768");
}

void testClient() {
    MockSdrConnect mock;
    if (!mock.start()) { check(false, "mock SDRconnect listening on 127.0.0.1"); return; }
    char addr[64];
    std::snprintf(addr, sizeof addr, "127.0.0.1:%d", mock.port());

    // ---- refused: nothing listening
    {
        sdrc::Client c;
        std::string err;
        bool ok = c.connect("127.0.0.1:1", err, 1500);
        check(!ok && err.find("Couldn't connect") == 0, "connect to a port with nothing there fails with OFFgrid-SDR's message");
    }

    sdrc::Client rsp;
    std::mutex sm;
    std::vector<std::pair<sdrc::LinkState, std::string> > states;
    std::set<std::thread::id> cbThreads;
    rsp.setStateCallback([&](sdrc::LinkState s, const std::string& msg) {
        std::lock_guard<std::mutex> lk(sm);
        states.push_back(std::make_pair(s, msg));
        cbThreads.insert(std::this_thread::get_id());
    });
    rsp.setAutoReconnect(true, 400, 200);  // (quicker than the JS's 3 s, for the test)
    std::string err;
    bool ok = rsp.connect(addr, err, 3000);
    check(ok, "connect to the mock SDRconnect" + (ok ? std::string() : ": " + err));
    if (!ok) return;
    check(rsp.apiVersion() == "1.0.3" && rsp.device() == "RSPdx" && rsp.devices().size() == 1 && rsp.canControl(),
          "api_version (from a fragmented answer), device, devices, can_control");
    check(waitFor([&]() { return mock.pongs() >= 1; }, 1000), "the server's ping answered with a pong");
    check(mock.unmasked() == 0, "every client frame masked");

    // ---- the full stream
    Capture cap;
    rsp.setSampleRate(2048000);
    rsp.setFrequency(100001000);  // the mock rounds to 100.000 MHz (a 2.5 kHz step): 1 kHz off
    rsp.setGainFraction(1.0);
    ok = rsp.startIq([&](const std::complex<float>* p, std::size_t n) { cap.add(p, n); }, err);
    check(ok && mock.started(), "startIq: the RSP started (device_stream_enable, waited for \"started\")" + (ok ? std::string() : ": " + err));
    check(mock.logged("selected_device_name=RSPdx") == false, "the active device isn't selected again (it may reopen)");
    check(waitFor([&]() { return mock.lna() == 0; }, 2000), "gain 100% -> LNA state 0 (full gain) on the band's range");
    check(waitFor([&]() { return cap.count() > 500000; }, 4000), "IQ samples arrive");
    double r1 = measureRate(cap, 1500);
    check(std::fabs(r1 - 2048000) < 0.1 * 2048000, fmt("IQ at 2.048 MHz: %.0f samples/s measured", r1));
    cap.reset();
    sleepMs(300);
    double tone = cap.toneHz(rsp.sampleRate());
    check(std::fabs(tone - (100.05e6 - 100001000)) < 50,
          fmt2("tone at %.0f Hz, expected 49000 (SDRconnect's 1 kHz rounding corrected; uncorrected it'd be 50000), rate %.0f", tone, rsp.sampleRate()));
    check(std::fabs(cap.meanPower() - 0.25) < 0.02, fmt("IQ scaling: int16 / 32768 (tone at half scale: power %.3f)", cap.meanPower()));

    // ---- auto gain, then a retune to HF: antenna B by the rules, the new band's range
    rsp.setGainAuto();
    check(waitFor([&]() { return mock.lna() == 13; }, 2000), "auto gain -> the middle LNA state (13 of 0-26)");
    rsp.setFrequency(10e6);
    check(waitFor([&]() { return mock.antenna() == "Antenna B"; }, 2000), "10 MHz: Antenna B chosen automatically (A from 30 MHz up, B below)");
    check(waitFor([&]() { return mock.lna() == 10; }, 2000), "auto gain follows the new band's range (10 of 0-20)");
    sleepMs(500);
    cap.reset();
    sleepMs(400);
    tone = cap.toneHz(rsp.sampleRate());
    check(std::fabs(tone - 20000) < 50, fmt("after the retune, the tone at %.0f Hz (expected 20000)", tone));

    // ---- SDRconnect keeping 2.048 MHz when asked for 1.024: resampled
    rsp.setSampleRate(1024000);
    check(waitFor([&]() { return rsp.deviceSampleRate() == 2048000; }, 2000) && rsp.sampleRate() == 1024000,
          "asked for 1.024 MHz, SDRconnect keeps 2.048 (as an RSPdx did)");
    sleepMs(300);
    double r2 = measureRate(cap, 1500);
    check(std::fabs(r2 - 1024000) < 0.1 * 1024000, fmt("resampled to 1.024 MHz: %.0f samples/s", r2));
    cap.reset();
    sleepMs(400);
    tone = cap.toneHz(1024000);
    check(std::fabs(tone - 20000) < 50, fmt("resampled: the tone still at %.0f Hz (expected 20000)", tone));
    rsp.setSampleRate(2048000);

    // ---- antennas
    std::vector<std::string> ants = rsp.antennas();
    check(ants.size() == 3 && ants[2] == "Antenna C", "antennas at 10 MHz: A, B, C");
    rsp.setFrequency(225.0e6);
    check(waitFor([&]() { return mock.antenna() == "Antenna A" && mock.get("valid_antennas") == "Antenna A,Antenna B"; }, 2000),
          "225 MHz: SDRconnect lists only A and B; antenna A in use");
    sleepMs(300);
    ants = rsp.antennas();
    check(ants.size() == 3, "Antenna C still offered above 200 MHz (remembered, as the JS)");
    check(rsp.antennaFor(225e6) == "Antenna A" && rsp.antennaFor(5e6) == "Antenna B" && rsp.antennaFor(100e6) == "Antenna A",
          "antennaFor: by the crossover (30 MHz)");
    sdrc::AntennaRules rules = rsp.antennaRules();
    rules.map["Antenna C"] = sdrc::AntennaRule(sdrc::AntennaRule::Range, 0.1, 2);
    rules.map["Antenna B"] = sdrc::AntennaRule(sdrc::AntennaRule::Below);
    rsp.setAntennaRules(rules);
    check(rsp.antennaFor(1e6) == "Antenna C" && rsp.antennaFor(5e6) == "Antenna B", "a custom range wins (a MW loop on C, 0.1-2 MHz)");
    rules.map["Antenna C"] = sdrc::AntennaRule(sdrc::AntennaRule::Above);
    rules.map["Antenna A"] = sdrc::AntennaRule(sdrc::AntennaRule::None);
    rsp.setAntennaRules(rules);
    check(rsp.antennaFor(225e6) == "Antenna B" && rsp.antennaFor(150e6) == "Antenna C",
          "Antenna C never chosen above 200 MHz (another one used)");
    rsp.setAntennaRules(sdrc::AntennaRules());  // (empty: back to... a user's own, with no rules)
    bool probed = rsp.probeAntennas();
    check(probed && mock.logged("device_center_frequency=1000000") &&
              waitFor([&]() { return std::fabs(mock.center() - 225e6) < 5000; }, 2000),
          "probeAntennas: tuned to 1 MHz for the list, then back to 225 MHz");

    // ---- profile
    check(rsp.applyProfile("Bias-T on") && mock.profile() == "Bias-T on", "applyProfile: apply_device_profile sent");

    // ---- Lite mode
    std::mutex lm;
    std::uint64_t audioFrames = 0, specMsgs = 0;
    std::size_t lastBins = 0;
    float peakDb = -999, minDb = 999;
    std::set<std::thread::id> liteThreads;
    ok = rsp.startLite(
        [&](const float* lr, std::size_t n) {
            std::lock_guard<std::mutex> lk(lm);
            audioFrames += n;
            (void)lr;
            liteThreads.insert(std::this_thread::get_id());
        },
        [&](const float* db, std::size_t n) {
            std::lock_guard<std::mutex> lk(lm);
            ++specMsgs;
            lastBins = n;
            for (std::size_t i = 0; i < n; ++i) { peakDb = std::max(peakDb, db[i]); minDb = std::min(minDb, db[i]); }
        },
        err);
    check(ok && rsp.lite(), "startLite: switched over from the full stream" + (ok ? std::string() : ": " + err));
    check(waitFor([&]() { return !mock.iqOn(); }, 1000), "Lite: the IQ stream switched off at SDRconnect");
    rsp.setLiteTuning(sdrc::Client::liteDemodulator("NBFM"), 145500000, 12500);
    check(waitFor([&]() { return mock.get("demodulator") == "NFM" && mock.get("device_vfo_frequency") == "145500000" &&
                                 mock.get("filter_bandwidth") == "12500"; }, 1500),
          "Lite: demodulator, VFO and filter sent");
    sleepMs(300);
    std::uint64_t a0;
    {
        std::lock_guard<std::mutex> lk(lm);
        a0 = audioFrames;
    }
    sleepMs(1500);
    double auRate;
    {
        std::lock_guard<std::mutex> lk(lm);
        auRate = (audioFrames - a0) / 1.5;
    }
    check(std::fabs(auRate - 48000) < 0.15 * 48000, fmt("Lite sound at 48 kHz: %.0f frames/s", auRate));
    {
        std::lock_guard<std::mutex> lk(lm);
        float expPeak = -130 + 230 / 255.0f * 110;
        check(specMsgs >= 10 && lastBins == 1024 && std::fabs(peakDb - expPeak) < 0.01 && minDb >= -130,
              fmt2("Lite spectrum: 1024 bins, dB from spectrum_base/ref (peak %.2f dB, expected %.2f)", peakDb, expPeak));
    }
    sdrc::LiteStats ls = rsp.liteStats();
    check(ls.soundFramesPerSec > 30 && ls.spectrumFramesPerSec > 10 && ls.bins == 1024 && ls.soundDb > -20 && !ls.stereoSeen,
          fmt2("liteStats: %.1f sound and %.1f spectrum messages a second", ls.soundFramesPerSec, ls.spectrumFramesPerSec));
    std::uint64_t before = cap.count();
    rsp.setLite(false);
    check(waitFor([&]() { return cap.count() > before + 200000; }, 3000) && !rsp.lite(), "setLite(false): the IQ stream back");

    // ---- stop, then the speed test
    rsp.stop();
    check(!mock.started() && !mock.anyStream() && rsp.isConnected(), "stop: the streams off and the RSP stopped; still connected");
    sdrc::SpeedResult sp = rsp.speedTest(2048000, 1500);
    check(sp.ok && sp.gotMBs > 1 && sp.needMBs > 8.19 && sp.needMBs < 8.2 && !sp.text.empty(),
          fmt2("speed test: %.2f MB/s of %.2f needed", sp.gotMBs, sp.needMBs) + " -> " + sp.text);
    check(!mock.started() && !mock.anyStream(), "speed test: the RSP stopped again afterwards (it wasn't running)");

    // ---- the link drops while streaming: reconnected
    ok = rsp.startIq([&](const std::complex<float>* p, std::size_t n) { cap.add(p, n); }, err);
    check(ok && waitFor([&]() { return mock.iqOn(); }, 2000), "streaming again");
    sleepMs(300);
    mock.dropAll();
    check(waitFor([&]() { return rsp.state() == sdrc::LinkState::Lost || rsp.state() == sdrc::LinkState::Connecting; }, 2000),
          "link dropped: state Lost (retrying)");
    check(waitFor([&]() { return rsp.state() == sdrc::LinkState::Connected && mock.iqOn(); }, 6000) && mock.connections() >= 2,
          "reconnected by itself, the IQ stream enabled again");
    before = cap.count();
    check(waitFor([&]() { return cap.count() > before + 300000; }, 4000), "IQ flowing after the reconnection");
    check(rsp.stats().reconnects == 1, "stats: one reconnection");
    {
        std::lock_guard<std::mutex> lk(sm);
        bool lost = false, back = false;
        for (std::size_t i = 0; i < states.size(); ++i) {
            if (states[i].first == sdrc::LinkState::Lost && states[i].second.find("trying again every few seconds") != std::string::npos) lost = true;
            if (states[i].first == sdrc::LinkState::Connected && states[i].second.find("Reconnected") == 0) back = true;
        }
        check(lost && back, "state callbacks: \"Lost ... trying again every few seconds\", then \"Reconnected ...\"");
    }

    // ---- callbacks on one thread, the client's own
    {
        std::lock_guard<std::mutex> lk(cap.m);
        std::set<std::thread::id> all = cap.threads;
        all.insert(liteThreads.begin(), liteThreads.end());
        all.insert(cbThreads.begin(), cbThreads.end());
        check(all.size() == 1 && !all.count(std::this_thread::get_id()), "every callback on the client's one thread");
    }
    sdrc::LinkStats st = rsp.stats();
    check(st.bytes > 10000000 && st.iqSamples > 1000000, fmt2("stats: %.1f MB received, %.0f IQ samples delivered", st.bytes / 1e6, double(st.iqSamples)));

    // ---- clean goodbye
    rsp.disconnect();
    check(waitFor([&]() { return mock.lastClose() == 1000; }, 1000) && !mock.started(), "disconnect: RSP stopped, close frame with code 1000");
    check(rsp.state() == sdrc::LinkState::Off && !rsp.isConnected(), "state Off after disconnect");

    // ---- the server going away while not streaming: no retrying, state Off
    {
        sdrc::Client c2;
        std::string e2;
        bool ok2 = c2.connect(addr, e2, 3000);
        sleepMs(100);
        mock.dropAll();
        check(ok2 && waitFor([&]() { return c2.state() == sdrc::LinkState::Off; }, 2000), "dropped while idle: Off, no retries (as the JS)");
    }
    mock.shutdown();
}

}  // namespace

int main() {
    std::printf("== liboffgrid-sdrconnect tests ==\n");
    testHashes();
    testFrames();
    testHelpers();
    testClient();
    std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
