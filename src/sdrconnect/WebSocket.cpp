// ============================================================================
//  offgrid-sdrconnect - an SDRplay RSP through SDRplay's SDRconnect WebSocket
//  API, in C++11 (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/sdrconnect/WebSocket.cpp
//  Layer:  Library liboffgrid-sdrconnect
//
//  What this file does
//  -------------------
//  Implementation of WebSocket.h: SHA-1 and base64 (for the handshake's
//  Sec-WebSocket-Accept), RFC 6455 framing, and the client connection over
//  POSIX sockets (non-blocking, driven by poll() with timeouts).
//
//  How it fits in
//  --------------
//  Used by sdrc::Client (SdrConnect.cpp).  The browser version of
//  OFFgrid-SDR (by diddy-boy) uses the browser's WebSocket; this is its
//  stand-in for a terminal program ("Addition to the JS" throughout).
//
//  Contributing
//  ------------
//  Portable POSIX only (Linux and macOS): SIGPIPE is avoided per send
//  (MSG_NOSIGNAL) on Linux and per socket (SO_NOSIGPIPE) on macOS, never
//  with a process-wide signal handler, so the library leaves the host
//  program's signals alone.
// ============================================================================
#include "WebSocket.h"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0  // macOS: SO_NOSIGPIPE on the socket instead
#endif

namespace sdrc {
namespace ws {

// ---------------------------------------------------------------- SHA-1 ----
// FIPS 180-4: 512-bit blocks, 80 rounds, five 32-bit words of state.  Only
// the handshake uses it (SHA-1 is not used here for anything secret).
namespace {
inline std::uint32_t rol(std::uint32_t x, int n) { return (x << n) | (x >> (32 - n)); }

void sha1Block(std::uint32_t h[5], const unsigned char* p) {
    std::uint32_t w[80];
    for (int i = 0; i < 16; ++i)
        w[i] = (std::uint32_t(p[4 * i]) << 24) | (std::uint32_t(p[4 * i + 1]) << 16) |
               (std::uint32_t(p[4 * i + 2]) << 8) | std::uint32_t(p[4 * i + 3]);
    for (int i = 16; i < 80; ++i) w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    for (int i = 0; i < 80; ++i) {
        std::uint32_t f, k;
        if (i < 20) { f = (b & c) | (~b & d); k = 0x5A827999u; }
        else if (i < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1u; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDCu; }
        else { f = b ^ c ^ d; k = 0xCA62C1D6u; }
        std::uint32_t t = rol(a, 5) + f + e + k + w[i];
        e = d; d = c; c = rol(b, 30); b = a; a = t;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
}
}  // namespace

std::string sha1(const std::string& data) {
    std::uint32_t h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};
    const unsigned char* p = reinterpret_cast<const unsigned char*>(data.data());
    std::size_t n = data.size(), i = 0;
    for (; i + 64 <= n; i += 64) sha1Block(h, p + i);
    // the tail: the remaining bytes, 0x80, zeros, then the length in bits (64-bit big-endian)
    unsigned char tail[128];
    std::size_t rest = n - i;
    std::memset(tail, 0, sizeof tail);
    if (rest) std::memcpy(tail, p + i, rest);
    tail[rest] = 0x80;
    std::size_t tailLen = rest + 1 + 8 <= 64 ? 64 : 128;
    std::uint64_t bits = std::uint64_t(n) * 8u;
    for (int k = 0; k < 8; ++k) tail[tailLen - 1 - k] = static_cast<unsigned char>(bits >> (8 * k));
    sha1Block(h, tail);
    if (tailLen == 128) sha1Block(h, tail + 64);
    std::string out(20, '\0');
    for (int k = 0; k < 5; ++k)
        for (int j = 0; j < 4; ++j) out[4 * k + j] = static_cast<char>(h[k] >> (24 - 8 * j));
    return out;
}

std::string toHex(const std::string& bytes) {
    static const char hx[] = "0123456789abcdef";
    std::string s;
    s.reserve(bytes.size() * 2);
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        unsigned char c = static_cast<unsigned char>(bytes[i]);
        s += hx[c >> 4];
        s += hx[c & 15];
    }
    return s;
}

std::string base64Encode(const std::string& bytes) {
    static const char tbl[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((bytes.size() + 2) / 3 * 4);
    std::size_t i = 0;
    const unsigned char* p = reinterpret_cast<const unsigned char*>(bytes.data());
    for (; i + 3 <= bytes.size(); i += 3) {
        std::uint32_t v = (std::uint32_t(p[i]) << 16) | (std::uint32_t(p[i + 1]) << 8) | p[i + 2];
        out += tbl[(v >> 18) & 63]; out += tbl[(v >> 12) & 63]; out += tbl[(v >> 6) & 63]; out += tbl[v & 63];
    }
    std::size_t rest = bytes.size() - i;
    if (rest == 1) {
        std::uint32_t v = std::uint32_t(p[i]) << 16;
        out += tbl[(v >> 18) & 63]; out += tbl[(v >> 12) & 63]; out += "==";
    } else if (rest == 2) {
        std::uint32_t v = (std::uint32_t(p[i]) << 16) | (std::uint32_t(p[i + 1]) << 8);
        out += tbl[(v >> 18) & 63]; out += tbl[(v >> 12) & 63]; out += tbl[(v >> 6) & 63]; out += '=';
    }
    return out;
}

std::string acceptKey(const std::string& key) {
    // the GUID fixed by RFC 6455 section 1.3
    return base64Encode(sha1(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"));
}

// -------------------------------------------------------------- framing ----
//  0                   1                   2                   3
//  FIN RSV1-3 opcode(4) | MASK len(7) | [len16 | len64] | [mask key 4] | payload
std::string encodeFrame(int opcode, const std::string& payload, bool fin, bool masked,
                        std::uint32_t maskKey) {
    std::string f;
    std::uint64_t n = payload.size();
    f.reserve(payload.size() + 14);
    f += static_cast<char>((fin ? 0x80 : 0x00) | (opcode & 0x0F));
    unsigned char m = masked ? 0x80 : 0x00;
    if (n <= 125) {
        f += static_cast<char>(m | static_cast<unsigned char>(n));
    } else if (n <= 0xFFFF) {
        f += static_cast<char>(m | 126);
        f += static_cast<char>((n >> 8) & 0xFF);
        f += static_cast<char>(n & 0xFF);
    } else {
        f += static_cast<char>(m | 127);
        for (int k = 7; k >= 0; --k) f += static_cast<char>((n >> (8 * k)) & 0xFF);
    }
    if (!masked) return f + payload;
    unsigned char key[4] = {static_cast<unsigned char>(maskKey >> 24), static_cast<unsigned char>(maskKey >> 16),
                            static_cast<unsigned char>(maskKey >> 8), static_cast<unsigned char>(maskKey)};
    for (int k = 0; k < 4; ++k) f += static_cast<char>(key[k]);
    std::size_t at = f.size();
    f += payload;
    for (std::size_t i = 0; i < payload.size(); ++i) f[at + i] = static_cast<char>(f[at + i] ^ key[i & 3]);
    return f;
}

void FrameDecoder::feed(const char* data, std::size_t n) {
    // compact now and then, so the buffer doesn't grow with everything ever read
    if (pos_ > 0 && (pos_ >= buf_.size() || pos_ > (1u << 20))) {
        buf_.erase(0, pos_);
        pos_ = 0;
    }
    buf_.append(data, n);
}

int FrameDecoder::next(Frame& out, std::string& error) {
    std::size_t avail = buf_.size() - pos_;
    if (avail < 2) return 0;
    const unsigned char* p = reinterpret_cast<const unsigned char*>(buf_.data() + pos_);
    bool fin = (p[0] & 0x80) != 0;
    if (p[0] & 0x70) { error = "reserved bits set (no extension was negotiated)"; return -1; }
    int opcode = p[0] & 0x0F;
    bool control = (opcode & 0x08) != 0;
    if (!(opcode <= 2 || opcode == OpClose || opcode == OpPing || opcode == OpPong)) {
        error = "reserved opcode";
        return -1;
    }
    bool masked = (p[1] & 0x80) != 0;
    std::uint64_t len = p[1] & 0x7F;
    std::size_t hdr = 2;
    if (len == 126) {
        if (avail < 4) return 0;
        len = (std::uint64_t(p[2]) << 8) | p[3];
        hdr = 4;
    } else if (len == 127) {
        if (avail < 10) return 0;
        len = 0;
        for (int k = 0; k < 8; ++k) len = (len << 8) | p[2 + k];
        if (len >> 63) { error = "64-bit length with the top bit set"; return -1; }
        hdr = 10;
    }
    if (control && (len > 125 || !fin)) { error = "control frame fragmented or over 125 bytes"; return -1; }
    if (len > maxPayload_) { error = "frame too large"; return -1; }
    std::size_t keyAt = hdr;
    if (masked) hdr += 4;
    if (avail < hdr + len) return 0;
    out.fin = fin;
    out.opcode = opcode;
    out.masked = masked;
    out.payload.assign(buf_.data() + pos_ + hdr, static_cast<std::size_t>(len));
    if (masked) {
        const unsigned char* key = p + keyAt;
        for (std::size_t i = 0; i < out.payload.size(); ++i)
            out.payload[i] = static_cast<char>(out.payload[i] ^ key[i & 3]);
    }
    pos_ += hdr + static_cast<std::size_t>(len);
    if (pos_ == buf_.size()) { buf_.clear(); pos_ = 0; }
    return 1;
}

int MessageAssembler::add(const Frame& f, int& opcode, std::string& data, std::string& error) {
    if (f.opcode == OpText || f.opcode == OpBinary) {
        if (opcode_ >= 0) { error = "a new message began before the last one finished"; return -1; }
        if (f.fin) { opcode = f.opcode; data = f.payload; return 1; }
        opcode_ = f.opcode;
        parts_ = f.payload;
        return 0;
    }
    if (f.opcode != OpContinuation) { error = "not a data frame"; return -1; }
    if (opcode_ < 0) { error = "a continuation frame with nothing to continue"; return -1; }
    if (parts_.size() + f.payload.size() > maxMessage_) { error = "message too large"; reset(); return -1; }
    parts_ += f.payload;
    if (!f.fin) return 0;
    opcode = opcode_;
    data.swap(parts_);
    reset();
    return 1;
}

}  // namespace ws

// ------------------------------------------------------------ WebSocket ----
namespace {
typedef std::chrono::steady_clock Clock;
int msLeft(Clock::time_point end) {
    long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - Clock::now()).count();
    return ms < 0 ? 0 : (ms > 0x7FFFFFFF ? 0x7FFFFFFF : static_cast<int>(ms));
}
std::string lower(std::string s) {
    for (std::size_t i = 0; i < s.size(); ++i)
        if (s[i] >= 'A' && s[i] <= 'Z') s[i] = static_cast<char>(s[i] - 'A' + 'a');
    return s;
}
std::string trim(const std::string& s) {
    std::size_t a = s.find_first_not_of(" \t"), b = s.find_last_not_of(" \t\r");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}
void setNonBlocking(int fd) {
    int fl = ::fcntl(fd, F_GETFL, 0);
    if (fl >= 0) ::fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}
}  // namespace

WebSocket::WebSocket()
    : fd_(-1), open_(false), closeSent_(false), rng_(std::random_device()()), closeCode_(0),
      bytesIn_(0), pings_(0), wakeRead_(-1), wakeWrite_(-1) {
    int p[2];
    if (::pipe(p) == 0) {
        wakeRead_ = p[0];
        wakeWrite_ = p[1];
        setNonBlocking(wakeRead_);
        setNonBlocking(wakeWrite_);
    }
}

WebSocket::~WebSocket() {
    abort();
    if (fd_ >= 0) ::close(fd_);
    if (wakeRead_ >= 0) ::close(wakeRead_);
    if (wakeWrite_ >= 0) ::close(wakeWrite_);
}

bool WebSocket::connect(const std::string& host, int port, const std::string& path, int timeoutMs,
                        std::string& error) {
    if (fd_ >= 0) { error = "already used: make a new WebSocket for a new connection"; return false; }
    Clock::time_point end = Clock::now() + std::chrono::milliseconds(timeoutMs);
    struct addrinfo hints;
    std::memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    char portStr[16];
    std::snprintf(portStr, sizeof portStr, "%d", port);
    struct addrinfo* res = 0;
    int gai = ::getaddrinfo(host.c_str(), portStr, &hints, &res);
    if (gai != 0) { error = std::string("can't find ") + host + ": " + ::gai_strerror(gai); return false; }
    std::string lastErr = "no address";
    for (struct addrinfo* a = res; a; a = a->ai_next) {
        int fd = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (fd < 0) { lastErr = std::strerror(errno); continue; }
        setNonBlocking(fd);
#ifdef SO_NOSIGPIPE
        { int one = 1; ::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one); }
#endif
        { int one = 1; ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one); }
        // a generous receive buffer: the full IQ stream is 8 MB/s at 2 MHz, 40 MB/s at 10 MHz
        { int sz = 4 * 1024 * 1024; ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &sz, sizeof sz); }
        int r = ::connect(fd, a->ai_addr, a->ai_addrlen);
        if (r != 0 && errno != EINPROGRESS) { lastErr = std::strerror(errno); ::close(fd); continue; }
        if (r != 0) {
            struct pollfd pfd;
            pfd.fd = fd; pfd.events = POLLOUT; pfd.revents = 0;
            int pr = ::poll(&pfd, 1, msLeft(end));
            int soErr = 0;
            socklen_t sl = sizeof soErr;
            if (pr <= 0) { lastErr = pr == 0 ? "timed out" : std::strerror(errno); ::close(fd); continue; }
            ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &soErr, &sl);
            if (soErr != 0) { lastErr = std::strerror(soErr); ::close(fd); continue; }
        }
        fd_ = fd;
        break;
    }
    ::freeaddrinfo(res);
    if (fd_ < 0) { error = "can't connect to " + host + ":" + portStr + " (" + lastErr + ")"; return false; }

    // ---- the opening handshake (RFC 6455 section 4.1)
    std::string nonce(16, '\0');
    {
        std::lock_guard<std::mutex> lk(rngMutex_);
        for (int i = 0; i < 16; ++i) nonce[i] = static_cast<char>(rng_() & 0xFF);
    }
    std::string key = ws::base64Encode(nonce);
    bool v6 = host.find(':') != std::string::npos;
    std::string hostHdr = (v6 ? "[" + host + "]" : host) + ":" + portStr;
    std::string req = "GET " + (path.empty() ? std::string("/") : path) + " HTTP/1.1\r\n"
                      "Host: " + hostHdr + "\r\n"
                      "Upgrade: websocket\r\n"
                      "Connection: Upgrade\r\n"
                      "Sec-WebSocket-Key: " + key + "\r\n"
                      "Sec-WebSocket-Version: 13\r\n\r\n";
    if (!writeAll(req.data(), req.size(), msLeft(end))) {
        error = "the connection failed while asking to upgrade to WebSocket";
        shutdownSocket();
        return false;
    }
    std::string resp;
    std::size_t headerEnd = std::string::npos;
    while (headerEnd == std::string::npos) {
        struct pollfd pfd;
        pfd.fd = fd_; pfd.events = POLLIN; pfd.revents = 0;
        int pr = ::poll(&pfd, 1, msLeft(end));
        if (pr <= 0) { error = "no WebSocket answer (timed out)"; shutdownSocket(); return false; }
        char buf[4096];
        ssize_t got = ::recv(fd_, buf, sizeof buf, 0);
        if (got < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
        if (got <= 0) { error = "the server closed the connection during the WebSocket handshake"; shutdownSocket(); return false; }
        resp.append(buf, static_cast<std::size_t>(got));
        headerEnd = resp.find("\r\n\r\n");
        if (headerEnd == std::string::npos && resp.size() > 16384) {
            error = "not a WebSocket server (an over-long answer)";
            shutdownSocket();
            return false;
        }
    }
    // status line and headers
    std::string head = resp.substr(0, headerEnd);
    std::size_t eol = head.find("\r\n");
    std::string status = head.substr(0, eol);
    if (status.compare(0, 5, "HTTP/") != 0 || status.find(" 101") == std::string::npos) {
        error = "not a WebSocket server (it answered \"" + status + "\")";
        shutdownSocket();
        return false;
    }
    bool upg = false, conn = false, acc = false;
    std::size_t at = eol == std::string::npos ? head.size() : eol + 2;
    while (at < head.size()) {
        std::size_t e = head.find("\r\n", at);
        if (e == std::string::npos) e = head.size();
        std::string line = head.substr(at, e - at);
        at = e + 2;
        std::size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string name = lower(trim(line.substr(0, colon))), value = trim(line.substr(colon + 1));
        if (name == "upgrade" && lower(value) == "websocket") upg = true;
        if (name == "connection" && lower(value).find("upgrade") != std::string::npos) conn = true;
        if (name == "sec-websocket-accept" && value == ws::acceptKey(key)) acc = true;
    }
    if (!upg || !conn || !acc) {
        error = std::string("the WebSocket handshake was refused (") + (!acc ? "wrong Sec-WebSocket-Accept" : "missing Upgrade headers") + ")";
        shutdownSocket();
        return false;
    }
    // whatever came after the headers is already frames
    if (resp.size() > headerEnd + 4) decoder_.feed(resp.data() + headerEnd + 4, resp.size() - headerEnd - 4);
    open_ = true;
    return true;
}

bool WebSocket::writeAll(const char* p, std::size_t n, int timeoutMs) {
    Clock::time_point end = Clock::now() + std::chrono::milliseconds(timeoutMs);
    while (n > 0) {
        ssize_t w = ::send(fd_, p, n, MSG_NOSIGNAL);
        if (w > 0) { p += w; n -= static_cast<std::size_t>(w); continue; }
        if (w < 0 && errno == EINTR) continue;
        if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd pfd;
            pfd.fd = fd_; pfd.events = POLLOUT; pfd.revents = 0;
            if (::poll(&pfd, 1, msLeft(end)) <= 0) return false;
            continue;
        }
        return false;
    }
    return true;
}

bool WebSocket::sendFrame(int opcode, const std::string& payload) {
    if (fd_ < 0 || !open_) return false;
    std::uint32_t mask;
    {
        std::lock_guard<std::mutex> lk(rngMutex_);
        mask = static_cast<std::uint32_t>(rng_());
    }
    std::string f = ws::encodeFrame(opcode, payload, true, true, mask);
    std::lock_guard<std::mutex> lk(sendMutex_);
    if (opcode != ws::OpClose && closeSent_) return false;  // nothing after our close (RFC 6455 5.5.1)
    if (!writeAll(f.data(), f.size(), 5000)) { abort(); return false; }
    return true;
}

bool WebSocket::sendText(const std::string& text) { return sendFrame(ws::OpText, text); }
bool WebSocket::sendBinary(const std::string& bytes) { return sendFrame(ws::OpBinary, bytes); }
bool WebSocket::sendPing(const std::string& payload) { return sendFrame(ws::OpPing, payload.substr(0, 125)); }

void WebSocket::close(int code, const std::string& reason) {
    if (!open_ || closeSent_.exchange(true)) return;
    std::string p;
    p += static_cast<char>((code >> 8) & 0xFF);
    p += static_cast<char>(code & 0xFF);
    p += reason.substr(0, 123);
    std::uint32_t mask;
    {
        std::lock_guard<std::mutex> lk(rngMutex_);
        mask = static_cast<std::uint32_t>(rng_());
    }
    std::string f = ws::encodeFrame(ws::OpClose, p, true, true, mask);
    std::lock_guard<std::mutex> lk(sendMutex_);
    writeAll(f.data(), f.size(), 1000);
}

void WebSocket::shutdownSocket() {
    if (fd_ >= 0) ::shutdown(fd_, SHUT_RDWR);
}

void WebSocket::abort() {
    open_ = false;
    shutdownSocket();  // the descriptor stays allocated until the destructor: no reuse races
    if (wakeWrite_ >= 0) {
        char c = 1;
        ssize_t r = ::write(wakeWrite_, &c, 1);
        (void)r;
    }
}

WebSocket::Result WebSocket::receive(Message& out, int timeoutMs) {
    Clock::time_point end = Clock::now() + std::chrono::milliseconds(timeoutMs < 0 ? 0 : timeoutMs);
    for (;;) {
        // 1. whatever whole frames are already buffered
        ws::Frame f;
        std::string err;
        int r = fd_ >= 0 ? decoder_.next(f, err) : 0;
        if (r < 0) {
            closeCode_ = err == "frame too large" ? 1009 : 1002;
            closeReason_ = err;
            close(closeCode_, err);
            abort();
            return Closed;
        }
        if (r > 0) {
            if (f.masked) {  // a server must not mask (RFC 6455 5.1)
                closeCode_ = 1002; closeReason_ = "masked frame from the server";
                close(1002, closeReason_); abort();
                return Closed;
            }
            if (f.opcode == ws::OpPing) {
                if (sendFrame(ws::OpPong, f.payload)) ++pings_;
                continue;
            }
            if (f.opcode == ws::OpPong) continue;
            if (f.opcode == ws::OpClose) {
                closeCode_ = f.payload.size() >= 2
                                 ? ((static_cast<unsigned char>(f.payload[0]) << 8) | static_cast<unsigned char>(f.payload[1]))
                                 : 1005;
                closeReason_ = f.payload.size() > 2 ? f.payload.substr(2) : std::string();
                close(closeCode_ == 1005 ? 1000 : closeCode_);  // the echo (once)
                abort();
                return Closed;
            }
            int op = 0;
            std::string data;
            int a = assembler_.add(f, op, data, err);
            if (a < 0) {
                closeCode_ = 1002; closeReason_ = err;
                close(1002, err); abort();
                return Closed;
            }
            if (a > 0) {
                out.binary = op == ws::OpBinary;
                out.data.swap(data);
                return GotMessage;
            }
            continue;
        }
        if (!open_) {
            if (closeCode_ == 0) closeCode_ = 1006;
            return Closed;
        }
        // 2. wait for more bytes (or the wake pipe)
        struct pollfd pfd[2];
        pfd[0].fd = fd_; pfd[0].events = POLLIN; pfd[0].revents = 0;
        pfd[1].fd = wakeRead_; pfd[1].events = POLLIN; pfd[1].revents = 0;
        int wait = msLeft(end);
        int pr = ::poll(pfd, wakeRead_ >= 0 ? 2 : 1, wait);
        if (pr < 0 && errno == EINTR) continue;
        if (pr == 0) return Timeout;
        if (wakeRead_ >= 0 && (pfd[1].revents & POLLIN)) {
            char drain[64];
            while (::read(wakeRead_, drain, sizeof drain) > 0) {}
        }
        if (pfd[0].revents & (POLLIN | POLLHUP | POLLERR | POLLNVAL)) {
            if (rbuf_.empty()) rbuf_.resize(256 * 1024);  // (on the heap: a thread's stack may be only 512 KB)
            char* buf = &rbuf_[0];
            const std::size_t bufSize = rbuf_.size();
            for (;;) {
                ssize_t got = ::recv(fd_, buf, bufSize, 0);
                if (got > 0) {
                    bytesIn_ += static_cast<std::uint64_t>(got);
                    decoder_.feed(buf, static_cast<std::size_t>(got));
                    if (static_cast<std::size_t>(got) < bufSize) break;
                    continue;
                }
                if (got < 0 && errno == EINTR) continue;
                if (got < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
                // 0 = the peer closed; < 0 = an error: either way, the end (after what's buffered)
                open_ = false;
                break;
            }
            continue;
        }
        if (!open_) continue;
        if (wait == 0) return Timeout;
    }
}

}  // namespace sdrc
