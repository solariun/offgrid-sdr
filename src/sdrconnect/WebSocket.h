// ============================================================================
//  offgrid-sdrconnect - an SDRplay RSP through SDRplay's SDRconnect WebSocket
//  API, in C++11 (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/sdrconnect/WebSocket.h
//  Layer:  Library liboffgrid-sdrconnect
//
//  What this file does
//  -------------------
//  A minimal RFC 6455 WebSocket client over plain POSIX sockets, with no
//  external library (no libwebsockets, no OpenSSL):
//    - sdrc::ws::sha1, base64Encode, acceptKey: the handshake's arithmetic
//      (SHA-1 per FIPS 180-4, base64 per RFC 4648), written here;
//    - sdrc::ws::encodeFrame / FrameDecoder / MessageAssembler: the framing
//      (text and binary, the client's masking, 7/16/64-bit lengths,
//      fragmentation, ping/pong/close), usable on either side of a
//      connection (the tests' mock server uses them too);
//    - sdrc::WebSocket: one ws:// connection: TCP connect (getaddrinfo, with
//      a timeout), the HTTP/1.1 Upgrade with Sec-WebSocket-Key, then
//      receive() (poll with a timeout: pings answered, fragments joined,
//      close answered) and thread-safe send.
//  Only ws:// is supported: SDRconnect's API is plain ws on port 5454 (the
//  browser version can't use wss either, which is why OFFgrid-SDR must be
//  opened from a file or an http:// page to reach it).
//
//  How it fits in
//  --------------
//  sdrc::Client (SdrConnect.h) owns a WebSocket and reads it on its own
//  thread; in OFFgrid-SDR's browser version (by diddy-boy) the browser's
//  own WebSocket object plays this part, so this whole file is an
//  "Addition to the JS".
//
//  Contributing
//  ------------
//  Keep it small and dependency-free.  Anything SDRconnect-specific belongs
//  in SdrConnect.cpp, not here.  The RFC examples in tests/test_sdrconnect.cpp
//  must keep passing.
// ============================================================================
#ifndef OFFGRID_SDRCONNECT_WEBSOCKET_H
#define OFFGRID_SDRCONNECT_WEBSOCKET_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <random>
#include <string>
#include <vector>

namespace sdrc {
namespace ws {

// Frame opcodes (RFC 6455 section 5.2).
enum Opcode {
    OpContinuation = 0x0,
    OpText = 0x1,
    OpBinary = 0x2,
    OpClose = 0x8,
    OpPing = 0x9,
    OpPong = 0xA
};

// SHA-1 of data: the 20-byte raw digest.
std::string sha1(const std::string& data);
// Lower-case hexadecimal of raw bytes (for the tests and for logs).
std::string toHex(const std::string& bytes);
// Standard base64 with '=' padding (RFC 4648 section 4).
std::string base64Encode(const std::string& bytes);
// Sec-WebSocket-Accept for a Sec-WebSocket-Key: base64(sha1(key + GUID)).
std::string acceptKey(const std::string& secWebSocketKey);

// One frame on the wire.  fin=false for every fragment but the last.
// masked: a client's frames must be masked (RFC 6455 5.3); maskKey is the
// 4-byte key, most significant byte first (0x37fa213d is 37 fa 21 3d).
std::string encodeFrame(int opcode, const std::string& payload, bool fin = true,
                        bool masked = false, std::uint32_t maskKey = 0);

// One decoded frame (payload already unmasked).
struct Frame {
    Frame() : fin(true), opcode(0), masked(false) {}
    bool fin;
    int opcode;
    bool masked;
    std::string payload;
};

// Incremental frame decoder: feed() whatever bytes arrived, then call
// next() until it returns 0.  next(): 1 = a frame in out, 0 = need more
// bytes, -1 = a protocol error (error says which; the connection should be
// closed with code 1002, or 1009 for a frame over maxPayload).
class FrameDecoder {
public:
    explicit FrameDecoder(std::uint64_t maxPayload = 64u * 1024u * 1024u)
        : maxPayload_(maxPayload), pos_(0) {}
    void feed(const char* data, std::size_t n);
    int next(Frame& out, std::string& error);
    std::size_t buffered() const { return buf_.size() - pos_; }
    void reset() { buf_.clear(); pos_ = 0; }

private:
    std::uint64_t maxPayload_;
    std::string buf_;
    std::size_t pos_;  // start of the unparsed bytes in buf_
};

// Joins fragments into messages.  add() a data or continuation frame:
// returns 1 when a whole message is ready (opcode = OpText or OpBinary,
// data = the joined payload), 0 while fragments are still coming, -1 on a
// protocol error (a continuation with nothing to continue, a new message
// before the last finished, a message over maxMessage).  Control frames
// (close, ping, pong) are never fragmented and are not given to it.
class MessageAssembler {
public:
    explicit MessageAssembler(std::size_t maxMessage = 64u * 1024u * 1024u)
        : maxMessage_(maxMessage), opcode_(-1) {}
    int add(const Frame& f, int& opcode, std::string& data, std::string& error);
    void reset() { opcode_ = -1; parts_.clear(); }

private:
    std::size_t maxMessage_;
    int opcode_;  // the message being assembled, -1 = none
    std::string parts_;
};

}  // namespace ws

// One ws:// connection to a server, as a client.
// Thread-safety: send*() and close() may be called from any thread;
// receive() from one thread at a time (the reader).
class WebSocket {
public:
    enum Result { GotMessage, Timeout, Closed };

    struct Message {
        Message() : binary(false) {}
        bool binary;
        std::string data;
    };

    WebSocket();
    ~WebSocket();

    // Connects to ws://host:port/path, with the whole of TCP connect and the
    // HTTP Upgrade within timeoutMs.  false with error on failure.
    bool connect(const std::string& host, int port, const std::string& path,
                 int timeoutMs, std::string& error);
    bool isOpen() const { return open_.load(); }

    // Thread-safe sends (each frame masked with a fresh random key).
    bool sendText(const std::string& text);
    bool sendBinary(const std::string& bytes);
    bool sendPing(const std::string& payload = std::string());
    // Sends a close frame (once) and shuts the socket down for writing;
    // receive() then reports Closed once the server's close arrives (or
    // the socket ends).  close() from another thread wakes a receive().
    void close(int code = 1000, const std::string& reason = std::string());
    // Drops the connection at once, without a close frame.
    void abort();

    // Waits up to timeoutMs for a whole message.  Pings are answered with
    // pongs, fragments joined, a server's close answered; Closed once the
    // connection has ended (closeCode()/closeReason() then say why: 1006 =
    // dropped without a close frame).
    Result receive(Message& out, int timeoutMs);

    int closeCode() const { return closeCode_; }
    std::string closeReason() const { return closeReason_; }
    std::uint64_t bytesReceived() const { return bytesIn_.load(); }
    std::uint64_t pingsAnswered() const { return pings_.load(); }

private:
    WebSocket(const WebSocket&);             // not copyable
    WebSocket& operator=(const WebSocket&);
    bool sendFrame(int opcode, const std::string& payload);
    bool writeAll(const char* p, std::size_t n, int timeoutMs);
    void shutdownSocket();

    int fd_;
    std::atomic<bool> open_;
    std::atomic<bool> closeSent_;
    std::mutex sendMutex_;
    std::mutex rngMutex_;
    std::mt19937 rng_;
    ws::FrameDecoder decoder_;
    ws::MessageAssembler assembler_;
    int closeCode_;
    std::string closeReason_;
    std::atomic<std::uint64_t> bytesIn_;
    std::atomic<std::uint64_t> pings_;
    std::vector<char> rbuf_;    // the reader's receive buffer
    int wakeRead_, wakeWrite_;  // a pipe: close() from another thread wakes receive()
};

}  // namespace sdrc

#endif
