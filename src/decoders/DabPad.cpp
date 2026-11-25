// ============================================================================
//  offgrid-decoders - reusable C++11 decoders for radio text and picture
//  modes (part of OFFgrid-SDR TUI, https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/decoders/DabPad.cpp
//  Layer:  Library liboffgrid-decoders (internal)
//
//  What this file does
//  -------------------
//  What DAB carries besides the sound (declared in DabInternal.h):
//    - Pad: programme-associated data (ETSI EN 300 401 7.4), as welle.io
//      reads it.  PAD comes with each audio frame: MP2 at the end of the
//      frame, DAB+ in a data stream element at the start of an audio unit.
//      The X-PAD is read in reverse; its contents list (up to 4 entries,
//      each a type and a length) splits it into subfields: 2 and 3 are
//      dynamic label (DLS) segments (start, continuation); 1 announces the
//      next MOT data group's length, 12 starts it and 13 continues it.
//      Label segments carry a toggle, first/last flags, a segment number
//      (0-7) and up to 16 characters, with a CRC; a label is complete when
//      segments 0 to the last have come with the same toggle.
//    - Slides: the SlideShow's MOT reassembly (EN 301 234, header mode;
//      ETSI TS 101 499): MSC data groups (header, segment field, user
//      access with the transport ID, CRC); type 3 the header (body size,
//      content type, name), type 4 the body; a JPEG or PNG is ready when
//      both are whole.
//    - Spi: the multiplex's SPI service, station logos (EN 300 401 5.3.2
//      packets, 5.3.3 data groups; EN 301 234 MOT, directory mode; TS 102
//      818): packets (24/48/72/96 bytes; address, first/last, CRC) join into
//      data groups; type 6 is the directory, type 4 bodies.  The directory
//      lists every file: transport ID, size, content type (2 = image: 1
//      JPEG, 3 PNG) and name (parameter 12); the picture's own header gives
//      its pixel size.
//
//  How it fits in
//  --------------
//  A port of OFFgrid-SDR's JS (diddy-boy, https://github.com/solariun/
//  offgrid-sdr, index.html, "dablib": DAB_CI_LEN, DabSlides, DabPad,
//  DabSpi).  Used by Dab.cpp for the chosen station (Pad) and the SPI
//  sub-channel (Spi).  Additions to the JS: every index is checked against
//  the data's length (the JS reads past an array's end as "undefined").
//
//  Contributing
//  ------------
//  Keep in step with the JS; mark changes "Addition to the JS".  Run
//  tests/test_decoders_dab.cpp (it checks the demo's DLS, slide and logos).
// ============================================================================
#include "DabInternal.h"

namespace dec {
namespace dab {

namespace {

const int kCiLen[8] = {4, 6, 8, 12, 16, 24, 32, 48};

// A MOT parameter block (from index `r` to `end`): the ContentName (parameter 12) or "".
std::string motName(const unsigned char* h, std::size_t r, std::size_t end) {
    std::string name;
    while (r < end) {
        const int pli = h[r] >> 6, id = h[r] & 63;
        ++r;
        long len = pli == 0 ? 0 : pli == 1 ? 1 : pli == 2 ? 4 : -1;
        if (len < 0) {
            if (r >= end) break;
            if (h[r] & 0x80) {
                if (r + 1 >= end) break;
                len = ((h[r] & 127) << 8) | h[r + 1];
                r += 2;
            } else {
                len = h[r];
                r += 1;
            }
        }
        if (id == 12 && len > 1) {
            name.clear();
            for (std::size_t i = r + 1; i < r + static_cast<std::size_t>(len) && i < end; ++i) appendUtf8(name, h[i]);
        }
        r += static_cast<std::size_t>(len);
    }
    return name;
}

// An MSC data group's header (EN 300 401 5.3.3): false if too short or its CRC fails.
struct GroupHead {
    int type, seg, tid;
    bool last;
    std::size_t body, bodyEnd;   // the segment's data: [body, bodyEnd)
};
bool readGroup(const unsigned char* g, std::size_t n, GroupHead& h) {
    if (n < 4) return false;
    const int ext = g[0] >> 7, crcF = (g[0] >> 6) & 1, segF = (g[0] >> 5) & 1, uaF = (g[0] >> 4) & 1;
    h.type = g[0] & 15;
    if (crcF && !crcOk(g, n)) return false;
    std::size_t q = 2 + (ext ? 2 : 0);
    h.last = false;
    h.seg = 0;
    h.tid = -1;
    if (segF) {
        if (q + 2 > n) return false;
        h.last = (g[q] >> 7) != 0;
        h.seg = ((g[q] & 127) << 8) | g[q + 1];
        q += 2;
    }
    if (uaF) {
        if (q + 1 > n) return false;
        const int len = g[q] & 15;
        if ((g[q] >> 4) & 1) {
            if (q + 3 > n) return false;
            h.tid = (g[q + 1] << 8) | g[q + 2];
        }
        q += 1 + static_cast<std::size_t>(len);
    }
    const std::size_t end = n - (crcF ? 2 : 0);
    if (q + 2 > end) return false;
    // the MOT segment header: 3 bits repetition count, 13 bits size
    const std::size_t size = static_cast<std::size_t>(((g[q] & 31) << 8) | g[q + 1]);
    h.body = q + 2;
    h.bodyEnd = h.body + size < end ? h.body + size : end;
    return true;
}

// segments 0..last all present: joined
bool joined(const std::map<int, std::vector<unsigned char> >& m, int last, std::vector<unsigned char>& out) {
    if (last < 0 || static_cast<int>(m.size()) != last + 1) return false;
    out.clear();
    for (int i = 0; i <= last; ++i) {
        std::map<int, std::vector<unsigned char> >::const_iterator it = m.find(i);
        if (it == m.end()) return false;
        out.insert(out.end(), it->second.begin(), it->second.end());
    }
    return true;
}

}  // namespace

// ---- the SlideShow (JS DabSlides.group)
void Slides::group(const unsigned char* g, std::size_t n) {
    GroupHead gh;
    if (!readGroup(g, n, gh)) return;
    if ((gh.type != 3 && gh.type != 4) || gh.tid < 0) return;
    std::vector<unsigned char> data(g + gh.body, g + gh.bodyEnd);
    Obj* o = 0;
    for (std::size_t i = 0; i < objs_.size(); ++i)
        if (objs_[i].tid == gh.tid) o = &objs_[i];
    if (!o) {
        objs_.push_back(Obj());
        o = &objs_.back();
        o->tid = gh.tid;
    } else if (gh.type == 3 && gh.seg == 0 && o->hl >= 0 && o->body.empty()) {
        *o = Obj();   // a header again with no body yet: a new object
        o->tid = gh.tid;
    }
    if (gh.type == 3) {
        o->hdr[gh.seg].swap(data);
        if (gh.last) o->hl = gh.seg;
    } else {
        o->body[gh.seg].swap(data);
        if (gh.last) o->bl = gh.seg;
    }
    const int tid = o->tid;
    if (objs_.size() > 8) objs_.erase(objs_.begin());   // (a few at a time, at most)
    std::size_t k = 0;
    while (k < objs_.size() && objs_[k].tid != tid) ++k;
    if (k == objs_.size()) return;
    std::vector<unsigned char> h, b;
    if (!joined(objs_[k].hdr, objs_[k].hl, h) || !joined(objs_[k].body, objs_[k].bl, b)) return;
    objs_.erase(objs_.begin() + static_cast<std::ptrdiff_t>(k));
    if (h.size() < 7) return;
    const int ctype = (h[5] >> 1) & 63, csub = ((h[5] & 1) << 8) | h[6];
    const std::string name = motName(&h[0], 7, h.size());
    if (ctype == 2 && (csub == 1 || csub == 3)) {
        Picture p;
        p.name = name;
        p.mime = csub == 3 ? "image/png" : "image/jpeg";
        p.w = p.h = 0;
        p.bytes.swap(b);
        ready.push_back(p);
    }
}

// ---- PAD (JS DabPad)
Pad::Pad()
    : changed(false), lastType_(-1), lastLen_(0), dgOn_(false), dgNeed_(0), toggle_(-1), charset_(0), motLen_(0),
      motOn_(false), motNeed_(0) {}

void Pad::process(const unsigned char* xd, int xlen, bool exact, int f0, int f1) {
    const int n = xlen < 196 ? (xlen > 0 ? xlen : 0) : 196;
    for (int i = 0; i < n; ++i) x_[i] = xd[xlen - 1 - i];
    const int ftype = f0 >> 6, ind = (f0 & 0x30) >> 4;
    const bool ciFlag = (f1 & 0x02) != 0;
    const int prevType = lastType_, prevLen = lastLen_;
    lastType_ = -1;
    lastLen_ = 0;
    int ciType[4], ciLen[4], nci = 0, cisLen = -1;
    if (ftype == 0) {
        if (ciFlag) {
            if (ind == 1) {   // short X-PAD: one contents indicator, 3 bytes of data
                if (n < 1) return;
                const int t = x_[0] & 31;
                if (t) {
                    cisLen = 1;
                    ciType[nci] = t;
                    ciLen[nci++] = 3;
                }
            } else if (ind == 2) {   // variable X-PAD: up to 4 indicators, an end marker (type 0) after fewer
                cisLen = 0;
                for (int i = 0; i < 4; ++i) {
                    if (n < i + 1) return;
                    const int c = x_[i];
                    ++cisLen;
                    if (!(c & 31)) break;
                    ciType[nci] = c & 31;
                    ciLen[nci++] = kCiLen[c >> 5];
                }
            }
        } else if ((ind == 1 || ind == 2) && prevType != -1) {   // no indicator: the last subfield continues
            cisLen = 0;
            ciType[nci] = prevType;
            ciLen[nci++] = prevLen;
        }
    }
    if (!nci) return;
    int ann = cisLen;
    for (int i = 0; i < nci; ++i) ann += ciLen[i];
    if (ann > n) return;
    if (exact && ann < n) return;   // (DAB+: announced shorter than given: discarded, TS 102 563 5.4.3)
    int off = cisLen, cont = -1;
    for (int i = 0; i < nci; ++i) {
        const int t = ciType[i], len = ciLen[i];
        if (t == 2 || t == 3) {
            dlSubfield(t == 2, x_ + off, len);
            cont = 3;
        } else if (t == 1 && len >= 4) {
            // (the SlideShow: type 1 announces the next MOT data group's length (14 bits, then a CRC))
            const unsigned char* d = x_ + off;
            if (crc16(d, 2) == ((static_cast<unsigned>(d[2]) << 8) | d[3])) motLen_ = ((d[0] & 0x3f) << 8) | d[1];
            cont = -1;
        } else if (t == 12 || t == 13) {
            motSubfield(t == 12, x_ + off, len);
            cont = 13;
        }
        off += len;
    }
    lastType_ = cont;
    lastLen_ = off;
}

void Pad::motSubfield(bool start, const unsigned char* d, int n) {
    if (start) {
        if (!motLen_) {
            motOn_ = false;
            return;
        }
        motOn_ = true;
        mot_.clear();
        motNeed_ = static_cast<std::size_t>(motLen_);
        motLen_ = 0;
    }
    if (!motOn_) return;
    for (int i = 0; i < n && mot_.size() < motNeed_; ++i) mot_.push_back(d[i]);
    if (mot_.size() >= motNeed_) {
        motOn_ = false;
        slides.group(&mot_[0], mot_.size());
    }
}

void Pad::dlSubfield(bool start, const unsigned char* d, int n) {   // a data group, gathered across subfields
    if (start) {
        dgOn_ = true;
        dg_.clear();
        dgNeed_ = 2;
    } else if (!dgOn_ || dg_.empty())
        return;
    if (dg_.size() >= dgNeed_) return;
    for (int i = 0; i < n && dg_.size() < 256; ++i) dg_.push_back(d[i]);   // (every byte kept: the length is known only later)
    if (dg_.size() < dgNeed_) return;
    const int p0 = dg_[0];
    const bool cmd = (p0 & 0x10) != 0;
    const std::size_t flen = cmd ? 0 : static_cast<std::size_t>((p0 & 15) + 1), real = 2 + flen;
    if (cmd && (p0 & 15) != 1) {   // (commands other than "remove label" ignored here)
        dgOn_ = false;
        return;
    }
    if (dgNeed_ < real + 2) {
        dgNeed_ = real + 2;
        if (dg_.size() < dgNeed_) return;
    }
    std::vector<unsigned char> by(dg_.begin(), dg_.begin() + static_cast<std::ptrdiff_t>(real + 2));
    dgOn_ = false;
    if (!crcOk(&by[0], real + 2)) return;   // (a damaged segment: dropped)
    if (cmd) {
        label.clear();
        segs_.clear();
        return;
    }
    const int tog = p0 >> 7, first = (p0 >> 6) & 1, last = (p0 >> 5) & 1, num = first ? 0 : (by[1] >> 4) & 7;
    if (tog != toggle_) {
        segs_.clear();
        toggle_ = tog;
    }
    if (first) charset_ = by[1] >> 4;
    Seg& s = segs_[num];
    s.last = last != 0;
    s.chars.assign(by.begin() + 2, by.begin() + static_cast<std::ptrdiff_t>(real));
    std::vector<unsigned char> all;
    for (int i = 0; i < 8; ++i) {
        std::map<int, Seg>::const_iterator it = segs_.find(i);
        if (it == segs_.end()) return;
        all.insert(all.end(), it->second.chars.begin(), it->second.chars.end());
        if (it->second.last) break;
        if (i == 7) return;
    }
    const std::string t = text(all.empty() ? 0 : &all[0], all.size(), charset_);
    if (t != label) {
        label = t;
        changed = true;
    }
}

// ---- SPI (JS DabSpi)
Spi::Spi(int addr) : packets(0), groups(0), addr_(addr), curOn_(false), haveDir_(false) {}

void Spi::cif(const unsigned char* by, std::size_t n) {
    static const std::size_t kLen[4] = {24, 48, 72, 96};
    for (std::size_t p = 0; p + 24 <= n;) {
        const std::size_t len = kLen[by[p] >> 6];
        if (p + len > n) break;
        const int addr = ((by[p] & 3) << 8) | by[p + 1];
        if (addr == addr_ && crcOk(by + p, len)) {
            ++packets;
            const bool first = ((by[p] >> 3) & 1) != 0, last = ((by[p] >> 2) & 1) != 0;
            std::size_t m = by[p + 2] & 127;
            if (m > len - 5) m = len - 5;
            if (first) {
                curOn_ = true;
                cur_.clear();
            }
            if (curOn_) {
                cur_.insert(cur_.end(), by + p + 3, by + p + 3 + m);
                if (cur_.size() > 9000) curOn_ = false;
            }
            if (last && curOn_) {
                curOn_ = false;
                std::vector<unsigned char> g;
                g.swap(cur_);
                group(&g[0], g.size());
            }
        }
        p += len;
    }
}

void Spi::group(const unsigned char* g, std::size_t n) {
    GroupHead gh;
    if (!readGroup(g, n, gh)) return;
    ++groups;
    if ((gh.type != 4 && gh.type != 6) || gh.tid < 0) return;
    const std::pair<int, int> key(gh.type, gh.tid);
    std::map<std::pair<int, int>, Obj>::iterator it = objs_.find(key);
    if (it == objs_.end()) {
        if (gh.type == 4 && haveDir_ && !dir_.count(gh.tid)) return;
        Obj o;
        o.type = gh.type;
        o.tid = gh.tid;
        it = objs_.insert(std::make_pair(key, o)).first;
    }
    it->second.segs[gh.seg].assign(g + gh.body, g + gh.bodyEnd);
    if (gh.last) it->second.last = gh.seg;
    if (it->second.last >= 0 && static_cast<int>(it->second.segs.size()) == it->second.last + 1) complete(it->second);
}

void Spi::complete(Obj& o) {
    std::vector<unsigned char> all;
    if (!joined(o.segs, o.last, all)) return;
    const int type = o.type, tid = o.tid;
    objs_.erase(std::make_pair(type, tid));   // (o is gone from here on)
    if (type == 6) {
        directory(all);
        return;
    }
    if (done_.count(tid)) return;
    std::map<int, DirEntry>::const_iterator e = dir_.find(tid);
    if (haveDir_ && e != dir_.end())
        file(tid, e->second, all);
    else
        waiting_[tid].swap(all);   // (the directory still to come)
}

void Spi::directory(const std::vector<unsigned char>& d) {
    const std::size_t n = d.size();
    if (n < 13) return;
    const int num = (d[4] << 8) | d[5], extLen = (d[11] << 8) | d[12];
    std::size_t q = 13 + static_cast<std::size_t>(extLen);
    std::map<int, DirEntry> dir;
    for (int k = 0; k < num && q + 9 <= n; ++k) {
        const int tid = (d[q] << 8) | d[q + 1];
        q += 2;
        const std::size_t hdr = static_cast<std::size_t>(((d[q + 3] & 15) << 9) | (d[q + 4] << 1) | (d[q + 5] >> 7));
        DirEntry e;
        e.ctype = (d[q + 5] >> 1) & 63;
        e.csub = ((d[q + 5] & 1) << 8) | d[q + 6];
        const std::size_t end = q + hdr < n ? q + hdr : n;
        e.name = motName(&d[0], q + 7, end);
        dir[tid] = e;
        if (hdr < 7) break;   // (Addition to the JS: a header too short to hold its own core ends the directory)
        q = q + hdr;
    }
    dir_.swap(dir);
    haveDir_ = true;
    for (std::map<std::pair<int, int>, Obj>::iterator it = objs_.begin(); it != objs_.end();) {   // (files no longer listed)
        if (it->first.first == 4 && !dir_.count(it->first.second))
            objs_.erase(it++);
        else
            ++it;
    }
    std::map<int, std::vector<unsigned char> > w;
    w.swap(waiting_);
    for (std::map<int, std::vector<unsigned char> >::const_iterator it = w.begin(); it != w.end(); ++it) {
        std::map<int, DirEntry>::const_iterator e = dir_.find(it->first);
        if (e != dir_.end()) file(it->first, e->second, it->second);
    }
}

void Spi::file(int tid, const DirEntry& e, const std::vector<unsigned char>& b) {
    done_[tid] = true;
    if (e.ctype != 2 || (e.csub != 1 && e.csub != 3)) return;   // (images only: JPEG and PNG)
    Picture p;
    p.w = p.h = 0;
    if (e.csub == 3 && b.size() > 24) {
        p.w = static_cast<int>((static_cast<unsigned>(b[16]) << 24) | (b[17] << 16) | (b[18] << 8) | b[19]);
        p.h = static_cast<int>((static_cast<unsigned>(b[20]) << 24) | (b[21] << 16) | (b[22] << 8) | b[23]);
    } else
        for (std::size_t i = 2; i + 9 < b.size();) {
            if (b[i] != 0xff) break;
            const int m = b[i + 1], len = (b[i + 2] << 8) | b[i + 3];
            if (m >= 0xc0 && m <= 0xc3) {
                p.h = (b[i + 5] << 8) | b[i + 6];
                p.w = (b[i + 7] << 8) | b[i + 8];
                break;
            }
            i += 2 + static_cast<std::size_t>(len);
        }
    p.name = e.name;
    p.mime = e.csub == 3 ? "image/png" : "image/jpeg";
    p.bytes = b;
    ready.push_back(p);
}

}  // namespace dab
}  // namespace dec
