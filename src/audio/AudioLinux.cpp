// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/audio/AudioLinux.cpp
//  Layer:  Sound output (Linux)
//
//  What this file does
//  -------------------
//  audio::AudioOutput on Linux, with no library at all, so a fully static
//  binary still makes sound:
//
//  1. ALSA kernel interface: opens /dev/snd/pcmC<card>D<dev>p and sets it up
//     with the same ioctls alsa-lib (and tinyalsa) use: HW_PARAMS (S16_LE,
//     2 channels, the rate, 4 periods of 1024 frames), SW_PARAMS, PREPARE,
//     then WRITEI_FRAMES from a thread.  An underrun (-EPIPE) re-prepares.
//     Rates tried: the one asked, then 48000, then 44100 (rate() tells).
//  2. When the device is busy (a sound server - PipeWire, PulseAudio - owns
//     it, the usual desktop case) or missing: pipes raw S16_LE stereo to
//     the first of pw-cat, pacat, aplay found in PATH (a child process: the
//     binary stays static).
//  device: "" (auto: ALSA card 0 device 0, then pipe), "hw:C,D" (only that
//  ALSA device), "pipe:<command>" (only that command; it must read raw
//  S16_LE stereo at the rate from standard input).
//
//  Contributing
//  ------------
//  The struct layouts come from <sound/asound.h> (the kernel's own header,
//  in linux-libc-dev / linux-headers); never redefine them here.
// ============================================================================
#ifdef __linux__

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <sound/asound.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <thread>
#include <vector>

#include "AudioOutput.h"

namespace audio {

namespace {

const int kPeriodFrames = 1024;
const int kPeriods = 4;

// ---- hw_params helpers (masks and intervals, as alsa-lib fills them)
snd_mask* paramMask(snd_pcm_hw_params* p, int n) { return &p->masks[n - SNDRV_PCM_HW_PARAM_FIRST_MASK]; }
snd_interval* paramInterval(snd_pcm_hw_params* p, int n) {
    return &p->intervals[n - SNDRV_PCM_HW_PARAM_FIRST_INTERVAL];
}

void paramInit(snd_pcm_hw_params* p) {
    std::memset(p, 0, sizeof(*p));
    for (int n = SNDRV_PCM_HW_PARAM_FIRST_MASK; n <= SNDRV_PCM_HW_PARAM_LAST_MASK; ++n) {
        snd_mask* m = paramMask(p, n);
        m->bits[0] = ~0u;
        m->bits[1] = ~0u;
    }
    for (int n = SNDRV_PCM_HW_PARAM_FIRST_INTERVAL; n <= SNDRV_PCM_HW_PARAM_LAST_INTERVAL; ++n) {
        snd_interval* i = paramInterval(p, n);
        i->min = 0;
        i->max = ~0u;
    }
    p->rmask = ~0u;
    p->cmask = 0;
    p->info = ~0u;
}

void paramSetMask(snd_pcm_hw_params* p, int n, unsigned bit) {
    snd_mask* m = paramMask(p, n);
    std::memset(m->bits, 0, sizeof(m->bits));
    m->bits[bit >> 5] |= 1u << (bit & 31);
}

void paramSetInt(snd_pcm_hw_params* p, int n, unsigned value) {
    snd_interval* i = paramInterval(p, n);
    i->min = value;
    i->max = value;
    i->integer = 1;
}

void paramSetMin(snd_pcm_hw_params* p, int n, unsigned value) {
    snd_interval* i = paramInterval(p, n);
    i->min = value;
}

unsigned paramGetInt(snd_pcm_hw_params* p, int n) { return paramInterval(p, n)->max; }

class LinuxOutput : public AudioOutput {
public:
    explicit LinuxOutput(const std::string& device)
        : device_(device), ring_(48000), fd_(-1), pipe_(0), rate_(48000), underruns_(0), running_(false), primed_(false) {}
    ~LinuxOutput() { close(); }

    bool open(int rate, std::string& error) {
        close();
        if (rate <= 0) rate = 48000;
        std::string alsaError, pipeError;
        bool tryAlsa = device_.empty() || device_.compare(0, 3, "hw:") == 0;
        bool tryPipe = device_.empty() || device_.compare(0, 5, "pipe:") == 0;
        if (tryAlsa) {
            int card = 0, dev = 0;
            if (!device_.empty() && std::sscanf(device_.c_str() + 3, "%d,%d", &card, &dev) < 1) {
                error = "bad ALSA device \"" + device_ + "\" (use hw:CARD,DEVICE, e.g. hw:0,0)";
                return false;
            }
            if (openAlsa(card, dev, rate, alsaError)) return startThread();
        }
        if (tryPipe) {
            if (openPipe(device_.empty() ? std::string() : device_.substr(5), rate, pipeError)) return startThread();
        }
        error = tryAlsa ? alsaError : std::string();
        if (!pipeError.empty()) error += (error.empty() ? "" : "; ") + pipeError;
        return false;
    }

    void close() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (fd_ >= 0) {
            ioctl(fd_, SNDRV_PCM_IOCTL_DROP);
            ::close(fd_);
            fd_ = -1;
        }
        if (pipe_) {
            pclose(pipe_);
            pipe_ = 0;
        }
        ring_.clear();
    }

    int rate() const { return rate_; }
    std::string name() const { return name_; }
    void write(const float* stereo, std::size_t frames) { ring_.push(stereo, frames); }
    double queuedSeconds() const { return static_cast<double>(ring_.size() + kPeriodFrames * kPeriods / 2) / rate_; }
    unsigned underruns() const { return underruns_; }
    void flush() {
        ring_.clear();
        primed_ = false;
    }

private:
    bool openAlsa(int card, int dev, int rate, std::string& error) {
        char path[64];
        std::snprintf(path, sizeof(path), "/dev/snd/pcmC%dD%dp", card, dev);
        fd_ = ::open(path, O_RDWR | O_CLOEXEC);
        if (fd_ < 0) {
            error = std::string(path) + ": " + std::strerror(errno);
            return false;
        }
        const int rates[3] = {rate, 48000, 44100};
        bool ok = false;
        for (int k = 0; k < 3 && !ok; ++k) {
            if (k > 0 && rates[k] == rates[0]) continue;
            snd_pcm_hw_params hw;
            paramInit(&hw);
            paramSetMask(&hw, SNDRV_PCM_HW_PARAM_ACCESS, SNDRV_PCM_ACCESS_RW_INTERLEAVED);
            paramSetMask(&hw, SNDRV_PCM_HW_PARAM_FORMAT, SNDRV_PCM_FORMAT_S16_LE);
            paramSetMask(&hw, SNDRV_PCM_HW_PARAM_SUBFORMAT, SNDRV_PCM_SUBFORMAT_STD);
            paramSetMin(&hw, SNDRV_PCM_HW_PARAM_PERIOD_SIZE, kPeriodFrames);
            paramSetInt(&hw, SNDRV_PCM_HW_PARAM_SAMPLE_BITS, 16);
            paramSetInt(&hw, SNDRV_PCM_HW_PARAM_FRAME_BITS, 32);
            paramSetInt(&hw, SNDRV_PCM_HW_PARAM_CHANNELS, 2);
            paramSetInt(&hw, SNDRV_PCM_HW_PARAM_PERIODS, kPeriods);
            paramSetInt(&hw, SNDRV_PCM_HW_PARAM_RATE, static_cast<unsigned>(rates[k]));
            if (ioctl(fd_, SNDRV_PCM_IOCTL_HW_PARAMS, &hw) == 0) {
                ok = true;
                rate_ = rates[k];
                periodFrames_ = static_cast<int>(paramGetInt(&hw, SNDRV_PCM_HW_PARAM_PERIOD_SIZE));
                periods_ = static_cast<int>(paramGetInt(&hw, SNDRV_PCM_HW_PARAM_PERIODS));
            }
        }
        if (!ok) {
            error = std::string(path) + ": no usable format (S16_LE stereo at 48/44.1 kHz): " + std::strerror(errno);
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        snd_pcm_sw_params sw;
        std::memset(&sw, 0, sizeof(sw));
        snd_pcm_uframes_t buffer = static_cast<snd_pcm_uframes_t>(periodFrames_) * periods_;
        sw.tstamp_mode = SNDRV_PCM_TSTAMP_NONE;
        sw.period_step = 1;
        sw.avail_min = static_cast<snd_pcm_uframes_t>(periodFrames_);
        sw.start_threshold = buffer / 2;  // start once half the buffer is full
        sw.stop_threshold = buffer;
        sw.xfer_align = static_cast<snd_pcm_uframes_t>(periodFrames_ / 2);
        // The boundary must be a power-of-two multiple of the buffer that
        // fits the type (alsa-lib's rule).
        snd_pcm_uframes_t boundary = buffer;
        const snd_pcm_uframes_t limit = static_cast<snd_pcm_uframes_t>(std::numeric_limits<long>::max()) - buffer;
        while (boundary * 2 <= limit) boundary *= 2;
        sw.boundary = boundary;
        if (ioctl(fd_, SNDRV_PCM_IOCTL_SW_PARAMS, &sw) != 0 || ioctl(fd_, SNDRV_PCM_IOCTL_PREPARE) != 0) {
            error = std::string(path) + ": setup failed: " + std::strerror(errno);
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        char label[96];
        std::snprintf(label, sizeof(label), "ALSA hw:%d,%d %d Hz", card, dev, rate_);
        name_ = label;
        return true;
    }

    static bool inPath(const std::string& program) {
        const char* path = std::getenv("PATH");
        std::string p = path ? path : "/usr/bin:/bin";
        size_t start = 0;
        while (start <= p.size()) {
            size_t end = p.find(':', start);
            std::string dir = p.substr(start, end == std::string::npos ? std::string::npos : end - start);
            if (!dir.empty() && access((dir + "/" + program).c_str(), X_OK) == 0) return true;
            if (end == std::string::npos) break;
            start = end + 1;
        }
        return false;
    }

    bool openPipe(const std::string& command, int rate, std::string& error) {
        std::string cmd = command;
        std::string r = std::to_string(rate);
        if (cmd.empty()) {
            if (inPath("pw-cat")) cmd = "pw-cat --playback --format s16 --rate " + r + " --channels 2 -";
            else if (inPath("pacat")) cmd = "pacat --playback --format=s16le --rate=" + r + " --channels=2 --latency-msec=60";
            else if (inPath("aplay")) cmd = "aplay -q -t raw -f S16_LE -c 2 -r " + r + " -";
            else {
                error = "no sound device free and none of pw-cat, pacat, aplay found (use --no-audio, or --audio pipe:<cmd>)";
                return false;
            }
        }
        signal(SIGPIPE, SIG_IGN);  // the player quitting must not kill us
        pipe_ = popen((cmd + " 2>/dev/null").c_str(), "w");
        if (!pipe_) {
            error = "cannot start \"" + cmd + "\": " + std::strerror(errno);
            return false;
        }
        rate_ = rate;
        periodFrames_ = kPeriodFrames;
        periods_ = kPeriods;
        name_ = "pipe: " + cmd.substr(0, cmd.find(' '));
        return true;
    }

    bool startThread() {
        running_ = true;
        thread_ = std::thread(&LinuxOutput::loop, this);
        return true;
    }

    // Moves sound from the ring to the device, a period at a time.
    void loop() {
        std::vector<float> f(static_cast<size_t>(periodFrames_) * 2);
        std::vector<short> s(static_cast<size_t>(periodFrames_) * 2);
        while (running_) {
            // Play only once two periods are queued (start, flush, ran dry).
            size_t got = 0;
            if (!primed_ && ring_.size() >= static_cast<size_t>(periodFrames_) * 2) primed_ = true;
            if (primed_) {
                got = ring_.pop(&f[0], static_cast<size_t>(periodFrames_));
                if (got < static_cast<size_t>(periodFrames_)) {
                    ++underruns_;
                    primed_ = false;
                }
            }
            if (got < static_cast<size_t>(periodFrames_)) std::memset(&f[got * 2], 0, (periodFrames_ - got) * 2 * sizeof(float));
            for (size_t i = 0; i < f.size(); ++i) {
                float v = f[i] * 32767.0f;
                s[i] = static_cast<short>(v > 32767.0f ? 32767 : (v < -32768.0f ? -32768 : v));
            }
            if (fd_ >= 0) {
                snd_xferi x;
                x.result = 0;
                x.buf = &s[0];
                x.frames = static_cast<snd_pcm_uframes_t>(periodFrames_);
                // Blocks until the device has room (a period plays meanwhile).
                if (ioctl(fd_, SNDRV_PCM_IOCTL_WRITEI_FRAMES, &x) != 0) {
                    if (errno == EPIPE) {  // underrun: start again
                        ++underruns_;
                        ioctl(fd_, SNDRV_PCM_IOCTL_PREPARE);
                    } else if (errno != EINTR && errno != EAGAIN) {
                        usleep(20000);
                    }
                }
            } else if (pipe_) {
                if (fwrite(&s[0], sizeof(short), s.size(), pipe_) != s.size()) usleep(20000);
                fflush(pipe_);
            }
        }
    }

    std::string device_, name_;
    FrameRing ring_;
    int fd_;
    FILE* pipe_;
    int rate_;
    int periodFrames_ = kPeriodFrames;
    int periods_ = kPeriods;
    std::atomic<unsigned> underruns_;
    std::atomic<bool> running_;
    std::atomic<bool> primed_;
    std::thread thread_;
};

}  // namespace

std::unique_ptr<AudioOutput> AudioOutput::create(const std::string& device) {
    return std::unique_ptr<AudioOutput>(new LinuxOutput(device));
}

}  // namespace audio

#endif
