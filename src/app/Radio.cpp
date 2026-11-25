// ============================================================================
//  OFFgrid-SDR TUI - a terminal software-defined radio receiver, in C++11.
//  Port of OFFgrid-SDR (https://github.com/solariun/offgrid-sdr).
//  SPDX-License-Identifier: GPL-3.0-or-later
// ----------------------------------------------------------------------------
//  File:   src/app/Radio.cpp
//  Layer:  Application (the receiver engine)
//
//  What this file does
//  -------------------
//  Implementation of app::Radio (see Radio.h).
//
//  Timing
//  ------
//  The stick's clock and the sound card's clock differ by a few ppm, so the
//  audio queue slowly grows or shrinks.  The loop keeps it between ~40 ms
//  and 250 ms: above 250 ms it drops a block of audio (inaudible), so the
//  delay never builds up.  Underruns (queue empty) are counted and shown.
// ============================================================================
#include "Radio.h"

#include "../decoders/Ais.h"
#include "../decoders/Ax25.h"
#include "../decoders/Callsign.h"
#include "../decoders/Ft8.h"
#include "../decoders/Wspr.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace app {

RadioSettings::RadioSettings()
    : frequency(100e6),
      mode(dsp::Mode::WFM),
      bandwidth(0),
      volume(0.8),
      squelchDb(-200),
      gainDb(-1),
      ppm(0),
      stereo(true),
      deemphasisUs(50),
      biasTee(false),
      muted(false),
      fftSize(2048),
      directSampling(0),
      decoderShift(0),
      noiseBlanker(false),
      noiseReduction(0),
      humFilterHz(0),
      autoNotch(false),
      voiceCleaner(0),
      limiter(true) {}

Snapshot::Snapshot()
    : spectrumSerial(0),
      centerHz(0),
      sampleRate(0),
      channelDbfs(-120),
      noiseDbfs(-120),
      stereoPilot(false),
      squelchOpen(false),
      audioQueued(0),
      audioUnderruns(0),
      samplesDropped(0),
      decoderBlocksDropped(0),
      dspLoad(0),
      sampleRateMeasured(0),
      overload(false),
      carrierHz(0),
      voiceProbability(0),
      decoderOn(false),
      decoderKind(dec::Kind::Cw) {}

Radio::Radio(std::unique_ptr<Source> source, std::unique_ptr<audio::AudioOutput> audio)
    : source_(std::move(source)),
      audio_(std::move(audio)),
      spectrum_(2048, 0.6),
      receiverDirty_(true),
      fftDirty_(false),
      flushAudio_(false),
      center_(0),
      running_(false),
      jobsDropped_(0),
      decoderActive_(false),
      decoderIq_(false),
      decoderOn_(false),
      decoderWanted_(false),
      decoderDirty_(false),
      decoderReset_(false),
      decoderKind_(dec::Kind::Cw),
      clockOffset_(0),
      decoderAudio_(false),
      decFifoOwner_(0),
      decPlaying_(false),
      centerOnDial_(false),
      upconverter_(0),
      ftxPending_(false),
      autoListening_(false),
      autoReady_(false),
      historyPos_(0),
      historyFill_(0),
      historyCenter_(0),
      historyRate_(0),
      replaying_(false),
      replayStop_(false),
      replayPos_(0),
      wav_(0),
      wavFrames_(0) {}

Radio::~Radio() { stop(); }

double Radio::offset() const { return settings_.frequency - center_; }

bool Radio::start(const RadioSettings& s, std::string& error) {
    settings_ = s;
    std::string audioError;
    if (!audio_->open(48000, audioError)) {
        // No sound is not fatal: keep the radio running silently.
        audio_ = audio::AudioOutput::createNull();
        audio_->open(48000, audioError);
    }
    receiver_.reset(new dsp::Receiver(audio_->rate()));

    // Initial centre: a quarter span below the dial (DC spike off the signal).
    double rate = source_->sampleRate();
    center_ = source_->tunable() ? settings_.frequency - rate / 4 : source_->centerFrequency();
    if (source_->tunable()) source_->setCenterFrequency(center_);
    if (!source_->start(error)) return false;
    source_->setGain(settings_.gainDb);
    if (settings_.ppm != 0) source_->setPpm(settings_.ppm);
    if (settings_.biasTee) source_->setBiasTee(true);
    if (settings_.directSampling) source_->setDirectSampling(settings_.directSampling);
    center_ = source_->centerFrequency();
    spectrum_.setSize(settings_.fftSize);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        applyReceiver();
    }
    running_ = true;
    thread_ = std::thread(&Radio::loop, this);
    decoderThread_ = std::thread(&Radio::decoderLoop, this);
    return true;
}

void Radio::stop() {
    stopReplay();
    running_ = false;
    if (thread_.joinable()) thread_.join();
    jobReady_.notify_all();
    if (decoderThread_.joinable()) decoderThread_.join();
    stopRecording();
    if (source_) source_->stop();
    if (audio_) audio_->close();
}

// The settings' clean-up filters into a receiver configuration.
static void applyFilters(const RadioSettings& s, dsp::ReceiverConfig& c) {
    static const double kMix[] = {0, 0.5, 0.75, 0.9, 1.0};
    c.noiseBlanker = s.noiseBlanker;
    c.noiseReduction = s.noiseReduction > 0;
    c.noiseReductionLevel = std::max(1, std::min(3, s.noiseReduction));
    c.humFilterHz = s.humFilterHz;
    c.autoNotch = s.autoNotch;
    c.voiceCleaner = s.voiceCleaner > 0;
    c.voiceCleanerMix = kMix[std::max(0, std::min(4, s.voiceCleaner))];
    c.limiter = s.limiter;
    c.retuneMuteSeconds = s.limiter ? 0.25 : 0;
}

void Radio::setFilters(const RadioSettings& f) {
    std::lock_guard<std::mutex> lock(mutex_);
    settings_.noiseBlanker = f.noiseBlanker;
    settings_.noiseReduction = f.noiseReduction;
    settings_.humFilterHz = f.humFilterHz;
    settings_.autoNotch = f.autoNotch;
    settings_.voiceCleaner = f.voiceCleaner;
    settings_.limiter = f.limiter;
    receiverDirty_ = true;
}

void Radio::applyReceiver() {
    dsp::ReceiverConfig c;
    c.sampleRate = source_->sampleRate();
    c.offsetHz = offset() - settings_.decoderShift;
    c.mode = settings_.mode;
    c.bandwidthHz = settings_.bandwidth;
    c.stereo = settings_.stereo;
    c.deemphasisUs = settings_.deemphasisUs;
    c.squelchDb = settings_.squelchDb;
    c.volume = settings_.muted ? 0 : settings_.volume;
    applyFilters(settings_, c);
    receiver_->configure(c);
    receiverDirty_ = false;
}

void Radio::tune(double hz) {
    std::lock_guard<std::mutex> lock(mutex_);
    hz = std::max(24e3, std::min(1.766e9, hz));
    if (!source_->tunable()) {
        // A recording: only inside its span.
        double half = source_->sampleRate() / 2 * 0.95;
        hz = std::max(center_ - half, std::min(center_ + half, hz));
    }
    settings_.frequency = hz;
    double rate = source_->sampleRate();
    const bool move = centerOnDial_ ? std::fabs(hz - center_) > 1 : std::fabs(hz - center_) > rate * 0.4;
    if (source_->tunable() && move) {
        center_ = centerOnDial_ ? hz : hz - rate / 4;
        if (center_ < 0) center_ = hz + rate / 4;
        source_->setCenterFrequency(hardwareFrequency(center_));
        center_ = source_->centerFrequency() - (hardwareFrequency(center_) - center_);
        flushAudio_ = true;
    }
    receiverDirty_ = true;
    decoderReset_ = true;  // a new signal: the decoder starts again
}

void Radio::setMode(dsp::Mode m) {
    std::lock_guard<std::mutex> lock(mutex_);
    settings_.mode = m;
    settings_.bandwidth = 0;
    receiverDirty_ = true;
    flushAudio_ = true;
}

#define RADIO_SETTER(name, type, field)        \
    void Radio::name(type v) {                 \
        std::lock_guard<std::mutex> lock(mutex_); \
        settings_.field = v;                   \
        receiverDirty_ = true;                 \
    }
RADIO_SETTER(setBandwidth, double, bandwidth)
RADIO_SETTER(setVolume, double, volume)
RADIO_SETTER(setSquelch, double, squelchDb)
RADIO_SETTER(setStereo, bool, stereo)
RADIO_SETTER(setDeemphasis, double, deemphasisUs)
RADIO_SETTER(setMuted, bool, muted)
#undef RADIO_SETTER

void Radio::setGain(double dB) {
    std::lock_guard<std::mutex> lock(mutex_);
    settings_.gainDb = dB;
    source_->setGain(dB);
}

void Radio::setPpm(double ppm) {
    std::lock_guard<std::mutex> lock(mutex_);
    settings_.ppm = ppm;
    source_->setPpm(ppm);
}

void Radio::setBiasTee(bool on) {
    std::lock_guard<std::mutex> lock(mutex_);
    settings_.biasTee = on;
    source_->setBiasTee(on);
}

void Radio::setFftSize(int n) {
    std::lock_guard<std::mutex> lock(mutex_);
    settings_.fftSize = n;
    fftDirty_ = true;
}

RadioSettings Radio::settings() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return settings_;
}

void Radio::snapshot(Snapshot& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    out = snap_;
}

std::string Radio::sourceName() const { return source_->name(); }
std::string Radio::audioName() const { return audio_->name(); }
std::vector<double> Radio::gains() const { return source_->gains(); }
bool Radio::tunable() const { return source_->tunable(); }

// ----------------------------------------------------------------- decoders --

void Radio::setDecoder(bool on, dec::Kind kind, std::function<void(dec::Decoder&)> setup) {
    std::lock_guard<std::mutex> lock(mutex_);
    decoderWanted_ = on;
    decoderKind_ = kind;
    decoderSetup_ = setup;
    decoderIq_ = kind == dec::Kind::Ais || kind == dec::Kind::Vdl2 || kind == dec::Kind::Dab || kind == dec::Kind::Drm;
    decoderAudio_ = on && (kind == dec::Kind::Dab || kind == dec::Kind::Drm);
    decoderActive_ = on;
    services_.clear();
    flushAudio_ = true;
    decoderDirty_ = true;
}

void Radio::setDecoderShift(double hz) {
    std::lock_guard<std::mutex> lock(mutex_);
    settings_.decoderShift = hz;
    receiverDirty_ = true;
}

void Radio::setClockOffset(double pcMinusTrue) {
    std::lock_guard<std::mutex> lock(mutex_);
    clockOffset_ = pcMinusTrue;
}

void Radio::setLocator(const std::string& grid) {
    std::lock_guard<std::mutex> lock(mutex_);
    locator_ = grid;
    double lat, lon;
    if (dec::Locator::toLatLon(grid, lat, lon)) heard_.setHomeLatLon(lat, lon);
    decoderReset_ = true;  // the decoder gets it at its next setup
}

bool Radio::takeFtxTimings(std::string& proto, std::vector<double>& dts) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ftxPending_) return false;
    ftxPending_ = false;
    proto = ftxProto_;
    dts.swap(ftxDts_);
    ftxDts_.clear();
    return true;
}

std::string Radio::heardText(double now, double windowSeconds, bool details) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return heard_.text(now, windowSeconds, details);
}

double Radio::hardwareFrequency(double dial) const {
    return upconverter_ > 0 && dial < 65e6 ? dial + upconverter_ : dial;
}

void Radio::setCenterOnDial(bool on) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        centerOnDial_ = on;
    }
    tune(settings().frequency);
}

void Radio::setUpconverter(double offsetHz) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        upconverter_ = offsetHz > 0 ? offsetHz : 0;
        center_ = -1e12;  // force a retune
    }
    tune(settings().frequency);
}

void Radio::decoderCommand(std::function<void(dec::Decoder&)> command) {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_.push_back(command);
}

std::vector<dec::Service> Radio::decoderServices() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return services_;
}

void Radio::takeDecoderImages(std::vector<dec::Decoder::EncodedImage>& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    out.insert(out.end(), images_.begin(), images_.end());
    images_.clear();
}

void Radio::resetDecoder() {
    std::lock_guard<std::mutex> lock(mutex_);
    decoderReset_ = true;
}

void Radio::takeDecoderText(std::string& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    out += decText_;
    decText_.clear();
}

void Radio::takeDecoderLines(std::vector<dec::PictureLine>& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    out.insert(out.end(), decLines_.begin(), decLines_.end());
    decLines_.clear();
}

void Radio::takeDecoderColumns(std::vector<std::vector<float> >& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    out.insert(out.end(), decColumns_.begin(), decColumns_.end());
    decColumns_.clear();
}

// ---------------------------------------------------------------- recording --

namespace {

void put16(FILE* f, unsigned v) {
    std::fputc(static_cast<int>(v & 0xFF), f);
    std::fputc(static_cast<int>((v >> 8) & 0xFF), f);
}
void put32(FILE* f, unsigned long v) {
    put16(f, static_cast<unsigned>(v & 0xFFFF));
    put16(f, static_cast<unsigned>((v >> 16) & 0xFFFF));
}

// 44-byte header of a 16-bit stereo PCM WAV with `frames` frames.
void wavHeader(FILE* f, int rate, unsigned long frames) {
    unsigned long data = frames * 4;
    std::fwrite("RIFF", 1, 4, f);
    put32(f, 36 + data);
    std::fwrite("WAVEfmt ", 1, 8, f);
    put32(f, 16);
    put16(f, 1);  // PCM
    put16(f, 2);  // channels
    put32(f, static_cast<unsigned long>(rate));
    put32(f, static_cast<unsigned long>(rate) * 4);
    put16(f, 4);
    put16(f, 16);
    std::fwrite("data", 1, 4, f);
    put32(f, data);
}

}  // namespace

bool Radio::startRecording(const std::string& path, std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (wav_) return true;
    wav_ = std::fopen(path.c_str(), "wb");
    if (!wav_) {
        error = "cannot create " + path;
        return false;
    }
    wavFrames_ = 0;
    wavHeader(wav_, audio_->rate(), 0);  // sizes fixed when it ends
    return true;
}

void Radio::stopRecording() {
    std::lock_guard<std::mutex> lock(mutex_);
    finishWav();
}

void Radio::finishWav() {
    if (!wav_) return;
    std::fseek(wav_, 0, SEEK_SET);
    wavHeader(wav_, audio_->rate(), wavFrames_);
    std::fclose(wav_);
    wav_ = 0;
}

bool Radio::recording() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return wav_ != 0;
}

double Radio::recordedSeconds() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<double>(wavFrames_) / audio_->rate();
}

void Radio::writeWav(const std::vector<float>& audio) {
    if (!wav_) return;
    for (size_t i = 0; i < audio.size(); ++i) {
        float v = audio[i] * 32767.0f;
        int s = v > 32767.0f ? 32767 : (v < -32768.0f ? -32768 : static_cast<int>(v));
        put16(wav_, static_cast<unsigned>(s) & 0xFFFF);
    }
    wavFrames_ += audio.size() / 2;
}

// ---------------------------------------------------------- AUTO, monitor --

void Radio::startAuto() {
    double dial = settings().frequency;
    std::lock_guard<std::mutex> al(analysisMutex_);
    auto_.reset();
    if (auto_.begin(dial)) {  // decided by the frequency alone
        autoListening_ = false;
        autoReady_ = true;
    } else {
        autoListening_ = true;
        autoReady_ = false;
    }
}

bool Radio::autoListening() const {
    std::lock_guard<std::mutex> al(analysisMutex_);
    return autoListening_;
}

bool Radio::takeAutoResult(dsp::AutoResult& out) {
    std::lock_guard<std::mutex> al(analysisMutex_);
    if (!autoReady_) return false;
    autoReady_ = false;
    out = auto_.result();
    return true;
}

int Radio::monitorAdd(double centreHz, double widthHz, const std::string& mode, const std::string& name) {
    std::lock_guard<std::mutex> al(analysisMutex_);
    return monitor_.add(centreHz, widthHz, mode, name, false);
}

void Radio::monitorRemove(int index) {
    std::lock_guard<std::mutex> al(analysisMutex_);
    monitor_.remove(index);
}

int Radio::monitorCount() const {
    std::lock_guard<std::mutex> al(analysisMutex_);
    return monitor_.count();
}

dsp::MonitorStats Radio::monitorStats(int index) const {
    std::lock_guard<std::mutex> al(analysisMutex_);
    return monitor_.stats(index);
}

double Radio::monitorCentre(int index) const {
    std::lock_guard<std::mutex> al(analysisMutex_);
    return monitor_.centreHz(index);
}

double Radio::monitorWidth(int index) const {
    std::lock_guard<std::mutex> al(analysisMutex_);
    return monitor_.widthHz(index);
}

std::string Radio::monitorName(int index) const {
    std::lock_guard<std::mutex> al(analysisMutex_);
    return monitor_.name(index);
}

std::string Radio::monitorCsv(double unixAtZero) const {
    std::lock_guard<std::mutex> al(analysisMutex_);
    return monitor_.csv(unixAtZero);
}

bool Radio::monitorFollowing(int index) const {
    std::lock_guard<std::mutex> al(analysisMutex_);
    return monitor_.follow(index);
}

void Radio::monitorFollow(int index, bool on) {
    std::lock_guard<std::mutex> al(analysisMutex_);
    monitor_.setFollow(index, on);
}

// -------------------------------------------------------------------- replay --

bool Radio::saveIq(const std::string& path, std::string& error) {
    std::vector<std::uint8_t> bytes;
    {
        std::lock_guard<std::mutex> hl(historyMutex_);
        if (historyFill_ == 0) {
            error = "nothing received yet";
            return false;
        }
        const size_t cap = history_.size(), start = (historyPos_ + cap - historyFill_) % cap;
        bytes.resize(historyFill_);
        for (size_t i = 0; i < historyFill_; ++i) bytes[i] = history_[(start + i) % cap];
    }
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        error = "cannot create " + path;
        return false;
    }
    bool ok = std::fwrite(&bytes[0], 1, bytes.size(), f) == bytes.size();
    ok = std::fclose(f) == 0 && ok;
    if (!ok) error = "cannot write " + path;
    return ok;
}

bool Radio::startReplay(std::string& error) {
    stopReplay();
    std::vector<std::uint8_t> bytes;
    double center, rate;
    {
        std::lock_guard<std::mutex> hl(historyMutex_);
        if (historyFill_ < static_cast<size_t>(historyRate_)) {  // at least half a second
            error = "not enough received yet to replay";
            return false;
        }
        const size_t cap = history_.size(), start = (historyPos_ + cap - historyFill_) % cap;
        bytes.resize(historyFill_);
        for (size_t i = 0; i < historyFill_; ++i) bytes[i] = history_[(start + i) % cap];
        center = historyCenter_;
        rate = historyRate_;
    }
    replayStop_ = false;
    replaying_ = true;
    audio_->flush();
    replayThread_ = std::thread(&Radio::replayLoop, this, bytes, center, rate);
    return true;
}

void Radio::stopReplay() {
    replayStop_ = true;
    if (replayThread_.joinable()) replayThread_.join();
    if (replaying_) audio_->flush();
    replaying_ = false;
}

double Radio::replayPosition() const { return replayPos_; }

// Plays the history through its own receiver, in real time, following the
// dial and mode (offset from the history's centre).
void Radio::replayLoop(std::vector<std::uint8_t> bytes, double center, double rate) {
    dsp::Receiver rx(audio_->rate());
    std::vector<dsp::Complex> iq;
    std::vector<float> sound;
    const size_t chunk = static_cast<size_t>(rate * 2 * 0.02);  // 20 ms of bytes
    typedef std::chrono::steady_clock Clock;
    Clock::time_point begin = Clock::now();
    for (size_t at = 0; at < bytes.size() && !replayStop_; at += chunk) {
        RadioSettings st = settings();
        dsp::ReceiverConfig c;
        c.sampleRate = rate;
        c.offsetHz = st.frequency - center - st.decoderShift;
        c.mode = st.mode;
        c.bandwidthHz = st.bandwidth;
        c.stereo = st.stereo;
        c.deemphasisUs = st.deemphasisUs;
        c.squelchDb = st.squelchDb;
        c.volume = st.muted ? 0 : st.volume;
        applyFilters(st, c);
        rx.configure(c);
        const size_t n = std::min(chunk, bytes.size() - at) & ~static_cast<size_t>(1);
        dsp::convertU8(&bytes[at], n, iq);
        sound.clear();
        rx.process(&iq[0], iq.size(), sound);
        if (!sound.empty()) audio_->write(&sound[0], sound.size() / 2);
        replayPos_ = static_cast<double>(at) / (2 * rate);
        // Real time: sleep until this chunk is due.
        std::this_thread::sleep_until(begin + std::chrono::microseconds(static_cast<long long>((at + n) / (2 * rate) * 1e6)));
    }
    replaying_ = false;
}

// ----------------------------------------------------------- decoder thread --

void Radio::decoderLoop() {
    std::string text;
    std::vector<std::vector<float> > columns;
    std::vector<dec::PictureLine> lines;
    while (running_) {
        DecoderJob job;
        bool have = false;
        {
            std::unique_lock<std::mutex> lock(jobMutex_);
            jobReady_.wait_for(lock, std::chrono::milliseconds(100), [this] { return !jobs_.empty() || !running_; });
            if (!jobs_.empty()) {
                job.audio.swap(jobs_.front().audio);
                job.iq.swap(jobs_.front().iq);
                job.audioRate = jobs_.front().audioRate;
                job.iqRate = jobs_.front().iqRate;
                job.timestamp = jobs_.front().timestamp;
                job.clockOffset = jobs_.front().clockOffset;
                job.dial = jobs_.front().dial;
                job.center = jobs_.front().center;
                jobs_.pop_front();
                have = true;
            }
        }
        // Create / replace / reset the decoder as asked.
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (decoderDirty_) {
                decoderDirty_ = false;
                decoderReset_ = false;
                decoder_.reset();
                if (decoderWanted_) {
                    decoder_ = dec::makeDecoder(decoderKind_);
                    if (decoder_ && decoderSetup_) decoderSetup_(*decoder_);
                    double lat, lon;
                    if (decoder_ && dec::Locator::toLatLon(locator_, lat, lon)) {
                        if (dec::FtxDecoder* f = dynamic_cast<dec::FtxDecoder*>(decoder_.get())) f->setHomeLatLon(lat, lon);
                        if (dec::AisDecoder* a = dynamic_cast<dec::AisDecoder*>(decoder_.get())) a->setHome(lat, lon);
                        if (dec::Ax25Decoder* x = dynamic_cast<dec::Ax25Decoder*>(decoder_.get())) x->setHome(lat, lon);
                    }
                }
                decoderOn_ = decoder_ != 0;
                decStatus_ = dec::Status();
            }
            if (decoderReset_) {
                decoderReset_ = false;
                if (decoder_) {
                    decoder_->reset();
                    if (decoderSetup_) decoderSetup_(*decoder_);  // settings that reset() may clear
                }
            }
        }
        if (decoder_) {  // commands from the UI, run here (the decoder's thread)
            std::deque<std::function<void(dec::Decoder&)> > cmds;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                cmds.swap(commands_);
            }
            for (size_t i = 0; i < cmds.size(); ++i) cmds[i](*decoder_);
        }
        if (!have || !decoder_) continue;
        // WSPR reports RF frequencies and the stick's error from the dial.
        dec::WsprDecoder* wspr = dynamic_cast<dec::WsprDecoder*>(decoder_.get());
        dec::FtxDecoder* ftx = dynamic_cast<dec::FtxDecoder*>(decoder_.get());
        if (wspr) wspr->setDialFrequency(job.dial);
        if (ftx) ftx->setDial(job.dial);
        decoder_->setTimestamp(job.timestamp);
        decoder_->setClockOffset(job.clockOffset);
        if (decoder_->wantsIq()) {
            if (!job.iq.empty()) decoder_->processIq(&job.iq[0], job.iq.size(), job.iqRate, job.center, job.dial);
        } else if (!job.audio.empty()) {
            decoder_->process(&job.audio[0], job.audio.size(), job.audioRate);
        }
        text.clear();
        lines.clear();
        columns.clear();
        if (decoder_->hasPicture()) decoder_->takeLines(lines);
        decoder_->takeText(text);
        if (decoder_->hasImage()) decoder_->takeColumns(columns);
        // Decoders with their own sound (DAB, DRM): played instead of the
        // receiver's, resampled (linear) to the sound card's rate.
        if (decoder_->hasAudio()) {
            if (decFifoOwner_ != decoder_.get()) {
                decFifoOwner_ = decoder_.get();
                decFifo_.clear();
                decPlaying_ = false;
            }
            std::vector<float> pcm;
            decoder_->takeAudio(pcm);
            const double inRate = decoder_->audioRate(), outRate = audio_->rate();
            float gain;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                gain = settings_.muted ? 0.f : static_cast<float>(settings_.volume);
            }
            if (!pcm.empty()) {
                const size_t inFrames = pcm.size() / 2;
                if (inRate == outRate || inRate <= 0) {
                    for (size_t i = 0; i < 2 * inFrames; ++i) decFifo_.push_back(pcm[i] * gain);
                } else {
                    // Linear resampling to the sound card's rate.
                    const size_t outFrames = static_cast<size_t>(inFrames * outRate / inRate);
                    for (size_t i = 0; i < outFrames; ++i) {
                        const double pos = i * inRate / outRate;
                        const size_t k = static_cast<size_t>(pos);
                        const float f = static_cast<float>(pos - k);
                        const size_t k1 = std::min(k + 1, inFrames - 1);
                        decFifo_.push_back((pcm[2 * k] * (1 - f) + pcm[2 * k1] * f) * gain);
                        decFifo_.push_back((pcm[2 * k + 1] * (1 - f) + pcm[2 * k1 + 1] * f) * gain);
                    }
                }
            }
            // Buffer first (DRM: more than one 1.2 s window), then keep the
            // sound card's queue (it holds about 1 s; decoder blocks come
            // every 0.1-0.3 s) at 0.7 s; more than 4 s waiting: the oldest
            // goes (the decoder ran ahead).
            const double target = decoder_->kind() == dec::Kind::Drm ? 1.6 : 0.4, queueTarget = 0.7;
            const double fifoSeconds = decFifo_.size() / 2 / outRate, queued = audio_->queuedSeconds();
            if (fifoSeconds > 4) decFifo_.erase(decFifo_.begin(), decFifo_.begin() + static_cast<long>(2 * static_cast<size_t>((fifoSeconds - target) * outRate)));
            if (!decPlaying_ && fifoSeconds + queued >= target) decPlaying_ = true;
            if (decPlaying_) {
                const size_t want = queued < queueTarget ? static_cast<size_t>((queueTarget - queued) * outRate) : 0;
                const size_t n = std::min(want, decFifo_.size() / 2);
                if (n > 0) {
                    std::vector<float> out(decFifo_.begin(), decFifo_.begin() + static_cast<long>(2 * n));
                    decFifo_.erase(decFifo_.begin(), decFifo_.begin() + static_cast<long>(2 * n));
                    audio_->write(&out[0], n);
                }
                if (decFifo_.empty() && queued < 0.02) decPlaying_ = false;  // ran dry: buffer again
            }
        }
        std::vector<dec::Decoder::EncodedImage> images;
        decoder_->takeImages(images);
        static unsigned serviceTick = 0;
        const bool refreshServices = (++serviceTick % 32) == 0;
        std::vector<dec::Service> services;
        if (refreshServices) services = decoder_->services();
        dec::Status st = decoder_->status();
        // FT8 / FT4 / WSPR: decodes to the Heard list, slot timings to the clock.
        std::vector<dec::FtxDecode> ftxDecodes;
        std::vector<dec::WsprDecode> wsprDecodes;
        std::vector<double> dts;
        std::string proto;
        bool timings = false;
        if (ftx) {
            ftx->takeDecodes(ftxDecodes);
            timings = ftx->takeSlotTimings(dts);
            proto = ftx->protoName();
        }
        if (wspr) {
            wspr->takeDecodes(wsprDecodes);
            if (wspr->newSlotDone()) {
                dts = wspr->lastSlotDts();
                timings = !dts.empty();
                proto = "WSPR";
            }
        }
        std::lock_guard<std::mutex> lock(mutex_);
        decStatus_ = st;
        if (refreshServices) services_ = services;
        if (!images.empty()) {
            images_.insert(images_.end(), images.begin(), images.end());
            if (images_.size() > 16) images_.erase(images_.begin(), images_.end() - 16);
        }
        for (size_t i = 0; i < ftxDecodes.size(); ++i) {
            const dec::FtxDecode& d = ftxDecodes[i];
            if (!d.caller.empty()) heard_.note(d.slotTime, job.dial, d.proto, d.caller, d.gridFromEarlier ? "" : d.grid, d.snr);
        }
        for (size_t i = 0; i < wsprDecodes.size(); ++i)
            heard_.note(wsprDecodes[i].slotStart, job.dial, "WSPR", wsprDecodes[i].call, wsprDecodes[i].grid, wsprDecodes[i].snrDb);
        if (timings) {
            ftxProto_ = proto;
            ftxDts_ = dts;
            ftxPending_ = true;
        }
        if (!text.empty()) {
            decText_ += text;
            if (decText_.size() > 65536) decText_.erase(0, decText_.size() - 65536);
        }
        if (!lines.empty()) {
            decLines_.insert(decLines_.end(), lines.begin(), lines.end());
            if (decLines_.size() > 8000) decLines_.erase(decLines_.begin(), decLines_.end() - 8000);
        }
        if (!columns.empty()) {
            decColumns_.insert(decColumns_.end(), columns.begin(), columns.end());
            if (decColumns_.size() > 4000) decColumns_.erase(decColumns_.begin(), decColumns_.end() - 4000);
        }
    }
}

// --------------------------------------------------------------------- loop --

void Radio::loop() {
    typedef std::chrono::steady_clock Clock;
    std::vector<std::uint8_t> block;
    std::vector<dsp::Complex> iq;
    std::vector<float> audio;
    double busy = 0, windowStart = 0;
    double windowSamples = 0;
    Clock::time_point t0 = Clock::now();
    unsigned rails = 0, railsTotal = 0;

    while (running_) {
        double arrived = 0;
        if (!source_->queue().pop(block, 50, &arrived)) continue;
        Clock::time_point begin = Clock::now();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (receiverDirty_) applyReceiver();
            if (fftDirty_) {
                spectrum_.setSize(settings_.fftSize);
                fftDirty_ = false;
            }
            if (flushAudio_) {
                audio_->flush();
                flushAudio_ = false;
            }
        }
        // Overload: count samples at the very ends of the 8-bit range.
        for (size_t i = 0; i < block.size(); i += 16) {
            ++railsTotal;
            if (block[i] < 2 || block[i] > 253) ++rails;
        }
        dsp::convertU8(&block[0], block.size(), iq);  // count = bytes
        {
            // AUTO (while listening) and the Signal Monitor see every block.
            std::lock_guard<std::mutex> al(analysisMutex_);
            if (autoListening_ && auto_.feedIq(&iq[0], iq.size(), source_->sampleRate(), center_)) {
                autoListening_ = false;
                autoReady_ = true;
            }
            if (monitor_.count() > 0) monitor_.feedIq(&iq[0], iq.size(), source_->sampleRate(), center_);
        }
        {
            // The IQ history ring (replay, Save IQ); restarted when the
            // centre or rate changes (old samples no longer line up).
            std::lock_guard<std::mutex> hl(historyMutex_);
            const double r = source_->sampleRate();
            const size_t cap = static_cast<size_t>(r * 2 * kHistorySeconds);
            if (history_.size() != cap || historyCenter_ != center_ || historyRate_ != r) {
                history_.assign(cap, 127);
                historyPos_ = historyFill_ = 0;
                historyCenter_ = center_;
                historyRate_ = r;
            }
            for (size_t i = 0; i < block.size(); ++i) {
                history_[historyPos_] = block[i];
                if (++historyPos_ == cap) historyPos_ = 0;
            }
            historyFill_ = std::min(cap, historyFill_ + block.size());
        }
        double rate = source_->sampleRate();
        bool frame = spectrum_.push(&iq[0], iq.size(), rate, 30);
        audio.clear();
        receiver_->process(&iq[0], iq.size(), audio);
        size_t frames = audio.size() / 2;
        // Keep the audio delay bounded (clock drift between stick and card).
        if (!decoderAudio_ && !replaying_ && audio_->queuedSeconds() < 0.25) audio_->write(audio.empty() ? 0 : &audio[0], frames);
        if (wav_) {
            std::lock_guard<std::mutex> lock(mutex_);
            writeWav(audio);
        }
        // The decoder (on its own thread) gets the receiver's tap (before
        // squelch / volume), or the stick's samples for IQ decoders.
        if (decoderActive_) {
            DecoderJob job;
            if (decoderIq_) job.iq = iq;
            else job.audio = receiver_->decoderAudio();
            job.audioRate = audio_->rate();
            job.iqRate = rate;
            // The computer's clock at this block's first sample: when it
            // arrived (= its last sample), minus its length.
            job.timestamp = arrived - static_cast<double>(iq.size()) / rate;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                job.clockOffset = clockOffset_;
                job.dial = settings_.frequency;
            }
            job.center = center_;
            {
                std::lock_guard<std::mutex> lock(jobMutex_);
                // ~2 s of work at most: a decoder that falls behind loses the
                // oldest blocks (counted) rather than the radio stalling.
                const size_t maxJobs = static_cast<size_t>(2.0 * rate / std::max<size_t>(1, iq.size())) + 1;
                while (jobs_.size() >= maxJobs) {
                    jobs_.pop_front();
                    ++jobsDropped_;
                }
                jobs_.push_back(DecoderJob());
                jobs_.back().audio.swap(job.audio);
                jobs_.back().iq.swap(job.iq);
                jobs_.back().audioRate = job.audioRate;
                jobs_.back().iqRate = job.iqRate;
                jobs_.back().timestamp = job.timestamp;
                jobs_.back().clockOffset = job.clockOffset;
                jobs_.back().dial = job.dial;
                jobs_.back().center = job.center;
            }
            jobReady_.notify_one();
        }

        Clock::time_point end = Clock::now();
        busy += std::chrono::duration<double>(end - begin).count();
        windowSamples += static_cast<double>(iq.size());
        double now = std::chrono::duration<double>(end - t0).count();

        std::lock_guard<std::mutex> lock(mutex_);
        if (frame) {
            snap_.spectrum = spectrum_.powerDb();
            ++snap_.spectrumSerial;
        }
        snap_.centerHz = center_;
        snap_.sampleRate = rate;
        snap_.channelDbfs = receiver_->channelDbfs();
        snap_.voiceProbability = receiver_->voiceProbability();
        snap_.noiseDbfs = receiver_->noiseDbfs();
        snap_.stereoPilot = receiver_->stereoPilot();
        snap_.squelchOpen = receiver_->squelchOpen();
        if (settings_.mode == dsp::Mode::WFM) snap_.rds = receiver_->rds();
        else snap_.rds = dsp::RdsInfo();
        snap_.carrierHz = receiver_->carrierOffsetHz();
        snap_.decoderOn = decoderOn_;
        snap_.decoderKind = decoderKind_;
        snap_.decoder = decStatus_;
        snap_.audioQueued = audio_->queuedSeconds();
        snap_.audioUnderruns = audio_->underruns();
        snap_.samplesDropped = source_->queue().dropped();
        {
            std::lock_guard<std::mutex> jl(jobMutex_);
            snap_.decoderBlocksDropped = jobsDropped_;
        }
        if (now - windowStart >= 1.0) {
            snap_.dspLoad = busy / (now - windowStart);
            snap_.sampleRateMeasured = windowSamples / (now - windowStart);
            snap_.overload = railsTotal > 0 && rails * 1000 > railsTotal;  // > 0.1% at the rails
            busy = 0;
            windowSamples = 0;
            windowStart = now;
            rails = railsTotal = 0;
        }
    }
}

}  // namespace app
