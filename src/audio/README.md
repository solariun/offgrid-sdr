# liboffgrid-audio — sound output in C++11, without sound libraries

Plays interleaved stereo float samples, never blocking the caller: `write()`
queues into a ring buffer, a playback thread (or the OS callback) takes from it,
and a late block becomes silence instead of a stall.

| Platform | How |
|---|---|
| macOS | AudioQueue (AudioToolbox, part of the OS) |
| Linux | the kernel's ALSA interface directly (`/dev/snd/pcmC*D*p`, no alsa-lib: works in a fully static binary); when PipeWire/PulseAudio owns the device, pipes to `pw-cat`, `pacat` or `aplay` |
| anywhere | `AudioOutput::createNull()`: no sound, same timing |

## Use

```cpp
#include "audio/AudioOutput.h"

std::unique_ptr<audio::AudioOutput> out = audio::AudioOutput::create();   // Linux: "" / "hw:1,0" / "pipe:cmd"
std::string error;
if (!out->open(48000, error)) out = audio::AudioOutput::createNull();
// out->rate() is the rate actually used (Linux may fall back to 44100)
out->write(stereo, frames);          // L, R, L, R ... in -1..1
double delay = out->queuedSeconds(); // keep it bounded (drop when > 0.25 s)
unsigned gaps = out->underruns();    // real dropouts (not counted before the start)
```

`make libs` builds `build/lib/liboffgrid-audio.a`; link with `-framework
AudioToolbox` on macOS, `-pthread` on Linux.
