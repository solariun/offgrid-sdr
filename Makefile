# OFFgrid-SDR TUI - a terminal software-defined radio receiver in C++11.
# SPDX-License-Identifier: GPL-3.0-or-later
#
# The program is built from four reusable static libraries (each usable on
# its own; see the README.md in each src/ directory):
#
#   build/lib/liboffgrid-rtlsdr.a    src/usb + src/rtlsdr  RTL-SDR driver (no libusb)
#   build/lib/liboffgrid-dsp.a       src/dsp               demodulators, RDS, spectrum, demo band
#   build/lib/liboffgrid-decoders.a  src/decoders          CW, RTTY, PSK, NAVTEX, DSC, Hell (+ encoders)
#   build/lib/liboffgrid-audio.a     src/audio             sound out (AudioQueue / ALSA / pipe)
#
#   make                 bin/offgrid (static on Linux; OS libraries only on macOS)
#   make libs            only the four libraries
#   make examples        bin/example-* (small programs using one or two libraries)
#   make tools           bin/rtl-probe
#   make test            build and run every test
#   make install         libraries + headers into $(PREFIX) (default /usr/local)
#   make check-static    show what bin/offgrid links against
#   make clean
#
# Variables: GRAPHICX=path (default external/Graphicx), STATIC=0 (Linux:
# dynamic), PREFIX=..., CXX=...

CXX      ?= c++
GRAPHICX ?= external/Graphicx
STATIC   ?= 1
PREFIX   ?= /usr/local
CXXFLAGS ?= -O2
CXXFLAGS += -std=c++11 -Wall -Wextra -Wpedantic -MMD -MP -Isrc -I$(GRAPHICX)/include -isystem $(GRAPHICX)/third_party -pthread

UNAME := $(shell uname -s)
ifeq ($(UNAME),Darwin)
  SYSLIBS := -framework IOKit -framework CoreFoundation -framework AudioToolbox
else
  ifeq ($(STATIC),1)
    LDFLAGS += -static
  endif
  SYSLIBS := -pthread
endif

ifeq ($(wildcard $(GRAPHICX)/include/graphicx/widgets.h),)
  $(error Graphicx not found at $(GRAPHICX): run "git submodule update --init" (or set GRAPHICX=path))
endif

obj = $(patsubst %.cpp,build/%.o,$(subst $(GRAPHICX)/,gfx/,$(1)))

RTLSDR_SRCS   := $(wildcard src/usb/*.cpp) $(wildcard src/rtlsdr/*.cpp)
DSP_SRCS      := $(wildcard src/dsp/*.cpp)
DECODERS_SRCS := $(wildcard src/decoders/*.cpp) $(wildcard src/decoders/ax25/*.cpp)
AUDIO_SRCS    := $(wildcard src/audio/*.cpp)
SDRC_SRCS     := $(wildcard src/sdrconnect/*.cpp)
CODECS_SRCS   := $(wildcard src/codecs/*.cpp)
APP_SRCS      := $(wildcard src/app/*.cpp)
GFX_SRCS      := $(addprefix $(GRAPHICX)/src/,graphicx.cpp terminal.cpp color.cpp canvas.cpp ui.cpp widgets.cpp png.cpp image_io.cpp)

LIB_RTLSDR   := build/lib/liboffgrid-rtlsdr.a
LIB_DSP      := build/lib/liboffgrid-dsp.a
LIB_DECODERS := build/lib/liboffgrid-decoders.a
LIB_AUDIO    := build/lib/liboffgrid-audio.a
LIB_SDRC     := build/lib/liboffgrid-sdrconnect.a
LIB_CODECS   := build/lib/liboffgrid-codecs.a
LIB_GFX      := build/lib/libgraphicx.a
LIBS         := $(LIB_RTLSDR) $(LIB_DSP) $(LIB_DECODERS) $(LIB_CODECS) $(LIB_AUDIO) $(LIB_SDRC)

TESTS    := $(patsubst tests/%.cpp,bin/%,$(wildcard tests/test_*.cpp))
EXAMPLES := $(patsubst examples/%.cpp,bin/example-%,$(wildcard examples/*.cpp))

.PHONY: all libs examples tools test install clean check-static

all: bin/offgrid

libs: $(LIBS)

$(LIB_RTLSDR): $(call obj,$(RTLSDR_SRCS))
$(LIB_DSP): $(call obj,$(DSP_SRCS))
$(LIB_DECODERS): $(call obj,$(DECODERS_SRCS))
$(LIB_AUDIO): $(call obj,$(AUDIO_SRCS))
$(LIB_SDRC): $(call obj,$(SDRC_SRCS))
$(LIB_CODECS): $(call obj,$(CODECS_SRCS))
$(LIB_GFX): $(call obj,$(GFX_SRCS))
build/lib/%.a:
	@mkdir -p $(dir $@)
	rm -f $@
	ar rcs $@ $^

# Link order: users before what they use (decoders need dsp's FFT and the
# codecs: DAB+ and DRM AAC, DAB MP2).
bin/offgrid: $(call obj,$(APP_SRCS)) $(LIB_DECODERS) $(LIB_CODECS) $(LIB_DSP) $(LIB_RTLSDR) $(LIB_SDRC) $(LIB_AUDIO) $(LIB_GFX)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ $(SYSLIBS) -o $@

examples: $(EXAMPLES)

bin/example-%: build/examples/%.o $(LIBS)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $< $(LIB_DECODERS) $(LIB_CODECS) $(LIB_DSP) $(LIB_RTLSDR) $(LIB_AUDIO) $(SYSLIBS) -o $@

tools: bin/rtl-probe

bin/rtl-probe: build/tools/rtl-probe.o $(LIB_RTLSDR)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ $(SYSLIBS) -o $@

bin/test_%: build/tests/test_%.o $(LIB_DECODERS) $(LIB_CODECS) $(LIB_DSP) $(LIB_SDRC)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ $(SYSLIBS) -o $@

test: $(TESTS)
	@set -e; for t in $(TESTS); do echo "== $$t"; ./$$t | tail -3; done

build/gfx/%.o: $(GRAPHICX)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

install: $(LIBS)
	mkdir -p $(PREFIX)/lib $(PREFIX)/include/offgrid
	cp $(LIBS) $(PREFIX)/lib/
	for d in usb rtlsdr dsp decoders codecs audio sdrconnect; do \
	  mkdir -p $(PREFIX)/include/offgrid/$$d; cp src/$$d/*.h $(PREFIX)/include/offgrid/$$d/; done

check-static: bin/offgrid
ifeq ($(UNAME),Darwin)
	@echo "macOS cannot link libSystem statically; only OS libraries may appear here:"
	otool -L bin/offgrid
else
	file bin/offgrid
	-ldd bin/offgrid
endif

clean:
	rm -rf build bin

-include $(shell find build -name '*.d' 2>/dev/null)
