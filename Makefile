# OFFgrid-SDR TUI - a terminal software-defined radio receiver in C++11.
# SPDX-License-Identifier: GPL-3.0-or-later
#
#   make                 bin/offgrid (static on Linux; system libraries only on macOS)
#   make tools           bin/rtl-probe (driver smoke test with a stick)
#   make test            build and run the DSP tests
#   make check-static    show what bin/offgrid links against
#   make clean
#
# Variables:
#   GRAPHICX=path        the Graphicx library (default: the external/Graphicx submodule)
#   STATIC=0             Linux: link dynamically (default 1 = fully static)
#   CXX=...              compiler (g++ / clang++); musl-gcc style wrappers work too
#
# No third-party library is needed: USB and sound talk to the kernel (Linux
# usbfs and ALSA) or to the OS frameworks (macOS IOKit, AudioToolbox).

CXX      ?= c++
GRAPHICX ?= external/Graphicx
STATIC   ?= 1
CXXFLAGS ?= -O2
CXXFLAGS += -std=c++11 -Wall -Wextra -Wpedantic -MMD -MP -I$(GRAPHICX)/include -pthread

UNAME := $(shell uname -s)
ifeq ($(UNAME),Darwin)
  LDLIBS  += -framework IOKit -framework CoreFoundation -framework AudioToolbox
else
  ifeq ($(STATIC),1)
    LDFLAGS += -static
  endif
  LDLIBS  += -pthread
endif

ifeq ($(wildcard $(GRAPHICX)/include/graphicx/widgets.h),)
  $(error Graphicx not found at $(GRAPHICX): run "git submodule update --init" (or set GRAPHICX=path))
endif

GFX_SRCS := $(addprefix $(GRAPHICX)/src/,graphicx.cpp terminal.cpp color.cpp canvas.cpp ui.cpp widgets.cpp)
DRV_SRCS := $(wildcard src/usb/*.cpp) $(wildcard src/rtlsdr/*.cpp)
DSP_SRCS := $(wildcard src/dsp/*.cpp)
AUD_SRCS := $(wildcard src/audio/*.cpp)
APP_SRCS := $(wildcard src/app/*.cpp)

obj = $(patsubst %.cpp,build/%.o,$(subst $(GRAPHICX)/,gfx/,$(1)))
GFX_OBJS := $(call obj,$(GFX_SRCS))
DRV_OBJS := $(call obj,$(DRV_SRCS))
DSP_OBJS := $(call obj,$(DSP_SRCS))
AUD_OBJS := $(call obj,$(AUD_SRCS))
APP_OBJS := $(call obj,$(APP_SRCS))

.PHONY: all tools test clean check-static

all: bin/offgrid

bin/offgrid: $(APP_OBJS) $(DSP_OBJS) $(DRV_OBJS) $(AUD_OBJS) $(GFX_OBJS)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

tools: bin/rtl-probe

bin/rtl-probe: build/tools/rtl-probe.o $(DRV_OBJS)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

bin/test-dsp: build/tests/test_dsp.o $(DSP_OBJS)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test: bin/test-dsp
	./bin/test-dsp

build/gfx/%.o: $(GRAPHICX)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

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
