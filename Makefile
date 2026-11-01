#
# LogicPort: an Intronix LogicPort LA1034 logic analyzer on the AMI tool kit
# (graphical model, static).
#
# AMIDIR is where amitk lives; ". setpath" in amitk sets it, otherwise the
# default below is used. The analyzer link needs libusb-1.0.
#
UNAME := $(shell uname 2>/dev/null)
ifeq ($(OS),Windows_NT)
    OSTYPE := Windows_NT
else ifneq (,$(filter MINGW% MSYS% CYGWIN%,$(UNAME)))
    OSTYPE := Windows_NT
else
    OSTYPE := $(UNAME)
endif

ifeq ($(OSTYPE),Windows_NT)
    AMIDIR ?= ../amitk
else
    AMIDIR ?= $(HOME)/projects/amitk
endif
AMILIB = $(AMIDIR)/lib/libami_graph.a

ifeq ($(OSTYPE),Windows_NT)
    INCS = $(shell pkg-config --cflags libusb-1.0) -DSTDIO_BYPASS
    LIBS = $(shell pkg-config --libs --static libusb-1.0) \
           -lwinmm -lgdi32 -lcomdlg32 -lssl -lcrypto -lws2_32 -lcrypt32
    KEEP =
    AMILINK = -static -Wl,--whole-archive $(AMILIB) -Wl,--no-whole-archive
    CAPTURE = $(AMIDIR)/windows/screen_capture.o
    EXE = .exe
else ifeq ($(OSTYPE),Darwin)
    BREW := /opt/homebrew/bin/brew
    SSL_PREFIX := $(shell $(BREW) --prefix openssl@3)
    USB_PREFIX := $(shell $(BREW) --prefix libusb)
    INCS = -I$(USB_PREFIX)/include/libusb-1.0
    LIBS = $(SSL_PREFIX)/lib/libssl.dylib $(SSL_PREFIX)/lib/libcrypto.dylib \
           $(USB_PREFIX)/lib/libusb-1.0.dylib \
           -framework CoreFoundation -framework CoreGraphics \
           -framework ImageIO -framework CoreMIDI -framework AudioToolbox \
           -framework Cocoa -framework CoreText -framework QuartzCore \
           -framework IOKit
    KEEP = -Wl,-u,_ami_maxx
    AMILINK = -Wl,-force_load,$(AMILIB) -Wl,-e,_pa_entry
    CAPTURE = $(AMIDIR)/macosx/screen_capture.o
else
    INCS = $(shell pkg-config --cflags libusb-1.0)
    LIBS = $(shell pkg-config --libs libusb-1.0) \
           -lasound -lfluidsynth -lssl -lcrypto -lwayland-client -lwayland-cursor \
           -lxkbcommon -lfreetype -lfontconfig -lm -lpthread
    KEEP = -Wl,-u,ami_maxx
    AMILINK = $(AMIDIR)/stub/keeper.o $(AMILIB)
    CAPTURE = $(AMIDIR)/linux/wayland/screen_capture.o
endif

CC = gcc
CFLAGS = -g3 -Wall -I$(AMIDIR)/include -I$(AMIDIR)/libc $(INCS)

SRCS = logicport.c la.c

all: logicport

logicport: $(SRCS) la.h $(AMILIB)
	$(CC) $(CFLAGS) $(SRCS) $(KEEP) $(AMILINK) $(LIBS) -lm -o $@$(EXE)

# test build: takes a picture of its own window, see LOGICPORT_SHOT in logicport.c
shot: $(SRCS) la.h $(AMILIB)
	$(CC) $(CFLAGS) -DSHOT $(SRCS) $(CAPTURE) $(KEEP) $(AMILINK) $(LIBS) $(shell pkg-config --libs libpng) -lm -o logicport_shot$(EXE)

sim: logicport
	./logicport$(EXE) --sim

clean:
	rm -rf logicport logicport_shot logicport.exe logicport_shot.exe logicport.dSYM

.PHONY: all sim shot clean
