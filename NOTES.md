# Intronix LogicPort LA1034 on Linux

## The program (2026-11-01)
logicport.c + la.c/la.h: an AMI tool kit logic analyzer for the LA1034, see README.md.
Verified on this unit (serial 51131559): FPGA load of images 7 (timing SDR) and 6
(500 MHz DDR, reconfiguration path), 2048 sample captures at 100 MHz, RLE captures at
1 MHz with time-bounded fills in repeat mode, trigger register programming. Inputs were
floating during the tests, so sample data has only been checked for sanity (one run,
plausible levels); the simulator (--sim) exercised the drawing, trigger, cursors, zoom
and the CSV/VCD export. State mode and real signals are untested.

Ami notes learned: ami_linewidth is in device pixels, not view units (text and figure
coordinates do follow ami_viewscale); the panel scales widths itself (lwidth()).

## Hardware on this machine (2026-11-01)
- USB ID 0403:dc48, iManufacturer "Intronix", iProduct "LogicPort Logic Analyzer"
- Serial 51131559, bcdDevice 4.00, full speed (12 Mbps), FT245BL FIFO + Altera Cyclone EP1C4
- sysfs /sys/bus/usb/devices/1-4.4 ; node /dev/bus/usb/001/013 (root:root 664, no udev rule yet)
- No kernel driver bound (correct: accessed via libusb/libftdi, not ftdi_sio)

Installed 2026-11-01 as /etc/udev/rules.d/99-logicport.rules (needed for non-root access):
    SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="dc48", MODE="0666", TAG+="uaccess"

## Vendor files (vendor/)
- logicport_2371.exe  — last vendor release (FileVersion 3.4.0.2371, (c) 2005-2019, Lindersoft SetupBuilder 10)
- logicporthelp_english_2371.zip — English help files
- intronix-logicport.ccf — FPGA bitstreams extracted from the installer (8 Altera RBF images,
  bit-bang encoded, XOR-obfuscated header). Byte-identical to fixtures/vendor/LogicPort.ccf in
  EmbeddedAndroid/stimulus. Hashes in SHA256SUMS.
- sigrok-fwextract-intronix-logicport — extractor script from sigrok-util PR #22 (GPLv3)

Site status: pctestinstruments.com live, copyright line 2005-2023, LA1034 still listed.

## Software status (verified 2026-11-01)
- libsigrok PR #315 (mangelajo, 2026-10-23): full driver, 1 kHz-500 MHz, 2048 samples + RLE,
  hw triggers. CLOSED 2026-10-26 by maintainer biot with comment "AI slop"; NOT merged.
  Driver still usable from fork branch mangelajo:intronix-logicport.
- sigrok-util PR #22: extractor tool, still open.
- EmbeddedAndroid/stimulus: Rust daemon, web UI + REST + MCP, 7 decoders, Docker-based,
  Apache-2.0, last push 2026-09-04. Ships the .ccf in-repo.
