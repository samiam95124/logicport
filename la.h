/*
 * Intronix LogicPort LA1034 link: the analyzer over USB (libusb), its FPGA
 * configuration, acquisition and sample memory readout, run on a thread of
 * its own so the window never waits on the bus.
 *
 * The instrument is an FTDI FT245BL USB FIFO in front of an Altera Cyclone
 * FPGA with no firmware of its own: the host loads one of eight FPGA
 * images (the vendor's LogicPort.ccf) through the FTDI chip in bit-bang
 * mode, then talks to the configured FPGA through the FIFO with a small
 * register protocol. The protocol is as worked out by the libsigrok driver
 * (Miguel Angel Ajo Pelayo, 2026) and the stimulus project, see README.
 *
 * 34 channels: D0 to D31 and the two clock inputs CLK1, CLK2. Bit k of a
 * sample is channel k. Sample memory is 2048 entries deep; with
 * compression (RLE) an entry can stand for up to 2^35 unchanging sample
 * periods, so a capture is handed over as a list of runs.
 *
 * The front panel owns the settings and sends them with la_apply(); the
 * link thread picks them up when it next arms. la_poll() copies the
 * thread's state out and hands over a finished capture, which the panel
 * then owns and frees with la_free_capture().
 */
#ifndef LA_H
#define LA_H

#include <stdint.h>

#define LA_VID        0x0403
#define LA_PID        0xdc48

#define LA_NCH        34
#define LA_CH_CLK1    32
#define LA_CH_CLK2    33
#define LA_DEPTH      2048
#define LA_ALLCH      ((UINT64_C(1) << LA_NCH) - 1)

#define LA_NRATES     19       /* 1 kHz to 500 MHz in 1-2-5 steps */
#define LA_RLE_MAXRATE 200000000ULL /* compression works up to here */

#define LA_THRESH_MIN  -6.0
#define LA_THRESH_MAX   6.0
#define LA_THRESH_STEP  0.05

/* a channel's trigger condition */
enum { TC_X, TC_LOW, TC_HIGH, TC_RISE, TC_FALL, TC_EDGE, TC_COUNT };

/* run modes */
enum { LA_STOP, LA_SINGLE, LA_REPEAT };

/* acquisition state */
enum { AS_IDLE, AS_CONFIG, AS_PREFILL, AS_ARMED, AS_POSTFILL, AS_READING };

/* RLE: the capture window, in samples, that bounds the pre- and
   post-trigger fills in time (an idle input never fills the memory) */
#define LA_NWINDOWS 6
extern const uint64_t la_windows[LA_NWINDOWS];

typedef struct {
    int      rate_ix;          /* index into la_rates (timing mode) */
    int      rle;              /* compression on */
    int      window_ix;        /* RLE window, index into la_windows */
    double   threshold;        /* V, 50 mV steps */
    int      pretrig;          /* % of the memory before the trigger */
    int      state;            /* state mode: sample on an external clock */
    int      clk_src;          /* state: 0 CLK1, 1 CLK2 */
    int      clk_fall;         /* state: sample on the falling edge */
    uint8_t  cond[LA_NCH];     /* trigger condition per channel, TC_* */
    uint64_t mask;             /* enabled channels: RLE watches only these */
} la_settings;

typedef struct {
    uint64_t value;            /* 34 bits of channel state */
    uint64_t len;              /* sample periods it was held */
} la_run;

typedef struct {
    la_run   *runs;
    int       nruns;
    uint64_t  total;           /* samples in all runs */
    int64_t   trigger;         /* sample index of the trigger, -1 if none */
    uint64_t  rate;            /* Hz, 0 in state mode */
    int       state;           /* state mode capture */
    int       rle;
    uint64_t  mask;            /* channels that were enabled */
    unsigned  seq;             /* goes up with each capture */
    int       sim;
} la_capture;

typedef struct {
    /* link */
    int    connected;          /* device open and the FPGA answering */
    int    sim;                /* simulated analyzer, no hardware */
    char   status[128];        /* short human readable link status */
    char   serial[32];
    int    image;              /* FPGA image loaded, -1 none */
    int    cal;                /* threshold DAC calibration from the EEPROM */
    int    have_ccf;           /* the FPGA image file was found */
    char   ccf[512];           /* and where */

    /* acquisition */
    int    run_mode;           /* LA_* */
    int    acq;                /* AS_* */
    long   captures;           /* since start */
    long   errors;             /* bus errors since start */
    long   acq_ms;             /* time the current acquisition has been armed */

    /* a finished capture handed over by la_poll, NULL if none new */
    la_capture *newcap;
} la_state;

typedef struct {
    uint64_t rate;
    int      ddr;
    uint8_t  prescaler, step;
    const char *name;
} la_rate;

extern const la_rate la_rates[LA_NRATES];

/* Open the analyzer and start the link thread. sim nonzero simulates. */
void la_init(la_state *s, const la_settings *set, int sim);

/* Service the link: copy the thread's state out. Returns nonzero if changed.
   A new capture, if any, is in s->newcap and is now the caller's. */
int la_poll(la_state *s);

/* Stop the thread and close the device. */
void la_shutdown(la_state *s);

/* The settings to use from the next arm on. A running acquisition is
   re-armed with them. */
void la_apply(la_state *s, const la_settings *set);

/* Requests, queued for the thread. */
void la_start(la_state *s, int mode);   /* LA_SINGLE, LA_REPEAT, LA_STOP */
void la_force(la_state *s);             /* trigger now */

void la_free_capture(la_capture *c);

/* Capture export, whole capture: returns bytes written or -1. */
long la_save_csv(const la_capture *c, const char *names[LA_NCH], const char *path);
long la_save_vcd(const la_capture *c, const char *names[LA_NCH], const char *path);

#endif
