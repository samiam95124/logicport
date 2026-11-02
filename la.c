/*
 * Intronix LogicPort LA1034 link. See la.h.
 *
 * The FTDI FT245BL is driven through libusb directly: the handful of
 * vendor control requests it takes (reset and purge, latency timer, baud
 * divisor, bit-bang mode, pin read, EEPROM read) and two bulk endpoints.
 * Every 64 byte packet it sends begins with two modem status bytes, which
 * are stripped here. The baud divisor only matters for the bit-bang clock
 * during FPGA configuration.
 *
 * Register protocol, once the FPGA is configured: a command is five bytes,
 * opcode, 16 bit offset, 16 bit count minus one (little endian), then the
 * write data; the FPGA answers with the opcode and a 16 bit big endian
 * sequence number, then any read data. Opcode 0xc3 selects the upper 16
 * bits of the 32 bit register address, 0xc1 writes, 0xc2 reads. The
 * sequence number lives on in the FPGA across host sessions, so a flood
 * of zeros and a 0x01 reset the parser and the counter on every open.
 *
 * All of this is from the libsigrok driver for the device, cross checked
 * against the stimulus project's protocol notes. The FPGA images are the
 * vendor's own, read from LogicPort.ccf (see la_find_ccf below).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <libusb.h>

#include <localdefs.h>
#include <services.h>

#include "la.h"

/* ------------------------------------------------------------ FTDI */

#define FTDI_OUT        0x40
#define FTDI_IN         0xc0
#define SIO_RESET       0x00
#define SIO_SET_BAUD    0x03
#define SIO_SET_LATENCY 0x09
#define SIO_SET_BITMODE 0x0b
#define SIO_READ_PINS   0x0c
#define SIO_READ_EEPROM 0x90
#define SIO_RESET_SIO   0
#define SIO_PURGE_OUT   1     /* the chip's receive buffer: host to device */
#define SIO_PURGE_IN    2     /* the chip's transmit buffer: device to host */
#define BITMODE_RESET   0x00
#define BITMODE_BITBANG 0x01
#define BAUD_460800     0x4006  /* FT232B/FT245B divisor encoding for 460800 */
#define LATENCY_MS      4
#define PKT             64      /* bulk packet size, status bytes included */
#define RXBUF           65536

/* bit-bang pins for passive serial configuration */
#define BB_DATA0        (1 << 0)
#define BB_DCLK         (1 << 1)
#define BB_NCONFIG      (1 << 2)
#define BB_NSTATUS      (1 << 3)
#define BB_CONF_DONE    (1 << 4)
#define BB_OUTPUTS      (BB_DATA0 | BB_DCLK | BB_NCONFIG)
#define BB_CHUNK        10000
#define BB_EXTRA_CLOCKS 32

/* ------------------------------------------------------------ registers */

#define CMD_WRITE       0xc1
#define CMD_READ        0xc2
#define CMD_SEGMENT     0xc3
#define CMD_RESYNC      0x01

#define REG_STATUS          0x00100000  /* R */
#define REG_CONFIG_CTRL     0x00100000  /* W: 0 releases the image */
#define REG_ARM             0x00100002
#define REG_RUN             0x00100003
#define REG_PRE_DEPTH       0x00100004
#define REG_POST_DEPTH      0x00100006
#define REG_THRESHOLD       0x00100008
#define REG_END_PREFILL     0x0010000a  /* W */
#define REG_UNFILLED        0x0010000a  /* R */
#define REG_END_POSTFILL    0x0010000b
#define REG_FORCE_TRIGGER   0x0010000c
#define REG_TRIGGER_COMBINE 0x0010000d
#define REG_IMAGE_ID        0x0010000f

#define REG_TRIGGER_A       0x00200000
#define REG_TRIGGER_B       0x00600000
#define TRIG_PATTERN_MASK   0x00
#define TRIG_PATTERN_VALUE  0x05
#define TRIG_COUNT_HIGH     0x0a
#define TRIG_COUNT_LOW      0x0e
#define TRIG_CONDITION      0x14
#define TRIG_COUNTER_SOURCE 0x16
#define TRIG_COUNTER_MODE   0x17
#define TRIG_CONSECUTIVE    0x18
#define TRIG_PREQUALIFY     0x19
#define TRIG_RISING_MASK    0x29
#define TRIG_FALLING_MASK   0x2e

#define REG_CLOCK_MODE      0x01000000  /* W */
#define REG_TRIGGER_POS     0x01000000  /* R */
#define REG_CHANNEL_CFG     0x01000001
#define REG_COMPRESSION     0x01000002
#define REG_COMPRESSION_MASK 0x01000003
#define REG_INVERT_MASK     0x01000008

#define REG_WRITE_PTR       0x02000000
#define REG_WRITE_PTR_B     0x02000100
#define MEM_LANES_A         0x02001000
#define MEM_LANES_B         0x02005000
#define MEM_LANE_STRIDE     0x1000
#define MEM_FLAGS           0x02009000

#define REG_PRESCALER       0x20000000
#define REG_STEP            0x20000001
#define REG_INIT_COUNTER    0x40000005

#define STATUS_DONE         (1 << 0)
#define STATUS_WRAPPED      (1 << 1)
#define STATUS_TRIGGERED    (1 << 5)
#define STATUS_PREFILLED    (1 << 6)

#define FLAG_CLK1           (1 << 1)
#define FLAG_CLK2           (1 << 2)
#define FLAG_COUNT          (1 << 3)
#define FLAG_DDR_B_SHIFT    4

#define CLOCK_MODE_CLK1     (1 << 2)
#define CLOCK_MODE_CLK2     (1 << 1)
#define CLOCK_MODE_FALLING  (1 << 0)
#define CLOCK_MODE_NO_QUAL  (1 << 4)

#define COND_PATTERN_TRUE   1
#define COUNTER_SRC_EDGE    3
#define COUNTER_MODE_OCCURRENCE 1

#define TRIGGER_DELAY_SDR   7
#define TRIGGER_DELAY_DDR   21
#define THRESHOLD_SETTLE_MS 200

enum {
    IMAGE_STATE_AFTER = 0, IMAGE_STATE_BEFORE_CLK1_RISE, IMAGE_STATE_BEFORE_CLK2_RISE,
    IMAGE_STATE_BEFORE_CLK1_FALL, IMAGE_STATE_BEFORE_CLK2_FALL, IMAGE_STATE_ON,
    IMAGE_TIMING_DDR, IMAGE_TIMING_SDR, IMAGE_COUNT
};

#define CCF_KEY         0x916662
#define CCF_SWAP_BLOCK  4096

#define ACK_MS          1000
#define READ_MS         2000
#define PROBE_MS        250
#define LINK_MS         3000
#define RETRY_MS        2000
#define POLL_MS         10

const la_rate la_rates[LA_NRATES] = {
    { 1000ULL,      0, 0x61, 0x00, "1 kHz"   },
    { 2000ULL,      0, 0x51, 0x44, "2 kHz"   },
    { 5000ULL,      0, 0x51, 0x11, "5 kHz"   },
    { 10000ULL,     0, 0x51, 0x00, "10 kHz"  },
    { 20000ULL,     0, 0x41, 0x44, "20 kHz"  },
    { 50000ULL,     0, 0x41, 0x11, "50 kHz"  },
    { 100000ULL,    0, 0x41, 0x00, "100 kHz" },
    { 200000ULL,    0, 0x31, 0x44, "200 kHz" },
    { 500000ULL,    0, 0x31, 0x11, "500 kHz" },
    { 1000000ULL,   0, 0x31, 0x00, "1 MHz"   },
    { 2000000ULL,   0, 0x21, 0x44, "2 MHz"   },
    { 5000000ULL,   0, 0x21, 0x11, "5 MHz"   },
    { 10000000ULL,  0, 0x21, 0x00, "10 MHz"  },
    { 20000000ULL,  0, 0x11, 0x44, "20 MHz"  },
    { 50000000ULL,  0, 0x11, 0x11, "50 MHz"  },
    { 100000000ULL, 0, 0x11, 0x00, "100 MHz" },
    { 200000000ULL, 0, 0x01, 0x00, "200 MHz" },
    { 250000000ULL, 0, 0x00, 0x00, "250 MHz" },
    { 500000000ULL, 1, 0x00, 0x00, "500 MHz" },
};

const uint64_t la_windows[LA_NWINDOWS] = {
    16384, 65536, 262144, 1048576, 4194304, 16777216
};

/* ------------------------------------------------------------ shared state */

/* Everything here is guarded by lk. The link thread works on its own copy
   (st) and publishes it; the front panel copies it out in la_poll. */
static ami_long     lk;
static la_state     shared;
static int          shared_changed;
static la_settings  pending;          /* settings from the panel */
static int          pending_new;
static int          req_mode = -1;    /* run mode requested, -1 none */
static int          req_force;
static int          stop_req, thread_done;
static unsigned     cap_seq;

/* ------------------------------------------------------------ thread state */

static la_state              st;
static la_settings           set;        /* settings in use */
static libusb_context       *ctx;
static libusb_device_handle *dev;
static uint8_t               ep_in, ep_out;
static uint8_t               rxbuf[RXBUF];
static int                   rxhead, rxtail;
static uint16_t              seq;        /* expected sequence of the next ack */
static int                   segment = -1;
static int                   io_error;
static int                   image = -1;
static int                   dac_last = -1;
static long                  next_retry;
static int                   trace;

/* the eight FPGA images, as bit-bang streams ready to send */
static uint8_t *images[IMAGE_COUNT];
static size_t   image_size[IMAGE_COUNT];

/* acquisition in progress */
static long     acq_start, trigger_seen;
static long     prefill_ms, postfill_ms;
static int      prefill_ended, postfill_ended;
static uint64_t compression_mask;
static int      trigger_used;
static uint8_t  raw[5][LA_DEPTH];      /* lanes D0-7 .. D24-31, then flags */

static long now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long)ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void sleep_ms(int ms)
{
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };

    nanosleep(&ts, NULL);
}

static void set_status(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(st.status, sizeof st.status, fmt, ap);
    va_end(ap);
    if (trace) fprintf(stderr, "la: %s\n", st.status);
}

static void dbg(const char *fmt, ...)
{
    va_list ap;

    if (!trace) return;
    va_start(ap, fmt);
    fprintf(stderr, "la: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
}

static void publish(void)
{
    la_capture *keep;

    ami_lock(lk);
    keep = shared.newcap;      /* an unconsumed capture stays until taken */
    shared = st;
    shared.newcap = keep;
    shared_changed = 1;
    ami_unlock(lk);
}

static void publish_capture(la_capture *c)
{
    ami_lock(lk);
    if (shared.newcap) la_free_capture(shared.newcap);
    c->seq = ++cap_seq;
    shared.newcap = c;
    shared.captures = ++st.captures;
    shared_changed = 1;
    ami_unlock(lk);
}

/* ------------------------------------------------------------ FTDI over libusb */

static int ftdi_ctl(uint8_t req, uint16_t value, uint16_t index)
{
    int r = libusb_control_transfer(dev, FTDI_OUT, req, value, index, NULL, 0, 1000);

    if (r < 0) dbg("control 0x%02x: %s", req, libusb_strerror(r));
    return r < 0 ? -1 : 0;
}

static void rx_clear(void) { rxhead = rxtail = 0; }

static int purge_in(void)
{
    rx_clear();
    return ftdi_ctl(SIO_RESET, SIO_PURGE_IN, 0);
}

static int purge_out(void) { return ftdi_ctl(SIO_RESET, SIO_PURGE_OUT, 0); }

static int purge_both(void)
{
    if (purge_out() < 0) return -1;
    return purge_in();
}

static int set_bitmode(uint8_t mask, uint8_t mode)
{
    return ftdi_ctl(SIO_SET_BITMODE, (uint16_t)(mode << 8) | mask, 0);
}

static int read_pins(uint8_t *pins)
{
    int r = libusb_control_transfer(dev, FTDI_IN, SIO_READ_PINS, 0, 0, pins, 1, 1000);

    return r == 1 ? 0 : -1;
}

/* reset the chip in place and set it up again: latency, bit-bang baud */
static int ftdi_setup(void)
{
    if (ftdi_ctl(SIO_RESET, SIO_RESET_SIO, 0) < 0) return -1;
    if (ftdi_ctl(SIO_SET_LATENCY, LATENCY_MS, 0) < 0) return -1;
    if (ftdi_ctl(SIO_SET_BAUD, BAUD_460800, 0) < 0) return -1;
    rx_clear();
    return 0;
}

static int write_raw(const uint8_t *buf, size_t len)
{
    int got, r;

    while (len) {
        int n = len > 4096 ? 4096 : (int)len;

        r = libusb_bulk_transfer(dev, ep_out, (uint8_t *)buf, n, &got, 2000);
        if (r < 0 || got != n) {
            dbg("bulk write: %s (%d of %d)", r < 0 ? libusb_strerror(r) : "short", got, n);
            return -1;
        }
        buf += n;
        len -= n;
    }
    return 0;
}

/* pull whatever the chip has into rxbuf, status bytes stripped; returns
   payload bytes added, or -1 on a bus error */
static int rx_fill(int timeout_ms)
{
    uint8_t pkt[4096];
    int got = 0, r, i, n = 0;

    r = libusb_bulk_transfer(dev, ep_in, pkt, sizeof pkt, &got, timeout_ms);
    if (r < 0 && r != LIBUSB_ERROR_TIMEOUT) {
        dbg("bulk read: %s", libusb_strerror(r));
        return -1;
    }
    for (i = 0; i < got; i += PKT) {
        int k = got - i < PKT ? got - i : PKT;

        if (k <= 2) continue;
        if (rxtail + k - 2 > RXBUF) {
            /* compact */
            memmove(rxbuf, rxbuf + rxhead, rxtail - rxhead);
            rxtail -= rxhead;
            rxhead = 0;
            if (rxtail + k - 2 > RXBUF) { dbg("rx overflow"); return -1; }
        }
        memcpy(rxbuf + rxtail, pkt + i + 2, k - 2);
        rxtail += k - 2;
        n += k - 2;
    }
    return n;
}

/* read exactly len payload bytes, within timeout */
static int read_raw(uint8_t *buf, size_t len, int timeout_ms)
{
    long deadline = now_ms() + timeout_ms;

    while ((size_t)(rxtail - rxhead) < len) {
        long left = deadline - now_ms();
        int r;

        if (left <= 0) {
            dbg("read timeout (%d of %zu bytes)", rxtail - rxhead, len);
            return -2;
        }
        r = rx_fill(left > 50 ? 50 : (int)left);
        if (r < 0) return -1;
    }
    memcpy(buf, rxbuf + rxhead, len);
    rxhead += len;
    if (rxhead == rxtail) rxhead = rxtail = 0;
    return 0;
}

/* ------------------------------------------------------------ register protocol */

static int send_cmd(uint8_t cmd, uint16_t arg0, uint16_t arg1, const uint8_t *wdata, size_t wlen,
                    uint8_t *rdata, size_t rlen, int timeout_ms)
{
    uint8_t buf[5 + 8], ack[3];
    int r;

    if (wlen > 8) return -1;
    if (io_error) return -1;   /* out of step: fail fast until the next resync */
    buf[0] = cmd;
    buf[1] = arg0 & 0xff; buf[2] = arg0 >> 8;
    buf[3] = arg1 & 0xff; buf[4] = arg1 >> 8;
    if (wlen) memcpy(buf + 5, wdata, wlen);
    r = write_raw(buf, 5 + wlen);
    if (r == 0) r = read_raw(ack, 3, timeout_ms);
    if (r == 0 && (ack[0] != cmd || ((ack[1] << 8) | ack[2]) != seq)) {
        dbg("bad ack %02x %02x %02x for command %02x seq %u", ack[0], ack[1], ack[2], cmd, seq);
        r = -1;
    }
    if (r == 0 && rlen) r = read_raw(rdata, rlen, timeout_ms);
    if (r != 0) {
        segment = -1;
        io_error = 1;
        return r;
    }
    seq++;
    return 0;
}

static int set_segment(uint32_t addr)
{
    uint16_t sg = addr >> 16;
    int r;

    if (segment == sg) return 0;
    r = send_cmd(CMD_SEGMENT, sg, 0, NULL, 0, NULL, 0, ACK_MS);
    if (r) return r;
    segment = sg;
    return 0;
}

static int reg_write(uint32_t addr, const uint8_t *data, size_t len)
{
    int r = set_segment(addr);

    if (r) return r;
    return send_cmd(CMD_WRITE, addr & 0xffff, len - 1, data, len, NULL, 0, ACK_MS);
}

static int reg_read_to(uint32_t addr, uint8_t *data, size_t len, int timeout_ms)
{
    int r = set_segment(addr);

    if (r) return r;
    return send_cmd(CMD_READ, addr & 0xffff, len - 1, NULL, 0, data, len, timeout_ms);
}

static int reg_read(uint32_t addr, uint8_t *data, size_t len)
{
    return reg_read_to(addr, data, len, READ_MS);
}

static int reg_write_u8(uint32_t addr, uint8_t v) { return reg_write(addr, &v, 1); }
static int reg_read_u8(uint32_t addr, uint8_t *v) { return reg_read(addr, v, 1); }

static int reg_read_u16(uint32_t addr, uint16_t *v)
{
    uint8_t b[2];
    int r = reg_read(addr, b, 2);

    if (r == 0) *v = b[0] | (b[1] << 8);
    return r;
}

static int reg_write_n(uint32_t addr, uint64_t value, size_t width)
{
    uint8_t b[8];
    size_t i;

    for (i = 0; i < width; i++) b[i] = (value >> (8 * i)) & 0xff;
    return reg_write(addr, b, width);
}

/* ------------------------------------------------------------ sync and probing */

/* Resynchronize the FPGA's command parser: drain any half received command
   of maximum length with zeros, then 0x01 resets the parser and the
   sequence counter. Optionally reset the FTDI chip first. */
static int resync_once(int reset)
{
    static uint8_t zeros[65535];
    uint8_t cmd = CMD_RESYNC;

    segment = -1;
    if (reset) {
        sleep_ms(50);
        if (ftdi_setup() < 0) return -1;
    }
    if (purge_both() < 0) return -1;
    if (write_raw(zeros, sizeof zeros) < 0) return -1;
    if (write_raw(zeros, 30000) < 0) return -1;
    if (write_raw(&cmd, 1) < 0) return -1;
    sleep_ms(50);
    if (purge_both() < 0) return -1;
    seq = 0;
    io_error = 0;
    return 0;
}

static int resync(int reset, int attempts)
{
    int i;

    for (i = 0; i < attempts; i++) {
        if (resync_once(reset || i > 0) == 0) return 0;
        dbg("resync attempt %d failed", i + 1);
        if (i + 1 < attempts) sleep_ms(500);
    }
    return -1;
}

/* which image is loaded; an unconfigured FPGA does not answer */
static int probe_image(void)
{
    uint8_t id;

    image = -1;
    if (reg_read_to(REG_IMAGE_ID, &id, 1, PROBE_MS) != 0) {
        dbg("no reply from the FPGA: unconfigured");
        io_error = 0;   /* expected: not a link fault */
        return -1;
    }
    if ((id & 0xf0) != 0x10 || (id & 0x0f) >= IMAGE_COUNT) {
        dbg("unexpected FPGA image id 0x%02x", id);
        return -1;
    }
    image = id & 0x0f;
    dbg("FPGA image %d loaded", image);
    return 0;
}

/* The threshold calibration is a signed decimal string ("+003") at the
   start of the EEPROM user area, after the string descriptors and a four
   byte PnP block. The EEPROM is only ever read. */
static int read_calibration(void)
{
    static const uint8_t pnp[4] = { 0x02, 0x03, 0x01, 0x00 };
    uint8_t buf[128];
    char str[5];
    size_t pos;
    int w, r;

    st.cal = 0;
    for (w = 0; w < 64; w++) {
        r = libusb_control_transfer(dev, FTDI_IN, SIO_READ_EEPROM, 0, w, buf + 2 * w, 2, 1000);
        if (r != 2) { dbg("EEPROM read failed at word %d", w); return -1; }
    }
    pos = (buf[0x12] & 0x7f) + buf[0x13];
    if (pos + 4 <= sizeof buf && !memcmp(buf + pos, pnp, 4)) pos += 4;
    if (pos + 4 > sizeof buf - 2 || (buf[pos] != '+' && buf[pos] != '-') ||
        buf[pos + 1] < '0' || buf[pos + 1] > '9' || buf[pos + 2] < '0' || buf[pos + 2] > '9' ||
        buf[pos + 3] < '0' || buf[pos + 3] > '9') {
        dbg("no threshold calibration in the EEPROM");
        return -1;
    }
    memcpy(str, buf + pos, 4);
    str[4] = 0;
    st.cal = atoi(str);
    dbg("threshold calibration %d", st.cal);
    return 0;
}

/* ------------------------------------------------------------ the FPGA images */

/* LogicPort.ccf: a text line of nine XOR obfuscated decimal numbers, the
   eight image sizes in 16 byte units and their XOR, then the images, each a
   bit-bang stream with its first two 4 KiB blocks swapped. */
static int load_ccf(const char *path)
{
    FILE    *f = fopen(path, "rb");
    char     line[160];
    uint32_t units[IMAGE_COUNT + 1], check = 0;
    char    *p, *e;
    int      i, k;
    long     total;
    uint8_t  tmp;

    if (!f) return -1;
    if (!fgets(line, sizeof line, f)) { fclose(f); return -1; }
    p = line;
    for (i = 0; i <= IMAGE_COUNT; i++) {
        units[i] = (uint32_t)strtoul(p, &e, 10) ^ CCF_KEY;
        if (e == p) { fclose(f); return -1; }
        if (i < IMAGE_COUNT) check ^= units[i];
        p = e;
        if (*p == ',') p++;
    }
    if (check != units[IMAGE_COUNT]) { fclose(f); return -1; }
    total = (long)strlen(line);
    for (k = 0; k < IMAGE_COUNT; k++) total += (long)units[k] * 16;
    fseek(f, 0, SEEK_END);
    if (ftell(f) != total) { fclose(f); return -1; }
    fseek(f, (long)strlen(line), SEEK_SET);
    for (k = 0; k < IMAGE_COUNT; k++) {
        size_t n = (size_t)units[k] * 16;

        if (n < 2 * CCF_SWAP_BLOCK) { fclose(f); return -1; }
        images[k] = malloc(n);
        if (!images[k] || fread(images[k], 1, n, f) != n) { fclose(f); return -1; }
        image_size[k] = n;
        for (i = 0; i < CCF_SWAP_BLOCK; i++) {
            tmp = images[k][i];
            images[k][i] = images[k][CCF_SWAP_BLOCK + i];
            images[k][CCF_SWAP_BLOCK + i] = tmp;
        }
        /* must start with the Altera RBF header: 16 x 0xff then 0x6a, two
           stream bytes per bit, data in bit 0 */
        for (i = 0; i < 17; i++) {
            uint8_t byte = 0;
            int b;

            for (b = 0; b < 8; b++) byte |= (images[k][(i * 8 + b) * 2] & 1) << b;
            if (byte != (i < 16 ? 0xff : 0x6a)) { fclose(f); return -1; }
        }
    }
    fclose(f);
    return 0;
}

/* look for the image file in the usual places */
static int find_ccf(void)
{
    static const char *names[] = { "intronix-logicport.ccf", "LogicPort.ccf" };
    char path[512], exe[512], usr[256];
    const char *env = getenv("LOGICPORT_CCF");
    const char *dirs[8];
    int nd = 0, d, n;
    size_t len;

    st.have_ccf = 0;
    if (env && *env && load_ccf(env) == 0) { snprintf(st.ccf, sizeof st.ccf, "%s", env); st.have_ccf = 1; return 0; }
    dirs[nd++] = "vendor";
    dirs[nd++] = ".";
    /* the toolkit knows where the executable lives on every platform */
    ami_getpgm(exe, sizeof exe - 1);
    exe[sizeof exe - 1] = 0;
    len = strlen(exe);
    while (len > 0 && (exe[len - 1] == '/' || exe[len - 1] == '\\')) exe[--len] = 0;
    if (len > 0) { strncat(exe, "/vendor", sizeof exe - strlen(exe) - 1); dirs[nd++] = exe; }
    ami_getusr(usr, sizeof usr - 1);
    usr[sizeof usr - 1] = 0;
    strncat(usr, "/.local/share/sigrok-firmware", sizeof usr - strlen(usr) - 1);
    dirs[nd++] = usr;
    dirs[nd++] = "/usr/share/sigrok-firmware";
    dirs[nd++] = "/usr/local/share/sigrok-firmware";
    for (d = 0; d < nd; d++)
        for (n = 0; n < 2; n++) {
            snprintf(path, sizeof path, "%s/%s", dirs[d], names[n]);
            if (load_ccf(path) == 0) {
                snprintf(st.ccf, sizeof st.ccf, "%s", path);
                st.have_ccf = 1;
                return 0;
            }
        }
    return -1;
}

/* ------------------------------------------------------------ FPGA configuration */

static int wait_pins(uint8_t mask)
{
    long deadline = now_ms() + 1000;
    uint8_t pins = 0;

    do {
        if (read_pins(&pins) < 0) return -1;
        if (pins & mask) return 0;
    } while (now_ms() < deadline);
    dbg("timeout waiting for pins 0x%02x (pins 0x%02x)", mask, pins);
    return -1;
}

static int bitbang_configure(const uint8_t *stream, size_t size)
{
    uint8_t pin, extra[2 * BB_EXTRA_CLOCKS];
    size_t off, len;
    int i;

    if (purge_in() < 0) return -1;
    if (set_bitmode(0, BITMODE_RESET) < 0 || set_bitmode(BB_OUTPUTS, BITMODE_BITBANG) < 0) return -1;
    pin = 0;                               /* nCONFIG low: release */
    if (write_raw(&pin, 1) < 0) return -1;
    sleep_ms(10);
    pin = BB_NCONFIG;                      /* nCONFIG high: configure */
    if (write_raw(&pin, 1) < 0) return -1;
    if (wait_pins(BB_NSTATUS) < 0) { set_status("FPGA did not enter configuration mode"); return -1; }
    for (off = 0; off < size; off += len) {
        len = size - off < BB_CHUNK ? size - off : BB_CHUNK;
        if (write_raw(stream + off, len) < 0) return -1;
        if (purge_in() < 0) return -1;    /* the bit-bang echo is of no use */
    }
    if (wait_pins(BB_CONF_DONE) < 0) { set_status("FPGA did not accept the configuration"); return -1; }
    /* Cyclone devices want extra DCLK edges to start initialization */
    for (i = 0; i < BB_EXTRA_CLOCKS; i++) { extra[2 * i] = BB_NCONFIG; extra[2 * i + 1] = BB_NCONFIG | BB_DCLK; }
    if (write_raw(extra, sizeof extra) < 0) return -1;
    if (set_bitmode(0, BITMODE_RESET) < 0) return -1;
    return 0;
}

/* after a power-on configuration the FPGA was seen to ignore commands for
   over a second; poll with short timeouts, resyncing in between */
static int wait_link(void)
{
    long deadline = now_ms() + LINK_MS;
    uint8_t count;

    for (;;) {
        if (reg_read_to(REG_INIT_COUNTER, &count, 1, PROBE_MS) == 0) return 0;
        if (now_ms() > deadline) return -1;
        dbg("FPGA not answering yet, resyncing");
        sleep_ms(100);
        resync(1, 1);
    }
}

/* the FPGA runs an init sequence of about 250 ms after configuration,
   counting; wait for the counter to settle */
static int wait_init_done(void)
{
    long start = now_ms(), stable = start, t;
    uint8_t count;
    int last = -1;

    for (;;) {
        if (reg_read_u8(REG_INIT_COUNTER, &count) != 0) return -1;
        t = now_ms();
        if (count != last) { last = count; stable = t; }
        else if (count && t - stable > 50) break;
        if (t - start > 3000) { dbg("FPGA init did not finish (counter %u)", count); return -1; }
        sleep_ms(5);
    }
    dbg("FPGA init done after %ld ms", t - start);
    return 0;
}

static int configure_fpga(int want)
{
    const char *step = "none";
    int i, r = -1;

    if (image == want) return 0;
    if (!st.have_ccf) { set_status("No LogicPort.ccf: FPGA image file not found"); return -1; }
    set_status("Loading FPGA image %d", want);
    st.acq = AS_CONFIG;
    publish();
    dac_last = -1;
    for (i = 0; i < 3; i++) {
        if (i > 0) {
            dbg("configuration attempt %d failed (%s)", i, step);
            sleep_ms(500);
            /* the image may have loaded and only the link failed */
            if (resync(1, 1) == 0 && probe_image() == 0 && image == want && wait_init_done() == 0) {
                r = 0;
                break;
            }
        }
        /* a configured FPGA ignores nCONFIG until told to release its
           configuration; an unconfigured one does not answer, which is fine */
        if (image >= 0 && reg_write_u8(REG_CONFIG_CTRL, 0x00) != 0) dbg("no ack for release");
        image = -1;
        step = "bit-bang";
        if (bitbang_configure(images[want], image_size[want]) < 0) { r = -1; continue; }
        step = "resync";
        if ((r = resync(1, 5)) != 0) continue;
        step = "link";
        if ((r = wait_link()) != 0) continue;
        step = "init";
        if ((r = wait_init_done()) != 0) continue;
        step = "image check";
        r = probe_image();
        if (r == 0 && image != want) r = -1;
        if (r == 0) break;
    }
    if (r != 0) set_status("Failed to configure the FPGA (%s)", step);
    else st.image = image;
    return r;
}

static int select_image(void)
{
    if (set.state) return IMAGE_STATE_ON;
    if (la_rates[set.rate_ix].ddr) return IMAGE_TIMING_DDR;
    return IMAGE_TIMING_SDR;
}

/* ------------------------------------------------------------ programming */

static uint16_t threshold_dac(double volts, int cal)
{
    double v = (volts - 1.315) * -0.4020381098624974 + 1.315;
    long dac = lround((4.559 - v) / 5.875 * 1023) + cal;

    return dac < 0 ? 0 : dac > 1023 ? 1023 : (uint16_t)dac;
}

static int half_even(int x)
{
    if (!(x & 1)) return x / 2;
    return ((x + 1) / 2) % 2 ? (x - 1) / 2 : (x + 1) / 2;
}

static void buffer_split(uint16_t *pre, uint16_t *post)
{
    int ratio = set.pretrig, pre_depth, post_depth;

    pre_depth = ratio * LA_DEPTH / 100 + 1;
    post_depth = (100 - ratio) * LA_DEPTH / 100 - 1;
    if (!set.state && la_rates[set.rate_ix].ddr) {
        pre_depth = half_even(pre_depth + 0x23);
        post_depth = half_even(post_depth - 0x1e > 0 ? post_depth - 0x1e : 0);
    } else {
        pre_depth += 7;
        post_depth = post_depth - 7 > 0 ? post_depth - 7 : 0;
    }
    *pre = pre_depth;
    *post = post_depth;
}

/* one hardware trigger level from the per channel conditions */
typedef struct { uint64_t pmask, pval, rise, fall; uint8_t cond, csrc, cmode; } triglevel;

static void make_trigger(triglevel *a)
{
    int c;

    memset(a, 0, sizeof *a);
    for (c = 0; c < LA_NCH; c++) {
        uint64_t bit = UINT64_C(1) << c;

        switch (set.cond[c]) {
            case TC_LOW:  a->pmask |= bit; break;
            case TC_HIGH: a->pmask |= bit; a->pval |= bit; break;
            case TC_RISE: a->rise |= bit; break;
            case TC_FALL: a->fall |= bit; break;
            case TC_EDGE: a->rise |= bit; a->fall |= bit; break;
            default: break;
        }
    }
    if (a->pmask) a->cond = COND_PATTERN_TRUE;
    if (a->rise || a->fall) a->csrc = COUNTER_SRC_EDGE;
    if (a->cond || a->csrc) a->cmode = COUNTER_MODE_OCCURRENCE;
}

static int write_trigger_level(uint32_t base, const triglevel *l)
{
    if (reg_write_n(base + TRIG_PATTERN_MASK, l->pmask, 5)) return -1;
    if (reg_write_n(base + TRIG_PATTERN_VALUE, l->pval, 5)) return -1;
    if (reg_write_n(base + TRIG_RISING_MASK, l->rise, 5)) return -1;
    if (reg_write_n(base + TRIG_FALLING_MASK, l->fall, 5)) return -1;
    if (reg_write_n(base + TRIG_CONSECUTIVE, 0, 1)) return -1;
    if (reg_write_n(base + TRIG_PREQUALIFY, 0, 1)) return -1;
    if (reg_write_n(base + TRIG_COUNT_LOW, 0, 4)) return -1;
    if (reg_write_n(base + TRIG_COUNT_HIGH, 0, 4)) return -1;  /* occurrences minus one */
    if (reg_write_n(base + TRIG_CONDITION, l->cond, 1)) return -1;
    if (reg_write_n(base + TRIG_COUNTER_SOURCE, l->csrc, 1)) return -1;
    if (reg_write_n(base + TRIG_COUNTER_MODE, l->cmode, 1)) return -1;
    return 0;
}

/* write all acquisition settings, in the order the vendor software uses */
static int program(void)
{
    triglevel a, b;
    uint16_t pre, post, dac;
    uint8_t mode;
    int rle = set.rle && !set.state && la_rates[set.rate_ix].rate <= LA_RLE_MAXRATE;

    compression_mask = set.mask ? set.mask : LA_ALLCH;
    make_trigger(&a);
    memset(&b, 0, sizeof b);
    trigger_used = a.cond || a.csrc;
    if (!set.state) {
        if (reg_write_n(REG_PRESCALER, la_rates[set.rate_ix].prescaler, 1)) return -1;
        if (reg_write_n(REG_STEP, la_rates[set.rate_ix].step, 1)) return -1;
        mode = CLOCK_MODE_CLK1 | CLOCK_MODE_NO_QUAL;
    } else {
        mode = CLOCK_MODE_NO_QUAL | (set.clk_src ? CLOCK_MODE_CLK2 : CLOCK_MODE_CLK1);
        if (set.clk_fall) mode |= CLOCK_MODE_FALLING;
    }
    if (reg_write_n(REG_CLOCK_MODE, mode, 1)) return -1;
    if (reg_write_n(REG_COMPRESSION, 0, 1)) return -1;
    /* all channels are always captured; the mask steers compression:
       changes on channels outside it do not start a new RLE entry */
    if (reg_write_n(REG_COMPRESSION_MASK, compression_mask, 5)) return -1;
    if (reg_write_n(REG_INVERT_MASK, 0, 5)) return -1;
    if (reg_write_n(REG_COMPRESSION, rle ? 1 : 0, 1)) return -1;
    if (reg_write_n(REG_CHANNEL_CFG, 0, 1)) return -1;
    if (reg_write_n(REG_RUN, 0, 1)) return -1;
    if (reg_write_n(REG_ARM, 1, 1)) return -1;
    if (write_trigger_level(REG_TRIGGER_A, &a)) return -1;
    if (write_trigger_level(REG_TRIGGER_B, &b)) return -1;
    if (reg_write_n(REG_TRIGGER_COMBINE, trigger_used ? 1 : 0, 1)) return -1;
    dac = threshold_dac(set.threshold, st.cal);
    if (reg_write_n(REG_THRESHOLD, dac, 2)) return -1;
    buffer_split(&pre, &post);
    if (reg_write_n(REG_PRE_DEPTH, pre, 2)) return -1;
    if (reg_write_n(REG_POST_DEPTH, post, 2)) return -1;
    if (dac != dac_last) {
        /* the DAC output settles with a time constant of about 30 ms */
        sleep_ms(THRESHOLD_SETTLE_MS);
        dac_last = dac;
    }
    return 0;
}

static int start_acq(void)
{
    int rle = set.rle && !set.state && la_rates[set.rate_ix].rate <= LA_RLE_MAXRATE;

    prefill_ms = postfill_ms = 0;
    if (rle) {
        /* with compression memory fills with transitions, so an idle input
           never completes a capture: bound the fills in time instead */
        uint64_t win = la_windows[set.window_ix], pre = win * set.pretrig / 100, post = win - pre;
        uint64_t rate = la_rates[set.rate_ix].rate;

        prefill_ms = (long)(pre * 1000 / rate); if (prefill_ms < 1) prefill_ms = 1;
        postfill_ms = (long)(post * 1000 / rate); if (postfill_ms < 1) postfill_ms = 1;
    }
    trigger_seen = 0;
    prefill_ended = postfill_ended = 0;
    if (reg_write_u8(REG_RUN, 1)) return -1;
    if (reg_write_u8(REG_ARM, 1)) return -1;
    acq_start = now_ms();
    return 0;
}

static int halt_acq(void)
{
    return reg_write_u8(REG_RUN, 0);
}

static int check_fill_limits(uint8_t status)
{
    long t = now_ms();

    if (!prefill_ms) return 0;
    if (!(status & STATUS_PREFILLED) && !prefill_ended && t - acq_start >= prefill_ms) {
        dbg("pre-fill time elapsed");
        prefill_ended = 1;
        if (reg_write_u8(REG_END_PREFILL, 1)) return -1;
    }
    if ((status & STATUS_TRIGGERED) && !trigger_seen) trigger_seen = t;
    if (trigger_seen && !postfill_ended && t - trigger_seen >= postfill_ms) {
        dbg("post-fill time elapsed");
        postfill_ended = 1;
        if (reg_write_u8(REG_END_POSTFILL, 1)) return -1;
    }
    return 0;
}

/* trigger now: end the pre-fill, then force the trigger */
static int force_trigger(uint8_t status)
{
    if (!(status & STATUS_PREFILLED)) {
        if (reg_write_u8(REG_END_PREFILL, 1)) return -1;
        prefill_ended = 1;
    }
    if (!(status & STATUS_TRIGGERED)) {
        if (reg_write_u8(REG_FORCE_TRIGGER, 1)) return -1;
    }
    return 0;
}

/* ------------------------------------------------------------ readout */

/* the capture under construction */
static la_capture *cap;
static int cap_room;

static void cap_begin(void)
{
    cap = calloc(1, sizeof *cap);
    cap_room = 256;
    cap->runs = malloc(cap_room * sizeof(la_run));
    cap->nruns = 0;
    cap->total = 0;
    cap->trigger = -1;
}

/* add count samples of value, merging with the run before when equal */
static void cap_put(uint64_t value, uint64_t count)
{
    if (!count) return;
    if (cap->nruns && cap->runs[cap->nruns - 1].value == value) cap->runs[cap->nruns - 1].len += count;
    else {
        if (cap->nruns == cap_room) {
            cap_room *= 2;
            cap->runs = realloc(cap->runs, cap_room * sizeof(la_run));
        }
        cap->runs[cap->nruns].value = value;
        cap->runs[cap->nruns].len = count;
        cap->nruns++;
    }
    cap->total += count;
}

/* sample memory is 2048 entries of 40 bits: four lanes of D0-D31 and a
   flags lane with CLK1 and CLK2; with compression an entry with FLAG_COUNT
   is a count of extra unchanged sample periods instead */
static uint64_t data_record(int row, int clk_shift)
{
    uint64_t v = (uint64_t)raw[0][row] | (uint64_t)raw[1][row] << 8 |
                 (uint64_t)raw[2][row] << 16 | (uint64_t)raw[3][row] << 24;
    uint8_t flags = raw[4][row] >> clk_shift;

    if (flags & FLAG_CLK1) v |= UINT64_C(1) << LA_CH_CLK1;
    if (flags & FLAG_CLK2) v |= UINT64_C(1) << LA_CH_CLK2;
    return v;
}

static uint64_t count_record(int row)
{
    return (uint64_t)(raw[4][row] & 0x07) << 32 | (uint64_t)raw[3][row] << 24 |
           (uint64_t)raw[2][row] << 16 | (uint64_t)raw[1][row] << 8 | (uint64_t)raw[0][row];
}

static int read_lanes(uint32_t lanes, uint16_t start, int offset, int count)
{
    int k;

    for (k = 0; k < 4; k++)
        if (reg_read(lanes + k * MEM_LANE_STRIDE + start, &raw[k][offset], count)) return -1;
    return reg_read(MEM_FLAGS + start, &raw[4][offset], count);
}

/* memory is a ring; the trigger position counts entries after it */
static int read_sdr(uint8_t status)
{
    uint16_t wp, unfilled, trigger_pos, start;
    uint64_t prev = 0, ts, prev_ts = 0, trigger_ts;
    int n, x = 0, i, trigger_idx;
    uint8_t flags;

    if (reg_read_u16(REG_WRITE_PTR, &wp)) return -1;
    if (wp > LA_DEPTH - 1) wp = LA_DEPTH - 1;
    if (!(status & STATUS_WRAPPED)) {
        if (reg_read_u16(REG_UNFILLED, &unfilled)) return -1;
        x = unfilled + 1 < LA_DEPTH - 1 ? unfilled + 1 : LA_DEPTH - 1;
    }
    n = LA_DEPTH - x;
    start = (wp + 1 + x) % LA_DEPTH;
    /* a leading count record has no data to repeat */
    if (reg_read_u8(MEM_FLAGS + start, &flags)) return -1;
    if ((flags & FLAG_COUNT) && n > 1) { start = (start + 1) % LA_DEPTH; n--; }
    if (read_lanes(MEM_LANES_A, start, 0, n)) return -1;
    if (reg_read_u16(REG_TRIGGER_POS, &trigger_pos)) return -1;
    if (trigger_pos > LA_DEPTH - 1) trigger_pos = LA_DEPTH - 1;
    dbg("SDR readout: wp %u skip %d start %u n %d trigger %u", wp, x, start, n, trigger_pos);
    /* the FPGA writes a record TRIGGER_DELAY_SDR samples after the match
       and the trigger position points just past it */
    trigger_idx = (n + 1 - trigger_pos) % (n + 1);
    trigger_idx = trigger_idx ? (trigger_idx < n ? trigger_idx : n) - 1 : 0;
    ts = 0;
    for (i = 1; i <= trigger_idx; i++) {
        ts++;
        if (raw[4][i] & FLAG_COUNT) ts += count_record(i);
    }
    trigger_ts = ts > TRIGGER_DELAY_SDR ? ts - TRIGGER_DELAY_SDR : 0;
    cap->trigger = trigger_used ? (int64_t)trigger_ts : -1;
    for (i = 0; i < n; i++) {
        if (raw[4][i] & FLAG_COUNT) {
            ts = prev_ts + 1 + count_record(i);
            cap_put(prev, ts - prev_ts);
        } else {
            prev = data_record(i, 0);
            ts = i ? prev_ts + 1 : 0;
            cap_put(prev, 1);
        }
        prev_ts = ts;
    }
    return 0;
}

/* 500 MHz: two banks of 1024 rows hold the even and odd samples */
static int read_ddr(uint8_t status)
{
    const int bank = LA_DEPTH / 2;
    uint16_t wp_a, wp_b, unfilled, trigger_pos;
    int n, x = 0, rows, i;
    int64_t trigger_idx;

    if (reg_read_u16(REG_WRITE_PTR, &wp_a)) return -1;
    if (reg_read_u16(REG_WRITE_PTR_B, &wp_b)) return -1;
    wp_a %= bank;
    wp_b %= bank;
    if (!(status & STATUS_WRAPPED)) {
        if (reg_read_u16(REG_UNFILLED, &unfilled)) return -1;
        if (unfilled + 1 > bank) x = 2 * (unfilled + 1 - bank);
    }
    n = x < LA_DEPTH - 2 ? LA_DEPTH - 1 - x : 1;
    rows = (n + 1) / 2;
    if (read_lanes(MEM_LANES_A, (wp_a + 1 + x) % bank, 0, rows)) return -1;
    if (read_lanes(MEM_LANES_B, (wp_b + 1 + x) % bank, bank, rows)) return -1;
    if (reg_read_u16(REG_TRIGGER_POS, &trigger_pos)) return -1;
    dbg("DDR readout: wp %u/%u skip %d n %d trigger %u", wp_a, wp_b, x, n, trigger_pos);
    trigger_idx = ((int64_t)n + 1 - trigger_pos) % ((int64_t)n + 1);
    if (trigger_idx < 0) trigger_idx += n + 1;
    trigger_idx -= TRIGGER_DELAY_DDR;
    if (trigger_idx < 0) trigger_idx = 0;
    if (trigger_idx > n - 1) trigger_idx = n - 1;
    cap->trigger = trigger_used ? trigger_idx : -1;
    for (i = 0; i < n; i++) {
        if (i & 1) cap_put(data_record(bank + i / 2, FLAG_DDR_B_SHIFT), 1);
        else cap_put(data_record(i / 2, 0), 1);
    }
    return 0;
}

static int read_samples(uint8_t status)
{
    int r;

    cap_begin();
    cap->rate = set.state ? 0 : la_rates[set.rate_ix].rate;
    cap->state = set.state;
    cap->rle = set.rle && !set.state && la_rates[set.rate_ix].rate <= LA_RLE_MAXRATE;
    cap->mask = compression_mask;
    if (!set.state && la_rates[set.rate_ix].ddr) r = read_ddr(status);
    else r = read_sdr(status);
    if (r) { la_free_capture(cap); cap = NULL; return -1; }
    dbg("capture: %d runs, %llu samples, trigger %lld", cap->nruns,
        (unsigned long long)cap->total, (long long)cap->trigger);
    publish_capture(cap);
    cap = NULL;
    return 0;
}

/* ------------------------------------------------------------ open and close */

static void close_device(void)
{
    if (dev) {
        libusb_release_interface(dev, 0);
        libusb_close(dev);
        dev = NULL;
    }
    st.connected = 0;
    st.image = image = -1;
    st.acq = AS_IDLE;
}

static int open_device(void)
{
    struct libusb_config_descriptor *cfg;
    libusb_device **list, *d;
    struct libusb_device_descriptor dd;
    ssize_t n, i;
    int r = 0, k;

    if (!ctx && (r = libusb_init(&ctx)) < 0) {
        set_status("libusb: %s", libusb_strerror(r));
        return -1;
    }
    dev = NULL;
    n = libusb_get_device_list(ctx, &list);
    for (i = 0; i < n && !dev; i++) {
        if (libusb_get_device_descriptor(list[i], &dd) != 0) continue;
        if (dd.idVendor != LA_VID || dd.idProduct != LA_PID) continue;
        if ((r = libusb_open(list[i], &dev)) != 0) dev = NULL;
        else {
            unsigned char s[32];

            if (dd.iSerialNumber && libusb_get_string_descriptor_ascii(dev, dd.iSerialNumber, s, sizeof s) > 0)
                snprintf(st.serial, sizeof st.serial, "%s", (char *)s);
        }
    }
    if (n >= 0) libusb_free_device_list(list, 1);
    if (!dev) {
        if (r != 0) set_status("LogicPort found but not opened: %s (see README for the udev rule)",
                               libusb_strerror(r));
        else set_status("No LogicPort found on USB");
        return -1;
    }
    libusb_set_auto_detach_kernel_driver(dev, 1);
    d = libusb_get_device(dev);
    ep_in = ep_out = 0;
    if (libusb_get_active_config_descriptor(d, &cfg) == 0) {
        if (cfg->bNumInterfaces > 0 && cfg->interface[0].num_altsetting > 0) {
            const struct libusb_interface_descriptor *ifd = &cfg->interface[0].altsetting[0];

            for (k = 0; k < ifd->bNumEndpoints; k++) {
                const struct libusb_endpoint_descriptor *e = &ifd->endpoint[k];

                if ((e->bmAttributes & 3) != LIBUSB_TRANSFER_TYPE_BULK) continue;
                if (e->bEndpointAddress & LIBUSB_ENDPOINT_IN) ep_in = e->bEndpointAddress;
                else ep_out = e->bEndpointAddress;
            }
        }
        libusb_free_config_descriptor(cfg);
    }
    if (!ep_in || !ep_out) {
        set_status("LogicPort has no bulk endpoints");
        libusb_close(dev); dev = NULL;
        return -1;
    }
    if ((r = libusb_claim_interface(dev, 0)) < 0) {
        set_status("Cannot claim the LogicPort: %s", libusb_strerror(r));
        libusb_close(dev); dev = NULL;
        return -1;
    }
    if (ftdi_setup() < 0) {
        set_status("FTDI setup failed");
        close_device();
        return -1;
    }
    read_calibration();
    image = -1;
    dac_last = -1;
    /* an unconfigured FPGA does not drain the FIFO, so a failed resync here
       is not an error: the image is loaded when the first acquisition starts */
    if (resync(0, 1) == 0) probe_image();
    st.image = image;
    st.connected = 1;
    st.acq = AS_IDLE;
    set_status("LogicPort %s on USB%s", st.serial, image >= 0 ? "" : " (FPGA not configured)");
    return 0;
}

/* a bus error: try to get back in step, else drop the link */
static void link_fail(const char *what)
{
    st.errors++;
    dbg("link failure: %s", what);
    if (dev && resync(1, 2) == 0 && probe_image() == 0) {
        set_status("Recovered after %s", what);
        return;
    }
    set_status("Lost the LogicPort (%s): reconnecting", what);
    close_device();
    next_retry = now_ms() + RETRY_MS;
}

/* ------------------------------------------------------------ simulator */

/* A made up board: a UART, an SPI bus, an I2C bus, a counter and a few
   clocks, generated at the chosen sample rate. */
static uint32_t sim_rng = 12345;
static uint32_t sim_rand(void) { sim_rng = sim_rng * 1103515245u + 12345u; return sim_rng >> 8; }

static void sim_capture(void)
{
    uint64_t rate = set.state ? 20000000ULL : la_rates[set.rate_ix].rate;
    uint64_t total = set.rle ? la_windows[set.window_ix] : LA_DEPTH, i;
    uint64_t v, prev = ~UINT64_C(0);
    double   t, dt = 1.0 / (double)rate;
    double   bit = 1.0 / 115200;
    const char *msg = "Hello, LogicPort! ";
    int      c, nmsg = (int)strlen(msg);
    triglevel a;
    int64_t  trig = -1;
    uint64_t pre = total * set.pretrig / 100;

    if (total > 4194304) total = 4194304;  /* keep the generator quick */
    make_trigger(&a);
    cap_begin();
    cap->rate = set.state ? 0 : rate;
    cap->state = set.state;
    cap->rle = set.rle;
    cap->mask = set.mask ? set.mask : LA_ALLCH;
    cap->sim = 1;
    for (i = 0; i < total; i++) {
        t = (double)(int64_t)(i - pre) * dt + 0.0001 * (sim_rand() % 7); /* a little jitter per capture */
        t = (double)(int64_t)(i - pre) * dt;
        v = 0;
        /* D0: UART 115200 8N1, the message repeating with idle gaps */
        {
            double  frame = 10 * bit, gap = 20 * bit, period = nmsg * frame + gap;
            double  tt = fmod(t + 1000 * period, period);
            int     ci = (int)(tt / frame), bi, lvl = 1;

            if (ci < nmsg) {
                bi = (int)((tt - ci * frame) / bit);
                if (bi == 0) lvl = 0;
                else if (bi <= 8) lvl = (msg[ci] >> (bi - 1)) & 1;
                else lvl = 1;
            }
            if (lvl) v |= 1;
        }
        /* D1 SPI CLK 1 MHz, D2 MOSI, D3 CS low during 8 bit bursts every 20 us */
        {
            double tt = fmod(t + 1000.0, 20e-6);
            int    cs = tt < 8e-6, k = (int)(tt / 1e-6);
            int    clk = cs && fmod(tt, 1e-6) >= 0.5e-6;
            int    mosi = cs ? ((0xA5 ^ (int)(t * 1000)) >> (7 - k)) & 1 : 0;

            if (clk) v |= 2;
            if (mosi) v |= 4;
            if (!cs) v |= 8;
        }
        /* D4 I2C SCL 100 kHz and D5 SDA: a start, an address byte, an ack */
        {
            double tt = fmod(t + 1000.0, 200e-6);
            int    scl = 1, sda = 1;

            if (tt < 95e-6) {
                int    k = (int)(tt / 10e-6);
                double ph = fmod(tt, 10e-6);

                if (k == 0) { scl = 1; sda = ph < 5e-6; }          /* start */
                else {
                    scl = ph >= 2.5e-6 && ph < 7.5e-6;
                    sda = k <= 8 ? ((0x50 << 1) >> (8 - k)) & 1 : 0; /* address 0x50 W, ack */
                }
            }
            if (scl) v |= 16;
            if (sda) v |= 32;
        }
        /* D6-D13: a byte counter at 2 MHz */
        v |= (uint64_t)(((uint64_t)(t * 2e6 + 1e9) & 0xff)) << 6;
        /* D14-D17: square waves 10 kHz, 50 kHz, 250 kHz, 1.25 MHz */
        if (fmod(t + 1000.0, 100e-6) < 50e-6) v |= UINT64_C(1) << 14;
        if (fmod(t + 1000.0, 20e-6) < 10e-6) v |= UINT64_C(1) << 15;
        if (fmod(t + 1000.0, 4e-6) < 2e-6) v |= UINT64_C(1) << 16;
        if (fmod(t + 1000.0, 0.8e-6) < 0.4e-6) v |= UINT64_C(1) << 17;
        /* D18-D31: slow noise, D31 a 1 Hz heartbeat; CLK1 a 25 MHz clock */
        for (c = 18; c < 31; c++) if (fmod(t * (c - 16) + 1000.0, 1.0) < 0.3 + 0.05 * c) v |= UINT64_C(1) << c;
        if (fmod(t + 1000.0, 1.0) < 0.5) v |= UINT64_C(1) << 31;
        if (fmod(t + 1000.0, 40e-9) < 20e-9) v |= UINT64_C(1) << LA_CH_CLK1;
        if (set.state && (i & 1)) v |= UINT64_C(1) << LA_CH_CLK2;
        /* the trigger: first match after the pre-trigger fill */
        if (trig < 0 && i >= pre && (a.cond || a.csrc)) {
            int ok = 1;

            if (a.pmask && (v & a.pmask) != a.pval) ok = 0;
            if (a.csrc && prev != ~UINT64_C(0)) {
                uint64_t up = v & ~prev, down = prev & ~v;

                if (!((up & a.rise) || (down & a.fall))) ok = 0;
            } else if (a.csrc) ok = 0;
            if (ok) trig = (int64_t)i;
        }
        cap_put(v & cap->mask, 1);
        prev = v;
    }
    if (a.cond || a.csrc) cap->trigger = trig >= 0 ? trig : (int64_t)pre;
    else cap->trigger = -1;
    publish_capture(cap);
    cap = NULL;
}

/* ------------------------------------------------------------ the thread */

static void take_requests(int *mode, int *force)
{
    ami_lock(lk);
    if (pending_new) { set = pending; pending_new = 0; *mode = -2; }  /* -2: settings changed */
    if (req_mode >= 0) { *mode = req_mode; req_mode = -1; }
    *force = req_force;
    req_force = 0;
    ami_unlock(lk);
}

static void worker(void)
{
    int mode, force, armed_mode = LA_STOP;
    uint8_t status;
    long t, sim_at = 0, last_pub = 0;

    while (!stop_req) {
        mode = -1;
        take_requests(&mode, &force);
        if (mode >= 0) st.run_mode = mode;
        if (st.sim) {
            if (mode >= 0 && mode != LA_STOP) { st.acq = AS_PREFILL; sim_at = now_ms() + 300; publish(); }
            if (mode == LA_STOP) { st.acq = AS_IDLE; publish(); }
            if (force && st.acq != AS_IDLE) sim_at = now_ms();
            if (st.acq != AS_IDLE && now_ms() >= sim_at) {
                st.acq = AS_READING;
                sim_capture();
                st.acq = st.run_mode == LA_REPEAT ? AS_PREFILL : AS_IDLE;
                if (st.run_mode == LA_SINGLE) st.run_mode = LA_STOP;
                sim_at = now_ms() + 500;
                publish();
            }
            sleep_ms(50);
            continue;
        }
        t = now_ms();
        if (!dev) {
            if (t >= next_retry) {
                if (open_device() < 0) next_retry = now_ms() + RETRY_MS;
                publish();
            }
            sleep_ms(100);
            continue;
        }
        if (mode == LA_STOP) {
            if (st.acq != AS_IDLE) {
                if (halt_acq()) { link_fail("halt"); publish(); continue; }
                st.acq = AS_IDLE;
            }
            publish();
        }
        /* arm: a run request, or new settings while running */
        if (st.run_mode != LA_STOP && (st.acq == AS_IDLE || mode == -2 || (mode >= 0 && mode != armed_mode))) {
            if (st.acq != AS_IDLE && halt_acq()) { link_fail("halt"); publish(); continue; }
            if (configure_fpga(select_image()) != 0) {
                st.run_mode = LA_STOP;
                st.acq = AS_IDLE;
                if (io_error) link_fail("configure");
                publish();
                continue;
            }
            if (program() != 0 || start_acq() != 0) {
                /* the link may be out of step: resync and try once more */
                if (resync(1, 1) != 0 || probe_image() != 0 || configure_fpga(select_image()) != 0 ||
                    program() != 0 || start_acq() != 0) {
                    st.run_mode = LA_STOP;
                    st.acq = AS_IDLE;
                    link_fail("arm");
                    publish();
                    continue;
                }
            }
            armed_mode = st.run_mode;
            st.acq = AS_PREFILL;
            set_status("LogicPort %s: armed", st.serial);
            publish();
        }
        if (st.acq == AS_IDLE) { sleep_ms(50); continue; }
        /* armed: watch the status */
        if (reg_read_u8(REG_STATUS, &status)) { link_fail("status"); publish(); continue; }
        if (force && force_trigger(status)) { link_fail("force"); publish(); continue; }
        if (!(status & STATUS_DONE)) {
            int was = st.acq;

            if (check_fill_limits(status)) { link_fail("fill limit"); publish(); continue; }
            st.acq = (status & STATUS_TRIGGERED) ? AS_POSTFILL : (status & STATUS_PREFILLED) ? AS_ARMED : AS_PREFILL;
            st.acq_ms = now_ms() - acq_start;
            if (st.acq != was || now_ms() - last_pub > 250) { publish(); last_pub = now_ms(); }
            sleep_ms(POLL_MS);
            continue;
        }
        st.acq = AS_READING;
        publish();
        if (read_samples(status)) { link_fail("readout"); publish(); continue; }
        if (st.run_mode == LA_SINGLE) { st.run_mode = LA_STOP; st.acq = AS_IDLE; }
        else {
            if (start_acq()) { link_fail("re-arm"); publish(); continue; }
            st.acq = AS_PREFILL;
        }
        publish();
    }
    if (dev && st.acq != AS_IDLE) halt_acq();
    close_device();
    thread_done = 1;
}

/* ------------------------------------------------------------ interface */

void la_init(la_state *s, const la_settings *setp, int sim)
{
    memset(&st, 0, sizeof st);
    trace = getenv("LOGICPORT_TRACE") != NULL;
    st.sim = sim;
    st.image = -1;
    st.run_mode = LA_STOP;
    set = *setp;
    pending = *setp;
    if (find_ccf() < 0 && !sim) set_status("No LogicPort.ccf found: see README");
    else if (sim) set_status("Simulated analyzer");
    else set_status("Looking for the LogicPort on USB");
    lk = ami_initlock();
    shared = st;
    shared_changed = 1;
    *s = st;
    ami_newthread(worker);
}

int la_poll(la_state *s)
{
    int changed;

    ami_lock(lk);
    changed = shared_changed;
    if (changed) {
        *s = shared;
        shared.newcap = NULL;   /* handed over */
        shared_changed = 0;
    } else s->newcap = NULL;
    ami_unlock(lk);
    return changed;
}

void la_shutdown(la_state *s)
{
    int i;

    stop_req = 1;
    for (i = 0; i < 60 && !thread_done; i++) sleep_ms(50);
    s->connected = 0;
}

void la_apply(la_state *s, const la_settings *setp)
{
    (void)s;
    ami_lock(lk);
    pending = *setp;
    pending_new = 1;
    ami_unlock(lk);
}

void la_start(la_state *s, int mode)
{
    (void)s;
    ami_lock(lk);
    req_mode = mode;
    ami_unlock(lk);
}

void la_force(la_state *s)
{
    (void)s;
    ami_lock(lk);
    req_force = 1;
    ami_unlock(lk);
}

void la_free_capture(la_capture *c)
{
    if (!c) return;
    free(c->runs);
    free(c);
}

/* ------------------------------------------------------------ export */

/* one line per run: time of the change, how long held, then the channels */
long la_save_csv(const la_capture *c, const char *names[LA_NCH], const char *path)
{
    FILE *f = fopen(path, "w");
    double dt = c->rate ? 1.0 / (double)c->rate : 1.0;
    int64_t pos = 0;
    int i, k;
    long n;

    if (!f) return -1;
    fprintf(f, "%s,%s", c->rate ? "time_s" : "sample", c->rate ? "held_s" : "held");
    for (k = 0; k < LA_NCH; k++) fprintf(f, ",%s", names[k]);
    fprintf(f, "\n");
    for (i = 0; i < c->nruns; i++) {
        const la_run *r = &c->runs[i];
        int64_t rel = pos - (c->trigger >= 0 ? c->trigger : 0);

        if (c->rate) fprintf(f, "%.12g,%.12g", rel * dt, (double)r->len * dt);
        else fprintf(f, "%lld,%llu", (long long)rel, (unsigned long long)r->len);
        for (k = 0; k < LA_NCH; k++) fprintf(f, ",%d", (int)((r->value >> k) & 1));
        fprintf(f, "\n");
        pos += (int64_t)r->len;
    }
    n = ftell(f);
    fclose(f);
    return n;
}

/* value change dump, timescale from the sample rate */
long la_save_vcd(const la_capture *c, const char *names[LA_NCH], const char *path)
{
    FILE *f = fopen(path, "w");
    uint64_t pos = 0, prev = ~UINT64_C(0);
    const char *ts;
    uint64_t mult;   /* timescale units per sample */
    int i, k;
    long n;

    if (!f) return -1;
    if (!c->rate) { ts = "1 ns"; mult = 1; }
    else if (c->rate >= 500000000ULL) { ts = "1 ps"; mult = 1000000000000ULL / c->rate; }
    else if (c->rate >= 1000000ULL) { ts = "1 ns"; mult = 1000000000ULL / c->rate; }
    else if (c->rate >= 1000ULL) { ts = "1 us"; mult = 1000000ULL / c->rate; }
    else { ts = "1 ms"; mult = 1000ULL / c->rate; }
    fprintf(f, "$timescale %s $end\n$scope module logicport $end\n", ts);
    for (k = 0; k < LA_NCH; k++) fprintf(f, "$var wire 1 %c%c %s $end\n", '!' + k / 32, '!' + k % 32 + 1, names[k]);
    fprintf(f, "$upscope $end\n$enddefinitions $end\n");
    for (i = 0; i < c->nruns; i++) {
        const la_run *r = &c->runs[i];

        fprintf(f, "#%llu\n", (unsigned long long)(pos * mult));
        for (k = 0; k < LA_NCH; k++) {
            int b = (int)((r->value >> k) & 1);

            if (prev == ~UINT64_C(0) || b != (int)((prev >> k) & 1))
                fprintf(f, "%d%c%c\n", b, '!' + k / 32, '!' + k % 32 + 1);
        }
        prev = r->value;
        pos += r->len;
    }
    fprintf(f, "#%llu\n", (unsigned long long)(pos * mult));
    n = ftell(f);
    fclose(f);
    return n;
}
