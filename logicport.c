/*
 * logicport CLI: a LogicPort LA1034 logic analyzer on the desktop,
 * written with the AMI tool kit.
 *
 * The analyzer has no screen of its own, so unlike the scope and supply
 * panels this window is the instrument: a control bar across the top, the
 * 34 channels down the left with their trigger conditions, the waveforms
 * across the middle with a time axis, trigger mark and two cursors, and a
 * readout strip along the bottom. Everything is drawn with the AMI graphics
 * calls and scaled to the window, so it can be any size.
 *
 * Use:
 *   logicport            find the analyzer on USB
 *   logicport --sim      no hardware: a simulated board
 *   logicport --version  the version and the notice below
 *
 * Not affiliated with or endorsed by Intronix Test Instruments. LogicPort
 * is a product name of Intronix; this is an independent program for that
 * hardware, with nothing of theirs in it.
 *
 * Mouse: click the buttons; the wheel over a value steps it. In the
 * waveforms the wheel zooms about the pointer, a drag pans, a click sets
 * cursor A and a right click cursor B. Click a channel's trigger box to
 * cycle its condition (X 0 1 rise fall edge), its square to take it out of
 * the display and the compression, and its name to rename it (type, Enter
 * or Esc).
 *
 * Keys: r run (repeat), s single, space stop, t force the trigger, f fit
 * the capture to the window, +/- zoom, Left/Right pan, a/b put the cursors
 * at the pointer, x save CSV, v save VCD, Up/Down sample rate, PgUp/PgDn
 * threshold.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <ctype.h>
#include <math.h>
#include <localdefs.h>
#include <services.h>
#include <graphics.h>

#include "la.h"

/* the panel is laid out in these units and scaled to the window */
#define PANW 1400
#define PANH 900

#define TICK_ID 1
#define TICK    500   /* 50 ms, in 100 us units */

/* regions */
#define TOPH   66            /* control bar */
#define AXY1   (TOPH + 4)    /* time axis */
#define AXY2   (TOPH + 26)
#define LBLW   184           /* channel label column */
#define WX1    (LBLW + 2)    /* waveform area */
#define WX2    (PANW - 8)
#define WY1    (AXY2 + 2)
#define BOTH   36            /* readout strip */
#define WY2    (PANH - BOTH - 4)
#define WW     (WX2 - WX1)
#define WH     (WY2 - WY1)

#define WINW 1400
#define WINH 900

/* things that can be clicked */
enum {
    H_NONE,
    H_RUN, H_SINGLE, H_STOP, H_FORCE,
    H_RATE_DN, H_RATE_UP, H_RATE, H_RLE, H_WIN_DN, H_WIN_UP, H_WIN,
    H_THR_DN, H_THR_UP, H_THR, H_PRE_DN, H_PRE_UP, H_PRE,
    H_MODE, H_CLKSRC, H_CLKEDGE,
    H_FIT, H_ZIN, H_ZOUT, H_CSV, H_VCD, H_CLEAR,
    H_WAVE, H_AXIS, H_ALL,
    H_NAME0 = 100, H_COND0 = 200, H_EN0 = 300,
    H_LAST = 400
};

typedef struct { int x1, y1, x2, y2, id; } hitbox;

#ifdef SHOT
/* test build: LOGICPORT_SHOT=<file> takes a picture of the window a few
   seconds in and quits; LOGICPORT_SCRIPT=SINGLE,FIT,ZIN,... runs clicks 1 s in */
void screen_capture(void);
void screen_capture_name(const char *fn);
static const char *shot_file;
static long shot_at;
static const char *script, *script2;  /* LOGICPORT_SCRIPT2 runs at 2.5 s */
static long script_at, script2_at;
static void run_script(const char *script);
#endif

static la_state    la;
static la_settings set;
static la_capture *cap;            /* the capture on show */
static char        names[LA_NCH][16];
static const char *namep[LA_NCH];

static hitbox hits[220];
static int    nhits;
static int    pressed = H_NONE;
static int    mx, my;

/* the view: positions in samples relative to time zero (the trigger) */
static double view_s0, view_span;  /* left edge and width, in samples */
static int    view_fit = 1;        /* follow each capture with a fit */
static double cur_a = -1e300, cur_b = -1e300; /* cursors, samples from zero; -1e300 none */
static int    dragging, drag_x, drag_moved;
static double drag_s0;

static double vscale = 0.1;        /* the view scale of this frame, for line widths */
static int    editing = -1;        /* channel whose name is being typed */
static char   edit[16];

static char   note[128];
static long   note_until;
static int    trace;

static const struct { int r, g, b; } grpcol[5] = {
    { 250, 220, 60 }, { 80, 210, 240 }, { 120, 230, 110 }, { 240, 120, 220 }, { 250, 160, 60 }
};

static const char *condname[TC_COUNT] = { "X", "0", "1", "/", "\\", "|" };
static const char *condlong[TC_COUNT] = { "don't care", "low", "high", "rising", "falling", "either edge" };

/* milliseconds since the first call, from ami_clock which wraps */
static long now_ms(void)
{
    static ami_long ref;
    static long     acc;
    static int      started;
    ami_long        ms;

    if (!started) { ref = ami_clock(); started = 1; }
    ms = ami_elapsed(ref) / 10;
    if (ms > 0) { ref += ms * 10; acc += ms; }
    return acc;
}

static void user_file(char *fn, int fnl, const char *sub, const char *name, const char *ext)
{
    char usr[256], dir[512];

    ami_getusr(usr, sizeof usr - 1);
    usr[sizeof usr - 1] = 0;
    if (sub) {
        ami_maknam(dir, sizeof dir - 1, usr, (char *)sub, "");
        dir[sizeof dir - 1] = 0;
    } else strcpy(dir, usr);
    ami_maknam(fn, fnl - 1, dir, (char *)name, (char *)ext);
    fn[fnl - 1] = 0;
}

static void local_time(ami_long t, int *y, int *mo, int *d, int *h, int *mi, int *sec)
{
    long days = (long)(t / 86400), rem = (long)(t % 86400), era, doe, yoe, doy, mp;

    if (rem < 0) { rem += 86400; days--; }
    *h = (int)(rem / 3600); *mi = (int)(rem / 60 % 60); *sec = (int)(rem % 60);
    days += 10957 + 719468;
    era = (days >= 0 ? days : days - 146096) / 146097;
    doe = days - era * 146097;
    yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    mp = (5 * doy + 2) / 153;
    *d = (int)(doy - (153 * mp + 2) / 5 + 1);
    *mo = (int)(mp < 10 ? mp + 3 : mp - 9);
    *y = (int)(yoe + era * 400 + (*mo <= 2));
}

static void set_note(const char *s)
{
    snprintf(note, sizeof note, "%s", s);
    note_until = now_ms() + 4000;
}

/* a value with an engineering prefix */
static void fmt_eng(double v, const char *unit, char *buf, size_t len)
{
    static const char *pfx[] = { "p", "n", "u", "m", "", "k", "M", "G" };
    double a = fabs(v);
    int    e = 4;

    if (a == 0) { snprintf(buf, len, "0 %s", unit); return; }
    while (a >= 1000 && e < 7) { a /= 1000; e++; }
    while (a < 1 && e > 0) { a *= 1000; e--; }
    if (a >= 100) snprintf(buf, len, "%s%.0f %s%s", v < 0 ? "-" : "", a, pfx[e], unit);
    else if (a >= 10) snprintf(buf, len, "%s%.1f %s%s", v < 0 ? "-" : "", a, pfx[e], unit);
    else snprintf(buf, len, "%s%.2f %s%s", v < 0 ? "-" : "", a, pfx[e], unit);
}

/* the capture's sample rate, or 0 in state mode */
static uint64_t cap_rate(void) { return cap ? cap->rate : (set.state ? 0 : la_rates[set.rate_ix].rate); }

/* a span or position in samples as time (or samples in state mode) */
static void fmt_time(double samples, char *buf, size_t len)
{
    uint64_t rate = cap_rate();

    if (!rate) {
        if (fabs(samples) >= 1e6) snprintf(buf, len, "%.3g smp", samples);
        else snprintf(buf, len, "%.0f smp", samples);
    } else fmt_eng(samples / (double)rate, "s", buf, len);
}

static int64_t cap_zero(void) { return cap && cap->trigger >= 0 ? cap->trigger : 0; }

/* ---------------------------------------------------------------- settings */

static void defaults(void)
{
    int c;

    memset(&set, 0, sizeof set);
    set.rate_ix = 15;         /* 100 MHz */
    set.rle = 0;
    set.window_ix = 3;        /* 1M samples */
    set.threshold = 1.5;
    set.pretrig = 20;
    set.mask = LA_ALLCH;
    for (c = 0; c < 32; c++) snprintf(names[c], sizeof names[c], "D%d", c);
    strcpy(names[LA_CH_CLK1], "CLK1");
    strcpy(names[LA_CH_CLK2], "CLK2");
    for (c = 0; c < LA_NCH; c++) namep[c] = names[c];
}

static void rc_path(char *fn, int fnl) { user_file(fn, fnl, NULL, ".logicportrc", ""); }

static void load_rc(void)
{
    char fn[600], line[256], *p, *q;
    FILE *f;
    int c;

    rc_path(fn, sizeof fn);
    f = fopen(fn, "r");
    if (!f) return;
    while (fgets(line, sizeof line, f)) {
        p = strchr(line, '=');
        if (!p) continue;
        *p++ = 0;
        q = p + strlen(p);
        while (q > p && (q[-1] == '\n' || q[-1] == '\r')) *--q = 0;
        if (!strcmp(line, "rate")) { set.rate_ix = atoi(p); if (set.rate_ix < 0 || set.rate_ix >= LA_NRATES) set.rate_ix = 15; }
        else if (!strcmp(line, "rle")) set.rle = atoi(p) != 0;
        else if (!strcmp(line, "window")) { set.window_ix = atoi(p); if (set.window_ix < 0 || set.window_ix >= LA_NWINDOWS) set.window_ix = 3; }
        else if (!strcmp(line, "threshold")) { set.threshold = atof(p); if (set.threshold < LA_THRESH_MIN || set.threshold > LA_THRESH_MAX) set.threshold = 1.5; }
        else if (!strcmp(line, "pretrig")) { set.pretrig = atoi(p); if (set.pretrig < 0 || set.pretrig > 100) set.pretrig = 20; }
        else if (!strcmp(line, "state")) set.state = atoi(p) != 0;
        else if (!strcmp(line, "clksrc")) set.clk_src = atoi(p) != 0;
        else if (!strcmp(line, "clkfall")) set.clk_fall = atoi(p) != 0;
        else if (!strcmp(line, "mask")) { set.mask = strtoull(p, NULL, 16) & LA_ALLCH; if (!set.mask) set.mask = LA_ALLCH; }
        else if (!strcmp(line, "cond")) for (c = 0; c < LA_NCH && p[c]; c++) set.cond[c] = p[c] >= '0' && p[c] < '0' + TC_COUNT ? p[c] - '0' : 0;
        else if (!strncmp(line, "name", 4)) {
            c = atoi(line + 4);
            if (c >= 0 && c < LA_NCH && *p) snprintf(names[c], sizeof names[c], "%s", p);
        }
    }
    fclose(f);
}

static void save_rc(void)
{
    char fn[600];
    FILE *f;
    int c;

    rc_path(fn, sizeof fn);
    f = fopen(fn, "w");
    if (!f) return;
    fprintf(f, "rate=%d\nrle=%d\nwindow=%d\nthreshold=%.2f\npretrig=%d\nstate=%d\nclksrc=%d\nclkfall=%d\n",
            set.rate_ix, set.rle, set.window_ix, set.threshold, set.pretrig, set.state, set.clk_src, set.clk_fall);
    fprintf(f, "mask=%llx\ncond=", (unsigned long long)set.mask);
    for (c = 0; c < LA_NCH; c++) fputc('0' + set.cond[c], f);
    fprintf(f, "\n");
    for (c = 0; c < LA_NCH; c++) fprintf(f, "name%d=%s\n", c, names[c]);
    fclose(f);
}

static void apply(void)
{
    la_apply(&la, &set);
}

/* ---------------------------------------------------------------- the view */

static int nvisible(void)
{
    int c, n = 0;

    for (c = 0; c < LA_NCH; c++) if (set.mask & (UINT64_C(1) << c)) n++;
    return n ? n : LA_NCH;
}

static void fit(void)
{
    if (!cap || !cap->total) { view_s0 = -1024; view_span = 2048; return; }
    view_s0 = -(double)cap_zero();
    view_span = (double)cap->total;
    view_fit = 1;
}

static double sample_at(int px)  /* panel x to a position in samples */
{
    return view_s0 + ((double)px / 10 - WX1) / WW * view_span;
}

static void zoom(double factor, double about)
{
    double span = view_span / factor, lo, hi;

    if (span < 4) span = 4;
    if (cap && span > (double)cap->total * 4) span = (double)cap->total * 4;
    view_s0 = about - (about - view_s0) * span / view_span;
    view_span = span;
    /* keep some of the capture in view */
    if (cap) {
        lo = -(double)cap_zero() - view_span * 0.9;
        hi = (double)cap->total - cap_zero() - view_span * 0.1;
        if (view_s0 < lo) view_s0 = lo;
        if (view_s0 > hi) view_s0 = hi;
    }
    view_fit = 0;
}

static void pan(double samples)
{
    view_s0 += samples;
    view_fit = 0;
}

/* ---------------------------------------------------------------- actions */

static void save_capture(int vcd)
{
    char     path[600], name[64], msg[760];
    ami_long wall = ami_local(ami_time());
    int      y, mo, d, h, mi, sec;
    long     n;

    if (!cap) { set_note("No capture to save yet"); return; }
    local_time(wall, &y, &mo, &d, &h, &mi, &sec);
    snprintf(name, sizeof name, "LogicPort_%04d%02d%02d_%02d%02d%02d", y, mo, d, h, mi, sec);
    user_file(path, sizeof path, "Desktop", name, vcd ? "vcd" : "csv");
    n = vcd ? la_save_vcd(cap, namep, path) : la_save_csv(cap, namep, path);
    if (n > 0) snprintf(msg, sizeof msg, "Saved Desktop/%s.%s", name, vcd ? "vcd" : "csv");
    else snprintf(msg, sizeof msg, "Could not write %s", path);
    set_note(msg);
}

static void step_rate(int dir)
{
    int ix = set.rate_ix + dir;

    if (ix < 0 || ix >= LA_NRATES) return;
    set.rate_ix = ix;
    if (set.rle && la_rates[ix].rate > LA_RLE_MAXRATE) set_note("Compression is off above 200 MHz");
    apply();
}

static void step_threshold(int dir)
{
    double v = set.threshold + dir * LA_THRESH_STEP;

    if (v < LA_THRESH_MIN - 1e-9 || v > LA_THRESH_MAX + 1e-9) return;
    set.threshold = round(v * 20) / 20;
    apply();
}

static void step_pretrig(int dir)
{
    int v = set.pretrig + dir * 5;

    if (v < 0 || v > 100) return;
    set.pretrig = v;
    apply();
}

static void step_window(int dir)
{
    int ix = set.window_ix + dir;

    if (ix < 0 || ix >= LA_NWINDOWS) return;
    set.window_ix = ix;
    apply();
}

static void click(int id)
{
    char msg[128];

    if (id >= H_NAME0 && id < H_NAME0 + LA_NCH) {
        editing = id - H_NAME0;
        snprintf(edit, sizeof edit, "%s", names[editing]);
        return;
    }
    if (id >= H_COND0 && id < H_COND0 + LA_NCH) {
        int c = id - H_COND0;

        set.cond[c] = (set.cond[c] + 1) % TC_COUNT;
        snprintf(msg, sizeof msg, "%s trigger: %s", names[c], condlong[set.cond[c]]);
        set_note(msg);
        apply();
        return;
    }
    if (id >= H_EN0 && id < H_EN0 + LA_NCH) {
        set.mask ^= UINT64_C(1) << (id - H_EN0);
        if (!set.mask) set.mask = UINT64_C(1) << (id - H_EN0);
        apply();
        return;
    }
    switch (id) {
        case H_RUN:     la_start(&la, LA_REPEAT); break;
        case H_SINGLE:  la_start(&la, LA_SINGLE); break;
        case H_STOP:    la_start(&la, LA_STOP); break;
        case H_FORCE:   la_force(&la); break;
        case H_RATE_DN: step_rate(-1); break;
        case H_RATE_UP: step_rate(1); break;
        case H_RLE:
            set.rle = !set.rle;
            if (set.rle && !set.state && la_rates[set.rate_ix].rate > LA_RLE_MAXRATE) set_note("Compression works up to 200 MHz");
            apply();
            break;
        case H_WIN_DN:  step_window(-1); break;
        case H_WIN_UP:  step_window(1); break;
        case H_THR_DN:  step_threshold(-1); break;
        case H_THR_UP:  step_threshold(1); break;
        case H_THR: {
            /* presets: TTL, 3.3 V CMOS, 2.5 V, 1.8 V, 5 V CMOS */
            static const double pre[] = { 1.5, 1.65, 1.25, 0.9, 2.5 };
            int i, n = sizeof pre / sizeof pre[0];

            for (i = 0; i < n; i++) if (fabs(set.threshold - pre[i]) < 1e-6) break;
            set.threshold = pre[(i + 1) % n];
            apply();
            break;
        }
        case H_PRE_DN:  step_pretrig(-1); break;
        case H_PRE_UP:  step_pretrig(1); break;
        case H_MODE:    set.state = !set.state; apply(); break;
        case H_CLKSRC:  set.clk_src = !set.clk_src; apply(); break;
        case H_CLKEDGE: set.clk_fall = !set.clk_fall; apply(); break;
        case H_FIT:     fit(); break;
        case H_ZIN:     zoom(2, view_s0 + view_span / 2); break;
        case H_ZOUT:    zoom(0.5, view_s0 + view_span / 2); break;
        case H_CSV:     save_capture(0); break;
        case H_VCD:     save_capture(1); break;
        case H_CLEAR:   cur_a = cur_b = -1e300; break;
        case H_ALL:
            set.mask = set.mask == LA_ALLCH ? 0xff : LA_ALLCH;
            apply();
            break;
        default: break;
    }
}

/* the wheel over a value field steps it */
static int wheel(int id, int dir)
{
    switch (id) {
        case H_RATE: case H_RATE_DN: case H_RATE_UP: step_rate(dir); return 1;
        case H_THR: case H_THR_DN: case H_THR_UP: step_threshold(dir); return 1;
        case H_PRE: case H_PRE_DN: case H_PRE_UP: step_pretrig(dir); return 1;
        case H_WIN: case H_WIN_DN: case H_WIN_UP: step_window(dir); return 1;
        default: return 0;
    }
}

/* ---------------------------------------------------------------- drawing */

#define GRID 10

static int X(double x) { return (int)lround(x * GRID); }
static int Y(double y) { return (int)lround(y * GRID); }
static int S(double d) { int v = (int)lround(d * GRID); return v < 1 ? 1 : v; }

static ami_long cv(int v) { return (ami_long)v * (LONG_MAX / 255); }
static void color(int r, int g, int b) { ami_fcolorg(stdout, cv(r), cv(g), cv(b)); }

static void addhit(double x1, double y1, double x2, double y2, int id)
{
    if (nhits >= (int)(sizeof hits / sizeof hits[0])) return;
    hits[nhits].x1 = X(x1); hits[nhits].y1 = Y(y1);
    hits[nhits].x2 = X(x2); hits[nhits].y2 = Y(y2);
    hits[nhits].id = id;
    nhits++;
}

static int hit(int x, int y)
{
    int i;

    for (i = nhits - 1; i >= 0; i--)
        if (x >= hits[i].x1 && x <= hits[i].x2 && y >= hits[i].y1 && y <= hits[i].y2)
            return hits[i].id;
    return H_NONE;
}

static void frect(double x1, double y1, double x2, double y2) { ami_frect(stdout, X(x1), Y(y1), X(x2), Y(y2)); }
static void frrect(double x1, double y1, double x2, double y2, double r) { ami_frrect(stdout, X(x1), Y(y1), X(x2), Y(y2), S(r), S(r)); }
/* line widths are in device pixels, not view units: w is panel units */
static void lwidth(double w)
{
    long px = lround(w * vscale * GRID);

    ami_linewidth(stdout, px < 1 ? 1 : px);
}
static void line(double x1, double y1, double x2, double y2, double w)
{
    lwidth(w);
    ami_line(stdout, X(x1), Y(y1), X(x2), Y(y2));
}
static void ftri(double x1, double y1, double x2, double y2, double x3, double y3)
{
    ami_ftriangle(stdout, X(x1), Y(y1), X(x2), Y(y2), X(x3), Y(y3));
}

static void font(int face, double size, int bold)
{
    ami_font(stdout, face);
    ami_fontsiz(stdout, S(size));
    ami_bold(stdout, bold);
}

static double tw(const char *s) { return (double)ami_strsiz(stdout, s) / GRID; }

/* text with its top left at x,y; align -1 left, 0 centre, 1 right of x */
static void text(double x, double y, int align, const char *s)
{
    double w = tw(s);

    if (align == 0) x -= w / 2;
    else if (align > 0) x -= w;
    ami_cursorg(stdout, X(x), Y(y));
    ami_binvis(stdout);
    fputs(s, stdout);
}

static void ctext(double x1, double y1, double x2, double y2, const char *s)
{
    double h = (double)ami_chrsizy(stdout) / GRID;

    text((x1 + x2) / 2, (y1 + y2) / 2 - h / 2, 0, s);
}

static void label(double x, double y, int align, const char *s)
{
    font(AMI_FONT_SIGN, 8, TRUE);
    color(130, 136, 150);
    text(x, y, align, s);
}

/* button looks */
enum { KS_DARK, KS_GREEN, KS_RED, KS_AMBER, KS_BLUE, KS_LIT };

static void key(double x1, double y1, double x2, double y2, const char *lab, int id, int style)
{
    int down = pressed == id;

    color(18, 19, 24);
    frrect(x1, y1 + 1.5, x2, y2 + 1.5, 4);
    switch (style) {
        case KS_GREEN: color(down ? 40 : 60, down ? 140 : 180, down ? 60 : 80); break;
        case KS_RED:   color(down ? 150 : 190, down ? 40 : 60, down ? 40 : 60); break;
        case KS_AMBER: color(down ? 170 : 210, down ? 120 : 150, down ? 20 : 30); break;
        case KS_BLUE:  color(down ? 50 : 70, down ? 90 : 120, down ? 150 : 190); break;
        case KS_LIT:   color(down ? 80 : 95, down ? 86 : 102, down ? 100 : 118); break;
        default:       color(down ? 48 : 58, down ? 50 : 61, down ? 58 : 70); break;
    }
    frrect(x1, y1 + (down ? 1 : 0), x2, y2 + (down ? 1 : 0), 4);
    color(style == KS_DARK ? 150 : 255, style == KS_DARK ? 155 : 255, style == KS_DARK ? 168 : 255);
    if (style == KS_DARK) color(205, 208, 215);
    font(AMI_FONT_SIGN, 9.5, TRUE);
    ctext(x1, y1, x2, y2, lab);
    addhit(x1, y1, x2, y2, id);
}

/* a setting: label above, [-] value [+] */
static void field(double x, double y, double w, const char *lab, const char *val, int iddn, int idup, int idval)
{
    label(x, y - 11, -1, lab);
    key(x, y, x + 16, y + 20, "-", iddn, KS_DARK);
    color(10, 11, 14);
    frrect(x + 18, y, x + w - 18, y + 20, 3);
    color(230, 235, 245);
    font(AMI_FONT_TERM, 10, FALSE);
    ctext(x + 18, y, x + w - 18, y + 20, val);
    addhit(x + 18, y, x + w - 18, y + 20, idval);
    key(x + w - 16, y, x + w, y + 20, "+", idup, KS_DARK);
}

/* a two state setting shown as a lit button */
static void toggle(double x, double y, double w, const char *lab, const char *val, int on, int id)
{
    label(x, y - 11, -1, lab);
    key(x, y, x + w, y + 20, val, id, on ? KS_BLUE : KS_DARK);
}

static void draw_top(void)
{
    char   b[160], b2[200];
    int    running = la.run_mode != LA_STOP;
    double x;

    color(34, 36, 42);
    frect(0, 0, PANW, TOPH);
    color(50, 52, 60);
    frect(0, TOPH - 1, PANW, TOPH);
    /* the name */
    font(AMI_FONT_SIGN, 13, TRUE);
    color(230, 232, 238);
    text(12, 7, -1, "logicport CLI");
    label(132, 11, -1, "LA1034  34 CH  500 MHz");
    /* run controls */
    key(12, 36, 72, 58, "RUN", H_RUN, running && la.run_mode == LA_REPEAT ? KS_GREEN : KS_DARK);
    key(78, 36, 138, 58, "SINGLE", H_SINGLE, running && la.run_mode == LA_SINGLE ? KS_GREEN : KS_DARK);
    key(144, 36, 204, 58, "STOP", H_STOP, running ? KS_RED : KS_DARK);
    key(210, 36, 270, 58, "FORCE", H_FORCE, la.acq == AS_ARMED ? KS_AMBER : KS_DARK);
    /* settings */
    x = 300;
    field(x, 36, 118, "SAMPLE RATE", la_rates[set.rate_ix].name, H_RATE_DN, H_RATE_UP, H_RATE);
    x += 130;
    toggle(x, 36, 44, "RLE", set.rle ? "ON" : "OFF", set.rle && !set.state && la_rates[set.rate_ix].rate <= LA_RLE_MAXRATE, H_RLE);
    x += 54;
    {
        uint64_t w = la_windows[set.window_ix];

        if (w >= 1048576) snprintf(b, sizeof b, "%lluM", (unsigned long long)(w / 1048576));
        else snprintf(b, sizeof b, "%lluk", (unsigned long long)(w / 1024));
    }
    field(x, 36, 90, "RLE WINDOW", b, H_WIN_DN, H_WIN_UP, H_WIN);
    x += 102;
    snprintf(b, sizeof b, "%+.2f V", set.threshold);
    field(x, 36, 108, "THRESHOLD", b, H_THR_DN, H_THR_UP, H_THR);
    x += 120;
    snprintf(b, sizeof b, "%d %%", set.pretrig);
    field(x, 36, 86, "PRE-TRIGGER", b, H_PRE_DN, H_PRE_UP, H_PRE);
    x += 98;
    toggle(x, 36, 66, "MODE", set.state ? "STATE" : "TIMING", set.state, H_MODE);
    x += 72;
    toggle(x, 36, 50, "CLOCK", set.clk_src ? "CLK2" : "CLK1", set.state, H_CLKSRC);
    x += 56;
    toggle(x, 36, 50, "EDGE", set.clk_fall ? "FALL" : "RISE", set.state, H_CLKEDGE);
    /* the link and the acquisition, top right */
    font(AMI_FONT_SIGN, 10, TRUE);
    if (la.sim) { color(230, 180, 40); snprintf(b, sizeof b, "SIMULATED"); }
    else if (la.connected) { color(60, 210, 100); snprintf(b, sizeof b, "LogicPort %s  USB", la.serial); }
    else { color(235, 120, 80); snprintf(b, sizeof b, "%s", la.status); }
    text(PANW - 12, 6, 1, b);
    font(AMI_FONT_TERM, 10, FALSE);
    color(200, 205, 215);
    switch (la.acq) {
        case AS_CONFIG:   snprintf(b, sizeof b, "Loading FPGA image"); break;
        case AS_PREFILL:  snprintf(b, sizeof b, "Pre-trigger fill  %.1f s", la.acq_ms / 1000.0); break;
        case AS_ARMED:    snprintf(b, sizeof b, "Waiting for trigger  %.1f s", la.acq_ms / 1000.0); break;
        case AS_POSTFILL: snprintf(b, sizeof b, "Triggered, post fill"); break;
        case AS_READING:  snprintf(b, sizeof b, "Reading memory"); break;
        default:          snprintf(b, sizeof b, running ? "Starting" : "Stopped"); break;
    }
    snprintf(b2, sizeof b2, "%s   %ld captures", b, la.captures);
    text(PANW - 12, 22, 1, b2);
    if (!la.connected && !la.sim && la.status[0]) {
        font(AMI_FONT_SIGN, 9, FALSE);
        color(170, 120, 100);
    }
}

/* the channel column */
static void draw_channels(void)
{
    int    c, row = 0, nv = nvisible(), grp;
    double rh = (double)WH / nv, y;
    char   b[32];

    color(26, 27, 32);
    frect(0, AXY1, LBLW, PANH - BOTH - 4);
    label(8, AXY1 + 6, -1, "CHANNEL");
    label(LBLW - 46, AXY1 + 6, 0, "TRIG");
    key(LBLW - 24, AXY1 + 3, LBLW - 6, AXY1 + 17, set.mask == LA_ALLCH ? "ALL" : "8", H_ALL, KS_DARK);
    for (c = 0; c < LA_NCH; c++) {
        if (!(set.mask & (UINT64_C(1) << c))) continue;
        y = WY1 + row * rh;
        grp = c / 8;
        color(grpcol[grp].r, grpcol[grp].g, grpcol[grp].b);
        /* the enable square */
        frrect(LBLW - 20, y + rh / 2 - 5, LBLW - 10, y + rh / 2 + 5, 2);
        addhit(LBLW - 24, y, LBLW - 6, y + rh, H_EN0 + c);
        /* the trigger condition */
        {
            int on = set.cond[c] != TC_X;

            color(on ? 190 : 44, on ? 60 : 46, on ? 60 : 54);
            frrect(LBLW - 62, y + rh / 2 - 7, LBLW - 32, y + rh / 2 + 7, 3);
            color(on ? 255 : 150, on ? 255 : 155, on ? 255 : 170);
            font(AMI_FONT_TERM, rh < 20 ? 9 : 10, TRUE);
            ctext(LBLW - 62, y + rh / 2 - 7, LBLW - 32, y + rh / 2 + 7, condname[set.cond[c]]);
            addhit(LBLW - 62, y, LBLW - 32, y + rh, H_COND0 + c);
        }
        /* the name */
        if (editing == c) {
            color(50, 60, 90);
            frrect(4, y + 2, LBLW - 68, y + rh - 2, 3);
            snprintf(b, sizeof b, "%s_", edit);
            color(255, 255, 255);
        } else {
            snprintf(b, sizeof b, "%s", names[c]);
            color(grpcol[grp].r, grpcol[grp].g, grpcol[grp].b);
        }
        font(AMI_FONT_SIGN, rh < 20 ? 9.5 : 11, TRUE);
        {
            double h = (double)ami_chrsizy(stdout) / GRID;

            text(10, y + rh / 2 - h / 2, -1, b);
        }
        addhit(0, y, LBLW - 68, y + rh, H_NAME0 + c);
        row++;
    }
    /* the hidden channels, if any, as a line of dim squares at the bottom */
}

/* a nice 1-2-5 step for the axis, in samples, giving about n ticks */
static double nice_step(double span, int n)
{
    double raw = span / n, mag = pow(10, floor(log10(raw))), r = raw / mag;

    if (r < 1.5) return mag;
    if (r < 3.5) return 2 * mag;
    if (r < 7.5) return 5 * mag;
    return 10 * mag;
}

static void draw_axis(void)
{
    uint64_t rate = cap_rate();
    double   step, sstep, t, x;
    char     b[48];

    color(26, 27, 32);
    frect(WX1 - 2, AXY1, PANW, AXY2);
    /* in seconds when there is a rate, in samples otherwise */
    if (rate) {
        step = nice_step(view_span / (double)rate, 10);
        sstep = step * (double)rate;
    } else sstep = step = nice_step(view_span, 10);
    if (sstep <= 0) return;
    color(70, 74, 86);
    line(WX1, AXY2, WX2, AXY2, 1);
    font(AMI_FONT_TERM, 9, FALSE);
    for (t = floor(view_s0 / sstep) * sstep; t <= view_s0 + view_span; t += sstep) {
        x = WX1 + (t - view_s0) / view_span * WW;
        if (x < WX1 || x > WX2) continue;
        color(90, 95, 110);
        line(x, AXY2 - 5, x, AXY2, 1);
        color(170, 176, 190);
        if (rate) fmt_eng(t / (double)rate, "s", b, sizeof b);
        else snprintf(b, sizeof b, "%.0f", t);
        {
            double hw = tw(b) / 2, lx = x;

            if (lx - hw < WX1) lx = WX1 + hw;
            if (lx + hw > WX2) lx = WX2 - hw;
            text(lx, AXY1 + 2, 0, b);
        }
    }
    addhit(WX1, AXY1, WX2, AXY2, H_AXIS);
}

/* the waveforms */
static void draw_waves(void)
{
    int      c, row, nv = nvisible(), i, grp;
    double   rh = (double)WH / nv, y, hi, lo, x, x1, x2, t;
    double   sstep;
    uint64_t rate = cap_rate();
    int64_t  zero;
    double   pos;
    char     b[48];

    color(12, 13, 16);
    frect(WX1, WY1, WX2, WY2);
    /* grid */
    if (rate) sstep = nice_step(view_span / (double)rate, 10) * (double)rate;
    else sstep = nice_step(view_span, 10);
    color(28, 30, 36);
    for (t = floor(view_s0 / sstep) * sstep; t <= view_s0 + view_span; t += sstep) {
        x = WX1 + (t - view_s0) / view_span * WW;
        if (x >= WX1 && x <= WX2) line(x, WY1, x, WY2, 0.5);
    }
    for (row = 1; row < nv; row++) line(WX1, WY1 + row * rh, WX2, WY1 + row * rh, 0.5);
    if (!cap) {
        font(AMI_FONT_SIGN, 14, FALSE);
        color(90, 100, 120);
        ctext(WX1, WY1, WX2, WY2, la.connected || la.sim ? "Press RUN or SINGLE to capture" : la.status);
        addhit(WX1, WY1, WX2, WY2, H_WAVE);
        return;
    }
    zero = cap_zero();
    /* the capture's extent, where it does not fill the view */
    {
        double xa = WX1 + (-(double)zero - view_s0) / view_span * WW;
        double xb = WX1 + ((double)cap->total - zero - view_s0) / view_span * WW;

        color(20, 21, 26);
        if (xa > WX1) frect(WX1, WY1, xa < WX2 ? xa : WX2, WY2);
        if (xb < WX2) frect(xb > WX1 ? xb : WX1, WY1, WX2, WY2);
    }
    row = 0;
    for (c = 0; c < LA_NCH; c++) {
        /* a channel's level segments: the runs merged where its bit does
           not change, since any other channel's change starts a new run */
        double dense_x1 = -1, dense_x2 = -1, seg_start = 0;
        int    prevbit = -1, bit, first = 1;

        if (!(set.mask & (UINT64_C(1) << c))) continue;
        y = WY1 + row * rh;
        hi = y + rh * 0.22;
        lo = y + rh * 0.78;
        grp = c / 8;
        color(grpcol[grp].r, grpcol[grp].g, grpcol[grp].b);
        pos = -(double)zero;
        for (i = 0; i <= cap->nruns; i++) {
            double end;

            if (i < cap->nruns) {
                end = pos + (double)cap->runs[i].len;
                bit = (int)((cap->runs[i].value >> c) & 1);
                if (prevbit < 0) { prevbit = bit; seg_start = pos; pos = end; continue; }
                if (bit == prevbit) { pos = end; continue; }
            } else { end = pos; bit = -1; }
            /* the segment seg_start..pos at level prevbit ends here */
            if (pos >= view_s0 && seg_start <= view_s0 + view_span) {
                x1 = WX1 + (seg_start - view_s0) / view_span * WW;
                x2 = WX1 + (pos - view_s0) / view_span * WW;
                if (x1 < WX1) x1 = WX1;
                if (x2 > WX2) x2 = WX2;
                if (x2 - x1 < 0.7) {
                    /* too narrow to draw: collect into a dense block */
                    if (dense_x1 < 0) dense_x1 = x1;
                    dense_x2 = x2;
                } else {
                    if (dense_x1 >= 0) {
                        frect(dense_x1, hi, dense_x2 + 0.5, lo);
                        dense_x1 = -1;
                    }
                    if (!first && seg_start >= view_s0) line(x1, hi, x1, lo, 1);
                    line(x1, prevbit ? hi : lo, x2, prevbit ? hi : lo, 1.2);
                }
            }
            first = 0;
            if (seg_start > view_s0 + view_span) break;
            seg_start = pos;
            prevbit = bit;
            pos = end;
        }
        if (dense_x1 >= 0) frect(dense_x1, hi, dense_x2 + 0.5, lo);
        row++;
    }
    /* the trigger */
    if (cap->trigger >= 0) {
        x = WX1 + (0 - view_s0) / view_span * WW;
        if (x >= WX1 && x <= WX2) {
            color(230, 60, 60);
            line(x, WY1, x, WY2, 1);
            ftri(x - 6, AXY2 - 8, x + 6, AXY2 - 8, x, AXY2);
            font(AMI_FONT_SIGN, 8, TRUE);
            color(255, 255, 255);
            text(x, AXY2 - 8.5, 0, "T");
        }
    }
    /* the cursors */
    if (cur_a > -1e299) {
        x = WX1 + (cur_a - view_s0) / view_span * WW;
        if (x >= WX1 && x <= WX2) {
            color(250, 220, 60);
            ami_linestyle(stdout, ami_lsdash);
            line(x, WY1, x, WY2, 1);
            ami_linestyle(stdout, ami_lssolid);
            frrect(x - 7, WY1, x + 7, WY1 + 12, 2);
            font(AMI_FONT_SIGN, 8, TRUE);
            color(20, 20, 20);
            text(x, WY1 + 1, 0, "A");
        }
    }
    if (cur_b > -1e299) {
        x = WX1 + (cur_b - view_s0) / view_span * WW;
        if (x >= WX1 && x <= WX2) {
            color(80, 210, 240);
            ami_linestyle(stdout, ami_lsdash);
            line(x, WY1, x, WY2, 1);
            ami_linestyle(stdout, ami_lssolid);
            frrect(x - 7, WY1, x + 7, WY1 + 12, 2);
            font(AMI_FONT_SIGN, 8, TRUE);
            color(20, 20, 20);
            text(x, WY1 + 1, 0, "B");
        }
    }
    /* the channel values at cursor A, in the label column's right edge */
    (void)b;
    addhit(WX1, WY1, WX2, WY2, H_WAVE);
}

/* the strip along the bottom: cursors, the view, messages, buttons */
static void draw_bottom(void)
{
    char   b1[48], b2[48], b3[48], buf[256];
    double y1 = PANH - BOTH, y2 = PANH - 4;

    color(26, 27, 32);
    frect(0, y1 - 2, PANW, PANH);
    key(10, y1 + 6, 56, y2 - 6, "FIT", H_FIT, KS_DARK);
    key(62, y1 + 6, 92, y2 - 6, "+", H_ZIN, KS_DARK);
    key(98, y1 + 6, 128, y2 - 6, "-", H_ZOUT, KS_DARK);
    font(AMI_FONT_TERM, 10, FALSE);
    color(200, 205, 215);
    fmt_time(view_span / 10, b1, sizeof b1);
    snprintf(buf, sizeof buf, "%s/div", b1);
    text(140, y1 + 11, -1, buf);
    if (cap) {
        char b4[48];

        fmt_time((double)cap->total, b4, sizeof b4);
        if (cap->rate) snprintf(buf, sizeof buf, "%d runs  %llu samples  %s  %s%s", cap->nruns,
                                (unsigned long long)cap->total, b4, la_rates[0].name[0] ? "" : "", cap->rle ? "RLE" : "");
        else snprintf(buf, sizeof buf, "%d runs  %llu clocks  state", cap->nruns, (unsigned long long)cap->total);
        /* the rate */
        {
            int i;

            for (i = 0; i < LA_NRATES; i++) if (la_rates[i].rate == cap->rate) break;
            if (i < LA_NRATES) {
                size_t n = strlen(buf);

                snprintf(buf + n, sizeof buf - n, "  @ %s", la_rates[i].name);
            }
        }
        color(130, 140, 160);
        text(240, y1 + 11, -1, buf);
    }
    /* cursors */
    if (cur_a > -1e299 || cur_b > -1e299) {
        buf[0] = 0;
        if (cur_a > -1e299) { fmt_time(cur_a, b1, sizeof b1); snprintf(buf, sizeof buf, "A %s", b1); }
        if (cur_b > -1e299) { fmt_time(cur_b, b2, sizeof b2); snprintf(buf + strlen(buf), sizeof buf - strlen(buf), "   B %s", b2); }
        if (cur_a > -1e299 && cur_b > -1e299) {
            double d = fabs(cur_b - cur_a);
            uint64_t rate = cap_rate();

            fmt_time(d, b3, sizeof b3);
            snprintf(buf + strlen(buf), sizeof buf - strlen(buf), "   dT %s", b3);
            if (rate && d > 0) {
                fmt_eng((double)rate / d, "Hz", b3, sizeof b3);
                snprintf(buf + strlen(buf), sizeof buf - strlen(buf), "  %s", b3);
            }
        }
        color(250, 220, 60);
        font(AMI_FONT_TERM, 10, FALSE);
        text(600, y1 + 11, -1, buf);
        key(900, y1 + 6, 950, y2 - 6, "CLEAR", H_CLEAR, KS_DARK);
    }
    if (note_until) {
        font(AMI_FONT_SIGN, 10, FALSE);
        color(250, 215, 50);
        text(PANW - 190, y1 + 11, 1, note);
    }
    key(PANW - 180, y1 + 6, PANW - 96, y2 - 6, "SAVE CSV", H_CSV, KS_BLUE);
    key(PANW - 90, y1 + 6, PANW - 10, y2 - 6, "SAVE VCD", H_VCD, KS_BLUE);
}

static void redraw(void)
{
    static int buf = 1;
    int    w, h;
    double scale, voffx, voffy;

    buf = 3 - buf;
    ami_select(stdout, buf, 3 - buf);
    ami_auto(stdout, FALSE);
    ami_curvis(stdout, FALSE);
    ami_binvis(stdout);
    ami_viewoffg(stdout, 0, 0);
    ami_viewscale(stdout, 1, 1);
    w = (int)ami_maxxg(stdout);
    h = (int)ami_maxyg(stdout);
    ami_bcolorg(stdout, cv(12), cv(13), cv(16));
    ami_fcolorg(stdout, cv(12), cv(13), cv(16));
    ami_frect(stdout, 1, 1, w, h);
    scale = fmin((double)w / (PANW * GRID), (double)h / (PANH * GRID));
    vscale = scale;
    voffx = floor((w - PANW * GRID * scale) / 2);
    voffy = floor((h - PANH * GRID * scale) / 2);
    ami_viewoffg(stdout, (ami_long)voffx, (ami_long)voffy);
    ami_viewscale(stdout, (float)scale, (float)scale);
    nhits = 0;
    color(20, 21, 26);
    frect(0, 0, PANW, PANH);
    draw_top();
    draw_axis();
    draw_channels();
    draw_waves();
    draw_bottom();
    ami_select(stdout, buf, buf);
}

#ifdef SHOT
static void run_script(const char *script)
{
    static const struct { const char *n; int id; } tab[] = {
        { "RUN", H_RUN }, { "SINGLE", H_SINGLE }, { "STOP", H_STOP }, { "FORCE", H_FORCE },
        { "RATE-", H_RATE_DN }, { "RATE+", H_RATE_UP }, { "RLE", H_RLE }, { "THR-", H_THR_DN },
        { "THR+", H_THR_UP }, { "PRE-", H_PRE_DN }, { "PRE+", H_PRE_UP }, { "MODE", H_MODE },
        { "FIT", H_FIT }, { "ZIN", H_ZIN }, { "ZOUT", H_ZOUT }, { "CSV", H_CSV }, { "VCD", H_VCD },
        { "ALL", H_ALL }
    };
    char  buf[256], *tok, *save;
    int   i;

    snprintf(buf, sizeof buf, "%s", script);
    for (tok = strtok_r(buf, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
        if (!strncmp(tok, "COND", 4)) { click(H_COND0 + atoi(tok + 4)); continue; }
        if (!strncmp(tok, "EN", 2)) { click(H_EN0 + atoi(tok + 2)); continue; }
        if (!strncmp(tok, "CURA", 4)) { cur_a = atof(tok + 4); continue; }
        if (!strncmp(tok, "CURB", 4)) { cur_b = atof(tok + 4); continue; }
        if (!strncmp(tok, "ZOOM", 4)) { zoom(atof(tok + 4), view_s0 + view_span / 2); continue; }
        if (!strncmp(tok, "S0", 2)) { view_s0 = atof(tok + 2); view_fit = 0; continue; }
        if (!strncmp(tok, "SPAN", 4)) { view_span = atof(tok + 4); view_fit = 0; continue; }
        for (i = 0; i < (int)(sizeof tab / sizeof tab[0]); i++)
            if (!strcmp(tab[i].n, tok)) { click(tab[i].id); fprintf(stderr, "script: %s at %ld ms\n", tok, now_ms()); }
    }
}
#endif

/* ---------------------------------------------------------------- input */

#define VERSION "0.1"

static void version(void)
{
    /* stdout is the window under Ami: this goes to the terminal, and the
       window is not held open afterwards */
    ami_autohold(FALSE);
    fprintf(stderr, "logicport CLI %s\n"
           "A logic analyzer for the LogicPort LA1034, built on the AMI tool kit.\n"
           "Not affiliated with or endorsed by Intronix Test Instruments.\n"
           "LogicPort is a product name of Intronix.\n", VERSION);
    exit(0);
}

static void usage(const char *prog)
{
    fprintf(stderr, "usage: %s [--sim] [--version]\n  --sim       a simulated analyzer, no hardware\n"
                    "  --version   print the version and exit\n", prog);
    exit(1);
}

static void take_capture(void)
{
    if (!la.newcap) return;
    la_free_capture(cap);
    cap = la.newcap;
    la.newcap = NULL;
    if (view_fit) fit();
    if (trace) fprintf(stderr, "panel: capture %u at %ld ms, %d runs, %llu samples, trigger %lld\n", cap->seq, now_ms(),
                       cap->nruns, (unsigned long long)cap->total, (long long)cap->trigger);
}

int main(int argc, char *argv[])
{
    ami_evtrec er;
    ami_long   wx, wy;
    int sim = 0, i, id, dirty;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--sim") || !strcmp(argv[i], "-s")) sim = 1;
        else if (!strcmp(argv[i], "--version") || !strcmp(argv[i], "-V")) version();
        else usage(argv[0]);
    }
    trace = getenv("LOGICPORT_TRACE") != NULL;
#ifdef SHOT
    shot_file = getenv("LOGICPORT_SHOT");
    if (shot_file) { screen_capture_name(shot_file); shot_at = now_ms() + (getenv("LOGICPORT_SHOT_MS") ? atol(getenv("LOGICPORT_SHOT_MS")) : 5000); }
    script = getenv("LOGICPORT_SCRIPT");
    if (script) script_at = now_ms() + 1000;
    script2 = getenv("LOGICPORT_SCRIPT2");
    if (script2) script2_at = now_ms() + 2500;
#endif
    defaults();
    load_rc();
    la_init(&la, &set, sim);
    fit();
    ami_autohold(FALSE);
    ami_auto(stdout, FALSE);
    ami_curvis(stdout, FALSE);
    ami_binvis(stdout);
    ami_title(stdout, sim ? "logicport CLI (simulated)" : "logicport CLI");
    ami_autobuf(stdout, FALSE);
    ami_winclientg(stdout, WINW, WINH, &wx, &wy, BIT(ami_wmframe) | BIT(ami_wmsize) | BIT(ami_wmsysbar));
    ami_setsizg(stdout, wx, wy);
    ami_timer(stdout, TICK_ID, TICK, TRUE);
    redraw();
    do {
        ami_event(stdin, &er);
        dirty = 1;
        switch (er.etype) {
            case ami_ettim:
                dirty = la_poll(&la);
                if (dirty) take_capture();
                if (note_until && now_ms() >= note_until) { note_until = 0; dirty = 1; }
#ifdef SHOT
                if (script && now_ms() >= script_at) { run_script(script); script = NULL; dirty = 1; }
                if (script2 && now_ms() >= script2_at) { run_script(script2); script2 = NULL; dirty = 1; }
                if (shot_file && now_ms() >= shot_at) {
                    redraw();
                    screen_capture();
                    er.etype = ami_etterm;
                }
#endif
                break;
            case ami_etmoumovg:
                mx = (int)ami_scalex(stdout, er.moupxg);
                my = (int)ami_scaley(stdout, er.moupyg);
                dirty = 0;
                if (dragging) {
                    double ds = (double)(drag_x - mx) / GRID / WW * view_span;

                    if (abs(mx - drag_x) > S(0.8)) drag_moved = 1;
                    if (drag_moved) {
                        view_s0 = drag_s0 + ds;
                        view_fit = 0;
                        dirty = 1;
                    }
                }
                break;
            case ami_etmouba:
                id = hit(mx, my);
                if (er.amoubn == 4 || er.amoubn == 5) {
                    int dir = er.amoubn == 4 ? 1 : -1;

                    if (!wheel(id, dir)) {
                        if (id == H_WAVE || id == H_AXIS) zoom(dir > 0 ? 1.5 : 1 / 1.5, sample_at(mx));
                        else dirty = 0;
                    }
                    break;
                }
                if (er.amoubn == 3) {
                    if (id == H_WAVE) cur_b = sample_at(mx);
                    else dirty = 0;
                    break;
                }
                if (er.amoubn != 1) { dirty = 0; break; }
                if (editing >= 0 && id != H_NAME0 + editing) { editing = -1; }
                if (id == H_WAVE) {
                    dragging = 1;
                    drag_x = mx;
                    drag_moved = 0;
                    drag_s0 = view_s0;
                    dirty = 0;
                } else pressed = id;
                break;
            case ami_etmoubd:
                if (er.dmoubn == 1 && dragging) {
                    if (!drag_moved) cur_a = sample_at(mx);
                    dragging = 0;
                } else if (er.dmoubn == 1 && pressed != H_NONE) {
                    if (hit(mx, my) == pressed) click(pressed);
                    pressed = H_NONE;
                } else dirty = 0;
                break;
            case ami_etchar:
                if (editing >= 0) {
                    size_t n = strlen(edit);

                    if (isprint((unsigned char)er.echar) && n < sizeof edit - 2) { edit[n] = er.echar; edit[n + 1] = 0; }
                    break;
                }
                switch (er.echar) {
                    case 'r': case 'R': click(H_RUN); break;
                    case 's': case 'S': click(H_SINGLE); break;
                    case ' ':           click(H_STOP); break;
                    case 't': case 'T': click(H_FORCE); break;
                    case 'f': case 'F': fit(); break;
                    case '+': case '=': zoom(1.5, view_s0 + view_span / 2); break;
                    case '-': case '_': zoom(1 / 1.5, view_s0 + view_span / 2); break;
                    case 'a': case 'A': if (hit(mx, my) == H_WAVE) cur_a = sample_at(mx); break;
                    case 'b': case 'B': if (hit(mx, my) == H_WAVE) cur_b = sample_at(mx); break;
                    case 'c': case 'C': cur_a = cur_b = -1e300; break;
                    case 'x': case 'X': save_capture(0); break;
                    case 'v': case 'V': save_capture(1); break;
                    default: dirty = 0; break;
                }
                break;
            case ami_etenter:
                if (editing >= 0) {
                    if (edit[0]) snprintf(names[editing], sizeof names[editing], "%s", edit);
                    editing = -1;
                    save_rc();
                } else dirty = 0;
                break;
            case ami_etcan: case ami_etstop:
                if (editing >= 0) editing = -1; else dirty = 0;
                break;
            case ami_etdelcb:
                if (editing >= 0 && edit[0]) edit[strlen(edit) - 1] = 0; else dirty = 0;
                break;
            case ami_etleft:  pan(-view_span / 10); break;
            case ami_etright: pan(view_span / 10); break;
            case ami_etup:    step_rate(1); break;
            case ami_etdown:  step_rate(-1); break;
            case ami_etpagu:  step_threshold(1); break;
            case ami_etpagd:  step_threshold(-1); break;
            case ami_etresize:
                ami_sizbufg(stdout, er.rszxg, er.rszyg);
                break;
            case ami_etredraw:
                break;
            case ami_etterm:
                dirty = 0;
                break;
            default:
                dirty = 0;
                break;
        }
        if (dirty && er.etype != ami_etterm) redraw();
    } while (er.etype != ami_etterm);
    save_rc();
    la_shutdown(&la);
    la_free_capture(cap);
    return 0;
}
