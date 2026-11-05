// Canon CAPT laser printers: the LBP3300. 1 bit per pixel, 600 dpi,
// HiSCoA-compressed bands of 70 lines inside CAPT commands, with the printer's
// status polled between them.
//
// Sources: captdriver (prn_lbp2900.c, rastertocapt.c, hiscoa; Copyright (C)
// 2013 Alexey Galakhov, (C) 2016 Alexei Gordeev) for the command layer and the
// compression; and for the LBP3300 itself the working job sequence, page
// parameters and paper source selection of https://github.com/thientranvan/
// esp32_printer_canon_3300 (README, print_test.py, mac_backend.pl), whose
// author observed the installed Canon driver. What that taught (confirmed by
// printing on the real printer): the model key is 94 2A (the IDENT reply),
// the paper source must be selected with E1A9 before every page and patched
// into the page parameters (byte 6), the image is a 592-byte x 6774-line
// window 120 pixels inside the 4960 x 7014 page, and there is no E0A6/GPIO
// step in a job. Without the source the printer says "paper out" the moment
// it is told to print and never feeds.
//
// Two-sided printing (the LBP3300 has a duplex unit): each side is a page of
// its own for the printer, the BACK of a sheet is sent first (flag 0x04 in byte
// 5 of the page parameters), then the FRONT (flag 0x02). AirPrint sends front,
// back, front, ...: the front is held, compressed, in the job's big buffer
// until its back has been sent. A last odd side is a one-sided page.
//
// Licensed under the GNU General Public License Version 3 or later; see
// hiscoa.c for the notice. Because of that, a firmware built with this file
// is distributed under the GPLv3 (the rest of the project is MIT).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "capt.h"
#include "hiscoa.h"
#include "printer_driver.h"

#ifdef ESP_PLATFORM
#include "esp_log.h"
static const char *TAG = "canon_capt";
#define LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#define LOGW(...) ESP_LOGW(TAG, __VA_ARGS__)
#else
#define LOGI(...) ((void)0)
#define LOGW(...) (fprintf(stderr, "canon_capt: " __VA_ARGS__), fputc('\n', stderr))
#endif

#define BAND_LINES 70           // lines per compressed band (the bands are independent)
#define PACKET_MAX 4092         // data of one PRINT_DATA command: 4096 bytes with its header
#define MARGIN_PX 120           // the image starts this far from the top and left edge (600 dpi), 5 mm
#define DECODE_MAX_S 60         // waits (seconds) for the printer to decode a page
#define PRINT_MAX_S 300         // ... to finish printing
#define BUFFER_MAX_S 120        // ... for room in its buffer
#define SOURCE_CASSETTE_1 1
#define MEDIA_PLAIN 1
#define FLAG_FRONT 0x02         // byte 5 of the page parameters: the front / back side of a two-sided sheet
#define FLAG_BACK 0x04

// Whether the back image must be rotated by 180 degrees for a long-edge / short-edge sheet. The pages
// arrive upright (AirPrint DM1, the web UI); the printer prints a back as it is, upside down when the sheet is
// turned over the long edge. Seen on the printer (2026-11-05, long edge: the back was upside down without the
// rotation); the short edge is the opposite turn, so it needs none, but that one is not tried yet.
#define BACK_ROTATE_LONG 1
#define BACK_ROTATE_SHORT 0

typedef struct {
    printer_io_t io;
    capt_t c;
    uint16_t job_id;
    uint8_t model_key[2];       // from the IDENT reply: 94 2A on the LBP3300

    int dpi, scale;             // scale: 600 / dpi
    unsigned page_w, page_h;    // the paper in pixels at 600 dpi
    unsigned line_bytes;        // bytes of an image line
    unsigned lines;             // lines of the image
    unsigned x0;                // first source pixel of the image
    unsigned paper_sz;          // the page size byte of the page parameters

    // two-sided printing
    int duplex;                 // mode of the next page (page_options): 0 one-sided, 1 long edge, 2 short edge
    enum { SIDE_ONE, SIDE_FRONT, SIDE_BACK } side;      // what the page being received is
    uint8_t *big;               // the job's big buffer: the held front from its start, a raw back from its end
    size_t big_size, held_cap, held_len;
    bool held;                  // a front is waiting for its back
    bool rotate;                // the back being received is rotated by 180 degrees (kept raw until its end)
    uint8_t *raw;               // lines x line_bytes at the end of the big buffer while a rotated back is received

    bool started;               // the printer has a job open (JOB_BEGIN answered)
    bool in_page;
    int err;                    // first failure, nonzero stops everything
    int pages;                  // pages (sides) sent to the printer in this job
    int sheets;                 // sheets among them: the printer counts a two-sided sheet once, at its front
    uint8_t cur_flag;           // the flag of the page being sent
    int src_w, src_h, bpp;
    unsigned abs_row;           // rows (600 dpi) of the page seen so far
    unsigned rows;              // image rows put in bands so far
    unsigned band_fill;         // rows in the band
    unsigned packets;           // PRINT_DATA packets sent on the page
    uint8_t *band, *comp, *row;
    size_t comp_size;
} capt_job_t;

static const uint8_t JOB_BEGIN_DATA[] = {0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00};

// ---------------------------------------------------------------------------
// papers (600 dpi pixels): the page, the image window and the page size code.
// The A4 numbers are Canon's own (4960 x 7014 page, 592 x 6774 image); Letter
// is the same layout on 5100 x 6600 with the code captdriver uses, untested.

typedef struct {
    unsigned px_w, px_h, sz, line_bytes, lines;
} capt_paper_t;

static const capt_paper_t GEOM[] = {
    {4960, 7014, 0x02, 592, 6774},    // A4 (= PAPERS[0])
    {5100, 6600, 0x0D, 610, 6360},    // Letter
};

static const printer_paper_t PAPERS[] = {
    {"A4", "iso_a4_210x297mm", 21000, 29700},
    {"Letter", "na_letter_8.5x11in", 21590, 27940},
};

// ---------------------------------------------------------------------------
// halftone: 8x8 ordered dither of the luminance, 1 = black

static uint8_t s_thr[8][8];
static uint8_t s_expand[16];    // 4 source bits -> 8 output bits (300 dpi: every pixel twice)
static uint8_t s_rev[256];      // a byte with its bits reversed

static void init_tables(void)
{
    if (s_thr[0][1])
        return;
    for (unsigned y = 0; y < 8; y++)
        for (unsigned x = 0; x < 8; x++) {
            unsigned xc = x ^ y;
            unsigned m = ((y & 1) << 5) | ((xc & 1) << 4) | ((y & 2) << 2) | ((xc & 2) << 1) | ((y & 4) >> 1) |
                         ((xc & 4) >> 2);
            s_thr[y][x] = (uint8_t)(m * 4 + 2);     // 2..254: white (255) never prints, black (0) always
        }
    for (unsigned n = 0; n < 16; n++) {
        unsigned v = 0;
        for (unsigned k = 0; k < 4; k++)
            if (n & (8u >> k))
                v |= 3u << (6 - 2 * k);
        s_expand[n] = (uint8_t)v;
    }
    for (unsigned n = 0; n < 256; n++) {
        unsigned v = 0;
        for (unsigned k = 0; k < 8; k++)
            if (n & (1u << k))
                v |= 0x80u >> k;
        s_rev[n] = (uint8_t)v;
    }
}

// The image window of source line px (job dpi) at dither row srow, as bits of
// 600 dpi into out[line_bytes].
static void render_window(const capt_job_t *j, const uint8_t *px, unsigned srow, uint8_t *out)
{
    if (!px) {
        memset(out, 0, j->line_bytes);
        return;
    }
    const unsigned ppb = j->scale == 2 ? 4 : 8;     // source pixels per output byte
    const unsigned w = (unsigned)j->src_w, bpp = (unsigned)j->bpp;
    const uint8_t *t = s_thr[srow & 7];
    for (unsigned b = 0; b < j->line_bytes; b++) {
        unsigned x = j->x0 + b * ppb, bits = 0;
        for (unsigned k = 0; k < ppb; k++, x++) {
            unsigned v = 255;
            if (x < w) {
                const uint8_t *p = px + x * bpp;
                v = bpp == 1 ? p[0] : (p[0] * 77u + p[1] * 151u + p[2] * 28u) >> 8;
            }
            bits = (bits << 1) | (v < t[x & 7]);
        }
        out[b] = ppb == 8 ? (uint8_t)bits : s_expand[bits];
    }
}

// One image row turned by 180 degrees. Turning the whole page maps the image window onto the
// page area from MARGIN_PX to the opposite edge: bits 0 .. r-1 of the new row are the first r bits
// of the old one in reverse order (r = the page width less both margins), the rest is white.
static void rotate_row(const capt_job_t *j, const uint8_t *src, uint8_t *dst)
{
    const unsigned r = j->page_w - 2 * MARGIN_PX;
    memset(dst, 0, j->line_bytes);
    if (r % 8 == 0) {
        const unsigned nb = r / 8;
        for (unsigned i = 0; i < nb; i++)
            dst[i] = s_rev[src[nb - 1 - i]];
    } else {
        for (unsigned i = 0; i < r; i++) {
            unsigned from = r - 1 - i;
            if (src[from / 8] & (0x80u >> (from % 8)))
                dst[i / 8] |= (uint8_t)(0x80u >> (i % 8));
        }
    }
}

// ---------------------------------------------------------------------------
// protocol steps. Each returns 0 or a negative code and sets j->err.

static int fail(capt_job_t *j, int r, const char *what)
{
    if (!j->err) {
        j->err = r < 0 ? r : -1;
        LOGW("%s failed (%d)", what, r);
    }
    return j->err;
}

#define STEP(j, expr, what)                                                                                           \
    do {                                                                                                               \
        int r_ = (expr);                                                                                               \
        if (r_ < 0)                                                                                                    \
            return fail((j), r_, (what));                                                                              \
    } while (0)

static int delay_ms(capt_job_t *j, uint32_t ms)
{
    return j->io.delay ? j->io.delay(j->io.ctx, ms) : 0;
}

// The job information command E1A1 (72 bytes): stage 1 starts the job, 2 a
// page's data is complete, 6 it was fired, 4 ends the job.
static int job_info(capt_job_t *j, uint8_t stage, uint16_t page)
{
    time_t t = time(NULL);
    const struct tm *tm = localtime(&t);
    unsigned year = tm ? (unsigned)tm->tm_year + 1900 : 2026;
    uint8_t buf[72] = {0};
    buf[4] = (uint8_t)(page & 0xFF);
    buf[5] = (uint8_t)(page >> 8);
    buf[16] = stage;
    buf[17] = 0x01;
    buf[18] = (uint8_t)(j->job_id & 0xFF);
    buf[19] = (uint8_t)(j->job_id >> 8);
    buf[20] = buf[22] = 0x5C;       // -420: the time zone (UTC+7) and its daylight saving, in minutes
    buf[21] = buf[23] = 0xFE;
    buf[24] = (uint8_t)(year & 0xFF);
    buf[25] = (uint8_t)(year >> 8);
    buf[26] = (uint8_t)(tm ? tm->tm_mon : 0);
    buf[27] = (uint8_t)(tm ? tm->tm_mday : 1);
    buf[28] = (uint8_t)(tm ? tm->tm_hour : 0);
    buf[29] = (uint8_t)(tm ? tm->tm_min : 0);
    buf[30] = (uint8_t)(tm ? tm->tm_sec : 0);
    buf[31] = 0x01;
    return capt_sendrecv(&j->c, CAPT_JOB_SETUP, buf, sizeof(buf), NULL, NULL);
}

// the session start E0A5: a page number of 0 for a new job
static int go_online(capt_job_t *j, uint16_t page)
{
    uint8_t d[16] = {0xEE, 0xDB, 0xEA, 0xAD, (uint8_t)(page & 0xFF), (uint8_t)(page >> 8)};
    return capt_sendrecv(&j->c, CAPT_UPLOAD_2, d, sizeof(d), NULL, NULL);
}

static int go_offline(capt_job_t *j)
{
    static const uint8_t d[2] = {0x01, 0x00};
    return capt_sendrecv(&j->c, CAPT_LBP3000_SETUP_0, d, sizeof(d), NULL, NULL);
}

static int job_prologue(capt_job_t *j)
{
    uint8_t buf[8] = {0};
    size_t n = sizeof(buf);

    memset(&j->c.st, 0, sizeof(j->c.st));
    STEP(j, capt_sendrecv(&j->c, CAPT_IDENT, NULL, 0, buf, &n), "IDENT");
    if (n >= 4) {
        j->model_key[0] = buf[2];
        j->model_key[1] = buf[3];
    }
    if (j->model_key[0] != 0x94 || j->model_key[1] != 0x2A)
        LOGW("model key %02X %02X (the LBP3300 answers 94 2A)", j->model_key[0], j->model_key[1]);
    STEP(j, capt_get_xstatus(&j->c), "status");
    if (j->c.st.page_decoding != j->c.st.page_printing || j->c.st.page_printing != j->c.st.page_out ||
        j->c.st.page_out != j->c.st.page_completed)
        LOGW("the printer still has a page in progress (%u/%u/%u/%u)", j->c.st.page_decoding, j->c.st.page_printing,
             j->c.st.page_out, j->c.st.page_completed);

    STEP(j, capt_sendrecv(&j->c, CAPT_START_0, NULL, 0, NULL, NULL), "START_0");
    n = sizeof(buf);
    STEP(j, capt_sendrecv(&j->c, CAPT_JOB_BEGIN, JOB_BEGIN_DATA, sizeof(JOB_BEGIN_DATA), buf, &n), "JOB_BEGIN");
    j->job_id = n >= 4 ? (uint16_t)(buf[2] | (buf[3] << 8)) : 0;
    j->started = true;
    LOGI("job %u started", j->job_id);
    STEP(j, job_info(j, 1, 0), "job start");
    STEP(j, capt_sendrecv(&j->c, CAPT_START_1, NULL, 0, NULL, NULL), "START_1");
    STEP(j, capt_sendrecv(&j->c, CAPT_START_2, NULL, 0, NULL, NULL), "START_2");
    STEP(j, capt_sendrecv(&j->c, CAPT_START_3, NULL, 0, NULL, NULL), "START_3");
    STEP(j, go_online(j, 0), "session start");
    return 0;
}

// (value | 0x100) for each overridden byte, 0 = as built. 0..39: a byte of the page parameters;
// 40 / 41: rotate the back of a long-edge / short-edge sheet by 180 degrees (value 0 or 1)
static uint16_t s_override[48];

void canon_capt_set_override(int offset, int value)
{
    if (value < 0)
        memset(s_override, 0, sizeof(s_override));
    else if (offset >= 0 && offset < 48)
        s_override[offset] = (uint16_t)(0x100 | (value & 0xFF));
}

static bool back_rotated(int duplex)
{
    int idx = duplex == 2 ? 41 : 40;
    if (s_override[idx])
        return (s_override[idx] & 0xFF) != 0;
    return duplex == 2 ? BACK_ROTATE_SHORT : BACK_ROTATE_LONG;
}

// The printer's own count of pages in progress, polled until it reaches want.
static int wait_counter(capt_job_t *j, int what, unsigned want, int max_s, uint32_t poll_ms)
{
    for (int i = 0; i * (int)poll_ms < max_s * 1000; i++) {
        STEP(j, capt_get_xstatus(&j->c), "status");
        unsigned have = what == 0 ? j->c.st.page_decoding : what == 1 ? j->c.st.page_printing : j->c.st.page_completed;
        if (have >= want)
            return 0;
        STEP(j, delay_ms(j, poll_ms), "wait");
    }
    return fail(j, -2, "page counter");
}

// Starts a page (a side) at the printer: the paper source and the page parameters. flag: 0 for a
// one-sided page, FLAG_FRONT / FLAG_BACK for the sides of a two-sided sheet.
static int page_prologue(capt_job_t *j, uint8_t flag)
{
    capt_t *c = &j->c;
    const unsigned page = (unsigned)j->pages + 1;
    j->cur_flag = flag;
    uint8_t parms[40] = {
        0x00, 0x00, j->model_key[0], j->model_key[1], (uint8_t)j->paper_sz, flag,
        0x00 /* paper source: patched below */, 0x00,
        0x00, 0x00, 0x00, 0x00,     // toner density
        0x00 /* media: plain */, 0x11 /* 600 dpi */, 0x04, 0x00,
        0x01, 0x01, 0x02 /* automatic image refinement */, 0x00 /* toner save */, 0x00, 0x00,
        (uint8_t)(MARGIN_PX & 0xFF), (uint8_t)(MARGIN_PX >> 8),     // top margin
        (uint8_t)(MARGIN_PX & 0xFF), (uint8_t)(MARGIN_PX >> 8),     // left margin
        (uint8_t)(j->line_bytes & 0xFF), (uint8_t)(j->line_bytes >> 8),
        (uint8_t)(j->lines & 0xFF), (uint8_t)(j->lines >> 8),
        (uint8_t)(j->page_w & 0xFF), (uint8_t)(j->page_w >> 8),
        (uint8_t)(j->page_h & 0xFF), (uint8_t)(j->page_h >> 8),
        0x00, 0x00, MEDIA_PLAIN /* fuser mode */, 0x00, 0x00, 0x00,
    };
    uint8_t hp[8];

    // the printer is a few pages ahead of the host at most
    if (page > 2)
        STEP(j, wait_counter(j, 1, page - 2, DECODE_MAX_S, 200), "engine");

    // select the paper source: size code, media, the page size at 300 dpi, source, raster direction flag
    // (E1A9's payload is [size, media, width/2 LE16, height/2 LE16, source, flag]: 02 01 B0 09 B3 0D 01 00
    // for A4; the page height there is 7014 / 2 = 3507)
    const uint8_t sel[8] = {parms[4], parms[36], (uint8_t)((j->page_w / 2) & 0xFF), (uint8_t)((j->page_w / 2) >> 8),
                            (uint8_t)((j->page_h / 2) & 0xFF), (uint8_t)((j->page_h / 2) >> 8),
                            SOURCE_CASSETTE_1, (uint8_t)(parms[5] & 8)};
    uint8_t reply[4] = {0};
    size_t rn = sizeof(reply);
    STEP(j, capt_sendrecv(c, 0xE1A9, sel, sizeof(sel), reply, &rn), "paper source");
    if (rn < 2 || reply[0] != 0x00 || reply[1] != SOURCE_CASSETTE_1) {
        capt_hex("paper source not available, reply", reply, rn);
        return fail(j, -1, "cassette 1");
    }
    parms[6] = reply[1];

    for (unsigned i = 0; i < sizeof(parms); i++)
        if (s_override[i]) {
            LOGW("page parameter %u: %02X replaced by %02X", i, parms[i], s_override[i] & 0xFF);
            parms[i] = (uint8_t)s_override[i];
        }

    size_t hn = hiscoa_format_params(hp, sizeof(hp));
    capt_multi_begin(c, CAPT_SET_PARMS);
    capt_multi_add(c, CAPT_SET_PARM_PAGE, parms, sizeof(parms));
    capt_multi_add(c, CAPT_SET_PARM_HISCOA, hp, hn);
    capt_multi_add(c, CAPT_SET_PARM_1, NULL, 0);
    capt_multi_add(c, CAPT_SET_PARM_2, NULL, 0);
    STEP(j, capt_multi_send(c), "page parameters");
    j->packets = 0;
    return 0;
}

// The page's data is sent: wait until the printer has decoded it, then start it.
// The next page can follow at once; the end of the job waits for them all.
static int page_epilogue(capt_job_t *j)
{
    capt_t *c = &j->c;
    const uint16_t page = (uint16_t)(j->pages + 1);

    STEP(j, capt_send(c, CAPT_PRINT_DATA_END, NULL, 0), "end of data");
    STEP(j, wait_counter(j, 0, page, DECODE_MAX_S, 500), "decode");
    STEP(j, job_info(j, 2, page), "job info");
    uint8_t fire[2] = {(uint8_t)(page & 0xFF), (uint8_t)(page >> 8)};
    STEP(j, capt_sendrecv(c, CAPT_FIRE, fire, 2, NULL, NULL), "FIRE");
    STEP(j, job_info(j, 6, page), "job info");
    j->pages++;
    if (j->cur_flag != FLAG_BACK)
        j->sheets++;
    return 0;
}

static int job_epilogue(capt_job_t *j)
{
    if (j->sheets)     // the printer's "completed" counts sheets
        STEP(j, wait_counter(j, 2, (unsigned)j->sheets, PRINT_MAX_S, 1000), "printing");
    STEP(j, job_info(j, 4, (uint16_t)j->pages), "job end info");
    uint8_t jb[2] = {(uint8_t)(j->job_id & 0xFF), (uint8_t)(j->job_id >> 8)};
    STEP(j, capt_sendrecv(&j->c, CAPT_JOB_END, jb, 2, NULL, NULL), "JOB_END");
    return 0;
}

// After an error or a cancel before a page was fired: the printer discards what
// it has and the job is closed (the sequence the Canon driver uses). Each step is
// tried whatever happened to the one before.
static void cancel_cleanup(capt_job_t *j)
{
    capt_t *c = &j->c;
    j->err = 0;
    go_offline(j);
    capt_sendrecv(c, CAPT_START_1, NULL, 0, NULL, NULL);        // clear misprint
    capt_sendrecv(c, CAPT_START_2, NULL, 0, NULL, NULL);        // clear error
    capt_sendrecv(c, CAPT_START_3, NULL, 0, NULL, NULL);        // discard data
    go_online(j, 0);
    go_offline(j);
    job_info(j, 4, 0);
    uint8_t jb[2] = {(uint8_t)(j->job_id & 0xFF), (uint8_t)(j->job_id >> 8)};
    capt_sendrecv(c, CAPT_JOB_END, jb, 2, NULL, NULL);
}

// ---------------------------------------------------------------------------
// bands

static int send_band(capt_job_t *j, const uint8_t *data, size_t size)
{
    while (size) {
        size_t n = size > PACKET_MAX ? PACKET_MAX : size;
        if (++j->packets % 16 == 0) {
            // flow control: don't send while the printer says its buffer is full
            for (int i = 0;; i++) {
                STEP(j, capt_get_xstatus(&j->c), "status");
                if (!CAPT_FLAG(&j->c.st, CAPT_FL_BUFFERFULL))
                    break;
                if (i >= BUFFER_MAX_S)
                    return fail(j, -2, "printer buffer full");
                STEP(j, delay_ms(j, 1000), "wait");
            }
        }
        STEP(j, capt_send(&j->c, CAPT_PRINT_DATA, data, n), "print data");
        data += n;
        size -= n;
    }
    return 0;
}

static int flush_band(capt_job_t *j)
{
    if (!j->band_fill)
        return 0;
    size_t n = hiscoa_compress_band(j->comp, j->comp_size, j->band, j->line_bytes, j->band_fill, 0);
    if (!n)
        return fail(j, -1, "compression");
    j->band_fill = 0;
    if (j->side == SIDE_FRONT) {            // held until its back has been sent
        if (j->held_len + 4 + n > j->held_cap)
            return fail(j, -3, "front side too big to hold");
        uint8_t *q = j->big + j->held_len;
        q[0] = (uint8_t)n, q[1] = (uint8_t)(n >> 8), q[2] = (uint8_t)(n >> 16), q[3] = (uint8_t)(n >> 24);
        memcpy(q + 4, j->comp, n);
        j->held_len += 4 + n;
        return 0;
    }
    return send_band(j, j->comp, n);
}

// one row of the image
static int put_row(capt_job_t *j, const uint8_t *bits)
{
    if (j->rows >= j->lines)
        return 0;                   // below the image
    if (j->side == SIDE_BACK && j->rotate) {
        memcpy(j->raw + (size_t)j->rows * j->line_bytes, bits, j->line_bytes);
        j->rows++;
        return 0;
    }
    memcpy(j->band + (size_t)j->band_fill * j->line_bytes, bits, j->line_bytes);
    j->rows++;
    if (++j->band_fill == BAND_LINES)
        return flush_band(j);
    return 0;
}

// the held front, as a page of its own
static int print_held(capt_job_t *j, uint8_t flag)
{
    STEP(j, page_prologue(j, flag), "front side");
    for (size_t off = 0; off < j->held_len;) {
        const uint8_t *q = j->big + off;
        size_t n = (size_t)q[0] | (size_t)q[1] << 8 | (size_t)q[2] << 16 | (size_t)q[3] << 24;
        STEP(j, send_band(j, q + 4, n), "front side data");
        off += 4 + n;
    }
    STEP(j, page_epilogue(j), "front side end");
    j->held = false;
    j->held_len = 0;
    return 0;
}

// a rotated back: the rows of the raw image in reverse order, each turned, compressed and sent
static int send_rotated_back(capt_job_t *j)
{
    for (unsigned r0 = 0; r0 < j->lines; r0 += BAND_LINES) {
        unsigned nl = j->lines - r0 < BAND_LINES ? j->lines - r0 : BAND_LINES;
        for (unsigned k = 0; k < nl; k++)
            rotate_row(j, j->raw + (size_t)(j->lines - 1 - (r0 + k)) * j->line_bytes, j->band + (size_t)k * j->line_bytes);
        size_t n = hiscoa_compress_band(j->comp, j->comp_size, j->band, j->line_bytes, nl, 0);
        if (!n)
            return fail(j, -1, "compression");
        STEP(j, send_band(j, j->comp, n), "back side data");
        if (j->io.delay)
            delay_ms(j, 0);         // yields (and notices a cancel)
    }
    return 0;
}

// ---------------------------------------------------------------------------
// the driver interface

static void *job_begin(const printer_driver_t *d, const printer_io_t *io, int dpi, int paper)
{
    (void)d;
    if ((dpi != 300 && dpi != 600) || paper < 0 || paper >= (int)(sizeof(GEOM) / sizeof(GEOM[0])))
        return NULL;
    init_tables();
    capt_job_t *j = calloc(1, sizeof(*j));
    if (!j)
        return NULL;
    j->io = *io;
    capt_init(&j->c, &j->io);
    j->model_key[0] = 0x94;
    j->model_key[1] = 0x2A;
    j->dpi = dpi;
    j->scale = 600 / dpi;
    j->page_w = GEOM[paper].px_w;
    j->page_h = GEOM[paper].px_h;
    j->line_bytes = GEOM[paper].line_bytes;
    j->lines = GEOM[paper].lines;
    j->paper_sz = GEOM[paper].sz;
    j->x0 = MARGIN_PX / (unsigned)j->scale;     // source pixels
    j->comp_size = 2 * (size_t)j->line_bytes * BAND_LINES;
    j->band = calloc(BAND_LINES, j->line_bytes);
    j->comp = malloc(j->comp_size);
    j->row = calloc(1, j->line_bytes);
    if (!j->band || !j->comp || !j->row) {
        free(j->band);
        free(j->comp);
        free(j->row);
        free(j);
        return NULL;
    }
    job_prologue(j);
    return j;
}

static void page_options(void *job, int duplex)
{
    ((capt_job_t *)job)->duplex = duplex;
}

static int page_begin(void *job, int src_w, int src_h, int bytes_pp)
{
    capt_job_t *j = job;
    if (j->err)
        return -1;
    j->src_w = src_w;
    j->src_h = src_h;
    j->bpp = bytes_pp;
    j->abs_row = 0;
    j->rows = 0;
    j->band_fill = 0;
    j->packets = 0;
    int mode = j->duplex;

    if (mode && !j->big && j->io.big_buffer)
        j->big = j->io.big_buffer(j->io.ctx, &j->big_size);
    if (mode && !j->big) {
        LOGW("no buffer for two-sided printing: this page is printed one-sided");
        mode = 0;
    }

    if (mode == 0) {
        // a front still held (a one-sided page after a two-sided one): it goes first, one-sided
        if (j->held && print_held(j, 0) < 0)
            return -1;
        j->side = SIDE_ONE;
        j->rotate = false;
        if (page_prologue(j, 0) < 0)
            return -1;
    } else if (!j->held) {
        j->side = SIDE_FRONT;                   // nothing goes to the printer until the back is here
        j->rotate = back_rotated(mode);
        j->held_len = 0;
        j->held_cap = j->big_size - (j->rotate ? (size_t)j->line_bytes * j->lines : 0);
    } else {
        j->side = SIDE_BACK;
        j->rotate = back_rotated(mode);
        if (j->rotate) {
            const size_t raw_size = (size_t)j->line_bytes * j->lines;
            if (j->big_size < raw_size + j->held_len)
                return fail(j, -3, "no room to turn the back side");
            j->raw = j->big + j->big_size - raw_size;
        } else if (page_prologue(j, FLAG_BACK) < 0) {
            return -1;
        }
    }
    j->in_page = true;
    return 0;
}

static int page_line(void *job, int y, const uint8_t *px)
{
    capt_job_t *j = job;
    if (j->err)
        return -1;
    // rows above the image are not rendered
    if (j->abs_row + (unsigned)j->scale <= MARGIN_PX) {
        j->abs_row += (unsigned)j->scale;
        return 0;
    }
    render_window(j, px, (unsigned)y, j->row);
    for (int k = 0; k < j->scale; k++)
        if (j->abs_row++ >= MARGIN_PX && put_row(j, j->row) < 0)
            return -1;
    return 0;
}

// the rest of the image is white; then the side goes where it belongs
static int finish_page(capt_job_t *j)
{
    memset(j->row, 0, j->line_bytes);
    while (j->rows < j->lines && !j->err)
        put_row(j, j->row);
    j->in_page = false;
    if (j->err)
        return -1;

    switch (j->side) {
    case SIDE_ONE:
        if (flush_band(j) < 0 || page_epilogue(j) < 0)
            return -1;
        break;
    case SIDE_FRONT:
        if (flush_band(j) < 0)
            return -1;
        j->held = true;
        break;
    case SIDE_BACK:
        if (j->rotate) {
            if (page_prologue(j, FLAG_BACK) < 0 || send_rotated_back(j) < 0)
                return -1;
        } else if (flush_band(j) < 0) {
            return -1;
        }
        if (page_epilogue(j) < 0 || print_held(j, FLAG_FRONT) < 0)
            return -1;
        break;
    }
    return 0;
}

static int page_end(void *job)
{
    return finish_page(job);
}

static int job_end(void *job)
{
    capt_job_t *j = job;
    // a cancelled page is finished with white lines (the sheet is ejected)
    if (j->in_page && !j->err)
        finish_page(j);
    // a front without a back (an odd number of sides) is a one-sided page
    if (j->held && !j->err)
        print_held(j, 0);
    int ret = 0;
    if (j->err) {
        if (j->started)
            cancel_cleanup(j);
        ret = -1;
    } else if (j->started) {
        if (job_epilogue(j) < 0) {
            if (j->started)
                cancel_cleanup(j);
            ret = -1;
        }
    }
    free(j->band);
    free(j->comp);
    free(j->row);
    free(j);
    return ret;
}

// ---------------------------------------------------------------------------
// test page: frame at the edges of the image, diagonals, a gray ramp and line
// pairs, so the margins, the scaling and the halftone show

static int test_page(const printer_driver_t *d, const printer_io_t *io, int dpi, const char *label)
{
    void *job = job_begin(d, io, dpi, 0);
    if (!job)
        return -1;
    capt_job_t *j = job;
    const int w = (int)j->page_w / j->scale, h = (int)j->page_h / j->scale;
    const int win0 = (int)j->x0, win1 = win0 + (int)j->line_bytes * 8 / j->scale;    // window in source pixels
    const int top = MARGIN_PX / j->scale, bot = top + (int)j->lines / j->scale;        // image rows
    const int u = j->scale == 2 ? 1 : 2;                                              // 1 unit = 1/300 inch
    uint8_t *line = malloc((size_t)w);
    int ret = -1;
    if (line && page_begin(j, w, h, 1) == 0) {
        for (int y = 0; y < h; y++) {
            memset(line, 255, (size_t)w);
            const int iy = y - top, ih = bot - top;
            if (iy >= 0 && iy < ih) {
                int in_frame = iy >= 8 * u && iy < 12 * u;
                int in_frame_b = iy >= ih - 12 * u && iy < ih - 8 * u;
                for (int x = win0; x < win1; x++) {
                    int v = 255;
                    if (in_frame || in_frame_b || (x >= win0 + 8 * u && x < win0 + 12 * u && iy > 8 * u && iy < ih - 8 * u) ||
                        (x >= win1 - 12 * u && x < win1 - 8 * u && iy > 8 * u && iy < ih - 8 * u))
                        v = 0;
                    // diagonals of the image
                    long dx = x - win0, ww = win1 - win0;
                    long d1 = (long)iy * ww / ih - dx, d2 = (long)iy * ww / ih - (ww - dx);
                    if (labs(d1) < 2 * u || labs(d2) < 2 * u)
                        v = 0;
                    // gray ramp, 16 steps
                    if (iy > ih / 10 && iy < ih / 10 + 40 * u && x > win0 + 30 * u && x < win1 - 30 * u) {
                        int step = (x - win0 - 30 * u) * 16 / (win1 - win0 - 60 * u);
                        v = step * 255 / 15;
                    }
                    // line pairs 1..4 px wide
                    if (iy > ih / 5 && iy < ih / 5 + 30 * u && x > win0 + 30 * u && x < win0 + 30 * u + 400) {
                        int k = (x - win0 - 30 * u) / 100 + 1;
                        v = (((x - win0 - 30 * u) % 100) / k) % 2 ? 255 : 0;
                    }
                    line[x] = (uint8_t)v;
                }
            }
            if (page_line(j, y, line) < 0)
                break;
            if ((y & 63) == 63 && io->progress)
                io->progress(io->ctx, y + 1, h);
        }
        ret = page_end(j);
    }
    (void)label;
    free(line);
    if (job_end(job) < 0)
        ret = -1;
    return ret;
}

// ---------------------------------------------------------------------------

// USB ids as seen on a real LBP3300: 04a9:267e "Canon CAPT USB Device", IEEE 1284 ID
// "MFG:Canon;MDL:LBP3300;CMD:CAPT;VER:2.1;CLS:PRINTER;DES:Canon LBP3300"
const printer_driver_t canon_lbp3300_driver = {
    .id = "canon-lbp3300",
    .make_model = "Canon LBP3300",
    .usb_mfg = "Canon",
    .usb_mdl = "LBP3300",
    .usb_vid = 0x04a9,
    .usb_pid = 0x267e,
    .color = false,
    .scanner = false,
    .duplex = true,
    .raw_io = true,
    .resolutions = {300, 600},
    .nresolutions = 2,
    .papers = PAPERS,
    .npapers = 2,
    .margin_left = 500,
    .margin_right = 500,
    .margin_top = 500,
    .margin_bottom = 500,
    .job_begin = job_begin,
    .page_options = page_options,
    .page_begin = page_begin,
    .page_line = page_line,
    .page_end = page_end,
    .job_end = job_end,
    .test_page = test_page,
};
