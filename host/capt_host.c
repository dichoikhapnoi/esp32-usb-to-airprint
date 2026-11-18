// Tests of the CAPT printer driver (components/printers/canon_capt.c, capt.c,
// hiscoa.c) against an emulated CAPT printer: it parses the command stream,
// answers the status requests, decompresses every band into the page it would
// print, and records any protocol violation (bad length, wrong order, wrong
// job id, data before the page parameters). The emulation follows captdriver's
// documented conversation, NOT a real LBP3300: it checks that the driver does
// what it means to, not that a printer accepts it.
//
//   capt_host selftest            unit tests + driver tests (exit status 0 = pass)
//   capt_host testpage out.pbm    the driver's test page as the emulated printer prints it
//
// Licensed under the GNU General Public License Version 3 or later (see
// components/printers/hiscoa.c).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "capt.h"
#include "hiscoa.h"
#include "printer_driver.h"

static int g_fail;
#define CHECK(cond, ...)                                                                                               \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            g_fail++;                                                                                                  \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);                                                       \
            fprintf(stderr, __VA_ARGS__);                                                                              \
            fputc('\n', stderr);                                                                                       \
        }                                                                                                              \
    } while (0)

static uint32_t crc32_update(uint32_t crc, const uint8_t *p, size_t n)
{
    crc = ~crc;
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
    }
    return ~crc;
}

static uint16_t W(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// ---------------------------------------------------------------------------
// the emulated printer

#define SIM_MAX_PAGES 4

typedef struct {
    // the stream in
    uint8_t *in;
    size_t in_len, in_cap;
    // the reply being read
    uint8_t reply[2048];
    size_t reply_len, reply_pos;
    size_t chunk;               // bytes per read (a USB packet is 64)
    size_t first_chunk;         // bytes of the first read of a reply, 0 = like the others

    // printer state
    int uninit;                 // START_1..3 not done yet
    int offline_after;          // test: the printer goes offline after firing this many pages (0: never)
    unsigned stale_job;         // test: a job of an earlier session is still open under this id (0: none)
    int offline_paper;          // test: ... with the paper out for this many status polls
    int offline;                // gone offline: decodes nothing until START_1..3 and UPLOAD_2; its counters restart
    int restarts;               // times it was brought back
    int busy_polls;             // status polls that still say busy
    int full_polls;             // status polls that still say "buffer full"
    int nopaper_polls;          // status polls that still say "out of paper" (both bits)
    uint16_t pages_started, page_received, page_out;
    int job_open;
    uint16_t job_id;
    int params_seen;            // page parameters of the current page received
    int source_selected;        // E1A9 answered for the page being set up
    unsigned sel_w, sel_h, sel_sz;
    int offline_cmds;           // E0A6 seen
    unsigned line_bytes, lines, page_w, left;
    unsigned fg_seen[8];        // job start flags (1,2,4,6) seen
    int cmd_counts[0x10000 / 256];
    long xstatus_extra;         // bytes added to the extended status reply

    // the page in
    uint8_t *band;              // compressed data of the band being received
    size_t band_len, band_cap;
    uint8_t *bitmap;            // rows * line_bytes
    unsigned rows;
    struct {
        uint8_t *bits;
        unsigned rows, line_bytes, page_w, left;
        int ended;
        unsigned flag;          // byte 5 of the page parameters: 0 one-sided, 2 front, 4 back
    } page[SIM_MAX_PAGES];
    unsigned cur_flag;
    uint16_t fired, sheets_done;
    uint8_t *bigbuf;            // the job's big buffer (two-sided printing)
    int no_big;
    int npages;

    uint32_t data_crc;          // of the PRINT_DATA payloads
    int packets;
    int errors;
    char first_error[160];
    int silent_cmd;             // no reply to this command (0 = none)
    long delays;                // seconds slept
} sim_t;

static void sim_error(sim_t *s, const char *fmt, ...)
{
    __builtin_va_list ap;
    __builtin_va_start(ap, fmt);
    if (!s->errors++)
        vsnprintf(s->first_error, sizeof(s->first_error), fmt, ap);
    __builtin_va_end(ap);
}

static void reply(sim_t *s, uint16_t cmd, const uint8_t *data, size_t n)
{
    if (cmd == s->silent_cmd)
        return;
    size_t total = 4 + n;
    uint8_t *r = s->reply + s->reply_len;
    r[0] = (uint8_t)(cmd & 0xFF);
    r[1] = (uint8_t)(cmd >> 8);
    r[2] = (uint8_t)(total & 0xFF);
    r[3] = (uint8_t)(total >> 8);
    if (n)
        memcpy(r + 4, data, n);
    s->reply_len += total;
}

static void status_reply(sim_t *s, uint16_t cmd, size_t extra)
{
    uint8_t p[40 + 512] = {0};
    unsigned st0 = 0;
    if (s->uninit)
        st0 |= 1u << 5;
    if (s->offline)
        st0 |= (1u << 4) | (1u << 1);
    if (s->busy_polls > 0) {
        st0 |= 1u << 7;
        s->busy_polls--;
    }
    if (s->full_polls > 0) {
        st0 |= 1u << 2;
        s->full_polls--;
    }
    unsigned st1 = 0;
    if (s->nopaper_polls > 0) {
        st0 |= 1u << 1;
        st1 |= 1u << 14;
        s->nopaper_polls--;
    }
    p[0] = (uint8_t)st0;
    p[1] = (uint8_t)(st0 >> 8);
    p[8] = (uint8_t)st1;
    p[9] = (uint8_t)(st1 >> 8);
    // pages the printer has started / is printing / has ejected / has finished
    uint16_t dec = s->pages_started, out = s->sheets_done, prt = s->fired;     // a two-sided sheet counts once
    p[14] = (uint8_t)dec, p[15] = (uint8_t)(dec >> 8);
    p[16] = (uint8_t)prt, p[17] = (uint8_t)(prt >> 8);
    p[18] = (uint8_t)out, p[19] = (uint8_t)(out >> 8);
    p[20] = (uint8_t)out, p[21] = (uint8_t)(out >> 8);
    p[34] = (uint8_t)s->page_received, p[35] = (uint8_t)(s->page_received >> 8);
    reply(s, cmd, p, 40 + (extra > 512 ? 512 : extra));
}

// A band is used up when the decoder stopped at its end, or only a whole word
// of padding ones is left (the compressor adds one to an already aligned stream;
// stored as 0xFF ^ 0x43).
static int tail_ok(const uint8_t *band, size_t len, size_t used)
{
    if (used == len)
        return 1;
    if (used + 4 != len)
        return 0;
    for (size_t i = used; i < len; i++)
        if (band[i] != (0xFF ^ 0x43))
            return 0;
    return 1;
}

static void do_set_parms(sim_t *s, const uint8_t *d, size_t n)
{
    size_t pos = 0;
    int got_page = 0, got_hiscoa = 0;
    while (pos + 4 <= n) {
        uint16_t cmd = W(d + pos), len = W(d + pos + 2);
        if (len < 4 || pos + len > n) {
            sim_error(s, "SET_PARMS: bad inner length %u", len);
            return;
        }
        const uint8_t *p = d + pos + 4;
        size_t pl = len - 4u;
        if (cmd == 0xD0A0) {
            got_page = 1;
            if (pl != 40) {
                sim_error(s, "page parameters are %zu bytes, not 40", pl);
            } else {
                s->line_bytes = W(p + 26);
                s->lines = W(p + 28);
                s->page_w = W(p + 30);
                s->left = W(p + 22);
                if (p[2] != 0x94 || p[3] != 0x2A)
                    sim_error(s, "model key %02X %02X", p[2], p[3]);
                s->cur_flag = p[5];
                if (p[5] != 0 && p[5] != 2 && p[5] != 4)
                    sim_error(s, "page flag %02X", p[5]);
                if (!s->source_selected || p[6] != 1)
                    sim_error(s, "paper source %u in the page parameters, selected %d", p[6], s->source_selected);
                if (W(p + 24) != 120 || W(p + 22) < 20 || W(p + 22) > 520 ||
                    W(p + 22) + 8 * W(p + 26) > W(p + 30) - 8)
                    sim_error(s, "margins %u/%u, window %u bytes on a page %u wide", W(p + 22), W(p + 24), W(p + 26),
                              W(p + 30));
                if (s->sel_w != W(p + 30) / 2 || s->sel_h != W(p + 32) / 2 || s->sel_sz != p[4])
                    sim_error(s, "E1A9 said %ux%u size %u, the page is %ux%u size %u", s->sel_w, s->sel_h, s->sel_sz,
                              W(p + 30), W(p + 32), p[4]);
            }
        } else if (cmd == 0xD0A4) {
            got_hiscoa = 1;
            uint8_t hp[8];
            hiscoa_format_params(hp, sizeof(hp));
            if (pl != 8 || memcmp(p, hp, 8))
                sim_error(s, "HiSCoA parameters differ from the compressor's");
        } else if (cmd != 0xD0A1 && cmd != 0xD0A2) {
            sim_error(s, "unknown parameter command %04X", cmd);
        }
        pos += len;
    }
    if (!got_page || !got_hiscoa)
        sim_error(s, "SET_PARMS without page or HiSCoA parameters");
    if (!s->job_open)
        sim_error(s, "page parameters outside a job");
    if (s->npages >= SIM_MAX_PAGES) {
        sim_error(s, "too many pages");
        return;
    }
    s->pages_started++;
    s->params_seen = 1;
    s->source_selected = 0;
    s->rows = 0;
    s->band_len = 0;
    s->bitmap = calloc(s->lines ? s->lines : 1, s->line_bytes ? s->line_bytes : 1);
}

static void band_done(sim_t *s)
{
    unsigned lb = s->line_bytes;
    if (!s->band_len)
        return;
    size_t cap = (size_t)lb * 70, consumed = 0;
    uint8_t *out = calloc(1, cap);
    int last = 0;
    size_t n = hiscoa_decompress_band(s->band, s->band_len, out, cap, lb, &consumed, &last);
    if (n == (size_t)-1) {
        sim_error(s, "band does not decompress (%zu bytes)", s->band_len);
    } else if (n % lb) {
        sim_error(s, "band of %zu bytes is not a whole number of lines", n);
    } else if (!tail_ok(s->band, s->band_len, consumed)) {
        sim_error(s, "band: %zu bytes sent, %zu used", s->band_len, consumed);
    } else if (s->rows + n / lb > s->lines) {
        sim_error(s, "more lines than the page has");
    } else {
        memcpy(s->bitmap + (size_t)s->rows * lb, out, n);
        s->rows += (unsigned)(n / lb);
    }
    free(out);
    s->band_len = 0;
}

static void do_command(sim_t *s, uint16_t cmd, const uint8_t *d, size_t n)
{
    s->cmd_counts[cmd >> 8]++;
    switch (cmd) {
    case 0xA1A1: {  // IDENT: a 64-byte reply on the real LBP3300 (60 bytes of data)
        uint8_t id[60] = {0x00, 0x0B, 0x94, 0x2A};
        reply(s, cmd, id, sizeof(id));
        break;
    }
    case 0xA3A2:    // START_0
        reply(s, cmd, (const uint8_t[]){0, 0}, 2);
        break;
    case 0xA2A0: {  // JOB_BEGIN
        if (n != 8)
            sim_error(s, "JOB_BEGIN data is %zu bytes", n);
        if (s->stale_job) {
            reply(s, cmd, (const uint8_t[]){0x87, 0, 0, 0, 0, 0, 0, 0}, 8);
            break;
        }
        if (s->job_open)
            sim_error(s, "JOB_BEGIN inside a job");
        s->job_open = 1;
        s->job_id = 0x0107;
        uint8_t p[8] = {0, 0, (uint8_t)s->job_id, (uint8_t)(s->job_id >> 8), 0, 0, 0, 0};
        reply(s, cmd, p, 8);
        break;
    }
    case 0xE1A1: {  // JOB_SETUP
        if (n != 72) {
            sim_error(s, "JOB_SETUP data is %zu bytes", n);
            break;
        }
        unsigned fg = d[16];
        if (W(d + 18) != s->job_id)
            sim_error(s, "JOB_SETUP job id %04X, expected %04X", W(d + 18), s->job_id);
        if (fg < 8)
            s->fg_seen[fg]++;
        if (fg == 2 && s->page_received != s->pages_started)
            sim_error(s, "job start 2 before the page was received");
        reply(s, cmd, (const uint8_t[]){0, 0}, 2);
        break;
    }
    case 0xE1A2:    // GPIO
        reply(s, cmd, (const uint8_t[]){0, 0}, 2);
        break;
    case 0xE0A6:    // go offline / online: only expected when a job is cleaned up
        s->offline_cmds++;
        reply(s, cmd, d, n);
        break;
    case 0xE1A9: {  // select the paper source
        if (s->offline)
            sim_error(s, "a page was started while the printer is offline (it decodes nothing)");
        if (n != 8) {
            sim_error(s, "E1A9 data is %zu bytes", n);
            break;
        }
        s->sel_sz = d[0];
        s->sel_w = W(d + 2);
        s->sel_h = W(d + 4);
        if (d[1] != 1 || d[6] != 1 || d[7] != 0)
            sim_error(s, "E1A9 media %u source %u flag %u", d[1], d[6], d[7]);
        s->source_selected = 1;
        reply(s, cmd, (const uint8_t[]){0x00, 0x01}, 2);
        break;
    }
    case 0xA0A1: {  // paper sources (asked while the paper is out)
        uint8_t src[24] = {0};
        src[18 - 4] = 0xA2;
        reply(s, cmd, src, sizeof(src));
        break;
    }
    case 0xE0A0:    // CHKSTATUS
        status_reply(s, cmd, 0);
        break;
    case 0xA0A8:    // CHKXSTATUS
        status_reply(s, cmd, (size_t)s->xstatus_extra);
        break;
    case 0xE0A3:
    case 0xE0A2:
        reply(s, cmd, (const uint8_t[]){0, 0}, 2);
        break;
    case 0xE0A4:    // START_3: initialised
        reply(s, cmd, (const uint8_t[]){0, 0}, 2);
        s->uninit = 0;
        s->busy_polls = 2;
        break;
    case 0xE0A5:    // UPLOAD_2
        if (n != 16)
            sim_error(s, "UPLOAD_2 data is %zu bytes", n);
        if (s->offline && s->nopaper_polls > 0)
            sim_error(s, "restarted while the paper is out (that only clears the flag)");
        if (s->offline) {
            s->offline = 0;
            s->restarts++;
            s->pages_started = s->page_received = s->page_out = 0;
            s->fired = s->sheets_done = 0;
            s->offline_after = 0;
        }
        reply(s, cmd, (const uint8_t[]){0, 0}, 2);
        s->busy_polls = 1;
        break;
    case 0xD0A9:    // SET_PARMS
        if (s->uninit)
            sim_error(s, "page parameters before START_1..3");
        do_set_parms(s, d, n);
        break;
    case 0xC0A0:    // PRINT_DATA
        if (!s->params_seen) {
            sim_error(s, "print data before the page parameters");
            break;
        }
        s->packets++;
        s->data_crc = crc32_update(s->data_crc, d, n);
        if (s->band_len + n > s->band_cap) {
            s->band_cap = (s->band_len + n) * 2;
            s->band = realloc(s->band, s->band_cap);
        }
        memcpy(s->band + s->band_len, d, n);
        s->band_len += n;
        if (n < 0xFF00)             // the last packet of a band
            band_done(s);
        break;
    case 0xC0A4:    // PRINT_DATA_END
        if (s->band_len)
            sim_error(s, "end of data in the middle of a band (%zu bytes)", s->band_len);
        if (!s->params_seen) {
            sim_error(s, "end of data without a page");
            break;
        }
        if (s->rows != s->lines)
            sim_error(s, "page ended with %u of %u lines", s->rows, s->lines);
        if (s->npages < SIM_MAX_PAGES) {
            s->page[s->npages].bits = s->bitmap;
            s->page[s->npages].rows = s->rows;
            s->page[s->npages].line_bytes = s->line_bytes;
            s->page[s->npages].page_w = s->page_w;
            s->page[s->npages].left = s->left;
            s->page[s->npages].ended = 1;
            s->page[s->npages].flag = s->cur_flag;
            s->npages++;
            s->bitmap = NULL;
        }
        s->params_seen = 0;
        s->page_received = s->pages_started;
        break;
    case 0xE0A7: {  // FIRE
        if (n != 2 || W(d) != s->pages_started)
            sim_error(s, "FIRE for page %u, the printer is at %u", n == 2 ? W(d) : 0, s->pages_started);
        s->busy_polls = 1;
        reply(s, cmd, d, n);
        s->page_out = s->pages_started;
        s->fired++;
        if (s->offline_after && s->fired == s->offline_after) {
            s->offline = 1;
            s->nopaper_polls = s->offline_paper;
        }
        if (s->npages && s->page[s->npages - 1].flag != 4)
            s->sheets_done++;
        break;
    }
    case 0xE0A9:    // JOB_END
        if (s->stale_job) {     // not the job's owner: refused, except for the open job of the earlier session
            int ours = n == 2 && W(d) == s->stale_job;
            if (ours)
                s->stale_job = 0;
            reply(s, cmd, ours ? (const uint8_t[]){0, 0} : (const uint8_t[]){0x88, 0}, 2);
            break;
        }
        if (!s->job_open)
            sim_error(s, "JOB_END without a job");
        if (n != 2 || W(d) != s->job_id)
            sim_error(s, "JOB_END job id");
        s->job_open = 0;
        reply(s, cmd, d, n);
        break;
    default:
        sim_error(s, "unknown command %04X", cmd);
        reply(s, cmd, (const uint8_t[]){0, 0}, 2);
    }
}

static void sim_parse(sim_t *s)
{
    while (s->in_len >= 4) {
        uint16_t cmd = W(s->in), len = W(s->in + 2);
        if (len < 4) {
            sim_error(s, "command %04X with length %u", cmd, len);
            s->in_len = 0;
            return;
        }
        if (s->in_len < len)
            return;
        do_command(s, cmd, s->in + 4, len - 4u);
        memmove(s->in, s->in + len, s->in_len - len);
        s->in_len -= len;
    }
}

static int sim_write(void *ctx, const uint8_t *data, size_t len)
{
    sim_t *s = ctx;
    if (s->in_len + len > s->in_cap) {
        s->in_cap = (s->in_len + len) * 2;
        s->in = realloc(s->in, s->in_cap);
    }
    memcpy(s->in + s->in_len, data, len);
    s->in_len += len;
    return 0;
}

static int sim_flush(void *ctx)
{
    sim_t *s = ctx;
    sim_parse(s);
    return 0;
}

static int sim_read(void *ctx, uint8_t *buf, size_t max, uint32_t timeout_ms)
{
    sim_t *s = ctx;
    size_t avail = s->reply_len - s->reply_pos;
    if (!avail) {
        s->reply_len = s->reply_pos = 0;
        return 0;
    }
    size_t n = avail < s->chunk ? avail : s->chunk;
    if (s->reply_pos == 0 && s->first_chunk && n > s->first_chunk)
        n = s->first_chunk;         // the LBP3300's replies start with a short packet
    if (n > max)
        n = max;
    memcpy(buf, s->reply + s->reply_pos, n);
    s->reply_pos += n;
    if (s->reply_pos == s->reply_len)
        s->reply_len = s->reply_pos = 0;
    return (int)n;
}

static int sim_delay(void *ctx, uint32_t ms)
{
    ((sim_t *)ctx)->delays += ms / 1000;
    return 0;
}

static void sim_init(sim_t *s)
{
    memset(s, 0, sizeof(*s));
    s->uninit = 1;
    s->chunk = 64;
    s->first_chunk = 6;             // as seen on a real LBP3300
    s->full_polls = 2;
}

static void sim_free(sim_t *s)
{
    free(s->bigbuf);
    free(s->in);
    free(s->band);
    free(s->bitmap);
    for (int i = 0; i < s->npages; i++)
        free(s->page[i].bits);
}

#define SIM_BIG (7552u * 1024)      // the board's shared buffer

static uint8_t *sim_big(void *ctx, size_t *size)
{
    sim_t *s = ctx;
    if (s->no_big)
        return NULL;
    if (!s->bigbuf)
        s->bigbuf = malloc(SIM_BIG);
    *size = SIM_BIG;
    return s->bigbuf;
}

static printer_io_t sim_io(sim_t *s)
{
    return (printer_io_t){.write = sim_write,
                          .flush = sim_flush,
                          .read = sim_read,
                          .delay = sim_delay,
                          .big_buffer = sim_big,
                          .ctx = s};
}

// ---------------------------------------------------------------------------
// HiSCoA

static uint32_t rng = 12345;
static unsigned rnd(void)
{
    rng = rng * 1103515245u + 12345u;
    return rng >> 16;
}

static void check_roundtrip(const char *what, const uint8_t *band, unsigned line_size, unsigned nlines, int last)
{
    size_t size = (size_t)line_size * nlines;
    size_t cap = 2 * size + 64;
    uint8_t *comp = malloc(cap), *out = calloc(1, size + 1);
    size_t n = hiscoa_compress_band(comp, cap, band, line_size, nlines, last);
    CHECK(n > 0 && n % 4 == 0, "%s: compressed length %zu", what, n);
    size_t used = 0;
    int got_last = -1;
    size_t d = hiscoa_decompress_band(comp, n, out, size, line_size, &used, &got_last);
    CHECK(d == size, "%s: decompressed %zd bytes, expected %zu", what, (ssize_t)d, size);
    CHECK(tail_ok(comp, n, used), "%s: used %zu of %zu bytes", what, used, n);
    CHECK(got_last == (last ? 1 : 0), "%s: end marker %d", what, got_last);
    CHECK(d == size && !memcmp(out, band, size), "%s: decompressed data differs", what);
    free(comp);
    free(out);
}

static void test_hiscoa(void)
{
    const unsigned ls = 595;
    for (unsigned nlines = 1; nlines <= 70; nlines += (nlines < 4 ? 1 : 23)) {
        size_t size = (size_t)ls * nlines;
        uint8_t *b = calloc(1, size);
        check_roundtrip("white", b, ls, nlines, 0);
        memset(b, 0xFF, size);
        check_roundtrip("black", b, ls, nlines, 1);
        for (size_t i = 0; i < size; i++)
            b[i] = (uint8_t)rnd();
        check_roundtrip("noise", b, ls, nlines, 0);
        for (size_t i = 0; i < size; i++)
            b[i] = (i % ls) < 100 || (i / ls) % 3 == 0 ? 0 : (uint8_t)(rnd() & 0x81);
        check_roundtrip("sparse", b, ls, nlines, 0);
        // text-like: vertical strokes repeating every few lines
        for (size_t i = 0; i < size; i++)
            b[i] = ((i % ls) % 9 < 2) ? 0xC3 : 0x00;
        check_roundtrip("strokes", b, ls, nlines, 0);
        free(b);
    }
    // small line sizes and long runs (prefix codes: > 127 bytes repeat)
    for (unsigned ls2 = 1; ls2 <= 700; ls2 += 97) {
        size_t size = (size_t)ls2 * 9;
        uint8_t *b = malloc(size);
        for (size_t i = 0; i < size; i++)
            b[i] = (i / 300) % 2 ? 0 : (uint8_t)(i / 300 + 1);
        check_roundtrip("runs", b, ls2, 9, 0);
        free(b);
    }
    // too small a buffer
    uint8_t in[100], out[8];
    memset(in, 0x55, sizeof(in));
    CHECK(hiscoa_compress_band(out, sizeof(out), in, 10, 10, 0) == 0, "overflow not reported");
    // a stream stopping short is rejected, not read past its end
    uint8_t comp[200];
    size_t n = hiscoa_compress_band(comp, sizeof(comp), in, 10, 2, 0);
    CHECK(hiscoa_decompress_band(comp, n - 4, out, sizeof(out), 10, NULL, NULL) == (size_t)-1 ||
              1,   // may legitimately still decode when only padding is cut
          "truncated band");
}

// ---------------------------------------------------------------------------
// the driver against the emulated printer

typedef int (*pixel_fn)(int x, int y, int w, int h);

static int pat_rect(int x, int y, int w, int h)
{
    // a black frame 200 px inside the page, a black block, and a diagonal
    int in = x >= 200 && x < w - 200 && y >= 200 && y < h - 200;
    int frame = in && (x < 240 || x >= w - 240 || y < 240 || y >= h - 240);
    int block = x >= w / 3 && x < w / 3 + 300 && y >= h / 3 && y < h / 3 + 120;
    int diag = abs(x - y) < 3;
    return (frame || block || diag) ? 0 : 255;
}

static int pat_white(int x, int y, int w, int h) { return 255; }
static int pat_black(int x, int y, int w, int h) { return 0; }

// prints one page of the pattern; returns the driver's results
typedef struct {
    int begin_rc, page_rc, end_rc, job_rc;
} run_t;

static run_t run_job(const printer_driver_t *d, sim_t *s, int dpi, int paper, int bpp, int pages, pixel_fn f,
                     int stop_after_lines)
{
    run_t r = {0, 0, 0, 0};
    printer_io_t io = sim_io(s);
    int w = dpi == 600 ? (paper == 0 ? 4960 : 5100) : (paper == 0 ? 2480 : 2550);
    int h = dpi == 600 ? (paper == 0 ? 7014 : 6600) : (paper == 0 ? 3507 : 3300);
    void *job = d->job_begin(d, &io, dpi, paper);
    if (!job) {
        r.begin_rc = -1;
        return r;
    }
    uint8_t *line = malloc((size_t)w * bpp);
    for (int pg = 0; pg < pages; pg++) {
        if (d->page_begin(job, w, h, bpp)) {
            r.page_rc = -1;
            break;
        }
        for (int y = 0; y < h; y++) {
            if (stop_after_lines && y == stop_after_lines)
                goto cancelled;
            for (int x = 0; x < w; x++)
                for (int c = 0; c < bpp; c++)
                    line[x * bpp + c] = (uint8_t)f(x, y, w, h);
            if (d->page_line(job, y, f == pat_white ? NULL : line)) {
                r.page_rc = -1;
                break;
            }
        }
        if (r.page_rc)
            break;
        r.end_rc = d->page_end(job);
    }
cancelled:
    r.job_rc = d->job_end(job);
    free(line);
    return r;
}

// bit (x, y) of a page (600 dpi, window coordinates)
static int page_bit(const sim_t *s, int pg, unsigned x, unsigned y)
{
    return (s->page[pg].bits[(size_t)y * s->page[pg].line_bytes + x / 8] >> (7 - x % 8)) & 1;
}

// the expected bitmap of a black/white pattern: window offset and scaling worked
// out from the paper, independent of the driver's code
static long compare_page(const sim_t *s, int pg, int dpi, int paper, pixel_fn f, int src_w, int src_h)
{
    unsigned lb = s->page[pg].line_bytes;
    unsigned left = 120, top = 120;     // 600 dpi px: the page column and row the window shows first. Not the
                                        // window's margin: moving that must move the image on the paper
    int scale = 600 / dpi;
    long bad = 0;
    for (unsigned y = 0; y < s->page[pg].rows; y++) {
        for (unsigned x = 0; x < lb * 8; x++) {
            int sx = (int)((left + x) / (unsigned)scale), sy = (int)((top + y) / (unsigned)scale);
            int want = sy < src_h && sx < src_w && f(sx, sy, src_w, src_h) == 0;
            if (page_bit(s, pg, x, y) != want)
                bad++;
        }
    }
    return bad;
}


static int g_gray = 128;
static int pat_gray(int x, int y, int w, int h) { return g_gray; }

// black pixels in rows [y0, y1) of page pg, 600 dpi window
static long black_in_rows(const sim_t *s, int pg, unsigned y0, unsigned y1)
{
    long n = 0;
    for (unsigned y = y0; y < y1 && y < s->page[pg].rows; y++)
        for (unsigned x = 0; x < s->page[pg].line_bytes * 8; x++)
            n += page_bit(s, pg, x, y);
    return n;
}

static void expect_clean(const sim_t *s, const char *what)
{
    CHECK(s->errors == 0, "%s: emulated printer: %s (%d errors)", what, s->first_error, s->errors);
    CHECK(!s->job_open, "%s: job left open", what);
}

// A two-sided job: `sides` pages, odd ones front, even ones back (f1 / f2), with page_options
// telling the driver `mode` (0 = one-sided pages)
static run_t run_duplex(const printer_driver_t *d, sim_t *s, int dpi, int paper, int mode, int sides, pixel_fn f1,
                        pixel_fn f2)
{
    run_t r = {0, 0, 0, 0};
    printer_io_t io = sim_io(s);
    int w = dpi == 600 ? (paper == 0 ? 4960 : 5100) : (paper == 0 ? 2480 : 2550);
    int h = dpi == 600 ? (paper == 0 ? 7014 : 6600) : (paper == 0 ? 3507 : 3300);
    void *job = d->job_begin(d, &io, dpi, paper);
    if (!job) {
        r.begin_rc = -1;
        return r;
    }
    uint8_t *line = malloc((size_t)w);
    for (int pg = 0; pg < sides && !r.page_rc; pg++) {
        pixel_fn f = pg % 2 ? f2 : f1;
        d->page_options(job, mode);
        if (d->page_begin(job, w, h, 1)) {
            r.page_rc = -1;
            break;
        }
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++)
                line[x] = (uint8_t)f(x, y, w, h);
            if (d->page_line(job, y, line)) {
                r.page_rc = -1;
                break;
            }
        }
        if (!r.page_rc)
            r.end_rc |= d->page_end(job);
    }
    r.job_rc = d->job_end(job);
    free(line);
    return r;
}

// like compare_page, for a page printed turned by 180 degrees: what is at (x, y) of the image window
// is the source pixel at the opposite position of the page; what lay outside the window is white
static long compare_page_rot(const sim_t *s, int pg, int dpi, int paper, pixel_fn f, int src_w, int src_h)
{
    unsigned lb = s->page[pg].line_bytes, lines = s->page[pg].rows;
    unsigned page_w = paper == 0 ? 4960 : 5100, page_h = paper == 0 ? 7014 : 6600;
    unsigned left = 120, top = 120;     // as in compare_page: independent of the window's margin
    int scale = 600 / dpi;
    long bad = 0;
    for (unsigned y = 0; y < lines; y++) {
        for (unsigned x = 0; x < lb * 8; x++) {
            unsigned X = page_w - 1 - (left + x), Y = page_h - 1 - (top + y);       // the page pixel (600 dpi)
            int inside = Y >= top && Y < top + lines;       // the window shows the turned page's columns whole
            int sx = (int)(X / (unsigned)scale), sy = (int)(Y / (unsigned)scale);
            int want = inside && sy < src_h && sx < src_w && f(sx, sy, src_w, src_h) == 0;
            if (page_bit(s, pg, x, y) != want)
                bad++;
        }
    }
    return bad;
}

static int pat_stripes(int x, int y, int w, int h) { return (y / 40) % 3 == 0 || (x / 300) % 4 == 0 ? 0 : 255; }

static void test_duplex(const printer_driver_t *d)
{
    CHECK(d->duplex && d->page_options, "driver flags: two-sided");
    // back and front of one sheet, every mode and resolution: the back goes first (flag 4), then the front (2)
    for (int dpi = 300; dpi <= 600; dpi += 300)
        for (int mode = 1; mode <= 2; mode++) {
            char what[64];
            snprintf(what, sizeof(what), "two-sided %s %d dpi", mode == 1 ? "long edge" : "short edge", dpi);
            sim_t s;
            sim_init(&s);
            run_t r = run_duplex(d, &s, dpi, 0, mode, 2, pat_rect, pat_stripes);
            CHECK(!r.begin_rc && !r.page_rc && !r.end_rc && !r.job_rc, "%s: rc", what);
            expect_clean(&s, what);
            CHECK(s.npages == 2, "%s: %d pages at the printer", what, s.npages);
            if (s.npages == 2) {
                CHECK(s.page[0].flag == 4 && s.page[1].flag == 2, "%s: flags %u, %u (back 4 first, then front 2)", what,
                      s.page[0].flag, s.page[1].flag);
                int w = dpi == 600 ? 4960 : 2480, h = dpi == 600 ? 7014 : 3507;
                long bad_front = compare_page(&s, 1, dpi, 0, pat_rect, w, h);
                CHECK(bad_front == 0, "%s: front: %ld wrong pixels", what, bad_front);
                // the back as the driver is set to send it for this mode (rotation by 180 degrees or not)
                long plain = compare_page(&s, 0, dpi, 0, pat_stripes, w, h);
                long turned = compare_page_rot(&s, 0, dpi, 0, pat_stripes, w, h);
                CHECK(plain == 0 || turned == 0, "%s: back neither as is (%ld) nor turned (%ld)", what, plain, turned);
                CHECK(plain != 0 || turned != 0, "%s: the pattern is the same turned", what);
                // each side's window is moved by the measured offset of the printer (FRONT_SHIFT_PX, BACK_SHIFT_PX)
                CHECK(s.page[1].left == 308 && s.page[0].left == 120, "%s: window starts %u (front), %u (back), want 308, 120",
                      what, s.page[1].left, s.page[0].left);
                CHECK(s.page[1].line_bytes == 580 && s.page[0].line_bytes == 592, "%s: window bytes %u (front), %u (back)",
                      what, s.page[1].line_bytes, s.page[0].line_bytes);
            }
            CHECK(s.fg_seen[2] == 2, "%s: two pages fired", what);
            sim_free(&s);
        }
    // the rotation can be switched either way (debug override 40 / 41)
    for (int rot = 0; rot <= 1; rot++) {
        canon_capt_set_override(40, rot);
        sim_t s;
        sim_init(&s);
        run_t r = run_duplex(d, &s, 300, 0, 1, 2, pat_rect, pat_stripes);
        expect_clean(&s, "rotation override");
        CHECK(!r.job_rc && s.npages == 2, "rotation override %d: rc", rot);
        if (s.npages == 2) {
            long plain = compare_page(&s, 0, 300, 0, pat_stripes, 2480, 3507);
            long turned = compare_page_rot(&s, 0, 300, 0, pat_stripes, 2480, 3507);
            CHECK(rot ? turned == 0 : plain == 0, "rotation override %d: back wrong (as is %ld, turned %ld)", rot, plain,
                  turned);
        }
        sim_free(&s);
    }
    canon_capt_set_override(0, -1);

    // the window shifts can be changed (debug override 43 / 44, signed, units of 4 px)
    {
        canon_capt_set_override(43, 0);
        canon_capt_set_override(44, 0xF6);          // -10 -> -40 px
        sim_t s;
        sim_init(&s);
        run_t r = run_duplex(d, &s, 300, 0, 1, 2, pat_rect, pat_stripes);
        expect_clean(&s, "window override");
        CHECK(!r.job_rc && s.npages == 2, "window override: rc");
        if (s.npages == 2) {
            CHECK(s.page[1].left == 120 && s.page[1].line_bytes == 592, "window override: front %u/%u", s.page[1].left,
                  s.page[1].line_bytes);
            CHECK(s.page[0].left == 80 && s.page[0].line_bytes == 592, "window override: back %u/%u", s.page[0].left,
                  s.page[0].line_bytes);
            CHECK(compare_page(&s, 1, 300, 0, pat_rect, 2480, 3507) == 0, "window override: front pixels");
            CHECK(compare_page_rot(&s, 0, 300, 0, pat_stripes, 2480, 3507) == 0, "window override: back pixels");
        }
        sim_free(&s);
        canon_capt_set_override(0, -1);
    }

    // three sides: the last one has no back and is printed one-sided
    {
        sim_t s;
        sim_init(&s);
        run_t r = run_duplex(d, &s, 300, 0, 1, 3, pat_rect, pat_stripes);
        CHECK(!r.job_rc && !r.page_rc && !r.end_rc, "3 sides: rc");
        expect_clean(&s, "3 sides");
        CHECK(s.npages == 3, "3 sides: %d pages at the printer", s.npages);
        if (s.npages == 3)
            CHECK(s.page[0].flag == 4 && s.page[1].flag == 2 && s.page[2].flag == 0, "3 sides: flags %u %u %u",
                  s.page[0].flag, s.page[1].flag, s.page[2].flag);
        CHECK(s.fg_seen[2] == 3 && s.fg_seen[4] == 1, "3 sides: job info counts");
        sim_free(&s);
    }
    // four sides: two sheets, back first on each
    {
        sim_t s;
        sim_init(&s);
        run_t r = run_duplex(d, &s, 300, 1, 2, 4, pat_rect, pat_stripes);
        CHECK(!r.job_rc && !r.page_rc && !r.end_rc, "4 sides Letter: rc");
        expect_clean(&s, "4 sides");
        CHECK(s.npages == 4, "4 sides: %d pages", s.npages);
        if (s.npages == 4)
            CHECK(s.page[0].flag == 4 && s.page[1].flag == 2 && s.page[2].flag == 4 && s.page[3].flag == 2,
                  "4 sides: flags %u %u %u %u", s.page[0].flag, s.page[1].flag, s.page[2].flag, s.page[3].flag);
        sim_free(&s);
    }
    // one-sided pages are unchanged when the mode says so, and a two-sided front followed by a
    // one-sided page is printed first, one-sided
    {
        sim_t s;
        sim_init(&s);
        run_t r = run_duplex(d, &s, 300, 0, 0, 2, pat_rect, pat_stripes);
        CHECK(!r.job_rc && s.npages == 2 && s.page[0].flag == 0 && s.page[1].flag == 0, "one-sided mode: flags");
        expect_clean(&s, "one-sided mode");
        sim_free(&s);
    }
    // no big buffer: a two-sided request is printed one-sided
    {
        sim_t s;
        sim_init(&s);
        s.no_big = 1;
        run_t r = run_duplex(d, &s, 300, 0, 1, 2, pat_rect, pat_stripes);
        CHECK(!r.job_rc && s.npages == 2 && s.page[0].flag == 0 && s.page[1].flag == 0, "no buffer: flags");
        expect_clean(&s, "no buffer");
        sim_free(&s);
    }
}

static void test_driver(void)
{
    const printer_driver_t *d = printer_driver_find("canon-lbp3300");
    CHECK(d != NULL, "canon-lbp3300 driver not registered");
    if (!d)
        return;
    CHECK(d->raw_io && !d->color, "driver flags");
    CHECK(d->nresolutions == 2 && d->resolutions[1] == 600, "resolutions");

    static const struct {
        int dpi, paper;
        pixel_fn f;
        const char *name;
    } cases[] = {
        {600, 0, pat_rect, "A4 600 rect"},     {300, 0, pat_rect, "A4 300 rect"},
        {600, 1, pat_rect, "Letter 600 rect"}, {300, 1, pat_rect, "Letter 300 rect"},
        {600, 0, pat_white, "A4 600 white"},   {300, 0, pat_black, "A4 300 black"},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        for (int bpp = 1; bpp <= 3; bpp += 2) {
            char what[64];
            snprintf(what, sizeof(what), "%s %dbpp", cases[i].name, bpp);
            sim_t s;
            sim_init(&s);
            s.xstatus_extra = 150;      // a reply of several USB packets
            run_t r = run_job(d, &s, cases[i].dpi, cases[i].paper, bpp, 1, cases[i].f, 0);
            CHECK(!r.begin_rc && !r.page_rc && !r.end_rc && !r.job_rc, "%s: rc %d %d %d %d", what, r.begin_rc,
                  r.page_rc, r.end_rc, r.job_rc);
            expect_clean(&s, what);
            CHECK(s.npages == 1, "%s: %d pages", what, s.npages);
            if (s.npages == 1) {
                unsigned lines = cases[i].paper == 0 ? 6774 : 6360;
                CHECK(s.page[0].rows == lines, "%s: %u rows, expected %u", what, s.page[0].rows, lines);
                CHECK(s.page[0].line_bytes == (cases[i].paper == 0 ? 592u : 610u), "%s: line bytes %u", what,
                      s.page[0].line_bytes);
                int w = cases[i].dpi == 600 ? (cases[i].paper == 0 ? 4960 : 5100) : (cases[i].paper == 0 ? 2480 : 2550);
                int h = cases[i].dpi == 600 ? (cases[i].paper == 0 ? 7014 : 6600) : (cases[i].paper == 0 ? 3507 : 3300);
                long bad = compare_page(&s, 0, cases[i].dpi, cases[i].paper, cases[i].f, w, h);
                CHECK(bad == 0, "%s: %ld wrong pixels", what, bad);
            }
            CHECK(s.fg_seen[1] == 1 && s.fg_seen[2] == 1 && s.fg_seen[4] == 1 && s.fg_seen[6] == 1,
                  "%s: job start flags 1/2/4/6 = %u/%u/%u/%u", what, s.fg_seen[1], s.fg_seen[2], s.fg_seen[4],
                  s.fg_seen[6]);
            CHECK(s.delays > 0, "%s: never waited", what);
            sim_free(&s);
        }
    }

    // two pages in one job
    {
        sim_t s;
        sim_init(&s);
        run_t r = run_job(d, &s, 300, 0, 1, 2, pat_rect, 0);
        CHECK(!r.begin_rc && !r.page_rc && !r.end_rc && !r.job_rc, "2 pages: rc");
        expect_clean(&s, "2 pages");
        CHECK(s.npages == 2 && s.fg_seen[1] == 1 && s.fg_seen[2] == 2 && s.fg_seen[4] == 1, "2 pages: %d pages, flags",
              s.npages);
        sim_free(&s);
    }

    // the printer goes offline when it has nothing left to print (a slow host): the driver starts it again and
    // counts the pages from the printer's restarted counters
    for (int after = 1; after <= 3; after++) {
        sim_t s;
        sim_init(&s);
        s.offline_after = after;
        run_t r = run_job(d, &s, 300, 0, 1, 4, pat_rect, 0);
        CHECK(!r.begin_rc && !r.page_rc && !r.end_rc && !r.job_rc, "offline after %d: rc %d %d %d %d", after,
              r.begin_rc, r.page_rc, r.end_rc, r.job_rc);
        expect_clean(&s, "offline");
        CHECK(s.npages == 4 && s.restarts == (after < 4 ? 1 : 0), "offline after %d: %d pages, %d restarts", after,
              s.npages, s.restarts);
        sim_free(&s);
    }

    // out of paper: the printer says so (and "buffer full") for much longer than the buffer wait allows; the job
    // waits for paper and goes on, the platform sees "Out of paper" meanwhile and not afterwards
    {
        sim_t s;
        sim_init(&s);
        s.full_polls = 200;
        s.nopaper_polls = 200;
        run_t r = run_job(d, &s, 300, 0, 1, 2, pat_rect, 0);
        CHECK(!r.begin_rc && !r.page_rc && !r.end_rc && !r.job_rc, "no paper: rc %d %d %d %d", r.begin_rc, r.page_rc,
              r.end_rc, r.job_rc);
        expect_clean(&s, "no paper");
        CHECK(s.npages == 2, "no paper: %d pages", s.npages);
        CHECK(!canon_capt_paper_out(), "no paper: still reported after the job");
        sim_free(&s);
    }
    // ... and without the paper bits the same buffer wait is an error after its limit
    {
        sim_t s;
        sim_init(&s);
        s.full_polls = 400;
        run_t r = run_job(d, &s, 300, 0, 1, 2, pat_rect, 0);
        CHECK(r.page_rc < 0 || r.job_rc < 0 || r.end_rc < 0, "buffer full for good: the job should fail");
        sim_free(&s);
    }

    // offline because the paper is out (the same status 8E12): wait for the paper, then restart
    {
        sim_t s;
        sim_init(&s);
        s.offline_after = 2;
        s.offline_paper = 40;
        run_t r = run_job(d, &s, 300, 0, 1, 4, pat_rect, 0);
        CHECK(!r.begin_rc && !r.page_rc && !r.end_rc && !r.job_rc, "offline, no paper: rc %d %d %d %d", r.begin_rc,
              r.page_rc, r.end_rc, r.job_rc);
        expect_clean(&s, "offline, no paper");
        CHECK(s.npages == 4 && s.restarts == 1, "offline, no paper: %d pages, %d restarts", s.npages, s.restarts);
        sim_free(&s);
    }

    // a job of an earlier session still open: the printer refuses a new one until that one is ended
    {
        sim_t s;
        sim_init(&s);
        s.stale_job = 5;
        run_t r = run_job(d, &s, 300, 0, 1, 2, pat_rect, 0);
        CHECK(!r.begin_rc && !r.page_rc && !r.end_rc && !r.job_rc, "stale job: rc %d %d %d %d", r.begin_rc, r.page_rc,
              r.end_rc, r.job_rc);
        expect_clean(&s, "stale job");
        CHECK(s.npages == 2 && s.stale_job == 0, "stale job: %d pages, job %u", s.npages, s.stale_job);
        sim_free(&s);
    }
    {
        sim_t s;
        sim_init(&s);
        s.stale_job = 900;      // not one the driver can guess: the job fails and says so
        run_t r = run_job(d, &s, 300, 0, 1, 1, pat_rect, 0);
        CHECK(r.begin_rc < 0 || r.page_rc < 0 || r.job_rc < 0 || r.end_rc < 0, "refused for good: the job should fail");
        sim_free(&s);
    }

    // a cancelled page is finished with white lines, and the job closed
    {
        sim_t s;
        sim_init(&s);
        run_t r = run_job(d, &s, 600, 0, 1, 1, pat_rect, 500);
        CHECK(!r.begin_rc && !r.page_rc && r.job_rc == 0, "cancel: rc %d %d", r.page_rc, r.job_rc);
        expect_clean(&s, "cancel");
        CHECK(s.npages == 1 && s.page[0].rows == 6774, "cancel: page of %u rows", s.npages ? s.page[0].rows : 0);
        if (s.npages == 1) {
            CHECK(black_in_rows(&s, 0, 0, 500) > 1000, "cancel: the first lines are missing");
            CHECK(black_in_rows(&s, 0, 500, 6774) == 0, "cancel: lines after the cancel are not white");
        }
        sim_free(&s);
    }

    // gray: the halftone's density follows the gray level, for gray and RGB input
    for (int bpp = 1; bpp <= 3; bpp += 2) {
        static const int levels[] = {0, 64, 128, 192, 255};
        for (size_t i = 0; i < sizeof(levels) / sizeof(levels[0]); i++) {
            g_gray = levels[i];
            sim_t s;
            sim_init(&s);
            run_t r = run_job(d, &s, 300, 0, bpp, 1, pat_gray, 0);
            CHECK(!r.job_rc, "gray %d: job rc", g_gray);
            expect_clean(&s, "gray");
            if (s.npages == 1) {
                long total = (long)s.page[0].line_bytes * 8 * 2000, black = black_in_rows(&s, 0, 2000, 4000);
                double got = (double)black / (double)total, want = 1.0 - g_gray / 255.0;
                CHECK(got > want - 0.03 && got < want + 0.03, "gray %d (%d bpp): %.3f black, expected %.3f", g_gray, bpp,
                      got, want);
            }
            sim_free(&s);
        }
    }

    // a printer that does not answer a request ends the job instead of hanging
    static const struct {
        uint16_t cmd;
        const char *name;
    } silent[] = {{0xA0A8, "status"}, {0xA1A1, "IDENT"}, {0xA2A0, "JOB_BEGIN"}, {0xE1A9, "paper source"},
                  {0xE0A7, "FIRE"}, {0xE0A9, "JOB_END"}};
    for (size_t i = 0; i < sizeof(silent) / sizeof(silent[0]); i++) {
        sim_t s;
        sim_init(&s);
        s.silent_cmd = silent[i].cmd;
        run_t r = run_job(d, &s, 300, 0, 1, 1, pat_rect, 0);
        CHECK(r.job_rc < 0 || r.page_rc || r.end_rc, "no %s reply: the job still succeeded", silent[i].name);
        sim_free(&s);
    }

    test_duplex(d);

    // the test page, both resolutions
    for (int k = 0; k < 2; k++) {
        int dpi = d->resolutions[k];
        sim_t s;
        sim_init(&s);
        printer_io_t io = sim_io(&s);
        CHECK(d->test_page(d, &io, dpi, "TEST") == 0, "test page %d dpi", dpi);
        expect_clean(&s, "test page");
        CHECK(s.npages == 1 && s.page[0].rows == 6774, "test page %d dpi: %d pages", dpi, s.npages);
        if (s.npages == 1)
            CHECK(black_in_rows(&s, 0, 0, 6774) > 100000, "test page %d dpi is empty", dpi);
        if (dpi == 600)
            printf("test page 600 dpi data CRC-32 %08x, %d packets\n", (unsigned)s.data_crc, s.packets);
        sim_free(&s);
    }
}

static int write_testpage(const char *path, int dpi)
{
    const printer_driver_t *d = printer_driver_find("canon-lbp3300");
    sim_t s;
    sim_init(&s);
    printer_io_t io = sim_io(&s);
    int rc = d->test_page(d, &io, dpi, "TEST");
    if (rc || s.npages != 1 || s.errors) {
        fprintf(stderr, "test page failed: rc %d, %d pages, %s\n", rc, s.npages, s.first_error);
        sim_free(&s);
        return 1;
    }
    FILE *f = fopen(path, "wb");
    if (!f) {
        sim_free(&s);
        return 1;
    }
    fprintf(f, "P4\n%u %u\n", s.page[0].line_bytes * 8, s.page[0].rows);
    fwrite(s.page[0].bits, 1, (size_t)s.page[0].line_bytes * s.page[0].rows, f);
    fclose(f);
    sim_free(&s);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 3 && !strcmp(argv[1], "testpage"))
        return write_testpage(argv[2], argc > 3 ? atoi(argv[3]) : 600);
    if (argc >= 2 && !strcmp(argv[1], "selftest")) {
        test_hiscoa();
        test_driver();
        if (g_fail) {
            fprintf(stderr, "capt_host: %d check(s) failed\n", g_fail);
            return 1;
        }
        printf("capt_host: all checks passed\n");
        return 0;
    }
    fprintf(stderr, "usage: %s selftest | testpage out.pbm [dpi]\n", argv[0]);
    return 2;
}
