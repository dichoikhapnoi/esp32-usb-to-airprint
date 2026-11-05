// Canon CAPT 2.x command layer. Ported from captdriver (capt-command.c,
// capt-status.c, Copyright (C) 2013 Alexey Galakhov). Licensed under the GNU
// General Public License Version 3 or later; see hiscoa.c for the notice.
#include "capt.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_log.h"
static const char *TAG = "capt";
#endif

#define REPLY_TIMEOUT_MS 15000      // for the start of a reply
#define CONTINUE_TIMEOUT_MS 1500    // for the rest of one

static inline uint8_t LO(unsigned x) { return (uint8_t)(x & 0xFF); }
static inline uint8_t HI(unsigned x) { return (uint8_t)((x >> 8) & 0xFF); }
static inline uint16_t WORD(uint8_t lo, uint8_t hi) { return (uint16_t)((hi << 8) | lo); }

// the size field of a reply is sometimes BCD (captdriver's quirk)
static uint16_t bcd(uint8_t lo, uint8_t hi)
{
    unsigned a = (hi >> 4) & 0xF, b = hi & 0xF, c = (lo >> 4) & 0xF, d = lo & 0xF;
    if (a > 9 || b > 9 || c > 9 || d > 9)
        return WORD(lo, hi);
    return (uint16_t)(a * 1000 + b * 100 + c * 10 + d);
}

void capt_hex(const char *what, const uint8_t *p, size_t n)
{
    char line[3 * 64 + 24];
    size_t shown = n > 64 ? 64 : n, k = 0;
    for (size_t i = 0; i < shown; i++)
        k += (size_t)snprintf(line + k, sizeof(line) - k, "%02X ", p[i]);
    if (n > shown)
        snprintf(line + k, sizeof(line) - k, "... (%u bytes)", (unsigned)n);
#ifdef ESP_PLATFORM
    ESP_LOGI(TAG, "%s: %s", what, line);
#else
    if (getenv("CAPT_TRACE"))
        fprintf(stderr, "CAPT %s: %s\n", what, line);
#endif
}

void capt_init(capt_t *c, const printer_io_t *io)
{
    memset(c, 0, sizeof(*c));
    c->io = io;
}

static int delay_ms(capt_t *c, uint32_t ms)
{
    return c->io->delay ? c->io->delay(c->io->ctx, ms) : 0;
}

int capt_send(capt_t *c, uint16_t cmd, const void *data, size_t len)
{
    if (len + 4 > 0xFFFF)
        return -1;
    uint8_t h[4] = {LO(cmd), HI(cmd), LO(len + 4), HI(len + 4)};
    if (cmd != CAPT_PRINT_DATA) {
        capt_hex("send", h, 4);
        if (len)
            capt_hex("  data", data, len);
    }
    if (c->io->write(c->io->ctx, h, 4) < 0)
        return -1;
    if (len && c->io->write(c->io->ctx, data, len) < 0)
        return -1;
    if (c->io->flush && c->io->flush(c->io->ctx) < 0)
        return -1;
    return 0;
}

// Reads the reply to `cmd` into c->rx.
static int recv_reply(capt_t *c, uint16_t cmd)
{
    if (!c->io->read)
        return -1;
    size_t off = 0;
    for (;;) {
        if (sizeof(c->rx) - off < 512)
            return -1;
        // The LBP3300 sends a reply in pieces (seen: 6 bytes, then the other 58 of a 64-byte reply):
        // keep reading until the size in the header is reached.
        int n = c->io->read(c->io->ctx, c->rx + off, sizeof(c->rx) - off,
                            off ? CONTINUE_TIMEOUT_MS : REPLY_TIMEOUT_MS);
        if (n < 0)
            return -1;
        if (n == 0) {
            // the size field of some replies is BCD (captdriver's quirk): the reply is
            // complete if the bytes received are that size and nothing more comes
            if (off >= 4 && off == bcd(c->rx[2], c->rx[3]) && WORD(c->rx[0], c->rx[1]) == cmd)
                break;
            capt_hex("no reply, got", c->rx, off);
            return -2;
        }
        off += (size_t)n;
        if (off < 4)
            continue;
        if (WORD(c->rx[0], c->rx[1]) != cmd) {
            capt_hex("bad reply", c->rx, off);
            return -1;
        }
        uint16_t size = WORD(c->rx[2], c->rx[3]);
        if (off == size)
            break;
        if (off > size && off != bcd(c->rx[2], c->rx[3])) {
            capt_hex("bad reply size", c->rx, off);
            return -1;
        }
        // off < size: the rest follows (or the size is BCD and nothing more comes)
    }
    c->rx_len = off;
    capt_hex("recv", c->rx, off);
    return 0;
}

int capt_sendrecv(capt_t *c, uint16_t cmd, const void *data, size_t len, uint8_t *reply, size_t *reply_len)
{
    int r = capt_send(c, cmd, data, len);
    if (r < 0)
        return r;
    r = recv_reply(c, cmd);
    if (r < 0)
        return r;
    size_t dlen = c->rx_len - 4;
    if (reply) {
        size_t n = reply_len ? *reply_len : dlen;
        if (n > dlen)
            n = dlen;
        memcpy(reply, c->rx + 4, n);
    }
    if (reply_len)
        *reply_len = dlen;
    return 0;
}

void capt_multi_begin(capt_t *c, uint16_t cmd)
{
    c->mbuf[0] = LO(cmd);
    c->mbuf[1] = HI(cmd);
    c->mlen = 4;
}

void capt_multi_add(capt_t *c, uint16_t cmd, const void *data, size_t len)
{
    if (c->mlen + 4 + len > sizeof(c->mbuf)) {
        c->mlen = sizeof(c->mbuf) + 1;      // overflow: capt_multi_send fails
        return;
    }
    c->mbuf[c->mlen + 0] = LO(cmd);
    c->mbuf[c->mlen + 1] = HI(cmd);
    c->mbuf[c->mlen + 2] = LO(len + 4);
    c->mbuf[c->mlen + 3] = HI(len + 4);
    if (len)
        memcpy(c->mbuf + c->mlen + 4, data, len);
    c->mlen += 4 + len;
}

int capt_multi_send(capt_t *c)
{
    if (c->mlen > sizeof(c->mbuf))
        return -1;
    c->mbuf[2] = LO(c->mlen);
    c->mbuf[3] = HI(c->mlen);
    capt_hex("send multi", c->mbuf, c->mlen);
    if (c->io->write(c->io->ctx, c->mbuf, c->mlen) < 0)
        return -1;
    if (c->io->flush && c->io->flush(c->io->ctx) < 0)
        return -1;
    return 0;
}

// ---------------------------------------------------------------------------
// status

static void decode_status(capt_t *c, const uint8_t *s, size_t n)
{
#define HAVE(off) ((off) + 2 <= n)
#define W(off) WORD(s[off], s[(off) + 1])
    capt_status_t *st = &c->st;
    if (HAVE(0))
        st->status[0] = W(0);
    if (HAVE(8))
        st->status[1] = W(8);
    if (HAVE(10))
        st->status[2] = W(10);
    if (HAVE(12))
        st->status[3] = W(12);
    if (HAVE(14))
        st->page_decoding = W(14);
    if (HAVE(16))
        st->page_printing = W(16);
    if (HAVE(18))
        st->page_out = W(18);
    if (HAVE(20))
        st->page_completed = W(20);
    if (HAVE(34))
        st->page_received = W(34);
    if (HAVE(24))
        st->status[4] = W(24);
    if (HAVE(30))
        st->status[5] = W(30);
    if (HAVE(38))
        st->status[6] = W(38);
#undef HAVE
#undef W
}

static int download_status(capt_t *c, uint16_t cmd)
{
    int r = capt_sendrecv(c, cmd, NULL, 0, NULL, NULL);
    if (r < 0)
        return r;
    decode_status(c, c->rx + 4, c->rx_len - 4);
    return 0;
}

int capt_get_xstatus(capt_t *c)
{
    return download_status(c, CAPT_CHKXSTATUS);
}

int capt_get_status(capt_t *c)
{
    int r = download_status(c, CAPT_CHKSTATUS);
    if (r < 0)
        return r;
    if (CAPT_FLAG(&c->st, CAPT_FL_XSTATUS_CHNG))
        return capt_get_xstatus(c);
    return 0;
}

int capt_wait_ready(capt_t *c, int max_s)
{
    for (int i = 0; i < max_s; i++) {
        int r = capt_get_status(c);
        if (r < 0)
            return r;
        if (!CAPT_FLAG(&c->st, CAPT_FL_BUSY))
            return 0;
        if (delay_ms(c, 1000) < 0)
            return -1;
    }
    return -2;
}

int capt_wait_xready(capt_t *c, int max_s)
{
    for (int i = 0; i < max_s; i++) {
        int r = capt_get_xstatus(c);
        if (r < 0)
            return r;
        if (!CAPT_FLAG(&c->st, CAPT_FL_BUSY))
            return 0;
        if (delay_ms(c, 1000) < 0)
            return -1;
    }
    return -2;
}
