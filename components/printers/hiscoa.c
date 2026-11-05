// HiSCoA band compression for Canon CAPT 2.x printers. Ported from captdriver
// (src/hiscoa-compress.c, Copyright (C) 2013 Alexey Galakhov), whose output
// is known to print on real LBP printers; the algorithm and the bit stream are
// unchanged. The decompressor is written from captdriver's tests/
// hiscoa-decompress.c for the host tests.
//
// Licensed under the GNU General Public License Version 3 or later, as the
// original. This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or (at your
// option) any later version. It is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
#include "hiscoa.h"

#include <string.h>

// captdriver's hiscoa_default_params
#define ORIGIN_3 1
#define ORIGIN_5 4
#define ORIGIN_0 0
#define ORIGIN_2 (-7)
#define ORIGIN_4 0
#define XORVAL 0x43

// ---------------------------------------------------------------------------
// compressor

typedef struct {
    const uint8_t *in;
    size_t in_size;
    size_t in_pos;

    uint8_t *out;
    size_t out_bits;            // capacity
    size_t bitpos;
    int overflow;               // something did not fit in the output

    unsigned line_size;

    unsigned nbytes;            // recently used bytes, most recent first
    uint8_t bytes[16];

    unsigned origin[6];         // distance back for each long repeat kind, 0 = unused
} enc_t;

static void push_bits(enc_t *e, uint32_t bits, unsigned count)
{
    while (count && e->bitpos < e->out_bits) {
        unsigned word = (unsigned)(e->bitpos / 8);
        unsigned spc = 8 - (unsigned)(e->bitpos % 8);
        unsigned cnt = count > spc ? spc : count;
        uint8_t mask = (uint8_t)((0xFFu >> (8 - cnt)) << (spc - cnt));
        uint32_t val = (bits >> (count - cnt)) << (spc - cnt);
        uint8_t x = e->out[word];
        x ^= XORVAL;
        x &= (uint8_t)~mask;
        x |= (uint8_t)(mask & val);
        x ^= XORVAL;
        e->out[word] = x;
        e->bitpos += cnt;
        count -= cnt;
    }
    if (count)
        e->overflow = 1;
}

// how many bytes from the current position repeat the bytes `diff` behind
static unsigned try_match(const enc_t *e, unsigned diff)
{
    size_t pos = e->in_pos;
    if (pos < diff)
        return 0;
    while (e->in[pos] == e->in[pos - diff]) {
        ++pos;
        if (pos >= e->in_size)
            break;
        if (pos >= e->in_pos + 512 + 127)
            break;
        if (pos % e->line_size == 0)
            break;
    }
    return (unsigned)(pos - e->in_pos);
}

static void swap_u(unsigned *a, unsigned *b)
{
    unsigned t = *a;
    *a = *b;
    *b = t;
}

static unsigned find_msb(unsigned val)
{
    unsigned n = 0;
    while (val) {
        n++;
        val >>= 1;
    }
    return n;
}

static int try_write_longrepeat(enc_t *e)
{
    unsigned best_cmd = 0, best_len = 0;

    for (unsigned cmd = 0; cmd < 6; ++cmd) {
        if (e->origin[cmd]) {
            unsigned len = try_match(e, e->origin[cmd]);
            if (len > best_len) {
                best_cmd = cmd;
                best_len = len;
            }
        }
    }
    if (best_len <= 1)
        return 0;

    unsigned len = best_len;
    if (len > 127) {
        unsigned val = len / 128;
        unsigned nbits = find_msb(val) - 1;
        push_bits(e, 0xFC, 8);
        push_bits(e, nbits, 2);
        push_bits(e, ~val, nbits);
        len %= 128;
    }

    push_bits(e, 0xFFFFFFFE, best_cmd + 1);        // the long repeat kind
    if (best_cmd == 2)
        push_bits(e, 0x0, 1);                      // subcommand 0

    switch (len) {
    case 0:
        push_bits(e, 0x3F, 6);
        break;
    case 1:
        push_bits(e, 0x0, 2);
        break;
    case 2:
        push_bits(e, 0x3, 3);
        break;
    case 3:
        push_bits(e, 0x2, 3);
        break;
    default: {
        unsigned nbits = find_msb(len) - 1;
        push_bits(e, 0xFFFFFFFE, nbits);
        push_bits(e, ~len, nbits);
        break;
    }
    }

    if (best_cmd == 2)
        swap_u(&e->origin[2], &e->origin[0]);
    if (best_cmd == 5)
        swap_u(&e->origin[5], &e->origin[3]);

    e->in_pos += best_len;
    return 1;
}

static int try_write_byterepeat(enc_t *e)
{
    uint8_t byte = e->in[e->in_pos];
    for (unsigned i = 0; i < e->nbytes; ++i) {
        if (byte == e->bytes[i]) {
            for (unsigned j = i; j > 0; --j)
                e->bytes[j] = e->bytes[j - 1];
            e->bytes[0] = byte;
            push_bits(e, 0x20 | ((15 - i) & 0xF), 6);
            e->in_pos += 1;
            return 1;
        }
    }
    return 0;
}

static void write_simple_byte(enc_t *e)
{
    uint8_t byte = e->in[e->in_pos];

    if (e->nbytes < 16)
        e->nbytes += 1;
    for (unsigned i = e->nbytes - 1; i > 0; --i)
        e->bytes[i] = e->bytes[i - 1];
    e->bytes[0] = byte;

    if (byte == 0)
        push_bits(e, 0xFD, 8);              // zero
    else
        push_bits(e, 0xD00u | byte, 12);    // byte
    e->in_pos += 1;
}

size_t hiscoa_compress_band(uint8_t *buf, size_t size, const uint8_t *band, unsigned line_size, unsigned nlines,
                            int last)
{
    enc_t e;
    memset(&e, 0, sizeof(e));
    e.in = band;
    e.in_size = (size_t)line_size * nlines;
    e.out = buf;
    e.out_bits = 8 * size;
    e.line_size = line_size;
    e.origin[1] = 0;
    e.origin[3] = ORIGIN_3;
    e.origin[5] = ORIGIN_5;
    e.origin[0] = ORIGIN_0 + line_size;
    e.origin[2] = (unsigned)(ORIGIN_2 + (int)line_size);
    e.origin[4] = ORIGIN_4;

    while (e.in_pos < e.in_size) {
        if (try_write_longrepeat(&e))
            continue;
        if (try_write_byterepeat(&e))
            continue;
        write_simple_byte(&e);
    }

    push_bits(&e, 0xFE, 8);                         // end
    push_bits(&e, last ? 1 : 0, 2);
    // padded to a multiple of 32 bits; as captdriver does, a stream that is
    // already aligned gets a whole word of ones
    push_bits(&e, 0xFFFFFFFF, 32 - (unsigned)(e.bitpos % 32));
    if (e.overflow)
        return 0;
    return e.bitpos / 8;
}

size_t hiscoa_format_params(uint8_t *buf, size_t size)
{
    const uint8_t b[8] = {
        (uint8_t)(int8_t)ORIGIN_3, (uint8_t)(int8_t)ORIGIN_5, 0x01, 0x01,
        (uint8_t)(int8_t)ORIGIN_0, (uint8_t)(int8_t)ORIGIN_2, (uint8_t)(ORIGIN_4 & 0xFF), (uint8_t)((ORIGIN_4 >> 8) & 0xFF),
    };
    if (size < sizeof(b))
        return 0;
    memcpy(buf, b, sizeof(b));
    return sizeof(b);
}

// ---------------------------------------------------------------------------
// decompressor (tests)

typedef struct {
    const uint8_t *in;
    size_t in_bits;
    size_t bitpos;
    uint8_t *out;
    size_t out_size, out_pos;
    unsigned prefix;
    uint8_t bytes[16];
    unsigned origin[6];
    int bad;
} dec_t;

static int read_bit(dec_t *d)
{
    if (d->bitpos >= d->in_bits) {
        d->bad = 1;
        return 1;
    }
    int b = !!((d->in[d->bitpos / 8] ^ XORVAL) & (0x80 >> (d->bitpos % 8)));
    d->bitpos++;
    return b;
}

static unsigned read_n10(dec_t *d, unsigned limit)
{
    unsigned n = 0;
    while (n < limit && read_bit(d) && !d->bad)
        ++n;
    return n;
}

static unsigned read_number(dec_t *d, unsigned bits)
{
    unsigned r = 0;
    for (unsigned i = 0; i < bits; ++i)
        r = (r << 1) | (unsigned)read_bit(d);
    return r;
}

static unsigned read_long_number(dec_t *d)
{
    unsigned order = read_n10(d, 6);
    if (order == 6)
        return 0;
    if (order == 0) {
        if (!read_bit(d))
            return 1;
        return read_bit(d) ? 2 : 3;
    }
    return (1u << (order + 2)) - 1 - read_number(d, order + 1);
}

static unsigned read_prefix(dec_t *d)
{
    unsigned nbits = read_number(d, 2);
    unsigned val = read_number(d, nbits);
    return 128 * ((1u << (nbits + 1)) - 1 - val);
}

static void out_byte(dec_t *d, uint8_t b)
{
    if (d->out_pos < d->out_size)
        d->out[d->out_pos] = b;
    else
        d->bad = 1;
    d->out_pos++;
}

static void copy_block(dec_t *d, unsigned origin)
{
    unsigned len = d->prefix + read_long_number(d);
    for (unsigned i = 0; i < len && !d->bad; ++i) {
        if (d->out_pos < origin) {
            d->bad = 1;
            return;
        }
        out_byte(d, d->out_pos < d->out_size ? d->out[d->out_pos - origin] : 0);
    }
}

static uint8_t stash_get(dec_t *d, unsigned pos)
{
    uint8_t r = d->bytes[pos];
    for (unsigned i = pos; i > 0; --i)
        d->bytes[i] = d->bytes[i - 1];
    d->bytes[0] = r;
    return r;
}

static uint8_t stash_save(dec_t *d, uint8_t b)
{
    for (unsigned i = 15; i > 0; --i)
        d->bytes[i] = d->bytes[i - 1];
    d->bytes[0] = b;
    return b;
}

size_t hiscoa_decompress_band(const uint8_t *in, size_t in_size, uint8_t *out, size_t out_size, unsigned line_size,
                              size_t *consumed, int *last)
{
    dec_t d;
    memset(&d, 0, sizeof(d));
    d.in = in;
    d.in_bits = in_size * 8;
    d.out = out;
    d.out_size = out_size;
    for (int i = 0; i < 16; i++)
        d.bytes[i] = (i % 8) == 7 ? 0xFF : (uint8_t[]){0xDE, 0xAD, 0xBE, 0xEF}[i % 4];
    d.origin[0] = ORIGIN_0 + line_size;
    d.origin[2] = (unsigned)(ORIGIN_2 + (int)line_size);
    d.origin[3] = ORIGIN_3;
    d.origin[4] = ORIGIN_4;
    d.origin[5] = ORIGIN_5;

    int end = 0;
    if (last)
        *last = -1;
    while (!end && !d.bad && d.bitpos < d.in_bits) {
        int keep_prefix = 0;
        unsigned group = read_n10(&d, 8);
        switch (group) {
        case 0:
            copy_block(&d, d.origin[0]);
            break;
        case 1:
            out_byte(&d, stash_get(&d, 15 - read_number(&d, 4)));
            break;
        case 2:
            if (read_bit(&d)) {
                out_byte(&d, stash_save(&d, (uint8_t)read_number(&d, 8)));
            } else {
                copy_block(&d, d.origin[2]);
                unsigned t = d.origin[2];
                d.origin[2] = d.origin[0];
                d.origin[0] = t;
            }
            break;
        case 3:
            copy_block(&d, d.origin[3]);
            break;
        case 4:
            copy_block(&d, d.origin[4]);
            break;
        case 5: {
            copy_block(&d, d.origin[5]);
            unsigned t = d.origin[5];
            d.origin[5] = d.origin[3];
            d.origin[3] = t;
            break;
        }
        case 6:
            if (read_bit(&d)) {
                out_byte(&d, stash_save(&d, 0));
            } else {
                d.prefix = read_prefix(&d);
                keep_prefix = 1;
            }
            break;
        case 7: {
            int v = (int)read_number(&d, 2);
            if (last)
                *last = v;
            end = 1;
            break;
        }
        default:    // 8: nop
            break;
        }
        if (!keep_prefix)
            d.prefix = 0;
    }
    if (!end)
        d.bad = 1;
    while (d.bitpos % 32 && d.bitpos < d.in_bits)
        read_bit(&d);
    if (consumed)
        *consumed = d.bitpos / 8;
    return d.bad ? (size_t)-1 : d.out_pos;
}
