#include "hiscoa.h"

#include <string.h>

const hiscoa_params_t hiscoa_default_params = {
    .origin_0 = 0,
    .origin_2 = -7,
    .origin_3 = 1,
    .origin_4 = 0,
    .origin_5 = 4,
};

size_t hiscoa_format_params(uint8_t *buf, size_t cap, const hiscoa_params_t *params)
{
    if (cap < 8)
        return 0;
    int16_t o4 = (int16_t)params->origin_4;
    buf[0] = (uint8_t)(int8_t)params->origin_3;
    buf[1] = (uint8_t)(int8_t)params->origin_5;
    buf[2] = 0x01;
    buf[3] = 0x01;
    buf[4] = (uint8_t)(int8_t)params->origin_0;
    buf[5] = (uint8_t)(int8_t)params->origin_2;
    buf[6] = (uint8_t)(o4 & 0xFF);
    buf[7] = (uint8_t)((o4 >> 8) & 0xFF);
    return 8;
}

// ---------------------------------------------------------------------------
// bit-level encoder: a packed, 0x43-obfuscated MSB-first bit stream.

typedef struct {
    uint8_t *out;
    size_t out_cap_bits;
    size_t bitpos;

    const uint8_t *in;
    size_t in_len;
    size_t pos;
    unsigned line_bytes;

    uint8_t lru[16];     // most-recently-seen bytes, [0] = most recent
    unsigned lru_n;

    unsigned origin[6];  // back-reference distances tried for a long repeat; 0 = unused
    bool overflow;       // out ran out of room
} enc_t;

// Appends the low `count` bits of `bits` (count <= 24 at every call site
// below), MSB first, XORing each touched output byte with 0x43 so the
// buffer is never readable in the clear (this is obfuscation, not crypto).
static void put_bits(enc_t *e, uint32_t bits, unsigned count)
{
    while (count && e->bitpos < e->out_cap_bits) {
        unsigned byte_i = (unsigned)(e->bitpos / 8);
        unsigned free_bits = 8 - (unsigned)(e->bitpos % 8);
        unsigned take = count < free_bits ? count : free_bits;
        uint8_t mask = (uint8_t)((0xFFu >> (8 - take)) << (free_bits - take));
        uint8_t val = (uint8_t)(((bits >> (count - take)) << (free_bits - take)) & 0xFF);
        uint8_t b = e->out[byte_i] ^ 0x43;
        b = (uint8_t)((b & ~mask) | (val & mask));
        e->out[byte_i] = b ^ 0x43;
        e->bitpos += take;
        count -= take;
    }
    if (count)
        e->overflow = true;
}

// A unary code: `n` 1-bits then a 0-bit, in n+1 bits total (the long-repeat
// command selector, and the Elias-gamma-style length prefix below).
static void put_unary(enc_t *e, unsigned n)
{
    put_bits(e, 0xFFFFFFFEu, n + 1);
}

static unsigned bit_length(unsigned v)
{
    unsigned n = 0;
    while (v) {
        v >>= 1;
        n++;
    }
    return n;
}

// How many bytes from e->pos match the bytes `dist` back, not crossing a
// line boundary (vertical repeats must not wrap into the next/previous row)
// and capped well under the longest length the length-prefix code can carry.
static unsigned match_len(const enc_t *e, unsigned dist)
{
    if (dist == 0 || dist > e->pos)
        return 0;
    size_t p = e->pos;
    size_t cap = e->pos + 512 + 127;
    while (e->in[p] == e->in[p - dist]) {
        p++;
        if (p >= e->in_len || p >= cap || p % e->line_bytes == 0)
            break;
    }
    return (unsigned)(p - e->pos);
}

// Tries every configured origin distance, keeps the longest match, and if
// it's worth using (> 1 byte), emits it: an optional >127 extension, the
// unary origin-slot selector, then the remaining length. Two of the slots
// are "last used" caches swapped to the front on a hit (origins 0<->2,
// 3<->5), mirroring how consecutive lines tend to repeat the same distance.
static bool emit_repeat(enc_t *e)
{
    int best_slot = -1;
    unsigned best_len = 0;
    for (int slot = 0; slot < 6; slot++) {
        if (!e->origin[slot])
            continue;
        unsigned len = match_len(e, e->origin[slot]);
        if (len > best_len) {
            best_len = len;
            best_slot = slot;
        }
    }
    if (best_slot < 0 || best_len <= 1)
        return false;

    unsigned remaining = best_len;
    if (remaining > 127) {
        unsigned hi = remaining / 128;
        unsigned nbits = bit_length(hi) - 1;
        put_bits(e, 0xFC, 8);
        put_bits(e, nbits, 2);
        put_bits(e, ~hi, nbits);
        remaining %= 128;
    }

    put_unary(e, (unsigned)best_slot);
    if (best_slot == 2)
        put_bits(e, 0, 1);      // sub-command 0 (distinguishes slot 2 from slot "1", unused)

    switch (remaining) {
    case 0:
        put_bits(e, 0x3F, 6);
        break;
    case 1:
        put_bits(e, 0x0, 2);
        break;
    case 2:
        put_bits(e, 0x3, 3);
        break;
    case 3:
        put_bits(e, 0x2, 3);
        break;
    default: {
        unsigned nbits = bit_length(remaining) - 1;
        put_unary(e, nbits - 1);
        put_bits(e, ~remaining, nbits);
        break;
    }
    }

    if (best_slot == 2) {
        unsigned t = e->origin[2];
        e->origin[2] = e->origin[0];
        e->origin[0] = t;
    } else if (best_slot == 5) {
        unsigned t = e->origin[5];
        e->origin[5] = e->origin[3];
        e->origin[3] = t;
    }
    e->pos += best_len;
    return true;
}

// Promotes `byte` to the front of the LRU (shifting the rest down one).
static void lru_push_front(enc_t *e, unsigned at, uint8_t byte)
{
    for (unsigned j = at; j > 0; j--)
        e->lru[j] = e->lru[j - 1];
    e->lru[0] = byte;
}

// A byte seen among the last 16 distinct literal bytes: a short 6-bit code
// ("10" + a 4-bit LRU index) instead of a full literal.
static bool emit_lru_byte(enc_t *e)
{
    uint8_t byte = e->in[e->pos];
    for (unsigned i = 0; i < e->lru_n; i++) {
        if (e->lru[i] != byte)
            continue;
        lru_push_front(e, i, byte);
        put_bits(e, 0x20 | ((15 - i) & 0xF), 6);
        e->pos += 1;
        return true;
    }
    return false;
}

// A literal byte: 8 bits for zero (the common case in sparse raster data),
// 12 bits otherwise, and pushed onto the LRU for future emit_lru_byte hits.
static void emit_literal(enc_t *e)
{
    uint8_t byte = e->in[e->pos];
    if (e->lru_n < 16)
        e->lru_n++;
    lru_push_front(e, e->lru_n - 1, byte);
    if (byte == 0)
        put_bits(e, 0xFD, 8);
    else
        put_bits(e, 0xD00u | byte, 12);
    e->pos += 1;
}

size_t hiscoa_compress(uint8_t *out, size_t out_cap, const uint8_t *band, unsigned line_bytes, unsigned nlines,
                        bool last_band, const hiscoa_params_t *params)
{
    if (!line_bytes || !out_cap)
        return 0;
    memset(out, 0, out_cap);
    enc_t e = {
        .out = out,
        .out_cap_bits = out_cap * 8,
        .bitpos = 0,
        .in = band,
        .in_len = (size_t)line_bytes * nlines,
        .pos = 0,
        .line_bytes = line_bytes,
        .lru_n = 0,
    };
    e.origin[0] = (unsigned)(params->origin_0 + (int)line_bytes);
    e.origin[1] = 0;      // unused slot (kept so slot numbering matches the sub-command bits)
    e.origin[2] = (unsigned)(params->origin_2 + (int)line_bytes);
    e.origin[3] = (unsigned)params->origin_3;
    e.origin[4] = (unsigned)params->origin_4;
    e.origin[5] = (unsigned)params->origin_5;

    while (e.pos < e.in_len) {
        if (emit_repeat(&e))
            continue;
        if (emit_lru_byte(&e))
            continue;
        emit_literal(&e);
    }

    put_bits(&e, 0xFE, 8);                         // end of band
    put_bits(&e, last_band ? 1 : 0, 2);             // 0 = more bands follow, 1 = last
    put_bits(&e, 0xFFFFFFFFu, 32 - (unsigned)(e.bitpos % 32));   // pad to a 32-bit boundary

    return e.overflow ? 0 : (e.bitpos + 7) / 8;
}
