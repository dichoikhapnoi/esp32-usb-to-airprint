#include "capt_fake.h"

#include <string.h>

#include "canon_capt.h"

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (uint16_t)p[1] << 8);
}

void capt_fake_init(capt_fake_t *f)
{
    memset(f, 0, sizeof(*f));
}

// Builds f->reply for the just-completed command frame `cmd` (ignoring its
// payload: this fake doesn't validate what was sent, only tracks enough
// engine state for the state machine's polling loops to terminate).
static void reply_to(capt_fake_t *f, uint16_t cmd)
{
    uint8_t payload[2 * CAPT_NSTATUS + 10] = {0};
    size_t payload_len = 2;     // every real reply carries at least this much

    switch (cmd) {
    case CAPT_JOB_BEGIN:
        f->job = f->job ? f->job : 1;
        put_u16(payload + 2, f->job);
        payload_len = 4;
        break;
    case CAPT_PRINT_DATA_END:
        f->page_decoding++;
        f->page_received = f->page_decoding;
        break;
    case CAPT_FIRE:
        f->page_out = f->page_decoding;
        f->page_completed = f->page_decoding;
        break;
    case CAPT_CHKXSTATUS: {
        // 7 status words, all zero: never busy, never uninitialized, buffer
        // never full, paper always loaded -- the fake has no failure modes.
        uint16_t counters[5] = {f->page_decoding, f->page_decoding, f->page_out, f->page_completed,
                                 f->page_received};
        for (int i = 0; i < 5; i++)
            put_u16(payload + 2 * CAPT_NSTATUS + 2 * i, counters[i]);
        payload_len = 2 * CAPT_NSTATUS + 10;
        break;
    }
    default:
        break;
    }

    size_t frame_len = 4 + payload_len;
    put_u16(f->reply, cmd);
    put_u16(f->reply + 2, (uint16_t)frame_len);
    memcpy(f->reply + 4, payload, payload_len);
    f->reply_len = frame_len;
    f->reply_pos = 0;
}

void capt_fake_feed(capt_fake_t *f, const uint8_t *data, size_t len)
{
    while (len) {
        size_t room = sizeof(f->in_buf) - f->in_len;
        size_t take = len < room ? len : room;
        memcpy(f->in_buf + f->in_len, data, take);
        f->in_len += take;
        data += take;
        len -= take;
        if (f->in_len >= 4) {
            uint16_t size = get_u16(f->in_buf + 2);
            if (size >= 4 && f->in_len >= size) {
                reply_to(f, get_u16(f->in_buf));
                f->in_len = 0;
            }
        }
        if (f->in_len >= sizeof(f->in_buf))
            f->in_len = 0;      // malformed/oversized frame: drop and resync
    }
}

size_t capt_fake_read(capt_fake_t *f, uint8_t *buf, size_t len)
{
    size_t avail = f->reply_len - f->reply_pos;
    size_t n = len < avail ? len : avail;
    if (n)
        memcpy(buf, f->reply + f->reply_pos, n);
    f->reply_pos += n;
    return n;
}
