// A minimal in-process fake CAPT printer: just enough protocol to let
// canon_capt's job/page state machine run to completion without a real
// LBP3300 attached (MODE=dev capture, and host/capt_host.c). It is not a
// correctness oracle for the wire bytes (compressed raster, page-ticket
// fields): it only answers the handful of request/response commands the
// state machine needs to see progress on (job id, status/page counters),
// so it can catch hangs and logic bugs, never whether a real printer would
// accept the data.
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t in_buf[96];
    size_t in_len;        // bytes of the command frame assembled so far

    uint8_t reply[32];
    size_t reply_len, reply_pos;   // drained by capt_fake_read

    uint16_t job;
    uint16_t page_decoding, page_received, page_out, page_completed;
} capt_fake_t;

void capt_fake_init(capt_fake_t *f);
// Feed bytes as the driver wrote them (any split across calls is fine: a
// command's 4-byte header and payload typically arrive separately).
void capt_fake_feed(capt_fake_t *f, const uint8_t *data, size_t len);
// Drains the reply to the most recently completed command, up to len bytes
// per call (mimicking one USB transfer). 0 once exhausted.
size_t capt_fake_read(capt_fake_t *f, uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
