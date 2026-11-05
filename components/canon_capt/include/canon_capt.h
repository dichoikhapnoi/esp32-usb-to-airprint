// Canon CAPT (Canon Advanced Printing Technology): the binary, bidirectional
// protocol spoken by USB laser printers such as the LBP2900/LBP3000 family
// (which the LBP3300 is treated as, see canon_lbp3000.c). Independent
// reimplementation from the published wire format (command codes, byte
// layouts, state machine), not derived from any particular codebase: this
// project is MIT-licensed, the reference CAPT implementations are GPL.
//
// Unlike Canon's BJL/IVEC (components/canon_bjl), CAPT is strictly
// synchronous: every command's reply must be read before the next command
// is sent, and the printer's progress (page decoding/printing/out/completed
// counters) only advances by polling CAPT_CHKXSTATUS. EXPERIMENTAL: written
// without access to the real hardware, see README.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "printer_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAPT_CHKJOBSTAT = 0xA0A1,
    CAPT_CHKXSTATUS = 0xA0A8,

    CAPT_IDENT = 0xA1A1,

    CAPT_JOB_BEGIN = 0xA2A0,

    CAPT_START_0 = 0xA3A2,

    CAPT_PRINT_DATA = 0xC0A0,
    CAPT_PRINT_DATA_END = 0xC0A4,

    CAPT_SET_PARM_PAGE = 0xD0A0,
    CAPT_SET_PARM_1 = 0xD0A1,
    CAPT_SET_PARM_2 = 0xD0A2,
    CAPT_SET_PARM_HISCOA = 0xD0A4,
    CAPT_SET_PARMS = 0xD0A9,

    CAPT_START_2 = 0xE0A2,
    CAPT_START_1 = 0xE0A3,
    CAPT_START_3 = 0xE0A4,
    CAPT_UPLOAD_2 = 0xE0A5,
    CAPT_LBP3000_SETUP_0 = 0xE0A6,
    CAPT_FIRE = 0xE0A7,
    CAPT_JOB_END = 0xE0A9,

    CAPT_JOB_SETUP = 0xE1A1,
    CAPT_GPIO = 0xE1A2,
} capt_cmd_t;

// 7 status words (CAPT_CHKXSTATUS reply), each tested for specific bits.
#define CAPT_NSTATUS 7
typedef struct {
    uint16_t word[CAPT_NSTATUS];
    uint16_t page_decoding, page_printing, page_out, page_completed, page_received;
} capt_status_t;

#define CAPT_FLAG(word_idx, bit) ((uint32_t)(word_idx) << 16 | (1u << (bit)))
enum {
    CAPT_FL_BUSY = CAPT_FLAG(0, 7),
    CAPT_FL_UNINIT1 = CAPT_FLAG(0, 5),
    CAPT_FL_UNINIT2 = CAPT_FLAG(0, 4),
    CAPT_FL_BUFFERFULL = CAPT_FLAG(0, 2),
    CAPT_FL_NOPAPER1 = CAPT_FLAG(0, 1),
    CAPT_FL_NOPAPER2 = CAPT_FLAG(1, 14),
    CAPT_FL_PROCESSING1 = CAPT_FLAG(1, 7),
    CAPT_FL_PRINTING = CAPT_FLAG(1, 2),
};
static inline bool capt_flag(const capt_status_t *s, uint32_t flag)
{
    return (s->word[flag >> 16] & (flag & 0xFFFFu)) != 0;
}

// A running job: owns the printer_io_t for its lifetime (job_begin..job_end).
typedef struct {
    const printer_io_t *io;
    uint16_t job;             // assigned by CAPT_JOB_BEGIN
    capt_status_t status;     // last poll
    bool err;                 // a command failed or timed out: abort the job
    bool uninit_done;         // the one-time START_1/2/3 + UPLOAD_2 ran
    int width, height;        // printable area, dots at 600 dpi
    int line_bytes;           // 1bpp packed row size
    int paper_width, paper_height;  // full sheet, dots at 600 dpi
} capt_job_t;

// Sends one framed command (2-byte LE cmd, 2-byte LE size incl. header,
// payload); -1 on write failure.
int capt_send(capt_job_t *j, uint16_t cmd, const void *payload, size_t len);
// Sends, then reads the reply (continuing past 64-byte block boundaries
// until the declared size is reached); reply/reply_len may be NULL to
// discard it. -1 on failure (write, read, or a malformed/mismatched reply).
int capt_sendrecv(capt_job_t *j, uint16_t cmd, const void *payload, size_t len, uint8_t *reply, size_t *reply_len);

// Refreshes j->status from CAPT_CHKXSTATUS. -1 on failure.
int capt_poll_status(capt_job_t *j);
// Polls until `flag` is clear (e.g. CAPT_FL_BUSY) or timeout_ms elapses.
// -1 on failure/timeout.
int capt_wait_clear(capt_job_t *j, uint32_t flag, uint32_t timeout_ms);
// Polls until *counter == target or timeout_ms elapses (a page-progress wait,
// e.g. &j->status.page_out). -1 on failure/timeout.
int capt_wait_counter(capt_job_t *j, const uint16_t *counter, uint16_t target, uint32_t timeout_ms);

// --- job/page lifecycle, built from the above -------------------------

// CAPT_IDENT, initial status, CAPT_START_0, CAPT_JOB_BEGIN, the job-setup
// ticket, CAPT_LBP3000_SETUP_0, wait-ready. 0 = ok.
int capt_job_prologue(capt_job_t *j);
// Poll page_completed, job-setup "end" ticket, CAPT_JOB_END.
int capt_job_epilogue(capt_job_t *j);
// media_size: 0x02 = A4, 0x0D = Letter (capt_cmd.h media size codes). Runs
// the one-time uninit sequence if needed, waits out BUFFERFULL, sends
// CAPT_SET_PARMS (page + HiSCoA params). 0 = ok.
int capt_page_prologue(capt_job_t *j, uint8_t media_size);
// CAPT_PRINT_DATA_END, waits page_received==page_decoding, fires the page
// (job-setup ticket + CAPT_FIRE), waits page_out==page_decoding. 0 = ok.
int capt_page_epilogue(capt_job_t *j);

#ifdef __cplusplus
}
#endif
