// Canon CAPT 2.x command layer (LBP2900, LBP3000, LBP3010, LBP3300, ...): a
// command is [cmd lo, cmd hi, size lo, size hi] + parameters, the printer
// answers most of them with the same header + data. Ported from captdriver
// (capt-command.c, capt-status.c; Copyright (C) 2013 Alexey Galakhov), so
// GPLv3 (see COPYING-GPL3). Portable C: the bytes go through printer_io_t.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "printer_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

enum capt_command {
    CAPT_NOP = 0xA0A0,
    CAPT_CHKJOBSTAT = 0xA0A1,
    CAPT_CHKXSTATUS = 0xA0A8,
    CAPT_IEEE_IDENT = 0xA1A0,
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
    CAPT_CHKSTATUS = 0xE0A0,
    CAPT_START_2 = 0xE0A2,
    CAPT_START_1 = 0xE0A3,
    CAPT_START_3 = 0xE0A4,
    CAPT_UPLOAD_2 = 0xE0A5,
    CAPT_LBP3000_SETUP_0 = 0xE0A6,
    CAPT_FIRE = 0xE0A7,
    CAPT_JOB_END = 0xE0A9,
    CAPT_LBP6000_SETUP_0 = 0xE0BA,
    CAPT_JOB_SETUP = 0xE1A1,
    CAPT_GPIO = 0xE1A2,
};

// Status words, as captdriver names them: (word index << 16) | bit mask
#define CAPT_FL(w, bit) (((w) << 16) | (1u << (bit)))
enum capt_flag {
    CAPT_FL_READY1 = CAPT_FL(0, 15),
    CAPT_FL_READY2 = CAPT_FL(0, 12),
    CAPT_FL_JOBSTAT_CHNG = CAPT_FL(0, 9),
    CAPT_FL_XSTATUS_CHNG = CAPT_FL(0, 8),
    CAPT_FL_BUSY = CAPT_FL(0, 7),
    CAPT_FL_UNINIT1 = CAPT_FL(0, 5),
    CAPT_FL_UNINIT2 = CAPT_FL(0, 4),
    CAPT_FL_BUFFERFULL = CAPT_FL(0, 2),
    CAPT_FL_NOPAPER1 = CAPT_FL(0, 1),
    CAPT_FL_PROCESSING = CAPT_FL(0, 0),
    CAPT_FL_NOPAPER2 = CAPT_FL(1, 14),
    CAPT_FL_PROCESSING1 = CAPT_FL(1, 7),
    CAPT_FL_BUTTON = CAPT_FL(1, 5),
    CAPT_FL_PRINTING = CAPT_FL(1, 2),
    CAPT_FL_POWERUP = CAPT_FL(1, 0),
    CAPT_FL_nERROR = CAPT_FL(2, 7),
    CAPT_FL_BUTTON1 = CAPT_FL(2, 8),
    CAPT_FL_POWERUP1 = CAPT_FL(3, 12),
    CAPT_FL_BUTTON_ON = CAPT_FL(4, 0),
};

typedef struct {
    uint16_t status[7];
    uint16_t page_decoding;     // pages the printer has started to decode / print / eject / finish
    uint16_t page_printing;
    uint16_t page_out;
    uint16_t page_completed;
    uint16_t page_received;
} capt_status_t;

#define CAPT_FLAG(st, f) (((st)->status[(f) >> 16] & ((f) & 0xFFFF)) != 0)

typedef struct {
    const printer_io_t *io;
    capt_status_t st;           // the last status the printer reported
    uint8_t rx[1536];           // the last reply (header included)
    size_t rx_len;
    uint8_t mbuf[256];          // a command made of several (capt_multi_*)
    size_t mlen;
} capt_t;

// results: 0 ok, -1 the printer is gone / job cancelled / bad reply, -2 timeout

void capt_init(capt_t *c, const printer_io_t *io);
// One command, no reply expected (print data).
int capt_send(capt_t *c, uint16_t cmd, const void *data, size_t len);
// A command and its reply. The reply's data (after the 4-byte header) is
// copied to reply (at most *reply_len bytes); *reply_len = the data length.
int capt_sendrecv(capt_t *c, uint16_t cmd, const void *data, size_t len, uint8_t *reply, size_t *reply_len);
void capt_multi_begin(capt_t *c, uint16_t cmd);
void capt_multi_add(capt_t *c, uint16_t cmd, const void *data, size_t len);
int capt_multi_send(capt_t *c);

// CHKSTATUS (and CHKXSTATUS when it says the extended status changed)
int capt_get_status(capt_t *c);
// CHKXSTATUS only (LBP3010 and later)
int capt_get_xstatus(capt_t *c);
// Polls every second while the printer is busy; max_s seconds.
int capt_wait_ready(capt_t *c, int max_s);
int capt_wait_xready(capt_t *c, int max_s);

void capt_hex(const char *what, const uint8_t *p, size_t n);

// Debugging a printer whose page parameters are not known (canon_capt.c): replace
// byte `offset` (0..39) of the 40-byte page parameters by `value`; value < 0
// removes every override. Used by the debug server (POST /capt).
void canon_capt_set_override(int offset, int value);

#ifdef __cplusplus
}
#endif
