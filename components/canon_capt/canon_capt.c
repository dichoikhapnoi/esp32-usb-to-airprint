#include "canon_capt.h"

#include <string.h>
#include <time.h>

#include "hiscoa.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
#include <unistd.h>
#endif

#define LO(x) ((uint8_t)((x) & 0xFF))
#define HI(x) ((uint8_t)(((x) >> 8) & 0xFF))
#define WORD(lo, hi) ((uint16_t)((uint8_t)(lo) | (uint16_t)(uint8_t)(hi) << 8))

static void capt_sleep_ms(int ms)
{
#ifdef ESP_PLATFORM
    vTaskDelay(pdMS_TO_TICKS(ms > 0 ? ms : 1));
#else
    usleep((unsigned)(ms > 0 ? ms : 1) * 1000);
#endif
}

int capt_send(capt_job_t *j, uint16_t cmd, const void *payload, size_t len)
{
    uint8_t head[4] = {LO(cmd), HI(cmd), LO(len + 4), HI(len + 4)};
    if (j->io->write(j->io->ctx, head, sizeof(head)) < 0)
        return -1;
    if (len && j->io->write(j->io->ctx, payload, len) < 0)
        return -1;
    return 0;
}

// Reads one reply frame: a 4-byte header (cmd, declared size), then payload,
// continuing past 64-byte block boundaries until the declared size is
// reached (the printer pauses at each 64-byte boundary short of the full
// reply unless that boundary is the end).
static int capt_recv(capt_job_t *j, uint16_t cmd, uint8_t *out, size_t out_cap, size_t *out_len)
{
    static uint8_t buf[256];       // CAPT replies are small (status is ~20 bytes)
    size_t have = 0;
    for (;;) {
        if (!j->io->read || have + 64 > sizeof(buf))
            return -1;
        int n = j->io->read(j->io->ctx, buf + have, have == 0 ? 6 : 64, 5000);
        if (n <= 0)
            return -1;
        have += (size_t)n;
        if (have < 4)
            continue;
        uint16_t got_cmd = WORD(buf[0], buf[1]);
        uint16_t size = WORD(buf[2], buf[3]);
        if (got_cmd != cmd)
            return -1;
        if (have >= size)
            break;
        if (have % 64 != 6 && have != 4)       // not at a continuation point: malformed
            return -1;
    }
    size_t payload = have - 4;
    if (out && payload) {
        if (payload > out_cap)
            payload = out_cap;
        memcpy(out, buf + 4, payload);
    }
    if (out_len)
        *out_len = have - 4;
    return 0;
}

// reply_len: in = capacity of `reply` (ignored if reply is NULL), out =
// actual payload length received.
int capt_sendrecv(capt_job_t *j, uint16_t cmd, const void *payload, size_t len, uint8_t *reply, size_t *reply_len)
{
    if (j->err)
        return -1;
    size_t cap = reply && reply_len ? *reply_len : 0;
    if (capt_send(j, cmd, payload, len) < 0 || capt_recv(j, cmd, reply, cap, reply_len) < 0) {
        j->err = true;
        return -1;
    }
    return 0;
}

int capt_poll_status(capt_job_t *j)
{
    uint8_t buf[2 * CAPT_NSTATUS + 10];
    size_t len = sizeof(buf);
    if (capt_sendrecv(j, CAPT_CHKXSTATUS, NULL, 0, buf, &len) < 0 || len < 2 * CAPT_NSTATUS)
        return -1;
    for (int i = 0; i < CAPT_NSTATUS; i++)
        j->status.word[i] = WORD(buf[2 * i], buf[2 * i + 1]);
    // page counters follow the status words in the extended reply, when present
    size_t p = 2 * CAPT_NSTATUS;
    uint16_t *counters[] = {&j->status.page_decoding, &j->status.page_printing, &j->status.page_out,
                             &j->status.page_completed, &j->status.page_received};
    for (int i = 0; i < 5 && p + 1 < len; i++, p += 2)
        *counters[i] = WORD(buf[p], buf[p + 1]);
    return 0;
}

int capt_wait_clear(capt_job_t *j, uint32_t flag, uint32_t timeout_ms)
{
    uint32_t waited = 0;
    for (;;) {
        if (capt_poll_status(j) < 0)
            return -1;
        if (!capt_flag(&j->status, flag))
            return 0;
        if (waited >= timeout_ms) {
            j->err = true;      // a stuck printer is a failed job, not a retryable one
            return -1;
        }
        capt_sleep_ms(200);
        waited += 200;
    }
}

int capt_wait_counter(capt_job_t *j, const uint16_t *counter, uint16_t target, uint32_t timeout_ms)
{
    uint32_t waited = 0;
    for (;;) {
        if (capt_poll_status(j) < 0)
            return -1;
        if (*counter == target)
            return 0;
        if (waited >= timeout_ms) {
            j->err = true;
            return -1;
        }
        capt_sleep_ms(200);
        waited += 200;
    }
}

// The ~72-byte job-ticket command (CAPT_JOB_SETUP): identifies the job to
// the spooler-less printer (fg = phase: 1 start, 2/6 page fired, 4 end).
static int send_job_ticket(capt_job_t *j, uint8_t fg, uint16_t page)
{
    uint8_t buf[32 + 40];
    memset(buf, 0, sizeof(buf));
    time_t now = time(NULL);
    struct tm tmv;
#ifdef ESP_PLATFORM
    localtime_r(&now, &tmv);
#else
    struct tm *tp = localtime(&now);
    tmv = tp ? *tp : (struct tm){0};
#endif
    buf[4] = LO(page);
    buf[5] = HI(page);
    buf[16] = fg;
    buf[17] = 0x01;
    buf[18] = LO(j->job);
    buf[19] = HI(j->job);
    buf[20] = 0xC4;
    buf[21] = 0xFF;
    buf[22] = 0x88;
    buf[23] = 0xFF;
    buf[24] = LO(tmv.tm_year);
    buf[25] = HI(tmv.tm_year);
    buf[26] = (uint8_t)tmv.tm_mon;
    buf[27] = (uint8_t)tmv.tm_mday;
    buf[28] = (uint8_t)tmv.tm_hour;
    buf[29] = (uint8_t)tmv.tm_min;
    buf[30] = (uint8_t)tmv.tm_sec;
    buf[31] = 0x01;
    return capt_sendrecv(j, CAPT_JOB_SETUP, buf, sizeof(buf), NULL, NULL);
}

int capt_job_prologue(capt_job_t *j)
{
    uint8_t reply[8];
    size_t len = sizeof(reply);
    static const uint8_t job_begin_magic[8] = {0, 0, 0x1E, 0, 0, 0, 0, 0};
    static const uint8_t gpio_init[12] = {0};
    static const uint8_t lbp3000_setup[2] = {0, 0};

    if (capt_sendrecv(j, CAPT_IDENT, NULL, 0, NULL, NULL) < 0)
        return -1;
    capt_sleep_ms(1000);
    if (capt_poll_status(j) < 0)
        return -1;
    if (capt_sendrecv(j, CAPT_START_0, NULL, 0, NULL, NULL) < 0)
        return -1;
    if (capt_sendrecv(j, CAPT_JOB_BEGIN, job_begin_magic, sizeof(job_begin_magic), reply, &len) < 0 || len < 4)
        return -1;
    j->job = WORD(reply[2], reply[3]);
    if (capt_sendrecv(j, CAPT_GPIO, gpio_init, sizeof(gpio_init), NULL, NULL) < 0)
        return -1;
    if (capt_wait_clear(j, CAPT_FL_BUSY, 15000) < 0)
        return -1;
    if (send_job_ticket(j, 1, 0) < 0)
        return -1;
    // the LBP3000 clone has one extra setup command the LBP2900 doesn't
    if (capt_sendrecv(j, CAPT_LBP3000_SETUP_0, lbp3000_setup, sizeof(lbp3000_setup), NULL, NULL) < 0)
        return -1;
    return capt_wait_clear(j, CAPT_FL_BUSY, 15000);
}

int capt_job_epilogue(capt_job_t *j)
{
    if (capt_wait_counter(j, &j->status.page_completed, j->status.page_decoding, 30000) < 0)
        return -1;
    if (send_job_ticket(j, 4, j->status.page_completed) < 0)
        return -1;
    uint8_t jbuf[2] = {LO(j->job), HI(j->job)};
    return capt_sendrecv(j, CAPT_JOB_END, jbuf, sizeof(jbuf), NULL, NULL);
}

int capt_page_prologue(capt_job_t *j, uint8_t media_size)
{
    if (!j->uninit_done &&
        (capt_flag(&j->status, CAPT_FL_UNINIT1) || capt_flag(&j->status, CAPT_FL_UNINIT2))) {
        static const uint8_t upload2[16] = {0xEE, 0xDB, 0xEA, 0xAD};
        if (capt_sendrecv(j, CAPT_START_1, NULL, 0, NULL, NULL) < 0 ||
            capt_sendrecv(j, CAPT_START_2, NULL, 0, NULL, NULL) < 0 ||
            capt_sendrecv(j, CAPT_START_3, NULL, 0, NULL, NULL) < 0 ||
            capt_wait_clear(j, CAPT_FL_BUSY, 15000) < 0 ||
            capt_sendrecv(j, CAPT_UPLOAD_2, upload2, sizeof(upload2), NULL, NULL) < 0 ||
            capt_wait_clear(j, CAPT_FL_BUSY, 15000) < 0)
            return -1;
        j->uninit_done = true;
    }
    if (capt_wait_clear(j, CAPT_FL_BUFFERFULL, 30000) < 0)
        return -1;

    // media_type 0 (plain paper) -> fuser mode 1, the only case this driver
    // uses (no thick paper / transparency / envelope support yet).
    int margin_h = 0, margin_w = 0, media_type = 0, media_adapt = 0, ink_k = 0, air = 0x02, save = 0, fm = 0x01;
    uint8_t page[40] = {
        0x00, 0x00, 0x30, 0x2A, media_size, 0x00, 0x00, 0x00,
        (uint8_t)ink_k, 0x1C, 0x1C, 0x1C, (uint8_t)media_type, (uint8_t)media_adapt, 0x04, 0x00,
        0x01, 0x01, (uint8_t)air, (uint8_t)save, 0x00, 0x00,
        LO(margin_h), HI(margin_h),
        LO(margin_w), HI(margin_w),
        LO(j->line_bytes), HI(j->line_bytes),
        LO(j->height), HI(j->height),
        LO(j->paper_width), HI(j->paper_width),
        LO(j->paper_height), HI(j->paper_height),
        0x00, 0x00, (uint8_t)fm, 0x00, 0x00, 0x00,
    };
    // CAPT_SET_PARMS bundles its sub-commands back to back in one frame
    // (page parameters, HiSCoA parameters, two empty markers).
    uint8_t bundle[4 + (4 + sizeof(page)) + (4 + 8) + 4 + 4];
    size_t p = 0;
#define PUT_CMD(c, data, n)                                                                                        \
    do {                                                                                                          \
        bundle[p + 0] = LO(c);                                                                                     \
        bundle[p + 1] = HI(c);                                                                                     \
        bundle[p + 2] = LO((n) + 4);                                                                               \
        bundle[p + 3] = HI((n) + 4);                                                                               \
        if ((n))                                                                                                   \
            memcpy(bundle + p + 4, (data), (n));                                                                   \
        p += 4 + (n);                                                                                               \
    } while (0)
    p = 4;      // room for the wrapper header, filled in below
    PUT_CMD(CAPT_SET_PARM_PAGE, page, sizeof(page));
    uint8_t hiscoa[8];
    hiscoa_format_params(hiscoa, sizeof(hiscoa), &hiscoa_default_params);
    PUT_CMD(CAPT_SET_PARM_HISCOA, hiscoa, sizeof(hiscoa));
    PUT_CMD(CAPT_SET_PARM_1, NULL, 0);
    PUT_CMD(CAPT_SET_PARM_2, NULL, 0);
#undef PUT_CMD
    bundle[0] = LO(CAPT_SET_PARMS);
    bundle[1] = HI(CAPT_SET_PARMS);
    bundle[2] = LO(p);
    bundle[3] = HI(p);
    if (j->io->write(j->io->ctx, bundle, p) < 0)
        return -1;
    return 0;
}

int capt_page_epilogue(capt_job_t *j)
{
    if (capt_send(j, CAPT_PRINT_DATA_END, NULL, 0) < 0)
        return -1;
    if (capt_wait_counter(j, &j->status.page_received, j->status.page_decoding, 30000) < 0)
        return -1;
    if (send_job_ticket(j, 2, j->status.page_decoding) < 0)
        return -1;
    if (capt_wait_clear(j, CAPT_FL_BUSY, 15000) < 0)
        return -1;
    uint8_t buf[2] = {LO(j->status.page_decoding), HI(j->status.page_decoding)};
    if (capt_sendrecv(j, CAPT_FIRE, buf, sizeof(buf), NULL, NULL) < 0)
        return -1;
    if (capt_wait_clear(j, CAPT_FL_BUSY, 15000) < 0)
        return -1;
    if (send_job_ticket(j, 6, j->status.page_decoding) < 0)
        return -1;
    return capt_wait_counter(j, &j->status.page_out, j->status.page_decoding, 60000);
}
