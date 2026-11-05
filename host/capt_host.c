// Host-side smoke test for components/canon_capt + the LBP3300/LBP3000
// driver (components/printers/canon_lbp3000.c), built with plain cc, no IDF
// and no real printer: drives the driver's full job/page state machine
// against capt_fake.c's fake CAPT printer.
//
// This can only catch hangs, wrong offsets, and infinite-poll bugs in the
// state machine and the HiSCoA bit-packer's bookkeeping -- it says nothing
// about whether a real LBP3300 would accept the bytes. See README.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "canon_capt.h"
#include "capt_fake.h"
#include "printer_driver.h"

extern const printer_driver_t canon_lbp3000_driver;

static int fake_write(void *ctx, const uint8_t *data, size_t len)
{
    capt_fake_feed((capt_fake_t *)ctx, data, len);
    return 0;
}

static int fake_read(void *ctx, uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    (void)timeout_ms;
    return (int)capt_fake_read((capt_fake_t *)ctx, buf, len);
}

static void fake_set_raw_io(void *ctx, bool on)
{
    (void)ctx;
    (void)on;
}

static void progress(void *ctx, int line, int lines)
{
    (void)ctx;
    if (line == lines || line % 1024 == 0)
        printf("  line %d/%d\n", line, lines);
}

// A synthetic page: alternating black/white rows, enough to exercise both
// the literal-byte and long-repeat-against-the-previous-line paths in
// hiscoa.c. Shorter than the driver's own A4 canvas, so page_end's blank-line
// padding path also runs.
static int run_job(const printer_driver_t *d, printer_io_t *io)
{
    void *job = d->job_begin(d, io, d->resolutions[0], 0);
    if (!job) {
        fprintf(stderr, "job_begin failed\n");
        return 1;
    }
    int width = 2000, height = 300;
    uint8_t *row = malloc((size_t)width);
    int err = d->page_begin(job, width, height, 1) != 0;
    for (int y = 0; !err && y < height; y++) {
        memset(row, (y % 3 == 0) ? 0 : 255, (size_t)width);
        err = d->page_line(job, y, row) != 0;
    }
    free(row);
    if (!err)
        err = d->page_end(job) != 0;
    if (d->job_end(job) != 0)
        err = 1;
    return err;
}

int main(void)
{
    capt_fake_t fake;
    capt_fake_init(&fake);
    printer_io_t io = {
        .write = fake_write, .read = fake_read, .set_raw_io = fake_set_raw_io, .progress = progress, .ctx = &fake};

    clock_t t0 = clock();
    int err = run_job(&canon_lbp3000_driver, &io);
    double secs = (double)(clock() - t0) / CLOCKS_PER_SEC;
    printf("capt_host: job %s, %.3fs host cpu\n", err ? "FAILED" : "completed", secs);

    capt_fake_init(&fake);
    clock_t t1 = clock();
    int err2 = canon_lbp3000_driver.test_page(&canon_lbp3000_driver, &io, canon_lbp3000_driver.resolutions[0],
                                               "TEST") != 0;
    printf("capt_host: test_page %s, %.3fs host cpu\n", err2 ? "FAILED" : "completed",
           (double)(clock() - t1) / CLOCKS_PER_SEC);

    return err || err2 ? 1 : 0;
}
