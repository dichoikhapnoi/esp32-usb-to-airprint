// Canon LBP3300: a USB laser printer, treated as an LBP3000 (its protocol
// clone -- LBP3300 is not itself a model captdriver, the reference Linux
// implementation, registers; community reports are that it accepts the same
// commands). Speaks Canon CAPT (components/canon_capt), not BJL/IVEC: binary,
// strictly request/response, monochrome only.
//
// EXPERIMENTAL, UNVERIFIED ON REAL HARDWARE (see README): written from the
// published CAPT wire format with no LBP3300 available to test against.
// Things a real printer may disagree with: the exact IEEE-1284 MDL string
// below (usb_mdl -- check the firmware's boot log "IEEE 1284 ID:" line and
// fix this if it differs), the unprintable margins, and the HiSCoA-encoded
// raster bytes themselves (components/canon_capt/hiscoa.c).
#include <stdlib.h>
#include <string.h>

#include "canon_capt.h"
#include "hiscoa.h"
#include "printer_driver.h"

#define BAND_LINES 128      // raster lines buffered between CAPT_PRINT_DATA sends

typedef struct {
    printer_io_t io;
    capt_job_t capt;
    int width, height;        // printable area, dots
    int line_bytes;
    uint8_t media_size;        // CAPT media-size code for the job's paper
    int src_w, bytes_pp;       // the current page's source raster shape
    int lines_sent;
    bool in_page;
    uint8_t *band;              // line_bytes * BAND_LINES
    int band_lines;
    uint8_t *comp;
    size_t comp_cap;
} capt_lbp_job_t;

// A deliberately simple 1-D error-diffusion ditherer (left to right only, no
// vertical carry): good enough for a monochrome laser engine without the
// bug surface of full Floyd-Steinberg, which matters here since none of
// this has been checked against real output.
static void dither_line(const capt_lbp_job_t *j, const uint8_t *px, uint8_t *out)
{
    memset(out, 0, j->line_bytes);
    int err = 0;
    for (int x = 0; x < j->width; x++) {
        int gray = 255;
        if (px && x < j->src_w)
            gray = j->bytes_pp == 1 ? px[x]
                                     : (px[3 * x] * 299 + px[3 * x + 1] * 587 + px[3 * x + 2] * 114) / 1000;
        int v = gray + err;
        bool black = v < 128;
        err = v - (black ? 0 : 255);
        if (black)
            out[x >> 3] |= (uint8_t)(0x80 >> (x & 7));
    }
}

static int flush_band(capt_lbp_job_t *j, bool last)
{
    size_t n = hiscoa_compress(j->comp, j->comp_cap, j->band, (unsigned)j->line_bytes, (unsigned)j->band_lines, last,
                               &hiscoa_default_params);
    if (!n)
        return -1;
    j->band_lines = 0;
    return capt_send(&j->capt, CAPT_PRINT_DATA, j->comp, n);
}

static void *job_begin(const printer_driver_t *d, const printer_io_t *io, int dpi, int paper)
{
    if (!io->read || !io->set_raw_io)
        return NULL;     // no binary back channel on this platform path (e.g. a dry run)
    capt_lbp_job_t *j = calloc(1, sizeof(*j));
    if (!j)
        return NULL;
    j->io = *io;
    j->capt.io = &j->io;
    const printer_paper_t *pp = &d->papers[paper];
    int mw = (d->margin_left + d->margin_right) * dpi / 2540;
    int mh = (d->margin_top + d->margin_bottom) * dpi / 2540;
    j->width = pp->width * dpi / 2540 - mw;
    j->height = pp->height * dpi / 2540 - mh;
    j->line_bytes = (j->width + 7) / 8;
    j->capt.width = j->width;
    j->capt.height = j->height;
    j->capt.line_bytes = j->line_bytes;
    j->capt.paper_width = pp->width * dpi / 2540;
    j->capt.paper_height = pp->height * dpi / 2540;
    j->media_size = paper == 1 ? 0x0D : 0x02;      // Letter : A4 (PAPERS[] below)
    j->band = malloc((size_t)j->line_bytes * BAND_LINES);
    j->comp_cap = (size_t)j->line_bytes * BAND_LINES + 256;     // compression only ever shrinks this
    j->comp = malloc(j->comp_cap);
    if (!j->band || !j->comp) {
        free(j->band);
        free(j->comp);
        free(j);
        return NULL;
    }
    io->set_raw_io(io->ctx, true);
    if (capt_job_prologue(&j->capt) < 0) {
        io->set_raw_io(io->ctx, false);
        free(j->band);
        free(j->comp);
        free(j);
        return NULL;
    }
    return j;
}

static int page_begin(void *job, int src_w, int src_h, int bytes_pp)
{
    capt_lbp_job_t *j = job;
    (void)src_h;
    j->src_w = src_w;
    j->bytes_pp = bytes_pp;
    j->lines_sent = 0;
    j->band_lines = 0;
    if (capt_page_prologue(&j->capt, j->media_size) < 0)
        return -1;
    j->in_page = true;
    return 0;
}

static int page_line(void *job, int y, const uint8_t *px)
{
    capt_lbp_job_t *j = job;
    (void)y;
    if (j->capt.err || j->lines_sent >= j->height)
        return j->capt.err ? -1 : 0;
    dither_line(j, px, j->band + (size_t)j->band_lines * j->line_bytes);
    j->band_lines++;
    j->lines_sent++;
    if (j->band_lines == BAND_LINES)
        return flush_band(j, false);
    return 0;
}

static int page_end(void *job)
{
    capt_lbp_job_t *j = job;
    if (j->capt.err)
        return -1;
    while (j->lines_sent < j->height) {
        dither_line(j, NULL, j->band + (size_t)j->band_lines * j->line_bytes);
        j->band_lines++;
        j->lines_sent++;
        if (j->band_lines == BAND_LINES && flush_band(j, false) < 0)
            return -1;
    }
    if (flush_band(j, true) < 0)
        return -1;
    j->in_page = false;
    return capt_page_epilogue(&j->capt);
}

static int job_end(void *job)
{
    capt_lbp_job_t *j = job;
    int ret = 0;
    if (j->in_page && !j->capt.err)
        ret = page_end(j);
    if (ret == 0 && !j->capt.err)
        ret = capt_job_epilogue(&j->capt);
    j->io.set_raw_io(j->io.ctx, false);
    free(j->band);
    free(j->comp);
    free(j);
    return ret;
}

// A geometric check pattern (border + diagonal hatching): no text label,
// unlike the BJL drivers' testpage_t -- that renderer is tied to Canon's
// other raster format. Good enough to confirm the printer accepts a job and
// ejects a sheet; not a toner/registration test chart.
static int test_page(const printer_driver_t *d, const printer_io_t *io, int dpi, const char *label)
{
    (void)label;
    void *job = job_begin(d, io, dpi, 0);
    if (!job)
        return -1;
    capt_lbp_job_t *j = job;
    int ret = page_begin(j, j->width, j->height, 1);
    uint8_t *row = ret == 0 ? malloc((size_t)j->width) : NULL;
    if (ret == 0 && !row)
        ret = -1;
    for (int y = 0; ret == 0 && y < j->height; y++) {
        for (int x = 0; x < j->width; x++) {
            bool border = x < 8 || x >= j->width - 8 || y < 8 || y >= j->height - 8;
            bool hatch = ((x + y) % 64) < 4;
            row[x] = (border || hatch) ? 0 : 255;
        }
        ret = page_line(j, y, row);
        if (io->progress && (y & 63) == 63)
            io->progress(io->ctx, y + 1, j->height);
    }
    free(row);
    if (ret == 0)
        ret = page_end(j);
    if (job_end(j) < 0)
        ret = -1;
    return ret;
}

static const printer_paper_t PAPERS[] = {
    {"A4", "iso_a4_210x297mm", 21000, 29700},
    {"Letter", "na_letter_8.5x11in", 21590, 27940},
};

const printer_driver_t canon_lbp3000_driver = {
    .id = "canon-lbp3300",
    .make_model = "Canon LBP3300",
    .usb_mfg = "Canon",
    .usb_mdl = "LBP3000",        // UNVERIFIED placeholder -- see the file comment above
    .usb_vid = 0x04a9,
    .usb_pid = 0,                 // unknown: matched by usb_mdl instead (printer_driver_match)
    .color = false,
    .scanner = false,
    .resolutions = {600},
    .nresolutions = 1,
    .papers = PAPERS,
    .npapers = 2,
    // unverified guess (no datasheet/hardware to measure): a typical
    // small-laser-engine unprintable border, ~4.2 mm on every edge.
    .margin_left = 423,
    .margin_right = 423,
    .margin_top = 423,
    .margin_bottom = 423,
    .job_begin = job_begin,
    .page_begin = page_begin,
    .page_line = page_line,
    .page_end = page_end,
    .job_end = job_end,
    .test_page = test_page,
};
