// HiSCoA band compression of Canon's CAPT 2.x laser printers (LBP2900, LBP3000,
// LBP3010, ...). Ported from captdriver (hiscoa-compress.c, Alexey Galakhov),
// so this file and the CAPT driver are GPLv3 (see COPYING-GPL3).
// Portable C.
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Window parameters of captdriver's default (what Canon's driver sends too):
// where a long repeat may copy from, in bytes behind the current position.
#define HISCOA_PARAM_BYTES 8

// Compresses `nlines` lines of `line_size` bytes (1 bit per pixel, 1 = black)
// into buf (size bytes, at least 2 * line_size * nlines). Returns the length
// in bytes (a multiple of 4), 0 if buf is too small. `last`: the band is the
// last one of the page.
size_t hiscoa_compress_band(uint8_t *buf, size_t size, const uint8_t *band, unsigned line_size, unsigned nlines,
                            int last);

// The 8 bytes of the "set HiSCoA parameters" command (0xD0A4) for the
// parameters hiscoa_compress_band uses. Returns the length (8).
size_t hiscoa_format_params(uint8_t *buf, size_t size);

// The inverse, for tests: decompresses one band into out (out_size bytes).
// Returns the number of bytes written, or (size_t)-1 on a malformed band;
// *consumed = input bytes used, *last = the end-of-band marker's value.
size_t hiscoa_decompress_band(const uint8_t *in, size_t in_size, uint8_t *out, size_t out_size, unsigned line_size,
                              size_t *consumed, int *last);

#ifdef __cplusplus
}
#endif
