// HiSCoA: the band-local raster compression Canon CAPT printers (LBP2900/
// LBP3000 family) expect for CAPT_PRINT_DATA. A custom LZ-like bit stream:
// long back-reference repeats against a handful of fixed "origin" distances
// (the most useful being one scanline back, for vertical runs), a 16-entry
// LRU of recently seen bytes for short repeats, and literal/zero bytes,
// XOR-obfuscated with 0x43 and 32-bit padded at the end of each band.
//
// Reimplemented from the published wire format (not derived from any GPL
// codebase: see components/canon_capt/canon_capt.h for the license note).
// EXPERIMENTAL: unverified against real hardware or a reference decoder.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int origin_0, origin_2, origin_3, origin_4, origin_5;
} hiscoa_params_t;

// The values Canon's own driver uses for every model in this family.
extern const hiscoa_params_t hiscoa_default_params;

// The 8-byte CAPT_SET_PARM_HISCOA payload for `params`.
size_t hiscoa_format_params(uint8_t *buf, size_t cap, const hiscoa_params_t *params);

// Compresses one band: `nlines` rows of `line_bytes` packed 1bpp bytes each
// (band = line_bytes * nlines contiguous bytes), ending with HiSCoA's
// end-of-band marker and padding to a 32-bit boundary. `last_band` marks the
// last band of the page (a different end-marker subtype). Returns bytes
// written, 0 if `out` is too small.
size_t hiscoa_compress(uint8_t *out, size_t out_cap, const uint8_t *band, unsigned line_bytes, unsigned nlines,
                        bool last_band, const hiscoa_params_t *params);

#ifdef __cplusplus
}
#endif
