# Printer drivers (printer_driver.h): one source file per printer family.
# canon_capt.c, capt.c and hiscoa.c are GPLv3 (ported from captdriver).
idf_component_register(SRCS "printers.c" "canon_pixma.c" "canon_capt.c" "capt.c" "hiscoa.c"
                       INCLUDE_DIRS "."
                       REQUIRES airprint canon_bjl)
