idf_component_register(SRCS "airprint.c" "ipp.c" "raster.c" "events.c" "printer_driver.c" "shared_buf.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "."
                       REQUIRES pthread lwip log)
