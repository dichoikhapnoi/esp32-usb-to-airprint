#include "airprint_esp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "airprint.h"
#include "canon_bjl.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "events.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "local_config.h"
#include "mdns.h"
#include "print_job.h"
#include "printer_driver.h"
#include "shared_buf.h"
#include "status_led.h"
#include "usb_printer.h"
#include "webui.h"

static const char *TAG = "airprint_esp";

#define IPP_PORT 631
#define WEB_PORT 80             // the web UI for browsers (also on 631)
#define CANCEL_FORCE_MS 8000    // graceful cancel didn't finish: abort the USB transfer

static airprint_config_t s_cfg;
static char s_uuid[40];
static char s_hostname[64];
static SemaphoreHandle_t s_identity_lock;   // the announced model (airprint_esp_driver_changed)
static char s_device_id[2][128];            // the current one and the one before (still being read)
static int s_device_id_cur;
static void (*s_on_job_state)(bool);
static esp_timer_handle_t s_cancel_timer;
static volatile bool s_job_running;

// The shared buffer takes all the PSRAM but this much, left for everything
// else (raster buffers, ...)
#define PSRAM_RESERVE (512 * 1024)
#define SHARED_BUF_MIN (1024 * 1024)

// MODE=dev: no printer; jobs are converted, checksummed and kept in the
// shared buffer.
static uint8_t *s_cap;
static size_t s_cap_size, s_cap_len;
static bool s_cap_truncated;
static uint32_t s_dev_crc;
static int64_t s_dev_t0;

// ---------------------------------------------------------------------------
// printer status

static void add_reason(airprint_status_t *st, const char *reason)
{
    if (st->nreasons < (int)(sizeof(st->reasons) / sizeof(st->reasons[0])))
        st->reasons[st->nreasons++] = reason;
}

static void get_status(void *ctx, airprint_status_t *st)
{
    st->connected = usb_printer_connected();
    if (!st->connected)
        return;
    st->busy = usb_printer_job_active() && !s_job_running;     // local test page

    // the printer's own status (reasons, message, ink) as its driver reads it
    usb_printer_status_t cs;
    bool have = usb_printer_get_status(&cs);
    const printer_driver_t *drv = print_job_driver();
    if (drv->status)
        drv->status(have ? cs.state : NULL, have ? cs.detail : NULL, have ? cs.data : NULL, st);
    const char *d = have ? cs.detail : "";
    // USB printer class port status: paper empty
    uint8_t port = 0;
    if (!s_job_running && usb_printer_port_status(&port) == ESP_OK && (port & 0x20) &&
        !strstr(d, "MediaEmpty")) {
        add_reason(st, "media-empty-error");
        snprintf(st->message, sizeof(st->message), "Out of paper");
    }
}

// ---------------------------------------------------------------------------
// job hooks

static int job_open(void *ctx)
{
    esp_err_t err = usb_printer_job_begin();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "printer not available: %s", esp_err_to_name(err));
        return -1;
    }
    s_job_running = true;
    return 0;
}

static int job_write(void *ctx, const uint8_t *data, size_t len)
{
    return usb_printer_write(data, len) == ESP_OK ? 0 : -1;
}

static int job_sync(void *ctx, const char *what)
{
    return print_job_sync(what);
}

// The job's big buffer for two-sided printing (the held front side): the shared PSRAM buffer, taken
// when a driver first asks and given back when the job ends.
static uint8_t *s_job_big;
static size_t s_job_big_size;
static bool s_job_big_taken;

static uint8_t *job_buffer(void *ctx, size_t *size)
{
    if (!s_job_big_taken) {
        s_job_big_taken = true;
        s_job_big = sbuf_acquire(SBUF_PRINT, &s_job_big_size);
        ESP_LOGI(TAG, "two-sided printing: %u KB buffer%s", (unsigned)(s_job_big_size / 1024), s_job_big ? "" : " (busy)");
    }
    if (!s_job_big)
        return NULL;
    *size = s_job_big_size;
    return s_job_big;
}

// for command/reply printers (CAPT)
static int job_flush(void *ctx)
{
    return usb_printer_flush() == ESP_OK ? 0 : -1;
}

static int job_read(void *ctx, uint8_t *buf, size_t max, uint32_t timeout_ms)
{
    return usb_printer_read(buf, max, timeout_ms);
}

static int job_delay(void *ctx, uint32_t ms)
{
    return usb_printer_delay(ms);
}

static void job_close(void *ctx, bool ok)
{
    usb_printer_job_end();      // sends the job's last bytes: a cancel may still need the timer
    esp_timer_stop(s_cancel_timer);
    if (s_job_big)
        sbuf_release(SBUF_PRINT, 0);
    s_job_big = NULL;
    s_job_big_taken = false;
    s_job_running = false;
    if (!ok && usb_printer_connected()) {
        ESP_LOGW(TAG, "job failed, resetting printer");
        usb_printer_soft_reset();
    }
}

static void cancel_timeout(void *arg)
{
    if (s_job_running) {
        ESP_LOGW(TAG, "job did not stop, aborting the transfer");
        usb_printer_cancel();
    }
}

// The pipeline stops at the next raster line and ejects the sheet. If the
// printer is stuck (no paper), writes block: force-abort after a while.
static void job_cancel(void *ctx)
{
    esp_timer_stop(s_cancel_timer);
    esp_timer_start_once(s_cancel_timer, CANCEL_FORCE_MS * 1000ULL);
}

static void job_state(void *ctx, bool printing)
{
    if (s_on_job_state)
        s_on_job_state(printing);
}

static void identify(void *ctx)
{
    ESP_LOGI(TAG, "Identify-Printer");
    status_led_identify();
}

// ---------------------------------------------------------------------------
// MODE=dev backend: a printer that accepts everything and prints nothing

static void dev_get_status(void *ctx, airprint_status_t *st)
{
    st->connected = true;
    snprintf(st->message, sizeof(st->message), "Dev mode: jobs are not printed");
    st->nsupplies = 2;
    st->supply[0].name = "Black";
    st->supply[0].color = "#000000";
    st->supply[0].level = 80;
    st->supply[1].name = "Color";
    st->supply[1].color = "#00FFFF#FF00FF#FFFF00";
    st->supply[1].level = 30;
}

static int dev_job_open(void *ctx)
{
    if (s_job_running)
        return -1;
    s_cap = sbuf_acquire(SBUF_PRINT, &s_cap_size);     // NULL: CRC only
    s_cap_len = 0;
    s_cap_truncated = false;
    s_dev_crc = 0;
    s_dev_t0 = esp_timer_get_time();
    s_job_running = true;
    if (s_cap)
        ESP_LOGI(TAG, "dev job started (capture buffer %u KB)", (unsigned)(s_cap_size / 1024));
    else
        ESP_LOGI(TAG, "dev job started (capture buffer in use or missing: CRC only)");
    return 0;
}

static int dev_job_write(void *ctx, const uint8_t *data, size_t len)
{
    s_dev_crc = bjl_crc32(s_dev_crc, data, len);
    if (s_cap && s_cap_len + len <= s_cap_size) {
        memcpy(s_cap + s_cap_len, data, len);
        s_cap_len += len;
    } else {
        s_cap_truncated = true;
        s_cap_len += s_cap ? 0 : len;
    }
    return 0;
}

static int dev_job_sync(void *ctx, const char *what)
{
    ESP_LOGI(TAG, "dev: %s (would wait for the printer here)", what);
    return 0;
}

static void dev_job_close(void *ctx, bool ok)
{
    if (s_cap)
        sbuf_release(SBUF_PRINT, s_cap_truncated ? 0 : s_cap_len);
    s_job_running = false;
    ESP_LOGI(TAG, "DEV job done: %u bytes of printer data, crc32 %08lx, %lld ms%s%s", (unsigned)s_cap_len,
             (unsigned long)s_dev_crc, (long long)((esp_timer_get_time() - s_dev_t0) / 1000),
             s_cap_truncated ? ", capture truncated" : "", ok ? "" : ", job failed");
    if (s_cap && !s_cap_truncated)
        ESP_LOGI(TAG, "download: curl -o last-job.prn http://<this IP>/dev/last-job.prn");
    ESP_LOGI(TAG, "internal RAM free %u (min %u), PSRAM free %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

static bool web_http(airprint_http_t *h, const char *method, const char *path)
{
    return events_http(h, method, path) || webui_http(h, method, path);
}

static bool dev_http(void *ctx, airprint_http_t *h, const char *method, const char *path)
{
    if (strcmp(path, "/dev/last-job.prn") != 0)
        return web_http(h, method, path);
    size_t len;
    const uint8_t *data = sbuf_read_begin(SBUF_PRINT, &len);
    if (!data)
        return false;
    airprint_http_send(h, "200 OK", "application/octet-stream", data, len);
    sbuf_read_end();
    return true;
}

static bool http_handler(void *ctx, airprint_http_t *h, const char *method, const char *path)
{
    return web_http(h, method, path);
}

// Most of the PSRAM, for two-sided printing (and the dev mode print capture)
static void alloc_shared_buf(void)
{
    size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    size_t size = largest > PSRAM_RESERVE + SHARED_BUF_MIN ? largest - PSRAM_RESERVE : 0;
    uint8_t *buf = size ? heap_caps_malloc(size, MALLOC_CAP_SPIRAM) : NULL;
    sbuf_init(buf, buf ? size : 0);
    if (buf)
        ESP_LOGI(TAG, "shared buffer: %u KB of PSRAM (%u KB PSRAM left)", (unsigned)(size / 1024),
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
    else
        ESP_LOGW(TAG, "no PSRAM for the shared buffer: no two-sided printing");
}

// ---------------------------------------------------------------------------

// The printer service of the current driver
static esp_err_t announce(const char *hostname)
{
    esp_err_t err;
    const printer_driver_t *drv = s_cfg.driver;
    char adminurl[96], product[64], urf[64];
    snprintf(adminurl, sizeof(adminurl), "http://%s.local/", hostname);
    snprintf(product, sizeof(product), "(%s)", drv->usb_mdl);
    printer_driver_urf(drv, urf, sizeof(urf));
    // Bonjour Printing / AirPrint TXT record
    mdns_txt_item_t txt[] = {
        {"txtvers", "1"},
        {"qtotal", "1"},
        {"rp", "ipp/print"},
        {"ty", s_cfg.make_model},
        {"product", product},
        {"note", ""},
        {"adminurl", adminurl},
        {"priority", "50"},
        {"pdl", AIRPRINT_PDL},
        {"URF", urf},
        {"UUID", s_uuid},
        {"Color", drv->color ? "T" : "F"},
        {"Duplex", "F"},
        {"Copies", "F"},
        {"kind", "document"},
        {"PaperMax", "<legal-A4"},     // up to A4
        {"usb_MFG", drv->usb_mfg},
        {"usb_MDL", drv->usb_mdl},
        {"air", "none"},
        {"Transparent", "T"},
        {"Binary", "T"},
        {"TBCP", "F"},
        {"Scan", "F"},
        {"Fax", "F"},
    };
    err = mdns_service_add(PRINTER_NAME, "_ipp", "_tcp", IPP_PORT, txt, sizeof(txt) / sizeof(txt[0]));
    if (err != ESP_OK)
        return err;
    // "_universal" is what iOS looks for; "_print" is IPP Everywhere
    mdns_service_subtype_add_for_host(PRINTER_NAME, "_ipp", "_tcp", NULL, "_universal");
    mdns_service_subtype_add_for_host(PRINTER_NAME, "_ipp", "_tcp", NULL, "_print");

    ESP_LOGI(TAG, "announced \"%s\" as %s.local:%d", PRINTER_NAME, hostname, IPP_PORT);
    return ESP_OK;
}

static esp_err_t start_mdns(const char *hostname)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK)
        return err;
    mdns_hostname_set(hostname);
    mdns_instance_name_set(PRINTER_NAME);
    return announce(hostname);
}

// What IPP, eSCL and mDNS say the printer is. Other tasks read these while
// they change: each pointer is written whole, and the device id that was
// replaced stays valid until the next change.
static void set_identity(const printer_driver_t *drv)
{
    int i = s_device_id_cur ^ 1;
    snprintf(s_device_id[i], sizeof(s_device_id[i]), "MFG:%s;MDL:%s;CMD:URF,PWG;CLS:PRINTER;", drv->usb_mfg,
             drv->usb_mdl);
    s_device_id_cur = i;
    s_cfg.device_id = s_device_id[i];
    s_cfg.make_model = drv->make_model;
    s_cfg.driver = drv;
}

void airprint_esp_driver_changed(void)
{
    if (!s_identity_lock)
        return;     // not started: airprint_esp_start takes the driver as it is then
    xSemaphoreTake(s_identity_lock, portMAX_DELAY);
    const printer_driver_t *drv = print_job_driver();
    if (s_cfg.driver && drv != s_cfg.driver) {
        ESP_LOGI(TAG, "printer model changed: %s -> %s", s_cfg.driver->make_model, drv->make_model);
        set_identity(drv);
        mdns_service_remove_all();
        announce(s_hostname);
    }
    xSemaphoreGive(s_identity_lock);
}

esp_err_t airprint_esp_start(const char *hostname, void (*on_job_state)(bool printing))
{
    s_on_job_state = on_job_state;
    // stable per-board UUID from the MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_uuid, sizeof(s_uuid), "a4d1e9b2-0b6f-4c3e-9d2a-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2],
             mac[3], mac[4], mac[5]);

    const esp_timer_create_args_t targs = {.callback = cancel_timeout, .name = "ipp_cancel"};
    esp_timer_create(&targs, &s_cancel_timer);

    const printer_driver_t *drv = print_job_driver();
    ESP_LOGI(TAG, "printer driver: %s (%s)", drv->id, drv->make_model);
    snprintf(s_hostname, sizeof(s_hostname), "%s", hostname);
    s_cfg = (airprint_config_t){
        .name = PRINTER_NAME,
        .uuid = s_uuid,
        .port = IPP_PORT,
        .web_port = WEB_PORT,
        .get_status = get_status,
        .job_open = job_open,
        .job_write = job_write,
        .job_sync = job_sync,
        .job_flush = job_flush,
        .job_read = job_read,
        .job_delay = job_delay,
        .job_buffer = job_buffer,
        .job_close = job_close,
        .job_cancel = job_cancel,
        .job_state = job_state,
        .identify = identify,
    };
    alloc_shared_buf();
    set_identity(drv);
    s_cfg.http_handler = http_handler;
    if (strcmp(MODE, "dev") == 0) {
        ESP_LOGW(TAG, "MODE=dev: AirPrint jobs are converted but NOT printed");
        s_cfg.get_status = dev_get_status;
        s_cfg.job_open = dev_job_open;
        s_cfg.job_write = dev_job_write;
        s_cfg.job_sync = dev_job_sync;
        s_cfg.job_flush = NULL;         // no printer behind the fake one
        s_cfg.job_read = NULL;
        s_cfg.job_delay = NULL;
        s_cfg.job_buffer = NULL;
        s_cfg.job_close = dev_job_close;
        s_cfg.job_cancel = NULL;
        s_cfg.http_handler = dev_http;
    }
    // live state for the web UI (GET /events)
    events_add_source("printer", airprint_status_json, NULL);
    if (airprint_start(&s_cfg) != 0 || events_start() != 0)
        return ESP_FAIL;
    esp_err_t err = start_mdns(hostname);
    // from now on a printer of another model changes what is announced; one
    // plugged in while this started is caught up here
    s_identity_lock = xSemaphoreCreateMutex();
    airprint_esp_driver_changed();
    return err;
}
