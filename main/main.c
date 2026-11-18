/*
 * Work timer for the Waveshare ESP32-S3-Touch-LCD-1.69.
 *
 * Dial: exact daisy PNG (daisy_img.h). Petals fill like a minute hand each hour
 * (each petal takes 5 minutes); at hour boundary the daisy clears and refills.
 * Text uses Charis SIL serif (font_serif.h): TOTAL digits large, session under TOTAL,
 * PAUSE / DO IT in the daisy center. A phone-style battery glyph sits top-right.
 *
 * Tap starts or pauses and gives a short quiet beep. Double tap clears
 * the current session. Timer never stops at 1h.
 * Display: ST7789V2 240x280 SPI. Touch: CST816T I2C 0x15.
 *
 * PWR (SYS_OUT, GPIO40, active low) toggles the screen. Turning it off
 * enters light sleep. A stopped timer sleeps until the next press. A
 * running timer keeps counting; the CPU wakes only to save every 15s.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "soc/usb_serial_jtag_struct.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "daisy_img.h"
#include "font_serif.h"

typedef struct {
    int cell;
    int bytes_per_glyph;
    const uint8_t *advance;
    const uint8_t *bits;
} serif_font_t;

static const serif_font_t FONT_LG = {
    .cell = SERIF_LG_CELL,
    .bytes_per_glyph = SERIF_LG_BYTES_PER_GLYPH,
    .advance = serif_lg_advance,
    .bits = serif_lg_bits,
};

static const serif_font_t FONT_TOT = {
    .cell = SERIF_TOT_CELL,
    .bytes_per_glyph = SERIF_TOT_BYTES_PER_GLYPH,
    .advance = serif_tot_advance,
    .bits = serif_tot_bits,
};

static const serif_font_t FONT_MD = {
    .cell = SERIF_MD_CELL,
    .bytes_per_glyph = SERIF_MD_BYTES_PER_GLYPH,
    .advance = serif_md_advance,
    .bits = serif_md_bits,
};

static const serif_font_t FONT_SM = {
    .cell = SERIF_SM_CELL,
    .bytes_per_glyph = SERIF_SM_BYTES_PER_GLYPH,
    .advance = serif_sm_advance,
    .bits = serif_sm_bits,
};

static const char *TAG = "work_timer";

#define LCD_W 240
#define LCD_H 280
#define STRIP_H 20

#define PIN_LCD_DC 4
#define PIN_LCD_CS 5
#define PIN_LCD_SCLK 6
#define PIN_LCD_MOSI 7
#define PIN_LCD_RST 8
#define PIN_LCD_BL 15

#define PIN_TP_SCL 10
#define PIN_TP_SDA 11
#define PIN_TP_RST 13
#define PIN_TP_INT 14
#define TP_ADDR 0x15

/* Battery power latch (new PCB: GPIO41; old PCB used GPIO35). */
#define PIN_SYS_EN 41
/* PWR sense, active low (new PCB: GPIO40; old PCB used GPIO36). */
#define PIN_SYS_OUT 40
#define PWR_DEBOUNCE_US 30000LL

/* Onboard buzzer (new PCB: GPIO42; old PCB used GPIO33). */
#define PIN_BUZZ 42
#define BEEP_HZ 4000
#define BEEP_DUTY 2
#define BEEP_US 28000LL

/* Cell tap on GPIO1: 200k over 100k, so VBAT = Vadc * 3.
 * ETA6098 STAT is not wired to a GPIO. */
#define PIN_BAT_ADC 1
#define BAT_DIV 3

#define LCD_SWAP_BYTES 1

#define SEC_US 1000000LL
#define HOUR_US (60LL * 60LL * SEC_US)
#define TIME_SCALE 1LL
#define CHECKPOINT_US (15LL * 1000000LL)
#define FRAME_US 100000LL

#define TAP_MIN_US 0LL
#define TAP_MAX_US 700000LL
#define TAP_DEBOUNCE_US 35000LL
#define DOUBLE_US 380000LL
#define MOVE_PX 60

#define DIAL_CX 120
#define DIAL_CY 191
#define PETAL_COUNT 12

_Static_assert(LCD_H % STRIP_H == 0, "strip height must divide LCD_H");
_Static_assert(DAISY_W <= LCD_W && DAISY_H <= LCD_H, "daisy must fit the LCD");

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t c = (uint16_t)(((r & 0xF8U) << 8) | ((g & 0xFCU) << 3) | (b >> 3));
#if LCD_SWAP_BYTES
    c = (uint16_t)((c >> 8) | (c << 8));
#endif
    return c;
}

#define COL_BG rgb565(0, 0, 0)
#define COL_INK rgb565(0, 0, 0) /* text inside daisy center */
#define COL_PETAL_OUTLINE rgb565(96, 96, 96)
#define COL_DIGIT rgb565(186, 150, 168)   /* TOTAL — slightly grayer */
#define COL_SESSION rgb565(255, 96, 168) /* former TOTAL pink */
#define COL_LABEL rgb565(150, 142, 148)
#define COL_BAT_FRAME rgb565(236, 232, 234)
#define COL_BOLT rgb565(255, 214, 64)

#define TWO_PI 6.28318530718f

static uint16_t *s_fb;
static uint16_t *s_strip[2];
static esp_lcd_panel_io_handle_t s_io;
static esp_lcd_panel_handle_t s_panel;
static i2c_master_dev_handle_t s_tp;
static volatile bool s_touch_irq;
static bool s_touch_ok;
static bool s_touch_reset;
static bool s_display_on;
static bool s_pwr_down;
static int64_t s_lcd_sleep_us;

static bool s_running;
static int64_t s_session_us;
static int64_t s_total_us;
static int64_t s_run_origin_us;
static int64_t s_last_checkpoint_us;
static bool s_dirty;
static bool s_nvs;
static int s_bat_pct = -1;
static bool s_bat_charging;

/* -------------------------------------------------------------------------- */
/*  Storage                                                                   */
/* -------------------------------------------------------------------------- */

/* Keep TOTAL and session on the same second grid so digits tick together. */
static int64_t round_to_sec(int64_t us)
{
    if (us <= 0) {
        return 0;
    }
    return ((us + SEC_US / 2) / SEC_US) * SEC_US;
}

static void snap_stored_to_sec(void)
{
    s_session_us = round_to_sec(s_session_us);
    s_total_us = round_to_sec(s_total_us);
}

static int64_t run_delta_us(int64_t now)
{
    int64_t delta = (now - s_run_origin_us) * TIME_SCALE;
    if (delta < 0) {
        delta = 0;
    }
    return delta;
}

static int64_t session_at(int64_t now)
{
    int64_t us = s_session_us;
    if (s_running) {
        us += run_delta_us(now);
    }
    if (us < 0) {
        us = 0;
    }
    /* Display whole seconds only; same floor for both clocks. */
    return (us / SEC_US) * SEC_US;
}

static int64_t total_at(int64_t now)
{
    int64_t us = s_total_us;
    if (s_running) {
        us += run_delta_us(now);
    }
    if (us < 0) {
        us = 0;
    }
    return (us / SEC_US) * SEC_US;
}

static void checkpoint(int64_t now, bool stop)
{
    if (s_running) {
        int64_t delta = run_delta_us(now);
        s_session_us += delta;
        s_total_us += delta;
        s_run_origin_us = now;
    }
    if (stop) {
        s_running = false;
        snap_stored_to_sec();
    }
}

/* Progress within the current hour for the daisy (wraps each hour). */
static int64_t session_in_hour(int64_t session_us)
{
    if (session_us <= 0) {
        return 0;
    }
    return session_us % HOUR_US;
}

static void persist(void)
{
    nvs_handle_t handle;
    esp_err_t err;

    if (!s_nvs) {
        return;
    }
    err = nvs_open("worktimer", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs open failed: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_u64(handle, "total", (uint64_t)s_total_us);
    if (err == ESP_OK) {
        err = nvs_set_u64(handle, "session", (uint64_t)s_session_us);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs write failed: %s", esp_err_to_name(err));
    }
}

static void storage_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    s_nvs = true;

    nvs_handle_t handle;
    if (nvs_open("worktimer", NVS_READONLY, &handle) != ESP_OK) {
        return;
    }
    uint64_t total = 0;
    uint64_t session = 0;
    nvs_get_u64(handle, "total", &total);
    nvs_get_u64(handle, "session", &session);
    nvs_close(handle);

    s_total_us = (int64_t)total;
    s_session_us = (int64_t)session;
    if (s_total_us < 0) {
        s_total_us = 0;
    }
    if (s_session_us < 0) {
        s_session_us = 0;
    }
    snap_stored_to_sec();
    ESP_LOGI(TAG, "loaded session %lld s, total %lld s",
             (long long)(s_session_us / SEC_US),
             (long long)(s_total_us / SEC_US));
}

/* -------------------------------------------------------------------------- */
/*  Drawing                                                                   */
/* -------------------------------------------------------------------------- */

static void px(int x, int y, uint16_t color)
{
    if ((unsigned)x < LCD_W && (unsigned)y < LCD_H) {
        s_fb[y * LCD_W + x] = color;
    }
}

static void fill_screen(uint16_t color)
{
    int n = LCD_W * LCD_H;
    for (int i = 0; i < n; i++) {
        s_fb[i] = color;
    }
}

static int text_width_font(const serif_font_t *font, const char *text, int tracking)
{
    int w = 0;
    int n = 0;
    for (int i = 0; text[i] != '\0'; i++) {
        unsigned char ch = (unsigned char)text[i];
        uint8_t idx = (ch < 128) ? serif_cmap[ch] : 0xFF;
        if (idx == 0xFF) {
            w += font->cell / 2;
        } else {
            w += font->advance[idx];
        }
        n++;
    }
    if (n > 1) {
        w += tracking * (n - 1);
    }
    return w;
}

static void draw_glyph_font(const serif_font_t *font, int x, int y, uint8_t idx, uint16_t color)
{
    const uint8_t *bits = font->bits + (size_t)idx * (size_t)font->bytes_per_glyph;
    int bit = 0;
    for (int row = 0; row < font->cell; row++) {
        for (int col = 0; col < font->cell; col++) {
            int byte = bit / 8;
            int shift = 7 - (bit % 8);
            if (bits[byte] & (uint8_t)(1U << shift)) {
                px(x + col, y + row, color);
            }
            bit++;
        }
    }
}

static void draw_text_font(const serif_font_t *font, int x, int y, const char *text, uint16_t color, int tracking)
{
    for (int i = 0; text[i] != '\0'; i++) {
        unsigned char ch = (unsigned char)text[i];
        uint8_t idx = (ch < 128) ? serif_cmap[ch] : 0xFF;
        if (idx != 0xFF) {
            int adv = font->advance[idx];
            /* Glyph ink is centered in cell; align that cell to the advance slot. */
            int gx = x + (adv - font->cell) / 2;
            draw_glyph_font(font, gx, y, idx, color);
            x += adv + tracking;
        } else {
            x += font->cell / 2 + tracking;
        }
    }
}

static void draw_centered_font(const serif_font_t *font, int cx, int y, const char *text, uint16_t color, int tracking)
{
    draw_text_font(font, cx - text_width_font(font, text, tracking) / 2, y, text, color, tracking);
}

static void blit_daisy_center(int ox, int oy)
{
    for (int y = 0; y < DAISY_H; y++) {
        const uint16_t *src = daisy_img + y * DAISY_W;
        const uint8_t *mask = daisy_mask + y * DAISY_W;
        uint16_t *dst = s_fb + (oy + y) * LCD_W + ox;
        for (int x = 0; x < DAISY_W; x++) {
            if (mask[x] == 2) {
                dst[x] = src[x];
            }
        }
    }
}

static int petal_mask_at(int x, int y)
{
    if (x < 0 || y < 0 || x >= DAISY_W || y >= DAISY_H) {
        return 0;
    }
    return daisy_mask[y * DAISY_W + x] == 1;
}

/* Light-gray edge contours showing where petals will appear. */
static void blit_petal_outlines(int ox, int oy)
{
    for (int y = 0; y < DAISY_H; y++) {
        for (int x = 0; x < DAISY_W; x++) {
            if (!petal_mask_at(x, y)) {
                continue;
            }
            if (!petal_mask_at(x - 1, y) || !petal_mask_at(x + 1, y) ||
                !petal_mask_at(x, y - 1) || !petal_mask_at(x, y + 1)) {
                px(ox + x, oy + y, COL_PETAL_OUTLINE);
            }
        }
    }
}

/*
 * Fill petals along a sweeping "minute hand": angle advances over the hour so
 * each of the 12 petals is fully filled in 5 minutes.
 * Petal geometry bias: shift fill boundary 15° clockwise.
 */
#define PETAL_HAND_OFFSET (TWO_PI * (15.f / 360.f))

static void reveal_petals(int ox, int oy, int64_t session_us)
{
    int cx = ox + DAISY_W / 2;
    int cy = oy + DAISY_H / 2;
    float hand_ang;

    if (session_us <= 0) {
        return;
    }
    /* New hour → daisy clears (mod == 0) and fills again. */
    hand_ang = (float)session_in_hour(session_us) * TWO_PI / (float)HOUR_US;

    for (int y = 0; y < DAISY_H; y++) {
        const uint16_t *src = daisy_img + y * DAISY_W;
        const uint8_t *mask = daisy_mask + y * DAISY_W;
        for (int x = 0; x < DAISY_W; x++) {
            float dx;
            float dy;
            float ang;

            if (mask[x] != 1) {
                continue;
            }
            dx = (float)(ox + x - cx);
            dy = (float)(cy - (oy + y)); /* up-positive → 0 at top, clockwise */
            ang = atan2f(dx, dy);
            if (ang < 0.f) {
                ang += TWO_PI;
            }
            /* Shift fill boundary clockwise to match petal geometry. */
            ang -= PETAL_HAND_OFFSET;
            if (ang < 0.f) {
                ang += TWO_PI;
            }
            if (ang <= hand_ang) {
                px(ox + x, oy + y, src[x]);
            }
        }
    }
}

static void split_hms(int64_t us, unsigned *hours, unsigned *mins, unsigned *secs)
{
    uint64_t sec;
    if (us < 0) {
        us = 0;
    }
    sec = (uint64_t)us / 1000000ULL;
    *hours = (unsigned)(sec / 3600ULL);
    *mins = (unsigned)((sec / 60ULL) % 60ULL);
    *secs = (unsigned)(sec % 60ULL);
    /* TOTAL counter spans up to 10000 hours. */
    if (*hours > 10000U) {
        *hours = 10000U;
        *mins = 0;
        *secs = 0;
    }
}

static void format_total(char *out, size_t n, int64_t us)
{
    unsigned hours;
    unsigned mins;
    unsigned secs;
    split_hms(us, &hours, &mins, &secs);
    snprintf(out, n, "%05u:%02u:%02u", hours, mins, secs);
}

static void format_session(char *out, size_t n, int64_t us)
{
    unsigned hours;
    unsigned mins;
    unsigned secs;
    if (us < 0) {
        us = 0;
    }
    split_hms(us, &hours, &mins, &secs);
    snprintf(out, n, "%02u:%02u:%02u", hours, mins, secs);
}

static void fill_rect(int x, int y, int w, int h, uint16_t color)
{
    for (int row = 0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            px(x + col, y + row, color);
        }
    }
}

static void stroke_round_rect(int x, int y, int w, int h, uint16_t color)
{
    for (int col = 1; col < w - 1; col++) {
        px(x + col, y, color);
        px(x + col, y + h - 1, color);
    }
    for (int row = 1; row < h - 1; row++) {
        px(x, y + row, color);
        px(x + w - 1, y + row, color);
    }
}

/* 5x7 lightning, bit 4 is the left pixel. */
static const uint8_t BAT_BOLT[7] = {
    0x04, 0x0c, 0x1e, 0x06, 0x0e, 0x04, 0x04,
};

static void draw_bolt(int x0, int y0, int x1, int y1, int ox, int oy)
{
    for (int pass = 0; pass < 2; pass++) {
        int rad = (pass == 0) ? 1 : 0;
        uint16_t color = (pass == 0) ? COL_BG : COL_BOLT;
        for (int row = 0; row < 7; row++) {
            for (int col = 0; col < 5; col++) {
                if ((BAT_BOLT[row] & (uint8_t)(1U << (4 - col))) == 0) {
                    continue;
                }
                for (int dy = -rad; dy <= rad; dy++) {
                    for (int dx = -rad; dx <= rad; dx++) {
                        int ix = ox + col + dx;
                        int iy = oy + row + dy;
                        if (ix > x0 && ix < x1 && iy > y0 && iy < y1) {
                            px(ix, iy, color);
                        }
                    }
                }
            }
        }
    }
}

/* Full is green, empty is red, yellow in between. Charging does not recolor it. */
static uint16_t battery_color(int pct)
{
    int r;
    int g;
    int b;
    int u;

    if (pct < 0) {
        pct = 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    if (pct >= 50) {
        u = (pct - 50) * 2;
        r = 232 + (46 - 232) * u / 100;
        g = 186 + (198 - 186) * u / 100;
        b = 32 + (78 - 32) * u / 100;
    } else {
        u = pct * 2;
        r = 220 + (232 - 220) * u / 100;
        g = 42 + (186 - 42) * u / 100;
        b = 48 + (32 - 48) * u / 100;
    }
    return rgb565((uint8_t)r, (uint8_t)g, (uint8_t)b);
}

/* Phone status glyph: fill is the cell level, a bolt means the charger is in. */
static void draw_battery(void)
{
    const int body_w = 22;
    const int body_h = 12;
    const int nub_w = 2;
    const int nub_h = 6;
    int x;
    int y;
    int cap_w;
    int cap_h;
    int bars;
    uint16_t color;

    if (s_bat_pct < 0) {
        return;
    }

    x = LCD_W - 17 - body_w - nub_w;
    y = 7;
    color = battery_color(s_bat_pct);

    stroke_round_rect(x, y, body_w, body_h, COL_BAT_FRAME);
    fill_rect(x + body_w, y + (body_h - nub_h) / 2, nub_w, nub_h, COL_BAT_FRAME);

    cap_w = body_w - 4;
    cap_h = body_h - 4;
    if (s_bat_pct >= 100) {
        bars = cap_w;
    } else if (s_bat_pct <= 0) {
        bars = 0;
    } else {
        bars = cap_w * s_bat_pct / 100;
        if (bars < 1) {
            bars = 1;
        }
    }
    if (bars > 0) {
        fill_rect(x + 2, y + 2, bars, cap_h, color);
    }
    if (s_bat_charging) {
        draw_bolt(x, y, x + body_w, y + body_h,
                  x + (body_w - 5) / 2, y + (body_h - 7) / 2);
    }
}

static void render(int64_t now)
{
    char total_text[20];
    char session_text[12];
    /* Swapped labels: PAUSE while idle, DO IT while running. */
    const char *status = "PAUSE";
    int64_t session = session_at(now);
    int64_t total = total_at(now);
    int ox = DIAL_CX - DAISY_W / 2;
    int oy = DIAL_CY - DAISY_H / 2;
    int total_w;
    int total_x;
    int y_total;
    int y_cur_label;
    int y_session;

    fill_screen(COL_BG);
    blit_daisy_center(ox, oy);
    blit_petal_outlines(ox, oy);
    reveal_petals(ox, oy, session);

    if (s_running) {
        status = "DO IT";
    }

    format_total(total_text, sizeof(total_text), total);
    format_session(session_text, sizeof(session_text), session);
    total_w = text_width_font(&FONT_TOT, total_text, 0);
    total_x = (LCD_W - total_w) / 2;
    y_total = 4 + FONT_SM.cell + 4;
    y_cur_label = y_total + FONT_TOT.cell + 4;
    y_session = y_cur_label + FONT_SM.cell;

    draw_centered_font(&FONT_SM, LCD_W / 2, 4, "TOTAL", COL_LABEL, 0);
    draw_text_font(&FONT_TOT, total_x, y_total, total_text, COL_DIGIT, 0);
    draw_centered_font(&FONT_SM, LCD_W / 2, y_cur_label, "CURRENT TIME", COL_LABEL, 0);
    draw_centered_font(&FONT_LG, LCD_W / 2, y_session, session_text, COL_SESSION, 0);

    /* DO IT starts, PAUSE stops. Always black inside the daisy. */
    draw_centered_font(&FONT_MD, DIAL_CX, DIAL_CY - FONT_MD.cell / 2, status, COL_INK, 0);
    draw_battery();
}

static void blit(void)
{
    static int index;

    for (int y = 0; y < LCD_H; y += STRIP_H) {
        uint16_t *strip = s_strip[index];
        memcpy(strip, s_fb + y * LCD_W, LCD_W * STRIP_H * sizeof(uint16_t));
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(s_panel, 0, y, LCD_W, y + STRIP_H, strip));
        index ^= 1;
    }
}

static esp_err_t gfx_init(void)
{
    s_fb = heap_caps_malloc(LCD_W * LCD_H * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_strip[0] = heap_caps_malloc(LCD_W * STRIP_H * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    s_strip[1] = heap_caps_malloc(LCD_W * STRIP_H * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (s_fb == NULL || s_strip[0] == NULL || s_strip[1] == NULL) {
        ESP_LOGE(TAG, "buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*  Display and touch                                                         */
/* -------------------------------------------------------------------------- */

static esp_err_t lcd_init(void)
{
    const gpio_config_t backlight = {
        .pin_bit_mask = 1ULL << PIN_LCD_BL,
        .mode = GPIO_MODE_OUTPUT,
    };
    const spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_LCD_SCLK,
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .data4_io_num = GPIO_NUM_NC,
        .data5_io_num = GPIO_NUM_NC,
        .data6_io_num = GPIO_NUM_NC,
        .data7_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_W * STRIP_H * (int)sizeof(uint16_t),
    };
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = PIN_LCD_DC,
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 1,
    };
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG,
        .bits_per_pixel = 16,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&backlight), TAG, "backlight gpio");
    gpio_set_level(PIN_LCD_BL, 0);

    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi bus");
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &s_io),
        TAG, "panel io");
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(s_io, &panel_cfg, &s_panel), TAG, "st7789");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "panel reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "panel init");
    /* 180° from previous orientation (was mirror true/true). */
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(s_panel, false, false), TAG, "mirror");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "display on");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(s_panel, 0, 20), TAG, "gap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(s_panel, true), TAG, "invert");
    /* Keep the panel bus driven in sleep so SLPIN is not disturbed. */
    gpio_sleep_sel_dis(PIN_LCD_DC);
    gpio_sleep_sel_dis(PIN_LCD_CS);
    gpio_sleep_sel_dis(PIN_LCD_SCLK);
    gpio_sleep_sel_dis(PIN_LCD_MOSI);
    gpio_sleep_sel_dis(PIN_LCD_RST);
    gpio_sleep_sel_dis(PIN_LCD_BL);
    return ESP_OK;
}

static void IRAM_ATTR tp_isr(void *arg)
{
    (void)arg;
    s_touch_irq = true;
}

static esp_err_t cst_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_tp, &reg, 1, data, len, 20);
}

static esp_err_t cst_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};

    return i2c_master_transmit(s_tp, buf, sizeof(buf), 20);
}

static esp_err_t touch_init(void)
{
    const gpio_config_t reset_cfg = {
        .pin_bit_mask = 1ULL << PIN_TP_RST,
        .mode = GPIO_MODE_OUTPUT,
    };
    const gpio_config_t int_cfg = {
        .pin_bit_mask = 1ULL << PIN_TP_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_TP_SDA,
        .scl_io_num = PIN_TP_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TP_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err;
    uint8_t chip_id = 0;

    ESP_RETURN_ON_ERROR(gpio_config(&reset_cfg), TAG, "touch reset gpio");
    ESP_RETURN_ON_ERROR(gpio_config(&int_cfg), TAG, "touch int gpio");
    gpio_set_level(PIN_TP_RST, 1);

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus), TAG, "i2c bus");
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &s_tp), TAG, "i2c device");

    gpio_set_level(PIN_TP_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_TP_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(PIN_TP_INT, tp_isr, NULL), TAG, "touch isr");
    s_touch_irq = false;

    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    esp_log_level_set("i2c", ESP_LOG_NONE);

    if (cst_read(0xA7, &chip_id, 1) == ESP_OK) {
        ESP_LOGI(TAG, "CST816 id %u", chip_id);
    } else {
        ESP_LOGI(TAG, "CST816 is asleep until the first touch");
    }
    gpio_sleep_sel_dis(PIN_TP_SCL);
    gpio_sleep_sel_dis(PIN_TP_SDA);
    gpio_sleep_sel_dis(PIN_TP_RST);
    gpio_sleep_sel_dis(PIN_TP_INT);
    s_touch_ok = true;
    return ESP_OK;
}

static void touch_sleep(void)
{
    if (!s_touch_ok) {
        return;
    }
    (void)cst_write(0xE5, 0x03);
    gpio_intr_disable(PIN_TP_INT);
}

static void touch_wake(void)
{
    if (!s_touch_ok) {
        return;
    }
    gpio_set_level(PIN_TP_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_TP_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    s_touch_irq = false;
    gpio_intr_enable(PIN_TP_INT);
}

static bool touch_sample(uint16_t *x, uint16_t *y, bool force)
{
    uint8_t raw[5];
    bool irq = s_touch_irq;
    int level = gpio_get_level(PIN_TP_INT);

    if (!force && level != 0 && !irq) {
        return false;
    }
    s_touch_irq = false;
    if (cst_read(0x02, raw, sizeof(raw)) != ESP_OK || raw[0] == 0) {
        return false;
    }
    *x = (uint16_t)(((raw[1] & 0x0F) << 8) | raw[2]);
    *y = (uint16_t)(((raw[3] & 0x0F) << 8) | raw[4]);
    /* Match 180° display orientation. */
    if (*x < LCD_W) {
        *x = (uint16_t)(LCD_W - 1 - *x);
    }
    if (*y < LCD_H) {
        *y = (uint16_t)(LCD_H - 1 - *y);
    }
    return true;
}

static void timer_start(int64_t now)
{
    if (s_running) {
        return;
    }
    s_running = true;
    s_run_origin_us = now;
    s_last_checkpoint_us = now;
    persist();
    s_dirty = true;
    ESP_LOGI(TAG, "start");
}

static void timer_pause(int64_t now)
{
    checkpoint(now, true);
    persist();
    s_dirty = true;
    ESP_LOGI(TAG, "pause");
}

static void timer_reset(int64_t now)
{
    checkpoint(now, true);
    s_session_us = 0;
    persist();
    s_dirty = true;
    ESP_LOGI(TAG, "reset");
}

/* -------------------------------------------------------------------------- */
/*  Buzzer                                                                    */
/* -------------------------------------------------------------------------- */

static bool s_beep_ready;
static int64_t s_beep_off_us;

static void beep_init(void)
{
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = BEEP_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t chan = {
        .gpio_num = PIN_BUZZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };

    if (ledc_timer_config(&timer) != ESP_OK || ledc_channel_config(&chan) != ESP_OK) {
        ESP_LOGW(TAG, "buzzer init failed");
        return;
    }
    gpio_sleep_sel_dis(PIN_BUZZ);
    s_beep_ready = true;
}

/* Short high pip. Duty is a small fraction of 255 so the buzzer stays quiet. */
static void beep(int64_t now)
{
    if (!s_beep_ready) {
        return;
    }
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, BEEP_DUTY) != ESP_OK ||
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0) != ESP_OK) {
        return;
    }
    s_beep_off_us = now + BEEP_US;
}

static void beep_poll(int64_t now)
{
    if (!s_beep_ready || s_beep_off_us == 0 || now < s_beep_off_us) {
        return;
    }
    s_beep_off_us = 0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void beep_stop(void)
{
    s_beep_off_us = 0;
    if (!s_beep_ready) {
        return;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_timer_pause(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
}

static void beep_resume(void)
{
    if (!s_beep_ready) {
        return;
    }
    ledc_timer_resume(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
}

static void touch_poll(int64_t now)
{
    static int phase;
    static int64_t down_us;
    static int64_t up_us;
    static int x0;
    static int y0;
    static bool have_anchor;
    static bool moved;
    static bool arm;
    static int64_t arm_us;
    static bool reset_gesture;
    uint16_t x = 0;
    uint16_t y = 0;

    if (s_touch_reset) {
        phase = 0;
        arm = false;
        reset_gesture = false;
        s_touch_reset = false;
        s_touch_irq = false;
    }
    bool pressed = touch_sample(&x, &y, phase != 0);
    bool on_screen = x < LCD_W && y < LCD_H;

    if (phase == 0) {
        if (pressed) {
            phase = 1;
            down_us = now;
            moved = false;
            have_anchor = false;
            reset_gesture = arm && (now - arm_us) <= DOUBLE_US;
            if (on_screen) {
                x0 = x;
                y0 = y;
                have_anchor = true;
            }
        }
    } else if (phase == 1) {
        if (pressed) {
            if (on_screen) {
                if (!have_anchor) {
                    x0 = x;
                    y0 = y;
                    have_anchor = true;
                } else if ((x > x0 ? x - x0 : x0 - x) > MOVE_PX ||
                           (y > y0 ? y - y0 : y0 - y) > MOVE_PX) {
                    moved = true;
                }
            }
        } else {
            phase = 2;
            up_us = now;
        }
    } else if (pressed) {
        phase = 1;
    } else if (now - up_us >= TAP_DEBOUNCE_US) {
        int64_t held = up_us - down_us;
        bool tap = !moved && held >= TAP_MIN_US && held <= TAP_MAX_US;
        phase = 0;
        if (reset_gesture) {
            if (tap) {
                timer_reset(now);
                beep(now);
            }
            arm = false;
            reset_gesture = false;
        } else if (tap) {
            if (s_running) {
                timer_pause(now);
            } else {
                timer_start(now);
            }
            beep(now);
            arm = true;
            arm_us = now;
        }
    }

    if (arm && phase == 0 && (now - arm_us) > DOUBLE_US) {
        arm = false;
    }
}

/* -------------------------------------------------------------------------- */
/*  Battery                                                                   */
/* -------------------------------------------------------------------------- */

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_adc_cali;
static adc_channel_t s_bat_chan;
static bool s_bat_ready;
static bool s_bat_suspended;
static bool s_ui_live;
static bool s_bat_usb;
static bool s_bat_volt_chg;
static int s_usb_hold;
static int s_bat_mv = -1;

static uint32_t usb_frame(void)
{
    const volatile uint32_t *reg = (const volatile uint32_t *)&USB_SERIAL_JTAG.fram_num;

    return (*reg) & 0x7ffu;
}

/* SOF frame index advances only while a USB host is on the Type-C port. */
static bool usb_host_present(void)
{
    static uint32_t prev;
    static bool primed;
    uint32_t frame = usb_frame();
    bool advanced = primed && frame != prev;

    prev = frame;
    primed = true;
    return advanced;
}

static int battery_percent(int mv)
{
    static const int curve[][2] = {
        {3300, 0},
        {3450, 5},
        {3600, 15},
        {3700, 30},
        {3750, 45},
        {3800, 55},
        {3900, 70},
        {4000, 80},
        {4100, 92},
        {4200, 100},
    };
    const int n = (int)(sizeof(curve) / sizeof(curve[0]));

    if (mv <= curve[0][0]) {
        return 0;
    }
    if (mv >= curve[n - 1][0]) {
        return 100;
    }
    for (int i = 0; i < n - 1; i++) {
        if (mv <= curve[i + 1][0]) {
            int span = curve[i + 1][0] - curve[i][0];
            int rise = curve[i + 1][1] - curve[i][1];
            return curve[i][1] + (mv - curve[i][0]) * rise / span;
        }
    }
    return 100;
}

static int battery_read_mv(void)
{
    int sum = 0;
    int n = 0;

    for (int i = 0; i < 16; i++) {
        int raw = 0;
        int pin_mv = 0;

        if (adc_oneshot_read(s_adc, s_bat_chan, &raw) != ESP_OK) {
            continue;
        }
        if (s_adc_cali != NULL) {
            if (adc_cali_raw_to_voltage(s_adc_cali, raw, &pin_mv) != ESP_OK) {
                continue;
            }
        } else {
            pin_mv = raw * 3300 / 4095;
        }
        sum += pin_mv;
        n++;
    }
    if (n == 0) {
        return -1;
    }
    return (sum / n) * BAT_DIV;
}

/* Latch "charging" on a step up, drop it when the cell falls off that peak. */
static int battery_note_voltage(int mv)
{
    static int filt = -1;
    static int floor_mv = -1;
    static int peak;
    static int rise;
    static int shown = -1;

    if (filt < 0) {
        filt = mv;
    } else {
        filt += (mv - filt) / 4;
    }
    s_bat_mv = filt;

    if (floor_mv < 0) {
        floor_mv = filt;
    } else if (filt + 15 < floor_mv) {
        floor_mv = filt;
    }

    if (!s_bat_volt_chg) {
        if (filt > floor_mv + 60) {
            if (++rise >= 2) {
                s_bat_volt_chg = true;
                peak = filt;
            }
        } else {
            rise = 0;
            if (floor_mv < filt) {
                floor_mv++;
            }
        }
    } else if (filt > peak) {
        peak = filt;
    } else if (filt + 35 < peak) {
        s_bat_volt_chg = false;
        rise = 0;
        floor_mv = filt;
    }

    if (shown < 0 || filt >= shown + 20 || filt <= shown - 20) {
        shown = filt;
    }
    return battery_percent(shown);
}

static void battery_set(int pct, bool charging)
{
    if (pct < 0) {
        return;
    }
    if (pct == s_bat_pct && charging == s_bat_charging) {
        return;
    }
    s_bat_pct = pct;
    s_bat_charging = charging;
    if (s_ui_live) {
        s_dirty = true;
    }
    ESP_LOGI(TAG, "battery %d mV, %d pct, %s",
             s_bat_mv, pct, charging ? "charging" : "on battery");
}

static void battery_poll(int64_t now)
{
    static int64_t next_usb;
    static int64_t next_adc;
    static int pct = -1;
    int mv;

    if (!s_bat_ready) {
        return;
    }
    if (now >= next_usb) {
        next_usb = now + 200000LL;
        if (usb_host_present()) {
            s_usb_hold = 3;
            s_bat_usb = true;
        } else if (s_usb_hold > 0 && --s_usb_hold == 0) {
            s_bat_usb = false;
        }
    }
    if (now >= next_adc) {
        next_adc = now + 1000000LL;
        mv = battery_read_mv();
        if (mv >= 0) {
            pct = battery_note_voltage(mv);
        }
    }
    battery_set(pct, s_bat_usb || s_bat_volt_chg);
}

static void battery_init(void)
{
    adc_unit_t unit = ADC_UNIT_1;
    adc_channel_t chan = ADC_CHANNEL_0;
    esp_err_t err;

    err = adc_oneshot_io_to_channel(PIN_BAT_ADC, &unit, &chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "battery pin: %s", esp_err_to_name(err));
        return;
    }

    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = unit,
    };
    err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "battery adc: %s", esp_err_to_name(err));
        return;
    }

    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc, chan, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "battery channel: %s", esp_err_to_name(err));
        return;
    }
    s_bat_chan = chan;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    const adc_cali_curve_fitting_config_t cal_cfg = {
        .unit_id = unit,
        .chan = chan,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cal_cfg, &s_adc_cali) != ESP_OK) {
        s_adc_cali = NULL;
        ESP_LOGW(TAG, "battery adc has no calibration");
    }
#else
    s_adc_cali = NULL;
#endif

    s_bat_ready = true;
    (void)usb_host_present();
    vTaskDelay(pdMS_TO_TICKS(5));
    battery_poll(esp_timer_get_time());
}

static void battery_suspend(void)
{
    if (!s_bat_ready) {
        return;
    }
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (s_adc_cali != NULL) {
        adc_cali_delete_scheme_curve_fitting(s_adc_cali);
        s_adc_cali = NULL;
    }
#endif
    adc_oneshot_del_unit(s_adc);
    s_adc = NULL;
    s_bat_ready = false;
    s_bat_suspended = true;
}

static void battery_resume(void)
{
    if (!s_bat_suspended) {
        return;
    }
    s_bat_suspended = false;
    battery_init();
}

/* -------------------------------------------------------------------------- */
/*  PWR and sleep                                                             */
/* -------------------------------------------------------------------------- */

static bool pwr_is_down(void)
{
    return gpio_get_level(PIN_SYS_OUT) == 0;
}

static bool pwr_stable(bool down)
{
    int64_t t0 = esp_timer_get_time();

    while ((esp_timer_get_time() - t0) < PWR_DEBOUNCE_US) {
        if (pwr_is_down() != down) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return pwr_is_down() == down;
}

static void pwr_wait_level(bool down)
{
    while (!pwr_stable(down)) {
        if (s_running && (esp_timer_get_time() - s_last_checkpoint_us) >= CHECKPOINT_US) {
            int64_t now = esp_timer_get_time();
            checkpoint(now, false);
            persist();
            s_last_checkpoint_us = now;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void pwr_init(void)
{
    /* GPIO40 is a JTAG pin until the mux is moved to GPIO. */
    gpio_reset_pin(PIN_SYS_OUT);
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_SYS_OUT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_sleep_sel_dis(PIN_SYS_OUT);
    s_pwr_down = pwr_is_down();
    ESP_LOGI(TAG, "PWR GPIO%d idle %s", PIN_SYS_OUT, s_pwr_down ? "low" : "high");
}

/* Press while the screen is on. The power-on hold is not a press. */
static bool pwr_pressed_edge(void)
{
    bool down = pwr_is_down();

    if (down == s_pwr_down) {
        return false;
    }
    if (!pwr_stable(down)) {
        return false;
    }
    s_pwr_down = down;
    if (down && s_display_on) {
        ESP_LOGI(TAG, "PWR pressed");
        return true;
    }
    return false;
}

static void lcd_cmd(uint8_t cmd)
{
    esp_lcd_panel_io_tx_param(s_io, cmd, NULL, 0);
}

static void display_off(void)
{
    gpio_set_level(PIN_LCD_BL, 0);
    beep_stop();
    esp_lcd_panel_disp_on_off(s_panel, false);
    lcd_cmd(0x10); /* SLPIN */
    s_lcd_sleep_us = esp_timer_get_time();
    touch_sleep();
    battery_suspend();
    s_display_on = false;
}

static void display_on(void)
{
    int64_t wait_us = 120000LL - (esp_timer_get_time() - s_lcd_sleep_us);
    int64_t woke;

    if (wait_us > 0) {
        vTaskDelay(pdMS_TO_TICKS((wait_us + 999) / 1000));
    }
    lcd_cmd(0x11); /* SLPOUT */
    woke = esp_timer_get_time();
    touch_wake();
    beep_resume();
    wait_us = 120000LL - (esp_timer_get_time() - woke);
    if (wait_us > 0) {
        vTaskDelay(pdMS_TO_TICKS((wait_us + 999) / 1000));
    }
    esp_lcd_panel_disp_on_off(s_panel, true);
    battery_resume();
    s_touch_reset = true;
    s_display_on = true;
    render(esp_timer_get_time());
    blit();
    gpio_set_level(PIN_LCD_BL, 1);
}

static void sleep_until_pwr(void)
{
    bool logged_reject = false;

    ESP_ERROR_CHECK(gpio_wakeup_enable(PIN_SYS_OUT, GPIO_INTR_LOW_LEVEL));
    while (1) {
        int64_t now;
        int64_t before;
        esp_err_t err;

        if (pwr_is_down() && pwr_stable(true)) {
            break;
        }

        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
        ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
        if (s_running) {
            int64_t remain = CHECKPOINT_US - (esp_timer_get_time() - s_last_checkpoint_us);

            if (remain < 1000000LL) {
                int64_t t = esp_timer_get_time();
                checkpoint(t, false);
                persist();
                s_last_checkpoint_us = t;
                remain = CHECKPOINT_US;
            }
            ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup((uint64_t)remain));
        }

        before = esp_timer_get_time();
        err = esp_light_sleep_start();
        now = esp_timer_get_time();

        if (s_running && (now - s_last_checkpoint_us) >= CHECKPOINT_US) {
            checkpoint(now, false);
            persist();
            s_last_checkpoint_us = now;
        }

        if (err == ESP_OK && (now - before) >= 20000) {
            continue;
        }
        if (err == ESP_ERR_SLEEP_REJECT && !logged_reject && !pwr_is_down()) {
            ESP_LOGW(TAG, "light sleep rejected");
            logged_reject = true;
        } else if (err != ESP_OK && err != ESP_ERR_SLEEP_REJECT) {
            ESP_LOGW(TAG, "light sleep: %s", esp_err_to_name(err));
        }
        if (!pwr_is_down()) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    gpio_wakeup_disable(PIN_SYS_OUT);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
}

static void screen_sleep(void)
{
    int64_t now = esp_timer_get_time();

    if (s_running) {
        checkpoint(now, false);
        persist();
        s_last_checkpoint_us = now;
    }
    display_off();
    ESP_LOGI(TAG, "display off, %s", s_running ? "timer running" : "timer stopped");
    pwr_wait_level(false);
    sleep_until_pwr();
    if (s_running) {
        now = esp_timer_get_time();
        checkpoint(now, false);
        persist();
        s_last_checkpoint_us = now;
    }
    display_on();
    ESP_LOGI(TAG, "display on");
}

static void power_hold_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_SYS_EN,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_reset_pin(PIN_SYS_EN);
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(PIN_SYS_EN, 1);
    gpio_sleep_sel_dis(PIN_SYS_EN);
    ESP_ERROR_CHECK(gpio_hold_en(PIN_SYS_EN));
}

void app_main(void)
{
    int64_t last_frame = 0;

    /* Keep battery rail on after PWR is released. */
    power_hold_init();

    storage_init();
    beep_init();
    ESP_ERROR_CHECK(lcd_init());
    if (touch_init() != ESP_OK) {
        ESP_LOGE(TAG, "touch init failed");
    }
    ESP_ERROR_CHECK(gfx_init());
    battery_init();

    render(esp_timer_get_time());
    blit();
    s_ui_live = true;
    gpio_set_level(PIN_LCD_BL, 1);
    s_display_on = true;
    pwr_init();
    ESP_LOGI(TAG, "tap: start/pause, double tap: reset; PWR: screen sleep");

    while (1) {
        int64_t now = esp_timer_get_time();
        bool draw;

        touch_poll(now);
        beep_poll(now);
        battery_poll(now);

        if (s_running && (now - s_last_checkpoint_us) >= CHECKPOINT_US) {
            checkpoint(now, false);
            persist();
            s_last_checkpoint_us = now;
        }

        if (pwr_pressed_edge()) {
            screen_sleep();
            last_frame = esp_timer_get_time();
            s_dirty = false;
            continue;
        }

        draw = s_dirty || (s_running && (now - last_frame) >= FRAME_US);
        if (draw) {
            render(now);
            blit();
            last_frame = esp_timer_get_time();
            s_dirty = false;
        }
        vTaskDelay(pdMS_TO_TICKS(8));
    }
}
