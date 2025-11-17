/**
 * Display Driver for ST7789 1.14" LCD
 * Using LVGL for graphics
 */

#include "display.h"
#include "config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "Display";

// LVGL objects
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[TFT_WIDTH * 10];  // Reduced buffer size
static lv_disp_drv_t disp_drv;
static lv_disp_t *disp;

// UI elements
static lv_obj_t *label_title;
static lv_obj_t *label_bt_status;
static lv_obj_t *label_track_info;
static lv_obj_t *label_accel_x;
static lv_obj_t *label_accel_y;
static lv_obj_t *label_accel_z;
static lv_obj_t *label_message;

// SPI device handle
static spi_device_handle_t spi;
// Global offsets (initialized from config macros)
int g_tft_offset_x = TFT_OFFSET_X;
int g_tft_offset_y = TFT_OFFSET_Y;

// ============================================================================
// ST7789 Commands
// ============================================================================
#define ST7789_SLPOUT   0x11
#define ST7789_NORON    0x13
#define ST7789_MADCTL   0x36
#define ST7789_COLMOD   0x3A
#define ST7789_PORCTRL  0xB2
#define ST7789_GCTRL    0xB7
#define ST7789_VCOMS    0xBB
#define ST7789_LCMCTRL  0xC0
#define ST7789_VDVVRHEN 0xC2
#define ST7789_VRHS     0xC3
#define ST7789_VDVS     0xC4
#define ST7789_FRCTRL2  0xC6
#define ST7789_PWCTRL1  0xD0
#define ST7789_PVGAMCTRL 0xE0
#define ST7789_NVGAMCTRL 0xE1
#define ST7789_INVON    0x21
#define ST7789_CASET    0x2A
#define ST7789_RASET    0x2B
#define ST7789_RAMWR    0x2C
#define ST7789_DISPON   0x29

// ============================================================================
// SPI Communication
// ============================================================================

static void st7789_write_cmd(uint8_t cmd) {
    gpio_set_level((gpio_num_t)TFT_DC, 0);  // Command mode
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    spi_device_polling_transmit(spi, &t);
}

static void st7789_write_data(uint8_t data) {
    gpio_set_level((gpio_num_t)TFT_DC, 1);  // Data mode
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &data;
    spi_device_polling_transmit(spi, &t);
}

static void st7789_write_data_buf(const uint8_t *data, int len) {
    if (len == 0) return;
    gpio_set_level((gpio_num_t)TFT_DC, 1);  // Data mode
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = data;
    spi_device_polling_transmit(spi, &t);
}

// ============================================================================
// ST7789 Initialization
// ============================================================================

static void st7789_init(void) {
    // Hardware reset
    gpio_set_level((gpio_num_t)TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level((gpio_num_t)TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Software reset and initialization sequence
    st7789_write_cmd(ST7789_SLPOUT);  // Sleep out
    vTaskDelay(pdMS_TO_TICKS(120));

    st7789_write_cmd(ST7789_MADCTL);
    // Default orientation. Adjust by setting TFT_ROTATION in config if needed.
    // 0x00 = default (RGB order). Some modules require 0x60/0xC0 to rotate display.
    st7789_write_data(0x00);  // RGB order, vertical refresh

    st7789_write_cmd(ST7789_COLMOD);
    st7789_write_data(0x55);  // 16-bit color

    st7789_write_cmd(ST7789_INVON);   // Inversion ON

    st7789_write_cmd(ST7789_NORON);   // Normal display on
    vTaskDelay(pdMS_TO_TICKS(10));

    st7789_write_cmd(ST7789_DISPON);  // Display on
    vTaskDelay(pdMS_TO_TICKS(10));

    // Turn on backlight
    gpio_set_level((gpio_num_t)TFT_BL, 1);

    ESP_LOGI(TAG, "ST7789 initialized");
    ESP_LOGI(TAG, "TFT offsets: X=%d, Y=%d", TFT_OFFSET_X, TFT_OFFSET_Y);
}

// ============================================================================
// LVGL Display Flush Callback
// ============================================================================

static void st7789_set_addr_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    // Apply hardware offsets for modules that require it
    x += (uint16_t)g_tft_offset_x;
    y += (uint16_t)g_tft_offset_y;
    uint16_t x_end = x + w - 1;
    uint16_t y_end = y + h - 1;

    st7789_write_cmd(ST7789_CASET);
    st7789_write_data(x >> 8);
    st7789_write_data(x & 0xFF);
    st7789_write_data(x_end >> 8);
    st7789_write_data(x_end & 0xFF);

    st7789_write_cmd(ST7789_RASET);
    st7789_write_data(y >> 8);
    st7789_write_data(y & 0xFF);
    st7789_write_data(y_end >> 8);
    st7789_write_data(y_end & 0xFF);

    st7789_write_cmd(ST7789_RAMWR);
}

static void disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    int32_t x = area->x1;
    int32_t y = area->y1;
    int32_t w = area->x2 - area->x1 + 1;
    int32_t h = area->y2 - area->y1 + 1;

    st7789_set_addr_window(x, y, w, h);
    st7789_write_data_buf((uint8_t *)color_p, w * h * 2);

    lv_disp_flush_ready(disp_drv);
}

// ============================================================================
// Display Initialization
// ============================================================================

void display_init(void) {
    ESP_LOGI(TAG, "Initializing display...");

    // Configure GPIO pins
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << TFT_DC) | (1ULL << TFT_RST) | (1ULL << TFT_BL);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Initialize backlight off
    gpio_set_level((gpio_num_t)TFT_BL, 0);

    // SPI bus configuration
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = TFT_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = TFT_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = TFT_WIDTH * TFT_HEIGHT * 2;

    // SPI device configuration
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 26 * 1000 * 1000;  // 26 MHz (max supported)
    devcfg.mode = 0;
    devcfg.spics_io_num = TFT_CS;
    devcfg.queue_size = 7;

    // Initialize SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    // Initialize ST7789
    st7789_init();

    // Initialize LVGL
    lv_init();

    // Initialize display buffers (single buffer to save memory)
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, TFT_WIDTH * 10);

    // Initialize display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = TFT_WIDTH;
    disp_drv.ver_res = TFT_HEIGHT;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp = lv_disp_drv_register(&disp_drv);

    // Create UI
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    // Title
    label_title = lv_label_create(scr);
    lv_label_set_text(label_title, "Smart Glove");
    lv_obj_set_style_text_color(label_title, lv_color_white(), 0);
    lv_obj_set_style_text_font(label_title, &lv_font_montserrat_14, 0);
    lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 5);

    // Bluetooth status (connection indicator)
    label_bt_status = lv_label_create(scr);
    lv_label_set_text(label_bt_status, "BT: Waiting...");
    lv_obj_set_style_text_color(label_bt_status, lv_color_make(255, 200, 100), 0);
    lv_obj_set_style_text_align(label_bt_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label_bt_status, LV_ALIGN_TOP_MID, 0, 25);

    // Track info (title and artist for music, or call status)
    label_track_info = lv_label_create(scr);
    lv_label_set_text(label_track_info, "");
    lv_obj_set_style_text_color(label_track_info, lv_color_make(150, 220, 255), 0);
    lv_obj_set_style_text_align(label_track_info, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(label_track_info, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_width(label_track_info, TFT_WIDTH - 10);
    lv_obj_align(label_track_info, LV_ALIGN_TOP_MID, 0, 45);

    // Accelerometer values (smaller, at bottom)
    label_accel_x = lv_label_create(scr);
    lv_label_set_text(label_accel_x, "X:0");
    lv_obj_set_style_text_color(label_accel_x, lv_color_make(100, 200, 255), 0);
    lv_obj_align(label_accel_x, LV_ALIGN_BOTTOM_LEFT, 5, -25);

    label_accel_y = lv_label_create(scr);
    lv_label_set_text(label_accel_y, "Y:0");
    lv_obj_set_style_text_color(label_accel_y, lv_color_make(100, 255, 100), 0);
    lv_obj_align(label_accel_y, LV_ALIGN_BOTTOM_MID, 0, -25);

    label_accel_z = lv_label_create(scr);
    lv_label_set_text(label_accel_z, "Z:0");
    lv_obj_set_style_text_color(label_accel_z, lv_color_make(255, 200, 100), 0);
    lv_obj_align(label_accel_z, LV_ALIGN_BOTTOM_RIGHT, -5, -25);

    // Message label
    label_message = lv_label_create(scr);
    lv_label_set_text(label_message, "");
    lv_obj_set_style_text_color(label_message, lv_color_white(), 0);
    lv_obj_set_style_text_align(label_message, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label_message, LV_ALIGN_BOTTOM_MID, 0, -5);

    ESP_LOGI(TAG, "Display initialized successfully");
}

// Adjust hardware offsets at runtime (helps tune mapping for specific modules)
void display_set_offsets(int x_offset, int y_offset) {
    // Note: Changing these values affects only future flushes; LVGL redraw is needed.
    // We simply update the defines via a static variable in code for tuning.
    // This function is best-effort for debugging; a more robust approach would
    // add offsets into st7789_set_addr_window parameters.
    ESP_LOGI(TAG, "Setting display offsets to X=%d, Y=%d (reinit not required)", x_offset, y_offset);
    // Update the global offsets used by the driver
    g_tft_offset_x = x_offset;
    g_tft_offset_y = y_offset;
}

// ============================================================================
// Display Update Functions
// ============================================================================

void display_update_accel(int16_t x, int16_t y, int16_t z) {
    char buf[32];

    snprintf(buf, sizeof(buf), "X:%d", x);
    lv_label_set_text(label_accel_x, buf);

    snprintf(buf, sizeof(buf), "Y:%d", y);
    lv_label_set_text(label_accel_y, buf);

    snprintf(buf, sizeof(buf), "Z:%d", z);
    lv_label_set_text(label_accel_z, buf);
}

void display_update_bluetooth_state(bool connected, bool playing, bool in_call,
                                     const char *track_title, const char *track_artist) {
    // Update connection status
    if (!connected) {
        lv_label_set_text(label_bt_status, "BT: Waiting...");
        lv_obj_set_style_text_color(label_bt_status, lv_color_make(255, 200, 100), 0);
        lv_label_set_text(label_track_info, "SmartGlove");
        lv_obj_set_style_text_color(label_track_info, lv_color_make(150, 150, 150), 0);
    } else if (in_call) {
        lv_label_set_text(label_bt_status, "BT: CALL");
        lv_obj_set_style_text_color(label_bt_status, lv_color_make(255, 100, 100), 0);
        lv_label_set_text(label_track_info, "Phone Call\nActive");
        lv_obj_set_style_text_color(label_track_info, lv_color_make(255, 150, 150), 0);
    } else if (playing) {
        lv_label_set_text(label_bt_status, "BT: PLAYING");
        lv_obj_set_style_text_color(label_bt_status, lv_color_make(100, 255, 100), 0);
        
        // Show track info if available
        if (track_title && track_title[0] != '\0') {
            char info_buf[256];
            if (track_artist && track_artist[0] != '\0') {
                snprintf(info_buf, sizeof(info_buf), "%s\n%s", track_title, track_artist);
            } else {
                snprintf(info_buf, sizeof(info_buf), "%s", track_title);
            }
            lv_label_set_text(label_track_info, info_buf);
            lv_obj_set_style_text_color(label_track_info, lv_color_make(150, 220, 255), 0);
        } else {
            lv_label_set_text(label_track_info, "Playing Music");
            lv_obj_set_style_text_color(label_track_info, lv_color_make(150, 220, 255), 0);
        }
    } else {
        lv_label_set_text(label_bt_status, "BT: CONNECTED");
        lv_obj_set_style_text_color(label_bt_status, lv_color_make(100, 255, 100), 0);
        lv_label_set_text(label_track_info, "Ready");
        lv_obj_set_style_text_color(label_track_info, lv_color_make(150, 220, 255), 0);
    }
}

void display_show_message(const char *message) {
    lv_label_set_text(label_message, message);
}

void display_lvgl_tick(void) {
    lv_timer_handler();
}
