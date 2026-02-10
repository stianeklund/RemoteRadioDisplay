/**
 * @file lvgl_init.cpp
 * @brief LVGL initialization using esp_lvgl_port component
 *
 * This module initializes LVGL with the esp_lvgl_port component which handles:
 * - Display driver registration
 * - Touch input registration
 * - LVGL tick timer
 * - LVGL task management
 * - Thread-safe mutex handling
 */

#include "lvgl_init.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "lcd_config.h"
#include "sdkconfig.h"
#include "radio/radio_subjects.h"  // For radio_subjects_init()

#if CONFIG_IDF_TARGET_ESP32S3
#include "esp_lcd_panel_rgb.h"
#endif

static const char *TAG = "LVGL_INIT";

static lv_display_t *s_disp = NULL;
static lv_indev_t *s_touch_indev = NULL;

esp_err_t lvgl_init(esp_lcd_panel_handle_t panel_handle, esp_lcd_touch_handle_t tp)
{
    ESP_LOGI(TAG, "Initializing LVGL with esp_lvgl_port");

    // Initialize esp_lvgl_port
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = LVGL_TASK_PRIORITY,
        .task_stack = LVGL_TASK_STACK_SIZE,
        .task_affinity = 1,  // Pin to core 1
        .task_max_sleep_ms = LVGL_TASK_MAX_DELAY_MS,
        .task_stack_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DEFAULT,
        .timer_period_ms = LVGL_TICK_PERIOD_MS,
    };
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

#if CONFIG_IDF_TARGET_ESP32S3
    // ============================================================
    // S3: RGB display with direct framebuffer in PSRAM
    // ============================================================
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = NULL,  // Not needed for RGB panels
        .panel_handle = panel_handle,
        .control_handle = NULL,
        .buffer_size = H_RES * 20,  // 20 lines in internal SRAM (~32KB)
        .double_buffer = false,
        .trans_size = 0,
        .hres = H_RES,
        .vres = V_RES,
        .monochrome = false,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_dma = false,
            .buff_spiram = false,
            .sw_rotate = true,    // SW rotation: internal SRAM -> PSRAM framebuffer
            .swap_bytes = false,
            .full_refresh = false,
            .direct_mode = false,
        },
    };

    const lvgl_port_display_rgb_cfg_t rgb_cfg = {
        .flags = {
            .bb_mode = (LCD_RGB_BOUNCE_BUFFER_HEIGHT > 0),
            .avoid_tearing = LCD_RGB_AVOID_TEARING,
        },
    };

    s_disp = lvgl_port_add_disp_rgb(&disp_cfg, &rgb_cfg);
    if (s_disp == NULL) {
        ESP_LOGE(TAG, "Failed to add RGB display");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "RGB display added: %dx%d", H_RES, V_RES);

    // Apply 180 degree rotation using LVGL 9 native API
    if (lvgl_port_lock(1000)) {
        lv_display_set_rotation(s_disp, LV_DISPLAY_ROTATION_180);
        lvgl_port_unlock();
        ESP_LOGI(TAG, "Display rotation set to 180 degrees (draw buffer in internal SRAM)");
    } else {
        ESP_LOGE(TAG, "Failed to acquire LVGL lock for rotation");
    }

#elif CONFIG_IDF_TARGET_ESP32P4
    // ============================================================
    // P4: DSI display (720x1280 native, LVGL rotated to 1280x720)
    // ============================================================
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = NULL,
        .panel_handle = panel_handle,
        .control_handle = NULL,
        .buffer_size = DSI_LCD_H_RES * 50,  // 50 lines (~72KB at RGB565) - P4 has 32MB PSRAM
        .double_buffer = true,
        .trans_size = 0,
        .hres = DSI_LCD_H_RES,   // Native 720
        .vres = DSI_LCD_V_RES,   // Native 1280
        .monochrome = false,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,   // P4 has 32MB PSRAM, use it for buffers
            .sw_rotate = true,     // SW rotation for landscape
            .swap_bytes = false,
            .full_refresh = false,
            .direct_mode = false,
        },
    };

    const lvgl_port_display_dsi_cfg_t dsi_cfg = {
        .flags = {
            .avoid_tearing = false,
        },
    };

    s_disp = lvgl_port_add_disp_dsi(&disp_cfg, &dsi_cfg);
    if (s_disp == NULL) {
        ESP_LOGE(TAG, "Failed to add DSI display");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "DSI display added: %dx%d (native)", DSI_LCD_H_RES, DSI_LCD_V_RES);

    // Rotate 270 degrees (-90) for landscape (720x1280 -> 1280x720)
    if (lvgl_port_lock(1000)) {
        lv_display_set_rotation(s_disp, LV_DISPLAY_ROTATION_270);
        lvgl_port_unlock();
        ESP_LOGI(TAG, "Display rotation set to 270 degrees (landscape %dx%d)", H_RES, V_RES);
    } else {
        ESP_LOGE(TAG, "Failed to acquire LVGL lock for rotation");
    }
#endif

    // Add touch input (shared across targets - GT911 on both)
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = s_disp,
        .handle = tp,
        .scale = {
            .x = 1.0f,
            .y = 1.0f,
        },
    };

    s_touch_indev = lvgl_port_add_touch(&touch_cfg);
    if (s_touch_indev == NULL) {
        ESP_LOGE(TAG, "Failed to add touch input");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Touch input added");

    // Initialize LVGL 9 observer subjects for radio state
    radio_subjects_init();

    ESP_LOGI(TAG, "LVGL initialization complete");
    return ESP_OK;
}

lv_display_t *lvgl_get_display(void)
{
    return s_disp;
}

lv_indev_t *lvgl_get_touch_indev(void)
{
    return s_touch_indev;
}
