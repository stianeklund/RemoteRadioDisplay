#ifndef LVGL_INIT_H
#define LVGL_INIT_H

#include "esp_err.h"
#include "esp_lcd_types.h"
#include "esp_lcd_touch.h"
#include "lvgl.h"

/**
 * @brief Initialize LVGL with esp_lvgl_port
 *
 * This function initializes LVGL using the esp_lvgl_port component which handles:
 * - Display driver registration (RGB panel)
 * - Touch input registration (GT911)
 * - LVGL tick timer
 * - LVGL task management
 * - Thread-safe mutex (use lvgl_port_lock/unlock)
 *
 * @param panel_handle LCD panel handle from lcd_init()
 * @param tp Touch handle from touch_init()
 * @return ESP_OK on success
 */
esp_err_t lvgl_init(esp_lcd_panel_handle_t panel_handle, esp_lcd_touch_handle_t tp);

/**
 * @brief Get the LVGL display handle
 * @return LVGL display pointer
 */
lv_display_t *lvgl_get_display(void);

/**
 * @brief Get the LVGL touch input device handle
 * @return LVGL input device pointer
 */
lv_indev_t *lvgl_get_touch_indev(void);

// Note: For thread-safe LVGL access, use lvgl_port_lock() and lvgl_port_unlock()
// from esp_lvgl_port.h instead of the old get_lvgl_mutex() pattern.

#endif // LVGL_INIT_H
