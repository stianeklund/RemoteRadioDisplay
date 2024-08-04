#ifndef LVGL_INIT_H
#define LVGL_INIT_H

#include "esp_err.h"
#include "esp_lcd_types.h"
#include "esp_lcd_touch.h"

esp_err_t lvgl_init(esp_lcd_panel_handle_t panel_handle, esp_lcd_touch_handle_t tp);

#endif // LVGL_INIT_H