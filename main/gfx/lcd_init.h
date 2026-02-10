#ifndef LCD_INIT_H
#define LCD_INIT_H

#include "esp_err.h"
#include "esp_lcd_types.h"
#include "sdkconfig.h"
#include <stdint.h>

// LCD initialization function
esp_err_t lcd_init(esp_lcd_panel_handle_t *panel_handle);

// Display refresh function to fix display corruption after NVS operations
void lcd_force_display_refresh(void);

// Backlight control functions
esp_err_t lcd_backlight_init(void);
void lcd_set_backlight_level(uint8_t level);
uint8_t lcd_get_backlight_level(void);
void lcd_set_backlight_enabled(bool enabled);
bool lcd_is_backlight_enabled(void);

#if CONFIG_IDF_TARGET_ESP32P4
// Initialize I2C backlight controller on P4.
// Must be called after touch_init() (uses the touch I2C bus).
esp_err_t lcd_backlight_i2c_init(void);
#endif

#endif // LCD_INIT_H
