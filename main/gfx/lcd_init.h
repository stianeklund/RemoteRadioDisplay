#ifndef LCD_INIT_H
#define LCD_INIT_H

#include "esp_err.h"
#include "esp_lcd_types.h"

// LCD initialization function
esp_err_t lcd_init(esp_lcd_panel_handle_t *panel_handle);

#endif // LCD_INIT_H