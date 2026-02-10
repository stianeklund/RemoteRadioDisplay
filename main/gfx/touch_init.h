#ifndef TOUCH_INIT_H
#define TOUCH_INIT_H

#include "esp_err.h"
#include "esp_lcd_touch.h"
#include "driver/i2c_master.h"

esp_err_t touch_init(esp_lcd_touch_handle_t *tp);

// Returns the I2C master bus handle created during touch_init().
// Other devices on the same physical bus (e.g. backlight IC) can use this
// to add themselves via i2c_master_bus_add_device().
i2c_master_bus_handle_t touch_get_i2c_bus(void);

#endif // TOUCH_INIT_H