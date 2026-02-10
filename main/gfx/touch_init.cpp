#include "touch_init.h"
#include "lcd_config.h"
#include "sdkconfig.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_log.h"

static const char *TAG = "TOUCH_INIT";
static i2c_master_bus_handle_t s_i2c_bus_handle = nullptr;

esp_err_t touch_init(esp_lcd_touch_handle_t *tp) {
    esp_err_t ret;

    ESP_LOGI(TAG, "Initialize I2C bus");

    const i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = TOUCH_I2C_HOST,
        .sda_io_num = static_cast<gpio_num_t>(TOUCH_SDA_PIN),
        .scl_io_num = static_cast<gpio_num_t>(TOUCH_SCL_PIN),
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,  // Synchronous mode
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    ret = i2c_new_master_bus(&i2c_bus_config, &s_i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C master bus created (SDA:%d, SCL:%d)", TOUCH_SDA_PIN, TOUCH_SCL_PIN);

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_config.scl_speed_hz = TOUCH_FREQUENCY;  // Clock speed is per-device in new driver

    ret = esp_lcd_new_panel_io_i2c(s_i2c_bus_handle, &tp_io_config, &tp_io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create touch panel IO: %s", esp_err_to_name(ret));
        return ret;
    }

#if CONFIG_IDF_TARGET_ESP32P4
    // Touch reports in native panel coordinates (portrait 720x1280)
    // LVGL rotation handles the landscape coordinate mapping
    const uint16_t touch_x_max = DSI_LCD_H_RES;
    const uint16_t touch_y_max = DSI_LCD_V_RES;
#else
    const uint16_t touch_x_max = H_RES;
    const uint16_t touch_y_max = V_RES;
#endif

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = touch_x_max,
        .y_max = touch_y_max,
        .rst_gpio_num = TOUCH_RST_PIN,
        .int_gpio_num = TOUCH_INT_PIN,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    if (HW_ROTATE_180) {
        tp_cfg.flags.mirror_x = 1;
        tp_cfg.flags.mirror_y = 1;
    }

    if (SW_ROTATE_180) {
        tp_cfg.flags.mirror_x = 1;
        tp_cfg.flags.mirror_y = 1;
    }

    ret = esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, tp);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create touch handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "GT911 touch controller initialized (I2C:%dHz, Res:%dx%d)",
             TOUCH_FREQUENCY, tp_cfg.x_max, tp_cfg.y_max);

    return ESP_OK;
}

i2c_master_bus_handle_t touch_get_i2c_bus(void) {
    return s_i2c_bus_handle;
}
