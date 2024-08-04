#include "lcd_init.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "lcd_config.h"
#include "driver/gpio.h"
#include "esp_lcd_types.h"
#include "esp_log.h"

static const char *TAG = "LCD_INIT";

esp_err_t lcd_init(esp_lcd_panel_handle_t *panel_handle)
{
    esp_err_t ret;


    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << BK_LIGHT_PIN_NUM
    };
    ret = gpio_config(&bk_gpio_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure backlight GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(BK_LIGHT_PIN_NUM, 0);

    ESP_LOGI(TAG, "Install RGB LCD panel driver");

    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16,
        .sram_trans_align = 0,
        .psram_trans_align = 64,
        .num_fbs = 1,
        .clk_src = LCD_CLK_SRC_PLL160M,
        .disp_gpio_num = DISP_PIN_NUM,
        .pclk_gpio_num = PCLK_PIN_NUM,
        .vsync_gpio_num = VSYNC_PIN_NUM,
        .hsync_gpio_num = HSYNC_PIN_NUM,
        .de_gpio_num = DE_PIN_NUM,
        .data_gpio_nums = {
            DATA00_PIN_NUM, DATA01_PIN_NUM, DATA02_PIN_NUM, DATA03_PIN_NUM,
            DATA04_PIN_NUM, DATA05_PIN_NUM, DATA06_PIN_NUM, DATA07_PIN_NUM,
            DATA08_PIN_NUM, DATA09_PIN_NUM, DATA10_PIN_NUM, DATA11_PIN_NUM,
            DATA12_PIN_NUM, DATA13_PIN_NUM, DATA14_PIN_NUM, DATA15_PIN_NUM,
        },
        .timings = {
            .pclk_hz = PIXEL_CLOCK_HZ,
            .h_res = H_RES,
            .v_res = V_RES,
            .hsync_back_porch = HSYNC_BACK_PORCH,
            .hsync_front_porch = HSYNC_FRONT_PORCH,
            .hsync_pulse_width = HSYNC_PULSE_WIDTH,
            .vsync_back_porch = VSYNC_BACK_PORCH,
            .vsync_front_porch = VSYNC_FRONT_PORCH,
            .vsync_pulse_width = VSYNC_PULSE_WIDTH,
            .flags = {
                .pclk_active_neg = PCLK_ACTIVE_NEG,
                .de_idle_high = DE_IDLE_HIGH,
                .pclk_idle_high = PCLK_IDLE_HIGH,
                .vsync_idle_low = VSYNC_IDLE_LOW,
                .hsync_idle_low = HSYNC_IDLE_LOW
            }
        },
        .flags = {
            .fb_in_psram = 1,
            .disp_active_low = DISP_ACTIVE_LOW,
            .refresh_on_demand = 0,
            .no_fb = 0,
            .bb_invalidate_cache = 0
        }
    };

    ret = esp_lcd_new_rgb_panel(&panel_config, panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install RGB LCD panel driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_lcd_panel_reset(*panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset LCD panel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_lcd_panel_init(*panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LCD panel: %s", esp_err_to_name(ret));
        return ret;
    }


if (ROTATE_180) {
    ret = esp_lcd_panel_mirror(*panel_handle, true, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mirror xy coords: %s", esp_err_to_name(ret));
        return ret;
    }
}


    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(BK_LIGHT_PIN_NUM, 1);

    return ESP_OK;
}