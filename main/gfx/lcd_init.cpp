#include "lcd_init.h"
#include "lcd_config.h"
#include "driver/gpio.h"
#include "esp_idf_version.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"
#include "esp_log.h"
#include "lvgl.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <math.h>

#if CONFIG_IDF_TARGET_ESP32S3
#include "driver/ledc.h"
#include "esp_lcd_panel_rgb.h"
#elif CONFIG_IDF_TARGET_ESP32P4
#include "driver/i2c_master.h"
#include "touch_init.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_hx8394.h"
#include "esp_ldo_regulator.h"
#endif

static const char *TAG = "LCD_INIT";

#define GAMMA_CORRECTION 2.2f

static uint8_t current_backlight_level = 200;
static uint8_t saved_backlight_level = 200;

#if CONFIG_IDF_TARGET_ESP32S3
// S3 Sunton: backlight driver only responds in a narrow high-duty band
#define MIN_SAFE_HW_LEVEL 230
#define MAX_PERCEPTUAL_HW_LEVEL 245

#define LCD_BACKLIGHT_LEDC_TIMER LEDC_TIMER_0
#define LCD_BACKLIGHT_LEDC_MODE LEDC_LOW_SPEED_MODE
#define LCD_BACKLIGHT_LEDC_CHANNEL LEDC_CHANNEL_0
#define LCD_BACKLIGHT_LEDC_RESOLUTION LEDC_TIMER_8_BIT
#define LCD_BACKLIGHT_LEDC_FREQ_HZ 9000

#elif CONFIG_IDF_TARGET_ESP32P4
// P4 Guition: backlight controlled via I2C IC at address 0x45, register 0x96
// The HX8394 driver sets full brightness during panel init; we take over after touch_init()
#define BACKLIGHT_I2C_ADDR 0x45
#define BACKLIGHT_I2C_REG  0x96

static i2c_master_dev_handle_t s_backlight_i2c_dev = nullptr;

// DSI handles kept module-scoped for potential future use (e.g., power management)
static esp_ldo_channel_handle_t s_ldo_mipi_phy = NULL;
static esp_lcd_dsi_bus_handle_t s_mipi_dsi_bus = NULL;
static esp_lcd_panel_io_handle_t s_mipi_dbi_io = NULL;
#endif

// ============================================================
// LCD Initialization
// ============================================================

#if CONFIG_IDF_TARGET_ESP32S3

esp_err_t lcd_init(esp_lcd_panel_handle_t *panel_handle)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initialize LCD backlight PWM");
    ret = lcd_backlight_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize backlight PWM: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Install RGB LCD panel driver");

    esp_lcd_rgb_panel_config_t panel_config = {};
    panel_config.clk_src = LCD_CLK_SRC_PLL160M;
    panel_config.data_width = 16;
    panel_config.bits_per_pixel = 16;
    panel_config.sram_trans_align = 8;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 0)
    panel_config.psram_trans_align = 64;
#else
    panel_config.dma_burst_size = 64;
#endif
    panel_config.num_fbs = 2;
#if LCD_RGB_BOUNCE_BUFFER_HEIGHT > 0
    panel_config.bounce_buffer_size_px = H_RES * LCD_RGB_BOUNCE_BUFFER_HEIGHT;
#else
    panel_config.bounce_buffer_size_px = 0;
#endif
    panel_config.hsync_gpio_num = HSYNC_PIN_NUM;
    panel_config.vsync_gpio_num = VSYNC_PIN_NUM;
    panel_config.de_gpio_num = DE_PIN_NUM;
    panel_config.pclk_gpio_num = PCLK_PIN_NUM;
    panel_config.disp_gpio_num = DISP_PIN_NUM;
    panel_config.data_gpio_nums[0] = DATA00_PIN_NUM;
    panel_config.data_gpio_nums[1] = DATA01_PIN_NUM;
    panel_config.data_gpio_nums[2] = DATA02_PIN_NUM;
    panel_config.data_gpio_nums[3] = DATA03_PIN_NUM;
    panel_config.data_gpio_nums[4] = DATA04_PIN_NUM;
    panel_config.data_gpio_nums[5] = DATA05_PIN_NUM;
    panel_config.data_gpio_nums[6] = DATA06_PIN_NUM;
    panel_config.data_gpio_nums[7] = DATA07_PIN_NUM;
    panel_config.data_gpio_nums[8] = DATA08_PIN_NUM;
    panel_config.data_gpio_nums[9] = DATA09_PIN_NUM;
    panel_config.data_gpio_nums[10] = DATA10_PIN_NUM;
    panel_config.data_gpio_nums[11] = DATA11_PIN_NUM;
    panel_config.data_gpio_nums[12] = DATA12_PIN_NUM;
    panel_config.data_gpio_nums[13] = DATA13_PIN_NUM;
    panel_config.data_gpio_nums[14] = DATA14_PIN_NUM;
    panel_config.data_gpio_nums[15] = DATA15_PIN_NUM;
    panel_config.timings.pclk_hz = PIXEL_CLOCK_HZ;
    panel_config.timings.h_res = H_RES;
    panel_config.timings.v_res = V_RES;
    panel_config.timings.hsync_back_porch = HSYNC_BACK_PORCH;
    panel_config.timings.hsync_front_porch = HSYNC_FRONT_PORCH;
    panel_config.timings.hsync_pulse_width = HSYNC_PULSE_WIDTH;
    panel_config.timings.vsync_back_porch = VSYNC_BACK_PORCH;
    panel_config.timings.vsync_front_porch = VSYNC_FRONT_PORCH;
    panel_config.timings.vsync_pulse_width = VSYNC_PULSE_WIDTH;
    panel_config.timings.flags.pclk_active_neg = PCLK_ACTIVE_NEG;
    panel_config.timings.flags.de_idle_high = DE_IDLE_HIGH;
    panel_config.timings.flags.pclk_idle_high = PCLK_IDLE_HIGH;
    panel_config.timings.flags.vsync_idle_low = VSYNC_IDLE_LOW;
    panel_config.timings.flags.hsync_idle_low = HSYNC_IDLE_LOW;
    panel_config.flags.fb_in_psram = 1;
    panel_config.flags.disp_active_low = DISP_ACTIVE_LOW;
    panel_config.flags.refresh_on_demand = 0;
    panel_config.flags.no_fb = 0;
    panel_config.flags.bb_invalidate_cache = 1;

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

    if (HW_ROTATE_180) {
        ret = esp_lcd_panel_mirror(*panel_handle, true, true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to mirror xy coords: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    ESP_LOGI(TAG, "Turn on LCD backlight");
    lcd_set_backlight_level(current_backlight_level);

    return ESP_OK;
}

#elif CONFIG_IDF_TARGET_ESP32P4

esp_err_t lcd_init(esp_lcd_panel_handle_t *panel_handle)
{
    esp_err_t ret;

    // P4 backlight is I2C-controlled — deferred to lcd_backlight_i2c_init() after touch_init()

    // Step 1: Power MIPI DSI PHY via internal LDO
    ESP_LOGI(TAG, "Power on MIPI DSI PHY (LDO chan %d, %dmV)", DSI_PHY_LDO_CHAN, DSI_PHY_LDO_VOLTAGE_MV);
    esp_ldo_channel_config_t ldo_config = {
        .chan_id = DSI_PHY_LDO_CHAN,
        .voltage_mv = DSI_PHY_LDO_VOLTAGE_MV,
    };
    ret = esp_ldo_acquire_channel(&ldo_config, &s_ldo_mipi_phy);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to acquire LDO channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Step 2: Create MIPI DSI bus
    // Note: HX8394_PANEL_BUS_DSI_2CH_CONFIG() macro is C-only, init manually for C++
    ESP_LOGI(TAG, "Create MIPI DSI bus (2 lanes, 700Mbps)");
    esp_lcd_dsi_bus_config_t bus_config = {};
    bus_config.bus_id = 0;
    bus_config.num_data_lanes = 2;
    bus_config.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
    bus_config.lane_bit_rate_mbps = 700;
    ret = esp_lcd_new_dsi_bus(&bus_config, &s_mipi_dsi_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create DSI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Step 3: Create DBI IO (command channel to LCD controller)
    ESP_LOGI(TAG, "Create MIPI DBI IO");
    esp_lcd_dbi_io_config_t dbi_config = {};
    dbi_config.virtual_channel = 0;
    dbi_config.lcd_cmd_bits = 8;
    dbi_config.lcd_param_bits = 8;
    ret = esp_lcd_new_panel_io_dbi(s_mipi_dsi_bus, &dbi_config, &s_mipi_dbi_io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create DBI IO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Step 4: Configure DPI panel (pixel data channel) - 720x1280 portrait, 30Hz
    // Note: HX8394_720_1280_PANEL_30HZ_DPI_CONFIG() macro is C-only, init manually for C++
    ESP_LOGI(TAG, "Configure DPI panel (%dx%d)", DSI_LCD_H_RES, DSI_LCD_V_RES);
    esp_lcd_dpi_panel_config_t dpi_config = {};
    dpi_config.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi_config.dpi_clock_freq_mhz = 58;
    dpi_config.virtual_channel = 0;
    dpi_config.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
    dpi_config.num_fbs = 1;
    dpi_config.video_timing.h_size = DSI_LCD_H_RES;
    dpi_config.video_timing.v_size = DSI_LCD_V_RES;
    dpi_config.video_timing.hsync_back_porch = 20;
    dpi_config.video_timing.hsync_pulse_width = 20;
    dpi_config.video_timing.hsync_front_porch = 40;
    dpi_config.video_timing.vsync_back_porch = 10;
    dpi_config.video_timing.vsync_pulse_width = 4;
    dpi_config.video_timing.vsync_front_porch = 24;
    dpi_config.flags.use_dma2d = true;

    // Step 5: Create HX8394 panel with vendor-specific init sequence
    hx8394_vendor_config_t vendor_config = {
        .init_cmds = NULL,       // Use default init commands from driver
        .init_cmds_size = 0,
        .mipi_config = {
            .dsi_bus = s_mipi_dsi_bus,
            .dpi_config = &dpi_config,
            .lane_num = 2,
        },
    };

    esp_lcd_panel_dev_config_t lcd_dev_config = {};
    lcd_dev_config.reset_gpio_num = LCD_RST_PIN;
    lcd_dev_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    lcd_dev_config.bits_per_pixel = 16;
    lcd_dev_config.vendor_config = &vendor_config;

    ret = esp_lcd_new_panel_hx8394(s_mipi_dbi_io, &lcd_dev_config, panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create HX8394 panel: %s", esp_err_to_name(ret));
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

    // Backlight is at full brightness from HX8394 init; lcd_backlight_i2c_init()
    // will take over control after touch_init() provides the I2C bus
    ESP_LOGI(TAG, "DSI LCD initialized: %dx%d (native), landscape %dx%d",
             DSI_LCD_H_RES, DSI_LCD_V_RES, H_RES, V_RES);

    return ESP_OK;
}

#endif // CONFIG_IDF_TARGET

// ============================================================
// Backlight control
// ============================================================

#if CONFIG_IDF_TARGET_ESP32S3

esp_err_t lcd_backlight_init(void)
{
    esp_err_t ret;

    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LCD_BACKLIGHT_LEDC_MODE;
    ledc_timer.timer_num = LCD_BACKLIGHT_LEDC_TIMER;
    ledc_timer.duty_resolution = LCD_BACKLIGHT_LEDC_RESOLUTION;
    ledc_timer.freq_hz = LCD_BACKLIGHT_LEDC_FREQ_HZ;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer configuration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.speed_mode = LCD_BACKLIGHT_LEDC_MODE;
    ledc_channel.channel = LCD_BACKLIGHT_LEDC_CHANNEL;
    ledc_channel.timer_sel = LCD_BACKLIGHT_LEDC_TIMER;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num = BK_LIGHT_PIN_NUM;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel configuration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

void lcd_set_backlight_level(uint8_t level)
{
    if (level > 0) {
        saved_backlight_level = level;
    }
    current_backlight_level = level;

    const uint32_t duty_max = (1u << LCD_BACKLIGHT_LEDC_RESOLUTION) - 1u;
    uint32_t duty_cycle;

    if (level == 0) {
        duty_cycle = 0;
    } else {
        float normalized = (level - 1) / 254.0f;
        float gamma_corrected = powf(normalized, 1.0f / GAMMA_CORRECTION);
        if (gamma_corrected > 1.0f) gamma_corrected = 1.0f;
        if (gamma_corrected < 0.0f) gamma_corrected = 0.0f;

        uint32_t duty_min = duty_max * MIN_SAFE_HW_LEVEL / 255u;
        uint32_t duty_perceptual_max = duty_max * MAX_PERCEPTUAL_HW_LEVEL / 255u;

        duty_cycle = duty_min + (uint32_t)(gamma_corrected * (duty_perceptual_max - duty_min));

        if (duty_cycle < duty_min) duty_cycle = duty_min;
        if (duty_cycle > duty_perceptual_max) duty_cycle = duty_perceptual_max;
        if (duty_cycle > duty_max) duty_cycle = duty_max;
    }

    #if BK_LIGHT_ON_LEVEL == 0
    duty_cycle = duty_max - duty_cycle;
    #endif

    ledc_set_duty(LCD_BACKLIGHT_LEDC_MODE, LCD_BACKLIGHT_LEDC_CHANNEL, duty_cycle);
    ledc_update_duty(LCD_BACKLIGHT_LEDC_MODE, LCD_BACKLIGHT_LEDC_CHANNEL);

    uint8_t equiv_hw_level = (uint8_t)((duty_cycle * 255u) / duty_max);
    ESP_LOGV(TAG, "level=%u -> duty=%" PRIu32 "/%" PRIu32 " (equiv_hw=%u)",
             level, duty_cycle, duty_max, equiv_hw_level);
}

#elif CONFIG_IDF_TARGET_ESP32P4

esp_err_t lcd_backlight_init(void)
{
    // P4 backlight is I2C-controlled; actual init deferred to lcd_backlight_i2c_init()
    return ESP_OK;
}

esp_err_t lcd_backlight_i2c_init(void)
{
    i2c_master_bus_handle_t i2c_bus = touch_get_i2c_bus();
    if (i2c_bus == nullptr) {
        ESP_LOGE(TAG, "Touch I2C bus not available for backlight init");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = BACKLIGHT_I2C_ADDR;
    dev_config.scl_speed_hz = 100000;

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_config, &s_backlight_i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add backlight I2C device (0x%02X): %s",
                 BACKLIGHT_I2C_ADDR, esp_err_to_name(ret));
        return ret;
    }

    // Apply saved backlight level (overrides the HX8394 driver's 0xFF default)
    lcd_set_backlight_level(current_backlight_level);
    ESP_LOGI(TAG, "I2C backlight controller initialized (addr=0x%02X, level=%u)",
             BACKLIGHT_I2C_ADDR, current_backlight_level);
    return ESP_OK;
}

void lcd_set_backlight_level(uint8_t level)
{
    if (level > 0) {
        saved_backlight_level = level;
    }
    current_backlight_level = level;

    if (s_backlight_i2c_dev == nullptr) {
        // I2C not yet initialized (called before lcd_backlight_i2c_init)
        return;
    }

    // Apply gamma correction for perceptual linearity.
    // The I2C brightness IC has a linear response, so we use gamma (not inverse)
    // to spread the perceptible dimming range across the full slider travel.
    uint8_t hw_value;
    if (level == 0) {
        hw_value = 0;
    } else {
        float normalized = (level - 1) / 254.0f;
        float gamma_corrected = powf(normalized, GAMMA_CORRECTION);

        // Map to 1-255 range (0 = off, 1 = minimum visible, 255 = full)
        hw_value = 1 + static_cast<uint8_t>(gamma_corrected * 254.0f);
    }

    uint8_t buf[2] = {BACKLIGHT_I2C_REG, hw_value};
    esp_err_t ret = i2c_master_transmit(s_backlight_i2c_dev, buf, sizeof(buf), 100);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Backlight I2C write failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGV(TAG, "level=%u -> i2c_hw=%u", level, hw_value);
}

#endif // CONFIG_IDF_TARGET

uint8_t lcd_get_backlight_level(void)
{
    return current_backlight_level;
}

void lcd_set_backlight_enabled(bool enabled)
{
    if (enabled) {
        uint8_t restore_level = saved_backlight_level;
        if (restore_level == 0) {
            restore_level = 1;
        }
        lcd_set_backlight_level(restore_level);
        return;
    }

    if (current_backlight_level > 0) {
        saved_backlight_level = current_backlight_level;
    }
    lcd_set_backlight_level(0);
}

bool lcd_is_backlight_enabled(void)
{
    return current_backlight_level > 0;
}

void lcd_force_display_refresh(void)
{
    lv_obj_invalidate(lv_screen_active());
    ESP_LOGD(TAG, "Display refresh forced after NVS operation");
}
