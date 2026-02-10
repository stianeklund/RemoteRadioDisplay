#ifndef LCD_CONFIG_H
#define LCD_CONFIG_H

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

// Whether or not to apply rotation in `lcd_init` and `lvgl_init`
#define SW_ROTATE_180 false
#define HW_ROTATE_180 false

// LVGL task timing (shared across targets)
#define LVGL_TICK_PERIOD_MS 2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE (8 * 1024)
#define LVGL_TASK_PRIORITY 6

#if CONFIG_IDF_TARGET_ESP32S3
// ============================================================
// ESP32-S3: Sunton ESP32-8048S050C (800x480 RGB parallel LCD)
// ============================================================

#define TOUCH_I2C_HOST I2C_NUM_0
#define TOUCH_SCL_PIN 20
#define TOUCH_SCL_PULLUP false
#define TOUCH_SDA_PIN 19
#define TOUCH_SDA_PULLUP false
#define TOUCH_RST_PIN GPIO_NUM_38
#define TOUCH_RST_ON_LEVEL 0
#define TOUCH_INT_PIN GPIO_NUM_NC
#define TOUCH_INT_ON_LEVEL 0
#define TOUCH_FREQUENCY 50000  // Reduced from 100kHz to 50kHz to prevent I2C ISR timeout

#define H_RES 800
#define V_RES 480
#define PIXEL_CLOCK_HZ (16 * 1000 * 1000)
#define LCD_RGB_BOUNCE_BUFFER_HEIGHT 0   // No bounce buffers
#define LCD_RGB_AVOID_TEARING false      // No VSYNC sync
#define LCD_RGB_DIRECT_MODE false        // Normal mode
#define BK_LIGHT_PIN_NUM 2
#define BK_LIGHT_ON_LEVEL 1
#define BK_LIGHT_OFF_LEVEL !BK_LIGHT_ON_LEVEL

// RGB timing pins
#define HSYNC_PIN_NUM 39
#define HSYNC_IDLE_LOW 0
#define HSYNC_FRONT_PORCH 8
#define HSYNC_BACK_PORCH 8
#define HSYNC_PULSE_WIDTH 4
#define VSYNC_PIN_NUM 41
#define VSYNC_IDLE_LOW 0
#define VSYNC_FRONT_PORCH 8
#define VSYNC_BACK_PORCH 8
#define VSYNC_PULSE_WIDTH 4
#define DISP_PIN_NUM -1
#define DISP_ACTIVE_LOW 0
#define DE_PIN_NUM 40
#define DE_IDLE_HIGH 1
#define PCLK_PIN_NUM 42
#define PCLK_ACTIVE_NEG 1
#define PCLK_IDLE_HIGH 1

// RGB data pins (16-bit RGB565)
#define DATA00_PIN_NUM 8
#define DATA01_PIN_NUM 3
#define DATA02_PIN_NUM 46
#define DATA03_PIN_NUM 9
#define DATA04_PIN_NUM 1
#define DATA05_PIN_NUM 5
#define DATA06_PIN_NUM 6
#define DATA07_PIN_NUM 7
#define DATA08_PIN_NUM 15
#define DATA09_PIN_NUM 16
#define DATA10_PIN_NUM 4
#define DATA11_PIN_NUM 45
#define DATA12_PIN_NUM 48
#define DATA13_PIN_NUM 47
#define DATA14_PIN_NUM 21
#define DATA15_PIN_NUM 14

#elif CONFIG_IDF_TARGET_ESP32P4
// ============================================================
// ESP32-P4: Guition JC-ESP32P4-M3 + 5" HX8394 DSI (720x1280)
// ============================================================

// Touch I2C (GT911, same as S3 but different pins)
#define TOUCH_I2C_HOST I2C_NUM_0
#define TOUCH_SCL_PIN 8
#define TOUCH_SCL_PULLUP false
#define TOUCH_SDA_PIN 7
#define TOUCH_SDA_PULLUP false
#define TOUCH_RST_PIN GPIO_NUM_23      // Waveshare BSP: BSP_LCD_TOUCH_RST
#define TOUCH_RST_ON_LEVEL 0
#define TOUCH_INT_PIN GPIO_NUM_NC
#define TOUCH_INT_ON_LEVEL 0
#define TOUCH_FREQUENCY 400000         // 400kHz - P4 I2C is more capable

// Native panel resolution: 720x1280 (portrait)
// Used in landscape: 1280x720 (rotation handled in LVGL)
#define DSI_LCD_H_RES 720
#define DSI_LCD_V_RES 1280

// Landscape resolution as seen by LVGL after rotation
#define H_RES 1280
#define V_RES 720

// MIPI DSI PHY power (LDO channel 3)
#define DSI_PHY_LDO_CHAN 3
#define DSI_PHY_LDO_VOLTAGE_MV 2500

// LCD reset: not needed on Guition board (handled via DSI commands)
// GPIO26 drives backlight boost converter (LCD_PWM); GPIO27 goes to RS485
#define LCD_RST_PIN GPIO_NUM_NC

#else
#error "Unsupported target: define display config for your board"
#endif

#endif // LCD_CONFIG_H
