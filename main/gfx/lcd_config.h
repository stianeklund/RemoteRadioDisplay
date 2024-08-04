#ifndef LCD_CONFIG_H
#define LCD_CONFIG_H

// Whether or not to apply rotation in `lcd_int` and `lvgl_init`
#define ROTATE_180 false 
#define TOUCH_I2C_HOST 0
#define TOUCH_SCL_PIN 20
#define TOUCH_SCL_PULLUP false
#define TOUCH_SDA_PIN 19
#define TOUCH_SDA_PULLUP false
#define TOUCH_RST_PIN 38
#define TOUCH_RST_ON_LEVEL 0
#define TOUCH_INT_PIN -1
#define TOUCH_INT_ON_LEVEL 0
#define TOUCH_FREQUENCY 400000
#define H_RES 800
#define V_RES 480
#define PIXEL_CLOCK_HZ (18 * 1000 * 1000)
#define BK_LIGHT_PIN_NUM 2
#define BK_LIGHT_ON_LEVEL 1
#define BK_LIGHT_OFF_LEVEL !BK_LIGHT_ON_LEVEL
#define HSYNC_PIN_NUM 39
#define HSYNC_IDLE_LOW 0
#define HSYNC_FRONT_PORCH 48
#define HSYNC_BACK_PORCH 8
#define HSYNC_PULSE_WIDTH 4
#define VSYNC_PIN_NUM 41
#define VSYNC_IDLE_LOW 0
#define VSYNC_FRONT_PORCH 12
#define VSYNC_BACK_PORCH 8
#define VSYNC_PULSE_WIDTH 4
#define DISP_PIN_NUM -1
#define DISP_ACTIVE_LOW 0
#define DE_PIN_NUM 40
#define DE_IDLE_HIGH 0
#define PCLK_PIN_NUM 42
#define PCLK_ACTIVE_NEG 1
#define PCLK_IDLE_HIGH 0
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
#define LVGL_TICK_PERIOD_MS 2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE (4 * 1024)
#define LVGL_TASK_PRIORITY 2

#endif // LCD_CONFIG_H