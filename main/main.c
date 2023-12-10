#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_gt911.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ui/ui.h"
#include "lvgl.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define TOUCH_I2C_HOST  0
#define TOUCH_SCL_PIN 20
#define TOUCH_SCL_PULLUP false
#define TOUCH_SDA_PIN 19
#define TOUCH_SDA_PULLUP false
#define TOUCH_RST_PIN 38
#define TOUCH_RST_ON_LEVEL 0
#define TOUCH_INT_PIN -1
#define TOUCH_INT_ON_LEVEL 0
#define TOUCH_FREQUENCY 400000

// The pixel number in horizontal and vertical
#define H_RES                      800
#define V_RES                      480

#define AVOID_TEAR_EFFECT_WITH_SEM   0
#define DOUBLE_FB                    0
#define USE_BOUNCE_BUFFER            0

#define BK_LIGHT_PIN_NUM             2
#define BK_LIGHT_ON_LEVEL            1
#define BK_LIGHT_OFF_LEVEL          !LCD_BK_LIGHT_ON_LEVEL

#define PIXEL_CLOCK_HZ              (18 * 1000 * 1000)

#define HSYNC_PIN_NUM               39
#define HSYNC_IDLE_LOW               0
#define HSYNC_FRONT_PORCH           48
#define HSYNC_BACK_PORCH             8
#define HSYNC_PULSE_WIDTH            4

#define VSYNC_PIN_NUM               41
#define VSYNC_IDLE_LOW               0
#define VSYNC_FRONT_PORCH           12
#define VSYNC_BACK_PORCH             8
#define VSYNC_PULSE_WIDTH            4

#define DISP_PIN_NUM                -1
#define DISP_ACTIIVE_LOW             0

#define DE_PIN_NUM                  40
#define DE_IDLE_HIGH                 0

#define PCLK_PIN_NUM                42
#define PCLK_ACTIVE_NEG              1
#define PCLK_IDLE_HIGH               0

#define DATA00_PIN_NUM               8 // B0
#define DATA01_PIN_NUM               3 // B1
#define DATA02_PIN_NUM              46 // B2
#define DATA03_PIN_NUM               9 // B3
#define DATA04_PIN_NUM               1 // B4
#define DATA05_PIN_NUM               5 // G0
#define DATA06_PIN_NUM               6 // G1
#define DATA07_PIN_NUM               7 // G2
#define DATA08_PIN_NUM              15 // G3
#define DATA09_PIN_NUM              16 // G4
#define DATA10_PIN_NUM               4 // G5
#define DATA11_PIN_NUM              45 // R0
#define DATA12_PIN_NUM              48 // R1
#define DATA13_PIN_NUM              47 // R2
#define DATA14_PIN_NUM              21 // R3
#define DATA15_PIN_NUM              14 // R4

/* END OF USER CHANGEABLE OPTIONS */


#if DOUBLE_FB
    #define NUM_FB                       2
#else
    #define NUM_FB                       1
#endif // DOUBLE_FB

#define LVGL_TICK_PERIOD_MS          2
#define LVGL_TASK_MAX_DELAY_MS     500
#define LVGL_TASK_MIN_DELAY_MS       1
#define LVGL_TASK_STACK_SIZE       (4 * 1024)
#define LVGL_TASK_PRIORITY           2

static SemaphoreHandle_t lvgl_mux = NULL;

// we use two semaphores to sync the VSYNC event and the LVGL task, to avoid potential tearing effect
#if AVOID_TEAR_EFFECT_WITH_SEM
    SemaphoreHandle_t sem_vsync_end;
    SemaphoreHandle_t sem_gui_ready;
#endif

#define TAG "SUNTON_RGB"

static bool on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;

    #if AVOID_TEAR_EFFECT_WITH_SEM
        if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
            xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
        }
    #endif

    return high_task_awoken == pdTRUE;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    #if AVOID_TEAR_EFFECT_WITH_SEM
        xSemaphoreGive(sem_gui_ready);
        xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
    #endif

    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void increase_lvgl_tick(void *arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
    // lv_tick_get();
}

bool lvgl_lock(int timeout_ms)
{
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            lvgl_unlock();
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}


lv_indev_data_t indev_data = {
    .point = {
        .x = -1,
        .y = -1
    },
    .state = LV_INDEV_STATE_RELEASED,
    .continue_reading = false
};

void indev_cb(lv_indev_drv_t * indev_drv, lv_indev_data_t*data)
{
    esp_lcd_touch_handle_t *tp = (esp_lcd_touch_handle_t *)indev_drv->user_data;
    esp_lcd_touch_read_data(*tp);

    uint16_t x[1];
    uint16_t y[1];
    uint16_t strength[1];
    uint8_t point_num = 0;

    //if (esp_lcd_touch_get_coordinates(*tp, &x, &y, &strength, &point_num, 1)) {
    if (esp_lcd_touch_get_coordinates(*tp, x, y, strength, &point_num, 1)) {
        data->point.x = x[0];
        data->point.y = y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        
        data->point.x = indev_data.point.x;
        data->point.y = indev_data.point.y;
        data->state = LV_INDEV_STATE_RELEASED;
    }

    indev_data.point.x = data->point.x;
    indev_data.point.y = data->point.y;
    indev_data.state = data->state;

}

// void indev_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
// {
//     uint16_t x;
//     uint16_t y;
//     uint16_t strength;
//     uint8_t point_num;
// 
//     esp_lcd_touch_handle_t *tp = (esp_lcd_touch_handle_t *)indev_drv->user_data;
// 
//     if (esp_lcd_touch_get_coordinates(*tp, &x, &y, &strength, &point_num, 1)) {
//         data->point.x = (lv_coord_t)x;
//         data->point.y = (lv_coord_t)y;
//         data->state = LV_INDEV_STATE_PRESSED;
//     } else {
//         data->point.x = indev_data.point.x;
//         data->point.y = indev_data.point.y;
//         data->state = LV_INDEV_STATE_RELEASED;
//     }
// 
//     indev_data.point.x = data->point.x;
//     indev_data.point.y = data->point.y;
//     indev_data.state = data->state;
// }



void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    #if AVOID_TEAR_EFFECT_WITH_SEM
        ESP_LOGI(TAG, "Create semaphores");
        sem_vsync_end = xSemaphoreCreateBinary();
        assert(sem_vsync_end);
        sem_gui_ready = xSemaphoreCreateBinary();
        assert(sem_gui_ready);
    #endif

    #if BK_LIGHT_PIN_NUM >= 0
        ESP_LOGI(TAG, "Turn off LCD backlight");

        gpio_config_t bk_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << BK_LIGHT_PIN_NUM
        };
        ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    #endif

    ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
        .sram_trans_align = 0,
        .psram_trans_align = 64,
        .num_fbs = NUM_FB,

        #if USE_BOUNCE_BUFFER
            .bounce_buffer_size_px = 10 * H_RES,
        #endif

        .clk_src = LCD_CLK_SRC_PLL160M,
        .disp_gpio_num = DISP_PIN_NUM,
        .pclk_gpio_num = PCLK_PIN_NUM,
        .vsync_gpio_num = VSYNC_PIN_NUM,
        .hsync_gpio_num = HSYNC_PIN_NUM,
        .de_gpio_num = DE_PIN_NUM,
        .data_gpio_nums = {
            DATA00_PIN_NUM,
            DATA01_PIN_NUM,
            DATA02_PIN_NUM,
            DATA03_PIN_NUM,
            DATA04_PIN_NUM,
            DATA05_PIN_NUM,
            DATA06_PIN_NUM,
            DATA07_PIN_NUM,
            DATA08_PIN_NUM,
            DATA09_PIN_NUM,
            DATA10_PIN_NUM,
            DATA11_PIN_NUM,
            DATA12_PIN_NUM,
            DATA13_PIN_NUM,
            DATA14_PIN_NUM,
            DATA15_PIN_NUM,
        },
        .timings = {
            .pclk_hz = PIXEL_CLOCK_HZ,
            .h_res = H_RES,
            .v_res = V_RES,
            // The following parameters should refer to LCD spec
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
            .fb_in_psram = 1, // allocate frame buffer in PSRAM
            .disp_active_low = DISP_ACTIIVE_LOW,
            .refresh_on_demand = 0,
            .no_fb = 0,
            .bb_invalidate_cache = 0
        }
    };

    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));

    ESP_LOGI(TAG, "Initialize RGB LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    #if BK_LIGHT_PIN_NUM >= 0
        ESP_LOGI(TAG, "Turn on LCD backlight");
        gpio_set_level(BK_LIGHT_PIN_NUM, BK_LIGHT_ON_LEVEL);
    #endif

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    void *buf1 = NULL;
    void *buf2 = NULL;

    #if DOUBLE_FB
        ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
        ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
        // initialize LVGL draw buffers
        lv_disp_draw_buf_init(&disp_buf, buf1, buf2, H_RES * V_RES);
    #else
        ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
        buf1 = heap_caps_malloc(H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
        // assert(buf1);
        // initialize LVGL draw buffers
        lv_disp_draw_buf_init(&disp_buf, buf1, buf2, H_RES * 100);
    #endif // DOUBLE_FB

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = H_RES;
    disp_drv.ver_res = V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;

    #if DOUBLE_FB
        disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
    #endif

    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Setup touch driver");

    esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    esp_lcd_panel_io_handle_t io_handle;

    esp_lcd_i2c_bus_handle_t bus_handle = (esp_lcd_i2c_bus_handle_t)TOUCH_I2C_HOST;

    i2c_config_t bus_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TOUCH_SDA_PIN,
        .scl_io_num = TOUCH_SCL_PIN,
        .sda_pullup_en = TOUCH_SDA_PULLUP,
        .scl_pullup_en = TOUCH_SCL_PULLUP,
        .master.clk_speed = TOUCH_FREQUENCY,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
    };

    ESP_ERROR_CHECK(i2c_param_config(TOUCH_I2C_HOST, &bus_config));
    ESP_ERROR_CHECK(i2c_driver_install(TOUCH_I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(bus_handle, &io_config, &io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = H_RES,
        .y_max = V_RES,
        .rst_gpio_num = TOUCH_RST_PIN,
        .int_gpio_num = TOUCH_INT_PIN,
        .levels = {
            .reset = TOUCH_RST_ON_LEVEL,
            .interrupt = TOUCH_INT_ON_LEVEL,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    static esp_lcd_touch_handle_t tp;
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(io_handle, &tp_cfg, &tp));

    ESP_LOGI(TAG, "Register touch driver with LVGL");

    // manually allocate room for this when we init
    lv_indev_drv_t *indev_drv = malloc(sizeof(*indev_drv));
    lv_indev_drv_init(indev_drv);
    indev_drv->type = LV_INDEV_TYPE_POINTER;
    indev_drv->read_cb = indev_cb;
    indev_drv->disp = disp;
    indev_drv->user_data = &tp;

    lv_indev_t * indev = lv_indev_drv_register(indev_drv);

    if (indev == NULL) {
        ESP_ERROR_CHECK(1);
    }

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    // assert(lvgl_mux);
    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_lock(-1)) {
        ui_init();
        // Release the mutex
        lvgl_unlock();
    }
}