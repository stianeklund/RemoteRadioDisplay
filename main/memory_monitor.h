#pragma once

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t interrupt_wdt_timeouts;
    uint32_t task_wdt_timeouts;
    uint32_t uart_errors;
    uint32_t i2c_errors;
    uint32_t lvgl_mutex_timeouts;
    uint32_t heap_corruption_events;
    uint32_t websocket_timeouts; // Track WebSocket send timeouts that could cause display freezes
    uint32_t last_health_check_time;
} hardware_health_stats_t;

void log_heap_status(const char* context);

void log_task_stack_usage(const char* task_name, TaskHandle_t task_handle);

bool check_heap_critical(void);

void init_memory_monitoring(void);

// Hardware health monitoring functions
void hardware_health_init(void);
void hardware_health_check(void);
void hardware_health_report_uart_error(void);
void hardware_health_report_i2c_error(void);
void hardware_health_report_lvgl_timeout(void);
void hardware_health_report_heap_corruption(void);
void hardware_health_report_ws_timeout(void); // Report WebSocket send timeouts
hardware_health_stats_t* hardware_health_get_stats(void);

#ifdef __cplusplus
}
#endif