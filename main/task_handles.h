#ifndef TASK_HANDLES_H
#define TASK_HANDLES_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // For SemaphoreHandle_t

// LVGL task/mutex is now managed by esp_lvgl_port
// Use lvgl_port_lock()/lvgl_port_unlock() from esp_lvgl_port.h for thread-safe access

// From uart.c
TaskHandle_t get_cat_parser_task_handle(void);
TaskHandle_t get_read_uart_task_handle(void);
TaskHandle_t get_uart_tx_task_handle(void);

// From gps_client.c
TaskHandle_t get_gps_task_handle(void);

#endif // TASK_HANDLES_H
