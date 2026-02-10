#include "gps_client.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_handles.h" // For task handle getter declarations
#include <string.h>
#include <time.h>
#include <sys/time.h>

#define GPS_UART_NUM UART_NUM_2
#define GPS_TXD_PIN 17
#define GPS_RXD_PIN 16
#define GPS_BAUD_RATE 9600
#define BUF_SIZE 1024

static const char *TAG = "GPS_CLIENT";
static TaskHandle_t gps_task_handle = NULL;

TaskHandle_t get_gps_task_handle(void) {
    return gps_task_handle;
}

esp_err_t init_gps_client(void) {
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TXD_PIN, GPS_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    xTaskCreate(gps_task, "gps_task", 2048, NULL, 5, &gps_task_handle);  // Reduced from 4KB to 2KB
    
    ESP_LOGI(TAG, "GPS client initialized");
    return ESP_OK;
}

void deinit_gps_client(void) {
    if (gps_task_handle != NULL) {
        vTaskDelete(gps_task_handle);
        gps_task_handle = NULL;
    }
    uart_driver_delete(GPS_UART_NUM);
    ESP_LOGI(TAG, "GPS client deinitialized");
}

static int parse_rmc_sentence(const char* sentence, struct tm *timeinfo) {
    char time_str[7], date_str[7];
    if (sscanf(sentence, "$GPRMC,%6s,A,%*f,%*c,%*f,%*c,%*f,%*f,%6s", time_str, date_str) == 2) {
        int hour, min, sec, day, month, year;
        sscanf(time_str, "%2d%2d%2d", &hour, &min, &sec);
        sscanf(date_str, "%2d%2d%2d", &day, &month, &year);
        
        timeinfo->tm_hour = hour;
        timeinfo->tm_min = min;
        timeinfo->tm_sec = sec;
        timeinfo->tm_mday = day;
        timeinfo->tm_mon = month - 1;  // tm_mon is 0-based
        timeinfo->tm_year = year + 2000 - 1900;  // tm_year is years since 1900
        
        return 1;
    }
    return 0;
}

esp_err_t gps_get_time(struct tm *timeinfo) {
    // Use static buffer to avoid heap allocation in hot path
    // Only one task calls this function, so static is safe
    static char buffer[BUF_SIZE];

    int len = uart_read_bytes(GPS_UART_NUM, buffer, BUF_SIZE - 1, 1000 / portTICK_PERIOD_MS);
    if (len > 0) {
        buffer[len] = 0;  // Null-terminate the string
        char* line = strtok(buffer, "\n");
        while (line != NULL) {
            if (strncmp(line, "$GPRMC", 6) == 0) {
                if (parse_rmc_sentence(line, timeinfo)) {
                    return ESP_OK;
                }
            }
            line = strtok(NULL, "\n");
        }
    }

    return ESP_ERR_NOT_FOUND;
}

void gps_task(void *pvParameters) {
    struct tm timeinfo;
    time_t now;
    char strftime_buf[64];

    while (1) {
        // Stack monitoring removed for performance
        if (gps_get_time(&timeinfo) == ESP_OK) {
            // Convert to time_t
            time(&now);
            localtime_r(&now, &timeinfo);

            // Set system time
            struct timeval tv = { .tv_sec = now };
            settimeofday(&tv, NULL);

            // Log the time
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            ESP_LOGI(TAG, "GPS Time synchronized. Time is: %s", strftime_buf);
        } else {
            ESP_LOGW(TAG, "Failed to get GPS time");
        }

        vTaskDelay(pdMS_TO_TICKS(60000));  // Update every minute
    }
}
