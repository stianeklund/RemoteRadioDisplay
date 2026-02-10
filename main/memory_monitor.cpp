#include "memory_monitor.h"
#include <string.h>
#include "esp_timer.h"

static const char* TAG = "MEM_MONITOR";

#define CRITICAL_INTERNAL_HEAP_THRESHOLD 10000  // 10KB threshold for ESP32-S3 (more realistic)
#define CRITICAL_SPIRAM_HEAP_THRESHOLD   500000 // 500KB threshold for SPIRAM

void log_heap_status(const char* context) {
    size_t internal_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    
    size_t spiram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t spiram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t spiram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    
    ESP_LOGI(TAG, "[%s] Internal SRAM: %u/%u bytes free (largest: %u), %.1f%% used", 
             context,
             (unsigned)internal_free, 
             (unsigned)internal_total,
             (unsigned)internal_largest,
             (float)(internal_total - internal_free) * 100.0f / internal_total);
             
    ESP_LOGI(TAG, "[%s] SPIRAM: %u/%u bytes free (largest: %u), %.1f%% used", 
             context,
             (unsigned)spiram_free, 
             (unsigned)spiram_total,
             (unsigned)spiram_largest,
             (float)(spiram_total - spiram_free) * 100.0f / spiram_total);
             
    // Warn if we're running low on memory
    if (internal_free < CRITICAL_INTERNAL_HEAP_THRESHOLD) {
        ESP_LOGW(TAG, "WARNING: Internal SRAM low! Only %u bytes free", (unsigned)internal_free);
    }
    if (spiram_free < CRITICAL_SPIRAM_HEAP_THRESHOLD) {
        ESP_LOGW(TAG, "WARNING: SPIRAM low! Only %u bytes free", (unsigned)spiram_free);
    }
}

void log_task_stack_usage(const char* task_name, TaskHandle_t task_handle) {
    if (task_handle == NULL) {
        ESP_LOGW(TAG, "Cannot check stack usage for %s: invalid handle", task_name);
        return;
    }
    
    UBaseType_t stack_high_water_mark = uxTaskGetStackHighWaterMark(task_handle);
    ESP_LOGI(TAG, "Task '%s' stack high water mark: %u bytes unused", 
             task_name, 
             (unsigned)(stack_high_water_mark * sizeof(StackType_t)));
}

bool check_heap_critical(void) {
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t spiram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    
    bool critical = (internal_free < CRITICAL_INTERNAL_HEAP_THRESHOLD) || 
                    (spiram_free < CRITICAL_SPIRAM_HEAP_THRESHOLD);
                    
    if (critical) {
        ESP_LOGE(TAG, "CRITICAL: Memory levels critical! Internal: %u, SPIRAM: %u", 
                 (unsigned)internal_free, (unsigned)spiram_free);
    }
    
    return critical;
}

void init_memory_monitoring(void) {
    ESP_LOGI(TAG, "Memory monitoring initialized");
    log_heap_status("Init");
    hardware_health_init();
}

// Hardware health monitoring implementation
static hardware_health_stats_t g_health_stats = {0};

void hardware_health_init(void) {
    memset(&g_health_stats, 0, sizeof(g_health_stats));
    g_health_stats.last_health_check_time = esp_timer_get_time() / 1000; // Convert to ms
    ESP_LOGI(TAG, "Hardware health monitoring initialized");
}

void hardware_health_check(void) {
    uint32_t current_time = esp_timer_get_time() / 1000; // Convert to ms
    uint32_t time_since_last_check = current_time - g_health_stats.last_health_check_time;
    
    // Check for hardware stress indicators
    bool hardware_stress = false;
    
    // Check error rates (errors per minute)
    if (time_since_last_check > 60000) { // Check every minute
        float uart_error_rate = (float)g_health_stats.uart_errors * 60000.0f / time_since_last_check;
        float i2c_error_rate = (float)g_health_stats.i2c_errors * 60000.0f / time_since_last_check;
        float lvgl_timeout_rate = (float)g_health_stats.lvgl_mutex_timeouts * 60000.0f / time_since_last_check;
        float ws_timeout_rate = (float)g_health_stats.websocket_timeouts * 60000.0f / time_since_last_check;
        
        if (uart_error_rate > 10.0f) { // More than 10 UART errors per minute
            ESP_LOGW(TAG, "High UART error rate detected: %.1f/min", uart_error_rate);
            hardware_stress = true;
        }
        
        if (i2c_error_rate > 5.0f) { // More than 5 I2C errors per minute
            ESP_LOGW(TAG, "High I2C error rate detected: %.1f/min", i2c_error_rate);
            hardware_stress = true;
        }
        
        if (lvgl_timeout_rate > 20.0f) { // More than 20 LVGL timeouts per minute
            ESP_LOGW(TAG, "High LVGL mutex timeout rate detected: %.1f/min", lvgl_timeout_rate);
            hardware_stress = true;
        }
        
        if (ws_timeout_rate > 3.0f) { // More than 3 WebSocket timeouts per minute
            ESP_LOGW(TAG, "High WebSocket timeout rate detected: %.1f/min - this indicates network connectivity issues", ws_timeout_rate);
            // WebSocket timeouts are network issues, not hardware stress, but worth tracking
        }
        
        // Report overall hardware health status
        ESP_LOGI(TAG, "Hardware Health Report - UART errors: %lu, I2C errors: %lu, LVGL timeouts: %lu, Heap corruptions: %lu, WebSocket timeouts: %lu",
                 g_health_stats.uart_errors, g_health_stats.i2c_errors, 
                 g_health_stats.lvgl_mutex_timeouts, g_health_stats.heap_corruption_events, g_health_stats.websocket_timeouts);
        
        if (hardware_stress) {
            ESP_LOGW(TAG, "Hardware stress detected! Consider checking power supply and electrical connections");
        }
        
        g_health_stats.last_health_check_time = current_time;
    }
}

void hardware_health_report_uart_error(void) {
    g_health_stats.uart_errors++;
    ESP_LOGD(TAG, "UART error reported (total: %lu)", g_health_stats.uart_errors);
}

void hardware_health_report_i2c_error(void) {
    g_health_stats.i2c_errors++;
    ESP_LOGD(TAG, "I2C error reported (total: %lu)", g_health_stats.i2c_errors);
}

void hardware_health_report_lvgl_timeout(void) {
    g_health_stats.lvgl_mutex_timeouts++;
    ESP_LOGD(TAG, "LVGL mutex timeout reported (total: %lu)", g_health_stats.lvgl_mutex_timeouts);
}

void hardware_health_report_heap_corruption(void) {
    g_health_stats.heap_corruption_events++;
    ESP_LOGE(TAG, "Heap corruption event reported (total: %lu)", g_health_stats.heap_corruption_events);
}

void hardware_health_report_ws_timeout(void) {
    g_health_stats.websocket_timeouts++;
    ESP_LOGD(TAG, "WebSocket timeout reported (total: %lu)", g_health_stats.websocket_timeouts);
}

hardware_health_stats_t* hardware_health_get_stats(void) {
    return &g_health_stats;
}