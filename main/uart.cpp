#include "uart.h"
#include "sdkconfig.h"  // For CONFIG_CAT_UART_* defines
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cat_parser.h"
#include "gps_client.h"
#include "lvgl.h"
#include "task_handles.h" // For task handle getter declarations
#include "memory_monitor.h" // For hardware health monitoring
#include "esp_timer.h" // For timestamp monitoring

#define BUF_SIZE (4096)  // Increased from 1024 to prevent interrupt watchdog timeouts
static const char *TAG = "UART";

// Resolve UART port from Kconfig choice
#if CONFIG_CAT_UART_PORT_UART0
static const uart_port_t s_uart_port = UART_NUM_0;
#elif CONFIG_CAT_UART_PORT_UART1
static const uart_port_t s_uart_port = UART_NUM_1;
#elif CONFIG_CAT_UART_PORT_UART2
static const uart_port_t s_uart_port = UART_NUM_2;
#else
static const uart_port_t s_uart_port = UART_NUM_1; // Fallback
#endif

// Pins and baud come from Kconfig; provide safe defaults if not set yet
#ifndef CONFIG_CAT_UART_TX_PIN
#define CONFIG_CAT_UART_TX_PIN 17
#endif
#ifndef CONFIG_CAT_UART_RX_PIN
#define CONFIG_CAT_UART_RX_PIN 18
#endif
#ifndef CONFIG_CAT_UART_BAUD
#define CONFIG_CAT_UART_BAUD 57600
#endif

#define CAT_UART_TX_PIN CONFIG_CAT_UART_TX_PIN
#define CAT_UART_RX_PIN CONFIG_CAT_UART_RX_PIN

#define DEBUG false  // Disabled for production - eliminates logging overhead in UART hot path

#define UART_TX_TASK_STACK_SIZE 4096
#define UART_TX_TASK_PRIORITY 5
#define UART_TX_QUEUE_SIZE 20

static QueueHandle_t uart_tx_queue = NULL;
static TaskHandle_t uart_tx_task_handle = NULL; // Handle for the UART TX task

#define UART_TX_MESSAGE_BUFFER_SIZE 64 // Max message length including \r\n and \0

typedef struct {
    char data[UART_TX_MESSAGE_BUFFER_SIZE];
    size_t len;
} uart_tx_item_t;

esp_err_t init_uart() {
    ESP_LOGI(TAG, "Initializing UART: port=%d tx=%d rx=%d baud=%d", s_uart_port, CAT_UART_TX_PIN, CAT_UART_RX_PIN, CONFIG_CAT_UART_BAUD);
    const uart_config_t uart_config = {
        .baud_rate = CONFIG_CAT_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {0},
    };
    // Use large buffers (8KB RX, 8KB TX) to handle high-frequency CAT polling
    // Previous 2KB buffers caused interrupt watchdog timeouts during burst traffic
    esp_err_t ret = uart_driver_install(s_uart_port, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install error");
        return ret;
    }
    ret = uart_param_config(s_uart_port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config error");
        return ret;
    }

    ret = uart_set_pin(s_uart_port, CAT_UART_TX_PIN, CAT_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin error");
        return ret;
    }

    uart_tx_queue = xQueueCreate(UART_TX_QUEUE_SIZE, sizeof(uart_tx_item_t));
    if (uart_tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create UART TX queue");
        return ESP_FAIL;
    }

    BaseType_t task_created = xTaskCreatePinnedToCore(uart_tx_task, "uart_tx_task", UART_TX_TASK_STACK_SIZE, NULL, UART_TX_TASK_PRIORITY, &uart_tx_task_handle, 0);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART TX task");
        // Clean up queue if task creation failed
        if (uart_tx_queue != NULL) {
            vQueueDelete(uart_tx_queue);
            uart_tx_queue = NULL;
        }
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UART initialized successfully");
    return ESP_OK;
}

// the idea of this is to use this to send uart stuff from the display
void uart_tx_task(void *pvParameters) {
    // Add this task to watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    
    uart_tx_item_t tx_item;
    uint32_t consecutive_empty_cycles = 0;
    uint32_t watchdog_feed_count = 0;

    auto feed_watchdog = [&watchdog_feed_count]() {
        esp_task_wdt_reset();
        watchdog_feed_count++;
    };
    
    while (1) {
        // Feed watchdog more frequently to prevent timeout
        feed_watchdog();
        
        // Log watchdog activity every 1000 feeds for monitoring
        if ((watchdog_feed_count % 1000) == 0) {
            ESP_LOGD(TAG, "UART TX watchdog feeds: %lu", watchdog_feed_count);
        }
        
        // **DEADLOCK PREVENTION**: Use shorter timeouts and monitor queue activity
        if (xQueueReceive(uart_tx_queue, &tx_item, pdMS_TO_TICKS(100))) {  // Reduced to 100ms for better responsiveness
            consecutive_empty_cycles = 0; // Reset counter on successful receive
            
            // Feed watchdog before potentially blocking UART write
            feed_watchdog();
            
            uart_write_bytes(s_uart_port, tx_item.data, tx_item.len);
            // No free needed as data is part of tx_item structure
        } else {
            // No message received within 500ms
            consecutive_empty_cycles++;
            
            // Feed watchdog during idle periods to prevent timeout
            feed_watchdog();
            
            if (consecutive_empty_cycles >= 20) { // 10 seconds of no activity (20 * 500ms)
                ESP_LOGV(TAG, "UART TX task idle for %lu cycles (%.1fs) - potential deadlock condition", 
                         consecutive_empty_cycles, consecutive_empty_cycles * 0.5f);
                
                // Check if we're in a deadlock scenario and yield more aggressively
                if (consecutive_empty_cycles >= 60) { // 30 seconds of complete inactivity (60 * 500ms)
                    ESP_LOGV(TAG, "UART TX task idle for %.1fs - performing yield",
                             consecutive_empty_cycles * 0.5f);
                    
                    // Feed watchdog before delay
                    feed_watchdog();
                    
                    vTaskDelay(pdMS_TO_TICKS(1000)); // Yield for 1 second to break potential deadlock
                    consecutive_empty_cycles = 0; // Reset to prevent continuous delays
                    
                    // Feed watchdog after recovery
                    feed_watchdog();
                }
            }
        }
    }
}

esp_err_t uart_write_message(const char *message) {
    if (uart_tx_queue == NULL) {
        ESP_LOGE(TAG, "UART TX queue not initialized");
        return ESP_FAIL;
    }

    // Add a small delay to ensure system is fully initialized // Delay REMOVED for performance
    // vTaskDelay(pdMS_TO_TICKS(10)); 
    size_t original_len = strlen(message);

    // Check if the message + \r\n + \0 will fit in the buffer
    if (original_len + 2 >= UART_TX_MESSAGE_BUFFER_SIZE) { // +2 for \r\n, buffer must also hold \0
        ESP_LOGE(TAG, "UART message too long to fit in buffer: %s", message);
        return ESP_ERR_INVALID_ARG;
    }

    uart_tx_item_t tx_item;
    
    // Copy the original message
    strcpy(tx_item.data, message);
    // Append \r\n
    tx_item.data[original_len] = '\r';
    tx_item.data[original_len+1] = '\n';
    tx_item.data[original_len+2] = '\0'; // Null terminate for safety, though len is used for sending

    tx_item.len = original_len + 2; // The actual length of the string to send

    if (xQueueSend(uart_tx_queue, &tx_item, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to queue UART message: %s", message);
        // No free(data) needed here as it's stack-allocated within tx_item
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t uart_write_raw(const char *data, size_t len) {
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    int written = uart_write_bytes(s_uart_port, data, len);
    return (written >= 0) ? ESP_OK : ESP_FAIL;
}

void uart_write_message_handler(void *arg, void *data) {
    const char *message = (const char *)data;
    esp_err_t ret = uart_write_message(message);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write UART message: %s", message);
    }
}
#define READ_BUFFER_SIZE 256
#define COMMAND_BUFFER_SIZE 64
#define CAT_CMD_TIMEOUT_MS 50   // Optimized from 100ms to 50ms for better responsiveness

// Create a command processing queue to offload parsing from the UART task
static QueueHandle_t cat_cmd_queue = NULL;
static TaskHandle_t cat_parser_task_handle = NULL; // Handle for CAT parser task
static TaskHandle_t read_uart_task_current_handle = NULL; // Handle for the current read_uart task instance

// Task to process CAT commands asynchronously
void cat_parser_task(void *pvParameters) {
    // Add this task to watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    
    char cmd_buffer[COMMAND_BUFFER_SIZE];
    
    while (1) {
        // Feed watchdog before blocking operations
        esp_task_wdt_reset();
        
        // Removed heap integrity check for performance - was causing delays during high-frequency processing
        
        // Stack monitoring removed for performance
        // OPTIMIZED: Reduced timeout and batch processing for better responsiveness
        if (xQueueReceive(cat_cmd_queue, cmd_buffer, pdMS_TO_TICKS(25))) {  // Further reduced to 25ms for batch processing
            // Process the command with minimal overhead
            parse_cat_command(cmd_buffer);
            
            // OPTIMIZATION: Process up to 5 additional commands in batch if available
            // This reduces context switching during high-frequency CAT operations
            int batch_count = 0;
            while (batch_count < 5 && xQueueReceive(cat_cmd_queue, cmd_buffer, 0) == pdTRUE) {
                parse_cat_command(cmd_buffer);
                batch_count++;
            }
        }
        // If no command received within 100ms, loop continues and feeds watchdog
    }
}

void read_uart(void *pvParameters) {
    read_uart_task_current_handle = xTaskGetCurrentTaskHandle(); // Store current task handle
    
    // Add this task to watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    
    // Initialize health monitoring variables
    uint32_t consecutive_errors = 0;
    uint32_t last_successful_read = esp_timer_get_time() / 1000; // ms
    const uint64_t task_start_ms = static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
    uint32_t watchdog_feed_count = 0;
    uint32_t total_bytes_processed = 0;
    uint32_t last_watchdog_feed_count = 0;
    uint32_t last_bytes_processed = 0;
    TickType_t last_health_report_tick = xTaskGetTickCount();
    const TickType_t health_report_interval_ticks = pdMS_TO_TICKS(300000); // 5 minutes

    auto feed_watchdog = [&watchdog_feed_count]() {
        esp_task_wdt_reset();
        watchdog_feed_count++;
    };

    // Ensure CAT command queue exists
    if (cat_cmd_queue == NULL) {
        // Increased queue size from 10 to 50 to handle high-frequency CAT command bursts
        // This prevents dropping commands during rapid frequency updates and S-meter readings
        cat_cmd_queue = xQueueCreate(50, COMMAND_BUFFER_SIZE);
        if (cat_cmd_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create CAT command queue - Free heap: %lu bytes", esp_get_free_heap_size());
            esp_task_wdt_delete(NULL); // Remove from watchdog before exiting
            vTaskDelete(NULL); // Exit current read_uart task
            return;
        }
    }

    // Ensure CAT parser task is running
    // We check if the handle is NULL. If so, we attempt to create the task.
    // If cat_parser_task_handle is not NULL, we assume the task is operational.
    if (cat_parser_task_handle == NULL) { 
        ESP_LOGI(TAG, "Attempting to create CAT parser task - Free heap: %lu bytes", esp_get_free_heap_size());
        
        // Allocate CAT parser task stack in INTERNAL RAM for cache-off safety
        #define CAT_PARSER_STACK_SIZE 3072
        StackType_t *cat_parser_stack = static_cast<StackType_t*>(heap_caps_malloc(CAT_PARSER_STACK_SIZE * sizeof(StackType_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        if (cat_parser_stack == NULL) {
            ESP_LOGE(TAG, "Failed to allocate CAT parser task stack in internal SRAM");
            esp_task_wdt_delete(NULL);
            vTaskDelete(NULL);
            return;
        }
        
        static StaticTask_t cat_parser_task_buffer;  // Keep CAT parser TCB in INTERNAL RAM
        cat_parser_task_handle = xTaskCreateStaticPinnedToCore(
            cat_parser_task,
            "cat_parser", 
            CAT_PARSER_STACK_SIZE,
            NULL,
            4,
            cat_parser_stack,
            &cat_parser_task_buffer,
            0  // Pin to core 0
        );
        
        if (cat_parser_task_handle == NULL) {
            ESP_LOGE(TAG, "Failed to create CAT parser task - Free heap: %lu bytes", esp_get_free_heap_size());
            heap_caps_free(cat_parser_stack);
            esp_task_wdt_delete(NULL);
            vTaskDelete(NULL);
            return;
        }
        
        ESP_LOGI(TAG, "CAT parser task created with %dKB stack in INTERNAL RAM", CAT_PARSER_STACK_SIZE / 1024);
    }
    
    // Allocate data buffer on the stack instead of the heap
    uint8_t data[READ_BUFFER_SIZE];
    // No malloc, so no need to check if data is NULL due to allocation failure here.

    int retry_count = 0;
    const int max_retries = 5;
    char cmd_buffer[COMMAND_BUFFER_SIZE] = {0};
    int cmd_index = 0;

    while (1) {
        // Feed watchdog at start of read loop and track feeds
        feed_watchdog();
        
        // Log task health every 5 minutes (time-based, not counter-based)
        const TickType_t now_tick = xTaskGetTickCount();
        if ((now_tick - last_health_report_tick) >= health_report_interval_ticks) {
            const uint32_t interval_ms =
                static_cast<uint32_t>((now_tick - last_health_report_tick) * portTICK_PERIOD_MS);
            const uint64_t uptime_ms = static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
            const uint64_t task_uptime_ms = (uptime_ms > task_start_ms) ? (uptime_ms - task_start_ms) : 0ULL;
            const uint32_t feeds_delta = watchdog_feed_count - last_watchdog_feed_count;
            const uint32_t bytes_delta = total_bytes_processed - last_bytes_processed;
            const uint32_t feeds_per_sec = (interval_ms > 0) ? (feeds_delta * 1000U) / interval_ms : 0;
            const uint32_t bytes_per_sec = (interval_ms > 0) ? (bytes_delta * 1000U) / interval_ms : 0;

            ESP_LOGI(TAG,
                     "UART Read task health: feeds=%lu (+%lu, %lu/s), bytes=%lu (+%lu, %lu/s), "
                     "uptime=%llu min %llu s (since boot), task=%llu min %llu s",
                     (unsigned long)watchdog_feed_count, (unsigned long)feeds_delta, (unsigned long)feeds_per_sec,
                     (unsigned long)total_bytes_processed, (unsigned long)bytes_delta, (unsigned long)bytes_per_sec,
                     (unsigned long long)(uptime_ms / 60000ULL),
                     (unsigned long long)((uptime_ms / 1000ULL) % 60ULL),
                     (unsigned long long)(task_uptime_ms / 60000ULL),
                     (unsigned long long)((task_uptime_ms / 1000ULL) % 60ULL));
            last_health_report_tick = now_tick;
            last_watchdog_feed_count = watchdog_feed_count;
            last_bytes_processed = total_bytes_processed;
        }
        
        // Stack monitoring removed for performance
        // Use timeout and error handling to prevent hardware lockup
        
        // Check if UART hardware has pending errors before attempting read
        // CRITICAL: uart_get_buffered_data_len() can hang indefinitely in hardware lockup scenarios
        size_t uart_buffered_len = 0;
        
        // Add timestamp-based timeout detection for uart_get_buffered_data_len()
        uint32_t buffered_call_start = esp_timer_get_time() / 1000;
        esp_err_t buffered_result = uart_get_buffered_data_len(s_uart_port, &uart_buffered_len);
        uint32_t buffered_call_duration = (esp_timer_get_time() / 1000) - buffered_call_start;
        
        // Detect if uart_get_buffered_data_len took too long (indicating hardware lockup)
        if (buffered_call_duration > 100) { // 100ms is far too long for this call
            ESP_LOGE(TAG, "CRITICAL: uart_get_buffered_data_len() took %lums - hardware lockup detected", buffered_call_duration);
            consecutive_errors += 2; // Count slow calls as double errors
        }
        
        if (buffered_result != ESP_OK) {
            ESP_LOGE(TAG, "CRITICAL: uart_get_buffered_data_len() failed with %s - hardware lockup detected", esp_err_to_name(buffered_result));
            consecutive_errors++;
            
            // Immediate UART recovery on buffer length failure or timeout
            if (consecutive_errors >= 3) {
                ESP_LOGE(TAG, "Multiple uart_get_buffered_data_len() failures/timeouts - performing emergency UART recovery");
                
                // More aggressive recovery sequence
                uart_flush(s_uart_port); // Clear any pending data
                uart_driver_delete(s_uart_port);
                vTaskDelay(pdMS_TO_TICKS(1000)); // Longer delay for complete hardware recovery
                
                if (init_uart() != ESP_OK) {
                    ESP_LOGE(TAG, "Emergency UART recovery failed - system restart required");
                    esp_restart();
                }
                
                consecutive_errors = 0;
                retry_count = 0;
                last_successful_read = esp_timer_get_time() / 1000; // Reset timeout tracking
                ESP_LOGI(TAG, "Emergency UART recovery completed after buffer length failures");
                continue;
            }
        }
        
        if (uart_buffered_len == 0) {
            // No data available, add small delay to prevent tight loop and excessive watchdog feeds
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        
        // Attempt UART read with timeout and watchdog protection before potentially blocking call
        feed_watchdog();

        // Streamlined UART read with reduced overhead
        int len = uart_read_bytes(s_uart_port, data, READ_BUFFER_SIZE - 1, pdMS_TO_TICKS(CAT_CMD_TIMEOUT_MS));

        if (len > 0) {
            retry_count = 0;  // Reset retry count on successful read
            consecutive_errors = 0; // Reset consecutive error count
            last_successful_read = esp_timer_get_time() / 1000; // Update last successful read time
            total_bytes_processed += len; // Track total data processed
            data[len] = '\0'; // Ensure null termination
            
            if (DEBUG) {
                ESP_LOGD(TAG, "Received %d bytes: %.*s", len, len, data);
            }
            
            // Validate len to prevent infinite loops
            if (len > READ_BUFFER_SIZE - 1) {
                ESP_LOGE(TAG, "Invalid length received: %d, truncating to %d", len, READ_BUFFER_SIZE - 1);
                len = READ_BUFFER_SIZE - 1;
            }
            
            // OPTIMIZED: Batch process characters with minimal overhead
            for (int i = 0; i < len; i++) {
                char c = data[i];
                
                // Fast path for normal characters - just add to buffer
                if (c != ';' && c != '\r' && c != '\n' && cmd_index < COMMAND_BUFFER_SIZE - 1) {
                    cmd_buffer[cmd_index++] = c;
                    continue;  // Skip the expensive terminator handling
                }
                
                // Terminator found or buffer full - process command
                if (cmd_index > 0) {
                    cmd_buffer[cmd_index] = '\0';
                    
                    // OPTIMIZED: Simplified queue logic with fast path for common case
                    if (xQueueSend(cat_cmd_queue, cmd_buffer, 0) != pdPASS) {
                        // OPTIMIZED: Fast queue overflow handling - simple drop strategy
                        // Check only first 2 chars for priority (much faster than full strncmp)
                        if (cmd_buffer[0] == 'I' && cmd_buffer[1] == 'F') {
                            // IF commands are critical - try once to make room
                            char dropped_cmd[COMMAND_BUFFER_SIZE];
                            if (xQueueReceive(cat_cmd_queue, dropped_cmd, 0) == pdPASS) {
                                xQueueSend(cat_cmd_queue, cmd_buffer, 0);  // Try to queue IF command
                            }
                            // Don't log - too expensive during high-frequency operations
                        }
                        // For all other commands when queue full: just drop silently for performance
                        // This prevents expensive priority checking and multiple queue operations
                        }
                        
                    // OPTIMIZED: Fast buffer reset - only reset index, no expensive memset
                    cmd_index = 0;
                    // memset removed - cmd_buffer[cmd_index] = '\0' above handles termination
                }
                // Character addition is now handled in the fast path above
            }
        } else if (len == 0) {
            // No data, just yield to other tasks
            taskYIELD();
        } else {
            ESP_LOGE(TAG, "UART read error, len: %d", len);
            
            // Report UART error to hardware health monitoring
            hardware_health_report_uart_error();
            
            // Track consecutive errors and time since last successful read
            consecutive_errors++;
            uint32_t current_time = esp_timer_get_time() / 1000;
            uint32_t time_since_success = current_time - last_successful_read;
            
            // Detect critical UART health conditions before watchdog timeout  
            // Reduced thresholds to catch problems earlier
            if (consecutive_errors > 15 || time_since_success > 20000) { // 15 errors or 20s without success
                ESP_LOGE(TAG, "CRITICAL UART health detected! Consecutive errors: %lu, Time since success: %lums", 
                         consecutive_errors, time_since_success);
                
                // Emergency watchdog reset before attempting recovery
                feed_watchdog();
                
                // Force aggressive UART reset before watchdog timeout
                uart_driver_delete(s_uart_port);
                vTaskDelay(pdMS_TO_TICKS(200)); // Allow hardware to fully reset
                
                if (init_uart() != ESP_OK) {
                    ESP_LOGE(TAG, "Critical UART recovery failed - system restart required");
                    esp_task_wdt_delete(NULL);
                    esp_restart();
                    return;
                }
                
                consecutive_errors = 0;
                retry_count = 0;
                ESP_LOGI(TAG, "Critical UART recovery completed");
                continue; // Skip normal error handling
            }
            
            // Enhanced error detection for various UART failure modes
            if (len == ESP_ERR_TIMEOUT || len == ESP_FAIL || len == ESP_ERR_INVALID_STATE || len < 0) {
                ESP_LOGW(TAG, "UART hardware error detected (len=%d), performing aggressive recovery", 
                         len);
                
                // Aggressive UART hardware recovery to prevent ring buffer lockup
                uart_flush_input(s_uart_port);  // Clear input buffer
                uart_flush(s_uart_port);        // Clear both buffers
                
                // Check ring buffer status and clear if necessary
                size_t buffered_len = 0;
                esp_err_t buf_ret = uart_get_buffered_data_len(s_uart_port, &buffered_len);
                if (buf_ret == ESP_OK && buffered_len > 0) {
                    ESP_LOGW(TAG, "Clearing %zu bytes from UART ring buffer after error", buffered_len);
                    // Force clear the ring buffer by reading and discarding data
                    uint8_t discard_buf[256];
                    int discard_attempts = 0;
                    while (buffered_len > 0 && discard_attempts < 10) { // Prevent infinite loop
                        uart_read_bytes(s_uart_port, discard_buf, sizeof(discard_buf), 0);
                        uart_get_buffered_data_len(s_uart_port, &buffered_len);
                        discard_attempts++;
                    }
                }
                
                vTaskDelay(pdMS_TO_TICKS(100)); // Give hardware more time to recover
            }
            
            if (++retry_count >= max_retries) {
                ESP_LOGE(TAG, "Max retries reached. Reinitializing UART...");
                
                // More aggressive UART reset to recover from hardware issues
                uart_flush(s_uart_port);
                uart_driver_delete(s_uart_port);
                vTaskDelay(pdMS_TO_TICKS(100)); // Allow hardware to settle
                
                if (init_uart() != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to reinitialize UART. Task will restart system...");
                    esp_task_wdt_delete(NULL); // Remove from watchdog before restart
                    esp_restart(); // Safer than task recreation
                    return;
                }
                retry_count = 0;
                ESP_LOGI(TAG, "UART reinitialized successfully after hardware error");
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // Longer delay after error
        }
    }
    // The free(data) call previously here was unreachable due to the infinite while(1) loop.
    // Memory for 'data' is freed in the error handling path that leads to task recreation.
}

// Implementation of task handle getters
TaskHandle_t get_uart_tx_task_handle(void) {
    return uart_tx_task_handle;
}

TaskHandle_t get_cat_parser_task_handle(void) {
    return cat_parser_task_handle;
}

TaskHandle_t get_read_uart_task_handle(void) {
    return read_uart_task_current_handle;
}

int uart_get_port(void) {
    return s_uart_port;
}

bool uart_is_ready(void) {
    return uart_tx_queue != NULL;
}
