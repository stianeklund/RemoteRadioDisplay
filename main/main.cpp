#include "cat_parser.h"
#include "cat_polling.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "gfx/lcd_init.h"
#include "gfx/lvgl_init.h"
#include "gfx/touch_init.h"
#include "memory_monitor.h"
#include "ntp_client.h"
#include "gps_client.h"
#include "nvs_flash.h"
#include "screensaver.h"
#include "settings_storage.h"
#include "uart.h"
#include "ui.h"
#include "wifi_init.h"
#include "antenna_control.h"
// #include "ntp_client.h" // Redundant include, already included above.
#include <stdio.h>
#include "lvgl.h" // For lv_timer_create
#include "esp_lvgl_port.h" // For lvgl_port_lock/unlock
#include "task_handles.h" // For getting task handles
#include "radio/radio_subject_updater.h" // For radio_subject_drain_updates()
#include "freertos/semphr.h" // For SemaphoreHandle_t and mutex functions
#include "sdkconfig.h"
#include "gfx/lcd_config.h"

#define USE_GPS 0  // Set to 0 to use NTP instead

static const char *TAG = "MAIN";
static char main_time_buffer_for_lvgl[10]; // Buffer for "HH:MM:SS\0" for UI updates


// LVGL Timer callback to update the time display periodically
static void lvgl_periodic_update_time_cb(lv_timer_t *timer) {
    (void) timer; // Unused parameter

    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    // Update UI only if time is valid (e.g., year is past 2000, indicating it's been set)
    // GPS/NTP might take a moment to set the time initially.
    if (ui_UtcTime && lv_obj_is_valid(ui_UtcTime)) {
        // ui_UtcTime is declared in ui_Screen1.c and externed via ui.h
        if (timeinfo.tm_year > (2000 - 1900)) {
            strftime(main_time_buffer_for_lvgl, sizeof(main_time_buffer_for_lvgl), "%H:%M:%S", &timeinfo);
            lv_label_set_text(ui_UtcTime, main_time_buffer_for_lvgl); // Changed to lv_label_set_text
            ESP_LOGV(TAG, "Time updated: %s (year=%d)", main_time_buffer_for_lvgl, timeinfo.tm_year + 1900);
        } else {
            // Set placeholder text if time is not yet valid
            // Using "18:20:30" as per original textarea placeholder
            lv_label_set_text(ui_UtcTime, "18:20:30"); // Changed to lv_label_set_text
            ESP_LOGV(TAG, "Time not valid yet (year=%d), showing placeholder", timeinfo.tm_year + 1900);
        }
    } else {
        ESP_LOGE(TAG, "ui_UtcTime is NULL or invalid, cannot update time display");
    }
}

// LVGL Timer callback to drain queued subject updates from other tasks
// This runs at 100Hz (10ms) for snappy VFO updates
static void lvgl_subject_drain_cb(lv_timer_t *timer) {
    (void) timer;
    int processed = radio_subject_drain_updates();
    // DIAG: Log when unusually large batches are processed (debug level, high threshold)
    if (processed >= 50) {
        ESP_LOGD("MAIN", "[DIAG] Subject drain: %d items", processed);
    }
}

void monitoring_task(void *pvParameters);

void time_sync_task(void *pvParameters);

void stack_monitoring_task(void *pvParameters);

extern "C" void app_main(void) {
    // Initialize memory monitoring first
    init_memory_monitoring();
    
    // Note: ESP-IDF v5.4.1 doesn't have heap_caps_set_prefer, but we configure
    // SPIRAM preferences via sdkconfig.defaults instead
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    log_heap_status("After NVS init");
    ESP_LOGI(TAG, "Config: LWIP_TCPIP_AFFINITY_CPU0=%d, LWIP_AFFINITY_MASK=0x%x",
             CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0,
             CONFIG_LWIP_TCPIP_TASK_AFFINITY);
#if CONFIG_IDF_TARGET_ESP32S3
    ESP_LOGI(TAG, "Config: LCD_RGB_ISR_IRAM_SAFE=%d, SPIRAM_XIP=%d",
             CONFIG_LCD_RGB_ISR_IRAM_SAFE,
             CONFIG_SPIRAM_XIP_FROM_PSRAM);
#endif
    if (CONFIG_LWIP_TCPIP_TASK_AFFINITY == CONFIG_FREERTOS_NO_AFFINITY) {
        ESP_LOGW(TAG, "LWIP TCPIP task has no affinity (may contend with LVGL)");
    } else if (CONFIG_LWIP_TCPIP_TASK_AFFINITY == 0) {
        ESP_LOGI(TAG, "LWIP TCPIP task pinned to CPU0");
    } else if (CONFIG_LWIP_TCPIP_TASK_AFFINITY == 1) {
        ESP_LOGI(TAG, "LWIP TCPIP task pinned to CPU1");
    } else {
        ESP_LOGW(TAG, "LWIP TCPIP task affinity value unexpected: 0x%x",
                 CONFIG_LWIP_TCPIP_TASK_AFFINITY);
    }
#ifdef CONFIG_LVGL_PORT_ENABLE_PPA
    ESP_LOGI(TAG, "Config: LVGL_PORT_ENABLE_PPA=%d", CONFIG_LVGL_PORT_ENABLE_PPA);
#endif
#if CONFIG_IDF_TARGET_ESP32S3
    ESP_LOGI(TAG, "LCD: pclk=%u, avoid_tearing=%d, direct_mode=%d, bb_height=%d",
             (unsigned)PIXEL_CLOCK_HZ,
             (int)LCD_RGB_AVOID_TEARING,
             (int)LCD_RGB_DIRECT_MODE,
             (int)LCD_RGB_BOUNCE_BUFFER_HEIGHT);
#elif CONFIG_IDF_TARGET_ESP32P4
    ESP_LOGI(TAG, "LCD: DSI %dx%d (landscape %dx%d)", DSI_LCD_H_RES, DSI_LCD_V_RES, H_RES, V_RES);
#endif

    // Reconfigure task watchdog (it's already initialized by ESP-IDF)
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 10000,  // 10 seconds
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = false
    };
    ret = esp_task_wdt_reconfigure(&twdt_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to reconfigure task watchdog: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Task watchdog reconfigured with 10s timeout");
    }
    
    // Note: We don't add long-sleeping tasks to watchdog since they sleep >10s

    // Initialize settings storage
    ret = settings_storage_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize settings storage: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Settings storage initialized");

    // Start WiFi connection (non-blocking) - connects in background while UI initializes
    ESP_LOGI(TAG, "Starting WiFi (non-blocking)");
    ret = wifi_init_start();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
    }

    // Initialize LCD and UI in parallel with WiFi connection for faster startup
    ESP_LOGI(TAG, "Initializing LCD and UI");
    esp_lcd_panel_handle_t panel_handle = NULL;
    ret = lcd_init(&panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init LCD: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "LCD initialized");

    // Initialize touch
    esp_lcd_touch_handle_t tp = NULL;
    ret = touch_init(&tp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize touch: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Touch initialized");

#if CONFIG_IDF_TARGET_ESP32P4
    // Initialize I2C backlight controller (shares touch I2C bus, must come after touch_init)
    ret = lcd_backlight_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize I2C backlight: %s", esp_err_to_name(ret));
        // Non-fatal: display stays at full brightness from HX8394 init
    }
#endif

    // Initialize LVGL
    ret = lvgl_init(panel_handle, tp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LVGL: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "LVGL initialized");
    log_heap_status("After LVGL init");

    // Initialize antenna control system BEFORE UI so antenna names are available
    // when ui_Screen2 creates antenna selection buttons
    ret = antenna_control_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize antenna control: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Antenna control initialized (WebSocket client deferred until WiFi connects)");
    }

    // Initialize UI (must be after antenna_control_init for antenna button labels)
    ui_init();
    ESP_LOGI(TAG, "UI initialized");
    log_heap_status("After UI init");

    // Create LVGL timer for periodic time updates on the UI
    // This timer will call lvgl_periodic_update_time_cb every 1000ms (1 second)
    // The last parameter (user_data) is NULL as it's not needed by the callback
    lv_timer_create(lvgl_periodic_update_time_cb, 1000, NULL);
    ESP_LOGI(TAG, "LVGL timer for time display created");

    // Create LVGL timer to drain subject updates from other tasks
    // Runs at 200Hz (5ms) for minimal latency on VFO updates from CAT parser, websocket, etc.
    lv_timer_create(lvgl_subject_drain_cb, 5, NULL);
    ESP_LOGI(TAG, "LVGL subject drain timer created (200Hz)");

    // Time client (NTP/GPS) initialization will now be handled by time_sync_task
    // to allow concurrent startup with UART.

    // Initialize CAT parser (sets up message pool before UART traffic starts)
    ret = cat_parser_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAT parser: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "CAT parser initialized");

    init_uart();
    
    // Initialize CAT polling manager
    ret = cat_polling_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize CAT polling manager: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "CAT polling manager initialized");
    }

    // Initialize screensaver (must be after LVGL and CAT polling init)
    ret = screensaver_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize screensaver: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Screensaver initialized");
    }

    // Create time_sync task with stack in PSRAM (runs once/minute, not timing-critical)
    #define TIME_SYNC_TASK_STACK_SIZE 3584  // In PSRAM - no internal SRAM cost, safe margin
    StackType_t *time_sync_stack = static_cast<StackType_t*>(
        heap_caps_malloc(TIME_SYNC_TASK_STACK_SIZE * sizeof(StackType_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (time_sync_stack == NULL) {
        ESP_LOGE(TAG, "Failed to allocate time_sync task stack in PSRAM");
    } else {
        static StaticTask_t time_sync_task_buffer;  // Keep TCB in INTERNAL RAM
        TaskHandle_t time_sync_handle = xTaskCreateStaticPinnedToCore(
            time_sync_task,
            "time_sync",
            TIME_SYNC_TASK_STACK_SIZE,
            NULL,
            5,
            time_sync_stack,
            &time_sync_task_buffer,
            0  // Pin to core 0
        );
        if (time_sync_handle == NULL) {
            ESP_LOGE(TAG, "Failed to create time_sync task");
            heap_caps_free(time_sync_stack);
        } else {
            ESP_LOGI(TAG, "time_sync task created with %dKB stack in PSRAM", TIME_SYNC_TASK_STACK_SIZE / 1024);
        }
    }
    
    // Create read_uart task with stack in internal SRAM (UART ISR timing-critical)
    // Stack monitoring shows only 428 bytes unused, indicating near-overflow condition
    #define READ_UART_TASK_STACK_SIZE 4096
    StackType_t *read_uart_stack = static_cast<StackType_t*>(heap_caps_malloc(READ_UART_TASK_STACK_SIZE * sizeof(StackType_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (read_uart_stack == NULL) {
        ESP_LOGE(TAG, "Failed to allocate read_uart task stack in internal SRAM");
    } else {
        static StaticTask_t read_uart_task_buffer;  // Keep TCB in INTERNAL RAM
        TaskHandle_t read_uart_handle = xTaskCreateStaticPinnedToCore(
            read_uart,
            "uart_read_task",
            READ_UART_TASK_STACK_SIZE,
            NULL,
            5,  // Reduced from 10 to 5 to prevent priority inversions with LVGL task
            read_uart_stack,
            &read_uart_task_buffer,
            0  // Pin to core 0
        );
        if (read_uart_handle == NULL) {
            ESP_LOGE(TAG, "Failed to create read_uart task");
            heap_caps_free(read_uart_stack);
        } else {
            ESP_LOGI(TAG, "read_uart task created with %dKB stack in internal SRAM", READ_UART_TASK_STACK_SIZE / 1024);
        }
    }
    
    // Create stack monitoring task with stack in PSRAM (runs every 120s, not timing-critical)
    #define STACK_MONITOR_TASK_STACK_SIZE 3584  // In PSRAM - no internal SRAM cost, safe margin
    StackType_t *stack_monitor_stack = static_cast<StackType_t*>(
        heap_caps_malloc(STACK_MONITOR_TASK_STACK_SIZE * sizeof(StackType_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (stack_monitor_stack == NULL) {
        ESP_LOGE(TAG, "Failed to allocate stack_monitor task stack in PSRAM");
    } else {
        static StaticTask_t stack_monitor_task_buffer;  // Keep TCB in INTERNAL RAM
        TaskHandle_t stack_monitor_handle = xTaskCreateStaticPinnedToCore(
            stack_monitoring_task,
            "stack_monitor",
            STACK_MONITOR_TASK_STACK_SIZE,
            NULL,
            1,
            stack_monitor_stack,
            &stack_monitor_task_buffer,
            1  // Pin to core 1
        );
        if (stack_monitor_handle == NULL) {
            ESP_LOGE(TAG, "Failed to create stack_monitor task");
            heap_caps_free(stack_monitor_stack);
        } else {
            ESP_LOGI(TAG, "stack_monitor task created with %dKB stack in PSRAM", STACK_MONITOR_TASK_STACK_SIZE / 1024);
        }
    }
    log_heap_status("After task creation");
    // cat_parser_init_activity_monitor(); // Removed: CAT activity is now checked by an LVGL timer in ui_Screen1.c

    // Main application loop
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void monitoring_task(void *pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "--- System Status ---");

        // Log heap usage
        ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
        ESP_LOGI(TAG, "Minimum free heap since boot: %lu bytes",
                 esp_get_minimum_free_heap_size());

        // Log uptime
        ESP_LOGI(TAG, "Uptime: %lld s", esp_timer_get_time() / 1000000);

        // Log current time
        time_t now;
        struct tm timeinfo;
        char strftime_buf[64];
        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        ESP_LOGI(TAG, "Current time: %s", strftime_buf);

        vTaskDelay(pdMS_TO_TICKS(3000)); // Run every 3 seconds
    }
}

void time_sync_task(void *pvParameters) {
    // Don't add to watchdog - this task sleeps for 60s which exceeds timeout
    
    struct tm timeinfo;
    char strftime_buf[64];

    // Initialize the time client (NTP or GPS) once at the beginning of this task.
    // This allows app_main to continue with other initializations (like UART) concurrently.
#if USE_GPS
    ESP_LOGI(TAG, "time_sync_task: Initializing GPS client");
    esp_err_t gps_result = init_gps_client(); // This might block this task, but not app_main
    ESP_LOGI(TAG, "GPS client init result: %s", esp_err_to_name(gps_result));
#else
    // NTP client is now started on IP acquisition in wifi_init.c (IP_EVENT_STA_GOT_IP)
    ESP_LOGI(TAG, "time_sync_task: NTP client handled by WiFi IP event");
#endif

    while (1) {
        // No watchdog reset needed - task not monitored due to long sleep period
        
        bool time_obtained = false; // Initialize time_obtained flag
#if USE_GPS
        // GPS time is obtained and system time is set directly by gps_get_time if successful
        if (gps_get_time(&timeinfo) == ESP_OK) {
            // Convert parsed GPS time to timeval and set system time
            struct timeval tv = {};
            tv.tv_sec = mktime(&timeinfo);
            tv.tv_usec = 0;
            settimeofday(&tv, nullptr);

            // Log the time
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            ESP_LOGI(TAG, "GPS Time synchronized. Time is: %s", strftime_buf);
            time_obtained = true;
        } else {
            ESP_LOGW(TAG, "Failed to get GPS time");
        }
#else // NTP
        if (ntp_get_time(&timeinfo) == ESP_OK) {
            // Time is already set by NTP (or system time is available)
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            ESP_LOGI(TAG, "NTP Mode. System time is: %s", strftime_buf);
            time_obtained = true;
        } else {
            ESP_LOGW(TAG, "NTP Mode. Failed to get system time (it may not be set yet).");
        }
#endif

        // The UI update is now handled by the lvgl_periodic_update_time_cb timer.
        // This time_sync_task remains responsible for:
        // 1. If USE_GPS=1: Getting time from GPS and setting the system time.
        // 2. If USE_GPS=0: Logging the NTP-set system time.
        // The actual setting of system time by NTP is handled by the ESP-IDF SNTP service and its callbacks.
        if (time_obtained) {
            // ESP_LOGI(TAG, "System time is available in time_sync_task. UI updates independently.");
        }

        vTaskDelay(pdMS_TO_TICKS(60000)); // This task runs every minute to check/log time
    }
}

void stack_monitoring_task(void *pvParameters) {
    (void) pvParameters;
    
    // Don't add to watchdog - this task sleeps for 30s which exceeds timeout
    
    // Note: LVGL task is managed internally by esp_lvgl_port - no handle access
    TaskHandle_t cat_parser_handle;
    TaskHandle_t read_uart_handle;
    TaskHandle_t uart_tx_handle;

    // Task health monitoring - track actual runtime progress per task
    struct TaskHealthMonitor {
        TaskHandle_t* handle_ptr;
        const char* name;
        uint32_t last_runtime;
        uint8_t stuck_count;
    };

    // Static to persist across function calls (task handles are set later)
    // Note: LVGL task is managed internally by esp_lvgl_port, no handle access
    static TaskHealthMonitor task_monitors[3] = {
        {nullptr, "CAT Parser", 0, 0},
        {nullptr, "Read UART", 0, 0},
        {nullptr, "UART TX", 0, 0}
    };
    static bool monitors_initialized = false;

    ESP_LOGI(TAG, "Stack monitoring task started, will log every 30 seconds");

    // Add 2 second offset to stagger with other periodic tasks (WebSocket polling every 30s, CAT polling every 10s)
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        // No watchdog reset needed - task not monitored due to long sleep period

        ESP_LOGI(TAG, "=== Memory Monitor Cycle Start (tick: %u) ===", (unsigned)xTaskGetTickCount());

        // Log comprehensive memory status
        log_heap_status("Periodic monitor");

        // Check for critical memory levels
        if (check_heap_critical()) {
            ESP_LOGE(TAG, "CRITICAL MEMORY LEVELS DETECTED!");
        }

        // **DEADLOCK DETECTION**: Track actual task runtime progress
        // This correctly detects stuck tasks by monitoring runtime counter changes
        // Note: LVGL task is managed internally by esp_lvgl_port, no handle access
        cat_parser_handle = get_cat_parser_task_handle();
        read_uart_handle = get_read_uart_task_handle();
        uart_tx_handle = get_uart_tx_task_handle();

        // Initialize monitor handle pointers on first run
        if (!monitors_initialized && cat_parser_handle != nullptr) {
            monitors_initialized = true;
        }

        TaskHandle_t handles[3] = {cat_parser_handle, read_uart_handle, uart_tx_handle};
        uint8_t critical_stuck_tasks = 0;

        for (int i = 0; i < 3; i++) {
            TaskHandle_t h = handles[i];
            if (h == nullptr) continue;

            TaskStatus_t status;
            vTaskGetInfo(h, &status, pdTRUE, eInvalid);

            // Check if task runtime has progressed (indicates actual work being done)
            if (status.ulRunTimeCounter == task_monitors[i].last_runtime &&
                task_monitors[i].last_runtime != 0) {
                task_monitors[i].stuck_count++;

                if (task_monitors[i].stuck_count >= 3) {
                    ESP_LOGW(TAG, "Task %s appears stuck (no runtime progress for %u cycles)",
                             task_monitors[i].name, task_monitors[i].stuck_count);
                    critical_stuck_tasks++;
                }
            } else {
                task_monitors[i].stuck_count = 0;
            }

            task_monitors[i].last_runtime = status.ulRunTimeCounter;
        }

        // Only restart if multiple critical tasks are stuck (indicates true system hang)
        if (critical_stuck_tasks >= 3) {
            ESP_LOGE(TAG, "CRITICAL: %u tasks stuck - system may be deadlocked", critical_stuck_tasks);
            ESP_LOGE(TAG, "Performing emergency system restart");
            esp_restart();
        }
        
        ESP_LOGI(TAG, "--- Task Stack High Water Marks & Health Status ---");

        // **TASK STATE MONITORING**: Check individual task health
        // Note: LVGL task is managed internally by esp_lvgl_port - no handle access for monitoring

        cat_parser_handle = get_cat_parser_task_handle();
        if (cat_parser_handle != NULL) {
            eTaskState parser_state = eTaskGetState(cat_parser_handle);
            if (parser_state == eSuspended) {
                ESP_LOGW(TAG, "CAT Parser task potentially stuck in state: %d", parser_state);
            } else if (parser_state == eBlocked) {
                ESP_LOGD(TAG, "CAT Parser task normal blocked state (waiting): %d", parser_state);
            }
            log_task_stack_usage("CAT Parser", cat_parser_handle);
        } else {
            ESP_LOGE(TAG, "CRITICAL: CAT Parser task handle is NULL!");
        }

        read_uart_handle = get_read_uart_task_handle();
        if (read_uart_handle != NULL) {
            eTaskState uart_state = eTaskGetState(read_uart_handle);
            if (uart_state == eSuspended) {
                ESP_LOGW(TAG, "Read UART task potentially stuck in state: %d", uart_state);
            } else if (uart_state == eBlocked) {
                ESP_LOGD(TAG, "Read UART task normal blocked state (waiting): %d", uart_state);
            }
            log_task_stack_usage("Read UART", read_uart_handle);
        } else {
            ESP_LOGE(TAG, "CRITICAL: Read UART task handle is NULL!");
        }

        uart_tx_handle = get_uart_tx_task_handle();
        if (uart_tx_handle != NULL) {
            eTaskState tx_state = eTaskGetState(uart_tx_handle);
            if (tx_state == eSuspended) {
                ESP_LOGW(TAG, "UART TX task potentially stuck in state: %d", tx_state);
            } else if (tx_state == eBlocked) {
                ESP_LOGD(TAG, "UART TX task normal blocked state (waiting): %d", tx_state);
            }
            log_task_stack_usage("UART TX", uart_tx_handle);
        } else {
            ESP_LOGE(TAG, "CRITICAL: UART TX task handle is NULL!");
        }

#if USE_GPS
        gps_handle = get_gps_task_handle();
        log_task_stack_usage("GPS", gps_handle);
#endif

        // Also log for the current task (stack_monitoring_task itself)
        log_task_stack_usage("Stack Monitor", xTaskGetCurrentTaskHandle());
        
        // Perform hardware health check
        hardware_health_check();

        // CPU Core Utilization Monitoring using FreeRTOS runtime stats
        // Note: For dual-core ESP32, each IDLE task accounts for max 50% of totalRuntime
        // when its core is fully idle. Scale by 2 to get actual per-core percentage.
        {
            static char runTimeStatsBuffer[2048];
            static char parseBuffer[2048];

            vTaskGetRunTimeStats(runTimeStatsBuffer);

            uint32_t idle0Runtime = 0, idle1Runtime = 0, totalRuntime = 0;

            strncpy(parseBuffer, runTimeStatsBuffer, sizeof(parseBuffer) - 1);
            parseBuffer[sizeof(parseBuffer) - 1] = '\0';

            char* line = strtok(parseBuffer, "\n\r");
            while (line != NULL) {
                char taskName[configMAX_TASK_NAME_LEN];
                uint32_t runtime;

                if (sscanf(line, "%s %lu", taskName, &runtime) >= 2) {
                    if (strcmp(taskName, "IDLE0") == 0) {
                        idle0Runtime = runtime;
                    } else if (strcmp(taskName, "IDLE1") == 0) {
                        idle1Runtime = runtime;
                    }
                    totalRuntime += runtime;
                }
                line = strtok(NULL, "\n\r");
            }

            if (totalRuntime > 0) {
                // For dual-core: each core's capacity is totalRuntime/2
                // Core usage = 100% - (idleRuntime / (totalRuntime/2) * 100%)
                //            = 100% - (2 * idleRuntime / totalRuntime * 100%)
                float idle0Pct = (float)idle0Runtime / totalRuntime * 100.0f * 2.0f;
                float idle1Pct = (float)idle1Runtime / totalRuntime * 100.0f * 2.0f;

                // Clamp to valid range (accounting for timing jitter)
                if (idle0Pct > 100.0f) idle0Pct = 100.0f;
                if (idle1Pct > 100.0f) idle1Pct = 100.0f;

                float core0Usage = 100.0f - idle0Pct;
                float core1Usage = 100.0f - idle1Pct;

                ESP_LOGI(TAG, "--- CPU Utilization (since boot) ---");
                ESP_LOGI(TAG, "Core 0: %4.1f%% busy, Core 1: %4.1f%% busy", core0Usage, core1Usage);
            }
        }

        // LVGL Memory Monitoring - skip if we can't get mutex quickly
        if (lvgl_port_lock(5)) {  // 5ms timeout
            ESP_LOGI(TAG, "--- LVGL Memory ---");
            // Note: lv_mem_monitor removed in LVGL 9, use heap stats instead
            size_t lvgl_free = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
            ESP_LOGI(TAG, "Heap free: %u bytes", (unsigned int)lvgl_free);
            lvgl_port_unlock();
        } else {
            ESP_LOGI(TAG, "LVGL Memory: Skipped (mutex busy)");
        }

        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Memory: Free=%luKB, Min=%luKB",
                 esp_get_free_heap_size()/1024, esp_get_minimum_free_heap_size()/1024);
        ESP_LOGI(TAG, "=== Memory Monitor Cycle Complete, sleeping for 120s ===");

        // Sleep for 120 seconds before next monitoring cycle
        // Task health monitoring runs every 30s in the main loop above
        vTaskDelay(pdMS_TO_TICKS(120000));
    }
}
