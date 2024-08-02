#include <stdio.h>
#include "esp_system.h"
#include "ntp_client.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "wifi_init.h"
#include "lcd_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

void app_main(void) 
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing WiFi");
    wifi_init();

    ESP_LOGI(TAG, "Initializing LCD and UI");
    lcd_init();
    init_ntp_client();

    // Main application loop
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}