#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "string.h"
#include "uart.h"
#include <stdio.h>


#define MAX_RESPONSE_LENGTH 50
static const char *TAG = "CAT_PARSER";

uint32_t parse_frequency(const char *response) {
  uint32_t freq = 0;
  if (strncmp(response, "FA", 2) == 0 && strlen(response) >= 13) {
    sscanf(response + 2, "%11d", &freq);
  }
  return freq;
}
void parse_uart() {
  char *data = (char *)malloc(MAX_RESPONSE_LENGTH);
  if (data == NULL) {
    printf("Failed to allocate memory\n");
    return;
  }

  while (1) {
    memset(data, 0, MAX_RESPONSE_LENGTH);
    uint8_t len = read_uart();
    if (len == 0) {
      ESP_LOGE(TAG, "Missing UART data here");
    }
    if (len > 0) {
      data[len] = '\0'; // Null-terminate the string
      if (data == NULL) {
        ESP_LOGE(TAG, "Data is null");
      }
      if (strncmp(data, "FA", 2) == 0) {
        uint32_t freq = parse_frequency(data);
        printf("Current frequency: %d Hz\n", freq);
        // Here you would update your display with the new frequency
        // update_display(freq);
      }
    }

    // Request frequency every 5 seconds
    /*
    static uint32_t last_request_time = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (current_time - last_request_time > 5000) {
        request_frequency();
        last_request_time = current_time;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    */
  }
  free(data);
}