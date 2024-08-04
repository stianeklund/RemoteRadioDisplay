#include "uart.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define BUF_SIZE (1024)
static const char *TAG = "UART";
extern uint8_t uart_data[BUF_SIZE]; 

esp_err_t init_uart() {
  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };
  esp_err_t ret = uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
  if (ret != ESP_OK) {
    printf("uart_driver_install error\n");
    return ret;
  }
  ret = uart_param_config(UART_NUM_0, &uart_config);
  if (ret != ESP_OK) {
    printf("uart_param_config error\n");
    return ret;
  }
  return ESP_OK;
}

void read_uart() {
  uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
  while (1) {
    int len =
        uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
    if (len) {
      // Process the received data
      // For example, print it:
      ESP_LOGI(TAG, "Received %d bytes: ", len);
      for (int i = 0; i < len; i++) {
        printf("%c", data[i]);
      }
      printf("\n");
    }
  }
  return data;
}
