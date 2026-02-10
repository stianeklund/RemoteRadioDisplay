#ifndef GPS_CLIENT_H
#define GPS_CLIENT_H

#include "esp_err.h"
#include <time.h>

esp_err_t init_gps_client(void);
void deinit_gps_client(void);
esp_err_t gps_get_time(struct tm *timeinfo);
void gps_task(void *pvParameters);

#endif // GPS_CLIENT_H
