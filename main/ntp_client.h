#ifndef NTP_CLIENT_H
#define NTP_CLIENT_H
#include "esp_err.h"

esp_err_t init_ntp_client();
void deinit_ntp_client(void);
void ntp_sync_callback(struct timeval *tv);

// Other function declarations...

#endif // NTP_CLIENT_H