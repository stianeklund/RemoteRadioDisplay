#ifndef NTP_CLIENT_H
#define NTP_CLIENT_H

void init_ntp_client();
void deinit_ntp_client(void);
void time_sync_notification_cb(struct timeval *tv);

// Other function declarations...

#endif // NTP_CLIENT_H