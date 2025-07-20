#ifndef WEB_OTA_H
#define WEB_OTA_H

typedef struct Credentials
{
  char id[17];        // e.g. username or Wi-Fi SSID
  char password[17];
} Credentials;

void ota_setup(Credentials *btnCreds, Credentials *apCreds, Credentials *otaCreds);
void ota_loop();

#endif // WEB_OTA_H
