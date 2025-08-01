#ifndef WEB_OTA_H
#define WEB_OTA_H

typedef struct Credentials
{
  char id[17];                  // e.g. username or Wi-Fi SSID
  char password[17];
} Credentials;

void web_ota_setup(AsyncWebServer& server, Credentials *btnCreds, int nbtn, Credentials *apCreds, Credentials *otaCreds);

#endif // WEB_OTA_H
