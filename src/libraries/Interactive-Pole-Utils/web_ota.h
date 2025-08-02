#ifndef WEB_OTA_H
#define WEB_OTA_H

#include <pole_wifi.h>

void web_ota_setup(AsyncWebServer& server, Credentials *otaCreds);

#endif // WEB_OTA_H
