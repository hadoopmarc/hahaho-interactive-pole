#ifndef POLE_WIFI_H
#define POLE_WIFI_H

typedef struct Credentials
{
  char id[17];                  // e.g. username or Wi-Fi SSID
  char password[17];
} Credentials;

void wifi_setup(Credentials *btnCreds, int nbtn, Credentials *apCreds);

#endif // POLE_WIFI_H
