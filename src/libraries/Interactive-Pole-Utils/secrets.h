#ifndef SECRETS_H
#define SECRETS_H

typedef struct Credentials
{
  char id[17];                  // e.g. username or Wi-Fi SSID
  char password[17];
} Credentials;

// Options for Wi-Fi networks to connect to
extern Credentials btnCredentials[];
extern const int btnCredsSize;

// Fallback option for the Interactive Pole to function as an access point
extern Credentials apCredentials;

// Account to execute OTA firmware updates
extern Credentials otaCredentials;

#endif // SECRETS_H
