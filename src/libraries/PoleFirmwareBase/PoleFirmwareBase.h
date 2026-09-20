#ifndef POLE_FIRMWARE_BASE_H
#define POLE_FIRMWARE_BASE_H

#include <Arduino.h>
#include <AsyncWebSerial.h>                // Install "AsyncWebSerial" by Sessa
#include <ESPAsyncWebServer.h>             // Install "ESP Aync WebServer" and "Async TCP" by ESP32Async
#include <ESPDash.h>

using namespace dash;


class PoleFirmwareBase {
    private:
        // Web server
        const char serialURI[11] = "/webserial";
        const char updateURI[8] = "/update";
        const char dashURI[6] = "/dash";
        const char *displayText;
        static void handlePush();
        void printToggleSwitches();

    public:
        AsyncWebServer* server = new AsyncWebServer(80);
        AsyncWebSerial webSerial;
        ESPDash dashboard = ESPDash(*server, dashURI, true);
        // Default dashboard Cards
        // ToDo: replace fake temp/humidity with red button toggle states and PIR readouts
        TemperatureCard<float> temperature = TemperatureCard(dashboard, "Temperature");
        HumidityCard<float> humidity = HumidityCard(dashboard, "humidity");

        PoleFirmwareBase();
        void setup();
        void sense_loop();
        void act_loop();
        bool getHitOnce();                          // Toggle state for a single hit
        bool getHitTwice();                         // Toggle state for a dual hit
        bool getHitThrice();                        // Toggle state for a triple hit
        void resetHitOnce();                        // Set HitOnce toggle switch to false
        void resetHitTwice();                       // Set HitTwice toggle switch to false
        void resetHitThrice();                      // Set HitTrice toggle switch to false
        void setDisplayText(const char *newText);
};

#endif // POLE_FIRMWARE_BASE_H
