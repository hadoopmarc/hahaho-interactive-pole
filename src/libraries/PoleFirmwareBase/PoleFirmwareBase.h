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
        char serialURI[11] = "/webserial";
        char updateURI[8] = "/update";
        char dashURI[6] = "/dash";

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
};

#endif // POLE_FIRMWARE_BASE_H
