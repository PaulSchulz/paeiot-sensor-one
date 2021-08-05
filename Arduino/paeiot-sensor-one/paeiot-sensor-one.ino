// -*- c -*-
/*
  PAE IoT LoRaWAN Sensor Module
  pariot-sensor-one

  Paul Schulz <paul@mawsonlakes.org>

  License: GNU Public License v3.0
*/

#include "arduino_secrets.h"

//////////////////////////////////////////////////////////////////////////////
// LoRaWAN code is based on mkrwan1310-example
#include <MKRWAN.h>
LoRaModem modem;

// Please enter your sensitive data in the Secret tab or arduino_secrets.h
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

//////////////////////////////////////////////////////////////////////////////
// DHT Sensor code is based on sensor-am2302-example
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2   // Digital pin connected to the DHT sensor
#define DHTPIN2 3  // Digital pin connected to the DHT sensor

// Type of sensor in use:
#define DHTTYPE    DHT22     // DHT 22 (AM2302)

DHT_Unified dht(DHTPIN, DHTTYPE);
DHT_Unified dht2(DHTPIN2, DHTTYPE);

uint32_t delayMS;
//////////////////////////////////////////////////////////////////////////////
void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println(F("------------------------------------"));
    Serial.println(F("---     Sensor One               ---"));
    Serial.println(F("---     PAE IoT Experimenters    ---"));
    Serial.println(F("------------------------------------"));

    // Setup LoRaWAN Modem
    Serial.println("Starting Modem: Australian Band Plan AU915 ");
    // change this to your regional band (eg. US915, AS923, ...)
    if (!modem.begin(AU915)) {
        Serial.println("Failed to start module");
        while (1) {}
    };
    modem.sendMask("ff000000f000ffff00020000");
    Serial.print("Your module version is: ");
    Serial.println(modem.version());
    Serial.print("Your device EUI is: ");
    Serial.println(modem.deviceEUI());
    int connected = modem.joinOTAA(appEui, appKey);

    if (!connected) {
        Serial.println("Something went wrong; are you indoor? Move near a window and retry");
        while (1) {}
    }

    // Set poll interval to 60 secs.
    modem.minPollInterval(60);

    // NOTE: independently by this setting the modem will
    // not allow to send more than one message every 2 minutes,
    // this is enforced by firmware and can not be changed.

    // Setup Sensors
    // Initialize device.
    dht.begin();
    dht2.begin();

    Serial.println(F("DHTxx Unified Sensor 1"));
    // Print temperature sensor details.
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.println(F("Temperature Sensor: ")); Serial.println(sensor.name);
    delayMS = sensor.min_delay / 1000;
    Serial.print  (F("Minimum Delay(ms):   ")); delayMS;
}

void loop() {
    // Get sensor data
    float t1,t2,h1,h2;
    sensors_event_t event;

    // Sensor 1
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        t1 = -99.9;
        Serial.println(F("Error reading temperature 1!"));
    }
    else {
        t1 = event.temperature;
        Serial.print(F("Temperature 1: "));
        Serial.print(t1); Serial.println(F("°C"));
    }

    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        h1 = -1.0;
        Serial.println(F("Error reading humidity 1!"));
    }
    else {
        h1 = event.relative_humidity;
        Serial.print(F("Humidity: "));
        Serial.print(h1); Serial.println(F("%"));
    }

    // Sensor 2
    dht2.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        t2 = -99.9;
        Serial.println(F("Error reading temperature 2!"));
    }
    else {
        t2 = event.temperature;
        Serial.print(F("Temperature 2: "));
        Serial.print(t1); Serial.println(F("°C"));
    }

    dht2.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        h2 = -1.0;
        Serial.println(F("Error reading humidity 2!"));
    }
    else {
        h2 = event.relative_humidity;
        Serial.print(F("Humidity: "));
        Serial.print(h1); Serial.println(F("%"));
    }

    char msg[100];
    snprintf(msg, sizeof(msg),
             "%4.1f %5.1f %4.1f %5.1f",
             t1,h1,t2,h2);

// String msg = "T: "+buff+"H: T2: H2:";
    Serial.print("Sending Message: "); Serial.println(msg);

    int err;
    modem.beginPacket();
    modem.print(msg);
    err = modem.endPacket(true);
    if (err > 0) {
        Serial.println("Message sent correctly!");
    } else {
        Serial.println("Error sending message.");
    }

    Serial.println("Wait for download messages");
    delay(1000);

    if (modem.available()) {
        Serial.println("- Downlink message received");

        char rcv[64];
        int i = 0;
        while (modem.available()) {
            rcv[i++] = (char)modem.read();
        }

        Serial.print("Received: ");
        for (unsigned int j = 0; j < i; j++) {
            Serial.print(rcv[j] >> 4, HEX);
            Serial.print(rcv[j] & 0xF, HEX);
            Serial.print(" ");
        }

    }
    Serial.println("Wait 2 min (120000ms)");
    delay(60000);
    delay(60000);

    Serial.println();
}
