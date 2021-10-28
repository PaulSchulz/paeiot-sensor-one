#include "arduino_secrets.h"
// -*- c -*-
/*
  PAE IoT LoRaWAN Sensor Module
  pariot-sensor-one

  Paul Schulz <paul@mawsonlakes.org>

  License: GNU Public License v3.0
*/

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

// Four Temperature/Humidity sensors are supported
#define DHTPIN1 2 // Digital pin connected to the DHT sensor 1
#define DHTPIN2 3 // Digital pin connected to the DHT sensor 2
#define DHTPIN3 4 // Digital pin connected to the DHT sensor 3
#define DHTPIN4 5 // Digital pin connected to the DHT sensor 4

#define NOT_A_TEMPERATURE -99.9
#define NOT_A_HUMIDITY    -1.0
#define DELAY             600000

// Type of sensor in use:
#define DHTTYPE    DHT22     // DHT 22 (AM2302)

// Declare sensors
DHT_Unified dht1(DHTPIN1, DHTTYPE);
DHT_Unified dht2(DHTPIN2, DHTTYPE);
DHT_Unified dht3(DHTPIN3, DHTTYPE);
DHT_Unified dht4(DHTPIN4, DHTTYPE);

uint32_t delayMS = DELAY;
uint32_t minDelayMS;
////////////////////////////////////////////////////////////////////////////// 
char* format_temperature(char * buff, int buffn, float temperature){
  float ctemp = NOT_A_TEMPERATURE;
  if (temperature == ctemp){
    snprintf(buff,buffn,"****");
  } else {
    snprintf(buff,buffn,"%4.1f",temperature);
  }
  return buff;
}

char* format_humidity(char * buff, int buffn, float humidity){
  float chumidity = NOT_A_HUMIDITY;
  if (humidity == chumidity){
    snprintf(buff,buffn,"*****");
  } else {
    snprintf(buff,buffn,"%5.1f",humidity);
  }
  return buff;
}

char* data_temperature(char * buff, int buffn, float temperature){
  float ctemp = NOT_A_TEMPERATURE;
  if (temperature == ctemp){
    snprintf(buff,buffn,"****");
  } else {
    snprintf(buff,buffn,"%4.1f",temperature);
  }
  return buff;
}

char* data_humidity(char * buff, int buffn, float humidity){
  float chumidity = NOT_A_HUMIDITY;
  if (humidity == chumidity){
    snprintf(buff,buffn,"*****");
  } else {
    snprintf(buff,buffn,"%5.1f",humidity);
  }
  return buff;
}

//////////////////////////////////////////////////////////////////////////////
void setup() {
  // Initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
   digitalWrite(LED_BUILTIN, HIGH);
   
    Serial.begin(115200);
    Serial.println();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(5000); // Allow serial monitor to catch up.
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println(F("---------------------------------"));
    Serial.println(F("--- Sensor One                ---"));
    Serial.println(F("--- Version: v1.2 29 Oct 2021 ---"));
    Serial.println(F("--- PAE IoT Experimenters     ---"));
    Serial.println(F("---------------------------------"));

    // Setup LoRaWAN Modem
    Serial.println("Starting Modem: Australian Band Plan AU915 ");
    // change this to your regional band (eg. US915, AS923, ...)
    if (!modem.begin(AU915)) {
        Serial.println("Failed to start module");
        while (1) {}
    };
    modem.sendMask("ff000000f000ffff00020000");
    Serial.print("Module version is: ");
    Serial.println(modem.version());
    Serial.print("DevEUI is: ");
    Serial.println(modem.deviceEUI());
    Serial.print("AppEUI is: ");
    Serial.println(SECRET_APP_EUI);
    Serial.print("AppKey is: ");
    Serial.println(SECRET_APP_KEY);
    
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
    dht1.begin();
    dht2.begin();
    dht3.begin();
    dht4.begin();

    // Print sensor details.
    sensor_t sensor;
    Serial.println(F("------------------------------------"));
    Serial.print(F("DHTxx Unified Sensor 1: "));
    dht1.temperature().getSensor(&sensor);
    Serial.println(sensor.name);
    
    Serial.print(F("DHTxx Unified Sensor 2: "));
    dht2.temperature().getSensor(&sensor);
    Serial.println(sensor.name);
    
    Serial.print(F("DHTxx Unified Sensor 3: "));
    dht3.temperature().getSensor(&sensor);
    Serial.println(sensor.name);
    
    Serial.print(F("DHTxx Unified Sensor 4: "));
    dht4.temperature().getSensor(&sensor);
    Serial.println(sensor.name);
    
    Serial.println(F("------------------------------------"));
    minDelayMS = sensor.min_delay / 1000;
    Serial.print  (F("Minimum sensor Delay(ms): ")); Serial.println(minDelayMS);
    Serial.print  (F("Transmit Delay(m):        ")); Serial.println(delayMS / 60000.0);
    Serial.println(F("------------------------------------"));
    
    Serial.println();
}

void loop() {
    // Get sensor data
    float t1,t2,t3,t4;
    float h1,h2,h3,h4;
    sensors_event_t event;

    // Sensor 1
    dht1.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        t1 = NOT_A_TEMPERATURE;
    }
    else {
        t1 = event.temperature;
    }

    dht1.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        h1 = NOT_A_HUMIDITY;
    }
    else {
        h1 = event.relative_humidity;
    }

    // Sensor 2
    dht2.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        t2 = NOT_A_TEMPERATURE;
    }
    else {
        t2 = event.temperature;
    }

    dht2.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        h2 = NOT_A_HUMIDITY;
    }
    else {
        h2 = event.relative_humidity;
    }

    // Sensor 3
    dht3.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        t3 = NOT_A_TEMPERATURE;
    }
    else {
        t3 = event.temperature;
    }

    dht3.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        h3 = NOT_A_HUMIDITY;
    }
    else {
        h3 = event.relative_humidity;
    }

    // Sensor 4
    dht4.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        t4 = NOT_A_TEMPERATURE;
    }
    else {
        t4 = event.temperature;
    }

    dht4.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        h4 = NOT_A_HUMIDITY;
    }
    else {
        h4 = event.relative_humidity;
    }

    char buff[100];
    char tbuff1[8];
    char tbuff2[8];
    char tbuff3[8];
    char tbuff4[8];
    char hbuff1[8];
    char hbuff2[8];
    char hbuff3[8];
    char hbuff4[8];
    snprintf(buff, sizeof(buff),
            "| %s %s %s %s | %s %s %s %s |",
            format_temperature(tbuff1, sizeof(tbuff1), t1),
            format_temperature(tbuff2, sizeof(tbuff2), t2),
            format_temperature(tbuff3, sizeof(tbuff3), t3),
            format_temperature(tbuff4, sizeof(tbuff4), t4),
            format_humidity(hbuff1, sizeof(hbuff1), h1),
            format_humidity(hbuff2, sizeof(hbuff2), h2),
            format_humidity(hbuff3, sizeof(hbuff3), h3),
            format_humidity(hbuff4, sizeof(hbuff4), h4)
            );
    Serial.print(buff);

    // Build LoRaWAN message
    char msg[100];
    snprintf(msg, sizeof(msg),
             "%4.1f %5.1f %4.1f %5.1f %4.1f %5.1f %4.1f %5.1f",
             t1,h1,
             t2,h2,
             t3,h3,
             t4,h4);

    // String msg = "T: "+buff+"H: T2: H2:";
    //Serial.print("Sending Message: "); Serial.print(msg);

    digitalWrite(LED_BUILTIN, HIGH);

    int err;
    modem.beginPacket();
    modem.print(msg);
    err = modem.endPacket(true);
    if (err > 0) {
        Serial.print(" [tx]");
    } else {
        Serial.print(" [error]");
    }

    delay(1000);

    if (modem.available()) {
        Serial.println(" [rx]");

        char rcv[64];
        int i = 0;
        while (modem.available()) {
            rcv[i++] = (char)modem.read();
        }

        Serial.print(" Received: ");
        for (unsigned int j = 0; j < i; j++) {
            Serial.print(rcv[j] >> 4, HEX);
            Serial.print(rcv[j] & 0xF, HEX);
            Serial.print(" ");
        }

    }

    digitalWrite(LED_BUILTIN, LOW);

    Serial.println(" [delay]");
    delay(delayMS);
}
