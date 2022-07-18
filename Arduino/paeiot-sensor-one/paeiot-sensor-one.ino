#include "arduino_secrets.h"
// -*- c -*-
/*
  PAE IoT LoRaWAN Sensor Module
  pariot-sensor-one

  Paul Schulz <paul@mawsonlakes.org>

  License: GNU Public License v3.0
*/

#define BUILD "v1.3.3rc1  16 Jul 2022                                  "

// TODO Fix capability code so that types of sensors can be
// easily switched in or out.

#define TRUE 1
#define FALSE 0

// Capabilities
#define FEATURE_SERIAL TRUE
#define LORAWAN        TRUE
#define SENSOR_STATUS  TRUE
#define SENSOR_DHT     TRUE
#define SENSOR_CCS811  FALSE
#define SENSOR_BME280  FALSE
#define SENSOR_CSD30   TRUE

#define FEATURE_RTC           FALSE
#define FEATURE_WATCHDOG      TRUE
#define FEATURE_POWER_MONITOR TRUE

#include <Wire.h>
//////////////////////////////////////////////////////////////////////////////
// MKRWAN Library
// LoRaWAN code is based on mkrwan1310-example
#include <MKRWAN.h>
LoRaModem modem;

// Please enter your sensitive data in the Secret tab or arduino_secrets.h
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

//////////////////////////////////////////////////////////////////////////////
// CayenneLPP Library
// Low Power Payload format
#include<CayenneLPP.h>
CayenneLPP lpp(51);

//////////////////////////////////////////////////////////////////////////////
// Status Message
void status_setup() {

}

void status_loop() {

}

//////////////////////////////////////////////////////////////////////////////
// Adafruit_CCS811 Library
// CCS811 Sensor code based on CCS811_test example
// Uses I2C Interface
# include "Adafruit_CCS811.h"
Adafruit_CCS811 ccs;

// SparkFun BME280 - Version: Latest 
#include <SparkFunBME280.h>
#define BME280_ADDR 0x76
BME280 myBME280;

//////////////////////////////////////////////////////////////////////////////
// DHT Sensor code is based on sensor-am2302-example
#include <Adafruit_Sensor.h>

#if SENSOR_DHT
#include <DHT.h>
#include <DHT_U.h>

// Four Temperature/Humidity sensors are supported
#define DHTPIN1 2 // Digital pin connected to the DHT sensor 1
#define DHTPIN2 3 // Digital pin connected to the DHT sensor 2
#define DHTPIN3 4 // Digital pin connected to the DHT sensor 3
#define DHTPIN4 5 // Digital pin connected to the DHT sensor 4

#define NOT_A_TEMPERATURE -99.9
#define NOT_A_HUMIDITY    -1.0
#define DELAY             60000
// Type of sensor in use:
#define DHTTYPE    DHT22     // DHT22 (AM2302)

// Declare sensors
DHT_Unified dht1(DHTPIN1, DHTTYPE);
DHT_Unified dht2(DHTPIN2, DHTTYPE);
DHT_Unified dht3(DHTPIN3, DHTTYPE);
DHT_Unified dht4(DHTPIN4, DHTTYPE);

uint32_t delayMS = DELAY;
uint32_t minDelayMS;

void dht_setup() {
  dht1.begin();
  dht2.begin();
  dht3.begin();
  dht4.begin();

  // Print sensor details
  sensor_t sensor;
  Serial.println(F("------------------------------------------------------------------------------"));
  Serial.println(F("DHTxx Unified Sensors - Temperature and Humidity"));
  Serial.print(F("  Sensor 1: "));
  dht1.temperature().getSensor(&sensor);
  Serial.println(sensor.name);
    
  Serial.print(F("  Sensor 2: "));
  dht2.temperature().getSensor(&sensor);
  Serial.println(sensor.name);
    
  Serial.print(F("  Sensor 3: "));
  dht3.temperature().getSensor(&sensor);
  Serial.println(sensor.name);
    
  Serial.print(F("  Sensor 4: "));
  dht4.temperature().getSensor(&sensor);
  Serial.println(sensor.name);
}

void dht_loop() {
  // DHT Sensors
  // Get sensor data
  float t1,t2,t3,t4;
  float h1,h2,h3,h4;
  sensors_event_t event;

  // Sensor1
  dht1.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    t1 = NOT_A_TEMPERATURE;
  } else {
    t1 = event.temperature;
  }

  dht1.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    h1 = NOT_A_HUMIDITY;
  } else {
    h1 = event.relative_humidity;
  }

  // Sensor 2
  dht2.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    t2 = NOT_A_TEMPERATURE;
  } else {
    t2 = event.temperature;
  }

  dht2.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    h2 = NOT_A_HUMIDITY;
  } else {
    h2 = event.relative_humidity;
  }

  // Sensor 3
  dht3.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    t3 = NOT_A_TEMPERATURE;
  } else {
    t3 = event.temperature;
  }

  dht3.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    h3 = NOT_A_HUMIDITY;
  } else {
    h3 = event.relative_humidity;
  }

  // Sensor 4
  dht4.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    t4 = NOT_A_TEMPERATURE;
  } else {
    t4 = event.temperature;
  }

  dht4.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    h4 = NOT_A_HUMIDITY;
  } else {
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

  //lpp.addTemperature(0, t1);
  //lpp.addRelativeHumidity(0, h1);
  //lpp.addTemperature(1, t2);
  //lpp.addRelativeHumidity(1, h2);
  //lpp.addTemperature(2, t3);
  //lpp.addRelativeHumidity(2, h3);
  //lpp.addTemperature(3, t4);
  //lpp.addRelativeHumidity(3, h4);
  // Testing
  lpp.addTemperature(0, 20.1);
  lpp.addRelativeHumidity(1, 0.9);

}

#endif // SENSOR_DHT

////////////////////////////////////////////////////////////////////////////// 
// Sensor CSD30 - C02, Temperatue and Humidity
#if SENSOR_CSD30
int csd30_ntemp=10;
float csd30_temp=20.2;

void csd30_setup() {
  Serial.println(F("------------------------------------------------------------------------------"));
  Serial.println(F("CSD30 - CO2, Temperature and Humidity Sensor"));
  Serial.print(F("  CayenneLPP Id: "));
  Serial.println(csd30_ntemp);
}

void csd30_loop() {
  lpp.addTemperature(csd30_ntemp, csd30_temp);
}

#endif // SENSOR_CSD30

//////////////////////////////////////////////////////////////////////////////
// Real Time Clock (RTC)
#if FEATURE_RTC
void rtc_setup() {
  // set alarm date in the future to test
  rtc.setAlarmMonth(12);
  rtc.setAlarmYear(2022);
  set_next_alarm(1);
  rtc.attachInterrupt(alarmMatch);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  Serial.println(F("----------------------------------------------------------------------------"));
  print_rtc();
  Serial.println(F("----------------------------------------------------------------------------"));
}

void rtc_loop() {
  if(rtc_matched) { // FEATURE_RTC interupt was fired
    rtc_matched=0;
    set_next_alarm(0);
    print_rtc();
  }
}

#endif //FEATURE_RTC

////////////////////////////////////////////////////////////////////////////// 
// Watchdog
// https://github.com/adafruit/Adafruit_SleepyDog
#if FEATURE_WATCHDOG
#include <Adafruit_SleepyDog.h>
#define FEATURE_WATCHDOG_TEST FALSE

int watchdog_timer=30000; // 30 second watchdog

void watchdog_setup() {
  Serial.println(F("----------------------------------------------------------------------------"));
  Serial.println(F("Watchdog"));
  Serial.print(F("  Timer: "));
  Serial.print(watchdog_timer/1000);
  Serial.println(F("s"));
  //Publish reason for restart (power, FEATURE_WATCHDOG etc)
  Serial.print("  Reset Reason code: ");  
  Serial.println(PM->RCAUSE.reg);

  Watchdog.enable(watchdog_timer);
}

void watchdog_loop() {
  Watchdog.reset();  // Kick watchdog

  Serial.print(".");

  #if FEATURE_WATCHDOG_TEST
  // trip the watchdog after 2 succesful loops
  if (millis() > 2*rtc_delay*60*1000) {
    delay(30*1000);  // this is longer than the watchdog trigger
  };
  #endif
}

#endif

////////////////////////////////////////////////////////////////////////////// 
// Power Monitor
#if FEATURE_POWER_MONITOR

int powermon_panel_pin   = A0;
int powermon_battery_pin = A1;
int powermon_enable_pin  = 0;

void powermon_setup() {
  Serial.println(F("----------------------------------------------------------------------------"));
  Serial.println(F("Power Monitor (for Microclimates Sensor)"));
  Serial.println(F("  Voltage 1: Panel Voltage"));
  Serial.println(F("  Voltage 2: Battery Voltage"));

  // Enable control for battery voltage reading, and disable.
  pinMode(powermon_enable_pin, OUTPUT);
  digitalWrite(powermon_enable_pin, LOW);
}

void powermon_loop() {
  int panel_reading   = 0;
  int battery_reading = 0;
  float panel_voltage;   // 0.0-1.0 -> 0-22V
  float battery_voltage; // 0.0-1.0 -> 0-1V

  digitalWrite(powermon_enable_pin, HIGH);
  delay(100);
  panel_reading = analogRead(powermon_panel_pin);
  battery_reading = analogRead(powermon_battery_pin);
  digitalWrite(powermon_enable_pin, LOW);

  panel_voltage = 22.0*panel_reading/1024;
  battery_voltage = 1.0*battery_reading/1024;

  lpp.addVoltage(0, panel_voltage); 
  lpp.addVoltage(1, battery_voltage); 
}

#endif // FEATURE_POWER_MONITOR

////////////////////////////////////////////////////////////////////////////// 
// Utility Functions

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
  Serial.println("-");
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(8000); // Allow serial monitor to catch up.
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println(F("------------------------------------------------------------------------------"));
  Serial.println(F("---    __  __ _                _ _            _                            ---"));    
  Serial.println(F("---   |  \\/  (_)__ _ _ ___  __| (_)_ __  __ _| |_ ___ ___                  ---"));
  Serial.println(F("---   | |\\/| | / _| '_/ _ \\/ _| | | '  \\/ _` |  _/ -_|_-<                  ---"));
  Serial.println(F("---   |_|  |_|_\\__|_| \\___/\\__|_|_|_|_|_\\__,_|\\__\\___/__/                  ---"));                     
  Serial.println(F("---                                                                        ---"));
  Serial.println(F("---   Sensor One                                                           ---"));  
  Serial.print("---   Build: ");
  Serial.print(BUILD);
  Serial.println(F("      ---"));
  Serial.println(F("---   PAE IoT Experimenters                                                ---"));
  Serial.println(F("---   https://stemlibrary.space/iot-experimenters                          ---"));
  Serial.println(F("------------------------------------------------------------------------------"));
  
  // Setup LoRaWAN Modem
  Serial.println("LoRaWAN - Radio");
  Serial.println("  Modem: Australian Band Plan AU915 ");
  // change this to your regional band (eg. US915, AS923, ...)
  if (!modem.begin(AU915)) {
    Serial.println("  Failed to start module");
    while (1) {}
  };
  modem.sendMask("ff000000f000ffff00020000");
  Serial.print("  Module version is: ");
  Serial.println(modem.version());
  Serial.print("  DevEUI is: ");
  Serial.println(modem.deviceEUI());
  Serial.print("  AppEUI is: ");
  Serial.println(SECRET_APP_EUI);
  Serial.print("  AppKey is: ");
  Serial.println(SECRET_APP_KEY);
    
  int connected = modem.joinOTAA(appEui, appKey);

  if (!connected) {
    Serial.println("Something went wrong; are you indoor? Move near a window and retry");
      while (1) {}
    }

  // Set poll interval to 60 secs.
  modem.minPollInterval(60);

  // Setup Sensors
  #if SENSOR_DHT
  dht_setup();
  #endif  

  #if SENSOR_CCS811  
  Serial.println(F("------------------------------------------------------------------------------"));
  Serial.print("CCS811 - C02 and Volatile Organic Compounds");

  if(!ccs.begin()){
    Serial.println(" Failed to start sensor! Please check your wiring.");
    // while(1);
  }
  Serial.print(" - Connected");

  // Wait for the sensor to be ready
  while(!ccs.available());
  Serial.println(" - Ready");
  #endif
  
  
  #if SENSOR_BME280
  Serial.println(F("------------------------------------------------------------------------------"));
  Serial.println("BME280 -Temperature, Pressure and Humidity");
  //For I2C, enable the following and disable the SPI section
  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x76; //BME280_ADDR
  delay(10);
  //Initialize BME280
  myBME280.settings.runMode = 3; //Normal mode
  myBME280.settings.tStandby = 0;
  myBME280.settings.filter = 4;
  myBME280.settings.tempOverSample = 5;
  myBME280.settings.pressOverSample = 5;
  myBME280.settings.humidOverSample = 5;
  delay(10); 
  myBME280.begin();
  #endif

  #if SENSOR_CSD30
  csd30_setup();
  #endif

  #if FEATURE_WATCHDOG
  watchdog_setup();
  #endif

  #if FEATURE_POWER_MONITOR
  powermon_setup();
  #endif

  Serial.println(F("------------------------------------------------------------------------------"));
  Serial.println(F("Features"));
  Serial.println(F("  CayenneLPP "));
  // minDelayMS = sensor.min_delay / 1000;
  // Serial.print  (F("  Minimum sensor Delay(ms): ")); Serial.println(minDelayMS);
  Serial.print  (F("  Transmit Delay(m):        ")); Serial.println(delayMS / 60000.0);

  Serial.println(F("------------------------------------------------------------------------------"));
  
} // End of setup

//////////////////////////////////////////////////////////////////////////////
void loop() {

  // Reset Packet
  lpp.reset();
 

  #if SENSOR_CSS811
  // CCS811 Sensor
  if(ccs.available()){
    if(!ccs.readData()){
      Serial.print("[");
      Serial.print(ccs.geteCO2());
      Serial.print(" ");
      Serial.print(ccs.getTVOC());
      Serial.print("]");
    } else {
      Serial.println("ERROR!");
      while(1);
   }
  }
  #endif

  #if SENSOR_BME280
  //Returns temperature
    Serial.print("[");
    Serial.print(myBME280.readTempC(), 2);
    Serial.print(" C]");
    //Returns pressure
    Serial.print("[");
    Serial.print(myBME280.readFloatPressure()/100.0, 2);
    Serial.print(" hPa]");
    Serial.print("[");
    Serial.print(myBME280.readFloatHumidity(), 0);
    Serial.print(" %RH]");
  #endif
   
  // LPP Message
  #if SENSOR_DHT
  dht_loop();
  #endif

  #if SENSOR_CSS811
  lpp.addConcentration(8, ccs.geteCO2());
  lpp.addConcentration(9, ccs.getTVOC());
  #endif

  #if FEATURE_POWER_MONITOR
  powermon_loop();
  #endif

  // Begin Transmit
  digitalWrite(LED_BUILTIN, HIGH);

  int err;
  modem.beginPacket();
  modem.write(lpp.getBuffer(),lpp.getSize());

  err = modem.endPacket(true);
  if (err > 0) {
    Serial.print("  [tx]");
  } else {
    Serial.print(" [error]");
  }

  delay(1000);

    if (modem.available()) {
        Serial.println(" [rx]");

        char rcv[64];
        unsigned int i = 0;
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

  Serial.print(" [delay]");

  for(int i=0; i<10; i++){
    #if FEATURE_WATCHDOG
    watchdog_loop();
    #endif

    delay(delayMS/10);
  }
  Serial.println();

}
