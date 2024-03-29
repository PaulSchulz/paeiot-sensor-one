#include "arduino_secrets.h"
// -*- c -*-
/*
  PAE IoT LoRaWAN Sensor Module
  pariot-sensor-one

  Paul Schulz <paul@mawsonlakes.org>

  License: GNU Public License v3.0
*/

#define BUILD "v1.4.1      01 Sep 2022                                 "

#define TRUE 1
#define FALSE 0

// Capabilities
#define FEATURE_SERIAL TRUE
#define LORAWAN        TRUE
#define SENSOR_STATUS  TRUE
#define SENSOR_DHT     FALSE
#define SENSOR_CCS811  FALSE
#define SENSOR_BME280  FALSE
#define SENSOR_SCD30   TRUE

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
// Status
// Global Loop Delay (when not using Watchdog.sleep)
#define DELAY             60000  // 1m

uint32_t delayMS = DELAY;

void status_setup() {
  
}

void status_loop() {

}

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

// CayenneLPP Channels
int dht_id1 = 0;
int dht_id2 = 1;
int dht_id3 = 2;
int dht_id4 = 3;

//#define DHT_NUMBER_SENSORS 4
//int dht_sensor_pins[DHT_NUMBER_SENSORS] = {2,3,4,5};
//int dht_sensor_ids[DHT_NUMBER_SENSORS]  = {0,1,2,3};

#define NOT_A_TEMPERATURE -99.9
#define NOT_A_HUMIDITY    -1.0
#define DELAY             60000
// Type of sensor in use:
//   DHT22 (AM2302)
#define DHTTYPE    DHT22

// Declare sensors
DHT_Unified dht1(DHTPIN1, DHTTYPE);
DHT_Unified dht2(DHTPIN2, DHTTYPE);
DHT_Unified dht3(DHTPIN3, DHTTYPE);
DHT_Unified dht4(DHTPIN4, DHTTYPE);

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
    lpp.addTemperature(dht_id1, t1);
  }

  dht1.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    h1 = NOT_A_HUMIDITY;
  } else {
    h1 = event.relative_humidity;
    lpp.addRelativeHumidity(dht_id1, h1);
  }

  // Sensor 2
  dht2.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    t2 = NOT_A_TEMPERATURE;
  } else {
    t2 = event.temperature;
    lpp.addTemperature(dht_id2, t2);
  }

  dht2.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    h2 = NOT_A_HUMIDITY;
  } else {
    h2 = event.relative_humidity;
    lpp.addRelativeHumidity(dht_id2, h2);
  }

  // Sensor 3
  dht3.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    t3 = NOT_A_TEMPERATURE;
  } else {
    t3 = event.temperature;
    lpp.addTemperature(dht_id3, t3);
  }

  dht3.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    h3 = NOT_A_HUMIDITY;
  } else {
    h3 = event.relative_humidity;
    lpp.addRelativeHumidity(dht_id3, h3);
  }

  // Sensor 4
  dht4.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    t4 = NOT_A_TEMPERATURE;
  } else {
    t4 = event.temperature;
    lpp.addTemperature(dht_id4, t4);
  }

  dht4.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    h4 = NOT_A_HUMIDITY;
  } else {
    h4 = event.relative_humidity;
     lpp.addRelativeHumidity(dht_id4, h4);
  }
}

#endif // SENSOR_DHT

////////////////////////////////////////////////////////////////////////////// 
// Sensor CCS811 - C02 and Volatile Organic Compounds
// Uses Adafruit_CCS811 Library
// CCS811 Sensor code based on CCS811_test example
// Uses I2C Interface
#if SENSOR_CCS811

# include "Adafruit_CCS811.h"
Adafruit_CCS811 ccs;

int ccs811_id1 = 16;
int ccs811_id2 = 17;
int ccs811_present=0;

void ccs811_setup() {
  Serial.println(F("------------------------------------------------------------------------------"));
  Serial.println(F("CCS811 - C02 and Volatile Organic Compounds"));
  Serial.print(F("  CayenneLPP Id CO2:  "));
  Serial.println(ccs811_id1);
  Serial.print(F("  CayenneLPP Id TVOC: "));
  Serial.println(ccs811_id2);

  if(ccs.begin()) {
    Serial.print("  Detected");
    ccs811_present=1;
  } else {
    Serial.println("Failed to detect sensor!");
  }
  
  // Wait for the sensor to be read.
  if (ccs811_present) {
    while(!ccs.available());
    Serial.println(" - Ready");
  }
}

void ccs811_loop() {
  if(ccs811_present) {
    if(ccs.available()){
      if(!ccs.readData()){
        lpp.addPower(ccs811_id1, ccs.geteCO2());
        lpp.addPower(ccs811_id2, ccs.getTVOC());        
      } else {
        Serial.println("ERROR!");
      }
    }
  } else {
    // Send Test Data
    lpp.addPower(ccs811_id1, 432);
    lpp.addPower(ccs811_id2, 234);
  }
}

#endif // SENSOR_CCS811
                                                  
////////////////////////////////////////////////////////////////////////////// 
// SparkFun BME280 - Version: Latest 
#if SENSOR_BME280

#include <SparkFunBME280.h>
#define BME280_ADDR 0x76
BME280 myBME280;

int bme280_id = 20;
int bme280_present = 0;

void bme280_setup() {
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
}

void bme280_loop() {
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
}

#endif // SENSOR_BME280

////////////////////////////////////////////////////////////////////////////// 
// Sensor SCD30 - C02, Temperatue and Humidity
#if SENSOR_SCD30
#include <Adafruit_SCD30.h>

Adafruit_SCD30  scd30;

int scd30_id=10;
int scd30_present=0;

void scd30_setup() {
  Serial.println(F("------------------------------------------------------------------------------"));
  Serial.println(F("SCD30 - CO2, Temperature and Humidity Sensor"));
  Serial.print(F("  CayenneLPP Id: "));
  Serial.println(scd30_id);

  // Try to initialize!
  if (scd30.begin()) { 
    Serial.println("  SCD30 Found!");
    scd30_present=1;
  } else {
    Serial.println("  Failed to find SCD30 chip");
  }
}

void scd30_loop() {
  //Serial.println("Reading SCD30 sensor data");

  if(scd30_present){
    if (scd30.dataReady()){
      if (!scd30.read()){
        Serial.println("Error reading SCD30 sensor data");
        return;
      }

      lpp.addTemperature(scd30_id, scd30.temperature);
      lpp.addRelativeHumidity(scd30_id, scd30.relative_humidity);
      lpp.addPower(scd30_id, scd30.CO2);
      // Concetration is not yet supported in NodeRed 
      //lpp.addConcentration(scd30_id, scd30.CO2);
    }
  } else {
    // Test Data
    lpp.addTemperature(scd30_id, 0.0);
    lpp.addRelativeHumidity(scd30_id, 0.0);
    lpp.addPower(scd30_id, 0);
    //lpp.addConcentration(scd30_id, 880);

  }
}

#endif // SENSOR_SCD30

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
  Watchdog.sleep();  // Low power sleep
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
int a2d_resolution       = 10;  // 10 bits
int powermon_max_reading = (1<<a2d_resolution) - 1; 
// It is possible to change the resolution if needed

void powermon_setup() {
  Serial.println(F("----------------------------------------------------------------------------"));
  Serial.println(F("Power Monitor (for Microclimates Sensor)"));
  Serial.println(F("  Voltage 1: Panel Voltage"));
  Serial.println(F("  Voltage 2: Battery Voltage"));
  Serial.print(F("  Resolution bits: "));
  Serial.println(a2d_resolution);
  Serial.print(F("  Maximum Count: "));
  Serial.println(powermon_max_reading);

  // Enable control for battery voltage reading, and disable.
  pinMode(powermon_enable_pin, OUTPUT);
  digitalWrite(powermon_enable_pin, LOW);

  // Change Resolution
  analogReadResolution(a2d_resolution); 
}

void powermon_loop() {
  int panel_reading   = 0;
  int battery_reading = 0;
  int max_reading = powermon_max_reading;
  float ref_voltage = 3.3;

  float panel_voltage;   // 0.0 - 22.0 V
  float battery_voltage; // 0.0 - 7.2 V 

  digitalWrite(powermon_enable_pin, HIGH);
  delay(100);
  panel_reading = analogRead(powermon_panel_pin);
  battery_reading = analogRead(powermon_battery_pin);
  digitalWrite(powermon_enable_pin, LOW);

  panel_voltage = 1.0*panel_reading/max_reading*ref_voltage*1047/47;
  battery_voltage = 1.0*battery_reading/max_reading*ref_voltage*1150/150;

  lpp.addVoltage(0, panel_voltage); 
  lpp.addVoltage(1, battery_voltage); 
}

#endif // FEATURE_POWER_MONITOR

////////////////////////////////////////////////////////////////////////////// 
// Utility Functions


//////////////////////////////////////////////////////////////////////////////
// Setup

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
  ccs811_setup();
  #endif // SENSOR_CCS811
  
  #if SENSOR_BME280
  bme280_setup();
  #endif // SENSOR_BME280

  #if SENSOR_SCD30
  scd30_setup();
  #endif // SENSOR_SCD30

  #if FEATURE_WATCHDOG
  watchdog_setup();
  #endif

  #if FEATURE_POWER_MONITOR
  powermon_setup();
  #endif

  Serial.println(F("------------------------------------------------------------------------------"));
  Serial.println(F("Features"));
  Serial.println(F("  CayenneLPP "));
  Serial.print  (F("  Transmit Delay:  2m40s (approx.)\n"));
  Serial.println(F("------------------------------------------------------------------------------"));
  
} // End of setup

//////////////////////////////////////////////////////////////////////////////
void loop() {

  // Reset LPP Packet
  lpp.reset();
 
  #if SENSOR_DHT
  dht_loop();
  #endif // SENSOR_DHT

  #if SENSOR_CCS811
  ccs811_loop();
  #endif // SENSOR_CCS811

  #if SENSOR_BME280
  bme280_loop();
  #endif // SENSOR_BME280 

  #if SENSOR_SCD30
  scd30_loop();
  #endif // SENSOR_SCD30

  #if FEATURE_POWER_MONITOR
  powermon_loop();
  #endif //FEATURE_POWER_MONITOR

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

  Serial.print(" [sleep]\n");

  for(int i=0; i<10; i++){
    #if FEATURE_WATCHDOG
    watchdog_loop();   // Kick watchdog and sleep.
    #else
    delay(delayMS/10); // Sleep
    #endif

  }
}
