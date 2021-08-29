
#define use_temp true
#define use_fan false
#define use_foggers false
#define use_dht false
#define use_nutrient_pumps true
#define use_hpa_pump true
#define use_ph true
#define use_ec true
#define use_lights false

// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(6, 7); // RX, TX
#endif

#define TdsSensorPin A12
#define TdsPowerPin 45
#define PhSensorPin A2
#define HpaPressureSensorPin A4
#define DHT22_PIN 44
#define DHTTYPE DHT22
