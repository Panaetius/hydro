

// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(6, 7); // RX, TX
#endif

#define TdsSensorPin A12
#define TdsPowerPin 45
#define PhSensorPin A2
#define PhPowerPin 42
#define TempPowerPin 43
#define HpaPressureSensorPin A4
#define DHT22_PIN 44
#define DHTTYPE DHT22
