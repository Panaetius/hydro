#define USE_TIMER_5 true
#define TDS_SENSOR_PIN A12
#define TDS_POWER_PIN 45
#define PH_SENSOR_PIN A1
#define DHT22_PIN 44
#define FOGGER_1_PIN 6
#define FOGGER_2_PIN 7

#define FAN_PIN 9
#define FAN_TACHO_PIN 2 // only use 2, 3, 18, 19, 20 or 21 on arduino mega 2560

#define LIGHT_PIN 44
#define DHTTYPE DHT22
#define DS18S20_PIN 4

#define WEBSERVER_PORT 80

/*
   pump
   have to be ints as needed by interrupt library accepts pointer only (which makes sense as maybe complex objects coudl be used/required)
*/
int PH_UP_PIN = 26;
int PH_DOWN_PIN = 27;
int FERT_1_PIN = 28;
int FERT_2_PIN = 29;
int FERT_3_PIN = 30;

const double mlToMs = 932.5;
const long waitBeforeNewAdjustment = 10 * 60 * 1000l;
const float phUpMl = 1.0;
const float phDownMl = 1.0;
const float fert1Ml = 3.0;
const float fert2Ml = 1.5;
const float fert3Ml = 0.75;
const float phMin = 5.8;
const float phMax = 6.5;


/*
   EEPROM
*/
int eepromStart = 512;
int fanSpeedOffset = 0;
int lightDutyOffset = 4;
int fogOnOffset = 8;
int fogOffOffset = 12;
int ecMinOffset = 16;
