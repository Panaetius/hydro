#include <DHT22.h>

#include "WiFiEsp.h"
#include <OneWire.h>
#include <EEPROM.h>
#include "GravityTDS.h"
#include <Regexp.h>

#include "arduino_secrets.h"

// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(6, 7); // RX, TX
#endif

#define TdsSensorPin A1

// wifi
char ssid[] = WIFI_SSID;            // your network SSID (name)
char pass[] = WIFI_PASSWD;        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

WiFiEspServer server(80);

// fan
word fanPin = 9;
word fanTacho = 2; // only use 2, 3, 18, 19, 20 or 21 on arduino mega 2560
float fanSpeed = 10;
volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;
unsigned int fanOnTime = 5;
unsigned int fanOffTime = 5;
bool fanState = true;
unsigned long fanStateTime;

// tds/EC
GravityTDS gravityTds;
float tdsValue = 0;

// water temp
int DS18S20_Pin = 4; //DS18S20 Signal pin on digital 4
OneWire ds(DS18S20_Pin);  // on digital pin 5
float waterTemp = 0;

// General
MatchState matchState;

void setup()
{
  // initialize serial for debugging
  Serial.begin(115200);
  // initialize serial for ESP module
  Serial1.begin(115200);

  // initialize fan
  pinMode(fanPin, OUTPUT);
  pinMode(fanTacho, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(fanTacho), rpm_fun, RISING);
  half_revolutions = 0;
  rpm = 0;
  timeold = 0;
  fanStateTime = 0;
  pwm25kHzBegin();
  pwmDuty((byte)fanSpeed);

  // initialize tds
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization
  
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  Serial.println("You're connected to the network");
  printWifiStatus();
  
  // start the web server on port 80
  server.begin();
}


void loop()
{  
  unsigned long ms = millis(); 
  
  if (ms < timeold){
    //Deal with overflow after ~50 days
    timeold = ms; 
  }
  
  if (ms < fanStateTime){
    //Deal with overflow after ~50 days
    fanStateTime = ms; 
  }
  if (half_revolutions >= 20 || (ms - timeold) > 3000) { 
    //Update RPM every 20 counts, increase this for better RPM resolution,
    //decrease for faster update
    rpm = 30*1000/(ms - timeold)*half_revolutions;
    timeold = ms;
    half_revolutions = 0;
  }

  if (fanState && (ms - fanStateTime) > fanOnTime * 60000){
    fanState = false;
    pwmDuty((byte)0);
    fanStateTime = ms;
  } else if(!fanState && (ms - fanStateTime) > fanOffTime * 60000){
    fanState = true;
    pwmDuty((byte)(fanSpeed / (100.0 / 79)));
    fanStateTime = ms;
  }
  // listen for incoming clients
  WiFiEspClient client = server.available();
  if (client) {
    Serial.println("New client");
    
    String currentLine = "";

    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (currentLine.length() < 20){
          currentLine += c;
        }
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (currentLine == "\r\n") {
          Serial.println("Sending response");

          waterTemp = getTemp();
          gravityTds.setTemperature(waterTemp);
          gravityTds.update();
          tdsValue = gravityTds.getTdsValue();
          
          //sendHttpResponse(client, waterTemp, tdsValue);
          sendJsonReponse(client, waterTemp, tdsValue);
          currentLine = "";
          break;
        }
        Serial.print(c);

        if (c == '\n'){
          
          if (currentLine.startsWith("GET /?")) {
            char target[100];
            currentLine.toCharArray(target, 100);
            matchState.Target(target);
            char result = matchState.Match("spd=([%d.]+)");
            if (result == REGEXP_MATCHED){
              char buf [10];
              matchState.GetCapture (buf, 0);
              fanSpeed = atof(buf);
              Serial.print("Setting speed to: ");
              Serial.println(fanSpeed / (100.0 / 79));
              pwmDuty((byte)(fanSpeed / (100.0 / 79)));
            }
            result = matchState.Match("fanOnTime=(%d+)");
            if (result == REGEXP_MATCHED){
              char buf [3];
              matchState.GetCapture (buf, 0);
              fanOnTime = atoi(buf);
              Serial.print("Setting fanOnTime to: ");
              Serial.println(fanOnTime);
            }
            result = matchState.Match("fanOffTime=(%d+)");
            if (result == REGEXP_MATCHED){
              char buf [3];
              matchState.GetCapture (buf, 0);
              fanOffTime = atoi(buf);
              Serial.print("Setting fanOffTime to: ");
              Serial.println(fanOffTime);
            }
//            char result = matchState.Match("ec=(%d+)");
//            if (result == REGEXP_MATCHED){
//              char buf [4];
//              matchState.GetCapture (buf, 0);
//              int measuredEc = atoi(buf);
//              Serial.print("Setting ec to: ");
//              Serial.println(ec);
//            }
          }
          currentLine = "";
        }         
      }
    }
    // give the web browser time to receive the data
    delay(10);

    // close the connection:
    client.stop();
    Serial.println("Client disconnected");
  }
}

void sendJsonReponse(WiFiEspClient client, float waterTemp, float tdsValue){
  client.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: application/json\r\n"
    "Connection: close\r\n"  // the connection will be closed after completion of the response
    "\r\n");
  client.print("{\"waterTemp\": ");
  client.print(waterTemp);
  client.print(", \"ec\": ");
  client.print(2 * tdsValue);
  client.print(", \"fanSpeed\": ");
  client.print(fanSpeed);
  client.print(", \"rpm\": ");
  client.print(rpm);
  client.print(", \"fanOnTime\": ");
  client.print(fanOnTime);
  client.print(", \"fanOffTime\": ");
  client.print(fanOffTime);
  client.print("}\r\n");
}

void sendHttpResponse(WiFiEspClient client, float waterTemp, float tdsValue){
  // send a standard http response header
  // use \r\n instead of many println statements to speedup data send
  client.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Connection: close\r\n"  // the connection will be closed after completion of the response
    "\r\n");
  client.print("<!DOCTYPE HTML>\r\n");
  client.print("<html>\r\n");
  client.print("<h1>Hydroponic Control!</h1>\r\n");
  client.print("<br><b>Water Temperature: </b>");
  client.print(waterTemp);
  client.print("&deg;C<br><b>EC: </b>");
  client.print(2 * tdsValue);
  client.print("&micro;S<br><b>Fan RPM: </b>");
  client.print(rpm);
  client.print("<br><br><form >\r\n");
  client.print("<b>Fan Speed: </b>0<input type='range' min='0' max='100' value='");
  client.print(fanSpeed);
  client.print("' class='slider' name='spd'>100(");
  client.print(fanSpeed);
  client.print(")<br><br>\r\n");
  client.print("<input type='submit' name='Submit'></form>\r\n");
  client.print("</html>\r\n");
}

void pwm25kHzBegin() {
  TCCR2A = 0;                               // TC2 Control Register A
  TCCR2B = 0;                               // TC2 Control Register B
  TIMSK2 = 0;                               // TC2 Interrupt Mask Register
  TIFR2 = 0;                                // TC2 Interrupt Flag Register
  TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);  // OC2B cleared/set on match when up/down counting, fast PWM
  TCCR2B |= (1 << WGM22) | (1 << CS21);     // prescaler 8
  OCR2A = 79;                               // TOP overflow value (Hz)
  OCR2B = 0;
}

void pwmDuty(byte ocrb) {
  OCR2B = ocrb;                             // PWM Width (duty)
}

void rpm_fun()
{
  half_revolutions++;
  //Each rotation, this interrupt function is run twice
}

float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;

}


void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  
  // print where to go in the browser
  Serial.println();
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
  Serial.println();
}
