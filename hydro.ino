#include <DHT.h>


#include "WiFiEsp.h"
#include <OneWire.h>
#include <EEPROM.h>
//#include "GravityTDS.h"
#include "DFRobot_PH.h"
#include <Regexp.h>


#define USE_TIMER_4     true
#define USE_TIMER_5     true

#include "TimerInterrupt.h"
#include "ISR_Timer.h"

#include "arduino_secrets.h"

// wifi
char ssid[] = WIFI_SSID;            // your network SSID (name)
char pass[] = WIFI_PASSWD;        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status
long lastWifiStateCheck = 0;

WiFiEspServer server(80);

// General
MatchState matchState;

void setup()
{
  // initialize serial for debugging
  Serial.begin(115200);
  // initialize serial for ESP module
  Serial1.begin(115200);

#if use_dht
  begin_dht()
#endif

#if use_pumps
  begin_pumps();
#endif
  
#if use_fan
  begin_fan();
#endif

#if use_ph    
  waterTemp = getTemp();
  begin_ph(waterTemp);
#endif

#if use_ec
  waterTemp = getTemp();
  begin_ec(waterTemp);
#endif
    
#if use_foggers
  begin_foggers();
#endif

#if use_hpa_pump
  begin_hpa();
#endif

#if use_light
  begin_light();
#endif

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

  if (ms < lastWifiStateCheck){
    lastWifiStateCheck = ms;
  }

#if use_dht
  update_dht(ms);
#endif

#if use_fan
  update_fan(ms);
#endif

#if use_foggers
  update_foggers(ms);
#endif

#if use_pumps
  update_pumps(ms);
#endif

#if use_hpa_pump
  update_hpa(ms);
#endif

#if use_ph
  update_ph(ms);
#endif

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
        if (currentLine.length() < 20) {
          currentLine += c;
        }
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (currentLine == "\r\n") {
          Serial.println("Sending response");
          sendJsonReponse(client);
          currentLine = "";
          break;
        }
        Serial.print(c);

        if (c == '\n') {

          if (currentLine.startsWith("GET /?")) {
            char target[100];
            currentLine.toCharArray(target, 100);
            matchState.Target(target);

#if use_fan
            handle_fan_update(matchState);
#endif

#if use_foggers
            handle_foggers_update(matchState);
#endif

#if use_lights
            handle_lights_update(matchState);
#endif

#if use_ph
            handle_ph_update(matchState);
#endif

#if use_nutrient_pumps
            handle_nutrient_pumps_update(matchState);
#endif  

#if use_hpa_pump
            handle_hpa_update(matchState);
#endif
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
    delay(100);
  } else {
    
    // attempt to reconnect to WiFi network
    if ((ms - lastWifiStateCheck) > 10000){
      status = WiFi.status();
      if (status != 1){
        Serial.println(status);
      }
      if (status != WL_CONNECTED){
        if ( status == WL_DISCONNECTED) {
          Serial.print("Attempting to connect to WPA SSID: ");
          Serial.println(ssid);
          // Connect to WPA/WPA2 network
          status = WiFi.begin(ssid, pass);
          if ( status != WL_DISCONNECTED) {
            Serial.println("starting server");
            server.begin();  
          }   
        }  
      }
      lastWifiStateCheck = ms;
    }
  }
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
