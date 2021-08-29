
// DHT22
DHT myDHT22(DHT22_PIN, DHTTYPE);
float dhtTemp = 0.0;
float dhtHumidity = 0.0;

unsigned long dhtTime = 0;

void begin_dht(){
  myDHT22.begin();
}

void update_dht(unsigned long ms) {
  if ((ms - dhtTime) > 2000) {
    dhtHumidity = myDHT22.readHumidity();
    if (isnan(dhtHumidity)){
      dhtHumidity = -1;
    }
    dhtTemp = myDHT22.readTemperature();
    if(isnan(dhtTemp)){
      dhtTemp = -40;
    }
    dhtTime = ms;
  }
}

void dht_json_response(WiFiEspClient client){  
  client.print(", \"boxTemp\": ");
  client.print(dhtTemp);
  client.print(", \"boxHumidity\": ");
  client.print(dhtHumidity);
}
