// tds/EC
//GravityTDS gravityTds;
float ecValue = 0;
float ecSum = 0;

void begin_ec(float waterTemp){
  // initialize tds
  pinMode(TdsPowerPin, OUTPUT);
  digitalWrite(TdsPowerPin, HIGH);
  delay(50);
//  gravityTds.setPin(TdsSensorPin);
//  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
//  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
//  gravityTds.begin();  //initialization
  pinMode(TdsSensorPin, INPUT);

  //turn on foggers
  ecValue = getEc(waterTemp); 
  digitalWrite(TdsPowerPin, LOW);
  delay(100);
}

void update_ec(unsigned long ms, float waterTemp){
  digitalWrite(TdsPowerPin, HIGH);
  delay(50);
  ecValue = getEc(waterTemp);
  digitalWrite(TdsPowerPin, LOW);
  delay(100);
}

float getEc(float waterTemp){
  ecSum = 0;
  for (int x = 0; x < 10; x++){
    ecSum += analogRead(TdsSensorPin);
    delay(40);
  }
  float averageVoltage = (ecSum / 10) * 5.0 / 1024.0;

  //float compensatedVoltage = averageVoltage / (1.0 + 0.02 * (waterTemp - 25.0)); //temperature compensation
  return (133.42 * averageVoltage * averageVoltage * averageVoltage - 255.86 * averageVoltage * averageVoltage + 857.39 * averageVoltage) / (1.0 + 0.02 * (waterTemp - 25.0));
}

void ec_json_response(WiFiEspClient client){  
  client.print(", \"ec\": ");
  client.print(ecValue);
}
