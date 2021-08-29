DFRobot_PH ph;
float phVoltage,phValue,phVoltageCorrected;


void begin_ph(float waterTemp){
  pinMode(PhSensorPin, INPUT);
  
  phVoltage = analogRead(PhSensorPin);
  phVoltageCorrected = phVoltage / 1024.0 * 5000;  // read the voltage
  phValue = ph.readPH(phVoltageCorrected, waterTemp);  // convert voltage to pH with temperature compensation
  delay(50);
}

void update_ph(unsigned long ms){
  float waterTemp = getTemp();

  phVoltage = analogRead(PhSensorPin);
  phVoltageCorrected = phVoltage / 1024.0 * 5000;  // read the voltage
  phValue = ph.readPH(phVoltageCorrected, waterTemp);  // convert voltage to pH with temperature compensation
  delay(50);
}

void handle_ph_update(MatchState matchState){
  //calibrate pH
  char result = matchState.Match("phcal");
  if (result == REGEXP_MATCHED) {
    float waterTemp = getTemp();
    Serial.print("Calibrating pH");
    phVoltage = analogRead(PhSensorPin);
    phVoltageCorrected = phVoltage / 1024.0 * 5000;  // read the voltage
    ph.calibration(phVoltageCorrected, waterTemp, "ENTERPH");
    ph.calibration(phVoltageCorrected, waterTemp, "CALPH");
    ph.calibration(phVoltageCorrected, waterTemp, "EXITPH");
  }
}

void ph_json_response(WiFiEspClient client){  
  client.print(", \"ph\": ");
  client.print(phValue);
}
