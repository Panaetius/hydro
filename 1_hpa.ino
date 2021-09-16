float minPressure = 6.0;
float maxPressure = 8.5;
float currentPressure, hpaCorrectedVoltage;
float pressureAfterSpray = -1.0;
float hpaZeroPressureVolt = 488.0;

long sprayDuration = 2 * 1000;
unsigned long sprayInterval = 15 * 60.0;
word boxPins[] = {33, 34, 35};
word hpaPumpPin = 32;
int currentBoxNum = 0;

bool hpaActive = false;
bool hpaPumpActive = false;
bool hpaError = false;
long maxPumpTime = 60;
unsigned long startPumpTime = 0;
unsigned long lastSprayTime = 0;
unsigned long lastHpaMeasurement = 0;
long hpaMeasurementInterval = 30;


int hpaDurationOffset = 20;
int hpaIntervalOffset = 24;


//ISR_Timer ISR_Timer4;

void begin_hpa(){
  //ITimer4.init();
  //ITimer4.attachInterruptInterval(10, Timer4Handler);
  
  sprayDuration = getEEPROM(eepromStart + hpaDurationOffset, sprayDuration);
  sprayInterval = getEEPROM(eepromStart + hpaIntervalOffset, sprayInterval);
  
  pinMode(HpaPressureSensorPin, INPUT);
  pinMode(hpaPumpPin, OUTPUT);

  for (byte i = 0; i < (sizeof(boxPins) / sizeof(boxPins[0])); i++){
    pinMode(boxPins[i], OUTPUT);
  }
}

void update_hpa(unsigned long ms){
  // check pump pressure every hpaMeasurementInterval seconds, unles currently pumping, then check every seconds
  if ((ms - lastHpaMeasurement > hpaMeasurementInterval * 1000) || (hpaPumpActive && (ms - lastHpaMeasurement > 250))){
    currentPressure = getHpaPressure();

    if (pressureAfterSpray > 0 && !hpaError && !hpaPumpActive && !hpaActive && pressureAfterSpray - currentPressure > 0.5){
      // pressure dropped 0.5 bar since last spray -> probably a leak -> DON'T PUMP!
      hpaError = true;
    }
    lastHpaMeasurement = ms;
  }
  
  if (ms - startPumpTime > maxPumpTime * 1000 && hpaPumpActive && !hpaError){
    Serial.println("Emergency HPA stop");
    digitalWrite(hpaPumpPin, LOW);
    hpaActive = false; 
    hpaPumpActive = false; 
    hpaError = true;
  }
  
  if ((ms - lastSprayTime) > (sprayInterval * 1000) && !hpaActive && !hpaError) {
    Serial.println("Spraying boxes");
    Serial.println(lastSprayTime);
    hpaActive = true;
    currentBoxNum = 0;
    lastSprayTime = ms + (sizeof(boxPins) / sizeof(boxPins[0])) * sprayDuration - 10; // since millis() doesn't work in timer, just set last spray time to the future
    digitalWrite(boxPins[currentBoxNum], HIGH);
    ISR_Timer5.setTimeout(sprayDuration, stopSpray);    
  } else if (!hpaActive && currentPressure < minPressure && !hpaError){
    Serial.println("Running pump to increase pressure.");
    hpaActive = true;
    hpaPumpActive = true;
    startPumpTime = ms;
    digitalWrite(hpaPumpPin, HIGH);
    ISR_Timer5.setTimeout(1000, maybeStopHpaPump);  
  } 
}

void hpa_pump_json_response(WiFiEspClient client){
  client.print(", \"hpaPressure\": ");
  client.print(currentPressure);  
  client.print(", \"hpaSprayInterval\": ");
  client.print(sprayInterval);  
  client.print(", \"hpaSprayDuration\": ");
  client.print(sprayDuration);  
  client.print(", \"hpalastSprayTime\": ");
  client.print(lastSprayTime);  
}

void handle_hpa_update(MatchState matchState){
  char result = matchState.Match("hpa_duration=(%d+)");
  if (result == REGEXP_MATCHED) {
    char buf [10];
    matchState.GetCapture (buf, 0);
    sprayDuration = atoi(buf);
    writeEEPROM(eepromStart + hpaDurationOffset, sprayDuration);
    Serial.print("Setting hpa spray duration to: ");
    Serial.println(sprayDuration);
  }

  result = matchState.Match("hpa_interval=(%d+)");
  if (result == REGEXP_MATCHED) {
    char buf [10];
    matchState.GetCapture (buf, 0);
    sprayInterval = atoi(buf);
    writeEEPROM(eepromStart + hpaDurationOffset, sprayInterval);
    Serial.print("Setting hpa spray interval to: ");
    Serial.println(sprayInterval);
  }
}

float getHpaPressure(){
  float hpaVoltage;
  float pressureSum = 0;
  for (int i = 0; i < 3; i++){
    hpaVoltage = analogRead(HpaPressureSensorPin);
    hpaCorrectedVoltage = hpaVoltage / 1024.0 * 5000;
    pressureSum += (hpaCorrectedVoltage - hpaZeroPressureVolt) * 9.8 / (4 * 1000);
    delay(50);
  }
  return pressureSum / 3.0;
}

//void Timer4Handler(void)
//{
//  ISR_Timer4.run();
//}

void maybeStopHpaPump(){
  if (currentPressure < maxPressure && !hpaError){
    ISR_Timer5.setTimeout(1000, maybeStopHpaPump);      
  } else{
    digitalWrite(hpaPumpPin, LOW);
    pressureAfterSpray = currentPressure;
    hpaActive = false;    
    hpaPumpActive = false;
  }
}

void stopSpray(){
  Serial.print("Stopping box spraying ");
  Serial.println(currentBoxNum);
  digitalWrite(boxPins[currentBoxNum], LOW);
  currentBoxNum++;

  if (currentBoxNum < (sizeof(boxPins) / sizeof(boxPins[0]))){
    digitalWrite(boxPins[currentBoxNum], HIGH);
    ISR_Timer5.setTimeout(sprayDuration, stopSpray);
  }else{
    currentBoxNum = 0;
    hpaActive = false;
    pressureAfterSpray = currentPressure;
    Serial.println("Finished spraying");
  }  
}
