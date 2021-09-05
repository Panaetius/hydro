float minPressure = 5.5;
float maxPressure = 7;
float currentPressure, hpaCorrectedVoltage;

long sprayDuration = 2 * 1000;
unsigned long sprayInterval = 15 * 60.0;
word boxPins[] = {33, 34, 35};
word hpaPumpPin = 32;

bool hpaActive = false;
unsigned long lastSprayTime = 0;

int hpaDurationOffset = 20;
int hpaIntervalOffset = 24;


ISR_Timer ISR_Timer4;

void begin_hpa(){
  ITimer4.init();
  ITimer4.attachInterruptInterval(10, Timer4Handler);
  
  sprayDuration = getEEPROM(eepromStart + hpaDurationOffset, sprayDuration);
  sprayInterval = getEEPROM(eepromStart + hpaIntervalOffset, sprayInterval);
  pinMode(hpaPumpPin, OUTPUT);

  for (byte i = 0; i < (sizeof(boxPins) / sizeof(boxPins[0])); i++){
    pinMode(boxPins[i], OUTPUT);
  }
}

void update_hpa(unsigned long ms){
  if ((ms - lastSprayTime) > (sprayInterval * 1000) && !hpaActive) {
    hpaActive = true;
    int box = 0;
    digitalWrite(boxPins[box], HIGH);
    ISR_Timer4.setTimeout(sprayDuration, stopSpray, &box);    
  } else if (!hpaActive && getHpaPressure() < minPressure){
    hpaActive = true;
    digitalWrite(hpaPumpPin, HIGH);
    ISR_Timer4.setTimeout(200, maybeStopHpaPump);  
  } 
}

void hpa_pump_json_response(WiFiEspClient client){
  client.print(", \"hpaPressure\": ");
  client.print(getHpaPressure());  
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
  float hpaVoltage = analogRead(HpaPressureSensorPin);
  hpaCorrectedVoltage = hpaVoltage / 1024.0 * 5000;
  currentPressure = (hpaCorrectedVoltage - 500) * 2.25;
  return currentPressure;
}

void Timer4Handler(void)
{
  ISR_Timer4.run();
}

void maybeStopHpaPump(){
  if (getHpaPressure() < maxPressure){
    ISR_Timer4.setTimeout(200, maybeStopHpaPump);      
  } else{
    digitalWrite(hpaPumpPin, LOW);
    hpaActive = false;    
  }
}

void stopSpray(void* boxNumber){
  int box = *static_cast<int*>(boxNumber);
  Serial.println("Stopping box spraying");
  Serial.println(box);
  digitalWrite(boxPins[box], LOW);
  box++;

  if (box < (sizeof(boxPins) / sizeof(boxPins[0]))){
    digitalWrite(boxPins[box], HIGH);
    ISR_Timer4.setTimeout(sprayDuration, stopSpray, &box);
  }else{
    hpaActive = false;
    Serial.println(millis());
    lastSprayTime = millis();
  }  
}
