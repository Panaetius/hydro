// pump timing
float mltos = 0.9325;
long waitBeforeNewAdjustment = 10 * 60 * 1000l;
long lastAdjustment = 0;
bool adjustmentActive = false;
float phUpMl = 5.0;
float phDownMl = 5.0;
float fert1Ml = 3.0;
float fert2Ml = 1.5;
float fert3Ml = 0.75;
float phMin = 5.8;
float phMax = 6.5;
float ecMin = 1500.0;
long lastPhAdjustment = 0;
long lastFertAdjustment = 0;

int phUpPin = 26;
int phDownPin = 27;
int fert1Pin = 28;
int fert2Pin = 29;
int fert3Pin = 30;

int ecMinOffset = 16;

void begin_pumps(){
  pinMode(phUpPin, OUTPUT);
  pinMode(phDownPin, OUTPUT);
  pinMode(fert1Pin, OUTPUT);
  pinMode(fert2Pin, OUTPUT);
  pinMode(fert3Pin, OUTPUT);
  ecMin = getEEPROM(eepromStart + ecMinOffset, ecMin);
}

void handle_nutrient_pumps_update(MatchState matchState){
  //ec target
  char result = matchState.Match("ecmin=(%d+)");
  if (result == REGEXP_MATCHED) {
    // TODO: implement this properly
    char buf [10];
    matchState.GetCapture (buf, 0);
    ecMin = atoi(buf);
    writeEEPROM(eepromStart + ecMinOffset, ecMin);
    Serial.print("Setting min ec to: ");
    Serial.println(ecMin);
  }
}

void update_pumps(unsigned long ms){  
  if (ms < lastAdjustment){
    lastAdjustment = ms;
  }
  Serial.println(adjustmentActive);
  Serial.println(ms - lastAdjustment);
  if (!adjustmentActive && (ms - lastAdjustment) > waitBeforeNewAdjustment){
    Serial.println("Checking parameters");
    adjustSolution(phValue, ecValue);
  }
}

void pumps_json_response(WiFiEspClient client){  
  client.print(", \"lastPhAdjustment\": ");
  client.print(lastPhAdjustment);
  client.print(", \"lastFertAdjustment\": ");
  client.print(lastFertAdjustment);
}

void adjustSolution(float phValue, float ecValue){
  long pumpTime;
  if(phValue < phMin){
    Serial.println("Increasing ph");
    pumpTime = (long)(mltos * phUpMl * 1000.0);
    Serial.println(pumpTime);
    digitalWrite(phUpPin, HIGH);
    ISR_Timer5.setTimeout(pumpTime, stopPump, &phUpPin);
    adjustmentActive = true;
    lastPhAdjustment = millis();
    return;
  }
  if(phValue > phMax){
    Serial.println("Decreasing ph");
    pumpTime = (long)(mltos * phDownMl * 1000.0);
    Serial.println(pumpTime);
    digitalWrite(phDownPin, HIGH);
    ISR_Timer5.setTimeout(pumpTime, stopPump, &phDownPin);
    adjustmentActive = true;
    lastPhAdjustment = millis();
    return;
  }
  if(ecValue < ecMin){
    Serial.println("Increasing fertilizer");
    pumpTime = (long)(mltos * fert1Ml * 1000.0);
    Serial.println(millis());
    Serial.println(pumpTime);
    digitalWrite(fert1Pin, HIGH);
    ISR_Timer5.setTimeout(pumpTime, stopPump, &fert1Pin);
    pumpTime = (long)(mltos * fert2Ml * 1000.0);
    digitalWrite(fert2Pin, HIGH);
    ISR_Timer5.setTimeout(pumpTime, stopPump, &fert2Pin);
    pumpTime = (long)(mltos * fert3Ml * 1000.0);
    digitalWrite(fert3Pin, HIGH);
    ISR_Timer5.setTimeout(pumpTime, stopPump, &fert3Pin);
    adjustmentActive = true;
    lastFertAdjustment = millis();
    return;
  }
}

void stopPump(void* pumpPin){
  int pin = *static_cast<int*>(pumpPin);
  Serial.println("Stopping pump");
  Serial.println(pin);
  digitalWrite(pin, LOW);
  adjustmentActive = false;
  Serial.println(millis());
  lastAdjustment = millis();
  
}
