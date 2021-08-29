
//light
word lightPin = 44;
int lightDuty = 50;
int lightDutyOffset = 4;


void begin_light(){
  lightDuty = getEEPROM(eepromStart + lightDutyOffset, lightDuty);
  pinMode(lightPin, OUTPUT);
  analogWrite(lightPin, lightDuty * 255 / 100);
}


void handle_lights_update(MatchState matchState){
  //light dimming
  char result = matchState.Match("light=(%d+)");
  if (result == REGEXP_MATCHED) {
    char buf [3];
    matchState.GetCapture (buf, 0);
    lightDuty = constrain(atoi(buf), 0, 100);
    writeEEPROM(eepromStart + lightDutyOffset, lightDuty);
    analogWrite(lightPin, lightDuty * 255 / 100);
    Serial.print("Setting light to: ");
    Serial.println(lightDuty);
  }
}
