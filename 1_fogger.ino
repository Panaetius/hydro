
//fogger
word fogger1Pin = 6;
word fogger2Pin = 7;
//unsigned int foggerOffset = 60;
bool foggerState = true;
unsigned long fogStateTime = 0;
unsigned long fogOnTime = 60;
unsigned long fogOffTime = 120;
int fogOnOffset = 8;
int fogOffOffset = 12;


void begin_foggers(){
  fogOnTime = getEEPROM(eepromStart + fogOnOffset, fogOnTime);
  fogOffTime = getEEPROM(eepromStart + fogOffOffset, fogOffTime);
  pinMode(fogger1Pin, OUTPUT);
  pinMode(fogger2Pin, OUTPUT);
  digitalWrite(fogger1Pin, HIGH);
  digitalWrite(fogger2Pin, HIGH);
  foggerState = true;
}


void update_foggers(unsigned long ms){
  if (foggerState && (ms - fogStateTime) > (fogOnTime * 1000)) {
    foggerState = false;
    digitalWrite(fogger1Pin, LOW);
    digitalWrite(fogger2Pin, LOW);
    fogStateTime = ms;
  } else if (!foggerState && (ms - fogStateTime) > (fogOffTime * 1000)) {
    foggerState = true;
    
    
    digitalWrite(fogger1Pin, HIGH);
    digitalWrite(fogger2Pin, HIGH);
    fogStateTime = ms;
  }
}

void handle_foggers_update(MatchState matchState){
  //fogger on/off
  char result = matchState.Match("fogOnTime=(%d+)");
  if (result == REGEXP_MATCHED) {
    char buf [3];
    matchState.GetCapture (buf, 0);
    fogOnTime = atoi(buf);
    writeEEPROM(eepromStart + fogOnOffset, fogOnTime);
    Serial.print("Setting fogOnTime to: ");
    Serial.println(fogOnTime);
  }
  result = matchState.Match("fogOffTime=(%d+)");
  if (result == REGEXP_MATCHED) {
    char buf [3];
    matchState.GetCapture (buf, 0);
    fogOffTime = atoi(buf);
    writeEEPROM(eepromStart + fogOffOffset, fogOffTime);
    Serial.print("Setting fogOffTime to: ");
    Serial.println(fogOffTime);
  }
}


void foggers_json_response(WiFiEspClient client){  
  client.print(", \"fogOnTime\": ");
  client.print(fogOnTime);
  client.print(", \"fogOffTime\": ");
  client.print(fogOffTime);
  client.print(", \"foggerState\": ");
  client.print(foggerState);
}
