word fanPin = 9;
word fanTacho = 2; // only use 2, 3, 18, 19, 20 or 21 on arduino mega 2560
int fanSpeed = 10;
volatile byte halfRevolutions;
unsigned int rpm;
unsigned long timeold;
unsigned int fanOnTime = 5;
unsigned int fanOffTime = 5;
bool fanState = true;
unsigned long fanStateTime;
int fanSpeedOffset = 0;

void begin_fan(){
  pinMode(fanPin, OUTPUT);
  pinMode(fanTacho, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(fanTacho), rpm_fun, RISING);
  halfRevolutions = 0;
  rpm = 0;
  timeold = 0;
  fanStateTime = 0;
  pwm25kHzBegin();
  fanSpeed = getEEPROM(eepromStart + fanSpeedOffset, fanSpeed);
  pwmDuty((byte)fanSpeed);
}

void update_fan(unsigned long ms){
  if (ms < fanStateTime) {
    //Deal with overflow after ~50 days
    fanStateTime = ms;
  }

  if (ms < timeold) {
    //Deal with overflow after ~50 days
    timeold = ms;
  }

  if (halfRevolutions >= 20 || (ms - timeold) > 3000) {
    //Update RPM every 20 counts, increase this for better RPM resolution,
    //decrease for faster update
    rpm = 30 * 1000 / (ms - timeold) * halfRevolutions;
    timeold = ms;
    halfRevolutions = 0;
  }
}

void handle_fan_update(MatchState matchState){
  // fan speed
  char result = matchState.Match("spd=([%d]+)");
  if (result == REGEXP_MATCHED) {
    char buf [10];
    matchState.GetCapture (buf, 0);
    fanSpeed = constrain(atoi(buf), 0, 100);
    writeEEPROM(eepromStart + fanSpeedOffset, fanSpeed);
    Serial.print("Setting speed to: ");
    Serial.println(fanSpeed / (100.0 / 79));
    pwmDuty((byte)(fanSpeed / (100.0 / 79)));
  }

  //fan on/off
  result = matchState.Match("fanOnTime=(%d+)");
  if (result == REGEXP_MATCHED) {
    char buf [3];
    matchState.GetCapture (buf, 0);
    fanOnTime = atoi(buf);
    Serial.print("Setting fanOnTime to: ");
    Serial.println(fanOnTime);
  }
  result = matchState.Match("fanOffTime=(%d+)");
  if (result == REGEXP_MATCHED) {
    char buf [3];
    matchState.GetCapture (buf, 0);
    fanOffTime = atoi(buf);
    Serial.print("Setting fanOffTime to: ");
    Serial.println(fanOffTime);
  }
}

void fan_json_response(WiFiEspClient client){
  client.print(", \"fanSpeed\": ");
  client.print(fanSpeed);
  client.print(", \"rpm\": ");
  client.print(rpm);
  client.print(", \"fanOnTime\": ");
  client.print(fanOnTime);
  client.print(", \"fanOffTime\": ");
  client.print(fanOffTime);
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
  halfRevolutions++;
  //Each rotation, this interrupt function is run twice
}
