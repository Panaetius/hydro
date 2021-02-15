hydro controller code.



Libraries needed:
DHT22
OneWire
WiFiEsp
GravityTDS
Regexp

For wiring, see:
https://wiki.dfrobot.com/Gravity__Analog_TDS_Sensor___Meter_For_Arduino_SKU__SEN0244
https://wiki.dfrobot.com/Waterproof_DS18B20_Digital_Temperature_Sensor__SKU_DFR0198_
https://create.arduino.cc/projecthub/tylerpeppy/25-khz-4-pin-pwm-fan-control-with-arduino-uno-3005a1
https://create.arduino.cc/projecthub/Topocalma/esp8266-arduino-wifi-shield-for-windows-255407

Default pins (On Arduino Mega 2560):
- Fan pwm: 9
- Fan tacho: 2
- Water temp: 4
- EC(TDS) pin: A1
- Wifi ESP: Serial1 (19(RX), 18(TX))
