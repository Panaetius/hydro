//EEPROM
int eepromStart = 512;

void writeEEPROM(int address, int value) {
  EEPROM.put(address, 1);
  EEPROM.put(address + 2, value);
}

int getEEPROM(int address, int def) {
  int set = 0;
  EEPROM.get(address, set);
  if (set != 1) {
    return def;
  }
  EEPROM.get(address + 2, set);
  return set;
}
