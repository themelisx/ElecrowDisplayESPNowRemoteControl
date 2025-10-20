#include <Arduino.h>

#include "settings.h"
#include "defines.h"
#include "vars.h"

#include "debug.h"

#ifdef ENABLE_EEPROM
  #include "myEEPROM.h"    
#endif

Settings::Settings() {

  debug->println(DEBUG_LEVEL_DEBUG, "[Settings]");

  #ifdef ENABLE_EEPROM
    myEEPROM = new MyEEPROM(512);
    myEEPROM->start();
    debug->println(DEBUG_LEVEL_DEBUG, "[OK]");
  #endif
}

void Settings::load() {
  debug->println(DEBUG_LEVEL_INFO, "Loading settings...");

  #ifdef ENABLE_EEPROM    
    if (myEEPROM->hasSignature()) {
        debug->println(DEBUG_LEVEL_INFO, "Signature OK");

        this->waitRelay = myEEPROM->readByte(EEPROM_WAIT_RELAY);

    } else {
      debug->println(DEBUG_LEVEL_INFO, "No signature");
      myEEPROM->createSignature();
      setDefaults();
      save();
    }
  #else
    setDefaults();
  #endif
  debug->println(DEBUG_LEVEL_DEBUG, "Done");
}

void Settings::save() {
  
  debug->println(DEBUG_LEVEL_DEBUG, "Saving settings...");
  #ifdef ENABLE_EEPROM
    if (!myEEPROM->hasSignature()) {
      debug->println(DEBUG_LEVEL_DEBUG, "No signature");
      myEEPROM->createSignature();
    }
    debug->println(DEBUG_LEVEL_DEBUG, "Writing data to EEPROM");

    myEEPROM->writeByte(EEPROM_WAIT_RELAY, this->waitRelay);

  #endif
  debug->println(DEBUG_LEVEL_DEBUG, "[OK] Saving settings");
}

void Settings::setDefaults() {
    
    debug->println(DEBUG_LEVEL_INFO, "Setting default values");

    this->waitRelay = 0;

    debug->println(DEBUG_LEVEL_DEBUG, "[OK] Setting default values");
}

void Settings::setWaitRelay(bool isOn) {
    if (isOn) {
      this->waitRelay = 1;
    } else {
      this->waitRelay = 0;
    }
}

bool Settings::IsWaitRelay() {
    return this->waitRelay == 1;
}

