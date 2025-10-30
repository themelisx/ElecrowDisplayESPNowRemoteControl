#ifndef SWITCHES_h
#define SWITCHES_h

#include <Arduino.h>

#include "defines.h"

class MySwitches {
  private:
    s_espNowButtons buttons[10];
    esp_err_t sendToRelay(int btnId, int state);
    
    long switchesLastAlive;
    long switchesLastSignal;

    SemaphoreHandle_t semaphoreData;

  public:
    MySwitches();
    void espNow(uint8_t id, uint16_t value);
    void btnClick(uint8_t btnId);
    void setupButton(int idx, bool enabled, int state, lv_obj_t *obj, uint32_t colorOn, uint32_t colorOff);
    void updateButtons();

};

#endif