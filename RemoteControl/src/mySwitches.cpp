#include <Arduino.h>

#include "defines.h"
#include "vars.h"
#include "myWiFi.h"
#include "mySwitches.h"
#include "MyDebug.h"

MySwitches::MySwitches() {
    myDebug->println(DEBUG_LEVEL_DEBUG, "[MySwitches]");
    switchesLastAlive = 0;
    switchesLastSignal = 0;

    semaphoreData = xSemaphoreCreateMutex();
    xSemaphoreGive(semaphoreData);
}

void MySwitches::espNow(uint8_t id, uint16_t value) {
    long now = millis();

    switchesLastSignal = now;
    switchesLastAlive = now;
    #ifdef USE_MULTI_THREAD
      xSemaphoreTake(semaphoreData, portMAX_DELAY);
    #endif
    if (buttons[id].obj != nullptr && buttons[id].state != value) {
      buttons[id].state = value;
      buttons[id].needsUpdate = true;
    } else {
      buttons[id].needsUpdate = false;
    }
    #ifdef USE_MULTI_THREAD
      xSemaphoreGive(semaphoreData);
    #endif
}

void MySwitches::setupButton(int idx, bool enabled, int state, lv_obj_t *obj, uint32_t colorOn, uint32_t colorOff) {
  buttons[idx].enabled = enabled;
  buttons[idx].state = state;
  buttons[idx].needsUpdate = true;
  buttons[idx].obj = obj;
  buttons[idx].colorOn = colorOn;
  buttons[idx].colorOff = colorOff;
}

void MySwitches::updateButtons() {
  for (int i=1; i<=BUTTONS_ON_SCREEN; i++) {
    if (buttons[i].needsUpdate) {
      xSemaphoreTake(semaphoreData, portMAX_DELAY);
      buttons[i].needsUpdate = false;
      xSemaphoreGive(semaphoreData);
      if (buttons[i].state == STATE_OFF) {
        lv_obj_set_style_bg_color(buttons[i].obj, lv_color_hex(buttons[i].colorOff), LV_PART_MAIN | LV_STATE_DEFAULT);
      } else {
        lv_obj_set_style_bg_color(buttons[i].obj, lv_color_hex(buttons[i].colorOn), LV_PART_MAIN | LV_STATE_DEFAULT);
      }
    }    
  }
}

esp_err_t MySwitches::sendToRelay(int btnId, int state) {

  uint8_t bs8_address[6] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6};
  
  myDebug->println(DEBUG_LEVEL_INFO, "Sending esp_now to Relay");  

  if (!mySettings->IsWaitRelay()) {
    #ifdef USE_MULTI_THREAD
      xSemaphoreTake(semaphoreData, portMAX_DELAY);
    #endif
    buttons[btnId].state = state;
    buttons[btnId].needsUpdate = true;
    #ifdef USE_MULTI_THREAD
      xSemaphoreGive(semaphoreData);
    #endif
  }
 
  return myWiFi->sendEspNow(bs8_address, 2, btnId, state);
}

void MySwitches::btnClick(uint8_t btnId) {

  uint8_t state = buttons[btnId].state;

  if (state == STATE_OFF) {
    state = STATE_ON;
  } else {
    state = STATE_OFF;
  }

  #ifndef MODE_RELEASE
    myDebug->println(DEBUG_LEVEL_DEBUG, "Request change for id %d, new state = %d", btnId, state);
  #endif

  #ifndef MODE_RELEASE
    esp_err_t result = sendToRelay(btnId, state);
    if (result == ESP_OK) {
      myDebug->println(DEBUG_LEVEL_DEBUG, " [OK]");
    } else {
      myDebug->println(DEBUG_LEVEL_DEBUG, " [ERROR]");
      myDebug->println(DEBUG_LEVEL_ERROR, esp_err_to_name(result));
    }    
  #else
    sendToRelay(btnId, state);
  #endif
}