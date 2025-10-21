#include <Arduino.h>

#include "defines.h"

#ifdef USE_MULTI_THREAD
  #ifdef DISPLAY_AT_CORE1
  #include "debug.h"
  #include "vars.h"

  void tft_task(void *pvParameters) {
    debug->print(DEBUG_LEVEL_INFO, "View manager TFT: Task running on core ");
    debug->println(DEBUG_LEVEL_INFO, xPortGetCoreID());

    for (;;) {

      #ifdef USE_MODULE_CONTROLS
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
      #endif

      vTaskDelay(DELAY_REFRESH_VIEW / portTICK_PERIOD_MS);

    }
  }
  #endif
#endif