#include <Arduino.h>

#include "defines.h"

#ifdef USE_MULTI_THREAD
  #ifdef DISPLAY_AT_CORE1
  #include "MyDebug.h"
  #include "vars.h"

  void tft_task(void *pvParameters) {
    myDebug->println(DEBUG_LEVEL_INFO, "View manager TFT: Task running on core %d", xPortGetCoreID()));

    for (;;) {

      #ifdef USE_MODULE_SWITCHES
      mySwitches->updateButtons();      
      #endif

      vTaskDelay(DELAY_REFRESH_VIEW / portTICK_PERIOD_MS);

    }
  }
  #endif
#endif