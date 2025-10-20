#ifndef VARS_h
#define VARS_h

#include <Arduino.h>

#include "debug.h"
#include "structs.h"
#include "settings.h"

#ifdef USE_MULTI_THREAD
    extern SemaphoreHandle_t semaphoreData;

    // Tasks
    #ifdef DISPLAY_AT_CORE1
        extern TaskHandle_t t_core1_tft;
    #endif

    #ifdef USE_MAIN_TAB_VIEW
        extern lv_obj_t * ui_MainTabView;
    #endif
#endif

extern Debug *debug;
extern Settings *mySettings;

#endif