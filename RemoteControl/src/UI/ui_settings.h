#ifndef UI_SETTING_h
#define UI_SETTING_h

    #ifdef __cplusplus
    extern "C" {
    #endif

    #include "defines.h"
    #ifdef USE_MODULE_SETTINGS

        #include "../lvgl/lvgl.h"

        void ui_init_settings(lv_obj_t *parent);

        lv_obj_t * ui_ViewSettings;

            #ifdef USE_MODULE_CONTROLS
            lv_obj_t * ui_WaitRelay;
            #endif
    #endif

    #ifdef __cplusplus
    } /*extern "C"*/
    #endif

#endif