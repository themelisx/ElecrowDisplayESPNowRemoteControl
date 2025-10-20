#ifndef UI_SCREEN_h
#define UI_SCREEN_h

#ifdef __cplusplus
extern "C" {
#endif

#include "defines.h"
#include "../lvgl/lvgl.h"

void ui_init_screen(void);
void ui_init_screen_events(void);

lv_obj_t * ui_Main;
#ifdef SHOW_TOP_BAR
lv_obj_t * ui_TopBar;
lv_obj_t * ui_TopBarLeftContainer;
#ifdef USE_MODULE_CONTROLS
    lv_obj_t * ui_ControlsStatus;
#endif
lv_obj_t * ui_TopBarMiddleContainer;
lv_obj_t * ui_TopBarRightContainer;
#ifdef ENABLE_RTC_CLOCK
    lv_obj_t * ui_DateTime;
#endif
#endif

#ifdef USE_MAIN_TAB_VIEW
    lv_obj_t * ui_MainTabView;
#endif

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif