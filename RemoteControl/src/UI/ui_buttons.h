#ifndef UI_BUTTONS_h
#define UI_BUTTONS_h

#include "defines.h"
#ifdef USE_MODULE_CONTROLS

#ifdef __cplusplus
extern "C" {
#endif

#include "../lvgl/lvgl.h"

void ui_init_buttons(lv_obj_t *parent);

lv_obj_t * ui_ViewControls;
lv_obj_t * ui_ContainerControls1;
lv_obj_t * ui_Panel1;
lv_obj_t * ui_ControlLabel1;
lv_obj_t * ui_ControlImage1;
lv_obj_t * ui_Panel2;
lv_obj_t * ui_ControlLabel2;
lv_obj_t * ui_ControlImage2;
lv_obj_t * ui_Panel3;
lv_obj_t * ui_ControlLabel3;
lv_obj_t * ui_ControlImage3;
lv_obj_t * ui_ContainerControls2;
lv_obj_t * ui_Panel4;
lv_obj_t * ui_ControlLabel4;
lv_obj_t * ui_ControlImage4;
lv_obj_t * ui_Panel5;
lv_obj_t * ui_ControlLabel5;
lv_obj_t * ui_ControlImage5;
lv_obj_t * ui_Panel6;
lv_obj_t * ui_ControlLabel6;
lv_obj_t * ui_ControlImage6;
lv_obj_t * ui_ContainerControls3;
lv_obj_t * ui_Panel7;
lv_obj_t * ui_ControlLabel7;

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif

#endif