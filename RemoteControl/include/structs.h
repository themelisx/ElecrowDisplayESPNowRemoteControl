#ifndef structs_h
#define structs_h

#include <Arduino.h>
#include "../lvgl/lvgl.h"

typedef struct s_espNowButtons {
  bool enabled;
  bool needsUpdate;
  int16_t state;
  lv_obj_t *obj;
  uint32_t colorOn;
  uint32_t colorOff;
  lv_obj_t *uiLabel;
  char *uiLabelText;
} s_espNowButtons;

#endif