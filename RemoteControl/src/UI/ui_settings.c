#include "defines.h"

#ifdef USE_MODULE_SETTINGS
#include "ui_settings.h"
#include "../fonts/fonts.h"
#include "../../include/configuration.h"

void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);

    if(code == LV_EVENT_VALUE_CHANGED) {
        #ifdef USE_MODULE_CONTROLS
            if (target == ui_WaitRelay) {
                onWaitRelayPressed(lv_obj_get_state(target) & LV_STATE_CHECKED);
            }
        #endif
    }
}

void ui_init_settings(lv_obj_t *parent) {
    #ifdef SHOW_TABS_AT_LEFT
    ui_ViewSettings = lv_tabview_add_tab(parent, " ");
    #else
        #if defined(LANG_EN)
        ui_ViewSettings = lv_tabview_add_tab(parent, "Settings");
        #elif defined(LANG_GR)
        ui_ViewSettings = lv_tabview_add_tab(parent, "Ρυθμίσεις");
        #endif
    
    #endif
    lv_obj_clear_flag(ui_ViewSettings, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                      LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_obj_set_scrollbar_mode(ui_ViewSettings, LV_SCROLLBAR_MODE_OFF);

    

#ifdef USE_MODULE_CONTROLS
    ui_WaitRelay = lv_checkbox_create(ui_ViewSettings);
    #if defined(LANG_EN)
    lv_checkbox_set_text(ui_WaitRelay, "Verify relay answer");
    #elif defined(LANG_GR)
    lv_checkbox_set_text(ui_WaitRelay, "Επιβεβαίωση λήψης από το ρελέ");
    #endif    
    lv_obj_set_width(ui_WaitRelay, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_WaitRelay, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_WaitRelay, -200);
    lv_obj_set_y(ui_WaitRelay, -170);
    lv_obj_set_align(ui_WaitRelay, LV_ALIGN_CENTER);
    lv_obj_set_style_text_font(ui_WaitRelay, &lv_font_arial_20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_flag(ui_WaitRelay, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_set_style_text_color(ui_WaitRelay, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_WaitRelay, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_WaitRelay, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_state(ui_WaitRelay, LV_STATE_DEFAULT);    
#endif

}
#endif