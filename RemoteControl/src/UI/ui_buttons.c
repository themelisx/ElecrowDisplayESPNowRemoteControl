#include "defines.h"
#include "ui_buttons.h"
#include "ui.h"

#ifdef USE_MODULE_CONTROLS

void ui_event_Panel(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if (target == ui_Panel1) {
        onMainBar(e);
    } else if (target == ui_Panel2) {
        onRoofBar(e);
    } else if (target == ui_Panel3) {
        onRoofLeds(e);
    } else if (target == ui_Panel4) {
        onLeftLeds(e);
    } else if (target == ui_Panel5) {
        onRightLeds(e);
    } else if (target == ui_Panel6) {
        onRearLeds(e);
    } else if (target == ui_Panel7) {
        onStrobeLights(e);
    } /*else if (target == ui_Panel8) {
        
    }*/
}


void ui_init_buttons(lv_obj_t *parent) {

    #ifdef USE_MAIN_TAB_VIEW
        #ifdef SHOW_TABS_AT_LEFT
            ui_ViewControls = lv_tabview_add_tab(parent, " ");
        #else
            #if defined(LANG_EN)
            ui_ViewControls = lv_tabview_add_tab(parent, "Controls");
            #elif defined(LANG_GR)
            ui_ViewControls = lv_tabview_add_tab(parent, "Διακόπτες");
            #endif
        #endif
    #else
        ui_ViewControls = lv_obj_create(parent);
        lv_obj_remove_style_all(ui_ViewControls);
        lv_obj_set_width(ui_ViewControls, lv_pct(100));
        #ifdef SHOW_TOP_BAR
            lv_obj_set_height(ui_ViewControls, lv_pct(90));
        #else
            lv_obj_set_height(ui_ViewControls, lv_pct(100));
        #endif
        lv_obj_set_x(ui_ViewControls, 0);
        lv_obj_set_y(ui_ViewControls, 50);
        lv_obj_set_align(ui_ViewControls, LV_ALIGN_CENTER);
        lv_obj_set_style_bg_color(ui_ViewControls, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(ui_ViewControls, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    #endif
    lv_obj_set_flex_flow(ui_ViewControls, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_ViewControls, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_AROUND);
    lv_obj_clear_flag(ui_ViewControls, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                    LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_obj_set_scrollbar_mode(ui_ViewControls, LV_SCROLLBAR_MODE_OFF);
        
    ui_ContainerControls1 = lv_obj_create(ui_ViewControls);
    
    lv_obj_remove_style_all(ui_ContainerControls1);
    lv_obj_set_width(ui_ContainerControls1, lv_pct(100));
    lv_obj_set_height(ui_ContainerControls1, lv_pct(40));
    lv_obj_set_align(ui_ContainerControls1, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_ContainerControls1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_ContainerControls1, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(ui_ContainerControls1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel1 = lv_obj_create(ui_ContainerControls1);
    lv_obj_set_width(ui_Panel1, lv_pct(30));
    lv_obj_set_height(ui_Panel1, lv_pct(100));
    lv_obj_set_x(ui_Panel1, -240);
    lv_obj_set_y(ui_Panel1, -120);
    lv_obj_set_align(ui_Panel1, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Panel1, lv_color_hex(0x404040), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ControlLabel1 = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_ControlLabel1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ControlLabel1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ControlLabel1, 0);
    lv_obj_set_y(ui_ControlLabel1, 50);
    lv_obj_set_align(ui_ControlLabel1, LV_ALIGN_CENTER);
    #if defined(LANG_EN)
    lv_label_set_text(ui_ControlLabel1, "Main LED Bar");
    #elif defined(LANG_GR)
    lv_label_set_text(ui_ControlLabel1, "Μεγάλη μπάρα");
    #endif    
    lv_obj_set_style_text_color(ui_ControlLabel1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_ControlLabel1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_ControlLabel1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_ControlLabel1, &lv_font_arial_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ControlImage1 = lv_img_create(ui_Panel1);
    lv_img_set_src(ui_ControlImage1, &ui_img_b_led_bar_png);
    lv_obj_set_width(ui_ControlImage1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ControlImage1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ControlImage1, 0);
    lv_obj_set_y(ui_ControlImage1, -15);
    lv_obj_set_align(ui_ControlImage1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ControlImage1, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ControlImage1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel2 = lv_obj_create(ui_ContainerControls1);
    lv_obj_set_width(ui_Panel2, lv_pct(30));
    lv_obj_set_height(ui_Panel2, lv_pct(100));
    lv_obj_set_x(ui_Panel2, 0);
    lv_obj_set_y(ui_Panel2, -120);
    lv_obj_set_align(ui_Panel2, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Panel2, lv_color_hex(0x404040), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ControlLabel2 = lv_label_create(ui_Panel2);
    lv_obj_set_width(ui_ControlLabel2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ControlLabel2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ControlLabel2, 0);
    lv_obj_set_y(ui_ControlLabel2, 50);
    lv_obj_set_align(ui_ControlLabel2, LV_ALIGN_CENTER);    
    #if defined(LANG_EN)
    lv_label_set_text(ui_ControlLabel2, "Roof LED Bar");
    #elif defined(LANG_GR)
    lv_label_set_text(ui_ControlLabel2, "Μπάρα οροφής");
    #endif
    lv_obj_set_style_text_color(ui_ControlLabel2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_ControlLabel2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_ControlLabel2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_ControlLabel2, &lv_font_arial_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ControlImage2 = lv_img_create(ui_Panel2);
    lv_img_set_src(ui_ControlImage2, &ui_img_b_slim_led_bar_png);
    lv_obj_set_width(ui_ControlImage2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ControlImage2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ControlImage2, 0);
    lv_obj_set_y(ui_ControlImage2, -15);
    lv_obj_set_align(ui_ControlImage2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ControlImage2, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ControlImage2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel3 = lv_obj_create(ui_ContainerControls1);
    lv_obj_set_width(ui_Panel3, lv_pct(30));
    lv_obj_set_height(ui_Panel3, lv_pct(100));
    lv_obj_set_x(ui_Panel3, 240);
    lv_obj_set_y(ui_Panel3, -120);
    lv_obj_set_align(ui_Panel3, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Panel3, lv_color_hex(0x404040), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ControlLabel3 = lv_label_create(ui_Panel3);
    lv_obj_set_width(ui_ControlLabel3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ControlLabel3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ControlLabel3, 0);
    lv_obj_set_y(ui_ControlLabel3, 50);
    lv_obj_set_align(ui_ControlLabel3, LV_ALIGN_CENTER);    
    #if defined(LANG_EN)
    lv_label_set_text(ui_ControlLabel3, "Roof Lights");
    #elif defined(LANG_GR)
    lv_label_set_text(ui_ControlLabel3, "Φώτα οροφής");
    #endif
    lv_obj_set_style_text_color(ui_ControlLabel3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_ControlLabel3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_ControlLabel3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_ControlLabel3, &lv_font_arial_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ControlImage3 = lv_img_create(ui_Panel3);
    lv_img_set_src(ui_ControlImage3, &ui_img_b_led_lights_png);
    lv_obj_set_width(ui_ControlImage3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ControlImage3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ControlImage3, 0);
    lv_obj_set_y(ui_ControlImage3, -15);
    lv_obj_set_align(ui_ControlImage3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ControlImage3, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ControlImage3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_ContainerControls2 = lv_obj_create(ui_ViewControls);
    lv_obj_remove_style_all(ui_ContainerControls2);
    lv_obj_set_width(ui_ContainerControls2, lv_pct(100));
    lv_obj_set_height(ui_ContainerControls2, lv_pct(40));
    lv_obj_set_align(ui_ContainerControls2, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_ContainerControls2, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_ContainerControls2, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_SPACE_BETWEEN);
    lv_obj_clear_flag(ui_ContainerControls2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel4 = lv_obj_create(ui_ContainerControls2);
    lv_obj_set_width(ui_Panel4, lv_pct(30));
    lv_obj_set_height(ui_Panel4, lv_pct(100));
    lv_obj_set_x(ui_Panel4, -240);
    lv_obj_set_y(ui_Panel4, 50);
    lv_obj_set_align(ui_Panel4, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Panel4, lv_color_hex(0x404040), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ControlLabel4 = lv_label_create(ui_Panel4);
    lv_obj_set_width(ui_ControlLabel4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ControlLabel4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ControlLabel4, 0);
    lv_obj_set_y(ui_ControlLabel4, 50);
    lv_obj_set_align(ui_ControlLabel4, LV_ALIGN_CENTER);    
    #if defined(LANG_EN)
    lv_label_set_text(ui_ControlLabel4, "Left Lights");
    #elif defined(LANG_GR)
    lv_label_set_text(ui_ControlLabel4, "Φώτα αριστερά");
    #endif
    lv_obj_set_style_text_color(ui_ControlLabel4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_ControlLabel4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_ControlLabel4, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_ControlLabel4, &lv_font_arial_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ControlImage4 = lv_img_create(ui_Panel4);
    lv_img_set_src(ui_ControlImage4, &ui_img_b_lights_left_png);
    lv_obj_set_width(ui_ControlImage4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ControlImage4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ControlImage4, 0);
    lv_obj_set_y(ui_ControlImage4, -15);
    lv_obj_set_align(ui_ControlImage4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ControlImage4, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ControlImage4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel5 = lv_obj_create(ui_ContainerControls2);
    lv_obj_set_width(ui_Panel5, lv_pct(30));
    lv_obj_set_height(ui_Panel5, lv_pct(100));
    lv_obj_set_x(ui_Panel5, 0);
    lv_obj_set_y(ui_Panel5, 50);
    lv_obj_set_align(ui_Panel5, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Panel5, lv_color_hex(0x404040), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ControlLabel5 = lv_label_create(ui_Panel5);
    lv_obj_set_width(ui_ControlLabel5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ControlLabel5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ControlLabel5, 0);
    lv_obj_set_y(ui_ControlLabel5, 50);
    lv_obj_set_align(ui_ControlLabel5, LV_ALIGN_CENTER);    
    #if defined(LANG_EN)
    lv_label_set_text(ui_ControlLabel5, "Right Lights");
    #elif defined(LANG_GR)
    lv_label_set_text(ui_ControlLabel5, "Φώτα δεξιά");
    #endif
    lv_obj_set_style_text_color(ui_ControlLabel5, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_ControlLabel5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_ControlLabel5, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_ControlLabel5, &lv_font_arial_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ControlImage5 = lv_img_create(ui_Panel5);
    lv_img_set_src(ui_ControlImage5, &ui_img_b_lights_right_png);
    lv_obj_set_width(ui_ControlImage5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ControlImage5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ControlImage5, 0);
    lv_obj_set_y(ui_ControlImage5, -15);
    lv_obj_set_align(ui_ControlImage5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ControlImage5, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ControlImage5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel6 = lv_obj_create(ui_ContainerControls2);
    lv_obj_set_width(ui_Panel6, lv_pct(30));
    lv_obj_set_height(ui_Panel6, lv_pct(100));
    lv_obj_set_x(ui_Panel6, 240);
    lv_obj_set_y(ui_Panel6, 50);
    lv_obj_set_align(ui_Panel6, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Panel6, lv_color_hex(0x404040), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ControlLabel6 = lv_label_create(ui_Panel6);
    lv_obj_set_width(ui_ControlLabel6, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ControlLabel6, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ControlLabel6, 0);
    lv_obj_set_y(ui_ControlLabel6, 50);
    lv_obj_set_align(ui_ControlLabel6, LV_ALIGN_CENTER);    
    #if defined(LANG_EN)
    lv_label_set_text(ui_ControlLabel6, "Rear Lights");
    #elif defined(LANG_GR)
    lv_label_set_text(ui_ControlLabel6, "Φώτα πίσω");
    #endif
    lv_obj_set_style_text_color(ui_ControlLabel6, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_ControlLabel6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_ControlLabel6, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_ControlLabel6, &lv_font_arial_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ControlImage6 = lv_img_create(ui_Panel6);
    lv_img_set_src(ui_ControlImage6, &ui_img_b_rear_lights_png);
    lv_obj_set_width(ui_ControlImage6, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ControlImage6, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ControlImage6, 0);
    lv_obj_set_y(ui_ControlImage6, -15);
    lv_obj_set_align(ui_ControlImage6, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ControlImage6, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ControlImage6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_ContainerControls3 = lv_obj_create(ui_ViewControls);
    lv_obj_remove_style_all(ui_ContainerControls3);
    lv_obj_set_width(ui_ContainerControls3, lv_pct(100));
    lv_obj_set_height(ui_ContainerControls3, lv_pct(15));
    lv_obj_set_align(ui_ContainerControls3, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_ContainerControls3, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_ContainerControls3, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_SPACE_BETWEEN);
    lv_obj_clear_flag(ui_ContainerControls3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel7 = lv_obj_create(ui_ContainerControls3);
    lv_obj_set_width(ui_Panel7, lv_pct(97));
    lv_obj_set_height(ui_Panel7, lv_pct(100));
    lv_obj_set_x(ui_Panel7, 0);
    lv_obj_set_y(ui_Panel7, 170);
    lv_obj_set_align(ui_Panel7, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel7, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Panel7, lv_color_hex(0x404040), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ControlLabel7 = lv_label_create(ui_Panel7);
    lv_obj_set_width(ui_ControlLabel7, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ControlLabel7, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_ControlLabel7, LV_ALIGN_CENTER);    
    #if defined(LANG_EN)
    lv_label_set_text(ui_ControlLabel7, "Emergency Strobe Lights");
    #elif defined(LANG_GR)
    lv_label_set_text(ui_ControlLabel7, "Φώτα έκτακτης ανάγκης");
    #endif
    lv_obj_set_style_text_color(ui_ControlLabel7, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_ControlLabel7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_ControlLabel7, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_ControlLabel7, &lv_font_arial_24, LV_PART_MAIN | LV_STATE_DEFAULT);

}

#endif