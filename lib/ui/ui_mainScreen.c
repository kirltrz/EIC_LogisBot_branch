// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 9.1.0
// Project name: EIC_LogisBot

#include "ui.h"

void ui_mainScreen_screen_init(void)
{
    ui_mainScreen = lv_obj_create(NULL);
    lv_obj_remove_flag(ui_mainScreen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel5 = lv_obj_create(ui_mainScreen);
    lv_obj_set_width(ui_Panel5, 308);
    lv_obj_set_height(ui_Panel5, 113);
    lv_obj_set_x(ui_Panel5, 0);
    lv_obj_set_y(ui_Panel5, 60);
    lv_obj_set_align(ui_Panel5, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_Panel5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label6 = lv_label_create(ui_Panel5);
    lv_obj_set_width(ui_Label6, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label6, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label6, 2);
    lv_obj_set_y(ui_Label6, -40);
    lv_obj_set_align(ui_Label6, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_Label6, "任务码");
    lv_obj_set_style_text_font(ui_Label6, &ui_font_chinese, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel6 = lv_obj_create(ui_Panel5);
    lv_obj_set_width(ui_Panel6, 294);
    lv_obj_set_height(ui_Panel6, 79);
    lv_obj_set_x(ui_Panel6, 0);
    lv_obj_set_y(ui_Panel6, 11);
    lv_obj_set_align(ui_Panel6, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_Panel6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_plus = lv_label_create(ui_Panel6);
    lv_obj_set_width(ui_plus, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_plus, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_plus, 0);
    lv_obj_set_y(ui_plus, 4);
    lv_obj_set_align(ui_plus, LV_ALIGN_CENTER);
    lv_label_set_text(ui_plus, "+");
    lv_obj_set_style_text_font(ui_plus, &ui_font_taskcode, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_taskcode2 = lv_label_create(ui_Panel6);
    lv_obj_set_width(ui_taskcode2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_taskcode2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_taskcode2, 80);
    lv_obj_set_y(ui_taskcode2, 4);
    lv_obj_set_align(ui_taskcode2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_taskcode2, "NA");
    lv_obj_set_style_text_font(ui_taskcode2, &ui_font_taskcode, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_taskcode1 = lv_label_create(ui_Panel6);
    lv_obj_set_width(ui_taskcode1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_taskcode1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_taskcode1, -80);
    lv_obj_set_y(ui_taskcode1, 4);
    lv_obj_set_align(ui_taskcode1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_taskcode1, "NA");
    lv_obj_set_style_text_font(ui_taskcode1, &ui_font_taskcode, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button1 = lv_button_create(ui_mainScreen);
    lv_obj_set_width(ui_Button1, 110);
    lv_obj_set_height(ui_Button1, 110);
    lv_obj_set_x(ui_Button1, -98);
    lv_obj_set_y(ui_Button1, -58);
    lv_obj_set_align(ui_Button1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_remove_flag(ui_Button1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button1, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button1, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label9 = lv_label_create(ui_Button1);
    lv_obj_set_width(ui_Label9, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label9, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label9, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label9, "停止");
    lv_obj_set_style_text_font(ui_Label9, &ui_font_bigchinese, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel7 = lv_obj_create(ui_mainScreen);
    lv_obj_set_width(ui_Panel7, 125);
    lv_obj_set_height(ui_Panel7, 110);
    lv_obj_set_x(ui_Panel7, 27);
    lv_obj_set_y(ui_Panel7, -58);
    lv_obj_set_align(ui_Panel7, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_Panel7, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label11 = lv_label_create(ui_Panel7);
    lv_obj_set_width(ui_Label11, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label11, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label11, 0);
    lv_obj_set_y(ui_Label11, -40);
    lv_obj_set_align(ui_Label11, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label11, "调试信息");
    lv_obj_set_style_text_font(ui_Label11, &ui_font_chinese, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel8 = lv_obj_create(ui_Panel7);
    lv_obj_set_width(ui_Panel8, 117);
    lv_obj_set_height(ui_Panel8, 80);
    lv_obj_set_x(ui_Panel8, 0);
    lv_obj_set_y(ui_Panel8, 11);
    lv_obj_set_align(ui_Panel8, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_Panel8, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_debugtext1 = lv_label_create(ui_Panel8);
    lv_obj_set_width(ui_debugtext1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_debugtext1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_debugtext1, 0);
    lv_obj_set_y(ui_debugtext1, -20);
    lv_obj_set_align(ui_debugtext1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_debugtext1, "debugText1");

    ui_debugtext2 = lv_label_create(ui_Panel8);
    lv_obj_set_width(ui_debugtext2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_debugtext2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_debugtext2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_debugtext2, "debugText2");

    ui_debugtext3 = lv_label_create(ui_Panel8);
    lv_obj_set_width(ui_debugtext3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_debugtext3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_debugtext3, 0);
    lv_obj_set_y(ui_debugtext3, 20);
    lv_obj_set_align(ui_debugtext3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_debugtext3, "debugText3");

    ui_Button3 = lv_button_create(ui_mainScreen);
    lv_obj_set_width(ui_Button3, 50);
    lv_obj_set_height(ui_Button3, 50);
    lv_obj_set_x(ui_Button3, 123);
    lv_obj_set_y(ui_Button3, -88);
    lv_obj_set_align(ui_Button3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button3, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_remove_flag(ui_Button3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button3, lv_color_hex(0x0FC121), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label13 = lv_label_create(ui_Button3);
    lv_obj_set_width(ui_Label13, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label13, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label13, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label13, "重新\n开始");
    lv_obj_set_style_text_font(ui_Label13, &ui_font_chinese, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button4 = lv_button_create(ui_mainScreen);
    lv_obj_set_width(ui_Button4, 50);
    lv_obj_set_height(ui_Button4, 50);
    lv_obj_set_x(ui_Button4, 123);
    lv_obj_set_y(ui_Button4, -30);
    lv_obj_set_align(ui_Button4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button4, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_remove_flag(ui_Button4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label12 = lv_label_create(ui_Button4);
    lv_obj_set_width(ui_Label12, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label12, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label12, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label12, "控制");
    lv_obj_set_style_text_font(ui_Label12, &ui_font_chinese, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Button1, ui_event_Button1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button3, ui_event_Button3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button4, ui_event_Button4, LV_EVENT_ALL, NULL);

}
