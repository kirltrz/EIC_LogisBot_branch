// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 9.1.0
// Project name: EIC_LogisBot

#ifndef _EIC_LOGISBOT_UI_H
#define _EIC_LOGISBOT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif

#include "ui_helpers.h"
#include "ui_events.h"

// SCREEN: ui_startupScreen
void ui_startupScreen_screen_init(void);
extern lv_obj_t * ui_startupScreen;
void ui_event_Button2(lv_event_t * e);
extern lv_obj_t * ui_Button2;
extern lv_obj_t * ui_Label10;
void ui_event_Button20(lv_event_t * e);
extern lv_obj_t * ui_Button20;
extern lv_obj_t * ui_Checkbox5;
void ui_event_Button29(lv_event_t * e);
extern lv_obj_t * ui_Button29;
extern lv_obj_t * ui_Label36;
// CUSTOM VARIABLES

// SCREEN: ui_initScreen
void ui_initScreen_screen_init(void);
extern lv_obj_t * ui_initScreen;
extern lv_obj_t * ui_Spinner1;
extern lv_obj_t * ui_Label5;
extern lv_obj_t * ui_Panel1;
extern lv_obj_t * ui_Checkbox4;
extern lv_obj_t * ui_Panel2;
extern lv_obj_t * ui_Checkbox1;
extern lv_obj_t * ui_Panel3;
extern lv_obj_t * ui_Checkbox2;
extern lv_obj_t * ui_Panel4;
extern lv_obj_t * ui_Checkbox3;
void ui_event_Button16(lv_event_t * e);
extern lv_obj_t * ui_Button16;
extern lv_obj_t * ui_Label32;
// CUSTOM VARIABLES

// SCREEN: ui_mainScreen
void ui_mainScreen_screen_init(void);
extern lv_obj_t * ui_mainScreen;
extern lv_obj_t * ui_Panel5;
extern lv_obj_t * ui_Label6;
extern lv_obj_t * ui_Panel6;
extern lv_obj_t * ui_plus;
extern lv_obj_t * ui_taskcode2;
extern lv_obj_t * ui_taskcode1;
void ui_event_Button1(lv_event_t * e);
extern lv_obj_t * ui_Button1;
extern lv_obj_t * ui_Label9;
extern lv_obj_t * ui_Panel7;
extern lv_obj_t * ui_Label11;
extern lv_obj_t * ui_Panel8;
extern lv_obj_t * ui_debugtext1;
extern lv_obj_t * ui_debugtext2;
extern lv_obj_t * ui_debugtext3;
void ui_event_Button3(lv_event_t * e);
extern lv_obj_t * ui_Button3;
extern lv_obj_t * ui_Label13;
void ui_event_Button4(lv_event_t * e);
extern lv_obj_t * ui_Button4;
extern lv_obj_t * ui_Label12;
// CUSTOM VARIABLES

// SCREEN: ui_controlScreen
void ui_controlScreen_screen_init(void);
extern lv_obj_t * ui_controlScreen;
void ui_event_Button5(lv_event_t * e);
extern lv_obj_t * ui_Button5;
extern lv_obj_t * ui_Label17;
extern lv_obj_t * ui_Slider1;
extern lv_obj_t * ui_TabView2;
extern lv_obj_t * ui_otherPage;
extern lv_obj_t * ui_Panel17;
void ui_event_Switch2(lv_event_t * e);
extern lv_obj_t * ui_Switch2;
extern lv_obj_t * ui_Label46;
void ui_event_Button34(lv_event_t * e);
extern lv_obj_t * ui_Button34;
extern lv_obj_t * ui_Label47;
void ui_event_Button35(lv_event_t * e);
extern lv_obj_t * ui_Button35;
extern lv_obj_t * ui_Label48;
extern lv_obj_t * ui_Panel18;
extern lv_obj_t * ui_Label49;
extern lv_obj_t * ui_Label50;
extern lv_obj_t * ui_xtext;
extern lv_obj_t * ui_ytext;
extern lv_obj_t * ui_Label53;
extern lv_obj_t * ui_dxtext;
extern lv_obj_t * ui_Label52;
extern lv_obj_t * ui_dytext;
extern lv_obj_t * ui_rpiPage;
extern lv_obj_t * ui_Panel9;
extern lv_obj_t * ui_Label37;
extern lv_obj_t * ui_Label38;
extern lv_obj_t * ui_Label39;
extern lv_obj_t * ui_Label40;
extern lv_obj_t * ui_modeText;
extern lv_obj_t * ui_data1Text;
extern lv_obj_t * ui_data2Text;
extern lv_obj_t * ui_colorText;
void ui_event_Button30(lv_event_t * e);
extern lv_obj_t * ui_Button30;
extern lv_obj_t * ui_Label41;
void ui_event_Button31(lv_event_t * e);
extern lv_obj_t * ui_Button31;
extern lv_obj_t * ui_Label42;
void ui_event_Button32(lv_event_t * e);
extern lv_obj_t * ui_Button32;
extern lv_obj_t * ui_Label43;
void ui_event_Button33(lv_event_t * e);
extern lv_obj_t * ui_Button33;
extern lv_obj_t * ui_Label44;
extern lv_obj_t * ui_motorPage;
extern lv_obj_t * ui_Label1;
void ui_event_univx(lv_event_t * e);
extern lv_obj_t * ui_univx;
extern lv_obj_t * ui_Label3;
void ui_event_univy(lv_event_t * e);
extern lv_obj_t * ui_univy;
extern lv_obj_t * ui_Label4;
void ui_event_uniomega(lv_event_t * e);
extern lv_obj_t * ui_uniomega;
void ui_event_Button8(lv_event_t * e);
extern lv_obj_t * ui_Button8;
extern lv_obj_t * ui_Label7;
void ui_event_Button10(lv_event_t * e);
extern lv_obj_t * ui_Button10;
extern lv_obj_t * ui_Label8;
void ui_event_Button11(lv_event_t * e);
extern lv_obj_t * ui_Button11;
extern lv_obj_t * ui_Label14;
extern lv_obj_t * ui_servoPage;
extern lv_obj_t * ui_Panel11;
void ui_event_servo0Arc(lv_event_t * e);
extern lv_obj_t * ui_servo0Arc;
extern lv_obj_t * ui_servo0Text;
void ui_event_Button18(lv_event_t * e);
extern lv_obj_t * ui_Button18;
extern lv_obj_t * ui_Label33;
void ui_event_Button19(lv_event_t * e);
extern lv_obj_t * ui_Button19;
extern lv_obj_t * ui_Label35;
void ui_event_Button28(lv_event_t * e);
extern lv_obj_t * ui_Button28;
extern lv_obj_t * ui_Label21;
extern lv_obj_t * ui_temp0;
extern lv_obj_t * ui_Panel10;
extern lv_obj_t * ui_Label23;
extern lv_obj_t * ui_servo1Text;
extern lv_obj_t * ui_temp1;
extern lv_obj_t * ui_Panel12;
extern lv_obj_t * ui_Label16;
extern lv_obj_t * ui_servo2Text;
extern lv_obj_t * ui_temp2;
extern lv_obj_t * ui_Panel13;
extern lv_obj_t * ui_Label25;
extern lv_obj_t * ui_servo3Text;
extern lv_obj_t * ui_temp3;
extern lv_obj_t * ui_Panel14;
void ui_event_Button14(lv_event_t * e);
extern lv_obj_t * ui_Button14;
extern lv_obj_t * ui_Label28;
void ui_event_Button15(lv_event_t * e);
extern lv_obj_t * ui_Button15;
extern lv_obj_t * ui_Label29;
extern lv_obj_t * ui_Label30;
extern lv_obj_t * ui_servoClawText;
extern lv_obj_t * ui_Panel16;
extern lv_obj_t * ui_Label27;
extern lv_obj_t * ui_actionGroupList;
void ui_event_Button27(lv_event_t * e);
extern lv_obj_t * ui_Button27;
extern lv_obj_t * ui_Label31;
extern lv_obj_t * ui_hwt101Page;
extern lv_obj_t * ui_Panel15;
extern lv_obj_t * ui_hwt101data;
void ui_event_Button17(lv_event_t * e);
extern lv_obj_t * ui_Button17;
extern lv_obj_t * ui_Label34;
extern lv_obj_t * ui_Label45;
void ui_event_Switch1(lv_event_t * e);
extern lv_obj_t * ui_Switch1;
void ui_event_Button6(lv_event_t * e);
extern lv_obj_t * ui_Button6;
extern lv_obj_t * ui_Label18;
void ui_event_Button7(lv_event_t * e);
extern lv_obj_t * ui_Button7;
extern lv_obj_t * ui_Label19;
void ui_event_Button9(lv_event_t * e);
extern lv_obj_t * ui_Button9;
extern lv_obj_t * ui_Label2;
// CUSTOM VARIABLES

// EVENTS

extern lv_obj_t * ui____initial_actions0;

// FONTS
LV_FONT_DECLARE(ui_font_bigchinese);
LV_FONT_DECLARE(ui_font_chinese);
LV_FONT_DECLARE(ui_font_taskcode);

// UI INIT
void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
