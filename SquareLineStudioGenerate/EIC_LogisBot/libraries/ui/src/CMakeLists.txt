SET(SOURCES ui_initSCreen.c
    ui_startupScreen.c
    ui_mainScreen.c
    ui_control.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    fonts/ui_font_chinese.c
    fonts/ui_font_taskcode.c
    fonts/ui_font_bigchinese.c)

add_library(ui ${SOURCES})
