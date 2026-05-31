SET(SOURCES ui_StartupScreen.c
    ui_MainScreen.c
    ui_SettingsScreen.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_events.cpp
    ui_img_logopng_png.c
    fonts/ui_font_Archimoto_EXTRA.c
    fonts/ui_font_Archimoto_Labels.c
    fonts/ui_font_Archimoto_Numbers.c)

add_library(ui ${SOURCES})
