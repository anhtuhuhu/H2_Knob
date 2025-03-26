SET(SOURCES screens/ui_Screen1.c
    SOURCES screens/ui_Screen2.c
    SOURCES screens/ui_test.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    )
add_library(ui ${SOURCES})
