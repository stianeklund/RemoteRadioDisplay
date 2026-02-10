# Add sources from ui, gfx, and radio subdirectories within the 'main' component.
# C++ sources for UI (excluding fonts which are auto-generated C)
file(GLOB_RECURSE SRC_UI_CPP "ui/*.cpp")
# Auto-generated font files
file(GLOB_RECURSE SRC_UI_FONTS "ui/fonts/*.c")
# P4 scaled font variants (explicit list so cmake picks them up without reconfigure)
set(P4_FONTS
    "ui/fonts/ui_font_FrequencyFont_P4.c"
    "ui/fonts/ui_font_FrequencyFontSmall_P4.c"
    "ui/fonts/ui_font_Glyphs70_P4.c"
    "ui/fonts/ui_font_Font16_P4.c"
    "ui/fonts/ui_font_Font18_P4.c"
    "ui/fonts/ui_font_Font30_P4.c"
    "ui/fonts/ui_font_Font48_P4.c"
    "ui/fonts/ui_font_Frequency_P4.c"
    "ui/fonts/ui_font_ButtonFont_P4.c"
    "ui/fonts/ui_font_ButtonFontLargeBold_P4.c"
    "ui/fonts/ui_font_ButtonFontSmallMedium_P4.c"
    "ui/fonts/ui_font_ButtonSmallRegular_P4.c"
    "ui/fonts/ui_font_MeterFont_P4.c"
    "ui/fonts/ui_font_UTCTime_P4.c"
    "ui/fonts/ui_font_Glyphs24_P4.c"
    "ui/fonts/ui_font_Glyphs30_P4.c"
    "ui/fonts/ui_font_Glyphs58_P4.c"
)
list(APPEND SRC_UI_FONTS ${P4_FONTS})
list(REMOVE_DUPLICATES SRC_UI_FONTS)
# Graphics subsystem (C++)
file(GLOB_RECURSE SRC_GFX "gfx/*.cpp")
# Radio subjects and state management (C++)
file(GLOB_RECURSE SRC_RADIO "radio/*.cpp")

# Define all source files for the main component.
# Core application sources (C++)
set(SOURCES
    "main.cpp"
    "ntp_client.cpp"
    "wifi_init.cpp"
    "uart.cpp"
    "cat_parser.cpp"
    "cat_polling.cpp"
    "cat_state.cpp"
    "screensaver.cpp"
    "settings_storage.cpp"
    "gps_client.cpp"
    "antenna_control.cpp"
    "websocket_client.cpp"
    "memory_monitor.cpp"
    ${SRC_UI_CPP}
    ${SRC_UI_FONTS}
    ${SRC_GFX}
    ${SRC_RADIO}
)

# Define include directories for the main component.
# These paths are relative to this CMakeLists.txt file.
set(INCLUDE_DIRS
    "."     # For headers in 'main/' (e.g., cat_parser.h, ntp_client.h)
    "ui"    # For headers in 'main/ui/' (e.g., ui.h if it's there)
    "gfx"   # For headers in 'main/gfx/' (e.g., lcd_init.h)
    "radio" # For headers in 'main/radio/' (e.g., radio_subjects.h)
    # Add more component-local include directories as needed
)

# List the components that 'main' depends on.
# These are typically IDF components or other components in your project (like managed components).
set(COMPONENT_REQUIRES
    "esp_system"
    "nvs_flash"
    "esp_wifi"
    "esp_event"
    "freertos"
    "esp_timer"
    "esp_hw_support"
    "mdns"
    "json"
    "esp_lcd"
    "espressif__esp_lcd_touch"
    "lvgl__lvgl"
    "espressif__esp_websocket_client"
)

# Target-specific dependencies
if(CONFIG_IDF_TARGET_ESP32P4)
    list(APPEND COMPONENT_REQUIRES
        "waveshare__esp_lcd_hx8394"
    )
endif()

# Add test sources and their include directory.
# The TEST_MODE definition is set in the root CMakeLists.txt.
# These paths point to the 'test' directory at the project root.
#list(APPEND SOURCES
#    "${CMAKE_PROJECT_DIR}/test/test_cat_parser.c"
#    "${CMAKE_PROJECT_DIR}/test/test_runner.c"
#)
# Add the project's 'test' directory to the include paths for the main component
# when compiling its sources (including the test sources).
list(APPEND INCLUDE_DIRS
    "${CMAKE_PROJECT_DIR}../test"
)
# Add 'unity' as a requirement for the main component because it now includes unit tests.
list(APPEND COMPONENT_REQUIRES
    "unity"
)

# Register the main component with the ESP-IDF build system.
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS ${INCLUDE_DIRS}
                    REQUIRES ${COMPONENT_REQUIRES})
