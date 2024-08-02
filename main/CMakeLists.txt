#Add sources from ui directory
file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR} "ui/*.c")

idf_component_register(SRCS "main.c" "ntp_client.c" "wifi_init.c" "lcd_init.c" ${SRC_UI}
                    INCLUDE_DIRS "." "ui")