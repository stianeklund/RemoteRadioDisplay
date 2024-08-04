#Add sources from ui directory

file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR} "ui/*.c")
file(GLOB_RECURSE SRC_GFX ${CMAKE_SOURCE_DIR} "gfx/*.c")

set(INCLUDE_DIRS
    "."
    "ui"
    "gfx"
    # Add more directories as needed
)

set(SOURCES
    "main.c"
    "ntp_client.c"
    "wifi_init.c"
    "uart.c"
    "cat_parser.c"
    ${SRC_UI}
    ${SRC_GFX}
)

idf_component_register(SRCS ${SOURCES} ${SRC_UI} ${SRC_GFX}
                    INCLUDE_DIRS ${INCLUDE_DIRS})