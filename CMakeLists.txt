# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.11)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_options("-Wno-format")

# If you intend for TEST_MODE to be a global compile definition available
# to all components, you can set it here.
# The if(CONFIG_IDF_TARGET) condition is generally always true during an IDF build.
if(CONFIG_IDF_TARGET)
    idf_build_set_property(COMPILE_DEFINITIONS TEST_MODE APPEND)
    # Point LVGL to our lv_conf.h in main/
    idf_build_set_property(COMPILE_DEFINITIONS "LV_CONF_PATH=${CMAKE_CURRENT_SOURCE_DIR}/main/lv_conf.h" APPEND)
endif()

project(RemoteRadioDisplay)
