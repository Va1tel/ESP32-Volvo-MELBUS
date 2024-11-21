# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
SET(IDF_PATH C:/Users/Vladimir/esp/v5.1.5/esp-idf)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(project-name1234)
SET(CMAKE_TOOLCHAIN_FILE ${IDF_PATH}/tools/cmake/toolchain-esp32.cmake)
set(EXTRA_COMPONENT_DIRS components/arduino-audio-tools)
