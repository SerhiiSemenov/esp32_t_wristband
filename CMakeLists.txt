# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS
           externals/ESP32_TFT_library/components/spidriver
           externals/ESP32_TFT_library/components/tft
           externals/ESP32_TFT_library/components/spiffs
           externals/esp32-PCF8563-master
           externals/esp32_serial_bus
           externals/mpu9250)
project(T-Wristband demo)
