FILE(GLOB SOURCES *.cpp *.c)

idf_component_register(SRCS ${SOURCES}
    INCLUDE_DIRS
    "."
    "../externals/ESP32_TFT_library/components"
    "../externals/esp32-PCF8563-master"
    "../externals/esp32_serial_bus"
    "../externals/mpu9250/include"
    ${CMAKE_CURRENT_LIST_DIR}
    $ENV{IDF_PATH}/components
#    REQUIRES ""tft"" ""spiffs""
    )

