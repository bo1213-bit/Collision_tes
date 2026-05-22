cmake_minimum_required(VERSION 3.14)
project(encoder_reader CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

# --- library ---
add_library(encoder_lib
    hal/ModbusRTU.cpp
    driver/EncoderDriver.cpp
    Encoder.cpp
)
target_include_directories(encoder_lib
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE ${MODBUS_INCLUDE_DIRS}
)
target_link_libraries(encoder_lib PRIVATE ${MODBUS_LIBRARIES})

# --- example executable ---
add_executable(encoder_example example/main.cpp)
target_link_libraries(encoder_example PRIVATE encoder_lib)
