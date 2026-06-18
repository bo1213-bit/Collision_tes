cmake_minimum_required(VERSION 3.14)
project(encoder_reader CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

find_package(Threads REQUIRED)

# --- library ---
add_library(encoder_lib
    hal/ModbusRTU.cpp
    hal/SerialPort.cpp
    hal/CustomProtocol.cpp
    driver/EncoderDriver.cpp
    driver/DeviceDriver.cpp
    Encoder.cpp
    Sensor.cpp
)
target_include_directories(encoder_lib
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE ${MODBUS_INCLUDE_DIRS}
)
target_link_libraries(encoder_lib PRIVATE ${MODBUS_LIBRARIES})

# --- example executables ---
add_executable(encoder_example example/main.cpp)
target_link_libraries(encoder_example PRIVATE encoder_lib Threads::Threads)

add_executable(encoder_print example/main_enc.cpp)
target_link_libraries(encoder_print PRIVATE encoder_lib Threads::Threads)

add_executable(check_ch_enable ${CMAKE_SOURCE_DIR}/../check_ch_enable.cpp)
target_link_libraries(check_ch_enable PRIVATE encoder_lib Threads::Threads)
