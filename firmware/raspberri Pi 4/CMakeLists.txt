cmake_minimum_required(VERSION 3.0)
project(RaspberryROVController)

add_executable(myapp src/main.cpp)

add_library(CA src/control_allocation.cpp src/control_allocation.h)
add_library(JM src/joystick_mapper.cpp  src/joystick_mapper.h)
add_library(PPC src/PolePlacementControl.cpp  src/PolePlacementControl.h)
add_library(RC src/reference_change.cpp  src/reference_change.h)
add_library(IMU src/IMU/WIT61P.c  src/IMU/WIT61P.h)
add_library(IMU_i2c src/IMU/i2c.c  src/IMU/i2c.h)
add_library(IMU_wit src/IMU/wit_c_sdk.c  src/IMU/wit_c_sdk.h)

add_library(MS_b src/BAR/driver_ms5837_basic.c  src/BAR/driver_ms5837_basic.h)
add_library(MS_i src/BAR/raspberrypi4b_driver_ms5837_interface.c  src/BAR/driver_ms5837_interface.h)
add_library(MS_iic src/BAR/iic.c  src/BAR/iic.h)
add_library(MS_d src/BAR/driver_ms5837.c  src/BAR/driver_ms5837.h)




find_package(OpenSSL REQUIRED)
find_package(eclipse-paho-mqtt-c REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_library(wiringPi_LIB wiringPi)

#find_package(ms5837 REQUIRED)
#target_link_libraries(myapp ms5837)

target_link_libraries(myapp MS_b)
target_link_libraries(myapp MS_d)
target_link_libraries(myapp MS_i)
target_link_libraries(myapp MS_iic)



target_link_libraries(myapp CA)
target_link_libraries(myapp JM)
target_link_libraries(myapp PPC)
target_link_libraries(myapp RC)
target_link_libraries(myapp paho-mqtt3a)
target_link_libraries(myapp ${wiringPi_LIB})
target_link_libraries(myapp PahoMqttCpp::paho-mqttpp3)
target_link_libraries(myapp IMU)
target_link_libraries(myapp IMU_i2c)
target_link_libraries(myapp IMU_wit)
