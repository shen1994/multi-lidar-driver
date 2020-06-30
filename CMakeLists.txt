cmake_minimum_required(VERSION 3.4 FATAL_ERROR)
project(lidar_driver)

set(TARGET_NAME multi_lidar_cpu)
add_executable(${TARGET_NAME} main.cpp 
src/common.cpp src/lds_lidar.cpp src/shared_mem.cpp)

target_link_libraries(${TARGET_NAME}
PRIVATE livox_sdk_static -lapr-1 -pthread)

include_directories(${TARGET_NAME} include/ /usr/include/apr-1.0)

set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 11)

install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION bin)
