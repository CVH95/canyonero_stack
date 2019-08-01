cmake_minimum_required(VERSION 2.8.3)
project(Canyonero)

find_package(OpenCV 3 REQUIRED)
find_library(wiringPi_LIB wiringPi)

# Includes
#include_directories(${OpenCV_INCLUDE_DIRS} ${WIRINGPI_INCLUDE_DIRS} include)
include_directories(${OpenCV_INCLUDE_DIRS} include)

# Set output paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/libs)

# Add SHARED libraries
add_library(camera_driver SHARED src/camera_driver.cpp)
add_library(gpioControl SHARED src/gpioControl.cpp)

############# EXECUTABLES #############

# Build C++ Keyboard Teleoperation
add_executable(keyboardTeleop src/keyboardTeleop.cpp)
target_link_libraries(keyboardTeleop gpioControl ${wiringPi_LIB} ncurses)

# Build WebCam Streaming
add_executable(webCam src/webCam.cpp)
target_link_libraries(webCam camera_driver ${OpenCV_LIBS})
