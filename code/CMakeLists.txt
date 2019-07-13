cmake_minimum_required(VERSION 2.8.3)
project(canyonero)

# Finds
find_package(OpenCV 3 REQUIRED)
#find_package(WiringPi REQUIRED)
#find_package(Threads REQUIRED) 
find_library(wiringPi_LIB wiringPi)

# Includes
#include_directories(${OpenCV_INCLUDE_DIRS} ${WIRINGPI_INCLUDE_DIRS} include)
include_directories(${OpenCV_INCLUDE_DIRS} include)

# Set output paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/libs)

add_library(gpioControl SHARED src/gpioControl.cpp)
add_library(webcamControl SHARED src/webcamControl.cpp)

############# EXECUTABLES #############

# Build C++ Keyboard Teleoperation
add_executable(keyboard_teleop controllers/keyboard_teleop.cpp)
target_link_libraries(keyboard_teleop gpioControl ${wiringPi_LIB} ncurses)

# Build WebCam Streaming
add_executable(webcam_controller controllers/webcam_controller.cpp)
target_link_libraries(webcam_controller webcamControl ${OpenCV_LIBS})
