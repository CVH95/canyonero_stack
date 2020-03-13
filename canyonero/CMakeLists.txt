cmake_minimum_required(VERSION 2.8.3)
project(canyonero)

find_package(catkin REQUIRED COMPONENTS)
find_package(OpenCV 3 REQUIRED)
find_library(wiringPi_LIB wiringPi)

catkin_package(
     INCLUDE_DIRS include
     LIBRARIES camera_driver gpio_control
     # DEPENDS Eigen
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)

# Set output paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/libs)

# Add SHARED libraries
add_library(camera_driver SHARED src/camera_driver.cpp)
target_link_libraries(camera_driver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(gpio_control SHARED src/gpio_control.cpp)
target_link_libraries(gpio_control ${catkin_LIBRARIES} ${wiringPi_LIB} ncurses)

## Mark executables and/or libraries for installation
install(TARGETS camera_driver gpio_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

############# EXECUTABLES #############

# # Build C++ Keyboard Teleoperation
# add_executable(keyboardTeleop src/keyboardTeleop.cpp)
# target_link_libraries(keyboardTeleop gpioControl ${wiringPi_LIB} ncurses)

# # Build WebCam Streaming
# add_executable(webCam src/webCam.cpp)
# target_link_libraries(webCam camera_driver ${OpenCV_LIBS})