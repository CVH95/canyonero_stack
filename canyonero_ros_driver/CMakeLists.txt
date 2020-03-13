cmake_minimum_required(VERSION 2.8.3)
project(canyonero_ros_driver)

#SET (UTILS "/home/pi/Workspace/Canyonero")

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  roslib
  cv_bridge
  geometry_msgs
  image_transport
  sensor_msgs
  canyonero_msgs
  canyonero
)

find_package(OpenCV 3 REQUIRED)
# find_library(wiringPi_LIB wiringPi)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES canyonero_ros_driver
  DEPENDS canyonero
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(canyonero_ros_driver SHARED src/canyonero_ros_driver.cpp)
target_link_libraries(canyonero_ros_driver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

## C++ executable: canyonero_ros_drivers_node
add_executable(feedback nodes/feedback.cpp)
target_link_libraries(feedback ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} canyonero_ros_driver )

add_executable(commander nodes/commander.cpp)
target_link_libraries(commander ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} canyonero_ros_driver )

install(TARGETS canyonero_ros_driver feedback commander
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

message("CMake is done!")
message("Exit compilation")
