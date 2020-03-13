cmake_minimum_required(VERSION 2.8.3)
project(canyonero_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  canyonero_state.msg
  )

generate_messages(DEPENDENCIES std_msgs )

catkin_package(CATKIN_DEPENDS message_runtime std_msgs )
