# cmake_minimum_required(VERSION 2.8.3)
project(my_node)

find_package(catkin REQUIRED
  COMPONENTS rospy std_msgs message_generation
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/ DESTINATION share/${PROJECT_NAME}/src USE_SOURCE_PERMISSIONS)

install_launch_files()
