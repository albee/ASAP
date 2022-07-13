cmake_minimum_required(VERSION 3.0)
project(my_node)

add_compile_options(-std=c++14)

# Find catkin, and add find_package() for other *catkin packages* (to be used later)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
  trajectory_msgs
  sensor_msgs
  ff_msgs
  rospy
)

# Specify catkin info to generate pkg-config and CMake files for *catkin CMake projects* and *other vanilla CMake projects*
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

# System dependencies found via normal CMake (just an example)
find_package(Eigen3 REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})

# Install e.g., Python scripts to a desired location
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/
  DESTINATION share/${PROJECT_NAME}/src
  USE_SOURCE_PERMISSIONS)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)