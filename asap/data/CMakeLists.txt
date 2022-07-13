cmake_minimum_required(VERSION 3.0)
project(data)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rosbag)

catkin_package(
  LIBRARIES
  DEPENDS
  CATKIN_DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})

# install to hardware rospkg directory (will be found by rospkg)
install(DIRECTORY input/ DESTINATION share/${PROJECT_NAME}/input)
