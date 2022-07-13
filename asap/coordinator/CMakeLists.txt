cmake_minimum_required(VERSION 3.0)
project(coordinator)

add_compile_options(-std=c++14)

find_package(catkin2 REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  ff_msgs
  ff_common
  nodelet
  ff_util
  roscpp
)

# Generate messages in the 'msg' folder
file(GLOB MSG_FILES msg/*.msg)
foreach(SRC ${MSG_FILES})
  get_filename_component(NAME ${SRC} NAME)
  list(APPEND MSG_FILE_NAMES ${NAME})
endforeach()
add_message_files(FILES ${MSG_FILE_NAMES})

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
  LIBRARIES
    PrimaryNodelet
    SecondaryNodelet
  DEPENDS
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    actionlib
    tf
    ff_msgs
    ff_util
    std_msgs
    geometry_msgs
    ff_common
    ff_nodelet
    msg_conversions
    ff_flight
)

find_package(Eigen3 REQUIRED)
include_directories(include ${EIGEN3_INCLUDE_DIR}  ${catkin_INCLUDE_DIRS})

# Primary Coordinator Nodelet
add_library(PrimaryNodelet src/primary_nodelet.cc)
target_include_directories(PrimaryNodelet PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)  # src added for templated functions
add_dependencies(PrimaryNodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(PrimaryNodelet ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

# Secondary Coordinator Nodelet
add_library(SecondaryNodelet src/secondary_nodelet.cc)
target_include_directories(SecondaryNodelet PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)  # src added for templated functions
add_dependencies(SecondaryNodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(SecondaryNodelet ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

# install(TARGETS PrimaryNodelet SecondaryNodelet DESTINATION lib)

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.tpp")

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)