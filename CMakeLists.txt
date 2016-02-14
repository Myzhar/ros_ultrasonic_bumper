cmake_minimum_required(VERSION 2.8.7)

project(ros_ultrasonic_bumper)

#########################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
#########################################################

#########################################################
# INCLUDES
set(ULTRASND_INC_PATH
    ${ros_ultrasonic_bumper_SOURCE_DIR}/src)

include_directories(${ULTRASND_INC_PATH})
#########################################################

#########################################################
# SOURCES
file(GLOB_RECURSE ULTRASND_SRC *.cpp)
# HEADERS
file(GLOB_RECURSE ULTRASND_HDR *.h)
#########################################################

#########################################################
# MSG FILES
# Adding MSG files to project source, without compiling them
file(GLOB_RECURSE MSG . *.msg)
set_source_files_properties(${MSG} PROPERTIES HEADER_FILE_ONLY TRUE)
#########################################################

#########################################################
# LAUNCH FILES
# Adding LAUNCH files to project source, without compiling them
file(GLOB_RECURSE LAUNCH . *.launch)
set_source_files_properties(${LAUNCH} PROPERTIES HEADER_FILE_ONLY TRUE)
#########################################################

#########################################################
#CATKIN
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    sensor_msgs
    geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
#########################################################


#########################################################
# EXECUTABLE
add_executable(
    ultrasonic_bumper_node
    ${ULTRASND_HDR}
    ${ULTRASND_SRC}
    ${MSG}
    ${LAUNCH}
)

target_link_libraries(
    ultrasonic_bumper_node
    ${catkin_LIBRARIES} pthread
)
#########################################################
