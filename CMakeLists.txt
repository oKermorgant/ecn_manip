cmake_minimum_required(VERSION 2.8.3)
project(ecn_manip)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2_ros
  urdf
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(VISP REQUIRED)

add_message_files(
    FILES
    RobotConfig.msg
)

# build messages
generate_messages(
    DEPENDENCIES std_msgs
    )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS src
LIBRARIES ecn_manip
CATKIN_DEPENDS roscpp sensor_msgs tf2_ros urdf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)

add_library(ecn_manip src/robot_base.h src/robot_base.cpp src/robot_init.h
            src/trig_solvers.h src/trig_solvers.cpp
            src/ur_kin/ur_kin.h src/ur_kin/ur_kin.cpp)
target_link_libraries(ecn_manip ${catkin_LIBRARIES} ${VISP_LIBRARIES} )
add_dependencies(ecn_manip ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(test_solvers src/test_solvers.cpp)
target_link_libraries(test_solvers ecn_manip)

## Declare a C++ executable
add_executable(control control.cpp 
               src/robot_turret.h robot_turret.cpp
               src/robot_ur10.h robot_ur10.cpp
               src/robot_kr16.h robot_kr16.cpp
               )

## Specify libraries to link a library or executable target against
target_link_libraries(control ecn_manip)