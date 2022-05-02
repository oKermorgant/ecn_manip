cmake_minimum_required(VERSION 3.5)
project(ecn_manip)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(urdfdom REQUIRED)

find_package(VISP REQUIRED COMPONENTS core)
set(CMAKE_CXX_STANDARD 17)

include_directories(include ${VISP_INCLUDE_DIRS})

# main library
add_library(ecn_manip SHARED    src/robot_base.cpp 
                                src/trig_solvers.cpp
                                src/ur_kin.cpp
                                src/node.cpp)
target_include_directories(ecn_manip PUBLIC include ${VISP_INCLUDE_DIRS})
target_link_libraries(ecn_manip ${VISP_LIBRARIES})
ament_target_dependencies(ecn_manip rclcpp sensor_msgs tf2_ros urdf urdfdom_headers)

# Main executable
add_executable(control control.cpp
               robot_turret.cpp
               robot_ur10.cpp
               robot_kr16.cpp
               )
target_include_directories(control PUBLIC include)
target_link_libraries(control ecn_manip)
ament_target_dependencies(control rclcpp tf2_ros urdfdom_headers)

# Other executables
add_executable(test_solvers src/test_solvers.cpp)
target_link_libraries(test_solvers ecn_manip)

add_executable(example_RRRP example/robot_rrrp.h example/robot_rrrp.cpp example/rrrp_solver.cpp)
target_link_libraries(example_RRRP ecn_manip)

#ament_target_dependencies(example_RRRP rclcpp ecn_manip tf2_ros)
#ament_target_dependencies(test_solvers rclcpp ecn_manip tf2_ros)

install(TARGETS ecn_manip control test_solvers example_RRRP
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
file(GLOB ecn_manip_python src/python/*.py) 
install(PROGRAMS ${ecn_manip_python} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY robots launch
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY include/ DESTINATION include/)
  
ament_export_libraries(ecn_manip)
ament_export_include_directories(include)

ament_package()

