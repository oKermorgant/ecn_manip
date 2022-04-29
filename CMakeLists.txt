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

# main library
add_library(ecn_manip SHARED src/robot_base.h src/robot_base.cpp src/robot_init.h
            src/trig_solvers.h src/trig_solvers.cpp
            src/ur_kin/ur_kin.h src/ur_kin/ur_kin.cpp
            src/node.cpp src/node.h)
target_include_directories(ecn_manip PUBLIC src ${VISP_INCLUDE_DIRS})
target_link_libraries(ecn_manip ${VISP_LIBRARIES})
ament_target_dependencies(ecn_manip rclcpp sensor_msgs tf2_ros urdf urdfdom_headers)

# Main executable
add_executable(control control.cpp
              src/robot_turret.h robot_turret.cpp
               src/robot_ur10.h robot_ur10.cpp
               src/robot_kr16.h robot_kr16.cpp
               )
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

ament_package()

