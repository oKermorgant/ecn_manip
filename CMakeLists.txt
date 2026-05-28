cmake_minimum_required(VERSION 3.15)
project(ecn_manip)

# ROS packages + visp
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(VISP REQUIRED COMPONENTS core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# get ROSDISTRO to adapt fors header extension
if("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "lyrical")
  add_compile_definitions(ROS_HEADERS_H_DEPRECATED)
endif()


# main library
ament_auto_add_library(ecn_manip SHARED src/robot_base.cpp
                                src/trig_solvers.cpp
                                src/ur_kin.cpp
                                src/node.cpp)
target_include_directories(ecn_manip PUBLIC include ${VISP_INCLUDE_DIRS})
target_link_libraries(ecn_manip ${VISP_LIBRARIES})

# Main executable
ament_auto_add_executable(control control.cpp
               robot_turret.cpp
               robot_ur10.cpp
               robot_kr16.cpp
               )
target_link_libraries(control ecn_manip)

# Other executables
ament_auto_add_executable(test_solvers src/test_solvers.cpp)
target_link_libraries(test_solvers ecn_manip)

ament_auto_add_executable(example_RRRP example/robot_rrrp.h example/robot_rrrp.cpp example/rrrp_solver.cpp)
target_link_libraries(example_RRRP ecn_manip)

# ament_auto_add_executable(example_5dof example/5dof_solver.cpp)
# target_link_libraries(example_5dof ecn_manip)
#
# ament_auto_add_executable(example_6dof example/6dof_solver.cpp)
# target_link_libraries(example_6dof ecn_manip)
#
# ament_auto_add_executable(example_cylinder example/robot_cylinder.h example/robot_cylinder.cpp example/cylinder_solver.cpp)
# target_link_libraries(example_cylinder ecn_manip)

  
file(GLOB ecn_manip_python src/python/*.py) 
install(PROGRAMS ${ecn_manip_python} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY robots launch
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY include/ DESTINATION include/)

ament_export_dependencies(urdf tf2_ros)
ament_export_libraries(ecn_manip)
ament_export_include_directories(include)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

