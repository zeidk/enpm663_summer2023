cmake_minimum_required(VERSION 3.8)
project(moveit_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# include folder which has all the header files
include_directories(include)

# create a variable for all dependencies
set(MOVEIT_DEMO_INCLUDE_DEPENDS
  rclcpp 
  std_msgs
  ariac_msgs
  tf2_ros
  geometry_msgs
  tf2
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  geometric_shapes
  moveit_ros_planning_interface
  robot_commander_msgs
)

# find all dependencies
foreach(dependency IN ITEMS ${MOVEIT_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()


#-----------------------------
# C++
#-----------------------------
add_executable(moveit_demo_exe src/floor_robot.cpp src/main.cpp)
ament_target_dependencies(moveit_demo_exe ${MOVEIT_DEMO_INCLUDE_DEPENDS})

install(TARGETS moveit_demo_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#-----------------------------
# Python
#-----------------------------
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

#-----------------------------
# Installation
#-----------------------------

# Install Python executables
install(PROGRAMS
  nodes/commander_main.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
moveit_demo_exe
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY 
  config
  launch
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
