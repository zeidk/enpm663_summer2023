cmake_minimum_required(VERSION 3.8)
project(frame_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# include folder whish has all the header files
include_directories(include)

# Create a variable for all dependencies
set(FRAME_DEMO_INCLUDE_DEPENDS
  rclcpp 
  tf2_ros
  geometry_msgs
  tf2
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
)

# Find all dependencies
foreach(dependency IN ITEMS ${FRAME_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()


#-----------------------------
# C++
#-----------------------------
add_executable(frame_demo_exe src/kdl_frame_demo.cpp src/utils.cpp src/main.cpp)
ament_target_dependencies(frame_demo_exe ${FRAME_DEMO_INCLUDE_DEPENDS})

install(TARGETS frame_demo_exe
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
  nodes/frame_demo_exe.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
frame_demo_exe
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()