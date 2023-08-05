cmake_minimum_required(VERSION 3.8)
project(competitor_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(test_competitor REQUIRED)

# include folder which has all the header files
include_directories(include)

set(LISTENER_DEPENDENCIES
  rclcpp 
  tf2_ros
  geometry_msgs
  tf2
  tf2_geometry_msgs
  floor_robot_msgs
)

# Find all dependencies
foreach(dependency IN ITEMS ${LISTENER_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

#-----------------------------
# C++
#-----------------------------
add_executable(listener_demo_exe src/listener_demo.cpp src/main.cpp)
ament_target_dependencies(listener_demo_exe ${LISTENER_DEPENDENCIES})

install(TARGETS listener_demo_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY 
  include
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
