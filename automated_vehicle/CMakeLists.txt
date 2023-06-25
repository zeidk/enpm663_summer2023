cmake_minimum_required(VERSION 3.8)
project(automated_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# Camera
add_executable(camera_exe src/camera.cpp)
ament_target_dependencies(camera_exe rclcpp sensor_msgs)

# Server
add_executable(server_exe src/server.cpp)
ament_target_dependencies(server_exe rclcpp sensor_msgs)


install(TARGETS
camera_exe
server_exe
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/camera_exe.py
  nodes/server_exe.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
