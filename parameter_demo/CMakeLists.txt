cmake_minimum_required(VERSION 3.8)
project(parameter_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Publisher
# add_executable(parameter_demo_exe src/parameter_demo.cpp)
# ament_target_dependencies(parameter_demo_exe rclcpp std_msgs)

# install(TARGETS
# parameter_demo_exe
# DESTINATION lib/${PROJECT_NAME}
# )

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
  nodes/parameter_demo_exe.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
