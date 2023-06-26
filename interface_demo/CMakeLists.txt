cmake_minimum_required(VERSION 3.8)
project(interface_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(enpm663_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include)

# Publisher
add_executable(message_demo_exe src/message_demo.cpp)
ament_target_dependencies(message_demo_exe rclcpp enpm663_msgs builtin_interfaces)


install(TARGETS
message_demo_exe
DESTINATION lib/${PROJECT_NAME}
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/message_demo_exe.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
