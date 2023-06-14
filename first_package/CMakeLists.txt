cmake_minimum_required(VERSION 3.8)
project(first_package)

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
add_executable(advanced_publisher_exe src/advanced_publisher.cpp)
ament_target_dependencies(advanced_publisher_exe rclcpp std_msgs)

# Subscriber
add_executable(subscriber_exe src/subscriber.cpp)
ament_target_dependencies(subscriber_exe rclcpp std_msgs)

install(TARGETS
subscriber_exe
advanced_publisher_exe
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/advanced_publisher.py
  nodes/subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
