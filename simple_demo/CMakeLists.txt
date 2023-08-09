cmake_minimum_required(VERSION 3.8)
project(simple_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Publisher
add_executable(simple_publisher_exe src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher_exe rclcpp std_msgs)

# Subscriber
add_executable(simple_subscriber_exe src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber_exe rclcpp std_msgs)

install(TARGETS
simple_publisher_exe
simple_subscriber_exe
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})


ament_package()
