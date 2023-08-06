cmake_minimum_required(VERSION 3.8)
project(qos_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw REQUIRED)

include_directories(include)

# Publisher
add_executable(qos_publisher_exe src/qos_publisher.cpp)
ament_target_dependencies(qos_publisher_exe rclcpp std_msgs)

# Subscriber
add_executable(qos_subscriber_exe src/qos_subscriber.cpp)
ament_target_dependencies(qos_subscriber_exe rclcpp std_msgs)

install(TARGETS
qos_publisher_exe
qos_subscriber_exe
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
