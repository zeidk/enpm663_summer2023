cmake_minimum_required(VERSION 3.8)
project(first_package_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Simple Node
add_executable(simple_node_exe src/simple_node.cpp)
ament_target_dependencies(simple_node_exe rclcpp)

# Publisher
add_executable(advanced_publisher_exe src/advanced_publisher.cpp)
ament_target_dependencies(advanced_publisher_exe rclcpp std_msgs)

# Subscriber
add_executable(subscriber_exe src/subscriber.cpp)
ament_target_dependencies(subscriber_exe rclcpp std_msgs)


install(TARGETS
subscriber_exe
simple_node_exe
advanced_publisher_exe
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include DESTINATION share/${PROJECT_NAME})

ament_package()
