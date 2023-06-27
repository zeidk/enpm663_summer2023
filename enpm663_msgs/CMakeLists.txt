cmake_minimum_required(VERSION 3.8)
project(enpm663_msgs)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message generation
set(msg_files
  "msg/WeatherStation.msg"
)

# Service generation
set(srv_files
  "srv/AddTwoInts.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}

  DEPENDENCIES
  builtin_interfaces
  std_msgs
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()
