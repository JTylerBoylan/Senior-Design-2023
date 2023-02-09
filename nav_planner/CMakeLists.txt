cmake_minimum_required(VERSION 3.8)
project(sd504_nav_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nvblox_msgs REQUIRED)

include_directories(include)

add_executable(planner src/planner_node.cpp)
ament_target_dependencies(planner rclcpp nvblox_msgs)

install(
	TARGETS planner
	DESTINATION lib/${PROJECT_NAME})

ament_package()
