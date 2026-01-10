cmake_minimum_required(VERSION 3.10)
project(safety_filter)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Create ROS2 node executable
add_executable(safety_filter_node src/safety_filter_node.cpp)

# Indicate to compiler where to search for header files
target_include_directories(safety_filter_node
  PUBLIC
  include)

# Add ROS2 dependencies required by executable
ament_target_dependencies(safety_filter_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Copy executable to installation location
install(TARGETS
  safety_filter_node
  DESTINATION lib/${PROJECT_NAME})

# Copy config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
