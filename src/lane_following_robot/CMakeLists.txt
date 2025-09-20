cmake_minimum_required(VERSION 3.8)
project(lane_following_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Camera debug executable
add_executable(camera_debug src/camera_debug.cpp)
ament_target_dependencies(camera_debug 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport 
  OpenCV)

# Install executables
install(TARGETS
  camera_debug
  DESTINATION lib/${PROJECT_NAME})

# Install directories
install(DIRECTORY
  launch
  urdf
  worlds
  include
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()