cmake_minimum_required(VERSION 3.5)
project(orbslam3_pose)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(image_transport REQUIRED)  # Eklenmiştir

# Include directories
include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels/GeometricCamera.h
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

# Mono executable
add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)

ament_target_dependencies(mono 
  rclcpp 
  sensor_msgs 
  tf2_ros 
  tf2_geometry_msgs 
  tf2 
  nav_msgs 
  cv_bridge 
  ORB_SLAM3 
  Pangolin 
  OpenCV
  visualization_msgs  
  std_srvs  
  image_transport  # Buraya eklenmiştir
)

# Install the target
install(TARGETS mono DESTINATION lib/${PROJECT_NAME})

ament_package()
