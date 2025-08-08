cmake_minimum_required(VERSION 3.5)
project(orbslam3_pose)

# Set the PYTHONPATH to your Python site-packages path
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
find_package(image_transport REQUIRED)  
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/monocular_inertial
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${Pangolin_INCLUDE_DIRS}
)

# Mono executable
add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)

target_link_libraries(mono
  ${Pangolin_LIBRARIES}
  ${ORB_SLAM3_LIBRARIES} 
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
  image_transport  
)

# Install the target
install(TARGETS mono DESTINATION lib/${PROJECT_NAME})

# Mono-inertial executable
add_executable(mono_inertial
  src/monocular_inertial/mono_inertial.cpp
  src/monocular_inertial/monocular-inertial-slam-node.cpp
)

target_link_libraries(mono_inertial
  ${Pangolin_LIBRARIES}
  ${ORB_SLAM3_LIBRARIES}
)

ament_target_dependencies(mono_inertial
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
  image_transport
)

install(TARGETS mono_inertial DESTINATION lib/${PROJECT_NAME})

ament_package()
