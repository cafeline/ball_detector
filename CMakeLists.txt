cmake_minimum_required(VERSION 3.8)
project(ball_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(ball_detector
  src/ball_detector_node.cpp
  src/core/ball_detector_core.cpp
  src/processing/pointcloud_processor.cpp
  src/visualizer/visualizer.cpp
  src/tracking/tracking_manager.cpp
  src/tracking/tracking_engine.cpp
  src/tracking/tracking_utils.cpp
  src/cluster/cluster_creator.cpp
  src/cluster/cluster_classifier.cpp
)

ament_export_include_directories(include)
ament_export_libraries(ball_detector)

ament_auto_package(INSTALL_TO_SHARE)