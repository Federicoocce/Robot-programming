cmake_minimum_required(VERSION 3.16)

project(rb_project)

# Find required ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
  roslib
)

# Find Eigen package
find_package(Eigen3 REQUIRED)

# Include ROS headers
include_directories(${catkin_INCLUDE_DIRS})

# Include Eigen headers
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${YAML_CPP_INCLUDE_DIR})

# OpenCV (optional, assuming you still need it)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Declare catkin package
catkin_package()

# Add executables, specifying the correct path (e.g., src)
add_executable(eigen_in_class
  src/eigen_in_class.cpp
  
)

add_executable(grid_main
  src/grid_main.cpp
  src/grid_map.cpp
  src/draw_helpers.cpp
  src/laser_scan.cpp
  src/world_item.cpp
  src/laser_scanner.cpp
)

add_executable(test_grid_map
  src/test_grid_map.cpp
  src/grid_map.cpp
  src/draw_helpers.cpp
)

# Simulator executable that integrates with ROS
add_executable(simulator
  src/simulator.cpp
  src/grid_map.cpp
  src/draw_helpers.cpp
  src/laser_scan.cpp
  src/world_item.cpp
  src/laser_scanner.cpp
  src/config_parser.h
)

# Link libraries for grid_main (OpenCV)
target_link_libraries(grid_main
  ${OpenCV_LIBS}
)

# Link libraries for test_grid_map (OpenCV)
target_link_libraries(test_grid_map
  ${OpenCV_LIBS}
)

# Link ROS, Eigen, and OpenCV libraries to simulator
target_link_libraries(simulator
  ${catkin_LIBRARIES}  # Link ROS libraries (roscpp, nav_msgs, etc.)
  ${OpenCV_LIBS}       # Link OpenCV libraries
  Eigen3::Eigen       # Link Eigen3 library
  yaml-cpp
)
