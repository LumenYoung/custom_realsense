cmake_minimum_required(VERSION 2.8.3)
project(realsense_publisher)

# Enable compile commands JSON generation
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# If needed, explicitly set the RealSense library path
# set(realsense2_DIR /path/to/librealsense/lib/cmake/realsense2)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
find_package(realsense2 REQUIRED)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

# Add executable
add_executable(realsense_publisher src/main.cpp)

# Specify include directories
target_include_directories(realsense_publisher PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(realsense_publisher
  ${catkin_LIBRARIES}
  ${realsense2_LIBRARIES}
  realsense2
)

# Install targets
install(TARGETS realsense_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
