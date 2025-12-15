cmake_minimum_required(VERSION 3.10)
project(aeva_tools)

set(CMAKE_CXX_STANDARD 14)

# Find OpenCV for the native viewer
find_package(OpenCV REQUIRED)

# Aeva API settings
set(AEVA_API_ATLAS TRUE)
set(AEVA_API_BUILD_TYPE "aeva")
set(AEVA_API_MIN_VERSION "3.1.0")

find_package(AevaAPI CONFIG PATHS "/opt/aeva/atlas-api/cmake" "/opt/aeva/atlas-api-cuda/cmake")

include_directories(
  ${AevaAPI_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${AevaAPI_LIBRARY_DIRS})

# Native viewer (no ROS dependency)
add_executable(aeva_native_viewer aeva_native_viewer.cpp)
target_link_libraries(aeva_native_viewer
  AevaAPI
  ${OpenCV_LIBS}
  pthread
)

# ROS bridge (optional - only if ROS is available)
find_package(catkin QUIET COMPONENTS roscpp sensor_msgs)
if(catkin_FOUND)
  include_directories(${catkin_INCLUDE_DIRS})

  add_executable(aeva_simple_bridge aeva_simple_bridge.cpp)
  target_link_libraries(aeva_simple_bridge
    ${catkin_LIBRARIES}
    AevaAPI
  )
  message(STATUS "ROS found - building aeva_simple_bridge")
else()
  message(STATUS "ROS not found - skipping aeva_simple_bridge")
endif()
