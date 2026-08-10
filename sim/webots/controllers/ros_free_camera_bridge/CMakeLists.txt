cmake_minimum_required(VERSION 3.16)
project(ros_free_camera_bridge_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT DEFINED ENV{WEBOTS_HOME})
  set(WEBOTS_HOME /usr/local/webots)
else()
  set(WEBOTS_HOME $ENV{WEBOTS_HOME})
endif()

get_filename_component(HFUT_PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)

add_executable(ros_free_camera_bridge ros_free_camera_bridge.cpp)

target_include_directories(ros_free_camera_bridge PRIVATE
  "${WEBOTS_HOME}/include/controller/cpp"
  "${HFUT_PROJECT_ROOT}"
)

target_link_directories(ros_free_camera_bridge PRIVATE
  "${WEBOTS_HOME}/lib/controller"
)

target_link_libraries(ros_free_camera_bridge
  CppController
  Controller
)

set_target_properties(ros_free_camera_bridge PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  BUILD_RPATH "${WEBOTS_HOME}/lib/controller"
)
