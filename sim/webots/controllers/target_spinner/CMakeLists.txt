cmake_minimum_required(VERSION 3.16)
project(target_spinner_controller)

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

add_executable(target_spinner target_spinner.cpp)

target_include_directories(target_spinner PRIVATE
  "${WEBOTS_HOME}/include/controller/cpp"
)

target_link_directories(target_spinner PRIVATE
  "${WEBOTS_HOME}/lib/controller"
)

target_link_libraries(target_spinner
  CppController
  Controller
)

set_target_properties(target_spinner PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  BUILD_RPATH "${WEBOTS_HOME}/lib/controller"
)
