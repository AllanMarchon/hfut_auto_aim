cmake_minimum_required(VERSION 3.16)

find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(OpenVINO QUIET COMPONENTS Runtime)
if(NOT OpenVINO_FOUND)
  find_package(OpenVINO REQUIRED)
endif()
if(NOT TARGET openvino::runtime)
  message(FATAL_ERROR "OpenVINO was found but target openvino::runtime is missing")
endif()

if(TARGET fmt::fmt)
  set(SP25_FMT_TARGET fmt::fmt)
else()
  set(SP25_FMT_TARGET fmt)
endif()

if(TARGET spdlog::spdlog)
  set(SP25_SPDLOG_TARGET spdlog::spdlog)
else()
  set(SP25_SPDLOG_TARGET spdlog)
endif()

add_library(sp25_tools STATIC
  tools/extended_kalman_filter.cpp
  tools/img_tools.cpp
  tools/logger.cpp
  tools/math_tools.cpp
  tools/pid.cpp
  tools/ransac_sine_fitter.cpp
  tools/trajectory.cpp
)

target_include_directories(sp25_tools PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(sp25_tools PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${SP25_FMT_TARGET}
  ${SP25_SPDLOG_TARGET}
)

add_library(sp25_auto_aim STATIC
  tasks/auto_aim/aimer.cpp
  tasks/auto_aim/armor.cpp
  tasks/auto_aim/classifier.cpp
  tasks/auto_aim/detector.cpp
  tasks/auto_aim/shooter.cpp
  tasks/auto_aim/solver.cpp
  tasks/auto_aim/target.cpp
  tasks/auto_aim/tracker.cpp
  tasks/auto_aim/voter.cpp
  tasks/auto_aim/yolo.cpp
  tasks/auto_aim/yolos/yolo11.cpp
  tasks/auto_aim/yolos/yolov5.cpp
  tasks/auto_aim/yolos/yolov8.cpp
)

target_include_directories(sp25_auto_aim PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(sp25_auto_aim PUBLIC
  sp25_tools
  Eigen3::Eigen
  ${OpenCV_LIBS}
  yaml-cpp
  openvino::runtime
  ${SP25_FMT_TARGET}
  ${SP25_SPDLOG_TARGET}
)
