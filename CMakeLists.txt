cmake_minimum_required(VERSION 3.16)
project(hfut_sp_vision_25 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenVINO QUIET COMPONENTS Runtime)
if(NOT OpenVINO_FOUND)
  find_package(OpenVINO REQUIRED)
endif()
if(NOT TARGET openvino::runtime)
  message(FATAL_ERROR "OpenVINO was found but target openvino::runtime is missing")
endif()

if(TARGET fmt::fmt)
  set(HFUT_FMT_TARGET fmt::fmt)
else()
  set(HFUT_FMT_TARGET fmt)
endif()

if(TARGET spdlog::spdlog)
  set(HFUT_SPDLOG_TARGET spdlog::spdlog)
else()
  set(HFUT_SPDLOG_TARGET spdlog)
endif()

if(UNIX)
  option(HFUT_ENABLE_HIK_CAMERA "Build Hikrobot MVS camera adapter" ON)
else()
  option(HFUT_ENABLE_HIK_CAMERA "Build Hikrobot MVS camera adapter" OFF)
endif()

set(HFUT_HIK_SDK_DIR "/opt/MVS" CACHE PATH "Hikrobot MVS SDK root")
set(HFUT_HIK_INCLUDE_DIR "" CACHE PATH "Hikrobot MVS include directory override")
set(HFUT_HIK_LIBRARY "" CACHE FILEPATH "Hikrobot libMvCameraControl.so override")

# SP25 工具库。
add_library(tools STATIC
  tools/extended_kalman_filter.cpp
  tools/img_tools.cpp
  tools/logger.cpp
  tools/math_tools.cpp
  tools/pid.cpp
  tools/ransac_sine_fitter.cpp
  tools/trajectory.cpp
)
target_include_directories(tools PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(tools PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${HFUT_FMT_TARGET}
  ${HFUT_SPDLOG_TARGET}
)

# TinyMPC planner used only when standard is configured with planner_mode=mpc.
add_library(tinympcstatic STATIC
  tasks/auto_aim/planner/tinympc/admm.cpp
  tasks/auto_aim/planner/tinympc/tiny_api.cpp
  tasks/auto_aim/planner/tinympc/rho_benchmark.cpp
)
target_include_directories(tinympcstatic PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/tasks/auto_aim/planner
)
target_link_libraries(tinympcstatic PUBLIC Eigen3::Eigen)

# SP25 自瞄核心。
add_library(auto_aim STATIC
  tasks/auto_aim/aimer.cpp
  tasks/auto_aim/armor.cpp
  tasks/auto_aim/classifier.cpp
  tasks/auto_aim/detector.cpp
  tasks/auto_aim/planner/planner.cpp
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
target_include_directories(auto_aim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(auto_aim PUBLIC
  tools
  tinympcstatic
  Eigen3::Eigen
  ${OpenCV_LIBS}
  yaml-cpp
  openvino::runtime
  ${HFUT_FMT_TARGET}
  ${HFUT_SPDLOG_TARGET}
)

# HFUT 实车 IO：保留已跑通的海康相机、infantry 串口和 Web 可视化。
add_library(hfut_real_io STATIC
  io/camera/opencv_camera_source.cpp
  io/serial/uart_transport.cpp
  io/serial/infantry_serial.cpp
  io/serial/infantry32_serial.cpp
  io/serial/hfut_serial_gimbal.cpp
  io/web/debug_mjpeg_server.cpp
)
target_include_directories(hfut_real_io PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(hfut_real_io PUBLIC ${OpenCV_LIBS} Threads::Threads)

if(HFUT_ENABLE_HIK_CAMERA)
  if(NOT HFUT_HIK_INCLUDE_DIR)
    find_path(HFUT_HIK_INCLUDE_DIR
      NAMES MvCameraControl.h PixelType.h
      PATHS
        ${HFUT_HIK_SDK_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hik/include
      NO_DEFAULT_PATH)
  endif()
  if(NOT HFUT_HIK_LIBRARY)
    find_library(HFUT_HIK_LIBRARY
      NAMES MvCameraControl
      PATHS
        ${HFUT_HIK_SDK_DIR}/lib/64
        ${HFUT_HIK_SDK_DIR}/lib/aarch64
        ${HFUT_HIK_SDK_DIR}/lib
      NO_DEFAULT_PATH)
  endif()
  if(NOT HFUT_HIK_INCLUDE_DIR OR NOT HFUT_HIK_LIBRARY)
    message(FATAL_ERROR
      "HFUT_ENABLE_HIK_CAMERA=ON but Hikrobot MVS SDK was not found. "
      "Set HFUT_HIK_SDK_DIR, HFUT_HIK_INCLUDE_DIR, or HFUT_HIK_LIBRARY.")
  endif()
  target_sources(hfut_real_io PRIVATE io/camera/hik_camera_source.cpp)
  target_compile_definitions(hfut_real_io PUBLIC HFUT_HAS_HIK_CAMERA=1)
  target_include_directories(hfut_real_io PUBLIC ${HFUT_HIK_INCLUDE_DIR})
  target_link_libraries(hfut_real_io PUBLIC ${HFUT_HIK_LIBRARY})
endif()

add_executable(standard src/standard.cpp)
target_include_directories(standard PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(standard PRIVATE
  auto_aim
  tools
  hfut_real_io
  ${OpenCV_LIBS}
  yaml-cpp
  Threads::Threads
)

# 上车前硬件验证工具，不依赖旧自瞄核心。
add_executable(serial_transport_test apps/serial_transport_test.cpp)
target_include_directories(serial_transport_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(serial_transport_test PRIVATE hfut_real_io Threads::Threads)

add_executable(manual_gimbal_test apps/manual_gimbal_test.cpp)
target_include_directories(manual_gimbal_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(manual_gimbal_test PRIVATE hfut_real_io ${OpenCV_LIBS} Threads::Threads)

add_executable(capture_calibration_images apps/capture_calibration_images.cpp)
target_include_directories(capture_calibration_images PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(capture_calibration_images PRIVATE
  hfut_real_io
  ${OpenCV_LIBS}
  yaml-cpp
  Threads::Threads
)
