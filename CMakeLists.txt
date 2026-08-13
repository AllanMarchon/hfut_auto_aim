cmake_minimum_required(VERSION 3.16)
project(hfut_auto_aim LANGUAGES CXX)

include(CTest)

# hfut_rm_auto_aim 仿真链路的 ROS-free 移植版。使用纯 CMake，不依赖 ament。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED)
find_library(LZ4_LIBRARY NAMES lz4 REQUIRED)

if(UNIX)
  option(HFUT_ENABLE_REAL_IO "Build POSIX real-vehicle serial IO adapters" ON)
else()
  option(HFUT_ENABLE_REAL_IO "Build POSIX real-vehicle serial IO adapters" OFF)
endif()
option(HFUT_ENABLE_DETECTOR "Build detector library and detector-dependent apps" ON)
option(HFUT_ENABLE_PIPELINE "Build tracker/controller pipeline and pipeline-dependent apps" ON)
option(HFUT_ENABLE_HIK_CAMERA "Build Hikvision MVS camera source" OFF)
option(HFUT_ENABLE_MINDVISION_CAMERA "Build MindVision camera source" OFF)

set(HFUT_HIK_SDK_DIR "/opt/MVS" CACHE PATH "Hikvision MVS SDK root")
set(HFUT_HIK_INCLUDE_DIR "" CACHE PATH "Hikvision SDK include directory override")
set(HFUT_HIK_LIBRARY "" CACHE FILEPATH "Hikvision libMvCameraControl.so override")
set(HFUT_MINDVISION_SDK_DIR "" CACHE PATH "MindVision SDK root")
set(HFUT_MINDVISION_INCLUDE_DIR "" CACHE PATH "MindVision SDK include directory override")
set(HFUT_MINDVISION_LIBRARY "" CACHE FILEPATH "MindVision libMVSDK.so override")

# ---------------------------------------------------------------------------
# io：Webots 文件桥接，以及 WSL2 Gestalt TCP 客户端。
# ---------------------------------------------------------------------------
set(HFUT_IO_SOURCES
  io/camera/opencv_camera_source.cpp
  io/camera/webots_bridge_camera.cpp
  io/gestalt/gestalt_bridge_client.cpp
  io/gestalt/gestalt_idle_scanner.cpp
  io/gestalt/gestalt_latest_frame_receiver.cpp
  io/gimbal/webots_bridge_gimbal.cpp
  io/plotter/plotter.cpp
)

if(HFUT_ENABLE_REAL_IO)
  list(APPEND HFUT_IO_SOURCES
    io/serial/uart_transport.cpp
    io/serial/infantry_serial.cpp
    io/serial/infantry32_serial.cpp
  )
endif()

if(HFUT_ENABLE_HIK_CAMERA)
  list(APPEND HFUT_IO_SOURCES io/camera/hik_camera_source.cpp)
endif()

if(HFUT_ENABLE_MINDVISION_CAMERA)
  list(APPEND HFUT_IO_SOURCES io/camera/mindvision_camera_source.cpp)
endif()

add_library(hfut_io STATIC ${HFUT_IO_SOURCES})
target_include_directories(hfut_io PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/io
)
target_include_directories(hfut_io PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(hfut_io PUBLIC ${OpenCV_LIBS} ${LZ4_LIBRARY})

if(HFUT_ENABLE_HIK_CAMERA)
  if(NOT HFUT_HIK_INCLUDE_DIR)
    find_path(HFUT_HIK_INCLUDE_DIR
      NAMES MvCameraControl.h PixelType.h
      PATHS
        ${HFUT_HIK_SDK_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hik/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hik/include/ros2_hik_camera/inc
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hik/hiksdk/include
      NO_DEFAULT_PATH)
  endif()
  if(NOT HFUT_HIK_LIBRARY)
    find_library(HFUT_HIK_LIBRARY
      NAMES MvCameraControl
      PATHS
        ${HFUT_HIK_SDK_DIR}/lib/64
        ${HFUT_HIK_SDK_DIR}/lib/aarch64
        ${HFUT_HIK_SDK_DIR}/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hik/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hik/hiksdk/lib/64
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hik/hiksdk/lib/aarch64
      NO_DEFAULT_PATH)
  endif()
  if(NOT HFUT_HIK_INCLUDE_DIR OR NOT HFUT_HIK_LIBRARY)
    message(FATAL_ERROR
      "HFUT_ENABLE_HIK_CAMERA=ON but Hikvision SDK was not found. "
      "Set HFUT_HIK_SDK_DIR, HFUT_HIK_INCLUDE_DIR, or HFUT_HIK_LIBRARY.")
  endif()
  target_compile_definitions(hfut_io PUBLIC HFUT_HAS_HIK_CAMERA=1)
  target_include_directories(hfut_io PUBLIC ${HFUT_HIK_INCLUDE_DIR})
  target_link_libraries(hfut_io PUBLIC ${HFUT_HIK_LIBRARY})
endif()

if(HFUT_ENABLE_MINDVISION_CAMERA)
  if(NOT HFUT_MINDVISION_INCLUDE_DIR)
    find_path(HFUT_MINDVISION_INCLUDE_DIR
      NAMES CameraApi.h CameraDefine.h CameraStatus.h
      PATHS
        ${HFUT_MINDVISION_SDK_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mindvision/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mindvision/mvsdk/include
      NO_DEFAULT_PATH)
  endif()
  if(NOT HFUT_MINDVISION_LIBRARY)
    find_library(HFUT_MINDVISION_LIBRARY
      NAMES MVSDK
      PATHS
        ${HFUT_MINDVISION_SDK_DIR}/lib/amd64
        ${HFUT_MINDVISION_SDK_DIR}/lib/arm64
        ${HFUT_MINDVISION_SDK_DIR}/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mindvision/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mindvision/mvsdk/lib/amd64
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mindvision/mvsdk/lib/arm64
      NO_DEFAULT_PATH)
  endif()
  if(NOT HFUT_MINDVISION_INCLUDE_DIR OR NOT HFUT_MINDVISION_LIBRARY)
    message(FATAL_ERROR
      "HFUT_ENABLE_MINDVISION_CAMERA=ON but MindVision SDK was not found. "
      "Set HFUT_MINDVISION_SDK_DIR, HFUT_MINDVISION_INCLUDE_DIR, or HFUT_MINDVISION_LIBRARY.")
  endif()
  target_compile_definitions(hfut_io PUBLIC HFUT_HAS_MINDVISION_CAMERA=1)
  target_include_directories(hfut_io PUBLIC ${HFUT_MINDVISION_INCLUDE_DIR})
  target_link_libraries(hfut_io PUBLIC ${HFUT_MINDVISION_LIBRARY})
endif()

# ---------------------------------------------------------------------------
# 后续阶段继续追加：hfut_detector（Phase B）、hfut_pipeline（Phase C）。
# 保持分阶段组织，方便 Phase A 独立构建。
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Phase B：NN detector 库（armor_detector_nn 移植）。
# ---------------------------------------------------------------------------
if(HFUT_ENABLE_DETECTOR)
  add_subdirectory(tasks/auto_aim/detector)
else()
  message(STATUS "HFUT_ENABLE_DETECTOR=OFF: skipping detector library and detector apps")
endif()

# ---------------------------------------------------------------------------
# Phase C：tracker + selector + controller 库（gimbal_pipeline 移植）。
# ---------------------------------------------------------------------------
if(HFUT_ENABLE_PIPELINE)
  add_subdirectory(tasks/auto_aim/pipeline)
else()
  message(STATUS "HFUT_ENABLE_PIPELINE=OFF: skipping tracker/controller pipeline")
endif()

if(HFUT_ENABLE_DETECTOR AND HFUT_ENABLE_PIPELINE)
  # bringup_sim 应用：detector + pipeline + IO 全链路。
  add_executable(bringup_sim apps/bringup_sim.cpp)
  target_link_libraries(bringup_sim PRIVATE hfut_io hfut_detector hfut_pipeline ${OpenCV_LIBS})
  # 让 bringup_sim 通过 ament_index 兼容层解析 package:// detector 路径，
  # 与 hfut_detector 目标保持同一套约定。
  target_compile_definitions(bringup_sim PRIVATE
    HFUT_DETECTOR_ASSET_DIR=${CMAKE_CURRENT_SOURCE_DIR}/tasks/auto_aim/detector)
endif()

if(HFUT_ENABLE_DETECTOR)
  # Phase B detector 验证程序。
  add_executable(detector_test apps/detector_test.cpp)
  target_link_libraries(detector_test PRIVATE hfut_io hfut_detector ${OpenCV_LIBS})
endif()

add_executable(camera_source_test apps/camera_source_test.cpp)
target_link_libraries(camera_source_test PRIVATE hfut_io ${OpenCV_LIBS})

if(HFUT_ENABLE_REAL_IO)
  add_executable(serial_transport_test apps/serial_transport_test.cpp)
  target_link_libraries(serial_transport_test PRIVATE hfut_io)

  if(HFUT_ENABLE_DETECTOR AND HFUT_ENABLE_PIPELINE)
    add_executable(bringup_real apps/bringup_real.cpp)
    target_include_directories(bringup_real PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(bringup_real PRIVATE
      hfut_io hfut_detector hfut_pipeline ${OpenCV_LIBS} yaml-cpp)
    target_compile_definitions(bringup_real PRIVATE
      HFUT_DETECTOR_ASSET_DIR=${CMAKE_CURRENT_SOURCE_DIR}/tasks/auto_aim/detector)
  endif()
endif()

if(HFUT_ENABLE_DETECTOR AND HFUT_ENABLE_PIPELINE)
  # 离线真实相机视频链路：detection -> PnP -> tracking -> prediction。
  add_executable(video_test apps/video_test.cpp)
  target_include_directories(video_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(video_test PRIVATE hfut_detector hfut_pipeline ${OpenCV_LIBS})
  target_compile_definitions(video_test PRIVATE
    HFUT_DETECTOR_ASSET_DIR=${CMAKE_CURRENT_SOURCE_DIR}/tasks/auto_aim/detector)
endif()

if(BUILD_TESTING)
  add_executable(camera_extrinsics_test tests/camera_extrinsics_test.cpp)
  target_link_libraries(camera_extrinsics_test PRIVATE Eigen3::Eigen ${OpenCV_LIBS})
  target_include_directories(camera_extrinsics_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
  add_test(NAME camera_extrinsics COMMAND camera_extrinsics_test)

  add_executable(gestalt_bridge_client_test tests/gestalt_bridge_client_test.cpp)
  target_link_libraries(gestalt_bridge_client_test PRIVATE hfut_io ${OpenCV_LIBS})
  add_test(NAME gestalt_bridge_client COMMAND gestalt_bridge_client_test)

  add_executable(gestalt_idle_scanner_test tests/gestalt_idle_scanner_test.cpp)
  target_link_libraries(gestalt_idle_scanner_test PRIVATE hfut_io)
  add_test(NAME gestalt_idle_scanner COMMAND gestalt_idle_scanner_test)

  if(HFUT_ENABLE_PIPELINE)
    add_executable(tracker_motion_guard_test tests/tracker_motion_guard_test.cpp)
    target_link_libraries(tracker_motion_guard_test PRIVATE Eigen3::Eigen)
    target_include_directories(tracker_motion_guard_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/tasks/auto_aim/pipeline/include)
    add_test(NAME tracker_motion_guard COMMAND tracker_motion_guard_test)

    add_executable(armor_selector_hysteresis_test tests/armor_selector_hysteresis_test.cpp)
    target_link_libraries(armor_selector_hysteresis_test PRIVATE hfut_pipeline)
    add_test(NAME armor_selector_hysteresis COMMAND armor_selector_hysteresis_test)

    add_executable(armor_geometry_3d_test tests/armor_geometry_3d_test.cpp)
    target_link_libraries(armor_geometry_3d_test PRIVATE hfut_pipeline)
    add_test(NAME armor_geometry_3d COMMAND armor_geometry_3d_test)

    add_executable(target_selector_blocked_test tests/target_selector_blocked_test.cpp)
    target_link_libraries(target_selector_blocked_test PRIVATE hfut_pipeline)
    add_test(NAME target_selector_blocked COMMAND target_selector_blocked_test)

    add_executable(vehicle_height_evidence_test tests/vehicle_height_evidence_test.cpp)
    target_link_libraries(vehicle_height_evidence_test PRIVATE hfut_pipeline)
    add_test(NAME vehicle_height_evidence COMMAND vehicle_height_evidence_test)

    add_executable(gimbal_cmd_filter_test tests/gimbal_cmd_filter_test.cpp)
    target_link_libraries(gimbal_cmd_filter_test PRIVATE hfut_pipeline)
    add_test(NAME gimbal_cmd_filter COMMAND gimbal_cmd_filter_test)

    add_executable(vehicle_rotation_witness_test tests/vehicle_rotation_witness_test.cpp)
    target_link_libraries(vehicle_rotation_witness_test PRIVATE hfut_pipeline)
    add_test(NAME vehicle_rotation_witness COMMAND vehicle_rotation_witness_test)
  endif()

  if(HFUT_ENABLE_DETECTOR)
    add_executable(armor_pose_ippe_test tests/armor_pose_ippe_test.cpp)
    target_link_libraries(armor_pose_ippe_test PRIVATE hfut_detector ${OpenCV_LIBS})
    add_test(NAME armor_pose_ippe COMMAND armor_pose_ippe_test)

    add_executable(detector_track_id_test tests/detector_track_id_test.cpp)
    target_link_libraries(detector_track_id_test PRIVATE hfut_detector ${OpenCV_LIBS})
    add_test(NAME detector_track_id COMMAND detector_track_id_test)
  endif()

  add_executable(video_calibration_test tests/video_calibration_test.cpp)
  target_include_directories(video_calibration_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME video_calibration COMMAND video_calibration_test)

  add_executable(fixed_packet_test tests/fixed_packet_test.cpp)
  target_include_directories(fixed_packet_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME fixed_packet COMMAND fixed_packet_test)

  if(HFUT_ENABLE_REAL_IO)
    add_executable(infantry_serial_protocol_test tests/infantry_serial_protocol_test.cpp)
    target_link_libraries(infantry_serial_protocol_test PRIVATE hfut_io)
    add_test(NAME infantry_serial_protocol COMMAND infantry_serial_protocol_test)
  endif()
endif()
