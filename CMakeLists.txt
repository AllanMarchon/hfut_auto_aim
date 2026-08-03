cmake_minimum_required(VERSION 3.16)
project(hfut_auto_aim LANGUAGES CXX)

include(CTest)

# ROS-free port of the hfut_rm_auto_aim sim chain. Plain CMake, no ament.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED)
find_library(LZ4_LIBRARY NAMES lz4 REQUIRED)

# ---------------------------------------------------------------------------
# io: Webots file bridge plus the WSL2 Gestalt TCP client.
# ---------------------------------------------------------------------------
add_library(hfut_io STATIC
  io/camera/webots_bridge_camera.cpp
  io/gestalt/gestalt_bridge_client.cpp
  io/gestalt/gestalt_idle_scanner.cpp
  io/gestalt/gestalt_latest_frame_receiver.cpp
  io/gimbal/webots_bridge_gimbal.cpp
  io/plotter/plotter.cpp
)
target_include_directories(hfut_io PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/io
)
target_include_directories(hfut_io PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(hfut_io PUBLIC ${OpenCV_LIBS} ${LZ4_LIBRARY})

# ---------------------------------------------------------------------------
# Later phases append: hfut_detector (Phase B), hfut_pipeline (Phase C).
# Kept behind options so Phase A builds standalone.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Phase B: NN detector library (armor_detector_nn port).
# ---------------------------------------------------------------------------
add_subdirectory(tasks/auto_aim/detector)

# ---------------------------------------------------------------------------
# Phase C: tracker + selector + controller library (gimbal_pipeline port).
# ---------------------------------------------------------------------------
add_subdirectory(tasks/auto_aim/pipeline)

# bringup_sim application — full chain: detector + pipeline + IO.
add_executable(bringup_sim apps/bringup_sim.cpp)
target_link_libraries(bringup_sim PRIVATE hfut_io hfut_detector hfut_pipeline ${OpenCV_LIBS})
# Lets bringup_sim resolve package:// detector URLs through the ament_index
# shim (same convention as the hfut_detector target).
target_compile_definitions(bringup_sim PRIVATE
  HFUT_DETECTOR_ASSET_DIR=${CMAKE_CURRENT_SOURCE_DIR}/tasks/auto_aim/detector)

# Phase B detector verification harness.
add_executable(detector_test apps/detector_test.cpp)
target_link_libraries(detector_test PRIVATE hfut_io hfut_detector ${OpenCV_LIBS})

# Offline real-camera video: detection -> PnP -> tracking -> prediction.
add_executable(video_test apps/video_test.cpp)
target_include_directories(video_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(video_test PRIVATE hfut_detector hfut_pipeline ${OpenCV_LIBS})
target_compile_definitions(video_test PRIVATE
  HFUT_DETECTOR_ASSET_DIR=${CMAKE_CURRENT_SOURCE_DIR}/tasks/auto_aim/detector)

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

  add_executable(armor_pose_ippe_test tests/armor_pose_ippe_test.cpp)
  target_link_libraries(armor_pose_ippe_test PRIVATE hfut_detector ${OpenCV_LIBS})
  add_test(NAME armor_pose_ippe COMMAND armor_pose_ippe_test)

  add_executable(detector_track_id_test tests/detector_track_id_test.cpp)
  target_link_libraries(detector_track_id_test PRIVATE hfut_detector ${OpenCV_LIBS})
  add_test(NAME detector_track_id COMMAND detector_track_id_test)

  add_executable(video_calibration_test tests/video_calibration_test.cpp)
  target_include_directories(video_calibration_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME video_calibration COMMAND video_calibration_test)
endif()
