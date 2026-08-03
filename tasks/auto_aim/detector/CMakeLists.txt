# armor_detector_nn — ROS-free port. Builds the NN detection + pose-estimation
# library (onnxruntime backend only). ROS types are satisfied by header shims
# under third_party/compat (rclcpp::Time, std_msgs::Header,
# sensor_msgs::CameraInfo, ament_index_cpp, tf2 LinearMath) so the algorithm
# sources compile unmodified.

find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(THIRD_PARTY "${CMAKE_SOURCE_DIR}/third_party")
set(COMPAT "${THIRD_PARTY}/compat")

# onnxruntime-gpu (installed binary, not a CMake package).
set(ONNXRUNTIME_ROOT "/opt/onnxruntime-gpu" CACHE PATH "onnxruntime-gpu root")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS "${ONNXRUNTIME_ROOT}/include" PATH_SUFFIXES onnxruntime onnxruntime/core/session)
find_library(ONNXRUNTIME_LIB onnxruntime PATHS "${ONNXRUNTIME_ROOT}/lib")
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR "onnxruntime-gpu not found under ${ONNXRUNTIME_ROOT}")
endif()

# rm_utils logger (vendored, ROS-free).
file(GLOB RM_UTILS_SRC "${THIRD_PARTY}/rm_utils/src/logger/*.cpp")

# onnxruntime backend only; openvino/tensorrt sources are intentionally excluded.
set(DETECTOR_SRC
  src/core/armor_detector_nn.cpp
  src/core/armor_pose_estimator_adapter.cpp
  src/core/label_map.cpp
  src/core/preprocessor.cpp
  src/core/corner_refine/roi_pca_corner_refiner.cpp
  src/core/pose_refine/single_yaw_refiner.cpp
  src/core/pose_refine/sliding_window_refiner.cpp
  src/core/tracker/internal_iou_tracker_strategy.cpp
  src/backend/inference_backend_factory.cpp
  src/backend/onnxruntime_backend.cpp
  src/postprocess/decode_strategy_factory.cpp
  src/postprocess/detection_quality_filter.cpp
  src/postprocess/robotpilots_landmark_decode_strategy.cpp
  src/postprocess/ultralytics_pose_decode_strategy.cpp
  src/geometry/nms.cpp
  src/debug/debug_drawer.cpp
  src/debug/profiler.cpp
  src/traditional/light_bar_detector.cpp
  src/traditional/number_classifier.cpp
  src/traditional/light_corner_corrector.cpp
  src/traditional/armor_detection_adapter.cpp
  config_loader.cpp
)

add_library(hfut_detector STATIC ${DETECTOR_SRC} ${RM_UTILS_SRC})

target_compile_definitions(hfut_detector PUBLIC ARMOR_DETECTOR_NN_HAS_ONNX=1)
# Asset dir for the ament_index_cpp shim (resolves package:// model/label paths).
target_compile_definitions(hfut_detector PRIVATE
  HFUT_DETECTOR_ASSET_DIR=${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(hfut_detector PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  "${THIRD_PARTY}/rm_utils/include"
  ${EIGEN3_INCLUDE_DIR}
)
# compat shims FIRST so <rclcpp/...> etc. resolve here, never to /opt/ros.
target_include_directories(hfut_detector BEFORE PUBLIC "${COMPAT}")
target_include_directories(hfut_detector SYSTEM PRIVATE "${ONNXRUNTIME_INCLUDE_DIR}")

target_link_libraries(hfut_detector PUBLIC ${OpenCV_LIBS} yaml-cpp fmt)
target_link_libraries(hfut_detector PRIVATE ${ONNXRUNTIME_LIB})
target_link_directories(hfut_detector PUBLIC "${ONNXRUNTIME_ROOT}/lib")
