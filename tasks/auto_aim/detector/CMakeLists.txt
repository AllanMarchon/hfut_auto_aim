# armor_detector_nn 的 ROS-free 移植版：构建神经网络检测和位姿估计库。
# 默认编译 onnxruntime 后端；OpenVINO 可用时一并编译，便于 Intel NUC
# 直接切换推理后端。ROS 类型通过 third_party/compat 下的头文件兼容层提供。

find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

option(HFUT_ENABLE_OPENVINO "Build OpenVINO detector backend when available" ON)
set(HFUT_OPENVINO_AVAILABLE OFF)
if(HFUT_ENABLE_OPENVINO)
  find_package(OpenVINO QUIET COMPONENTS Runtime)
  if(NOT OpenVINO_FOUND)
    find_package(OpenVINO QUIET)
  endif()
  if(OpenVINO_FOUND AND TARGET openvino::runtime)
    set(HFUT_OPENVINO_AVAILABLE ON)
    message(STATUS "OpenVINO detector backend enabled")
  else()
    message(WARNING
      "OpenVINO was not found; detector will build with ONNX Runtime only. "
      "Set OpenVINO_DIR or install OpenVINO to enable the OpenVINO backend.")
  endif()
endif()

set(THIRD_PARTY "${CMAKE_SOURCE_DIR}/third_party")
set(COMPAT "${THIRD_PARTY}/compat")

# onnxruntime-gpu 是预编译安装包，不是标准 CMake package。
set(ONNXRUNTIME_ROOT "/opt/onnxruntime-gpu" CACHE PATH "onnxruntime-gpu root")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS "${ONNXRUNTIME_ROOT}/include" PATH_SUFFIXES onnxruntime onnxruntime/core/session)
find_library(ONNXRUNTIME_LIB onnxruntime PATHS "${ONNXRUNTIME_ROOT}/lib")
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR
    "在 ${ONNXRUNTIME_ROOT} 下没有找到 onnxruntime-gpu。"
    "如果只是做串口/IO 冒烟测试，请配置 -DHFUT_ENABLE_DETECTOR=OFF。"
    "如果要跑完整自瞄，请安装 ONNX Runtime，或通过 -DONNXRUNTIME_ROOT=/path/to/onnxruntime 指定路径。")
endif()

# 内置的 rm_utils logger，已经去掉 ROS 依赖。
file(GLOB RM_UTILS_SRC "${THIRD_PARTY}/rm_utils/src/logger/*.cpp")

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

if(HFUT_OPENVINO_AVAILABLE)
  list(APPEND DETECTOR_SRC src/backend/openvino_backend.cpp)
endif()

add_library(hfut_detector STATIC ${DETECTOR_SRC} ${RM_UTILS_SRC})

target_compile_definitions(hfut_detector PUBLIC ARMOR_DETECTOR_NN_HAS_ONNX=1)
if(HFUT_OPENVINO_AVAILABLE)
  target_compile_definitions(hfut_detector PUBLIC ARMOR_DETECTOR_NN_HAS_OPENVINO=1)
endif()
# 提供给 ament_index_cpp 兼容层，用于解析 package:// 模型和标签路径。
target_compile_definitions(hfut_detector PRIVATE
  HFUT_DETECTOR_ASSET_DIR=${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(hfut_detector PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  "${THIRD_PARTY}/rm_utils/include"
  ${EIGEN3_INCLUDE_DIR}
)
# 兼容层必须放在前面，避免 <rclcpp/...> 等头文件解析到 /opt/ros。
target_include_directories(hfut_detector BEFORE PUBLIC "${COMPAT}")
target_include_directories(hfut_detector SYSTEM PRIVATE "${ONNXRUNTIME_INCLUDE_DIR}")

target_link_libraries(hfut_detector PUBLIC ${OpenCV_LIBS} yaml-cpp fmt)
target_link_libraries(hfut_detector PRIVATE ${ONNXRUNTIME_LIB})
if(HFUT_OPENVINO_AVAILABLE)
  target_link_libraries(hfut_detector PRIVATE openvino::runtime)
endif()
target_link_directories(hfut_detector PUBLIC "${ONNXRUNTIME_ROOT}/lib")
