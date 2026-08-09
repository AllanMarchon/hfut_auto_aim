# gimbal_pipeline — ROS-free port. Builds the full tracker (max_entropy_tracker)
# + target_selector + gimbal_controller (incl. qpOASES MPC) as one library.
# ROS types are satisfied by header shims under third_party/compat (rclcpp
# logging+Time, geometry_msgs, rm_interfaces messages) so the algorithm sources
# compile unmodified. The ROS node, tf_handler, ballistic service client, buff
# adapter, and visualization markers are excluded/stubbed.

find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(THIRD_PARTY "${CMAKE_SOURCE_DIR}/third_party")
set(COMPAT "${THIRD_PARTY}/compat")

# qpOASES (installed at /usr/local).
find_path(QPOASES_INCLUDE_DIR qpOASES.hpp PATHS /usr/local/include /usr/include)
find_library(QPOASES_LIB qpOASES PATHS /usr/local/lib /usr/lib)
if(NOT QPOASES_INCLUDE_DIR OR NOT QPOASES_LIB)
  message(FATAL_ERROR "qpOASES not found")
endif()

# All algorithm sources (recursively); the ROS node was never copied here, and
# the ballistic service client .cpp + visualization were removed.
file(GLOB_RECURSE PIPELINE_SRC CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
# ParameterHost lives in the package root (not src/).
list(APPEND PIPELINE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/parameter_host.cpp")

add_library(hfut_pipeline STATIC ${PIPELINE_SRC})

# Asset dir for the url_resolver shim. package:// URLs look like
# package://gimbal_pipeline/config/norm4_v3/... so the base must be the source
# root (rel already includes the leading config/).
target_compile_definitions(hfut_pipeline PRIVATE
  HFUT_PIPELINE_ASSET_DIR=${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(hfut_pipeline PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  "${THIRD_PARTY}/rm_utils/include"
  ${EIGEN3_INCLUDE_DIR}
)
# compat shims FIRST so <rclcpp/...>, <geometry_msgs/...>, <rm_interfaces/...>
# resolve here, never to /opt/ros.
target_include_directories(hfut_pipeline BEFORE PUBLIC "${COMPAT}")
target_include_directories(hfut_pipeline SYSTEM PRIVATE "${QPOASES_INCLUDE_DIR}")

target_link_libraries(hfut_pipeline PUBLIC
  hfut_rm_utils_logger ${OpenCV_LIBS} fmt yaml-cpp)
target_link_libraries(hfut_pipeline PRIVATE ${QPOASES_LIB})
