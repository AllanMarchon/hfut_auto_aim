#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORLD_TEMPLATE="${PROJECT_DIR}/worlds/flat_camera_world.wbt.template"
WORLD_FILE="${PROJECT_DIR}/worlds/dark_camera_world.wbt"
RENDER_ONLY=0

if [ "${1:-}" = "--render-only" ]; then
  RENDER_ONLY=1
  shift
fi

CONFIG_FILES=(
  "${PROJECT_DIR}/config/environment.env"
  "${PROJECT_DIR}/config/target_robot.env"
  "${PROJECT_DIR}/config/camera_robot.env"
)

if [ -n "${WEBOTS_CAMERA_CONFIG:-}" ]; then
  CONFIG_FILES+=("${WEBOTS_CAMERA_CONFIG}")
fi

for CONFIG_FILE in "${CONFIG_FILES[@]}"; do
  if [ -f "${CONFIG_FILE}" ]; then
    set -a
    # shellcheck source=/dev/null
    source "${CONFIG_FILE}"
    set +a
  fi
done

: "${WEBOTS_BASIC_TIME_STEP:=16}"
: "${WEBOTS_OPTIMAL_THREAD_COUNT:=2}"
: "${WEBOTS_GROUND_SIZE_X:=12}"
: "${WEBOTS_GROUND_SIZE_Y:=8}"
: "${WEBOTS_GROUND_THICKNESS:=0.05}"
: "${WEBOTS_GROUND_Z:=-0.025}"
: "${WEBOTS_GROUND_COLOR:=0.24 0.25 0.24}"
: "${WEBOTS_GROUND_ROUGHNESS:=0.92}"
: "${WEBOTS_SKY_COLOR:=0.012 0.013 0.016}"
: "${WEBOTS_LIGHT_INTENSITY:=1.045}"
: "${WEBOTS_LIGHT_AMBIENT_INTENSITY:=0.55}"
: "${WEBOTS_LIGHT_DIRECTION:=0.25 0.35 -1}"
: "${WEBOTS_ARMOR_TARGET_X:=3}"
: "${WEBOTS_ARMOR_TARGET_Y:=0}"
: "${WEBOTS_ARMOR_TARGET_Z:=0.165}"
: "${WEBOTS_ARMOR_TARGET_YAW:=0}"
: "${WEBOTS_ARMOR_BODY_MESH_URL:=../meshes/armor_red4_body_plate_lights.obj}"
: "${WEBOTS_ARMOR_MESH_SCALE:=0.001 0.001 0.001}"
: "${WEBOTS_ARMOR_MESH_ROTATION:=0 1 0 3.14159265359}"
: "${WEBOTS_ARMOR_MESH_OFFSET:=-0.0018354 -0.13427105 -0.01999975}"
: "${WEBOTS_ARMOR_MESH_CCW:=TRUE}"
: "${WEBOTS_ARMOR_BOUNDING_SIZE:=0.135 0.080 0.135}"
: "${WEBOTS_ARMOR_PITCH:=0.2617993877991494}"
: "${WEBOTS_CAMERA_X:=0}"
: "${WEBOTS_CAMERA_Y:=0}"
: "${WEBOTS_CAMERA_Z:=0.405}"
: "${WEBOTS_CAMERA_LOOK_AT_TARGET:=true}"
: "${WEBOTS_CAMERA_LOOK_AT_Z:=}"
: "${WEBOTS_CAMERA_LOOK_AT_Z_OFFSET:=0.055}"
: "${WEBOTS_CAMERA_GREEN_AXIS_OFFSET:=1.57079632679}"
: "${WEBOTS_CAMERA_YAW:=0}"
: "${WEBOTS_CAMERA_ROTATION:=0 0 1 ${WEBOTS_CAMERA_YAW}}"
: "${WEBOTS_CAMERA_TILT:=0.06158867632}"
: "${WEBOTS_CAMERA_X_ROTATION:=0}"
: "${WEBOTS_CAMERA_ROLL:=0}"
: "${WEBOTS_CAMERA_NAME:=camera}"
: "${WEBOTS_CAMERA_WIDTH:=1440}"
: "${WEBOTS_CAMERA_HEIGHT:=1080}"
: "${WEBOTS_CAMERA_FOV:=0.7850335620966933}"
: "${WEBOTS_CAMERA_NEAR:=0.05}"
: "${WEBOTS_CAMERA_FAR:=30}"
: "${WEBOTS_CAMERA_ANTI_ALIASING:=TRUE}"
: "${WEBOTS_CAMERA_MOTION_BLUR:=0}"
: "${WEBOTS_CAMERA_BODY_SIZE:=0.10 0.08 0.08}"
: "${WEBOTS_CAMERA_BODY_COLOR:=0.82 0.86 0.90}"
: "${WEBOTS_TARGET_SPIN_RATE:=3.0}"
: "${WEBOTS_CONTROLLER_STEP_MS:=32}"
: "${WEBOTS_CAMERA_PERIOD_MS:=32}"
: "${WEBOTS_TARGET_CONTROLLER_STEP_MS:=16}"

compute_camera_look_at() {
  case "${WEBOTS_CAMERA_LOOK_AT_TARGET}" in
    1|true|TRUE|yes|YES|on|ON) ;;
    *) return ;;
  esac

  local look_at_z="${WEBOTS_CAMERA_LOOK_AT_Z}"
  if [ -z "${look_at_z}" ]; then
    look_at_z="$(awk -v target_z="${WEBOTS_ARMOR_TARGET_Z}" -v offset_z="${WEBOTS_CAMERA_LOOK_AT_Z_OFFSET}" 'BEGIN { printf "%.12g", target_z + offset_z; }')"
  fi

  local computed
  computed="$(awk \
    -v cx="${WEBOTS_CAMERA_X}" -v cy="${WEBOTS_CAMERA_Y}" -v cz="${WEBOTS_CAMERA_Z}" \
    -v tx="${WEBOTS_ARMOR_TARGET_X}" -v ty="${WEBOTS_ARMOR_TARGET_Y}" \
    -v offset="${WEBOTS_CAMERA_GREEN_AXIS_OFFSET}" -v tz="${look_at_z}" \
    'BEGIN { dx = tx - cx; dy = ty - cy; dz = tz - cz; h = sqrt(dx * dx + dy * dy); yaw = atan2(dy, dx); tilt = atan2(-h, -dz) + offset; printf "%.12g %.12g", yaw, tilt; }')"
  WEBOTS_CAMERA_YAW="${computed%% *}"
  WEBOTS_CAMERA_TILT="${computed#* }"
  WEBOTS_CAMERA_ROTATION="0 0 1 ${WEBOTS_CAMERA_YAW}"
  export WEBOTS_CAMERA_YAW WEBOTS_CAMERA_TILT WEBOTS_CAMERA_ROTATION
}

render_world() {
  local content
  content="$(<"${WORLD_TEMPLATE}")"
  content="${content//@WEBOTS_BASIC_TIME_STEP@/${WEBOTS_BASIC_TIME_STEP}}"
  content="${content//@WEBOTS_OPTIMAL_THREAD_COUNT@/${WEBOTS_OPTIMAL_THREAD_COUNT}}"
  content="${content//@WEBOTS_GROUND_SIZE_X@/${WEBOTS_GROUND_SIZE_X}}"
  content="${content//@WEBOTS_GROUND_SIZE_Y@/${WEBOTS_GROUND_SIZE_Y}}"
  content="${content//@WEBOTS_GROUND_THICKNESS@/${WEBOTS_GROUND_THICKNESS}}"
  content="${content//@WEBOTS_GROUND_Z@/${WEBOTS_GROUND_Z}}"
  content="${content//@WEBOTS_GROUND_COLOR@/${WEBOTS_GROUND_COLOR}}"
  content="${content//@WEBOTS_GROUND_ROUGHNESS@/${WEBOTS_GROUND_ROUGHNESS}}"
  content="${content//@WEBOTS_SKY_COLOR@/${WEBOTS_SKY_COLOR}}"
  content="${content//@WEBOTS_LIGHT_INTENSITY@/${WEBOTS_LIGHT_INTENSITY}}"
  content="${content//@WEBOTS_LIGHT_AMBIENT_INTENSITY@/${WEBOTS_LIGHT_AMBIENT_INTENSITY}}"
  content="${content//@WEBOTS_LIGHT_DIRECTION@/${WEBOTS_LIGHT_DIRECTION}}"
  content="${content//@WEBOTS_ARMOR_TARGET_X@/${WEBOTS_ARMOR_TARGET_X}}"
  content="${content//@WEBOTS_ARMOR_TARGET_Y@/${WEBOTS_ARMOR_TARGET_Y}}"
  content="${content//@WEBOTS_ARMOR_TARGET_Z@/${WEBOTS_ARMOR_TARGET_Z}}"
  content="${content//@WEBOTS_ARMOR_TARGET_YAW@/${WEBOTS_ARMOR_TARGET_YAW}}"
  content="${content//@WEBOTS_ARMOR_BODY_MESH_URL@/${WEBOTS_ARMOR_BODY_MESH_URL}}"
  content="${content//@WEBOTS_ARMOR_MESH_SCALE@/${WEBOTS_ARMOR_MESH_SCALE}}"
  content="${content//@WEBOTS_ARMOR_MESH_ROTATION@/${WEBOTS_ARMOR_MESH_ROTATION}}"
  content="${content//@WEBOTS_ARMOR_MESH_OFFSET@/${WEBOTS_ARMOR_MESH_OFFSET}}"
  content="${content//@WEBOTS_ARMOR_MESH_CCW@/${WEBOTS_ARMOR_MESH_CCW}}"
  content="${content//@WEBOTS_ARMOR_BOUNDING_SIZE@/${WEBOTS_ARMOR_BOUNDING_SIZE}}"
  content="${content//@WEBOTS_ARMOR_PITCH@/${WEBOTS_ARMOR_PITCH}}"
  content="${content//@WEBOTS_CAMERA_X@/${WEBOTS_CAMERA_X}}"
  content="${content//@WEBOTS_CAMERA_Y@/${WEBOTS_CAMERA_Y}}"
  content="${content//@WEBOTS_CAMERA_Z@/${WEBOTS_CAMERA_Z}}"
  content="${content//@WEBOTS_CAMERA_ROTATION@/${WEBOTS_CAMERA_ROTATION}}"
  content="${content//@WEBOTS_CAMERA_TILT@/${WEBOTS_CAMERA_TILT}}"
  content="${content//@WEBOTS_CAMERA_X_ROTATION@/${WEBOTS_CAMERA_X_ROTATION}}"
  content="${content//@WEBOTS_CAMERA_ROLL@/${WEBOTS_CAMERA_ROLL}}"
  content="${content//@WEBOTS_CAMERA_NAME@/${WEBOTS_CAMERA_NAME}}"
  content="${content//@WEBOTS_CAMERA_WIDTH@/${WEBOTS_CAMERA_WIDTH}}"
  content="${content//@WEBOTS_CAMERA_HEIGHT@/${WEBOTS_CAMERA_HEIGHT}}"
  content="${content//@WEBOTS_CAMERA_FOV@/${WEBOTS_CAMERA_FOV}}"
  content="${content//@WEBOTS_CAMERA_NEAR@/${WEBOTS_CAMERA_NEAR}}"
  content="${content//@WEBOTS_CAMERA_FAR@/${WEBOTS_CAMERA_FAR}}"
  content="${content//@WEBOTS_CAMERA_ANTI_ALIASING@/${WEBOTS_CAMERA_ANTI_ALIASING}}"
  content="${content//@WEBOTS_CAMERA_MOTION_BLUR@/${WEBOTS_CAMERA_MOTION_BLUR}}"
  content="${content//@WEBOTS_CAMERA_BODY_SIZE@/${WEBOTS_CAMERA_BODY_SIZE}}"
  content="${content//@WEBOTS_CAMERA_BODY_COLOR@/${WEBOTS_CAMERA_BODY_COLOR}}"
  printf '%s\n' "${content}" > "${WORLD_FILE}"
}

detect_webots_home() {
  if [ -n "${WEBOTS_HOME:-}" ] && [ -d "${WEBOTS_HOME}" ]; then
    return
  fi
  for candidate in \
    /snap/webots/current/usr/share/webots \
    /usr/local/webots \
    /opt/webots; do
    if [ -d "${candidate}" ]; then
      export WEBOTS_HOME="${candidate}"
      return
    fi
  done
  echo "WEBOTS_HOME not found. Set WEBOTS_HOME or install Webots first." >&2
  exit 127
}

build_cpp_controller() {
  local controller_dir="$1"
  cmake -S "${controller_dir}" -B "${controller_dir}/build" \
    -DCMAKE_BUILD_TYPE=Release
  cmake --build "${controller_dir}/build" --parallel "${WEBOTS_CONTROLLER_BUILD_JOBS:-2}"
}

run_webots() {
  local webots_bin
  webots_bin="$(command -v webots || true)"
  if [ -z "${webots_bin}" ]; then
    webots_bin="${WEBOTS_HOME}/webots"
  fi

  export WEBOTS_ROS_FREE_BRIDGE_DIR="${WEBOTS_ROS_FREE_BRIDGE_DIR:-/tmp/hfut_auto_aim_webots}"
  mkdir -p "${WEBOTS_ROS_FREE_BRIDGE_DIR}"
  rm -f \
    "${WEBOTS_ROS_FREE_BRIDGE_DIR}/armor_pose_frame.bin" \
    "${WEBOTS_ROS_FREE_BRIDGE_DIR}/armor_pose_frame.bin.tmp" \
    "${WEBOTS_ROS_FREE_BRIDGE_DIR}/gimbal_command.bin" \
    "${WEBOTS_ROS_FREE_BRIDGE_DIR}/gimbal_command.bin.tmp" \
    "${WEBOTS_ROS_FREE_BRIDGE_DIR}/target_truth.jsonl" \
    "${WEBOTS_ROS_FREE_BRIDGE_DIR}/tracking_diagnostics.jsonl"

  if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    export LD_LIBRARY_PATH="${WEBOTS_HOME}/lib/controller:${LD_LIBRARY_PATH}"
  else
    export LD_LIBRARY_PATH="${WEBOTS_HOME}/lib/controller"
  fi
  export WEBOTS_PYTHON_COMMAND="${WEBOTS_PYTHON_COMMAND:-/usr/bin/python3}"
  unset http_proxy https_proxy ftp_proxy all_proxy
  unset HTTP_PROXY HTTPS_PROXY FTP_PROXY ALL_PROXY

  local use_xvfb="${HFUT_WEBOTS_USE_XVFB:-auto}"
  if { [ "${use_xvfb}" = "auto" ] && [ -z "${DISPLAY:-}" ]; } || [ "${use_xvfb}" = "1" ] || [ "${use_xvfb}" = "true" ]; then
    local auth_file="${HFUT_WEBOTS_XAUTHORITY:-${HOME}/snap/webots/common/Xauthority}"
    mkdir -p "$(dirname "${auth_file}")"
    rm -f "${auth_file}"
    exec xvfb-run -a \
      -f "${auth_file}" \
      -s "${HFUT_WEBOTS_XVFB_SCREEN:--screen 0 1280x720x24}" \
      env QT_X11_NO_MITSHM=1 "${webots_bin}" "${WORLD_FILE}" "$@"
  fi

  exec "${webots_bin}" "${WORLD_FILE}" "$@"
}

compute_camera_look_at
render_world

if [ "${RENDER_ONLY}" -eq 1 ]; then
  echo "Rendered ${WORLD_FILE}"
  exit 0
fi

detect_webots_home

: "${WEBOTS_BUILD_CPP_CONTROLLERS:=true}"
case "${WEBOTS_BUILD_CPP_CONTROLLERS}" in
  1|true|TRUE|yes|YES|on|ON)
    build_cpp_controller "${PROJECT_DIR}/controllers/target_spinner"
    build_cpp_controller "${PROJECT_DIR}/controllers/ros_free_camera_bridge"
    ;;
esac

run_webots "$@"
