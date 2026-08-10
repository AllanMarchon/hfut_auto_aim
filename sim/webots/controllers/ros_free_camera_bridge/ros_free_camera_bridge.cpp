#include <webots/Camera.hpp>
#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "io/bridge_protocol.hpp"

namespace {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Mat3 {
  double m[3][3]{};
};

struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ArmorSample {
  std::string name;
  std::string number;
  Vec3 center_world;
  Vec3 normal_world;
  Vec3 width_axis_world;
  Vec3 height_axis_world;
  double radial_yaw{0.0};
  Quaternion surface_q;
};

double getEnvDouble(const char *name, double defaultValue) {
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') return defaultValue;
  return std::stod(value);
}

int getEnvInt(const char *name, int defaultValue) {
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') return defaultValue;
  return std::stoi(value);
}

std::string getEnvString(const char *name, const std::string &defaultValue) {
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') return defaultValue;
  return value;
}

bool getEnvBool(const char *name, bool defaultValue) {
  std::string value = getEnvString(name, defaultValue ? "true" : "false");
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (value == "1" || value == "true" || value == "yes" || value == "on") return true;
  if (value == "0" || value == "false" || value == "no" || value == "off") return false;
  throw std::runtime_error(std::string(name) + " must be a boolean value");
}

int getControlStepMs(int basicTimeStep, int defaultStep) {
  const int controlStep = getEnvInt("WEBOTS_CONTROLLER_STEP_MS", defaultStep);
  if (controlStep < basicTimeStep) {
    throw std::runtime_error("WEBOTS_CONTROLLER_STEP_MS must be >= basicTimeStep");
  }
  if (controlStep % basicTimeStep != 0) {
    throw std::runtime_error("WEBOTS_CONTROLLER_STEP_MS must be a multiple of basicTimeStep");
  }
  return controlStep;
}

std::string resolveBridgeDir() {
  const std::string configured = getEnvString("WEBOTS_ROS_FREE_BRIDGE_DIR", "");
  return configured.empty() ? hfut::bridge::kDefaultBridgeDir : configured;
}

std::string joinPath(std::string dir, const std::string &file) {
  while (!dir.empty() && (dir.back() == '/' || dir.back() == '\\')) dir.pop_back();
  return dir + "/" + file;
}

Vec3 operator+(const Vec3 &a, const Vec3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3 &a, double scale) {
  return {a.x * scale, a.y * scale, a.z * scale};
}

double dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3 &a, const Vec3 &b) {
  return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x};
}

double norm(const Vec3 &value) {
  return std::sqrt(dot(value, value));
}

Vec3 normalized(const Vec3 &value) {
  const double n = norm(value);
  if (n <= 1e-12 || !std::isfinite(n)) return {};
  return value * (1.0 / n);
}

double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double stepToward(double current, double target, double maxStep, bool wrap) {
  double error = wrap ? normalizeAngle(target - current) : target - current;
  if (maxStep > 0.0) error = std::clamp(error, -maxStep, maxStep);
  const double next = current + error;
  return wrap ? normalizeAngle(next) : next;
}

Mat3 identity() {
  Mat3 out;
  out.m[0][0] = 1.0;
  out.m[1][1] = 1.0;
  out.m[2][2] = 1.0;
  return out;
}

Mat3 multiply(const Mat3 &a, const Mat3 &b) {
  Mat3 out;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out.m[r][c] = a.m[r][0] * b.m[0][c] + a.m[r][1] * b.m[1][c] +
                    a.m[r][2] * b.m[2][c];
    }
  }
  return out;
}

Mat3 rotationX(double angle) {
  Mat3 out = identity();
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  out.m[1][1] = c;
  out.m[1][2] = -s;
  out.m[2][1] = s;
  out.m[2][2] = c;
  return out;
}

Mat3 rotationY(double angle) {
  Mat3 out = identity();
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  out.m[0][0] = c;
  out.m[0][2] = s;
  out.m[2][0] = -s;
  out.m[2][2] = c;
  return out;
}

Mat3 rotationZ(double angle) {
  Mat3 out = identity();
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  out.m[0][0] = c;
  out.m[0][1] = -s;
  out.m[1][0] = s;
  out.m[1][1] = c;
  return out;
}

Quaternion quaternionFromMatrix(const Mat3 &matrix) {
  Quaternion q;
  const double trace = matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2];
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q.w = 0.25 * s;
    q.x = (matrix.m[2][1] - matrix.m[1][2]) / s;
    q.y = (matrix.m[0][2] - matrix.m[2][0]) / s;
    q.z = (matrix.m[1][0] - matrix.m[0][1]) / s;
  } else if (matrix.m[0][0] > matrix.m[1][1] && matrix.m[0][0] > matrix.m[2][2]) {
    const double s = std::sqrt(1.0 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2]) * 2.0;
    q.w = (matrix.m[2][1] - matrix.m[1][2]) / s;
    q.x = 0.25 * s;
    q.y = (matrix.m[0][1] + matrix.m[1][0]) / s;
    q.z = (matrix.m[0][2] + matrix.m[2][0]) / s;
  } else if (matrix.m[1][1] > matrix.m[2][2]) {
    const double s = std::sqrt(1.0 + matrix.m[1][1] - matrix.m[0][0] - matrix.m[2][2]) * 2.0;
    q.w = (matrix.m[0][2] - matrix.m[2][0]) / s;
    q.x = (matrix.m[0][1] + matrix.m[1][0]) / s;
    q.y = 0.25 * s;
    q.z = (matrix.m[1][2] + matrix.m[2][1]) / s;
  } else {
    const double s = std::sqrt(1.0 + matrix.m[2][2] - matrix.m[0][0] - matrix.m[1][1]) * 2.0;
    q.w = (matrix.m[1][0] - matrix.m[0][1]) / s;
    q.x = (matrix.m[0][2] + matrix.m[2][0]) / s;
    q.y = (matrix.m[1][2] + matrix.m[2][1]) / s;
    q.z = 0.25 * s;
  }
  const double length = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (length > 1e-12 && std::isfinite(length)) {
    q.w /= length;
    q.x /= length;
    q.y /= length;
    q.z /= length;
  }
  return q;
}

Mat3 matrixFromColumns(const Vec3 &xAxis, const Vec3 &yAxis, const Vec3 &zAxis) {
  Mat3 out;
  out.m[0][0] = xAxis.x;
  out.m[1][0] = xAxis.y;
  out.m[2][0] = xAxis.z;
  out.m[0][1] = yAxis.x;
  out.m[1][1] = yAxis.y;
  out.m[2][1] = yAxis.z;
  out.m[0][2] = zAxis.x;
  out.m[1][2] = zAxis.y;
  out.m[2][2] = zAxis.z;
  return out;
}

Vec3 matrixColumn(const double *orientation, int column) {
  return {orientation[column], orientation[3 + column], orientation[6 + column]};
}

Vec3 matrixVectorProduct(const double *orientation, const Vec3 &local) {
  return {
      orientation[0] * local.x + orientation[1] * local.y + orientation[2] * local.z,
      orientation[3] * local.x + orientation[4] * local.y + orientation[5] * local.z,
      orientation[6] * local.x + orientation[7] * local.y + orientation[8] * local.z};
}

Vec3 nodePosition(webots::Node *node) {
  const double *position = node ? node->getPosition() : nullptr;
  if (position == nullptr) throw std::runtime_error("cannot read node position");
  return {position[0], position[1], position[2]};
}

ArmorSample readArmor(
    webots::Node *node,
    webots::Node *targetRoot,
    const std::string &name,
    const std::string &number,
    double armorPitch) {
  if (node == nullptr) throw std::runtime_error("missing Webots armor node: " + name);
  const double *orientation = node->getOrientation();
  if (orientation == nullptr) throw std::runtime_error("cannot read armor orientation: " + name);

  ArmorSample sample;
  sample.name = name;
  sample.number = number;
  sample.center_world = nodePosition(node);
  const Vec3 targetCenter = nodePosition(targetRoot);
  const Vec3 radial = sample.center_world - targetCenter;
  sample.radial_yaw = std::atan2(radial.y, radial.x);

  Vec3 widthAxis = normalized(matrixColumn(orientation, 0));
  Vec3 normal = normalized(matrixVectorProduct(
      orientation, {0.0, std::cos(armorPitch), std::sin(armorPitch)}));
  Vec3 heightAxis = normalized(matrixVectorProduct(
      orientation, {0.0, -std::sin(armorPitch), std::cos(armorPitch)}));
  if (norm(cross(normal, widthAxis)) <= 1e-6) {
    widthAxis = normalized(cross({0.0, 0.0, 1.0}, normal));
  }
  heightAxis = normalized(cross(normal, widthAxis));
  widthAxis = normalized(cross(heightAxis, normal));
  sample.normal_world = normal;
  sample.width_axis_world = widthAxis;
  sample.height_axis_world = heightAxis;
  sample.surface_q = quaternionFromMatrix(matrixFromColumns(normal, widthAxis, heightAxis));
  return sample;
}

Mat3 cameraOpticalToControl(double yaw, double pitch) {
  Mat3 opticalToCameraLink;
  opticalToCameraLink.m[0][2] = 1.0;
  opticalToCameraLink.m[1][0] = -1.0;
  opticalToCameraLink.m[2][1] = -1.0;

  const Mat3 cameraLinkToBarrel = multiply(
      multiply(rotationZ(getEnvDouble("WEBOTS_CAMERA_TF_YAW", 0.0)),
               rotationY(getEnvDouble("WEBOTS_CAMERA_TF_PITCH", 0.0))),
      rotationX(getEnvDouble("WEBOTS_CAMERA_TF_ROLL", 0.0)));
  const Mat3 opticalToBarrel = multiply(cameraLinkToBarrel, opticalToCameraLink);
  const Mat3 barrelToControl = multiply(rotationZ(yaw), rotationY(-pitch));
  return multiply(barrelToControl, opticalToBarrel);
}

bool readCommandFile(const std::string &path, uint64_t &lastSeq, hfut::bridge::CommandPacket &packet) {
  std::ifstream input(path, std::ios::binary);
  if (!input) return false;
  hfut::bridge::CommandPacket next{};
  input.read(reinterpret_cast<char *>(&next), sizeof(next));
  if (!input || static_cast<size_t>(input.gcount()) != sizeof(next)) return false;
  if (std::memcmp(next.magic, hfut::bridge::kCommandMagic, 8) != 0) return false;
  if (next.version != hfut::bridge::kProtocolVersion || next.seq <= lastSeq) return false;
  lastSeq = next.seq;
  packet = next;
  return true;
}

bool writeArmorPoseFrame(
    const std::string &path,
    uint64_t seq,
    double simTime,
    int width,
    int height,
    double fov,
    double yaw,
    double pitch,
    double yawVelocity,
    double pitchVelocity,
    const Vec3 &shooterWorld,
    const std::vector<ArmorSample> &armors) {
  const std::string tmpPath = path + ".tmp";
  std::filesystem::create_directories(std::filesystem::path(path).parent_path());

  hfut::bridge::ArmorPoseFrameHeader header{};
  std::memcpy(header.magic, hfut::bridge::kArmorPoseMagic, 8);
  header.version = hfut::bridge::kArmorPoseProtocolVersion;
  header.seq = seq;
  header.sim_time_s = simTime;
  header.width = static_cast<uint32_t>(width);
  header.height = static_cast<uint32_t>(height);
  header.armor_count = static_cast<uint32_t>(std::min<size_t>(armors.size(), hfut::bridge::kMaxArmorPoses));
  const double defaultFx = width / (2.0 * std::tan(fov * 0.5));
  header.fx = getEnvDouble("WEBOTS_CAMERA_FX", defaultFx);
  header.fy = getEnvDouble("WEBOTS_CAMERA_FY", header.fx);
  header.cx = getEnvDouble("WEBOTS_CAMERA_CX", (width - 1.0) * 0.5);
  header.cy = getEnvDouble("WEBOTS_CAMERA_CY", (height - 1.0) * 0.5);
  header.gimbal_yaw = yaw;
  header.gimbal_pitch = pitch;
  header.gimbal_yaw_vel = yawVelocity;
  header.gimbal_pitch_vel = pitchVelocity;
  const Quaternion q = quaternionFromMatrix(cameraOpticalToControl(yaw, pitch));
  header.cam_qw = q.w;
  header.cam_qx = q.x;
  header.cam_qy = q.y;
  header.cam_qz = q.z;
  header.cam_tx = shooterWorld.x;
  header.cam_ty = shooterWorld.y;
  header.cam_tz = shooterWorld.z;
  header.position_noise_std_m = getEnvDouble("WEBOTS_ARMOR_POSITION_NOISE_STD_M", 0.0);
  header.yaw_noise_std_rad = getEnvDouble("WEBOTS_ARMOR_YAW_NOISE_STD_RAD", 0.0);

  std::ofstream output(tmpPath, std::ios::binary | std::ios::trunc);
  if (!output) return false;
  output.write(reinterpret_cast<const char *>(&header), sizeof(header));
  for (size_t i = 0; i < header.armor_count; ++i) {
    const auto &armor = armors[i];
    hfut::bridge::ArmorPoseRecord record{};
    std::snprintf(record.number, sizeof(record.number), "%s", armor.number.c_str());
    std::snprintf(record.type, sizeof(record.type), "%s", "small");
    const Vec3 control = armor.center_world - shooterWorld;
    record.x = control.x;
    record.y = control.y;
    record.z = control.z;
    record.radial_yaw = armor.radial_yaw;
    record.confidence = 1.0;
    record.position_noise_std_m = header.position_noise_std_m;
    record.yaw_noise_std_rad = header.yaw_noise_std_rad;
    const Vec3 toShooter = normalized(shooterWorld - armor.center_world);
    const double viewCos = std::clamp(dot(armor.normal_world, toShooter), -1.0, 1.0);
    record.view_angle_rad = std::acos(viewCos);
    record.surface_orientation_valid = 1;
    record.surface_qw = armor.surface_q.w;
    record.surface_qx = armor.surface_q.x;
    record.surface_qy = armor.surface_q.y;
    record.surface_qz = armor.surface_q.z;
    output.write(reinterpret_cast<const char *>(&record), sizeof(record));
  }
  if (!output) return false;
  output.close();
  return std::rename(tmpPath.c_str(), path.c_str()) == 0;
}

void appendTruth(
    std::ofstream &truth,
    uint64_t seq,
    double simTime,
    const Vec3 &shooterWorld,
    const Vec3 &targetWorld,
    const std::vector<ArmorSample> &armors) {
  truth << std::fixed << std::setprecision(6)
        << "{\"seq\":" << seq
        << ",\"sim_time_s\":" << simTime
        << ",\"shooter_position\":[" << shooterWorld.x << ',' << shooterWorld.y << ',' << shooterWorld.z << ']'
        << ",\"target_position\":[" << targetWorld.x << ',' << targetWorld.y << ',' << targetWorld.z << ']'
        << ",\"armors\":[";
  for (size_t i = 0; i < armors.size(); ++i) {
    if (i > 0) truth << ',';
    const auto &armor = armors[i];
    truth << "{\"name\":\"" << armor.name << "\",\"number\":\"" << armor.number
          << "\",\"position\":[" << armor.center_world.x << ',' << armor.center_world.y << ','
          << armor.center_world.z << "],\"radial_yaw\":" << armor.radial_yaw << '}';
  }
  truth << "]}\n";
}

}  // namespace

int main() {
  try {
    webots::Supervisor robot;
    const int basicTimeStep = static_cast<int>(robot.getBasicTimeStep());
    const std::string cameraName = getEnvString("WEBOTS_CAMERA_NAME", "camera");
    const int cameraPeriodMs = getEnvInt("WEBOTS_CAMERA_PERIOD_MS", basicTimeStep);
    const int timestep = getControlStepMs(basicTimeStep, cameraPeriodMs);
    const double maxYawRate = getEnvDouble("WEBOTS_GIMBAL_MAX_YAW_RATE", 20.0);
    const double maxPitchRate = getEnvDouble("WEBOTS_GIMBAL_MAX_PITCH_RATE", 20.0);
    const double webotsYawSign = getEnvDouble("WEBOTS_GIMBAL_WEBOTS_YAW_SIGN", 1.0);
    const double webotsPitchSign = getEnvDouble("WEBOTS_GIMBAL_WEBOTS_PITCH_SIGN", -1.0);
    const double armorPitch = getEnvDouble("WEBOTS_ARMOR_PITCH", 0.2617993877991494);
    const std::string armorNumber = getEnvString("WEBOTS_ARMOR_NUMBER", "4");
    const bool writeTruth = getEnvBool("WEBOTS_WRITE_TARGET_TRUTH", true);

    webots::Camera *camera = robot.getCamera(cameraName);
    if (camera == nullptr) throw std::runtime_error("Webots camera device not found: " + cameraName);
    camera->enable(cameraPeriodMs);

    webots::Node *self = robot.getSelf();
    if (self == nullptr) throw std::runtime_error("ros_free_camera_bridge has no self node");
    webots::Field *cameraYawField = self->getField("rotation");
    webots::Node *cameraPitchNode = robot.getFromDef("SIM_CAMERA_PITCH");
    webots::Field *cameraPitchField = cameraPitchNode ? cameraPitchNode->getField("rotation") : nullptr;
    if (cameraYawField == nullptr || cameraPitchField == nullptr) {
      throw std::runtime_error("missing camera yaw/pitch fields");
    }

    webots::Node *targetRoot = robot.getFromDef("RM_ARMOR_ROBOT");
    std::array<webots::Node *, 4> armorNodes = {
        robot.getFromDef("FRONT_ARMOR"),
        robot.getFromDef("REAR_ARMOR"),
        robot.getFromDef("LEFT_ARMOR"),
        robot.getFromDef("RIGHT_ARMOR")};
    const std::array<std::string, 4> armorNames = {"front", "rear", "left", "right"};
    if (targetRoot == nullptr) throw std::runtime_error("missing RM_ARMOR_ROBOT");

    double yaw = getEnvDouble("WEBOTS_CAMERA_YAW", 0.0);
    double pitch = getEnvDouble("WEBOTS_CAMERA_TILT", 0.0);
    double desiredYaw = yaw;
    double desiredPitch = pitch;
    double previousYaw = yaw;
    double previousPitch = pitch;

    const std::string bridgeDir = resolveBridgeDir();
    const std::string armorPosePath = joinPath(bridgeDir, hfut::bridge::kArmorPoseFrameFile);
    const std::string commandPath = joinPath(bridgeDir, hfut::bridge::kCommandFile);
    const std::string truthPath = joinPath(bridgeDir, "target_truth.jsonl");
    std::filesystem::create_directories(bridgeDir);

    std::ofstream truth;
    if (writeTruth) {
      truth.open(truthPath, std::ios::out | std::ios::trunc);
      if (!truth) throw std::runtime_error("failed to open target truth file: " + truthPath);
    }

    std::cout << "ros_free_camera_bridge bridge_dir=" << bridgeDir
              << " armor_pose=" << armorPosePath
              << " command=" << commandPath << std::endl;

    uint64_t frameSeq = 0;
    uint64_t lastCommandSeq = 0;
    auto lastLog = std::chrono::steady_clock::now();
    while (robot.step(timestep) != -1) {
      hfut::bridge::CommandPacket command{};
      if (readCommandFile(commandPath, lastCommandSeq, command)) {
        const bool normalMode = command.mode == static_cast<int8_t>(hfut::bridge::GimbalMode::normal_measurement);
        const bool unknownMode = command.mode == static_cast<int8_t>(hfut::bridge::GimbalMode::unknown);
        if (normalMode || unknownMode) {
          desiredYaw = normalizeAngle(command.yaw);
          desiredPitch = command.pitch;
        }
      }

      const double dt = static_cast<double>(timestep) / 1000.0;
      previousYaw = yaw;
      previousPitch = pitch;
      yaw = stepToward(yaw, desiredYaw, maxYawRate * dt, true);
      pitch = stepToward(pitch, desiredPitch, maxPitchRate * dt, false);
      const double yawVelocity = dt > 0.0 ? normalizeAngle(yaw - previousYaw) / dt : 0.0;
      const double pitchVelocity = dt > 0.0 ? (pitch - previousPitch) / dt : 0.0;

      const double yawRotation[4] = {0.0, 0.0, 1.0, webotsYawSign * yaw};
      const double pitchRotation[4] = {0.0, 1.0, 0.0, webotsPitchSign * pitch};
      cameraYawField->setSFRotation(yawRotation);
      cameraPitchField->setSFRotation(pitchRotation);

      std::vector<ArmorSample> armors;
      armors.reserve(armorNodes.size());
      for (size_t i = 0; i < armorNodes.size(); ++i) {
        armors.push_back(readArmor(armorNodes[i], targetRoot, armorNames[i], armorNumber, armorPitch));
      }
      const Vec3 shooterWorld = nodePosition(self);
      const Vec3 targetWorld = nodePosition(targetRoot);
      if (!writeArmorPoseFrame(
              armorPosePath, ++frameSeq, robot.getTime(), camera->getWidth(), camera->getHeight(),
              camera->getFov(), yaw, pitch, yawVelocity, pitchVelocity, shooterWorld, armors)) {
        std::cerr << "failed to write " << armorPosePath << std::endl;
      }
      if (truth) {
        appendTruth(truth, frameSeq, robot.getTime(), shooterWorld, targetWorld, armors);
        if ((frameSeq & 0x0fU) == 0) truth.flush();
      }

      const auto now = std::chrono::steady_clock::now();
      if (now - lastLog >= std::chrono::seconds(2)) {
        std::cout << "ros_free_camera_bridge seq=" << frameSeq
                  << " t=" << robot.getTime()
                  << " yaw=" << yaw
                  << " pitch=" << pitch
                  << " cmd_seq=" << lastCommandSeq << std::endl;
        lastLog = now;
      }
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "ros_free_camera_bridge error: " << error.what() << std::endl;
    return 1;
  }
}
