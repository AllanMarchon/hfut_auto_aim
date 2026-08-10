#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

double getEnvDouble(const char *name, double defaultValue) {
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == '\0')
    return defaultValue;
  return std::stod(value);
}

int getEnvInt(const char *name, int defaultValue) {
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == '\0')
    return defaultValue;
  return std::stoi(value);
}

std::string getEnvString(const char *name, const std::string &defaultValue) {
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == '\0')
    return defaultValue;
  return value;
}

bool getEnvBool(const char *name, bool defaultValue) {
  std::string value = getEnvString(name, defaultValue ? "true" : "false");
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "1" || value == "true" || value == "yes" || value == "on")
    return true;
  if (value == "0" || value == "false" || value == "no" || value == "off")
    return false;
  throw std::runtime_error(std::string(name) + " must be a boolean value");
}

int getAxisIndex(const std::string &axis) {
  std::string value = axis;
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "x" || value == "0")
    return 0;
  if (value == "y" || value == "1")
    return 1;
  if (value == "z" || value == "2")
    return 2;
  throw std::runtime_error(
      "WEBOTS_TARGET_LATERAL_AXIS must be x, y, z, 0, 1, or 2");
}

int getControlStepMs(int basicTimeStep) {
  const int controlStep =
      getEnvInt("WEBOTS_TARGET_CONTROLLER_STEP_MS", basicTimeStep);
  if (controlStep < basicTimeStep) {
    throw std::runtime_error(
        "WEBOTS_TARGET_CONTROLLER_STEP_MS must be >= basicTimeStep");
  }
  if (controlStep % basicTimeStep != 0) {
    throw std::runtime_error(
        "WEBOTS_TARGET_CONTROLLER_STEP_MS must be a multiple of basicTimeStep");
  }
  return controlStep;
}

struct MotionProfile {
  double accelTime = 0.0;
  double cruiseTime = 0.0;
  double decelTime = 0.0;
  double peakSpeed = 0.0;
  double accelDistance = 0.0;
  double cruiseDistance = 0.0;

  double duration() const {
    return accelTime + cruiseTime + decelTime;
  }
};

MotionProfile makeMotionProfile(double distance, double maxSpeed,
                                double acceleration, double deceleration) {
  if (distance <= 0.0 || maxSpeed <= 0.0 || acceleration <= 0.0 ||
      deceleration <= 0.0) {
    throw std::runtime_error(
        "lateral amplitude, speed, acceleration, and deceleration must be positive");
  }

  MotionProfile profile;
  profile.accelDistance = maxSpeed * maxSpeed / (2.0 * acceleration);
  const double decelDistance = maxSpeed * maxSpeed / (2.0 * deceleration);

  if (profile.accelDistance + decelDistance <= distance) {
    profile.peakSpeed = maxSpeed;
    profile.accelTime = profile.peakSpeed / acceleration;
    profile.decelTime = profile.peakSpeed / deceleration;
    profile.cruiseDistance =
        distance - profile.accelDistance - decelDistance;
    profile.cruiseTime = profile.cruiseDistance / profile.peakSpeed;
    return profile;
  }

  profile.peakSpeed =
      std::sqrt(2.0 * distance * acceleration * deceleration /
                (acceleration + deceleration));
  profile.accelTime = profile.peakSpeed / acceleration;
  profile.decelTime = profile.peakSpeed / deceleration;
  profile.accelDistance =
      profile.peakSpeed * profile.peakSpeed / (2.0 * acceleration);
  profile.cruiseDistance = 0.0;
  profile.cruiseTime = 0.0;
  return profile;
}

double profileDistanceAt(const MotionProfile &profile, double elapsed,
                         double distance, double acceleration,
                         double deceleration) {
  if (elapsed <= 0.0)
    return 0.0;

  if (elapsed < profile.accelTime)
    return 0.5 * acceleration * elapsed * elapsed;

  elapsed -= profile.accelTime;
  if (elapsed < profile.cruiseTime) {
    return profile.accelDistance + profile.peakSpeed * elapsed;
  }

  elapsed -= profile.cruiseTime;
  if (elapsed < profile.decelTime) {
    return profile.accelDistance + profile.cruiseDistance +
           profile.peakSpeed * elapsed -
           0.5 * deceleration * elapsed * elapsed;
  }

  return distance;
}

void advanceLateralMotion(double &offset, double &segmentStart,
                          double &segmentDistance, double &segmentElapsed,
                          double &direction, double amplitude, double maxSpeed,
                          double acceleration, double deceleration, double dt) {
  if (dt <= 0.0)
    return;

  direction = direction >= 0.0 ? 1.0 : -1.0;
  segmentElapsed += dt;

  MotionProfile profile =
      makeMotionProfile(segmentDistance, maxSpeed, acceleration, deceleration);
  while (segmentElapsed >= profile.duration()) {
    segmentElapsed -= profile.duration();
    offset = segmentStart + direction * segmentDistance;
    direction = -direction;
    segmentStart = offset;
    segmentDistance = 2.0 * amplitude;
    profile =
        makeMotionProfile(segmentDistance, maxSpeed, acceleration, deceleration);
  }

  const double segmentOffset =
      profileDistanceAt(profile, segmentElapsed, segmentDistance, acceleration,
                        deceleration);
  offset = segmentStart + direction * segmentOffset;
}

}  // namespace

int main() {
  try {
    webots::Supervisor robot;
    const int basicTimeStep = static_cast<int>(robot.getBasicTimeStep());
    const int timestep = getControlStepMs(basicTimeStep);
    const double spinRate = getEnvDouble("WEBOTS_TARGET_SPIN_RATE", 3.0);
    const double referenceFps =
        getEnvDouble("WEBOTS_TARGET_REFERENCE_FPS", 0.0);
    const bool lateralEnabled =
        getEnvBool("WEBOTS_TARGET_LATERAL_ENABLED", false);
    int lateralAxis = 1;
    double lateralAmplitude = 1.0;
    double lateralSpeed = 1.0;
    double lateralAcceleration = 3.0;
    double lateralDeceleration = 3.0;
    double lateralDirection = 1.0;
    if (lateralEnabled) {
      lateralAxis =
          getAxisIndex(getEnvString("WEBOTS_TARGET_LATERAL_AXIS", "y"));
      lateralAmplitude = getEnvDouble("WEBOTS_TARGET_LATERAL_AMPLITUDE", 1.5);
      lateralSpeed = getEnvDouble("WEBOTS_TARGET_LATERAL_SPEED", 1.0);
      lateralAcceleration = getEnvDouble("WEBOTS_TARGET_LATERAL_ACCEL", 3.0);
      lateralDeceleration =
          getEnvDouble("WEBOTS_TARGET_LATERAL_DECEL", 3.0);
      lateralDirection =
          getEnvDouble("WEBOTS_TARGET_LATERAL_INITIAL_DIRECTION", 1.0);
    }

    webots::Node *self = robot.getSelf();
    if (self == nullptr)
      throw std::runtime_error("target_spinner has no self node");

    webots::Field *rotationField = self->getField("rotation");
    if (rotationField == nullptr)
      throw std::runtime_error("target_spinner cannot access self rotation field");

    const double *currentRotation = rotationField->getSFRotation();
    double currentYaw = currentRotation != nullptr ? currentRotation[3] : 0.0;
    double rotation[4] = {0.0, 0.0, 1.0, currentYaw};

    webots::Field *translationField = nullptr;
    double translation[3] = {0.0, 0.0, 0.0};
    double lateralOffset = 0.0;
    double lateralSegmentStart = 0.0;
    double lateralSegmentDistance = lateralAmplitude;
    double lateralSegmentElapsed = 0.0;
    if (lateralEnabled) {
      translationField = self->getField("translation");
      if (translationField == nullptr)
        throw std::runtime_error(
            "target_spinner cannot access self translation field");

      const double *currentTranslation = translationField->getSFVec3f();
      if (currentTranslation == nullptr)
        throw std::runtime_error(
            "target_spinner cannot read self translation field");
      translation[0] = currentTranslation[0];
      translation[1] = currentTranslation[1];
      translation[2] = currentTranslation[2];
    }

    while (robot.step(timestep) != -1) {
      const double dt =
          referenceFps > 0.0 ? 1.0 / referenceFps : timestep / 1000.0;

      if (referenceFps > 0.0) {
        currentYaw += spinRate / referenceFps;
      } else {
        currentYaw += spinRate * timestep / 1000.0;
      }
      rotation[3] = currentYaw;
      rotationField->setSFRotation(rotation);

      if (lateralEnabled) {
        advanceLateralMotion(lateralOffset, lateralSegmentStart,
                             lateralSegmentDistance, lateralSegmentElapsed,
                             lateralDirection, lateralAmplitude, lateralSpeed,
                             lateralAcceleration, lateralDeceleration, dt);
        double nextTranslation[3] = {translation[0], translation[1],
                                     translation[2]};
        nextTranslation[lateralAxis] = translation[lateralAxis] + lateralOffset;
        translationField->setSFVec3f(nextTranslation);
      }
    }

    return 0;
  } catch (const std::exception &error) {
    std::cerr << "target_spinner controller error: " << error.what() << std::endl;
    return 1;
  }
}
