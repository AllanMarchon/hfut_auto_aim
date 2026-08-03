// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GIMBAL_CONTROLLER__GIMBAL_CONTROL_CORE_HPP_
#define GIMBAL_CONTROLLER__GIMBAL_CONTROL_CORE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "gimbal_controller/fire_advice_engine.hpp"
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_controller/gimbal_cmd_filter.hpp"
#include "gimbal_controller/gimbal_control_orchestrator.hpp"
#include "gimbal_controller/gimbal_control_strategy.hpp"

namespace gimbal_controller
{

struct GimbalControlCoreOutput
{
  rm_interfaces::msg::GimbalCmd cmd;
  DelayAuditSnapshot delay_audit{};
  ControlTargetDebugSnapshot control_target_debug{};
  MpcDebugSnapshot mpc_debug{};
  FireAdviceDebugSnapshot fire_advice_debug{};
  bool has_tracking{false};
  bool strategy_found{true};
};

class GimbalControlCore
{
public:
  using StrategyMap = std::unordered_map<std::string, GimbalControlStrategy::SharedPtr>;

  void setStrategies(const StrategyMap * strategies)
  {
    strategies_ = strategies;
  }

  void setFireModules(
    std::shared_ptr<FireAdviceEngine> fire_advice_engine,
    std::shared_ptr<FireAdvisor> fire_advisor)
  {
    orchestrator_.setFireModules(fire_advice_engine, fire_advisor);
  }

  void setFireDecisionConfig(const FireDecisionConfig & config)
  {
    orchestrator_.setFireDecisionConfig(config);
  }

  void setFilterConfig(const GimbalCmdFilterConfig & config)
  {
    filter_.setConfig(config);
  }

  // Seconds to hold the last aimed gimbal direction after the target becomes
  // unavailable, before falling back to the zeroed (home) idle command. A
  // sweeping/oscillating target re-enters a FOV held near its last known
  // sector far sooner than the home one; <= 0 restores plain homing.
  void setIdleHold(double seconds) { idle_hold_s_ = seconds; }

  GimbalControlCoreOutput compute(
    const GimbalControlContext & context,
    const std::string & strategy_name,
    const std::string & selected_target_id,
    bool enable);

  void updateFov(double fov_half_yaw, double fov_half_pitch);

private:
  GimbalControlStrategy::SharedPtr findStrategy(const std::string & name) const;

  StrategyMap const * strategies_{nullptr};
  GimbalControlOrchestrator orchestrator_;
  GimbalCmdFilter filter_;
  std::string prev_tracking_target_id_;
  double idle_hold_s_ = 0.0;
  bool have_last_aim_ = false;
  double last_aim_yaw_ = 0.0;
  double last_aim_pitch_ = 0.0;
  double last_aim_time_s_ = 0.0;
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__GIMBAL_CONTROL_CORE_HPP_
