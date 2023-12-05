// Copyright 2023 Gustavo Rezende Silva
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

#ifndef SUAVE_ROSA__RECHARGE_BATTERY_HPP_
#define SUAVE_ROSA__RECHARGE_BATTERY_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rosa_plan/rosa_action.hpp"

namespace suave_rosa
{

class RechargeBattery : public rosa_plan::RosaAction{

public:
  RechargeBattery(const std::string& name, const BT::NodeConfig & conf);

  BT::NodeStatus onStart();

  BT::NodeStatus onRunning() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
      });
  }

private:
  std::chrono::system_clock::time_point _completion_time;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr battery_charged_pub_;
};

}  // namespace suave_rosa

#endif  // SUAVE_ROSA__MOCK_RECHARGE_BATTERY_HPP_
