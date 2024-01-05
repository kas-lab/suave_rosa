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

#include "suave_rosa/action_recharge_battery.hpp"

namespace suave_rosa
{
  using namespace std::placeholders;
  RechargeBattery::RechargeBattery(
    const std::string& name, const BT::NodeConfig & conf)
  : rosa_plan::RosaAction(name, conf)
  {
    battery_level_sub  = node_->create_subscription<std_msgs::msg::Bool>(
      "/battery_monitor/recharge/complete",
      10,
      std::bind(&RechargeBattery::battery_level_cb, this, _1));
  }

  void
  RechargeBattery::battery_level_cb(const std_msgs::msg::Bool &msg)
  {
    _recharged = msg.data;
  }

  BT::NodeStatus RechargeBattery::onStart(){
    std::cout << "Async action starting: " << this->name() << std::endl;
    // _completion_time = std::chrono::system_clock::now() + std::chrono::milliseconds(5000);
    _recharged = false;
    return rosa_plan::RosaAction::onStart();
  }

  BT::NodeStatus RechargeBattery::onRunning(){
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if(_recharged == true){
      std::cout << "Async action finished: "<< this->name() << std::endl;
      cancel_action();
      return BT::NodeStatus::SUCCESS;
    }
    std::cout<<"Recharging! "<<std::endl;
    return BT::NodeStatus::RUNNING;
  }
} //namespace suave_rosa
