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

#ifndef SUAVE_ROSA_BT__RECHARGE_BATTERY_HPP_
#define SUAVE_ROSA_BT__RECHARGE_BATTERY_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rosa_task_plan_bt/rosa_action.hpp"

using namespace std::placeholders;

namespace suave_rosa_bt
{

template<class T>
class RechargeBattery : public rosa_task_plan_bt::RosaAction<T>{

public:
  RechargeBattery(
    const std::string& name, const BT::NodeConfig & conf)
  : rosa_task_plan_bt::RosaAction<T>(name, conf)
  {
    battery_level_sub_  = this->node_->template create_subscription<std_msgs::msg::Bool>(
      "/battery_monitor/recharge/complete",
      10,
      std::bind(&RechargeBattery::battery_level_cb, this, _1));
  };

  BT::NodeStatus onStart(){
    std::cout << "Async action starting: " << this->name() << std::endl;
    // _completion_time = std::chrono::system_clock::now() + std::chrono::milliseconds(5000);
    recharged_ = false;
    return rosa_task_plan_bt::RosaAction<T>::onStart();
  };

  BT::NodeStatus onRunning() override {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if(this->node_->template time_limit_reached()){
      std::cout << "Time limit reached. Canceling action "<< this->name() << std::endl;
      this->cancel_action();
      return BT::NodeStatus::FAILURE;
    }

    if(recharged_ == true){
      std::cout << "Async action finished: "<< this->name() << std::endl;
      this->cancel_action();
      return BT::NodeStatus::SUCCESS;
    }
    std::cout<<"Recharging! "<<std::endl;
    return BT::NodeStatus::RUNNING;
  };

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
      });
  }

private:
  // std::chrono::system_clock::time_point _completion_time;
  bool recharged_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr battery_level_sub_;
  void battery_level_cb(const std_msgs::msg::Bool &msg){
    recharged_ = msg.data;
  };
};

}  // namespace suave_rosa_bt

#endif  // SUAVE_ROSA_BT__MOCK_RECHARGE_BATTERY_HPP_
