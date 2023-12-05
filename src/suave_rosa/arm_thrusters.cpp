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

#include "suave_rosa/arm_thrusters.hpp"


namespace suave_rosa
{
  using namespace std::placeholders;
  using namespace std::chrono_literals;

  ArmThrusters::ArmThrusters(
    const std::string& name, const BT::NodeConfig & conf)
  : BT::StatefulActionNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    arm_motors_cli_ = node_->create_client<mavros_msgs::srv::CommandBool>(
      "mavros/cmd/arming");
  }

  BT::NodeStatus ArmThrusters::onRunning(){

    if(arm_motors_cli_->service_is_ready()){
      auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      request->value = true;
      auto arm_motors_result_ = arm_motors_cli_->async_send_request(request);

      // Wait for the result.
      if (rclcpp::spin_until_future_complete(node_, arm_motors_result_, std::chrono::milliseconds(100)) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        auto arm_result_ = arm_motors_result_.get();
        if(!arm_result_->success){
          return BT::NodeStatus::FAILURE;
        }

        if(arm_result_->success){
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Thrusters armed!");
          return BT::NodeStatus::SUCCESS;
        }
      } else {
        return BT::NodeStatus::FAILURE;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mavros/cmd/arming service not available, waiting again...");
    return BT::NodeStatus::RUNNING;
  }
} //namespace suave_rosa
