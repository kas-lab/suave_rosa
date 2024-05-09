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
#include "std_msgs/msg/bool.hpp"
#include "rosa_task_plan_plansys/rosa_action.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class RechargeBatteryAction : public rosa_task_plan_plansys::RosaAction{
public:
  RechargeBatteryAction(const std::string & node_name,
    const std::chrono::nanoseconds & rate) : RosaAction(node_name, rate){
      battery_level_sub  = this->create_subscription<std_msgs::msg::Bool>(
        "/battery_monitor/recharge/complete",
        10,
        std::bind(&RechargeBatteryAction::battery_level_cb, this, _1));
  };

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state){
    _recharged = false;
    return rosa_task_plan_plansys::RosaAction::on_activate(previous_state);
  };

private:
  bool _recharged;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr battery_level_sub;
  void battery_level_cb(const std_msgs::msg::Bool &msg){
    _recharged = msg.data;
  };

  void do_work(){
    if(_recharged==true) finish(true, 1.0, "Battery recharged!");
  };
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RechargeBatteryAction>(
    "recharge", 500ms);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
