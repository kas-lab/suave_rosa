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

class InspectPipelineAction : public rosa_task_plan_plansys::RosaAction{
public:
  InspectPipelineAction(const std::string & node_name,
    const std::chrono::nanoseconds & rate) : RosaAction(node_name, rate){
      pipeline_inspected_sub_  = this->create_subscription<std_msgs::msg::Bool>(
        "/pipeline/inspected",
        10,
        std::bind(&InspectPipelineAction::pipeline_inspected_cb, this, _1));
  };

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pipeline_inspected_sub_;

  bool _pipeline_inspected=false;
  void pipeline_inspected_cb(const std_msgs::msg::Bool &msg){
    _pipeline_inspected = msg.data;
  };

  void do_work(){
    if(_pipeline_inspected==true) finish(true, 1.0, "Pipeline inspected!");
  };
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rosa_task_plan_plansys::RosaAction>(
    "inspect_pipeline", 500ms);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
