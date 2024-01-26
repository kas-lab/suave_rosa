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

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include "rclcpp/rclcpp.hpp"

#include "suave_rosa/action_recharge_battery.hpp"
#include "suave_rosa/action_search_pipeline.hpp"
#include "suave_rosa/action_inspect_pipeline.hpp"
#include "suave_rosa/arm_thrusters.hpp"
#include "suave_rosa/set_guided_mode.hpp"
#include "suave_rosa/suave_mission.hpp"
#include "rosa_plan/is_action_feasible.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<suave_rosa::SuaveMission> node = std::make_shared<suave_rosa::SuaveMission>("mission_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerNodeType<suave_rosa::SearchPipeline<std::shared_ptr<suave_rosa::SuaveMission>>>("search_pipeline");
  factory.registerNodeType<suave_rosa::InspectPipeline<std::shared_ptr<suave_rosa::SuaveMission>>>("inspect_pipeline");
  factory.registerNodeType<suave_rosa::RechargeBattery<std::shared_ptr<suave_rosa::SuaveMission>>>("recharge");

  factory.registerNodeType<rosa_plan::IsActionFeasible<std::shared_ptr<suave_rosa::SuaveMission>>>("IsActionFeasible");

  factory.registerNodeType<suave_rosa::ArmThrusters>("ArmThrusters");
  factory.registerNodeType<suave_rosa::SetGuidedMode>("SetGuidedMode");

  std::string pkgpath = ament_index_cpp::get_package_share_directory("suave_rosa");
  std::string xml_file = pkgpath + "/bts/suave.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::shared_ptr<suave_rosa::SuaveMission>>("node", node);

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    if(node->time_limit_reached()){
      node->request_save_mission_results();
      tree.haltTree();
      finish = true;
    } else{
      finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    }
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
