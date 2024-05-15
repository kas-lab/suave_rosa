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

#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/json_export.h"

#include "rclcpp/rclcpp.hpp"

#include "suave_bt/action_arm_thrusters.hpp"
#include "suave_bt/action_set_guided_mode.hpp"
#include "suave_bt/suave_mission.hpp"
#include "suave_bt/is_pipeline_found.hpp"
#include "suave_bt/is_pipeline_inspected.hpp"

#include "suave_rosa_bt/action_recharge_battery.hpp"
#include "suave_rosa_bt/action_search_pipeline.hpp"
#include "suave_rosa_bt/action_inspect_pipeline.hpp"
#include "rosa_task_plan_bt/is_action_feasible.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<suave_bt::SuaveMission> node = std::make_shared<suave_bt::SuaveMission>("mission_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerNodeType<suave_rosa_bt::SearchPipeline<std::shared_ptr<suave_bt::SuaveMission>>>("search_pipeline");
  factory.registerNodeType<suave_rosa_bt::InspectPipeline<std::shared_ptr<suave_bt::SuaveMission>>>("inspect_pipeline");
  factory.registerNodeType<suave_rosa_bt::RechargeBattery<std::shared_ptr<suave_bt::SuaveMission>>>("recharge");

  factory.registerNodeType<rosa_task_plan_bt::IsActionFeasible<std::shared_ptr<suave_bt::SuaveMission>>>("is_action_feasible");
  factory.registerNodeType<suave_bt::IsPipelineFound>("is_pipeline_found");
  factory.registerNodeType<suave_bt::IsPipelineInspected>("is_pipeline_inspected");

  factory.registerNodeType<suave_bt::ArmThrusters>("arm_thrusters");
  factory.registerNodeType<suave_bt::SetGuidedMode>("set_guided_mode");

  factory.registerSimpleCondition(
    "is_mission_aborted",
    [&](const auto&) { return node->is_mission_aborted() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; });

  std::string pkgpath = ament_index_cpp::get_package_share_directory("suave_rosa_bt");
  std::string xml_file = pkgpath + "/bts/suave_extended.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::shared_ptr<suave_bt::SuaveMission>>("node", node);

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  const unsigned port = 1667;
  BT::Groot2Publisher publisher(tree, port);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  std::thread t([&executor]() {
    executor.spin();
  });

  bool finish = false;
  while (!finish & rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    std::this_thread::sleep_for(100ms);
  }

  if(!node->is_mission_aborted()) node->finish_mission();
  std::this_thread::sleep_for(1s);

  executor.cancel();
  t.join();
  rclcpp::shutdown();

  return 0;
}
