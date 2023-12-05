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

#include "suave_rosa/action_search_pipeline.hpp"

namespace suave_rosa
{
  using namespace std::placeholders;

  SearchPipeline::SearchPipeline(
    const std::string& name, const BT::NodeConfig & conf)
  : rosa_plan::RosaAction(name, conf), _pipeline_detected(false)
  {
    pipeline_detection_sub_  = node_->create_subscription<std_msgs::msg::Bool>(
      "/pipeline/detected",
      10,
      std::bind(&SearchPipeline::pipeline_detected_cb, this, _1));
  }

  void
  SearchPipeline::pipeline_detected_cb(const std_msgs::msg::Bool &msg)
  {
    _pipeline_detected = msg.data;
  }

  BT::NodeStatus SearchPipeline::onStart(){
    return rosa_plan::RosaAction::onStart();
  }

  void SearchPipeline::onHalted(){
    rosa_plan::RosaAction::onHalted();
  }

  BT::NodeStatus SearchPipeline::onRunning(){
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if(_pipeline_detected == true){
      std::cout << "Async action finished: "<< this->name() << std::endl;
      cancel_task();
      return BT::NodeStatus::SUCCESS;
    }
    std::cout<<"Searching for pipeline! "<<std::endl;
    return BT::NodeStatus::RUNNING;
  }
} //namespace suave_rosa
