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

#ifndef SUAVE_ROSA_BT__SEARCH_PIPELINE_HPP_
#define SUAVE_ROSA_BT__SEARCH_PIPELINE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rosa_plan/rosa_action.hpp"

using namespace std::placeholders;

namespace suave_rosa_bt
{

template<class T>
class SearchPipeline : public rosa_plan::RosaAction<T>{

public:
  SearchPipeline(
    const std::string& name, const BT::NodeConfig & conf)
  : rosa_plan::RosaAction<T>(name, conf), _pipeline_detected(false)
  {
    pipeline_detection_sub_  = this->_node->template create_subscription<std_msgs::msg::Bool>(
      "/pipeline/detected",
      10,
      std::bind(&SearchPipeline::pipeline_detected_cb, this, _1));
  };

  BT::NodeStatus onStart(){
    this->_node->template set_search_started();
    return rosa_plan::RosaAction<T>::onStart();
  };

  BT::NodeStatus onRunning() override{
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if(this->_node->template time_limit_reached()){
      std::cout << "Time limit reached. Canceling action "<< this->name() << std::endl;
      this->cancel_action();
      return BT::NodeStatus::FAILURE;
    }

    if(_pipeline_detected == true){
      std::cout << "Async action finished: "<< this->name() << std::endl;
      this->cancel_action();
      return BT::NodeStatus::SUCCESS;
    }
    std::cout<<"Searching for pipeline! "<<std::endl;
    return BT::NodeStatus::RUNNING;
  };

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
      });
  }

protected:
  bool _pipeline_detected;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pipeline_detection_sub_;
  void pipeline_detected_cb(const std_msgs::msg::Bool &msg){
    _pipeline_detected = msg.data;
  };
};

}  // namespace suave_rosa_bt

#endif  // SUAVE_ROSA_BT__SEARCH_PIPELINE_HPP_
