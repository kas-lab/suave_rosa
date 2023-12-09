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

#include "suave_rosa/suave_mission.hpp"

namespace suave_rosa
{
  using namespace std::placeholders;

  SuaveMission::SuaveMission(std::string none_name): Node(none_name){
    distance_inspected_sub_  = this->create_subscription<std_msgs::msg::Float32>(
      "/pipeline/distance_inspected",
      10,
      std::bind(&SuaveMission::distance_inspected_cb, this, _1));

    this->declare_parameter("time_limit", 300);
    this->declare_parameter("result_path", "~/suave/results");
    this->declare_parameter("result_filename", "mission_results");

    _time_limit = this->get_parameter("time_limit").as_int();
  }

  void SuaveMission::distance_inspected_cb(const std_msgs::msg::Float32 &msg){
    _distance_inspected = msg.data;
  }

  bool SuaveMission::time_limit_reached(){
    if(_search_started){
      return (this->get_clock()->now() - start_time) >= rclcpp::Duration(_time_limit, 0);
    }
    return false;
  }

  void SuaveMission::save_mission_result(){
    // TODO: save csv with the following header
    // self.metrics_header = [
    // 'mission_name' == _mission_name + now_str,
    // 'datetime' == now_str,
    // 'initial pos (x,y)' I think this can be ingored for now,
    // 'time budget (s)', == time_limit
    // 'time search (s)' == end_search_time - start_time,
    // 'distance inspected (m)' == distance_inspected]
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    auto now_str = oss.str();

    auto result_path = this->get_parameter("result_path").as_string();
    auto result_filename = this->get_parameter("result_filename").as_string();
  }
}
