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

#ifndef SUAVE_ROSA__SUAVE_MISSION_HPP_
#define SUAVE_ROSA__SUAVE_MISSION_HPP_

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace suave_rosa
{

class SuaveMission : public rclcpp::Node{

public:
  SuaveMission(std::string none_name);
  rclcpp::Time start_time;
  rclcpp::Time end_search_time;
  rclcpp::Time end_time;

  bool time_limit_reached();
  void save_mission_result();

private:
  std::string _mission_name = "time_constrained_rosa";
  bool _search_started = false;
  double _distance_inspected;
  int _time_limit;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_inspected_sub_;
  void distance_inspected_cb(const std_msgs::msg::Float32 &msg);
};

}  // namespace suave_rosa

#endif  // SUAVE_ROSA__SUAVE_MISSION_HPP_
