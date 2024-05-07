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

#ifndef SUAVE_ROSA_BT__SUAVE_MISSION_HPP_
#define SUAVE_ROSA_BT__SUAVE_MISSION_HPP_

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/empty.hpp"

namespace suave_rosa_bt
{

class SuaveMission : public rclcpp::Node{

public:
  SuaveMission(std::string none_name);

  bool time_limit_reached();

  bool request_save_mission_results();
  void set_search_started();

private:
  rclcpp::Time _start_time;
  bool _search_started = false;
  int _time_limit;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr save_mission_results_cli;
};

}  // namespace suave_rosa_bt

#endif  // SUAVE_ROSA_BT__SUAVE_MISSION_HPP_
