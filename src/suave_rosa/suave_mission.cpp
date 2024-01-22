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
  using namespace std::chrono_literals;

  SuaveMission::SuaveMission(std::string none_name): Node(none_name){
    this->declare_parameter("time_limit", 300);
    _time_limit = this->get_parameter("time_limit").as_int();

    save_mission_results_cli =
      this->create_client<std_srvs::srv::Empty>("mission_metrics/save");
  }


  bool SuaveMission::time_limit_reached(){
    if(_search_started){
      std::cout << "Time limit reached!"<< std::endl;
      return (this->get_clock()->now() - _start_time) >= rclcpp::Duration(_time_limit, 0);
    }
    return false;
  }

  bool SuaveMission::request_save_mission_results(){
    while (!save_mission_results_cli->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "mission_metrics/save service not available, waiting again...");
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto response = save_mission_results_cli->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), response) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service mission_metrics/save");
      return false;
    }
  }

  void SuaveMission::set_search_started(){
    _start_time = this->get_clock()->now();
    _search_started = true;
  }

}
