// Copyright 2024 Gustavo Rezende Silva
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
#include <ctime>

#include "rosa_task_plan_plansys/rosa_plansys_controller.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/empty.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

class SuaveRosaController : public rosa_task_plan_plansys::RosaPlansysController
{
public:
  SuaveRosaController(const std::string & node_name)
    : RosaPlansysController(node_name){
      this->declare_parameter("time_limit", 300);
      _time_limit = this->get_parameter("time_limit").as_int();

      save_mission_results_cli =
        this->create_client<std_srvs::srv::Empty>("mission_metrics/save");

      search_pipeline_transition_sub_  = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
        "/search_pipeline/transition_event",
        10,
        std::bind(&SuaveRosaController::search_pipeline_transition_cb_, this, _1));

      time_limit_timer_cb_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
      // TODO: create parameter for timer rate?
      time_limit_timer_ = this->create_wall_timer(
        100ms, std::bind(&SuaveRosaController::time_limit_cb, this), time_limit_timer_cb_group_);
  };

private:

  void time_limit_cb(){
    if(_search_started && (this->get_clock()->now() - _start_time) >= rclcpp::Duration(_time_limit, 0)){
      RCLCPP_INFO(this->get_logger(), "Time limit reached!");
      this->finish_controlling();
    }
  };

  bool request_save_mission_results(){
    while (!save_mission_results_cli->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "mission_metrics/save service not available, waiting again...");
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto response = save_mission_results_cli->async_send_request(request);
    if (response.wait_for(1s) != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service mission_metrics/save");
      return false;
    }
    return true;
  }

  void search_pipeline_transition_cb_(const lifecycle_msgs::msg::TransitionEvent &msg){
    if(msg.goal_state.id == 3){
      _start_time = this->get_clock()->now();
      _search_started = true;
    }
  }

  void finish_controlling() override{
    this->step_timer_->cancel();
    this->executor_client_->cancel_plan_execution();
    this->request_save_mission_results();
    this->time_limit_timer_->cancel();
  }

  rclcpp::Time _start_time;
  bool _search_started = false;
  int _time_limit;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr save_mission_results_cli;

  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr search_pipeline_transition_sub_;

  rclcpp::CallbackGroup::SharedPtr time_limit_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr time_limit_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SuaveRosaController>(
    "suave_rosa_controller");

  rclcpp::Rate rate(5);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
}
