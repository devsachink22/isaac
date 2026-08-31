// Copyright 2019 Intelligent Robotics Lab
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

#ifndef PLANSYS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define PLANSYS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <map>

//#include "lifecycle_msgs/msg/state.hpp"
//#include "lifecycle_msgs/msg/transition.hpp"
//#include "lifecycle_msgs/srv/change_state.hpp"
//#include "lifecycle_msgs/srv/get_state.hpp"
#include <lifecycle/client.h>

#include <ros/ros.h>

//#include "rcutils/logging_macros.h"

namespace plansys2
{


class LifecycleServiceClient 
{
public:
  LifecycleServiceClient(const std::string & node_name,
			 const std::string & managed_node);

  void init();
  unsigned int get_state(std::chrono::seconds time_out = std::chrono::seconds(3));
  bool change_state(ros::lifecycle::State transition,
		    std::chrono::seconds time_out = std::chrono::seconds(3));

protected:
  void change_state_cb(bool result);
  void reset_result();
  
private:
  //std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  //std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;

  std::string managed_node_;
  ros::NodeHandle nh_;

  std::shared_ptr<ros::lifecycle::LifecycleClient> lm_client_;
  bool result_;
  bool received_result_;
};

bool
startup_function(
  std::map<std::string, std::shared_ptr<LifecycleServiceClient>> & manager_nodes,
  std::chrono::seconds timeout);

}  // namespace plansys2

#endif  // PLANSYS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
