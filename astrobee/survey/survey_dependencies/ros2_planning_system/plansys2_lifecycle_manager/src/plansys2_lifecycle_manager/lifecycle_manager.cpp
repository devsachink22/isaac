// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <string>
#include <map>

//#include <lifecycle_msgs/State.h>
//#include <lifecycle_msgs/Transition.h>
//#include <lifecycle_msgs/ChangeState.h>
//#include <lifecycle_msgs/GetState.h>

#include <ros/ros.h>
//#include "rcutils/logging_macros.h"

#include <plansys2_lifecycle_manager/lifecycle_manager.hpp>

namespace plansys2
{

LifecycleServiceClient::LifecycleServiceClient(
  const std::string & node_name, const std::string & managed_node)
: nh_(node_name), managed_node_(managed_node)
{}

void
LifecycleServiceClient::init()
{
  /*
  std::string get_state_service_name = managed_node_ + "/get_state";
  std::string change_state_service_name = managed_node_ + "/change_state";
  RCLCPP_INFO(get_logger(), "Creating client for service [%s]", get_state_service_name.c_str());
  RCLCPP_INFO(
    get_logger(), "Creating client for service [%s]",
    change_state_service_name.c_str());
  client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(get_state_service_name);
  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    change_state_service_name);
  */
  lm_client_.reset(new ros::lifecycle::LifecycleClient(nh_, managed_node_));
}

unsigned int
LifecycleServiceClient::get_state(std::chrono::seconds time_out)
{
  return lm_client_->getState();
}


void LifecycleServiceClient::change_state_cb(bool result)
{
  result_ = result;
  received_result_ = true;
}

void LifecycleServiceClient::reset_result()
{
  result_ = false;
  received_result_ = false;
}
 
  bool LifecycleServiceClient::change_state(ros::lifecycle::State transition, std::chrono::seconds time_out)
{
  reset_result();
  lm_client_->goToState(transition,
			boost::bind(&LifecycleServiceClient::change_state_cb, this, _1));

  ros::Time ts = ros::Time::now();
  while( (ros::Time::now() - ts).toSec() < time_out.count())
  {
    ros::spinOnce();
    if(received_result_)
      break;
  }

  bool res;
  if(!received_result_)
    res = false;
  else
    res = result_;
  
  reset_result();
  
  return res;
}

bool
startup_function(
  std::map<std::string, std::shared_ptr<LifecycleServiceClient>> & manager_nodes,
  std::chrono::seconds timeout)
{
  // configure domain_expert
  {
    if (!manager_nodes["domain_expert"]->change_state(
						      ros::lifecycle::INACTIVE,
						      timeout))
    {
      return false;
    }

    while (manager_nodes["domain_expert"]->get_state() !=
	   ros::lifecycle::INACTIVE)
    {
      std::cerr << "Waiting for inactive state for domain_expert" << std::endl;
    }
  }

  // configure problem_expert
  {
    if (!manager_nodes["problem_expert"]->change_state(
						       ros::lifecycle::INACTIVE,
        timeout))
    {
      return false;
    }

    while (manager_nodes["problem_expert"]->get_state() !=
	   ros::lifecycle::INACTIVE)
    {
      std::cerr << "Waiting for inactive state for problem_expert" << std::endl;
    }
  }

  // configure planner
  {
    if (!manager_nodes["planner"]->change_state(
						ros::lifecycle::INACTIVE,
						timeout))
    {
      return false;
    }

    while (manager_nodes["planner"]->get_state() !=
	   ros::lifecycle::INACTIVE)
    {
      std::cerr << "Waiting for inactive state for planner" << std::endl;
    }
  }

  // configure executor
  {
    if (!manager_nodes["executor"]->change_state(
						 ros::lifecycle::INACTIVE,
						 timeout))
    {
      return false;
    }

    while (manager_nodes["executor"]->get_state() !=
	   ros::lifecycle::INACTIVE)
    {
      std::cerr << "Waiting for inactive state for planner" << std::endl;
    }
  }

  // activate
  {
    if (!ros::ok()) {
      return false;
    }
    if (!manager_nodes["domain_expert"]->change_state(
						      ros::lifecycle::ACTIVE,
						      timeout))
    {
      return false;
    }
    if (!manager_nodes["problem_expert"]->change_state(
						       ros::lifecycle::ACTIVE,
        timeout))
    {
      return false;
    }
    if (!manager_nodes["planner"]->change_state(
        ros::lifecycle::ACTIVE,
        timeout))
    {
      return false;
    }
    if (!manager_nodes["executor"]->change_state(
        ros::lifecycle::ACTIVE,
        timeout))
    {
      return false;
    }
    if (!manager_nodes["domain_expert"]->get_state()) {
      return false;
    }
    if (!manager_nodes["problem_expert"]->get_state()) {
      return false;
    }
    if (!manager_nodes["planner"]->get_state()) {
      return false;
    }
    if (!manager_nodes["executor"]->get_state()) {
      return false;
    }
  }
  return true;
}

}  // namespace plansys2
