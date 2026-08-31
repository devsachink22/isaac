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

#ifndef PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_
#define PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_

#include <optional>
#include <string>
#include <memory>
#include <vector>

#include <plansys2_msgs/ExecutePlanAction.h>
#include <plansys2_msgs/GetOrderedSubGoals.h>
#include <plansys2_msgs/GetPlan.h>
#include <plansys2_msgs/Plan.h>
#include <plansys2_msgs/Tree.h>
#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>
//#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2
{

class ExecutorClient
{
public:

  ExecutorClient();
  explicit ExecutorClient(const std::string & node_name);

  bool start_plan_execution(const plansys2_msgs::Plan & plan);
  bool execute_and_check_plan();
  void cancel_plan_execution();
  std::vector<plansys2_msgs::Tree> getOrderedSubGoals();
  std::optional<plansys2_msgs::Plan> getPlan();

  plansys2_msgs::ExecutePlanFeedback getFeedBack() {return feedback_;}
  std::optional<plansys2_msgs::ExecutePlanResult> getResult();

  std::string getName() { return std::string("ExecutorClient"); }
  
private:
  std::shared_ptr<ros::NodeHandle> node_;

  std::shared_ptr<actionlib::SimpleActionClient<plansys2_msgs::ExecutePlanAction> > action_client_;
  ros::ServiceClient get_ordered_sub_goals_client_;
  ros::ServiceClient get_plan_client_;

  plansys2_msgs::ExecutePlanFeedback feedback_;
  plansys2_msgs::ExecutePlanResultConstPtr result_;

  bool goal_result_available_{false};

  bool executing_plan_{false};

  void result_callback(const actionlib::SimpleClientGoalState& state,
		       const plansys2_msgs::ExecutePlanResultConstPtr& result);
  void feedback_callback(const plansys2_msgs::ExecutePlanFeedbackConstPtr& feedback);

  bool on_new_goal_received(const plansys2_msgs::Plan & plan);
  bool should_cancel_goal();
  void createActionClient();
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_
