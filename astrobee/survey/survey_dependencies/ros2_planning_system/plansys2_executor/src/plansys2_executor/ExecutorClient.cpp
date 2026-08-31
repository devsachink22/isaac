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

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include <plansys2_executor/ExecutorClient.hpp>
#include <plansys2_msgs/ActionExecutionInfo.h>

namespace plansys2
{

using namespace std::chrono_literals;
using namespace std::placeholders;


ExecutorClient::ExecutorClient()
{
  node_ = std::make_shared<ros::NodeHandle>(ros::NodeHandle("executor_client"));

  createActionClient();

  std::string prefix = std::string("/executor/");
  get_ordered_sub_goals_client_ = node_->serviceClient<plansys2_msgs::GetOrderedSubGoals>(prefix + 
    "get_ordered_sub_goals");
  get_plan_client_ = node_->serviceClient<plansys2_msgs::GetPlan>(prefix + "get_plan");
}

ExecutorClient::ExecutorClient(const std::string & node_name)
{
  node_ = std::make_shared<ros::NodeHandle>(ros::NodeHandle(node_name));

  createActionClient();
  
  std::string prefix = std::string("/executor/");
  get_ordered_sub_goals_client_ = node_->serviceClient<plansys2_msgs::GetOrderedSubGoals>(prefix +
    "get_ordered_sub_goals");
  get_plan_client_ = node_->serviceClient<plansys2_msgs::GetPlan>(prefix + "get_plan");
}

void
ExecutorClient::createActionClient()
{
  std::string prefix = std::string("/executor/");
  std::string action_name = prefix + "execute_plan";
  action_client_.reset( new actionlib::SimpleActionClient<plansys2_msgs::ExecutePlanAction>(action_name, true));
  printf("Creating simple action client to: %s \n", action_name.c_str()); 
  if (!this->action_client_->waitForServer(ros::Duration(3.0))) {
    ROS_ERROR("%s -- Action server not available after waiting",
	      getName().c_str());
  }
}

bool
ExecutorClient::start_plan_execution(const plansys2_msgs::Plan & plan)
{
  if (!executing_plan_) {
    createActionClient();
    auto success = on_new_goal_received(plan);

    if (success) {
      executing_plan_ = true;
      return true;
    }
  } else {
    ROS_INFO("%s -- Already executing a plan", getName().c_str());
  }

  return false;
}

bool
ExecutorClient::execute_and_check_plan()
{
  if (ros::ok() && !goal_result_available_) {
    ros::spinOnce();

    if (!goal_result_available_) {
      return true;  // Plan not finished
    }
  }

  switch ( action_client_->getState().state_ ) {
  case actionlib::SimpleClientGoalState::SUCCEEDED:
    if (result_ == nullptr) {
        ROS_WARN("%s -- Plan failed due to a nullptr in the result", getName().c_str());
    }
    else
      ROS_WARN("%s -- Plan  seems to have succeeded", getName().c_str());
    /*else if (result_.result->success) {
        RCLCPP_INFO(node_->get_logger(), "Plan Succeeded");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Plan Failed");
        for (auto msg : result_.result->action_execution_status) {
          switch (msg.status) {
            case plansys2_msgs::ActionExecutionInfo::SUCCEEDED:
              RCLCPP_WARN_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " succeeded with message_status: " <<
                  msg.message_status);
              break;
            case plansys2_msgs::ActionExecutionInfo::FAILED:
              RCLCPP_ERROR_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " failed with message_status: " <<
                  msg.message_status);
              break;
            case plansys2_msgs::ActionExecutionInfo::NOT_EXECUTED:
              RCLCPP_WARN_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " was not executed");
              break;
            case plansys2_msgs::ActionExecutionInfo::CANCELLED:
              RCLCPP_WARN_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " was cancelled");
              break;
            case plansys2_msgs::ActionExecutionInfo::EXECUTING:
              RCLCPP_WARN_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " was executing");
          }
        }
	}*/
      break;

    case actionlib::SimpleClientGoalState::ABORTED:
      ROS_WARN("%s -- Plan Aborted", getName().c_str());
      break;

    case actionlib::SimpleClientGoalState::PREEMPTED:
      ROS_INFO("%s -- Plan Preempted", getName().c_str());
      break;

    default:
      throw std::logic_error("ExecutorClient::executePlan: invalid status value");
  }

  executing_plan_ = false;
  goal_result_available_ = false;

  return false;  // Plan finished
}


bool
ExecutorClient::on_new_goal_received(const plansys2_msgs::Plan & plan)
{
  auto goal = plansys2_msgs::ExecutePlanGoal();
  goal.plan = plan;

  action_client_->sendGoal(goal,
			   boost::bind(&ExecutorClient::result_callback, this, boost::placeholders::_1, boost::placeholders::_2),
			   actionlib::SimpleActionClient<plansys2_msgs::ExecutePlanAction>::SimpleActiveCallback(),
			   boost::bind(&ExecutorClient::feedback_callback, this, boost::placeholders::_1));


  return true;
}

bool
ExecutorClient::should_cancel_goal()
{
  if (!executing_plan_) {
    return false;
  }

  ros::spinOnce();
  auto status = action_client_->getState();

  return status == actionlib::SimpleClientGoalState::PENDING ||
    status == actionlib::SimpleClientGoalState::ACTIVE;
}

void
ExecutorClient::cancel_plan_execution()
{
  if (should_cancel_goal()) {
    action_client_->cancelGoal();
    /*
    if (rclcpp::spin_until_future_complete(
        node_->get_node_base_interface(), future_cancel, 3s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to cancel action server for execute_plan");
	}*/
  }

  executing_plan_ = false;
  goal_result_available_ = false;
}

std::vector<plansys2_msgs::Tree> ExecutorClient::getOrderedSubGoals()
{
  std::vector<plansys2_msgs::Tree> ret;

  while (!get_ordered_sub_goals_client_.waitForExistence(ros::Duration(5.0))) {
    if (!ros::ok()) {
      return ret;
    }
    ROS_ERROR_STREAM(getName() <<
		     get_ordered_sub_goals_client_.getService() <<
		     " service  client: waiting for service to appear...");
  }

  plansys2_msgs::GetOrderedSubGoals srv;

  if (!get_ordered_sub_goals_client_.call(srv))
  {
    return ret;
  }

  if (srv.response.success) {
    ret = srv.response.sub_goals;
  } else {
    ROS_INFO_STREAM(getName() <<
		    get_ordered_sub_goals_client_.getService() << ": " <<
		    srv.response.error_info);
  }

  return ret;
}

std::optional<plansys2_msgs::Plan> ExecutorClient::getPlan()
{
  while (!get_plan_client_.waitForExistence(ros::Duration(5.0))) {
    if (!ros::ok()) {
      return {};
    }
    ROS_ERROR_STREAM(getName() <<
		     get_plan_client_.getService() <<
		     " service  client: waiting for service to appear...");
  }

  plansys2_msgs::GetPlan srv;

  if (!get_plan_client_.call(srv))
  {
    return {};
  }

  if (srv.response.success) {
    return srv.response.plan;
  } else {
    ROS_ERROR_STREAM(getName() <<
		     get_plan_client_.getService() << ": " <<
		     srv.response.error_info);
    return {};
  }
}

void
ExecutorClient::feedback_callback( const plansys2_msgs::ExecutePlanFeedbackConstPtr& feedback)
{
  feedback_ = *feedback;
}

void
ExecutorClient::result_callback(const actionlib::SimpleClientGoalState& state,
				const plansys2_msgs::ExecutePlanResultConstPtr& result)
{
  goal_result_available_ = true;
  result_ = result;
  feedback_ = plansys2_msgs::ExecutePlanFeedback();
}

  std::optional<plansys2_msgs::ExecutePlanResult>
ExecutorClient::getResult()
{
  if(result_)
    return *result_;
  else
    return {};

}

}  // namespace plansys2
