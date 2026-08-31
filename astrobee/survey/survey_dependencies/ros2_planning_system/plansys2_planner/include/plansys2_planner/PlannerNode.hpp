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

#ifndef PLANSYS2_PLANNER__PLANNERNODE_HPP_
#define PLANSYS2_PLANNER__PLANNERNODE_HPP_

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

#include <plansys2_domain_expert/DomainExpertClient.hpp>
#include <plansys2_problem_expert/ProblemExpertClient.hpp>

#include <plansys2_core/PlanSolverBase.hpp>

#include <std_msgs/Empty.h>
//#include <lifecycle_msgs/msg/state.h>
//#include <lifecycle_msgs/msg/transition.h>
#include <plansys2_msgs/GetPlan.h>

#include <ros/ros.h>
#include <lifecycle/managed_node.h>

#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace plansys2
{

class PlannerNode : public ros::lifecycle::ManagedNode
{
public:
  PlannerNode(ros::NodeHandle _nh);

  using SolverMap = std::unordered_map<std::string, plansys2::PlanSolverBase::Ptr>;

  bool onConfigure();
  bool onActivate();
  bool onDeactivate();
  bool onCleanup();
  bool onShutdown();
  bool onError(const std::exception &);

  bool get_plan_service_callback(plansys2_msgs::GetPlan::Request &request,
				 plansys2_msgs::GetPlan::Response &response);

private:
  pluginlib::ClassLoader<plansys2::PlanSolverBase> lp_loader_;
  SolverMap solvers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> solver_ids_;
  std::vector<std::string> solver_types_;

  ros::ServiceServer get_plan_service_;
};

  /*
template<typename NodeT>
void declare_parameter_if_not_declared(
				       NodeT node,
				       const std::string & param_name,
				       const rclcpp::ParameterValue & default_value = rclcpp::ParameterValue(),
				       const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
				       rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
  }*/

   /*
template<typename NodeT>
std::string get_plugin_type_param(
  NodeT node,
  const std::string & plugin_name)
{
  declare_parameter_if_not_declared(node, plugin_name + ".plugin", rclcpp::ParameterValue(""));
  std::string plugin_type;
  if (!node->get_parameter(plugin_name + ".plugin", plugin_type)) {
    RCLCPP_FATAL(node->get_logger(), "'plugin' param not defined for %s", plugin_name.c_str());
    exit(-1);
  }
  return plugin_type;
  }*/

}  // namespace plansys2

#endif  // PLANSYS2_PLANNER__PLANNERNODE_HPP_
