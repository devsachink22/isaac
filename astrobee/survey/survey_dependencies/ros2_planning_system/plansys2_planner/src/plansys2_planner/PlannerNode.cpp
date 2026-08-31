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

#include <string>
#include <memory>
#include <iostream>
#include <fstream>

#include <plansys2_planner/PlannerNode.hpp>
#include <plansys2_popf_plan_solver/popf_plan_solver.hpp>

//#include "lifecycle_msgs/msg/state.hpp"

namespace plansys2
{

  PlannerNode::PlannerNode(ros::NodeHandle _nh)
  : ros::lifecycle::ManagedNode(_nh),
  lp_loader_("plansys2_core", "plansys2::PlanSolverBase"),
  default_ids_{},
  default_types_{}
{ 
  std::string prefix(""); //("planner/");
  get_plan_service_ = getBaseNode().advertiseService(prefix + "get_plan",
						     &PlannerNode::get_plan_service_callback,
						     this);

  // No declaration needed in ROS1
  //declare_parameter("plan_solver_plugins", default_ids_);
}



bool
PlannerNode::onConfigure()
{
  ROS_INFO("[%s] Configuring...", get_name().c_str());
  
  std::string prefix = "planner/"; // "";
  getBaseNode().getParam(prefix + "plan_solver_plugins", solver_ids_);
  
  auto node = shared_from_this();

  if (!solver_ids_.empty()) {
    /*
    if (solver_ids_ == default_ids_) {
      for (size_t i = 0; i < default_ids_.size(); ++i) {
        plansys2::declare_parameter_if_not_declared(
						    node, default_ids_[i] + ".plugin",
						    rclcpp::ParameterValue(default_types_[i]));
      }
    }*/
    solver_types_.resize(solver_ids_.size());
    printf("Solver types size: %d \n", solver_types_.size());
    for (size_t i = 0; i != solver_types_.size(); i++) {
      try {
        //solver_types_[i] = plansys2::get_plugin_type_param(node, solver_ids_[i]);
	getBaseNode().getParam(prefix + solver_ids_[i] + "/plugin", solver_types_[i]);
	if(solver_types_[i].empty())
	  continue;

	plansys2::PlanSolverBase::Ptr solver = lp_loader_.createUniqueInstance(solver_types_[i]);
        solver->configure(node, solver_ids_[i]);

        ROS_INFO("%s -- Created solver : %s of type %s",
		 get_name().c_str(),
		 solver_ids_[i].c_str(), solver_types_[i].c_str());
        solvers_.insert({solver_ids_[i], solver});
      } catch (const pluginlib::PluginlibException & ex) {
        ROS_FATAL("%s -- Failed to create solver. Exception: %s", get_name().c_str(), ex.what());
        exit(-1);
      }
    }
  } else {
    auto default_solver = std::make_shared<plansys2::POPFPlanSolver>();
    default_solver->configure(node, "POPF");
    solvers_.insert({"POPF", default_solver});
    ROS_INFO("%s -- Created default solver : %s of type %s",
	     get_name().c_str(),
	     "POPF", "plansys2/POPFPlanSolver");
  }

  ROS_INFO("[%s] Configured", get_name().c_str());
  return true;
}

bool
PlannerNode::onActivate()
{
  ROS_INFO("[%s] Activating...", get_name().c_str());
  ROS_INFO("[%s] Activated", get_name().c_str());
  return true;
}

bool
PlannerNode::onDeactivate()
{
  ROS_INFO("[%s] Deactivating...", get_name().c_str());
  ROS_INFO("[%s] Deactivated", get_name().c_str());

  return true;
}

bool
PlannerNode::onCleanup()
{
  ROS_INFO("[%s] Cleaning up...", get_name().c_str());
  ROS_INFO("[%s] Cleaned up", get_name().c_str());

  return true;
}

bool
PlannerNode::onShutdown()
{
  ROS_INFO("[%s] Shutting down...", get_name().c_str());
  ROS_INFO("[%s] Shutted down", get_name().c_str());

  return true;
}

bool
PlannerNode::onError(const std::exception &)
{
  ROS_ERROR("[%s] Error transition", get_name().c_str());

  return false;
}


bool
PlannerNode::get_plan_service_callback(
  plansys2_msgs::GetPlan::Request &request,
  plansys2_msgs::GetPlan::Response &response)
{
  auto plan = solvers_.begin()->second->getPlan(request.domain,
						request.problem,
						get_namespace());

  if (plan) {
    response.success = true;
    response.plan = plan.value();
  } else {
    response.success = false;
    response.error_info = "Plan not found";
  }

  return true;
}

}  // namespace plansys2
