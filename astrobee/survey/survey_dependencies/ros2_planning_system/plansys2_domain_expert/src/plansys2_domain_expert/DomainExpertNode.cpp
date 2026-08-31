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

#include "plansys2_domain_expert/DomainExpertNode.hpp"

#include <string>
#include <memory>
#include <vector>

#include <plansys2_core/Utils.hpp>
#include <plansys2_popf_plan_solver/popf_plan_solver.hpp>

//#include "lifecycle_msgs/msg/state.hpp"

namespace plansys2
{

  DomainExpertNode::DomainExpertNode(const ros::NodeHandle &_nh)
  : ros::lifecycle::ManagedNode(_nh)
{
  node_name_ = std::string("DomainExpertNode");

  // ROS1 : No need to declare parameter
  //declare_parameter("model_file", "");

  std::string prefix(""); //("domain_expert/");
  get_name_service_ = getBaseNode().advertiseService( prefix + "get_domain_name",
						     &DomainExpertNode::get_domain_name_service_callback,
						     this);
  
  get_types_service_ = getBaseNode().advertiseService( prefix + "get_domain_types",
						      &DomainExpertNode::get_domain_types_service_callback,
						      this);
  get_domain_actions_service_ = getBaseNode().advertiseService( prefix + "get_domain_actions",
							       &DomainExpertNode::get_domain_actions_service_callback,
							       this);
  get_domain_action_details_service_ = getBaseNode().advertiseService( prefix + "get_domain_action_details",
								      &DomainExpertNode::get_domain_action_details_service_callback,
								      this);
  get_domain_durative_actions_service_ = getBaseNode().advertiseService( prefix + "get_domain_durative_actions",
									&DomainExpertNode::get_domain_durative_actions_service_callback,
									this);
  get_domain_durative_action_details_service_ = getBaseNode().advertiseService( prefix + "get_domain_durative_action_details",
									       &DomainExpertNode::get_domain_durative_action_details_service_callback,
									       this);
  get_domain_predicates_service_ = getBaseNode().advertiseService( prefix + "get_domain_predicates", 
								  &DomainExpertNode::get_domain_predicates_service_callback,
								  this);
  get_domain_predicate_details_service_ = getBaseNode().advertiseService( prefix + "get_domain_predicate_details",
									 &DomainExpertNode::get_domain_predicate_details_service_callback,
									 this);
  get_domain_functions_service_ = getBaseNode().advertiseService( prefix + "get_domain_functions",
								 &DomainExpertNode::get_domain_functions_service_callback,
								 this);
  get_domain_function_details_service_ = getBaseNode().advertiseService( prefix + "get_domain_function_details",
									&DomainExpertNode::get_domain_function_details_service_callback,
									this);
  get_domain_service_ = getBaseNode().advertiseService( prefix + "get_domain", 
						       &DomainExpertNode::get_domain_service_callback,
						       this);
}


bool
DomainExpertNode::onConfigure()
{
  ROS_INFO("%s -- [%s] Configuring...",
	   getNodeName().c_str(),
	   get_name().c_str());
  std::string model_file;

  if(!getBaseNode().getParam("model_file", model_file))
  {
     ROS_ERROR("model_file parameter not defined for domain_expert");
     return false;
  }

  auto model_files = tokenize(model_file, ":");

  std::ifstream domain_ifs(model_files[0]);
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto planner = std::make_shared<plansys2::POPFPlanSolver>();

  domain_expert_ = std::make_shared<DomainExpert>(domain_str);
  
  std::string name= domain_expert_->getName();
  std::vector<std::string> types = domain_expert_->getTypes();
    
  bool check_valid = planner->is_valid_domain(domain_expert_->getDomain(), get_namespace());
  if (!check_valid) {
    ROS_ERROR("%s -- PDDL syntax error", getNodeName().c_str());
    return false;
  }

  for (size_t i = 1; i < model_files.size(); i++) {
    std::ifstream domain_ifs(model_files[i]);
    std::string domain_str((
        std::istreambuf_iterator<char>(domain_ifs)),
      std::istreambuf_iterator<char>());
    domain_expert_->extendDomain(domain_str);

    bool check_valid = planner->is_valid_domain(domain_expert_->getDomain(), get_namespace());

    if (!check_valid) {
      ROS_ERROR("%s -- PDDL syntax error", getNodeName().c_str());
      return false;
    }
  }

  ROS_INFO("%s --[%s] Configured", getNodeName().c_str(), get_name().c_str());
  return true;
}

bool
DomainExpertNode::onActivate()
{
  ROS_INFO("%s --[%s] Activating...", getNodeName().c_str(), get_name().c_str());
  ROS_INFO("%s --[%s] Activated", getNodeName().c_str(), get_name().c_str());

  return true;
}

bool
DomainExpertNode::onDeactivate()
{
  ROS_INFO("%s --[%s] Deactivating...", getNodeName().c_str(), get_name().c_str());
  ROS_INFO("%s --[%s] Deactivated", getNodeName().c_str(), get_name().c_str());

  return true;
}

bool
DomainExpertNode::onCleanup()
{
  ROS_INFO("%s --[%s] Cleaning up...", getNodeName().c_str(), get_name().c_str());
  ROS_INFO("%s --[%s] Cleaned up", getNodeName().c_str(), get_name().c_str());

  return true;
}

bool
DomainExpertNode::onShutdown()
{
  ROS_INFO("%s --[%s] Shutting down...", getNodeName().c_str(), get_name().c_str());
  ROS_INFO("%s --[%s] Shutted down", getNodeName().c_str(), get_name().c_str());

  return true;
}

bool
DomainExpertNode::onError(const std::exception &)
{
  ROS_ERROR("%s -- [%s] Error transition", getNodeName().c_str(), get_name().c_str());

  return false;
}

bool
DomainExpertNode::get_domain_name_service_callback(
						   plansys2_msgs::GetDomainName::Request &request,
						   plansys2_msgs::GetDomainName::Response &response)
{
  if (domain_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", getNodeName().c_str());
  } else {
    response.success = true;
    response.name = domain_expert_->getName();
  }

  return true;
}

bool
DomainExpertNode::get_domain_types_service_callback(
  plansys2_msgs::GetDomainTypes::Request &request,
  plansys2_msgs::GetDomainTypes::Response &response)
{
  if (domain_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", getNodeName().c_str());
  } else {
    response.success = true;
    response.types = domain_expert_->getTypes();
  }

  return true;
}

bool
DomainExpertNode::get_domain_actions_service_callback(
  plansys2_msgs::GetDomainActions::Request &request,
  plansys2_msgs::GetDomainActions::Response &response)
{
  if (domain_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", getNodeName().c_str());
  } else {
    response.success = true;
    for (const auto & action : domain_expert_->getActions()) {
      response.actions.push_back(action);
    }
  }

  return true;
}

bool
DomainExpertNode::get_domain_action_details_service_callback(
  plansys2_msgs::GetDomainActionDetails::Request &request,
  plansys2_msgs::GetDomainActionDetails::Response &response)
{
  if (domain_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";

    ROS_WARN("%s --Requesting service in non-active state", getNodeName().c_str());
  } else {
    auto action = domain_expert_->getAction(request.action, request.parameters);

    if (action) {
      response.action = *action;
      response.success = true;
    } else {
      ROS_WARN("%s --Requesting a non-existing action [%s]",
	       getNodeName().c_str(),
	       request.action.c_str());
      response.success = false;
      response.error_info = "Action not found";
    }
  }

  return true;
}

bool
DomainExpertNode::get_domain_durative_actions_service_callback(
  plansys2_msgs::GetDomainActions::Request &request,
  plansys2_msgs::GetDomainActions::Response &response)
{
  if (domain_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", getNodeName().c_str());
  } else {
    response.success = true;
    for (const auto & action : domain_expert_->getDurativeActions()) {
      response.actions.push_back(action);
    }
  }

  return true;
}

bool
DomainExpertNode::get_domain_durative_action_details_service_callback(
  plansys2_msgs::GetDomainDurativeActionDetails::Request &request,
  plansys2_msgs::GetDomainDurativeActionDetails::Response &response)
{
  if (domain_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";

    ROS_WARN("%s --Requesting service in non-active state",
	     getNodeName().c_str());
  } else {
    auto action = domain_expert_->getDurativeAction(request.durative_action, request.parameters);

    if (action) {
      response.durative_action = *action;
      response.success = true;
    } else {
      ROS_WARN("%s -- Requesting a non-existing durative action [%s]",
	       getNodeName().c_str(),
	       request.durative_action.c_str());
      response.success = false;
      response.error_info = "Durative action not found";
    }
  }

  return true;
}

bool
DomainExpertNode::get_domain_predicates_service_callback(
  plansys2_msgs::GetStates::Request &request,
  plansys2_msgs::GetStates::Response &response)
{
  if (domain_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s -- Requesting service in non-active state", getNodeName().c_str());
  } else {
    response.success = true;
    response.states = plansys2::convertVector<plansys2_msgs::Node, plansys2::Predicate>(
      domain_expert_->getPredicates());
  }

  return true;
}

bool
DomainExpertNode::get_domain_predicate_details_service_callback(
  plansys2_msgs::GetNodeDetails::Request &request,
  plansys2_msgs::GetNodeDetails::Response &response)
{
  if (domain_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s -- Requesting service in non-active state",
	     getNodeName().c_str());
  } else {
    auto predicate = domain_expert_->getPredicate(request.expression);
    if (predicate) {
      response.node = predicate.value();
      response.success = true;
    } else {
      ROS_WARN("%s -- Requesting a non-existing predicate [%s]",
	       getNodeName().c_str(),
	       request.expression.c_str());
      response.success = false;
      response.error_info = "Predicate not found";
    }
  }

  return true;
}

bool
DomainExpertNode::get_domain_functions_service_callback(
  plansys2_msgs::GetStates::Request &request,
  plansys2_msgs::GetStates::Response &response)
{
  if (domain_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s -- Requesting service in non-active state",
	     getNodeName().c_str());
  } else {
    response.success = true;
    response.states = plansys2::convertVector<plansys2_msgs::Node, plansys2::Function>(
      domain_expert_->getFunctions());
  }

  return true;
}

bool
DomainExpertNode::get_domain_function_details_service_callback(
  plansys2_msgs::GetNodeDetails::Request &request,
  plansys2_msgs::GetNodeDetails::Response &response)
{
  if (domain_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s -- Requesting service in non-active state", getNodeName().c_str());
  } else {
    auto function = domain_expert_->getFunction(request.expression);
    if (function) {
      response.node = function.value();
      response.success = true;
    } else {
      ROS_WARN("%s -- Requesting a non-existing function [%s]",
	       getNodeName().c_str(),
	       request.expression.c_str());
      response.success = false;
      response.error_info = "Function not found";
    }
  }

  return true;
}

bool
DomainExpertNode::get_domain_service_callback(
  plansys2_msgs::GetDomain::Request &request,
  plansys2_msgs::GetDomain::Response &response)
{
  if (domain_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s -- Requesting service in non-active state",
	     getNodeName().c_str());
  } else {
    response.success = true;

    std::ostringstream stream;
    stream << domain_expert_->getDomain();
    response.domain = stream.str();
  }

  return true;
}


}  // namespace plansys2
