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

#include "plansys2_problem_expert/ProblemExpertNode.hpp"

#include <string>
#include <memory>
#include <vector>

#include "plansys2_pddl_parser/Utils.h"

std::vector<std::string> tokenize(const std::string & string,
				  const std::string & delim)
{
  std::string::size_type lastPos = 0, pos = string.find_first_of(delim, lastPos);
  std::vector<std::string> tokens;

  while (lastPos != std::string::npos) {
    if (pos != lastPos) {
      tokens.push_back(string.substr(lastPos, pos - lastPos));
    }
    lastPos = pos;
    if (lastPos == std::string::npos || lastPos + 1 == string.length()) {
      break;
    }
    pos = string.find_first_of(delim, ++lastPos);
  }

  return tokens;
}

namespace plansys2
{

  ProblemExpertNode::ProblemExpertNode(const ros::NodeHandle &_nh)
    : ros::lifecycle::ManagedNode(_nh)
{
  // ROS1 No need to declare params
  //declare_parameter("model_file", "");
  //declare_parameter("problem_file", "");

  std::string prefix(""); //("problem_expert/");
  add_problem_service_ = getBaseNode().advertiseService( prefix + "add_problem",
							&ProblemExpertNode::add_problem_service_callback,
							this);

  add_problem_goal_service_ = getBaseNode().advertiseService( prefix + "add_problem_goal",
							     &ProblemExpertNode::add_problem_goal_service_callback,
							     this);

  add_problem_instance_service_ = getBaseNode().advertiseService( prefix + "add_problem_instance",
								 &ProblemExpertNode::add_problem_instance_service_callback,
								 this);

  add_problem_predicate_service_ = getBaseNode().advertiseService( prefix + "add_problem_predicate",
								  &ProblemExpertNode::add_problem_predicate_service_callback,
								  this);

  add_problem_function_service_ = getBaseNode().advertiseService( prefix + "add_problem_function",
								 &ProblemExpertNode::add_problem_function_service_callback,
								 this);

  get_problem_goal_service_ = getBaseNode().advertiseService( prefix + "get_problem_goal",
							     &ProblemExpertNode::get_problem_goal_service_callback,
							     this);

  get_problem_instance_details_service_ = getBaseNode().advertiseService( prefix + "get_problem_instance",
									 &ProblemExpertNode::get_problem_instance_details_service_callback,
									 this);

  get_problem_instances_service_ = getBaseNode().advertiseService( prefix + "get_problem_instances",
								  &ProblemExpertNode::get_problem_instances_service_callback,
								  this);

  get_problem_predicate_details_service_ =
    getBaseNode().advertiseService( prefix + "get_problem_predicate",
				   &ProblemExpertNode::get_problem_predicate_details_service_callback,
				   this);

  get_problem_predicates_service_ = getBaseNode().advertiseService( prefix + "get_problem_predicates",
								   &ProblemExpertNode::get_problem_predicates_service_callback,
								   this);

  get_problem_function_details_service_ = getBaseNode().advertiseService( prefix + "get_problem_function",
									 &ProblemExpertNode::get_problem_function_details_service_callback,
									 this);

  get_problem_functions_service_ = getBaseNode().advertiseService( prefix + "get_problem_functions",
								  &ProblemExpertNode::get_problem_functions_service_callback,
								  this);

  get_problem_service_ = getBaseNode().advertiseService( prefix + "get_problem", 
							&ProblemExpertNode::get_problem_service_callback,
							this);

  is_problem_goal_satisfied_service_ = getBaseNode().advertiseService( prefix + "is_problem_goal_satisfied", 
								      &ProblemExpertNode::is_problem_goal_satisfied_service_callback,
								      this);

  remove_problem_goal_service_ = getBaseNode().advertiseService( prefix + "remove_problem_goal",
								&ProblemExpertNode::remove_problem_goal_service_callback,
								this);

  clear_problem_knowledge_service_ = getBaseNode().advertiseService( prefix + "clear_problem_knowledge",
								    &ProblemExpertNode::clear_problem_knowledge_service_callback,
								    this);

  remove_problem_instance_service_ = getBaseNode().advertiseService( prefix + "remove_problem_instance",
								    &ProblemExpertNode::remove_problem_instance_service_callback,
								    this);

  remove_problem_predicate_service_ = getBaseNode().advertiseService( prefix + "remove_problem_predicate",
								     &ProblemExpertNode::remove_problem_predicate_service_callback,
      this);
  
  remove_problem_function_service_ = getBaseNode().advertiseService( prefix + "remove_problem_function",
								    &ProblemExpertNode::remove_problem_function_service_callback,
								    this);

  exist_problem_predicate_service_ = getBaseNode().advertiseService( prefix + "exist_problem_predicate",
								    &ProblemExpertNode::exist_problem_predicate_service_callback,
								    this);

  exist_problem_function_service_ = getBaseNode().advertiseService( prefix + "exist_problem_function",
								   &ProblemExpertNode::exist_problem_function_service_callback,
								   this);

  update_problem_function_service_ = getBaseNode().advertiseService( prefix + "update_problem_function",
								    &ProblemExpertNode::update_problem_function_service_callback,
								    this);
  /*
  update_pub_ = create_publisher<std_msgs::Empty>(
     prefix + "update_notify",
    rclcpp::QoS(100));

  knowledge_pub_ = create_publisher<plansys2_msgs::Knowledge>(
     prefix + "knowledge",
    rclcpp::QoS(100).transient_local());
  */
  update_pub_ = getBaseNode().advertise<std_msgs::Empty>( prefix + "update_notify", 10);
  knowledge_pub_ = getBaseNode().advertise<plansys2_msgs::Knowledge>( prefix + "knowledge", 10);
}


bool ProblemExpertNode::onConfigure()
{
  ROS_INFO("[%s] Configuring...", get_name().c_str());

  // (fmrico) Here we could have a discussion if we should read the domain from file or
  // from the domain_expert service. Then, we should configure first domain_expert node
  std::string model_file;
  getBaseNode().getParam("model_file", model_file);

  auto model_files = tokenize(model_file, ":");

  std::ifstream domain_first_ifs(model_files[0]);
  std::string domain_first_str((
      std::istreambuf_iterator<char>(domain_first_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<DomainExpert>(domain_first_str);

  for (size_t i = 1; i < model_files.size(); i++) {
    std::ifstream domain_ifs(model_files[i]);
    std::string domain_str((
        std::istreambuf_iterator<char>(domain_ifs)),
      std::istreambuf_iterator<char>());
    domain_expert->extendDomain(domain_str);
  }

  problem_expert_ = std::make_shared<ProblemExpert>(domain_expert);

  std::string problem_file;
  getBaseNode().getParam("problem_file", problem_file);
  if (!problem_file.empty()) {
    std::ifstream problem_ifs(problem_file);
    std::string problem_str((
			     std::istreambuf_iterator<char>(problem_ifs)),
			    std::istreambuf_iterator<char>());
    problem_expert_->addProblem(problem_str);
  }

  ROS_INFO("[%s] Configured", get_name().c_str());
  return true;
}

bool ProblemExpertNode::onActivate()
{
  ROS_INFO("[%s] Activating...", get_name().c_str());
  //update_pub_->on_activate(); // ANA HACK
  //knowledge_pub_->on_activate(); // ANA HACK
  ROS_INFO("[%s] Activated", get_name().c_str());
  return true;
}

bool ProblemExpertNode::onDeactivate()
{
  ROS_INFO("[%s] Deactivating...", get_name().c_str());
  //update_pub_.on_deactivate(); // ANA HACK
  //knowledge_pub_.on_deactivate(); // ANA HACK
  ROS_INFO("[%s] Deactivated", get_name().c_str());

  return true;
}

bool ProblemExpertNode::onCleanup()
{
  ROS_INFO("[%s] Cleaning up...", get_name().c_str());
  ROS_INFO("[%s] Cleaned up", get_name().c_str());

  return true;
}

bool ProblemExpertNode::onShutdown()
{
  ROS_INFO("[%s] Shutting down...", get_name().c_str());
  ROS_INFO("[%s] Shutted down", get_name().c_str());

  return true;
}

bool ProblemExpertNode::onError(std::exception &)
{
  ROS_ERROR("[%s] Error transition", get_name().c_str());

  return false;
}

bool ProblemExpertNode::add_problem_service_callback(plansys2_msgs::AddProblem::Request &request,
						     plansys2_msgs::AddProblem::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s -- Requesting service in non-active state", get_name().c_str());
  } else {
    ROS_INFO("Adding problem:\n%s", request.problem.c_str());
    response.success = problem_expert_->addProblem(request.problem);

    if (response.success) {
      update_pub_.publish(std_msgs::Empty());
      knowledge_pub_.publish(*get_knowledge_as_msg());
    } else {
      response.error_info = "Problem not valid";
    }
  }
  
  return true;
}

bool ProblemExpertNode::add_problem_goal_service_callback(plansys2_msgs::AddProblemGoal::Request &request,
							  plansys2_msgs::AddProblemGoal::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s -- Requesting service in non-active state", get_name().c_str());
  } else {
    if (!parser::pddl::empty(request.tree)) {
      response.success = problem_expert_->setGoal(request.tree);
      if (response.success) {
        update_pub_.publish(std_msgs::Empty());
        knowledge_pub_.publish(*get_knowledge_as_msg());
      } else {
        response.error_info = "Goal not valid";
      }
    } else {
      response.success = false;
      response.error_info = "Malformed expression";
    }
  }
  
  return true;
}

bool ProblemExpertNode::add_problem_instance_service_callback(plansys2_msgs::AffectParam::Request &request,
							      plansys2_msgs::AffectParam::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s -- Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = problem_expert_->addInstance(request.param);
    if (response.success) {
      update_pub_.publish(std_msgs::Empty());
      knowledge_pub_.publish(*get_knowledge_as_msg());
    } else {
      response.error_info = "Instance not valid";
    }
  }
  
  return true;
}

bool ProblemExpertNode::add_problem_predicate_service_callback(plansys2_msgs::AffectNode::Request &request,
  plansys2_msgs::AffectNode::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = problem_expert_->addPredicate(request.node);
    if (response.success) {
      update_pub_.publish(std_msgs::Empty());
      knowledge_pub_.publish(*get_knowledge_as_msg());
    } else {
      response.error_info =
        "Predicate [" + parser::pddl::toString(request.node) + "] not valid";
    }
  }
  
  return true;
}

bool ProblemExpertNode::add_problem_function_service_callback(plansys2_msgs::AffectNode::Request &request,
  plansys2_msgs::AffectNode::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = problem_expert_->addFunction(request.node);
    if (response.success) {
      update_pub_.publish(std_msgs::Empty());
      knowledge_pub_.publish(*get_knowledge_as_msg());
    } else {
      response.error_info =
        "Function [" + parser::pddl::toString(request.node) + "] not valid";
    }
  }
  
  return true;
}

bool ProblemExpertNode::get_problem_goal_service_callback(plansys2_msgs::GetProblemGoal::Request &request,
  plansys2_msgs::GetProblemGoal::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = true;
    response.tree = problem_expert_->getGoal();
  }
  
  return true;
}

bool ProblemExpertNode::get_problem_instance_details_service_callback(plansys2_msgs::GetProblemInstanceDetails::Request &request,
								      plansys2_msgs::GetProblemInstanceDetails::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    auto instance = problem_expert_->getInstance(request.instance);
    if (instance) {
      response.success = true;
      response.instance = instance.value();
    } else {
      response.success = false;
      response.error_info = "Instance not found";
    }
  }
  
  return true;
}

bool ProblemExpertNode::get_problem_instances_service_callback(plansys2_msgs::GetProblemInstances::Request &request,
  plansys2_msgs::GetProblemInstances::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = true;
    response.instances = plansys2::convertVector<plansys2_msgs::Param, plansys2::Instance>(
      problem_expert_->getInstances());
  }
  
  return true;
}

bool ProblemExpertNode::get_problem_predicate_details_service_callback(plansys2_msgs::GetNodeDetails::Request &request,
  plansys2_msgs::GetNodeDetails::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    auto predicate = problem_expert_->getPredicate(request.expression);
    if (predicate) {
      response.node = predicate.value();
      response.success = true;
    } else {
      response.success = false;
      response.error_info = "Predicate not found";
    }
  }
  
  return true;
}

bool ProblemExpertNode::get_problem_predicates_service_callback(plansys2_msgs::GetStates::Request &request,
  plansys2_msgs::GetStates::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = true;
    response.states = plansys2::convertVector<plansys2_msgs::Node, plansys2::Predicate>(
      problem_expert_->getPredicates());
  }
  
  return true;
}

bool ProblemExpertNode::get_problem_function_details_service_callback(plansys2_msgs::GetNodeDetails::Request &request,
  plansys2_msgs::GetNodeDetails::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    auto function = problem_expert_->getFunction(request.expression);
    if (function) {
      response.node = function.value();
      response.success = true;
    } else {
      response.success = false;
      response.error_info = "Function not found";
    }
  }
  
  return true;
}

bool ProblemExpertNode::get_problem_functions_service_callback(plansys2_msgs::GetStates::Request &request,
  plansys2_msgs::GetStates::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = true;
    response.states = plansys2::convertVector<plansys2_msgs::Node, plansys2::Function>(
      problem_expert_->getFunctions());
  }
  
  return true;
}

bool ProblemExpertNode::get_problem_service_callback(plansys2_msgs::GetProblem::Request &request,
  plansys2_msgs::GetProblem::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = true;
    response.problem = problem_expert_->getProblem();
  }
  
  return true;
}

bool ProblemExpertNode::is_problem_goal_satisfied_service_callback(plansys2_msgs::IsProblemGoalSatisfied::Request &request,
  plansys2_msgs::IsProblemGoalSatisfied::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = true;
    response.satisfied = problem_expert_->isGoalSatisfied(request.tree);
  }
  
  return true;
}

bool ProblemExpertNode::remove_problem_goal_service_callback(plansys2_msgs::RemoveProblemGoal::Request &request,
  plansys2_msgs::RemoveProblemGoal::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = problem_expert_->clearGoal();

    if (response.success) {
      update_pub_.publish(std_msgs::Empty());
      knowledge_pub_.publish(*get_knowledge_as_msg());
    } else {
      response.error_info = "Error clearing goal";
    }
  }
  
  return true;
}

bool ProblemExpertNode::clear_problem_knowledge_service_callback(plansys2_msgs::ClearProblemKnowledge::Request &request,
  plansys2_msgs::ClearProblemKnowledge::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = problem_expert_->clearKnowledge();

    if (response.success) {
      update_pub_.publish(std_msgs::Empty());
      knowledge_pub_.publish(*get_knowledge_as_msg());
    } else {
      response.error_info = "Error clearing knowledge";
    }
  }
  
  return true;
}


bool ProblemExpertNode::remove_problem_instance_service_callback(plansys2_msgs::AffectParam::Request &request,
  plansys2_msgs::AffectParam::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = problem_expert_->removeInstance(request.param);
    if (response.success) {
      update_pub_.publish(std_msgs::Empty());
      knowledge_pub_.publish(*get_knowledge_as_msg());
    } else {
      response.error_info = "Error removing instance";
    }
  }
  
  return true;
}

bool
ProblemExpertNode::remove_problem_predicate_service_callback(plansys2_msgs::AffectNode::Request &request,
							     plansys2_msgs::AffectNode::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = problem_expert_->removePredicate(request.node);
    if (response.success) {
      update_pub_.publish(std_msgs::Empty());
      knowledge_pub_.publish(*get_knowledge_as_msg());
    } else {
      response.error_info = "Error removing predicate";
    }
  }
  
  return true;
}

bool
ProblemExpertNode::remove_problem_function_service_callback(plansys2_msgs::AffectNode::Request &request,
							    plansys2_msgs::AffectNode::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = problem_expert_->removeFunction(request.node);
    if (response.success) {
      update_pub_.publish(std_msgs::Empty());
    } else {
      response.error_info = "Error removing function";
    }
  }
  
  return true;
}

bool ProblemExpertNode::exist_problem_predicate_service_callback(plansys2_msgs::ExistNode::Request &request,
  plansys2_msgs::ExistNode::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.exist = false;
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.exist = problem_expert_->existPredicate(request.node);
  }
  
  return true;
}

bool ProblemExpertNode::exist_problem_function_service_callback(plansys2_msgs::ExistNode::Request &request,
							   plansys2_msgs::ExistNode::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.exist = false;
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.exist = problem_expert_->existFunction(request.node);
  }

  return true;
}

bool ProblemExpertNode::update_problem_function_service_callback(plansys2_msgs::AffectNode::Request &request,
  plansys2_msgs::AffectNode::Response &response)
{
  if (problem_expert_ == nullptr) {
    response.success = false;
    response.error_info = "Requesting service in non-active state";
    ROS_WARN("%s --Requesting service in non-active state", get_name().c_str());
  } else {
    response.success = problem_expert_->updateFunction(request.node);
    if (response.success) {
      update_pub_.publish(std_msgs::Empty());
    } else {
      response.error_info = "Function not valid";
    }
  }

  return true;
}

std::shared_ptr<plansys2_msgs::Knowledge>
ProblemExpertNode::get_knowledge_as_msg() const
{
  auto ret_msgs = std::make_shared<plansys2_msgs::Knowledge>();

  for (const auto & instance : problem_expert_->getInstances()) {
    ret_msgs->instances.push_back(instance.name);
  }

  for (const auto & predicate : problem_expert_->getPredicates()) {
    ret_msgs->predicates.push_back(parser::pddl::toString(predicate));
  }

  for (const auto & function : problem_expert_->getFunctions()) {
    ret_msgs->functions.push_back(parser::pddl::toString(function));
  }

  auto goal = problem_expert_->getGoal();
  ret_msgs->goal = parser::pddl::toString(goal);

  return ret_msgs;
}

}  // namespace plansys2
