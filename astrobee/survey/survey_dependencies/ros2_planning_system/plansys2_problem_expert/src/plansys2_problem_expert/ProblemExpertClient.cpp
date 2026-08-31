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

#include <plansys2_problem_expert/ProblemExpertClient.hpp>

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include <plansys2_pddl_parser/Utils.h>

namespace plansys2
{

ProblemExpertClient::ProblemExpertClient()
{
  node_ = ros::NodeHandle("problem_expert_client");
  
  std::string prefix = std::string("/problem_expert/");
  add_problem_client_ = node_.serviceClient<plansys2_msgs::AddProblem>(prefix + "add_problem");
  add_problem_goal_client_ = node_.serviceClient<plansys2_msgs::AddProblemGoal>( prefix + "add_problem_goal");
  add_problem_instance_client_ = node_.serviceClient<plansys2_msgs::AffectParam>( prefix + "add_problem_instance");
  add_problem_predicate_client_ = node_.serviceClient<plansys2_msgs::AffectNode>( prefix + "add_problem_predicate");
  add_problem_function_client_ = node_.serviceClient<plansys2_msgs::AffectNode>( prefix + "add_problem_function");
  get_problem_goal_client_ = node_.serviceClient<plansys2_msgs::GetProblemGoal>( prefix + "get_problem_goal");
  get_problem_instance_details_client_ = node_.serviceClient<plansys2_msgs::GetProblemInstanceDetails>( prefix + "get_problem_instance");
  get_problem_instances_client_ = node_.serviceClient<plansys2_msgs::GetProblemInstances>( prefix + "get_problem_instances");
  get_problem_predicate_details_client_ = node_.serviceClient<plansys2_msgs::GetNodeDetails>( prefix + "get_problem_predicate");
  get_problem_predicates_client_ = node_.serviceClient<plansys2_msgs::GetStates>( prefix + "get_problem_predicates");
  get_problem_function_details_client_ = node_.serviceClient<plansys2_msgs::GetNodeDetails>( prefix + "get_problem_function");
  get_problem_functions_client_ = node_.serviceClient<plansys2_msgs::GetStates>( prefix + "get_problem_functions");
  get_problem_client_ = node_.serviceClient<plansys2_msgs::GetProblem>( prefix + "get_problem");
  remove_problem_goal_client_ = node_.serviceClient<plansys2_msgs::RemoveProblemGoal>( prefix + "remove_problem_goal");
  clear_problem_knowledge_client_ = node_.serviceClient<plansys2_msgs::ClearProblemKnowledge>( prefix + "clear_problem_knowledge");
  remove_problem_instance_client_ = node_.serviceClient<plansys2_msgs::AffectParam>( prefix + "remove_problem_instance");
  remove_problem_predicate_client_ = node_.serviceClient<plansys2_msgs::AffectNode>( prefix + "remove_problem_predicate");
  remove_problem_function_client_ = node_.serviceClient<plansys2_msgs::AffectNode>( prefix + "remove_problem_function");
  exist_problem_predicate_client_ = node_.serviceClient<plansys2_msgs::ExistNode>( prefix + "exist_problem_predicate");
  exist_problem_function_client_ = node_.serviceClient<plansys2_msgs::ExistNode>( prefix + "exist_problem_function");
  update_problem_function_client_ = node_.serviceClient<plansys2_msgs::AffectNode>(  prefix + "update_problem_function");
  is_problem_goal_satisfied_client_ = node_.serviceClient<plansys2_msgs::IsProblemGoalSatisfied>(  prefix + "is_problem_goal_satisfied");
  
}

std::vector<plansys2::Instance>
ProblemExpertClient::getInstances()
{
  while (!get_problem_instances_client_.waitForExistence(ros::Duration(1.0)))
  {
    if (!ros::ok())
      return {};
    
    ROS_INFO_STREAM(getName() << " Waiting for service " << get_problem_instances_client_.getService() << " to appear");      
  }
  
  plansys2_msgs::GetProblemInstances srv;;

  if (!get_problem_instances_client_.call(srv))
    return {};
  

  if (srv.response.success) {
    return plansys2::convertVector<plansys2::Instance, plansys2_msgs::Param>(
      srv.response.instances);
  } else {
    ROS_ERROR_STREAM(getName() <<
		     node_.getNamespace() <<
		     get_problem_instances_client_.getService() << ": " <<
		     srv.response.error_info);
    return {};
  }
}

bool
ProblemExpertClient::addInstance(const plansys2::Instance & instance)
{
  while (!add_problem_instance_client_.waitForExistence(ros::Duration(1.0)))
  {
    if (!ros::ok())
      return false;
    
    ROS_INFO_STREAM(getName() << " Waiting for service " << add_problem_instance_client_.getService() << " to appear");      
  }

  plansys2_msgs::AffectParam srv;
  srv.request.param = instance;

  if (!add_problem_instance_client_.call(srv))
  {
    return false;
  }

  if (srv.response.success) {
    update_time_ = ros::Time::now();
    return true;
  } else {
    ROS_ERROR_STREAM(getName() <<
		     add_problem_instance_client_.getService() << ": " <<
		     srv.response.error_info);
    return false;
  }
}

bool
ProblemExpertClient::removeInstance(const plansys2::Instance & instance)
{
  while (!remove_problem_instance_client_.waitForExistence(ros::Duration(1.0)))
  {
    if (!ros::ok())
      return {};
    
    ROS_INFO_STREAM(getName() << " Waiting for service " << remove_problem_instance_client_.getService() << " to appear");      
  }
  
  plansys2_msgs::AffectParam srv;
  srv.request.param = instance;

  if (!remove_problem_instance_client_.call(srv))
  {
    return false;
  }

  if (srv.response.success) {
    update_time_ = ros::Time::now();
    return true;
  } else {
    ROS_ERROR_STREAM(getName() <<
		     remove_problem_instance_client_.getService() << ": " <<
		     srv.response.error_info);
    return false;
  }

}


std::optional<plansys2::Instance>
ProblemExpertClient::getInstance(const std::string & name)
{
  while (!get_problem_instance_details_client_.waitForExistence(ros::Duration(1.0)))
  {
    if (!ros::ok())
      return {};
    
    ROS_INFO_STREAM(getName() << " Waiting for service " << get_problem_instance_details_client_.getService() << " to appear");      
  }

  
  plansys2_msgs::GetProblemInstanceDetails srv;
  srv.request.instance = name;

  if (!get_problem_instance_details_client_.call(srv))
  {
    return {};
  }

  if (srv.response.success) {
    return srv.response.instance;
  } else {
    ROS_ERROR_STREAM(getName() <<
		     get_problem_instance_details_client_.getService() << ": " <<
		     srv.response.error_info);
    return {};
  }

}

std::vector<plansys2::Predicate>
ProblemExpertClient::getPredicates()
{
  while (!get_problem_predicates_client_.waitForExistence(ros::Duration(1.0)))
  {
    if (!ros::ok())
      return {};
    
    ROS_INFO_STREAM(getName() << " Waiting for service " << get_problem_predicates_client_.getService() << " to appear");      
  }

  plansys2_msgs::GetStates srv;

  if( !get_problem_predicates_client_.call(srv))
  {
    return {};
  }

  if (srv.response.success) {
    return plansys2::convertVector<plansys2::Predicate, plansys2_msgs::Node>(srv.response.states);
  } else {
    ROS_ERROR_STREAM(getName() << 
		     get_problem_predicates_client_.getService() << ": " <<
		     srv.response.error_info);
    return {};
  }
}

bool
ProblemExpertClient::addPredicate(const plansys2::Predicate & predicate)
{
  while (!add_problem_predicate_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return false;
    }
    ROS_ERROR_STREAM(getName() << 
      add_problem_predicate_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::AffectNode srv;
  srv.request.node = predicate;

  if (!add_problem_predicate_client_.call(srv))
  {
    return false;
  }

  if (srv.response.success) {
    update_time_ = ros::Time::now();
    return true;
  } else {
    ROS_ERROR_STREAM(getName() << 
		     add_problem_predicate_client_.getService() << ": " <<
		     srv.response.error_info);
    return false;
  }
}

bool
ProblemExpertClient::removePredicate(const plansys2::Predicate & predicate)
{
  while (!remove_problem_predicate_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return false;
    }
    ROS_ERROR_STREAM(getName() << 
      remove_problem_predicate_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::AffectNode srv;
  srv.request.node = predicate;

  if (!remove_problem_predicate_client_.call(srv))
  {
    return false;
  }

  if (srv.response.success) {
    update_time_ = ros::Time::now();
    return true;
  } else {
    ROS_ERROR_STREAM(getName() << 
      remove_problem_predicate_client_.getService() << ": " <<
        srv.response.error_info);
    return false;
  }
  
}

bool
ProblemExpertClient::existPredicate(const plansys2::Predicate & predicate)
{
  while (!exist_problem_predicate_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return false;
    }
    ROS_ERROR_STREAM(getName() << 
      exist_problem_predicate_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::ExistNode srv;
  srv.request.node = predicate;

  if (!exist_problem_predicate_client_.call(srv))
  {
    return false;
  }

  return srv.response.exist;

}

std::optional<plansys2::Predicate>
ProblemExpertClient::getPredicate(const std::string & predicate)
{
  
  while (!get_problem_predicate_details_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return {};
    }
    ROS_ERROR_STREAM(getName() << 
      get_problem_predicate_details_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::GetNodeDetails srv;

  srv.request.expression = predicate;

  

  if (!get_problem_predicate_details_client_.call(srv))
  {
    return {};
  }

  if (srv.response.success) {
    return srv.response.node;
  } else {
    ROS_DEBUG_STREAM(
		     getName() <<
		     get_problem_predicate_details_client_.getService() << ": " <<
		     srv.response.error_info);
    return {};
  }
}

std::vector<plansys2::Function>
ProblemExpertClient::getFunctions()
{
  while (!get_problem_functions_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return {};
    }
    ROS_ERROR_STREAM(getName() << 
      get_problem_functions_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::GetStates srv;

  if (!get_problem_functions_client_.call(srv))
  {
    return {};
  }

  if (srv.response.success) {
    return plansys2::convertVector<plansys2::Function, plansys2_msgs::Node>(
      srv.response.states);
  } else {
    ROS_ERROR_STREAM(getName() << 
      get_problem_functions_client_.getService() << ": " <<
        srv.response.error_info);
    return {};
  }
}

bool ProblemExpertClient::addFunction(const plansys2::Function & function)
{
  while (!add_problem_function_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return false;
    }
    ROS_ERROR_STREAM(getName() << 
      add_problem_function_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::AffectNode srv;
  srv.request.node = function;

  if (!add_problem_function_client_.call(srv))
  {
    return false;
  }

  if (srv.response.success) {
    update_time_ = ros::Time::now();
    return true;
  } else {
    ROS_ERROR_STREAM(getName() << 
      add_problem_function_client_.getService() << ": " <<
        srv.response.error_info);
    return false;
    }
  
}

bool
ProblemExpertClient::removeFunction(const plansys2::Function & function)
{
  while (!remove_problem_function_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return false;
    }
    ROS_ERROR_STREAM(getName() << 
      remove_problem_function_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::AffectNode srv;
  srv.request.node = function;

  if (!remove_problem_function_client_.call(srv))
  {
    return false;
  }

  if (srv.response.success) {
    update_time_ = ros::Time::now();
    return true;
  } else {
    ROS_ERROR_STREAM(getName() << 
      remove_problem_function_client_.getService() << ": " <<
        srv.response.error_info);
    return false;
    }
  
}

bool
ProblemExpertClient::existFunction(const plansys2::Function & function)
{
  while (!exist_problem_function_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return false;
    }
    ROS_ERROR_STREAM(getName() << 
      exist_problem_function_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::ExistNode srv;
  srv.request.node = function;

  if (!exist_problem_function_client_.call(srv))
  {
    return false;
  }

  return srv.response.exist;
}

bool ProblemExpertClient::updateFunction(const plansys2::Function & function)
{
  while (!update_problem_function_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return false;
    }
    ROS_ERROR_STREAM(getName() << 
      update_problem_function_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::AffectNode srv;
  srv.request.node = function;

  

  if (!update_problem_function_client_.call(srv))
  {
    return false;
  }

  if (srv.response.success) {
    update_time_ = ros::Time::now();
    return true;
  } else {
    ROS_ERROR_STREAM(getName() << 
		     update_problem_function_client_.getService() << ": " <<
		     srv.response.error_info);
    return false;
  }
  
}

std::optional<plansys2::Function>
ProblemExpertClient::getFunction(const std::string & function)
{
  while (!get_problem_function_details_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return {};
    }
    ROS_ERROR_STREAM(getName() << 
      get_problem_function_details_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::GetNodeDetails srv;
  srv.request.expression = function;

  if (!get_problem_function_details_client_.call(srv))
  {
    return {};
  }

  if (srv.response.success) {
    return srv.response.node;
  } else {
    ROS_ERROR_STREAM(getName() << 
		     get_problem_function_details_client_.getService() << ": " <<
		     srv.response.error_info);
    return {};
  }

}


plansys2::Goal
ProblemExpertClient::getGoal()
{
  plansys2_msgs::Tree ret;
  
  while (!get_problem_goal_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return ret;
    }
    ROS_ERROR_STREAM(getName() << 
      get_problem_goal_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::GetProblemGoal srv;

  if (!get_problem_goal_client_.call(srv))
  {
    return ret;
  }

  if (srv.response.success) {
    return srv.response.tree;
  } else {
    ROS_ERROR_STREAM(getName() << 
		     get_problem_goal_client_.getService() << ": " <<
		     srv.response.error_info);
  }
  
  return ret;
}

bool
ProblemExpertClient::setGoal(const plansys2::Goal & goal)
{
  while (!add_problem_goal_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return false;
    }
    ROS_ERROR_STREAM(getName() << 
      add_problem_goal_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::AddProblemGoal srv;
  srv.request.tree = goal;

  

  if (!add_problem_goal_client_.call(srv))
  {
    return false;
  }

  if (srv.response.success) {
    update_time_ = ros::Time::now();
    return true;
  } else {
    ROS_ERROR_STREAM(getName() << 
		     add_problem_goal_client_.getService() << ": " <<
		     srv.response.error_info);
    return false;
  }
  return true;
}

bool
ProblemExpertClient::isGoalSatisfied(const plansys2::Goal & goal)
{
  while (!is_problem_goal_satisfied_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return false;
    }
    ROS_ERROR_STREAM(getName() << 
      is_problem_goal_satisfied_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::IsProblemGoalSatisfied srv;
  srv.request.tree = goal;

  if (!is_problem_goal_satisfied_client_.call(srv))
  {
    return false;
  }

  if (srv.response.success) {
    return srv.response.satisfied;
  } else {
    ROS_ERROR_STREAM(getName() << 
		     is_problem_goal_satisfied_client_.getService() << ": " <<
		     srv.response.error_info);
    return false;
  }

}

bool
ProblemExpertClient::clearGoal()
{
  while (!remove_problem_goal_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return false;
    }
    ROS_ERROR_STREAM(getName() << 
      remove_problem_goal_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::RemoveProblemGoal srv;

  if (!remove_problem_goal_client_.call(srv))
  {
    return false;
  }

  if (srv.response.success) {
    update_time_ = ros::Time::now();
    return true;
  } else {
    ROS_ERROR_STREAM(getName() << 
		     remove_problem_goal_client_.getService() << ": " <<
		     srv.response.error_info);
    return false;
  }
  
}

bool
ProblemExpertClient::clearKnowledge()
{
  while (!clear_problem_knowledge_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return false;
    }
    ROS_ERROR_STREAM(getName() << 
      clear_problem_knowledge_client_.getService() <<
        " service  client: waiting for service to appear...");
  }

  plansys2_msgs::ClearProblemKnowledge srv;

  if (!clear_problem_knowledge_client_.call(srv))
  {
    return false;
  }

  if (srv.response.success) {
    update_time_ = ros::Time::now();
    return true;
  } else {
    ROS_ERROR_STREAM(getName() << 
		     clear_problem_knowledge_client_.getService() << ": " <<
		     srv.response.error_info);
    return false;
  }
}


std::string
ProblemExpertClient::getProblem()
{
  while (!get_problem_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return {};
    }
    ROS_ERROR_STREAM(getName() << 
		     get_problem_client_.getService() <<
		     " service  client: waiting for service to appear...");
  }
  
  plansys2_msgs::GetProblem srv;

  if (!get_problem_client_.call(srv))
  {
    return {};
  }

  if (srv.response.success) {
    return srv.response.problem;
  } else {
    ROS_ERROR_STREAM(getName() << 
		     get_problem_client_.getService() << ": " <<
		     srv.response.error_info);
    return {};
    }
}

bool
ProblemExpertClient::addProblem(const std::string & problem_str)
{
  while (!add_problem_client_.waitForExistence(ros::Duration(1.0))) {
    if (!ros::ok()) {
      return false;
    }
    
    ROS_ERROR_STREAM(getName() << 
		     add_problem_client_.getService() <<
		     " service  client: waiting for service to appear...");
  }

  plansys2_msgs::AddProblem srv;
  srv.request.problem = problem_str;

  if (!add_problem_client_.call(srv))
  {
    return false;
  }

  if (srv.response.success) {
    update_time_ = ros::Time::now();
    return true;
  } else {
    ROS_ERROR_STREAM(getName() << 
		     add_problem_client_.getService() << ": " <<
		     srv.response.error_info);
    return false;
  }
}

}  // namespace plansys2
