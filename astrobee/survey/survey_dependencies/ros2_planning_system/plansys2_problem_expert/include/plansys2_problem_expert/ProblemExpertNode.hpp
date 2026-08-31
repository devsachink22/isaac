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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_

#include <memory>

#include <plansys2_problem_expert/ProblemExpert.hpp>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
//#include <lifecycle_msgs/state.h>
//#include <lifecycle_msgs/transition.h>
#include <plansys2_msgs/Knowledge.h>
#include <plansys2_msgs/AffectNode.h>
#include <plansys2_msgs/AffectParam.h>
#include <plansys2_msgs/AddProblem.h>
#include <plansys2_msgs/AddProblemGoal.h>
#include <plansys2_msgs/ExistNode.h>
#include <plansys2_msgs/GetProblem.h>
#include <plansys2_msgs/GetProblemGoal.h>
#include <plansys2_msgs/GetProblemInstanceDetails.h>
#include <plansys2_msgs/GetProblemInstances.h>
#include <plansys2_msgs/GetNodeDetails.h>
#include <plansys2_msgs/GetStates.h>
#include <plansys2_msgs/IsProblemGoalSatisfied.h>
#include <plansys2_msgs/RemoveProblemGoal.h>
#include <plansys2_msgs/ClearProblemKnowledge.h>

#include <ros/ros.h>
#include <lifecycle/managed_node.h>

namespace plansys2
{

class ProblemExpertNode : public ros::lifecycle::ManagedNode
{
public:
  ProblemExpertNode(const ros::NodeHandle& _nh);

  bool onConfigure();
  bool onActivate();
  bool onDeactivate();
  bool onCleanup();
  bool onShutdown();
  bool onError(std::exception &);

  std::shared_ptr<plansys2_msgs::Knowledge> get_knowledge_as_msg() const;

  bool add_problem_service_callback(plansys2_msgs::AddProblem::Request &request,
				    plansys2_msgs::AddProblem::Response &response);

  bool add_problem_goal_service_callback(plansys2_msgs::AddProblemGoal::Request &request,
					 plansys2_msgs::AddProblemGoal::Response &response);

  bool add_problem_instance_service_callback(plansys2_msgs::AffectParam::Request &request,
					     plansys2_msgs::AffectParam::Response &response);

  bool add_problem_predicate_service_callback(plansys2_msgs::AffectNode::Request &request,
					      plansys2_msgs::AffectNode::Response &response);

  bool add_problem_function_service_callback(plansys2_msgs::AffectNode::Request &request,
					     plansys2_msgs::AffectNode::Response &response);
  
  bool get_problem_goal_service_callback(plansys2_msgs::GetProblemGoal::Request &request,
					 plansys2_msgs::GetProblemGoal::Response &response);
  
  bool get_problem_instance_details_service_callback(plansys2_msgs::GetProblemInstanceDetails::Request &request,
						     plansys2_msgs::GetProblemInstanceDetails::Response &response);
  
  bool get_problem_instances_service_callback(plansys2_msgs::GetProblemInstances::Request &request,
					      plansys2_msgs::GetProblemInstances::Response &response);

  bool get_problem_predicate_details_service_callback(plansys2_msgs::GetNodeDetails::Request &request,
						      plansys2_msgs::GetNodeDetails::Response &response);
  
  bool get_problem_predicates_service_callback(plansys2_msgs::GetStates::Request &request,
					       plansys2_msgs::GetStates::Response &response);
  
  bool get_problem_function_details_service_callback(plansys2_msgs::GetNodeDetails::Request &request,
						     plansys2_msgs::GetNodeDetails::Response &response);
  
  bool get_problem_functions_service_callback(plansys2_msgs::GetStates::Request &request,
					      plansys2_msgs::GetStates::Response &response);
  
  bool get_problem_service_callback(plansys2_msgs::GetProblem::Request &request,
				    plansys2_msgs::GetProblem::Response &response);
  
  bool is_problem_goal_satisfied_service_callback(plansys2_msgs::IsProblemGoalSatisfied::Request &request,
						  plansys2_msgs::IsProblemGoalSatisfied::Response &response);
  
  bool remove_problem_goal_service_callback(plansys2_msgs::RemoveProblemGoal::Request &request,
					    plansys2_msgs::RemoveProblemGoal::Response &response);
  
  bool clear_problem_knowledge_service_callback(plansys2_msgs::ClearProblemKnowledge::Request &request,
						plansys2_msgs::ClearProblemKnowledge::Response &response);
  
  bool remove_problem_instance_service_callback(plansys2_msgs::AffectParam::Request &request,
						plansys2_msgs::AffectParam::Response &response);
  
  bool remove_problem_predicate_service_callback(plansys2_msgs::AffectNode::Request &request,
						 plansys2_msgs::AffectNode::Response &response);

  bool remove_problem_function_service_callback(plansys2_msgs::AffectNode::Request &request,
						plansys2_msgs::AffectNode::Response &response);

  bool exist_problem_predicate_service_callback(plansys2_msgs::ExistNode::Request &request,
						plansys2_msgs::ExistNode::Response &response);

  bool exist_problem_function_service_callback(plansys2_msgs::ExistNode::Request &request,
					       plansys2_msgs::ExistNode::Response &response);

  bool update_problem_function_service_callback(plansys2_msgs::AffectNode::Request &request,
						plansys2_msgs::AffectNode::Response &response);

private:
  std::shared_ptr<ProblemExpert> problem_expert_;

  ros::ServiceServer add_problem_service_;
  ros::ServiceServer add_problem_goal_service_;
  ros::ServiceServer add_problem_instance_service_;
  ros::ServiceServer add_problem_predicate_service_;
  ros::ServiceServer add_problem_function_service_;
  ros::ServiceServer get_problem_goal_service_;
  ros::ServiceServer get_problem_instance_details_service_;
  ros::ServiceServer get_problem_instances_service_;
  ros::ServiceServer get_problem_predicate_details_service_;
  ros::ServiceServer get_problem_predicates_service_;
  ros::ServiceServer get_problem_function_details_service_;
  ros::ServiceServer get_problem_functions_service_;
  ros::ServiceServer get_problem_service_;
  ros::ServiceServer is_problem_goal_satisfied_service_;
  ros::ServiceServer remove_problem_goal_service_;
  ros::ServiceServer clear_problem_knowledge_service_;
  ros::ServiceServer remove_problem_instance_service_;
  ros::ServiceServer remove_problem_predicate_service_;
  ros::ServiceServer remove_problem_function_service_;
  ros::ServiceServer exist_problem_predicate_service_;
  ros::ServiceServer exist_problem_function_service_;
  ros::ServiceServer update_problem_function_service_;

  /*
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::Empty>::SharedPtr update_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::Knowledge>::SharedPtr knowledge_pub_;*/

  ros::Publisher update_pub_;
  ros::Publisher knowledge_pub_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_
