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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_

#include <optional>
#include <string>
#include <vector>

#include <plansys2_problem_expert/ProblemExpertInterface.hpp>
#include <plansys2_core/Types.hpp>

#include <plansys2_msgs/Node.h>
#include <plansys2_msgs/Param.h>
#include <plansys2_msgs/Tree.h>
#include <plansys2_msgs/AddProblem.h>
#include <plansys2_msgs/AddProblemGoal.h>
#include <plansys2_msgs/AffectNode.h>
#include <plansys2_msgs/AffectParam.h>
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

namespace plansys2
{

class ProblemExpertClient : public ProblemExpertInterface
{
public:
  ProblemExpertClient();

  std::vector<plansys2::Instance> getInstances();
  bool addInstance(const plansys2::Instance & instance);
  bool removeInstance(const plansys2::Instance & instance);
  std::optional<plansys2::Instance> getInstance(const std::string & name);

  std::vector<plansys2::Predicate> getPredicates();
  bool addPredicate(const plansys2::Predicate & predicate);
  bool removePredicate(const plansys2::Predicate & predicate);
  bool existPredicate(const plansys2::Predicate & predicate);
  std::optional<plansys2::Predicate> getPredicate(const std::string & predicate);

  std::vector<plansys2::Function> getFunctions();
  bool addFunction(const plansys2::Function & function);
  bool removeFunction(const plansys2::Function & function);
  bool existFunction(const plansys2::Function & function);
  bool updateFunction(const plansys2::Function & function);
  std::optional<plansys2::Function> getFunction(const std::string & function);

  plansys2::Goal getGoal();
  bool setGoal(const plansys2::Goal & goal);
  bool isGoalSatisfied(const plansys2::Goal & goal);

  bool clearGoal();
  bool clearKnowledge();

  std::string getProblem();
  bool addProblem(const std::string & problem_str);

  ros::Time getUpdateTime() const {return update_time_;}

  std::string getName() { return std::string("ProblemExpertClient"); }
  
private:
  ros::ServiceClient add_problem_client_;
  ros::ServiceClient add_problem_goal_client_;
  ros::ServiceClient add_problem_instance_client_;
  ros::ServiceClient add_problem_predicate_client_;
  ros::ServiceClient add_problem_function_client_;
  ros::ServiceClient get_problem_goal_client_;
  ros::ServiceClient get_problem_instance_details_client_;
  ros::ServiceClient get_problem_instances_client_;
  ros::ServiceClient get_problem_predicate_details_client_;
  ros::ServiceClient get_problem_predicates_client_;
  ros::ServiceClient get_problem_function_details_client_;
  ros::ServiceClient get_problem_functions_client_;
  ros::ServiceClient get_problem_client_;
  ros::ServiceClient remove_problem_goal_client_;
  ros::ServiceClient clear_problem_knowledge_client_;
  ros::ServiceClient remove_problem_instance_client_;
  ros::ServiceClient remove_problem_predicate_client_;
  ros::ServiceClient remove_problem_function_client_;
  ros::ServiceClient exist_problem_predicate_client_;
  ros::ServiceClient exist_problem_function_client_;
  ros::ServiceClient update_problem_function_client_;
  ros::ServiceClient is_problem_goal_satisfied_client_;

  ros::NodeHandle node_;
  ros::Time update_time_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_
