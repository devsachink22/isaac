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
//
// Modifications by TRACLabs: Backported to ROS1
// Copyright 2022
//
#ifndef PLANSYS2_CORE__PLANSOLVERBASE_HPP_
#define PLANSYS2_CORE__PLANSOLVERBASE_HPP_

#include <optional>
#include <string>
#include <memory>

#include <plansys2_msgs/Plan.h>

#include <ros/ros.h>
#include <lifecycle/managed_node.h>

namespace plansys2
{

class PlanSolverBase
{
public:
  using Ptr = std::shared_ptr<plansys2::PlanSolverBase>;

  PlanSolverBase() {}

  virtual void configure(std::shared_ptr<ros::lifecycle::ManagedNode> &,
			 const std::string &) {}

  virtual std::optional<plansys2_msgs::Plan> getPlan(const std::string & domain,
						     const std::string & problem,
						     const std::string & node_namespace = "") = 0;
};

}  // namespace plansys2

#endif  // PLANSYS2_CORE__PLANSOLVERBASE_HPP_
