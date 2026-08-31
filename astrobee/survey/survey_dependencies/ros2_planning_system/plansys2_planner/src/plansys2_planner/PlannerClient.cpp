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

#include <plansys2_planner/PlannerClient.hpp>

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

namespace plansys2
{

PlannerClient::PlannerClient()
{
  node_ = ros::NodeHandle("planner_client");

  std::string prefix("/planner/"); 
  get_plan_client_ = node_.serviceClient<plansys2_msgs::GetPlan>(prefix + "get_plan");
}

std::string PlannerClient::getName()
{ return std::string("planner_client"); }
  
std::optional<plansys2_msgs::Plan>
PlannerClient::getPlan(const std::string & domain,
		       const std::string & problem,
		       const std::string & node_namespace)
{
  while (!get_plan_client_.waitForExistence(ros::Duration(30.0))) {
    if (!ros::ok()) {
      return {};
    }
    ROS_ERROR_STREAM(getName() <<
		     get_plan_client_.getService() <<
		     " service  client: waiting for service to appear...");
  }

  plansys2_msgs::GetPlan srv;
  srv.request.domain = domain;
  srv.request.problem = problem;  

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

}  // namespace plansys2
