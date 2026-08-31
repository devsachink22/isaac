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

#ifndef PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_
#define PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_

#include <string>
#include <memory>
#include <vector>

#include <plansys2_msgs/ActionExecution.h>
#include <plansys2_msgs/ActionPerformerStatus.h>

#include <plansys2_domain_expert/DomainExpertClient.hpp>
#include <plansys2_problem_expert/ProblemExpertClient.hpp>

#include <ros/ros.h>
#include <lifecycle/cascade_lifecycle_node.h>
#include <lifecycle/lifecycle_publisher.h>
#include <lifecycle/lifecycle_subscriber.h>

namespace plansys2
{

class ActionExecutorClient : public ros::lifecycle::CascadeLifecycleNode
{
public:
  using Ptr = std::shared_ptr<ActionExecutorClient>;
  static Ptr make_shared(const ros::NodeHandle &nh, const std::string & node_name, const std::chrono::nanoseconds & rate)
  {
    return std::make_shared<ActionExecutorClient>(nh, node_name, rate);
  }

  ActionExecutorClient(
    const ros::NodeHandle &nh,
    const std::string & node_name,
    const std::chrono::nanoseconds & rate);

  plansys2_msgs::ActionPerformerStatus get_internal_status() const {return status_;}

protected:
  virtual void do_work() {}

  const std::vector<std::string> & get_arguments() const {return current_arguments_;}
  const std::string get_action_name() const {return action_managed_;}


  virtual bool onConfigure();
  virtual bool onActivate();
  virtual bool onDeactivate();
  void action_hub_callback(const plansys2_msgs::ActionExecution::ConstPtr &msg);

  bool should_execute(const std::string & action, const std::vector<std::string> & args);
  void send_response(const plansys2_msgs::ActionExecution::ConstPtr &msg);
  void send_feedback(float completion, const std::string & status = "");
  void finish(bool success, float completion, const std::string & status = "");

  std::chrono::nanoseconds rate_;
  std::string action_managed_;
  bool commited_;

  std::vector<std::string> current_arguments_;
  std::vector<std::string> specialized_arguments_;

  std::shared_ptr<ros::lifecycle::LifecyclePublisher<plansys2_msgs::ActionExecution> >
    action_hub_pub_;
  std::shared_ptr<ros::Subscriber>  action_hub_sub_;
  std::shared_ptr<ros::WallTimer> timer_;

  std::shared_ptr<ros::lifecycle::LifecyclePublisher<plansys2_msgs::ActionPerformerStatus> >
    status_pub_;
  std::shared_ptr<ros::WallTimer> hearbeat_pub_;
  plansys2_msgs::ActionPerformerStatus status_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_
