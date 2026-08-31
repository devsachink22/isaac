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
#include <vector>

//#include <lifecycle_msgs/transition.hpp>
//#include <lifecycle_msgs/state.hpp>

#include <plansys2_msgs/ActionExecutionInfo.h>

#include <plansys2_executor/ActionExecutorClient.hpp>

namespace plansys2
{

using namespace std::chrono_literals;

ActionExecutorClient::ActionExecutorClient(const ros::NodeHandle &nh, 
					   const std::string & node_name,
					   const std::chrono::nanoseconds & rate)
  : 
    CascadeLifecycleNode(nh),
    rate_(rate),
    commited_(false)
{
  // Note that nh should have the same namespace as node_name
  // node_name should be removed after I test a bit more
  // Ana 
    printf("Node inside ActionExecutorClient: namespace: %s \n", nh.getNamespace().c_str());
  //declare_parameter<std::string>("action_name", "");
  //declare_parameter<std::vector<std::string>>(
  //"specialized_arguments", std::vector<std::string>({}));

  double default_rate = 1.0 / std::chrono::duration<double>(rate_).count();
  //declare_parameter<double>("rate", default_rate);
  status_.state = plansys2_msgs::ActionPerformerStatus::NOT_READY;
  status_.status_stamp = now();
  status_.node_name = get_name();
}


bool ActionExecutorClient::onConfigure()
{ 
  printf("On configure. Node base name: %s namespace: %s \n", get_name().c_str(), getBaseNode().getNamespace().c_str());
  
  status_pub_.reset(new ros::lifecycle::LifecyclePublisher<plansys2_msgs::ActionPerformerStatus>(shared_from_this(),
												 "performers_status"));
  status_pub_->on_activate();

  hearbeat_pub_ = std::make_shared<ros::WallTimer>(getBaseNode().createWallTimer(ros::WallDuration(1.0),
										 [this](const ros::WallTimerEvent &event) {
										   status_.status_stamp = now();
										   status_pub_->publish(status_);
										 }) );

  if (!getBaseNode().getParam("action_name", action_managed_)) {
    ROS_ERROR("%s -- action_name parameter not set", get_name().c_str());
    status_.state = plansys2_msgs::ActionPerformerStatus::FAILURE;
    status_.status_stamp = now();
  }
 
  if(!getBaseNode().getParam("specialized_arguments", specialized_arguments_))
    specialized_arguments_.clear();

  double rate;
  if(!getBaseNode().getParam("rate", rate))
    rate = 1.0 / std::chrono::duration<double>(rate_).count(); // From constructor

  rate_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / rate));

  action_hub_pub_.reset(new ros::lifecycle::LifecyclePublisher<plansys2_msgs::ActionExecution>(shared_from_this(),
											       "/actions_hub"));
  action_hub_sub_ = std::make_shared<ros::Subscriber>( getBaseNode().subscribe("/actions_hub", 10,
									       &ActionExecutorClient::action_hub_callback, this));

  action_hub_pub_->on_activate();

  status_.state = plansys2_msgs::ActionPerformerStatus::READY;
  status_.status_stamp = now();
  status_.action = action_managed_;
  status_.specialized_arguments = specialized_arguments_;

  return true;
}

bool
ActionExecutorClient::onActivate()
{ 
  status_.state = plansys2_msgs::ActionPerformerStatus::RUNNING;
  status_.status_stamp = now();
  // nanoseconds?
  timer_ = std::make_shared<ros::WallTimer>(getBaseNode().createWallTimer(ros::WallDuration(rate_.count()/1e9), 
							    std::bind(&ActionExecutorClient::do_work, this)));

//  do_work();

  return true;
}

bool ActionExecutorClient::onDeactivate()
{
  status_.state = plansys2_msgs::ActionPerformerStatus::READY;
  status_.status_stamp = now();
  timer_ = nullptr;

  return true;
}

void ActionExecutorClient::action_hub_callback(const plansys2_msgs::ActionExecution::ConstPtr &msg)
{  
  switch (msg->type) {
    case plansys2_msgs::ActionExecution::REQUEST:
      if (getCurrentState() == ros::lifecycle::INACTIVE &&
        !commited_ && should_execute(msg->action, msg->arguments))
      {  
        commited_ = true;
        send_response(msg);
      }
      break;
    case plansys2_msgs::ActionExecution::CONFIRM:
      if (getCurrentState() == ros::lifecycle::INACTIVE &&
        commited_ && msg->node_id == get_name())
      {
        current_arguments_ = msg->arguments;
        trigger_transition(ros::lifecycle::ACTIVATE);

        commited_ = false;
      }
      break;
    case plansys2_msgs::ActionExecution::REJECT:
      if (msg->node_id == get_name()) {
        commited_ = false;
      }
      break;
    case plansys2_msgs::ActionExecution::CANCEL:
      if (getCurrentState() == ros::lifecycle::ACTIVE &&
        msg->node_id == get_name())
      {
        trigger_transition(ros::lifecycle::DEACTIVATE);
      }
      break;
    case plansys2_msgs::ActionExecution::RESPONSE:
    case plansys2_msgs::ActionExecution::FEEDBACK:
    case plansys2_msgs::ActionExecution::FINISH:
      break;
    default:
      ROS_ERROR(
        "%s -- Msg %d type not recognized in %s executor performer",
	get_name().c_str(),
	msg->type, get_name().c_str());
      break;
  }
}

bool
ActionExecutorClient::should_execute(
  const std::string & action, const std::vector<std::string> & args)
{
  if (action != action_managed_) {
    return false;
  }

  if (!specialized_arguments_.empty()) {
    if (specialized_arguments_.size() != args.size()) {
      ROS_WARN( "%s -- current and specialized arguments length doesn't match %zu %zu",
		get_name().c_str(),
		args.size(), specialized_arguments_.size());
    }

    for (size_t i = 0; i < specialized_arguments_.size() && i < args.size(); i++) {
      if (specialized_arguments_[i] != "" && args[i] != "" &&
        specialized_arguments_[i] != args[i])
      {
        return false;
      }
    }
  }

  return true;
}

void
ActionExecutorClient::send_response(const plansys2_msgs::ActionExecution::ConstPtr &msg)
{
  plansys2_msgs::ActionExecution msg_resp(*msg);
  msg_resp.type = plansys2_msgs::ActionExecution::RESPONSE;
  msg_resp.node_id = get_name();

  action_hub_pub_->publish(msg_resp);
}

void
ActionExecutorClient::send_feedback(float completion, const std::string & status)
{
  plansys2_msgs::ActionExecution msg_resp;
  msg_resp.type = plansys2_msgs::ActionExecution::FEEDBACK;
  msg_resp.node_id = get_name();
  msg_resp.action = action_managed_;
  msg_resp.arguments = current_arguments_;
  msg_resp.completion = completion;
  msg_resp.status = status;

  action_hub_pub_->publish(msg_resp);
}

void
ActionExecutorClient::finish(bool success, float completion, const std::string & status)
{
  if (getCurrentState() == ros::lifecycle::ACTIVE) {
    trigger_transition(ros::lifecycle::DEACTIVATE);
  }

  plansys2_msgs::ActionExecution msg_resp;
  msg_resp.type = plansys2_msgs::ActionExecution::FINISH;
  msg_resp.node_id = get_name();
  msg_resp.action = action_managed_;
  msg_resp.arguments = current_arguments_;
  msg_resp.completion = completion;
  msg_resp.status = status;
  msg_resp.success = success;

  action_hub_pub_->publish(msg_resp);
}

}  // namespace plansys2
