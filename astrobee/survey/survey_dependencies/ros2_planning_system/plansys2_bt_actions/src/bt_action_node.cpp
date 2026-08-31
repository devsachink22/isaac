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

#include <memory>
#include <string>

#include <plansys2_bt_actions/BTAction.hpp>
#include <ros/ros.h>


using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bt_action_node");
  
  std::string name = ros::this_node::getName();
  if(name.empty() || (name.size() == 1 && name[0] == '/') )
    name = "default";
  else if(name[0] == '/')
    name = name.substr(1);
  
  ros::NodeHandle nh(name);
  auto action_node = std::make_shared<plansys2::BTAction>(nh, name, 200ms);

  action_node->trigger_transition(ros::lifecycle::CONFIGURE);

  //ros::spin();

ros::AsyncSpinner spinner(4); // Use 4 threads
spinner.start();
ros::waitForShutdown();

  return 0;
}
