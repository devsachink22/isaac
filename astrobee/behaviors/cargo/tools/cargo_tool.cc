/* Copyright (c) 2021, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

// Command line flags
#include <gflags/gflags.h>
#include <gflags/gflags_completions.h>

//  ROS includes
#include <ros/ros.h>
#include <ros/package.h>

// FSW includes
#include <config_reader/config_reader.h>
#include <ff_common/ff_names.h>
#include <isaac_util/isaac_names.h>
#include <ff_util/ff_action.h>

// Action
#include <isaac_msgs/CargoAction.h>

// Service
#include <ff_msgs/SetState.h>

// TF2 support
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// C++ STL includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <cmath>

// Robot namespace
DEFINE_string(ns, "", "Robot namespace");

// Toggle based commands
DEFINE_bool(pause, false, "Send a pause command");
DEFINE_bool(resume, false, "Send an unpause command");
DEFINE_bool(pick, false, "Send a pick up cargo command");
DEFINE_bool(drop, false, "Send a drop cargo command");
DEFINE_int32(block_berth, 0, "Select blocked cargo berth (1 or 2)");
DEFINE_string(id, "cargo", "If of the cargo bag");
DEFINE_string(pose, "", "Berth pose: xyz xyzw");

// Timeout values
DEFINE_double(connect, 10.0, "Action connect timeout");
DEFINE_double(active, 10.0, "Action active timeout");
DEFINE_double(response, 200.0, "Action response timeout");
DEFINE_double(deadline, -1.0, "Action deadline timeout");

// Match the internal states and responses with the message definition
using STATE = isaac_msgs::CargoState;

// Inspection action feedback
void FeedbackCallback(isaac_msgs::CargoFeedbackConstPtr const& feedback) {
  std::cout << "\r                                                   "
            << "\rFSM: " << feedback->state.fsm_event
            << " -> " << feedback->state.fsm_state << std::flush;
}

// Inspection action result
void ResultCallback(ff_util::FreeFlyerActionState::Enum code,
  isaac_msgs::CargoResultConstPtr const& result) {
  std::cout << std::endl << "Result: ";
  // Print general response code
  switch (code) {
  case ff_util::FreeFlyerActionState::Enum::SUCCESS:
    std::cout << "[SUCCESS] ";               break;
  case ff_util::FreeFlyerActionState::Enum::PREEMPTED:
    std::cout << "[PREEMPT] ";               break;
  case ff_util::FreeFlyerActionState::Enum::ABORTED:
    std::cout << "[ABORTED] ";               break;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_CONNECT:
    std::cout << "Action timed out on connect";        goto teardown;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_ACTIVE:
    std::cout << "Action timed out on active";         goto teardown;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_RESPONSE:
    std::cout << "Action timed out on response";       goto teardown;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_DEADLINE:
    std::cout << "Action timed out on deadline";       goto teardown;
  }
  // If we get there then we have some response data
  std::cout << result->fsm_result
            << " (Code " << result->response << ")" << std::endl;
teardown:
  std::cout << std::endl;
  // In all cases we must shutdown
  ros::shutdown();
}

// Ensure all clients are connected
void ConnectedCallback(
  ff_util::FreeFlyerActionClient<isaac_msgs::CargoAction> *client) {
  // Check to see if connected
  if (!client->IsConnected()) return;
  // Print out a status message
  std::cout << "\r                                                   "
            << "\rState: CONNECTED" << std::flush;
  // Prepare the goal
  isaac_msgs::CargoGoal goal;
  if (FLAGS_pause) {
    goal.command = isaac_msgs::CargoGoal::PAUSE;
  } else if (FLAGS_resume) {
    goal.command = isaac_msgs::CargoGoal::RESUME;
  } else if (FLAGS_pick) {
    // Fill in command type
    goal.command = isaac_msgs::CargoGoal::PICK;
    goal.cargo_id = FLAGS_id;

    // Read file geometry
    std::cout << "Reading: " << FLAGS_pose << std::endl;

    // Parse and modify the pose
    std::string str_p = FLAGS_pose;
    if (!str_p.empty()) {
      std::istringstream iss_p(str_p);
      std::vector<double> vec_p {
        std::istream_iterator<double>(iss_p),
        std::istream_iterator<double>()
      };
      if (vec_p.size() > 0) goal.pose.pose.position.x  = vec_p[0];
      if (vec_p.size() > 1) goal.pose.pose.position.y  = vec_p[1];
      if (vec_p.size() > 2) goal.pose.pose.position.z  = vec_p[2];

      if (vec_p.size() == 7) {
        goal.pose.pose.orientation.x = vec_p[3];
        goal.pose.pose.orientation.y = vec_p[4];
        goal.pose.pose.orientation.z = vec_p[5];
        goal.pose.pose.orientation.w = vec_p[6];
      }
    }

  } else if (FLAGS_drop) {
    // Fill in command type
    goal.command = isaac_msgs::CargoGoal::DROP;
    // goal.pose.pose.header.frame_id = "world";

    // Read file geometry
    std::cout << "Reading: " << FLAGS_pose << std::endl;

    // Parse and modify the pose
    std::string str_p = FLAGS_pose;
    if (!str_p.empty()) {
      std::istringstream iss_p(str_p);
      std::vector<double> vec_p {
        std::istream_iterator<double>(iss_p),
        std::istream_iterator<double>()
      };
      if (vec_p.size() > 0) goal.pose.pose.position.x    = vec_p[0];
      if (vec_p.size() > 1) goal.pose.pose.position.y    = vec_p[1];
      if (vec_p.size() > 2) goal.pose.pose.position.z    = vec_p[2];

      if (vec_p.size() == 7) {
        goal.pose.pose.orientation.x = vec_p[3];
        goal.pose.pose.orientation.y = vec_p[4];
        goal.pose.pose.orientation.z = vec_p[5];
        goal.pose.pose.orientation.w = vec_p[6];
      }
    }
  }

  client->SendGoal(goal);
}

// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "cargo_tool", ros::init_options::AnonymousName);
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun cargo cargo_tool <opts>");
  google::SetVersionString("0.1.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Some simple checks
  uint8_t cmd = 0;
  if (FLAGS_pause) cmd++;
  if (FLAGS_resume) cmd++;
  if (FLAGS_pick)   cmd++;
  if (FLAGS_drop) cmd++;
  if (FLAGS_block_berth != 0) cmd++;
  // Check we have specified one of the required switches
  if (cmd != 1) {
    std::cerr << "Specify one cargo command: -pick, -drop, -pause, "
              << "-resume, or -block_berth" << std::endl;
    return 1;
  }
  if (FLAGS_block_berth < 0 || FLAGS_block_berth > 2) {
    std::cerr << "The blocked berth must be 1 or 2" << std::endl;
    return 1;
  }
  if ((FLAGS_pick || FLAGS_drop) && FLAGS_pose.empty()) {
    std::cout << "The pick or drop command must also have a pose flag" << std::endl;
    return 1;
  }
  if (FLAGS_pick && FLAGS_id.empty()) {
    std::cout << "The pick command must also specify an id flag" << std::endl;
    return 1;
  }
  // Create a node handle in the selected robot namespace.
  ros::NodeHandle nh(std::string("/") + FLAGS_ns);

  // Berth selection is a synchronous service command, not a cargo action.
  if (FLAGS_block_berth != 0) {
    ros::ServiceClient berth_client = nh.serviceClient<ff_msgs::SetState>(
      SERVICE_BEHAVIORS_CARGO_SET_BLOCKED_BERTH);
    if (!berth_client.waitForExistence(ros::Duration(FLAGS_connect))) {
      std::cerr << "Timed out waiting for the blocked-berth service"
                << std::endl;
      return 1;
    }
    ff_msgs::SetState srv;
    srv.request.state = FLAGS_block_berth;
    if (!berth_client.call(srv) || !srv.response.success) {
      std::cerr << "Unable to select cargo berth " << FLAGS_block_berth
                << " as blocked" << std::endl;
      return 1;
    }
    std::cout << "Cargo berth " << FLAGS_block_berth
              << " is now blocked" << std::endl;
    google::ShutDownCommandLineFlags();
    return 0;
  }

  // Action client for cargo manipulation commands.
  ff_util::FreeFlyerActionClient<isaac_msgs::CargoAction> client;
  // Setup SWITCH action
  client.SetConnectedTimeout(FLAGS_connect);
  client.SetActiveTimeout(FLAGS_active);
  client.SetResponseTimeout(FLAGS_response);
  if (FLAGS_deadline > 0)
    client.SetDeadlineTimeout(FLAGS_deadline);
  client.SetFeedbackCallback(std::bind(FeedbackCallback,
    std::placeholders::_1));
  client.SetResultCallback(std::bind(ResultCallback,
    std::placeholders::_1, std::placeholders::_2));
  client.SetConnectedCallback(std::bind(ConnectedCallback, &client));
  client.Create(&nh, ACTION_BEHAVIORS_CARGO);
  // Print out a status message
  std::cout << "\r                                                   "
            << "\rState: CONNECTING" << std::flush;
  // Synchronous mode
  ros::spin();
  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
