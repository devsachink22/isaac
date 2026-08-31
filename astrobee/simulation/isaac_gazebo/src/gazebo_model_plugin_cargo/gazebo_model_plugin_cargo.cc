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

// ROS includes
#include <ros/ros.h>

// Astrobee simulation API
#include <astrobee_gazebo/astrobee_gazebo.h>

// ISAAC names include
#include <isaac_util/isaac_names.h>

// Finite state machine for tracking dock state
#include <ff_util/ff_fsm.h>

// Messages
#include <isaac_msgs/CargoAction.h>
#include <ff_msgs/SetState.h>

// STL includes
#include <string>
#include <thread>

// US lab
#define us_lab_x_min -0.6
#define us_lab_x_max 5.4
#define us_lab_y_min -1.5
#define us_lab_y_max 1.0
#define us_lab_z_min 3.5
#define us_lab_z_max 5.8


namespace gazebo {

using FSM = ff_util::FSM;

/*
  Simulates cargo pickup and drop. While held, cargo follows Astrobee. When
  released near the available cargo berth, it snaps flush to the berth's
  front face before being fixed to the ISS.
*/

class GazeboModelPluginCargo : public FreeFlyerModelPlugin {
 public:
  // All possible states the EPS node can be in
  enum : FSM::State {
    UNKNOWN       = 1,       // We don't know if we are docked or undocked
    NO_CARGO      = 2,       // We are undocked
    CARGO         = 4,       // We are grasping cargo
  };

  // All possible events that can occur
  enum : FSM::Event {
    PICKED        = (1<<0),  // We picked up the cargo
    DROPPED       = (1<<1),  // We docked the cargo
  };

  // Constructor
  GazeboModelPluginCargo() : FreeFlyerModelPlugin("cargo_driver", "", false),
    fsm_(UNKNOWN, std::bind(&GazeboModelPluginCargo::StateCallback,
      this, std::placeholders::_1, std::placeholders::_2)),
    lock_(false),
    available_berth_name_("cargo_berth_available"),
    magnetic_capture_distance_(0.60),
    magnetic_surface_offset_(0.464894) {
      // In an unknown state, if we are sensed to be near or far from a berth
      // then update to either a docked or undocked state.
      fsm_.Add(UNKNOWN, PICKED,
        [this](FSM::Event const& event) -> FSM::State {
          // Create a virtual link
          Lock();
          return CARGO;
      });
      fsm_.Add(UNKNOWN, DROPPED,
        [this](FSM::Event const& event) -> FSM::State {
          // Destroy a virtual link
          Unlock();
          return NO_CARGO;
      });
      // In an undocked state, if we come near to some berth then we need
      fsm_.Add(NO_CARGO, PICKED,
        [this](FSM::Event const& event) -> FSM::State {
          // Create a virtual link
          Lock();
          // We are now holding the cargo
          return CARGO;
        });
      // In a DOCKED state if we receive a FAR message then we have undocked,
      fsm_.Add(CARGO, DROPPED,
        [this](FSM::Event const& event) -> FSM::State {
          // Destroy a virtual link
          Unlock();
          // We are now free from the cargo
          return NO_CARGO;
      });
    }

  // Destructor
  ~GazeboModelPluginCargo() {
    #if GAZEBO_MAJOR_VERSION > 7
    update_.reset();
    #else
    event::Events::DisconnectWorldUpdateEnd(update_);
    #endif
  }

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(ros::NodeHandle *nh,
    physics::ModelPtr model, sdf::ElementPtr sdf) {
    if (sdf->HasElement("available_berth")) {
      available_berth_name_ = sdf->Get<std::string>("available_berth");
    }
    if (sdf->HasElement("magnetic_capture_distance")) {
      magnetic_capture_distance_ =
        sdf->Get<double>("magnetic_capture_distance");
    }
    if (sdf->HasElement("magnetic_surface_offset")) {
      magnetic_surface_offset_ =
        sdf->Get<double>("magnetic_surface_offset");
    }

    // Subscribe to the cargo state callback
    sub_cargo_ = nh->subscribe(TOPIC_BEHAVIORS_CARGO_STATE, 10,
        &GazeboModelPluginCargo::CargoActionCallback, this);

    // Defer the extrinsics setup to allow plugins to load
    update_ = event::Events::ConnectWorldUpdateEnd(
      std::bind(&GazeboModelPluginCargo::BerthCallback, this));

    // Service to make cargo despawn
    server_despawn_ = nh->advertiseService("beh/cargo/despawn",
      &GazeboModelPluginCargo::DespawnCallback, this);
  }

  // Called on registration of aplanner
  bool DespawnCallback(ff_msgs::SetState::Request& req,
                        ff_msgs::SetState::Response& res) {
    // If fixed joint exists, detach
    if (joint_) {
      joint_->Detach();
      joint_->Fini();
    }
    #if GAZEBO_MAJOR_VERSION > 7
    physics::ModelPtr cargo = GetWorld()->ModelByName(cargo_name_);
    #else
    physics::ModelPtr cargo = GetWorld()->GetModel(cargo_name_);
    #endif
    if (cargo == nullptr)
      return false;

    #if GAZEBO_MAJOR_VERSION > 7
    ignition::math::Pose3d away_pose(GetModel()->WorldPose().Pos().X(),
                                     GetModel()->WorldPose().Pos().Y(),
                                     GetModel()->WorldPose().Pos().Z()+100,
                                     GetModel()->WorldPose().Rot().W(),
                                     GetModel()->WorldPose().Rot().X(),
                                     GetModel()->WorldPose().Rot().Y(),
                                     GetModel()->WorldPose().Rot().Z());
    #else
    ignition::math::Pose3d away_pose(GetModel()->GetWorldPose().pos.x,
                                     GetModel()->GetWorldPose().pos.y,
                                     GetModel()->GetWorldPose().pos.z+100,
                                     GetModel()->GetWorldPose().rot.w,
                                     GetModel()->GetWorldPose().rot.x,
                                     GetModel()->GetWorldPose().rot.y,
                                     GetModel()->GetWorldPose().rot.z);
    #endif

    cargo->SetWorldPose(away_pose);
    res.success = true;
    return true;
  }

  // When the FSM state changes we get a callback here, so that we can send
  // feedback to any active client, print debug info and publish our state
  void StateCallback(FSM::State const& state, FSM::FSM::Event const& event) {
    // Debug events
    std::string str = "UNKNOWN";
    switch (event) {
    case PICKED:                  str = "PICKED";             break;
    case DROPPED:                 str = "DROPPED";            break;
    }
    NODELET_DEBUG_STREAM("Received event " << str);
    // Debug state changes
    switch (state) {
    case UNKNOWN:                 str = "UNKNOWN";            break;
    case NO_CARGO:                str = "NO_CARGO";           break;
    case CARGO:                   str = "CARGO";              break;
    }
    NODELET_DEBUG_STREAM("State changed to " << str);
  }

  // If the robot is holding the cargo, make it follow the robot
  void BerthCallback() {
    if (lock_) {
      #if GAZEBO_MAJOR_VERSION > 7
      physics::ModelPtr cargo = GetWorld()->ModelByName(cargo_name_);
      #else
      physics::ModelPtr cargo = GetWorld()->GetModel(cargo_name_);
      #endif
      if (cargo == nullptr)
        return;

      #if GAZEBO_MAJOR_VERSION > 7
      ignition::math::Pose3d tf_bs = rel_pose_;
      ignition::math::Pose3d tf_wb = GetModel()->WorldPose();
      ignition::math::Pose3d world_pose(tf_bs + tf_wb);
      #else
      ignition::math::Pose3d pose_robot(GetModel()->GetWorldPose().pos.x,
                                          GetModel()->GetWorldPose().pos.y,
                                          GetModel()->GetWorldPose().pos.z,
                                          GetModel()->GetWorldPose().rot.w,
                                          GetModel()->GetWorldPose().rot.x,
                                          GetModel()->GetWorldPose().rot.y,
                                          GetModel()->GetWorldPose().rot.z);
      ignition::math::Pose3d world_pose(rel_pose_ + pose_robot);
      #endif

      cargo->SetWorldPose(world_pose);
    }
  }

  // Create a virtual joint to lock the freeflyer to the berth
  void Lock() {
    ROS_ERROR_STREAM("LOCK");
    // We are not guaranteed to have a cargo yet, so we need to check to see
    // that the model pointer is valid. If it is valid, then we to quietly
    // ignore locking for the time being.
    if (joint_) {
      joint_->Detach();
      joint_->Fini();
    }
    #if GAZEBO_MAJOR_VERSION > 7
    physics::ModelPtr cargo = GetWorld()->ModelByName(cargo_name_);
    #else
    physics::ModelPtr cargo = GetWorld()->GetModel(cargo_name_);
    #endif
    if (cargo == nullptr)
      return;
    ros::Duration(2).sleep();  // wait for grasp to fully finish
    // Save the relative position to the robot
    #if GAZEBO_MAJOR_VERSION > 7
      // Get robot pose
      ignition::math::Pose3d pose_robot(GetModel()->WorldPose().Pos().X(),
                                        GetModel()->WorldPose().Pos().Y(),
                                        GetModel()->WorldPose().Pos().Z(),
                                        GetModel()->WorldPose().Rot().W(),
                                        GetModel()->WorldPose().Rot().X(),
                                        GetModel()->WorldPose().Rot().Y(),
                                        GetModel()->WorldPose().Rot().Z());

      // Get cargo pose
      ignition::math::Pose3d pose_cargo(cargo->WorldPose().Pos().X(),
                                        cargo->WorldPose().Pos().Y(),
                                        cargo->WorldPose().Pos().Z(),
                                        cargo->WorldPose().Rot().W(),
                                        cargo->WorldPose().Rot().X(),
                                        cargo->WorldPose().Rot().Y(),
                                        cargo->WorldPose().Rot().Z());
    rel_pose_ = ignition::math::Pose3d(pose_cargo - pose_robot);
    #else
      // Get robot pose
      ignition::math::Pose3d pose_robot(GetModel()->GetWorldPose().pos.x,
                                        GetModel()->GetWorldPose().pos.y,
                                        GetModel()->GetWorldPose().pos.z,
                                        GetModel()->GetWorldPose().rot.w,
                                        GetModel()->GetWorldPose().rot.x,
                                        GetModel()->GetWorldPose().rot.y,
                                        GetModel()->GetWorldPose().rot.z);

      // Get cargo pose
      ignition::math::Pose3d pose_cargo(cargo->GetWorldPose().pos.x,
                                        cargo->GetWorldPose().pos.y,
                                        cargo->GetWorldPose().pos.z,
                                        cargo->GetWorldPose().rot.w,
                                        cargo->GetWorldPose().rot.x,
                                        cargo->GetWorldPose().rot.y,
                                        cargo->GetWorldPose().rot.z);
      rel_pose_ = pose_cargo - pose_robot;
    #endif

    lock_ = true;
    // Stop colliding with the handrail
    physics::LinkPtr link = cargo->GetLink("body");
    if (link)
      link->SetCollideMode("none");
  }

  // Snap cargo to the front face of the available berth when it is close
  // enough to be captured. Both objects have the same local dimensions, so
  // one full local-Y length separates their centers at surface contact.
  bool SnapToAvailableBerth(physics::ModelPtr const& cargo) {
    #if GAZEBO_MAJOR_VERSION > 7
    physics::ModelPtr berth =
      GetWorld()->ModelByName(available_berth_name_);
    #else
    physics::ModelPtr berth = GetWorld()->GetModel(available_berth_name_);
    #endif
    if (berth == nullptr)
      return false;

    #if GAZEBO_MAJOR_VERSION > 7
    ignition::math::Pose3d berth_pose = berth->WorldPose();
    ignition::math::Pose3d cargo_pose = cargo->WorldPose();
    #else
    ignition::math::Pose3d berth_pose(berth->GetWorldPose().pos.x,
                                      berth->GetWorldPose().pos.y,
                                      berth->GetWorldPose().pos.z,
                                      berth->GetWorldPose().rot.w,
                                      berth->GetWorldPose().rot.x,
                                      berth->GetWorldPose().rot.y,
                                      berth->GetWorldPose().rot.z);
    ignition::math::Pose3d cargo_pose(cargo->GetWorldPose().pos.x,
                                      cargo->GetWorldPose().pos.y,
                                      cargo->GetWorldPose().pos.z,
                                      cargo->GetWorldPose().rot.w,
                                      cargo->GetWorldPose().rot.x,
                                      cargo->GetWorldPose().rot.y,
                                      cargo->GetWorldPose().rot.z);
    #endif

    ignition::math::Vector3d offset = berth_pose.Rot().RotateVector(
      ignition::math::Vector3d(0, magnetic_surface_offset_, 0));
    ignition::math::Pose3d target_pose(berth_pose.Pos() + offset,
                                       berth_pose.Rot());
    double distance = cargo_pose.Pos().Distance(target_pose.Pos());
    if (distance > magnetic_capture_distance_)
      return false;

    cargo->SetWorldPose(target_pose);
    cargo->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
    cargo->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
    NODELET_INFO_STREAM("Magnetically attached cargo " << cargo_name_
      << " to the front of " << available_berth_name_);
    return true;
  }

  void Unlock() {
      #if GAZEBO_MAJOR_VERSION > 7
      physics::ModelPtr cargo = GetWorld()->ModelByName(cargo_name_);
      #else
      physics::ModelPtr cargo = GetWorld()->GetModel(cargo_name_);
      #endif
      if (cargo == nullptr)
        return;
      lock_ = false;

      SnapToAvailableBerth(cargo);

      // Fixed joint with the ISS
      #if GAZEBO_MAJOR_VERSION > 7
      physics::ModelPtr iss = GetWorld()->ModelByName("iss");
      #else
      physics::ModelPtr iss = GetWorld()->GetModel("iss");
      #endif
      if (iss == nullptr)
        return;
      // By this point we are guaranteed to have a dock
      #if GAZEBO_MAJOR_VERSION > 7
      joint_ = GetWorld()->Physics()->CreateJoint("fixed", cargo);
      #else
      joint_ = GetWorld()->GetPhysicsEngine()->CreateJoint("fixed", cargo);
      #endif
      joint_->Attach(cargo->GetLink(), iss->GetLink());

      // If we drop off the cargo in the US lab, 'delete' it afterwards
      bool inside_US_lab;
      #if GAZEBO_MAJOR_VERSION > 7
      inside_US_lab =(cargo->WorldPose().Pos().X() > us_lab_x_min &&
            cargo->WorldPose().Pos().X() < us_lab_x_max &&
            cargo->WorldPose().Pos().Y() > us_lab_y_min &&
            cargo->WorldPose().Pos().Y() < us_lab_y_max &&
            cargo->WorldPose().Pos().Z() > us_lab_z_min &&
            cargo->WorldPose().Pos().Z() < us_lab_z_max);
      #else
      inside_US_lab =(cargo->GetWorldPose().pos.x > us_lab_x_min &&
            cargo->GetWorldPose().pos.x < us_lab_x_max &&
            cargo->GetWorldPose().pos.y > us_lab_y_min &&
            cargo->GetWorldPose().pos.y < us_lab_y_max &&
            cargo->GetWorldPose().pos.z > us_lab_z_min &&
            cargo->GetWorldPose().pos.z < us_lab_z_max);
      #endif
      if (!inside_US_lab) {
        // Start colliding with the handrail
        physics::LinkPtr link = cargo->GetLink("body");
        if (link)
          link->SetCollideMode("all");
      }
  }

  // Called when we have berth transforms populated
  void CargoActionCallback(isaac_msgs::CargoState::ConstPtr const& msg) {
    // Check for cargo name and if it matches with this one
    switch (msg->state) {
      case(isaac_msgs::CargoState::PICK_MOVE_FINAL):
        cargo_name_ = msg->cargo_id;
        return fsm_.Update(PICKED);
      case(isaac_msgs::CargoState::DROP_ARM_OPEN):
        return fsm_.Update(DROPPED);
    }
  }

 private:
  ff_util::FSM fsm_;
  bool lock_;
  std::string available_berth_name_;
  double magnetic_capture_distance_;
  double magnetic_surface_offset_;
  event::ConnectionPtr update_;
  ros::Subscriber sub_cargo_;
  ros::ServiceServer server_despawn_;
  std::string cargo_name_;
  std::map<std::string, ignition::math::Pose3d> berths_;
  ignition::math::Pose3d rel_pose_;
  physics::JointPtr joint_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginCargo)

}   // namespace gazebo
