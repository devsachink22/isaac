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

#ifndef PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTNODE_HPP_
#define PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTNODE_HPP_

#include <optional>
#include <memory>

#include <plansys2_domain_expert/DomainExpert.hpp>

#include <std_msgs/String.h>
//#include "lifecycle_msgs/msg/state.hpp"
//#include "lifecycle_msgs/msg/transition.hpp"
#include <plansys2_msgs/GetDomainName.h>
#include <plansys2_msgs/GetDomainTypes.h>
#include <plansys2_msgs/GetDomainActions.h>
#include <plansys2_msgs/GetDomainActionDetails.h>
#include <plansys2_msgs/GetDomainDurativeActionDetails.h>
#include <plansys2_msgs/GetDomain.h>
#include <plansys2_msgs/GetNodeDetails.h>
#include <plansys2_msgs/GetStates.h>

#include <ros/ros.h>
#include <lifecycle/managed_node.h>

namespace plansys2
{

/// DomainExpertNode contains a model, and manages the requests from the DomainExpertClient.
class DomainExpertNode : public ros::lifecycle::ManagedNode
{
public:
  /// Create a new DomainExpertNode
  DomainExpertNode(const ros::NodeHandle &_nh);

  /// Configures domain by creating a DomainExpert object
  /**
   * \param[in] state LifeCycle Node's state
   * \return Success or Failure
   */
  bool onConfigure();

  /// Activates the node
  /**
   * \param[in] state LifeCycle Node's state
   * \return Success or Failure
   */
  bool onActivate();

  /// Deactivates the node
  /**
   * \param[in] state LifeCycle Node's state
   * \return Success or Failure
   */
  bool onDeactivate();

  /// Cleans up the node
  /**
   * \param[in] state LifeCycle Node's state
   * \return Success or Failure
   */
  bool onCleanup();

  /// Shuts down the node
  /**
   * \param[in] state LifeCycle Node's state
   * \return Success or Failure
   */
  bool onShutdown();

  /// Manages the error in the node
  /**
   * \param[in] state LifeCycle Node's state
   * \return Success or Failure
   */
  bool onError(const std::exception &);


  /// Receives the result of the GetDomainName service call
  /**
   * \param[in] request_header The header of the request
   * \param[in] request The request
   * \param[out] request The response
   */
  bool get_domain_name_service_callback(
    plansys2_msgs::GetDomainName::Request &request,
    plansys2_msgs::GetDomainName::Response &response);

  /// Receives the result of the GetDomainTypes service call
  /**
   * \param[in] request_header The header of the request
   * \param[in] request The request
   * \param[out] request The response
   */
  bool get_domain_types_service_callback(
    plansys2_msgs::GetDomainTypes::Request &request,
    plansys2_msgs::GetDomainTypes::Response &response);

  /// Receives the result of the GetDomainActions service call
  /**
   * \param[in] request_header The header of the request
   * \param[in] request The request
   * \param[out] request The response
   */
  bool get_domain_actions_service_callback(
    plansys2_msgs::GetDomainActions::Request &request,
    plansys2_msgs::GetDomainActions::Response &response);

  /// Receives the result of the GetDomainActionDetails service call
  /**
   * \param[in] request_header The header of the request
   * \param[in] request The request
   * \param[out] request The response
   */
  bool get_domain_action_details_service_callback(
    plansys2_msgs::GetDomainActionDetails::Request &request,
    plansys2_msgs::GetDomainActionDetails::Response &response);

  /// Receives the result of the GetDomainActions service call
  /**
   * \param[in] request_header The header of the request
   * \param[in] request The request
   * \param[out] request The response
   */
  bool get_domain_durative_actions_service_callback(
    plansys2_msgs::GetDomainActions::Request &request,
    plansys2_msgs::GetDomainActions::Response &response);

  /// Receives the result of the GetDomainDurativeActionDetails service call
  /**
   * \param[in] request_header The header of the request
   * \param[in] request The request
   * \param[out] request The response
   */
  bool get_domain_durative_action_details_service_callback(
    plansys2_msgs::GetDomainDurativeActionDetails::Request &request,
    plansys2_msgs::GetDomainDurativeActionDetails::Response &response);

  /// Receives the result of the GetDomainPredicates service call
  /**
   * \param[in] request_header The header of the request
   * \param[in] request The request
   * \param[out] request The response
   */
  bool get_domain_predicates_service_callback(
    plansys2_msgs::GetStates::Request &request,
    plansys2_msgs::GetStates::Response &response);

  /// Receives the result of the GetNodeDetails service call
  /**
   * \param[in] request_header The header of the request
   * \param[in] request The request
   * \param[out] request The response
   */
  bool get_domain_predicate_details_service_callback(
    plansys2_msgs::GetNodeDetails::Request &request,
    plansys2_msgs::GetNodeDetails::Response &response);

  /// Receives the result of the GetDomainFunctions service call
  /**
   * \param[in] request_header The header of the request
   * \param[in] request The request
   * \param[out] request The response
   */
  bool get_domain_functions_service_callback(
    plansys2_msgs::GetStates::Request &request,
    plansys2_msgs::GetStates::Response &response);

  /// Receives the result of the GetDomainFunctionDetails service call
  /**
   * \param[in] request_header The header of the request
   * \param[in] request The request
   * \param[out] request The response
   */
  bool get_domain_function_details_service_callback(
    plansys2_msgs::GetNodeDetails::Request &request,
    plansys2_msgs::GetNodeDetails::Response &response);

  /// Receives the result of the GetDomain service call
  /**
   * \param[in] request_header The header of the request
   * \param[in] request The request
   * \param[out] request The response
   */
  bool get_domain_service_callback(
    plansys2_msgs::GetDomain::Request &request,
    plansys2_msgs::GetDomain::Response &response);

  std::string getNodeName() { return node_name_; }
  
private:
  std::shared_ptr<DomainExpert> domain_expert_;

  ros::ServiceServer get_name_service_;
  ros::ServiceServer get_types_service_;
  ros::ServiceServer get_domain_actions_service_;
  ros::ServiceServer get_domain_action_details_service_;
  ros::ServiceServer get_domain_durative_actions_service_;
  ros::ServiceServer get_domain_durative_action_details_service_;
  ros::ServiceServer get_domain_predicates_service_;
  ros::ServiceServer get_domain_predicate_details_service_;
  ros::ServiceServer get_domain_functions_service_;
  ros::ServiceServer get_domain_function_details_service_;
  ros::ServiceServer get_domain_service_;

  std::string node_name_;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTNODE_HPP_
