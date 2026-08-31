#pragma once

#include <ros/ros.h>
#include <lifecycle/lifecycle_model.h>
#include <lifecycle/managed_node.h>

namespace ros
{

namespace lifecycle
{

/**
 * @class LifecycleSubscriber
 */  
template<typename T>
class LifecycleSubscriber_ {

 public:
  
  /** @brief Constructor */
 template<class C>
 LifecycleSubscriber_( std::shared_ptr<ManagedNode> _node,
		       std::string _topic_name,
		       void(C::*fp)(const boost::shared_ptr<T const>&),
		       C* obj) :
  node_(_node),
  state_(State::UNCONFIGURED),
  topic_name_(_topic_name)
 {
   sub_ = node_->getBaseNode().subscribe(topic_name_, 10,
					 fp, obj);  
   on_configure();
 }

  void on_configure()
  {
    
    state_ = State::INACTIVE;
  }
  
  void on_deactivate()
  {
    state_ = State::INACTIVE;
  }

  void on_activate()
  {
    state_ = State::ACTIVE;
  }
  

  bool is_active()
  {
    return (state_ == State::ACTIVE);
  }

 protected:

  ros::lifecycle::State state_;
  ros::Subscriber sub_;

  std::shared_ptr<ManagedNode> node_;
  std::string topic_name_;

  
 };

} // namespace lifecycle
 
} // namespace ros
