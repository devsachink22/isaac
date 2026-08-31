
#pragma once

#include <memory>
#include <plansys2_msgs/Param.h>
#include <plansys2_msgs/Node.h>
#include <plansys2_msgs/Tree.h>
#include <plansys2_msgs/Action.h>
#include <plansys2_msgs/DurativeAction.h>

namespace plansys2_msgs
{
  typedef std::shared_ptr< ::plansys2_msgs::Param > ParamSharedPtr;
  typedef std::shared_ptr< ::plansys2_msgs::Node > NodeSharedPtr;
  typedef std::shared_ptr< ::plansys2_msgs::Tree > TreeSharedPtr;
  typedef std::shared_ptr< ::plansys2_msgs::Action > ActionSharedPtr;
  typedef std::shared_ptr< ::plansys2_msgs::DurativeAction > DurativeActionSharedPtr;

}

