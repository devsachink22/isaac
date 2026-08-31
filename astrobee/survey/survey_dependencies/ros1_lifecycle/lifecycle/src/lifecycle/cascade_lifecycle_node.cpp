
#include <lifecycle/cascade_lifecycle_node.h>

namespace ros
{
  namespace lifecycle
  {

    CascadeLifecycleNode::CascadeLifecycleNode(const ros::NodeHandle& nh) :
      ManagedNode(nh)
    {
    }

  } // namespace lifecycle
} // namespace ros
