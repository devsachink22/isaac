
#include <lifecycle/managed_node.h>

namespace ros
{
  namespace lifecycle
  {

    class CascadeLifecycleNode : public ManagedNode
    {

    public:
      CascadeLifecycleNode(const ros::NodeHandle& nh);
      
      
    }; // class Cascade
  }

}
