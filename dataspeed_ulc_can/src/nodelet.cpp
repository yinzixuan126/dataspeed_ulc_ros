#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "UlcNode.h"

namespace dataspeed_ulc_can
{

class UlcNodelet : public nodelet::Nodelet
{
public:
  UlcNodelet()
  {
  }
  ~UlcNodelet()
  {
  }

  void onInit(void)
  {
    node_.reset(new UlcNode(getNodeHandle(), getPrivateNodeHandle()));
  }

private:
  boost::shared_ptr<UlcNode> node_;
};

} // namespace dataspeed_ulc_ros

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(dataspeed_ulc_can::UlcNodelet, nodelet::Nodelet);
