#include <ros/ros.h>
#include "UlcNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dataspeed_ulc");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create UlcNode class
  dataspeed_ulc_can::UlcNode n(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
