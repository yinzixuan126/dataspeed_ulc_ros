#ifndef ULCNODE_H
#define ULCNODE_H

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <dataspeed_dbw_msgs/LatLonCmd.h>
#include <dataspeed_dbw_msgs/LatLonReport.h>
#include "dispatch.h"

namespace dataspeed_ulc_ros
{

class UlcNode
{
public:
  UlcNode(ros::NodeHandle n, ros::NodeHandle pn);
private:

  void recvCan(const can_msgs::FrameConstPtr& msg);
  void recvLatLonCmd(const dataspeed_dbw_msgs::LatLonCmdConstPtr& msg);
  void configTimerCb(const ros::TimerEvent& event);

  inline bool enabled() { return true; }
  inline bool clear() { return false; }

  ros::Subscriber sub_lat_lon_cmd_;
  ros::Subscriber sub_can_;
  ros::Publisher pub_report_;
  ros::Publisher pub_can_;
  ros::Timer config_timer_;

  dataspeed_dbw_msgs::LatLonCmd lat_lon_cmd_;
  dataspeed_dbw_msgs::LatLonReport lat_lon_report_;
};

}

#endif // ULCNODE_H
