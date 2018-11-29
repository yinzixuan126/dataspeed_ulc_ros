#ifndef ULCNODE_H
#define ULCNODE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <can_msgs/Frame.h>
#include <dataspeed_ulc_msgs/UlcCmd.h>
#include <dataspeed_ulc_msgs/UlcReport.h>
#include <dataspeed_ulc_can/dispatch.h>

namespace dataspeed_ulc_can
{

class UlcNode
{
public:
  UlcNode(ros::NodeHandle n, ros::NodeHandle pn);
private:

  void recvCan(const can_msgs::FrameConstPtr& msg);
  void recvUlcCmd(const dataspeed_ulc_msgs::UlcCmdConstPtr& msg);
  void recvTwist(const geometry_msgs::TwistConstPtr& msg);
  void recvTwistStamped(const geometry_msgs::TwistStampedConstPtr& msg);
  void recvEnable(const std_msgs::BoolConstPtr& msg);
  void configTimerCb(const ros::TimerEvent& event);
  void sendCmdMsg();
  static void processTwistInput(const geometry_msgs::Twist& twist, dataspeed_ulc_msgs::UlcCmd& cmd);

  ros::Subscriber sub_lat_lon_cmd_;
  ros::Subscriber sub_twist_;
  ros::Subscriber sub_twist_stamped_;
  ros::Subscriber sub_can_;
  ros::Subscriber sub_enable_;
  ros::Publisher pub_report_;
  ros::Publisher pub_can_;
  ros::Timer config_timer_;

  dataspeed_ulc_msgs::UlcCmd ulc_cmd_;
  ros::Time cmd_stamp_;
  bool enable_;
};

}

#endif // ULCNODE_H
