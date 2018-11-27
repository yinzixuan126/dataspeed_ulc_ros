#include "UlcNode.h"

namespace dataspeed_ulc_can
{

UlcNode::UlcNode(ros::NodeHandle n, ros::NodeHandle pn) :
  enable_(false)
{
  pub_report_ = n.advertise<dataspeed_ulc_msgs::UlcReport>("ulc_report", 2);
  pub_can_ = n.advertise<can_msgs::Frame>("can_tx", 10);

  sub_can_ = n.subscribe<can_msgs::Frame>("can_rx", 100, &UlcNode::recvCan, this);
  sub_lat_lon_cmd_ = n.subscribe<dataspeed_ulc_msgs::UlcCmd>("ulc_cmd", 1, &UlcNode::recvUlcCmd, this);
  sub_twist_ = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &UlcNode::recvTwist, this);
  sub_twist_stamped_ = n.subscribe<geometry_msgs::TwistStamped>("cmd_vel_stamped", 1, &UlcNode::recvTwistStamped, this);
  sub_enable_ = n.subscribe<std_msgs::Bool>("dbw_enabled", 1, &UlcNode::recvEnable, this);

  double config_frequency = 5.0;
  pn.getParam("config_frequency", config_frequency);
  if (config_frequency > 50.0) {
    config_frequency = 50.0;
  } else if (config_frequency < 5.0) {
    config_frequency = 5.0;
  }
  config_timer_ = n.createTimer(ros::Duration(1.0 / config_frequency), &UlcNode::configTimerCb, this);
  cmd_timer_ = n.createTimer(ros::Duration(0.02), &UlcNode::cmdTimerCb, this);
}

void UlcNode::recvEnable(const std_msgs::BoolConstPtr& msg)
{
  enable_ = msg->data;
}

void UlcNode::recvUlcCmd(const dataspeed_ulc_msgs::UlcCmdConstPtr& msg)
{
  ulc_cmd_ = *msg;
  cmd_stamp_ = ros::Time::now();
}

void UlcNode::processTwistInputs(const geometry_msgs::Twist& twist, dataspeed_ulc_msgs::UlcCmd& cmd)
{
  cmd.clear = false;
  cmd.enable_pedals = true;
  cmd.enable_shifting = true;
  cmd.enable_steering = true;
  cmd.shift_from_park = false;
  cmd.linear_accel = 0;
  cmd.linear_decel = 0;
  cmd.angular_accel = 0;
  cmd.lateral_accel = 0;
  cmd.linear_velocity = twist.linear.x;
  cmd.yaw_command = twist.angular.z;
  cmd.steering_mode = dataspeed_ulc_msgs::UlcCmd::YAW_RATE_MODE;
}

void UlcNode::recvTwist(const geometry_msgs::TwistConstPtr& msg)
{
  processTwistInputs(*msg, ulc_cmd_);
  cmd_stamp_ = ros::Time::now();
}

void UlcNode::recvTwistStamped(const geometry_msgs::TwistStampedConstPtr& msg)
{
  processTwistInputs(msg->twist, ulc_cmd_);
  cmd_stamp_ = ros::Time::now();
}

void UlcNode::recvCan(const can_msgs::FrameConstPtr& msg)
{
  if (!msg->is_rtr && !msg->is_error && !msg->is_extended) {
    switch (msg->id) {
      case ID_ULC_REPORT:
        if (msg->dlc >= sizeof(MsgUlcReport)) {
          const MsgUlcReport *ptr = (const MsgUlcReport *)msg->data.elems;
          dataspeed_ulc_msgs::UlcReport ulc_report;
          ulc_report.header.stamp = msg->header.stamp;
          ulc_report.speed_ref = (float)ptr->speed_ref * 0.02f;
          ulc_report.accel_ref = (float)ptr->accel_ref * 0.05f;
          ulc_report.speed_meas = (float)ptr->speed_meas * 0.02f;
          ulc_report.accel_meas = (float)ptr->accel_meas * 0.05f;
          ulc_report.max_steering_angle = (float)ptr->max_steering_angle * 5.0f;
          ulc_report.max_steering_vel = (float)ptr->max_steering_vel * 8.0f;
          ulc_report.speed_enabled = ptr->speed_enabled;
          ulc_report.steering_enabled = ptr->steering_enabled;
          ulc_report.reference_source = ptr->reference_source;
          ulc_report.speed_preempted = ptr->speed_preempted;
          ulc_report.steering_preempted = ptr->steer_preempted;
          ulc_report.override_latched = ptr->override;
          ulc_report.steering_mode = ptr->steering_mode;
          ulc_report.timeout = ptr->timeout;
          pub_report_.publish(ulc_report);
        }
        break;
    }
  }
}

template <class T>
static T overflowSaturation(float input, T limit_min, T limit_max, float scale_factor, const std::string& input_name, const std::string& units)
{
  if (input < (limit_min * scale_factor)) {
    ROS_WARN("%s [%f %s] out of range -- saturating to %f %s", input_name.c_str(), input, units.c_str(), limit_min * scale_factor, units.c_str());
    return limit_min;
  } else if (input > (limit_max * scale_factor)) {
    ROS_WARN("%s [%f %s] out of range -- saturating to %f %s", input_name.c_str(), input, units.c_str(), limit_max * scale_factor, units.c_str());
    return limit_max;
  } else {
    return input / scale_factor;
  }
}

void UlcNode::cmdTimerCb(const ros::TimerEvent& event)
{
  if ((event.current_real - cmd_stamp_).toSec() > 0.1) {
    return;
  }

  can_msgs::Frame cmd_out;
  cmd_out.id = ID_ULC_CMD;
  cmd_out.is_extended = false;
  cmd_out.dlc = sizeof(MsgUlcCmd);
  MsgUlcCmd *cmd_ptr = (MsgUlcCmd *)cmd_out.data.elems;
  memset(cmd_ptr, 0x00, sizeof(*cmd_ptr));

  // Populate enable bits
  if (enable_) {
    cmd_ptr->enable_pedals = ulc_cmd_.enable_pedals;
    cmd_ptr->enable_steering = ulc_cmd_.enable_steering;
    cmd_ptr->enable_shifting = ulc_cmd_.enable_shifting;
    cmd_ptr->shift_from_park = ulc_cmd_.shift_from_park;
  }

  cmd_ptr->clear = ulc_cmd_.clear;
  cmd_ptr->linear_velocity = overflowSaturation(ulc_cmd_.linear_velocity, INT16_MIN, INT16_MAX, 0.0025f, "ULC command speed", "m/s");
  cmd_ptr->steering_mode = ulc_cmd_.steering_mode;
  if (ulc_cmd_.steering_mode == dataspeed_ulc_msgs::UlcCmd::YAW_RATE_MODE) {
    cmd_ptr->yaw_command = overflowSaturation(ulc_cmd_.yaw_command, INT16_MIN, INT16_MAX, 0.00025f, "ULC yaw rate command", "rad/s");
  } else if (ulc_cmd_.steering_mode == dataspeed_ulc_msgs::UlcCmd::CURVATURE_MODE) {
    cmd_ptr->yaw_command = overflowSaturation(ulc_cmd_.yaw_command, INT16_MIN, INT16_MAX, 0.0000061f, "ULC curvature command", "1/m");
  } else {
    cmd_ptr->yaw_command = 0;
    ROS_WARN_THROTTLE(1.0, "Unsupported ULC steering control mode [%d]", ulc_cmd_.steering_mode);
  }

  pub_can_.publish(cmd_out);
}

void UlcNode::configTimerCb(const ros::TimerEvent& event)
{
  if ((event.current_real - cmd_stamp_).toSec() > 0.1) {
    return;
  }

  can_msgs::Frame config_out;
  config_out.id = ID_ULC_CONFIG;
  config_out.is_extended = false;
  config_out.dlc = sizeof(MsgUlcCfg);
  MsgUlcCfg *config_ptr = (MsgUlcCfg *)config_out.data.elems;
  config_ptr->linear_accel = overflowSaturation(ulc_cmd_.linear_accel, 0, UINT8_MAX, 0.02f, "Linear accel limit", "m/s^2");
  config_ptr->linear_decel = overflowSaturation(ulc_cmd_.linear_decel, 0, UINT8_MAX, 0.02f, "Linear decel limit", "m/s^2");
  config_ptr->lateral_accel = overflowSaturation(ulc_cmd_.lateral_accel, 0, UINT8_MAX, 0.05f, "Lateral accel limit", "m/s^2");
  config_ptr->angular_accel = overflowSaturation(ulc_cmd_.angular_accel, 0, UINT8_MAX, 0.02f, "Angular accel limit", "rad/s^2");
  pub_can_.publish(config_out);
}

}
