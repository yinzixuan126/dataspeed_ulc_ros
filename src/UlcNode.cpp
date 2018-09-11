#include "UlcNode.h"

namespace dataspeed_ulc_ros
{

UlcNode::UlcNode(ros::NodeHandle n, ros::NodeHandle pn)
{
  pub_report_ = n.advertise<dataspeed_dbw_msgs::LatLonReport>("lat_lon_report", 2);
  pub_can_ = n.advertise<can_msgs::Frame>("can_tx", 10);

  sub_can_ = n.subscribe<can_msgs::Frame>("can_rx", 100, &UlcNode::recvCan, this);
  sub_lat_lon_cmd_ = n.subscribe<dataspeed_dbw_msgs::LatLonCmd>("lat_lon_cmd", 1, &UlcNode::recvLatLonCmd, this);

  double config_frequency = 5.0;
  pn.getParam("config_frequency", config_frequency);
  config_frequency = (config_frequency > 50) ? 50.0 : config_frequency;
  config_timer_ = n.createTimer(ros::Duration(1.0 / config_frequency), &UlcNode::configTimerCb, this);

}

void UlcNode::recvLatLonCmd(const dataspeed_dbw_msgs::LatLonCmdConstPtr& msg)
{
  lat_lon_cmd_ = *msg;

  can_msgs::Frame cmd_out;
  cmd_out.id = ID_LAT_LON_CMD;
  cmd_out.is_extended = false;
  cmd_out.dlc = sizeof(MsgLatLonCmd);
  MsgLatLonCmd *cmd_ptr = (MsgLatLonCmd *)cmd_out.data.elems;
  memset(cmd_ptr, 0x00, sizeof(*cmd_ptr));
  if (enabled()) {
    // Populate flag bits
    cmd_ptr->enable_pedals = msg->enable_pedals;
    cmd_ptr->enable_steering = msg->enable_steering;
    cmd_ptr->enable_shifting = msg->enable_shifting;
    cmd_ptr->shift_from_park = msg->shift_from_park;

    // Populate speed command with range check
    if (msg->linear_velocity < (INT16_MIN * 0.0025f)) {
      cmd_ptr->linear_velocity = INT16_MIN;
      ROS_WARN_THROTTLE(1.0, "LatLon command speed [%f m/s] out of range -- saturating to %f m/s", msg->linear_velocity, INT16_MIN * 0.0025f);
    } else if (msg->linear_velocity > (INT16_MAX * 0.0025f)) {
      cmd_ptr->linear_velocity = INT16_MAX;
      ROS_WARN_THROTTLE(1.0, "LatLon command speed [%f m/s] out of range -- saturating to %f m/s", msg->linear_velocity, INT16_MAX * 0.0025f);
    } else {
      cmd_ptr->linear_velocity = (int16_t) (msg->linear_velocity / 0.0025f);
    }

    // Populate yaw command with range check
    cmd_ptr->steering_mode = msg->steering_mode;
    if (msg->steering_mode == dataspeed_dbw_msgs::LatLonCmd::YAW_RATE_MODE) {
      if (msg->yaw_command < (INT16_MIN * 0.00025f)) {
        cmd_ptr->yaw_command = INT16_MIN;
        ROS_WARN_THROTTLE(1.0, "LatLon yaw rate command [%f rad/s] out of range -- saturating to %f rad/s", msg->yaw_command, INT16_MIN * 0.00025f);
      } else if (msg->yaw_command > (INT16_MAX * 0.00025f)) {
        cmd_ptr->yaw_command = INT16_MAX;
        ROS_WARN_THROTTLE(1.0, "LatLon yaw rate command [%f rad/s] out of range -- saturating to %f rad/s", msg->yaw_command, INT16_MAX * 0.00025f);
      } else {
        cmd_ptr->yaw_command = (int16_t)(msg->yaw_command / 0.00025f);
      }
    } else if (msg->steering_mode == dataspeed_dbw_msgs::LatLonCmd::CURVATURE_MODE) {
      if (msg->yaw_command < (INT16_MIN * 0.0000061f)) {
        cmd_ptr->yaw_command = INT16_MIN;
        ROS_WARN_THROTTLE(1.0, "LatLon curvature command [%f 1/m] out of range -- saturating to %f 1/m", msg->yaw_command, INT16_MIN * 0.0000061f);
      } else if (msg->yaw_command > (INT16_MAX * 0.0000061f)) {
        cmd_ptr->yaw_command = INT16_MAX;
        ROS_WARN_THROTTLE(1.0, "LatLon curvature command [%f 1/m] out of range -- saturating to %f 1/m", msg->yaw_command, INT16_MAX * 0.0000061f);
      } else {
        cmd_ptr->yaw_command = (int16_t)(msg->yaw_command / 0.0000061f);
      }
    } else {
      cmd_ptr->yaw_command = 0;
      ROS_WARN_THROTTLE(1.0, "Unsupported LatLon steering control mode [%d]", msg->steering_mode);
    }
  }
  if (clear() || msg->clear) {
    cmd_ptr->clear = 1;
  }
  pub_can_.publish(cmd_out);
}

void UlcNode::recvCan(const can_msgs::FrameConstPtr& msg)
{
  if (!msg->is_rtr && !msg->is_error && !msg->is_extended) {
    switch (msg->id) {
      case ID_LAT_LON_REPORT:
        if (msg->dlc >= sizeof(MsgLatLonReport)) {
          const MsgLatLonReport *ptr = (const MsgLatLonReport *)msg->data.elems;
          lat_lon_report_.header.stamp = msg->header.stamp;
          lat_lon_report_.speed_ref = (float)ptr->speed_ref / 256.0f;
          lat_lon_report_.accel_ref = (float)ptr->accel_ref * 0.05f;
          lat_lon_report_.speed_meas = (float)ptr->speed_meas / 256.0f;
          lat_lon_report_.accel_meas = (float)ptr->accel_meas * 0.05f;
          lat_lon_report_.max_steering_angle = (float)ptr->max_steering_angle * 2.0f;
          lat_lon_report_.max_steering_vel = (float)ptr->max_steering_vel * 2.0f;
          lat_lon_report_.speed_enabled = ptr->speed_enabled;
          lat_lon_report_.steering_enabled = ptr->steering_enabled;
          lat_lon_report_.reference_source = ptr->reference_source;
          lat_lon_report_.steering_mode = ptr->steering_mode;
          pub_report_.publish(lat_lon_report_);
        }
        break;
    }
  }
}

void UlcNode::configTimerCb(const ros::TimerEvent& event)
{
  can_msgs::Frame config_out;
  config_out.id = ID_LAT_LON_CONFIG;
  config_out.is_extended = false;
  config_out.dlc = sizeof(MsgLatLonConfig);
  MsgLatLonConfig *config_ptr = (MsgLatLonConfig *)config_out.data.elems;

  // Populate linear accel limit with range check
  if (lat_lon_cmd_.linear_accel < 0.0) {
    config_ptr->linear_accel = 0;
    ROS_WARN_THROTTLE(1.0, "Linear accel limit [%f m/s^2] out of range -- saturating to 0", lat_lon_cmd_.linear_accel);
  } else if (lat_lon_cmd_.linear_accel >= (UINT8_MAX * 0.02f)) {
    config_ptr->linear_accel = UINT8_MAX;
    ROS_WARN_THROTTLE(1.0, "Linear accel limit [%f m/s^2] out of range -- saturating to %f m/s^2", lat_lon_cmd_.linear_accel, UINT8_MAX * 0.02f);
  } else {
    config_ptr->linear_accel = (uint8_t)(lat_lon_cmd_.linear_accel / 0.02f);
  }

  // Populate linear decel limit with range check
  if (lat_lon_cmd_.linear_decel < 0.0) {
    config_ptr->linear_decel = 0;
    ROS_WARN_THROTTLE(1.0, "Linear decel limit [%f m/s^2] out of range -- saturating to 0", lat_lon_cmd_.linear_decel);
  } else if (lat_lon_cmd_.linear_decel >= (UINT8_MAX * 0.02f)) {
    config_ptr->linear_decel = UINT8_MAX;
    ROS_WARN_THROTTLE(1.0, "Linear decel limit [%f m/s^2] out of range -- saturating to %f m/s^2", lat_lon_cmd_.linear_decel, UINT8_MAX * 0.02f);
  } else {
    config_ptr->linear_decel = (uint8_t)(lat_lon_cmd_.linear_decel / 0.02f);
  }

  // Populate angular accel limit with range check
  if (lat_lon_cmd_.angular_accel < 0.0) {
    config_ptr->angular_accel = 0;
    ROS_WARN_THROTTLE(1.0, "Angular accel limit [%f rad/s^2] out of range -- saturating to 0", lat_lon_cmd_.angular_accel);
  } else if (lat_lon_cmd_.angular_accel >= (UINT8_MAX * 0.02f)) {
    config_ptr->angular_accel = UINT8_MAX;
    ROS_WARN_THROTTLE(1.0, "Angular accel limit [%f rad/s^2] out of range -- saturating to %f rad/s^2", lat_lon_cmd_.angular_accel, UINT8_MAX * 0.02f);
  } else {
    config_ptr->angular_accel = (uint8_t)(lat_lon_cmd_.angular_accel / 0.02f);
  }

  // Populate lateral accel limit with range check
  if (lat_lon_cmd_.lateral_accel < 0.0) {
    config_ptr->lateral_accel = 0;
    ROS_WARN_THROTTLE(1.0, "Lateral accel limit [%f m/s^2] out of range -- saturating to 0", lat_lon_cmd_.lateral_accel);
  } else if (lat_lon_cmd_.lateral_accel >= (UINT8_MAX * 0.05f)) {
    config_ptr->lateral_accel = UINT8_MAX;
    ROS_WARN_THROTTLE(1.0, "Lateral accel limit [%f m/s^2] out of range -- saturating to %f m/s^2", lat_lon_cmd_.lateral_accel, UINT8_MAX * 0.05f);
  } else {
    config_ptr->lateral_accel = (uint8_t)(lat_lon_cmd_.lateral_accel / 0.05f);
  }

  pub_can_.publish(config_out);
}

}
