#ifndef _DATASPEED_ULC_ROS_DISPATCH_H
#define _DATASPEED_ULC_ROS_DISPATCH_H
#include <stdint.h>

namespace dataspeed_ulc_ros
{

typedef struct {
  int16_t linear_velocity  :16; // 0.0025 m/s, -81.920 to 81.915 m/s, +-183 mph
  int16_t yaw_command      :16; // yaw rate mode: 0.00025 rad/s, -8.1920 to 8.1915 rad/s
                                // curvature mode: 0.0000061 1/m, -0.1999 1/m to 0.1999 1/m

  uint8_t steering_mode    :1;  // 0 = yaw rate mode, 1 = curvature mode
  uint8_t shift_from_park  :1;
  uint8_t enable_shifting  :1;
  uint8_t enable_steering  :1;
  uint8_t enable_pedals    :1;
  uint8_t clear            :1;
  uint8_t                  :2;

  uint8_t                  :8;
  uint8_t                  :8;
  uint8_t wdc;
} MsgLatLonCmd;

typedef struct {
    uint8_t linear_accel;   // 0.02 m/s^2, 0 to 5.1 m/s^2
    uint8_t linear_decel;   // 0.02 m/s^2, 0 to 5.1 m/s^2
    uint8_t lateral_accel;  // 0.05 m/s^2, 0 to 12.75 m/s^2
    uint8_t angular_accel;  // 0.02 rad/s^2, 0 to 5.1 rad/s^2
    uint8_t :8;
    uint8_t :8;
    uint8_t :8;
    uint8_t wdc;
} MsgLatLonConfig;

typedef struct {
  int16_t speed_ref :14; // 0.01 m/s,
  uint16_t speed_enabled :1;
  uint16_t reference_source :1;
  int16_t speed_meas :14; // 0.01 m/s
  uint16_t steering_enabled :1;
  uint16_t steering_mode: 1;
  int8_t  accel_ref; // 0.05 m/s^2
  int8_t accel_meas; // 0.05 m/s^2
  uint8_t max_steering_angle: 7; // 5 deg
  uint8_t override: 1;
  uint8_t max_steering_vel :6; //  8 deg/s
  uint8_t steer_preempted: 1;
  uint8_t speed_preempted: 1;
} MsgLatLonReport;

#define BUILD_ASSERT(cond) do { (void) sizeof(char [1 - 2*!(cond)]); } while(0)
static void dispatchAssertSizes() {
  BUILD_ASSERT(8 == sizeof(MsgLatLonCmd));
  BUILD_ASSERT(8 == sizeof(MsgLatLonConfig));
  BUILD_ASSERT(8 == sizeof(MsgLatLonReport));
}
#undef BUILD_ASSERT

enum {
  ID_LAT_LON_CMD            = 0x076,
  ID_LAT_LON_CONFIG         = 0x077,
  ID_LAT_LON_REPORT         = 0x078,
};

} // namespace dataspeed_ulc_ros

#endif // _DATASPEED_ULC_ROS_DISPATCH_H
