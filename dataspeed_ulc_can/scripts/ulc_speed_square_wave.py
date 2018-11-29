#! /usr/bin/env python
import rospy
from ulc_speed import UlcSpeed


class UlcSpeedSquareWave(UlcSpeed):

    def __init__(self):
        rospy.init_node('ulc_speed_square_wave')
        super(UlcSpeedSquareWave, self).__init__()

    def timer_callback(self, event):
        if self.enabled:
            if self.v1 == 0 and self.v2 == 0:
                rospy.logwarn_throttle(1.0, 'both speed targets are zero')

            if self.t >= self.period:
                # Reset time when period is reached and switch back to initial speed
                self.t = 0
                self.ulc_cmd.linear_velocity = self.v1
            elif self.t >= 0.5 * self.period:
                # During second half of period, switch to other speed
                self.t += 0.02
                self.ulc_cmd.linear_velocity = self.v2
            else:
                # During first half of period, use initial speed
                self.t += 0.02
                self.ulc_cmd.linear_velocity = self.v1

            self.pub_ulc_cmd.publish(self.ulc_cmd)


if __name__ == '__main__':
    try:
        node_instance = UlcSpeedSquareWave()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
