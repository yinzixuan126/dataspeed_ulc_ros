#! /usr/bin/env python
import rospy
import math
from ulc_speed import UlcSpeed
import dbw_mkz_msgs.msg
import dbw_fca_msgs.msg


class UlcSpeedSineWave(UlcSpeed):
    APPROACHING = 0
    TRACKING = 1

    def __init__(self):
        rospy.init_node('ulc_speed_sine_wave')

        self.speed_meas = 0
        self.reached_target_stamp = -1
        self.state = self.APPROACHING

        # Wait for /vehicle/steering_report topic to be advertised so its
        # message type can be determined between dbw_mkz_msgs/SteeringReport
        # and dbw_fca_msgs/SteeringReport
        while not rospy.is_shutdown():
            found_steering_report = False
            for topic in rospy.get_published_topics('vehicle'):
                if topic[0] == '/vehicle/steering_report':
                    rospy.Subscriber(
                        name=topic[0],
                        data_class=eval(topic[1].split('/')[0] + '.msg.SteeringReport'),
                        callback=self.recv_steering_report
                    )
                    found_steering_report = True
                    rospy.loginfo('Found steering report topic with message type %s' % topic[1])
                    break
            if found_steering_report:
                break
            else:
                rospy.loginfo_once('Waiting for /vehicle/steering_report...')
            rospy.sleep(0.1)

        super(UlcSpeedSineWave, self).__init__()

    def timer_callback(self, event):

        if not self.enabled:
            self.t = 0
            self.state = self.APPROACHING
            return

        if self.state == self.APPROACHING:
            self.ulc_cmd.linear_velocity = self.v1
            self.t = 0
            if abs(self.ulc_cmd.linear_velocity - self.speed_meas) < 0.4 and self.reached_target_stamp < 0:
                self.reached_target_stamp = event.current_real.to_sec()

            # Wait 3 seconds before starting the sine wave input
            if self.reached_target_stamp > 0 and (event.current_real.to_sec() - self.reached_target_stamp) > 3:
                self.state = self.TRACKING
                self.reached_target_stamp = -1

        elif self.state == self.TRACKING:
            amplitude = 0.5 * (self.v2 - self.v1)
            offset = 0.5 * (self.v2 + self.v1)
            self.ulc_cmd.linear_velocity = offset - amplitude * math.cos(2 * math.pi / self.period * self.t)
            self.t += 0.02

        self.pub_ulc_cmd.publish(self.ulc_cmd)
        if self.v1 == 0 and self.v2 == 0:
            rospy.logwarn_throttle(1.0, 'both speed targets are zero')

    def recv_steering_report(self, msg):
        self.speed_meas = msg.speed


if __name__ == '__main__':
    try:
        node_instance = UlcSpeedSineWave()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
