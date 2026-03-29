#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist


class CmdVelAckermannGuard:
    def __init__(self):
        rospy.init_node("cmd_vel_ackermann_guard", anonymous=True)

        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_nav_raw")
        self.output_topic = rospy.get_param("~output_topic", "/four_wheel_steering_controller/cmd_vel")
        self.min_linear_when_turning = rospy.get_param("~min_linear_when_turning", 0.12)
        self.angular_epsilon = rospy.get_param("~angular_epsilon", 0.03)
        self.linear_epsilon = rospy.get_param("~linear_epsilon", 0.02)
        self.keep_sign = rospy.get_param("~keep_sign", True)

        self.pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Twist, self.cb, queue_size=50)

        rospy.loginfo("[ackermann_guard] %s -> %s (min_linear_when_turning=%.3f)",
                      self.input_topic, self.output_topic, self.min_linear_when_turning)

    def cb(self, msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z

        # 关键逻辑：如果只有角速度而无线速度，则补一个最小线速度
        if abs(out.angular.z) > self.angular_epsilon and abs(out.linear.x) < self.linear_epsilon:
            if self.keep_sign and out.linear.x < 0:
                out.linear.x = -self.min_linear_when_turning
            else:
                out.linear.x = self.min_linear_when_turning

        self.pub.publish(out)


if __name__ == "__main__":
    try:
        CmdVelAckermannGuard()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
