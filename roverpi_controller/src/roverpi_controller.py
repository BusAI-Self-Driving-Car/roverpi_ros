#! /usr/bin/env python
from __future__ import division

import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

from motor_driver import MotorDriver


class RoverpiController:

    def __init__(self, robot_width=0.16, diameter=0.066, rmp_to_pwm_gain=0.15):
        self.robot_width = robot_width
        self.diameter = diameter
        self.rmp_to_pwm_gain = rmp_to_pwm_gain
        self.vel_x = 0.0
        self.vel_th = 0.0

        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        self.lwheel_pub = rospy.Publisher('lwheel_rpm', Int32, queue_size=1)
        self.rwheel_pub = rospy.Publisher('rwheel_rpm', Int32, queue_size=1)

        self.previous_time = rospy.get_time()

        self.motor = MotorDriver()

    def cmd_vel_callback(self, msg):
        self.vel_x = msg.linear.x
        self.vel_th = msg.angular.z
        self.previous_time = rospy.get_time()

    def run(self):
        rate = rospy.Rate(10)
        self.previous_time = rospy.get_time()
        while not rospy.is_shutdown():

            if rospy.get_time() - self.previous_time > 1:
                rospy.loginfo("Did not get command for 1 second, stopping")
                try:
                    self.motor.e_stop()
                    self.lwheel_pub.publish(0)
                    self.rwheel_pub.publish(0)
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)

            if self.vel_x == 0:
                right_vel = self.vel_th * self.robot_width / 2.0
                left_vel = -right_vel
            elif vel_th == 0:
                left_vel = right_vel = self.vel_x
            else:
                left_vel = self.vel_x - self.vel_th / 2.0
                right_vel = self.vel_x + self.vel_th / 2.0

            RPMleft = int((60 * left_vel) / (self.diameter * math.pi))
            RPMright = int((60 * right_vel) / (self.diameter * math.pi))

            self.lwheel_pub.publish(RPMleft)
            self.rwheel_pub.publish(RPMright)

            PWMleft = self.rmp_to_pwm_gain * RPMleft
            PWMright = self.rmp_to_pwm_gain * RPMright
            self.motor.left_wheel(100, 1500)
            self.motor.right_wheel(100, 1500)

            rate.sleep()

def main():
    rospy.init_node('roverpi_controller')
    nodeName = rospy.get_name()
    rospy.loginfo("{0} started".format(nodeName))

    controller = RoverpiController()
    controller.run()
    controller.motor.exit()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
