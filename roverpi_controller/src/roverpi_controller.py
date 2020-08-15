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
        self.vx = 0.0
        self.vth = 0.0
        self.vx_max = 1.0
        self.vth_max = 1.0

        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        self.lwheel_pub = rospy.Publisher('lwheel_rpm', Int32, queue_size=1)
        self.rwheel_pub = rospy.Publisher('rwheel_rpm', Int32, queue_size=1)

        self.current_time = None
        self.last_cmd_time = None

        self.motor = MotorDriver()

    def cmd_vel_callback(self, msg):
        self.vx = max(min(msg.linear.x, self.vx_max), -self.vx_max)
        self.vth = max(min(msg.angular.z, self.vth_max), -self.vth_max)
        self.last_cmd_time = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            self.current_time = rospy.Time.now()

            if self.last_cmd_time == None:
                rospy.loginfo("Waiting for command.")
                self.lwheel_pub.publish(0)
                self.rwheel_pub.publish(0)
            elif (self.current_time - self.last_cmd_time).to_sec() > 1.0:
                rospy.loginfo("Did not get command for 1 second, stopping.")
                try:
                    self.motor.e_stop()
                    self.lwheel_pub.publish(0)
                    self.rwheel_pub.publish(0)
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)
            else:
                if abs(self.vx) < 0.01 and abs(self.vth) < 0.01:
                    left_vel = right_vel = 0
                elif abs(self.vx) < 0.01:
                    right_vel = self.vth * self.robot_width / 2.0
                    left_vel = -right_vel
                elif abs(self.vth) < 0.01:
                    left_vel = right_vel = self.vx
                else:
                    left_vel = self.vx - self.vth / 2.0
                    right_vel = self.vx + self.vth / 2.0

                RPMleft = int((60 * left_vel) / (self.diameter * math.pi))
                RPMright = int((60 * right_vel) / (self.diameter * math.pi))

                self.lwheel_pub.publish(RPMleft)
                self.rwheel_pub.publish(RPMright)

                addon = 50.0 # dc to be added

                if RPMleft > 0 and RPMright > 0:
                    self.rmp_to_pwm_gain = addon / (RPMleft + RPMright)
                    PWMleft = 50 + self.rmp_to_pwm_gain * RPMleft
                    PWMright = 50 + self.rmp_to_pwm_gain * RPMright
                    self.motor.left_wheel(PWMleft, 1000)
                    self.motor.right_wheel(PWMright, 1000)
                elif RPMleft < 0 and RPMright < 0:
                    self.rmp_to_pwm_gain = - addon / (RPMleft + RPMright)
                    PWMleft = 50 + self.rmp_to_pwm_gain * -RPMleft
                    PWMright = 50 + self.rmp_to_pwm_gain * -RPMright
                    self.motor.left_wheel(PWMleft, 1000, reverse=True)
                    self.motor.right_wheel(PWMright, 1000, reverse=True)
                elif RPMleft > 0 and RPMright < 0:
                    self.rmp_to_pwm_gain = addon / (RPMleft - RPMright)
                    PWMleft = 50 + self.rmp_to_pwm_gain * RPMleft
                    PWMright = 50 + self.rmp_to_pwm_gain * -RPMright
                    self.motor.left_wheel(PWMleft, 1000)
                    self.motor.right_wheel(PWMright, 1000, reverse=True)
                elif RPMleft < 0 and RPMright > 0:
                    self.rmp_to_pwm_gain = addon / (-RPMleft + RPMright)
                    PWMleft = 50 + self.rmp_to_pwm_gain * -RPMleft
                    PWMright = 50 + self.rmp_to_pwm_gain * RPMright
                    self.motor.left_wheel(PWMleft, 1000, reverse=True)
                    self.motor.right_wheel(PWMright, 1000)
                else:
                    self.motor.e_stop()

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
