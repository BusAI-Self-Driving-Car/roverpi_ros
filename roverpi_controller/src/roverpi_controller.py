#! /usr/bin/env python
from __future__ import division

import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

from motor_driver import MotorDriver

class RoverpiController:

    '''
    GPIO PORTS
    '''
    in1 = 20
    in2 = 21
    in3 = 23
    in4 = 24
    ena = 25
    enb = 12

    def __init__(self, diameter=0.66):
        self.diameter = diameter
        self.vel_x = 0.0
        self.vel_th = 0.0

        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)

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

            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 1:
                rospy.loginfo("Did not get command for 1 second, stopping")
                try:
                    self.motor.e_stop()
                    self.left_wheel_pub.publish(0)
                    self.right_wheel_pub.publish(0)
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)

            if self.vel_x == 0:
                right_vel = self.vel_th * width_robot / 2.0
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

            PWMleft = k * RPMleft
            PWMright = k * RPMright
            self.motor.left_wheel(100, PWMleft)
            self.motor.right_wheel(100, PWMright)

            rate.sleep()

def main():
    rospy.init_node('roverpi_controller')
    nodeName = rospy.get_name()
    rospy.loginfo("{0} started".format(nodeName))

    controller = RoverpiController()
    controller.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
