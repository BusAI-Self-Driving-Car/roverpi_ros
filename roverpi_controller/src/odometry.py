#!/usr/bin/env python
from math import sin, cos, pi
import rospy
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RoverpiOdometry:

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.left_wheel_sub = rospy.Subscriber("lwheel_ticks", Int32, self.left_callback)
        self.right_wheel_sub = rospy.Subscriber("rwheel_ticks", Int32, self.right_callback)
        self.initial_pose_sub = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.on_initial_pose)
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=100)
        self.odom_broadcaster = TransformBroadcaster()

        self.current_time = rospy.Time.now()
        self.previous_time = rospy.Time.now()

    def on_initial_pose(self, msg):
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

    def left_callback(self, msg):
        pass

    def right_callback(self, msg):
        pass

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vth = msg.angular.z

    def run(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()

            # compute odometry in a typical way given the velocities of the robot
            dt = (self.current_time - self.previous_time).to_sec()

            delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
            delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
            delta_th = self.vth * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            # create quaternion from yaw
            odom_quat = quaternion_from_euler(0, 0, self.th)

            # publish the transform over tf
            self.odom_broadcaster.sendTransform((self.x, self.y, 0),
                                                odom_quat,
                                                self.current_time,
                                                "base_link",
                                                "odom")

            # construct odometry message
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.x = odom_quat[0]
            odom.pose.pose.orientation.y = odom_quat[1]
            odom.pose.pose.orientation.z = odom_quat[2]
            odom.pose.pose.orientation.w = odom_quat[3]

            odom.twist.twist.linear.x = self.vx
            odom.twist.twist.linear.y = self.vy
            odom.twist.twist.angular.z = self.vth

            # publish the message
            self.odom_pub.publish(odom)

            self.previous_time = self.current_time
            rate.sleep()

def main():
    rospy.init_node('roverpi_odometry')
    nodeName = rospy.get_name()
    rospy.loginfo("{0} started".format(nodeName))

    odometry = RoverpiOdometry()
    odometry.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
