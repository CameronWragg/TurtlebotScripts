#!/usr/bin/env python

import rospy
import cv2
import numpy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

wheel_radius = 0.05
robot_radius = 0.25

# computing the forward kinematics for a differential drive
def forward_kinematics(w_l, w_r):
    c_l = wheel_radius * w_l
    c_r = wheel_radius * w_r
    v = (c_l + c_r) / 2
    a = (c_r - c_l) / (2 * robot_radius)
    return (v, a)

# computing the inverse kinematics for a differential drive
def inverse_kinematics(v, a):
    c_l = v - (robot_radius * a)
    c_r = v + (robot_radius * a)
    w_l = c_l / wheel_radius
    w_r = c_r / wheel_radius
    return (w_l, w_r)

# inverse kinematics from a Twist message (This is what a ROS robot has to do)
def inverse_kinematics_from_twist(t):
    return inverse_kinematics(t.linear.x, t.angular.z)

class kinematics_script:

    def __init__(self):

        self.sub = rospy.Subscriber("/wheel_vel_left", Float32, self.callback)
        self.pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

    def callback(self, data):
        #(w_l, w_r) = inverse_kinematics(0.0, 0.0)
        #print "w_l = %f,\tw_r = %f" % (w_l, w_r)
        w_l_d = data.data
        print(w_l_d)
        w_r = 0

        (v, a) = forward_kinematics(w_l_d, w_r)
        print "v = %f,\ta = %f" % (v, a)

        t = Twist()
        t.linear.x = v
        t.angular.z = a
        self.pub.publish(t)

        #(w_l, w_r) = inverse_kinematics_from_twist(t)
        #print "w_l = %f,\tw_r = %f" % (w_l, w_r)

kinematics_script()
rospy.init_node('kinematics_script', anonymous=True)
rospy.spin()
