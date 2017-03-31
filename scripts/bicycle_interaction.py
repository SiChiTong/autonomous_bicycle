#!/usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
from tf.transformations import euler_from_quaternion

from dynamic_reconfigure.server import Server
from autonomous_bicycle.cfg import bicycle_interactionConfig

rad2degrees = 180.0/math.pi
degrees2rad = math.pi / 180.0

class BicycleInteraction:
    def __init__(self):
        self.pub_vel_wheel = rospy.Publisher('/bycycle_interaction/vel_wheel', Twist, queue_size=1)

        self.twist = Twist()
        self.vel_wheel = 0
        self.angle_wheel = 0

        self.rate = rospy.get_param('~rate', 3.0)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.30)

        self.srv = Server(bicycle_interactionConfig, self.reconfig_callback) # define dynamic_reconfigure callback

        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            self.publish_vel_wheel()
            rate.sleep()

    def reconfig_callback(self, config, level):
        self.vel_wheel = config['vel_wheel']
        return config

    def publish_vel_wheel(self):
        # send angular velocity to wheels.
        self.twist.linear.x = self.vel_wheel / self.wheel_radius
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = self.angle_wheel * degrees2rad

        self.pub_vel_wheel.publish(self.twist)

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('bicycle_interaction_helper')

    try:
        obj = BicycleInteraction()
    except:
        pass