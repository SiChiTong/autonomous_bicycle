#!/usr/bin/env python

import time
import rospy
import numpy as np
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist
from tf.broadcaster import TransformBroadcaster
from sensor_msgs.msg import Imu

from classes.ImuDriver import *


class RobotImuPublisherNode:
    def __init__(self):
        self.degrees2rad = math.pi/180.0

        # Get values from launch file
        self.rate = rospy.get_param('~rate', 80.0)  # the rate at which to publish the transform
        # Static transform between sensor and fixed frame: x, y, z, roll, pitch, yaw
        # <rosparam param="static_transform">[0, 0, 0, 0, 0, 0]</rosparam>
        self.static_transform = rospy.get_param('~static_transform', [0, 0, 0, 0, 0, 0])
        self.serial_port = rospy.get_param('~serial_port', "/dev/ttyUSB0")
        self.topic_name = rospy.get_param('~topic_name', "/imu")
        self.fixed_frame = rospy.get_param('~fixed_frame', "world")
        self.frame_name = rospy.get_param('~frame_name', "imu")
        self.publish_transform = rospy.get_param('~publish_transform', False)

        self.imu = ImuDriver(serial_port=self.serial_port)
        self.imu.init_imu()

        # Create a publisher for imu message
        self.pub_imu = rospy.Publisher(self.topic_name, Imu, queue_size=1)
        self.odomBroadcaster_imu = TransformBroadcaster()

        self.imu_msg = Imu()
        self.imu_msg.orientation_covariance[0] = -1
        self.imu_msg.angular_velocity_covariance[0] = -1
        self.imu_msg.linear_acceleration_covariance[0] = -1

        self.current_time = rospy.get_time()
        self.last_time = rospy.get_time()

        self.seq = 0

        rospy.on_shutdown(self.shutdown_node)
        rate = rospy.Rate(self.rate)

        rospy.loginfo("Ready for publishing imu:" + self.serial_port)

        # Main while loop.
        while not rospy.is_shutdown():
            self.current_time = rospy.get_time()

            self.imu.read()

            if self.publish_transform:
                quaternion = self.imu.quaternion_from_euler(self.static_transform[3]*self.degrees2rad,
                                                            self.static_transform[4]*self.degrees2rad,
                                                            self.static_transform[5]*self.degrees2rad)

                # send static transformation tf between imu and fixed frame
                self.odomBroadcaster_imu.sendTransform(
                    (self.static_transform[0], self.static_transform[1], self.static_transform[2]),
                    (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                    rospy.Time.now(), self.frame_name, self.fixed_frame
                )

            # publish imu message
            self.publish_info(imu=self.imu)
            rate.sleep()

    def publish_info(self, imu):
        self.imu_msg = Imu()
        self.imu_msg.linear_acceleration.x = imu.linear_acceleration_x
        self.imu_msg.linear_acceleration.y = imu.linear_acceleration_y
        self.imu_msg.linear_acceleration.z = imu.linear_acceleration_z

        self.imu_msg.angular_velocity.x = imu.angular_velocity_x
        self.imu_msg.angular_velocity.y = imu.angular_velocity_y
        self.imu_msg.angular_velocity.z = imu.angular_velocity_z

        self.imu_msg.orientation.x = imu.orientation_x
        self.imu_msg.orientation.y = imu.orientation_y
        self.imu_msg.orientation.z = imu.orientation_z
        self.imu_msg.orientation.w = imu.orientation_w

        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = self.frame_name
        self.imu_msg.header.seq = self.seq

        self.pub_imu.publish(self.imu_msg)
        self.seq += 1

    def shutdown_node(self):
        rospy.loginfo("Turning off node: robot_imu_publisher")

# Main function.
if __name__ == '__main__':
    rospy.loginfo('Starting RobotImuPublisherNode')

    # Initialize the node and name it.
    rospy.init_node('sensor_imu_publisher')

    try:
        obj_temp = RobotImuPublisherNode()
    except rospy.ROSInterruptException:
        pass
