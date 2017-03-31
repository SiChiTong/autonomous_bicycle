#!/usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

rad2degrees = 180.0/math.pi
degrees2rad = math.pi / 180.0

class TwoImu2Angles:
    def __init__(self):
        self.rate = rospy.get_param('~rate', 100.0)
        self.topic_name_angle = rospy.get_param('~topic_name_angle', 'topic_name_angle')
        self.topic_name_reference = rospy.get_param('~topic_name_reference', 'topic_name_reference')
        self.imu_name = rospy.get_param('~imu_name', 'imu_steer')

        self.roll_steer = 0.0
        self.pitch_steer = 0.0
        self.yaw_steer = 0.0

        self.roll_frame = 0.0
        self.pitch_frame = 0.0
        self.yaw_frame = 0.0

        self.enable_reference = False
        self.enable_angle = False

        self.pub_imu_roll_msg = Float32()
        self.pub_imu_pitch_msg = Float32()
        self.pub_imu_yaw_msg = Float32()

        self.pub_imu_roll = rospy.Publisher('/' + self.imu_name +'/roll', Float32, queue_size=1)
        self.pub_imu_pitch = rospy.Publisher('/' + self.imu_name +'/pitch', Float32, queue_size=1)
        self.pub_imu_yaw = rospy.Publisher('/' + self.imu_name +'/yaw', Float32, queue_size=1)

        self.sub = rospy.Subscriber(self.topic_name_angle, Imu, self.process_imu_message_angle, queue_size=1)
        self.sub = rospy.Subscriber(self.topic_name_reference, Imu, self.process_imu_message_reference, queue_size=1)

        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            rate.sleep()

    def process_imu_message_angle(self, imuMsg):
        quaternion = (
            imuMsg.orientation.x,
            imuMsg.orientation.y,
            imuMsg.orientation.z,
            imuMsg.orientation.w)

        (self.roll_steer, self.pitch_steer, self.yaw_steer) = euler_from_quaternion(quaternion)

        self.roll_steer = self.roll_steer * rad2degrees
        self.pitch_steer = self.pitch_steer * rad2degrees
        self.yaw_steer = self.yaw_steer * rad2degrees

        self.enable_angle = True

        self.publish_angles()

    def process_imu_message_reference(self, imuMsg):
        quaternion = (
            imuMsg.orientation.x,
            imuMsg.orientation.y,
            imuMsg.orientation.z,
            imuMsg.orientation.w)

        (self.roll_frame, self.pitch_frame, self.yaw_frame) = euler_from_quaternion(quaternion)

        self.roll_frame = self.roll_frame * rad2degrees
        self.pitch_frame = self.pitch_frame * rad2degrees
        self.yaw_frame = self.yaw_frame * rad2degrees

        self.enable_reference = True

        self.publish_angles()

    def publish_angles(self):
        if  self.enable_reference and self.enable_angle:
            self.pub_imu_roll_msg = Float32()
            self.pub_imu_roll_msg.data = self.roll_frame - self.roll_steer
            self.pub_imu_roll.publish(self.pub_imu_roll_msg)

            self.pub_imu_pitch_msg = Float32()
            self.pub_imu_pitch_msg.data = self.pitch_frame - self.pitch_steer
            self.pub_imu_pitch.publish(self.pub_imu_pitch_msg)

            self.pub_imu_yaw_msg = Float32()
            self.pub_imu_yaw_msg.data = self.yaw_frame - self.yaw_steer
            self.pub_imu_yaw.publish(self.pub_imu_yaw_msg)


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('TwoImu2Angles')

    try:
        obj = TwoImu2Angles()
    except:
        pass