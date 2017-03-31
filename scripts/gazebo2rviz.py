#!/usr/bin/env python

# based on: gazebo2rviz - Developed by: andreasBihlmaier
# source: https://github.com/andreasBihlmaier/gazebo2rviz

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
import tf_conversions.posemath as pm
from tf.transformations import *

from conversions import link2marker_msg
import pysdf

class Gazebo2Rviz:
    def __init__(self):
        self.rate = rospy.get_param('~rate', 10.0)  # the rate at which to publish the transform
        self.submodelsToBeIgnored = rospy.get_param('~ignore_submodels_of', '').split(';')

        self.tfBroadcaster = tf.TransformBroadcaster()
        self.link_states_msg = None

        self.model_states_msg = None
        self.model_cache = {}
        self.updatePeriod = 1. / self.rate
        self.enable_publisher_marker = False
        self.enable_publisher_tf = False

        self.markerPub = rospy.Publisher('/model_marker', Marker, queue_size=10)
        self.modelStatesSub = rospy.Subscriber('gazebo/model_states', ModelStates, self.on_model_states_msg, queue_size=1)
        self.linkStatesSub = rospy.Subscriber('gazebo/link_states', LinkStates, self.on_link_states_msg, queue_size=1)
        rate_sleep = rospy.Rate(self.rate)

        # Main while loop
        while not rospy.is_shutdown():
            if self.enable_publisher_marker and self.enable_publisher_tf:
                self.publish_tf()
                self.publish_marker()

            rate_sleep.sleep()

    def publish_marker(self):
        for (model_idx, modelinstance_name) in enumerate(self.model_states_msg.name):
            # print(model_idx, modelinstance_name)
            model_name = pysdf.name2modelname(modelinstance_name)
            # print('model_name:', model_name)
            if not model_name in self.model_cache:
                sdf = pysdf.SDF(model=model_name)
                self.model_cache[model_name] = sdf.world.models[0] if len(sdf.world.models) >= 1 else None
                if not self.model_cache[model_name]:
                    print('Unable to load model: %s' % model_name)
            model = self.model_cache[model_name]
            if not model:  # Not an SDF model
                continue
            # print('model:', model)
            model.for_all_links(self.publish_link_marker, model_name=model_name, instance_name=modelinstance_name)

    def publish_link_marker(self, link, full_linkname, **kwargs):
        full_linkinstancename = full_linkname

        if 'model_name' in kwargs and 'instance_name' in kwargs:
            full_linkinstancename = full_linkinstancename.replace(kwargs['model_name'], kwargs['instance_name'], 1)

        marker_msgs = link2marker_msg(link, full_linkinstancename, False, rospy.Duration(10 * self.updatePeriod))
        if len(marker_msgs) > 0:
            for marker_msg in marker_msgs:
                self.markerPub.publish(marker_msg)

    def on_model_states_msg(self, model_states_msg):
        # save model states
        self.model_states_msg = model_states_msg
        self.enable_publisher_marker = True

    def on_link_states_msg(self, link_states_msg):
        self.link_states_msg = link_states_msg
        self.enable_publisher_tf = True

    def publish_tf(self):
        model_cache = {}
        poses = {'gazebo_world': identity_matrix()}
        for (link_idx, link_name) in enumerate(self.link_states_msg.name):
            poses[link_name] = pysdf.pose_msg2homogeneous(self.link_states_msg.pose[link_idx])
            # print('%s:\n%s' % (link_name, poses[link_name]))

        for (link_idx, link_name) in enumerate(self.link_states_msg.name):
            # print(link_idx, link_name)
            modelinstance_name = link_name.split('::')[0]
            # print('modelinstance_name:', modelinstance_name)
            model_name = pysdf.name2modelname(modelinstance_name)
            # print('model_name:', model_name)
            if not model_name in model_cache:
                sdf = pysdf.SDF(model=model_name)
                model_cache[model_name] = sdf.world.models[0] if len(sdf.world.models) >= 1 else None
                if not model_cache[model_name]:
                    print('Unable to load model: %s' % model_name)
            model = model_cache[model_name]
            link_name_in_model = link_name.replace(modelinstance_name + '::', '')
            if model:
                link = model.get_link(link_name_in_model)

                if link.tree_parent_joint:
                    parent_link = link.tree_parent_joint.tree_parent_link
                    parent_link_name = parent_link.get_full_name()
                    # print('parent:', parent_link_name)
                    parentinstance_link_name = parent_link_name.replace(model_name, modelinstance_name, 1)
                else:  # direct child of world
                    parentinstance_link_name = 'gazebo_world'
            else:  # Not an SDF model
                parentinstance_link_name = 'gazebo_world'
            # print('parentinstance:', parentinstance_link_name)
            pose = poses[link_name]
            #parent_pose = pysdf.pose_msg2homogeneous(self.model_states_msg.pose[1])
            parent_pose = poses[parentinstance_link_name]
            rel_tf = concatenate_matrices(inverse_matrix(parent_pose), pose)
            translation, quaternion = pysdf.homogeneous2translation_quaternion(rel_tf)
            # print('Publishing TF %s -> %s: t=%s q=%s' % (pysdf.sdf2tfname(parentinstance_link_name), pysdf.sdf2tfname(link_name), translation, quaternion))
            self.tfBroadcaster.sendTransform(translation, quaternion, rospy.get_rostime(), pysdf.sdf2tfname(link_name),
                                        pysdf.sdf2tfname(parentinstance_link_name))

# Main function.
if __name__ == '__main__':

    # Initialize the node and name it.
    rospy.init_node('Gazebo2Rviz')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        asd = Gazebo2Rviz()
    except rospy.ROSInterruptException:
        pass
