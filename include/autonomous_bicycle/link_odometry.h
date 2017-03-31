/*
* Based on ROS Diff Drive
 **/

#ifndef SUBDRIVE_PLUGIN_HH
#define SUBDRIVE_PLUGIN_HH

#include <map>
#include <math.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include "mavros_msgs/RCOut.h"

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

    class LinkOdometry : public ModelPlugin {

    public:
        LinkOdometry();
        ~LinkOdometry();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void Reset();

    protected:
        virtual void UpdateChild();
        virtual void resetWorldEvent();
        virtual void FiniChild();

    private:
        // Update Rate
        double update_rate_, update_period_, temp_x, temp_y, temp_z, prev_temp_x, prev_temp_y, prev_temp_z;
        double temp_vel_x, temp_vel_y, temp_vel_z;
        bool alive_;
        std::string odometry_topic_, frame_name_;

        GazeboRosPtr gazebo_ros_;
        physics::ModelPtr parent;
        event::ConnectionPtr update_connection_, create_connection_;
        physics::JointPtr joint_motor_left, joint_motor_right, joint_motor_center;

        // ROS STUFF
        ros::Publisher odometry_publisher_;
        ros::CallbackQueue queue_;
        nav_msgs::Odometry odom_;
        nav_msgs::Odometry pose_sub;
        geometry_msgs::Pose2D pose_encoder_;

        // Custom Callback Queue
        boost::mutex lock;
        boost::thread callback_queue_thread_;
        common::Time last_update_time_;
        common::Time last_odom_update_;

        void QueueThread();
    };
}

#endif

