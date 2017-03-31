#ifndef PROJECT_BICYCLE_INTERACTION_H
#define PROJECT_BICYCLE_INTERACTION_H

//
// Based on gazebo_ros_diff_drive
//

#include <algorithm>
#include <assert.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/math/gzmath.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Time.hh>
#include <sdf/sdf.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <control_msgs/PidState.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <autonomous_bicycle/PID.h>

namespace gazebo {

    class Joint;

    class Entity;

    class BicycleInteraction : public ModelPlugin {

    public:
        BicycleInteraction();
        ~BicycleInteraction();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void Reset();

    protected:
        virtual void UpdateChild();
        virtual void resetWorldEvent();
        virtual void FiniChild();

    private:
        double angular_velocity, wheel_speed_instr_[2];
        unsigned int seq_number, restart_position = 0;
        double x_, rot_;
        bool alive_, initial_pose_saved;

        // Update Rate
        double update_rate_, update_period_;

        std::string topic_velocity;

        math::Pose original_pose;

        GazeboRosPtr gazebo_ros_;
        event::ConnectionPtr update_connection_;
        event::ConnectionPtr create_connection_;

        physics::ModelPtr parent;
        physics::JointController *j2_controller;
        physics::JointPtr joint_front_wheel, joint_rear_wheel, joint_steering;

        boost::thread callback_queue_thread_;
        boost::mutex lock;

        // ROS STUFF
        ros::Subscriber cmd_vel_subscriber_;
        ros::CallbackQueue queue_;
        common::Time last_update_time_;
        common::Time last_odom_update_;

        PID controller_steering_angle;

        void QueueThread();
        void getWheelVelocities();
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_msg);
    };
}

#endif //PROJECT_BICYCLE_INTERACTION_H
