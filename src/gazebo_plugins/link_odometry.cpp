//
// Based on gazebo_ros_diff_drive
//
#include <autonomous_bicycle/link_odometry.h>

namespace gazebo {
    LinkOdometry::LinkOdometry() {}
    LinkOdometry::~LinkOdometry() {}

    // Load the controller
    void LinkOdometry::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        this->parent = _parent;

        gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "LinkOdometry"));

        // Make sure the ROS node for Gazebo has already been initialized
        gazebo_ros_->isInitialized();

        gazebo_ros_->getParameter<std::string>(odometry_topic_, "odometry_topic", "odom");
        gazebo_ros_->getParameter<std::string>(frame_name_, "frame_name", "frame_name");
        gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 100.0);

        // creates a publisher for sending odometry data
        odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

        // Initialize update rate stuff
        if (this->update_rate_ > 0.0) this->update_period_ = 1.0 / this->update_rate_;
        else this->update_period_ = 0.0;
        last_update_time_ = parent->GetWorld()->GetSimTime();

        alive_ = true;

        // start custom queue for link odometry
        this->callback_queue_thread_ =
                boost::thread(boost::bind(&LinkOdometry::QueueThread, this));

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ =
                event::Events::ConnectWorldUpdateBegin(boost::bind(&LinkOdometry::UpdateChild, this));

        this->create_connection_ =
                event::Events::ConnectWorldReset(boost::bind(&LinkOdometry::resetWorldEvent, this));

    }

    void LinkOdometry::Reset() {
        last_update_time_ = parent->GetWorld()->GetSimTime();
    }

    void LinkOdometry::UpdateChild() {
        common::Time current_time = parent->GetWorld()->GetSimTime();
        double seconds_since_last_update = (current_time - last_update_time_).Double();

        if (seconds_since_last_update > update_period_) {
            temp_x = this->parent->GetWorldPose().pos.x;
            temp_y = this->parent->GetWorldPose().pos.y;
            temp_z = this->parent->GetWorldPose().pos.z;

            // Compute odometry
            odom_.pose.pose.position.x = temp_x;
            odom_.pose.pose.position.y = temp_y;
            odom_.pose.pose.position.z = temp_y;

            odom_.pose.pose.orientation.x = this->parent->GetWorldPose().rot.x;
            odom_.pose.pose.orientation.y = this->parent->GetWorldPose().rot.y;
            odom_.pose.pose.orientation.z = this->parent->GetWorldPose().rot.z;
            odom_.pose.pose.orientation.w = this->parent->GetWorldPose().rot.w;

            temp_vel_x = (temp_x - prev_temp_x) / seconds_since_last_update;
            temp_vel_y = (temp_y - prev_temp_y) / seconds_since_last_update;

            odom_.twist.twist.linear.x = sqrt( pow(temp_vel_x, 2.0) + pow(temp_vel_y, 2.0) ) ;
            odom_.twist.twist.linear.y = 0;
            odom_.twist.twist.linear.z = 0;

            // set header
            odom_.header.stamp = ros::Time::now();
            odom_.header.frame_id = frame_name_;
            odom_.child_frame_id = frame_name_;

            odometry_publisher_.publish(odom_);

            last_update_time_ += common::Time(update_period_);
            prev_temp_x = temp_x;
            prev_temp_y = temp_y;
            prev_temp_z = temp_z;
        }
    }

    void LinkOdometry::resetWorldEvent() {
        ROS_INFO("LinkOdometry::resetWorldEvent()");
    }

    // Finalize the controller
    void LinkOdometry::FiniChild() {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        gazebo_ros_->node()->shutdown();
        callback_queue_thread_.join();
    }

    void LinkOdometry::QueueThread() {
        static const double timeout = 0.01;

        while (alive_ && gazebo_ros_->node()->ok()) {
            queue_.callAvailable(ros::WallDuration(timeout));
        }

        this->Reset();
        this->FiniChild();
    }

    GZ_REGISTER_MODEL_PLUGIN (LinkOdometry)
}

