//
// Based on gazebo_ros_diff_drive
//
#include <autonomous_bicycle/bicycle_interaction.h>

namespace gazebo {
    BicycleInteraction::BicycleInteraction() {}
    BicycleInteraction::~BicycleInteraction() {}

    // Load the controller
    void BicycleInteraction::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        this->parent = _parent;

        gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "BicycleInteraction"));

        // Make sure the ROS node for Gazebo has already been initialized
        gazebo_ros_->isInitialized();

        gazebo_ros_->getParameter<std::string>(topic_velocity, "topic_velocity", "cmd_vel");
        gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 100.0);

        controller_steering_angle = PID();
        controller_steering_angle.setSampleTime(0.0001);
        controller_steering_angle.setKp(10);
        controller_steering_angle.setKd(0);
        controller_steering_angle.setKi(0);
        controller_steering_angle.setWindUp(0);

        angular_velocity = 0;

        joint_front_wheel = gazebo_ros_->getJoint(parent, "joint_front_wheel", "_joint_front_wheel");
        joint_rear_wheel = gazebo_ros_->getJoint(parent, "joint_rear_wheel", "_joint_rear_wheel");
        joint_steering = gazebo_ros_->getJoint(parent, "joint_steering", "_joint_steering");

        int limit_torque = 5;
        joint_front_wheel->SetParam("fmax", 0, limit_torque);
        joint_rear_wheel->SetParam("fmax", 0, limit_torque);
        joint_steering->SetParam("fmax", 0, limit_torque);

        if (!joint_front_wheel) {
            ROS_ERROR("Not found: (joint_wheel)");
            return;
        }

        if (!joint_rear_wheel) {
            ROS_ERROR("Not found: (joint_wheel_90)");
            return;
        }

        if (!joint_steering) {
            ROS_ERROR("Not found: (joint_wheel_90)");
            return;
        }

        j2_controller = new physics::JointController(this->parent);

        // Initialize update rate stuff
        if (this->update_rate_ > 0.0) this->update_period_ = 1.0 / this->update_rate_;
        else this->update_period_ = 0.0;
        last_update_time_ = parent->GetWorld()->GetSimTime();

        x_ = 0;
        rot_ = 0;

        restart_position = 0;

        alive_ = true;

        // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
        ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), topic_velocity.c_str());

        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<geometry_msgs::Twist>(topic_velocity, 1,
                                                                    boost::bind(&BicycleInteraction::cmdVelCallback,
                                                                                this, _1),
                                                                    ros::VoidPtr(), &queue_);
        cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
        ROS_INFO("%s: Subscribe to %s!", gazebo_ros_->info(), topic_velocity.c_str());

        // start custom queue for bicycle interaction
        this->callback_queue_thread_ =
                boost::thread(boost::bind(&BicycleInteraction::QueueThread, this));

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ =
                event::Events::ConnectWorldUpdateBegin(boost::bind(&BicycleInteraction::UpdateChild, this));

        this->create_connection_ =
                event::Events::ConnectWorldReset(boost::bind(&BicycleInteraction::resetWorldEvent, this));

    }

    void BicycleInteraction::Reset() {
        last_update_time_ = parent->GetWorld()->GetSimTime();
        x_ = 0;
        rot_ = 0;
    }

    void BicycleInteraction::UpdateChild() {
        common::Time current_time = parent->GetWorld()->GetSimTime();
        double seconds_since_last_update = (current_time - last_update_time_).Double();

        if(!initial_pose_saved){
            this->original_pose = this->parent->GetWorldPose();
            initial_pose_saved = true;
        }

        if (seconds_since_last_update > update_period_) {
            getWheelVelocities();

            // if velocity < 0 reset model pose
            if(angular_velocity < 0){
                joint_front_wheel->SetVelocity(0, 0);
                joint_rear_wheel->SetVelocity(0, 0);
                joint_steering->SetVelocity(0, 0);

                joint_front_wheel->SetPosition(0, 0);
                joint_rear_wheel->SetPosition(0, 0);
                joint_steering->SetPosition(0, 0);

                //this->original_pose.rot.y = 0.5;
                this->parent->SetWorldPose(this->original_pose);
            }else{
                //joint_base->SetForce(0, control_output);
                // Temporal: unable controller
                controller_steering_angle.setKp(0);
                controller_steering_angle.setKd(0);
                controller_steering_angle.setKi(0);
                controller_steering_angle.setWindUp(0);
                controller_steering_angle.clear();

                // only rear traction
                //joint_front_wheel->SetVelocity(0, -angular_velocity);
                joint_rear_wheel->SetVelocity(0, -angular_velocity);
                joint_steering->SetVelocity(0, 0);

                math::Angle angle_steering = joint_steering->GetAngle(0);
                controller_steering_angle.setPoint = 0;
                controller_steering_angle.update(angle_steering.Degree());
                joint_steering->SetForce(0, controller_steering_angle.output);
            }

            last_update_time_ += common::Time(update_period_);
        }
    }

    void BicycleInteraction::resetWorldEvent() {
        ROS_INFO("BicycleInteraction::resetWorldEvent()");
    }

    // Finalize the controller
    void BicycleInteraction::FiniChild() {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        gazebo_ros_->node()->shutdown();
        callback_queue_thread_.join();
    }

    void BicycleInteraction::getWheelVelocities() {
        boost::mutex::scoped_lock scoped_lock(lock);
        angular_velocity = x_;
    }

    void BicycleInteraction::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_msg) {
        boost::mutex::scoped_lock scoped_lock(lock);
        x_ = cmd_msg->linear.x;
        rot_ = cmd_msg->angular.z;
    }

    void BicycleInteraction::QueueThread() {
        static const double timeout = 0.01;

        while (alive_ && gazebo_ros_->node()->ok()) {
            queue_.callAvailable(ros::WallDuration(timeout));
        }

        this->Reset();
        this->FiniChild();
    }

    GZ_REGISTER_MODEL_PLUGIN (BicycleInteraction)
}

