# Autonomous bicycle

ROS package for localization, pose estimation and autonomous navigation algorithms of autonomous bikes.

Current available features:
- Simulation of bicycle in gazebo and synchronized with RVIZ (model + tf)

A short video:

[![Autonomous bicycle](http://img.youtube.com/vi/t7ZZPeML2Fw/0.jpg)](https://www.youtube.com/watch?v=t7ZZPeML2Fw "Autonomous bicycle")


Install dependencies:

    sudo apt-get install git ros-kinetic-hector-gazebo-plugins 

How to test it?

    cd path_ros_workspace/src/
    git clone https://github.com/francisc0garcia/autonomous_bicycle
    git clone https://github.com/gareth-cross/rviz_satellite
    git clone https://github.com/ccny-ros-pkg/imu_tools
    cd .. 
    catkin_make
    source devel/setup.bash
    roslaunch autonomous_bicycle autonomous_bicycle.launch

Project under heavy development!

- Developed by:

        Francisco J. Garcia R.
        2017