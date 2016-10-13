#!/usr/bin/env bash


echo "--------------------------------------------------"
echo " Script to install necessary ROS packages on the RPi"
echo " with a clean Ubuntu 16.04"
echo " Usage : bash install_ros_packages.bash [init|fk_deps|fk_deps_aptget] "
echo "--------------------------------------------------"


function init() {
    echo ""
    echo ""
    echo "*******************************************************"
    echo "- Configure and install ROS base packages :"
    echo "*******************************************************"
    echo ""
    echo ""

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
    sudo apt-get update
    sudo apt-get install ros-kinetic-ros-base git
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
}


function fk_deps() {

    echo ""
    echo ""
    echo "*******************************************************"
    echo "- Install Futurakart necessary packages :"
    echo "*******************************************************"
    echo ""
    echo ""

    echo "- libfreenect and ROS freenect_stack"
    sudo apt-get install libyaml-cpp-dev libfreenect
    #sudo apt-get install
    mkdir -p ~/generic_ws/src; cd $_; catkin_init_workspace
    git clone https://github.com/ros-drivers/freenect_stack
    cd ../; catkin_make install -j1

    echo "- rtabmap and rtamap_ros"
    sudo apt-get install ros-kinetic-libg2o ros-kinetic-tf-conversions

    cd ~/; git clone https://github.com/introlab/rtabmap.git rtabmap_source
    mkdir Build_rtabmap; cd $_
    cmake ../rtabmap_source
    sudo make install -j1




}


function fk_deps_aptget() {
    echo ""
    echo ""
    echo "*******************************************************"
    echo "- Install Futurakart dependencies with apt-get"
    echo "*******************************************************"
    echo ""
    echo ""

    sudo apt-get install ros-kinetic-navigation ros-kinetic-robot-localization ros-kinetic-controller-manager ros-kinetic-joint-state-controller ros-kinetic-diff-drive-controller ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-plugins             ros-kinetic-lms1xx ros-kinetic-pointgrey-camera-description ros-kinetic-roslint ros-kinetic-amcl ros-kinetic-gmapping      ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-message-runtime ros-kinetic-topic-tools ros-kinetic-teleop-twist-joy

    sudo apt-get install ros-kinetic-depthimage-to-laserscan

}

if [ -z $1 ]; then
    init
    fk_deps
else
    $1
fi