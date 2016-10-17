#!/usr/bin/env bash


echo "--------------------------------------------------"
echo " Script to install necessary ROS packages on the RPi"
echo " with a clean Ubuntu 16.04"
echo " Usage : bash install_ros_packages.bash [init|fk_deps|rtabmap|rtabmap_ros] "
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
    sudo apt-get -y install ros-kinetic-ros-base git
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
}


function fk_deps() {

    echo ""
    echo ""
    echo "*******************************************************"
    echo "- Install Futurakart necessary ROS packages : "
    echo "*******************************************************"
    echo ""
    echo ""

    sudo apt-get -y install libyaml-cpp-dev libpcl-dev
    sudo apt-get -y install ros-kinetic-camera-info-manager ros-kinetic-diagnostic-updater ros-kinetic-rgbd-launch ros-kinetic-freenect-stack
    sudo apt-get -y install ros-kinetic-navigation ros-kinetic-robot-localization ros-kinetic-roslint ros-kinetic-gmapping ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-message-runtime ros-kinetic-teleop-twist-joy ros-kinetic-ros-control
    sudo apt-get -y install ros-kinetic-depthimage-to-laserscan
    sudo apt-get -y install ros-kinetic-libg2o ros-kinetic-tf-conversions ros-kinetic-octomap-ros

}

function rtabmap() {
    echo ""
    echo ""
    echo "*******************************************************"
    echo "- Build and install Rtabmap and rtabmap_ros from source"
    echo "*******************************************************"
    echo ""
    echo ""

    echo ""
    echo "- Build rtabmap "
    echo ""

    cd ~/
    if [ ! -d "rtabmap_source" ]; then
        git clone https://github.com/introlab/rtabmap.git rtabmap_source
    fi
    if [ -d "Build_rtabmap" ]; then
        sudo rm -R Build_rtabmap
    fi
    mkdir Build_rtabmap; cd $_
    cmake ../rtabmap_source
    sudo make install -j1


}

function rtabmap_ros() {
    echo ""
    echo ""
    echo "*******************************************************"
    echo "- Build and install Rtabmap and rtabmap_ros from source"
    echo "*******************************************************"
    echo ""
    echo ""

    echo ""
    echo "- Build rtamap_ros"
    echo ""
    cd ~/
    if [ -d "generic_ws" ]; then
        rm -R generic_ws/build generic_ws/devel generic_ws/install
        cd generic_ws/src
    else
        mkdir -p generic_ws/src; cd $_; catkin_init_workspace
    fi
    if [ ! -d "rtabmap_ros" ]; then
        git clone https://github.com/introlab/rtabmap_ros.git
    fi
    cd ../; catkin_make install -j1
}

if [ -z $1 ]; then
    init
    fk_deps
    rtabmap
    rtabmap_ros
else
    $1
fi