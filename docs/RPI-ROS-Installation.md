# ROS packages installation on RPi 3 with Ubuntu MATE 16.04

Assuming that your SD card has a clean Ubuntu MATE 16.04 distribution, you can use a script `tools/install_rpi_ros_packages.bash` or do it yourself.

### Configure and install ROS base packages :
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-ros-base git
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
```

### Install Futurakart necessary ROS packages using `apt-get` :

```
sudo apt-get install libyaml-cpp-dev libpcl-dev
sudo apt-get install ros-kinetic-camera-info-manager ros-kinetic-diagnostic-updater ros-kinetic-rgbd-launch
sudo apt-get install ros-kinetic-libg2o ros-kinetic-tf-conversions 
sudo apt-get install ros-kinetic-freenect-stack ros-kinetic-depthimage-to-laserscan 
sudo apt-get install ros-kinetic-navigation ros-kinetic-robot-localization ros-kinetic-controller-manager ros-kinetic-joint-state-controller ros-kinetic-diff-drive-controller ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-plugins             ros-kinetic-lms1xx ros-kinetic-pointgrey-camera-description ros-kinetic-roslint ros-kinetic-amcl ros-kinetic-gmapping      ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-message-runtime ros-kinetic-topic-tools ros-kinetic-teleop-twist-joy
```

### Build rtabmap library and rtabmap_ros
```
cd ~/; git clone https://github.com/introlab/rtabmap.git rtabmap_source
mkdir Build_rtabmap; cd $_
cmake ../rtabmap_source
sudo make install -j1
```
It is OK if Qt4 or Qt5 is not found and `RTAB-Map GUI lib is not built, some tools won't be built...`

```
$ mkdir -p ~/generic_ws/src; cd $_; catkin_init_workspace 

$ git clone https://github.com/introlab/rtabmap_ros.git

$ cd ../; catkin_make 
 
```






----- 
# -------------- OLD STUFF ----------------------

You need to install :

- [ros-kinetic-navigation](https://github.com/ros-planning/navigation.git)
- latest `libfreenect` and [ros-kinetic-freenect-stack](https://github.com/ros-drivers/freenect_stack)   
- [rtabmap](https://github.com/introlab/rtabmap.git)
- [rtabmap_ros](https://github.com/introlab/rtabmap_ros.git)


## Install `ros-kinetic-navigation`

### Install dependencies

List of packages needed, assuming minimal ros-kinetic-base installed :

```
 sudo apt-get install ros-kinetic-tf ros-kinetic-angles ros-kinetic-bfl ros-kinetic-laser-geometry ros-kinetic-map-msgs ros-kinetic-pcl-msgs ros-kinetic-tf2-eigen
        
 sudo apt-get install libpcl-dev
 sudo apt-get install libsdl-image1.2-dev libyaml-cpp0.3-dev libopenni2
              
``` 

Dependencies to build :
- [pcl-conversions](https://github.com/ros-perception/pcl_conversions.git) (version *indigo*) 
- [pcl_ros](https://github.com/ros-perception/perception_pcl.git)

### Install package

This package is very big and can take a lot of time to build. Consider to build it using `chroot` technique on your desktop PC instead of the RPi.

```
$ mkdir -p ~/ros-kinetic-navigation-stack_ws/src; cd $_; catkin_init_workspace

$ git clone https://github.com/ros-planning/navigation.git; git checkout kinetic-devel

$ git clone https://github.com/ros-perception/pcl_conversions.git

$ git clone https://github.com/ros-perception/perception_pcl.git

$ cd ../; catkin_make 
 
```

If some strange error complaining about memory appears, try to build using a single thread: `catkin_make -j1`. When everything is done without errors, install packages into ROS


## Install `libfreenect` and `ros-kinetic-freenect-stack`

Instead of `apt-get` installation of the package `ros-kinetic-freenect-stack` which installs automatically `libfreenect` version 0.5.1, 
we install the latest `libfreenect` (0.5.3) and build `ros-kinetic-freenect-stack` manually.

### Install dependencies

List of packages needed, assuming minimal ros-kinetic-base installed and the previous dependencies :

```
sudo apt-get install libyaml-cpp-dev libfreenect 
```

**`libyaml-cpp-dev` can cause problems to ros-kinetic-navigation-stack depending on `libyaml-cpp0.3-dev`**

### Install package

```
$ mkdir -p ~/generic_ws/src; cd $_; catkin_init_workspace

$ git clone https://github.com/ros-drivers/freenect_stack

$ cd ../; catkin_make
```


## Install rtabmap_ros

We need to install latest versions of the library `rtabmap` and `rtabmap_ros`

### Install dependencies 

```
sudo apt-get install ros-kinetic-libg2o ros-kinetic-tf-conversions
```

Build `rtabmap`
```
$ cd ~/

$ git clone https://github.com/introlab/rtabmap.git rtabmap_source

$ mkdir Build_rtabmap; cd $_

$ cmake ../rtabmap_source

$ sudo make install -j1
```
It is OK if Qt4 or Qt5 is not found and `RTAB-Map GUI lib is not built, some tools won't be built...`

### Install package

```
$ cd ~/generic_ws/src; git clone https://github.com/introlab/rtabmap_ros.git

$ cd ../; catkin_make 
 
```


## Before building `futurakart_ws`

**To build `futurakart_ws` with dependencies of `generic_ws` you need to `source` this ws:**
```
$ cd ~/generic_ws
$ source devel/setup.bash
$ cd ~/futurakart_ws; catkin_make
```


