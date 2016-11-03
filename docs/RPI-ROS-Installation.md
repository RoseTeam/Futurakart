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
sudo apt-get install ros-kinetic-libg2o ros-kinetic-tf-conversions ros-kinetic-octomap-ros
sudo apt-get install ros-kinetic-freenect-stack ros-kinetic-depthimage-to-laserscan ros-kinetic-ros-control
sudo apt-get install ros-kinetic-navigation ros-kinetic-robot-localization ros-kinetic-controller-manager ros-kinetic-joint-state-controller ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-plugins ros-kinetic-roslint ros-kinetic-amcl ros-kinetic-gmapping      ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-message-runtime ros-kinetic-topic-tools ros-kinetic-teleop-twist-joy
sudo apt-get install ros-kinetic-ackermann-msgs ros-kinetic-robot-state-publisher
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

### Build `rosserial` package
We recommend you to build and install `rosserial` from sources. It is a critical part of communication between RPi and motor controlling card,
therefore we build it separately of other custom packages.
```
$ mkdir -p ~/rosserial_ws/src; cd $_; catkin_init_workspace
$ git clone https://github.com/ros-drivers/rosserial.git
$ cd ../; catkin_make
```

### Build `futurakart_ws`

To build `futurakart_ws` with dependencies of `generic_ws` and `rosserial_ws` you need to add it in `ROS_PACKAGE_PATH` :
```
source ~/generic_ws/devel/setup.bash
source ~/rosserial_ws/devel/setup.bash
echo $ROS_PACKAGE_PATH
```
Make sure that you see the path to `generic_ws` and `rosserial_ws`. Otherwise, add it manually.
```
$ cd ~/; git clone --recursive https://github.com/RoseTeam/Futurakart.git futurakart_ws 
$ cd ~/futurakart_ws; catkin_make
```

### Configure ROS_MASTER_URI and ROS_HOSTNAME

To export rapidly these variables and change between *local* and *remote* configurations we suggest to use `setup_all.bash` script.
Copy `ros.conf.example` to `ros.conf` and edit `ros.conf` file to define your configuration.
```
cp ros.conf.example ros.conf
nano ros.conf
```
The master machine in our Ros network is RPi, thus : 
- ROS_MASTER_URI should be something like `http://<hostname>:11311` where `<hostname>` can be found running in terminal : `$ hostname`
- ROS_HOSTNAME should be `<hostname>`

Any desktop machine should be configured as (let hostname of RPi master machine be `ubuntu-rpi`) :
- ROS_MASTER_URI should be something like `http://ubuntu-rpi:11311`
- ROS_HOSTNAME should be ip address in the network
 

Setup the configuration to bash:
```
source setup_all.bash
```
If you want temporarily to pass to a local configuration, then do
```
source setup_all.bash --local
```
To comeback to `ros.conf` defined configuration, run the previous command without `--local`. 



  



