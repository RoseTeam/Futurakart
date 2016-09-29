# ROS packages installation on RPi 3

## Build [ros-kinetic-navigation](https://github.com/ros-planning/navigation.git)

List of packages needed, assuming minimal ros-kinetic-base installed :

```
 sudo apt-get install ros-kinetic-tf ros-kinetic-angles ros-kinetic-bfl ros-kinetic-laser-geometry ros-kinetic-map-msgs ros-kinetic-pcl-msgs ros-kinetic-tf2-eigen
        
 sudo apt-get install libpcl-dev
 sudo apt-get install libsdl-image1.2-dev libyaml-cpp0.3-dev
              
``` 

Dependencies to build :
- [pcl-conversions](https://github.com/ros-perception/pcl_conversions.git) (version *indigo*) 
- [pcl_ros](https://github.com/ros-perception/perception_pcl.git)


### What to do exactly

Create a catkin workspace, clone sources into `src` folder, checkout necessary versions and build. 

```
$ mkdir -p generic_ws/src; catkin_init_workspace

$ git clone https://github.com/ros-planning/navigation.git; git checkout kinetic-devel

$ git clone https://github.com/ros-perception/pcl_conversions.git

$ git clone https://github.com/ros-perception/perception_pcl.git

$ cd ../; catkin_make
 
```
If some strange error complaining about memory appears, try to build using a single thread: `catkin_make -j1`  

When everything is done without errors, install packages into ROS
 
 
 
## Pi camera 