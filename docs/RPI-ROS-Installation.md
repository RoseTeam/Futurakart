# ROS packages installation on RPi 3



### Install dependencies for [ros-kinetic-navigation](https://github.com/ros-planning/navigation.git)

List of packages needed, assuming minimal ros-kinetic-base installed :

```
 sudo apt-get install ros-kinetic-tf ros-kinetic-angles ros-kinetic-bfl ros-kinetic-laser-geometry ros-kinetic-map-msgs ros-kinetic-pcl-msgs ros-kinetic-tf2-eigen
        
 sudo apt-get install libpcl-dev
 sudo apt-get install libsdl-image1.2-dev libyaml-cpp0.3-dev libopenni2
              
``` 

Dependencies to build :
- [pcl-conversions](https://github.com/ros-perception/pcl_conversions.git) (version *indigo*) 
- [pcl_ros](https://github.com/ros-perception/perception_pcl.git)


### Install dependencies for [ros-kinetic-freenect-stack](https://github.com/ros-drivers/freenect_stack)  

List of packages needed, assuming minimal ros-kinetic-base installed and the previous dependencies :

```
sudo apt-get install libyaml-cpp-dev 
```

**`libyaml-cpp-dev` can cause problems to ros-kinetic-navigation-stack depending on `libyaml-cpp0.3-dev`**

### Install dependencies for [RTabmap_ros](https://github.com/introlab/rtabmap_ros) 



## What to do exactly

Create a catkin workspace, clone sources into `src` folder, checkout necessary versions and build. 

```
$ mkdir -p generic_ws/src; cd src; catkin_init_workspace

$ git clone https://github.com/ros-planning/navigation.git; git checkout kinetic-devel

$ git clone https://github.com/ros-perception/pcl_conversions.git

$ git clone https://github.com/ros-perception/perception_pcl.git

$ git clone https://github.com/introlab/rtabmap.git

$ git clone https://github.com/introlab/rtabmap_ros.git; git checkout kinetic-devel

$ cd ../; catkin_make 
 
```

If some strange error complaining about memory appears, try to build using a single thread: `catkin_make -j1`. When everything is done without errors, install packages into ROS


**To build `futurakart_ws` with dependencies of `generic_ws` you need to `source` this ws:**
```
$ cd ~/generic_ws
$ source devel/setup.bash
$ cd ~/futurakart_ws; catkin_make
```



### DELETED AFTER THAT

~~```
~~# in generic_ws/
~~user@~/generic_ws$ sudo su

~~root@~/generic_ws# source devel/setup.bash; exit

~~user@~/generic_ws$ catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic
~~``` 
 
~~## Pi camera~~ 
~~To enable Picamera, install `sudo apt-get install raspi-config`, if needed and call `raspi-config`.~~ 




