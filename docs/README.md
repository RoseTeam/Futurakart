# Documents 

## ROS dependencies

Install :

- ROS indigo or kinetic

- ROS Navigation stack:
    * move_base : `sudo apt-get install ros-$ROS_DISTRO-move-base`
        - **! IS MISING IN ARMHF !**
        - http://answers.ros.org/question/235521/move_base-for-kinetic-armhf/
    * [RGBD to laser scan](http://wiki.ros.org/depthimage_to_laserscan) : `sudo apt-get install ros-$ROS_DISTRO-depthimage-to-laserscan` 
    * [RTABMap_ros](http://wiki.ros.org/rtabmap_ros/) : `sudo apt-get install ros-$ROS_DISTRO-rtabmap-ros`
    ~~* Map server : `sudo apt-get install ros-$ROS_DISTRO-map-server`~~
    ~~* SLAM gmapping : `sudo apt-get install ros-$ROS_DISTRO-gmapping`~~
- [Freenect stack](http://wiki.ros.org/freenect_stack) : `sudo apt-get install ros-$ROS_DISTRO-freenect-stack`
- [freenect_launch]
- etc


## Robot Setup 

See ROS documentation [here](http://wiki.ros.org/navigation/Tutorials/RobotSetup)  

### Transform Configuration

- launch/robot_setup_tf.launch 

```
base_link 
    -> base_kinect
        -> kinect_rgb_frame
        -> kinect_depth_frame
    -> base_us_1
    -> base_us_2
    ...
    -> base_us_6
    -> base_ir_1
    -> base_ir_2
```        

### Sensor Information (sensor sources)

The navigation stack uses information from sensors to avoid obstacles in the world, it assumes that these sensors are publishing either sensor_msgs/LaserScan or sensor_msgs/PointCloud messages over ROS.

In `moc_nodes` package we have moc_kinect_node to simulate kinect output

### Odometry Information (odometry source)

The navigation stack requires that odometry information be published using tf and the nav_msgs/Odometry message.

In `moc_nodes` package we have moc_odom_node to simulate odometry output


### Base Controller (base controller)

The navigation stack assumes that it can send velocity commands using a geometry_msgs/Twist message assumed to be in the base coordinate frame of the robot on the "cmd_vel" topic. This means there must be a node subscribing to the "cmd_vel" topic that is capable of taking (vx, vy, vtheta) <==> (cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z) velocities and converting them into motor commands to send to a mobile base.

see an [example](http://wiki.ros.org/pr2_mechanism_controllers)


## Troubleshooting

- If you want to start a python script from a file .launch as `<node name="myname" pkg="mypkg" type"myscript.py"/>`, do not forget `chmod +x myscript.py`  
