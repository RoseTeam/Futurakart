# Documents 

## ROS dependencies

Install :

- ROS Navigation stack:
    - move_base : `sudo apt-get install ros-indigo-move-base`
    - 
    
- [freenect_launch](http://wiki.ros.org/freenect_launch)
-


## Robot Setup 

See [here](http://wiki.ros.org/navigation/Tutorials/RobotSetup)  

### Transform Configuration

- launch/robot_setup_tf.launch 

```
base_link 
    -> base_kinect
    -> base_us_1
    -> base_us_2
    ...
    -> base_us_6
    -> base_ir_1
    -> base_ir_2
```        

### Sensor Information (sensor sources)

The navigation stack uses information from sensors to avoid obstacles in the world, it assumes that these sensors are publishing either sensor_msgs/LaserScan or sensor_msgs/PointCloud messages over ROS.

### Odometry Information (odometry source)

The navigation stack requires that odometry information be published using tf and the nav_msgs/Odometry message.

### Base Controller (base controller)

The navigation stack assumes that it can send velocity commands using a geometry_msgs/Twist message assumed to be in the base coordinate frame of the robot on the "cmd_vel" topic. This means there must be a node subscribing to the "cmd_vel" topic that is capable of taking (vx, vy, vtheta) <==> (cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z) velocities and converting them into motor commands to send to a mobile base.

see an [example](http://wiki.ros.org/pr2_mechanism_controllers)



