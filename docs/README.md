# Documentation





## Futurakart ROS packages  

- Core : `futurakart_control`, `futurakart_description`, `futurakart_msgs`, `futurakart_2dnav` 
- Desktop : `futurakart_viz`
- Robot : `futurakart_bringup`, `futurakart_base` 
- Simulator : `futurakart_gazebo`


#### Core packages

These packages are destinated for the desktop and the robot.  

**`futurakart_description`** package contains information about robot using URDF formalism. 
This defines joints and links, robot caracteristics, dimensions etc and can also associate with meshes for gazebo simulation.

**TODO: define proper robot dimensions and useful parts and sensors** 

See docs on [robot_state_publisher](http://wiki.ros.org/robot_state_publisher/Tutorials/Using%20the%20robot%20state%20publisher%20on%20your%20own%20robot)


**`futurakart_msgs`**


**`futurakart_control`**  


**`futurakart_2dnav`** package contains few launch files for mapping and localization purposes: 

- `create_map.launch gmapping:={true|false} rtabmap:={true|false}` to create a map using `gmapping` or `rtabmap`. 
If you use `gmapping` and want to save the map, run `rosrun map_server map_saver -f mymap` before killing the node or use the alias `save_gmap`.
If you use `rtabmap`, the map is automatically saved as `futurakart/futurakart_2dnav/maps/_maps.db`. 

- `odom_navigation.launch`






#### Robot packages 

These packages are destinated for the robot only. 

Futurakart robot is composed of two Raspberry Pi cards, reponsible for the *mobile* part and the *vision* part
Both RPi cards should have *robot packages* and *core packages*, however the bringup procedure varies depending on the card.

**`futurakart_bringup`** package is responsible to bringup the robot. 

For instance, we use a simple bringup procedure: 
- Connect with SSH to the robot (*mobile* part RPi)
- Run the following on the robot side to start mobile part
```
roslaunch futurakart_bringup futurakart.launch
```
and it calls launch files from 
* `futurakart_base` (base.launch) to initialize the main *futurakart* node
* connect to the *vision* part RPi and run the following `roslaunch futurakart_base vision.launch`. See below for details
* `futurakart_description`
* other drivers


When the *mobile* part RPi connects with SSH the *vision* part PRi, it uses ssh public keys. 
In case of any errors, make sure that the ssh key is added. To check run on the *vision* part RPi
```
less ~/.ssh/authorized_keys
```
and find your user id.

To add new ssh key, connect to the RPi and run the following, replacing "<key-part> and user@PC" by your data from `~/.ssh/id_rsa.pub`   
```
echo "ssh-rsa <key-part> user@PC" >> ~/.ssh/authorized_keys 
```

**TODO: There is another more 'pro' way to bringup a robot. See for example [here](http://wiki.ros.org/husky_bringup/Tutorials/Install%20Husky%20Software)**

**`futurakart_base`** : hardware driver for communicating with the onboard MCU




```
roslaunch futurakart_base vision.launch
```
which initializes the Kinect sensor.


#### Desktop packages

`futurakart_viz` package helps to display the robot using RViz: 
 
In simulation mode:
```
$ roslaunch futurakart_gazebo futurakart_world.launch gui:=false
$ roslaunch futurakart_viz view_robot.launch 
``` 

You can also use available configurations `navigation`, `localization`:
```
$ roslaunch futurakart_viz view_robot.launch config:=<config>
```
 
 

#### Simulator 

The package `futurakart_gazebo` uses some of core packages and allows to run a gazebo simulation:
```
roslaunch futurakart_gazebo futurakart_world.launch
```


## Useful tools 

#### setup_all.bash

A script that sets few aliases :
 
- `fkart` to perform 'cd ~/futurakart_ws/; source devel/setup.bash'
- `save_gmap` to save map produced by gmapping (see `futurakart_2dnav` package)
- `src` to source from current folder : `source devel/setup.bash`

Also the script sets the variables ROS_MASTER_URI and ROS_HOSTNAME defined in `ros.conf`. 
Before the usage copy and rename ros.conf.example to ros.conf and setup your value.    
  
Usage :
```
$ cd /path/to/repo/futurakart_ws/; source setup_all.bash [--local]
```

The option `--local` sets ROS_MASTER_URI and ROS_HOSTNAME as `localhost` to work standalone.



## Installation

### ROS dependencies

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

[Coordinate system definitions](http://www.ros.org/reps/rep-0120.html#coordinate-frames)

- launch/robot_setup_tf.launch 

```
base_link (Center of the kart)
   -> base_footprint (Center of the kart projected on the floor) 
   -> camera_link (Center of the camera)
        -> camera_rgb_optical_frame
        -> camera_depth_optical_frame             
        -> camera_rgb_frame
        -> camera_depth_frame
    
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
