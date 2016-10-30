# ...

## Installations:

nav-core
navfn
global-planner

 

## Useful links and info

### [Navigation stack](http://wiki.ros.org/navigation?distro=kinetic)

### [TEB local planner](http://wiki.ros.org/teb_local_planner)

[How Obstacle Avoidance works](http://wiki.ros.org/teb_local_planner/Tutorials/Obstacle%20Avoidance%20and%20Robot%20Footprint%20Model)



### [`staticmap` layer](http://wiki.ros.org/costmap_2d/hydro/staticmap)

### [`obstacles` layer](http://wiki.ros.org/costmap_2d/hydro/obstacles)

`~<name>/<source_name>/obstacle_range` (double, default: 2.5) The maximum range in meters at which to insert obstacles into the costmap using sensor data. 

`~<name>/<source_name>/raytrace_range` (double, default: 3.0) The maximum range in meters at which to raytrace out obstacles from the map using sensor data.
 
`~<name>/<source_name>/data_type` (string, default: "PointCloud") The data type associated with the topic, right now only "PointCloud", "PointCloud2", and "LaserScan" are supported. 
 

### [Costmap_2d](http://wiki.ros.org/costmap_2d)

`<name>` can be `global_costmap` or `local_costmap` 

#### Subscribed Topics

`~<name>/footprint (geometry_msgs/Polygon)` Specification for the footprint of the robot. This replaces the previous parameter specification of the footprint. 

#### Published Topics

`~<name>/grid (nav_msgs/OccupancyGrid)` The values in the costmap 

`~<name>/grid_updates (map_msgs/OccupancyGridUpdate)` The value of the updated area of the costmap 

`~<name>/voxel_grid (costmap_2d/VoxelGrid)` Optionally advertised when the underlying occupancy grid uses voxels and the user requests the voxel grid be published. 

#### Parameters

The costmap_2d::Costmap2DROS is highly configurable with several categories of ROS Parameters: coordinate frame and tf, rate, robot description and map management.

Coordinate frame and tf parameters

`~<name>/global_frame` (string, default: "/map") The global frame for the costmap to operate in. 

`~<name>/robot_base_frame` (string, default: "base_link") The name of the frame for the base link of the robot. 

`~<name>/transform_tolerance` (double, default: 0.2) Specifies the delay in transform (tf) data that is tolerable in seconds. This parameter serves as a safeguard to losing a link in the tf tree while still allowing an amount of latency the user is comfortable with to exist in the system. For example, a transform being 0.2 seconds out-of-date may be tolerable, but a transform being 8 seconds out of date is not. If the tf transform between the coordinate frames specified by the global_frame and robot_base_frame parameters is transform_tolerance seconds older than ros::Time::now(), then the navigation stack will stop the robot. 

#### Rate parameters

`~<name>/update_frequency` (double, default: 5.0) The frequency in Hz for the map to be updated. 

`~<name>/publish_frequency` (double, default: 0.0) The frequency in Hz for the map to be publish display information. 

#### Map management parameters

`~<name>/rolling_window` (bool, default: false) Whether or not to use a rolling window version of the costmap. If the static_map parameter is set to true, this parameter must be set to false. 

`~<name>/always_send_full_costmap` (bool, default: false) If true the full costmap is published to "~<name>/grid" every update. If false only the part of the costmap that has changed is published on the "~<name>/grid_updates" topic. 

#### The following parameters can be overwritten by some layers, namely the static map layer.

`~<name>/width (int, default: 10)`
    The width of the map in meters. 
`~<name>/height (int, default: 10)`
    The height of the map in meters. 
`~<name>/resolution (double, default: 0.05)`
    The resolution of the map in meters/cell. 
`~<name>/origin_x (double, default: 0.0)`
    The x origin of the map in the global frame in meters. 
`~<name>/origin_y (double, default: 0.0)`
    The y origin of the map in the global frame in meters. 

#### Required tf Transforms
(value of global_frame parameter) → (value of robot_base_frame parameter)
Usually provided by a node responsible for odometry or localization such as amcl. 
