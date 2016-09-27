
## Mapping and localization with [rtabmap_ros](http://wiki.ros.org/rtabmap_ros)


### Configuration A : Kinect

<img src="http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot?action=AttachFile&do=get&target=setupD.png"/>

* Kinect sensor produces messages for `rtabmap`, `depthimage_to_laserscan` and `visual_odometry`









### Configuration B : Kinect + PiCamera

<img src="http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot?action=AttachFile&do=get&target=setupA2.jpg"/>

* Kinect sensor produces messages for `rtabmap` and `depthimage_to_laserscan` 
* PiCamera sensor produces images for `visual_odometry` with [viso2_ros](http://wiki.ros.org/viso2_ros)


