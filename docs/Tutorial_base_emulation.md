# Futurakart tutorials

## Base part emulation on the desktop

You can test `futurakart` base part on the PC without any cards required.  
 
Start mbed card emulation : 
```
roslaunch futurakart_mbed_emu start.launch
```
which emulates an ideal motor, that receives commands and directly applies it to its state.
There is no encoder emulation.

 
Next, start `futurakart_base` base node:
```
rosrun futurakart_base futurakart_node
```
You can see that emulator start to print that it receives a zero `motordrive_cmd` messages.
These messages are created from current state of the joints : direction and propulsion.


Now we need to start `futurakart_description` and `futurakart_control` :
```
roslaunch futurakart_description description.launch
```
and 
```
roslaunch futurakart_control control.launch
```
The last launch file start also the node `twist_to_ackermann` which converts twist messages to ackermann messages.

Now, you have to choices to drive with interactive markers or manually using terminal

### Driving with interactive markers
You need to have `interactive_markers_twist_server` package installed. See below the installation part.
Most probably, you need to `source`  from the path of `interactive_markers_twist_server` package. Once everything is OK, you can type:
```
roslaunch futurakart_control teleop.launch
```
and open rviz :
```
roslaunch futurakart_viz view_robot.launch
```

To command the kart you can pull the blue circle around it and it will drive.
Today, ackermann interactive markers controls is a hack of the original package [`interactive_markers_twist_server`](https://github.com/ros-visualization/interactive_marker_twist_server.git)
so it does not work perfectly.


### Drive it manually like a real man

And finally, now we are ready to publish a driving command using Ackermann messages :
```
rostopic pub -r 50.0 /ackermann_cmd ackermann_msgs/AckermannDrive "{'steering_angle': 0.05, 'speed': 0.1}"
``` 

Open rviz and observe the kart moving:
```
roslaunch futurakart_viz view_robot.launch
```


## Install `interactive_markers_twist_server`
Clone the sources in a generic workspace, for example, like that:
```
$ mkdir -p ~/generic_ws/src; cd $_; catkin_init_workspace
$ git clone https://github.com/vfdev-5/interactive_marker_twist_server.git
$ cd ../; catkin_make
```
