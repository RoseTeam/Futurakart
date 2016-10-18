# Futurakart tutorials

## Base part emulation on the desktop

You can test `futurakart` base part on the PC without any cards required.  
 
Start mbed card emulation : 
```
roslaunch futurakart_mbed_emu start.launch
```
which emulates an ideal motor, received commands are directly applied.
 
Next, start `futurakart_base` base node:
```
rosrun futurakart_base futurakart_node
```
You can see that emulator start to print that it receives a zero `cmd_vel` messages.
These messages are created from current state of the joints : direction and propulsion.

Now we start `futurakart_description` and `futurakart_control` :
```
roslaunch futurakart_description description.launch
```
and 
```
roslaunch futurakart_control control.launch
```

Now we are ready to publish a driving command using Ackermann messages :  
```
rostopic pub -r 50.0 /ackermann_cmd ackermann_msgs/AckermannDrive "{'steering_angle': 0.05, 'speed': 0.1}"
``` 

Open rviz and observe the kart moving:
```
roslaunch futurakart_viz view_robot.launch
```

or going to the next step : publish `Twist` message, convert it 


