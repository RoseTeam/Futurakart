# Futurakart tutorials

Make sure you have builded `futurakart` catkin workspace and run `source devel/setup.bash` before proceed the tutorial. 
To build `futurakart` package and its dependencies, please see [here](./RPI-ROS-Installation.md). 
If you have any trouble while launching some modules, please revisit the installation part.

To open `futurakart` location and `source` fast, you can use the alias `fkart`. 
See 'setup_all.bash' paragraphe in 'Useful tools' from [here](./README.md) to learn more about.

**Before proceeding make sure that you have flashed the latest firmware on the Nucleo card and has the latest `futurakart` package**

## Check communication between RPi and Nucleo

Connect the *Nucleo* to the *RPi* and you should see a green led blinks under `reset` with about 2 seconds period. 

To exchange messages between motor controlling card *Nucleo* and the RPi, 
you can simply open serial connection between the cards and check the list of topics.
You will need to open 4 terminals. **Do not forget to `source` the workspace in open terminals.** 
 
In the first one start `roscore`.
In the second open serial connection :
```
sudo chmod 666 /dev/ttyACM0
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200 
```
In the third terminal, display motor feedback :
```
rostopic echo /motorfeedback
```
And in the last terminal, send motor commands :
```
rostopic pub -r 5.0 /motordrive_cmd futurakart_msgs/MotorDrive '{prop_vel: 0.2, dir_pos: 0.05}'
```
You should see in the third terminal similar messages :
```
---
dir_pos: 0.0500000007451
prop_pos: 181.745269775
prop_vel: 0.20000000298
---
dir_pos: 0.0500000007451
prop_pos: 181.74647522
prop_vel: 0.20000000298
---
dir_pos: 0.0500000007451
prop_pos: 181.7474823
prop_vel: 0.20000000298

```


## Drive the kart from desktop

TODO
