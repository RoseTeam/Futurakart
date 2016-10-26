# Futurakart tutorials

Make sure you have builded `futurakart` catkin workspace and run `source devel/setup.bash` before proceed the tutorial. 
To build `futurakart` package and its dependencies, please see [here](./RPI-ROS-Installation.md). 
If you have any trouble while launching some modules, please revisit the installation part.

To open `futurakart` location and `source` fast, you can use the alias `fkart`. 
See 'setup_all.bash' paragraph in 'Useful tools' from [here](./README.md) to learn more about.

**Before proceeding make sure that you have flashed the latest firmware on the Nucleo card and has the latest `futurakart` package**

## Check communication between RPi and Nucleo

Connect the *Nucleo* to the *RPi* and you should see a green led blinks under `reset` button with about 2 seconds period. 

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
Command values are in *rad/s* for `prop_vel` (propulsion velocity) and *rad* for `dir_pos` (direction position).

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

Connect the *Nucleo* to the *RPi* and you should see a green led blinks under `reset` button with about 2 seconds period. 
Ping *RPi* from your desktop and connect to it using SSH.


At first, we need to configure properly ROS_MASTER_URI and ROS_HOSTNAME. Configure `ros.conf` and use `setup_all.bash`. 
For more information about this, consult 'setup_all.bash' paragraphe in 'Useful tools' from [here](./README.md).
```
nano ros.conf
source setup_all.bash
```

We will start the base node of `futurakart` on the RPi :
```
roslaunch futurakart_base base.launch
```
You should see the last messages like these :
```
[INFO] [WallTime: 1476914401.109845] ROS Serial Python Node
[INFO] [WallTime: 1476914401.131534] Connecting to /dev/ttyACM0 at 115200 baud
[INFO] [WallTime: 1476914401.467880] Controller Spawner: Waiting for service controller_manager/load_controller
[INFO] [WallTime: 1476914401.475105] Controller Spawner: Waiting for service controller_manager/switch_controller
[INFO] [WallTime: 1476914401.484267] Controller Spawner: Waiting for service controller_manager/unload_controller
[INFO] [WallTime: 1476914401.492238] Loading controller: futurakart_joint_publisher
[INFO] [WallTime: 1476914401.670108] Loading controller: futurakart_velocity_controller
[INFO] [WallTime: 1476914401.909790] Controller Spawner: Loaded controllers: futurakart_joint_publisher, futurakart_velocity_controller
[INFO] [WallTime: 1476914401.968209] Started controllers: futurakart_joint_publisher, futurakart_velocity_controller
[INFO] [WallTime: 1476914403.279211] Note: publish buffer size is 512 bytes
[INFO] [WallTime: 1476914403.282587] Setup publisher on motorfeedback [futurakart_msgs/MotorFeedback]
[INFO] [WallTime: 1476914403.308983] Note: subscribe buffer size is 512 bytes
[INFO] [WallTime: 1476914403.311611] Setup subscriber on motordrive_cmd [futurakart_msgs/MotorDrive]
```

Optionally, you can check topics `rostopic list`

Next on the desktop, we can start rviz :
```
roslaunch futurakart_viz view_robot.launch
```

