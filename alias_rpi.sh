#!/bin/bash

#aliases
alias base='roslaunch futurakart_base base.launch'
alias cmd_g_max='rostopic pub -r 50.0 /ackermann_cmd ackermann_msgs/AckermannDrive "{prop_vel: 0.5, dir_pos: -0.45}"'
alias cmd_d_max='rostopic pub -r 50.0 /ackermann_cmd ackermann_msgs/AckermannDrive "{prop_vel: 0.5, dir_pos: +0.45}"'
alias cmd_g='rostopic pub -r 50.0 /ackermann_cmd ackermann_msgs/AckermannDrive "{prop_vel: 1.0, dir_pos: -0.20}"'
alias cmd_d='rostopic pub -r 50.0 /ackermann_cmd ackermann_msgs/AckermannDrive "{prop_vel: 1.0, dir_pos: +0.20}"'
alias vision='roslaunch futurakart_base vision.launch'
alias cmap='roslaunch futurakart_2dnav create_map.launch gmapping:=true'
