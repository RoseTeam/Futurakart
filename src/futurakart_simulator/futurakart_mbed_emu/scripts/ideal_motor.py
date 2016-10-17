#!/usr/bin/python2


#
# Script to emulate an ideally responsive motor
#
# Our nucleo card controls direction / propulsion motors
#
# It subcribes to a geometry_msgs/Pose2D topic named 'cmd_vel'
# It publishes a geometry_msgs/Pose2D message as a topic named 'state_pos_vel'
#

import signal
import rospy
from geometry_msgs.msg import Pose2D


running_ = False

pos_ = 0.0 # Direction angle
vel_ = 0.0 # Propulsion velocity
cmd_pos_ = 0.0 # Direction command
cmd_vel_ = 0.0 # Velocity command


def cmd_vel_cb(cmd_vel_msg):
    global cmd_vel_, cmd_pos_
    rospy.loginfo("cmd_vel_cb : {}".format(cmd_vel_msg))
    cmd_vel_ = cmd_vel_msg.x
    cmd_pos_ = cmd_vel_msg.theta



def stop_node_handler(*args, **kwargs):
    global running_
    running_ = False
    rospy.signal_shutdown("Shutdown...")


def create_state_pos_vel_msg():
    msg = Pose2D()
    msg.x = vel_
    msg.theta = pos_
    return msg


def recompute():
    global pos_, vel_
    # ideal responsivity
    pos_ = cmd_pos_
    vel_ = cmd_vel_

if __name__ == "__main__":

    signal.signal(signal.SIGINT, stop_node_handler)
    rospy.init_node('mbed_emu', anonymous=False, disable_signals=True)
    rospy.loginfo("Start mbed emulator : Ideal motor")
    rate = rospy.Rate(50) # 50hz

    cmd_vel_topic = rospy.get_param("~cmd_vel_topic")
    hw_feedback_topic = rospy.get_param("~hw_feedback_topic")

    pub = rospy.Publisher(hw_feedback_topic, Pose2D, queue_size=10)
    sub = rospy.Subscriber(cmd_vel_topic, Pose2D, cmd_vel_cb)

    running_ = True
    while (running_):

        recompute()
        rate.sleep()
        pub.publish(create_state_pos_vel_msg())

