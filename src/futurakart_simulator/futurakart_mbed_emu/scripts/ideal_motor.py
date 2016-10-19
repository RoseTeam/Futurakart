#!/usr/bin/python2


#
# Script to emulate an ideally responsive motor
#
# Our nucleo card controls direction / propulsion motors
#
# It subcribes to a futurakart_msgs/MotorDrive topic named 'motordrive_cmd'
# It publishes a futurakart_msgs/MotorFeedback message as a topic named 'motorfeedback'
#

import signal
import rospy
from futurakart_msgs.msg import MotorDrive, MotorFeedback

verbose_ = True
running_ = False

dir_pos_ = 0.0 # Direction steering wheel angle in radians
prop_pos_ = 0.0 # Propulsion wheels rotation angle in radians
prop_vel_ = 0.0 # Propulsion wheels velocity
cmd_dir_pos_ = 0.0 # Direction command
cmd_prop_vel_ = 0.0 # Velocity command


def motordrive_cmd_cb(cmd_msg):
    global cmd_prop_vel_, cmd_dir_pos_
    if verbose_:
        rospy.loginfo("motordrive_cmd_cb : {}".format(cmd_msg))
    cmd_prop_vel_ = cmd_msg.prop_vel
    cmd_dir_pos_ = cmd_msg.dir_pos


def recompute(delta):
    global dir_pos_, prop_pos_, prop_vel_
    # ideal responsivity
    dir_pos_ = cmd_dir_pos_
    prop_pos_ += cmd_prop_vel_ * delta
    prop_vel_ = cmd_prop_vel_


def create_motorfeedback_msg():
    msg = MotorFeedback()
    msg.dir_pos = dir_pos_
    msg.prop_pos = prop_pos_
    msg.prop_vel = prop_vel_
    return msg


def stop_node_handler(*args, **kwargs):
    global running_
    running_ = False
    rospy.signal_shutdown("Shutdown...")


if __name__ == "__main__":
    signal.signal(signal.SIGINT, stop_node_handler)
    rospy.init_node('mbed_emu', anonymous=False, disable_signals=True)
    rospy.loginfo("Start mbed emulator : Ideal motor")
    rate = rospy.Rate(50) # 50hz

    motordrive_cmd_topic = rospy.get_param("~motordrive_cmd_topic")
    motorfeedback_topic = rospy.get_param("~motorfeedback_topic")
    verbose_ = rospy.get_param("~verbose")

    sub = rospy.Subscriber(motordrive_cmd_topic, MotorDrive, motordrive_cmd_cb)
    pub = rospy.Publisher(motorfeedback_topic, MotorFeedback, queue_size=10)

    running_ = True
    last_time = rospy.get_time()
    while (running_):

        current_time = rospy.get_time()
        delta = current_time - last_time
        last_time = current_time
        recompute(delta)
        pub.publish(create_motorfeedback_msg())
        rate.sleep()

