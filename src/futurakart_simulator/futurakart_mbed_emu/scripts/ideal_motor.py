#!/usr/bin/python2


#
# Script to emulate an ideally responsive motor
#
# Our nucleo card controls direction / propulsion motors
#
# It subcribes to a futurakart_msgs/MotorDrive topic named 'motordrive_cmd' to receive motor commands
# It subcribes to a std_msgs/Float32MultiArray topic named 'pid_coeff_cmd' to receive PID coefficients
# It publishes a futurakart_msgs/MotorFeedback message as a topic named 'motorfeedback'
#

import signal
import rospy
from futurakart_msgs.msg import MotorDrive, MotorFeedback
from std_msgs.msg import Float32MultiArray
from kart_motors_controller import KartMotorsController, Motor, QEI, PotentioMeter, DEBUG_PROP, DEBUG_DIR

verbose_ = True
running_ = False

#feedback_dir_pos_ = 0.0  # Direction steering wheel angle in rads
#feedback_prop_pos_ = 0.0  # Propulsion wheels rotation angle in rads
#feedback_prop_vel_ = 0.0  # Propulsion wheels velocity in rad/s
feedback_ = [0.0, 0.0, 0.0]  # [feedback_dir_pos_, feedback_prop_pos_, feedback_prop_vel_]

setpoint_prop_vel_ = 0.0  # Setpoint propulsion velocity in rad/s
setpoint_dir_pos_ = 0.0  # Setpoint propulsion velocity in rads

cmd_dir_pos_ = 0.0  # Direction command in unitary values
cmd_prop_vel_ = 0.0  # Velocity command in unitary values


wait_time_ = 2.0  #
pulses_per_rev_ = 2048

dir_motor_ = Motor()
prop_motor_ = Motor()
wheel_ = QEI(prop_motor_, pulses_per_rev_)
dir_potmeter_ = PotentioMeter(dir_motor_)
controller_ = KartMotorsController(1.0, feedback_, wheel_, pulses_per_rev_, dir_potmeter_)


def motordrive_cmd_cb(cmd_msg):
    global setpoint_prop_vel_, setpoint_dir_pos_
    if verbose_:
        rospy.loginfo("motordrive_cmd_cb : {}".format(cmd_msg))
    setpoint_prop_vel_ = cmd_msg.prop_vel
    setpoint_dir_pos_ = cmd_msg.dir_pos


def pid_coeff_cmd_cb(pid_coeff_msg):
    rospy.loginfo("pid_coeff_cmd_cb : {}".format(pid_coeff_msg))
    controller_.pid_coeff = pid_coeff_msg.data


def publish_motorfeedback():
    msg = MotorFeedback()
    msg.dir_pos = feedback_[0]
    msg.prop_pos = feedback_[1]
    msg.prop_vel = feedback_[2]
    pub.publish(msg)


def stop_node_handler(*args, **kwargs):
    global running_
    running_ = False
    rospy.signal_shutdown("Shutdown...")


if __name__ == "__main__":

    signal.signal(signal.SIGINT, stop_node_handler)
    rospy.init_node('mbed_emu', anonymous=False, disable_signals=True)
    rospy.loginfo("Start mbed emulator : Ideal motor")
    rate = rospy.Rate(5) # wait 1/200 ms
    
    motordrive_cmd_topic = rospy.get_param("~motordrive_cmd_topic")
    pid_coeff_cmd_topic = rospy.get_param("~pid_coeff_cmd_topic")
    motorfeedback_topic = rospy.get_param("~motorfeedback_topic")
    verbose_ = rospy.get_param("~verbose")

    sub_motordrive = rospy.Subscriber(motordrive_cmd_topic, MotorDrive, motordrive_cmd_cb)
    sub_pid_coeff = rospy.Subscriber(pid_coeff_cmd_topic, Float32MultiArray, pid_coeff_cmd_cb)
    pub = rospy.Publisher(motorfeedback_topic, MotorFeedback, queue_size=10)

    running_ = True
    last_time = rospy.get_time()
    while running_:

        current_time = rospy.get_time()
        delta = current_time - last_time
        if delta > wait_time_:
            if DEBUG_DIR or DEBUG_PROP:
                print "------- publish /motorfeedback : ", feedback_
            publish_motorfeedback()
            last_time = current_time

        if DEBUG_DIR or DEBUG_PROP:
            print "- before control : ", setpoint_dir_pos_, setpoint_prop_vel_, cmd_dir_pos_, cmd_prop_vel_

        cmd_dir_pos_, cmd_prop_vel_ = controller_.control(current_time, setpoint_dir_pos_, setpoint_prop_vel_, cmd_dir_pos_, cmd_prop_vel_)

        if DEBUG_DIR or DEBUG_PROP:
            print "- after control : ", cmd_dir_pos_, cmd_prop_vel_
            print "- before cmd : ", dir_motor_._pos, prop_motor_._vel 

        dir_motor_.set_cmd_pos(cmd_dir_pos_)
        prop_motor_.set_cmd_vel(cmd_prop_vel_, current_time)

        if DEBUG_DIR or DEBUG_PROP:
            print "-  after cmd : ", dir_motor_._pos, prop_motor_._vel 
        
        rate.sleep()




