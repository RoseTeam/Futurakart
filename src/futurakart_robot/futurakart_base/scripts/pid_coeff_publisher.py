#!/usr/bin/env python

import rospy
import yaml
from dynamic_reconfigure.server import Server
from futurakart_base.cfg import PidCoeffConfig
from std_msgs.msg import Float32MultiArray

coeff_names = ['Prop_vel_Kp',
               'Prop_vel_Ki',
               'Prop_vel_Kd',
               'Prop_vel_sat_tot',
               'Prop_vel_sat_Ki',
               'Dir_pos_Kp',
               'Dir_pos_Ki',
               'Dir_pos_Kd',
               'Dir_pos_sat_tot',
               'Dir_pos_sat_Ki']


def callback(config, level):
    msg = Float32MultiArray()
    msg.data = [config[item] for item in coeff_names]
    pub.publish(msg)

    return config

if __name__ == "__main__":
    rospy.init_node("pid_coeff_publisher", anonymous=True)
    rospy.loginfo("Start pid coefficient publisher")

    pid_coeff_cmd_topic = rospy.get_param("~pid_coeff_cmd_topic")
    default_pid_coeff_file = rospy.get_param("~default_pid_coeff_file", "../config/PidCoeffDefault.yaml")
    pub = rospy.Publisher(pid_coeff_cmd_topic, Float32MultiArray, queue_size=1)

    srv = Server(PidCoeffConfig, callback)

    try:
        with open(default_pid_coeff_file, 'r') as f:
            def_pid_coeff = yaml.load(f)
        for item in coeff_names:
            PidCoeffConfig.defaults[item] = def_pid_coeff[item]
    except Exception as e:
        rospy.logerr("Failed to setup default pid coefficients. Error : {}".format(e))

    # Send once to initialize:
    #print "!!!! PUBLISH DEFAULT VALUES : {}".format(PidCoeffConfig.defaults)
    # !!! Probably, this does not work !!!
    srv.update_configuration(PidCoeffConfig.defaults)
    
    rospy.spin()
