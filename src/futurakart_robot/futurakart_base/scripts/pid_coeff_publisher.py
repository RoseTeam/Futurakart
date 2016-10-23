#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from futurakart_base.cfg import PidCoeffConfig
from std_msgs.msg import Float32MultiArray


def callback(config, level):
    msg = Float32MultiArray()
    msg.data = [
        config['Prop_vel_Kp'],
        config['Prop_vel_Ki'],
        config['Prop_vel_Kd'],
        config['Prop_vel_sat_tot'],
        config['Prop_vel_sat_Ki'],
        config['Dir_pos_Kp'],
        config['Dir_pos_Ki'],
        config['Dir_pos_Kd'],
        config['Dir_pos_sat_tot'],
        config['Dir_pos_sat_Ki']
    ]
    pub.publish(msg)

    return config

if __name__ == "__main__":
    rospy.init_node("pid_coeff_publisher", anonymous=True)
    rospy.loginfo("Start pid coefficient publisher")

    pid_coeff_cmd_topic = rospy.get_param("~pid_coeff_cmd_topic")
    pub = rospy.Publisher(pid_coeff_cmd_topic, Float32MultiArray, queue_size=1)

    srv = Server(PidCoeffConfig, callback)
    rospy.spin()
