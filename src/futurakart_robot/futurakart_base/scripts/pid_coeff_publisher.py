#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from futurakart_base.cfg import PidCoeffConfig
from std_msgs.msg import Float32MultiArray


def callback(config, level):
    """
    Config :
    {
        'Prop_pos_Ki': 1.0,
        'Dir_vel_sat_min': 1.0,
        'Dir_vel_Kp': 1.0,
        'Prop_vel_Kp': 1.0,
        'Dir_vel_sat_max': 1.0,
        'Prop_pos_Kd': 1.0,
        'Prop_vel_sat_min': 1.0,
        'Dir_vel_Kd': 1.0,
        'Prop_pos_sat_min': 1.0,
        'Prop_vel_Ki': 1.0,
        'Prop_vel_Kd': 1.0,
        'Prop_pos_Kp': 1.0,
        'Prop_pos_sat_max': 1.0,
        'Prop_vel_sat_max': 1.0,
        'Dir_vel_Ki': 1.0,

        ...
    }
    """
    msg = Float32MultiArray()
    msg.data = [
        config['Prop_pos_Kp'],
        config['Prop_pos_Ki'],
        config['Prop_pos_Kd'],
        config['Prop_pos_sat_min'],
        config['Prop_pos_sat_max'],
        config['Prop_vel_Kp'],
        config['Prop_vel_Ki'],
        config['Prop_vel_Kd'],
        config['Prop_vel_sat_min'],
        config['Prop_vel_sat_max'],
        config['Dir_vel_Kp'],
        config['Dir_vel_Ki'],
        config['Dir_vel_Kd'],
        config['Dir_vel_sat_min'],
        config['Dir_vel_sat_max']
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
