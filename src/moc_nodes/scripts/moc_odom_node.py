#!/usr/bin/env python

# Python
from math import cos, sin

# Ros PY
import rospy
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped

# Project
from base_node import BaseNode

# GLOBAL PARAMETERS
ODOM_TOPIC_NAME = "/odom"
ODOM_FRAME_NAME = "odom"
RATE = 20
INIT_X = 0.0
INIT_Y = 0.0
INIT_THETA = 0.0
INIT_VEL_X = 0.2
INIT_VEL_Y = 0.02
INIT_VEL_THETA = 0.1


class MocOdomNode(BaseNode):

    def __init__(self):
        BaseNode.__init__(self, "MocOdomNode", False)
        rospy.loginfo("Start MocOdomNode ...")
        self.odom_pub = rospy.Publisher(ODOM_TOPIC_NAME, Odometry, queue_size=50)
        self.odom_tf_broadcaster = TransformBroadcaster()

        self.coords = [INIT_X, INIT_Y, INIT_THETA]  # x,y,th
        self.velocity = [INIT_VEL_X, INIT_VEL_Y, INIT_VEL_THETA]  # vx, vy, vth

        self.last_time = rospy.Time.now()
        self.rate = rospy.Rate(RATE)
        self.running = True
        rospy.loginfo("Publish simulated odometry ...")

        while self.running:
            self.publish_odom(rospy.Time.now())
            self.rate.sleep()

    def publish_odom(self, current_time):

        dt = (current_time - self.last_time).nsecs * 1.0 / 1e9
        dx = (self.velocity[0] * cos(self.coords[2]) - self.velocity[1] * sin(self.coords[2])) * dt
        dy = (self.velocity[0] * sin(self.coords[2]) + self.velocity[1] * cos(self.coords[2])) * dt
        dth = self.velocity[2] * dt
        self.coords[0] += dx
        self.coords[1] += dy
        self.coords[2] += dth

        q = quaternion_from_euler(0, 0, self.coords[2])
        odom_quat = Quaternion(*q)

        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = ODOM_FRAME_NAME
        odom_trans.child_frame_id = "base_link"

        odom_trans.transform.translation.x = self.coords[0]
        odom_trans.transform.translation.y = self.coords[1]
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat

        self.odom_tf_broadcaster.sendTransformMessage(odom_trans)

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = ODOM_FRAME_NAME
        odom_msg.child_frame_id = "base_link"
        # set position
        odom_msg.pose.pose.position.x = self.coords[0]
        odom_msg.pose.pose.position.y = self.coords[1]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = odom_quat
        # set velocity
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.angular.z = self.velocity[2]

        self.odom_pub.publish(odom_msg)
        self.last_time = current_time


if __name__ == "__main__":
    try:
        ne = MocOdomNode()
    except rospy.ROSInterruptException:
        pass
