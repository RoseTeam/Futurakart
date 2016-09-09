#!/usr/bin/env python

# Numpy
import numpy as np

# Opencv
import cv2

# ROS
import rospy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header

# Project
from base_node import BaseNode

# GLOBAL PARAMETERS
RGB_IMAGE_TOPIC_NAME = "/moc_camera/rgb/image_color"
DEPTH_IMAGE_TOPIC_NAME = "/moc_camera/depth/image"
DEPTH_POINTS_TOPIC_NAME = "/moc_camera/depth/points"
RATE = 30
RGB_IMAGE_WIDTH = 400
RGB_IMAGE_HEIGHT = 300
DEPTH_IMAGE_WIDTH = 360
DEPTH_IMAGE_HEIGHT = 240


class MocKinectNode(BaseNode):

    def __init__(self):
        BaseNode.__init__(self, "MocKinectNode", False)
        rospy.loginfo("Start MocKinectNode ...")
        self.rgb_image_pub = rospy.Publisher(RGB_IMAGE_TOPIC_NAME, Image, queue_size=1)
        self.depth_image_pub = rospy.Publisher(DEPTH_IMAGE_TOPIC_NAME, Image, queue_size=1)
        self.depth_points_pub = rospy.Publisher(DEPTH_POINTS_TOPIC_NAME, PointCloud2, queue_size=1)

        self.rate = rospy.Rate(RATE)
        self.running = True
        rospy.loginfo("Publish RGB, Depth, Points ...")
        while self.running:
            self.publish_rgb_image()
            self.publish_depth_image()
            self.publish_depth_points()
            self.rate.sleep()

    def publish_rgb_image(self):
        image = MocKinectNode._create_rgb_image()
        image_msg = Image()
        image_msg.header = Header()
        image_msg.header.frame_id = 'mockinect_rgb_frame'
        image_msg.height = image.shape[0]
        image_msg.width = image.shape[1]
        image_msg.step = image.shape[1] * image.shape[2]
        image_msg.encoding = 'rgb8'
        image_msg.data = image.flatten().tolist()
        self.rgb_image_pub.publish(image_msg)

    def publish_depth_image(self):
        depth = MocKinectNode._create_depth_image()
        depth_msg = Image()
        depth_msg.header = Header()
        depth_msg.header.frame_id = 'mockinect_depth_frame'
        depth_msg.height = depth.shape[0]
        depth_msg.width = depth.shape[1]
        depth_msg.step = depth.shape[1] * depth.shape[2]
        depth_msg.encoding = 'mono8'
        depth_msg.data = depth.flatten().tolist()

        self.depth_image_pub.publish(depth_msg)

    def publish_depth_points(self):
        depth = MocKinectNode._create_depth_image()
        points = PointCloud2()
        points.header = Header()
        points.header.frame_id = 'mockinect_depth_frame'
        points.height = depth.shape[0]
        points.width = depth.shape[1]
        points.point_step = 3
        points.row_step = points.point_step * depth.shape[1]
        points.is_dense = False
        data = np.zeros(depth.shape[:2] + (3,), dtype=np.uint8)
        for i in range(depth.shape[0]):
            for j in range(depth.shape[0]):
                data[i, j, 0] = i
                data[i, j, 1] = j
                data[i, j, 2] = depth[i, j]
        points.data = data.flatten().tolist()
        # !!! SOMETHING IS STILL MISSING !!!
        self.depth_points_pub.publish(points)

    @staticmethod
    def _create_rgb_image():
        data = np.zeros((RGB_IMAGE_HEIGHT, RGB_IMAGE_WIDTH, 3))

        data[0:2*RGB_IMAGE_HEIGHT/3, 0:3*RGB_IMAGE_WIDTH/4, 0] = 255
        data[2*RGB_IMAGE_HEIGHT/3:, 0:3*RGB_IMAGE_WIDTH/4, :] = 122
        data[2*RGB_IMAGE_HEIGHT/3:, 3*RGB_IMAGE_WIDTH/4:, 2] = 50
        data[0:2*RGB_IMAGE_HEIGHT/3:, 3*RGB_IMAGE_WIDTH/4:, 1:2] = 200

        data += 30*np.random.randn(RGB_IMAGE_HEIGHT, RGB_IMAGE_WIDTH, 3)
        data = (255.0 * (data - data.min()) / (data.max() - data.min())).astype(np.uint8)
        return data

    @staticmethod
    def _create_depth_image():
        data = np.zeros((DEPTH_IMAGE_HEIGHT, DEPTH_IMAGE_WIDTH, 1))

        data[0:2*DEPTH_IMAGE_HEIGHT/3, 0:3*DEPTH_IMAGE_WIDTH/4, 0] = 100
        data[2*DEPTH_IMAGE_HEIGHT/3:, 0:3*DEPTH_IMAGE_WIDTH/4, 0] = 60
        data[2*DEPTH_IMAGE_HEIGHT/3:, 3*DEPTH_IMAGE_WIDTH/4:, 0] = 255
        data[0:2*DEPTH_IMAGE_HEIGHT/3:, 3*DEPTH_IMAGE_WIDTH/4:, 0] = 70

        data += 40*np.random.randn(DEPTH_IMAGE_HEIGHT, DEPTH_IMAGE_WIDTH, 1)
        data = (255.0 * (data - data.min()) / (data.max() - data.min())).astype(np.uint8)
        return data


if __name__ == '__main__':
    try:
        ne = MocKinectNode()
    except rospy.ROSInterruptException:
        pass


    ## Partial tests:
    # img = MocKinectNode._create_rgb_image()
    # cv2.imshow("RGB", img)
    # dpt = MocKinectNode._create_depth_image()
    # cv2.imshow("Depth", dpt)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
