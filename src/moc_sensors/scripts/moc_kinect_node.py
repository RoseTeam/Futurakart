#!/usr/bin/env python

# Numpy
import numpy as np

# Opencv
import cv2

# ROS
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2, PointField
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
        points = MocKinectNode._create_depth_points()
        header = Header()
        header.frame_id = 'mockinect_depth_frame'
        points_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.depth_points_pub.publish(points_msg)

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

    @staticmethod
    def _create_depth_points():
        points = np.zeros((DEPTH_IMAGE_HEIGHT, DEPTH_IMAGE_WIDTH, 3), dtype=np.float32)

        points[0:2*DEPTH_IMAGE_HEIGHT/3, 0:3*DEPTH_IMAGE_WIDTH/4, 2] = 1.0
        points[2*DEPTH_IMAGE_HEIGHT/3:, 0:3*DEPTH_IMAGE_WIDTH/4, 2] = 0.6
        points[2*DEPTH_IMAGE_HEIGHT/3:, 3*DEPTH_IMAGE_WIDTH/4:, 2] = 1.2
        points[0:2*DEPTH_IMAGE_HEIGHT/3:, 3*DEPTH_IMAGE_WIDTH/4:, 2] = 0.7

        for i in range(DEPTH_IMAGE_HEIGHT):
            points[i, :, 0] = (i - 0.5 * DEPTH_IMAGE_HEIGHT) * 0.01
        for i in range(DEPTH_IMAGE_WIDTH):
            points[:, i, 1] = (i - 0.5 * DEPTH_IMAGE_WIDTH) * 0.01
        points[..., 2] += 0.01*np.random.randn(DEPTH_IMAGE_HEIGHT, DEPTH_IMAGE_WIDTH)
        points = points.reshape((points.shape[0]*points.shape[1], points.shape[2]))
        return points


if __name__ == '__main__':
    try:
        ne = MocKinectNode()
    except rospy.ROSInterruptException:
        pass


    ## Partial tests:
    # pts = MocKinectNode._create_depth_points()
    # print pts[0:15, :], pts.shape
    #
    # header = Header()
    # header.frame_id = 'id0'
    # pts_msg = point_cloud2.create_cloud_xyz32(header, pts)
    # exit(1)
    #
    # img = MocKinectNode._create_rgb_image()
    # cv2.imshow("RGB", img)
    # dpt = MocKinectNode._create_depth_image()
    # cv2.imshow("Depth", dpt)
    # cv2.waitKey()
    # cv2.destroyAllWindows()

