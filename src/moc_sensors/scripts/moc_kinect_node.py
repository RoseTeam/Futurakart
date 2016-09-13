#!/usr/bin/env python

# Numpy
import numpy as np

# Opencv
import cv2

# ROS
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, RegionOfInterest
from std_msgs.msg import Header

# Project
from base_node import BaseNode

# GLOBAL PARAMETERS
RGB_IMAGE_TOPIC_NAME = "/camera/rgb/image_color"
RGB_CAMERA_INFO_TOPIC_NAME = "/camera/rgb/camera_info"
DEPTH_IMAGE_TOPIC_NAME = "/camera/depth/image"
DEPTH_IMAGE_RAW_TOPIC_NAME = "/camera/depth/image_raw"
DEPTH_POINTS_TOPIC_NAME = "/camera/depth/points"
DEPTH_CAMERA_INFO_TOPIC_NAME = "/camera/depth/camera_info"
RATE = 30
RGB_IMAGE_WIDTH = 640
RGB_IMAGE_HEIGHT = 480
DEPTH_IMAGE_WIDTH = 640
DEPTH_IMAGE_HEIGHT = 488


class MocKinectNode(BaseNode):

    def __init__(self):
        BaseNode.__init__(self, "MocKinectNode", False)
        rospy.loginfo("Start MocKinectNode ...")
        self.rgb_image_pub = rospy.Publisher(RGB_IMAGE_TOPIC_NAME, Image, queue_size=1)
        self.depth_image_pub = rospy.Publisher(DEPTH_IMAGE_TOPIC_NAME, Image, queue_size=1)
        self.depth_image_raw_pub = rospy.Publisher(DEPTH_IMAGE_RAW_TOPIC_NAME, Image, queue_size=1)
        self.depth_points_pub = rospy.Publisher(DEPTH_POINTS_TOPIC_NAME, PointCloud2, queue_size=1)

        self.rgb_cam_info_pub = rospy.Publisher(RGB_CAMERA_INFO_TOPIC_NAME, CameraInfo, queue_size=1)
        self.depth_cam_info_pub = rospy.Publisher(DEPTH_CAMERA_INFO_TOPIC_NAME, CameraInfo, queue_size=1)

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
        image_msg.header.frame_id = 'kinect_rgb_frame'
        image_msg.height = image.shape[0]
        image_msg.width = image.shape[1]
        image_msg.step = image.shape[1] * image.shape[2]
        image_msg.encoding = 'rgb8'
        image_msg.data = image.flatten().tolist()

        self.rgb_image_pub.publish(image_msg)
        self.rgb_cam_info_pub.publish(MocKinectNode._create_camera_info(type="rgb"))

    def publish_depth_image(self):
        depth = MocKinectNode._create_depth_image()

        depth_msg = Image()
        depth_msg.header = Header()
        depth_msg.header.frame_id = 'kinect_depth_frame'
        depth_msg.height = depth.shape[0]
        depth_msg.width = depth.shape[1]
        depth_msg.step = depth.shape[1] * depth.shape[2]
        depth_msg.encoding = 'mono8'
        data_uint8 = depth.copy()
        data_uint8 = (255.0 * (data_uint8 - data_uint8.min()) / (data_uint8.max() - data_uint8.min())).astype(np.uint8)
        depth_msg.data = data_uint8.flatten().tolist()

        self.depth_image_pub.publish(depth_msg)

        depth_msg2 = Image()
        depth_msg2.header = Header()
        depth_msg2.header.frame_id = 'kinect_depth_frame'
        depth_msg2.height = depth.shape[0]
        depth_msg2.width = depth.shape[1]
        depth_msg2.step = 4 * depth.shape[1] * depth.shape[2]
        depth_msg2.encoding = '32FC1'
        depth_msg2.data = list(bytearray(depth.flatten()))

        self.depth_image_raw_pub.publish(depth_msg2)
        self.depth_cam_info_pub.publish(MocKinectNode._create_camera_info(type="depth"))

    def publish_depth_points(self):
        points = MocKinectNode._create_depth_points()
        header = Header()
        header.frame_id = 'kinect_depth_frame'
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
        data = np.zeros((DEPTH_IMAGE_HEIGHT, DEPTH_IMAGE_WIDTH, 1), dtype=np.float32)

        data[0:2*DEPTH_IMAGE_HEIGHT/3, 0:3*DEPTH_IMAGE_WIDTH/4, 0] = 5.0
        data[2*DEPTH_IMAGE_HEIGHT/3:, 0:3*DEPTH_IMAGE_WIDTH/4, 0] = 2.0
        data[2*DEPTH_IMAGE_HEIGHT/3:, 3*DEPTH_IMAGE_WIDTH/4:, 0] = 3.61
        data[0:2*DEPTH_IMAGE_HEIGHT/3:, 3*DEPTH_IMAGE_WIDTH/4:, 0] = 4.5

        # data += 40*np.random.randn(DEPTH_IMAGE_HEIGHT, DEPTH_IMAGE_WIDTH, 1)
        return data

    @staticmethod
    def _create_depth_points():
        points = np.zeros((DEPTH_IMAGE_HEIGHT, DEPTH_IMAGE_WIDTH, 3), dtype=np.float32)
        points[..., 2] = MocKinectNode._create_depth_image()[..., 0]

        for i in range(DEPTH_IMAGE_HEIGHT):
            points[i, :, 0] = (i - 0.5 * DEPTH_IMAGE_HEIGHT) * 0.01
        for i in range(DEPTH_IMAGE_WIDTH):
            points[:, i, 1] = (i - 0.5 * DEPTH_IMAGE_WIDTH) * 0.01
        points[..., 2] += 0.01*np.random.randn(DEPTH_IMAGE_HEIGHT, DEPTH_IMAGE_WIDTH)
        points = points.reshape((points.shape[0]*points.shape[1], points.shape[2]))
        return points

    @staticmethod
    def _create_camera_info(type):
        cam_info = CameraInfo()
        cam_info.header = Header()
        cam_info.distortion_model = "plumb_bob"
        if type == "rgb":
            cam_info.height = RGB_IMAGE_HEIGHT
            cam_info.width = RGB_IMAGE_WIDTH
            cam_info.K = (521.179233, 0, 322.515987,
                          0, 493.033034, 259.055966,
                          0, 0, 1)
            cam_info.P = (521.179233, 0, 322.515987, 0.0,
                          0, 493.033034, 259.055966, 0.0,
                          0, 0, 1, 0)
            cam_info.D = [5.858325e-02, 3.856792e-02, 0.000000e+00, 0.000000e+00, 0.000000e+00]

        elif type == "depth":
            cam_info.height = DEPTH_IMAGE_HEIGHT
            cam_info.width = DEPTH_IMAGE_WIDTH
            cam_info.K = (572.882768, 0, 314.649173,
                          0, 542.739980, 240.160459,
                          0, 0, 1)
            cam_info.P = (572.882768, 0, 314.649173, 0.0,
                          0, 542.739980, 240.160459, 0.0,
                          0, 0, 1, 0)
            cam_info.D = [-4.747169e-03, -4.357976e-03, 0.000000e+00, 0.000000e+00, 0.000000e+00]
        else:
            raise Exception("Type '%s' is unknown" % type)

        cam_info.R = [1, 0, 0,
                      0, 1, 0,
                      0, 0, 1]

        cam_info.binning_x = 2
        cam_info.binning_y = 2
        cam_info.roi = RegionOfInterest()
        cam_info.roi.x_offset = 10
        cam_info.roi.y_offset = 10
        cam_info.roi.width = 50
        cam_info.roi.height = 50

        return cam_info


if __name__ == '__main__':
    try:
        ne = MocKinectNode()
    except rospy.ROSInterruptException:
        pass


    ## Partial tests:
    # cam_info = MocKinectNode._create_camera_info('rgb')
    #
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

