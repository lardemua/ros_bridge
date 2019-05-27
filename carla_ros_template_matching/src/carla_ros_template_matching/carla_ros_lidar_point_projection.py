#!/usr/bin/env python

# ------------------------
#   IMPORTS
# ------------------------
from __future__ import print_function

import sys
import rospy
import cv2
import carla
import numpy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import roslib
roslib.load_manifest('carla_ros_template_matching')


class Lidar_Point_Projection:
    """
    Class used for converting ROS images to OpenCV images and apply Template Matching with Shape Selection and
    point projection from the LIDAR sensors.
    """
    def __init__(self):
        self.image_pub = rospy.Publisher("/carla/ego_vehicle/camera/rgb/front/lidar_point_projection", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, self.callback)
        self.front_lidar_sub = rospy.Subscriber("/carla/ego_vehicle/lidar/front/point_cloud", PointCloud2,
                                                self.callback_lidar_front)
        self.left_lidar_sub = rospy.Subscriber("/carla/ego_vehicle/lidar/left/point_cloud", PointCloud2,
                                               self.callback_lidar_left)
        self.right_lidar_sub = rospy.Subscriber("/carla/ego_vehicle/lidar/right/point_cloud", PointCloud2,
                                                self.callback_lidar_right)
        self.ref_point = []
        self.image = None
        self.template_img = None
        self.tW = None
        self.tH = None
        self.cropping = False
        self.template_matching = False
        self.project_points = False
        self.threshold = 0.8
        self.obj_points = []     # 3d points in real world space
        self.img_points = []     # 2d points in image plane
        self.calibration = []
        self.distCoeffs = []
        self.rotVector = []
        self.transVector = []
        self.end_point_3d = []
        self.end_point_2d = []
        self.view_width = 1920//2
        self.view_height = 1080//2
        self.view_fov = 90

        self.front_width = None
        self.front_height = None
        self.front_depth = None
        self.left_width = None
        self.left_height = None
        self.left_depth = None
        self.right_width = None
        self.right_height = None
        self.right_depth = None

    def shape_selection(self, event, x, y, flags, param):
        # if the left mouse button was clicked, record the starting (x,y) coordinates
        # and indicate that the croping is being performed.
        if event == cv2.EVENT_LBUTTONDOWN:
            self.ref_point = [(x, y)]
            self.cropping = True
        # check to see if the left mouse button was released
        elif event == cv2.EVENT_LBUTTONUP:
            # record the ending (x,y) coordinates and indicate that the cropping operation is finished
            self.ref_point.append((x, y))
            self.cropping = False

            # draw a rectangle around the region of interest
            cv2.rectangle(self.image, self.ref_point[0], self.ref_point[1], (0, 255, 0), 2)
            cv2.imshow("Cropped Image", self.image)

    def read_depth_lidar(self, width, height, carla_lidar_data):
        # read function
        if (height >= carla_lidar_data.height) or (width >= carla_lidar_data.width):
            return -1
        data_out = pc2.read_points(carla_lidar_data, field_names=None, skip_nans=False, uvs=[[width, height]])
        int_data = next(data_out)
        rospy.loginfo("int_data " + str(int_data))
        return int_data

    def callback_lidar_front(self, carla_lidar_data):
        # pick a height
        self.front_height = int(carla_lidar_data.height / 2)
        # pick a width
        self.front_width = int(carla_lidar_data.width / 2)
        # examine point and read depth
        self.front_depth = self.read_depth_lidar(self.front_width, self.front_height, carla_lidar_data)
        # print info
        rospy.loginfo("Front LIDAR --> Width: " + str(self.front_width) + " , Height: " + str(self.front_height) +
                      ", Depth: " + str(self.front_depth))

    def callback_lidar_left(self, carla_lidar_data):
        # pick a height
        self.left_height = int(carla_lidar_data.height / 2)
        # pick a width
        self.left_width = int(carla_lidar_data.width / 2)
        # examine point and read depth
        self.left_depth = self.read_depth_lidar(self.left_width, self.left_height, carla_lidar_data)
        # print info
        rospy.loginfo("Left LIDAR --> Width: " + str(self.left_width) + " , Height: " + str(self.left_height) +
                      ", Depth: " + str(self.left_depth))

    def callback_lidar_right(self, carla_lidar_data):
        # pick a height
        self.right_height = int(carla_lidar_data.height / 2)
        # pick a width
        self.right_width = int(carla_lidar_data.width / 2)
        # examine point and read depth
        self.right_depth = self.read_depth_lidar(self.right_width, self.right_height, carla_lidar_data)
        # print info
        rospy.loginfo("Right LIDAR --> Width: " + str(self.right_width) + " , Height: " + str(self.right_height) +
                      ", Depth: " + str(self.right_depth))

    def callback(self, data):
        cv_img = None
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(2000)
        world = client.get_world()
        blueprints = world.get_blueprint_library().filter('vehicle.*')

        calibration = numpy.identity(3)
        calibration[0, 2] = self.view_width/ 2.0
        calibration[1, 2] = self.view_height / 2.0
        calibration[0, 0] = calibration[1, 1] = self.view_width / (2.0 * numpy.tan(self.view_fov * numpy.pi / 360.0))
        self.calibration = calibration

        self.distCoeffs = numpy.array([0, 0, 0, 0], dtype=numpy.float)
        self.rotVector = numpy.array([0], dtype=numpy.float)
        self.transVector = numpy.array([0], dtype=numpy.float)
        self.end_point_3d = numpy.array([0, 0, 1000], dtype=numpy.float)
        self.end_point_2d = numpy.array([0, 0], dtype=numpy.float)

        # load the image, clone it, and setup the mouse callback function.
        clone = cv_img.copy()
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        self.image = cv_img
        cv2.namedWindow("Image Window")
        cv2.setMouseCallback("Image Window", self.shape_selection)
        cv2.namedWindow("3D Window")
        cv_img_3D = numpy.array([0, 0, 1000], dtype=numpy.float)

        # Press r to reset the cropping region
        if (cv2.waitKey(1) & 0xFF) == ord("r"):
            cv_img = clone.copy()
            self.template_matching = False
        # Press q on the keyboard to shutdown.
        elif (cv2.waitKey(1) & 0xFF) == ord("q"):
            rospy.is_shutdown()

        # if there are two reference points, then crop the region of interest
        # from the image and display it
        if len(self.ref_point) == 2:
            crop_img = clone[self.ref_point[0][1]:self.ref_point[1][1], self.ref_point[0][0]:self.ref_point[1][0]]
            cv2.imshow("crop_img", crop_img)
            template = crop_img
            self.template_img = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            self.tW, self.tH = self.template_img.shape[::-1]
            self.template_matching = True
            cv2.waitKey(0)

        if self.template_matching is True:
            res = cv2.matchTemplate(gray, self.template_img, cv2.TM_CCOEFF_NORMED)
            loc = numpy.where(res >= self.threshold)
            for pt in zip(*loc[::-1]):
                cv2.rectangle(cv_img, pt, (pt[0] + self.tW, pt[1] + self.tH), (0, 0, 255), 2)

        if self.project_points is True:
            cv2.projectPoints(cv_img_3D, self.rotVector, self.transVector, self.calibration, self.distCoeffs, cv_img)
        cv2.imshow("Image Window", cv_img)
        cv2.imshow("3D Window", cv_img_3D)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    point_projection = Lidar_Point_Projection()
    rospy.init_node('lidar_point_projection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down!")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

