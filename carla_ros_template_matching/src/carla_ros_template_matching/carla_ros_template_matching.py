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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import roslib
roslib.load_manifest('carla_ros_template_matching')


class Template_Matching:
    """
    Class used for converting ROS images to OpenCV images and apply Template Matching
    """
    def __init__(self, role_name):
        self.role_name = role_name
        self.image_pub = rospy.Publisher("/carla/{}/camera/rgb/front/template_matching".format(self.role_name), Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/carla/{}/camera/rgb/front/image_color".format(self.role_name), Image, self.callback)

    def callback(self, data):
        cv_img = None
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_img.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_img, (50,50), 10, 255)
        # cv2.circle(cv_img, (50,50), 10, 255)

        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(2000)
        world = client.get_world()
        blueprints = world.get_blueprint_library().filter('vehicle.*')

        # print blueprints
        # print(blueprints)

        template_1 = cv2.imread('/home/pedro/catkin_ws/src/ros_bridge/carla_ros_template_matching/templates/template09.png', cv2.IMREAD_GRAYSCALE)
        tW1, tH1 = template_1.shape[::-1]

        template_2 = cv2.imread('/home/pedro/catkin_ws/src/ros_bridge/carla_ros_template_matching/templates/template10.png', cv2.IMREAD_GRAYSCALE)
        tW2, tH2 = template_2.shape[::-1]

        template_3 = cv2.imread('/home/pedro/catkin_ws/src/ros_bridge/carla_ros_template_matching/templates/template11.png', cv2.IMREAD_GRAYSCALE)
        tW3, tH3 = template_3.shape[::-1]

        gray_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        threshold = 0.8

        # define range of white color in HSV and change it according to your need.
        sensitivity = 15
        # lower_white = np.array([0,0,0], dtype=np.uint8)
        # upper_white = np.array([0,0,255], dtype=np.uint8)
        lower_white = numpy.array([0, 0, 255 - sensitivity])
        upper_white = numpy.array([255, sensitivity, 255])

        # threshold the HSV image to get only white colors
        # mask = cv2.inRange(hsv_img, lower_white, upper_white)
        # cv2.imshow('mask', mask)
        res1 = cv2.matchTemplate(gray_img, template_1, cv2.TM_CCOEFF_NORMED)
        loc1 = numpy.where(res1 >= threshold)
        res2 = cv2.matchTemplate(gray_img, template_2, cv2.TM_CCOEFF_NORMED)
        loc2 = numpy.where(res2 >= threshold)
        res3 = cv2.matchTemplate(gray_img, template_3, cv2.TM_SQDIFF_NORMED)
        min_value0, max_value0, min_loc0, max_loc0 = cv2.minMaxLoc(res3)
        loc3 = numpy.where(res3 <= min_value0)

        for pt in zip(*loc1[::-1]):
            cv2.rectangle(cv_img, pt, (pt[0] + tW1, pt[1] + tH1), (0, 0, 255), 2)

        for pt in zip(*loc2[::-1]):
            cv2.rectangle(cv_img, pt, (pt[0] + tW2, pt[1] + tH2), (0, 255, 255), 3)

        for pt in zip(*loc3[::-1]):
            cv2.rectangle(cv_img, pt, (pt[0] + tW3, pt[1] + tH3), (255, 255, 255), 5)

        cv2.imshow("Detected Window", cv_img)
        # Press q on the keyboard to shutdown.
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            rospy.is_shutdown()

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    template_matching = Template_Matching(role_name)
    rospy.init_node('template_matching', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down!")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

