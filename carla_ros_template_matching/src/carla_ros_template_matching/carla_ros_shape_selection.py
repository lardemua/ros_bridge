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
from carla_ros_bridge_msgs.msg import CarlaTemplateInfo, Carla3DTemplateInfo
from cv_bridge import CvBridge, CvBridgeError
import roslib
roslib.load_manifest('carla_ros_template_matching')


class Shape_Selection:
    """
    Class used for converting ROS images to OpenCV images and apply Template Matching with Shape Selection
    """
    def __init__(self, role_name):
        self.role_name = role_name
        self.image_pub = rospy.Publisher("/carla/{}/camera/rgb/front/shape_selection".format(self.role_name), Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/carla/{}/camera/rgb/front/image_color".format(self.role_name), Image,
                                          self.callback)
        self.template_pub = rospy.Publisher("/carla/{}/templates".format(self.role_name), CarlaTemplateInfo)
        self.template_sub = rospy.Publisher("/carla/{}/3Dtemplates".format(self.role_name), Carla3DTemplateInfo,
                                            self.design_3D_template)
        self.template_info = None
        self.ref_point = []
        self.image = None
        self.template_img = None
        self.tW = None
        self.tH = None
        self.cropping = False
        self.template_matching = False
        self.threshold = 0.8

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

    def callback(self, data):
        cv_img = None
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # client = carla.Client('127.0.0.1', 2000)
        # client.set_timeout(2000)
        # world = client.get_world()
        # blueprints = world.get_blueprint_library().filter('vehicle.*')

        # load the image, clone it, and setup the mouse callback function.
        clone = cv_img.copy()
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        self.image = cv_img
        cv2.namedWindow("Image Window")
        cv2.setMouseCallback("Image Window", self.shape_selection)
        print("I AM HERE")
        cv2.waitKey(100)

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

        cv2.imshow("Image Window", cv_img)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def on_template(self, width, height):
        if width is not None and height is not None:
            self.template_info = CarlaTemplateInfo()
            self.template_info.template_width = width
            self.template_info.template_height = height
            # publish ROS message
            self.template_pub.publish(self.template_info)

    def design_3D_template(self, data):
        print("tW: " + str(data.template_width) + ", tH: " + str(data.template_height) +
              ", tD: " + str(data.template_depth))


def main(args):
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    shape_selection = Shape_Selection(role_name)
    rospy.init_node('shape_selection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down!")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

