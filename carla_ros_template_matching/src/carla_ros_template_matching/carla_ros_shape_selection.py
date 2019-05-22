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


class Shape_Selection:
    """
    Class used for converting ROS images to OpenCV images and apply Template Matching
    """
    def __init__(self):
        self.image_pub = rospy.Publisher("/carla/ego_vehicle/camera/rgb/front/shape_selection", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, self.callback)
        self.ref_point = []
        self.image = None
        self.cropping = False

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

        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(2000)
        world = client.get_world()
        blueprints = world.get_blueprint_library().filter('vehicle.*')

        # load the image, clone it, and setup the mouse callback function.
        clone = cv_img.copy()
        self.image = cv_img
        cv2.namedWindow("Image Window")
        cv2.setMouseCallback("Image Window", self.shape_selection)

        cv2.imshow("Image Window", cv_img)

        # Press r to reset the cropping region
        if (cv2.waitKey(1) & 0xFF) == ord("r"):
            cv_img = clone.copy()
        # Press q on the keyboard to shutdown.
        elif (cv2.waitKey(1) & 0xFF) == ord("q"):
            rospy.is_shutdown()

        # if there are two reference points, then crop the region of interest
        # from teh image and display it
        if len(self.ref_point) == 2:
            crop_img = clone[self.ref_point[0][1]:self.ref_point[1][1], self.ref_point[0][0]:self.ref_point[1][0]]
            cv2.imshow("crop_img", crop_img)
            cv2.waitKey(0)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))
        except CvBridgeError as e:
            print(e)




def main(args):
    shape_selection = Shape_Selection()
    rospy.init_node('shape_selection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down!")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

