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
    def __init__(self):
        self.image_pub = rospy.Publisher("/carla/ego_vehicle/camera/rgb/front/template_matching", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, self.callback)

    def callback(self, data):
        cv_img = None
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        # binarize the image
        ret, bw = cv2.threshold(gray, 128, 255,
                                cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        # find connected components
        connectivity = 4
        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(bw, connectivity, cv2.CV_32S)
        sizes = stats[1:, -1]; nb_components = nb_components - 1
        min_size = 250 #threshhold value for objects in scene
        img2 = numpy.zeros((cv_img.shape), numpy.uint8)
        for i in range(0, nb_components+1):
            # use if sizes[i] >= min_size: to identify your objects
            color = numpy.random.randint(255,size=3)
            # draw the bounding rectangele around each object
            cv2.rectangle(img2, (stats[i][0],stats[i][1]),(stats[i][0]+stats[i][2],stats[i][1]+stats[i][3]), (0,255,0), 2)
            img2[output == i + 1] = color

        cv2.imshow("Detected Window", img2)
        # Press q on the keyboard to shutdown.
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            rospy.is_shutdown()

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img2, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    template_matching = Template_Matching()
    rospy.init_node('template_matching', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down!")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

