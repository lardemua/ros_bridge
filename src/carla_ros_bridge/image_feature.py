#!/usr/bin/env python

"""
OpenCV feature detectors with ros CompressedImage topics in python.
This example subscribes to a ros topic containing sensor_msgs CompressedImage.
It converts the CompressedImage into a numpy.ndarray, then detects and mark features in that image.
It finally displays and publishes the new image - again as CompressedImage topic.
"""
__author__ = 'pedro'
__version__ = '0.1'
__license__ = 'BSD'

# ------------------------
#   IMPORTS
# ------------------------
# Python Libs
import sys
import time
# Numpy and scipy
import numpy as np
import scipy.ndimage.filters
# OpenCV
import cv2
# ROS Libraries
import rospy
import roslib
# ROS Messages
from sensor_msgs.msg import CompressedImage

VERBOSE = false


class Image_Feature:

      def __init__(self):
          """
          Initialize ROS Publisher, ROS Subscriber
          """
          # topic where we publish
          self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)
          # subscribed topic
          self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size=1)
          if VERBOSE:
              print("subscribed to /camera/image/compressed")

      def callback(self, ros_data):
          """
          Callback function of subscribed topic.
          Here images get converted and features detected.
          :param ros_data:
          :return:
          """
          if VERBOSE:
              print('received image of type: "%s"' % ros_data.format)

          # direct conversion to CV2
          np_array = np.fromstring(ros_data.data, np.uint8)
          image_np = cv2.imdecode(np_array, cv2.LOAD_IMAGE_COLOR)

          # feature detectors using CV2
          method = "GridFAST"
          feat_detector = cv2.FeatureDetector_create(method)
          time1 = time.time()

          # convert np image to grayscale
          featPoints = feat_detector.detect(cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
          time2 = time.time()
          if VERBOSE:
              print('%s detector found: %s points in: %s sec.'%(method,len(featPoints),time2-time1))

          for featpoint in featPoints:
              x,y = featpoint.pt
              cv2.circle(image_np, (int(x), int(y)), 3, (0,0,255), -1)

          cv2.imshow('cv_img', image_np)
          cv2.waitKey(2)

          # create CompressedImage
          msg = CompressedImage()
          msg.header.stamp = rospy.Time.now()
          msg.format = "jpeg"
          msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
          # Publish new image
          self.image_pub.publish(msg)


def main(args):
    """
    Initialize and cleanup ros node
    :param args:
    :return:
    """
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

