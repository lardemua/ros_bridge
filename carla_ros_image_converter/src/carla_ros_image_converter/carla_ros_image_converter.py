#!/usr/bin/env python

# ------------------------
#   IMPORTS
# ------------------------
from __future__ import print_function

import sys
import rospy
import cv2
import carla
import json
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import roslib
roslib.load_manifest('carla_ros_image_converter')


class Image_Converter:
    """
    Class used for converting ROS images to OpenCV images
    """
    def __init__(self):
        self.image_pub = rospy.Publisher("/carla/ego_vehicle/camera/rgb/front/image_color_2", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, self.callback)

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

        data = {}
        for vehicle in world.get_actors().filter('vehicle.*'):
            # print(vehicle.bounding_box)
            # Draw bounding box
            transform = vehicle.get_transform()
            bounding_box = vehicle.bounding_box
            bounding_box.location += transform.location
            world.debug.draw_box(bounding_box, transform.rotation)
            data = self.write_json_dataset(data, vehicle, transform)

        cv2.imshow("Image Window", cv_img)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))
        except CvBridgeError as e:
            print(e)
        # save json dataset
        self.save_json_dataset(data)

    def write_json_dataset(self, data, vehicle, transform):
        data['vehicle'] = []
        data['vehicle'].append({
            'x': transform.location.x,
            'y': transform.location.y,
            'z': transform.location.z,
            'yaw': transform.rotation.yaw
        })
        # data['people'].append({
        #     'name': 'Larry',
        #     'website': 'google.com',
        #     'from': 'Michigan'
        # })
        # data['people'].append({
        #     'name': 'Tim',
        #     'website': 'apple.com',
        #     'from': 'Alabama'
        # })
        return data

    def save_json_dataset(self, data):
        with open('/home/pedro/catkin_ws/src/ros_bridge/datasets/data.json', 'w') as outfile:
            json.dump(data, outfile)




def main(args):
    img_converter = Image_Converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down!")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

