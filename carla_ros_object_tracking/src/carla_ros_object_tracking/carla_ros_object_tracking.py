#!/usr/bin/env python

# ------------------------
#   IMPORTS
# ------------------------
from __future__ import print_function

import sys
import rospy
import random
import cv2
import carla
import json
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import roslib
roslib.load_manifest('carla_ros_object_tracking')


class Object_Tracking:
    """
    Class used for object tracking using ROS images
    """
    def __init__(self):
        self.image_pub = rospy.Publisher("/carla/ego_vehicle/camera/rgb/front/object_tracking", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, self.callback)

    def callback(self, data):
        cv_img = None
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_img.shape

        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(2000)
        world = client.get_world()
        blueprints = world.get_blueprint_library().filter('vehicle.*')

        # print blueprints
        # print(blueprints)

        data = {}
        data['vehicles'] = []
        for vehicle in world.get_actors().filter('vehicle.*'):
            # print(vehicle.bounding_box)
            # Draw bounding box
            transform = vehicle.get_transform()
            bounding_box = vehicle.bounding_box
            bounding_box.location += transform.location
            extent = bounding_box.extent
            world.debug.draw_box(bounding_box, transform.rotation)
            data = self.write_json_dataset(data, vehicle, transform, extent)

        cv2.imshow("Image Window", cv_img)
        cv2.waitKey(1)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

        # save json dataset
        self.save_json_dataset(data)
        self.parse_json_dataset()

    def write_json_dataset(self, data, vehicle, transform, extent):
        volume = self.bounding_box_volume(extent)
        classification_model = None
        blueprint_model = None
        if str(vehicle.attributes['number_of_wheels']) == '4':
            if volume <= 2.17:
                classification_model = 'car'
                blueprint_model = random.choice(['vehicle.audi.tt', 'vehicle.citroen.c3', 'vehicle.mercedes.benz',
                                                 'vehicle.nissan.micra', 'vehicle.nissan.patrol', 'vehicle.bmw.isetta'])
            else:
                classification_model = 'truck'
                blueprint_model = random.choice(['vehicle.jeep.rubicon', 'vehicle.carla.colatruck',
                                                 'vehicle.volkswaggen.T2'])
        elif str(vehicle.attributes['number_of_wheels']) == '2':
            if volume <= 0.18:
                classification_model = 'bicycle'
                blueprint_model = random.choice(['vehicle.bike.cross','vehicle.bike.road'])
            else:
                classification_model = 'motorcycle'
                blueprint_model = random.choice(['vehicle.yamaha.yzf','vehicle.ninja.kawasaki'])
        data['vehicles'].append({
            'model': blueprint_model,
            'class': classification_model,
            'x': str(transform.location.x),
            'y': str(transform.location.y),
            'z': str(transform.location.z),
            'bx': str(extent.x),
            'by': str(extent.y),
            'bz': str(extent.z),
            'yaw': str(transform.rotation.yaw),
        })
        # data['vehicle'].append("\\n\\n")
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
        with open('/home/pedro/catkin_ws/src/ros_bridge/datasets/data.json', 'w') as json_file:
            json.dump(data, json_file)

    def parse_json_dataset(self):
        with open('/home/pedro/catkin_ws/src/ros_bridge/datasets/data.json', 'r') as json_file:
            for row in json_file:
                data = json.loads(row)
                json.dumps(data, sort_keys=True, indent=2, separators=(',', ': '))

    def bounding_box_volume(self, extent):
        volume = extent.x * extent.y * extent.z
        return volume


def main(args):
    obj_tracking = Object_Tracking()
    rospy.init_node('object_tracking', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down!")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

