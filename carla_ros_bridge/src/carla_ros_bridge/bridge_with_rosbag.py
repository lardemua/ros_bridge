#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
RosBridge class with rosbag support
"""

# ------------------------
#   IMPORTS
# ------------------------
import os
import rospy
import rosbag
from bridge import CarlaRosBridge


class CarlaRosBridgeWithBag(CarlaRosBridge):
    """
    Carla ROS bridge class with ROS bag writing functionality
    """

    def __init__(self, carla_world, params):
        """
        Constructor for CarlaRosBridgeWithBag class
        :param carla_world: Carla World Object
        :type carla_world: carla.World
        :param params: dict of parameters, see settings.yaml
        :type params: dict
        """
        super(CarlaRosBridgeWithBag, self).__init__(carla_world=carla_world, params=params)
        prefix, dummy_ext = os.path.splitext(self.get_param('rosbag_fname'))
        rosbag_fname = os.path.abspath(prefix + self.get_param('curr_episode'))
        self.bag = rosbag.Bag(rosbag_fname, mode="w")

    def destroy(self):
        """
        Virtual Function used to destroy this object
        Closes the ROS bag file
        Finally forwards call to the super class
        :return:
        """
        rospy.loginfo("Closing the bag file")
        self.bag.close()
        super(CarlaRosBridgeWithBag, self).destroy()

    def send_msgs(self):
        """
        Overriden Function used to write the collected ROS messages out into the ROSBAG before sending them via ROS publisher
        :return:
        """
        for publisher, msg in self.msgs_to_publish:
            self.bag.write(publisher.name, msg, self.get_current_ros_time())
        super(CarlaRosBridgeWithBag, self).send_msgs()
