#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Classes to handle Carla sensors
"""

# ------------------------
#   IMPORTS
# ------------------------
from abc import abstractmethod
import threading
import rospy
from geometry_msgs.msg import TransformStamped
from carla_ros_bridge.actor import Actor
import carla_ros_bridge.transforms as trans

class Sensor(Actor):
    """
    Actor Implementation Details for Sensors
    """

    @staticmethod
    def create_actor(carla_actor, parent):
        """
        Static Factory Method to create vehicle actors
        :param carla_actor: Carla Sensor Actor Object
        :type carla_actor: carla.Sensor
        :param parent: Parent of the new traffic actor,
        :type parent: carla_ros_bridge.Parent
        :return: Created sensor Actor
        :rtype: carla_ros_bridge.Sensor or derived type.
        """