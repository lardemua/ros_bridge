#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Class to handle Carla camera sensors
"""

# ------------------------
#   IMPORTS
# ------------------------
from abc import abstractmethod
import math
import numpy
import rospy
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
import carla
from carla_ros_bridge.sensor import Sensor
import carla_ros_bridge.transforms as trans