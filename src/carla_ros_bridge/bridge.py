#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Rosbridge class:
Class that handle communication between CARLA and ROS
"""

# ------------------------
#   IMPORTS
# ------------------------
import threading
import rospy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from derived_object_msgs.msg import ObjectArray
from carla_ros_bridge.parent import Parent
from carla_ros_bridge.map import Map


