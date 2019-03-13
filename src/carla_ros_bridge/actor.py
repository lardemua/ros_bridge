#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Base Classes to Handle Actor Objects
"""

# ------------------------
#   IMPORTS
# ------------------------

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from carla_ros_bridge.child import Child
from carla_ros_bridge.actor_id_registry import ActorIdRegisty
from carla_ros_bridge.transfroms as trans

