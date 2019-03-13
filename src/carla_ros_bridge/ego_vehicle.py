#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Classes to handle Carla vehicles
"""

# ------------------------
#   IMPORTS
# ------------------------
import sys
import datetime
import numpy
from simple_pid import PID
import rospy
from dynamic_reconfigure.server import Server
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from ackermann_msgs.msg import AckermannDrive
from carla import VehicleControl
from carla_ros_bridge.vehicle import Vehicle
import carla_ros_bridge.physics as phys
from carla_ros_bridge.msg import CarlaVehicleControl                # pylint: disable=no-name-in-module,import-error
from carla_ros_bridge.msg import EgoVehicleControlInfo              # pylint: disable=no-name-in-module,import-error
from carla_ros_bridge.cfg import EgoVehicleControlParameterConfig   # pylint: disable=no-name-in-module,import-error

