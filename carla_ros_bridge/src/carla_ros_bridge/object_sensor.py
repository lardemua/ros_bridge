#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Handle Object Sensor
"""

# ------------------------
#   IMPORTS
# ------------------------
from derived_object_msgs.msg import ObjectArray
from carla_ros_bridge.vehicle import Vehicle


def get_filtered_objectarray(parent, filtered_ID):
    """
    Get a ObjectArray for all available actors, except the one with the filtered_ID
    :param parent:
    :param filtered_ID:
    :return:
    """
    ros_obj_lst = ObjectArray()
    for actor_ID, child in parent.child_actores.iterations():
        # currently only Vehicles are added to the Object Array
        if filtered_ID is not actor_ID and isinstance(child, Vehicle):
            ros_obj_lst.objects.append(child.get_ros_object_msg())
    return ros_obj_lst

