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
from carla_ros_bridge.actor_id_registry import ActorIDRegistry
import carla_ros_bridge.transforms as trans

class Actor(Child):
    """
    Generic Base Class for all carla Actors
    """
    global_id_registry = ActorIDRegistry()

    def __init__(self, carla_actor, parent, topic_prefix='', append_role_name_topic_postfix=True):
        """
        Constructor for Actor Class Object
        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        :param parent: parent of carla vehicle actor object
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: topic prefix to be used for this actor
        :type topic_prefix: string
        :param append_role_name_topic_postfix: if this flag is set True, the role_name of the actor is used as a topic postfix.
        :type append_role_name_topic_postfix: boolean
        """

        # Each actor defines its own frame
        if append_role_name_topic_postfix:
            if carla_actor.attributes.has_key('role_name'):
                topic_prefix += '/' + carla_actor.attributes['role_name']
            else:
                topic_prefix += '/' + \
                                str(Actor.global_id_registry.get_id(carla_actor.id))

