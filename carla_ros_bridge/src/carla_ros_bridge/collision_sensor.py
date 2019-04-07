#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Rosbridge class:
Class used to handle collision events
"""

# ------------------------
#   IMPORTS
# ------------------------
from sensor import Sensor
from carla_ros_bridge.msg import CarlaCollisionEvent


class CollisionSensor(Sensor):
    """
    Actor Implementation Details for Collision Sensor
    """

    def __init__(self, carla_actor, parent, topic_prefix=None, append_role_name_topic_postfix=True):
        """
        Constructor for CollisionSensor class
        :param carla_actor: Carla Actor Object
        :type carla_actor: carla.Actor
        :param parent: Parent of this Node
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: Topic Prefix to be used by this Actor
        :type topic_prefix: string
        :param append_role_name_topic_postfix: if this flag is set True, then the role_name of the actor is used as topic postfix.
        :type append_role_name_topic_postfix: boolean
        """
        super(CollisionSensor, self).__init__(carla_actor=carla_actor, parent=parent,
                                              topic_prefix="collision", append_role_name_topic_postfix=False)

    def sensor_data_updated(self, collision_event):
        """
        Function used to wrap up the Collision Event into a ROS message
        :param collision_event: Carla Collision Event Object
        :type collision_event: carla.CollisionEvent
        :return:
        """
        collision_msg = CarlaCollisionEvent()
        collision_msg.header = self.get_msg_header(use_parent_frame=False)
        collision_msg.other_actor_id = collision_event.other_actor_id
        collision_msg.normal_impulse.x = collision_event.normal_impulse.x
        collision_msg.normal_impulse.y = collision_event.normal_impulse.y
        collision_msg.normal_impulse.z = collision_event.normal_impulse.z
        # Publish ROS message
        self.publish_ros_message(self.topic_name(), collision_msg)


