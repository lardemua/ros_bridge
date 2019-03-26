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
    Generic Base Class for all Carla Actors
    """

    global_id_registry = ActorIDRegistry()

    def __init__(self, carla_actor, parent, topic_prefix='', append_role_name_topic_postfix=True):
        """
        Constructor for class Actor
        :param carla_actor: Carla Vehicle Actor Object
        :type carla_actor: carla.Vehicle
        :param parent: Parent of this Actor Node
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: Topic Prefix to be used for this Actor
        :type topic_prefix: string
        :param append_role_name_topic_postfix: If this flag is set True, then the role_name of the actor is used as topic postfix
        :type apped_role_name_topic_postfix: boolean
        """

        # each actor defines its own frame
        if append_role_name_topic_postfix:
            if carla_actor.attributes.has_key('role_name'):
                topic_prefix += '/' + carla_actor.attributes['role_name']
            else:
                topic_prefix += '/' + \
                    str(Actor.global_id_registry.get_ID(carla_actor.id))
        super(Actor, self).__init__(carla_ID=carla_actor.id, carla_world=carla_actor.get_world(),
                                    parent=parent, topic_prefix=topic_prefix)
        self.carla_actor = carla_actor
        rospy.logdebug("Created Actor-{}(id={}, parent_id={},"
                       " type={}, topic_name={}, attributes={}".format(
                           self.__class__.__name__, self.get_ID(),
                           self.get_parent_ID(), self.carla_actor.type_id,
                           self.topic_name(), self.carla_actor.attributes))

        if self.__class__.__name__ == "Actor":
            rospy.logwarn("Created Unsupported Actor(id={}, parent_id={},"
                          " type={}, attributes={}".format(
                              self.get_ID(), self.get_parent_ID(),
                              self.carla_actor.type_id, self.carla_actor.attributes))

    def destroy(self):
        """
        Override Function used to destroy this object
        Removes the reference to the carla.Actor object
        Finally forward call to super class.
        :return:
        """
        rospy.logdebug("Destroying {}-Actor(id={})".format(self.__class__.__name__, self.get_ID()))
        self.carla_actor = None
        super(Actor, self).destroy()

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Virtual (non-abstract) function to get the ROS std_msgs.msg.ColorRGBA used for rviz objects of this Actor
        Reimplement this in the derived actor class if ROS visualization messages
        (e.g. visualization_msgs.msg.Marker) are sent out and you want a different color than blue.
        :return: blue color object
        :rtype: std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 0
        color.g = 0
        color.b = 255
        return color

    def get_marker(self, use_parent_frame=True):
        """
        Helper function to create a ROS visualization_msgs.msg.Marker for the Actor
        :param use_parent_frame: per default (True) the header.frame-id is set to the frame of the actor's parent.
                                 If this is set to False, the actor's own frame is used as basis.
        :type use_parent_frame: boolean
        :return: visualization-msgs.msg.Marker
        """
        marker = Marker(header=self.get_msg_header(use_parent_frame=use_parent_frame))
        marker.color = self.get_marker_color()
        marker.color.a = 0.3
        marker.id = self.get_global_ID()
        marker.text = "id = {}".format(marker.id)
        return marker

    def get_current_ros_transform(self):
        """
        Override Function used to provide the current ROS transform
        :return: the ROS transfrom of this Actor
        :rtype: geometry-msgs.msg.Transform
        """
        return trans.carla_transform_to_ros_transform(self.carla_actor.get_transform())

    def get_current_ros_pose(self):
        """
        Function used to provide the current ROS pose
        :return: the ROS pose of this Actor
        :rtype: geometry-msgs.msg.Pose
        """
        return trans.carla_transform_to_ros_pose(self.carla_actor.get_transform())

    def get_current_ros_twist(self):
        """
        Function used to provide the current ROS twist
        :return: the ROS twist of this actor
        :rtype: geometry-msgs.msg.Twist
        """
        return trans.carla_velocity_to_ros_twist(self.carla_actor.get_velocity())

    def get_current_ros_accel(self):
        """
        Function used to provide the current ROS acceleration
        :return: the ROS acceleration of this Actor
        :rtype: geometry_msgs.msg.Accel
        """
        return trans.carla_acceleration_to_ros_accel(self.carla_actor.get_acceleration())

    def get_global_ID(self):
        """
        Returns a unique global ID for the actor to be used for markers, objects IDs, etc...
        ROS Marker ID should be int32
        carla/unrealengine seems to use int64
        A lookup table is used to remap actor_id to small number between 0 and max_int32
        :return: mapped id of this actor (unique increasing counter value)
        :rtype: uint32
        """
        return Actor.global_id_registry.get_ID(self.carla_actor.id)
