#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Classes to handle Carla Pedestrians
"""

# ------------------------
#   IMPORTS
# ------------------------
import rospy
from std_msgs.msg import ColorRGBA
from derived_object_msgs.msg import Object
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from carla_ros_bridge.actor import Actor
import carla_ros_bridge.transforms as transforms


class Walker(Actor):
    """
    Actor Implementation Details For Walker Objects
    """

    @staticmethod
    def create_actor(carla_actor, parent):
        """
        Static Factory Method to Create Walker Actors
        :param carla_actor: Carla Walker Actor Object
        :type carla_actor: carla.Walker
        :param parent: Parent of the new traffic actor
        :type parent: carla_ros_bridge.Parent
        :return: The Created Vehicle Actor
        :rtype: carla_ros_bridge.Walker or derived type
        """
        return Walker(carla_actor=carla_actor, parent=parent)

    def __init__(self, carla_actor, parent, topic_prefix=None, append_role_name_topic_postfix=True):
        """
        Constructor for Walker Class
        :param carla_actor: Carla Walker Actor Object
        :type carla_actor: carla.Walker
        :param parent: Parent of this Walker Actor Object
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        :param append_role_name_topic_postfix: if this flag is set True,
            the role_name of the actor is used as topic postfix
        :type append_role_name_topic_postfix: boolean
        """
        if topic_prefix is None:
            topic_prefix = "walker/{:03}".format(Actor.global_id_registry.get_ID(carla_actor.id))

        super(Walker, self).__init__(carla_actor=carla_actor,
                                     parent=parent,
                                     topic_prefix=topic_prefix,
                                     append_role_name_topic_postfix=append_role_name_topic_postfix)

        # self.carla_actor = carla_actor
        self.classification = Object.CLASSIFICATION_UNKNOWN
        if carla_actor.attributes.has_key('object_type'):
            self.classification = Object.CLASSIFICATION_PEDESTRIAN
        self.classification_age = 0

    def destroy(self):
        """
        Override Function used to destroy this object
        Finally forwards call to super class
        :return:
        """
        rospy.logdebug("Destroy Walker(id={})".format(self.get_ID()))
        super(Walker, self).destroy()

    def update(self):
        """
        Override Function used to update this object
        On update walker send:
        - tf global frame
        - object message
        - marker message
        :return:
        """
        self.send_tf_msg()
        self.send_object_msg()
        self.send_marker_msg()
        super(Walker, self).update()

    def get_marker_color(self):
        """
        Override Function used to return the color for marker messages
        :return: Color used by a walker marker
        :rtype: std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 0
        color.g = 0
        color.b = 255
        return color

    def send_marker_msg(self):
        """
        Function to send marker messages of this walker.
        :return:
        """
        marker = self.get_marker(use_parent_frame=False)
        marker.type = Marker.CUBE
        marker.ns = str(self.classification)
        marker.text = str(self.classification)
        marker.pose = transforms.carla_location_to_pose(self.carla_actor.bounding_box.location)
        marker.scale.x = self.carla_actor.bounding_box.extent.x * 2.0
        marker.scale.y = self.carla_actor.bounding_box.extent.y * 2.0
        marker.scale.z = self.carla_actor.bounding_box.extent.z * 2.0
        self.publish_ros_message('/carla/pedestrian_marker', marker)

    def send_object_msg(self):
        """
        Function to send object messages of this walker.
        A derived_object_msgs.msg.Object is prepared to be published via '/carla/objects'
        :return:
        """
        walker_object = Object(header=self.get_msg_header())
        # ID
        walker_object.id = self.get_global_ID()
        # Pose
        walker_object.pose = self.get_current_ros_pose()
        # Twist
        walker_object.twist = self.get_current_ros_twist()
        # Acceleration
        walker_object.accel = self.get_current_ros_accel()
        # Shape
        walker_object.shape.type = SolidPrimitive.BOX
        walker_object.shape.dimensions.extend(
            [self.carla_actor.bounding_box.extent.x * 2.0,
             self.carla_actor.bounding_box.extent.y * 2.0,
             self.carla_actor.bounding_box.extent.z * 2.0]
        )
        # Classification if available in attributes
        if self.classification != Object.CLASSIFICATION_UNKNOWN:
            walker_object.object_classified = True
            walker_object.classification = self.classification
            walker_object.classification_certainty = 1.0
            self.classification_age += 1
            walker_object.classification_age = self.classification_age

        # self.publish_ros_message('/carla/objects', vehicle_object)
        return walker_object
