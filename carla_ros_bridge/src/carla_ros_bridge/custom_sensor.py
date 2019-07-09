#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Class used to handle Carla Custom Events
"""

# ------------------------
#   IMPORTS
# ------------------------
import numpy
import tf
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from carla_ros_bridge.sensor import Sensor
import carla_ros_bridge.transforms as trans


class CustomSensor(Sensor):
    """
    Actor Implementation Details for Custom Lidar Sensor
    """

    def __init__(self, carla_actor, parent, topic_prefix=None, append_role_name_topic_postfix=True):
        """
                Constructor for CustomSensor Class
                :param carla_actor: carla actor object
                :type carla_actor: carla.Actor
                :param parent: the parent of this
                :type parent: carla_ros_bridge.Parent
                :param topic_prefix: the topic prefix to be used for this actor
                :type topic_prefix: string
                :param append_role_name_topic_postfix: if this flag is set True,
                    the role_name of the actor is used as topic postfix
                :type append_role_name_topic_postfix: boolean
        """
        if topic_prefix is None:
            topic_prefix = 'custom'
        super(CustomSensor, self).__init__(carla_actor=carla_actor, parent=parent,
                                           topic_prefix=topic_prefix, append_role_name_topic_postfix=append_role_name_topic_postfix)
        self.parent = parent

    def get_tf_msg(self):
        """
        Override Function used to modify the tf messages sent by this lidar.
        The LIDAR transformation has to be altered:
        For some reason the LIDAR already sends a rotated cloud, so herein, we need to ignore pitch and roll
        :return: the filled tf message.
        :rtype: geometry_msgs.msg.TransformStamped
        """
        tf_msg = super(CustomSensor, self).get_tf_msg()
        rotation = tf_msg.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        dummy_roll, dummy_pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        # set roll and pitch to zero
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        tf_msg.transform.rotation = trans.numpy_quaternion_to_ros_quaternion(quat)
        return tf_msg

    def sensor_data_updated(self, carla_custom_event):
        """
        Function used to transform a received custom event into a ROS NavSatFix message
        :param carla_custom_event: Carla Custom Event Object
        :type carla_custom_event: carla.CustomEvent
        :return:
        """

        # Publish custom Event message to sensor event serializer here!


