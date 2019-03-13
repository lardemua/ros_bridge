#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Base Classes to handle child objects
"""

# ------------------------
#   IMPORTS
# ------------------------
import rospy
from abc import abstractmethod
from geometry_msgs.msg import TransformStamped
from carla_ros_bridge.parent import Parent


class Child(Parent):
    """
    Genereic Base Class for all Child Entities
    """
    def __init__(self, carla_ID, carla_world, parent, topic_prefix=''):
        """
        Constructor for Child Class
        :param carla_ID: unique carla_id of this child object
            carla_ID > 0: carla actor ids (see also carla.Actor)
            carla_ID == 0: resevered for the (root) bridge object; not allowed in here
            carla_ID == -1: used by the map object
        :type carla_ID: int64
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: Parent object of this Child Node
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: Topic Prefix to be used for this child node
        :type topic_prefix: string
        """
        if carla_ID == 0:
            raise ValueError("A child node can never have an carla_id of zero"
                             "Zero is reserved for the parent root(the bridge object)")
        self.topic_prefix = topic_prefix.replace(".", "/").replace("-", "_")

        # Each child node defines its own frame
        super(Child, self).__init__(
            carla_ID = carla_ID, carla_world = carla_world, frame_ID = self.topic_prefix
        )
        self.parent = parent
        rospy.logdebug("Created {}-Child(id={}, parent_id={}, topic_name={})".format(
            self.__class__.__name__, self.get_ID(), self.get_parent_ID(), self.topic_name()))

    def destroy(self):
        """
        Override Function used to destroy this object
        Removes the reference to the carla_ros_bridge.Parent object
        Finally forward call to the super class
        :return:
        """
        rospy.logdebug("Destroying {}-Child(id={})".format(self.__class__.__name__, self.get_ID()))
        self.parent = None
        super(Child, self).destroy()

    def get_current_ros_time(self):
        """
        Override Function used to query current ROS time
        Just forwards the request made by the parent node
        :return: The latest recieved ROS time of the bridge
        :rtype: rospy.Time
        """
        return self.parent.get_current_ros_time()

    def publish_ros_message(self, topic, msg):
        """
        Override Function used to publish ROS messages
        Just forwards the child request to the parent node
        :param topic: ROS topic to publish ROS message
        :type topic: string
        :param msg: ROS message to be published
        :type msg: a valid ROS Message Type
        :return:
        """
        self.parent.publish_ros_message(topic, msg)

    def get_param(self, key, default=None):
        """
        Override Function used to get the topic name of the current entity
        Just forwards the child request to the parent node
        :param key: Key of the parameter
        :type key: string
        :param default: Default value of the parameter to return if key is not found
        :type default: string
        :return: Parameter String
        :rtype: string
        """
        return self.parent.get_param(key, default)

    def topic_name(self):
        """
        Override Function used to get the topic name of the current entity
        Concatenates the child's own topic prefix to the parent topic name if this is not empty.
        :return: The Final Topic Name of this node
        :rtype: string
        """
        if len(self.topic_prefix) > 0:
            return self.parent.topic_name() + "/" + self.topic_prefix
        else:
            return self.parent.topic_name()

    def get_parent_ID(self):
        """
        Getter Function for the carla_ID of the parent
        :return: Unique carla_ID of the parent of this child node
        :rtype: int64
        """
        return self.parent.get_ID()

    def get_msg_header(self, use_parent_frame=True): # pylint disable=arguments-differ
        """
        Helper Function used to create a ROS message header
        :param use_parent_frame:  per default the header.frame_ID is set to the frame of the child's parent.
                                  If this is set to False, the child's own frame is used as the basis.
        :type use_parent_frame: boolean
        :return: preffiled Header object
        :rtype: std_msgs.msg.Header
        """
        header = super(Child, self).get_msg_header()
        if use_parent_frame:
            header.frame_id = self.parent.get_frame_ID()
        return header

    def send_tf_msg(self):
        """
        Helper Function used to send a ROS tf message of this child
        Mainly calls the get_tf_msg() method and the publish_ros_message() method.
        :return:
        """
        tf_msg = self.get_tf_msg()
        self.publish_ros_message('tf', tf_msg)

    def get_tf_msg(self):
        """
        Helper Function used to create a ROS tf message for this child
        :return: The Filled Tf message
        :rtype: geometry_msgs.msg.TransformStamped
        """
        tf_msg = TransformStamped()
        tf_msg.header = self.get_msg_header()
        tf_msg.child_frame_id = self.get_frame_ID()
        tf_msg.transform = self.get_current_ros_transform()
        return tf_msg

    @abstractmethod
    def get_current_ros_transform(self):
        """
        Pure Virtual Function to query the current ROS transform
        :return: The ROS Transform of this entity
        :rtype: geometry_msgs.msg.Transform
        """
        raise NotImplementedError("This function has to be implemented by the derived classes")
