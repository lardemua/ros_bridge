#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Class to handle the carla map
"""

# ------------------------
#   IMPORTS
# ------------------------
import rospy
from geometry_msgs.msg import Transform
from std_msgs.msg import String
from carla_ros_bridge.child import Child
from carla_ros_bridge_msgs.msg import CarlaMapInfo


class Map(Child):
    """
    Child Implementation Details for the Map
    """

    def __init__(self, carla_world, parent, topic):
        """
        Constructor for Class Map
        :param carla_world: Carla World Object
        :type carla_world: carla.World
        :param parent: Parent of Child Node
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: The topic prefix to be used for this child
        :type topic_prefix: string
        """
        super(Map, self).__init__(carla_ID=-1, carla_world=carla_world,
                                  parent=parent, topic_prefix=topic)
        self.carla_map = self.get_carla_world().get_map()

        self.open_drive_publisher = rospy.Publisher('/carla/map', CarlaMapInfo, queue_size=1, latch=True)
        open_drive_msg = CarlaMapInfo(header=self.get_msg_header())
        open_drive_msg.map_name = self.carla_map.name
        open_drive_msg.opendrive = self.carla_map.to_opendrive()
        self.open_drive_publisher.publish(open_drive_msg)

    def destroy(self):
        """
        Override Function used to destroy this object
        Removes the reference to carla.Map object.
        Finally forwards the call to the super class
        :return:
        """
        rospy.logdebug("Destroying Map()")
        self.carla_map = None
        self.open_drive_publisher = None
        super(Map, self).destroy()

    def update(self):
        """
        Override Function used to update this object
        On Update the Map sends: tf global frame message
        :return:
        """
        self.send_tf_msg()
        super(Map).update()

    def get_current_ros_transform(self):
        """
        Override function used to return the current ROS transform of the object
        The global map frame has an empty transform.
        :return:
        """
        return Transform()

    def send_tf_msg(self):
        """
        Override function used to send tf messages of the map
        The camera defines the global frame and this frame has to be set with frame_ID = 1
        :return:
        """
        tf_msg = self.get_tf_msg()
        tf_msg.header.frame_id = 1
        self.parent.publish_ros_message('tf', tf_msg)
