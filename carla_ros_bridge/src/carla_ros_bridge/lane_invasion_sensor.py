#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Rosbridge class:
Class used to handle lane invasion events
"""

# ------------------------
#   IMPORTS
# ------------------------
from carla_ros_bridge.sensor import Sensor
from carla_ros_bridge_msgs.msg import CarlaLaneInvasionEvent     # pylint: disable=no-name-in-module,import-error


class LaneInvasionSensor(Sensor):
    """
    Actor Implementation Details for a Lane Invasion Sensor
    """

    def __init__(self, carla_actor, parent, topic_prefix=None, append_role_name_topic_postfix=True):
        """
               Constructor for LaneInvasionSensor class

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
        super(LaneInvasionSensor, self).__init__(carla_actor=carla_actor, parent=parent,
                                                 topic_prefix="lane_invasion",
                                                 append_role_name_topic_postfix=False)

    def sensor_data_updated(self, lane_invasion_event):
        """
        Function used to wrap up the lane invasion event into a ROS Message
        :param lane_invasion_event: Carla Lane Invasion Event Object
        :type lane_invasion_event: carla.LaneInvasionEvent
        :return:
        """
        lane_invasion_msg = CarlaLaneInvasionEvent()
        lane_invasion_msg.header = self.get_msg_header(use_parent_frame=False)
        for marking in lane_invasion_event.crossed_lane_markings:
            lane_invasion_msg.crossed_lane_markings.append(marking)
        # Publish ROS Message
        self.publish_ros_message(self.topic_name(), lane_invasion_msg)
