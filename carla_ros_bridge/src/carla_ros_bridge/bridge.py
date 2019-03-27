#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Rosbridge class:
Class that handle communication between CARLA and ROS
"""

# ------------------------
#   IMPORTS
# ------------------------
import threading
import time
import rospy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from derived_object_msgs.msg import ObjectArray
from carla_ros_bridge.parent import Parent
from carla_ros_bridge.map import Map
import carla_ros_bridge.object_sensor as obj_sensor


class CarlaRosBridge(Parent):
    """
    Carla ROS Bridge Class
    """

    def __init__(self, carla_world, params):
        """
        Constructor for CarlaRosBridge Class
        :param carla_world: Carla World Object
        :type carla_world: carla.World
        :param params: dict of parameters, see settings.yaml
        :type params: dict
        """
        self.params = params
        super(CarlaRosBridge, self).__init__(carla_ID=0, carla_world=carla_world, frame_ID='/map')
        self.timestamp_last_run = 0.0
        self.ros_timestamp = rospy.Time()
        self.tf_to_publish = []
        self.msgs_to_publish = []
        self.actor_list = []
        # register callback to create/delete actors
        self.carla_world.on_tick(self._carla_update_child_actors)
        self.update_child_actors_lock = threading.Lock()
        # register callback to update actors
        self.carla_world.on_tick(self._carla_time_tick)
        self.update_lock = threading.Lock()
        self.publishers = {}
        self.publishers['clock'] = rospy.Publisher('clock', Clock, queue_size=10)
        self.publishers['tf'] = rospy.Publisher('tf', TFMessage, queue_size=100)
        if not self.get_param("challenge_mode"):
            self.publishers['/carla/objects'] = rospy.Publisher('/carla/objects', ObjectArray, queue_size=10)
            self.object_array = ObjectArray()
        self.map = Map(carla_world=self.carla_world, parent=self, topic='/map')

    def destroy(self):
        """
        Virtual Function used to destroy this object
        Locks the update mutex.
        Removes all publishers
        Finally forwards call to the super class
        :return:
        """
        self.update_child_actors_lock.acquire()
        self.update_lock.acquire()
        self.actor_list = []
        rospy.loginfo("Exiting Bridge")
        self.publishers.clear()
        super(CarlaRosBridge, self).destroy()

    def get_current_ros_time(self):
        """
        Override Function used to return the current ROS time
        :return: The latest received ROS time of the bridge
        :rtype: rospy.Time
        """
        return self.ros_timestamp

    def publish_ros_message(self, topic, msg):
        """
        Override Function used to publish ROS messages
        Store the message in a list (waiting for their publication) with their associated publisher.
        If required corresponding publishers are created automatically
        Message for /tf topics and /carla/objects are handled differently in order to publish all transforms,
        all objects must be in the same message
        :param topic: ROS topic to publish message
        :type topic: string
        :param msg: ROS message to be published
        :type msg: a valid ROS message type
        :return:
        """
        if topic == 'tf':
            if not self.get_param("challenge_mode"):
                # transforms are merged in the same message
                self.tf_to_publish.append(msg)
        # elif topic == '/carla/objects':
        #     # objects are collected in the same message
        #     self.object_array.objects.append(msg)
        else:
            if topic not in self.publishers:
                self.publishers[topic] = rospy.Publisher(topic, type(msg), queue_size=10)
            self.msgs_to_publish.append((self.publishers[topic], msg))

    def get_param(self, key, default=None):
        """
        Override Function used to query global parameters from the outside
        :param key: Key of the parameter
        :type key: string
        :param default: the default value of the parameter to return if key is not found
        :type default: string
        :return: the parameter string
        :rtype. string
        """
        return self.params.get(key, default)

    def topic_name(self):
        """
        Override Function used to get the topic name of this root entity
        The Topic root '/carla' is returned by this bridge class
        :return: the final topic name of this
        :rtype: string
        """
        return "/carla"

    def _carla_time_tick(self, carla_timestamp):
        """
        Private callback function registered at carla.World.on_tick() used to trigger cyclic updates
        After successfully locking the update mutex (only perform trylock to respect bridge processing time), the clock and the children are updated.
        Finally the ROS messages are collected and then published and sent out
        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if not rospy.is_shutdown():
            if self.update_lock.acquire(False):
                if self.timestamp_last_run < carla_timestamp.elapsed_seconds:
                    self.timestamp_last_run = carla_timestamp.elapsed_seconds
                    self._update_clock(carla_timestamp)
                    self.update()
                    self._prepare_msgs()
                    self.send_msgs()
                self.update_lock.release()

    def _carla_update_child_actors(self, _):
        """
        Private callback function required at carla.World.on_tick() to trigger cyclic updates of the actors
        After successful locking the mutex (only perform trylock to respect bridge processing time) the existing actors are updated.
        :param _:
        :return:
        """
        if not rospy.is_shutdown():
            if self.update_child_actors_lock.acquire(False):
                # cache actor_list once during this update-loop
                self.actor_list = self.carla_world.get_actors()
                self.update_child_actors()
                # actors are only created/deleted around once per second
                time.sleep(1)
                self.update_child_actors_lock.release()

    def _update_clock(self, carla_timestamp):
        """
        Private Function used to perform the update of the clock
        :param carla_timestamp: Current Carla Timestamp
        :type carla_timestamp: carla.TimeStamp
        :return:
        """
        self.ros_timestamp = rospy.Time.from_sec(carla_timestamp.elapsed_seconds)
        self.publish_ros_message('clock', Clock(self.ros_timestamp))

    def _prepare_msgs(self):
        """
        Private Function used to prepare tf and object message to be sent out
        :return:
        """
        if not self.get_param("challenge_mode"):
            tf_msg = TFMessage(self.tf_to_publish)
            self.msgs_to_publish.append((self.publishers['tf'], tf_msg))
            self.tf_to_publish = []
            self.publish_ros_message('/carla/objects', obj_sensor.get_filtered_objectarray(self, None))

        # self.object_array.header = self.get_msg_header()
        # self.msgs_to_publish.append((self.publishers['/carla/objects'], self.object_array))
        # self.object_array = ObjectArray()

    def send_msgs(self):
        """
        Function used to send the collected ROS messages out via the ROS publisher
        :return:
        """
        for publisher, msg in self.msgs_to_publish:
            try:
                publisher.publish(msg)
            except rospy.ROSSerializationException as error:
                rospy.logwarn("Failed to serialize message on publishing: {}".format(error))
            except Exception as error:  # pylint: disable=broad-except
                rospy.logwarn("Failed to publish ROS message: {}".format(error))
        self.msgs_to_publish = []

    def run(self):
        """
        Function used to run bridge module functionality.
        Registers on shutdown callback at rospy and spins ROS
        :return:
        """
        rospy.on_shutdown(self.on_shutdown)
        rospy.spin()

    def on_shutdown(self):
        """
        Function used to be called on shutdown
        This function is registered at rospy as shutdown handler.
        :return:
        """
        rospy.loginfo("Shutdown requested")
        self.destroy()

    def get_actor_list(self):
        """
        Override Function used to provide actor list
        :return: Actor List
        :rtype: List
        """
        return self.actor_list

    def get_filtered_objectarray(self, filtered_ID):
        """
        Function used to get ObjectArray of other available actors, except the one with the filtered ID
        :param filtered_ID: Filter Actor ID
        :return: ObjectArray of remaining actors
        :rtype: derived_object_msgs.ObjectArray
        """
        return obj_sensor.get_filtered_objectarray(self, filtered_ID)



