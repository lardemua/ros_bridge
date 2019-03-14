#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Parent factory class for carla.Actor lifecycle handling
"""

# ------------------------
#   IMPORTS
# ------------------------
import rospy
from abc import abstractmethod
from std_msgs.msg import Header


class Parent(object):
    """
    Factory(Parent) class to create actors(children) and manage lifecycle of the children objects
    """

    def __init__(self, carla_ID, carla_world, frame_ID):
        """
        Constructor for Parent Class

        :param carla_ID:  Unique carla_ID of this parent object
               carla_ID > 0   => carla actor IDs (see carla.Actor>
               carla_ID == 0  => used by the root bridge object
               carla_ID == -1 => used by the map object
        :type carla_ID: int64
        :param carla_world: Carla World Object
        :type carla_world: carla.World
        :param frame_ID: ROS tf framed ID of this object
        :type frame_ID: string
        """
        self.carla_ID = carla_ID
        self.carla_world = carla_world
        self.frame_ID = frame_ID
        self.child_actors = {}

    def destroy(self):
        """
        Virtual Function used to destroy the object
        Recursively the reference to the carla.World object.
        Remove the reference to the carla.World object.
        Finally remove the references to the children object.
        :return:
        """
        for dummy_actor_id in self.child_actors.iteritems():
            actor.destroy()
        self.child_actors.clear()
        self.carla_world = None

    def get_frame_ID(self):
        """
        Getter function for the frame ID of current object
        :return: ROS tf frame ID of the object
        :rtype: string
        """
        return self.frame_ID

    def get_ID(self):
        """
        Getter function for the carla_ID of parent object
        :return: Unique carla_ID of the parent object
        :rtype: int64
        """
        return self.carla_ID

    def get_carla_world(self):
        """
        Getter function for the carla world object ID of the current object
        :return: The carla world
        :rtype: carla.World
        """
        return self.carla_world

    def _create_new_children(self):
        """
        Private function to create the actors in the carla world which are the children actors of this parent
        :return:
        """
        for actor in self.carla_world.get_actors():
            if ((actor.parent and actor.parent.id == self.carla_id) or (actor.parent is None and self.carla_id == 0)):
                if actor.id not in self.child_actors:
                    if actor.type_id.startswith('traffic'):
                        self.child_actors[actor.id] = Traffic.create_actor(carla_actor=actor, parent=self)
                    elif actor.type_id.startswith("vehicle"):
                        self.child_actors[actor.id] = Vehicle.create_actor(carla_actor=actor, parent=self)
                    elif actor.type_id.startswith("sensor"):
                        self.child_actors[actor.id] = Sensor.create_actor(carla_actor=actor, parent=self)
                    elif actor.type_id.startswith("spectator"):
                        self.child_actors[actor.id] = Spectator(carla_actor=actor, parent=self)
                    else:
                        self.child_actors[actor.id] = Actor(carla_actor=actor, parent=self)

    def _destroy_dead_children(self):
        """
        Private function to detect and remove non existing children actors
        :return:
        """
        actors_to_delete = []
        for child_actor_id, child_actor in self.child_actors.iteritems():
            if not child_actor.carla_actor.is_alive:
                rospy.loginfo("Detected Non Alive Child Actor(id={})".format(child_actor_id))
                actors_to_delete.append(child_actor_id)
            else:
                found_actor = False
                for actor in self.carla_world.get_actors():
                    if actor.id == child_actor_id:
                        found_actor = True
                        break
                if not found_actor:
                    rospy.loginfo("Detected Non Alive Child Actor(id={})".format(child_actor_id))
                    actors_to_delete.append(child_actor_id)

        for actor_id in actors_to_delete:
            self.child_actors[actor_id].destroy()
            del self.child_actors[actor_id]


    def update(self):
        """
        Virtual (non abstract) function to update parent object.
        Override this function if the derived class has to perform some additional update tasks. But don't forget
        to forward the update call to the super class, ensuring that this concrete function is called.
        The update part of the parent class consists of updating the children of this by:
        -> Create new child actors
        -> Destroy dead children
        -> Update the existing children
        :return:
        """
        self._create_new_children()
        self._destroy_dead_children()
        for dummy_actor_id, actor in self.child_actors.iteritems():
            actor.update()


    def get_msg_header(self):
        """
        Helper function to create a ROS Message Header
        :return: prefilled Header object
        """
        header = Header()
        header.stamp = self.get_current_ros_time()
        header.frame_id = self.get_frame_ID()
        return header

    @abstractmethod
    def get_current_ros_time(self):
        """
        Pure Virtual Function used to query the current ROS time from the carla_ros_bridge.CarlaRosBridge parent root.
        This method is intended to be implemented by the directly derived classes:
                    carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge.
        Be aware: Its not intended that classes further down in the class hierarchy override this!
        :return: The latest received ROS time of the bridge
        :rtype: rospy.Time
        """
        raise NotImplementedError(
            "This function is re-implemented by"
            "carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge"
            "If this error becomes visible the class hierarchy is somehow broken")

    @abstractmethod
    def publish_ros_message(self, topic, msg):
        """
        Pure Virtual Function used to publish ROS messages via the carla_ros_bridge.CarlaRosBridge parent root.
        This method is intended to be implemented by the directly derived classes:
                    carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge.
        Be aware: Its not intended that classes further down in the class hierarchy override this!
        :param topic: ROS topic to publish ROS message on
        :type topic: string
        :param msg: ROS message to be published
        :type msg: a valid ROS message type.
        :return:
        """
        raise NotImplementedError(
            "This function is re-implemented by"
            "carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge"
            "If this error becomes visible the class hierarchy is somehow broken")

    @abstractmethod
    def get_param(self, key, default=None):
        """
        Pure Virtual Function used to query global parameters passed from the outside.
        This method is intended to be implemented by the directly derived classes:
                    carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge.
        Be aware: Its not intended that classes further down in the class hierarchy override this!
        :param key: Key of the parameter
        :type key: string
        :param default: Default value of the parameter to return if key is not found
        :type default: string
        :return: the parameter string
        :rtype: string
        """
        raise NotImplementedError(
            "This function is re-implemented by"
            "carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge"
            "If this error becomes visible the class hierarchy is somehow broken")

    @abstractmethod
    def topic_name(self):
        """
        Pure Virtual Function used to get the topic name of the current entity.
        This method is intended to be implemented by the directly derived classes:
                    carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge.
        Be aware: Its not intended that classes further down in the class hierarchy override this!
        :return: the final topic name of this
        :rtype: string
        """
        raise NotImplementedError(
            "This function is re-implemented by"
            "carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge"
            "If this error becomes visible the class hierarchy is somehow broken")

# These import have to be added at the end of the file to resolve cyclic dependency
from carla_ros_bridge.actor import Actor                     # noqa, pylint: disable=wrong-import-position
from carla_ros_bridge.spectator import Spectator             # noqa, pylint: disable=wrong-import-position
from carla_ros_bridge.sensor import Sensor                   # noqa, pylint: disable=wrong-import-position
from carla_ros_bridge.traffic import Traffic                 # noqa, pylint: disable=wrong-import-position
from carla_ros_bridge.vehicle import Vehicle                 # noqa, pylint: disable=wrong-import-position
