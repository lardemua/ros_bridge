#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Base Class used for spawning a Ego Vehicle in ROS

Two modes are available:
- spawn at random Carla Spawnpoint
- spawn at the pose read from ROS topic /initialpose

Whenever a pose is received via /initialpose, the vehicle gets respawned at that
position. If no /initialpose is set at startup, a random spawnpoint is used.

/initialpose might be published via RVIZ '2D Pose Estimate" button.
"""

# ==============================================================================
# -- IMPORTS -------------------------------------------------------------------
# ==============================================================================
from abc import abstractmethod
import sys
import glob
import os
import random
import math
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
# ==============================================================================
# -- Find CARLA Module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla    # pylint: disable=wrong-import-position


# ==============================================================================
# -- CarlaEgoVehicleBase -------------------------------------------------------
# ==============================================================================
class CarlaEgoVehicleBase(object):
    """
    Class used to handle the spawning of the Ego Vehicle and its sensors.
    Derive from this class and implement method sensors()
    """
    def __init__(self):
        """
        Constructor for CarlaEgoVehicleBase
        """
        rospy.init_node('ego_vehicle')
        self.host = rospy.get_param('/carla/host', '127.0.0.1')
        self.port = rospy.get_param('/carla/port', '2000')
        self.world = None
        self.player = None
        self.sensor_actors = []
        self.actor_filter = rospy.get_param('/carla/client/vehicle_filter', 'vehicle.*')
        self.actor_spawnpoint = None
        self.initialpose_subscriber = rospy.Subscriber("/initalpose", PoseWithCovarianceStamped, self.on_initialpose)
        rospy.loginfo('Listening to Server %s:%s', self.host, self.port)
        rospy.loginfo('Using Vehicle Filter: %s', self.actor_filter)

    def on_initialpose(self, initial_pose):
        """
        Callback function for /initialpose ROS topic.
        Receiving an initial pose (e.g. from RVIZ '2D Pose estimate') triggers a respawn.
        :param initial_pose: /initialpose ROS topic.
        :return:
        """
        self.actor_spawnpoint = initial_pose.pose.pose
        self.restart()

    def restart(self):
        """
        Function used to (Re)spawn the Vehicle.
        Either at a given actor_spawnpoint or at a random CARLA spawnpoint.
        :return:
        """
        # get the vehicle blueprint
        blueprint = random.choice(self.world.get_blueprint_library().filter(self.actor_filter))
        blueprint.set_attribute('role_name', 'ego_vehicle')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # spawn the vehicle
        if self.actor_spawnpoint:
            spawn_point = carla.Transform()
            spawn_point.location.x = self.actor_spawnpoint.position.x
            spawn_point.location.y = -self.actor_spawnpoint.position.y
            spawn_point.location.z = self.actor_spawnpoint.position.z + 2   # spawn vehicle 2m above the ground
            quaternion = (self.actor_spawnpoint.orientation.x,
                          self.actor_spawnpoint.orientation.y,
                          self.actor_spawnpoint.orientation.z,
                          self.actor_spawnpoint.orientation.w)
            _, _, yaw = euler_from_quaternion(quaternion)
            spawn_point.rotation.yaw = -math.degrees(yaw)
            rospy.loginfo("Spawn at x={} y={} z={} yaw={}".format(spawn_point.location.x,
                                                                  spawn_point.location.y,
                                                                  spawn_point.location.z,
                                                                  spawn_point.rotation.yaw))
            if self.player is None:
                self.destroy()
            while self.player is None:
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        else:
            if self.player is not None:
                spawn_point = self.player.get_transform()
                spawn_point.location.z += 2.0
                spawn_point.rotation.roll = 0.0
                spawn_point.rotation.pitch = 0.0
                self.destroy()
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            while self.player is None:
                spawn_points = self.world.get_map().get_spawn_points()
                spawn_point = random.choice(spawn_points) if spawn_points else carla.Transforms()
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # setup the vehicle sensors
        self.sensor_actors = self.setup_sensors(self.sensors())

    def setup_sensors(self, sensors):
        """
        Function used to create sensors defined by the user and attach them to the Ego-Vehicle
        :param sensors: list of sensors
        :return:
        """
        actors = []
        blueprint_library = self.world.get_blueprint_library()
        for sensor_spec in sensors:
            try:
                blueprint = blueprint_library.find(sensor_spec['type'])
                blueprint.set_attribute('role_name', str(sensor_spec['role_name']))
                if sensor_spec['type'].startswith('sensor.camera'):
                    blueprint.set_attribute('image_size_x', str(sensor_spec['width']))
                    blueprint.set_attribute('image_size_y', str(sensor_spec['height']))
                    blueprint.set_attribute('fov', str(sensor_spec['fov']))
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'], z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'], roll=sensor_spec['roll'], yaw=sensor_spec['yaw'])
                elif sensor_spec['type'].startswith('sensor.lidar'):
                    blueprint.set_attribute('range', '200')
                    blueprint.set_attribute('rotation_frequency', '10')
                    blueprint.set_attribute('channels', '32')
                    blueprint.set_attribute('upper_fov', '15')
                    blueprint.set_attribute('lower_fov', '-30')
                    blueprint.set_attribute('points_per_second', '500000')
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'], z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'], roll=sensor_spec['roll'], yaw=sensor_spec['yaw'])
                elif sensor_spec['type'].startswith('sensor.other.gnss'):
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'], z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation()
            except KeyError as e:
                rospy.logfatal("Sensor will not be spawned, because sensor spec is invalid: '{}'".format(e))
                continue

            # create sensor based on its location and rotation
            sensor_transform = carla.Transform(sensor_location, sensor_rotation)
            sensor = self.world.spawn_actor(blueprint, sensor_transform, attach_to=self.player)
            actors.append(sensor)
        return actors

    @abstractmethod
    def sensors(self):
        """
        Function used to return the list of sensors attached to each actor
        :return:
        """
        return []

    def destroy(self):
        """
        Function used to destroy current Ego Vehicle and its sensors
        :return:
        """
        for i, _ in enumerate(self.sensor_actors):
            if self.sensor_actors[i] is not None:
                self.sensor_actors[i].destroy()
                self.sensor_actors[i] = None
        self.sensor_actors = []
        if self.player and self.player.is_alive:
            self.player.destroy()
        self.player = None

    def run(self):
        """
        Main Loop Function
        :return:
        """
        client = carla.Client(self.host, self.port)
        client.set_timeout(2.0)
        self.world = client.get_world()
        self.restart()
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
