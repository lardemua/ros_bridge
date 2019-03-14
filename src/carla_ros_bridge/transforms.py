#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Tool functions to convert transforms from carla to ROS coordinate system
"""

# ------------------------
#   IMPORTS
# ------------------------
import math
import numpy
import tf
from geometry_msgs.msg import Vector3, Quaternion, Transform, Pose, Point, Twist, Accel


def carla_location_to_numpy_vector(carla_location):
    """
        Convert carla location to a ROS Vector3 object
        Considers the conversion from the left-handed system (unreal) to the right-handed system (ROS).
        :param carla_location: The Carla Location
        :type carla_location: carla.Location
        :return: a numpy.array with 3 elements(Vector3).
        :rtype: numpy.array
    """
    return numpy.array([carla_location.x, -carla_location.y, carla_location.z])


def carla_location_to_ros_vector3(carla_location):
    """
        Convert carla location to a ROS Vector3 object
        Considers the conversion from the left-handed system (unreal) to the right-handed system (ROS).
        :param carla_location: The Carla Location
        :type carla_location: carla.Location
        :return: a ROS Vector3
        :rtype: geometry_msgs.msg.Vector3
    """
    ros_vector = Vector3()
    ros_vector.x = carla_location.x
    ros_vector.y = carla_location.y
    ros_vector.z = carla_location.z
    return ros_vector


def carla_location_to_ros_point(carla_location):
    """
       Convert carla location to a ROS Point object
       Considers the conversion from the left-handed system (unreal) to the right-handed system (ROS).
       :param carla_location: The Carla Location
       :type carla_location: carla.Location
       :return: a ROS Point
       :rtype: geometry_msgs.msg.Point
    """
    ros_point = Point()
    ros_point.x = carla_location.x
    ros_point.y = carla_location.y
    ros_point.z = carla_location.z
    return ros_point


def numpy_quaternion_to_ros_quaternion(numpy_quaternion):
    """
        Convert a quaternion from transforms to a ROS msg quaternion
        :param numpy_quaternion: a numpy quaternion
        :type numpy_quaternion: numpy.array
        :return: a ROS quaternion
        :rtype: geometry_msgs.msg.Quaternion
    """
    ros_quaternion = Quaternion()
    ros_quaternion.x = numpy_quaternion[0]
    ros_quaternion.y = numpy_quaternion[1]
    ros_quaternion.z = numpy_quaternion[2]
    ros_quaternion.w = numpy_quaternion[3]
    return ros_quaternion


def carla_rotation_to_RPY(carla_rotation):
    """
        Convert a carla rotation to a roll, pitch, yaw tuple
        Considers the conversion from the left-handed system (unreal) to the right-handed system (ROS).
        Considers the conversion from the degrees system (carla) to the radians system (ROS).
        :param carla_rotation: The Carla Rotation
        :type carla_rotation: carla.Rotation
        :return: a tuple with 3 elements(roll, pitch, yaw)
        :rtype: tuple
    """
    roll = -math.radians(carla_rotation.roll)
    pitch = -math.radians(carla_rotation.pitch)
    yaw = -math.radians(carla_rotation.yaw)
    return roll, pitch, yaw


def carla_rotation_to_numpy_quaternion(carla_rotation):
    """
        Convert a carla rotation to a numpy quaternion
        Considers the conversion from the left-handed system (unreal) to the right-handed system (ROS).
        Considers the conversion from the degrees system (carla) to the radians system (ROS).
        :param carla_rotation: The Carla Rotation
        :type carla_rotation: carla.Rotation
        :return: a numpy.array with 4 elements (Quaternion)
        :rtype: numpy.array
    """
    roll, pitch, yaw, = carla_rotation_to_RPY(carla_rotation)
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return quat


def carla_rotation_to_ros_quaternion(carla_rotation):
    """
        Convert a carla rotation to a ROS quaternion
        Considers the conversion from the left-handed system (unreal) to the right-handed system (ROS).
        Considers the conversion from the degrees system (carla) to the radians system (ROS).
        :param carla_rotation: The Carla Rotation
        :type carla_rotation: carla.Rotation
        :return: a ROS Quaternion
        :rtype: geometry_msgs.msg.Quaternion
    """
    quat = carla_rotation_to_numpy_quaternion(carla_rotation)
    ros_quat = numpy_quaternion_to_ros_quaternion(quat)
    return ros_quat


def carla_rotation_to_numpy_rotation_matrix(carla_rotation):
    """
        Convert a carla rotation to a numpy rotation matrix 3x3
        Considers the conversion from the left-handed system (unreal) to the right-handed system (ROS).
        Considers the conversion from the degrees system (carla) to the radians system (ROS).
        :param carla_rotation: The Carla Rotation
        :type carla_rotation: carla.Rotation
        :return: a numpy.array with 3x3 elements
        :rtype: numpy.array
    """
    roll, pitch, yaw = carla_rotation_to_RPY(carla_rotation)
    numpy_array = tf.transformations.euler_matrix(roll, pitch, yaw)
    rotation_matrix = numpy_array[:3, :3]
    return rotation_matrix


def carla_rotation_to_directional_numpy_vector(carla_rotation):
    """
        Convert a carla rotation (as orientation) into a numpy directional vector
        Considers the conversion from the left-handed system (unreal) to the right-handed system (ROS).
        Considers the conversion from the degrees system (carla) to the radians system (ROS).
        :param carla_rotation: The Carla Rotation
        :type carla_rotation: carla.Rotation
        :return: a numpy.array with 3x3 elements represented as a directional vector representation of the orientation
        :rtype: numpy.array
    """
    rotation_matrix = carla_rotation_to_numpy_rotation_matrix(carla_rotation)
    directional_vector = numpy.array([1, 0, 0])
    rotated_directional_vector = rotation_matrix.dot(directional_vector)
    return rotated_directional_vector


def carla_velocity_to_ros_twist(carla_velocity):
    """
        Convert carla velocity to a ROS twist object
        Considers the conversion from the left-handed system (unreal) to the right-handed system (ROS)
        Considers that the angular velocities remain zero.
        :param carla_velocity: The Carla Velocity
        :type carla_velocity: carla.Vector3D
        :return: a ROS twist
        :rtype: geometry_msgs.msg.Twist
    """
    ros_twist = Twist()
    ros_twist.linear.x = carla_velocity.x
    ros_twist.linear.y = carla_velocity.y
    ros_twist.linear.z = carla_velocity.z
    return ros_twist


def carla_velocity_to_numpy_vector(carla_velocity):
    """
       Convert carla velocity to a numpy array
       Considers the conversion from the left-handed system (unreal) to the right-handed system (ROS)
       Considers that the angular velocities remain zero.
       :param carla_velocity: The Carla Velocity
       :type carla_velocity: carla.Vector3D
       :return: a numpy.array with 3 elements
       :rtype: numpy.array
    """
    return numpy.array([carla_velocity.x, -carla_velocity.y, carla_velocity.z])


def carla_acceleration_to_ros_accel(carla_acceleration):
    """
        Convert carla acceleration to a ROS accel
        Considers the conversion from the left-handed system (unreal) to the right-handed system (ROS)
        Considers that the angular accelerations remain zero.
        :param carla_acceleration: The Carla Acceleration
        :type carla_acceleration: carla.Vector3D
        :return: a ROS accel
        :rtype: geometry_msgs.msg.Accel
    """
    ros_accel = Accel()
    ros_accel.linear.x = carla_acceleration.x
    ros_accel.linear.y = -carla_acceleration.y
    ros_accel.linear.z = carla_acceleration.z
    return ros_accel


def carla_transform_to_ros_transform(carla_transform):
    """
        Convert carla transform to a ROS transform
        Considers the conversion from the left-handed system (unreal) to the right-handed system (ROS)
        See carla_location_to_ros_vector3() and carla_rotation_to_ros_quaternion() for details
        :param carla_transform: The Carla Transform
        :type carla_transform: carla.Transform
        :return: a ROS transform
        :rtype: geometry_msgs.msg.Transform
    """
    ros_transform = Transform()
    ros_transform.translation = carla_location_to_ros_vector3(carla_transform.location)
    ros_transform.rotation = carla_rotation_to_ros_quaternion(carla_transform.rotation)
    return ros_transform


def carla_transform_to_ros_pose(carla_transform):
    """
        Convert carla transform to a ROS pose
        Considers the conversion from the left-handed system (unreal) to the right-handed system (ROS)
        See carla_location_to_ros_point() and carla_rotation_to_ros_quaternion() for details
        :param carla_transform: The Carla Transform
        :type carla_transform: carla.Transform
        :return: a ROS transform
        :rtype: geometry_msgs.msg.Transform
    """
    ros_pose = Pose()
    ros_pose.position = carla_location_to_ros_point(carla_transform.location)
    ros_pose.orientation = carla_rotation_to_ros_quaternion(carla_transform.rotation)
    return ros_pose


def carla_location_to_pose(carla_location):
    """
        Convert carla location to a ROS pose
        Considers the converrsion from the left-handed system (unreal) to the right-handed system (ROS)
        See carla-location-to_ros_point() for details
        Considers pose quaternion remains zero.
        :param carla_location: The Carla Location
        :type carla_location: carla.Location
        :return: a ROS pose
        :rtype: geometry_msgs.msg.Pose
    """
    ros_pose = Pose()
    ros_pose.position = carla_location_to_ros_point(carla_location)
    return ros_pose



