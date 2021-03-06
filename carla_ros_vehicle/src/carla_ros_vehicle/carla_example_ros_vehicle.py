#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Base Class used as an example for Carla ROS Vehicle

"""

# ==============================================================================
# -- IMPORTS -------------------------------------------------------------------
# ==============================================================================
from carla_ros_vehicle_base import CarlaRosVehicleBase


class CarlaExampleRosVehicle(CarlaRosVehicleBase):
    """
    Example Class for Carla ROS Vehicle
    """
    def sensors(self):
        """
        Define all sensors attached to your ego vehicle
        :return:
        """
        return [
            {
                'type': 'sensor.camera.rgb',
                'role_name': 'front',
                'x': 2.0, 'y': 0.0, 'z': 2.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'width': 800,
                'height': 600,
                'fov': 100
            },
            {
                'type': 'sensor.camera.rgb',
                'role_name': 'view',
                'x': -4.5, 'y': 0.0, 'z': 2.8, 'roll': 0.0, 'pitch': -20.0, 'yaw': 0.0,
                'width': 800,
                'height': 600,
                'fov': 100
            },
            {
                'type': 'sensor.camera.depth',
                'role_name': 'front',
                'x': -2.0, 'y': 0.0, 'z': -2.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'width': 800,
                'height': 600,
                'fov': 100
            },
            {
                'type': 'sensor.camera.depth',
                'role_name': 'view',
                'x': -4.5, 'y': 0.0, 'z': 2.8, 'roll': 0.0, 'pitch': -20.0, 'yaw': 0.0,
                'width': 800,
                'height': 600,
                'fov': 100
            },
            {
                'type': 'sensor.camera.semantic_segmentation',
                'role_name': 'front',
                'x': 4.0, 'y': 0.0, 'z': 4.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'width': 800,
                'height': 600,
                'fov': 100
            },
            {
                'type': 'sensor.camera.semantic_segmentation',
                'role_name': 'view',
                'x': -4.5, 'y': 0.0, 'z': 2.8, 'roll': 0.0, 'pitch': -20.0, 'yaw': 0.0,
                'width': 800,
                'height': 600,
                'fov': 100
            },
            {
                'type': 'sensor.lidar.ray_cast',
                'role_name': 'front',
                'x': 0.0, 'y': 0.0, 'z': 2.4, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'range': 1000,
                'channels': 32,
                'points_per_second': 320000,
                'upper_fov': 2.0,
                'lower_fov': -26.8,
                'rotation_frequency': 20
            },
            {
                'type': 'sensor.lidar.ray_cast',
                'role_name': 'lidar1',
                'x': 0.0, 'y': 0.0, 'z': 2.4, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'range': 200,
                'channels': 32,
                'points_per_second': 500000,
                'upper_fov': 15,
                'lower_fov': -30,
                'rotation_frequency': 10
            },
            {
                'type': 'sensor.lidar.ray_cast',
                'role_name': 'left',
                'x': 0.0, 'y': 2.0, 'z': 2.4, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'range': 1000,
                'channels': 32,
                'points_per_second': 320000,
                'upper_fov': 2.0,
                'lower_fov': -26.8,
                'rotation_frequency': 20
            },
            {
                'type': 'sensor.lidar.ray_cast',
                'role_name': 'right',
                'x': 0.0, 'y': -2.0, 'z': 2.4, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'range': 1000,
                'channels': 32,
                'points_per_second': 320000,
                'upper_fov': 2.0,
                'lower_fov': -26.8,
                'rotation_frequency': 20
            },
            {
                'type': 'sensor.other.gnss',
                'role_name': 'sensor',
                'x': 1.0, 'y': 0.0, 'z': 2.0
            },
            {
                'type': 'sensor.other.collision',
                'role_name': 'sensor',
                'x': 0.0, 'y': 0.0, 'z': 0.0
            },
            {
                'type': 'sensor.other.lane_invasion',
                'role_name': 'sensor',
                'x': 0.0, 'y': 0.0, 'z': 0.0
            }
        ]

# ==============================================================================
# -- Main Function ---------------------------------------------------------
# ==============================================================================


def main():
    """
    Main Function
    :return:
    """
    ros_vehicle = CarlaExampleRosVehicle()
    try:
        ros_vehicle.run()
    finally:
        if ros_vehicle is not None:
            ros_vehicle.destroy()


if __name__ == '__main__':
    main()