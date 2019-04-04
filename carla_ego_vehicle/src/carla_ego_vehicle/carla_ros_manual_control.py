#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Class used for ROS Manual Control

Welcome to CARLA ROS manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    B            : toggle manual control

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

# ==============================================================================
# -- IMPORTS ---------------------------------------------------------
# ==============================================================================

from __future__ import print_function
import datetime
import math
import numpy
import rospy
import tf
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from carla_ros_bridge.msg import CarlaCollisionEvent            # pylint: disable=no-name-in-module, import-error
from carla_ros_bridge.msg import CarlaLaneInvasionEvent         # pylint: disable=no-name-in-module, import-error
from carla_ros_bridge.msg import CarlaEgoVehicleControl         # pylint: disable=no-name-in-module, import-error
from carla_ros_bridge.msg import CarlaEgoVehicleStatus          # pylint: disable=no-name-in-module, import-error
from carla_ros_bridge.msg import CarlaEgoVehicleInfo            # pylint: disable=no-name-in-module, import-error
try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_b
except ImportError:
    raise RuntimeError('Cannot import pygame, make sure pygame package is installed before running this module')

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    """
    Class used to handle the CARLA world rendering
    """
    def __init__(self, hud):
        """
        Constructor for World class
        :param hud: info display
        """
        self._surface = None
        self.hud = hud
        self.image_subscriber = rospy.Subscriber("/carla/ego_vehicle/camera/rgb/view/image_color", Image, self.on_view_image)
        self.collision_subscriber = rospy.Subscriber("/carla/ego_vehicle/collision", CarlaCollisionEvent, self.on_collision)
        self.lane_invasion_subscriber = rospy.Subscriber("/carla/ego_vehicle/lane_invasion", CarlaLaneInvasionEvent, self.on_lane_invasion)

    def on_view_image(self, data):
        """
        Callback function used on view camera image event
        :param data: data info
        :return:
        """
        array = numpy.frombuffer(data.data, dtype=numpy.dtype("uint8"))
        array = numpy.reshape(array, (data.height, data.width, 4))
        array = array[:, :, 3]
        array = array[:, :, ::-1]
        self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def on_collision(self, data):
        """
        Callback function used on collision event
        :param data: data info
        :return:
        """
        intensity = math.sqrt(data.normal_impulse.x*2 + data.normal_impulse.y*2 + data.normal_impulse.z*2)
        self.hud.notification('Collision with {} (impulse {})'.format(data.other_actor_id, intensity))

    def on_lane_invasion(self, data):
        """
        Callback function used on lane invasion event
        :param data: data info
        :return:
        """
        text = []
        for marking in data.crossed_lane_markings:
            if marking is CarlaLaneInvasionEvent.LANE_MARKING_OTHER:
                text.append("Other")
            elif marking is CarlaLaneInvasionEvent.LANE_MARKING_BROKEN:
                text.append("Broken")
            elif marking is CarlaLaneInvasionEvent.LANE_MARKING_SOLID:
                text.append("Solid")
            else:
                text.append("Unknown")
        self.hud.notification('Crossed Line %s' % ' and '.join(text))

    def render(self, display):
        """
        Function used to render current image data
        :param display: image data
        :return:
        """
        if self._surface is not None:
            display.blit(self._surface, (0, 0))
        self.hud.render(display)

    def destroy(self):
        """
        Function used to destroy all objects in the world
        :return:
        """
        self.image_subscriber.unregister()
        self.collision_subscriber.unregister()
        self.lane_invasion_subscriber.unregister()




