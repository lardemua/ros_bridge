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

# ==============================================================================
# -- KeyboardControl ---------------------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """
    Class used to handle keyboard input events
    """
    def __init__(self, hud):
        """
           Constructor for KeyboardControl class
           :param hud: info display
        """
        self.vehicle_control_manual_override_publisher = rospy.Publisher("/vehicle_control_manual_override",
                                                                         Bool, queue_size=1, latch=True)
        self.vehicle_control_manual_override = False
        self.auto_pilot_enable_publisher = rospy.Publisher("/carla/ego_vehicle/enable_autopilot", Bool, queue_size=1)
        self.vehicle_control_publisher = rospy.Publisher("/carla/ego_vehicle/vehicle_control_cmd",
                                                         CarlaEgoVehicleControl, queue_size=1)
        self._autopilot_enabled = False
        self._control = CarlaEgoVehicleControl()
        self.set_autopilot(self._autopilot_enabled)
        self._steer_cache = 0.0
        self.hud = hud
        self.set_vehicle_control_manual_override(self.vehicle_control_manual_override)   # disable manual override
        self.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def __del__(self):
        """
        Destructor for KeyboardControl class
        :return:
        """
        self.auto_pilot_enable_publisher.unregister()
        self.vehicle_control_publisher.unregister()
        self.vehicle_control_manual_override_publisher.unregister()

    def set_vehicle_control_manual_override(self, enable):
        """
        Function used to set the manual control override
        :param enable:
        :return:
        """
        self.hud.notification('Set Vehicle Control Manual Override to: {}'.format(enable))
        self.vehicle_control_manual_override_publisher.publish((Bool(data=enable)))

    def set_autopilot(self, enable):
        """
        Function used to enable/disable the autopilot
        :param enable:
        :return:
        """
        self.auto_pilot_enable_publisher.publish(Bool(data=enable))

    def parse_events(self, clock):
        """
        Function used to parse input events
        :param clock:
        :return:
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_F1:
                    self.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    self.hud.help.toggle()
                elif event.key == K_b:
                    self.vehicle_control_manual_override = not self.vehicle_control_manual_override
                    self.set_vehicle_control_manual_override(self.vehicle_control_manual_override)
                if event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_m:
                    self._control.manual_gear_shift = not self._control.manual_gear_shift
                    self.hud.notification(
                        '%s Transmission' % ('Manual' if self._control.manual_gear_shift else 'Automatic')
                    )
                elif self._control.manual_gear_shift and event.key == K_COMMA:
                    self._control.gear = max(-1, self._control.gear - 1)
                elif self._control.manual_gear_shift and event.key == K_PERIOD:
                    self._control.gear = self._control.gear + 1
                elif event.key == K_p:
                    self._autopilot_enabled = not self._autopilot_enabled
                    self.set_autopilot(self._autopilot_enabled)
                    self.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
        if not self._autopilot_enabled and self.vehicle_control_manual_override:
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            self._control.reverse = self._control.gear < 0
            self.vehicle_control_publisher.publish(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        """
        Private function used to parse vehicle key events
        :param keys: key events
        :param milliseconds: timestamps
        :return:
        """
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)