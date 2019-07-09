#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Class used for ROS Manual Control for ROS Vehicle

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

    C/(Shift+C)  : change weather conditions
    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

# ==============================================================================
# -- IMPORTS -------------------------------------------------------------------
# ==============================================================================

from __future__ import print_function
import datetime
import math
import numpy
import rospy
import tf
import re
import carla
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from carla_ros_bridge_msgs.msg import CarlaCollisionEvent            # pylint: disable=no-name-in-module, import-error
from carla_ros_bridge_msgs.msg import CarlaLaneInvasionEvent         # pylint: disable=no-name-in-module, import-error
from carla_ros_bridge_msgs.msg import CarlaEgoVehicleControl         # pylint: disable=no-name-in-module, import-error
from carla_ros_bridge_msgs.msg import CarlaEgoVehicleStatus          # pylint: disable=no-name-in-module, import-error
from carla_ros_bridge_msgs.msg import CarlaEgoVehicleInfo            # pylint: disable=no-name-in-module, import-error
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
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_b
    from pygame.locals import K_1
    from pygame.locals import K_2
    from pygame.locals import K_3
except ImportError:
    raise RuntimeError('Cannot import pygame, make sure pygame package is installed before running this module')

# ==============================================================================
# -- Global Functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def set_weather_params(preset):
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(2000)
    world = client.get_world()
    world.set_weather(preset)

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    """
    Class used to handle the CARLA world rendering
    """
    def __init__(self, role_name, hud):
        """
        Constructor for World class
        :param hud: info display
        """
        self._surface = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self.hud = hud
        self.role_name = role_name
        self.image_subscriber = rospy.Subscriber("/carla/{}/camera/rgb/view/image_color".format(self.role_name),
                                                 Image, self.on_view_image)
        self.collision_subscriber = rospy.Subscriber("/carla/{}/collision".format(self.role_name),
                                                     CarlaCollisionEvent, self.on_collision)
        self.lane_invasion_subscriber = rospy.Subscriber("/carla/{}/lane_invasion".format(self.role_name),
                                                         CarlaLaneInvasionEvent, self.on_lane_invasion)

    def on_view_image(self, data):
        """
        Callback function used on view semantic segmentation camera image event
        :param data: data info
        :return:
        """
        array = numpy.frombuffer(data.data, dtype=numpy.dtype("uint8"))
        array = numpy.reshape(array, (data.height, data.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def on_collision(self, data):
        """
        Callback function used on collision event
        :param data: data info
        :return:
        """
        intensity = math.sqrt(data.normal_impulse.x**2 + data.normal_impulse.y**2 + data.normal_impulse.z**2)
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

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        set_weather_params(preset[0])

    def destroy(self):
        """
        Function used to destroy all objects in the world
        :return:
        """
        self.image_subscriber.unregister()
        self.collision_subscriber.unregister()
        self.lane_invasion_subscriber.unregister()

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """
    Class used to handle keyboard input events
    """
    def __init__(self, role_name, world, hud):
        """
           Constructor for KeyboardControl class
           :param hud: info display
        """
        self.role_name = role_name
        self.vehicle_control_manual_override_publisher = rospy.Publisher("/carla/{}/vehicle_control_manual_override".format(self.role_name),
                                                                         Bool, queue_size=1, latch=True)
        self.vehicle_control_manual_override = False
        self.auto_pilot_enable_publisher = rospy.Publisher("/carla/{}/enable_autopilot".format(self.role_name),
                                                           Bool, queue_size=1)
        self.vehicle_control_publisher = rospy.Publisher("/carla/{}/vehicle_control_cmd".format(self.role_name),
                                                         CarlaEgoVehicleControl, queue_size=1)
        self._autopilot_enabled = False
        self._control = CarlaEgoVehicleControl()
        self.set_autopilot(self._autopilot_enabled)
        self._steer_cache = 0.0
        self.hud = hud
        self.world = world
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
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    self.world.next_weather(reverse=True)
                elif event.key == K_c:
                    self.world.next_weather()
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

# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    """
    Class used to handle Info Display
    """
    def __init__(self, role_name, width, height):
        """
        Constructor for HUD Class
        :param width: display width
        :param height: display height
        """
        self.role_name = role_name
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        fonts = [x for x in pygame.font.get_fonts() if 'mono' in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self._show_info = True
        self._info_text = []
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.vehicle_status_subscriber = rospy.Subscriber("/carla/{}/vehicle_status".format(self.role_name),
                                                          CarlaEgoVehicleStatus, self.vehicle_status_updated)
        self.vehicle_info = CarlaEgoVehicleInfo()
        self.vehicle_info_subscriber = rospy.Subscriber("/carla/{}/vehicle_info".format(self.role_name),
                                                        CarlaEgoVehicleInfo, self.vehicle_info_updated)
        self.latitude = 0
        self.longitude = 0
        self.gnss_subscriber = rospy.Subscriber("/carla/{}/gnss/gnss1/fix".format(self.role_name),
                                                NavSatFix, self.gnss_updated)
        self.tf_listener = tf.TransformListener()
        self.manual_control = False
        self.manual_control_subscriber = rospy.Subscriber(
                                               "/carla/{}/vehicle_control_manual_override".format(self.role_name),
                                               Bool, self.manual_control_override_updated)

    def __del__(self):
        """
        Destructor for HUD Class
        :return:
        """
        self.gnss_subscriber.unregister()
        self.vehicle_status_subscriber.unregister()
        self.vehicle_info_subscriber.unregister()

    def tick(self, clock):
        """
        Function used to tick the clock
        :param clock:
        :return:
        """
        self._notifications.tick(clock)

    def vehicle_status_updated(self, vehicle_status):
        """
        Callback function used on vehicle status updates
        :param vehicle_status: vehicle status info
        :return:
        """
        self.vehicle_status = vehicle_status
        self.update_info_text()

    def vehicle_info_updated(self, vehicle_info):
        """
        Callback function used on vehicle info updates
        :param vehicle_info: vehicle info
        :return:
        """
        self.vehicle_info = vehicle_info
        self.update_info_text()

    def gnss_updated(self, data):
        """
        Callback function used on gnss position updates
        :param data: gnss data position
        :return:
        """
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.update_info_text()

    def manual_control_override_updated(self, data):
        """
        Callback on vehicle manual control updates
        """
        self.manual_control = data.data
        self.update_info_text()

    def update_info_text(self):
        """
        Function used to update the displayed info text
        :return:
        """
        if not self._show_info:
            return
        try:
            (position, quaternion) = self.tf_listener.lookupTransform(
                '/map', "{}".format(self.role_name), rospy.Time())
            _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
            yaw = -math.degrees(yaw)
            x = position[0]
            y = -position[1]
            z = position[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            x = 0
            y = 0
            z = 0
            yaw = 0
        heading = 'N' if abs(yaw) < 89.5 else ''
        heading += 'S' if abs(yaw) > 90.5 else ''
        heading += 'E' if 179.5 > yaw > 0.5 else ''
        heading += 'W' if -0.5 > yaw > -179.5 else ''
        self._info_text = [
            'Vehicle: % 20s' % ' '.join(self.vehicle_info.type.title().split('.')[1:]),
            'Simulation time: % 12s' % datetime.timedelta(
                seconds=int(rospy.get_rostime().to_sec())),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * self.vehicle_status.velocity),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (x, y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (self.latitude, self.longitude)),
            'Height:  % 18.0f m' % z,
            '']
        self._info_text += [
            ('Throttle:', self.vehicle_status.control.throttle, 0.0, 1.0),
            ('Steer:', self.vehicle_status.control.steer, -1.0, 1.0),
            ('Brake:', self.vehicle_status.control.brake, 0.0, 1.0),
            ('Reverse:', self.vehicle_status.control.reverse),
            ('Hand brake:', self.vehicle_status.control.hand_brake),
            ('Manual:', self.vehicle_status.control.manual_gear_shift),
            'Gear:        %s' % {-1: 'R', 0: 'N'}.get(self.vehicle_status.control.gear,
                                                      self.vehicle_status.control.gear),
            '']
        self._info_text += [('Manual ctrl:', self.manual_control)]

    def toggle_info(self):
        """
        Function used to show/hide the info text
        """
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        """
        Function used to display a notification for x seconds
        """
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        """
        Function used to display an error message
        :param text:
        :return:
        """
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        """
        Function used to render the display
        :param display:
        :return:
        """
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30)
                                  for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect(
                                (bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)

# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    """
    Support Class for info display, fade out text
    """
    def __init__(self, font, dim, pos):
        """
        Constructor for FadingText Support Class
        :param font:
        :param dim:
        :param pos:
        """
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        """
        Function used for set the text
        :param text:
        :param color:
        :param seconds:
        :return:
        """
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, clock):
        """
        Function used to tick for fading text
        :param clock:
        :return:
        """
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        """
        Function used to render the fading text
        :param display:
        :return:
        """
        display.blit(self.surface, self.pos)

# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """
    Support Class Help Text Info
    """

    def __init__(self, font, width, height):
        """
        Constructor for HelpText Support Class
        :param font:
        :param width:
        :param height:
        """
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        """
        Function used to show/hide the help text
        """
        self._render = not self._render

    def render(self, display):
        """
        Function used to render the help text
        """
        if self._render:
            display.blit(self.surface, self.pos)

# ==============================================================================
# -- Main Function -------------------------------------------------------------
# ==============================================================================


def main():
    """
    Main Function
    """
    rospy.init_node('carla_ros_manual_control')
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    # resolution should be similar to spawned camera with role-name 'view'
    resolution = {"width": 800, "height": 600}
    # resolution = {"width": 1624, "height": 1224}

    pygame.init()
    pygame.font.init()
    world = None
    try:
        display = pygame.display.set_mode(
            (resolution['width'], resolution['height']),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(role_name, resolution['width'], resolution['height'])
        world = World(role_name, hud)
        controller = KeyboardControl(role_name, world, hud)

        clock = pygame.time.Clock()

        while not rospy.core.is_shutdown():
            clock.tick_busy_loop(60)
            if controller.parse_events(clock):
                return
            hud.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:
        if world is not None:
            world.destroy()
        pygame.quit()


if __name__ == '__main__':

    main()
