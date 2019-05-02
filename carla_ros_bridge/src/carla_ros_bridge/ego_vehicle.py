#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Classes to handle Carla Ego Vehicles
"""

# ------------------------
#   IMPORTS
# ------------------------
import math
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Bool
from carla import VehicleControl
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge_msgs.msg import CarlaEgoVehicleControl         # pylint: disable=no-name-in-module,import-error
from carla_ros_bridge_msgs.msg import CarlaEgoVehicleStatus          # pylint: disable=no-name-in-module,import-error
from carla_ros_bridge_msgs.msg import CarlaEgoVehicleInfo            # pylint: disable=no-name-in-module,import-error
from carla_ros_bridge_msgs.msg import CarlaEgoVehicleInfoWheel       # pylint: disable=no-name-in-module,import-error


class EgoVehicle(Vehicle):
    """
    Vehicle Implementation Details for the Ego Vehicle
    """

    @staticmethod
    def create_actor(carla_actor, parent):
        """
        Static factory method to create ego vehicle actors
        :param carla_actor: Carla Vehicle Actor Object
        :type carla_actor: carla.Vehicle
        :param parent: The Parent of the new Traffic Actor
        :type parent: carla_ros_bridge.Parent
        :return: The Created Vehicle Actor
        :rtype: carla_ros_bridge.Vehicle or derived type
        """
        return EgoVehicle(carla_actor=carla_actor, parent=parent)

    def __init__(self, carla_actor, parent):
        """
        Constructor for EgoVehicle Object
        :param carla_actor: Carla Actor Object
        :type carla_actor: carla.Actor
        :param parent: The Parent of this Actor Object
        :type parent: carla-ros_bridge.Parent
        """
        super(EgoVehicle, self).__init__(carla_actor=carla_actor,
                                         parent=parent,
                                         topic_prefix=carla_actor.attributes.get('role_name'),
                                         append_role_name_topic_postfix=False)
        self.vehicle_info_published = False
        self.control_subscriber = rospy.Subscriber(self.topic_name() + "/vehicle_control_cmd",
                                                   CarlaEgoVehicleControl,
                                                   self.control_command_updated)
        self.enable_autopilot_subscriber = rospy.Subscriber(self.topic_name() + "/enable_autopilot",
                                                            Bool,
                                                            self.enable_autopilot_updated)

    def get_marker_color(self):
        """
        Override Function used to return the color for marker messages
        The Ego Vehicle uses a different marker color than other vehicles
        :return: The Color Used By a Ego Vehicle Marker
        :rtype: std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 0
        color.g = 255
        color.b = 0
        return color

    def send_vehicle_msgs(self):
        """
        Override Function used to send vehicle messages of the ego vehicle
        instead of an object message.
        The Ego Vehicle doesn't send its information as part of the object list.
        A nav_msgs.msg.Odometry is prepared to be published via '/carla/ego_vehicle'
        :return:
        """
        # Input Vehicle Status Values
        vehicle_status = CarlaEgoVehicleStatus()
        vehicle_status.header.stamp = self.get_current_ros_time()
        vehicle_status.velocity = self.get_vehicle_speed_abs(self.carla_actor)
        vehicle_status.acceleration = self.get_vehicle_acceleration_abs(self.carla_actor)
        vehicle_status.orientation = self.get_current_ros_pose().orientation
        vehicle_status.control.throttle = self.carla_actor.get_control().throttle
        vehicle_status.control.steer = self.carla_actor.get_control().steer
        vehicle_status.control.brake = self.carla_actor.get_control().brake
        vehicle_status.control.hand_brake = self.carla_actor.get_control().hand_brake
        vehicle_status.control.reverse = self.carla_actor.get_control().reverse
        vehicle_status.control.gear = self.carla_actor.get_control().gear
        vehicle_status.control.manual_gear_shift = self.carla_actor.get_control().manual_gear_shift
        # Publish Vehicle Status Message
        self.publish_ros_message(self.topic_name() + "/vehicle_status", vehicle_status)

        # Check if no info has yet been published
        if not self.vehicle_info_published:
            self.vehicle_info_published = True
            vehicle_info = CarlaEgoVehicleInfo()
            vehicle_info.ID = self.carla_actor.id
            vehicle_info.type = self.carla_actor.type_id
            vehicle_info.rolename = self.carla_actor.attributes.get('role_name')
            vehicle_physics = self.carla_actor.get_physics_control()
            # Check wheel info
            for wheel in vehicle_physics.wheels:
                wheel_info = CarlaEgoVehicleInfoWheel()
                wheel_info.tire_friction = wheel.tire_friction
                wheel_info.damping_rate = wheel.damping_rate
                wheel_info.steer_angle = math.radians(wheel.steer_angle)
                wheel_info.disable_steering = wheel.disable_steering
                vehicle_info.wheels.append(wheel_info)
            vehicle_info.max_rpm = vehicle_physics.max_rpm
            vehicle_info.moi = vehicle_physics.moi
            vehicle_info.damping_rate_full_throttle = vehicle_physics.damping_rate_full_throttle
            vehicle_info.damping_rate_zero_throttle_clutch_engaged = \
                vehicle_physics.damping_rate_zero_throttle_clutch_engaged
            vehicle_info.damping_rate_zero_throttle_clutch_disengaged = \
                vehicle_physics.damping_rate_zero_throttle_clutch_disengaged
            vehicle_info.use_gear_autobox = vehicle_physics.use_gear_autobox
            vehicle_info.gear_switch_time = vehicle_physics.gear_switch_time
            vehicle_info.clutch_strength = vehicle_physics.clutch_strength
            vehicle_info.mass = vehicle_physics.mass
            vehicle_info.drag_coefficient = vehicle_physics.drag_coefficient
            vehicle_info.center_of_mass.x = vehicle_physics.center_of_mass.x
            vehicle_info.center_of_mass.y = vehicle_physics.center_of_mass.y
            vehicle_info.center_of_mass.z = vehicle_physics.center_of_mass.z
            # Publish Vehicle Info
            self.publish_ros_message(self.topic_name() + "/vehicle_info", vehicle_info, True)

        # @todo: Is this still needed?
        if not self.parent.get_param("challenge mode"):
            self.send_object_msg()

    def send_object_msg(self):
        """
        Override Function used to send odometry message of the ego vehicle
        instead of an object message.
        The Ego Vehicle doesn't send its information as part of the object list.
        A nav_msgs.msg.Odometry is prepared to be published via '/carla/ego_vehicle'
        :return:
        """
        odometry = Odometry(header=self.get_msg_header())
        odometry.child_frame_id = self.get_frame_ID()
        odometry.pose.pose = self.get_current_ros_pose()
        odometry.twist.twist = self.get_current_ros_twist()
        self.publish_ros_message(self.topic_name() + "/odometry", odometry)

    def update(self):
        """
        Override Function used to update this object
        On Update Ego Vehicle calculates and sends the new values for VehicleControl()
        :return:
        """
        objects = super(EgoVehicle, self).get_filtered_objectarray(self.carla_actor.id)
        self.publish_ros_message(self.topic_name() + "/objects", objects)
        self.send_vehicle_msgs()
        super(EgoVehicle, self).update()

    def destroy(self):
        """
        Override Function used to destroy this object
        Terminate ROS subscription on CarlaEgoVehicleControl Commands
        Finally forward call to the super class
        :return:
        """
        rospy.logdebug("Destroy Vehicle(id={})".format(self.get_ID()))
        self.control_subscriber.unregister()
        self.control_subscriber = None
        self.enable_autopilot_subscriber.unregister()
        self.enable_autopilot_subscriber = None
        super(EgoVehicle, self).destroy()

    def control_command_updated(self, ros_vehicle_control):
        """
        Function used to receive CarlaEgoVehicleControl messages and send them to Carla module.
        This function gets called whenever a ROS message is received via
        '/carla/ego_vehicle/vehicle_control_cmd' topic.
        The received ROS message is converted into carla.VehicleControl command and
        sent to CARLA.
        This bridge is not responsible for any restrictions on velocity or steering.
        It's just forwarding the ROS input to CARLA module.
        :param ros_vehicle_control: current vehicle control input received via ROS.
        :type ros_vehicle_control: carla_ros_bridge.msg.CarlaEgoVehicleControl
        :return:
        """
        vehicle_control = VehicleControl()
        vehicle_control.hand_brake = ros_vehicle_control.hand_brake
        vehicle_control.brake = ros_vehicle_control.brake
        vehicle_control.steer = ros_vehicle_control.steer
        vehicle_control.throttle = ros_vehicle_control.throttle
        vehicle_control.reverse = ros_vehicle_control.reverse
        self.carla_actor.apply_control(vehicle_control)

    def enable_autopilot_updated(self, enable_auto_pilot):
        """
        Function used to enable/disable auto pilot
        :param enable_auto_pilot: should the autopilot be enabled?
        :type enable_auto_pilot: std_msgs.Bool
        :return:
        """
        rospy.logdebug("Ego vehicle: Set autopilot to {}".format(enable_auto_pilot.data))
        self.carla_actor.set_autopilot(enable_auto_pilot.data)

    @staticmethod
    def get_vector_length_squared(carla_vector):
        """
        Static Function used to calculate the squared length of a carla_vector
        :param carla_vector: the carla vector
        :type carla_vector: carla.Vector3D
        :return: squared vector length
        :rtype: float64
        """
        return carla_vector.x * carla_vector.x + carla_vector.y * carla_vector.y + carla_vector.z * carla_vector.z

    @staticmethod
    def get_vehicle_speed_squared(carla_vehicle):
        """
        Static Function used to get the squared speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: squared speed of a carla vehicle [(m/s)^2]
        :rtype: float64
        """
        return EgoVehicle.get_vector_length_squared(carla_vehicle.get_velocity())

    @staticmethod
    def get_vehicle_speed_abs(carla_vehicle):
        """
        Static Function used to get the absolute speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: speed of a carla vehicle [m/s >= 0]
        :rtype: float64
        """
        speed = math.sqrt(EgoVehicle.get_vehicle_speed_squared(carla_vehicle))
        return speed

    @staticmethod
    def get_vehicle_acceleration_abs(carla_vehicle):
        """
        Static Function used to get the absolute acceleration of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: vehicle acceleration value [m/s^2 >=0]
        :rtype: float64
        """
        return math.sqrt(EgoVehicle.get_vector_length_squared(carla_vehicle.get_acceleration()))
