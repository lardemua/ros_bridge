#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Classes to handle Carla vehicles
"""

# ------------------------
#   IMPORTS
# ------------------------
import math
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from carla import VehicleControl
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.msg import CarlaEgoVehicleControl         # pylint: disable=no-name-in-module,import-error
from carla_ros_bridge.msg import CarlaEgoVehicleStatus          # pylint: disable=no-name-in-module,import-error
from carla_ros_bridge.msg import CarlaEgoVehicleInfo            # pylint: disable=no-name-in-module,import-error
from carla_ros_bridge.msg import CarlaEgoVehicleInfoWheel       # pylint: disable=no-name-in-module,import-error


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

    # def apply_control(self):
    #     """
    #     Function used to apply current control output to CARLA module
    #     :return:
    #     """
    #     vehicle_control = VehicleControl()
    #     vehicle_control.hand_brake = self.info.output.hand_brake
    #     vehicle_control.brake = self.info.output.brake
    #     vehicle_control.steer = self.info.output.steer
    #     vehicle_control.throttle = self.info.output.throttle
    #     vehicle_control.reverse = self.info.output.reverse
    #     self.carla_actor.apply_control(vehicle_control)
    #
    # def update_current_values(self):
    #     """
    #     Function used to update vehicle control current values
    #     We calculate the acceleration on ourselves, because we are interested only in
    #     the acceleration in respect to the driving direction
    #     In addition a small average filter is applied
    #     :return:
    #     """
    #     current_time_sec = self.get_current_ros_time().to_sec()
    #     delta_time = current_time_sec - self.info.current.time_sec
    #     current_speed = phys.get_vehicle_speed(self.carla_actor)
    #     if delta_time > 0:
    #         delta_speed = current_speed - self.info.current.speed
    #         current_accel = delta_speed / delta_time
    #         # average filter
    #         self.info.current.accel = (self.info.current.accel * 4 + current_accel) / 5
    #     self.info.current.time_sec = current_time_sec
    #     self.info.current.speed = current_speed
    #     self.info.current.speed_abs = abs(current_speed)
    #
    # def vehicle_control_cycle(self):
    #     """
    #     Function used to perform a control cycle
    #             To be overridden by derived control classes
    #     :return:
    #     """
    #     pass

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
        rospy.logdebug("Destroy Vehicle(id={})".format(self.get_id()))
        self.control_subscriber.unregister()
        self.control_subscriber = None
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

    # def send_ego_vehicle_control_info_msg(self):
    #     """
    #     Function used to send carla_ros_bridge.msg.EgoVehicleControlInfo message
    #     :return:
    #     """
    #     self.info.header = self.get_msg_header()
    #     self.info.output.header = self.info.header
    #     self.publish_ros_message(self.topic_name() + "/ego_vehicle_control_info", self.info)

    @staticmethod
    def get_vector_length_squared(carla_vector):
        """
        Static Function used to calculate the squared length of a carla_vector
        :param carla_vector: the carla vector
        :type carla_vector: carla.Vector3D
        :return: squared vector length
        :rtype: float64
        """
        return carla_vector.x * carla_vector.x + \
               carla_vector.y * carla_vector.y + \
               carla_vector.z * carla_vector.z

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


# class PedalControlVehicle(EgoVehicle):
#     """
#     Vehicle Implementation Details for the Ego Vehicle in Pedal Control Variant
#     """
#
#     def __init__(self, carla_actor, parent):
#         """
#         Constructor for PedalControlVehicle class
#         :param carla_actor: carla actor object
#         :type carla_actor: carla.Actor
#         :param parent: the parent of this
#         :type parent: carla_ros_bridge.Parent
#         """
#         super(PedalControlVehicle, self).__init__(carla_actor=carla_actor, parent=parent)
#         self.control_subscriber = rospy.Subscriber(
#             self.topic_name() + "/vehicle_control_cmd", CarlaVehicleControl, self.control_command_updated
#         )
#
#     def destroy(self):
#         """
#         Override Function used to destroy this object
#         Terminate ROS subscription on CarlaVehicleControl commands
#         Finally forward call to the super class
#         :return:
#         """
#         rospy.logdebug("Destroy PedalControlVehicle(id={})".format(self.get_ID()))
#         self.control_subscriber = None
#         super(PedalControlVehicle, self).destroy()
#
#     def control_command_updated(self, ros_vehicle_control):
#         """
#         Function is used to receive a CarlaVehicleControl msg and send to CARLA module
#         This function gets called whenever a ROS message is received via
#         '/carla/ego_vehicle/vehicle_control_cmd' topic.
#         The received ROS message is converted into carla.VehicleControl command and
#         sent to CARLA.
#         This brigde is not responsible for any restrictions on velocity or steering.
#         It's just forwarding the ROS input to CARLA
#         :param ros_vehicle_control: Currrent Vehicle Control Input received via ROS
#         :type ros_vehicle_control: carla_ros_bridge.msg.CarlaVehicleControl
#         :return:
#         """
#         self.info_output = ros_vehicle_control
#         self.apply_control()
#
#
# class AckermannControlVehicle(EgoVehicle):
#     """
#     Vehicle Implementation Details for the Ego Vehicle in Ackermann Control Variant
#     """
#
#     def __init__(self, carla_actor, parent):
#         """
#         Constructor for the AckermannControlVehicle class
#         :param carla_actor: Carla Actor Object
#         :type carla_actor: carla.Actor
#         :param parent: Parent of this Actor Object
#         :type parent: carla_ros_bridge.Parent
#         """
#         super(AckermannControlVehicle, self).__init__(carla_actor=carla_actor, parent=parent)
#         # Adapt values
#         self.info.restrictions.min_accel = parent.get_param('ego_vehicle').get('min_accel', 1.)
#
#         # Clipping the pedal in both directions to the same range using the usual lower
#         # Border: the max_accel to ensure the the pedal target is in symmetry to zero
#         self.info.restrictions.max_pedal = min(
#             self.info.restrictions.max_accel, self.info.restrictions.max_decel)
#
#         # PID controller
#         # the controller has to run with the simulation time, not with real-time
#         #
#         # To prevent "float division by zero" within PID controller initialize it with
#         # a previous point in time (the error happens because the time doesn't
#         # change between initialization and first call, therefore dt is 0)
#         sys.modules['simple_pid.PID']._current_time = (  # pylint: disable=protected-access
#             lambda: AckermannControlVehicle.get_current_ros_time(self).to_sec() - 0.1)
#
#         # we might want to use a PID controller to reach the final target speed
#         self.speed_controller = PID(Kp=0.0,
#                                     Ki=0.0,
#                                     Kd=0.0,
#                                     sample_time=0.05,
#                                     output_limits=(-1., 1.))
#         self.accel_controller = PID(Kp=0.0,
#                                     Ki=0.0,
#                                     Kd=0.0,
#                                     sample_time=0.05,
#                                     output_limits=(-1, 1))
#
#         # use the correct time for further calculations
#         sys.modules['simple_pid.PID']._current_time = (  # pylint: disable=protected-access
#             lambda: AckermannControlVehicle.get_current_ros_time(self).to_sec())
#
#         self.reconfigure_server = Server(
#             EgoVehicleControlParameterConfig,
#             namespace=self.topic_name(),
#             callback=(lambda config, level: AckermannControlVehicle.reconfigure_pid_parameters(
#                 self, config, level)))
#
#         # ROS subscriber
#         # the lastMsgReceived is updated within the callback
#         # (it's used to stop updating the carla client control if no new messages were received)
#         self.lastMsgReceived = datetime.datetime(datetime.MINYEAR, 1, 1)
#         self.control_subscriber = rospy.Subscriber(
#             self.topic_name() + "/ackermann_cmd", AckermannDrive, self.ackermann_command_updated)
#
#     def destroy(self):
#         """
#         Override Function to destroy this object.
#         Terminate ROS subscription on AckermannDrive commands.
#         Finish the PID controllers.
#         Destroy the reconfiguration server.
#         Finally forward the call to the super class
#         :return:
#         """
#         rospy.logdebug("Destroy EgoVehicleAckermannControl(id={})".format(self.get_ID()))
#         self.control_subscriber = None
#         self.speed_controller = None
#         self.accel_controller = None
#         # first cleanup the server (otherwise leaves a mess behind ;-)
#         self.reconfigure_server.set_service.shutdown()
#         self.reconfigure_server = None
#         super(AckermannControlVehicle, self).destroy()
#
#     def reconfigure_pid_parameters(self, ego_vehicle_control_parameter, dummy_level):
#         """
#         Callback Function used for dynamic reconfigure call to set the PID parameters
#         :param ego_vehicle_control_parameter:
#         :type ego_vehicle_control_parameter: carla_ros_bridge.cfg.EgoVehicleControlParameterConfig
#         :param dummy_level:
#         :return:
#         """
#         rospy.loginfo("Reconfigure Request:  "
#                       "speed ({speed_Kp}, {speed_Ki}, {speed_Kd}),"
#                       "accel ({accel_Kp}, {accel_Ki}, {accel_Kd}),"
#                       "".format(**ego_vehicle_control_parameter))
#         self.speed_controller.tunings = (
#             ego_vehicle_control_parameter['speed_Kp'],
#             ego_vehicle_control_parameter['speed_Ki'],
#             ego_vehicle_control_parameter['speed_Kd']
#         )
#         self.accel_controller.tunings = (
#             ego_vehicle_control_parameter['accel_Kp'],
#             ego_vehicle_control_parameter['accel_Ki'],
#             ego_vehicle_control_parameter['accel_Kd']
#         )
#         return ego_vehicle_control_parameter
#
#     def ackermann_command_updated(self, ros_ackermann_drive):
#         """
#         Function used to converta Ackermann Drive msg into a Carla Control msg
#         This function gets called whenever a ROS message is received via
#         '/carla/ego_vehicle/vehicle_control_cmd' topic.
#         The received ROS message is converted into carla.VehicleControl command and
#         sent to CARLA.
#         This brigde is not responsible for any restrictions on velocity or steering.
#         It's just forwarding the ROS input to CARLA
#         :param ros_ackermann_drive: Current Ackermann Control Input
#         :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
#         :return:
#         """
#         # Update the last reception timestamp
#         self.lastMsgReceived = datetime.datetime.now()
#         # Set target values
#         self.set_target_steering_angle(ros_ackermann_drive.steering_angle)
#         self.set_target_speed(ros_ackermann_drive.speed)
#         self.set_target_accel(ros_ackermann_drive.acceleration)
#         self.set_target_jerk(ros_ackermann_drive.jerk)
#
#     def set_target_steering_angle(self, target_steering_angle):
#         """
#         Function used to set target steering angle
#         :param target_steering_angle: Input Steering Angle
#         :type target_steering_angle: float64
#         :return:
#         """
#         self.info.target.steering_angle = -target_steering_angle
#         if abs(self.info.target.steering_angle) > self.info.restrictions.max_steering_angle:
#             rospy.logerr("Max steering angle reached, clipping value")
#             self.info.target.steering_angle = numpy.clip(
#                 self.info.target.steering_angle,
#                 -self.info.restrictions.max_steering_angle,
#                 self.info.restrictions.max_steering_angle
#             )
#
#     def set_target_speed(self, target_speed):
#         """
#         Function used to set target speed
#         :param target_speed: Input Target Speed
#         :type target_speed: float64
#         :return:
#         """
#         if abs(target_speed) > self.info.restrictions.max_speed:
#             rospy.logerr("Max speed reached, clipping value")
#             self.info.target.speed = numpy.clip(
#                 target_speed, -self.info.restrictions.max_speed, self.info.restrictions.max_speed)
#         else:
#             self.info.target.speed = target_speed
#         self.info.target.speed_abs = abs(self.info.target.speed)
#
#     def set_target_accel(self, target_accel):
#         """
#         Function used to set target acceleration
#         :param target_accel: Input Target Acceleration
#         :type target_accel: float64
#         :return:
#         """
#         epsilon = 0.00001
#         # if speed is set to zero, then use max decel value
#         if self.info.target.speed_abs < epsilon:
#             self.info.target.accel = -self.info.restrictions.max_decel
#         else:
#             self.info.target.accel = numpy.clip(
#                 target_accel, -self.info.restrictions.max_decel, self.info.restrictions.max_accel)
#
#     def set_target_jerk(self, target_jerk):
#         """
#         Function used to set target jerk
#         :param target_jerk: Input Target Jerl
#         :return:
#         """
#         self.info.target.jerk = target_jerk
#
#     def vehicle_control_cycle(self):
#         """
#         Function used to perform a vehicle control cycle and sends out carla.VehicleControl message
#         :return:
#         """
#         # perform actual control
#         # perform actual control
#         self.control_steering()
#         self.control_stop_and_reverse()
#         self.run_speed_control_loop()
#         self.run_accel_control_loop()
#         if not self.info.output.hand_brake:
#             self.update_drive_vehicle_control_command()
#         # apply control command to CARLA
#         # (only if a control message was received in the last 3 seconds. This is a workaround
#         # for rospy.subscriber.get_num_connections() not working in Ubuntu 16.04)
#         if (self.lastMsgReceived + datetime.timedelta(0, 3)) > datetime.datetime.now():
#             self.apply_control()
#
#     def control_steering(self):
#         """
#         Function used for basic steering control
#         :return:
#         """
#         self.info.output.steer = self.info.target.steering_angle / self.info.restrictions.max_steering_angle
#
#     def control_stop_and_reverse(self):
#         """
#         Function used to handle stop and switching to reverse gear
#         :return:
#         """
#         # from this velocity on it is allowed to switch to reverse gear
#         standing_still_epsilon = 0.1
#         # from this velocity on hand brake is turned on
#         full_stop_epsilon = 0.00001
#         # auto-control of hand-brake and reverse gear
#         self.info.output.hand_brake = False
#         if self.info.current.speed_abs < standing_still_epsilon:
#             # standing still, change of driving direction allowed
#             self.info.state.status = "standing"
#             if self.info.target.speed < 0:
#                 if not self.info.output.reverse:
#                     rospy.loginfo(
#                         "VehicleControl: Change of driving direction to reverse")
#                     self.info.output.reverse = True
#             elif self.info.target.speed > 0:
#                 if self.info.output.reverse:
#                     rospy.loginfo(
#                         "VehicleControl: Change of driving direction to forward")
#                     self.info.output.reverse = False
#             if self.info.target.speed_abs < full_stop_epsilon:
#                 self.info.state.status = "full stop"
#                 self.info.state.speed_control_accel_target = 0.
#                 self.info.state.accel_control_pedal_target = 0.
#                 self.set_target_speed(0.)
#                 self.info.current.speed = 0.
#                 self.info.current.speed_abs = 0.
#                 self.info.current.accel = 0.
#                 self.info.output.hand_brake = True
#                 self.info.output.brake = 1.0
#                 self.info.output.throttle = 0.0
#         elif numpy.sign(self.info.current.speed) * numpy.sign(self.info.target.speed) == -1:
#             # request for change of driving direction
#             # first we have to come to full stop before changing driving
#             # direction
#             rospy.loginfo("VehicleControl: Request change of driving direction."
#                           " v_current={} v_desired={}"
#                           " Set desired speed to 0".format(self.info.current.speed,
#                                                            self.info.target.speed))
#             self.set_target_speed(0.)
#
#     def run_speed_control_loop(self):
#         """
#         Function used to run the PID control loop for the speed.
#         The speed control is only activated if desired acceleration is moderate
#         otherwise we try to follow the desired acceleration values
#         Reasoning behind:
#         An autonomous vehicle calculates a trajectory including position and velocities.
#         The ackermann drive is derived directly from that trajectory.
#         The acceleration and jerk values provided by the ackermann drive command
#         reflect already the speed profile of the trajectory.
#         It makes no sense to try to mimick this a-priori knowledge by the speed PID
#         controller.
#         => The speed controller is mainly responsible to keep the speed.
#         On expected speed changes, the speed control loop is disabled
#         :return:
#         """
#         epsilon = 0.00001
#         target_accel_abs = abs(self.info.target.accel)
#         if target_accel_abs < self.info.restrictions.min_accel:
#             if self.info.state.speed_control_activation_count < 5:
#                 self.info.state.speed_control_activation_count += 1
#         else:
#             if self.info.state.speed_control_activation_count > 0:
#                 self.info.state.speed_control_activation_count -= 1
#         # set the auto_mode of the controller accordingly
#         self.speed_controller.auto_mode = self.info.state.speed_control_activation_count >= 5
#
#         if self.speed_controller.auto_mode:
#             self.speed_controller.setpoint = self.info.target.speed_abs
#             self.info.state.speed_control_accel_delta = self.speed_controller(
#                 self.info.current.speed_abs)
#             # clipping borders
#             clipping_lower_border = -target_accel_abs
#             clipping_upper_border = target_accel_abs
#             # per definition of ackermann drive: if zero, then use max value
#             if target_accel_abs < epsilon:
#                 clipping_lower_border = -self.info.restrictions.max_decel
#                 clipping_upper_border = self.info.restrictions.max_accel
#             self.info.state.speed_control_accel_target = numpy.clip(
#                 self.info.state.speed_control_accel_target +
#                 self.info.state.speed_control_accel_delta,
#                 clipping_lower_border, clipping_upper_border)
#         else:
#             self.info.state.speed_control_accel_delta = 0.
#             self.info.state.speed_control_accel_target = self.info.target.accel
#
#     def update_drive_vehicle_control_command(self):
#         """
#         Function used to apply the current speed_control_target value to throttle/brake commands
#         :return:
#         """
#         # the driving impedance moves the 'zero' acceleration border
#         # Interpretation: To reach a zero acceleration the throttle has to pushed
#         # down for a certain amount
#         self.info.state.throttle_lower_border = phys.get_vehicle_driving_impedance_acceleration(
#             self.carla_actor, self.info.output.reverse)
#         # the engine lay off acceleration defines the size of the coasting area
#         # Interpretation: The engine already prforms braking on its own;
#         #  therefore pushing the brake is not required for small decelerations
#         self.info.state.brake_upper_border = self.info.state.throttle_lower_border + \
#             phys.get_vehicle_lay_off_engine_acceleration(self.carla_actor)
#
#         if self.info.state.accel_control_pedal_target > self.info.state.throttle_lower_border:
#             self.info.state.status = "accelerating"
#             self.info.output.brake = 0.0
#             # the value has to be normed to max_pedal
#             # be aware: is not required to take throttle_lower_border into the scaling factor,
#             # because that border is in reality a shift of the coordinate system
#             # the global maximum acceleration can practically not be reached anymore because of
#             # driving impedance
#             self.info.output.throttle = (
#                 (self.info.state.accel_control_pedal_target -
#                  self.info.state.throttle_lower_border) /
#                 abs(self.info.restrictions.max_pedal))
#         elif self.info.state.accel_control_pedal_target > self.info.state.brake_upper_border:
#             self.info.state.status = "coasting"
#             # no control required
#             self.info.output.brake = 0.0
#             self.info.output.throttle = 0.0
#         else:
#             self.info.state.status = "braking"
#             # braking required
#             self.info.output.brake = (
#                 (self.info.state.brake_upper_border -
#                  self.info.state.accel_control_pedal_target) /
#                 abs(self.info.restrictions.max_pedal))
#             self.info.output.throttle = 0.0
#
#         # finally clip the final control output (should actually never happen)
#         self.info.output.brake = numpy.clip(
#             self.info.output.brake, 0., 1.)
#         self.info.output.throttle = numpy.clip(
#             self.info.output.throttle, 0., 1.)
