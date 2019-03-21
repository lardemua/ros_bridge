#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Control Package for Carla Ego Vehicle using AckermannDrive Messages
"""

# ------------------------
#   IMPORTS
# ------------------------
import sys
import datetime
import numpy
import rospy
from simple_pid import PID
from dynamic_reconfigure.server import Server
from ackermann_msgs.msg import AckermannDrive
from carla_ros_bridge.msg import CarlaEgoVehicleStatus              # pylint: disable=no-name-in-module,import-error
from carla_ros_bridge.msg import CarlaEgoVehicleControl             # pylint: disable=no-name-in-module,import-error
from carla_ros_bridge.msg import CarlaEgoVehicleInfo                # pylint: disable=no-name-in-module,import-error
from carla_ackermann_control.msg import EgoVehicleControlInfo       # pylint: disable=no-name-in-module,import-error
import carla_control_physics as phys                                # pylint: disable=no-name-in-module,import-error


class CarlaAckermannControl(object):
    """
    Class used to convert ackermann_drive messages to Carla VehicleCommand with a PID controller
    """

    def __init__(self):
        """
        Constructor for CarlaAckermannContrl Class
        """
        self.control_loop_rate = rospy.Rate(10)     #10Hz
        self.lastAckermannMsgReceived = datetime.datetime(datetime.MINYEAR, 1, 1)
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.vehicle_info = CarlaEgoVehicleInfo()
        # control info
        self.info = EgoVehicleControlInfo()
        # set initial maximum values
        self.vehicle_info_updated(self.vehicle_info)
        # target values
        self.info.target.steering_angle = 0.
        self.info.target.speed = 0.
        self.info.target.speed_abs = 0.
        self.info.target.accel = 0.
        self.info.target.jerk = 0.
        # current values
        self.info.current.time_sec = rospy.get_rostime().to_sec()
        self.info.current.speed = 0.
        self.info.current.speed_abs = 0.
        self.info.current.accel = 0.
        # control values
        self.info.status.status = 'n/a'
        self.info.status.speed_control_activation_count = 0
        self.info.status.speed_control_accel_delta = 0.
        self.info.status.speed_control_accel_target = 0.
        self.info.status.accel_control_pedal_delta = 0.
        self.info.status.accel_control_pedal_target = 0.
        self.info.status.brake_upper_border = 0.
        self.info.status.throttle_lower_border = 0.
        # control output
        self.info.output.throttle = 0.
        self.info.output.brake = 1.0
        self.info.output.steer = 0.
        self.info.output.reverse = False
        self.info.output.hand_brake = True

        # PID controller
        # the controller has to run with the simulation time, not with real-time
        #
        # To prevent "float division by zero" within PID controller initialize it with
        # a previous point in time (the error happens because the time doesn't
        # change between initialization and first call, therefore dt is 0)
        sys.modules['simple_pid.PID']._current_time = (lambda: rospy.get_rostime().to_sec() - 0.1)  # pylint: disable=protected-access

        # we might want to use a PID controller to reach the final target speed
        self.speed_controller = PID(Kp=0.0, Ki=0.0, Kd=0.0,
                                    sample_time=0.05,
                                    output_limits=(-1., 1.))
        self.accel_controller = PID(Kp=0.0, Ki=0.0, Kd=0.0,
                                    sample_time=0.05,
                                    output_limits=(-1, 1))

        # use the correct time for further calculations
        sys.modules['simple_pid.PID']._current_time = (lambda: rospy.get_rostime().to_sec()) # pylint: disable=protected-access

        self.reconfigure_server = Server(EgoVehicleControlParameterConfig, namespace="/carla/ackermann_control",
                                         callback=(lambda config, level: CarlaAckermannControl.reconfigure_pid_parameters(self, config, level)))
        # ackermann drive commands
        self.control_subscriber = rospy.Subscriber("/carla/ego_vehicle/ackermann_cmd",
                                                   AckermannDrive, self.ackermann_command_updated)
        # current status of the vehicle
        self.vehicle_status_subscriber = rospy.Subscriber("/carla/ego_vehicle/vehicle_status",
                                                          CarlaEgoVehicleStatus, self.vehicle_status_updated)
        # vehicle info
        self.vehicle_info_subscriber = rospy.Subscriber("/carla/ego_vehicle/vehicle_info",
                                                        CarlaEgoVehicleInfo, self.vehicle_info_updated)
        # to send command to carla
        self.carla_control_publisher = rospy.Publisher("/carla/ego_vehicle/vehicle_control_cmd",
                                                       CarlaEgoVehicleControl, queue_size=1)
        # report controller info
        self.control_info_publisher = rospy.Publisher("/carla/ackermann_control/control_info",
                                                      EgoVehicleControlInfo, queue_size=1)

    def destroy(self):
        """
        Override Function used to destroy this object.
        Terminate ROS subscription on AckermannDrive commands.
        Finish PID controllers.
        Destroy the reconfiguration server
        :return:
        """
        self.control_subscriber = None
        self.speed_controller = None
        self.accel_controller = None
        # first cleanup the server
        self.reconfigure_server.set_service.shutdown()
        self.reconfigure_server = None

    def reconfigure_PID_parameters(self, ego_vehicle_control_parameter, dummy_level):
        """
        Callback Function used for dynamic reconfiguration call to set PID parameters
        :param ego_vehicle_control_parameter:
        :type ego_vehicle_control_parameter: carla_ackermann_control.cfg.EgoVehicleControlParameterConfig
        :param dummy_level:
        :return:
        """
        rospy.loginfo("Reconfigure Request:  "
                      "speed ({speed_Kp}, {speed_Ki}, {speed_Kd}),"
                      "accel ({accel_Kp}, {accel_Ki}, {accel_Kd}),"
                      "".format(**ego_vehicle_control_parameter))
        self.speed_controller.tunings = (
                                            ego_vehicle_control_parameter['speed_Kp'],
                                            ego_vehicle_control_parameter['speed_Ki'],
                                            ego_vehicle_control_parameter['speed_Kd']
                                        )
        self.accel_controller.tunings = (
                                            ego_vehicle_control_parameter['accel_Kp'],
                                            ego_vehicle_control_parameter['accel_Ki'],
                                            ego_vehicle_control_parameter['accel_Kd']
        )
        return ego_vehicle_control_parameter

