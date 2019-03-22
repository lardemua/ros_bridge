#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# This node subscribes to the VehicleControl topic.
# - If data is received, the ROS bridge gets started.
# - If no data is received for a specific time, the ROS bridge gets stopped.
#

"""
Node used to start/stop the ros bridge package if control commands are received
"""

# ------------------------
#   IMPORTS
# ------------------------
import threading
import datetime
import rospy
import roslaunch
from carla_ros_bridge.msg import CarlaEgoVehicleControl     # pylint: disable=no-name-in-module,import-error


class RosBridgeLifeCycle(object):
    """
    Class that starts/stops the ROS Bridge package if control commands are received
    """
    def __init__(self):
        """
        Constructor for RosBridgeLifeCycle class
        """
        rospy.init_node('ros_bridge_lifecycle')
        self.node = roslaunch.core.Node("carla_ros_bridge", "client.py")
        self.timeout_seconds = int(rospy.get_param('/carla/ros_bridge/timeout', '5')) # default: 5 seconds
        self.lastMsgReceived = datetime.datetime(datetime.MINYEAR, 1, 1)
        self.lock = threading.Lock()    # prevents start/stop process while previous actions have not yet been finished
        self.is_active = False
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.process = None
        self.launch = None
        roslaunch.configure_logging(self.uuid)
        rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, self.callback)

    def __del__(self):
        """
        Destructor for RosBridgeLifeCycle class
        :return:
        """
        if self.is_active:
            if self.process and self.process.is_alive():
                self.process.stop()
            if self.launch:
                self.launch.stop()

    def callback(self, _):
        """
        Callback Function used to store timestamp on received messages
        :param _:
        :return:
        """
        self.lastMsgReceived = datetime.datetime.now()

    def run(self):
        """
        Function used to run execution loop
        :return:
        """
        while not rospy.core.is_shutdown():
            if self.lock.acquire(False):
                if not self.is_active and \
                        (self.lastMsgReceived + datetime.timedelta(0, self.timeout_seconds)) > datetime.datetime.now():
                    self.is_active = True
                    rospy.loginfo("Starting ROS Bridge")
                    self.launch = roslaunch.scriptapi.ROSLaunch()
                    self.launch.start()
                    self.process = self.launch.launch(self.node)
                if self.is_active and \
                        (self.lastMsgReceived + datetime.timedelta(0, self.timeout_seconds)) < datetime.datetime.now():
                    self.is_active = False
                    rospy.loginfo("Stopping ROS Bridge")
                    self.process.stop()
                    self.launch.stop()
                    self.process = None
                    self.launch = None
                self.lock.release()
            rospy.rostime.wallsleep(0.5)


def main():
    """
    Main Function for carla_ros_bridge_lifecycle module
    :return:
    """
    lifecycle = RosBridgeLifeCycle()
    try:
        lifecycle.run()
    finally:
        del lifecycle
        rospy.loginfo("Done")


if __name__ == '__main__':
    main()

