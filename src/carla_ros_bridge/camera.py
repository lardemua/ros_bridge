#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Class to handle Carla camera sensors
"""

# ------------------------
#   IMPORTS
# ------------------------
from abc import abstractmethod
import math
import numpy
import rospy
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
import carla
from carla_ros_bridge.sensor import Sensor
import carla_ros_bridge.transforms as trans


class Camera(Sensor):
    """
    Sensor Implementation details for the Camera objects
    """

    # global cv bridge to convert image between OpenCV and ROS
    cv_bridge = CvBridge()

    @staticmethod
    def create_actor(carla_actor, parent):
        """
        Static Factory Method to create camera actors
        :param carla_actor: Carla Camera Actor Object
        :type carla_actor: carla.Camera
        :param parent: Parent of new Traffic Actor
        :type parent: carla_ros_bridge.Parent
        :return: Created Camera Actor
        :rtype: carla_ros_bridge.Camera or derived type
        """
        if carla_actor.type_id.startswith("sensor.camera.rgb"):
            return RGB_Camera(carla_actor=carla_actor, parent=parent)
        elif carla_actor.type_id.startswith("sensor.camera.depth"):
            return Depth_Camera(carla_actor=carla_actor, parent=parent)
        elif carla_actor.type_id.startswith("sensor.camera.semantic_segmentation"):
            return Semantic_Segmentation_Camera(carla_actor=carla_actor, parent=parent)
        else:
            return Camera(carla_actor=carla_actor, parent=parent)

    def __init__(self, carla_actor, parent, topic_prefix=True):
        """
        Constructor for Camera Class
        :param carla_actor: Carla Actor Object
        :type carla_actor: carla.Actor
        :param parent: Parent of this Actor Object
        :param topic_prefix: The Topic Prefix to be used for this actor
        :type topic_prefix: string
        """
        if topic_prefix is None:
            topic_prefix = 'camera'
            super(Camera, self).__init__(carla_actor=carla_actor, parent=parent, topic_prefix=topic_prefix)
        if self.__class__.__name__ == "Camera":
            rospy.logwarn("Created Unsupported Camera Actor"
                          "(id={}, parent_id={}, type={}, attributes={})".format(self.get_ID(), self.get_parent_ID(),
                                                                                 self.carla_actor.type_id, self.carla_actor.attributes))
        else:
            self._build_camera_info()

    def _build_camera_info(self):
        """
        Private Function used to compute camera info
        Considers that the camera info does not change over time
        :return:
        """
        camera_info = CameraInfo()
        # store camera info without the header
        camera_info.header = None
        camera_info.width = int(self.carla_actor.attributes['image_size_x'])
        camera_info.height = int(self.carla_actor.attributes['image_size_y'])
        camera_info.distortion_model = 'plumb_bob'
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (2.0 * math.tan(float(self.carla_actor.attributes['fov']) * math.pi / 360.0))
        fy = fx
        camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info.D = [0, 0, 0, 0, 0]
        camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
        self._camera_info = camera_info

    def sensor_data_updated(self, carla_image):
        """
        Override Function used to transform the received carla image data into a ROS image message
        :param carla_image: Carla Image Object
        :type carla_image: carla.Image
        :return:
        """
        if (carla_image.height != self._camera_info.height) or (carla_image.width != self._camera_info.width):
            rospy.logerr("Camera{} received image not matching configuration".format(self.topic_name()))

        image_data_array, encoding = self.get_carla_image_data_array(carla_image=carla_image)
        img_msg = Camera.cv_bridge.cv2_to_imgmsg(image_data_array, encoding=encoding)
        # the camera data is in respect to the camera's own frame
        img_msg.header = self.get_msg_header(use_parent_frame=False)

        cam_info = self._camera_info
        cam_info.header = img_msg.header

        self.publish_ros_message(self.topic_name() + '/camera_info', cam_info)
        self.publish_ros_message(self.topic_name() + '/' + self.get_image_topic_name(), img_msg)

    def get_tf_msg(self):
        """
        Override Function used to modify the tf messages sent by this camera.
        The camera transformation has to bve altered to look at the same axis as the OpenCV projection in order to get
        easy depth cloud for RGBD cameras.
        :return: Filled tf Message
        :rtype: geometry_msgs.msg.TransformStamped
        """
        tf_msg = super(Camera, self).get_tf_msg()
        rotation = tf_msg.transform.rotation
        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        quaternion_swap = tf.transformations.quaternion_from_matrix([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
        quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_swap)

        tf_msg.transform.rotation = trans.numpy_quaternion_to_ros_quaternion(quaternion)
        return tf_msg

    @abstractmethod
    def get_carla_image_data_array(self, carla_image):
        """
        Virtual Function used to convert the Carla Image to a numpy data array
        as input for the cv_bridge.cv2_to_imgmsg() function
        :param carla_image: Carla Image Object
        :type carla_image: carla.Image
        :return: Tuple (numpy data array containing the image transformation, encoding)
        :rtype: Tuple (numpy.ndarray, string)
        """
        raise NotImplementedError("This function has to be re-implemented by derived classes")

    @abstractmethod
    def get_image_topic_name(self):
        """
        Virtual Function used to provide the actual image topic name
        :return: Image Topic Name
        :rtype: string
        """
        raise NotImplementedError("This function has to be re-implemented by derived classes")


class RGB_Camera(Camera):
    """
       Camera Implementation Details for RGB Camera
    """

    def __init__(self, carla_actor, parent, topic_prefix=None):
        """
        Constructor of the RGB_Camera object
        :param carla_actor: Carla Actor Object
        :type carla_actor: carla.Actor
        :param parent: Parent of this Actor Object Node
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: Topic prefix to be used for this actor
        :type topic_prefix: string
        """
        if topic_prefix is None:
            topic_prefix = 'camera/rgb'
        super(RGB_Camera, self).__init__(carla_actor=carla_actor, parent=parent, topic_prefix=topic_prefix)

    def get_carla_image_data_array(self, carla_image):
        """
        Override Function used to convert the Carla Image to a numpy data array
        as input for the cv_bridge.cv2_to_imgmsg() function
        The RGB Camera provides a 4-channel int8 color format(BGRA).
        :param carla_image: Carla Image Object
        :type carla_image: carla.Image
        :return: Tuple (numpy data array containing the image information, encoding)
        :rtype: Tuple (numpy.ndarray, string)
        """
        carla_image_data_array = numpy.ndarray(shape=(carla_image.height, carla_image.width, 4),
                                               dtype=numpy.uint8,
                                               buffer=carla_image.raw_data)
        return carla_image_data_array, 'bgra8'

    def get_image_topic_name(self):
        """
        Virtual Function used to provide the actual image topic name
        :return: Image Topic Name
        :rtype: string
        """
        return "image_color"


class Depth_Camera(Camera):
    """
    Camera Implementation Details for Depth Camera
    """

    def __init__(self, carla_actor, parent, topic_prefix=None):
        """
        Constructor of the Depth Camera Object
        :param carla_actor: Carla Actor Object
        :type carla_actor: carla.Actor
        :param parent: Parent of this Actor Object Node
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: Topic prefix to be used for this actor
        :type topic_prefix: string
        """
        if topic_prefix is None:
            topic_prefix = 'camera/depth'
        super(Depth_Camera, self).__init__(carla_actor=carla_actor, parent=parent, topic_prefix=topic_prefix)

    def get_carla_image_data_array(self, carla_image):
        """
        Override Function used to convert the Carla Image to a numpy data array
        as input for the cv_bridge.cv2_to_imgmsg() function
        The RGB Camera provides a 4-channel int8 color format(BGRA).
        :param carla_image: Carla Image Object
        :type carla_image: carla.Image
        :return: Tuple (numpy data array containing the image information, encoding)
        :rtype: Tuple (numpy.ndarray, string)
        """
        # color conversion within C++ code is broken, when transforming a
        #  4-channel uint8 color pixel into a 1-channel float32 grayscale pixel
        # therefore, we do it on our own here
        #
        # @todo: After fixing https://github.com/carla-simulator/carla/issues/1041
        # the final code in here should look like:
        #
        # carla_image.convert(carla.ColorConverter.Depth)
        #
        # carla_image_data_array = numpy.ndarray(
        #    shape=(carla_image.height, carla_image.width, 1),
        #    dtype=numpy.float32, buffer=carla_image.raw_data)
        #
        bgra_image = numpy.ndarray(shape=(carla_image.height, carla_image.width, 4),
                                   dtype=numpy.uint8,
                                   buffer=carla_image.raw_data)

        # Apply (R + G * 256 + B * 256 * 256) / (256**3 - 1) * 1000
        # according to the documentation:
        # https://carla.readthedocs.io/en/latest/cameras_and_sensors/#camera-depth-map
        scales = numpy.array([65536.0, 256.0, 1.0, 0]) / (256 ** 3 - 1) * 1000
        depth_image = numpy.dot(bgra_image, scales).astype(numpy.float32)

        # actually we want encoding '32FC1'
        # which is automatically selected by cv bridge with passthrough
        return depth_image, 'passthrough'

    def get_image_topic_name(self):
        """
        Virtual Function used to provide the actual image topic name
        :return: Image Topic Name
        :rtype: string
        """
        return "image_depth"


class Semantic_Segmentation_Camera(Camera):
    """
        Camera Implementation Details for Semantic Segmentation Camera
    """

    def __init__(self, carla_actor, parent, topic_prefix=None):
        """
        Constructor of the Depth Camera Object
        :param carla_actor: Carla Actor Object
        :type carla_actor: carla.Actor
        :param parent: Parent of this Actor Object Node
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: Topic prefix to be used for this actor
        :type topic_prefix: string
        """
        if topic_prefix is None:
            topic_prefix = 'camera/semantic_segmentation'
        super(Semantic_Segmentation_Camera, self).__init__(carla_actor=carla_actor, parent=parent, topic_prefix=topic_prefix)

    def get_carla_image_data_array(self, carla_image):
        """
        Override Function used to convert the Carla Image to a numpy data array
        as input for the cv_bridge.cv2_to_imgmsg() function
        The RGB Camera provides a 4-channel int8 color format(BGRA).
        :param carla_image: Carla Image Object
        :type carla_image: carla.Image
        :return: Tuple (numpy data array containing the image information, encoding)
        :rtype: Tuple (numpy.ndarray, string)
        """
        carla_image.convert(carla.ColorConverter.CityScapesPalette)
        carla_image_data_array = numpy.ndarray(shape=(carla_image.height, carla_image.width, 4),
                                               dtype=numpy.uint8,
                                               buffer=carla_image.raw_data)
        return carla_image_data_array, 'bgra8'

    def get_image_topic_name(self):
        """
        Virtual Function used to provide the actual image topic name
        :return: Image Topic Name
        :rtype: string
        """
        return "image_segmentation"



