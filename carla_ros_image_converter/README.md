# Carla ROS Image Converter



The ```carla_ros_image_converter``` package is used to convert images from a ROS Topic to an OpenCV image by running the following command:



    roslaunch carla_ros_image_converter image_converter.launch



# Carla ROS Bounding Boxes

The ```carla_ros_image_converter``` package can be used to draw the bounding boxes around the objects present in the CARLA world according to their location/rotation and convert the results into an OpenCV image.

![rviz setup](../assets/images/carla_bounding_box_01.png "box_01")
![rviz setup](../assets/images/carla_bounding_box_02.png "box_02")


# Carla ROS Datasets