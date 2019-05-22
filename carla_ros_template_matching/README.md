# Carla ROS Template Matching



The ```carla_ros_template_matching``` package is used to convert images from a ROS Topic to an OpenCV image and implements the Template Matching algorithm to find template objects in the image sequences by running the following command:



    roslaunch carla_ros_template_matching template_matching.launch



# Carla ROS Template Matching Bounding Boxes

The ```carla_ros_template_matching``` package can be used to draw the 2D bounding boxes around the objects present in the CARLA world that match the template object according to their location in the image and converts the results into an OpenCV image.

The template objects are cropped from the image and the corresponding bouding box is designed via OpenCV mouse events. 

![rviz setup](../assets/images/carla_template_matching_01.png?style=centerme "box_01")
![rviz setup](../assets/images/carla_template_matching_02.png?style=centerme "box_02")


# Carla ROS Template Matching Datasets

The position,location, orientation, rotation of these objects as well as a label for these objects can be recorded via a json file `data.json`.

The json format is defined like this:

    { 
        "vehicle" = [
            {
              "id": "<NAME>",
              "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, # pose of the sensor, relative to the vehicle
              <ADDITIONAL-VEHICLE-ATTRIBUTES>
            },
            ...
        ]
    }
    
img[src$="centerme"] {
  display:block;
  margin: 0 auto;
}