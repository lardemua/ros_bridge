# ros_bridge -> ROS bridge for Carla Simulator
- ROS Bridge Driver for Carla Simulator package : https://github.com/carla-simulator/carla <br /> 
- Based on the code provided by the "ros-bridge" repository of CARLA: https://github.com/carla-simulator/ros-bridge <br />
- The goal of this ROS package is to provide a simple ROS bridge for CARLA simulator in order to be used for the ATLASCAR project of Aveiro University. <br />
- This repository is directly related to my Master's Thesis - "Semi-Automatic Labelling for Atlascar using Adaptive Perception" : https://silvamfpedro.github.io/ <br />
<br />
__Important Note:__

This documentation is for CARLA versions *newer* than 0.9.4.

# Setup



## Create a catkin workspace and install carla_ros_bridge package



First, clone or download the carla_ros_bridge, for example into



   ~/carla_ros_bridge



### Create the catkin workspace:



    mkdir -p ~/ros/catkin_ws_for_carla/src

    cd ~/ros/catkin_ws_for_carla/src

    ln -s ~/carla_ros_bridge .

    source /opt/ros/kinetic/setup.bash

    catkin_make

    source ~/ros/catkin_ws_for_carla/devel/setup.bash



For more information about configuring a ROS environment see

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment



## Install the CARLA Python API



    export PYTHONPATH=$PYTHONPATH:<path/to/carla/>/PythonAPI/<your_egg_file>



Please note that you have to put in the complete path to the egg-file including

the egg-file itself. Please use the one, that is supported by your Python version.

Depending on the type of CARLA (pre-build, or build from source), the egg files

are typically located either directly in the PythonAPI folder or in PythonAPI/dist.



Check the installation is successfull by trying to import carla from python:



    python -c 'import carla;print("Success")'



You should see the Success message without any errors.



# Start the ROS bridge



First run the simulator (see carla documentation: http://carla.readthedocs.io/en/latest/)



    ./CarlaUE4.sh -windowed -ResX=320 -ResY=240 -benchmark -fps=10





Wait for the message:



    Waiting for the client to connect...



Then start the ros bridge:



    source ~/ros/catkin_ws/devel/setup.bash

    roslaunch carla_ros_bridge client.launch



To start the ros bridge with rviz use:



    source ~/ros/catkin_ws/devel/setup.bash

    roslaunch carla_ros_bridge client_with_rviz.launch



You can setup the ros bridge configuration [carla_ros_bridge/config/settings.yaml](carla_ros_bridge/config/settings.yaml).



As we have not spawned any vehicle and have not added any sensors in our carla world there would not be any stream of data yet.



You can make use of the CARLA Python API script manual_control.py.

```

cd <path/to/carla/>

python manual_control.py --rolename=ego_vehicle

```

This spawns a carla client with role_name='ego_vehicle'. 

If the rolename is within the list specified by ROS parameter `/carla/ego_vehicle/rolename`, the client is interpreted as an controllable ego vehicle and all relevant ROS topics are created.



To simulate traffic, you can spawn automatically moving vehicles by using spawn_npc.py from CARLA Python API.


