# ROS bridge for CARLA simulator

This ROS package aims at providing a simple ROS bridge for CARLA simulator.

# Setup

## Create a catkin workspace and install carla_ros_bridge package

    #setup folder structure
    mkdir -p ~/carla-ros-bridge/catkin_ws/src
    cd ~/carla-ros-bridge
    git clone https://github.com/carla-simulator/ros-bridge.git
    cd catkin_ws/src
    ln -s ../../ros-bridge
    source /opt/ros/kinetic/setup.bash
    cd ..

    #install required ros-dependencies
    rosdep update
    rosdep install --from-path .

    #build
    catkin_make

For more information about configuring a ROS environment see
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

# Start the ROS bridge

First run the simulator (see carla documentation: http://carla.readthedocs.io/en/latest/)

    ./CarlaUE4.sh -windowed -ResX=320 -ResY=240 -benchmark -fps=10


Wait for the message:

    Waiting for the client to connect...

Then start the ros bridge (choose one option):

    export PYTHONPATH=$PYTHONPATH:<path/to/carla/>/PythonAPI/<your_egg_file>
    source ~/carla-ros-bridge/catkin_ws/devel/setup.bash

    # Option 1: start the ros bridge
    roslaunch carla_ros_bridge client.launch

    # Option 2: start the ros bridge together with RVIZ
    roslaunch carla_ros_bridge client_with_rviz.launch
    
    # Option 3: start the ros bridge together with rqt_bag
    roslaunch carla_ros_bridge client_with_rqt.launch

    # Option 4: start the ros bridge together with an example ego vehicle
    roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
    
    # Option 5: start the ros bridge in challenge mode
    roslaunch carla_ros_bridge challenge.launch

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