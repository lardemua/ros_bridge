# Carla ROS Bridge Lifecycle

The node `carla_ros_bridge_lifecycle` is used to control the lifecycle of the ROS bridge package.
This node starts and stops the ros bridge according to certain control commands.

It uses certain commands to start and stop the ROS bridge package according to a certain timestamp(5 seconds).

## Startup

If the lifecycle publisher requires some Carla PythonAPI functionality that is not part of the python egg-file, you
have to extend your PYTHONPATH.

    export PYTHONPATH=$PYTHONPATH:<path-to-carla>/PythonAPI/carla-<carla_version_and_arch>.egg:<path-to-carla>/PythonAPI/

To run it:

    roslaunch carla_ros_bridge_lifecycle lifecycle.launch


