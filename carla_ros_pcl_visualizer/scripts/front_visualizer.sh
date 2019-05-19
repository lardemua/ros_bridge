#!/usr/bin/env bash
# Move to front LIDAR cloud directory
echo "Moving to front LIDAR cloud directory"
echo " "
cd /home/pedro/catkin_ws/src/ros_bridge/pointclouds/front/
sleep 5
echo " "

# Concatenate point cloud files
echo "Concatenating point cloud files"
echo " "
pcl_concatenate_points_pcd *.pcd
sleep 5
echo " "

# Filter duplicate results
echo "Filtering duplicated results"
echo " "
pcl_voxel_grid -leaf 0.1,0.1,0.1 output.pcd map.pcd
sleep 5
echo " "


# Show result
echo "Showing Point Cloud Map Result for Front Lidar Sensor"
echo " "
pcl_viewer map.pcd
sleep 5
echo " "