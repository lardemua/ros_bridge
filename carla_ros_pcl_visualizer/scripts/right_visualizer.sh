#!/usr/bin/env bash
# Move to front LIDAR cloud directory
cd /home/pedro/catkin_ws/src/ros_bridge/pointclouds/right/

# Concatenate point cloud files
pcl_concatenate_ponts_pcd *.pcd

# Filter duplicate results
pcl_voxel_grid -leaf 0.1,0.1,0.1 output.pcd map.pcd

# Show result
pcl_viewer map.pcd