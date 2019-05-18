//
// Created by pedro on 15-05-2019.
//

/*
 * Copyright (c) 2019 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */

/* System Includes */
#include "PclRecorder.h"
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <sstream>

PclRecorder::PclRecorder()
{
    tfListener = new tf2_ros::TransformListener(tf_buffer_);

    if (mkdir("/home/pedro/catkin_ws/src/ros_bridge/pointclouds", 0777) == -1) {
        ROS_WARN("Could not create directory!");
    }

    // Create a ROS subscriber for the input point clouds
    sub_lidar_front = nh.subscribe("/carla/ego_vehicle/lidar/front/point_cloud", 1000000, &PclRecorder::callback_lidar_front, this);
    sub_lidar_left = nh.subscribe("/carla/ego_vehicle/lidar/left/point_cloud", 1000000, &PclRecorder::callback_lidar_left, this);
    sub_lidar_right = nh.subscribe("/carla/ego_vehicle/lidar/right/point_cloud", 1000000, &PclRecorder::callback_lidar_right, this);
}

void PclRecorder::callback_lidar_front(const pcl::PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

    std::stringstream ss;
    ss << "/home/pedro/catkin_ws/src/ros_bridge/pointclouds/front_capture" << cloud->header.stamp << ".pcd";

    ROS_INFO ("Received %d data points from front LIDAR sensor. Storing in %s",
              (int)cloud->width * cloud->height,
              ss.str().c_str());

    Eigen::Affine3d transform;
    try {
        transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

        pcl::PointCloud<pcl::PointXYZ> pclCloud;
        pcl::fromPCLPointCloud2(*cloud, pclCloud);

        pcl::PointCloud<pcl::PointXYZ> transformedCloud;
        pcl::transformPointCloud (pclCloud, transformedCloud, transform);

        pcl::PCDWriter writer;
        writer.writeBinary(ss.str(), transformedCloud);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}

void PclRecorder::callback_lidar_left(const pcl::PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

    std::stringstream ss;
    ss << "/home/pedro/catkin_ws/src/ros_bridge/pointclouds/left_capture" << cloud->header.stamp << ".pcd";

    ROS_INFO ("Received %d data points from left LIDAR sensor. Storing in %s",
              (int)cloud->width * cloud->height,
              ss.str().c_str());

    Eigen::Affine3d transform;
    try {
        transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

        pcl::PointCloud<pcl::PointXYZ> pclCloud;
        pcl::fromPCLPointCloud2(*cloud, pclCloud);

        pcl::PointCloud<pcl::PointXYZ> transformedCloud;
        pcl::transformPointCloud (pclCloud, transformedCloud, transform);

        pcl::PCDWriter writer;
        writer.writeBinary(ss.str(), transformedCloud);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}

void PclRecorder::callback_lidar_right(const pcl::PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

    std::stringstream ss;
    ss << "/home/pedro/catkin_ws/src/ros_bridge/pointclouds/right_capture" << cloud->header.stamp << ".pcd";

    ROS_INFO ("Received %d data points from right LIDAR sensor. Storing in %s",
              (int)cloud->width * cloud->height,
              ss.str().c_str());

    Eigen::Affine3d transform;
    try {
        transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

        pcl::PointCloud<pcl::PointXYZ> pclCloud;
        pcl::fromPCLPointCloud2(*cloud, pclCloud);

        pcl::PointCloud<pcl::PointXYZ> transformedCloud;
        pcl::transformPointCloud (pclCloud, transformedCloud, transform);

        pcl::PCDWriter writer;
        writer.writeBinary(ss.str(), transformedCloud);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}

