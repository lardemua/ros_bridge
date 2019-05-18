//
// Created by pedro on 16-05-2019.
//

/*
 * Copyright (c) 2019 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */

/* System Includes */
#include "PclVisualizer.h"
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <sstream>

PclVisualizer::PclVisualizer()
{
    tfListener = new tf2_ros::TransformListener(tf_buffer_);

//    if (mkdir("/home/pedro/catkin_ws/src/ros_bridge/pointclouds", 0777) == -1) {
//        ROS_WARN("Could not create directory!");
//    }

    // Create a ROS subscriber for the input point cloud
    sub_lidar_front = nh.subscribe("/carla/ego_vehicle/lidar/front/point_cloud", 1000000, &PclVisualizer::callback_lidar_front, this);
    sub_lidar_left = nh.subscribe("/carla/ego_vehicle/lidar/left/point_cloud", 1000000, &PclVisualizer::callback_lidar_left, this);
    sub_lidar_right = nh.subscribe("/carla/ego_vehicle/lidar/right/point_cloud", 1000000, &PclVisualizer::callback_lidar_right, this);

    // Create a PCL viewer for the front LIDAR point cloud
//    viewer_lidar_front = new pcl::visualization::PCLVisualizer("Front Viewer");
//    viewer_lidar_front->setBackgroundColor(0.33f, 0.33f, 0.33f);
//    viewer_lidar_front->initCameraParameters();
//    viewer_lidar_front->setCameraPosition(0.0f, 0.0f, 0.0f,
//                              0.0f, 0.0f, 1.0f,
//                              0.0f, -1.0f, 0.0f);
    // Create a PCL viewer for the left LIDAR point cloud
//    viewer_lidar_left = new pcl::visualization::PCLVisualizer("Left Viewer");
//    viewer_lidar_left->setBackgroundColor(0.33f, 0.33f, 0.33f);
//    viewer_lidar_left->initCameraParameters();
//    viewer_lidar_left->setCameraPosition(0.0f, 0.0f, 0.0f,
//                              0.0f, 0.0f, 1.0f,
//                              0.0f, -1.0f, 0.0f);
    // Create a PCL viewer for the right LIDAR point cloud
//    viewer_lidar_right = new pcl::visualization::PCLVisualizer("Right Viewer");
//    viewer_lidar_right->setBackgroundColor(0.33f, 0.33f, 0.33f);
//    viewer_lidar_right->initCameraParameters();
//    viewer_lidar_right->setCameraPosition(0.0f, 0.0f, 0.0f,
//                              0.0f, 0.0f, 1.0f,
//                              0.0f, -1.0f, 0.0f);
}

void PclVisualizer::callback_lidar_front(const pcl::PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

//    std::stringstream ss;
//    ss << "/home/pedro/catkin_ws/src/ros_bridge/pointclouds/capture" << cloud->header.stamp << ".pcd";

    ROS_INFO ("Received %d data points.",
              (int)cloud->width * cloud->height);

    Eigen::Affine3d transform;
    try {
        transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

        pcl::PointCloud<pcl::PointXYZ> pclCloud;
        pcl::fromPCLPointCloud2(*cloud, pclCloud);

        pcl::PointCloud<pcl::PointXYZ> transformedCloud;
        pcl::transformPointCloud (pclCloud, transformedCloud, transform);

        // TO DO : Implement cloud visualizer for front LIDAR point cloud
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}


void PclVisualizer::callback_lidar_left(const pcl::PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

//    std::stringstream ss;
//    ss << "/home/pedro/catkin_ws/src/ros_bridge/pointclouds/capture" << cloud->header.stamp << ".pcd";

    ROS_INFO ("Received %d data points.",
              (int)cloud->width * cloud->height);

    Eigen::Affine3d transform;
    try {
        transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

        pcl::PointCloud<pcl::PointXYZ> pclCloud;
        pcl::fromPCLPointCloud2(*cloud, pclCloud);

        pcl::PointCloud<pcl::PointXYZ> transformedCloud;
        pcl::transformPointCloud (pclCloud, transformedCloud, transform);

        // TO DO: Implement cloud visualizer for left LIDAR point cloud
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}

void PclVisualizer::callback_lidar_right(const pcl::PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

//    std::stringstream ss;
//    ss << "/home/pedro/catkin_ws/src/ros_bridge/pointclouds/capture" << cloud->header.stamp << ".pcd";

    ROS_INFO ("Received %d data points.",
              (int)cloud->width * cloud->height);

    Eigen::Affine3d transform;
    try {
        transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

        pcl::PointCloud<pcl::PointXYZ> pclCloud;
        pcl::fromPCLPointCloud2(*cloud, pclCloud);

        pcl::PointCloud<pcl::PointXYZ> transformedCloud;
        pcl::transformPointCloud (pclCloud, transformedCloud, transform);

        // TO DO : Implement cloud visualizer for right LIDAR point cloud
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}
