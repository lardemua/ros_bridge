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
    sub = nh.subscribe("/carla/ego_vehicle/lidar/lidar1/point_cloud", 1000000, &PclVisualizer::callback, this);

    // Create a PCL viewer for the output point cloud
//    viewer = new pcl::visualization::PCLVisualizer("3d viewer");
//    viewer->setBackgroundColor(0.33f, 0.33f, 0.33f);
//    viewer->initCameraParameters();
//    viewer->setCameraPosition(0.0f, 0.0f, 0.0f,
//                              0.0f, 0.0f, 1.0f,
//                              0.0f, -1.0f, 0.0f);
}

void PclVisualizer::callback(const pcl::PCLPointCloud2::ConstPtr& cloud)
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

//        pcl::PCDWriter writer;
//        writer.writeBinary(ss.str(), transformedCloud);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}
