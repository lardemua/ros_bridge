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
#include "PclConverter.h"
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <cstdlib>

// OpenCV Includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// PCL Includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/recognition/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

using namespace std;
using namespace boost;
using namespace ros;
using namespace pcl;

PclConverter::PclConverter()
{
    tfListener = new tf2_ros::TransformListener(tf_buffer_);

//    if (mkdir("/tmp/pcl_capture", 0777) == -1) {
//        ROS_WARN("Could not create directory!");
//    }

    // Create a ROS subscriber for the input point cloud
    sub = nh.subscribe("/carla/ego_vehicle/lidar/lidar1/point_cloud", 1000000, &PclConverter::callback, this);
}

void PclConverter::callback(const PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

//    std::stringstream ss;
//    ss << "/tmp/pcl_capture/capture" << cloud->header.stamp << ".pcd";

    ROS_INFO ("Received %d data points.", (int)cloud->width * cloud->height);

    Eigen::Affine3d transform;
    try {
        transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

        PointCloud<PointXYZ> pclCloud;
        fromPCLPointCloud2(*cloud, pclCloud);

        PointCloud<PointXYZ> transformedCloud;
        transformPointCloud (pclCloud, transformedCloud, transform);

//        pcl::PCDWriter writer;
//        writer.writeBinary(ss.str(), transformedCloud);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}
