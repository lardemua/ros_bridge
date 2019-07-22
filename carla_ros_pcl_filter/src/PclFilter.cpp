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
#include "PclFilter.h"
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <math.h>
#include <cstdlib>

// OpenCV Includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// PCL Includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/recognition/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace boost;
using namespace ros;
using namespace pcl;

PclFilter::PclFilter()
{
    tfListener = new tf2_ros::TransformListener(tf_buffer_);

    // Create a ROS subscriber for the input point clouds
    sub_lidar_front = nh.subscribe("/carla/ego_vehicle/lidar/front/point_cloud", 1000000, &PclFilter::callback_lidar_front_spherical, this);
    sub_lidar_left = nh.subscribe("/carla/ego_vehicle/lidar/left/point_cloud", 1000000, &PclFilter::callback_lidar_left_spherical, this);
    sub_lidar_right = nh.subscribe("/carla/ego_vehicle/lidar/right/point_cloud", 1000000, &PclFilter::callback_lidar_right_spherical, this);

    // Create a ROS publisher for the PCL point clouds and advertise ROS publisher
    pub_lidar_front = (boost::shared_ptr<Publisher>) new Publisher;
    (*pub_lidar_front) = nh.advertise<sensor_msgs::PointCloud2>("/carla/ego_vehicle/lidar/front/new_point_cloud", 0);
    pub_lidar_left = (boost::shared_ptr<Publisher>) new Publisher;
    (*pub_lidar_left) = nh.advertise<sensor_msgs::PointCloud2>("/carla/ego_vehicle/lidar/left/new_point_cloud", 0);
    pub_lidar_right = (boost::shared_ptr<Publisher>) new Publisher;
    (*pub_lidar_right) = nh.advertise<sensor_msgs::PointCloud2>("/carla/ego_vehicle/lidar/right/new_point_cloud", 0);

}

void PclFilter::callback_lidar_front_spherical(const PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

    ROS_INFO ("Received %d data points from front lidar.", (int)cloud->width * cloud->height);

    try {
        PointCloud<PointXYZ> pclCloud;
        fromPCLPointCloud2(*cloud, pclCloud);

        PointCloud<PointXYZ>::Ptr pclCloudPtr (new PointCloud<PointXYZ>);
        pclCloudPtr = pclCloud.makeShared();

        PointCloud<PointXYZ>::Ptr outputCloud (new PointCloud<PointXYZ>);
//        outputCloud->points.resize(transformedCloudPtr->points.size());
        sensor_msgs::PointCloud2 cloud_out_msg;

        PointXYZ pt;
        ROS_INFO("Calculating spherical coordinates");
        float rho = 0.0;
        float theta = 0.0;
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        for(size_t i = 0; i < pclCloudPtr->points.size(); i++){
            x = pclCloudPtr->points[i].x;
            y = pclCloudPtr->points[i].y;
            z = pclCloudPtr->points[i].z;
            rho = sqrt(x*x + y*y + z*z);
            theta = atan2(y, x);
            if ( (theta >= -M_PI/4 && theta <= M_PI/4)){
                pt.x = x;
                pt.y = y;
                pt.z = z;
                outputCloud->points.push_back(pt);
            };

        }
        // Convert PCL cloud to PointCloud2 message
        pcl::toROSMsg(*outputCloud.get(),cloud_out_msg );
        cloud_out_msg.header.frame_id = cloud->header.frame_id; // get header frame ID
        pub_lidar_front->publish (cloud_out_msg);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}

void PclFilter::callback_lidar_left_spherical(const PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

    ROS_INFO ("Received %d data points from left lidar.", (int)cloud->width * cloud->height);

    try {
        PointCloud<PointXYZ> pclCloud;
        fromPCLPointCloud2(*cloud, pclCloud);

        PointCloud<PointXYZ>::Ptr pclCloudPtr (new PointCloud<PointXYZ>);
        pclCloudPtr = pclCloud.makeShared();

        PointCloud<PointXYZ>::Ptr outputCloud (new PointCloud<PointXYZ>);
//        outputCloud->points.resize(transformedCloudPtr->points.size());
        sensor_msgs::PointCloud2 cloud_out_msg;

        PointXYZ pt;
        ROS_INFO("Calculating spherical coordinates");
        float rho = 0.0;
        float theta = 0.0;
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        for(size_t i = 0; i < pclCloudPtr->points.size(); i++){
            x = pclCloudPtr->points[i].x;
            y = pclCloudPtr->points[i].y;
            z = pclCloudPtr->points[i].z;
            rho = sqrt(x*x + y*y + z*z);
            theta = atan2(y, x);
            if ( (theta >= 0 && theta <= M_PI)){
                pt.x = x;
                pt.y = y;
                pt.z = z;
                outputCloud->points.push_back(pt);
            };

        }
        // Convert PCL cloud to PointCloud2 message
        pcl::toROSMsg(*outputCloud.get(),cloud_out_msg );
        cloud_out_msg.header.frame_id = cloud->header.frame_id; // get header frame ID
        pub_lidar_left->publish (cloud_out_msg);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}

void PclFilter::callback_lidar_right_spherical(const PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

    ROS_INFO ("Received %d data points from right lidar.", (int)cloud->width * cloud->height);

    try {
        PointCloud<PointXYZ> pclCloud;
        fromPCLPointCloud2(*cloud, pclCloud);

        PointCloud<PointXYZ>::Ptr pclCloudPtr (new PointCloud<PointXYZ>);
        pclCloudPtr = pclCloud.makeShared();

        PointCloud<PointXYZ>::Ptr outputCloud (new PointCloud<PointXYZ>);
//        outputCloud->points.resize(transformedCloudPtr->points.size());
        sensor_msgs::PointCloud2 cloud_out_msg;

        PointXYZ pt;
        ROS_INFO("Calculating spherical coordinates");
        float rho = 0.0;
        float theta = 0.0;
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        for(size_t i = 0; i < pclCloudPtr->points.size(); i++){
            x = pclCloudPtr->points[i].x;
            y = pclCloudPtr->points[i].y;
            z = pclCloudPtr->points[i].z;
            rho = sqrt(x*x + y*y + z*z);
            theta = atan2(y, x);
            if ( (theta >= -M_PI && theta <= 0)){
                pt.x = x;
                pt.y = y;
                pt.z = z;
                outputCloud->points.push_back(pt);
            };

        }
        // Convert PCL cloud to PointCloud2 message
        pcl::toROSMsg(*outputCloud.get(),cloud_out_msg );
        cloud_out_msg.header.frame_id = cloud->header.frame_id; // get header frame ID
        pub_lidar_right->publish (cloud_out_msg);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}
