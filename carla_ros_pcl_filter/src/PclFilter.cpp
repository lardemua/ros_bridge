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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
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

//    if (mkdir("/tmp/pcl_capture", 0777) == -1) {
//        ROS_WARN("Could not create directory!");
//    }

    // Create a ROS subscriber for the input point clouds
    sub_lidar_front = nh.subscribe("/carla/ego_vehicle/lidar/front/point_cloud", 1000000, &PclFilter::callback_lidar_front, this);
    sub_lidar_left = nh.subscribe("/carla/ego_vehicle/lidar/left/point_cloud", 1000000, &PclFilter::callback_lidar_left, this);
    sub_lidar_right = nh.subscribe("/carla/ego_vehicle/lidar/right/point_cloud", 1000000, &PclFilter::callback_lidar_right, this);


    // Create a ROS publisher for the PCL point clouds and advertise ROS publisher
    pub_lidar_front = (boost::shared_ptr<Publisher>) new Publisher;
    (*pub_lidar_front) = nh.advertise<sensor_msgs::PointCloud2>("/carla/ego_vehicle/lidar/front/new_point_cloud", 0);
    pub_lidar_left = (boost::shared_ptr<Publisher>) new Publisher;
    (*pub_lidar_left) = nh.advertise<sensor_msgs::PointCloud2>("/carla/ego_vehicle/lidar/left/new_point_cloud", 0);
    pub_lidar_right = (boost::shared_ptr<Publisher>) new Publisher;
    (*pub_lidar_right) = nh.advertise<sensor_msgs::PointCloud2>("/carla/ego_vehicle/lidar/right/new_point_cloud", 0);

}

void PclFilter::callback_lidar_front(const PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

//    std::stringstream ss;
//    ss << "/tmp/pcl_capture/capture" << cloud->header.stamp << ".pcd";

    ROS_INFO ("Received %d data points from front lidar.", (int)cloud->width * cloud->height);

    Eigen::Affine3d transform;
    try {
        transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

        PointCloud<PointXYZ> pclCloud;
        fromPCLPointCloud2(*cloud, pclCloud);

        PointCloud<PointXYZ> transformedCloud;
        transformPointCloud (pclCloud, transformedCloud, transform);

        PointCloud<PointXYZ>::Ptr transformedCloudPtr (new PointCloud<PointXYZ>);
        transformedCloudPtr = transformedCloud.makeShared();

        PointCloud<PointXYZ>::Ptr filteredCloud (new PointCloud<PointXYZ>);

        PointCloud<PointXYZ> cloud_out;
        PointCloud<PointXYZ>::Ptr cloud_out_ptr (new PointCloud<PointXYZ>);
        sensor_msgs::PointCloud2 cloud_out_msg;


//        pcl::visualization::PCLVisualizer visualizer("Cloud Visualizer");

//        pcl::PCDWriter writer;
//        writer.writeBinary(ss.str(), transformedCloud);

        // Create the filtering object: downsample the dataset using a leaf size
        pcl::VoxelGrid<PointXYZ> avg;
        avg.setInputCloud(transformedCloudPtr);
        avg.setLeafSize(0.25f, 0.25f, 0.25f);
        avg.filter(*filteredCloud);

        // searchPoint
        PointXYZ searchPoint = filteredCloud->at(0);

        //result from radiusSearch()
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        //kdTree
        pcl::KdTreeFLANN<PointXYZ> kdtree;
        kdtree.setInputCloud (filteredCloud);
        kdtree.setSortedResults(true);

        if ( kdtree.radiusSearch (searchPoint, 100, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            //delete every point in target
            for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
            {
                //is this the way to erase correctly???
                cloud_out.push_back(filteredCloud->points[pointIdxRadiusSearch[j]]);
            }
        }

        cloud_out_ptr = cloud_out.makeShared();
        // Convert PCL cloud to PointCloud2 message
        pcl::toROSMsg(*cloud_out_ptr.get(),cloud_out_msg );

        // Publish PointCloud2 message in another topic
        cloud_out_msg.header.frame_id = cloud->header.frame_id; // get header frame ID
//        cloud_out_msg.header.stamp = cloud->header.stamp;       // get header time stamp
        pub_lidar_front->publish (cloud_out_msg);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}

void PclFilter::callback_lidar_left(const PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

//    std::stringstream ss;
//    ss << "/tmp/pcl_capture/capture" << cloud->header.stamp << ".pcd";

    ROS_INFO ("Received %d data points from left lidar.", (int)cloud->width * cloud->height);

    Eigen::Affine3d transform;
    try {
        transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

        PointCloud<PointXYZ> pclCloud;
        fromPCLPointCloud2(*cloud, pclCloud);

        PointCloud<PointXYZ> transformedCloud;
        transformPointCloud (pclCloud, transformedCloud, transform);

        PointCloud<PointXYZ>::Ptr transformedCloudPtr (new PointCloud<PointXYZ>);
        transformedCloudPtr = transformedCloud.makeShared();

        PointCloud<PointXYZ>::Ptr filteredCloud (new PointCloud<PointXYZ>);

        PointCloud<PointXYZ> cloud_out;
        PointCloud<PointXYZ>::Ptr cloud_out_ptr (new PointCloud<PointXYZ>);
        sensor_msgs::PointCloud2 cloud_out_msg;


//        pcl::visualization::PCLVisualizer visualizer("Cloud Visualizer");

//        pcl::PCDWriter writer;
//        writer.writeBinary(ss.str(), transformedCloud);

        // Create the filtering object: downsample the dataset using a leaf size
        pcl::VoxelGrid<PointXYZ> avg;
        avg.setInputCloud(transformedCloudPtr);
        avg.setLeafSize(0.25f, 0.25f, 0.25f);
        avg.filter(*filteredCloud);

        // searchPoint
        PointXYZ searchPoint = filteredCloud->at(0);

        //result from radiusSearch()
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        //kdTree
        pcl::KdTreeFLANN<PointXYZ> kdtree;
        kdtree.setInputCloud (filteredCloud);
        kdtree.setSortedResults(true);

        if ( kdtree.radiusSearch (searchPoint, 100, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            //delete every point in target
            for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
            {
                //is this the way to erase correctly???
                cloud_out.push_back(filteredCloud->points[pointIdxRadiusSearch[j]]);
            }
        }

        cloud_out_ptr = cloud_out.makeShared();
        // Convert PCL cloud to PointCloud2 message
        pcl::toROSMsg(*cloud_out_ptr.get(),cloud_out_msg );

        // Publish PointCloud2 message in another topic
        cloud_out_msg.header.frame_id = cloud->header.frame_id; // get header frame ID
//        cloud_out_msg.header.stamp = cloud->header.stamp;       // get header time stamp
        pub_lidar_left->publish (cloud_out_msg);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}

void PclFilter::callback_lidar_right(const PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

//    std::stringstream ss;
//    ss << "/tmp/pcl_capture/capture" << cloud->header.stamp << ".pcd";

    ROS_INFO ("Received %d data points from right lidar.", (int)cloud->width * cloud->height);

    Eigen::Affine3d transform;
    try {
        transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

        PointCloud<PointXYZ> pclCloud;
        fromPCLPointCloud2(*cloud, pclCloud);

        PointCloud<PointXYZ> transformedCloud;
        transformPointCloud (pclCloud, transformedCloud, transform);

        PointCloud<PointXYZ>::Ptr transformedCloudPtr (new PointCloud<PointXYZ>);
        transformedCloudPtr = transformedCloud.makeShared();

        PointCloud<PointXYZ>::Ptr filteredCloud (new PointCloud<PointXYZ>);

        PointCloud<PointXYZ> cloud_out;
        PointCloud<PointXYZ>::Ptr cloud_out_ptr (new PointCloud<PointXYZ>);
        sensor_msgs::PointCloud2 cloud_out_msg;


//        pcl::visualization::PCLVisualizer visualizer("Cloud Visualizer");

//        pcl::PCDWriter writer;
//        writer.writeBinary(ss.str(), transformedCloud);

        // Create the filtering object: downsample the dataset using a leaf size
        pcl::VoxelGrid<PointXYZ> avg;
        avg.setInputCloud(transformedCloudPtr);
        avg.setLeafSize(0.25f, 0.25f, 0.25f);
        avg.filter(*filteredCloud);

        // searchPoint
        PointXYZ searchPoint = filteredCloud->at(0);

        //result from radiusSearch()
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        //kdTree
        pcl::KdTreeFLANN<PointXYZ> kdtree;
        kdtree.setInputCloud (filteredCloud);
        kdtree.setSortedResults(true);

        if ( kdtree.radiusSearch (searchPoint, 100, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            //delete every point in target
            for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
            {
                //is this the way to erase correctly???
                cloud_out.push_back(filteredCloud->points[pointIdxRadiusSearch[j]]);
            }
        }

        cloud_out_ptr = cloud_out.makeShared();
        // Convert PCL cloud to PointCloud2 message
        pcl::toROSMsg(*cloud_out_ptr.get(),cloud_out_msg );

        // Publish PointCloud2 message in another topic
        cloud_out_msg.header.frame_id = cloud->header.frame_id; // get header frame ID
//        cloud_out_msg.header.stamp = cloud->header.stamp;       // get header time stamp
        pub_lidar_right->publish (cloud_out_msg);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}