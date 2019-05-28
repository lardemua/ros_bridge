//
// Created by pedro on 27-05-2019.
//

/*
 * Copyright (c) 2019 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */

/* System Includes */
#include "PclViewer.h"
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

//OpenCV Includes
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//PCL Includes
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

cv::Mat intrinsic_matrix;
cv::Mat display_matrix;

PclViewer::PclViewer()
{
    tfListener = new tf2_ros::TransformListener(tf_buffer_);

    intrinsic_matrix = cv::Mat (3, 3, CV_32FC1);
    // Read camera parameters file
    cv::FileStorage file_storage("/home/pedro/catkin_ws/src/ros_bridge/carla_ros_pcl_opencv/config/cameraParams.xml", cv::FileStorage::READ );
    if ( !file_storage.isOpened () )
    {
        std::cout << "Failed to open cameraParams.xml" << std::endl;
        return;
    }
    file_storage [ "intrinsic_matrix" ] >> intrinsic_matrix;
    file_storage.release ();


    // Create a ROS subscriber for the input point cloud
    sub_lidar_front = nh.subscribe("/carla/ego_vehicle/lidar/front/point_cloud", 1000000, &PclViewer::callback_lidar_front, this);
//    sub_lidar_left = nh.subscribe("/carla/ego_vehicle/lidar/left/point_cloud", 1000000, &PclViewer::callback_lidar_left, this);
//    sub_lidar_right = nh.subscribe("/carla/ego_vehicle/lidar/right/point_cloud", 1000000, &PclViewer::callback_lidar_right, this);


}

void PclViewer::callback_lidar_front(const pcl::PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

//    std::stringstream ss;
//    ss << "/home/pedro/catkin_ws/src/ros_bridge/pointclouds/capture" << cloud->header.stamp << ".pcd";

    ROS_INFO ("Received %d data points.",
              (int)cloud->width * cloud->height);

    Eigen::Affine3d transform;
//    Eigen::Matrix<int,int,int> cv_transform_matrix;
//    Eigen::Map<Eigen::Matrix3f> camera_transform(intrinsic_matrix.data());
    try {
        transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

        pcl::PointCloud<pcl::PointXYZ> pclCloud;
        pcl::fromPCLPointCloud2(*cloud, pclCloud);

        pcl::PointCloud<pcl::PointXYZ> transformedCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloudPtr;
        pcl::transformPointCloud (pclCloud, transformedCloud, transform);
        transformedCloudPtr = transformedCloud.makeShared();
        // Apply camera transform

        // TO DO : Implement OpenCV cloud viewer for front LIDAR point cloud

        display_matrix = cv::Mat(transformedCloudPtr->points.size(), 1,  CV_32FC3);
//        std::vector<cv::Point3d> test;
//
//        for(size_t i = 0; i < transformedCloudPtr->points.size(); i++){
//            test.push_back(cv::Point3d(transformedCloudPtr->points[i].x, transformedCloudPtr->points[i].y, transformedCloudPtr->points[i].z));
//        }
//
//        //display_matrix = cv::Mat(test.size(), test.at(0).size(), CV_64FC1);
//        for(int i=0; i<display_matrix.rows; ++i) {
//            for (int j = 0; j < display_matrix.cols; ++j) {
//                display_matrix.at<cv::Point3d>(i, j) = test.at(i);
//            }
//        }
        int count = 0;
        for (int v = 0; v < display_matrix.rows; ++v)
        {
            for (int u = 0; u < display_matrix.cols; ++u)
            {
                display_matrix.at<float>(v, u) = transformedCloudPtr->points.at(count++).z * 1000;

            }
        }

        display_matrix.convertTo(display_matrix,CV_8U);
        // Display results
        cv::namedWindow("Front LIDAR Display Window", cv::WINDOW_AUTOSIZE);
        ROS_INFO ("Showing LIDAR results.");
        cv::imshow("Front LIDAR Display Window", display_matrix);


    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}


void PclViewer::callback_lidar_left(const pcl::PCLPointCloud2::ConstPtr& cloud)
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

        // TO DO: Implement OpenCV cloud viewer for left LIDAR point cloud

    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}

void PclViewer::callback_lidar_right(const pcl::PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }


    ROS_INFO ("Received %d data points.",
              (int)cloud->width * cloud->height);

    Eigen::Affine3d transform;
    try {
        transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

        pcl::PointCloud<pcl::PointXYZ> pclCloud;
        pcl::fromPCLPointCloud2(*cloud, pclCloud);

        pcl::PointCloud<pcl::PointXYZ> transformedCloud;
        pcl::transformPointCloud (pclCloud, transformedCloud, transform);

        // TO DO : Implement OpenCV cloud viewer for right LIDAR point cloud

    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}
