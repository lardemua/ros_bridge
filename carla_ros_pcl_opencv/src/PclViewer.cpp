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
cv::Mat distortion_matrix;
cv::Mat rectification_matrix;
cv::Mat projection_matrix;

cv::Mat display_matrix_front;
cv::Mat display_matrix_left;
cv::Mat display_matrix_right;

using namespace Eigen;

PclViewer::PclViewer()
{
    tfListener = new tf2_ros::TransformListener(tf_buffer_);

    //Initialize camera matrixes
    intrinsic_matrix = cv::Mat (3, 3, CV_32FC1);
    distortion_matrix = cv::Mat (1, 5, CV_32FC1);
    rectification_matrix = cv::Mat (3, 3, CV_32FC1);
    projection_matrix = cv::Mat(3, 4, CV_32FC1);

    // Read camera parameters file
    cv::FileStorage file_storage("/home/pedro/catkin_ws/src/ros_bridge/carla_ros_pcl_opencv/config/cameraParams.xml", cv::FileStorage::READ );
    if ( !file_storage.isOpened () )
    {
        std::cout << "Failed to open cameraParams.xml" << std::endl;
        return;
    }
    file_storage [ "intrinsic_matrix" ] >> intrinsic_matrix;
    file_storage [ "distortion_matrix" ] >> distortion_matrix;
    file_storage [ "rectification_matrix" ] >> rectification_matrix;
    file_storage [ "projection_matrix" ] >> projection_matrix;
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
    try {
        transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

        pcl::PointCloud<pcl::PointXYZ> pclCloud;
        pcl::fromPCLPointCloud2(*cloud, pclCloud);

        pcl::PointCloud<pcl::PointXYZ> transformedCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloudPtr;
        pcl::transformPointCloud (pclCloud, transformedCloud, transform);
        transformedCloudPtr = transformedCloud.makeShared();

        // TO DO : Implement OpenCV cloud viewer for front LIDAR point cloud

//        display_matrix_front = cv::Mat(transformedCloudPtr->points.size(), 1,  CV_32FC1);
        display_matrix_front = cv::Mat(transformedCloudPtr->height, transformedCloudPtr->width, CV_8UC3);
        std::vector<cv::Point2d> points_2d;
        std::vector<cv::Point2d> pixels_2d;

        float u;
        float v;
        float y;
        float x;
        for(size_t i = 0; i < transformedCloudPtr->points.size(); i++){
//            sample.push_back(cv::Point2d(transformedCloudPtr, 7));
//              points_2d.push_back(cv::Point2d(transformedCloudPtr->points[i].x,transformedCloudPtr->points[i].y));
              x = transformedCloudPtr->points[i].x / transformedCloudPtr->points[i].z;
              y = transformedCloudPtr->points[i].y / transformedCloudPtr->points[i].z;
              u = intrinsic_matrix.at<double>(0,0)*x + intrinsic_matrix.at<double>(2,0);
              v = intrinsic_matrix.at<double>(1,1)*y + intrinsic_matrix.at<double>(2,1);
//              pixels_2d.push_back(cv::Point2d(u,v));
              cv::line(display_matrix_front,
                     cv::Point2d(transformedCloudPtr->points[i].x, transformedCloudPtr->points[i].y),
                     cv::Point2d(transformedCloudPtr->points[i+1].x, transformedCloudPtr->points[i+1].y),
                     cv::Scalar(255,255,255), 1, 8 , CV_8S);
        }

        // Display results
        cv::namedWindow("Front LIDAR Display Window", cv::WINDOW_AUTOSIZE);
        cv::resizeWindow("Front LIDAR Display Window", 1024, 768);
        ROS_INFO ("Showing LIDAR results.");
        cv::imshow("Front LIDAR Display Window", display_matrix_front);
        cv::waitKey(0); // wait for a keystroke in the window;

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
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloudPtr;
        pcl::transformPointCloud (pclCloud, transformedCloud, transform);
        transformedCloudPtr = transformedCloud.makeShared();

        // TO DO: Implement OpenCV cloud viewer for left LIDAR point cloud
        display_matrix_left = cv::Mat(transformedCloudPtr->points.size(), 1,  CV_32FC1);
        std::vector<cv::Point2d> points_2d;
        std::vector<cv::Point2d> pixels_2d;

        float u;
        float v;
        float y;
        float x;
        for(size_t i = 0; i < transformedCloudPtr->points.size(); i++){
//            sample.push_back(cv::Point2d(transformedCloudPtr, 7));
//              points_2d.push_back(cv::Point2d(transformedCloudPtr->points[i].x,transformedCloudPtr->points[i].y));
            x = transformedCloudPtr->points[i].x / transformedCloudPtr->points[i].z;
            y = transformedCloudPtr->points[i].y / transformedCloudPtr->points[i].z;
            u = intrinsic_matrix.at<double>(0,0)*x + intrinsic_matrix.at<double>(2,0);
            v = intrinsic_matrix.at<double>(1,1)*y + intrinsic_matrix.at<double>(2,1);
//              pixels_2d.push_back(cv::Point2d(u,v));
            cv::line(display_matrix_left,
                     cv::Point2d(transformedCloudPtr->points[i].x, transformedCloudPtr->points[i].y),
                     cv::Point2d(transformedCloudPtr->points[i+1].x, transformedCloudPtr->points[i+1].y),
                     cv::Scalar(255,255,255), 1, 8 , CV_8S);
        }

        // Display results
        cv::namedWindow("Left LIDAR Display Window", cv::WINDOW_AUTOSIZE);
        cv::resizeWindow("Left LIDAR Display Window", 1024, 768);
        ROS_INFO ("Showing LIDAR results.");
        cv::imshow("Left LIDAR Display Window", display_matrix_left);
        cv::waitKey(0); // wait for a keystroke in the window;

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
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloudPtr;
        pcl::transformPointCloud (pclCloud, transformedCloud, transform);
        transformedCloudPtr = transformedCloud.makeShared();

        // TO DO : Implement OpenCV cloud viewer for right LIDAR point cloud
        display_matrix_right = cv::Mat(transformedCloudPtr->points.size(), 1,  CV_32FC1);
        std::vector<cv::Point2d> points_2d;
        std::vector<cv::Point2d> pixels_2d;

        float u;
        float v;
        float y;
        float x;
        for(size_t i = 0; i < transformedCloudPtr->points.size(); i++){
//            sample.push_back(cv::Point2d(transformedCloudPtr, 7));
//              points_2d.push_back(cv::Point2d(transformedCloudPtr->points[i].x,transformedCloudPtr->points[i].y));
            x = transformedCloudPtr->points[i].x / transformedCloudPtr->points[i].z;
            y = transformedCloudPtr->points[i].y / transformedCloudPtr->points[i].z;
            u = intrinsic_matrix.at<double>(0,0)*x + intrinsic_matrix.at<double>(2,0);
            v = intrinsic_matrix.at<double>(1,1)*y + intrinsic_matrix.at<double>(2,1);
//              pixels_2d.push_back(cv::Point2d(u,v));
            cv::line(display_matrix_right,
                     cv::Point2d(transformedCloudPtr->points[i].x, transformedCloudPtr->points[i].y),
                     cv::Point2d(transformedCloudPtr->points[i+1].x, transformedCloudPtr->points[i+1].y),
                     cv::Scalar(255,255,255), 1, 8 , CV_8S);
        }

        // Display results
        cv::namedWindow("Right LIDAR Display Window", cv::WINDOW_AUTOSIZE);
        cv::resizeWindow("Right LIDAR Display Window", 1024, 768);
        ROS_INFO ("Showing LIDAR results.");
        cv::imshow("Right LIDAR Display Window", display_matrix_right);
        cv::waitKey(0); // wait for a keystroke in the window;

    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}
