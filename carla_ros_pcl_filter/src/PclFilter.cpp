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

//    sub_lidar_front = nh.subscribe("/carla/ego_vehicle/lidar/front/point_cloud", 1000000, &PclFilter::callback_lidar_front, this);
//    sub_lidar_left = nh.subscribe("/carla/ego_vehicle/lidar/left/point_cloud", 1000000, &PclFilter::callback_lidar_left, this);
//    sub_lidar_right = nh.subscribe("/carla/ego_vehicle/lidar/right/point_cloud", 1000000, &PclFilter::callback_lidar_right, this);

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

        PointCloud<PointXYZ>::Ptr cloud_out_filtered (new PointCloud<PointXYZ>);
        PointCloud<PointXYZ>::Ptr cloud_out_final (new PointCloud<PointXYZ>);

        // Create the filtering object: downsample the dataset using a leaf size
        // define a voxelgrid
        pcl::VoxelGrid<PointXYZ> voxel_grid_y;
        // set input point cloud
        voxel_grid_y.setInputCloud(transformedCloudPtr);
        // set leaf size (x, y, z)
        voxel_grid_y.setLeafSize(0.1f, 0.1f, 0.1f);
        // apply the filter to dereferenced filteredCloud
        voxel_grid_y.filter(*filteredCloud);

        // define a PassThrough filter
        pcl::PassThrough<pcl::PointXYZ> pass_filter_y;
        // set input to filteredCloud
        pass_filter_y.setInputCloud(filteredCloud);
        // filter along z-axis
        pass_filter_y.setFilterFieldName("z");
        // set z-limits
        pass_filter_y.setFilterLimits(-0.2, 1.0);
        pass_filter_y.filter(*cloud_out_ptr);

        // define a voxelgrid
//        pcl::VoxelGrid<PointXYZ> voxel_grid_x;
//        // set input point cloud
//        voxel_grid_x.setInputCloud(cloud_out_ptr);
//        // set leaf size (x, y, z)
//        voxel_grid_x.setLeafSize(0.1f, 0.1f, 0.1f);
//        // apply the filter to dereferenced filteredCloud
//        voxel_grid_x.filter(*cloud_out_filtered);

        // define a PassThrough filter
        pcl::PassThrough<pcl::PointXYZ> pass_filter_x;
        // set input to filteredCloud
        pass_filter_x.setInputCloud(cloud_out_ptr);
        // filter along x-axis
        pass_filter_x.setFilterFieldName("x");
        // set z-limits
        pass_filter_x.setFilterLimits(0, 2.0);
        pass_filter_x.filter(*cloud_out_final);

        // searchPoint
//        PointXYZ searchPoint = filteredCloud->at(0);
//
//        //result from radiusSearch()
//        std::vector<int> pointIdxRadiusSearch;
//        std::vector<float> pointRadiusSquaredDistance;
//
//        //kdTree
//        pcl::KdTreeFLANN<PointXYZ> kdtree;
//        kdtree.setInputCloud (filteredCloud);
//        kdtree.setSortedResults(true);
//
//        if ( kdtree.radiusSearch (searchPoint, 100, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
//        {
//            //delete every point in target
//            for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
//            {
//                //is this the way to erase correctly???
//                cloud_out.push_back(filteredCloud->points[pointIdxRadiusSearch[j]]);
//            }
//        }

//        cloud_out_ptr = cloud_out.makeShared();
        // Convert PCL cloud to PointCloud2 message
        pcl::toROSMsg(*cloud_out_final.get(),cloud_out_msg );

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

void PclFilter::callback_lidar_planar(const PCLPointCloud2::ConstPtr& cloud)
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
        sensor_msgs::PointCloud2 cloud_out_msg;

        // Create a set of planar coefficients with X=Y=0 and Z=1
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        coefficients->values.resize (4);
        coefficients->values[0] = coefficients->values[1] = 0;
        coefficients->values[2] = 0;
        coefficients->values[3] = 0;

        // Create the filtering object
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (transformedCloudPtr);
        proj.setModelCoefficients (coefficients);
        proj.filter (*filteredCloud);

        // Convert PCL cloud to PointCloud2 message
        pcl::toROSMsg(*filteredCloud.get(),cloud_out_msg );

        // Publish PointCloud2 message in another topic
        cloud_out_msg.header.frame_id = cloud->header.frame_id; // get header frame ID
        pub_lidar_front->publish (cloud_out_msg);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
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

void PclFilter::callback_lidar_left(const PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

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
        pub_lidar_left->publish (cloud_out_msg);
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
            if ( (theta >= -M_PI/2 && theta <= 0)){
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

void PclFilter::callback_lidar_right(const PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

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
        pub_lidar_right->publish (cloud_out_msg);
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
            if ( (theta >= 0 && theta <= M_PI/2)){
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

void PclFilter::callback_radius_outlier_removal(const PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

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

        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // build the filter
        outrem.setInputCloud(transformedCloudPtr);
        outrem.setRadiusSearch(0.8);
        outrem.setMinNeighborsInRadius (2);
        // apply filter
        outrem.filter (*filteredCloud);

        // Convert PCL cloud to PointCloud2 message
        pcl::toROSMsg(*filteredCloud.get(),cloud_out_msg );

        // Publish PointCloud2 message in another topic
        cloud_out_msg.header.frame_id = cloud->header.frame_id; // get header frame ID
        pub_lidar_front->publish (cloud_out_msg);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}

void PclFilter::callback_conditional_removal(const PCLPointCloud2::ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0) {
        return;
    }

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

        // build the condition
        ConditionAnd<PointXYZ>::Ptr range_cond (new ConditionAnd<PointXYZ> ());
        range_cond->addComparison (FieldComparison<PointXYZ>::ConstPtr (new FieldComparison<PointXYZ> ("z", ComparisonOps::GT, 0.0)));
        range_cond->addComparison (FieldComparison<PointXYZ>::ConstPtr (new FieldComparison<PointXYZ> ("z", ComparisonOps::LT, 0.8)));
        // build the filter
        ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (transformedCloudPtr);
        condrem.setKeepOrganized(true);
        // apply filter
        condrem.filter (*filteredCloud);

        // Convert PCL cloud to PointCloud2 message
        pcl::toROSMsg(*filteredCloud.get(),cloud_out_msg );

        // Publish PointCloud2 message in another topic
        cloud_out_msg.header.frame_id = cloud->header.frame_id; // get header frame ID
        pub_lidar_front->publish (cloud_out_msg);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}