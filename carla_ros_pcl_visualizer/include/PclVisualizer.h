//
// Created by pedro on 16-05-2019.
//

#ifndef ROS_BRIDGE_PCLVISUALIZER_H
#define ROS_BRIDGE_PCLVISUALIZER_H

/* System Includes */
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>


class PclVisualizer{
    public:
        PclVisualizer();
        void callback_lidar_front(const pcl::PCLPointCloud2::ConstPtr& cloud);
        void callback_lidar_left(const pcl::PCLPointCloud2::ConstPtr& cloud);
        void callback_lidar_right(const pcl::PCLPointCloud2::ConstPtr& cloud);

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub_lidar_front;
        ros::Subscriber sub_lidar_left;
        ros::Subscriber sub_lidar_right;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener *tfListener;
        pcl::visualization::PCLVisualizer *viewer_lidar_front;
        pcl::visualization::PCLVisualizer *viewer_lidar_left;
        pcl::visualization::PCLVisualizer *viewer_lidar_right;
        static constexpr const char* fixed_frame_ = "map";

};


#endif //ROS_BRIDGE_PCLVISUALIZER_H
