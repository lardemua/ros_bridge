//
// Created by pedro on 27-05-2019.
//

#ifndef ROS_BRIDGE_PCLCLUSTER_H
#define ROS_BRIDGE_PCLCLUSTER_H

/* System Includes */
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>


class PclCluster{
    public:
        PclCluster();
        void callback_lidar_front(const pcl::PCLPointCloud2::ConstPtr& cloud);
        void callback_lidar_left(const pcl::PCLPointCloud2::ConstPtr& cloud);
        void callback_lidar_right(const pcl::PCLPointCloud2::ConstPtr& cloud);

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub_lidar_front;
        ros::Subscriber sub_lidar_left;
        ros::Subscriber sub_lidar_right;
        boost::shared_ptr<ros::Publisher> pub_lidar_front;
        boost::shared_ptr<ros::Publisher> pub_lidar_left;
        boost::shared_ptr<ros::Publisher> pub_lidar_right;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener *tfListener;
        static constexpr const char* fixed_frame_ = "map";

};

#endif //ROS_BRIDGE_PCLCLUSTER_H
