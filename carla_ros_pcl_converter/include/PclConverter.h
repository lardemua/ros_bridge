//
// Created by pedro on 15-05-2019.
//

#ifndef ROS_BRIDGE_PCLCONVERTER_H
#define ROS_BRIDGE_PCLCONVERTER_H

/* System Includes */
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>


class PclConverter{
    public:
        PclConverter();
        void callback(const pcl::PCLPointCloud2::ConstPtr& cloud);

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener *tfListener;
        static constexpr const char* fixed_frame_ = "map";

};

#endif //ROS_BRIDGE_PCLCONVERTER_H
