//
// Created by pedro on 14-04-2019.
//

/**************************************************************************************************
 Software License Agreement (BSD License)
 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.
 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/

/**
\file
\brief Demo source code just to teach the students about c++ features
*/

// System Includes
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
//#include <atlascar_base/AtlascarStatus.h>
//#include <atlascar_base/AtlascarPartialStatus.h>
//#include <atlascar_base/AtlascarVelocityStatus.h>
#include <pcl_ros/transforms.h>
#include <pcl-1.8/pcl/ros/conversions.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/filters/project_inliers.h>

using namespace std;
using namespace ros;

class Conversion
{
    public:

        Conversion():
                n_("~"),
                listener_(n_,ros::Duration(10))
        {
            laser_1_report_name_ = ros::names::remap("laser_1_report");
            laser_2_report_name_ = ros::names::remap("laser_2_report");
            laser_3_report_name_ = ros::names::remap("laser_3_report");

            atlascar_status_report_name_ = ros::names::remap("atlascar_status_report");
            atlascar_velocity_status_report_name_ = ros::names::remap("atlascar_velocity_status_report");

            remove(laser_1_report_name_.c_str());
            remove(laser_2_report_name_.c_str());
            remove(laser_3_report_name_.c_str());

            remove(atlascar_status_report_name_.c_str());
            remove(atlascar_velocity_status_report_name_.c_str());

            laser_1_handler_ = n_.subscribe<sensor_msgs::LaserScan> ("laser_1",1000,boost::bind(&Conversion::laserHandler,this,_1,laser_1_report_name_));
            laser_2_handler_ = n_.subscribe<sensor_msgs::LaserScan> ("laser_2",1000,boost::bind(&Conversion::laserHandler,this,_1,laser_2_report_name_));
            laser_3_handler_ = n_.subscribe<sensor_msgs::LaserScan> ("laser_3",1000,boost::bind(&Conversion::laserHandler,this,_1,laser_3_report_name_));

            atlascar_status_handler_ = n_.subscribe("atlascar_status", 1000, &Conversion::atlascarStatusHandler,this);
            atlascar_velocity_status_handler_ = n_.subscribe("atlascar_velocity_status", 1000, &Conversion::atlascarVelocityStatusHandler,this);
        }

        ~Conversion()
        {

        }

        void laserHandler(const sensor_msgs::LaserScan::ConstPtr& scan_in,string report_name)
        {
            sensor_msgs::PointCloud2 cloud;

            listener_.waitForTransform(ros::names::remap("base_link"), scan_in->header.frame_id, ros::Time::now(), ros::Duration(10.0));

            try
            {
                projector_.transformLaserScanToPointCloud(ros::names::remap("base_link"),*scan_in,cloud,listener_);
            }
            catch (tf::TransformException ex)
            {
                cout<<"Error!! "<<ex.what()<<endl;
                return;
            }

            pcl::PointCloud<pcl::PointXYZ> data;
            pcl::fromROSMsg(cloud,data);

            ofstream report_laser;
            report_laser.open(report_name.c_str(),ios::app);

            boost::format fm("%.4f");
            fm % scan_in->header.stamp.toSec();

            report_laser<<scan_in->header.seq<<" "<<fm.str()<<" "<<ros::names::remap("base_link");

            for(pcl::PointCloud<pcl::PointXYZ>::iterator it = data.begin();it!=data.end();it++)
                report_laser<<" "<<it->x<<" "<<it->y<<" "<<it->z;

            report_laser<<endl;

            report_laser.close();
        }

        void atlascarStatusHandler(const atlascar_base::AtlascarStatusPtr& status)
        {
            ofstream report_status;
            report_status.open(atlascar_status_report_name_.c_str(),ios::app);

            boost::format fm_stamp("%.4f");
            fm_stamp % status->header.stamp.toSec();

            report_status<<status->header.seq<<" "<<fm_stamp.str()<<" ";

            boost::format fm_data("%.6f %.6f %.6f %.6f %.6f %.6f");
            fm_data % status->throttle % status->brake % status->clutch % status->steering_wheel % status->speed % status->rpm;

            report_status<<fm_data.str()<<endl;
        }


        void atlascarVelocityStatusHandler(const atlascar_base::AtlascarVelocityStatusPtr& status)
        {
            ofstream report_status;
            report_status.open(atlascar_velocity_status_report_name_.c_str(),ios::app);

            boost::format fm_stamp("%.4f");
            fm_stamp % status->header.stamp.toSec();

            report_status<<status->header.seq<<" "<<fm_stamp.str()<<" ";

            boost::format fm_data("%.1f %.6f %.6f %.6f");
            fm_data % status->counting % status->pulses_sec % status->revolutions_sec % status->velocity;

            report_status<<fm_data.str()<<endl;

        }

        string laser_1_report_name_;
        string laser_2_report_name_;
        string laser_3_report_name_;
        string atlascar_status_report_name_;
        string atlascar_velocity_status_report_name_;

        ros::Subscriber laser_1_handler_;
        ros::Subscriber laser_2_handler_;
        ros::Subscriber laser_3_handler_;

        ros::Subscriber atlascar_status_handler_;
        ros::Subscriber atlascar_velocity_status_handler_;

        ros::NodeHandle n_;
        tf::TransformListener listener_;
        laser_geometry::LaserProjection projector_;
};

int main(int argc,char**argv)
{
    ros::init(argc, argv, "conversion");

    Conversion convert;

    cout<<"spining ..."<<endl;
    ros::spin();

    return 0;
}