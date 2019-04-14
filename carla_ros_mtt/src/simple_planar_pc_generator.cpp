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
 * \file
 * \brief Simple planar scan generator
 * \author Miguel Oliveira, Jorge Almeida
 * \version v1
 * \date 2012-02-29
 */

// System Includes
#include <stdio.h>
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl-1.8/pcl/conversions.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/filters/project_inliers.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudGeneretor
{
    public:
        ros::NodeHandle nh;
        ros::Publisher points_publisher;
        tf::TransformListener transform_listener;

        bool wait_for_laser_1;
        bool wait_for_laser_2;

        bool laserscan_1_arrived;
        bool laserscan_2_arrived;

        double output_freq;
        double perpendicular_treshold;

        pcl::PointCloud<pcl::PointXYZ> pc_accumulated;
        pcl::PointCloud<pcl::PointXYZ> convex_hull;

        ros::Subscriber scan_1_subscriber;
        ros::Subscriber scan_2_subscriber;

        PointCloudGeneretor(ros::NodeHandle nh_)
                :nh(nh_),
                 transform_listener(nh_,ros::Duration(10))
        {
            laserscan_1_arrived = false;
            laserscan_2_arrived = false;

            nh.param("wait_for_laser_1", wait_for_laser_1, true);
            nh.param("wait_for_laser_2", wait_for_laser_2, true);

            nh.param("output_frequency", output_freq, 200.0);
            nh.param("perpendicular_treshold", perpendicular_treshold, 0.2);

            //Test the remapping of /tracking_frame
            if (ros::names::remap("/tracking_frame")=="/tracking_frame")
            {
                ROS_ERROR("/tracking_frame was not remapped. Aborting program.");
                ros::shutdown();
            }

            pc_accumulated.header.frame_id = ros::names::remap("/tracking_frame");

            scan_1_subscriber = nh.subscribe ("/laserscan0", 1000, &PointCloudGeneretor::scan1Handler,this);
            scan_2_subscriber = nh.subscribe ("/laserscan1", 1000, &PointCloudGeneretor::scan2Handler,this);

            //declare the publisher
            points_publisher = nh.advertise<sensor_msgs::PointCloud2>("/pc_out", 1000);


            //compute the convex hull to use in extract polygonal prism data
            //Its just a very large square (500m*2 of side) around the /tracking_frame origin
            convex_hull.header.frame_id = ros::names::remap("/tracking_frame");
            pcl::PointXYZ pt;
            pt.x = -500; pt.y=-500; pt.z=0;
            convex_hull.points.push_back(pt);

            pt.x = 500; pt.y=-500; pt.z=0;
            convex_hull.points.push_back(pt);

            pt.x = 500; pt.y= 500; pt.z=0;
            convex_hull.points.push_back(pt);

            pt.x = -500; pt.y=500; pt.z=0;
            convex_hull.points.push_back(pt);
        }

        void filterCloud(pcl::PointCloud<pcl::PointXYZ>* pc_in, std::string pc_frame_id)
        {
            pcl::PointCloud<pcl::PointXYZ> pc_transformed;
            pcl::PointCloud<pcl::PointXYZ> pc_filtered;
            pcl::PointCloud<pcl::PointXYZ> pc_projected;
            tf::StampedTransform transform;

            try
            {
                transform_listener.lookupTransform(pc_frame_id, ros::names::remap("/tracking_frame"), ros::Time(0), transform);
                //p_listener->lookupTransform(pcmsg_in.header.frame_id, ros::names::remap("/tracking_frame"), scan_in->header.stamp, transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }


            pcl_ros::transformPointCloud(*pc_in,pc_transformed,transform.inverse());
            pc_transformed.header.frame_id = ros::names::remap("/tracking_frame");
            //pc_transformed.header.stamp = scan_in->header.stamp;

            pcl::ExtractPolygonalPrismData<pcl::PointXYZ> epp;
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud_constptr;
            input_cloud_constptr.reset (new pcl::PointCloud<pcl::PointXYZ> (pc_transformed));
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr convex_hull_constptr;
            convex_hull_constptr.reset (new pcl::PointCloud<pcl::PointXYZ> (convex_hull));

            //pcl::PointIndices::Ptr ind =  pcl::PointIndices::Ptr(new pcl::PointIndices);
            pcl::ExtractIndices<pcl::PointXYZ> extract; //Create the extraction object
            pcl::PointIndices::Ptr indices;
            indices.reset();
            indices = pcl::PointIndices::Ptr(new pcl::PointIndices);

            //Set epp parameters
            epp.setInputCloud(input_cloud_constptr);
            epp.setInputPlanarHull(convex_hull_constptr);
            epp.setHeightLimits(-perpendicular_treshold, perpendicular_treshold);
            epp.setViewPoint(0,0,0); //i dont think this serves any purpose in the case of epp
            epp.segment(*indices);

            extract.setInputCloud(pc_transformed.makeShared());
            extract.setIndices(indices);
            extract.setNegative(false);
            extract.filter(pc_filtered);

            pcl::ProjectInliers<pcl::PointXYZ> projection;
            projection.setModelType(pcl::SACMODEL_PLANE); //set model type
            pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
            coeff->values.resize(4);
            coeff->values[0] = 0;
            coeff->values[1] = 0;
            coeff->values[2] = 1;
            coeff->values[3] = 0;

            projection.setInputCloud(pc_filtered.makeShared());
            projection.setModelCoefficients(coeff);
            projection.filter(pc_projected);

            coeff.reset();
            input_cloud_constptr.reset();
            convex_hull_constptr.reset();
            indices.reset();

            pc_accumulated += pc_projected;

    #if _USE_DEBUG_
            ROS_INFO("pc_transformed has %d points", (int)pc_transformed.size());
                        ROS_INFO("pc_filtered has %d points", (int)pc_filtered.size());
                        ROS_INFO("pc_accumulated now has %d points", (int)pc_accumulated.size());
    #endif
        }

        void scan1Handler(const sensor_msgs::LaserScan::ConstPtr& scan_in)
        {
            laser_geometry::LaserProjection projector;
            sensor_msgs::PointCloud2 pcmsg_in;

            projector.transformLaserScanToPointCloud(scan_in->header.frame_id, *scan_in,pcmsg_in,transform_listener);   // não é necessário
            pcmsg_in.header.stamp=ros::Time();
            pcmsg_in.header.frame_id=scan_in->header.frame_id;

            pcl::PointCloud<pcl::PointXYZ> pc_in;
            pcl::PCLPointCloud2 pcl_pc;
            pcl_conversions::toPCL(pcmsg_in, pcl_pc);
            pcl::fromPCLPointCloud2(pcl_pc, pc_in);

            filterCloud(&pc_in, scan_in->header.frame_id);

            laserscan_1_arrived=true;

            publishCloud();
        }

        void scan2Handler(const sensor_msgs::LaserScan::ConstPtr& scan_in)
        {
            laser_geometry::LaserProjection projector;
            sensor_msgs::PointCloud2 pcmsg_in;

            projector.transformLaserScanToPointCloud(scan_in->header.frame_id, *scan_in,pcmsg_in,transform_listener);   // não é necessário
            pcmsg_in.header.stamp=ros::Time();
            pcmsg_in.header.frame_id=scan_in->header.frame_id;

            pcl::PointCloud<pcl::PointXYZ> pc_in;
            pcl::PCLPointCloud2 pcl_pc;
            pcl_conversions::toPCL(pcmsg_in, pcl_pc);
            pcl::fromPCLPointCloud2(pcl_pc, pc_in);

            filterCloud(&pc_in, scan_in->header.frame_id);

            laserscan_2_arrived=true;

            publishCloud();
        }

        void publishCloud()
        {
            if(!laserscan_1_arrived && wait_for_laser_1)
                return;

            if(!laserscan_2_arrived && wait_for_laser_2)
                return;

            laserscan_1_arrived=false;
            laserscan_2_arrived=false;

            sensor_msgs::PointCloud2 pcmsg_out;
            pcl::toROSMsg(pc_accumulated, pcmsg_out);
            pcmsg_out.header.stamp = ros::Time::now();

    #if _USE_DEBUG_
            ROS_INFO("Publishing pc_accumulated with %d points", (int)pc_accumulated.size());
    #endif

            points_publisher.publish(pcmsg_out);
            pc_accumulated.erase(pc_accumulated.begin(), pc_accumulated.end());
        }

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_planar_pc_generator_node");
    ros::NodeHandle n("~");

    PointCloudGeneretor generator(n);

    ros::spin();
}