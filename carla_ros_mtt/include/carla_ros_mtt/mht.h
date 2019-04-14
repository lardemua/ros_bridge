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

#ifndef ROS_BRIDGE_MHT_H
#define ROS_BRIDGE_MHT_H

/**
\file
\brief Mht implementation main header
*/

// System Includes
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <mtt/graph_wrapper.h>
#include <mtt/mtt_declaration.h>
#include <mtt/k_best.h>
#include <mtt/cluster.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.8/pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <eigen3/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <iostream>
#include <istream>
#include <fstream>
#include <string>
#include <complex>
#include <boost/thread/thread.hpp>

using Eigen::Vector4d;

///Xgtk external graphviz library/plugin (only plugin, there's no need to create a library since it can be linked internally).
extern gvplugin_library_t gvplugin_xgtk_LTX_library;

extern "C"
{
    /**
    \brief External function used to install a gv plugin from a library
    */
    extern void gvconfig_plugin_install_from_library(GVC_t*,char*,gvplugin_library_t*);
}

/**
 * \brief Dynamic markers support class
 *
 * This class allows to easily publish a variable number of markers without paying attention to the delete action requests.
 */
class Markers
{
    public:
        /**
         * \brief Update a internal marker
         *
         * The updated marker will be marked for publishing, this marker may or may not already be present in the class.
         *
         * \param marker a marker to update
         */
        void update(visualization_msgs::Marker& marker);

        /**
         * \brief Mark existing markers for deletion
         */
        void decrement(void);

        /**
         * \brief Remove markers that should not be transmitted
         */
        void clean(void);

        /**
         * \brief Obtain the list of outgoing markers
         *
         * \return a vector of visualization_msgs::Marker
         */
        vector<visualization_msgs::Marker> getOutgoingMarkers(void);
    private:
        ///Internal storing vector of markers
        vector<visualization_msgs::Marker> markers;
};

/**
 * \brief Create markers from targets
 *
 * This function creates a set of markers for the current targets, this uses the Markers class to keep a up to date list of the current markers.
 *
 * \param targets vector of Target 's
 * \return markers to be published
 */
vector<visualization_msgs::Marker> createTargetMarkers(vector<TargetPtr>& targets);

/**
 * \brief Create markers from clusters
 *
 * This function creates a set of markers for the current point clusters, this uses the Markers class to keep a up to date list of the current markers.
 *
 * \param clusters vector of Measurement 's classes
 * \return markers to be published
 */
vector<visualization_msgs::Marker> createClustersMarkers(vector<MeasurementPtr>& clusters);

/**
 * \brief Create a std_msgs color from a hsv color map
 *
 * \param id color code
 *
 * \return a std_msgs::ColorRGBA color
 */
std_msgs::ColorRGBA makeColor(int id);

/**
 * \brief Create a std_msgs color from rgba values
 *
 * \param r red value
 * \param g green value
 * \param b blue value
 * \param a alpha value
 *
 * \return a std_msgs::ColorRGBA color
 */
std_msgs::ColorRGBA makeColor(double r,double g, double b, double a);

/**
 * \brief Create a geometry_msgs::Point from values
 *
 * \param x x component
 * \param y y component
 * \param z z component
 *
 * \return a geometry_msgs::Point
 */
geometry_msgs::Point makePose(double x,double y, double z);

/**
\brief Handler for the incoming data
\param msg incoming point cloud
*/
void dataHandler(const sensor_msgs::PointCloud2& msg);

/**
\brief Graphical thread
This function is the main workhorse for the graphical interface. The graphical interface works in a separate thread that runs this
function.
\param data generic pointer, is not used
\return data generic pointer, is not used
*/
void* graphicalThread(void*data);

/**
\brief Clustering function
This functions separates the incoming data vector into a set of clusters based on a clustering_distance.
\param data input point vector (it is in fact a std::vector\<Mht::PointPtr\> reference since its safer to work with shared_ptr's.
\param clusters output vector of clusters, these clusters are treated has measurements and use the Measurement class.
\param clustering_distance distance value used to break clusters.
*/
void createObjects(vector<PointPtr>&data,vector<MeasurementPtr>&clusters,double clustering_distance);

/**
\brief Convert the point cloud data into a laser scan type structure
Convert from ros format to a local format.
\param cloud ros format used in point clouds, input message.
\param data outgoing vector of Point class points.
*/
void pointCloud2ToVector(const sensor_msgs::PointCloud2& cloud,vector<PointPtr>& data);

/**
\brief Calculate Euclidean distance between Point class types
Only the x and y coordinates are used.
\param p1 first point
\param p2 second point
\return euclidean distance between the two points
*/
double euclideanDistance(Point&p1,Point&p2);

#endif //ROS_BRIDGE_MHT_H
