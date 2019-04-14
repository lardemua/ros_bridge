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

#ifndef ROS_BRIDGE_PMHT_H
#define ROS_BRIDGE_PMHT_H

/**
\file
\brief Header for the Multi-Hypothesis Tracking data association algorithm
For additional info on the Mht algorithm check \ref mhtpage "here".
*/

// System Includes
#include <ros/ros.h>
#include <mtt/tree.hh>
#include <mtt/tree_util.hh>
#include <colormap/colormap.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.8/pcl/surface/mls.h>
//#include <eigen3/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include <boost/lexical_cast.hpp>
// #include <boost/format.hpp>
#include <graphviz/gvc.h>
#include <graphviz/types.h>
#include <graphviz/gvplugin.h>
#include <mtt/types_declaration.h>
#include <mtt/graph_wrapper.h>
#include <mtt/mht.h>

extern "C" {
extern void gvconfig_plugin_install_from_library(GVC_t*,char*,gvplugin_library_t*);
}

using namespace std;
using namespace ros;


//Prototypes

/**
\brief  Create a parametric ellipse from covariance matrix
This function creates a parametric ellipse from a covariance matrix, the mean values and a threshold.
The main axes directions are calculated using the eigenvectors of the covariance matrix, then a crawling
algorithm detects the threshold points obtaining the lengths of the axes. After that the generic parametric
equation is used to calculate points in the threshold boundary.
\param cov covariance matrix
\param mu mean values
\param MahThreshold Mahalanobis threshold to use
\return the xy points of the ellipse, the last points is equal to the first so the ellipse closes
*/
MatrixXd EllipseParametric(Matrix2d& cov,Vector2d& mu,double MahThreshold);

/**
\brief Create markers for Rviz
*/
void CreateMarkers(vector<visualization_msgs::Marker>& marker_vector,vector<t_clusterPtr>&clusters);
/**
\brief Convert from a point cloud representation to a stl vector of shared pointers to the class t_point
This function first puts all the points in a polar grid representation and reorders them to be angularly sequenced.\n
This function has an important parameter which is the angular resolution of the grid that should be a ros param.\n
\todo Convert the current hard coded angular resolution of the grid (vulgar grid spacing).
\param cloud input point cloud
\param data stl vector of boost::shared_ptr of the class t_point
*/
void PointCloud2ToVector(const sensor_msgs::PointCloud2& cloud,vector<t_pointPtr>& data);

/**
\brief This function converts a set of points into groups (aka clusters)
The clustering algorithm is solely based on the distance between consecutive points in the polar grid.\n
It copies the support points of the cluster into the own cluster so no reference to the original data is needed not even desired.
\param data vector of data
\param clusters vector of clusters (this is the output so it will be filled)
\param clustering_distance maximum distance between two points for them to be in the same cluster
*/
void CreateObjects(vector<t_pointPtr>&data,vector<t_clusterPtr>&clusters,double clustering_distance);

/**
\brief Calculate the Mahalanobis distance
Calculate the Mahalanobis distance for a measurement y from a set with a given mean and covariance.
\param y measurement
\param mean mean value of the variable
\param cov covariance of the variable
\return the Mahalanobis distance
*/
double Mahalanobis(Vector2d&y,Vector2d&mean,Matrix2d& cov);

/**
\brief Calculate the distance between a node and a cluster
This function uses the Mahalanobis distance between the node position estimation and the cluster position.
\param node tree node to use
\param cluster cluster to use
\return the Mahalanobis distance
*/
double Mahalanobis_distance(t_nodePtr&node,t_clusterPtr&cluster);

/**
\brief Calculate the Euclidean distance from node to cluster
Calculate the euclidean distance from a node and a cluster.
\param node Node, the coordinates in x_p point of the node will be used
\param cluster Cluster, the centroid coordinates will be used
\return the euclidean distance
*/
double Euclidean_distance(t_nodePtr&node,t_clusterPtr&cluster)
{
    return sqrt( pow(node->x_p(0)-cluster->centroid.x,2) + pow(node->x_p(1)-cluster->centroid.y,2));
}

/**
\brief Calculate the Euclidean distance from point to point
Calculate the euclidean distance from a point to a point
\param p1 point 1
\param p2 point 2
\return the euclidean distance
*/
double Euclidean_distance(t_point&p1,t_point&p2)
{
    return sqrt( pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2));
}

/**
\brief Get the closest candidate to the node from all the clusters
This function calls the CalculateDistance function to do the real node to cluster evaluation.
\param node node to use
\param clusters clusters to search
\return iterator to a member of the clusters
*/
vector<t_clusterPtr>::iterator GetClosestCandidate(t_nodePtr&node,vector<t_clusterPtr>&clusters);

#endif //ROS_BRIDGE_PMHT_H
