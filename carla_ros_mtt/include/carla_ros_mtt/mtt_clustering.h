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

#ifndef ROS_BRIDGE_MTT_CLUSTERING_H
#define ROS_BRIDGE_MTT_CLUSTERING_H

/**
\file
\brief Clustering related functions header.
*/
#include "mtt.h"
void FlagCollisionWithOcclusion(t_cluster**clusters,int object_size,t_data*data,t_config*config);

/**
@brief Computes dietmayer clustering threshold
@param r
@param config general config structure
@return dietmayer threshold
*/
double dietmayer_threshold(double r,t_config*config);
double ClusteringThreshold(double r1,double t1,double r2,double t2,t_config*config);

/**
@brief Performs clustering operation
@param data general data storage vector
@param count number of clusters
@param config general config structure
@param flags general flags structure
@return clusters
*/
t_cluster**clustering(t_data*data,int*count,t_config*config,t_flag*flags);

bool clustering(t_data&data,vector<t_clustersPtr> &clustersPtr,t_config*config,t_flag*flags);

/**
@brief Removes cluster border points
@param clusters clusters to be drawn
@param size number of clusters
@param npoints number of points to be removed
@return void
*/
void remove_border_points(t_cluster**clusters,int size,int npoints);

/**
@brief Removes clusters based on their size
@param clusters clusters to be drawn
@param size number of clusters
@param threshold number of points a cluster must have not to be deleted
@return void
*/
void remove_small_clusters(t_cluster**clusters,int*size,int threshold);

/**
@brief Calculates cluster properties
@param clusters clusters to be drawn
@param size number of clusters
@param data general data storage vector
@param config general config structure
@return void
*/
void calc_cluster_props(t_cluster**clusters,int size,t_data*data,t_config*config);

/**
@brief Converts clusters of points into objects using recursive line fitting
@param objectsPtr Output objects
@param clusters Input clusters
@param data Data points
@param config general config structure
@return always true
*/
bool clusters2objects(vector<t_objectPtr> &objectsPtr,vector<t_clustersPtr> &clusters,t_data& data,t_config& config);

/**
@brief Computes object properties, such as centroid and size
@param objects list of objects
@return void
*/
void calc_object_props(vector<t_objectPtr> &objects);

void calc_cluster_props(vector<t_clustersPtr> &clusters,t_data&data);

/**
@brief Frees memory associated with objects
@param objects Input objects
@return void
*/
void clean_objets(vector<t_objectPtr> &objects);


/**
@brief Recursive line fitting, complete process
@param object single object
@param cluster single cluster
@param data data points
@param config general config structure
@return void
*/
void recursive_line_fitting(t_objectPtr& object,t_cluster& cluster,t_data& data,t_config& config);

/**
@brief Removes lines from memory
@param objects list of objects
@return void
*/
void free_lines(vector<t_objectPtr> &objects);

/**
@brief Recursive iterative end point fit. Single step
@param object single object
@param data data points
@param start start point
@param end end point
@param config general config structure
@return void
*/
void recursive_IEPF(t_objectPtr& object,t_data& data,int start,int end,t_config& config);

extern double point2point_distance(double xi,double yi,double xf,double yf);
extern double point2line_distance(double alpha,double ro,double x,double y);
extern int real2print(double x,t_config*config);
extern void _p(const char*text);

#endif //ROS_BRIDGE_MTT_CLUSTERING_H
