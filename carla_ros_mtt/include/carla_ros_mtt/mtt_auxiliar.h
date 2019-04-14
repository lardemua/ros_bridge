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

#ifndef ROS_BRIDGE_MTT_AUXILIAR_H
#define ROS_BRIDGE_MTT_AUXILIAR_H

/**
\file
\brief Auxiliary functions header for the mtt tracking
*/

#define mtic my_tictoc(1)
#define mtoc my_tictoc(0)
#define TIC 1
#define TOC 0

// System Includes
#include "mtt_common.h"

double get_fps(double dt,t_fps*acc);

/**
@brief Init flags
@param flags general flags structure
@return void
*/
void init_flags(t_flag*flags);

/**
@brief Init configuration
@param config general configuration structure
@return void
*/
void init_config(t_config*config);

/**
@brief TIC TOC implementation functions
@param status TIC or TOC
@return timediff in useconds if TOC, 0 if TIC, -1 if unknown
*/
int my_tictoc(int status);

/**
@brief Calculates timediff
@param t1 first time
@param t2 second time
@return timediff in useconds
*/
int timediff(struct timeval t1,struct timeval t2);

/**
@brief Calculates the line to point distance
@param alpha polar coordinates of the line
@param ro polar coordinates of the line
@param x cartesian coordinates of the point
@param y
@return algebric distance
*/
double point2line_distance(double alpha,double ro,double x,double y);

/**
@brief Calculates the algebric distante between two points.
@param xi
@param yi
@param xf
@param yf
@return algebric distance
*/
double point2point_algebric_distance(double xi,double yi,double xf,double yf);

/**
@brief Calculates the distante between two points.
@param xi
@param yi
@param xf
@param yf
@return distance
*/
double point2point_distance(double xi,double yi,double xf,double yf);

void CreateMeasurementFromDisplacement(double dx,double dy,double dtheta,double z[2],double dt,double l,double bwa);
void ConvertEstimatedToMeasurment(double vl,double dir,float*dx,float*dy,float*dtheta,double dt,double l,double bwa);

#endif //ROS_BRIDGE_MTT_AUXILIAR_H
