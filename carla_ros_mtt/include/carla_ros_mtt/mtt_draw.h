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

#ifndef ROS_BRIDGE_MTT_DRAW_H
#define ROS_BRIDGE_MTT_DRAW_H

/**
\file
\brief Draw functions header for the old mtt tracking
*/

#include "mtt_common.h"

void DrawListPaths(IplImage*img,t_list*list,t_config*config);
void DrawListCenters(IplImage*img,t_list*list,CvScalar color,t_config*config);
void DrawListIds(IplImage*img,t_list*list,CvScalar color,t_config*config);
void DrawSearchArea(IplImage*img,t_list*list,t_config*config);
void DrawVelocity(IplImage*img,t_list*list,t_config*config);
void draw_end_lines(IplImage*img,t_cluster**clusters,int size,t_data*data,t_config*config);
void draw_oclusion_area(IplImage*img,t_object**list,int size,CvScalar color,t_config*config);
void drawarrow(int x0, int y0, double o, double lenght, IplImage *dst, CvScalar color, int thickness, int line_type, int shift);
void draw_ambient(IplImage*img,t_config*config,enum_background_style style=STYLE_DARK,int laser=1);
void draw_midle_circle(IplImage*img,t_config*config);
void draw_clusters_npoints(IplImage*img,t_cluster**clusters,int size,t_data*data,t_config*config);
void draw_clusters_centers(IplImage*img,t_cluster**clusters,int size,t_config*config);
void draw_clusters_area(IplImage*img,t_cluster**clusters,int size,t_config*config,t_data*data);
void draw_clusters(IplImage*img,t_cluster**clusters,int size,t_data*data,t_config*config);
void draw_objects(IplImage*img,t_object**objects,int size,CvScalar color,t_config*config);
void draw_objects_centers(IplImage*img,t_object**objects,int size,CvScalar color,t_config*config);
void draw_objects_ids(IplImage*img,t_object**objects,int size,CvScalar color,t_config*config);
void draw_objects_ppos(IplImage*img,t_object**objects,int size,t_config*config);
void draw_raw_data(IplImage*img,t_data*data,CvScalar color,t_config*config);
void draw_raw_data_acc(IplImage*img,t_data_acc*data,CvScalar color,t_config*config);
void DrawGlobal(char key,IplImage*img,t_config*config,t_data*data,t_object**objects,int object_size,t_list*list,t_flag*flags,int laser=0,bool raw_only=false);
extern void get_mean_velocity(t_circular_data*data,double*mean_module,double*mean_angle);
extern int real2print(double x,t_config*config);

#endif //ROS_BRIDGE_MTT_DRAW_H
