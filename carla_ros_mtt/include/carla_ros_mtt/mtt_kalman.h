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

#ifndef ROS_BRIDGE_MTT_KALMAN_H
#define ROS_BRIDGE_MTT_KALMAN_H

/**
\file
\brief Kalman estimation related functions header.
*/

#include "mtt_common.h"

extern void SetSearchArea(t_list& list,t_config&config);
extern void AddPointPath(t_path*path,double x,double y);
// void MotionModelsIteration(t_list*list,t_config*config);
void MotionModelsIteration(vector<t_listPtr> &list,t_config& config);

void GetErrorConvariance(t_errors*error);
void AllocMotionModels(t_list&list,t_config&config);
extern int real2print(double x,t_config*config);
extern double point2line_distance(double alpha,double ro,double x,double y);

void UpdateTransitionMatrixCTRV(CvKalman*model,double /*x*/,double /*y*/,double t,double v,double w,double dt);
void UpdateTransitionMatrixCV(CvKalman*model,double dt);
void UpdateTransitionMatrixCV_SC(CvKalman*model,double dt);

void IterateMotionModelCTRV(CvKalman*model,double vm,double wm);
void IterateMotionModelCV(CvKalman*model,double vxm,double vym);
double IterateMotionModelCV_SC(CvKalman*model,double vm);

CvKalman*CreateModelCTRV(void);
CvKalman*CreateModelCV(void);
CvKalman*CreateModelCV_SC(void);

extern double s[6];
extern double vel,theta;
extern int select_object;
extern FILE*fp;
extern double grxy;
void dA_FwdCt(CvKalman*model,double q[6],double dt,double l=2.54);
void dH_FwdCt(CvKalman*model);
void IterateMotionModelFwdCt(CvKalman*model,double z[2]);
CvKalman*CreateModelFwdCt(void);

#endif //ROS_BRIDGE_MTT_KALMAN_H
