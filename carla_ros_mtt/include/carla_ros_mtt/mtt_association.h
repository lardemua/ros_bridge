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

#ifndef ROS_BRIDGE_MTT_ASSOCIATION_H
#define ROS_BRIDGE_MTT_ASSOCIATION_H

/**
\file
\brief Data association functions header.
*/

// System Includes
#include "mtt_common.h"
#ifdef _MTT_DATA_ASSOCIATION_CPP_
///This variable is a global id to give to new objects, this ensures that new objects have new ids
#else
extern unsigned int last_id;
#endif

unsigned int GetListSize(t_list*list);

// void AssociateObjects(t_list**list,t_object**objects,int object_size,t_config*config,t_flag*flags);
void AssociateObjects(vector<t_listPtr> &list,vector<t_objectPtr> &objects,t_config& config,t_flag& flags);

void AddObjectToList(vector<t_listPtr> &list,t_object& object,t_config&config);

void RemoveFromList(vector<t_listPtr> &list, unsigned int id);

extern void AllocMotionModels(t_list& list, t_config& config);

void AllocPath(t_path* path,t_config& config);
void InitialiseSearchArea(t_list& list,t_config& config);

void PrintNobjects(t_list* list);

void InitialiseTimers(t_timers*timer);
void InitialiseClassification(t_classification*classification);

/*double CheckAssociationCriteria(t_list*list,t_object*object,t_config*config);
double CheckAssociationCriteria(t_list*list,t_object& object);*/
double CheckAssociationCriteria(t_list& list, t_object& object);

void SingleObjectAssociation(t_list& list, t_object& object);

void AddPointPath(t_path*path,double x, double y);

void SetSearchArea(t_list& list, t_config& config);

void AllocErrors(t_errors* error, t_config& config);

void SetOjectMorphology(t_list& list, t_object& object);

void PrintTree(t_list* list, int l);

extern double point2point_distance(double xi, double yi, double xf, double yf);

extern int real2print(double x, t_config* config);

#endif //ROS_BRIDGE_MTT_ASSOCIATION_H
