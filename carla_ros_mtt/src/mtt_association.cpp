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
\brief Data association functions header.
*/

// System Includes
#include <carla_ros_mtt/mtt_association.h>

unsigned int last_id=0;

void AssociateObjects(vector<t_listPtr> &list,vector<t_objectPtr> &objects,t_config& config,t_flag& flags)
{
    for(uint i=0;i<objects.size();i++)
        objects[i]->object_found=false;

    double min_ret=1;
    int min_index=-1;
    double ret=1;
    bool association_found;
    double remove_threshold;

    ///Make the static objects association
    for(uint i=0;i<list.size();i++)
    {
        min_ret=1;
        association_found=false;

        for(uint h=0;h<objects.size();h++)///Run throu all the new objects
        {
            ret = CheckAssociationCriteria(*list[i],*objects[h]);

            if(ret<min_ret && ret<0) ///Inside ellipse
            {
                min_ret=ret;
                min_index=h;
                association_found=true;
            }
        }

        //PFLN
        if(association_found)
        {
            // 			double lret;
            // 			double min_lret=1;
            double dist;
            double min_dist=1e6;

            int index_e=-1;
            for(uint e=0;e<list.size();e++)
            {
                if(i==e)
                    continue;

                dist=point2point_distance(objects[min_index]->cx,objects[min_index]->cy,list[e]->position.predicted_x,list[e]->position.predicted_y);

                if(dist<min_dist)
                {
                    min_dist=dist;
                    index_e=e;
                }
            }

            //Exclusion zone B
            // 			printf("%d %2.2f %2.2f\n",mlist->id,min_lret,min_ret);

            if(min_dist < config.ezB && list[index_e]->timers.lifetime > list[i]->timers.lifetime)
            {
                // 				printf("C%d A%d C%2.2f A%2.2f\n",mlist->id,aux2->id,min_ret,min_lret);
                association_found=false;
            }
        }

        //PFLN
        if(association_found && objects[min_index]->object_found==false)///Object found
        {

            if(list[i]->classification.partialy_occluded==false && objects[min_index]->partialy_occluded)
            {

                objects[min_index]->cx=(objects[min_index]->cx+list[i]->position.predicted_x)/2;
                objects[min_index]->cy=(objects[min_index]->cy+list[i]->position.predicted_y)/2;

            }
            //PFLN
            SingleObjectAssociation(*list[i],*objects[min_index]);
            objects[min_index]->object_found=true;
            list[i]->classification.occluded=false;
            list[i]->classification.partialy_occluded=objects[min_index]->partialy_occluded;

            //PFLN
        }else///Object not found
        {
            //PFLN

            list[i]->classification.occluded=true;
            list[i]->timers.occludedtime++;


            list[i]->measurements.x=list[i]->position.predicted_x;
            list[i]->measurements.y=list[i]->position.predicted_y;

            remove_threshold=list[i]->timers.lifetime;

            if(remove_threshold>config.max_missing_iterations)
                remove_threshold=config.max_missing_iterations;

            if(list[i]->timers.occludedtime>remove_threshold)
                RemoveFromList(list,list[i]->id);
        }
    }

    ///Add not found objects to list
    double dist_to_object=1e6;
    for(uint g=0;g<objects.size();g++)
    {
        ///Calc min_distance_to_existing_object
        objects[g]->min_distance_to_existing_object=1e6;

        for(uint i=0;i<list.size();i++)
        {
            dist_to_object=point2point_distance(objects[g]->cx,objects[g]->cy,list[i]->measurements.x,list[i]->measurements.y);

            if(dist_to_object<objects[g]->min_distance_to_existing_object)
                objects[g]->min_distance_to_existing_object=dist_to_object;
        }

        //Exclusion zone A
        if(objects[g]->min_distance_to_existing_object < config.ezA && flags.fi==false)
            continue;

        if(objects[g]->object_found==false)
            AddObjectToList(list,*objects[g],config);
    }

    return;
}

void RemoveFromList(vector<t_listPtr> &list,unsigned int id)
{
    vector<t_listPtr>::iterator it;

    for(it=list.begin();it!=list.end();it++)
    {
        if((*it)->id==id)
        {
            ///Free kalman motion models
            cvReleaseKalman(&((*it)->motion_models.cv_x_kalman));
            cvReleaseKalman(&((*it)->motion_models.cv_y_kalman));
            cvReleaseKalman(&((*it)->motion_models.ca_x_kalman));
            cvReleaseKalman(&((*it)->motion_models.ca_y_kalman));

            free((*it)->errors_cv.x_innovation);
            free((*it)->errors_cv.x_residue);
            free((*it)->errors_cv.y_innovation);
            free((*it)->errors_cv.y_residue);
            free((*it)->errors_cv.lateral_error);
            //PFLN
            free((*it)->errors_ca.x_innovation);
            free((*it)->errors_ca.x_residue);
            free((*it)->errors_ca.y_innovation);
            free((*it)->errors_ca.y_residue);
            free((*it)->errors_ca.lateral_error);

            ///Free error vectors
            //PFLN
            ///Free path
            free((*it)->path_cv.x);
            free((*it)->path_cv.y);
            //PFLN
            ///Free path
            free((*it)->path_ca.x);
            free((*it)->path_ca.y);

            list.erase(it);
            return;
        }
    }
}

void AllocErrors(t_errors*error,t_config&config)
{
    error->x_innovation=(double*)malloc(config.estimation_window_size*sizeof(double));

    error->x_residue=(double*)malloc(config.estimation_window_size*sizeof(double));

    error->y_innovation=(double*)malloc(config.estimation_window_size*sizeof(double));

    error->y_residue=(double*)malloc(config.estimation_window_size*sizeof(double));

    error->lateral_error=(double*)malloc(config.estimation_window_size*sizeof(double));

    error->x_inno_cov=0;
    error->x_resi_cov=0;
    error->y_inno_cov=0;
    error->y_resi_cov=0;
    error->lateral_error_cov=0;

    error->max_number_points=config.estimation_window_size;
    error->number_points=0;
    error->position=0;
    error->latest=0;
    error->next=1;
}

void SingleObjectAssociation(t_list& list,t_object& object)
{
    list.measurements.x=object.cx;
    list.measurements.y=object.cy;
    list.shape=object;

    SetOjectMorphology(list,object);
    object.object_found=true;
    object.id=list.id;

    list.timers.lifetime++;

    list.timers.occludedtime-=1;
    if(list.timers.occludedtime<0)
        list.timers.occludedtime=0;
}

void SetOjectMorphology(t_list&list,t_object& object)
{
    list.object_morphology.apparent_size=object.size;
}

void SetSearchArea(t_list& list,t_config&config)
{
    list.search_area.angle=list.velocity.velocity_angle;
    list.search_area.center_x=list.position.predicted_x;
    list.search_area.center_y=list.position.predicted_y;

    // 	printf("%d %p Xp %2.2f Yp %2.2f\n",list->id,list,list->search_area.center_x,list->search_area.center_y);

    // 	printf("%d EA %2.2f EB %2.2f\n",list->id,list->search_area.ellipse_A,list->search_area.ellipse_B);

    double size=list.object_morphology.apparent_size;
    double size_factor=0.1;

    double default_size=config.min_ellipse_axis;

    double xIl,yIl;

    if(list.model==CV)
    {
        xIl=list.errors_cv.x_inno_cov;
        yIl=list.errors_cv.y_inno_cov;
    }else
    {
        xIl=list.errors_ca.x_inno_cov;
        yIl=list.errors_ca.y_inno_cov;
    }

    int not_found_counter=list.timers.occludedtime;
    double not_found_factor=2;

    double innovation_error=sqrt(sqrt(xIl*xIl+yIl*yIl));
    double lateral_error=sqrt(list.errors_cv.lateral_error_cov);
    double A_innovation_factor,B_innovation_factor;
    double lateral_factor;

    if( list.velocity.velocity_module < 0.200 )
    {
        A_innovation_factor=5;
        B_innovation_factor=4;
        lateral_factor=1;

    }else
    {
        A_innovation_factor=5;
        B_innovation_factor=0.5;
        lateral_factor=4;
    }

    if(innovation_error<0.001)
        innovation_error=0.001;

    list.search_area.ellipse_A = not_found_factor*pow(not_found_counter,2) + A_innovation_factor*innovation_error + default_size + size_factor*size;
    list.search_area.ellipse_B = not_found_factor*pow(not_found_counter,2) + B_innovation_factor*innovation_error + default_size + size_factor*size + lateral_error*lateral_factor;

    if(list.search_area.ellipse_A > config.max_ellipse_axis)
        list.search_area.ellipse_A = config.max_ellipse_axis + default_size + size_factor*size;

    if(list.search_area.ellipse_B > config.max_ellipse_axis)
        list.search_area.ellipse_B = config.max_ellipse_axis + default_size + size_factor*size;

    if(list.timers.lifetime < 5)///Small bonus to new objects @todo I should use a equation that allowed a more soft transition
    {
// 		list.search_area.ellipse_A+=1.0;
// 		list.search_area.ellipse_B+=1.0;
    }
}

void AddPointPath(t_path*path,double x,double y)
{
    path->x[path->position]=x;
    path->y[path->position]=y;

    path->latest=path->position;
    path->position++;

    if(path->position==path->max_number_points)
        path->position=0;

    path->next=path->position;

    if(path->number_points<path->max_number_points)
        path->number_points++;
}

double CheckAssociationCriteria(t_list&list,t_object& object)
{
    double ox=object.cx;
    double oy=object.cy;

    double angle=-list.search_area.angle;
    double s=list.search_area.ellipse_B;
    double r=list.search_area.ellipse_A;
    double M=cos(angle);
    double N=sin(angle);
    double c=list.search_area.center_x;
    double d=list.search_area.center_y;
    double tx=ox-c;
    double ty=oy-d;
    double A=(M*tx-N*ty)*(M*tx-N*ty);
    double B=(N*tx+M*ty)*(N*tx+M*ty);
    double Z=s*s*A+r*r*B-r*r*s*s;

    if(Z<0)
        return Z;
    else
        return 1;
}

void AddObjectToList(vector<t_listPtr> &list,t_object& object,t_config&config)
{
    t_listPtr element(new t_list);

    AllocMotionModels(*element,config);

    AllocPath(&(element->path_cv),config);
    AllocPath(&(element->path_ca),config);

    AllocErrors(&(element->errors_cv),config);
    AllocErrors(&(element->errors_ca),config);

    element->measurements.x=object.cx;
    element->measurements.y=object.cy;
    element->position.estimated_x=object.cx;
    element->position.estimated_y=object.cy;

    InitialiseSearchArea(*element,config);
    InitialiseTimers(&(element->timers));
    InitialiseClassification(&(element->classification));

    element->object_morphology.apparent_size=object.size;

    element->model=CV;
    element->shape=object;

    element->id=last_id;
// 	element->branches=NULL;
// 	element->next=NULL;

// 	cout<<"id "<<element->id<<endl;
    last_id++;

    list.push_back(element);

    return;
}

void InitialiseClassification(t_classification*classification)
{
    classification->velocity_classification=MOVING;
    classification->occluded=false;
    classification->partialy_occluded=false;
}

void InitialiseTimers(t_timers*timer)
{
    timer->occludedtime=0;
    timer->lifetime=0;
}

void InitialiseSearchArea(t_list&list,t_config&config)
{
    list.search_area.ellipse_A=config.max_ellipse_axis;
    list.search_area.ellipse_B=config.max_ellipse_axis;
    list.search_area.angle=0;
    list.search_area.center_x=list.position.estimated_x;
    list.search_area.center_y=list.position.estimated_y;
}

void AllocPath(t_path*path,t_config&config)
{
    path->x=(double*)malloc(config.path_lenght*sizeof(double));

    path->y=(double*)malloc(config.path_lenght*sizeof(double));

    path->max_number_points=config.path_lenght;
    path->number_points=0;
    path->position=0;
    path->latest=0;
    path->next=1;

    return;
}