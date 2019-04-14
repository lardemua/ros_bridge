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
\brief Clustering related functions.
*/

// System Includes
#include <carla_ros_mtt/mtt_clustering.h>

void FlagCollisionWithOcclusion(t_cluster**clusters,int object_size,t_data*data,t_config*/*config*/)
{
    t_point start_cur,end_cur;
    t_point end_prev,start_next;

    double angular_discrepance=1;
    double range_discrepance=500;


    for(int k=1;k<object_size-1;k++)
    {
        start_cur.r=data->r[clusters[k]->stp];
        start_cur.t=data->t[clusters[k]->stp];

        end_cur.r=data->r[clusters[k]->enp];
        end_cur.t=data->t[clusters[k]->enp];

        end_prev.r=data->r[clusters[k-1]->enp];
        end_prev.t=data->t[clusters[k-1]->enp];

        start_next.r=data->r[clusters[k+1]->stp];
        start_next.t=data->t[clusters[k+1]->stp];


        if(fabs(start_cur.t-end_prev.t)<angular_discrepance*M_PI/180.)
            if(start_cur.r-end_prev.r > range_discrepance)
                clusters[k]->partialy_occluded=true;

        if(fabs(end_cur.t-start_next.t)<angular_discrepance*M_PI/180.)
            if(end_cur.r-start_next.r > range_discrepance)
                clusters[k]->partialy_occluded=true;
    }
}

double ClusteringThreshold(double r1,double t1,double r2,double t2,t_config*config)
{
    double min_dist;
    double Ax,Ay,Bx,By;

    if(r1<r2)
    {
        Ax=r1*cos(t1);
        Ay=r1*sin(t1);

        Bx=r1*cos(t1+deg2rad(0.5));
        By=r1*sin(t1+deg2rad(0.5));
    }else
    {
        Ax=r2*cos(t2+deg2rad(0.5));
        Ay=r2*sin(t2+deg2rad(0.5));

        Bx=r2*cos(t2);
        By=r2*sin(t2);
    }

    min_dist=sqrt( pow(Ax-Bx,2) + pow(Ay-By,2) );

    // 	printf("Cos beta %f\n",cos(config->beta));
    // 	printf("MD %f\n",min_dist);

    return config->C0 + min_dist/cos(config->beta);
}

double dietmayer_threshold(double r,t_config*config)
{
    //return config->C0+r*((tan(config->beta)*sqrt(2*(1.-cos(config->fi))))/(cos(config->fi/2)-sin(config->fi/2)));		//compute threshold
    printf("C0 %f\n",config->C0);
    printf("sqrt(2*(1+cos(config->fi))) %f\n",sqrt(2*(1+cos(config->fi))));
    printf("Fi %f\n",config->fi);
    printf("R %f\n",r);
    double min_dist=tan(0.5*M_PI/180.)*r;
    printf("Min dist %f\n",min_dist);
    printf("CALCILC THIS\n");

    // 	return min_dist + config->C0 +

    return config->C0+r*sqrt(2*(1+cos(config->fi)));
}

t_cluster**clustering(t_data*data,int*count,t_config*config,t_flag*flags)
{
    int i,cluster_count=0;
    double x,y,xold=0,yold=0;
    double dist,threshold;

    static bool initialise=true;
    static t_cluster**clusters;

    if(initialise)
    {
        clusters=(t_cluster**)malloc(_MAX_CLUSTERS_*sizeof(t_cluster*));	//malloc dos ponteiros
        memset(clusters,0,_MAX_CLUSTERS_*sizeof(t_cluster*));
        for(i=0;i<_MAX_CLUSTERS_;i++)
        {
            clusters[i]=(t_cluster*)malloc(sizeof(t_cluster));
        }
        initialise=false;
    }

    for(i=0;i<data->n_points;i++)
    {
        x=data->x[i];
        y=data->y[i];

        if(i>0)
        {
            dist = sqrt((x-xold)*(x-xold)+(y-yold)*(y-yold));		//compute distance

            //  			threshold=dietmayer_threshold(rmin,config);
            threshold=ClusteringThreshold(data->r[i-1],data->t[i-1],data->r[i],data->t[i],config);
            // 			printf("Threshold %f\n",threshold);
            // 			printf("Dist %f\n",dist);
            // 			printf("Rmin %f\n",rmin);
            if(dist>threshold)
            {
                clusters[cluster_count]->enp=i-1;		//set the last cluster endpoint
                clusters[cluster_count]->n_points=clusters[cluster_count]->enp-clusters[cluster_count]->stp;	//sets the number of points in the cluster
                clusters[cluster_count]->partialy_occluded=false;
                cluster_count++;
                clusters[cluster_count]->stp=i;											//sets the new cluster start and end point
                clusters[cluster_count]->lenght=0;
            }else
            {
                clusters[cluster_count]->lenght+=dist;
                if(clusters[cluster_count]->lenght>config->cluster_break_distance)
                {
                    clusters[cluster_count]->enp=i-1;		//set the last cluster endpoint
                    clusters[cluster_count]->n_points=clusters[cluster_count]->enp-clusters[cluster_count]->stp;	//sets the number of points in the cluster
                    clusters[cluster_count]->partialy_occluded=false;
                    cluster_count++;
                    clusters[cluster_count]->stp=i;											//sets the new cluster start and end point
                    clusters[cluster_count]->lenght=0;
                }
            }

            if(i==(data->n_points-1))//last point
            {
                //in case all points are in the same cluster
                clusters[cluster_count]->enp=i;
                clusters[cluster_count]->n_points=clusters[cluster_count]->enp-clusters[cluster_count]->stp;	//sets the number of points in the cluster
            }

        }else//first point
        {
            flags->fp_s=0;			//negates the first point of scan flag
            clusters[cluster_count]->stp=0;
            clusters[cluster_count]->enp=0;
            clusters[cluster_count]->lenght=0;
        }

        xold=x;
        yold=y;
    }

    if(!data->n_points)
        *count=0;
    else
        *count=cluster_count+1;

    return clusters;
}

bool clustering(t_data& data,vector<t_clustersPtr> &clustersPtr,t_config*config,t_flag*flags)
{
    int i;
    double x,y,xold=0,yold=0;
    double dist,threshold;

    t_clustersPtr cluster(new t_cluster);

    clustersPtr.clear();

    cluster->id=clustersPtr.size();

    for(i=0;i<data.n_points;i++)
    {
        x=data.x[i];
        y=data.y[i];

        if(i>0)
        {
            dist = sqrt((x-xold)*(x-xold)+(y-yold)*(y-yold));		//compute distance
            threshold=ClusteringThreshold(data.r[i-1],data.t[i-1],data.r[i],data.t[i],config);

            if(dist>threshold)
            {
                cluster->enp=i-1;		//set the last cluster endpoint
                cluster->n_points=cluster->enp-cluster->stp;	//sets the number of points in the cluster
                cluster->partialy_occluded=false;
                clustersPtr.push_back(cluster);

                cluster.reset(new t_cluster);

                cluster->id=clustersPtr.size();
                cluster->stp=i;											//sets the new cluster start and end point
                cluster->lenght=0;
            }else
            {
                cluster->lenght+=dist;
                if(cluster->lenght>config->cluster_break_distance)
                {
                    cluster->enp=i-1;		//set the last cluster endpoint
                    cluster->n_points=cluster->enp-cluster->stp;	//sets the number of points in the cluster
                    cluster->partialy_occluded=false;
                    clustersPtr.push_back(cluster);

                    cluster.reset(new t_cluster);

                    cluster->id=clustersPtr.size();
                    cluster->stp=i;											//sets the new cluster start and end point
                    cluster->lenght=0;
                }
            }

            if(i==(data.n_points-1))//last point
            {
                //in case all points are in the same cluster
                cluster->enp=i;
                cluster->n_points=cluster->enp-cluster->stp;	//sets the number of points in the cluster
            }

        }else//first point
        {
            flags->fp_s=0;			//negates the first point of scan flag
            cluster->stp=0;
            cluster->enp=0;
            cluster->lenght=0;
        }

        xold=x;
        yold=y;
    }

    return true;
}

void remove_small_clusters(t_cluster**clusters,int*size,int threshold)
{
    int i,e;

    for(i=0;i<*size;i++)
    {
        if(clusters[i]->n_points<threshold)
        {
            for(e=i;e<*size-1;e++)
            {
                clusters[e]->n_points=clusters[e+1]->n_points;
                clusters[e]->stp=clusters[e+1]->stp;
                clusters[e]->enp=clusters[e+1]->enp;
            }
            memset(clusters[*size],0,sizeof(t_cluster));
            (*size)--;
            i--;
        }
    }
}

void remove_border_points(t_cluster**clusters,int size,int npoints)
{
    int i;

    for(i=0;i<size;i++)
    {
        if(clusters[i]->n_points>npoints*2)
        {
            clusters[i]->stp=clusters[i]->stp+npoints;
            clusters[i]->enp=clusters[i]->enp-npoints;
        }

        clusters[i]->n_points=clusters[i]->n_points-npoints*2;
    }
}

void calc_cluster_props(t_cluster**clusters,int size,t_data*data,t_config*/*config*/)
{
    double rmin;
    int i,e;

    for(i=0;i<size;i++)
    {
        rmin=1e12;
        clusters[i]->lenght=0;
        for(e=clusters[i]->stp;e<clusters[i]->enp;e++)
        {
            if(e<clusters[i]->enp-1)
                clusters[i]->lenght+=point2point_distance(data->x[e],data->y[e],data->x[e+1],data->y[e+1]);

            if(data->r[e]<rmin)
                rmin=data->r[e];
        }


        clusters[i]->rmin=rmin;
        clusters[i]->tm=(data->t[clusters[i]->stp]+data->t[clusters[i]->enp])/2;
    }
}

void calc_cluster_props(vector<t_clustersPtr> &clusters,t_data&data)
{
    double rmin;
    int e;

    for(uint i=0;i<clusters.size();i++)
    {
        rmin=1e12;
        clusters[i]->lenght=0;
        for(e=clusters[i]->stp;e<clusters[i]->enp;e++)
        {
            if(e<clusters[i]->enp-1)
                clusters[i]->lenght+=point2point_distance(data.x[e],data.y[e],data.x[e+1],data.y[e+1]);

            if(data.r[e]<rmin)
                rmin=data.r[e];
        }


        clusters[i]->rmin=rmin;
        clusters[i]->tm=(data.t[clusters[i]->stp]+data.t[clusters[i]->enp])/2;
    }
}

bool clusters2objects(vector<t_objectPtr> &objectsPtr,vector<t_clustersPtr> &clusters,t_data& data,t_config& config)
{
    t_objectPtr object(new t_object);

    objectsPtr.clear();

    for(uint i=0;i<clusters.size();i++)
    {
        object->rmin=clusters[i]->rmin;
        object->tm=clusters[i]->tm;
        object->object_found=false;

        object->partialy_occluded=clusters[i]->partialy_occluded;

        recursive_line_fitting(object,*clusters[i],data,config);

        objectsPtr.push_back(object);
        object.reset(new t_object);
    }

    return true;
}

void calc_object_props(vector<t_objectPtr> &objects)
{
    uint e;
    double r,t;
    double xi,yi,xf,yf;
    double rmin;

    for(uint i=0;i<objects.size();i++)
    {
        t=objects[i]->tm;

        rmin=1e12;

        for(e=0;e<objects[i]->lines.size();e++)
        {
            r=sqrt(pow(objects[i]->lines[e]->xi,2)+pow(objects[i]->lines[e]->yi,2));

            if(r<rmin)
                rmin=r;
        }

        r=sqrt(pow(objects[i]->lines[objects[i]->lines.size()-1]->xf,2)+pow(objects[i]->lines[objects[i]->lines.size()-1]->yf,2));
        if(r<rmin)
            rmin=r;

        r=rmin;

        objects[i]->cx=r*cos(t);
        objects[i]->cy=r*sin(t);

        xi=objects[i]->lines[0]->xi;
        yi=objects[i]->lines[0]->yi;

        xf=objects[i]->lines[objects[i]->lines.size()-1]->xf;
        yf=objects[i]->lines[objects[i]->lines.size()-1]->yf;

        objects[i]->size=point2point_distance(xi,yi,xf,yf);
    }
}

void clean_objets(vector<t_objectPtr> &objects)
{
    free_lines(objects);
}

void recursive_IEPF(t_objectPtr& object,t_data& data,int start,int end,t_config& config)
{
    /**This functions malloc a line to work with, each time it mallocs a line it increments the number of lines object data*/

    int i,index=0;
    double mean_variance,max_variance,current_variance;

    t_linePtr line(new t_line);

    line->alpha=atan2(data.x[start]-data.x[end],data.y[end]-data.y[start])+M_PI;
    line->ro=data.x[start]*cos(line->alpha)+data.y[start]*sin(line->alpha);
    line->xi=data.x[start];
    line->yi=data.y[start];
    line->xf=data.x[end];
    line->yf=data.y[end];

    mean_variance=0;
    max_variance=0;
    for(i=start;i<end;i++)
    {
        current_variance=pow(point2line_distance(line->alpha,line->ro,data.x[i],data.y[i]),2);
        mean_variance+=current_variance;

        if(current_variance>max_variance)
        {
            max_variance=current_variance;
            index=i;
        }
    }

    mean_variance/=end-start;
    mean_variance=sqrt(mean_variance);

// 	if(object->lines.size()>20)
// 		goto F2;

    if(mean_variance>config.max_mean_variance)
    {
        recursive_IEPF(object,data,start,index,config);
        recursive_IEPF(object,data,index,end,config);
        return;
    }

// 	F2:

    object->lines.push_back(line);
// 	object->line[object->lines.size()]=(t_line*)malloc(sizeof(t_line));
// 	memcpy(object->line[object->lines.size()],&line,sizeof(t_line));
// 	object->n_lines++;

    return;
}

void free_lines(vector<t_objectPtr> &objects)
{
    for(uint i=0;i<objects.size();i++)
        objects[i]->lines.clear();
}

void recursive_line_fitting(t_objectPtr& object,t_cluster& cluster,t_data& data,t_config& config)
{
    if(!data.n_points)
        return;

    recursive_IEPF(object,data,cluster.stp,cluster.enp,config);
}