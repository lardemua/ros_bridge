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
\brief Line fitting experimental code
*/

// System Includes
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <carla_ros_mtt/TargetListPC.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <iostream>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.8/pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <carla_ros_mtt/mtt_common.h>

using namespace std;

ros::Publisher target_publisher;
ros::Publisher target_publisher;
ros::Publisher markers_publisher;
string frame_id;
ros::Time message_stamp;
map<pair<string,int>, pair<visualization_msgs::Marker,int> > marker_map;
int marker_id=0;
double max_mean_variance=0.03;
double clustering_threshold=0.3;
double angular_resolution=3*M_PI/180.;

double point2point_distance(double xi,double yi,double xf,double yf)
{	return sqrt((xi-xf)*(xi-xf)+(yi-yf)*(yi-yf));}

double point2line_distance(double alpha,double ro,double x,double y)
{	return ro-x*cos(alpha)-y*sin(alpha);}

void free_lines(vector<t_objectPtr> &objects)
{
    for(uint i=0;i<objects.size();i++)
        objects[i]->lines.clear();
}

void cleanObjets(vector<t_objectPtr> &objects)
{
    free_lines(objects);
}

void calcObjectProps(vector<t_objectPtr> &objects)
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

void recursive_IEPF(t_objectPtr& object,t_data& data,int start,int end)
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

    if(mean_variance>max_mean_variance)
// 		config.max_mean_variance)
    {
        recursive_IEPF(object,data,start,index);
        recursive_IEPF(object,data,index,end);
        return;
    }

    object->lines.push_back(line);
    return;
}

void recursive_line_fitting(t_objectPtr& object,t_cluster& cluster,t_data& data)
{
    if(!data.n_points)
        return;

    recursive_IEPF(object,data,cluster.stp,cluster.enp);
}

bool clustersToObjects(vector<t_clustersPtr> &clusters,t_data& data,vector<t_objectPtr> &objectsPtr)
{
    t_objectPtr object(new t_object);

    objectsPtr.clear();

    for(uint i=0;i<clusters.size();i++)
    {
        object->rmin=clusters[i]->rmin;
        object->tm=clusters[i]->tm;
        object->object_found=false;

        object->partialy_occluded=clusters[i]->partialy_occluded;

        recursive_line_fitting(object,*clusters[i],data);

        objectsPtr.push_back(object);
        object.reset(new t_object);
    }

    return true;
}

void calcClusterProps(vector<t_clustersPtr> &clusters,t_data& data)
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

bool clustering(t_data& data,vector<t_clustersPtr> &clusters)
{
    double x,y,xold=0,yold=0;
    double dist,threshold;

    t_clustersPtr cluster(new t_cluster);

    clusters.clear();

    cluster->id=clusters.size();

    for(int i=0;i<data.n_points;i++)
    {
        x=data.x[i];
        y=data.y[i];

        if(i>0 && i<data.n_points-1)
        {
            dist = sqrt( pow(x-xold,2) + pow(y-yold,2));		//compute distance

            threshold=clustering_threshold;

            if(dist>threshold)
            {
                cluster->enp=i-1;		//set the last cluster endpoint
                cluster->n_points=cluster->enp-cluster->stp;	//sets the number of points in the cluster
                cluster->partialy_occluded=false;
                clusters.push_back(cluster);

                cluster.reset(new t_cluster);

                cluster->id=clusters.size();
                cluster->stp=i;											//sets the new cluster start and end point
                cluster->lenght=0;
            }

            if(i==(data.n_points-1))//last point
            {
                //in case all points are in the same cluster
                cluster->enp=i;
                cluster->n_points=cluster->enp-cluster->stp;	//sets the number of points in the cluster
            }

        }else if(i==data.n_points-1)//end last cluster
        {
            cluster->enp=i;		//set the last cluster endpoint
            cluster->n_points=cluster->enp-cluster->stp;	//sets the number of points in the cluster
            cluster->partialy_occluded=false;
            clusters.push_back(cluster);

        }else if(i==0)//first point
        {
            cluster->stp=0;
            cluster->enp=0;
            cluster->lenght=0;
        }

        xold=x;
        yold=y;
    }

    return true;
}

void pointCloud2ToData(const sensor_msgs::PointCloud2& cloud,t_data& data)//this function will convert the point cloud data into a laser scan type structure
{
    pcl::PointCloud<pcl::PointXYZ> PclCloud;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(cloud, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, PclCloud);


    double theta;

    //map <angle, pair<id, range> >
    map<double,pair<uint,double> > grid;
    map<double,pair<uint,double> >::iterator it;

    double spacing = angular_resolution;

    double rightBorder;
    double leftBorder;

    double r;

    //get points into grid to reorder them
    for(uint i=0;i<PclCloud.points.size();i++)
    {
        theta=atan2(PclCloud.points[i].y,PclCloud.points[i].x);
        r=sqrt(pow(PclCloud.points[i].x,2)+pow(PclCloud.points[i].y,2));

        //get closest theta, given a predefined precision
        rightBorder = spacing * ceil(theta/spacing);
        leftBorder = rightBorder - spacing;

        if(fabs(rightBorder-theta)<fabs(leftBorder-theta))
            theta=rightBorder;
        else
            theta=leftBorder;

        if(grid.find(theta)!=grid.end())
        {
            if(r<grid[theta].second)//using closest measurement
            {
                grid[theta].first=i;
                grid[theta].second=r;
            }
        }else
        {
            grid[theta].first=i;
            grid[theta].second=r;
        }
    }

    uint i=0;
    for(it=grid.begin();it!=grid.end();it++)
    {
        //the map auto orders the theta values
        data.x[i]=PclCloud.points[(*it).second.first].x;
        data.y[i]=PclCloud.points[(*it).second.first].y;
        data.r[i]=sqrt(pow(data.x[i],2)+pow(data.y[i],2));
        data.t[i]=atan2(data.y[i],data.x[i]);

        i++;
    }

    data.n_points=grid.size();
}

void convertObjectsToTargetListPC(vector<t_objectPtr> &objects,mtt::TargetListPC& targets)
{
    //the outgoing message must be empty

    pcl::PointCloud<pcl::PointXYZ> target_positions;
    pcl::PointCloud<pcl::PointXYZ> velocity;

    targets.header.stamp = ros::Time::now();
    targets.header.frame_id = frame_id;

    for(uint i=0;i<objects.size();i++)
    {
        //para cada objecto preencher o topo

        targets.id.push_back(i);

        pcl::PointXYZ position;

        position.x = objects[i]->cx;
        position.y = objects[i]->cy;
        position.z = 0;

        target_positions.points.push_back(position);

        pcl::PointXYZ vel;

        vel.x=0;
        vel.y=0;
        vel.z=0;

        velocity.points.push_back(vel);

        //Now i must copy shape
        pcl::PointCloud<pcl::PointXYZ> shape;
        pcl::PointXYZ line_point;

        line_point.z=0;

        uint j;
        for(j=0;j<objects[i]->lines.size();j++)
        {
            line_point.x=objects[i]->lines[j]->xi;
            line_point.y=objects[i]->lines[j]->yi;

            shape.points.push_back(line_point);
        }

        line_point.x=objects[i]->lines[j-1]->xf;
        line_point.y=objects[i]->lines[j-1]->yf;

        shape.points.push_back(line_point);

        sensor_msgs::PointCloud2 shape_cloud;
        pcl::toROSMsg(shape,shape_cloud);

        shape_cloud.header.stamp=message_stamp;
        shape_cloud.header.frame_id=frame_id;

        targets.obstacle_lines.push_back(shape_cloud);
    }

    pcl::toROSMsg(target_positions, targets.position);

    targets.position.header.stamp=message_stamp;
    targets.position.header.frame_id=frame_id;

    pcl::toROSMsg(velocity, targets.velocity);

    targets.velocity.header.stamp=message_stamp;
    targets.velocity.header.frame_id=frame_id;
}

void cleanMarkers(void)
{
    map<pair<string,int>, pair<visualization_msgs::Marker,int> >::iterator it;

    //Put all static objects to a delete state or overdeleted state (decrement the index)
    for(it=marker_map.begin();it!=marker_map.end();it++)
        it->second.second--;

    marker_id=0;
}

void maintenanceMarkers(vector<visualization_msgs::Marker>& markers)
{
    map<pair<string,int>, pair<visualization_msgs::Marker,int> >::iterator it;

    for(it=marker_map.begin();it!=marker_map.end();it++)
    {
        if( it->second.second==0 )//0 means that this object is to delete
            it->second.first.action = visualization_msgs::Marker::DELETE;
        else if(it->second.second<=-1)//has already been deleted
        {
            /**
            \todo This should be erased but it is not working now
             marker_map.erase(it);
            */
            continue;
        }

        markers.push_back(it->second.first);//push to marker vector
    }
}

void createMarkers(mtt::TargetListPC& targets)
{
    std_msgs::ColorRGBA color;
    class_colormap colormap("hsv",10, 1, false);

    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;

    marker.ns = "ids";
    marker.pose.position.z=0.3;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r=0;
    marker.color.g=0;
    marker.color.b=0;
    marker.color.a=1;

    pcl::PointCloud<pcl::PointXYZ> positions;
    pcl::fromROSMsg(targets.position,positions);

    for(uint i=0;i<positions.points.size();i++)
    {
        marker.pose.position.x=positions.points[i].x;
        marker.pose.position.y=positions.points[i].y;

        marker.text = boost::lexical_cast<string>(targets.id[i]);

        marker.id=marker_id++;
        marker_map[make_pair(marker.ns,marker.id) ] = make_pair(marker,1);//isto substitui ou cria o novo marker no map
    }

    // Markers for Line objects
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.ns = "objects";

    marker.pose.position.x=0;
    marker.pose.position.y=0;
    marker.pose.position.z=0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    for(uint i=0;i<targets.obstacle_lines.size();i++)
    {
        marker.color = colormap.color(i);

        geometry_msgs::Point p;
        p.z = 0.1;

        marker.points.clear();

        uint l;

        pcl::PointCloud<pcl::PointXYZ> shape;
        pcl::fromROSMsg(targets.obstacle_lines[i],shape);

        for(l=0;l<shape.points.size();l++)
        {
            p.x = shape.points[l].x;
            p.y = shape.points[l].y;
            p.z = 0;

            marker.points.push_back(p);
        }

        marker.id=marker_id++;
        marker_map[make_pair(marker.ns,marker.id) ] = make_pair(marker,1);//isto substitui ou cria o novo marker no map
    }
}

void markersFromData(t_data& data)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;

    marker.ns = "data";
    marker.pose.position.z=0.0;
    marker.type = visualization_msgs::Marker::POINTS;

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.color.r=0;
    marker.color.g=0;
    marker.color.b=1;
    marker.color.a=1;

    for(int i=0;i<data.n_points;i++)
    {
        geometry_msgs::Point p;

        p.x=data.x[i];
        p.y=data.y[i];
        p.z=0;

        marker.points.push_back(p);
    }

    marker.id=marker_id++;
    marker_map[make_pair(marker.ns,marker.id) ] = make_pair(marker,1);//isto substitui ou cria o novo marker no map
}

void pointsHandler(const sensor_msgs::PointCloud2& points)
{
    t_data data;//data holding structure
    vector<t_clustersPtr> clusters;//cluster vector
    vector<t_objectPtr> objects;//object vector
    mtt::TargetListPC targets;//outgoing target message
    visualization_msgs::MarkerArray markers;//Marker vector

    //Copy message stamp and frame to global var to be used in the outgoing messages
    frame_id=points.header.frame_id;
    message_stamp=points.header.stamp;

    //Convert the point cloud data into a laser scan type structure
    pointCloud2ToData(points,data);

    //Cluster points
    clustering(data,clusters);

    //Calculate cluster properties
    calcClusterProps(clusters,data);

    //Do recursive line fitting
    clustersToObjects(clusters,data,objects);

    //Calculate objects properties, center, size ...
    calcObjectProps(objects);

    //Convert objects to the target message
    convertObjectsToTargetListPC(objects,targets);

    //Clean makers
    cleanMarkers();

    //Create markers from targets
    createMarkers(targets);

    //Create points marker from data
    markersFromData(data);

    //Copy marker map to actual marker vector
    maintenanceMarkers(markers.markers);

    //Send targets
    target_publisher.publish(targets);

    //Send markers
    markers_publisher.publish(markers);

    //Clean current objects
    cleanObjets(objects);
}

int main(int argc,char**argv)
{
    // Initialize ROS
    ros::init(argc,argv,"mtt");

    ros::NodeHandle nh("~");

    ros::Subscriber subs_points = nh.subscribe("/points", 1000, pointsHandler);
    target_publisher =  nh.advertise<mtt::TargetListPC>("/targets", 1000);
    markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("/markers", 1000);

    visualization_msgs::MarkerArray markersMsg;

    ros::spin();

    return 0;
}
