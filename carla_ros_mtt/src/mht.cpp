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
\brief MHT main implementation
*/

// System Includes
#include <carla_ros_mtt/mht.h>

using namespace std;

///Initialization of static variables in the Mht::Node class
long Hypothesis::_euid=0;
long Target::_euid=0;
long Target::_ntotal=0;

///Graph context pointer, used in the xgtk plug-in
GVGraph*graph_context;

///Tracking frame, frame id of the current tracking frame
string tracking_frame;

///Mht context class (main workhorse of the mht algorithm)
Mht mht;
hypothesisTreePtr htreePtr;

Publisher markers_publisher;

string fileName;

double euclideanDistance(PointPtr& p1,PointPtr& p2)
{
    return sqrt( pow(p1->x-p2->x,2) + pow(p1->y-p2->y,2));
}

void pointCloud2ToUnorganizedVector(const sensor_msgs::PointCloud2& cloud,vector<PointPtr>& data)
{
    pcl::PointCloud<pcl::PointXYZ> PclCloud;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(cloud, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, PclCloud);

    //get points into grid to reorder them
    for(uint i=0;i<PclCloud.points.size();i++)
    {
        PointPtr pt(new Point);

        pt->x=PclCloud.points[i].x;
        pt->y=PclCloud.points[i].y;
        pt->r=sqrt(pow(pt->x,2)+pow(pt->y,2));
        pt->t=atan2(pt->y,pt->x);
        pt->n=i++;

        data.push_back(pt);
    }
}

void pointCloud2ToVector(const sensor_msgs::PointCloud2& cloud,vector<PointPtr>& data)
{
    pcl::PointCloud<pcl::PointXYZ> PclCloud;

    pcl::fromROSMsg(cloud,PclCloud);

    double theta;
    map<double,pair<uint,double> > grid;
    map<double,pair<uint,double> >::iterator it;

    double spacing = 0.5*M_PI/180.;

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
            if(grid[theta].second>r)
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
        PointPtr pt(new Point);

        pt->x=PclCloud.points[(*it).second.first].x;
        pt->y=PclCloud.points[(*it).second.first].y;
        pt->r=sqrt(pow(pt->x,2)+pow(pt->y,2));
        pt->t=atan2(pt->y,pt->x);
        pt->n=i++;

        data.push_back(pt);
    }
}

void recursiveAssociation(vector<PointPtr>& data,vector<pair<MeasurementPtr,bool> >& point_association,double clustering_distance, uint p)
{
// 	cout<<"recursive on point: "<<p<<endl;

    MeasurementPtr null_ptr;

    for(uint l=0;l<data.size();l++)
    {
        //If the point was not yet associated with anybody
        if(point_association[l].second==false)
        {
            double dist = euclideanDistance(data[p],data[l]);

            //This point associates with the point p
            if(dist<clustering_distance)
            {
                //Put the association pointing to the existing measurement
                point_association[l].first = point_association[p].first;
                point_association[l].second=true;

// 				cout<<"added point: "<<l<<" to measurement: "<<point_association[l].first->id<<endl;

                //Go to the existing measurement and add this new point
                point_association[p].first->points.push_back(data[l]);

                recursiveAssociation(data,point_association,clustering_distance,l);
            }
        }
    }
}

void createObjects(vector<PointPtr>& data,vector<MeasurementPtr>& measurements,double clustering_distance)
{
    static long id=0;

    //Create empty point associations
    vector<pair<MeasurementPtr,bool> > point_association;
    point_association.resize(data.size(),make_pair(MeasurementPtr(),false));

    //Go thought all points in the data vector
    for(uint p=0;p<data.size();p++)
    {
        //If point was not associated with any measurement
        if(point_association[p].second==false)
        {
            //Create a new measurement
            MeasurementPtr new_measurement(new Measurement);

            //Set id and increment static variable
            new_measurement->id=id++;

            //Add this point to the measurement
            new_measurement->points.push_back(data[p]);

            //Make this point associate with the new measurement
            point_association[p].first=new_measurement;
            point_association[p].second=true;

            measurements.push_back(new_measurement);
        }

        //Go though all the other points and see which ones associate with this measurement
        recursiveAssociation(data,point_association,clustering_distance,p);
    }

    for(uint m=0;m<measurements.size();m++)
        measurements[m]->calculateCentroid();
}

void dataHandler(const sensor_msgs::PointCloud2& msg)
{
    //Create the outgoing marker message for targets
    visualization_msgs::MarkerArray targets_markers;
    visualization_msgs::MarkerArray clusters_markers;

    vector<PointPtr> raw_points;
    vector<PointPtr>::iterator raw_it;

    vector<MeasurementPtr> clusters;
    vector<MeasurementPtr>::iterator clusters_it;

    static visualization_msgs::MarkerArray markersMsg;
    static visualization_msgs::MarkerArray markersTreeMsg;

    tracking_frame=msg.header.frame_id;

    //Convert from the pointcloud unsorted structure to a scan like sorted structure
    pointCloud2ToUnorganizedVector(msg,raw_points);
// 	pointCloud2ToVector(msg,raw_points);

    //Create objects, for now is simple clustering
    createObjects(raw_points,clusters,1.5);

    //Lock the draw mutex to avoid trying to draw the tree while it is being processed
    pthread_mutex_lock(&(htreePtr->_draw_mutex));

    //Iterate the new data in the filter, all the major work is done here
    mht.iterate(clusters);

    //Unlock the mutex to allow drawing
    pthread_mutex_unlock(&(htreePtr->_draw_mutex));

    //Obtain the current list of targets, only the most probable of each mht cluster
    vector<TargetPtr> targets = mht.getTargets();

    //Now create a set of markers for the targets
    targets_markers.markers = createTargetMarkers(targets);

    //And a set of markers for the clusters
    clusters_markers.markers = createClustersMarkers(clusters);

    markers_publisher.publish(targets_markers);
    markers_publisher.publish(clusters_markers);

    return;
}

void dataHandlerFromFile_points(vector<PointPtr>& raw_points)
{
    //Create the outgoing marker message for targets
    visualization_msgs::MarkerArray targets_markers;
    visualization_msgs::MarkerArray clusters_markers;

    vector<PointPtr>::iterator raw_it;

    vector<MeasurementPtr> clusters;
    vector<MeasurementPtr>::iterator clusters_it;

    static visualization_msgs::MarkerArray markersMsg;
    static visualization_msgs::MarkerArray markersTreeMsg;

// 	tracking_frame=msg.header.frame_id;
    tracking_frame="/atc/vehicle/center_bumper";

    //Convert from the pointcloud unsorted structure to a scan like sorted structure
// 	pointCloud2ToUnorganizedVector(msg,raw_points);//Do not convert to raw points, in this dataHandler the incoming points are already in the right format
// 	pointCloud2ToVector(msg,raw_points);

    //Create objects, for now is simple clustering
    createObjects(raw_points,clusters,1.0);

    //Lock the draw mutex to avoid trying to draw the tree while it is being processed
    pthread_mutex_lock(&(htreePtr->_draw_mutex));

    //Iterate the new data in the filter, all the major work is done here
    mht.iterate(clusters);

    //Unlock the mutex to allow drawing
    pthread_mutex_unlock(&(htreePtr->_draw_mutex));

    //Obtain the current list of targets, only the most probable of each mht cluster
    vector<TargetPtr> targets = mht.getTargets();

    //Now create a set of markers for the targets
    targets_markers.markers = createTargetMarkers(targets);

    //And a set of markers for the clusters
    clusters_markers.markers = createClustersMarkers(clusters);

    markers_publisher.publish(targets_markers);
    markers_publisher.publish(clusters_markers);

    return;
}

void dataHandlerFromFile_clusters(vector<MeasurementPtr>& clusters)
{
    //Create the outgoing marker message for targets
    visualization_msgs::MarkerArray targets_markers;
    visualization_msgs::MarkerArray clusters_markers;

    vector<MeasurementPtr>::iterator clusters_it;

    static visualization_msgs::MarkerArray markersMsg;
    static visualization_msgs::MarkerArray markersTreeMsg;

// 	tracking_frame=msg.header.frame_id;
    tracking_frame="/atc/vehicle/center_bumper";

    //Convert from the pointcloud unsorted structure to a scan like sorted structure
// 	pointCloud2ToUnorganizedVector(msg,raw_points);//Do not convert to raw points, in this dataHandler the incoming points are already in the right format
// 	pointCloud2ToVector(msg,raw_points);

    //Create objects, for now is simple clustering
// 	createObjects(raw_points,clusters,1.0);//In this version we consider the incoming points as measurements

    //Lock the draw mutex to avoid trying to draw the tree while it is being processed
    pthread_mutex_lock(&(htreePtr->_draw_mutex));

    //Iterate the new data in the filter, all the major work is done here
    mht.iterate(clusters);

    //Unlock the mutex to allow drawing
    pthread_mutex_unlock(&(htreePtr->_draw_mutex));

    //Obtain the current list of targets, only the most probable of each mht cluster
    vector<TargetPtr> targets = mht.getTargets();

    //Now create a set of markers for the targets
    targets_markers.markers = createTargetMarkers(targets);

    //And a set of markers for the clusters
    clusters_markers.markers = createClustersMarkers(clusters);

    markers_publisher.publish(targets_markers);
    markers_publisher.publish(clusters_markers);

    for(uint i=0;i<clusters.size();i++)
        clusters[i]->assigned_clusters.clear();

    return;
}

void Markers::update(visualization_msgs::Marker& marker)
{
    for(uint i=0;i<markers.size();++i)
        if(markers[i].ns==marker.ns && markers[i].id==marker.id)//Marker found
        {
            markers.erase(markers.begin()+i);
            break;
        }

    markers.push_back(marker);
}

void Markers::decrement(void)
{
    for(uint i=0;i<markers.size();++i)
    {
        switch(markers[i].action)
        {
            case visualization_msgs::Marker::ADD:
                markers[i].action = visualization_msgs::Marker::DELETE;
                break;
            case visualization_msgs::Marker::DELETE:
                markers[i].action = -1;
                break;
        }
    }
}

void Markers::clean(void)
{
    vector<visualization_msgs::Marker> new_markers;

    for(uint i=0;i<markers.size();++i)
        if(markers[i].action!=-1)
            new_markers.push_back(markers[i]);

    markers=new_markers;
}

vector<visualization_msgs::Marker> Markers::getOutgoingMarkers(void)
{
    vector<visualization_msgs::Marker> m_out(markers);
    return markers;
}

geometry_msgs::Point makePoint(double x,double y, double z)
{
    geometry_msgs::Point p;
    p.x=x;
    p.y=y;
    p.z=z;
    return p;
}

visualization_msgs::Marker makeEllipse(Vector2d mean, Matrix2d covariance, double threshold,string ns, std_msgs::ColorRGBA color, int id,geometry_msgs::Point position)
{
// 	cout<<"makeEllipse in"<<endl;
    static visualization_msgs::Marker ellipse;
    static bool init = true;

// 	cout<<"mean: "<<mean<<endl;
// 	cout<<"cov: "<<covariance<<endl;

    if(init)
    {
        ellipse.header.frame_id = tracking_frame;
        ellipse.header.stamp = ros::Time::now();

        ellipse.action = visualization_msgs::Marker::ADD;
        ellipse.type = visualization_msgs::Marker::LINE_STRIP;

        ellipse.scale.x = 0.5;
        ellipse.scale.y = 0.5;
        ellipse.scale.z = 0.5;

        init = false;
    }

    ellipse.ns = ns;

// 	ellipse.pose.position=position;

    ellipse.color=color;
    ellipse.id=id;

    ellipse.points.clear();

    assert(is_finite(covariance));
    assert(is_finite(mean));

    MatrixXd data = ellipseParametric(covariance,mean,threshold);

    for(uint c=0;c<data.cols();c++)
    {
        geometry_msgs::Point p;

        p.x = data(0,c);
        p.y = data(1,c);
        p.z = position.z;

        ellipse.points.push_back(p);
    }

// 	cout<<"makeEllipse out"<<endl;

    return ellipse;
}

template <class T>
visualization_msgs::Marker makeTrail(vector<T>& past_states,string ns, std_msgs::ColorRGBA color, int id,geometry_msgs::Point position)
{
// 	cout<<"makeTrail in"<<endl;
    static visualization_msgs::Marker trail;
    static bool init = true;

    if(init)
    {
        trail.header.frame_id = tracking_frame;
        trail.header.stamp = ros::Time::now();

        trail.action = visualization_msgs::Marker::ADD;
        trail.type = visualization_msgs::Marker::LINE_STRIP;

        trail.scale.x = 0.1;
        trail.scale.y = 0.1;
        trail.scale.z = 0.1;

        init = false;
    }

    trail.ns = ns;

    trail.color=color;
    trail.id=id;

    trail.points.clear();

    for(uint i=0;i<past_states.size();i++)
    {
        geometry_msgs::Point p;

        p.x=past_states[i][0];
        p.y=past_states[i][1];
        p.z = position.z;

        trail.points.push_back(p);
    }

// 	cout<<"makeTrail out"<<endl;

    return trail;
}
vector<visualization_msgs::Marker> createTargetMarkers(vector<TargetPtr>& targets)
{
// 	cout<<"createTargetMarkers in"<<endl;
    static Markers marker_list;

    //Reduce the elements status, ADD to REMOVE and REMOVE to delete
    marker_list.decrement();

    //Create a color map
    class_colormap colormap("hsv",10, 1, false);

    visualization_msgs::Marker marker_ids;
    visualization_msgs::Marker marker_centers;
    visualization_msgs::Marker marker_velocity;
    visualization_msgs::Marker marker_association;

    marker_ids.header.frame_id = tracking_frame;
    marker_ids.header.stamp = ros::Time::now();

    marker_centers.header.frame_id = tracking_frame;
    marker_centers.header.stamp = marker_ids.header.stamp;

    marker_velocity.header.frame_id = tracking_frame;
    marker_velocity.header.stamp = marker_ids.header.stamp;

    marker_association.header.frame_id = tracking_frame;
    marker_association.header.stamp = marker_ids.header.stamp;

    marker_ids.ns = "ids";
    marker_ids.action = visualization_msgs::Marker::ADD;

    marker_centers.ns = "target_centers";
    marker_centers.action = visualization_msgs::Marker::ADD;

    marker_velocity.ns = "velocity";
    marker_velocity.action = visualization_msgs::Marker::ADD;

    marker_association.ns = "associations";
    marker_association.action = visualization_msgs::Marker::ADD;

    marker_ids.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_centers.type = visualization_msgs::Marker::POINTS;
    marker_velocity.type = visualization_msgs::Marker::ARROW;
    marker_association.type = visualization_msgs::Marker::LINE_LIST;

    marker_ids.scale.x = 2.5;
    marker_ids.scale.y = 2.5;
    marker_ids.scale.z = 2.5;

    marker_centers.scale.x = 1.4;
    marker_centers.scale.y = 1.4;
    marker_centers.scale.z = 1.4;

    marker_velocity.scale.x = 0.4;
    marker_velocity.scale.y = 0.5;
    marker_velocity.scale.z = 0.8;

    marker_association.scale.x = 0.8;

    marker_centers.color.r = 0;
    marker_centers.color.g = 0;
    marker_centers.color.b = 0;
    marker_centers.color.a = 1;

    marker_ids.color.r=0;
    marker_ids.color.g=0;
    marker_ids.color.b=0;
    marker_ids.color.a=1;

    marker_association.color.r = 0;
    marker_association.color.g = 0;
    marker_association.color.b = 0;
    marker_association.color.a = 1;

    marker_centers.points.resize(targets.size());
    marker_centers.colors.resize(targets.size());

    marker_centers.id = 0;

    double velocity_scale=0.5;

    cout<<"number of targets: "<<targets.size()<<endl;
// 	assert(targets.size()==10);

    for(uint i=0;i<targets.size();i++)
    {
// 		cout<<*targets[i]<<endl;
// 		cout<<"t1"<<endl;

        Color color(colormap.color(targets[i]->_exuid));

        if(targets[i]->_missed_associations>0)
            color.changeColorBrightness(0.9);

// 		cout<<"t2"<<endl;
// 		double velocity = sqrt( pow(targets[i]->_x(2),2)+pow(targets[i]->_x(3),2));
        double velocity = targets[i]->_x(2);

        marker_centers.colors[i] = colormap.color(targets[i]->_id);

        marker_centers.points[i].x = targets[i]->_x(0);
        marker_centers.points[i].y = targets[i]->_x(1);
        marker_centers.points[i].z = 0.0;

        marker_ids.pose.position.x = targets[i]->_x(0);
        marker_ids.pose.position.y = targets[i]->_x(1);
        marker_ids.pose.position.z = 1.0;

// 		cout<<"t3"<<endl;
        boost::format fm("%ld (%ld)");
        fm % targets[i]->_exuid % targets[i]->_cluster;

        marker_ids.text = fm.str();

// 		cout<<"exuid: "<<targets[i]->_exuid<<endl;
        marker_ids.id = targets[i]->_id;

        marker_list.update(marker_ids);

// 		cout<<"t4"<<endl;

        if(velocity>0.1)
        {
            marker_velocity.points.clear();
            marker_velocity.points.resize(2);

            marker_velocity.color = colormap.color(targets[i]->_id);

            marker_velocity.points[0].x = targets[i]->_x(0);
            marker_velocity.points[0].y = targets[i]->_x(1);
            marker_velocity.points[0].z = 0.0;

// 			marker_velocity.points[1].x = targets[i]->_x(0) + targets[i]->_x(2)*velocity_scale;
// 			marker_velocity.points[1].y = targets[i]->_x(1) + targets[i]->_x(3)*velocity_scale;

            marker_velocity.points[1].x = targets[i]->_x(0) + targets[i]->_x(2)*cos(targets[i]->_x(3))*velocity_scale;
            marker_velocity.points[1].y = targets[i]->_x(1) + targets[i]->_x(2)*sin(targets[i]->_x(3))*velocity_scale;

            marker_velocity.points[1].z = 0.0;

            marker_velocity.id=targets[i]->_id;

            marker_list.update(marker_velocity);
        }

// 		cout<<"t5"<<endl;

        //Uncertainty ellipse
        visualization_msgs::Marker ellipse = makeEllipse(targets[i]->_xp.head(2),targets[i]->_P.topLeftCorner(2,2),mht.getMaxGating(),"search areas",color,targets[i]->_id,makePoint(0,0,0));

// 		cout<<"t5.1"<<endl;

        marker_list.update(ellipse);

        //Hypothesis id
        visualization_msgs::Marker ellipse_hypothesis = makeEllipse(targets[i]->_xp.head(2),targets[i]->_P.topLeftCorner(2,2),mht.getMaxGating(),"hypothesis cluster",makeColor(targets[i]->_cluster),targets[i]->_id,makePoint(0,0,-0.15));

// 		cout<<"t6"<<endl;

        marker_list.update(ellipse_hypothesis);

        //Past positions

        visualization_msgs::Marker trail = makeTrail(targets[i]->_past_states,"trail",makeColor(targets[i]->_id),targets[i]->_id,makePoint(0,0,-0.15));

        marker_list.update(trail);

        //Association lines
        geometry_msgs::Point start;
        geometry_msgs::Point end;

        start.x = targets[i]->_x(0);
        start.y = targets[i]->_x(1);
        start.z = 0;

        end.x = targets[i]->_z(0);
        end.y = targets[i]->_z(1);
        end.z = 0;

// 		cout<<"t7"<<endl;

        marker_association.points.push_back(start);
        marker_association.points.push_back(end);

        marker_association.colors.push_back(colormap.color(targets[i]->_exuid));
        marker_association.colors.push_back(colormap.color(targets[i]->_exuid));
    }

// 	cout<<"out of targets"<<endl;

    marker_list.update(marker_centers);
    marker_list.update(marker_association);

    //Remove markers that should not be transmitted
    marker_list.clean();

// 	cout<<"createTargetMarkers out"<<endl;

    //Clean the marker_vector and put the new markers in it
    return marker_list.getOutgoingMarkers();
}

std_msgs::ColorRGBA makeColor(double r,double g, double b, double a)
{
    std_msgs::ColorRGBA color;

    color.r=r;
    color.g=g;
    color.b=b;
    color.a=a;

    return color;
}

std_msgs::ColorRGBA makeColor(int id)
{
    static class_colormap colormap("hsv",10, 1, false);
    return colormap.color(id);
}

vector<visualization_msgs::Marker> createClustersMarkers(vector<MeasurementPtr>& clusters)
{
    static Markers marker_list;

    //Reduce the elements status, ADD to REMOVE and REMOVE to delete
    marker_list.decrement();

    //Create a color map
    class_colormap colormap("hsv",10, 1, false);

    visualization_msgs::Marker points;
    visualization_msgs::Marker spheres;

    points.header.frame_id = tracking_frame;
    points.header.stamp = ros::Time::now();

    spheres.header.frame_id = tracking_frame;
    spheres.header.stamp = ros::Time::now();

    points.ns = "clusters";
    points.action = visualization_msgs::Marker::ADD;
    points.id=0;

    spheres.ns = "clusters_centers";
    spheres.action = visualization_msgs::Marker::ADD;

    points.type = visualization_msgs::Marker::POINTS;
    spheres.type = visualization_msgs::Marker::SPHERE;

    points.color.a = 1;

// 	points.scale.x = 0.05;
// 	points.scale.y = 0.05;
// 	points.scale.z = 0.05;

    points.scale.x = 0.15;
    points.scale.y = 0.15;
    points.scale.z = 0.15;

    spheres.scale.x = 0.1;
    spheres.scale.y = 0.1;
    spheres.scale.z = 0.1;

    spheres.pose.orientation.x = 0.0;
    spheres.pose.orientation.y = 0.0;
    spheres.pose.orientation.z = 0.0;
    spheres.pose.orientation.w = 1.0;

    for(uint i=0;i<clusters.size();++i)
    {
        for(uint f=0;f<clusters[i]->points.size();++f)
        {
            geometry_msgs::Point p;

            p.x = clusters[i]->points[f]->x;
            p.y = clusters[i]->points[f]->y;
            p.z = 0.0;

            points.points.push_back(p);
            points.colors.push_back(colormap.color(i));
        }

        spheres.pose.position.x=clusters[i]->centroid.x;
        spheres.pose.position.y=clusters[i]->centroid.y;

        spheres.id=i;

        spheres.color = colormap.color(i);

        marker_list.update(spheres);
    }

    marker_list.update(points);

    //Remove markers that should not be transmitted
    marker_list.clean();

    //Clean the marker_vector and put the new markers in it
    return marker_list.getOutgoingMarkers();
}

void* graphicalThread(void*data)
{
    graph_context=new GVGraph("graph");

    gvconfig_plugin_install_from_library(graph_context->getGVCcontext(), NULL, &gvplugin_xgtk_LTX_library);

    graph_context->applyLayout();
    graph_context->startRender();

    delete graph_context;

    return NULL;
}

int main(int argc,char**argv)
{
    ///Additional thread to run the graphviz plug-in
    pthread_t graph_thread;

    // Initialize ROS
    init(argc,argv,"mht");

    NodeHandle nh("~");

    markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("/targets", 1000);

    htreePtr = mht.getHypothesisTree();

    //Allways use the same file
    fileName = argv[1];
// 	fileName += lexical_cast<string>(0);
    fileName += ".txt";


    for(uint kk=3;kk<4;kk++)
    {
        mht._k=kk;
        mht._j=4;

        mht._aux1 = "/home/atlas/Dropbox/taem/matlab/trials/roundabout/report_rb_";
// 		mht._aux1 = "/home/atlas/Desktop/report_rb_";
        mht._aux1+=lexical_cast<string>(mht._k);
        mht._aux1+=lexical_cast<string>(mht._j);
        mht._aux1+="_";

        cout<<"Reading data from file: "<<fileName<<endl;

        string report = mht.getReportName("");
        cout<<"report name: "<<report<<endl;

        if(remove(report.c_str()) != 0 )
            perror( "Error deleting file" );

        report = mht.getReportName("_dist");

        if(remove(report.c_str()) != 0 )
            perror( "Error deleting file" );

        report = mht.getReportName("_total");

        if(remove(report.c_str()) != 0 )
            perror( "Error deleting file" );

        ifstream file (fileName.c_str());
        if(!file.is_open())
        {
            cout<<"error opening file: "<<fileName<<endl;
            return 1;
        }

        string line;

// 		cout<<"h"<<endl;
        while(file.good() && ros::ok())
        {
            getline(file,line);

// 			cout<<"line: "<<line<<endl;

            if(line[0]!='%')
                continue;

            int it;

            getline(file,line);//Get iteration
            sscanf(line.c_str(),"%*c %d",&it);

            cout<<"iteration: "<<it<<endl;

            vector<MeasurementPtr> measurements;

            do
            {
// 				PointPtr point(new Point);
                double x,y,r;

                MeasurementPtr measurement(new Measurement);

                getline(file,line);//Get target

                if(line[0]=='-')
                    break;

                sscanf(line.c_str(),"%ld %lf %lf %lf",&measurement->id,&x,&y,&r);

                measurement->state[0]=x;
                measurement->state[1]=y;
                measurement->state[2]=r;

// 				measurement->points.push_back(point);
// 				measurement->calculateCentroid();

                measurements.push_back(measurement);

                cout<<"target: "<<measurement->id<<" x: "<<x<<" y: "<<y<<" r: "<<r<< endl;

            }while(line[0] != '-');

            cout<<"measurements size: "<<measurements.size()<<endl;

            //Now i have the target data from iteration it
            //I must call the datahandler
            dataHandlerFromFile_clusters(measurements);

// 			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
// 			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            boost::this_thread::sleep(boost::posix_time::milliseconds(50));
// 			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
// 			boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        }

        cout<<"close file"<<endl;
        file.close();

    }

    pthread_cancel(graph_thread);

    cout<<"return"<<endl;
    return 0;



// 	pthread_create( &graph_thread, NULL, graphicalThread,NULL);

// 	Subscriber data_handler = nh.subscribe("/points", 1000, dataHandler);

// 	cout<<"Subscribed to "<<names::remap("/points")<<endl;

// 	cout<<"Spinning"<<endl;

// 	mht._k=3;
// 	mht._j=4;

// 	spin();

// 	pthread_cancel(graph_thread);

// 	return 0;



// 	/*
//
// 	for(uint kk=2;kk<=10;kk++)
// 	{
//
// 	uint c = 0;
//
// 	while(ros::ok())
// 	{
// 		fileName = argv[1];
// 		fileName += lexical_cast<string>(c);
// 		fileName += ".txt";
//
// // 		fileName = "/home/atlas/Desktop/trials/data_85.txt";
//
// 		c++;
//
// // 		if(c>10)
// // 		{
// // 			cout<<"force quit! next report: "<<mht.getReportName()<<endl;
// // 			exit(0);
// // 		}
//
// 		mht.clear();
//
// 	if(1)//From file
// 	{
//
// // 		string file_name = "/home/atlas/Desktop/trials/data_0.txt";
//
// 		cout<<"Reading data from file: "<<fileName<<endl;
//
// 		mht._k=kk;
// // 		mht._k=5;
// 		mht._j=10;
//
// 		mht._aux1 = "/home/atlas/Dropbox/taem/matlab/trials/MKJ/report_M";
// 		mht._aux1+=lexical_cast<string>(mht._k);
// 		mht._aux1+=lexical_cast<string>(mht._j);
// 		mht._aux1+="_";
//
// // 		mht._aux1 = "/home/atlas/Dropbox/taem/matlab/trials/MKJ/report_M58_";
//
// 		string report = mht.getReportName("");
//
// 		if(remove(report.c_str()) != 0 )
// 			perror( "Error deleting file" );
//
// 		report = mht.getReportName("_dist");
//
// 		if(remove(report.c_str()) != 0 )
// 			perror( "Error deleting file" );
//
//
// // 		cout<<"report name: "<<report<<endl;
// // 		string cmd = "rm ";
// // 		cmd+=report;
// // 		int ret;
// // 		ret = system(cmd.c_str());
//
// // 		cout<<"removing: "<<cmd<<endl;
//
// 		ifstream file (fileName.c_str());
// 		if(file.is_open())
// 		{
// 			string line;
//
// 			while(file.good() && ros::ok())
// 			{
// 				getline(file,line);
//
// 				if(line[0]!='%')
// 					continue;
//
// 				int it;
//
// 				getline(file,line);//Get iteration
// 				sscanf(line.c_str(),"%*c %d",&it);
//
// 				cout<<"iteration: "<<it<<endl;
//
// // 				vector<Mht::PointPtr> points;
// 				vector<Mht::MeasurementPtr> measurements;
//
// 				do
// 				{
// 					Mht::PointPtr point(new Mht::Point);
// 					Mht::MeasurementPtr measurement(new Mht::Measurement);
//
// 					getline(file,line);//Get target
//
// 					if(line[0]=='-')
// 						break;
//
// // 					sscanf(line.c_str(),"%lf %lf %lf",&point->n,&point->x,&point->y);
// 					sscanf(line.c_str(),"%ld %lf %lf",&measurement->id,&point->x,&point->y);
//
// 					measurement->points.push_back(point);
// 					measurement->calculateCentroid();
//
// 					measurements.push_back(measurement);
//
// // 					cout<<"target: "<<measurement->id<<" x: "<<point->x<<" y: "<<point->y<<endl;
//
// 				}while(line[0] != '-');
//
// // 				dataHandlerFromFile_points(points);
// 				dataHandlerFromFile_clusters(measurements);
//
// 				//Now i have the target data from iteration it
// 				//I must now wait a bit (variable amount)
// 				//And then call the dataHandler
// // 				boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
// 				boost::this_thread::sleep(boost::posix_time::milliseconds(10));
// // 				boost::this_thread::sleep(boost::posix_time::milliseconds(50));
// // 				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
// // 				boost::this_thread::sleep(boost::posix_time::milliseconds(100));
// 			}
//
// 			file.close();
//
// // 			return 0;
//
// 			cout<<"file closed"<<endl;
// 		}else
// 		{
// 			cout<<"file is not open"<<endl;
// 			break;
// // 			return 0;
// 		}
//
// // 		pthread_cancel(graph_thread);
// 	}else
// 	{
// 		Subscriber data_handler = nh.subscribe("/points", 1000, dataHandler);
//
// 		cout<<"Subscribed to "<<names::remap("/points")<<endl;
//
// 		cout<<"Spinning"<<endl;
//
// 		spin();
//
// 		pthread_cancel(graph_thread);
// 	}
//
// 	}
//
// 	}
//
// 	return 0;*/
}