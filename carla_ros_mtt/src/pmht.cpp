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
\brief Source code for the Multi-Hypothesis Tracking data association algorithm
For additional info on the Mht algorithm check \ref mhtpage "here".
*/

// System Includes
#include <carla_ros_mtt/pmht.h>

///Main super hyper tree
tree<t_nodePtr> global_tree;

///Xgtk external graphviz library/plugin (only plugin, there's no need to create a library since it can be linked internally.
extern gvplugin_library_t gvplugin_xgtk_LTX_library;

///Graph context pointer, used in the xgtk plug-in
GVGraph*graph_context;

///Marker publisher
Publisher markers_publisher;

///Tree marker publisher, only leafs
Publisher markers_tree_publisher;

///Tracking frame, frame id of the current tracking frame
string tracking_frame;

///Max Mahalanobis distance used in gating
double max_mahalanobis=7;

///Additional thread to run the graphviz plug-in
pthread_t graph_thread;

///Current iteration counter, used in the naming of tree nodes
long iteration=0;


void CreateTreeMarker(vector<visualization_msgs::Marker>& mrk,tree<t_nodePtr>::iterator root)
{
    visualization_msgs::Marker marker;

    (*root)->marker.lifetime=ros::Duration(1.0);
// 	marker.lifetime=ros::Duration();

    (*root)->marker.header.frame_id = tracking_frame;
    (*root)->marker.header.stamp = ros::Time::now();

    /* Markers for tracking points */
    (*root)->marker.ns = "nodes";
    (*root)->marker.action = visualization_msgs::Marker::ADD;
    (*root)->marker.pose.orientation.w = 1.0;
    (*root)->marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    (*root)->marker.scale.x = 0.5;
    (*root)->marker.scale.y = 0.5;
    (*root)->marker.scale.z = 0.5;

    (*root)->marker.color.r = 1.0;
    (*root)->marker.color.g = 0.6;
    (*root)->marker.color.b = 0.0;
    (*root)->marker.color.a = 1.0;

    tree<t_nodePtr>::iterator parent = global_tree.parent(root);
    tree<t_nodePtr>::sibling_iterator sibling = root;
    sibling--;

    (*root)->marker.pose.position.y = 2;

    if(parent==0)
        (*root)->marker.pose.position.z = 0;
    else
        (*root)->marker.pose.position.z = (*parent)->marker.pose.position.z + 1.0;

    if(root==global_tree.begin())
        (*root)->marker.pose.position.x = 0;
    else if(sibling==0)
        (*root)->marker.pose.position.x = (*parent)->marker.pose.position.x;
    else
        (*root)->marker.pose.position.x = (*sibling)->marker.pose.position.x + 1.0;

    (*root)->marker.text = boost::lexical_cast<string>((*root)->id) + "." + boost::lexical_cast<string>((*root)->age);
    cout<<"Text:"<<(*root)->marker.text<<endl;

    if(mrk.size()==0)
        (*root)->marker.id=0;
    else
        (*root)->marker.id = mrk[mrk.size()-1].id+1;

    mrk.push_back((*root)->marker);

    tree<t_nodePtr>::sibling_iterator children;

    for(children=global_tree.begin(root);children!=global_tree.end(root);++children)//all siblings
    {
        CreateTreeMarker(mrk,children);
    }
}

MatrixXd EllipseParametric(Matrix2d& cov,Vector2d& mu,double MahThreshold)
{
// 	cout<<"Cov: "<<cov<<endl;
// 	cout<<"Mu: "<<mu<<endl;
// 	cout<<"Thres: "<<MahThreshold<<endl;

    //How to calculate Major and Minor axes and rotation from Covariance?

    //First calculate the eigen values and vectors
    Eigen::SelfAdjointEigenSolver<Matrix2d> eigensolver(cov);
    if(eigensolver.info() != Eigen::Success)
        abort();

    Matrix2d eigvectors=eigensolver.eigenvectors();

    Vector2d ev1=eigvectors.row(0);
    Vector2d ev2=eigvectors.row(1);

    //Use eigenvectors directions to find the major and minor axes
    Vector2d p;

    //Step in the outgoing direction
    double k_step=0.2;
    //Stepping value
    double k=0;
    //Current value for the Mahalanobis distance
    double d;
    //Major and minor axes of the ellipse
    double ea,eb;
    //Angle of the ellipse
    double g;

    //Calc a point along the main direction and get its distance
    do
    {
        p=k*ev1+mu;
        k+=k_step;

        d=Mahalanobis(p,mu,cov);

        if(MahThreshold-d < 0.5)
            k_step=0.01;

    }while(d<MahThreshold);

    k_step=0.2;
    ea=k;

    k=0;
    do
    {
        p=k*ev2+mu;
        k+=k_step;

        d=Mahalanobis(p,mu,cov);

        if(MahThreshold-d < 0.5)
            k_step=0.01;

    }while(d<MahThreshold);

    eb=k;

    g=atan2(ev1(1),ev1(0));

    //Angular step for the parametric curve
    double t_step=0.2;
    //Maximum angle for the parametric curve
    double t_max=2*M_PI+t_step;
    //Total number of nodes of the parametric curve
    double steps=ceil(t_max/t_step);

    MatrixXd el(2,steps);

    for(uint i=0;i<steps;i++)
    {
        double t=i*t_step;
        el(0,i)=mu(0)+ea*cos(t)*cos(g)-eb*sin(t)*sin(g);
        el(1,i)=mu(1)+ea*cos(t)*sin(g)+eb*sin(t)*cos(g);
    }

    return el;
}

void CreateMarkersFromTreeLeafs(vector<visualization_msgs::Marker>& marker_vector)
{
    std_msgs::ColorRGBA color;
    class_colormap colormap("hsv",10, 1, false);

    uint size_markers=0;

    tree<t_nodePtr>::leaf_iterator leaf;

    uint leaf_size=0;
    for(leaf=global_tree.begin_leaf();leaf!=global_tree.end_leaf();leaf++)
        leaf_size++;

    if(marker_vector.size()<2*leaf_size+1)
        marker_vector.resize(2*leaf_size+1);

    for(uint i=0;i<marker_vector.size();i++)
    {
        marker_vector[i].action=visualization_msgs::Marker::DELETE;
        marker_vector[i].header.frame_id=tracking_frame;
    }

    visualization_msgs::Marker marker; //declare the msg

    marker.lifetime=ros::Duration(1.0);

    marker.header.frame_id = tracking_frame;
    marker.header.stamp = ros::Time::now();

    /* Markers for tracking points */
    marker.ns = "centers";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::POINTS;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    color.r=1.0;
    color.g=0.6;
    color.b=0.1;
    color.a=1;

    marker.color = color;

    geometry_msgs::Point p;

// 	cout<<"Centers"<<endl;
    for(leaf=global_tree.begin_leaf();leaf!=global_tree.end_leaf();leaf++)
    {
        p.x = (*leaf)->x(0);
        p.y = (*leaf)->x(1);
        p.z = 0;

        marker.points.push_back(p);
    }

    marker.id=0;//id 0 is reserved for target centers
    marker_vector[size_markers++]=marker;

    /* Markers IDS text */
    color.r=0.;
    color.g=0;
    color.b=0.;
    color.a=1;

    marker.pose.position.z=0.3;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.ns = "ids";

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color = color;

    for(leaf=global_tree.begin_leaf();leaf!=global_tree.end_leaf();leaf++)
    {
        marker.pose.position.x=(*leaf)->x(0);
        marker.pose.position.y=(*leaf)->x(1);

// 		boost::format fmter("%g");
// 		fmter % (double)sqrt( pow((*leaf)->x(2),2)+pow((*leaf)->x(3),2));
// 		marker.text = fmter.str();

        marker.text = boost::lexical_cast<string>((*leaf)->id);

        marker.id=size_markers;
        marker_vector[size_markers++]=marker;
    }

    /* Markers for Line objects */
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.ns = "search areas";

    marker.pose.position.x=0;
    marker.pose.position.y=0;
    marker.pose.position.z=0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    color.a=1;

    for(leaf=global_tree.begin_leaf();leaf!=global_tree.end_leaf();leaf++)
    {
        switch((*leaf)->mode)
        {
            case t_node::NEW:
                color.r=0.;
                color.g=1;
                color.b=0.;
                break;
            case t_node::MAIN:
                color.r=1.;
                color.g=0;
                color.b=0.;
                break;
            case t_node::SIBLING:
                color.r=1.;
                color.g=0.5;
                color.b=0.;
                break;
            case t_node::FAILED:
                color.r=0.5;
                color.g=0;
                color.b=1.;
                break;
            case t_node::TEST:
                color.r=0.;
                color.g=0;
                color.b=0.;
                break;
        }

        marker.color = color;

        p.z = -0.2;

        marker.points.clear();

        Vector2d mu;
        Matrix2d cov;

        mu(0)=(*leaf)->x_p(0);
        mu(1)=(*leaf)->x_p(1);

        cov(0,0)=(*leaf)->P(0,0);
        cov(0,1)=(*leaf)->P(0,1);
        cov(1,0)=(*leaf)->P(1,0);
        cov(1,1)=(*leaf)->P(1,1);

        MatrixXd ellipse = EllipseParametric(cov,mu,max_mahalanobis);

        uint l;
        for(l=0;l<ellipse.cols();l++)
        {
            p.x = ellipse(0,l);
            p.y = ellipse(1,l);

            marker.points.push_back(p);
        }

        marker.id=size_markers;
        marker_vector[size_markers++]=marker;
    }
}

void CreateMarkers(vector<visualization_msgs::Marker>& marker_vector,vector<t_clusterPtr>&clusters)
{
    std_msgs::ColorRGBA color;
    class_colormap colormap("hsv",10, 1, false);

    uint size_markers=0;

    color.r=0.;
    color.g=0.2;
    color.b=1.;
    color.a=1;

    if(marker_vector.size()<2*clusters.size()+1)
        marker_vector.resize(2*clusters.size()+1);

    for(uint i=0;i<marker_vector.size();i++)
    {
        marker_vector[i].action=visualization_msgs::Marker::DELETE;
        marker_vector[i].header.frame_id=tracking_frame;
    }

    visualization_msgs::Marker marker; //declare the msg

    marker.lifetime=ros::Duration(0.2);

    marker.header.frame_id = tracking_frame;
    marker.header.stamp = ros::Time::now();

    /* Markers for tracking points */
    marker.ns = "clusters centers";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::POINTS;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color = color;

    geometry_msgs::Point p;

    for(uint i=0; i<clusters.size(); i++)
    {
        p.x = clusters[i]->centroid.x;
        p.y = clusters[i]->centroid.y;
        p.z = 0;

        marker.points.push_back(p);
    }

    marker.id=0;//id 0 is reserved for target centers
    marker_vector[size_markers++]=marker;

    /* Markers IDS text */
    color.r=0.;
    color.g=0;
    color.b=0.;
    color.a=1;

    marker.ns = "ids";

    marker.pose.position.z=0.3;

    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color = color;

    for(uint i=0;i<clusters.size();i++)
    {
        marker.pose.position.x=clusters[i]->centroid.x;
        marker.pose.position.y=clusters[i]->centroid.y;

        marker.text = boost::lexical_cast<string>(clusters[i]->id);

        marker.id=size_markers;
        marker_vector[size_markers++]=marker;
    }
}


void PointCloud2ToVector(const sensor_msgs::PointCloud2& cloud,vector<t_pointPtr>& data)//this function will convert the point cloud data into a laser scan type structure
{
    pcl::PointCloud<pcl::PointXYZ> PclCloud;

    pcl::fromROSMsg(cloud,PclCloud);

    double theta;
    map<double,pair<uint,double> > grid;
    map<double,pair<uint,double> >::iterator it;

    double spacing = 1.*M_PI/180.;

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
        t_pointPtr pt(new t_point);

        pt->x=PclCloud.points[(*it).second.first].x;
        pt->y=PclCloud.points[(*it).second.first].y;
        pt->r=sqrt(pow(pt->x,2)+pow(pt->y,2));
        pt->t=atan2(pt->y,pt->x);
        pt->n=i++;

        data.push_back(pt);
    }
}

void CreateObjects(vector<t_pointPtr>&data,vector<t_clusterPtr>&clusters,double clustering_distance)
{
    static double id=0;

    vector<t_pointPtr>::iterator p1;
    vector<t_pointPtr>::iterator p2;

    double dist;
    t_clusterPtr cls;

    for(p1=data.begin();p1!=data.end();p1++)
    {
        //do base distance clustering

        if((*p1)->cl.use_count()==0)//this point is not yet associated with anybody
        {
            //create new cluster
            cls.reset(new t_cluster);
            cls->id=id++;

            clusters.push_back(cls);

            (*p1)->cl=cls;

            cls->points.push_back(*p1);//add this point to the cluster
        }

        for(p2=data.begin();p2!=data.end();p2++)//go through all the other points and find points close to this one
        {
            if((*p2)->cl.use_count()==0)//this point is not associated, lets add him to the cluster
            {
                dist=Euclidean_distance(*(*p1),*(*p2));
                if(dist<clustering_distance)
                {
                    (*p1)->cl->points.push_back(*p2);
                    (*p2)->cl=(*p1)->cl;
                }
            }
        }
    }

    vector<t_clusterPtr>::iterator it;

    for(it=clusters.begin();it!=clusters.end();it++)
        (*it)->CalculateCentroid();
}

double Mahalanobis(Vector2d&y,Vector2d&mean,Matrix2d& cov)
{
    VectorXd a=y-mean;
    MatrixXd i= cov.inverse();
    double squared=a.transpose()*i*a;
    return sqrt(squared);
}

double Mahalanobis_distance(t_nodePtr&node,t_clusterPtr&cluster)
{
    //calculate the Mahalanobis between the node and the cluster
    Vector2d measurement;
    Vector2d mean;
    Matrix2d covariance;
    measurement(0)=cluster->centroid.x;
    measurement(1)=cluster->centroid.y;

    mean(0)=node->x_p(0);
    mean(1)=node->x_p(1);

    covariance(0,0)=node->P(0,0);
    covariance(0,1)=node->P(0,1);

    covariance(1,0)=node->P(1,0);
    covariance(1,1)=node->P(1,1);

    return Mahalanobis(measurement,mean,covariance);
}

vector<t_clusterPtr>::iterator GetClosestCandidate(t_nodePtr&node,vector<t_clusterPtr>&clusters)
{
    vector<t_clusterPtr>::iterator it;
    vector<t_clusterPtr>::iterator it_min=clusters.end();

    double distance;
    double min=max_mahalanobis;

    for(it=clusters.begin();it<clusters.end();it++)
    {
        distance=Mahalanobis_distance(node,*it);
        if(distance<min)
        {
            it_min=it;
            min=distance;
        }
    }

    return it_min;//Return an iterator to the closest cluster or the clusters end if none is found
}

vector<vector<t_clusterPtr>::iterator> GateMeasurements(t_nodePtr&node,vector<t_clusterPtr>&clusters, double max_gating)
{
    vector<vector<t_clusterPtr>::iterator> positions;
    vector<t_clusterPtr>::iterator it;

    double gating_distance;
// 	double max_gating_distance = max_mahalanobis;

    for(it=clusters.begin();it<clusters.end();it++)
    {
        gating_distance=Mahalanobis_distance(node,*it);
        if(gating_distance<max_gating)
            positions.push_back(it);
    }

    return positions;
}

void ReAgeTree(void)
{
    tree<t_nodePtr>::iterator it;

    for(it=global_tree.begin();it!=global_tree.end();it++)
        (*it)->IncreaseAge();
}

/**
\brief Remove old nodes from the tree
This will remove root nodes that are too old.\n
The age of a root node is calculated via its maximum depth.\n
The root node must only have one child for this process to work.
\param size age limit, maximum depth for this branch
*/
void RemoveOld(int size)
{
    tree<t_nodePtr>::iterator it;
    tree<t_nodePtr>::iterator it_next;
    tree<t_nodePtr>::iterator it_child;

    for(it=global_tree.begin();it!=0;it=global_tree.next_at_same_depth(it))
    {
        it_next=global_tree.next_at_same_depth(it);

        if(it_next==0)
            break;

        if(global_tree.max_depth(it)>=size)
        {
            it_child=it.node->first_child;
            global_tree.move_ontop_same_branch(it,it_child);
            it=it_child;
        }
    }
}

void nthPruning(int size)
{
    //This pruning will go to the tree at a fixed distance from the leafs and select the correct branch from a few

// 	cout<<"Start pruning"<<endl;

// 	tree<t_nodePtr>::iterator it;
// 	tree<t_nodePtr>::iterator it_next;
// 	tree<t_nodePtr>::iterator it_child;
//
// 	for(it=global_tree.begin();it!=0;it=global_tree.next_at_same_depth(it))
// 	{
// 		it_next=global_tree.next_at_same_depth(it);
//
// 		if(it_next==0)
// 			break;
//
// 		if(global_tree.max_depth(it)>=size)
// 		{
// 			it_child=it.node->first_child;
// 			global_tree.move_ontop_same_branch(it,it_child);
// 			it=it_child;
// 		}
// 	}

// 	cout<<"Done pruning"<<endl;
}

/**
\brief Handler for the incoming data
\param msg incoming point cloud
*/
void DataHandler(const sensor_msgs::PointCloud2& msg)
{
    iteration++;

    vector<t_pointPtr> raw_points;
    vector<t_pointPtr>::iterator raw_it;

    vector<t_clusterPtr> clusters;
    vector<t_clusterPtr>::iterator clusters_it;

    static visualization_msgs::MarkerArray markersMsg;
    static visualization_msgs::MarkerArray markersTreeMsg;

    tracking_frame=msg.header.frame_id;

    //Process new data
// 	cout<<endl<<endl;

    ///Convert from the pointcloud unsorted structure to a scan like sorted structure
    PointCloud2ToVector(msg,raw_points);

    ///Create objects, for now is simple clustering
    CreateObjects(raw_points,clusters,1.5);

    tree<t_nodePtr>::leaf_iterator leaf;
    tree<t_nodePtr>::leaf_iterator leaf_next;
    tree<t_nodePtr>::leaf_iterator leaf_aux;

    global_tree.ready_from_draw=false;
    global_tree.need_layout=true;

    for(leaf=global_tree.begin_leaf();leaf!=global_tree.end_leaf();)
    {
        leaf_next=leaf;
        leaf_next++;
        vector<vector<t_clusterPtr>::iterator> gated_measurements;

        gated_measurements = GateMeasurements(*leaf,clusters,max_mahalanobis);

// 		cout<<"Number of gated measurements:"<<gated_measurements.size()<<endl;

        if(gated_measurements.size()!=0)
        {
// 			if(leaf_next!=global_tree.end_leaf())
// 				cout<<"Leaf:"<<*leaf<<" Next: "<<*leaf_next<<endl;

            cout<<"Number of associations: "<<gated_measurements.size()<<endl;
            for(uint i=0;i<gated_measurements.size();i++)
            {
                //Each measurement that is within the gate will be propagated
                t_nodePtr new_leaf(new t_node(*(*leaf),*(*gated_measurements[i]),iteration));
                global_tree.append_child(leaf,new_leaf);//Append a new node
            }
// 			leaf=leaf_next;
        }else//no associations for this leaf
        {
            //iterate in the empty
            t_nodePtr new_leaf(new t_node(*(*leaf),iteration));

            //I should not remove the tree branch, i should put some kind of stop signal to prevent further processing but keep it
            //in memory, 50 iterations mean 1 second
            if(new_leaf->failed_associations_counter>5)
            {
                leaf_aux=leaf;//save position of leaf to remove
                leaf--;//move the global pointer backward
                global_tree.erase_branch(leaf_aux);//Remove branch from tree
            }else
            {
                global_tree.append_child(leaf,new_leaf);//Append a new node
                leaf++;//Jump over the added node
            }
        }

        leaf=leaf_next;
        // Closest
// 		clusters_it = GetClosestCandidate(*leaf,clusters);

// 		if(clusters_it != clusters.end())
// 		{
        //If association is found (main association) create a new tree node that will be the son of the current node
        //and will inherit all of the father's proprieties
// 			t_nodePtr new_leaf(new t_node(*(*leaf),*(*clusters_it)));
// 			global_tree.append_child(leaf,new_leaf);//Append a new node
// 			leaf++;//Jump over the added node
// 		}else
// 		{
        //iterate in the empty
// 			t_nodePtr new_leaf(new t_node(*(*leaf)));

        //I should not remove the tree branch, i should put some kind of stop signal to prevent further processing but keep it
        //in memory, 50 iterations mean 1 second
// 			if(new_leaf->failed_associations_counter>10)
// 			{
// 				leaf_aux=leaf;//save position of leaf to remove
// 				leaf--;//move the global pointer backward
// 				global_tree.erase_branch(leaf_aux);//Remove branch from tree
// 			}else
// 			{
// 				global_tree.append_child(leaf,new_leaf);//Append a new node
// 				leaf++;//Jump over the added node
// 			}
// 		}
    }

    //For all clusters not associated, create new tree base nodes
    for(clusters_it=clusters.begin();clusters_it!=clusters.end();clusters_it++)
    {
        if( (*clusters_it)->associations.size()==0)//node not associated with anybody
        {
            ///New nodes are created from clusters, maybe in the future they will be created from other type of data
            t_nodePtr node(new t_node( *(*clusters_it),iteration));
            ///Insert into the tree the start clusters, at root
            global_tree.insert(global_tree.begin(),node);
        }
    }


    ReAgeTree();

    RemoveOld(10);
// 	nthPruning(3);

// 	cout<<"Create markers from clusters"<<endl;
    CreateMarkers(markersMsg.markers,clusters);
    markers_publisher.publish(markersMsg);

// 	cout<<"Create markers from leafs"<<endl;
    CreateMarkersFromTreeLeafs(markersTreeMsg.markers);
    markers_tree_publisher.publish(markersTreeMsg);
// 	cout<<"Done"<<endl;


    global_tree.ready_from_draw=true;

// 	kptree::print_tree_bracketed<t_nodePtr>(global_tree);

// 	static uint counter=0;
// 	counter++;

// 	if(counter==4)
// 	{
// 		cout<<"Shut down ros"<<endl;
// 		ros::shutdown();
// 		exit(0);
// 	}
}

void*GraphicalThread(void*data)
{
// 	GVC_t *gvc;

    /* set up a graphviz context */
// 	gvc = gvContext();

    graph_context=new GVGraph("graph");

    gvconfig_plugin_install_from_library(graph_context->getGVCcontext(), NULL, &gvplugin_xgtk_LTX_library);

    /* Create a simple digraph */
// 	g = agopen((char*)"G", AGDIGRAPH);
// 	g = agopen((char*)"G", AGMETAGRAPH);
// 	g = agopen((char*)"G", AGRAPHSTRICT);

    graph_context->applyLayout();
    graph_context->startRender();

    /* Compute a layout using layout engine from command line args */
// 	gvLayout(gvc, g,(char*)"dot");
// 	gvRender(gvc, g,(char*)"xgtk",NULL);

    /* Free layout data */
// 	gvFreeLayout(gvc, g);

    /* Free graph structures */
// 	agclose(g);

    /* close output file, free context, and return number of errors */
// 	gvFreeContext(gvc);

    delete graph_context;

    return NULL;
}

// class treta
// {
// 	public:
// 		treta(int _b=2)
// 		:b(_b)
// 		{
// 		}
//
// 		int b;
// };
//
// typedef boost::shared_ptr<treta> tretaPtr;
//
// class cenas
// {
// 	public:
// 		cenas(int _a=2)
// 		:a(_a)
// 		{
// 		}
//
// 		int a;
//
// 		tretaPtr tp;
//
// 		cenas& operator=(const cenas &rhs)
// 		{
// 			cout<<"Calling this shit"<<endl;
// 			a=rhs.a;
//
// 			tp.reset(new treta);
// 			*tp=*(rhs.tp);
//
// 			return *this;  // Return a reference to myself.
// 		}
// };
//
// typedef boost::shared_ptr<cenas> cenasPtr;
//
// ostream& operator<< (ostream &o, const cenasPtr &i)
// {
// 	if(i->tp)
// 		return o <<"a: "<<i->a<<" b: "<<i->tp->b;
// 	else
// 		return o <<"a: "<<i->a<<" b: "<<"null";
// }

int main(int argc,char**argv)
{


// 	return 0;
// 	cenasPtr cena1(new cenas(8));
// 	cenasPtr cena2(new cenas);
//
// 	cena1->tp.reset(new treta);
//
// 	*cena2=*cena1;
//
// 	cout<<"Cena1: "<<cena1<<endl;
// 	cout<<"Cena2: "<<cena2<<endl;
//
// 	cena1->a=5;
// 	cena1->tp->b=45;
//
// 	cout<<"Cena1: "<<cena1<<endl;
// 	cout<<"Cena2: "<<cena2<<endl;

// 	return 0;

    // Initialize ROS
    init(argc,argv,"mtt");

    NodeHandle nh("~");

    Subscriber data_handler = nh.subscribe("/points", 1000, DataHandler);

    markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("/markersClusters", 1000);

    markers_tree_publisher = nh.advertise<visualization_msgs::MarkerArray>("/markersTree", 1000);

    cout<<"Subscribed to "<<names::remap("/points")<<endl;
    cout<<"Spinning"<<endl;

// 	spin();

// 	t_cluster c1;
// 	c1.id=1;
// 	c1.centroid.x=0;
// 	c1.centroid.y=0;
// 	c1.centroid.z=0;
//
// 	t_cluster c2;
// 	c2.id=2;
// 	c2.centroid.x=1;
// 	c2.centroid.y=1;
// 	c2.centroid.z=0;
//
// 	t_cluster c3;
// 	c3.id=3;
// 	c3.centroid.x=1;
// 	c3.centroid.y=1;
// 	c3.centroid.z=0;
//
// 	t_cluster c4;
// 	c4.id=4;
// 	c4.centroid.x=1;
// 	c4.centroid.y=1;
// 	c4.centroid.z=0;
//
// 	t_nodePtr node1(new t_node(c1));
// 	t_nodePtr node2(new t_node(c2));
// 	t_nodePtr node3(new t_node(c3));
// 	t_nodePtr node4(new t_node(c4));
//
//
// 	tree<t_nodePtr>::iterator it1;
// 	tree<t_nodePtr>::iterator it2;
// 	tree<t_nodePtr>::iterator it3;
// 	tree<t_nodePtr>::iterator it4;

// 	t_nodePtr node(new t_node(c1));
// 	old_node=node;

// 	it1=global_tree.insert(global_tree.begin(),node1);
// 	it2=global_tree.insert(global_tree.begin(),node2);
// 	it3=global_tree.insert(global_tree.begin(),node3);
// 	it4=global_tree.insert(global_tree.begin(),node4);

// 	node.reset(new t_node(c2));
// 	global_tree.insert(global_tree.begin(),node);

// 	node.reset(new t_node(*node));

// 	node->local_cluster.id=5;

// 	ReAgeTree();

// 	global_tree.append_child(it,node);

// 	node.reset(new t_node(*node));
// 	node->id=2;
// 	global_tree.append_child(it,node);


    kptree::print_tree_bracketed<t_nodePtr>(global_tree);

    pthread_create( &graph_thread, NULL, GraphicalThread,NULL);

    spin();

// 	while(1)
// 	{
// 		usleep(100000);
// 	}
// 	kptree::print_tree_bracketed<t_nodePtr>(global_tree);
//
// 	cout<<endl;

// 	int counter=0;
//
// 	Rate r(5);
// 	while(ok())
// 	{
// 		spinOnce();
// 		r.sleep();
//
// 		counter++;
// 		if(counter==10)
// 		{
// 			cout<<"Delete node: "<<*it1<<endl;
// 			global_tree.erase(it1);
// 		}
//
// // 		markers_publisher.publish(markersMsg);
// 	}
//
// 	tree<string> tr;
// 	tree<string>::iterator i1,i2,i3,i4;

// 	i1=tr.insert(tr.begin(),"0");
// 	i2=tr.append_child(i1,"00");
// 	tr.append_child(i1,"01");
// 	tr.append_child(i1,"02");

// 	i3=tr.append_child(i2,"000");
// 	tr.append_child(i2,"001");

// 	i4=tr.append_child(i3,"0000");

// 	i1=tr.insert(tr.begin(),"1");
// 	i2=tr.append_child(i1,"10");
// 	tr.append_child(i1,"11");

// 	tr.insert(tr.begin(),"2");

// 	cout<<endl;
// 	kptree::print_tree_bracketed<string>(tr);
// 	cout<<endl<<endl;

// 	tr.erase_branch(i4);

// 	cout<<endl;
// 	kptree::print_tree_bracketed<string>(tr);
// 	cout<<endl<<endl;


// 	MatrixXd cov(2,2);
// 	VectorXd mu(2);
//
// 	cov<<	1.1501,	1.0348,
// 			1.0348,	1.1050;
//
// 	mu<< 	0.1342,	0.1317;
//
// 	cout<<"Ellipse: "<<EllipseParametric(cov,mu,1)<<endl;

// 	constant_velocity_ekfilter f1(1./20.);
// 	constant_velocity_ekfilter f2(1./20.);
//
// 	Vector u(0);
//
// 	Vector z1(2);
// 	Vector z2(2);
//
// 	Vector x1(4);
// 	Vector x2(4);
//
// 	Rate r(20);
//
// 	Time lt=Time::now();
//
// 	double t=0;
//
// 	while(ok())
// 	{
// 		t+=Time::now().toSec()-lt.toSec();
// 		lt=Time::now();
//
// 		z1(1)=t;
// 		z1(2)=t;
//
// 		z2(1)=0;
// 		z2(2)=0;
//
//
// 		f1.step(u,z1);
// 		f2.step(u,z2);
//
// 		x1=f1.getX();
// 		x2=f2.getX();
//
// 		cout<<"z1 "<<z1<<endl;
// 		cout<<"z2 "<<z2<<endl;
//
// 		cout<<"x1 "<<x1<<endl;
// 		cout<<"x2 "<<x2<<endl<<endl;
//
// 		spinOnce();
// 		r.sleep();
// 	}

    return 0;
}