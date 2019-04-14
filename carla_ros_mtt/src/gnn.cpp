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

// System Includes
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.8/pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <carla_ros_colormap/colormap.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <carla_ros_mtt/TargetList.h>
#include <kfilter/ekfilter.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <carla_ros_mtt/k_best.h>
#include <map>
#include <vector>
#include <cmath>

using namespace std;

using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::Matrix2d;

using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::VectorXd;

typedef Kalman::EKFilter<double,0,false,false,false>::Vector kVector;
typedef Kalman::EKFilter<double,0,false,false,false>::Matrix kMatrix;


class Point: public pcl::PointXYZ
{
    public:

        typedef boost::shared_ptr<Point> Ptr;

        double r,t,n;

        double euclideanDistance(const Point::Ptr p2)
        {
            return sqrt( pow(x-p2->x,2) + pow(y-p2->y,2));
        }

        double euclideanDistance(const Point p2)
        {
            return sqrt( pow(x-p2.x,2) + pow(y-p2.y,2));
        }

        double module()
        {
            return sqrt(x*x + y*y);
        }

        double angle()
        {
            return atan2(y,x);
        }
};

geometry_msgs::Point makePoint(double x,double y, double z)
{
    geometry_msgs::Point p;
    p.x=x;
    p.y=y;
    p.z=z;
    return p;
}

visualization_msgs::Marker makeEllipse(Point center,double A,double B,double phi, string ns, std_msgs::ColorRGBA color, int id)
{
    visualization_msgs::Marker ellipse;

    ellipse.header.frame_id = "";
    ellipse.header.stamp = ros::Time::now();

    ellipse.action = visualization_msgs::Marker::ADD;
    ellipse.type = visualization_msgs::Marker::LINE_STRIP;

    ellipse.scale.x = 0.1;
    ellipse.scale.y = 0.1;
    ellipse.scale.z = 0.1;

    ellipse.ns = ns;

    ellipse.color=color;
    ellipse.id=id;

    ellipse.points.clear();

    for(double theta = 0; theta<= 2*M_PI + 0.1;theta+=0.1)
    {
        geometry_msgs::Point p;
        p.x = center.x + A*cos(theta)*cos(phi)-B*sin(theta)*sin(phi);
        p.y = center.y + A*cos(theta)*sin(phi)+B*sin(theta)*cos(phi);
        p.z = 0;
        ellipse.points.push_back(p);
    }

    return ellipse;
}

class Markers
{
    public:
        /**
         * \brief Update a internal marker
         *
         * The updated marker will be marked for publishing, this marker may or may not already be present in the class.
         *
         * \param marker a marker to update
         */
        void update(visualization_msgs::Marker& marker)
        {
            for(uint i=0;i<markers.size();++i)
                if(markers[i].ns==marker.ns && markers[i].id==marker.id)//Marker found
                {
                    markers.erase(markers.begin()+i);
                    break;
                }

            markers.push_back(marker);
        }

        /**
         * \brief Mark existing markers for deletion
         */
        void decrement(void)
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

        /**
         * \brief Remove markers that should not be transmitted
         */
        void clean(void)
        {
            vector<visualization_msgs::Marker> new_markers;

            for(uint i=0;i<markers.size();++i)
                if(markers[i].action!=-1)
                    new_markers.push_back(markers[i]);

            markers=new_markers;
        }

        /**
         * \brief Obtain the list of outgoing markers
         *
         * \return a vector of visualization_msgs::Marker
         */
        vector<visualization_msgs::Marker> getOutgoingMarkers(void)
        {
            vector<visualization_msgs::Marker> m_out(markers);
            return markers;
        }

    private:
        ///Internal storing vector of markers
        vector<visualization_msgs::Marker> markers;
};

///Ellipsoid target gait area
class SearchArea
{
    public:
        ///Ellipse major axis
        double ellipse_A;
        ///Ellipse minor axis
        double ellipse_B;
        ///Ellipse orientation
        double angle;
        ///Ellipse center position
        double center_x,center_y;
};


class MotionModel: public Kalman::EKFilter<double,0,false,false,false>
{
    public:

        double dt;
        Vector2d inovation_error;
        Vector4d x_predicted;
        Vector4d x_estimated;
        Matrix4d PosteriorCovariance;
        MotionModel()
        {
        }

        void setIdentity(kMatrix& M,int size,double value=1)
        {
            for(int l=0;l<size;l++)
                for(int c=0;c<size;c++)
                    if(c==l)
                        M(l,c)=value;
                    else
                        M(l,c)=0;
        }

        void initFilter(Point measurement)
        {
            setDim(4,0,4,2,2);
            dt=1./50.;

            Vector x_0(4);

            x_0(0)=measurement.x;//x
            x_0(1)=measurement.y;//y
            x_0(2)=0;//velocity x
            x_0(3)=0;//velocity y

            Matrix P_0(4,4);

            //Large initial uncertainty
            setIdentity(P_0,4,4.0);

            init(x_0,P_0);
        }

        void stepEstimator(Vector2d measurement)
        {
            inovation_error=measurement - x_predicted.head(2);

            //Convert to ekfilter format
            kVector z_(2);
            z_(0)=measurement[0];
            z_(1)=measurement[1];

            kVector u(0);
            //Makes one prediction-correction step
            step(u,z_);

            //Returns the corrected state (a posteriori state estimate)
            kVector x_(4);
            x_ = getX();

            //Convert from ekfilter to eigen format
            x_estimated(0) = x_(0);
            x_estimated(1) = x_(1);
            x_estimated(2) = x_(2);
            x_estimated(3) = x_(3);

            //Returns the a posteriori error covariance estimate matrix
            kMatrix P = calculateP();

            //Convert from ekfilter to eigen format
            PosteriorCovariance(0,0)=P(0,0);
            PosteriorCovariance(0,1)=P(0,1);
            PosteriorCovariance(0,2)=P(0,2);
            PosteriorCovariance(0,3)=P(0,3);

            PosteriorCovariance(1,0)=P(1,0);
            PosteriorCovariance(1,1)=P(1,1);
            PosteriorCovariance(1,2)=P(1,2);
            PosteriorCovariance(1,3)=P(1,3);

            PosteriorCovariance(2,0)=P(2,0);
            PosteriorCovariance(2,1)=P(2,1);
            PosteriorCovariance(2,2)=P(2,2);
            PosteriorCovariance(2,3)=P(2,3);

            PosteriorCovariance(3,0)=P(3,0);
            PosteriorCovariance(3,1)=P(3,1);
            PosteriorCovariance(3,2)=P(3,2);
            PosteriorCovariance(3,3)=P(3,3);

            //Returns the predicted state vector (a priori state estimate)
            kVector xp = predict(u);

            //Convert from ekfilter to eigen format
            x_predicted(0)=xp(0);
            x_predicted(1)=xp(1);
            x_predicted(2)=xp(2);
            x_predicted(3)=xp(3);
        }

        /**
        \brief Make the process Jacobian matrix
        */
        void makeA()
        {
            //x y vx vy
            A(0,0)=1.0;
            A(0,1)=0.0;
            A(0,2)=dt;
            A(0,3)=0;

            A(1,0)=0.0;
            A(1,1)=1.0;
            A(1,2)=0;
            A(1,3)=dt;

            A(2,0)=0.0;
            A(2,1)=0.0;
            A(2,2)=1.0;
            A(2,3)=0.0;

            A(3,0)=0.0;
            A(3,1)=0.0;
            A(3,2)=0.0;
            A(3,3)=1.0;
        }

        void makeH()
        {
            H(0,0) = 1;
            H(0,1) = 0;

            H(1,0) = 0;
            H(1,1) = 1;
        }

        void makeBaseW()
        {
            setIdentity(W,4,1.0);
        }

        /**
        \brief Make measurement noise sensitivity matrix
        */
        void makeBaseV()
        {
            setIdentity(V,2,1.0);
        }

        void makeR()
        {
            R(0,0) = 1.0;
            R(0,1) = 0.0;

            R(1,0) = 0.0;
            R(1,1) = 1.0;
        }

        /**
        \brief Make process noise covariance matrix
        */
        void makeQ()
        {
            double q=2.0;

            //X
            Q(0,0) = q*0.1;
            Q(0,1) = 0.0;
            Q(0,2) = 0.0;
            Q(0,3) = 0.0;

            //Y
            Q(1,0) = 0.0;
            Q(1,1) = q*0.1;
            Q(1,2) = 0.0;
            Q(1,3) = 0.0;

            //Vx
            Q(2,0) = 0.0;
            Q(2,1) = 0.0;
            Q(2,2) = q*5.;
            Q(2,3) = 0.0;

            //Vy
            Q(3,0) = 0.0;
            Q(3,1) = 0.0;
            Q(3,2) = 0.0;
            Q(3,3) = q*5.;

        }

        void makeProcess()
        {
            // x = x0 + vx*dt
            // y = y0 + vy*dt
            // vx = vx0
            // vy = vy0

            //  x(0) -> x
            //  x(1) -> y
            //  x(2) -> vx
            //  x(3) -> vy

            Vector x_(x.size());

            x_(0) = x(0) + x(2)*dt;
            x_(1) = x(1) + x(3)*dt;
            x_(2) = x(2);
            x_(3) = x(3);

            x.swap(x_);
        }

        /**
        \brief Make measurement, used when measurement is not possible (i'm not using it now)
        */
        void makeMeasure()
        {
            z(0)=x(0);//x
            z(1)=x(1);//y
        }

};

class Target
{
    public:
        typedef boost::shared_ptr<Target> Ptr;

        void calculateProperties()
        {
            double xsum = 0;
            double ysum = 0;

            for(Point::Ptr p: points)
            {
                xsum+=p->x;
                ysum+=p->y;

                if(p->r<rmin)
                    rmin = p->r;
            }

            tm = (points[0]->t+points[points.size()-1]->t)/2.;

            centroid.x = xsum/points.size();
            centroid.y = ysum/points.size();
        }

        void associate(Target::Ptr target)
        {
            found = true;
            measurement = target->centroid;
            occluded=false;

            life_time++;

            occluded_time-=1;

            if(occluded_time<0)
                occluded_time=0;
        }

        double checkAssociationCriteria(Target::Ptr target)
        {
            //Distance to target from predicted position

            return predicted_position.euclideanDistance(target->centroid);

            //             double ox=target->centroid.x;
            //             double oy=target->centroid.y;
            //
            //             double angle=-search_area.angle;
            //             double s=search_area.ellipse_B;
            //             double r=search_area.ellipse_A;
            //             double M=cos(angle);
            //             double N=sin(angle);
            //             double c=search_area.center_x;
            //             double d=search_area.center_y;
            //             double tx=ox-c;
            //             double ty=oy-d;
            //             double A=(M*tx-N*ty)*(M*tx-N*ty);
            //             double B=(N*tx+M*ty)*(N*tx+M*ty);
            //             double Z=s*s*A+r*r*B-r*r*s*s;
            //
            //             if(Z<0)
            //                 return Z;
            //             else
            //                 return 1;
        }

        void stepModel()
        {
            Vector2d z;

            z(0) = measurement.x;
            z(1) = measurement.y;

            motion_model.stepEstimator(z);

            predicted_position.x = motion_model.x_predicted(0);
            predicted_position.y = motion_model.x_predicted(1);

            estimated_position.x = motion_model.x_estimated(0);
            estimated_position.y = motion_model.x_estimated(1);

            estimated_velocity.x = motion_model.x_estimated(2);
            estimated_velocity.y = motion_model.x_estimated(3);
        }


        Point centroid;
        Point estimated_position;
        Point predicted_position;
        Point measurement;
        Point estimated_velocity;

        MotionModel motion_model;

        bool occluded;
        double rmin,tm;
        long id;
        long local_id;

        long life_time;
        long occluded_time;
        bool found;
        bool to_remove;
        double length;

        SearchArea search_area;

        vector<Point::Ptr> points;
};


class GNN
{
    public:
        GNN(ros::NodeHandle nh_)
                :nh(nh_)
        {
            string parameters_url;
            nh.param("parameters",parameters_url,std::string("not avaliable"));

            string package_tag = "package://";
            int pos = parameters_url.find(package_tag);
            if(pos != string::npos)
            {
                //String contains package, replace by package path
                int fpos = parameters_url.find("/",pos+package_tag.length());
                string package = parameters_url.substr(pos+package_tag.length(),fpos-(pos+package_tag.length()));

                string package_path = ros::package::getPath(package);
    //                string package_path = ros::package::getSharedPath(package);
    //                string package_path = "/home/pedro/catkin_ws_atlas/src/mtt";
                parameters_url.replace(pos,fpos-pos,package_path);
            }

            parameters.open(parameters_url.c_str(),cv::FileStorage::READ);

            points_subscriber = nh.subscribe("/points", 1,&GNN::pointsHandler,this);
            targets_publisher = nh.advertise<mtt::TargetList>("/targets", 1000);
            markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("/targets_markers", 1000);
            measurements_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("/measurements_markers", 1000);

            ezA = parameters["exclusion_zone_A"];
            ezB = parameters["exclusion_zone_B"];
            max_missing_iterations = parameters["max_missing_iterations"];
            max_ellipse_axis = parameters["max_ellipse_axis"];
            min_ellipse_axis = parameters["min_ellipse_axis"];
            size_factor = parameters["size_factor"];
            not_found_factor = parameters["not_found_factor"];
            clustering_distance = parameters["clustering_distance"];
        }

        ~GNN()
        {}

        void pointCloudToVector(const sensor_msgs::PointCloud2& cloud,vector<Point::Ptr>& data)
        {
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            pcl::PCLPointCloud2 pcl_pc;
            pcl_conversions::toPCL(cloud, pcl_pc);
            pcl::fromPCLPointCloud2(pcl_pc, pcl_cloud);

            //get points into grid to reorder them
            for(uint i=0;i<pcl_cloud.points.size();i++)
            {
                Point::Ptr pt(new Point);

                pt->x=pcl_cloud.points[i].x;
                pt->y=pcl_cloud.points[i].y;
                pt->r=sqrt(pow(pt->x,2)+pow(pt->y,2));
                pt->t=atan2(pt->y,pt->x);
                pt->n=i++;

                data.push_back(pt);
            }
        }


        void clustering(const vector<Point::Ptr> data,vector<Target::Ptr>& current_targets,double clustering_distance)
        {
            long new_targets_ids=0;

            //Create empty point associations
            vector<pair<Target::Ptr,bool> > point_association;
            point_association.resize(data.size(),make_pair(Target::Ptr(),false));

            //Go thought all points in the data vector
            for(uint p=0;p<data.size();p++)
            {
                //If point was not associated with any measurement
                if(point_association[p].second==false)
                {
                    //Create a new measurement
                    Target::Ptr new_target(new Target);

                    //Set id and increment static variable
                    new_target->local_id=new_targets_ids++;

                    //Add this point to the measurement
                    new_target->points.push_back(data[p]);

                    //Make this point associate with the new measurement
                    point_association[p].first=new_target;
                    point_association[p].second=true;

                    current_targets.push_back(new_target);
                }

                //Go though all the other points and see which ones associate with this measurement
                recursiveAssociation(data,point_association,clustering_distance,p);
            }

            for(uint m=0;m<current_targets.size();m++)
                current_targets[m]->calculateProperties();
        }

        void recursiveAssociation(const vector<Point::Ptr> data,vector<pair<Target::Ptr,bool> >& point_association,double clustering_distance, uint p)
        {
            Target::Ptr null_ptr;

            for(uint l=0;l<data.size();l++)
            {
                //If the point was not yet associated with anybody
                if(point_association[l].second==false)
                {
                    double dist = data[p]->euclideanDistance(data[l]);

                    //This point associates with the point p
                    if(dist<clustering_distance)
                    {
                        //Put the association pointing to the existing measurement
                        point_association[l].first = point_association[p].first;
                        point_association[l].second=true;

                        //Go to the existing measurement and add this new point
                        point_association[p].first->points.push_back(data[l]);

                        recursiveAssociation(data,point_association,clustering_distance,l);
                    }
                }
            }
        }

        void addNewTargetToList(Target::Ptr new_target)
        {
            static long targets_ids = 0;

            new_target->motion_model.initFilter(new_target->centroid);
            new_target->predicted_position = new_target->centroid;
            new_target->measurement = new_target->centroid;

            new_target->search_area.ellipse_A = max_ellipse_axis;
            new_target->search_area.ellipse_B = max_ellipse_axis;
            new_target->search_area.angle = 0;
            new_target->search_area.center_x = new_target->predicted_position.x;
            new_target->search_area.center_y=new_target->predicted_position.y;

            new_target->occluded_time=0;
            new_target->life_time=1;

            new_target->occluded=false;

            new_target->id = targets_ids++;

            targets.push_back(new_target);

            return;
        }

        void stepModels(const vector<Target::Ptr> targets)
        {
            for(Target::Ptr target: targets)
                target->stepModel();
        }

        void updateSearchAreas(const vector<Target::Ptr> targets)
        {
            for(Target::Ptr target: targets)
            {
                target->search_area.angle = target->estimated_velocity.angle();
                target->search_area.center_x = target->predicted_position.x;
                target->search_area.center_y = target->predicted_position.y;

                double size = target->length;

                double default_size = min_ellipse_axis;

                double xIl = target->motion_model.inovation_error(0);
                double yIl = target->motion_model.inovation_error(1);

                int not_found_counter=target->occluded_time;

                double innovation_error=sqrt(sqrt(xIl*xIl+yIl*yIl));

                double A_innovation_factor,B_innovation_factor;

                if( target->estimated_velocity.module() < 0.200 )
                {
                    A_innovation_factor=5;
                    B_innovation_factor=4;

                }else
                {
                    A_innovation_factor=5;
                    B_innovation_factor=0.5;
                }

                if(innovation_error<0.001)
                    innovation_error=0.001;

                target->search_area.ellipse_A = not_found_factor*pow(not_found_counter,2) + A_innovation_factor*innovation_error + default_size + size_factor*size;
                target->search_area.ellipse_B = not_found_factor*pow(not_found_counter,2) + B_innovation_factor*innovation_error + default_size + size_factor*size;

                if(target->search_area.ellipse_A > max_ellipse_axis)
                    target->search_area.ellipse_A = max_ellipse_axis;

                if(target->search_area.ellipse_B > max_ellipse_axis)
                    target->search_area.ellipse_B = max_ellipse_axis;

                if(target->life_time < 5)///Small bonus to new objects @todo I should use a equation that allowed a more soft transition
                {
                    target->search_area.ellipse_A+=1.0;
                    target->search_area.ellipse_B+=1.0;
                }
            }
        }

        void associateTargets(const vector<Target::Ptr> current_targets,vector<Target::Ptr>& targets)
        {
            for(Target::Ptr target: targets)
                target->found = false;

            double min_ret=1;
            Target::Ptr associated_new_target;
            int min_index=-1;
            double ret=1;
            bool association_found;
            double remove_threshold;

            for(Target::Ptr target: targets)
            {
                min_ret=1;
                association_found=false;

                for(Target::Ptr current_target: current_targets)///Run thorough all the new objects
                {
                    ret = target->checkAssociationCriteria(current_target);

                    if(ret<min_ret && ret<0) ///Inside ellipse
                    {
                        associated_new_target = current_target;
                        min_ret=ret;
                        association_found=true;
                    }
                }

                if(association_found)
                {
                    double dist;
                    double min_dist=1e6;

                    Target::Ptr closest_existing_target;

                    for(Target::Ptr existing_target: targets)
                    {
                        if(existing_target->id == target->id)
                            continue;

                        dist = associated_new_target->centroid.euclideanDistance(existing_target->predicted_position);

                        if(dist<min_dist)
                        {
                            closest_existing_target = existing_target;
                            min_dist=dist;
                        }
                    }

                    //Exclusion zone B
                    if(min_dist < ezB && closest_existing_target->life_time > target->life_time)
                        association_found=false;
                }

                if(association_found && associated_new_target->found==false)///New target found in list
                {
                    target->associate(associated_new_target);
                    associated_new_target->found=true;
                    associated_new_target->id = target->id;

                }else///Object not found
                {
                    target->occluded=true;
                    target->occluded_time++;

                    target->measurement = target->predicted_position;//void update

                    int remove_threshold;

                    if(target->life_time > max_missing_iterations)
                        remove_threshold = max_missing_iterations;
                    else
                        remove_threshold = target->life_time;

                    if(target->occluded_time > max_missing_iterations)
                        target->to_remove = true;
                }
            }

            //Remove all targets marked for deletion
            targets.erase(remove_if(targets.begin(),targets.end(),[](Target::Ptr t){return t->to_remove;}),targets.end());

            ///Add not found objects to list
            double dist_to_object=1e6;
            for(Target::Ptr current_target: current_targets)
            {
                ///Calculate min_distance_to_existing_object
                double min_distance_to_existing_object = 1e6;

                for(Target::Ptr target: targets)
                {
                    dist_to_object = current_target->estimated_position.euclideanDistance(target->measurement);

                    if(dist_to_object<min_distance_to_existing_object)
                        min_distance_to_existing_object=dist_to_object;
                }

                //Exclusion zone A
                if(min_distance_to_existing_object < ezA && first_iteration == false)
                    continue;

                if(current_target->found==false)
                    addNewTargetToList(current_target);
            }

            return;
        }

        void createOutputTargets(const vector<Target::Ptr> targets,mtt::TargetList& target_list)
        {
            for(Target::Ptr target: targets)
            {
                //structure to be fed to array
                mtt::Target out_target;

                //build header
                out_target.header.stamp = ros::Time::now();
                out_target.header.frame_id = frame_id;

                out_target.id = target->id;

                out_target.velocity.linear.x = target->estimated_velocity.x;
                out_target.velocity.linear.y = target->estimated_velocity.y;
                out_target.velocity.linear.z = 0;

                out_target.pose.position.x = target->estimated_position.x;
                out_target.pose.position.y = target->estimated_position.y;
                out_target.pose.position.z = 0;

                out_target.pose.orientation = tf::createQuaternionMsgFromYaw(target->estimated_velocity.angle());

                out_target.initialpose.x = target->points[0]->x;
                out_target.initialpose.y = target->points[0]->y;
                out_target.initialpose.z = 0;

                out_target.finalpose.x = target->points[target->points.size()-1]->x;
                out_target.finalpose.y = target->points[target->points.size()-1]->y;
                out_target.finalpose.z = 0;

                out_target.size = target->length;

                target_list.Targets.push_back(out_target);
            }
        }

        vector<visualization_msgs::Marker> createMeasurementsMarkers(const vector<Target::Ptr> targets)
        {
            static Markers marker_list;
            string tracking_frame = frame_id;

            //Reduce the elements status, ADD to REMOVE and REMOVE to delete
            marker_list.decrement();

            //Create a color map
            class_colormap colormap("hsv",10, 1, false);

            visualization_msgs::Marker marker_centers;

            marker_centers.header.frame_id = tracking_frame;
            marker_centers.header.stamp = ros::Time::now();

            marker_centers.ns = "measurements_centers";
            marker_centers.action = visualization_msgs::Marker::ADD;

            marker_centers.type = visualization_msgs::Marker::POINTS;

            marker_centers.scale.x = 0.4;
            marker_centers.scale.y = 0.4;
            marker_centers.scale.z = 0.4;

            marker_centers.color.r = 0;
            marker_centers.color.g = 0;
            marker_centers.color.b = 0;
            marker_centers.color.a = 1;

            marker_centers.points.resize(targets.size());
            marker_centers.colors.resize(targets.size());

            marker_centers.id = 0;

            for(int i=0;i<targets.size();i++)
            {
                Target::Ptr target = targets[i];

                marker_centers.colors[i] = colormap.color(5);

                marker_centers.points[i].x = target->centroid.x;
                marker_centers.points[i].y = target->centroid.y;
                marker_centers.points[i].z = 0.0;
            }

            marker_list.update(marker_centers);

            //Remove markers that should not be transmitted
            marker_list.clean();

            //Clean the marker_vector and put the new markers in it
            return marker_list.getOutgoingMarkers();
        }


        vector<visualization_msgs::Marker> createTargetMarkers(const vector<Target::Ptr> targets)
        {
            static Markers marker_list;
            string tracking_frame = frame_id;

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

            marker_ids.scale.x = 0.5;
            marker_ids.scale.y = 0.5;
            marker_ids.scale.z = 0.5;

            marker_centers.scale.x = 0.4;
            marker_centers.scale.y = 0.4;
            marker_centers.scale.z = 0.4;

            marker_velocity.scale.x = 0.2;
            marker_velocity.scale.y = 0.3;
            marker_velocity.scale.z = 0.4;

            marker_association.scale.x = 0.2;

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

            for(int i=0;i<targets.size();i++)
            {
                Target::Ptr target = targets[i];

                double velocity = target->estimated_velocity.module();

                marker_centers.colors[i] = colormap.color(target->id);

                marker_centers.points[i].x = target->estimated_position.x;
                marker_centers.points[i].y = target->estimated_position.y;
                marker_centers.points[i].z = 0.0;

                marker_ids.pose.position.x = target->estimated_position.x;
                marker_ids.pose.position.y = target->estimated_position.y;
                marker_ids.pose.position.z = 1.0;

                boost::format fm("%ld");
                fm % target->id;

                marker_ids.text = fm.str();

                marker_ids.id = target->id;

                marker_list.update(marker_ids);

                if(velocity>0.1)
                {
                    marker_velocity.points.clear();
                    marker_velocity.points.resize(2);

                    marker_velocity.color = colormap.color(target->id);

                    marker_velocity.points[0].x = target->estimated_position.x;
                    marker_velocity.points[0].y = target->estimated_position.y;
                    marker_velocity.points[0].z = 0.0;

                    marker_velocity.points[1].x = target->estimated_position.x + target->estimated_velocity.module()*cos(target->estimated_velocity.angle())*velocity_scale;
                    marker_velocity.points[1].y = target->estimated_position.y + target->estimated_velocity.module()*sin(target->estimated_velocity.angle())*velocity_scale;

                    marker_velocity.points[1].z = 0.0;

                    marker_velocity.id=target->id;

                    marker_list.update(marker_velocity);
                }

                //Gatting ellipse, not working now
                //                 visualization_msgs::Marker ellipse = makeEllipse(target->predicted_position,target->search_area.ellipse_A,target->search_area.ellipse_B,target->search_area.angle,"search areas",colormap.color(target->id),target->id);
                visualization_msgs::Marker ellipse = makeEllipse(target->predicted_position,0.5,0.4,target->search_area.angle,"simple_marker",colormap.color(target->id),target->id);

                ellipse.header.frame_id = tracking_frame;
                ellipse.header.stamp = ros::Time::now();
                marker_list.update(ellipse);

                //Association lines
                geometry_msgs::Point start;
                geometry_msgs::Point end;

                start.x = target->estimated_position.x;
                start.y = target->estimated_position.y;
                start.z = 0;

                end.x = target->measurement.x;
                end.y = target->measurement.y;
                end.z = 0;

                marker_association.points.push_back(start);
                marker_association.points.push_back(end);

                marker_association.colors.push_back(colormap.color(target->id));
                marker_association.colors.push_back(colormap.color(target->id));
            }

            marker_list.update(marker_centers);
            marker_list.update(marker_association);

            //Remove markers that should not be transmitted
            marker_list.clean();

            //Clean the marker_vector and put the new markers in it
            return marker_list.getOutgoingMarkers();
        }

        void hungarianMatching(vector<Target::Ptr>& targets,vector<Target::Ptr>& measurements)
        {
            for(Target::Ptr target: targets)
            {
                target->found = false;
                target->to_remove = false;
            }

            MatrixXd cost_matrix(targets.size(),measurements.size());

            for(uint t=0; t<targets.size(); t++)
            {
                for(uint m=0; m<measurements.size(); m++)
                {
                    double cost = targets[t]->checkAssociationCriteria(measurements[m]);
                    if(cost<1)
                        cost_matrix(t,m) = cost;
                    else
                        cost_matrix(t,m) = 1e12;
                }
            }

            //Declare the assignments
            vector<orderedPairPtr> assignments;

            //Calculate the first best assignment
            double cost = munkers_wrapper(cost_matrix,assignments);

            //Do all associations
            for(uint i=0;i<assignments.size();i++)
            {
                int t = assignments[i]->row;
                int m = assignments[i]->col;

                if(cost_matrix(t,m) >= 1e12)
                    continue;

                Target::Ptr target = targets[t];
                Target::Ptr measurement = measurements[m];

                Target::Ptr closest_existing_target;
                double min_dist = 1e6; //to closest existing target

                for(Target::Ptr current_target: targets)
                {
                    if(current_target->id == target->id)
                        continue;

                    double dist = target->centroid.euclideanDistance(current_target->predicted_position);

                    if(dist<min_dist)
                    {
                        closest_existing_target = current_target;
                        min_dist=dist;
                    }
                }

                //Exclusion zone B
                if(min_dist < ezB && closest_existing_target->life_time > target->life_time)
                {
                    //do not associate
                }else
                {
                    measurement->found = true;
                    target->found = true;
                    target->associate(measurement);
                }
            }

            //Do all not associated targets
            for(Target::Ptr target: targets)
            {
                if(target->found == false)
                {
                    target->occluded=true;
                    target->occluded_time++;

                    target->measurement = target->predicted_position;//void update

                    int remove_threshold;

                    if(target->life_time > max_missing_iterations)
                        remove_threshold = max_missing_iterations;
                    else
                        remove_threshold = target->life_time;

                    if(target->occluded_time > max_missing_iterations)
                        target->to_remove = true;
                }
            }

            //Remove all targets marked for deletion
            targets.erase(remove_if(targets.begin(),targets.end(),[](Target::Ptr t){return t->to_remove;}),targets.end());

            ///Add not found objects to list
            double dist_to_object=1e6;
            for(Target::Ptr measurement: measurements)
            {
                ///Calculate min_distance_to_existing_object
                double min_distance_to_existing_object = 1e6;

                for(Target::Ptr target: targets)
                {
                    dist_to_object = measurement->estimated_position.euclideanDistance(target->measurement);

                    if(dist_to_object<min_distance_to_existing_object)
                        min_distance_to_existing_object=dist_to_object;
                }

                //Exclusion zone A
                if(min_distance_to_existing_object < ezA && first_iteration == false)
                    continue;

                if(measurement->found==false)
                    addNewTargetToList(measurement);
            }
        }


        void pointsHandler(const sensor_msgs::PointCloud2& msg)
        {
            //             cout<<"received points"<<endl;

            //Get frame of current point cloud
            frame_id = msg.header.frame_id;

            //             cout<<"to local format"<<endl;
            //Get local data format
            vector<Point::Ptr> data;
            pointCloudToVector(msg,data);
            //             cout<<"data points: "<<data.size()<<endl;

            //             cout<<"clustering"<<endl;
            //Cluster data to produce the current list of targets
            vector<Target::Ptr> current_targets;
            clustering(data,current_targets,clustering_distance);

            //             cout<<"current targets size: "<<current_targets.size()<<endl;

            //             cout<<"association"<<endl;

            //Associate current targets with previous existing targets
            hungarianMatching(targets,current_targets);
            //             associateTargets(current_targets,targets);
            //             cout<<"global targets: "<<targets.size()<<endl;

            //             cout<<"step models"<<endl;

            //Update Motion models
            stepModels(targets);

            //             cout<<"update search areas"<<endl;

            //Update search areas
            updateSearchAreas(targets);

            //             cout<<"publish targets"<<endl;
            //Get output targets from current iteration
            mtt::TargetList target_list;
            createOutputTargets(targets,target_list);

            //Publish target list
            targets_publisher.publish(target_list);

            //Create markers from targets list
            visualization_msgs::MarkerArray markers;
            markers.markers = createTargetMarkers(targets);
            //Publish marker list
            markers_publisher.publish(markers);

            //Create markers from current targets
            visualization_msgs::MarkerArray measurement_markers;
            measurement_markers.markers = createMeasurementsMarkers(current_targets);
            //Publish marker list
            measurements_markers_publisher.publish(measurement_markers);
        }


    private:

        double ezA;
        double ezB;
        double max_ellipse_axis;
        double min_ellipse_axis;
        double size_factor;
        double not_found_factor;
        int max_missing_iterations;
        bool first_iteration;
        double clustering_distance;
        string frame_id;

        ros::NodeHandle nh;
        ros::Subscriber points_subscriber;

        ros::Publisher targets_publisher;
        ros::Publisher markers_publisher;
        ros::Publisher measurements_markers_publisher;

        cv::FileStorage parameters;

        //List of current targets
        vector<Target::Ptr> targets;
};

int main(int argc, char**argv)
{
    cout<<"GNN tracking active"<<endl;

    // Initialize ROS
    ros::init(argc,argv,"gnn");

    ros::NodeHandle nh("~");

    GNN gnn(nh);

    ros::spin();

    return 0;
}
