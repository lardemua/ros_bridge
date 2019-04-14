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

#ifndef ROS_BRIDGE_TYPES_DECLARATION_H
#define ROS_BRIDGE_TYPES_DECLARATION_H

/**
\file
\brief Header for type declaration, only constant velocity Kalman filter
*/

#error This file should not be used, DEPRECATED

// System Includes
#include <ros/ros.h>
#include <kfilter/ekfilter.hpp>
//#include <eigen3/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace ros;
using namespace Kalman;

using Eigen::Matrix4d;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
\brief Constant velocity Kalman filter
This class implements a constant velocity Kalman filter on two variables (x,y).\n
This filter is linear and should use KFilter instead of EKFilter, but this way its easier to expand.
 \f[ X = \left [ \begin{array}{c}
 x \\ \\
 \dot x \\ \\
 y\\ \\
 \dot y
 \end{array} \right ] \f]
 ...
*/
class constant_velocity_ekfilter:public EKFilter<double,0,false,false,false>
{
    public:

        /**
        \brief Set a matrix to identity
        Set diagonal elements to 1 or the input parameter value and all others to 0.\n
        This function only works properly in square matrices.
        \param M matrix to set
        \param size size of the matrix, this should not be needed, i'll remove it in a further expansion
        \param value optional value for the diagonal elements, default is 1
        */
        void SetIdentity(Matrix& M,int size,double value=1);

        /**
        \brief Set a matrix to zero
        Set all elements of a matrix to zero.\n
        This function only works properly in square matrices.
        \param M matrix to set
        \param size size of the matrix, this should not be needed, i'll remove it in a further expansion
        */
        void SetZero(Matrix& M,int size);

        /**
        \brief Init filter
        Inits the filter with a start vector
        \param x_init start position of the filter
        */
        void InitFilter(Vector4d& x_init);

    protected:

        ///Time interval between measurements
        double dt;
        ///Time of the last call to the makeCommonProcess function
        Time lt;

        Vector4d x_predicted;
        Vector2d z_measured;
        Vector2d inovation_error;

        /**
        \brief This function is called before all other make something functions
        It's currently being used to update the time interval between iterations (dt).
        */
        void makeCommonProcess();

        /**
        \brief Make the base constant process Jacobian matrix
        */
        void makeBaseA();

        /**
        \brief Make the process Jacobian matrix
        */
        void makeA();

        /**
        \brief Make measurement sensitivity matrix
        */
        void makeBaseH();

        /**
        \brief Make process noise sensitivity matrix
        */
        void makeBaseW();

        /**
        \brief Make measurement noise sensitivity matrix
        */
        void makeBaseV();

        /**
        \brief Make measurement noise covariance matrix
        */
        void makeR();

        /**
        \brief Make process noise covariance matrix
        */
        void makeQ();

        /**
        \brief Make process, model iteration
        */
        void makeProcess();

        /**
        \brief Make measurement, used when measurement is not possible (i'm not using it now)
        */
        void makeMeasure();
};

/**
\brief EKfilter vector type
This is only a redefinition of a type from within a class to the outside.
*/
typedef EKFilter<double,0,false,false,false>::Vector Vector;

/**
\brief EKfilter matrix type
This is only a redefinition of a type from within a class to the outside.
*/
typedef EKFilter<double,0,false,false,false>::Matrix Matrix;

///Shared pointer to the Kalman constant velocity filter
typedef boost::shared_ptr<constant_velocity_ekfilter> constant_velocity_ekfilterPtr;

class t_cluster;/*this declaration has been pulled up because class t_point needs it*/
///Shared pointer to the t_cluster class
typedef boost::shared_ptr<t_cluster> t_clusterPtr;

/**
\brief Simple point class
*/
class t_point
{
    public:

        t_point();

        /**
        \brief Set all the point fields to zero
        */
        void SetZero();

        /**
        \brief Overload += operator to allow class addition
        */
        t_point& operator+=(const t_point &rhs);

        /**
        \brief Scale all point values by a factor
        \param val scale factor
        */
        void Scale(double val);

        ///x coordinate (Cartesian)
        double x;
        ///y coordinate (Cartesian)
        double y;
        ///z coordinate (Cartesian)
        double z;
        ///r coordinate (Polar)
        double r;
        ///t coordinate (Polar)
        double t;
        ///auxiliary identifier
        double n;
        ///auxiliary identifier 2
        double ni;

        t_clusterPtr cl;
    private:
};

///Shared pointer to the t_point class
typedef boost::shared_ptr<t_point> t_pointPtr;

/**
\brief Cluster type class, clusters are groups of points in close proximity
*/
class t_cluster
{
    public:
        ///Vector of the support points of the cluster
        vector<t_pointPtr> points;
        ///Centroid of the cluster
        t_point centroid;
        ///Id of the cluster
        double id;
        ///Association vector, multiple associations maybe possible
        vector<int> associations;

        t_cluster();

        t_cluster & operator=(const t_cluster &rhs);

        /**
        \brief Break point detection used as clustering criterion
        \param pt first point
        \param pt_m1 next point
        \return true if the next point belongs to a new cluster
        */
        bool BreakPointDetector(t_pointPtr&pt,t_pointPtr&pt_m1);

        /**
        \brief Calculate the centroid of the cluster
        */
        void CalculateCentroid(void);

    private:
};

/**
\brief Class handler for tree nodes
This class will hold all the information on the tree nodes, each node will correspond to a possible association of the parent node.\n
The node is compromised of a morphology representation, a process estimator, ... (in continuous expansion)
*/
class t_node
{
    //nodes will use IMM to predict and estimate positions, maybe even using kalman smother

    public:
        ///Support points for this node
        t_cluster local_cluster;
        ///Local estimator for process, Kalman filter with a constant velocity
        constant_velocity_ekfilter estimator;
        ///Estimated state
        Vector4d x;
        ///Predicted x
        Vector4d x_p;
        ///Measurement vector
        Vector2d z;
        ///Posteriori error covariance
        Matrix4d P;
        ///Number of iterations with failed associations
        int failed_associations_counter;
        ///Number of iterations with successful associations
        int successful_association_counter;
        ///Number of iterations that this node suffered
        long age;
        ///associations
        enum e_mode {NEW,MAIN,SIBLING,FAILED,TEST} mode;
        ///Id of the node
        long id;
        ///Iteration from the data used in its creation
        long data_iteration;

        ///Marker for rviz
        visualization_msgs::Marker marker;

        /**
        \brief Increase the age on the node
        This functions only increases the age variable of this node.
        \param increment age increment, default is 1
        */
        void IncreaseAge(int increment=1);

        /**
        \brief Step the filter of this node using measurements in the vector z
        */
        void Step();

        /**
        \brief Constructor for the node based on a cluster
        This constructor is typically called when there's no prior to this node.\n
        \param cluster Supporting cluster points
        \param iteration cluster data iteration
        */
        t_node(t_cluster&cluster,long iteration);

        /**
        \brief Constructor for the node based on a previous node and the data from a new cluster
        This constructor is called when the new node is the son of a previous one
        \param node inherited node
        \param cluster Supporting cluster points
        \param iteration cluster data iteration
        */
        t_node(t_node&node,t_cluster&cluster,long iteration);

        /**
        \brief Constructor for the node based on a previous node and no new data
        This constructor is called when no association was made and the node is iterated empty
        \param node inherited node
        \param iteration cluster data iteration
        */
        t_node(t_node&node,long iteration);

        /**
        \brief Function common to all constructors
        */
        void CommonConstructor();

        /**
        \brief Destructor of the node class
        */
        ~t_node(void);

        /**
        \brief Get the name of this node, used everywhere!!!
        */
        string GetName();

    private:
};

///Shared pointer to the t_node class
typedef boost::shared_ptr<t_node> t_nodePtr;

/**
\brief Overload of the operator \<\< for the t_nodePtr typedef
This overload allows us to print the node value even when doing cout<< of the shared pointer.
\param o output stream
\param i node shared pointer reference
*/
ostream& operator<< (ostream &o, const t_nodePtr &i);

#endif //ROS_BRIDGE_TYPES_DECLARATION_H
