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

#ifndef ROS_BRIDGE_MHT_TYPES_H
#define ROS_BRIDGE_MHT_TYPES_H

#error This file should not be used, DEPRECATED

/**
\file
\brief MHT generic types source code, NOT USED, DEPRECATED
*/

// System Includes
#include <algorithm>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>
//#include <eigen3/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <kfilter/ekfilter.hpp>
#include <mtt/tree.hh>
#include <mtt/tree_util.hh>
#include <mtt/cluster.h>
#include <mtt/mht_types_shared.h>
#include <mtt/nonholonomic_kalman_filter.h>

using Eigen::Matrix4d;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace Kalman;
using namespace ros;
using namespace std;

/**
\brief EKfilter vector type
This is only a redefinition of a type from within a class to the outside.
*/
typedef EKFilter<double,0,false,false,false>::Vector kVector;

/**
\brief EKfilter matrix type
This is only a redefinition of a type from within a class to the outside.
*/
typedef EKFilter<double,0,false,false,false>::Matrix kMatrix;

/**
\brief Constant velocity Kalman filter
This class implements a constant velocity Kalman filter on two variables (x,y).\n
This filter is linear and should use KFilter instead of EKFilter, but this way its easier to expand.
 \f[ X = \left[ \begin{array}{c}
 x \\ \\
  y\\ \\
 \dot x \\ \\
 \dot y
 \end{array} \right ] \f]
 ...
*/
class constantVelocityEKFilter:public EKFilter<double,0,false,false,false>
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
        void SetIdentity(kMatrix& M,int size,double value=1);

        /**
        \brief Set a matrix to zero
        Set all elements of a matrix to zero.\n
        This function only works properly in square matrices.
        \param M matrix to set
        \param size size of the matrix, this should not be needed, i'll remove it in a further expansion
        */
        void SetZero(kMatrix& M,int size);

        /**
        \brief Init filter
        Inits the filter with a start vector
        \param x_init start position of the filter
        */
        void InitFilter(Vector4d& x_init);

        void InitFilter(Vector4d& x_init,Matrix4d& P_init);

        int miss_associations;
        int life_time;

        Vector4d x_predicted;
        Vector3d z_measured;
        Vector3d inovation_error;

    protected:

        ///Time interval between measurements
        double dt;
        ///Time of the last call to the makeCommonProcess function
        Time lt;

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

typedef boost::shared_ptr<constantVelocityEKFilter> constantVelocityEKFilterPtr;

namespace Mht
{

    /**
    \brief This function compares two hypotheses just to sort them
    The sorting is preformed using the _uid variable in the Hypothesis class.
    \param h1 first hypothesis
    \param h2 second hypothesis
    \return true if h1\<h2
    */

}

#endif //ROS_BRIDGE_MHT_TYPES_H
