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

#ifndef ROS_BRIDGE_NONHOLONOMICEKFILTER_H
#define ROS_BRIDGE_NONHOLONOMICEKFILTER_H

/**
\file
\brief Nonholonomic extended Kalman filter class
*/

// System Includes
#include <algorithm>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>
//#include <eigen3/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <kfilter/ekfilter.hpp>

using Eigen::Matrix4d;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace Kalman;
using namespace ros;
using namespace std;

typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 5, 5> Matrix5d;


typedef EKFilter<double,0,false,false,false>::Vector kVector;
typedef EKFilter<double,0,false,false,false>::Matrix kMatrix;

class nonholonomicEKFilter;
typedef boost::shared_ptr<nonholonomicEKFilter> nonholonomicEKFilterPtr;

/**
\brief nonholonomic Kalman filter
*/
class nonholonomicEKFilter:public EKFilter<double,0,false,false,false>
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
        void SetIdentity(kMatrix& M,int size,double value=1)
        {
            for(int l=0;l<size;l++)
                for(int c=0;c<size;c++)
                    if(c==l)
                        M(l,c)=value;
                    else
                        M(l,c)=0;
        }


        /**
        \brief Set a matrix to zero
        Set all elements of a matrix to zero.\n
        This function only works properly in square matrices.
        \param M matrix to set
        \param size size of the matrix, this should not be needed, i'll remove it in a further expansion
        */
        void SetZero(kMatrix& M,int size)
        {
            for(int l=0;l<size;l++)
                for(int c=0;c<size;c++)
                    M(l,c)=0;
        }

        /**
        \brief Init filter
        Inits the filter with a start vector
        \param x_init start position of the filter
        */
        void InitFilter(Vector5d& x_init)
        {
            setDim(5,0,5,3,3);
            dt=1./50.;
            lt=Time::now();

            Vector x_0(5);

            x_0(0)=x_init(0);//x
            x_0(1)=x_init(1);//y
            x_0(2)=x_init(2);//v
            x_0(3)=x_init(3);//t
            x_0(4)=x_init(4);//f

            Matrix P_0(5,5);

            miss_associations=0;

            //Large initial uncertainty
            SetIdentity(P_0,5,4.0);

            init(x_0,P_0);
        }

        void InitFilter(Vector5d& x_init,Matrix5d& P_init)
        {
            setDim(5,0,5,3,3);
            dt=1./50.;
            lt=Time::now();

            Vector x_0(5);

            x_0(0)=x_init(0);//x
            x_0(1)=x_init(1);//y
            x_0(2)=x_init(2);//v
            x_0(3)=x_init(3);//t
            x_0(4)=x_init(4);//f

            Matrix P_0(5,5);

            P_0(0,0)=P_init(0,0);
            P_0(0,1)=P_init(0,1);
            P_0(0,2)=P_init(0,2);
            P_0(0,3)=P_init(0,3);
            P_0(0,4)=P_init(0,4);

            P_0(1,0)=P_init(1,0);
            P_0(1,1)=P_init(1,1);
            P_0(1,2)=P_init(1,2);
            P_0(1,3)=P_init(1,3);
            P_0(1,4)=P_init(1,4);

            P_0(2,0)=P_init(2,0);
            P_0(2,1)=P_init(2,1);
            P_0(2,2)=P_init(2,2);
            P_0(2,3)=P_init(2,3);
            P_0(2,4)=P_init(2,4);

            P_0(3,0)=P_init(3,0);
            P_0(3,1)=P_init(3,1);
            P_0(3,2)=P_init(3,2);
            P_0(3,3)=P_init(3,3);
            P_0(3,4)=P_init(3,4);

            P_0(4,0)=P_init(4,0);
            P_0(4,1)=P_init(4,1);
            P_0(4,2)=P_init(4,2);
            P_0(4,3)=P_init(4,3);
            P_0(4,4)=P_init(4,4);

            init(x_0,P_0);

        }

        int miss_associations;
        int life_time;

        Vector5d x_predicted;
        Vector3d z_measured;
        Vector3d inovation_error;

    protected:

        ///Time interval between measurements
        double dt;
        ///Time of the last call to the makeCommonProcess function
        Time lt;

        double l;

        /**
        \brief This function is called before all other make something functions
        It's currently being used to update the time interval between iterations (dt).
        */
        void makeCommonProcess()
        {
            dt=1./50.;
            l=2.5;
        }

        /**
        \brief Make the process Jacobian matrix
        */
        void makeA()
        {
            double v=x(2);
            double t=x(3);
            double f=x(4);

            A(0,0)=1.0;
            A(0,1)=0.0;
            A(0,2)=cos(t)*cos(f)*dt;
            A(0,3)=-sin(t)*cos(f)*v*dt;
            A(0,4)=cos(t)*(-sin(f))*v*dt;

            A(1,0)=0.0;
            A(1,1)=1.0;
            A(1,2)=sin(t)*cos(f)*dt;
            A(1,3)=cos(t)*cos(f)*v*dt;
            A(1,4)=sin(t)*(-sin(f))*v*dt;

            A(2,0)=0.0;
            A(2,1)=0.0;
            A(2,2)=1.0;
            A(2,3)=0.0;
            A(2,4)=0.0;

            A(3,0)=0.0;
            A(3,1)=0.0;
            A(3,2)=sin(f)*dt/l;
            A(3,3)=1.0;
            A(3,4)=cos(f)/l*v*dt;

            A(4,0)=0.0;
            A(4,1)=0.0;
            A(4,2)=0.0;
            A(4,3)=0.0;
            A(4,4)=1.0;
        }

        /**
        \brief Make measurement sensitivity matrix
        */
        void makeH()
        {
            // 			if(miss_associations==0)
            // 			{
            H(0,0) = 1;
            H(0,1) = 0;
            H(0,2) = 0;
            H(0,3) = 0;
            H(0,4) = 0;

            H(1,0) = 0;
            H(1,1) = 1;
            H(1,2) = 0;
            H(1,3) = 0;
            H(1,4) = 0;

            H(2,0) = 0;
            H(2,1) = 0;
            H(2,2) = 0;
            H(2,3) = 1;
            H(2,4) = 0;
            // 			}else
            // 			{
            // 				H(0,0) = 0;
            // 				H(0,1) = 0;
            // 				H(0,2) = 0;
            // 				H(0,3) = 0;
            // 				H(0,4) = 0;
            //
            // 				H(1,0) = 0;
            // 				H(1,1) = 0;
            // 				H(1,2) = 0;
            // 				H(1,3) = 0;
            // 				H(1,4) = 0;
            //
            // 				H(2,0) = 0;
            // 				H(2,1) = 0;
            // 				H(2,2) = 0;
            // 				H(2,3) = 0;
            // 				H(2,4) = 0;
            // 			}
            //

        }

        /**
        \brief Make process noise sensitivity matrix
        */
        void makeBaseW()
        {
            SetIdentity(W,5,1.0);
        }

        /**
        \brief Make measurement noise sensitivity matrix
        */
        void makeBaseV()
        {
            SetIdentity(V,3,1.0);
        }

        /**
        \brief Make measurement noise covariance matrix
        */
        void makeR()
        {
            Matrix2d eigval;

            double inovation_factor = 5.0;
            double distortion_y = 0.1;

            double x_min = 0.2;
            double y_min = 0.2;

            double x_max = 6.0;
            double y_max = 6.0;

            double x_size = inovation_error.norm()*inovation_factor;
            double y_size = inovation_error.norm()*inovation_factor*distortion_y;

            double dir = atan2(inovation_error(1),inovation_error(0));

            cout<<"miss_associations: "<<miss_associations<<endl;

            if(miss_associations>0)
            {
                x_size+=20;
                y_size+=20;
            }

            if(y_size<y_min)
                y_size = y_min;

            if(x_size<x_min)
                x_size = x_min;

            if(y_size>y_max)
                y_size = y_max;

            if(x_size>x_max)
                x_size = x_max;

            eigval << x_size, 		0,
                    0,  y_size;

            Matrix2d rot;
            rot << cos(dir), -sin(dir),
                    sin(dir), cos(dir);

            Matrix2d cov = rot*eigval*rot.inverse();

            R(0,0) = cov(0,0);
            R(0,1) = cov(0,1);
            //
            R(1,0) = cov(1,0);
            R(1,1) = cov(1,1);

            // 			R(0,0) = 0.200;//x
            // 			R(0,1) = 0.000;
            R(0,2) = 0.000;

            // 			R(1,0) = 0.000;//y
            // 			R(1,1) = 0.200;
            R(1,2) = 0.000;

            R(2,0) = 0.000;//t
            R(2,1) = 0.000;
            R(2,2) = 0.010;
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
            Q(0,4) = 0.0;

            //Y
            Q(1,0) = 0.0;
            Q(1,1) = q*0.1;
            Q(1,2) = 0.0;
            Q(1,3) = 0.0;
            Q(1,4) = 0.0;

            //Vl
            Q(2,0) = 0.0;
            Q(2,1) = 0.0;
            Q(2,2) = q*20.;
            Q(2,3) = 0.0;
            Q(2,4) = 0.0;

            //Theta
            Q(3,0) = 0.0;
            Q(3,1) = 0.0;
            Q(3,2) = 0.0;
            Q(3,3) = q*0.01;
            Q(3,4) = 0.0;

            //Fi
            Q(4,0) = 0.0;
            Q(4,1) = 0.0;
            Q(4,2) = 0.0;
            Q(4,3) = 0.0;
            Q(4,4) = q*0.01;

        }

        /**
        \brief Make process, model iteration
        */
        void makeProcess()
        {

            // 	x(0) -> x
            // 	x(1) -> y
            // 	x(2) -> v
            // 	x(3) -> t
            // 	x(4) -> f

            Vector x_(x.size());

            x_(0) = x(0) + cos(x(3))*cos(x(4))*x(2)*dt;
            x_(1) = x(1) + sin(x(3))*cos(x(4))*x(2)*dt;
            x_(2) = x(2);
            x_(3) = x(3) + sin(x(4))*x(2)*dt/l;
            x_(4) = x(4);


            x.swap(x_);
        }

        /**
        \brief Make measurement, used when measurement is not possible (i'm not using it now)
        */
        void makeMeasure()
        {
            z(0)=x(0);//x
            z(1)=x(1);//y
            z(2)=x(3);//t
        }
};


#endif //ROS_BRIDGE_NONHOLONOMICEKFILTER_H
