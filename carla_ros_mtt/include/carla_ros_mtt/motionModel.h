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

#ifndef ROS_BRIDGE_MOTIONMODEL_H
#define ROS_BRIDGE_MOTIONMODEL_H

/**
\file
\brief Motion model class declaration
*/

//#include <eigen3/Eigen>
#include <eigen3/Eigen/Dense>
#include <mtt/nonholonomicEKFilter.h>

class MotionModel;

/**
\brief Nonholonomic Motion model abstraction layer
This class encapsulates the underlaying motion model used. It provides an abstraction layer for the Mht class to work with,
without having to deal with the Kalman filter directly.
*/
class MotionModel
{
    public:
        ///Estimated state
        Vector5d _x;
        ///Predicted x
        Vector5d _xp;
        ///Measurement vector
        Vector3d _z;
        ///Posteriori error covariance
        Matrix5d _P;

        ///Local estimator for process, Kalman filter with a constant velocity
        nonholonomicEKFilter _estimator;

        MotionModel();

        void initEstimator(Vector5d& x,Matrix5d& P);

        void initEstimator(Vector3d& z);

        void stepEstimator(Vector3d z);
};


#endif //ROS_BRIDGE_MOTIONMODEL_H
