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
\brief Motion model class source code
*/

// System Includes
#include <carla_ros_mtt/motionModel.h>

MotionModel::MotionModel()
{
    _x = Vector5d::Zero();
    _xp = Vector5d::Zero();
    _z = Vector3d::Zero();
    _P = Matrix5d::Zero();
}

void MotionModel::initEstimator(Vector5d& x,Matrix5d& P)
{
    //Pre existing estimator
    _estimator.InitFilter(x,P);
}

void MotionModel::initEstimator(Vector3d& z)
{
    //New estimator

    Vector5d x_init;

    x_init(0)=z(0);//x
    x_init(1)=z(1);//y
    x_init(2)=0;//v
    x_init(3)=z(2);//t
    x_init(4)=0;//f

    _x=x_init;
    _xp=_x;
    _z=z;

    _estimator.InitFilter(x_init);

    kMatrix P = _estimator.calculateP();
    //Convert from ekfilter to eigen format
    _P(0,0)=P(0,0);
    _P(0,1)=P(0,1);
    _P(0,2)=P(0,2);
    _P(0,3)=P(0,3);
    _P(0,4)=P(0,4);

    _P(1,0)=P(1,0);
    _P(1,1)=P(1,1);
    _P(1,2)=P(1,2);
    _P(1,3)=P(1,3);
    _P(1,4)=P(1,4);

    _P(2,0)=P(2,0);
    _P(2,1)=P(2,1);
    _P(2,2)=P(2,2);
    _P(2,3)=P(2,3);
    _P(2,4)=P(2,4);

    _P(3,0)=P(3,0);
    _P(3,1)=P(3,1);
    _P(3,2)=P(3,2);
    _P(3,3)=P(3,3);
    _P(3,4)=P(3,4);

    _P(4,0)=P(4,0);
    _P(4,1)=P(4,1);
    _P(4,2)=P(4,2);
    _P(4,3)=P(4,3);
    _P(4,4)=P(4,4);
}

void MotionModel::stepEstimator(Vector3d z)
{
    kVector u(0);

    _estimator.z_measured=z;//Copy to estimator
    _z=z;

    _estimator.inovation_error=_estimator.z_measured-_estimator.x_predicted.head(3);

    //Convert from eigen format to ekfilter
    kVector z_(3);
    z_(0)=z(0);
    z_(1)=z(1);
    z_(2)=z(2);

    //Makes one prediction-correction step
    _estimator.step(u,z_);

// 	cout<<"Xa: "<<_estimator.getX()<<endl;
// 	cout<<"Pa: "<<_estimator.calculateP()<<endl;

// 	cout<<"Z: "<<z<<endl;

    //Returns the corrected state (a posteriori state estimate)
    kVector x_(5);
    x_ = _estimator.getX();

    //Convert from ekfilter to eigen format
    _x(0)=x_(0);
    _x(1)=x_(1);
    _x(2)=x_(2);
    _x(3)=x_(3);
    _x(4)=x_(4);

// 	cout<<"_X: "<<_x<<endl;

    //Returns the a posteriori error covariance estimate matrix
    kMatrix P=_estimator.calculateP();

    //Convert from ekfilter to eigen format
    _P(0,0)=P(0,0);
    _P(0,1)=P(0,1);
    _P(0,2)=P(0,2);
    _P(0,3)=P(0,3);
    _P(0,4)=P(0,4);

    _P(1,0)=P(1,0);
    _P(1,1)=P(1,1);
    _P(1,2)=P(1,2);
    _P(1,3)=P(1,3);
    _P(1,4)=P(1,4);

    _P(2,0)=P(2,0);
    _P(2,1)=P(2,1);
    _P(2,2)=P(2,2);
    _P(2,3)=P(2,3);
    _P(2,4)=P(2,4);

    _P(3,0)=P(3,0);
    _P(3,1)=P(3,1);
    _P(3,2)=P(3,2);
    _P(3,3)=P(3,3);
    _P(3,4)=P(3,4);

    _P(4,0)=P(4,0);
    _P(4,1)=P(4,1);
    _P(4,2)=P(4,2);
    _P(4,3)=P(4,3);
    _P(4,4)=P(4,4);

// 	cout<<"P:"<<_P<<endl;

    //Returns the predicted state vector (a priori state estimate)
    kVector xp=_estimator.predict(u);

    //Convert from ekfilter to eigen format
    _xp(0)=xp(0);
    _xp(1)=xp(1);
    _xp(2)=xp(2);
    _xp(3)=xp(3);
    _xp(4)=xp(4);

// 	cout<<"h123"<<endl;
// 	cout<<"xP:"<<_xp<<endl;
// 	assert(is_finite(_xp));
// 	assert(is_finite(_P));

    _estimator.x_predicted=_xp;//Copy to estimator
}
