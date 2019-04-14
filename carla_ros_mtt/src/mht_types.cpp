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
\brief MHT generic types source code, NOT USED, DEPRECATED
*/

// System Includes
#include <carla_ros_mtt/mht_types.h>
#error This file should not be used, DEPRECATED

long association_errors = 0;

void constantVelocityEKFilter::SetIdentity(Matrix& M,int size,double value)
{
    for(int l=0;l<size;l++)
        for(int c=0;c<size;c++)
            if(c==l)
                M(l,c)=value;
            else
                M(l,c)=0;
}

void constantVelocityEKFilter::SetZero(Matrix& M,int size)
{
    for(int l=0;l<size;l++)
        for(int c=0;c<size;c++)
            M(l,c)=0;
}

void constantVelocityEKFilter::InitFilter(Vector4d& x_init)
{
    setDim(4,0,4,2,2);
    dt=1./50.;
    lt=Time::now();

    Vector x_0(4);

    x_0(0)=x_init(0);//x
    x_0(1)=x_init(1);//y
    x_0(2)=x_init(2);//vx
    x_0(3)=x_init(3);//vy

    Matrix P_0(4,4);

    miss_associations=0;

    //Large initial uncertainty
    SetIdentity(P_0,4,2.0);

    init(x_0,P_0);

// 	cout<<"Real init"<<endl;
// 	cout<<"P_0: "<<P_0<<endl;
// 	cout<<"x_0: "<<x_0<<endl;

// 	cout<<"P: "<<calculateP() <<endl;
// 	cout<<"x: "<<x<<endl;
}

void constantVelocityEKFilter::InitFilter(Vector4d& x_init,Matrix4d& P_init)
{
    setDim(4,0,4,2,2);
    dt=1./50.;
    lt=Time::now();

    Vector x_0(4);

    x_0(0)=x_init(0);//x
    x_0(1)=x_init(1);//y
    x_0(2)=x_init(2);//vx
    x_0(3)=x_init(3);//vy

    Matrix P_0(4,4);

    P_0(0,0)=P_init(0,0);
    P_0(0,1)=P_init(0,1);
    P_0(0,2)=P_init(0,2);
    P_0(0,3)=P_init(0,3);

    P_0(1,0)=P_init(1,0);
    P_0(1,1)=P_init(1,1);
    P_0(1,2)=P_init(1,2);
    P_0(1,3)=P_init(1,3);

    P_0(2,0)=P_init(2,0);
    P_0(2,1)=P_init(2,1);
    P_0(2,2)=P_init(2,2);
    P_0(2,3)=P_init(2,3);

    P_0(3,0)=P_init(3,0);
    P_0(3,1)=P_init(3,1);
    P_0(3,2)=P_init(3,2);
    P_0(3,3)=P_init(3,3);

// 	cout<<"initing stuff"<<endl;
// 	cout<<"xinit: "<<x_init<<endl;
// 	cout<<"pinit: "<<P_init<<endl;

    init(x_0,P_0);

// 	cout<<"check if init is ok"<<endl;
// 	cout<<"x: "<<x<<endl;
// 	cout<<"P: "<<calculateP()<<endl;

}

void constantVelocityEKFilter::makeCommonProcess()
{
    dt=1./50.;
}

void constantVelocityEKFilter::makeBaseA()
{
    A(0,0)=1.0;
    A(0,1)=0.0;
// 	A(0,2)=dt;
    A(0,3)=0.0;

    A(1,0)=0.0;
    A(1,1)=1.0;
    A(1,2)=0.0;
// 	A(1,3)=dt;

    A(2,0)=0.0;
    A(2,1)=0.0;
    A(2,2)=1.0;
    A(2,3)=0.0;

    A(3,0)=0.0;
    A(3,1)=0.0;
    A(3,2)=0.0;
    A(3,3)=1.0;
}

void constantVelocityEKFilter::makeA()
{
    A(0,2)=dt;
    A(1,3)=dt;
}

void constantVelocityEKFilter::makeBaseH()
{
    H(0,0) = 1.0;
    H(0,1) = 0.0;
    H(1,1) = 1.0;
    H(1,0) = 0.0;
}

void constantVelocityEKFilter::makeBaseW()
{
    SetIdentity(W,4,1.0);
}

void constantVelocityEKFilter::makeBaseV()
{
    SetIdentity(V,2,1.0);
}

void constantVelocityEKFilter::makeR()
{
    //Innovation dependent error
// 	inovation_error //type Vetor2d

    Matrix2d eigval;

    double inovation_factor = 3.0;
    double distortion_y = 0.1;

    double x_min = 0.2;
    double y_min = 0.2;

    double x_size = inovation_error.norm()*inovation_factor;
    double y_size = inovation_error.norm()*inovation_factor*distortion_y;

    double dir = atan2(inovation_error(1),inovation_error(0));


    if(miss_associations>0)
    {
        x_size+=5;
        y_size+=5;
    }

    //Velocity dependent error
// 	Vector2d vel;
//
// 	vel(0)=x(2);//x
// 	vel(1)=x(3);//y
//
//
// 	Matrix2d eigval;
//
// 	double velocity_factor = 1.0;
// 	double distortion_y = 0.1;
//
// 	double x_min = 0.2;
// 	double y_min = 0.2;
//
// 	double x_size = vel.norm()*velocity_factor;
// 	double y_size = vel.norm()*velocity_factor*distortion_y;

// 	double dir = atan2(vel(1),vel(0));

    if(y_size<y_min)
        y_size = y_min;

    if(x_size<x_min)
        x_size = x_min;


    eigval << x_size, 		0,
            0,  y_size;

    Matrix2d rot;
    rot << cos(dir), -sin(dir),
            sin(dir), cos(dir);

    Matrix2d cov = rot*eigval*rot.inverse();

// 	cout<<"making R"<<endl;
// 	R(0,0) = 0.1;
// 	R(0,1) = 0.0;

// 	R(1,0) = 0.0;
// 	R(1,1) = 0.1;
//
// 	if(miss_association)
// 	{
// 		R(0,0) = cov(0,0)*100;
// 		R(0,1) = cov(0,1)*100;

// 		R(1,0) = cov(1,0)*100;
// 		R(1,1) = cov(1,1)*100;
// 		cout<<"coaissa"<<endl;
// 	}else
// 	{

// 	cout<<"vel: "<<vel.norm()<<endl;

// 	if(magnitude>0.1)
// 	{
// 		R(0,0) = 10;
// 		R(0,1) = 0;
//
// 		R(1,0) = 0;
// 		R(1,1) = 0.1;
// 	}
// 	else
// 	{
// 		R(0,0) = 1;
// 		R(0,1) = 0;
//
// 		R(1,0) = 0;
// 		R(1,1) = 1;
// 	}
    R(0,0) = cov(0,0);
    R(0,1) = cov(0,1);
//
    R(1,0) = cov(1,0);
    R(1,1) = cov(1,1);
// 	}

// 	cout<<"R:"<<R<<endl;
}

void constantVelocityEKFilter::makeQ()
{
    SetZero(Q,4);
// 	double sigma=10.0;
// 	double sigma=5.0;
// 	double sigma=2.0;
// 	double sigma=1.0;
// 	double sigma=0.5;

    double sigma;

    if(life_time<10)
        sigma=150;
    else if(life_time>12)
        sigma=100;
    else if(life_time<15)
        sigma=50;
    else
        sigma=10;


    double factor=pow(sigma,2)*dt/6.;

    Q(0,0)=factor*2*pow(dt,2);//x x
    Q(1,1)=factor*2*pow(dt,2);//y y

    Q(0,2)=factor*3*pow(dt,1);//x vx
    Q(2,1)=factor*3*pow(dt,1);//vx v

    Q(1,3)=factor*3*pow(dt,1);//y vy
    Q(3,1)=factor*3*pow(dt,1);//vy y

    Q(2,2)=factor*6;//vx vx
    Q(3,3)=factor*6;//vy vy
}

void constantVelocityEKFilter::makeProcess()
{
    Vector x_(x.size());

    x_(0) = x(0) + x(2)*dt;//X
    x_(1) = x(1) + x(3)*dt;//Y
    x_(2) = x(2);//Vx
    x_(3) = x(3);//Vy

    x.swap(x_);
}

void constantVelocityEKFilter::makeMeasure()
{
    z(0)=x(0); //X
    z(1)=x(1); //Y
}
